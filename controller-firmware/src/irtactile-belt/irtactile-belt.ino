
#include <Arduino.h>

#include <Preferences.h>

#include "GP8413_DAC.h"
#include "driver/uart.h"
#include "Pins.h"
#include "SampleRing.h"
#include "Playout.h"
#include "SerialDecoder.h"
#include "ForceCurve.h"
#include "Counter.h"
#include "BeltState.h"
#include "Logging.h"

#include "esp_bt.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "esp_system.h"

// UART config. The pins live in Pins.h; these are the port and the framing.
#define UART_PORT UART_NUM_0
//#define UART_BAUD 921600
#define UART_BAUD 1200000
#define RX_BUF_SIZE 512   // must exceed the 128 byte hardware FIFO
#define CHUNK_SIZE 64

void dataHandler(uint16_t data[2]);
void configHandler(uint32_t rateMilliHz, uint16_t blockSamples);
void tuningHandler(uint16_t fullScaleCounts, uint8_t gammaQ);

GP8413_DAC dac;
SerialDecoder decoder(dataHandler, configHandler, tuningHandler);
SampleRing sampleRing;
Playout playout;

hw_timer_t *timer = NULL;

TaskHandle_t serialTaskHandle;
TaskHandle_t dacTaskHandle;

// Stream config, published by the decoder (serialTask) and picked up by
// dacTask on the next tick. Nothing is known until the first valid header, so
// the default must be a rate the ladder can produce (48000 >> rateCode).
volatile uint32_t g_rateMilliHz = 6000u * 1000u;
volatile uint16_t g_blockSamples = 16;
volatile uint32_t g_configGen = 0;
volatile bool g_configured = false;

// Encoder force-curve parameters. Written by tuningHandler() (serialTask,
// core 0) and by tuningLoad() at boot, read by loop() (core 1), which rebuilds
// the LUT on a generation bump. Aligned single words, published after write -
// the same ordering the arming path uses (BeltState.h). The defaults reproduce
// the old fixed linear ramp.
volatile uint16_t g_fullScaleCounts = DEFAULT_FULL_SCALE_COUNTS;
volatile uint8_t g_gammaQ = DEFAULT_GAMMA_Q;
volatile uint32_t g_tuningGen = 0;

// Double-buffered force curve. loop() builds into the spare slot and then
// publishes the index; processData() (core 0) snapshots the index once per tick
// so ch0 and ch1 cannot straddle a swap. Internal SRAM, coherent across cores.
ForceCurve g_curves[2];
volatile uint8_t g_curveIndex = 0;

// Tuning hold. belt-tune holds the port exclusively, so while it is connected
// there is no sample stream and the playout starves. A valid tuning header
// pushes g_tuningHoldUntilTick forward and arms the hold; dacTask holds the
// encoder floor gain up until g_dacTicks passes the deadline, then disarms.
// A refreshed deadline, never a latch: unplug the USB mid-pull and the belt
// releases on its own, as a dead host does.
//
// The armed flag bounds how old a deadline can get. dacTask's signed-difference
// compare survives the wrap of g_dacTicks only while the deadline is recent:
// once g_dacTicks has passed it by 2^31 (4.1 days at 6 kHz) a stale deadline
// reads as being in the future again. Clearing the flag on the first tick past
// the deadline keeps one from ageing that far.
//
// TUNING_HOLD_TICKS mirrors UNDERRUN_SILENCE_TICKS (Playout.h): 250 ms, i.e. 5
// missed headers of tolerance at belt-tune's 20 Hz cadence and a ~300 ms
// release, the same as the dead-host latency.
#define TUNING_HOLD_TICKS (DAC_TICK_HZ / 4)
volatile uint32_t g_dacTicks = 0;
volatile uint32_t g_tuningHoldUntilTick = 0;
volatile bool g_tuningHoldArmed = false;

// NVS-persisted tuning. Plain statics: only ever touched from loop() on core 1.
// The shadow copies are what a commit compares against, so idle resends of the
// same values never touch flash.
Preferences g_tuningPrefs;
static uint16_t s_persistedFullScale = DEFAULT_FULL_SCALE_COUNTS;
static uint8_t s_persistedGammaQ = DEFAULT_GAMMA_Q;

// DAC write failure counters. Written by dacTask (core 0) in writeDac(), read
// by loop() (core 1) for the PLAY line - aligned 32-bit, so each is read
// atomically and no sequence lock is needed; they are only displayed, never
// compared.
//   g_dacFailTotal - cumulative failed writes since boot
//   g_dacFailRun   - length of the current unbroken run of failures, 0 when
//                    the last write succeeded
volatile uint32_t g_dacFailTotal = 0;
volatile uint32_t g_dacFailRun = 0;

// Timer ticks the DAC task was notified of but never serviced, cumulative
// since boot. Same core/ownership as the counters above.
//
// dacTask takes its notification with ulTaskNotifyTake(pdTRUE, ...), which
// clears the count rather than decrementing it, so ticks arriving while the
// task is busy collapse into the one it is about to handle - about six of them
// for a 1 ms I2C timeout against the 167 us tick. Counted, never caught up:
// ticking the extra times would burst the I2C writes back-to-back and turn a
// scheduling hiccup into a bus overrun.
volatile uint32_t g_missedTicks = 0;

// Consecutive failed writes before the output is ramped down. A single NAK
// leaves a stale value latched for one tick (167 us); a run means the bus is
// gone and the latched value is no longer a command anyone issued. 16 ticks is
// ~2.7 ms - past any transient, and well inside the 50 ms ramp that follows.
// Bus timeouts stretch the wall-clock time; the count is what matters.
#define DAC_FAIL_MUTE_RUN 16

// Consecutive failed writes before the drives are cut at the relays - one
// second of ticks.
//
// The mute above cannot un-freeze a dead bus: a GP8413 that is not
// acknowledging holds the value it latched, and two of the three processData()
// branches have no gain for the mute to act on anyway (the INITIALIZING preload
// is unramped, the disarmed branch already writes zero). The enables do not go
// through the DAC, so pulling them HIGH is the one authority left.
//
// A second rather than the ~2.7 ms mute threshold because this is neither cheap
// nor reversible: at 1 ms of I2C timeout per attempt it is several hundred
// genuine attempts.
//
// The cut is latched in g_drivesCut (BeltState.h) and cleared only by a pass
// through INACTIVE, i.e. by the safety switch.
#define DAC_FAIL_DISABLE_RUN ((uint32_t)DAC_TICK_HZ)

// Task watchdog. The enables are a static GPIO level, so frozen firmware looks
// healthy from the drives' side: the last DAC value stays latched and the belts
// keep pulling. The safety switch is a real hardware kill path (wiring doc
// section 4) but needs a human to notice first.
//
// The Arduino build already runs the TWDT with panic-on-timeout and core 0's
// idle task subscribed. That watch catches a spin, not a block: dacTask waiting
// forever on a notification that stopped arriving leaves core 0 idle and looks
// healthy, so both of our tasks are subscribed too.
//
// A timeout panics and reboots, which releases both enables for the ~1 s setup()
// takes and comes back in PRE_INITIALIZED, i.e. 0 V until someone presses the
// mode button - a false trip costs a deliberate re-arm and nothing worse.
//
// 1 s against a 100 ms feed interval is ten missed feeds, far above any
// legitimate stall: the I2C timeout is 1 ms per transfer and the UART read
// blocks for a single tick. The period is shared with the idle-task watch, which
// this shortens from the 5 s default.
#define WDT_TIMEOUT_MS 1000
#define WDT_FEED_MS    100
#define WDT_FEED_TICKS ((DAC_TICK_HZ * WDT_FEED_MS) / 1000)


void IRAM_ATTR onTimer() {
  BaseType_t xHigherPriorityWoken = pdFALSE;
  vTaskNotifyGiveFromISR(dacTaskHandle, &xHigherPriorityWoken);
  if (xHigherPriorityWoken) {
    portYIELD_FROM_ISR();
  }
}

// Runs in serialTask, the ring's only producer.
//
// The sample goes in as it arrived, whatever the arming state: processData()
// already writes zero in the states that are not INITIALIZED, and the drift
// lock needs the cadence even while the unit is not armed.
void dataHandler(uint16_t data[2]) {
  dac_sample_t sample;
  sample.ch0 = data[0];
  sample.ch1 = data[1];
  sampleRing.push(sample);
}

// A valid header repeats at ~10 Hz; only a real change bumps the generation
// counter.
void configHandler(uint32_t rateMilliHz, uint16_t blockSamples) {
  if (g_configured && rateMilliHz == g_rateMilliHz && blockSamples == g_blockSamples) return;

  g_rateMilliHz = rateMilliHz;
  g_blockSamples = blockSamples;
  g_configured = true;
  g_configGen = g_configGen + 1;
}

// A tuning header arrived (serialTask, same core as dacTask). Two jobs:
//
//  1. Push the floor-hold deadline forward - on every valid header, before the
//     change check, so the belt stays live for TUNING_HOLD_TICKS after the last.
//  2. Publish the curve parameters, but only on a real change, so a 20 Hz
//     stream of identical headers does not bump the generation forever. The
//     clamp runs first, so the debounce compares clamped values.
//
// No flash write here; persistence is deferred to a settle point in loop().
void tuningHandler(uint16_t fullScaleCounts, uint8_t gammaQ) {
  forceCurveClamp(fullScaleCounts, gammaQ);

  // Deadline before the flag: dacTask must never see the hold armed against a
  // stale deadline.
  g_tuningHoldUntilTick = g_dacTicks + TUNING_HOLD_TICKS;
  g_tuningHoldArmed = true;

  if (g_fullScaleCounts == fullScaleCounts && g_gammaQ == gammaQ) return;
  g_fullScaleCounts = fullScaleCounts;
  g_gammaQ = gammaQ;
  g_tuningGen = g_tuningGen + 1;
}

// Load the persisted curve at boot. Called from setup() after disableWifi(),
// which brings NVS up (nvs_flash_init). Clamps whatever came back - a bad
// persisted value must not survive a reboot - then seeds the live volatiles and
// the shadow.
void tuningLoad() {
  g_tuningPrefs.begin("tuning", false);
  uint16_t fs = g_tuningPrefs.getUShort("fscnt", DEFAULT_FULL_SCALE_COUNTS);
  uint8_t gq = g_tuningPrefs.getUChar("gammaq", DEFAULT_GAMMA_Q);
  forceCurveClamp(fs, gq);
  g_fullScaleCounts = fs;
  g_gammaQ = gq;
  s_persistedFullScale = fs;
  s_persistedGammaQ = gq;
}

// Commit the live curve to flash if it differs from the shadow, so idle resends
// never erase a sector. Called only from the INACTIVE settle point in loop().
void tuningCommitIfChanged() {
  const uint16_t fs = g_fullScaleCounts;
  const uint8_t gq = g_gammaQ;
  if (fs == s_persistedFullScale && gq == s_persistedGammaQ) return;
  g_tuningPrefs.putUShort("fscnt", fs);
  g_tuningPrefs.putUChar("gammaq", gq);
  s_persistedFullScale = fs;
  s_persistedGammaQ = gq;
}

// Unrecoverable initialisation failure. Stopping is not by itself safe: the
// enables are a static GPIO level that setup() has already driven LOW by the
// time either caller can fail, so a bare spin leaves both drives energised
// holding whatever the DAC last latched, with no watchdog yet configured to
// notice. So drop the relays first - the only kill path the firmware owns -
// then say why on the logging UART.
//
// Not a reboot: both callers fail on hardware faults (no timer, no DAC on the
// bus) that would recur every cycle, chattering the enable relays once a second
// forever.
void fatalHalt(const char *why) {
  digitalWrite(ENABLE_PIN_1, HIGH);   // HIGH releases the relay - see setup()
  digitalWrite(ENABLE_PIN_2, HIGH);
  for (;;) {
    Serial1.printf("FATAL: %s - drives disabled, halted\n", why);
    vTaskDelay(pdMS_TO_TICKS(1000));  // never a bare spin: this runs at prio 24
  }
}

void timerSetupTask(void *pvParameters) {
  // Count at the DAC tick rate, so one timer tick is one DAC sample period.
  timer = timerBegin(DAC_TICK_HZ);
  if (!timer) {
    fatalHalt("timer init failed");
  }

  // Attach the ISR before arming the alarm, or the first edge fires with no
  // handler installed.
  timerAttachInterrupt(timer, &onTimer);

  // Fire every tick - DAC_TICK_HZ times a second - and repeat forever.
  timerAlarm(timer, 1, true, 0);

  timerStart(timer);

  vTaskDelete(NULL);  // the ISR runs on this core
}


// Encoder counts -> the force floor for that pull, via the runtime curve. The
// lookup clamps counter to the curve's full-scale count and holds the ceiling
// there. The caller passes the curve - one snapshot for both channels, so ch0
// and ch1 cannot straddle a LUT swap.
uint16_t forceFloor(const ForceCurve &curve, uint16_t counter) {
  return forceCurveLookup(curve, counter);
}

// Every DAC write goes through here, so exactly one place looks at the result.
// The failure is not fail-safe: on a NAK or a bus timeout the GP8413 holds its
// previous output *indefinitely*, and a stale latched value is most likely
// under electrical stress - motors working hardest, command highest.
//
// A run of DAC_FAIL_MUTE_RUN failures arms the playout mute, which ramps the
// commanded value down over the same 50 ms path the underrun silence uses. It
// cannot un-freeze a dead bus, but it makes the first write that does get
// through carry a low value rather than the frozen one. The mute clears on the
// first success, so a recovered bus ramps back up.
void writeDac(uint16_t ch0, uint16_t ch1) {
  if (dac.setBothChannels(ch0, ch1)) {
    g_dacFailRun = 0;
    playout.setMuted(false);
    return;
  }

  if (g_dacFailTotal != 0xFFFFFFFFu) g_dacFailTotal = g_dacFailTotal + 1;

  // Saturate rather than wrap: a wrap would drop the run back below the
  // threshold and un-mute a belt whose bus is still dead.
  const uint32_t run = (g_dacFailRun == 0xFFFFFFFFu) ? 0xFFFFFFFFu : g_dacFailRun + 1;
  g_dacFailRun = run;
  if (run >= DAC_FAIL_MUTE_RUN) playout.setMuted(true);

  // Past anything the command path can still influence: cut the relays.
  // Guarded on the flag so this is one pair of GPIO writes at the crossing, not
  // two per tick afterwards. beltStateUpdate() owns clearing the latch.
  if (run >= DAC_FAIL_DISABLE_RUN && !g_drivesCut) {
    digitalWrite(ENABLE_PIN_1, HIGH);   // HIGH releases the relay
    digitalWrite(ENABLE_PIN_2, HIGH);
    g_drivesCut = true;
  }
}

// Both arguments are 0-based DAC channels, matching dac_sample_t and
// CounterStatus - see the numbering note at the top of Counter.h.
void processData(uint16_t ch0, uint16_t ch1){

  // Snapshot once: beltStateUpdate() can change g_status from the other core
  // mid-call, and a tick that read INITIALIZING then not-INITIALIZED would
  // blank the output for one sample.
  const int status = g_status;
  const uint16_t preload = g_preloadLevel;

  // Snapshot the curve index once, same idiom as the status above: loop() can
  // publish a new index from the other core mid-call, and both floor terms must
  // use the same LUT.
  const ForceCurve &curve = g_curves[g_curveIndex];

  if(status == Status::INITIALIZING && preload > 0){
    // Button-held preload: deliberate, host-independent, and not ramped.
    writeDac(preload, preload);
  }
  else if (status == Status::INITIALIZED){
    EncoderCounter* counter = EncoderCounter::GetInstance();

    // Tracked, not raw: the PCNT counter wraps to 0 at 32767 instead of
    // saturating, which would drop the floor from full scale to nothing at
    // maximum pull. The only caller allowed the tracked form - it is the one
    // that runs every tick, which is what the tracker needs.
    CounterStatus cnt = counter->getCountsTracked();

    // The encoder-derived floor is scaled by the playout's floor gain, so the
    // silence ramp governs the *final* output and not just the stream - this is
    // what makes a dead host release the belts.
    //
    // With no tuning hold active, floorGainQ16() == gainQ16() and scaling each
    // term is equivalent to scaling the combined value - max(g*s, g*f) ==
    // g*max(s, f), so the stream is not gained twice. During a hold the two
    // gains diverge: the stream ramps down while the floor is held up.
    const uint32_t gain = playout.floorGainQ16();
    if(cnt.ch0>0){
      ch0= max(ch0, (uint16_t)(((uint32_t)forceFloor(curve, cnt.ch0) * gain) >> 16));
    }
    if(cnt.ch1>0){
      ch1= max(ch1, (uint16_t)(((uint32_t)forceFloor(curve, cnt.ch1) * gain) >> 16));
    }

    writeDac(ch0, ch1);
  }
  else{
     writeDac(0, 0);
  }

}

// Fixed DAC_TICK_HZ tick, zero-order hold resampling from the stream rate, with
// the consumption rate locked to the production rate by a PI loop. The playout
// maths lives in Playout.h so it can be simulated off-target.
void dacTask(void *pvParameters) {
  uint32_t seenGen = 0xFFFFFFFF;
  uint32_t wdtTicks = 0;
  playout_stats_t stats;

  while (true) {
    // The count, not just the wake-up: anything above 1 is ticks that fired
    // while this task was still inside the previous one. See g_missedTicks.
    const uint32_t pending = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (pending > 1) g_missedTicks += pending - 1;

    // Scope trigger, HIGH for the whole of this tick's work. See DIAG_PIN.
    DIAG_HIGH();

    // Free-running tick count, the time base for the tuning hold. Advanced by
    // `pending`, not by one, so ticks that fired while this task was still in
    // the previous one still count as elapsed time. Integer-only: nothing on
    // this path calls millis().
    g_dacTicks += pending;

    // Signed difference so the compare survives the counter wrap, guarded by
    // the armed flag so an expired deadline is retired within a tick and cannot
    // come back round as a future one. See g_tuningHoldArmed.
    bool floorHold = false;
    if (g_tuningHoldArmed) {
      if ((int32_t)(g_dacTicks - g_tuningHoldUntilTick) < 0) {
        floorHold = true;
      } else {
        g_tuningHoldArmed = false;
      }
    }
    playout.setFloorHold(floorHold);

    // Pick up a new stream configuration.
    if (seenGen != g_configGen) {
      seenGen = g_configGen;
      playout.configure(g_rateMilliHz, g_blockSamples);
    }

    const dac_sample_t out = playout.tick(sampleRing);
    processData(out.ch0, out.ch1);

    if (playout.takeStats(stats)) {
      logPublishStats(stats);
    }

    DIAG_LOW();

    // Fed at the end of the tick path, so a feed means a whole tick was
    // serviced and not merely that the task is still scheduled. Every
    // WDT_FEED_TICKS rather than every tick: the reset takes a spinlock and
    // walks the subscriber list.
    if (++wdtTicks >= WDT_FEED_TICKS) {
      wdtTicks = 0;
      esp_task_wdt_reset();
    }
  }
}


// Serial RX: read and decode in one task.
void serialTask(void *pvParameters) {
  uint8_t rxBuf[CHUNK_SIZE];
  uint32_t lastFeed = millis();

  // Flush whatever accumulated before we were ready.
  while (uart_read_bytes(UART_PORT, rxBuf, CHUNK_SIZE, 0) > 0) {
  }

  while (true) {
    int len = uart_read_bytes(UART_PORT, rxBuf, CHUNK_SIZE, 1);  // 1 tick
    if (len > 0) {
      decoder.process(rxBuf, len);
    }

    // On time, not on data: the read returns after its tick whether or not
    // anything arrived, so an idle link still feeds. A silent host is the
    // playout's problem; this only watches the task.
    const uint32_t now = millis();
    if (now - lastFeed >= WDT_FEED_MS) {
      lastFeed = now;
      esp_task_wdt_reset();
    }
  }
}


// Arduino never brings Wi-Fi or BT up on its own, so the stop/deinit calls all
// return NOT_INIT and are kept only to tear down an explicitly started radio.
// What buys something is releasing the BT controller's static RAM, which is
// never reclaimed otherwise; the controller cannot be initialised again this
// boot.
void disableWifi(){
    esp_netif_init();
    esp_err_t nvs = nvs_flash_init();   // required by esp_wifi even if unused
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      nvs_flash_erase();
      nvs_flash_init();
    }
    esp_wifi_stop();
    esp_wifi_deinit();

    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);   // ~64 KB back
}


void setup() {

  logBegin();

  beltStateBegin();

  EncoderCounter::GetInstance();

  disableWifi();

  // NVS is up now (disableWifi calls nvs_flash_init). Load the persisted force
  // curve and build the initial LUT before the tasks start, so processData()
  // never reads an unbuilt g_curves[0].
  tuningLoad();
  buildForceCurve(g_fullScaleCounts, g_gammaQ, g_curves[0]);

  pinMode(DIAG_PIN, OUTPUT);
  DIAG_LOW();

  pinMode(ENABLE_PIN_1, OUTPUT);
  pinMode(ENABLE_PIN_2, OUTPUT);

  digitalWrite(ENABLE_PIN_1, HIGH);
  digitalWrite(ENABLE_PIN_2, HIGH);

  const uart_config_t uart_config = {
    .baud_rate = UART_BAUD,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };

  uart_param_config(UART_PORT, &uart_config);
  uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_PORT, RX_BUF_SIZE, 0, 0, NULL, 0);

  // Post partial reads promptly instead of sitting on a nearly full FIFO.
  uart_set_rx_full_threshold(UART_PORT, 8);
  uart_set_rx_timeout(UART_PORT, 2);   // in symbol times

  delay(1000);
  digitalWrite(ENABLE_PIN_1, LOW);
  digitalWrite(ENABLE_PIN_2, LOW);
  if (!dac.begin(OUTPUT_RANGE_10V)) {
    fatalHalt("GP8413 DAC init failed");
  }

  // Tasks start last, and only once the DAC is live: dacTask writes the DAC on
  // every timer tick, so starting it earlier races dac.begin()'s I2C driver
  // install.
  xTaskCreatePinnedToCore(dacTask, "DAC Task", 4096, NULL, 24, &dacTaskHandle, 0);
  xTaskCreatePinnedToCore(timerSetupTask, "TimerSetup", 2048, NULL, 24, NULL, 1);

  xTaskCreatePinnedToCore(serialTask, "SerialRX", 4096, NULL, 20, &serialTaskHandle, 0);

  // Watch both core 0 tasks. Subscribing from here keeps the failure paths on
  // the core allowed to touch the logging UART, and a task is watched from the
  // moment it exists.
  //
  // loop() is left out: it does not hold the enable, a hung loop() leaves the
  // DAC path running normally, and it blocks on a 115200 baud UART inside its
  // own watchdog period - a source of false trips rather than of coverage.
  const esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WDT_TIMEOUT_MS,
    .idle_core_mask = (1u << 0),   // as the build ships it: core 0's idle task
    .trigger_panic = true,
  };
  if (esp_task_wdt_reconfigure(&wdtConfig) != ESP_OK) {
    Serial1.println("WDT: reconfigure failed, left at the build default");
  }
  if (esp_task_wdt_add(dacTaskHandle) != ESP_OK) {
    Serial1.println("WDT: dacTask NOT watched");
  }
  if (esp_task_wdt_add(serialTaskHandle) != ESP_OK) {
    Serial1.println("WDT: serialTask NOT watched");
  }
}

// Housekeeping only: one pass of the arming machine, then the log lines it and
// the DAC task have earned. Everything with a deadline runs in the two core 0
// tasks.
void loop() {
  EncoderCounter* counter = EncoderCounter::GetInstance();

  const BeltUpdate belt = beltStateUpdate(counter);

  // Rebuild the force-curve LUT here, off the tick path: 65 powf calls is
  // ~100-250 us, a whole DAC tick, and would give dacTask an FPU context to
  // save on every context switch. Build into the spare slot, then publish the
  // index.
  static uint32_t seenTuningGen = 0;
  const uint32_t tuningGen = g_tuningGen;
  if (tuningGen != seenTuningGen) {
    seenTuningGen = tuningGen;
    const uint8_t next = 1 - g_curveIndex;
    buildForceCurve(g_fullScaleCounts, g_gammaQ, g_curves[next]);
    g_curveIndex = next;
  }

  // Persist the live curve on the entry to INACTIVE, where the DAC output is a
  // constant: INACTIVE writes writeDac(0, 0). A flash sector erase disables the
  // instruction cache on both cores for ~20-40 ms, so the stall must land on
  // ticks whose value does not change. INITIALIZING (an unramped preload) is
  // also constant, but the safety switch is the one settle point we commit at:
  // every mode-button press re-enters INITIALIZING, so committing there would
  // erase a sector on each press for no gain over the shadow check.
  // This commit point is not freely relocatable.
  if (belt.status == Status::INACTIVE && belt.prevStatus != Status::INACTIVE) {
    tuningCommitIfChanged();
  }

  logDrivesRestored(belt);
  logStatusTransition(belt);
  logPins(belt);
  logDrivesCut(DAC_FAIL_DISABLE_RUN);

  const dac_health_t health = { g_dacFailTotal, g_dacFailRun, g_missedTicks };
  logPlayLine(g_configured, health);

  // dacTask retires the deadline on the first tick past it, so the armed flag
  // alone is the hold state.
  logTuning(g_tuningHoldArmed, g_fullScaleCounts, g_gammaQ);

  logCounts(counter);

  delay(100);
  taskYIELD();
}
