#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>

#include "Pins.h"
#include "Playout.h"
#include "Counter.h"
#include "esp_system.h"   // esp_reset_reason()
#include "BeltState.h"

// Everything that reaches the logging UART, and the sequence lock that carries
// the playout statistics across to it.
//
// dacTask, on core 0, must never touch Serial1: it runs at 6 kHz against a
// 115200 baud port. It publishes into g_stats and returns; loop() on core 1
// does the printing.
#define LOGGING_BAUD 115200

// Playout statistics, published once per second by dacTask and printed by
// loop(). Sequence lock: the writer makes g_statsSeq odd while the 36-byte
// struct is in flux, so a reader that sees the same even value either side of
// its copy knows the copy is coherent. Nothing in the DAC path waits on the
// reader.
playout_stats_t g_stats = {};
volatile uint32_t g_statsSeq = 0;

// DAC-path health, owned by dacTask and only ever displayed. Separate from the
// stats record beside it, which is a snapshot up to a second old.
struct dac_health_t {
  uint32_t failTotal;
  uint32_t failRun;
  uint32_t missedTicks;
};

inline void logBegin() {
  Serial1.begin(LOGGING_BAUD, SERIAL_8N1, LOGGING_RX1_PIN, LOGGING_TX1_PIN);

  // Reset reason, once per boot. The panic handler prints on UART0, which here
  // is the data link from the host, so a watchdog reboot is otherwise invisible
  // from the logging side. 4 (panic) or 6 (task WDT) mid-session means a watched
  // task stopped feeding.
  Serial1.printf("BOOT reset=%d\n", (int)esp_reset_reason());
}

// Writer side of the sequence lock. Called from dacTask on core 0.
inline void logPublishStats(const playout_stats_t &s) {
  g_statsSeq = g_statsSeq + 1;   // odd: write in progress
  __sync_synchronize();
  g_stats = s;
  __sync_synchronize();
  g_statsSeq = g_statsSeq + 1;   // even: stable
}

// One line per transition, not one per 100 ms poll.
inline void logStatusTransition(const BeltUpdate &u) {
  if (u.status == u.prevStatus) return;
  static const char *const kStatusName[] = {
    "INACTIVE", "PRE_INITIALIZED", "INITIALIZING", "INITIALIZED"
  };
  Serial1.printf("Status: %s\n", kStatusName[u.status]);
}

// Raw pin states, every poll. Prefixed PINS, not "Status:", which the
// transition line above owns.
inline void logPins(const BeltUpdate &u) {
  Serial1.printf("PINS status=%d button=%d\n", u.statusPin, u.buttonPin);
}

// The two edges of a DAC-failure cut: the cut happens in writeDac() on core 0,
// which must never touch the logging UART, and the restore in
// beltStateUpdate(), which prints nothing. Separate calls because they sit
// either side of the status lines in the log.
inline void logDrivesRestored(const BeltUpdate &u) {
  if (u.drivesRestored) {
    Serial1.println("DRIVES: enables restored after DAC-failure cut");
  }
}

inline void logDrivesCut(uint32_t disableRun) {
  static bool lastDrivesCut = false;
  if (g_drivesCut != lastDrivesCut) {
    lastDrivesCut = g_drivesCut;
    if (lastDrivesCut) {
      Serial1.printf("DRIVES: cut after %lu consecutive DAC write failures"
                     " - open the safety switch to re-arm\n",
                     (unsigned long)disableRun);
    }
  }
}

// Playout statistics - the drift lock pass/fail criterion. Read under the
// sequence lock, so a record straddling dacTask's update is retried on the next
// pass rather than printed torn.
inline void logPlayLine(bool configured, const dac_health_t &h) {
  static uint32_t lastStatsSeq = 0;
  const uint32_t seq = g_statsSeq;
  if ((seq & 1u) != 0u || seq == lastStatsSeq) return;

  __sync_synchronize();
  const playout_stats_t s = g_stats;
  __sync_synchronize();
  if (g_statsSeq != seq) return;   // torn: try again next pass

  lastStatsSeq = seq;
  Serial1.printf("PLAY %s rate=%lu.%03lu Hz blk=%u fill[%lu..%lu] target=%u under=%lu resync=%lu drop=%lu ppm=%ld dacerr=%lu dacrun=%lu missed=%lu\n",
                 configured ? "sync" : "nosync",
                 (unsigned long)(s.rateMilliHz / 1000u),
                 (unsigned long)(s.rateMilliHz % 1000u),
                 (unsigned)s.blockSamples,
                 (unsigned long)s.minFill,
                 (unsigned long)s.maxFill,
                 (unsigned)s.targetMin,
                 (unsigned long)s.underrunTicks,
                 (unsigned long)s.resynced,
                 (unsigned long)s.dropped,
                 (long)s.corrPpm,
                 (unsigned long)h.failTotal,
                 (unsigned long)h.failRun,
                 (unsigned long)h.missedTicks);
}

// Force-curve diagnostics: a line when the parameters change, and one at each
// edge of the tuning floor hold. The hold suspends the dead-host release, so
// the bench needs to see it.
inline void logTuning(bool holdActive, uint16_t fullScaleCounts, uint8_t gammaQ) {
  static bool s_lastHold = false;
  static uint16_t s_lastFs = 0;
  static uint8_t s_lastGq = 0;
  static bool s_first = true;

  if (s_first || fullScaleCounts != s_lastFs || gammaQ != s_lastGq) {
    s_lastFs = fullScaleCounts;
    s_lastGq = gammaQ;
    Serial1.printf("TUNING: fullScale=%u gammaQ=%u\n",
                   (unsigned)fullScaleCounts, (unsigned)gammaQ);
  }
  s_first = false;

  if (holdActive != s_lastHold) {
    s_lastHold = holdActive;
    Serial1.printf("TUNING: floor hold %s\n", holdActive ? "asserted" : "released");
  }
}

// Raw counts on purpose: the diagnostic view shows what the hardware register
// holds. The wrap counts alongside stay 0 through a normal session; non-zero
// means a belt was pulled past the wrap. Reading them from the other core is a
// benign torn-free int32 read.
//
// Labels are 1-based: Count1/Wrap1 is index 0 in the code. See Counter.h.
inline void logCounts(EncoderCounter *counter) {
  const CounterStatus cnt = counter->getCounts();
  Serial1.printf("Count1: %d Count2: %d Wrap1: %lu Wrap2: %lu\n",
                 cnt.ch0, cnt.ch1,
                 (unsigned long)counter->wrapsCh0(),
                 (unsigned long)counter->wrapsCh1());
}

#endif
