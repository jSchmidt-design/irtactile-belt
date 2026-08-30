# controller-firmware

ESP32 Arduino firmware. Receives the tactile stream over USB serial from the
[host bridge](../host-bridge/) and plays it out on a GP8413 dual 15-bit DAC
driving two shaker channels.

```
bridge ──USB/CH340──▶ UART0 ──▶ serialTask ──▶ SerialDecoder ──▶ SampleRing
                                                                      │
6 kHz timer ISR ──▶ dacTask ──▶ Playout ◀─────────────────────────────┘
                                   │    (ZOH + drift lock)
                                   ▼
                              processData ──▶ GP8413  (I²C, 5 B)
```

The DAC tick is fixed at 6 kHz. The stream rate arrives in-band in the sync
marker, so any rate on the ladder is accepted **without a recompile**: it is an
internal resampling ratio rather than a hardware timer setting. 6 kHz is itself
a ladder point, so every rate divides it by a power of two and each sample is
held for a whole number of ticks (6000 → 1, 3000 → 2, 375 → 16).

## Hardware

Target board: ESP32 devkit (`esp32:esp32:esp32`), USB-serial via **CH340**.

| Function | Pins | Notes |
|---|---|---|
| Data link (UART0) | RX 3, TX 1 | 1 200 000 baud, host → device only |
| Logging (UART1) | TX 17, RX 16 | 115200 baud, device → host only |
| DAC (I²C) | SDA 21, SCL 22 | GP8413 @ `0x58`, 1 MHz, 0-10 V range |
| Encoder ch1 (PCNT) | A 4, B 5 | hardware pulse counter, direction from B |
| Encoder ch2 (PCNT) | A 26, B 27 | hardware pulse counter, direction from B |
| Status input | 19 | `INPUT_PULLUP`; high = INACTIVE |
| Mode button | 18 | `INPUT_PULLUP`, momentary; held = INITIALIZING, released = INITIALIZED |
| Enable outputs | drive1 25, drive2 13 | held high during boot, low in operation |
| Diagnostic | 14 | scope trigger: HIGH for the duration of each DAC tick's work |

**The logging UART needs its own adapter.** The COM data link is one-way and
carries no logs back, so reading `Serial1` needs either a USB-TTL adapter on
GPIO17 or a second ESP32 running `tools/logger/logger.ino`.

At boot the Bluetooth controller's static RAM is released (~64 KB) and the
Wi-Fi/BT stacks are torn down; BT cannot be initialised for that boot
afterwards.

## Tasks

| Task | Core | Prio | Role |
|---|---|---|---|
| `dacTask` | 0 | 24 | woken by the 6 kHz timer; resamples, writes the DAC |
| `serialTask` | 0 | 20 | reads UART and decodes inline (no intermediate queue) |
| `timerSetupTask` | 1 | 24 | starts the hardware timer, then deletes itself |
| `loop()` | 1 | n/a | status/button/encoder handling and logging, every 100 ms |

Both core 0 tasks are subscribed to the task watchdog (1 s, panic on timeout).
A timeout reboots, which drops the enables and brings the unit back in
`PRE_INITIALIZED` at 0 V, needing a deliberate re-arm.

## Operating states

| State | Output |
|---|---|
| `INACTIVE` | 0 V; status pin (19) is high |
| `PRE_INITIALIZED` | 0 V; status pin low, waiting for the button |
| `INITIALIZING` | both channels held at `PRELOAD_DAC_VALUE` (2000); button held |
| `INITIALIZED` | `gain × max(stream sample, encoder-derived level)` per channel |

The button on pin 18 is momentary and **held to preload**: releasing it arms
the unit and resets the encoder counters, which is the zero the encoder level
is measured from. PRE_INITIALIZED is only left by pressing, so an untouched
unit stays at 0 V. The reading is debounced over 50 ms.

`gain` is the playout ramp. It scales the encoder-derived level as well as the
stream sample, which is what makes a dead host **release the belts** rather
than hold their preload. Before INITIALIZED the decoder still pushes real
samples into the ring: the cadence has to keep flowing or the drift lock has
nothing to lock to, and `processData()` writes zero in every state but
`INITIALIZED`.

## Wire protocol

Full reference: [`protocol.md`](protocol.md). 5-byte data frames
with high bytes masked to `0x7F`, plus a 9-byte sync marker carrying the stream
rate and block size in-band, repeated at ~10 Hz, so an ESP32 that resets
mid-session re-acquires within one header interval. Rates above the 6 kHz DAC
tick are rejected as a bad header, and the decoder re-syncs on the next good
one.

A second header type (`0x02`) carries the encoder force-curve parameters —
full-scale pull count and gamma — from the separate `belt-tune` tool. While
those headers are arriving the firmware holds the encoder floor gain up (there
is no sample stream to feed the playout), on a refreshed ~250 ms deadline that
lapses on its own when they stop. Both values are clamped in the firmware
(`ForceCurve.h`) on the wire and on the persistence-load path, and committed to
NVS only on a safety-switch trip (the entry to `INACTIVE`).

## Playout and drift lock

`Playout.h` resamples the stream onto the fixed 6 kHz tick with a zero-order
hold. The host audio clock and the ESP32 crystal are both nominally correct and
neither is the other; at 100 ppm a small buffer drifts through a full
over/underrun in seconds. So once per second the consumption rate is trimmed to
keep the ring from either draining or growing.

Fault behaviour:

- **Underrun** holds the last sample. After 250 ms of continuous underrun both
  channels ramp to 0 over 50 ms, and back up over 50 ms when the stream
  returns. The ramp governs the final output, encoder floor included, so a host
  that dies with the belt displaced releases the tension after ~300 ms.
- **A run of failed DAC writes ramps down too.** The GP8413 holds its previous
  output indefinitely on a NAK or I²C timeout, so 16 consecutive failures ramp
  the commanded value down the same 50 ms path. Not latched; it clears on the
  first success.
- **A sustained run cuts the drives.** One second of unbroken failures
  de-asserts both enables, which do not go through the DAC. This one **is
  latched**: open the safety switch to clear it. Both edges are marked by
  `DRIVES:` lines on `Serial1`.
- **Ring full is a drop, never a block**, and a gross backlog is discarded
  rather than played out late.

## Building

```
compile.bat                  build (incremental, ~10 s)
compile.bat --clean          full rebuild (much slower)
compile.bat --warnings all   build with all compiler warnings
compile.bat -u -p COM6       build and upload
```

Extra arguments pass through to `arduino-cli`. Build output lands in `build/`
(git-ignored), which is what makes retries incremental.

Requires the **esp32 core 3.x** (developed against 3.3.1); `timerBegin(freq)`
is the 3.x timer API and will not compile against 2.x.

Uploading: `compile.bat -u -p COM6`, or the Arduino IDE. See also
[`flashing.md`](flashing.md). On a CH340 devkit DTR/RTS drive the
auto-reset circuit, so anything that opens the port with driver-default line
states reboots the board; the bridge suppresses them.

## Tests

The protocol, playout, counter-wrap and force-curve logic are plain C++ headers
with no Arduino dependency, so they are tested natively; no board involved.
Needs MSVC; the script locates it via `vswhere`.

```
test.bat            build and run all suites
test.bat decoder    protocol/framing only
test.bat playout    resampling and drift lock only
test.bat counter    encoder counter wrap handling only
test.bat forcecurve encoder force curve (full-scale count + gamma) only
```

## Reading the log

`Serial1` at 115200 carries one `BOOT reset=<n>` line per boot
(`esp_reset_reason()`): `1` is an ordinary power-up, `4` (panic) or `6` (task
WDT) mid-session means a watched task stopped feeding.

Then one line per second:

```
PLAY sync rate=6000.000 Hz blk=16 fill[6..22] target=6 under=0 resync=0 drop=0 ppm=16 dacerr=0 dacrun=0 missed=0
```

| Field | Meaning |
|---|---|
| `sync` / `nosync` | whether a valid header has ever been accepted |
| `rate`, `blk` | stream configuration in force, as the host sent it |
| `fill[min..max]` | ring occupancy over the window, in samples |
| `target` | what `min` is driven to; **this is the pass criterion** |
| `under` | ticks spent starved during the window |
| `resync` | stale samples discarded (gross backlog) |
| `drop` | cumulative producer-side drops (ring full) |
| `ppm` | current drift correction |
| `dacerr` | cumulative failed DAC writes since boot; **should be 0** |
| `dacrun` | current unbroken run of failed writes: 16 means muted now, a full second's worth means the enables are cut and a deliberate re-arm is needed |
| `missed` | timer ticks the DAC task was notified of but never serviced; **should be 0** |

In steady state `under` and `resync` should be 0 and `min` should sit at
`target` ± 1. At 375 and 750 Hz a `min` of 0 is normal, most ticks consume no
sample, so `under` is the criterion there.

Interleaved are `loop()`'s lines every 100 ms: `PINS status=<pin>
button=<pin>` and `Count1`/`Count2` with `Wrap1`/`Wrap2` beside them; the
counts are the **raw** hardware registers. A `Status:`-prefixed line is printed
**only on a state change**, so a repeat means the state really is flapping.
`FATAL:` once a second means an initialisation failure: no timer, or no DAC on
the bus, and the unit is halted with the drives released until a power cycle.

## Layout

| Path | |
|---|---|
| `src/irtactile-belt/irtactile-belt.ino` | tasks, timer ISR, DAC write path |
| `src/irtactile-belt/Pins.h` | every GPIO the sketch claims |
| `src/irtactile-belt/BeltState.h` | arming state machine: safety switch, mode button, preload |
| `src/irtactile-belt/Logging.h` | the log lines, and the seqlock the stats cross on |
| `src/irtactile-belt/SerialDecoder.h` | wire protocol: marker, header (rate + tuning), data frames |
| `src/irtactile-belt/SampleRing.h` | lock-free SPSC ring, 512 × 4 B |
| `src/irtactile-belt/Playout.h` | ZOH resampler, drift lock, silence ramp, tuning floor hold |
| `src/irtactile-belt/ForceCurve.h` | runtime encoder force curve: full-scale count + gamma LUT |
| `src/irtactile-belt/GP8413_DAC.h` | DAC driver over I²C |
| `src/irtactile-belt/Counter.h` | quadrature encoder counters (PCNT) |
| `src/irtactile-belt/WrapTracker.h` | folds PCNT wraps back into a usable displacement |
| `tests/` | host-side test suites |
| `tools/logger/` | second-ESP32 sketch: forwards `Serial1` to USB |
| `tools/DecoderTest/` | standalone PCNT counter sketch (encoders, not the decoder) |
| `tools/DacTest/` | standalone GP8413 bench sketch |
