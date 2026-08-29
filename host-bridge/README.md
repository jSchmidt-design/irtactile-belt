# host-bridge

Windows console process that forwards an irTactile **shared-memory stream** to
a **serial port**. It attaches to a stream published over the
[irtactile-shm](https://github.com/jSchmidt-design/irtactile-shm) interface,
picks two channels, encodes them in the wire format the
[controller firmware](../controller-firmware/) speaks, and writes the bytes to
a COM port.

```
irTactile engine ──► shm section + event
                              │
                    irTactileSerialBridge
                              │  5 bytes / frame @ baud
                     CH340 ──► ESP32 ──► DAC ──► AASD-15A drives
```

The bridge is **not** launched by the engine; run it by hand, from a shortcut
or from a startup script. It is a plain shm consumer: it never writes the
section and holds no state the engine can see. The link is **one-way,
host → device**; the port is opened `GENERIC_WRITE` only.

## Building

```sh
cmake -S . -B build
cmake --build build --config Release --target irTactileSerialBridge
```

Output: `build/bin/Release/irTactileSerialBridge.exe`. The bridge links
`irtactile::stream-shm` (header-only) and `shmlog::logger` only; no wx, no
PortAudio, no engine code. Both are fetched from GitHub on first configure;
`-DIRTACTILE_SHM_LOCAL_DIR=` and `-DSHMLOG_LOCAL_DIR=` point at local
checkouts instead. Tests are **on by default**, so a plain configure also
fetches doctest; `-DHOST_BRIDGE_BUILD_TESTS=OFF` skips both.

## Running

```sh
./build/bin/Release/irTactileSerialBridge.exe \
    --port COM6 \
    --shm-name irTactile_Stream_Primary_256k \
    --event-name irTactile_Stream_Primary_Event_256k
```

`--port` and `--shm-name` are required; everything else has a default.

| Option | Default | Notes |
|---|---|---|
| `--port <COMx>` | *required* | Opened as `\\.\COMx`, so `COM10`+ works. |
| `--shm-name <name>` | *required* | Resolved section name, **including size suffix**. |
| `--event-name <name>` | *(none)* | Resolved event name. Omitted or empty selects polling mode: same data, higher latency. |
| `--baud <n>` | `1200000` | CH340G is good to 2 Mbaud; a CP2102 is spec'd to 921600. |
| `--channels <a,b>` | `0,1` | Stream channel indices feeding protocol slots 1 and 2. Must differ and must exist. |
| `--marker-interval <n>` | *auto* | Frames between sync markers; default `rateHz / 10` (~10 Hz cadence). At most one marker per block. |
| `--legacy-marker` | off | Emit the bare 4×`0xFF` marker with no header, for firmware predating the in-band header. |
| `--log-name <name>` | `irTactile_Belt_Bridge_Log` | shmlog partition for diagnostics. |
| `--log-source <id>` | `0` | shmlog source id, 0-255. |
| `--help`, `-h`, `/?` | | Usage text. |

### Names

`--shm-name` and `--event-name` must name the same section and event irTactile
publishes to. Those are the **resolved** names: the profile's `shm.shmName` /
`shm.eventName` plus a size-class suffix (`_256k`, `_512k`, `_1m`) the producer
appends from the total section size. The producer logs them at open and the
Editor's device configuration panel shows them; copy from there. Pin
`"sizeClass"` in the profile to stop the suffix moving when channel count,
block size or ring depth change. An empty `shm.eventName` means polling only.

A mismatch is not loud: the open returns `NotFound`, the same as "irTactile is
not running yet", and the bridge waits indefinitely.

Two other profile keys matter here: `shm.sampleRate` must be a ladder rate, and
`shm.framesPerPublish` is the dominant latency term; set it to the smallest
value the rate divides evenly (1 at 375 Hz and above). The bridge warns when
the stream publishes slower than the rate allows, but it is only a warning: the
block size travels in the marker, so the firmware decodes any block size.

## Wire protocol

Full reference: [`controller-firmware/protocol.md`](../controller-firmware/protocol.md). In brief: 5-byte
data frames plus a 9-byte sync marker carrying the rate and the block size
in-band. The header repeats at ~10 Hz, so a receiver that resets relocks
within ~100 ms with no handshake.

The rate travels as a ladder code, where `rateMilliHz == 48'000'000 >> rateCode`.
Codes 3-10 are accepted:

| rateCode | Rate | rateCode | Rate |
|---|---|---|---|
| 3 | 6000 Hz | 7 | 375 Hz |
| 4 | 3000 Hz | 8 | 187.5 Hz |
| 5 | 1500 Hz | 9 | 93.75 Hz |
| 6 | 750 Hz | 10 | 46.875 Hz |

Codes 0-2 (48/24/12 kHz) are rejected: they sit above the firmware's 6 kHz DAC tick.
An off-ladder rate is rejected at attach, because the header cannot represent
it.

**Bandwidth:** 5 bytes/frame plus one marker per interval; 8N1 gives
`baud / 10` bytes/s. At 1.2 Mbaud, 6000 Hz needs ~30 of 120 kB/s. The bridge
warns above 80 % of the budget (115200 baud tops out around 1.8 kHz).

## Latency

Four stages, at `framesPerPublish = 1`. Analytic figures, not bench
measurements.

| Stage | Budget |
|---|---|
| shm publish cadence | 2.67 ms |
| Windows COM TX buffer (256-byte queue) | ≤ 2 ms |
| USB OUT frame scheduling | ~1 ms, not tunable |
| ESP32 playout + DAC | ~3 ms |
| **Total** | **~7 ms** |

Only the first stage is tunable, and `framesPerPublish` is the knob: the
publish cadence is `P × 2.667 ms` at 375 Hz and above; the sample rate does
not appear, so rate flexibility is essentially free. Below 375 Hz a block
cannot hold less than one sample, so the cadence stretches to
`samplesPerBlock / rate`, up to 21.3 ms at the bottom of the ladder.

The TX buffer is deliberately sized at 256 bytes rather than the 8 KB driver
default, which would hold 68 ms of wire time at 1.2 Mbaud.

## Runtime behaviour

- **Neither irTactile nor the port has to be up first.** `NotFound`/`NotReady`
  are retried every 250 ms; a port that cannot be opened is retried every 2 s
  while the stream keeps draining. `BadVersion`, `Malformed` and
  `AccessDenied` are fatal.
- **Latency is bounded ahead of completeness.** When the driver's TX queue
  backs up the bridge skips to the newest block rather than falling behind,
  and counts what it skipped. Write failures are non-fatal: after 4 in a row
  the port is dropped, output discarded and a reopen attempted every 2 s.
- **Reconfiguration is picked up live.** A geometry or rate change re-runs the
  plausibility checks and re-arms the header. An unusable stream exits with
  code 3; a clean producer shutdown drains the ring and logs a counter
  summary.
- **DTR/RTS are held deasserted.** On an ESP32 devkit they drive the
  auto-reset transistors, so driver defaults would reboot the target.
- Every repeating condition reports at most once per 5 s. `Quiet` (engine up,
  this device not driven) is silent.

## Logging

All runtime diagnostics go through [shmlog](https://github.com/jSchmidt-design/shmlog),
so a bridge running alongside irTactile lands in the same merged,
timestamp-ordered log as the engine. The destination is a **build-time**
choice:

```sh
cmake -S . -B build                                    # shm (default)
cmake -S . -B build-console -DHOST_BRIDGE_LOG_BACKEND=stdout
cmake -S . -B build-quiet   -DHOST_BRIDGE_LOG_BACKEND=null
```

| Backend | Behaviour |
|---|---|
| `shm` *(default)* | Into the named partition. **The console stays empty**: a collector must be attached to see anything. |
| `stdout` | One line per entry on stdout. Use this when running by hand. |
| `null` | Discarded. |

```
[2026-08-17 19:26:03.503487] [7      ] [INFO ] [Thread 17432] waiting for irTactile (NotFound)...
 └ local time, µs precision  └ source id  └ level  └ OS thread id  └ message
```

The partition is created lazily with an unqualified name, so it lives in the
caller's session namespace and is readable only by the creating user. Two
bridges side by side need different `--log-name`, or at least different
`--log-source`. If `Initialize` fails, the bridge prints one line to stderr and
runs with logging disabled rather than exiting. `--help` and argument errors
always go to the console, because they happen before the logger exists.

## Exit codes

| Code | Meaning |
|---|---|
| 0 | Producer closed cleanly, or `--help`. |
| 1 | Fatal shm open failure. An unavailable serial port is **not** fatal. |
| 2 | Bad arguments. |
| 3 | Plausibility check failed, at attach or on a later reconfiguration. |

## Troubleshooting

| Symptom | Cause |
|---|---|
| No output at all from a running bridge | Default `shm` backend with no collector attached. Rebuild with `-DHOST_BRIDGE_LOG_BACKEND=stdout`, or attach a collector to `--log-name`. Check the session/account too. |
| `waiting for irTactile (NotFound)...` forever, engine up | Size-class suffix on `--shm-name` is wrong. Pin `"sizeClass"` in the profile. |
| `event "..." not found -- falling back to polling` | Resolved event name is wrong, or the profile has `"eventName": ""`. |
| `stream rate ... is not one of the 11 stream ladder rates` | Hand-edited profile. Pick a ladder rate. |
| `stream rate ... exceeds the 6 kHz firmware maximum` | Lower the profile rate. |
| `wire format needs ... bytes/s` | Raise `--baud` or lower the rate. |
| `stream publishes at ... Hz` | `framesPerPublish` is above the floor for this rate. Lower it. |
| `stream publishes ... more than the 16383 the marker header can carry` | `framesPerPublish` far above the floor, so the block size would truncate. Lower it. |
| `COMx unusable after 4 consecutive failed writes` | Adapter unplugged, or another process took the port. It keeps retrying every 2 s. |
| `COMx not available at start` | Same causes before the first write, or a wrong `--port`. It keeps retrying every 2 s. |
| Silence, or a burst-then-hold rhythm | Firmware predates the in-band header. Use `--legacy-marker`, or update the firmware. |
| ESP32 reboots when the bridge starts | Build predates the DTR/RTS fix in `SerialPort::open`. |

## Tests

```sh
test.bat
```

or via CMake/CTest:

```sh
cmake -S . -B build
cmake --build build --config Release --target irTactileSerialBridge_tests
ctest --test-dir build -C Release --output-on-failure
```

[`SerialProtocol_test.cpp`](tests/SerialProtocol_test.cpp) covers the wire
format byte-for-byte, [`WireParams_test.cpp`](tests/WireParams_test.cpp) the
rate ladder and the derived wire arithmetic,
[`BridgeConfig_test.cpp`](tests/BridgeConfig_test.cpp) the command line and
[`Throttle_test.cpp`](tests/Throttle_test.cpp) the log rate limiting. All four
are pure: no board, no port, no live shm section. `SerialPort` and `main.cpp`
are not unit-tested; both are thin layers over Win32 handles and the shm
client.

## Layout

| Path | |
|---|---|
| `src/main.cpp` | attach, drain loop, failure handling |
| `src/BridgeConfig.*` | command-line parsing |
| `src/WireParams.*` | rate ladder, marker cadence, backlog threshold, sample scaling |
| `src/SerialProtocol.*` | frame and header encoding |
| `src/SerialPort.*` | Win32 COM port |
| `src/Throttle.h` | log rate limiting |
| `tests/` | doctest suites |
