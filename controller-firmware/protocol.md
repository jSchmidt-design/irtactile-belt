# Serial protocol

This document is the single source of truth for the wire protocol between the
host bridge ([`host-bridge/`](../host-bridge/)) and the belt controller
firmware ([`controller-firmware/`](.)).

Both sides must implement exactly the same framing and header semantics; a
version mismatch will cause the decoder to never sync.

## Framing

Frames are 5 bytes. High bytes are masked to `0x7F` upstream, so a run of four
`0xFF` cannot occur in data and the marker is an unambiguous escape.

### Data frame

```text
lo0  hi0  lo1  hi1  chk        chk = lo0 ^ hi0 ^ lo1 ^ hi1
```

`hi0` and `hi1` are limited to `0x7F`, giving an effective 15-bit sample per
channel.

### Header / sync

```text
FF FF FF FF                    sync marker
0x01                           frame type / version   (<= 0x7F)
rateCode                       3..10                  (<= 0x7F)
blkLo                          samplesPerBlock      & 0x7F
blkHi                          (samplesPerBlock >> 7) & 0x7F
chk                            (type ^ rateCode ^ blkLo ^ blkHi) & 0x7F
```

The rate is expressed as a ladder exponent:

```c
rateMilliHz = 48000000u >> rateCode;   /* 3 -> 6 kHz, 7 -> 375 Hz */
```

The ladder runs `rateCode` 0..10 (48 kHz down to 46.875 Hz). **Both sides accept
only codes 3..10** — 6000 / 3000 / 1500 / 750 / 375 / 187.5 / 93.75 / 46.875 Hz.
Codes 0-2 sit above the firmware's 6 kHz `DAC_TICK_HZ`, where the playout
resampler would become a decimator with no anti-alias filter, so they are
rejected: the bridge refuses them at attach; the firmware treats one as a bad
header, counts it in `headersRejected()`, and re-syncs on the next good header.

375 Hz is the practical floor the bridge documents. Codes 8-10 (187.5 / 93.75 /
46.875 Hz) are still accepted, but below the useful range for tactile output.

## Block size

`samplesPerBlock` is carried in two 7-bit fields, so the wire permits 1..16383
and the firmware decodes any of them. It is not a free parameter in practice:
the controller's playout ring is 512 entries, so a block larger than
`PLAYOUT_MAX_BLOCK` (64) cannot be buffered whole and the excess is dropped at
the producer, reported as `drop` in the controller's `PLAY` log line. The
firmware bounds its internal buffer sizing accordingly but still reports the
`blockSamples` it was sent, so the mismatch is diagnosable from the log alone.

Keep `samplesPerBlock` at or below 64 by lowering `shm.framesPerPublish`; the
bridge already warns when the resulting block carries excessive latency.

### Tuning header

A second header type shapes the encoder-derived force floor at runtime, without
a reflash. It is emitted by the separate [`belt-tune`](../host-bridge/README.md#belt-tune)
tool, never by the bridge.

```text
FF FF FF FF                    sync marker
0x02                           frame type: tuning     (<= 0x7F)
fsLo                           (fullScaleCounts / 2)        & 0x7F
fsHi                           ((fullScaleCounts / 2) >> 7) & 0x7F
gammaQ                         gamma * 16, Q3.4              (<= 0x7F)
chk                            (type ^ fsLo ^ fsHi ^ gammaQ) & 0x7F
```

Same fixed 5-byte payload, same checksum shape, every byte `<= 0x7F`, so the
sync-run invariant is unchanged and the decoder keys off `HEADER_LEN` exactly as
before.

- **`fullScaleCounts`** — encoder counts of pull at which the floor saturates.
  Carried in **2-count units**: two 7-bit fields give 0..16383 raw, i.e. 0..32766
  counts, covering the 20000 ceiling with headroom. The resolution cost is 2
  counts everywhere — negligible at the 6667 default, ~2 % at the 100 floor.
  `belt-tune` **rounds** on the way down (`(n + 1) / 2`) and the firmware
  multiplies straight back by 2, so the round trip is exact for even counts and
  off by at most one for odd ones. `6667` (the untuned default) is odd, so
  typing `6667` lands the device on `6668`; one count, imperceptible, but
  surprising to meet cold. The compile-time default and the persisted value
  never traverse the wire — an untuned device is exactly 6667.
- **`gammaQ`** — the ramp shape, `round(gamma * 16)` as Q3.4: fixed point with 3
  integer bits and 4 fractional bits, so the step is 1/16 = 0.0625 and the whole
  field is 7 bits — which is what keeps it `<= 0x7F`. Wire range 0..127 is gamma
  0..7.9375.

#### Clamping

The wire encoding permits values that are unsafe on a harness worn against a
torso, so **both** ends clamp:

| Parameter | Min | Max |
|---|---|---|
| gamma | 0.25 (`gammaQ` 4) | 7.9375 (`gammaQ` 127) |
| `fullScaleCounts` | 100 | 20000 |

`gammaQ = 0` would give a flat LUT at full scale — full commanded force at the
slightest encoder movement — and a near-zero `fullScaleCounts` is the same
hazard by another route. `belt-tune` clamps so it can report a rejected value to
the operator; the firmware clamps because it does not trust the host, and also
on the persistence load path so one bad value cannot brick a device into
full-force-on-touch with no host attached.

#### Hold semantics

`belt-tune` cannot feed the sample stream, so while it is connected the playout
starves and the encoder floor would normally collapse within ~300 ms. A valid
tuning header instead **holds the floor gain up** for `TUNING_HOLD` (~250 ms).
The hold is a refreshed deadline, never a latched state: each header pushes the
deadline forward and it lapses on its own when the headers stop — unplug the USB
mid-pull and the belt releases by itself, exactly as a dead host does today.

This is why `belt-tune` resends at 20 Hz by default (5 missed headers of
tolerance, ~300 ms release — identical to the current dead-host latency). A
slower cadence forces the hold up and makes the release measurably slower.

The header checksum is a 7-bit XOR, so a corrupted frame can pass and assert the
hold; the exposure is bounded to one hold window by the deadline, and is another
reason the hold must not latch.

## Sync-run handling

The sync run can be five bytes, not four. A data frame whose checksum is `0xFF`
immediately before a marker produces `0xFF` × 5. A naive four-byte sliding window
would match one byte early and mis-parse the header. The decoder therefore
consumes the entire leading `0xFF` run and reads the type from the first
non-`0xFF` byte. Every payload byte is guaranteed to be `<= 0x7F`, so this is
unambiguous.

## Header cadence

The bridge repeats the header at approximately 10 Hz. This makes the link
self-healing: an ESP32 that resets mid-session re-acquires sync within one
header interval.

## Checksumming

- Data frame: XOR of all four payload bytes.
- Rate header: XOR of `type`, `rateCode`, `blkLo`, `blkHi`, masked to `0x7F`.
- Tuning header: XOR of `type`, `fsLo`, `fsHi`, `gammaQ`, masked to `0x7F`.

A checksum mismatch discards the frame/header and the decoder continues scanning
for the next sync marker.
