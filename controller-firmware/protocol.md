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
- Header: XOR of `type`, `rateCode`, `blkLo`, `blkHi`, masked to `0x7F`.

A checksum mismatch discards the frame/header and the decoder continues scanning
for the next sync marker.
