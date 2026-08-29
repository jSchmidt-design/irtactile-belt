#pragma once
#include <cstddef>
#include <cstdint>

// The rate ladder, the derived wire parameters and the sample scaling: every
// decision the bridge makes about *what* to put on the wire, separated from the
// shm stream and the serial port it reads and writes.
//
// Nothing here touches irtactile-shm, shmlog or windows.h - the stream-facing
// callers pass the two header fields these need (samplesPerBlock, the rate) as
// plain values. That keeps the arithmetic testable natively, which matters
// because a wrong ladder exponent or marker interval is silent: the firmware
// just plays at the wrong rate.

/// Highest sample rate the firmware handles, in milli-Hz. 12 kHz is on the
/// ladder but unchecked against the wire budget and the ESP32's per-tick I2C
/// budget.
inline constexpr uint32_t kMaxSampleRateMilliHz{6'000'000};
/// Top of the ladder in milli-Hz. Every ladder rate is this shifted right by
/// its rateCode, exactly, for codes 0..10 (48 kHz down to 46.875 Hz).
inline constexpr uint32_t kLadderTopMilliHz{48'000'000};
inline constexpr uint8_t  kMaxRateCode{10};
/// Fastest publish cadence the ladder permits: 48 kHz / 128 frames, reached at
/// framesPerPublish = 1. Below 375 Hz a block cannot hold less than one sample,
/// so there the rate itself is the ceiling.
inline constexpr double kMaxPublishHz{375.0};
/// Warn when the wire format needs more than this fraction of the baud budget.
inline constexpr double kBandwidthWarnFraction{0.8};
/// Target marker cadence when --marker-interval is not given.
inline constexpr double kMarkerCadenceHz{10.0};
/// TX queue size the port requests from the driver, in bytes. Whatever the
/// driver has accepted but not yet clocked out is added latency, and the queue
/// only ever empties at line rate: 256 bytes caps that at ~2 ms @1.2 Mbaud,
/// against the 68 ms an 8 KB default would hold. A plain number with a latency
/// rationale, so it lives here rather than behind windows.h - SerialPort asks
/// the driver for it, backlogThresholdBytes() budgets against it, and the
/// tests need it without either.
inline constexpr uint32_t kTxQueueBytes{256};

/// Resolves the ladder exponent the header carries: rateMilliHz ==
/// 48'000'000 >> code, exact for all 11 ladder rates, so the firmware recovers
/// the rate with a shift. Off-ladder rates have no code and return false.
[[nodiscard]] bool rateCodeFor(uint32_t rateMilliHz, uint8_t& out) noexcept;

/// Returns the marker cadence in frames. A non-zero `explicitInterval` (from
/// --marker-interval) wins; otherwise the interval scales with the rate for
/// ~kMarkerCadenceHz. The encoder emits at most one marker per encode() call,
/// so an interval below samplesPerBlock caps the real cadence at the publish
/// rate.
[[nodiscard]] size_t markerIntervalFor(size_t explicitInterval, double rateHz) noexcept;

/// Returns the wire backlog above which the drain switches to
/// latest-value-wins: one block of wire bytes, floored at the ~1 ms of USB
/// full-speed frame scheduling that legitimately sits in the queue at any
/// instant, and capped below the TX queue so the threshold stays reachable -
/// an unreachable one would never fire and the drain would meet a full queue
/// as blocking writes instead of dropping.
///
/// `txQueueBytes` is the driver's TX queue size - kTxQueueBytes above in the
/// bridge, a parameter here so the clamp's two degenerate ends stay testable.
[[nodiscard]] uint32_t backlogThresholdBytes(uint32_t samplesPerBlock, int baud,
                                             uint32_t txQueueBytes) noexcept;

/// Scales one stream sample to the 15-bit unsigned range the wire format
/// carries.
///
/// The stream contract is 0..1, so this is defensive on both ends. Clamping
/// keeps an out-of-range sample from wrapping into a neighbouring sample's high
/// bits. The test is written as `!(v > 0.f)` rather than std::clamp because
/// clamp is specified as `comp(v, lo) ? lo : comp(hi, v) ? hi : v` - both
/// comparisons are false for a NaN, so it returns the NaN unchanged, and
/// converting a NaN to an integer type is undefined behaviour. Inverting the
/// first comparison routes NaN to silence instead.
[[nodiscard]] inline uint16_t toWireSample(float v) noexcept
{
    if (!(v > 0.f)) return 0;                       // <= 0, or NaN
    if (v >= 1.f)   return 0x7FFF;
    return static_cast<uint16_t>(v * 0x7FFF);
}
