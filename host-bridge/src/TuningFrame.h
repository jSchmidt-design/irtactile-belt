#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

// Encoder for the "tuning header" - the second header type on the marker-sync
// serial protocol (controller-firmware/protocol.md is the source of truth).
//
//   FF FF FF FF   sync
//   0x02          frame type: tuning                        (<= 0x7F)
//   fsLo          (fullScaleCounts / 2)        & 0x7F
//   fsHi          ((fullScaleCounts / 2) >> 7) & 0x7F        -- 14 bits, 2-count units
//   gammaQ        gamma * 16                                 -- Q3.4
//   chk           (type ^ fsLo ^ fsHi ^ gammaQ) & 0x7F
//
// Same 5-byte payload and checksum shape as the data/rate header, every byte
// <= 0x7F, so the marker/run-detection invariant is untouched.
//
// fullScaleCounts travels in 2-count units so two 7-bit fields cover the 20000
// ceiling: the resolution cost is 2 counts everywhere. encode() rounds rather
// than truncates, so the round trip is exact for even counts and off by at most
// one for odd ones.
//
// The host clamps so the tool can report a rejected value; the firmware clamps
// again and does not trust the host.
namespace tuning {

inline constexpr uint8_t kFrameType{0x02};

// Bytes a tuning frame occupies: 4 sync + 5 payload. Matches the rate header.
inline constexpr size_t kFrameBytes{9};

// Bounds, mirrored from the firmware's ForceCurve.h. gamma 0 gives a flat LUT
// at full scale (full force at the slightest movement); a near-zero full-scale
// is the same hazard by another route.
inline constexpr uint16_t kMinFullScaleCounts{100};    // a deliberately tight belt
inline constexpr uint16_t kMaxFullScaleCounts{20000};  // the firmware's historical maxCounter
inline constexpr uint8_t  kMinGammaQ{4};                // gamma 0.25
inline constexpr uint8_t  kMaxGammaQ{127};              // gamma 7.9375, the Q3.4 wire maximum

// The firmware's own defaults, duplicated so the tool can show the operator
// where an untuned device starts. They never travel the wire.
inline constexpr uint16_t kDefaultFullScaleCounts{6667};
inline constexpr uint8_t  kDefaultGammaQ{16};           // gamma 1.0

// gammaQ = round(gamma * 16) - Q3.4, i.e. 3 integer bits and 4 fractional, so
// steps of 1/16. Clamped to [kMinGammaQ, kMaxGammaQ]; a NaN or a value at or
// below the minimum returns kMinGammaQ.
[[nodiscard]] uint8_t gammaToWire(float gamma) noexcept;

// Builds the 9-byte sync marker + tuning header. fullScaleCounts is clamped to
// [kMinFullScaleCounts, kMaxFullScaleCounts] and gammaQ to
// [kMinGammaQ, kMaxGammaQ] before encoding.
[[nodiscard]] std::array<uint8_t, kFrameBytes> encode(uint16_t fullScaleCounts,
                                                      uint8_t gammaQ) noexcept;

} // namespace tuning
