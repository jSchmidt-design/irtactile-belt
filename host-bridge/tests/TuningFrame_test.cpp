#include <doctest/doctest.h>

#include "TuningFrame.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>

// Pins the tuning header byte-for-byte (see controller-firmware/protocol.md):
//   FF FF FF FF, 0x02, fsLo, fsHi, gammaQ, chk
// where fsLo/fsHi carry fullScaleCounts/2 across two 7-bit fields and
// chk = (type ^ fsLo ^ fsHi ^ gammaQ) & 0x7F.

namespace {

// Undoes the wire scaling exactly as the firmware decoder does: reassemble the
// 14-bit field, then multiply by 2.
uint16_t decodeFullScale(const std::array<uint8_t, tuning::kFrameBytes>& f)
{
    const uint16_t half{static_cast<uint16_t>(f[5] | (f[6] << 7))};
    return static_cast<uint16_t>(half << 1);
}

} // namespace

TEST_CASE("TuningFrame encodes the marker, type, fields and checksum")
{
    const auto f{tuning::encode(6668, 16)};   // even count, gamma 1.0

    REQUIRE(f.size() == 9);
    CHECK(f[0] == 0xFF);
    CHECK(f[1] == 0xFF);
    CHECK(f[2] == 0xFF);
    CHECK(f[3] == 0xFF);
    CHECK(f[4] == tuning::kFrameType);

    const uint16_t half{3334};               // 6668 / 2
    CHECK(f[5] == (half & 0x7F));
    CHECK(f[6] == ((half >> 7) & 0x7F));
    CHECK(f[7] == 16);
    CHECK(f[8] == static_cast<uint8_t>((tuning::kFrameType ^ f[5] ^ f[6] ^ f[7]) & 0x7F));
}

TEST_CASE("TuningFrame keeps every payload byte 7-bit")
{
    // The largest fields the wire permits must still not extend a marker run.
    const auto f{tuning::encode(tuning::kMaxFullScaleCounts, tuning::kMaxGammaQ)};
    for (size_t i{4}; i < f.size(); ++i)
        CHECK(f[i] <= 0x7F);
}

TEST_CASE("TuningFrame clamps fullScaleCounts to both bounds")
{
    const auto low{tuning::encode(0, 16)};
    CHECK(decodeFullScale(low) == tuning::kMinFullScaleCounts);

    const auto high{tuning::encode(65535, 16)};
    CHECK(decodeFullScale(high) == tuning::kMaxFullScaleCounts);

    // Just inside each bound passes through (to within the 2-count wire step).
    const auto atMin{tuning::encode(tuning::kMinFullScaleCounts, 16)};
    CHECK(decodeFullScale(atMin) == tuning::kMinFullScaleCounts);
}

TEST_CASE("TuningFrame clamps gammaQ to the safety floor and the wire ceiling")
{
    CHECK(tuning::encode(6000, 0)[7]   == tuning::kMinGammaQ);
    CHECK(tuning::encode(6000, 1)[7]   == tuning::kMinGammaQ);
    CHECK(tuning::encode(6000, 255)[7] == tuning::kMaxGammaQ);
    CHECK(tuning::encode(6000, 40)[7]  == 40);   // in range, untouched
}

TEST_CASE("TuningFrame round-trips the ceiling exactly")
{
    // 20000 is even, so the halved wire field loses nothing.
    CHECK(decodeFullScale(tuning::encode(20000, 16)) == 20000);
}

TEST_CASE("TuningFrame rounds an odd count to within one")
{
    const uint16_t decoded{decodeFullScale(tuning::encode(6667, 16))};
    CHECK(decoded == 6668);                       // (6667 + 1) / 2 * 2
    CHECK(std::abs(static_cast<int>(decoded) - 6667) <= 1);
}

TEST_CASE("gammaToWire maps gamma to Q3.4 and clamps the range")
{
    CHECK(tuning::gammaToWire(1.0f)  == 16);
    CHECK(tuning::gammaToWire(0.5f)  == 8);
    CHECK(tuning::gammaToWire(2.0f)  == 32);
    CHECK(tuning::gammaToWire(0.625f) == 10);     // 10.0 exactly

    // Below the safety floor (gamma 0.25) and above the wire ceiling.
    CHECK(tuning::gammaToWire(0.0f)   == tuning::kMinGammaQ);
    CHECK(tuning::gammaToWire(0.1f)   == tuning::kMinGammaQ);
    CHECK(tuning::gammaToWire(-3.0f)  == tuning::kMinGammaQ);
    CHECK(tuning::gammaToWire(99.0f)  == tuning::kMaxGammaQ);

    // NaN routes to the floor rather than an undefined float->int conversion.
    CHECK(tuning::gammaToWire(std::nanf("")) == tuning::kMinGammaQ);
}
