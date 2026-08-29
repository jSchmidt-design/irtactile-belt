#include <doctest/doctest.h>

#include "WireParams.h"
#include "SerialProtocol.h"

#include <cmath>
#include <cstdint>
#include <limits>

// The rate ladder, the derived marker cadence, the backlog threshold and the
// sample scaling. None of it is observable from the host: a wrong ladder
// exponent or a wrong marker interval produces a stream the firmware accepts
// and plays at the wrong rate.

TEST_CASE("rateCodeFor resolves every ladder rate to its exponent")
{
    // 48 kHz >> code, for codes 0..10. The firmware recovers the rate with the
    // same shift, so this table is the contract.
    const uint32_t ladder[]{48'000'000, 24'000'000, 12'000'000, 6'000'000,
                            3'000'000,  1'500'000,  750'000,    375'000,
                            187'500,    93'750,     46'875};

    for (uint8_t code{0}; code <= kMaxRateCode; ++code) {
        uint8_t out{0xFF};
        REQUIRE(rateCodeFor(ladder[code], out));
        CHECK(out == code);
        // The shift the firmware performs must round-trip exactly.
        CHECK((kLadderTopMilliHz >> out) == ladder[code]);
    }
}

TEST_CASE("rateCodeFor rejects off-ladder rates")
{
    uint8_t out{0xFF};
    // Between ladder steps, and either side of the ends.
    CHECK_FALSE(rateCodeFor(44'100'000, out));
    CHECK_FALSE(rateCodeFor(8'000'000, out));
    CHECK_FALSE(rateCodeFor(4'000'000, out));
    CHECK_FALSE(rateCodeFor(0, out));
    CHECK_FALSE(rateCodeFor(96'000'000, out));
    // One below the bottom rung; the loop must not run past kMaxRateCode.
    CHECK_FALSE(rateCodeFor(46'875 / 2, out));
    // A rejected lookup leaves the caller's value alone.
    CHECK(out == 0xFF);
}

TEST_CASE("rateCodeFor covers exactly the 11 ladder rates and nothing else")
{
    // Sweeps every rate a shift of the ladder top can produce, including the
    // inexact ones past code 10, and counts the acceptances.
    size_t accepted{0};
    for (uint8_t shift{0}; shift < 32; ++shift) {
        uint8_t out{0};
        if (rateCodeFor(kLadderTopMilliHz >> shift, out)) ++accepted;
    }
    CHECK(accepted == 11);
}

TEST_CASE("markerIntervalFor honours an explicit override at any rate")
{
    CHECK(markerIntervalFor(1, 3000.0) == 1);
    CHECK(markerIntervalFor(600, 6000.0) == 600);
    // The override wins even where it is absurd for the rate - the operator
    // asked for it, and checkStream() warns on the resulting wire budget.
    CHECK(markerIntervalFor(100'000, 46.875) == 100'000);
}

TEST_CASE("markerIntervalFor derives a ~10 Hz cadence from the rate")
{
    // frames = rate / 10, so the marker lands ten times a second at every rate
    // where a block can hold that many frames.
    CHECK(markerIntervalFor(0, 6000.0) == 600);
    CHECK(markerIntervalFor(0, 3000.0) == 300);
    CHECK(markerIntervalFor(0, 375.0) == 37);      // truncated, not rounded
    CHECK(markerIntervalFor(0, 93.75) == 9);
}

TEST_CASE("markerIntervalFor floors at one frame below the cadence")
{
    // Under 10 Hz the ideal interval is a fraction of a frame; 0 would mean a
    // marker before every frame and a division by zero in the wire budget.
    CHECK(markerIntervalFor(0, 46.875) == 4);
    CHECK(markerIntervalFor(0, 10.0) == 1);
    CHECK(markerIntervalFor(0, 9.9) == 1);
    CHECK(markerIntervalFor(0, 0.001) == 1);
    CHECK(markerIntervalFor(0, 0.0) == 1);
}

TEST_CASE("backlogThresholdBytes prefers one block of wire bytes")
{
    // 48 samples * 5 bytes = 240, above the ~1 ms USB floor (120 at 1.2 Mbaud)
    // and below the 192-byte ceiling... which caps it.
    CHECK(backlogThresholdBytes(48, 1'200'000, kTxQueueBytes) == 192);
    // 16 samples * 5 = 80, under the 120-byte USB floor, so the floor wins.
    CHECK(backlogThresholdBytes(16, 1'200'000, kTxQueueBytes) == 120);
    // 32 samples * 5 = 160: above the floor, below the ceiling, used as-is.
    CHECK(backlogThresholdBytes(32, 1'200'000, kTxQueueBytes) == 160);
}

TEST_CASE("backlogThresholdBytes stays inside the TX queue")
{
    // An unreachable threshold would never fire, and the drain would meet a
    // full queue as blocking writes instead of dropping stale blocks.
    for (uint32_t samples : {1u, 8u, 64u, 512u, 100'000u}) {
        const uint32_t t{backlogThresholdBytes(samples, 1'200'000, kTxQueueBytes)};
        CHECK(t >= 1);
        CHECK(t <= kTxQueueBytes * 3 / 4);
    }
}

TEST_CASE("backlogThresholdBytes never returns a zero threshold")
{
    // A zero block size at a baud too low to fill a USB frame drives both
    // inputs to 0; the result still has to be a byte count the queue can pass.
    CHECK(backlogThresholdBytes(0, 9600, kTxQueueBytes) == 1);
    CHECK(backlogThresholdBytes(0, 0, kTxQueueBytes) == 1);
    // A queue too small to have a 3/4 point must not invert the clamp bounds.
    CHECK(backlogThresholdBytes(48, 1'200'000, 1) == 1);
    CHECK(backlogThresholdBytes(48, 1'200'000, 0) == 1);
}

TEST_CASE("toWireSample maps the 0..1 contract onto 15 bits")
{
    CHECK(toWireSample(0.f) == 0);
    CHECK(toWireSample(1.f) == 0x7FFF);
    CHECK(toWireSample(0.5f) == static_cast<uint16_t>(0.5f * 0x7FFF));
    CHECK(toWireSample(0.25f) == static_cast<uint16_t>(0.25f * 0x7FFF));
    // Never wider than the 15 bits the wire format carries: a 16th bit would
    // land in the high byte the encoder masks to 0x7F.
    CHECK(toWireSample(1.f) <= 0x7FFF);
}

TEST_CASE("toWireSample clamps out-of-contract samples instead of wrapping")
{
    // Without the clamp these wrap into a neighbouring sample's high bits.
    CHECK(toWireSample(1.5f) == 0x7FFF);
    CHECK(toWireSample(1000.f) == 0x7FFF);
    CHECK(toWireSample(-0.5f) == 0);
    CHECK(toWireSample(-1000.f) == 0);
    CHECK(toWireSample(std::numeric_limits<float>::infinity()) == 0x7FFF);
    CHECK(toWireSample(-std::numeric_limits<float>::infinity()) == 0);
}

TEST_CASE("toWireSample maps NaN to silence rather than undefined behaviour")
{
    // std::clamp would return the NaN unchanged - both of its comparisons are
    // false - and converting a NaN to an integer type is undefined behaviour.
    //
    // Read this as pinning the *defined* result, not as a regression alarm:
    // reverting to the clamp still passes here, because x86-64 lowers the
    // conversion to cvttss2si, which yields the integer indefinite value
    // 0x80000000 - whose low 16 bits are also 0. That coincidence is the whole
    // hazard. It holds for one target at one optimisation level, and nothing
    // in the standard obliges the next compiler, /fp:fast, or a constant-folded
    // call to reproduce it. If this case ever fails, the conversion is being
    // done a different way and the clamp form would have been silently wrong.
    CHECK(toWireSample(std::numeric_limits<float>::quiet_NaN()) == 0);
    CHECK(toWireSample(-std::numeric_limits<float>::quiet_NaN()) == 0);
    CHECK(toWireSample(std::nanf("")) == 0);
}

TEST_CASE("toWireSample agrees with the clamped scaling it replaced")
{
    // Pins the finite-input behaviour byte-for-byte against the original
    // expression, so the NaN fix cannot have shifted the audible range.
    for (int i{-100}; i <= 1100; ++i) {
        const float v{static_cast<float>(i) / 1000.f};
        const float clamped{v < 0.f ? 0.f : (v > 1.f ? 1.f : v)};
        CHECK(toWireSample(v) == static_cast<uint16_t>(clamped * 0x7FFF));
    }
}
