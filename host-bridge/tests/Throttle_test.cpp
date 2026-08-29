#include <doctest/doctest.h>

#include "Throttle.h"

#include <chrono>

// Throttle is what keeps a condition that recurs on every wake - a skipped
// block, a stalled producer, a dead port - from scrolling a whole session out
// of the log ring. Too permissive and the ring is useless; too strict and an
// outage goes unreported.
//
// The clock is a parameter, so these run without sleeping.

using namespace std::chrono_literals;

namespace {
constexpr std::chrono::steady_clock::time_point kT0{};
}

TEST_CASE("Throttle reports the first call and then holds")
{
    Throttle t{5s};

    CHECK(t.due(kT0));              // nothing reported yet
    CHECK_FALSE(t.due(kT0));
    CHECK_FALSE(t.due(kT0 + 1s));
    CHECK_FALSE(t.due(kT0 + 4999ms));
}

TEST_CASE("Throttle reports again once the interval has elapsed")
{
    Throttle t{5s};

    REQUIRE(t.due(kT0));
    // The boundary is inclusive: exactly one interval later is due.
    CHECK(t.due(kT0 + 5s));
    CHECK_FALSE(t.due(kT0 + 5s));
    CHECK_FALSE(t.due(kT0 + 9999ms));
    CHECK(t.due(kT0 + 10s));
}

TEST_CASE("Throttle measures from the last report, not from a fixed grid")
{
    Throttle t{5s};

    REQUIRE(t.due(kT0));
    // A report at t=7s rearms from 7s, so 11s is still inside the interval
    // even though it is past the 10s a fixed grid would have allowed.
    REQUIRE(t.due(kT0 + 7s));
    CHECK_FALSE(t.due(kT0 + 11s));
    CHECK(t.due(kT0 + 12s));
}

TEST_CASE("Throttle survives a long silence without banking reports")
{
    Throttle t{5s};

    REQUIRE(t.due(kT0));
    // An hour with nothing to say earns exactly one line, not 720.
    CHECK(t.due(kT0 + 1h));
    CHECK_FALSE(t.due(kT0 + 1h));
}

TEST_CASE("reset makes the next call report immediately")
{
    Throttle t{5s};

    REQUIRE(t.due(kT0));
    REQUIRE_FALSE(t.due(kT0 + 1s));

    // The drain calls this when the layout changed under it: the next anomaly
    // is a new one and must not be swallowed by the previous interval.
    t.reset();
    CHECK(t.due(kT0 + 1s));
    // ...and the interval re-arms from there.
    CHECK_FALSE(t.due(kT0 + 2s));
    CHECK(t.due(kT0 + 6s));
}

TEST_CASE("reset before any report is harmless")
{
    Throttle t{5s};
    t.reset();
    CHECK(t.due(kT0));
    CHECK_FALSE(t.due(kT0));
}

TEST_CASE("a zero interval reports every call")
{
    // Not used by the bridge, but the boundary must not wedge: now - last is
    // never < 0, so every call is due.
    Throttle t{0s};
    CHECK(t.due(kT0));
    CHECK(t.due(kT0));
    CHECK(t.due(kT0 + 1s));
}
