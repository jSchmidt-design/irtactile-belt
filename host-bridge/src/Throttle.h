#pragma once
#include <chrono>

/// Rate limiter: one line per interval for a condition that recurs every wake.
///
/// The caller supplies the clock reading rather than the class taking one, so a
/// drain loop that already holds a `now` reuses it across several throttles -
/// and so the behaviour is testable without sleeping.
class Throttle {
public:
    explicit Throttle(std::chrono::steady_clock::duration interval) noexcept
        : m_interval{interval} {}

    /// True when the interval has elapsed since the last report, or after a
    /// reset(). Arms the next interval.
    [[nodiscard]] bool due(std::chrono::steady_clock::time_point now) noexcept
    {
        if (m_armed && now - m_last < m_interval) return false;
        m_last  = now;
        m_armed = true;
        return true;
    }

    /// Makes the next due() report immediately.
    void reset() noexcept { m_armed = false; }

private:
    std::chrono::steady_clock::duration   m_interval;
    std::chrono::steady_clock::time_point m_last{};
    bool                                  m_armed{false};
};
