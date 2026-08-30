// Off-target simulation of WrapTracker.h: PCNT wrap handling for the encoder
// torque floor.
//
// ESP32 PCNT resets the counter to 0 on reaching counter_h_lim (+32767) or
// counter_l_lim (-32768): it neither rolls over modulo 2^16 nor saturates, and
// the two limits are one count apart in magnitude. PcntModel reproduces that.
#include <cstdio>
#include <cstdint>

#include "WrapTracker.h"
#include "ForceCurve.h"

static int g_fail = 0;
#define CHECK(cond) do { if (!(cond)) { printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } } while (0)

// The force curve's default saturation point, taken from ForceCurve.h rather
// than hand-duplicated.
static const int32_t FULL_SCALE_COUNTS = DEFAULT_FULL_SCALE_COUNTS;

struct PcntModel {
    int32_t raw = 0;    // the 16 bit hardware register
    int32_t truth = 0;  // unbounded position

    // One move of up to a wrap's worth. Larger single-tick moves are neither
    // representable here nor physical (~400x the glitch filter's ceiling).
    void step(int32_t counts) {
        if (counts > WRAP_SPAN_UP) counts = WRAP_SPAN_UP;
        if (counts < -WRAP_SPAN_UP) counts = -WRAP_SPAN_UP;
        raw += counts;
        truth += counts;
        if (raw >= 32767) raw -= WRAP_SPAN_UP;      // h_lim: reset to 0, not saturate
        if (raw <= -32768) raw += WRAP_SPAN_DOWN;   // l_lim: likewise
    }
    int16_t read() const { return (int16_t)raw; }
};

// Move `counts` at `perTick` counts per DAC tick, feeding the tracker every
// tick the way processData() does. Returns the last tracked value.
static int16_t pull(PcntModel &pc, WrapTracker &t, int32_t counts, int32_t perTick) {
    int16_t out = t.update(pc.read());
    const int32_t dir = counts > 0 ? 1 : -1;
    int32_t left = counts > 0 ? counts : -counts;
    while (left > 0) {
        const int32_t n = left < perTick ? left : perTick;
        pc.step(dir * n);
        out = t.update(pc.read());
        left -= n;
    }
    return out;
}

int main() {
    // 1: below the wrap the tracker is transparent.
    {
        PcntModel pc; WrapTracker t;
        for (int32_t target = 100; target <= 30000; target += 100) {
            CHECK(pull(pc, t, 100, 4) == (int16_t)target);
        }
        printf("1: transparent to 30000 counts, wraps=%lu\n", (unsigned long)t.wraps());
        CHECK(t.wraps() == 0);
    }

    // 2: pulled through the wrap, the raw counter collapses to near zero
    //    (full force to none at maximum pull) while the tracked value holds.
    {
        PcntModel pc; WrapTracker t;
        const int16_t before = pull(pc, t, 32760, 4);
        const int16_t after = pull(pc, t, 20, 4);
        printf("2: across the wrap  raw %d -> %d   tracked %d -> %d\n",
               32760, (int)pc.read(), (int)before, (int)after);
        CHECK(pc.read() < FULL_SCALE_COUNTS);   // raw floor collapsed
        CHECK(before == 32760);                 // one count short of the wrap
        CHECK(after == 32767);                  // tracked floor held
        CHECK(t.wraps() == 1);
        CHECK(t.position() == pc.truth);
    }

    // 3: the floor falls away again as the belt is let in, tracking true
    //    position exactly the whole way down - including back across the wrap,
    //    where the up and down spans differ by one count.
    {
        PcntModel pc; WrapTracker t;
        pull(pc, t, 40000, 4);
        CHECK(t.update(pc.read()) == 32767);
        CHECK(pull(pc, t, -20000, 4) == 20000);      // true 20000, still linear
        const int16_t v = pull(pc, t, -17000, 4);    // true 3000, back over the wrap
        printf("3: let back in to true %ld -> tracked %d, wraps=%lu\n",
               (long)pc.truth, (int)v, (unsigned long)t.wraps());
        CHECK(v == 3000);
        CHECK(t.position() == pc.truth);
        CHECK(pull(pc, t, -3000, 4) == 0);           // and back to the arming zero
    }

    // 4: pushed past the l_lim wrap and pulled back out, the raw counter reads
    //    large and positive while the belt is still slack - a false floor.
    {
        PcntModel pc; WrapTracker t;
        pull(pc, t, -40000, 4);
        CHECK(t.update(pc.read()) == 0);
        const int16_t v = pull(pc, t, 20000, 4);     // true -20000, raw large and positive
        printf("4: true %ld of slack  raw %d  tracked %d  wraps=%lu\n",
               (long)pc.truth, (int)pc.read(), (int)v, (unsigned long)t.wraps());
        CHECK(pc.read() > FULL_SCALE_COUNTS);        // raw says full force
        CHECK(v == 0);                               // tracked says no floor
        CHECK(t.position() == pc.truth);
    }

    // 5: the delta test must not fire on real motion. The 1 us glitch filter
    //    caps a 167 us tick at 83 counts; 500 covers 6x that, including a tick
    //    stretched by a timed-out I2C write.
    {
        for (int32_t perTick = 1; perTick <= 500; perTick += 7) {
            PcntModel pc; WrapTracker t;
            CHECK(pull(pc, t, 30000, perTick) == 30000);
            CHECK(t.wraps() == 0);
        }
        printf("5: no false wrap up to 500 counts/tick (6x the filter ceiling)\n");
    }

    // 6: sustained travel in one direction stays exact across many wraps.
    //    Well inside WRAP_OFFSET_LIMIT; case 8 covers the clamp itself.
    {
        PcntModel pc; WrapTracker t;
        for (int i = 0; i < 20; i++) {
            pull(pc, t, 32767, 4096);
            CHECK(t.position() == pc.truth);
        }
        printf("6: exact after %lu wraps, position=%ld truth=%ld\n",
               (unsigned long)t.wraps(), (long)t.position(), (long)pc.truth);
        CHECK(t.wraps() == 20);
        CHECK(t.update(pc.read()) == 32767);
        CHECK(pull(pc, t, -(int32_t)pc.truth, 4096) == 0);   // all the way home
        CHECK(t.position() == 0);
    }

    // 7: reset clears prev, offset and wrap count. Every entry into
    //    INITIALIZED goes through it, so no stale m_prev crosses a gap in
    //    which ticks were not taken.
    {
        PcntModel pc; WrapTracker t;
        pull(pc, t, 40000, 4);
        CHECK(t.wraps() == 1);
        t.reset();
        CHECK(t.wraps() == 0);
        CHECK(t.position() == 0);
        CHECK(t.update(0) == 0);
        CHECK(t.update(500) == 500);
        printf("7: reset clears prev, offset and wrap count\n");
    }

    // 8: past WRAP_OFFSET_LIMIT the offset stops accumulating. Tracking is
    //    given up there, but the alternative is int32 overflow: a wrapped
    //    offset makes pos negative, which reads as slack and drops the floor
    //    to zero at full pull. The clamp trades a stale floor for a safe one.
    //    1001 up-wraps is ~33 million counts in one direction without
    //    re-arming, so this is a structural bound, not a working limit.
    {
        PcntModel pc; WrapTracker t;
        const int32_t wrapsToClamp = WRAP_OFFSET_LIMIT / WRAP_SPAN_UP + 1;
        for (int32_t i = 0; i < wrapsToClamp + 100; i++) {
            pull(pc, t, WRAP_SPAN_UP, 4096);
            CHECK(t.update(pc.read()) == 32767);       // floor stays at full scale
            CHECK(t.position() > 0);                   // never wrapped negative
            CHECK(t.position() <= WRAP_OFFSET_LIMIT + 2 * WRAP_SPAN_UP);
        }
        printf("8: clamped at %ld after %lu wraps, truth=%ld\n",
               (long)t.position(), (unsigned long)t.wraps(), (long)pc.truth);
        CHECK(t.wraps() == (uint32_t)(wrapsToClamp + 100));   // still counted
        CHECK(t.position() < pc.truth);                       // tracking given up
    }

    printf(g_fail ? "\n%d CHECK(s) FAILED\n" : "\nall checks passed\n", g_fail);
    return g_fail ? 1 : 0;
}
