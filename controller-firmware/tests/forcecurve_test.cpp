// Native test for ForceCurve.h: the runtime encoder force curve (full-scale
// count + gamma), its clamp, LUT build, and every-tick lookup.
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <initializer_list>

#include "ForceCurve.h"

static int g_fail = 0;
#define CHECK(cond) do { if (!(cond)) { printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } } while (0)

int main() {
    // 1: forceCurveClamp is the single enforcement point. The wire and NVS
    //    paths both run values through it before anything else sees them.
    {
        uint16_t fs; uint8_t gq;

        fs = 0;     gq = 0;   forceCurveClamp(fs, gq);
        CHECK(fs == MIN_FULL_SCALE_COUNTS);   // 0 divides by zero in the lookup
        CHECK(gq == MIN_GAMMA_Q);             // gamma 0 -> flat LUT at full force

        fs = 50000; gq = 240; forceCurveClamp(fs, gq);
        CHECK(fs == MAX_FULL_SCALE_COUNTS);
        CHECK(gq == MAX_GAMMA_Q);

        fs = 4000;  gq = 16;  forceCurveClamp(fs, gq);
        CHECK(fs == 4000 && gq == 16);        // in range: untouched

        fs = MIN_FULL_SCALE_COUNTS; gq = MIN_GAMMA_Q; forceCurveClamp(fs, gq);
        CHECK(fs == MIN_FULL_SCALE_COUNTS && gq == MIN_GAMMA_Q);   // bounds inclusive
        fs = MAX_FULL_SCALE_COUNTS; gq = MAX_GAMMA_Q; forceCurveClamp(fs, gq);
        CHECK(fs == MAX_FULL_SCALE_COUNTS && gq == MAX_GAMMA_Q);
        printf("1: clamp pins gammaQ=0 and fullScale=0 to the safe minimums\n");
    }

    // 2: every built LUT is monotonic non-decreasing and spans exactly 0..32767.
    {
        for (uint8_t gq = MIN_GAMMA_Q; gq <= MAX_GAMMA_Q; gq++) {
            ForceCurve c;
            buildForceCurve(6667, gq, c);
            CHECK(c.lut[0] == 0);
            CHECK(c.lut[FORCE_CURVE_LUT_POINTS - 1] == 32767);
            CHECK(c.fullScaleCounts == 6667);
            for (int i = 1; i < FORCE_CURVE_LUT_POINTS; i++) {
                CHECK(c.lut[i] >= c.lut[i - 1]);
            }
        }
        printf("2: LUT monotonic and 0..32767 for every gammaQ in range\n");
    }

    // 3: gamma 1.0 (the untuned default) is a straight line - the same linear
    //    ramp to saturation the firmware baked in before this feature. LUT
    //    points and the interpolated lookup both track counter*32767/fullScale.
    {
        ForceCurve c;
        buildForceCurve(DEFAULT_FULL_SCALE_COUNTS, DEFAULT_GAMMA_Q, c);

        for (int i = 0; i < FORCE_CURVE_LUT_POINTS; i++) {
            const int32_t ideal = (int32_t)llround((double)i * 32767.0 / 64.0);
            CHECK(std::abs((int)c.lut[i] - ideal) <= 1);
        }

        int worst = 0;
        for (uint16_t counter = 0; counter < DEFAULT_FULL_SCALE_COUNTS; counter++) {
            const int got = forceCurveLookup(c, counter);
            const int ideal = (int)((uint32_t)counter * 32767u / DEFAULT_FULL_SCALE_COUNTS);
            const int d = std::abs(got - ideal);
            if (d > worst) worst = d;
            CHECK(d <= 2);
        }
        CHECK(forceCurveLookup(c, 0) == 0);
        printf("3: gamma 1.0 linear, lookup within %d LSB of the straight line\n", worst);
    }

    // 4: gamma shapes the interior. gamma < 1 builds fast then flattens (sits
    //    above the diagonal); gamma > 1 stays low then walls up (below it).
    {
        ForceCurve lo, mid, hi;
        buildForceCurve(6667, 4, lo);     // gamma 0.25
        buildForceCurve(6667, 16, mid);   // gamma 1.0
        buildForceCurve(6667, 64, hi);    // gamma 4.0

        bool strictLo = false, strictHi = false;
        for (int i = 1; i < FORCE_CURVE_LUT_POINTS - 1; i++) {
            CHECK(lo.lut[i] >= mid.lut[i]);
            CHECK(mid.lut[i] >= hi.lut[i]);
            if (lo.lut[i] > mid.lut[i]) strictLo = true;
            if (mid.lut[i] > hi.lut[i]) strictHi = true;
        }
        CHECK(strictLo && strictHi);
        printf("4: gamma<1 above the diagonal, gamma>1 below\n");
    }

    // 5: the lookup clamps counter to fullScaleCounts and holds the ceiling
    //    there - this is what subsumes the old explicit maxCounter clamp.
    {
        ForceCurve c;
        buildForceCurve(2000, 16, c);
        CHECK(forceCurveLookup(c, 2000) == 32767);
        CHECK(forceCurveLookup(c, 2001) == 32767);
        CHECK(forceCurveLookup(c, 30000) == 32767);
        CHECK(forceCurveLookup(c, 65535) == 32767);

        uint16_t prev = 0;
        for (uint32_t counter = 0; counter <= 4000; counter += 5) {
            const uint16_t v = forceCurveLookup(c, (uint16_t)counter);
            CHECK(v >= prev);         // monotonic non-decreasing in counter
            prev = v;
        }
        printf("5: lookup clamps counter to fullScale and is monotonic\n");
    }

    // 6: the tightest and widest permitted belts both build and look up
    //    cleanly - no divide-by-zero, no overflow at the extremes.
    {
        for (uint16_t fs : { (uint16_t)MIN_FULL_SCALE_COUNTS,
                             (uint16_t)MAX_FULL_SCALE_COUNTS }) {
            ForceCurve c;
            buildForceCurve(fs, 16, c);
            CHECK(forceCurveLookup(c, 0) == 0);
            CHECK(forceCurveLookup(c, fs) == 32767);
            CHECK(forceCurveLookup(c, fs / 2) > 0);
            CHECK(forceCurveLookup(c, fs / 2) < 32767);
        }
        printf("6: extremes of fullScaleCounts build and look up cleanly\n");
    }

    printf(g_fail ? "\n%d CHECK(s) FAILED\n" : "\nall checks passed\n", g_fail);
    return g_fail ? 1 : 0;
}
