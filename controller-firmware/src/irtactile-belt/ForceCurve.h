#ifndef FORCE_CURVE_H
#define FORCE_CURVE_H

#include <stdint.h>
#include <math.h>

// The encoder-derived force floor, as a runtime-shaped curve rather than a
// fixed linear ramp.
//
// Two parameters describe it:
//   fullScaleCounts - encoder counts of pull at which the floor saturates
//   gammaQ          - the ramp shape, gamma * 16 as Q3.4 (fixed point, 3
//                     integer bits and 4 fractional, so steps of 1/16)
//
// A 65-point lookup table is built from those on core 1 (powf is too heavy for
// the 6 kHz tick, see irtactile-belt.ino) and the every-tick path only does an
// integer interpolation into it.
//
// Plain header, <math.h> only - same convention as WrapTracker.h, so test.bat
// compiles it natively.

// Indices 0..64 span pull fraction 0..1.
#define FORCE_CURVE_LUT_POINTS 65

// Compile-time defaults: gamma 1.0 at 6667 counts reproduces the old fixed
// linear ramp (3 * counter * 32767 / 20000, saturating at 6667), so an untuned
// device behaves as it did before. These never travel the wire.
//
// The match is within ~4 counts of 32767, not bit-for-bit: the old slope
// divided by 20000/3 = 6666.67 where this one divides by 6667, and the LUT
// interpolation rounds. forcecurve_test.cpp case 3 pins the bound.
//
// 6667 is odd, so it cannot round-trip the 2-count wire field exactly. That
// applies only to a value the host sends (protocol.md), not to this constant or
// the persisted one.
#define DEFAULT_FULL_SCALE_COUNTS 6667
#define DEFAULT_GAMMA_Q 16

// Bounds, enforced by forceCurveClamp() on both the wire path and the NVS load
// path. The wire encoding permits values that are dangerous on a harness pulled
// against a torso:
//   gammaQ = 0            -> pow(x, 0) == 1 everywhere: a flat LUT at full
//                           scale, i.e. full force at the slightest movement
//   fullScaleCounts small -> the same hazard by another route (and a divide by
//                           zero at 0 in the interpolation)
// Mirrored in host-bridge/src/TuningFrame.h.
#define MIN_GAMMA_Q 4              // gamma 0.25
#define MAX_GAMMA_Q 127            // gamma 7.9375, the Q3.4 wire maximum
#define MIN_FULL_SCALE_COUNTS 100  // a deliberately tight belt
#define MAX_FULL_SCALE_COUNTS 20000  // the ceiling the firmware has always carried

// The LUT and the count it was built against, travelling together so a swap
// publishes both at once and no tick sees a new full-scale against an old LUT.
struct ForceCurve {
  uint16_t lut[FORCE_CURVE_LUT_POINTS];
  uint16_t fullScaleCounts;
};

// The single enforcement point for the bounds above. Called from
// tuningHandler() (wire) and tuningLoad() (NVS) before the values reach
// anything else.
inline void forceCurveClamp(uint16_t &fullScaleCounts, uint8_t &gammaQ) {
  if (fullScaleCounts < MIN_FULL_SCALE_COUNTS) fullScaleCounts = MIN_FULL_SCALE_COUNTS;
  if (fullScaleCounts > MAX_FULL_SCALE_COUNTS) fullScaleCounts = MAX_FULL_SCALE_COUNTS;
  if (gammaQ < MIN_GAMMA_Q) gammaQ = MIN_GAMMA_Q;
  if (gammaQ > MAX_GAMMA_Q) gammaQ = MAX_GAMMA_Q;
}

// Fill the LUT with round(pow(i/64, gamma) * 32767) and store the count. The
// caller passes already-clamped values; this only does the maths.
inline void buildForceCurve(uint16_t fullScaleCounts, uint8_t gammaQ, ForceCurve &out) {
  const float gamma = (float)gammaQ / 16.0f;
  for (int i = 0; i < FORCE_CURVE_LUT_POINTS; i++) {
    const float x = (float)i / (float)(FORCE_CURVE_LUT_POINTS - 1);
    long v = lroundf(powf(x, gamma) * 32767.0f);
    if (v < 0) v = 0;
    if (v > 32767) v = 32767;
    out.lut[i] = (uint16_t)v;
  }
  out.fullScaleCounts = fullScaleCounts;
}

// counter -> floor level, integer only. Clamps counter to fullScaleCounts, then
// linearly interpolates between the two adjacent LUT entries. This is the only
// part that runs every tick.
inline uint16_t forceCurveLookup(const ForceCurve &c, uint16_t counter) {
  const uint16_t fs = c.fullScaleCounts;   // >= MIN_FULL_SCALE_COUNTS when built
  if (counter >= fs) return c.lut[FORCE_CURVE_LUT_POINTS - 1];

  // Position along the LUT, in LUT-segment units. counter < fs <= 20000 and
  // (FORCE_CURVE_LUT_POINTS - 1) == 64, so scaled < 1.28e6 - well inside u32.
  const uint32_t scaled = (uint32_t)counter * (FORCE_CURVE_LUT_POINTS - 1);
  const uint32_t idx = scaled / fs;        // 0..63, so idx + 1 is in range
  const uint32_t rem = scaled % fs;

  const uint16_t a = c.lut[idx];
  const uint16_t b = c.lut[idx + 1];       // b >= a: the LUT is non-decreasing
  return (uint16_t)(a + (uint32_t)(b - a) * rem / fs);
}

#endif
