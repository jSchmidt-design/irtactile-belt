#ifndef PLAYOUT_H
#define PLAYOUT_H

#include <stdint.h>
#include "SampleRing.h"

// Fixed-point notation, used in both forms below. Qn is n fractional bits and
// no integer part: value = raw / 2^n. The gains and the smoother state are Q16,
// so GAIN_ONE (65536) is 1.0. Qi.f spells out both halves: m_phase is Q16.16, a
// sample index in the high 16 bits and the position within that sample in the
// low 16, so PHASE_ONE (0x10000) is one whole sample.

// The DAC tick is fixed; the stream rate is an internal resampling ratio, so a
// rate change needs no timer restart and no I2C duty change.
//
// 6 kHz is a point on the stream ladder itself (48000 >> rateCode, see
// SerialDecoder.h) and its top rung. Every reachable rate divides it by a power
// of two, so each sample is held for a whole number of ticks - 6000 -> 1,
// 3000 -> 2, 375 -> 16, 46.875 -> 128 - and the resampler adds no pattern
// jitter. Keep any future tick rate on the ladder: a tick off it divides no
// rate evenly, so every rate alternates between two hold lengths.
//
// The 167 us tick also holds the 5 byte DAC write to ~34 % I2C duty, so it
// still fits one tick if the bus has to fall back from 1 MHz to 400 kHz.
//
// 80 MHz APB has no factor of 3, so no ladder rate is exactly generatable:
// timerBegin(6000) prescales to 13333 and lands on 6000.15 Hz, +25 ppm. The
// PLL integral term absorbs it, and the residual surfaces as a single-tick slip
// every few seconds rather than a continuous modulation.
#define DAC_TICK_HZ 6000
#define PHASE_ONE 0x10000        // Q16.16 "one sample"

// Drift lock. The correction is a rate (ppm) and the error a position (samples
// of fill), and the ring integrates one into the other, so the loop is a double
// integrator: the P term damps it, the I term carries only the static crystal
// offset. Both gains are normalised by the stream rate, since the same ppm buys
// 16x more samples per second at 6000 Hz than at 375 Hz.
//   kp = 1e9 / rateMilliHz is the deadbeat gain: one sample of correction per
//   one-second window. Half of that is well damped.
#define PLL_P_SHIFT 1            // kp/2
#define PLL_I_SHIFT 3            // ki = kp/8
#define PLL_MAX_PPM 1000         // +-0.1 %, a 0.017 semitone shift
#define UNDERRUN_SILENCE_TICKS (DAC_TICK_HZ / 4)   // 250 ms before ramping down
#define GAIN_ONE 65536
// Round up so the ramp always completes inside the 50 ms.
#define GAIN_STEP ((GAIN_ONE + (DAC_TICK_HZ / 20) - 1) / (DAC_TICK_HZ / 20))

// Optional one-pole smoother at the tick rate (~100 Hz corner). Off by default:
// it only helps when content sits well below Nyquist and costs ~1.6 ms of group
// delay.
#define ENABLE_SMOOTHER 0
#define SMOOTHER_ALPHA_Q16 4988

// Playout margin: how empty the ring may get in the steady state. The
// fraction-of-block term (blockSamples/8) is 0.33 ms at 6000 Hz, thinner than
// the ~1 ms of USB frame scheduling irreducible on this link, so arrival jitter
// alone underruns it. The larger of the two terms wins; costs at most 1 ms of
// added latency. 0 gives fraction-only behaviour.
#define PLAYOUT_MARGIN_US 1000

// Largest block length the ring can be sized around. blockSamples arrives in
// two 7-bit header fields, so the wire permits up to 16383. Decoding is
// unaffected, but every sizing term here is a fraction or a multiple of the
// block against a fixed 512-entry ring:
//
//   m_resyncLimit = targetMin + 4*blk + 8   exceeds the ring above blk ~121
//   keep          = targetMin + blk         exceeds any reachable fill above
//                                           blk ~454, underflowing the resync
//                                           subtraction and flushing the ring
//
// So the sizing terms use a bounded copy while the reported blockSamples stays
// the value the host sent. RING_SIZE/8 is the largest value that keeps
// m_resyncLimit inside the ring.
#define PLAYOUT_MAX_BLOCK (RING_SIZE / 8)

typedef struct {
  uint32_t minFill;
  uint32_t maxFill;
  uint32_t underrunTicks;   // ticks spent starved during the window
  uint32_t resynced;        // stale samples discarded during the window
  uint32_t dropped;         // cumulative producer-side drops (ring full)
  int32_t corrPpm;
  uint32_t rateMilliHz;
  uint16_t blockSamples;
  uint16_t targetMin;
} playout_stats_t;

// Zero-order-hold resampler from the stream rate onto the fixed DAC tick, with
// the consumption rate locked to the production rate by the PI loop above.
// Every method is called from the DAC task only.
class Playout {
public:

  // Pre-sync default: a real ladder rate.
  Playout() { configure(6000u * 1000u, 16); }

  void configure(uint32_t rateMilliHz, uint16_t blockSamples) {
    if (blockSamples == 0) blockSamples = 1;
    m_rateMilliHz = rateMilliHz;
    m_blockSamples = blockSamples;

    // Everything below sizes buffers against the ring, so it uses the bounded
    // copy; m_blockSamples keeps the true value for the stats line. See
    // PLAYOUT_MAX_BLOCK.
    m_blockSizing = (blockSamples > PLAYOUT_MAX_BLOCK) ? (uint16_t)PLAYOUT_MAX_BLOCK
                                                       : blockSamples;

    // 375 Hz -> 4096, 6000 Hz -> 65536, 12000 Hz -> 131072; all exact.
    m_inc = (uint32_t)(((uint64_t)rateMilliHz << 16) / ((uint64_t)DAC_TICK_HZ * 1000u));
    if (m_inc == 0) m_inc = 1;
    m_corrPpm = 0;
    m_corrTrim = 0;
    m_incCorrected = m_inc;

    // ppm needed to move the fill by one sample in one PLL window.
    m_kp = (int32_t)(1000000000ull / (uint64_t)rateMilliHz);
    if (m_kp < 1) m_kp = 1;

    // Round up: truncating gives 1 sample (0.67 ms) at 1500 Hz where the margin
    // asks for 1 ms. Below 1500 Hz one sample already exceeds
    // PLAYOUT_MARGIN_US, so the floor of 1 covers those rates.
    const uint32_t jitterMargin =
      (uint32_t)(((uint64_t)rateMilliHz * PLAYOUT_MARGIN_US + 999999999ull) / 1000000000ull);
    m_targetMin = m_blockSizing / 8;
    if (m_targetMin < jitterMargin) m_targetMin = (uint16_t)jitterMargin;
    if (m_targetMin < 1) m_targetMin = 1;
    // The PLL drives minFill to this, so a target the ring cannot hold is one
    // the loop can never reach: the error stays negative, the correction pins
    // at -PLL_MAX_PPM, and the ring grows until it drops. Unreachable now that
    // m_blockSizing is bounded, but it costs one compare.
    if (m_targetMin > RING_MASK / 2) m_targetMin = (uint16_t)(RING_MASK / 2);

    // Ceiling on accumulated playout latency; the PLL keeps us far below it.
    m_resyncLimit = (uint32_t)m_targetMin + 4u * m_blockSizing + 8u;
    if (m_resyncLimit > RING_MASK - 1) m_resyncLimit = RING_MASK - 1;

    m_phase = 0;
    startWindow();
  }

  // Advance one DAC tick and return the value to write to the DAC.
  dac_sample_t tick(SampleRing &ring) {
    uint32_t fill = ring.fill();

    // maxFill reports what the ring actually reached, so take it before any
    // resync - otherwise the backlog that triggered the discard never shows up
    // in the logged fill[min..max] range.
    if (fill > m_maxFill) m_maxFill = fill;

    // Drop, never queue: if we somehow fell far behind (host backlog, missed
    // ticks), discard the stale samples instead of playing them out late.
    if (fill > m_resyncLimit) {
      // What the discard must leave behind. Guarded because the subtraction is
      // unsigned: if `keep` exceeded `fill` the result would wrap to ~4e9,
      // discard() would clamp it to the whole ring, and a backlog spike would
      // empty the buffer outright. m_blockSizing being bounded keeps
      // keep < m_resyncLimit < fill, so this cannot trigger.
      const uint32_t keep = (uint32_t)m_targetMin + m_blockSizing;
      const uint32_t want = (fill > keep) ? fill - keep : 0;
      const uint32_t got = ring.discard(want);   // may be short of want
      fill -= got;
      m_windowResync += got;
    }

    if (fill < m_minFill) m_minFill = fill;

    m_phase += m_incCorrected;
    bool starved = false;
    bool advanced = false;
    dac_sample_t next;
    while (m_phase >= PHASE_ONE) {
      if (!ring.pop(next)) {
        // Underrun: hold m_cur and retry on the next tick.
        starved = true;
        m_phase = PHASE_ONE - 1;
        break;
      }
      m_cur = next;
      m_phase -= PHASE_ONE;
      advanced = true;
    }

    if (starved) {
      m_underrunTicks++;
      m_windowUnderrun++;
    } else if (advanced) {
      m_underrunTicks = 0;
    }

    // After a sustained underrun, ramp to zero rather than parking a DC offset
    // on the actuator. An externally set mute (see setMuted) takes the same
    // path, so there is one way down to silence and not two.
    const uint32_t targetGain =
      (m_muted || m_underrunTicks >= UNDERRUN_SILENCE_TICKS) ? 0 : GAIN_ONE;
    if (m_gain < targetGain) {
      m_gain = (m_gain + GAIN_STEP > GAIN_ONE) ? GAIN_ONE : m_gain + GAIN_STEP;
    } else if (m_gain > targetGain) {
      m_gain = (m_gain > GAIN_STEP) ? m_gain - GAIN_STEP : 0;
    }

    // The encoder floor's own gain: as m_gain above, except that a floor hold
    // (see setFloorHold) keeps it up through an underrun. The fault mute still
    // wins over the hold. m_gain itself is never held - m_cur holds its last
    // sample when starved, so holding it would park a DC offset.
    //
    // With no hold active floorTarget == targetGain every tick, so
    // floorGainQ16() == gainQ16().
    const uint32_t floorTarget =
      m_muted ? 0
              : ((m_floorHold || m_underrunTicks < UNDERRUN_SILENCE_TICKS) ? GAIN_ONE : 0);
    if (m_floorGain < floorTarget) {
      m_floorGain = (m_floorGain + GAIN_STEP > GAIN_ONE) ? GAIN_ONE : m_floorGain + GAIN_STEP;
    } else if (m_floorGain > floorTarget) {
      m_floorGain = (m_floorGain > GAIN_STEP) ? m_floorGain - GAIN_STEP : 0;
    }

    uint32_t out0 = ((uint32_t)(m_cur.ch0 & 0x7FFF) * m_gain) >> 16;
    uint32_t out1 = ((uint32_t)(m_cur.ch1 & 0x7FFF) * m_gain) >> 16;

#if ENABLE_SMOOTHER
    m_sm0 += (int32_t)(((int64_t)SMOOTHER_ALPHA_Q16 * (((int32_t)out0 << 16) - m_sm0)) >> 16);
    m_sm1 += (int32_t)(((int64_t)SMOOTHER_ALPHA_Q16 * (((int32_t)out1 << 16) - m_sm1)) >> 16);
    out0 = (uint32_t)(m_sm0 >> 16);
    out1 = (uint32_t)(m_sm1 >> 16);
#endif

    // Drift lock, once per second. The minimum fill over the window is the
    // quantity that must not reach zero; the instantaneous fill swings by a
    // full block every block period and says nothing.
    if (++m_windowTicks >= DAC_TICK_HZ) {
      const int32_t err = (int32_t)m_minFill - (int32_t)m_targetMin;
      m_corrTrim += (err * m_kp) >> PLL_I_SHIFT;
      if (m_corrTrim > PLL_MAX_PPM) m_corrTrim = PLL_MAX_PPM;      // anti-windup
      if (m_corrTrim < -PLL_MAX_PPM) m_corrTrim = -PLL_MAX_PPM;
      m_corrPpm = ((err * m_kp) >> PLL_P_SHIFT) + m_corrTrim;
      if (m_corrPpm > PLL_MAX_PPM) m_corrPpm = PLL_MAX_PPM;
      if (m_corrPpm < -PLL_MAX_PPM) m_corrPpm = -PLL_MAX_PPM;
      m_incCorrected = (uint32_t)((int32_t)m_inc + (int32_t)(((int64_t)m_inc * m_corrPpm) / 1000000));
      if (m_incCorrected == 0) m_incCorrected = 1;

      m_stats.minFill = m_minFill;
      m_stats.maxFill = m_maxFill;
      m_stats.underrunTicks = m_windowUnderrun;
      m_stats.resynced = m_windowResync;
      m_stats.dropped = ring.dropped();
      m_stats.corrPpm = m_corrPpm;
      m_stats.rateMilliHz = m_rateMilliHz;
      m_stats.blockSamples = m_blockSamples;
      m_stats.targetMin = m_targetMin;
      m_statsReady = true;

      startWindow();
    }

    dac_sample_t out;
    out.ch0 = (uint16_t)out0;
    out.ch1 = (uint16_t)out1;
    return out;
  }

  // True once per second; clears the flag.
  bool takeStats(playout_stats_t &out) {
    if (!m_statsReady) return false;
    m_statsReady = false;
    out = m_stats;
    return true;
  }

  // Force the silence ramp from outside the playout path. The DAC write path
  // uses it when a run of I2C writes has failed: the GP8413 holds its last
  // value on a NAK or a timeout, so the commanded value is ramped down and the
  // first write that does get through carries a low value instead of the frozen
  // one. Not latched - clearing it ramps back up over the same 50 ms.
  void setMuted(bool muted) { m_muted = muted; }
  bool muted() const { return m_muted; }

  // Hold the encoder floor's gain up without a sample stream. dacTask asserts
  // this while belt-tune's tuning headers keep arriving and clears it when they
  // stop; the floor then ramps down over the same 50 ms as a dead host. The
  // stream gain is unaffected.
  void setFloorHold(bool hold) { m_floorHold = hold; }

  // Current ramp gain, Q16. Exposed so a caller that adds its own contribution
  // to the output *after* tick() - the encoder-derived preload floor in
  // processData() - can scale it by the same gain. Applying the gain to each
  // term is equivalent to applying it to the combined value, since
  // max(g*a, g*b) == g*max(a, b), so the stream is not gained twice.
  uint32_t gainQ16() const { return m_gain; }

  // The gain the encoder-derived floor is scaled by, used by processData().
  // Equal to gainQ16() every tick unless a floor hold is active, in which case
  // it stays up while the stream gain ramps down.
  uint32_t floorGainQ16() const { return m_floorGain; }

  uint16_t targetMin() const { return m_targetMin; }
  int32_t corrPpm() const { return m_corrPpm; }

  // The block length the buffer maths is sized against: the received one
  // bounded to PLAYOUT_MAX_BLOCK. Tests and diagnostics; the stats line reports
  // the received value instead.
  uint16_t blockSizing() const { return m_blockSizing; }
  uint32_t resyncLimit() const { return m_resyncLimit; }

  // Nominal Q16.16 samples per tick, before the PLL correction. Tests only.
  uint32_t phaseInc() const { return m_inc; }

private:

  void startWindow() {
    m_windowTicks = 0;
    m_minFill = RING_SIZE;
    m_maxFill = 0;
    m_windowUnderrun = 0;
    m_windowResync = 0;
  }

  dac_sample_t m_cur;          // the held sample

  uint32_t m_rateMilliHz = 0;
  uint16_t m_blockSamples = 1;   // as received, for the stats line
  uint16_t m_blockSizing = 1;    // bounded to PLAYOUT_MAX_BLOCK, for the maths

  uint32_t m_phase = 0;          // Q16.16
  uint32_t m_inc = PHASE_ONE;    // nominal samples per tick
  uint32_t m_incCorrected = PHASE_ONE;
  int32_t m_corrPpm = 0;
  int32_t m_corrTrim = 0;        // integral term: the static crystal offset
  int32_t m_kp = 1;

  uint16_t m_targetMin = 1;
  uint32_t m_resyncLimit = RING_MASK;

  uint32_t m_windowTicks = 0;
  uint32_t m_minFill = RING_SIZE;
  uint32_t m_maxFill = 0;
  uint32_t m_windowUnderrun = 0;
  uint32_t m_windowResync = 0;

  uint32_t m_underrunTicks = 0;
  uint32_t m_gain = GAIN_ONE;    // Q16, drives the silence ramp
  uint32_t m_floorGain = GAIN_ONE;  // Q16, the floor's ramp - held up during tuning
  bool m_muted = false;          // external fault mute, same ramp
  bool m_floorHold = false;      // tuning hold: keep m_floorGain up without a stream

#if ENABLE_SMOOTHER
  int32_t m_sm0 = 0, m_sm1 = 0;  // Q16 one-pole state
#endif

  playout_stats_t m_stats = {};
  bool m_statsReady = false;
};

#endif
