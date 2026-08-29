#ifndef WRAPTRACKER_H
#define WRAPTRACKER_H

#include <stdint.h>

// Extends one PCNT unit's 16 bit counter across its wraps, in software, from
// the sample stream the DAC tick already takes.
//
// PCNT *resets to 0* at counter_h_lim / counter_l_lim: it neither saturates nor
// rolls over modulo 2^16 (Counter.h). mapTo15bit() saturates at 6667 counts, so
// past that the torque command is flat - the belt is a constant-force pull with
// no mechanical stop, and 32767 counts is reachable. Untracked, the floor would
// drop from full scale to zero there.
//
// The delta test replaces the PCNT h_lim/l_lim event ISR because the caller
// samples at the DAC tick: the 1 us glitch filter caps the input at ~83 counts
// per tick, and even a ~1 ms I2C stall only multiplies that by six. A jump
// beyond half the counter range is a wrap, never motion.
#define WRAP_DELTA 16384

// The counter restarts from 0 rather than from the opposite limit, and the two
// limits are not symmetric: an up-wrap advances true position by counter_h_lim
// (32767), a down-wrap retreats it by |counter_l_lim| (32768).
#define WRAP_SPAN_UP 32767
#define WRAP_SPAN_DOWN 32768

// Bounds the accumulated offset against int32_t overflow however long a session
// runs. 1000 wraps is ~33 million counts of travel in one direction without
// re-arming - a structural guarantee, not a working limit.
#define WRAP_OFFSET_LIMIT (1000 * 32768)

class WrapTracker {
public:
  // One raw PCNT reading in, one usable displacement out, clamped to the range
  // the caller can represent. Must be fed every DAC tick for the delta test to
  // hold; see reset().
  //
  // Out of range is clamped rather than reported: above the top the floor holds
  // at full scale, below the arming zero there is no floor.
  int16_t update(int16_t raw) {
    const int32_t delta = (int32_t)raw - (int32_t)m_prev;
    if (delta < -WRAP_DELTA) {                                  // through h_lim
      if (m_offset <= WRAP_OFFSET_LIMIT) m_offset += WRAP_SPAN_UP;
      if (m_wraps != 0xFFFFFFFFu) m_wraps++;
    } else if (delta > WRAP_DELTA) {                            // through l_lim
      if (m_offset >= -WRAP_OFFSET_LIMIT) m_offset -= WRAP_SPAN_DOWN;
      if (m_wraps != 0xFFFFFFFFu) m_wraps++;
    }
    m_prev = raw;

    const int32_t pos = m_offset + (int32_t)raw;
    if (pos < 0) return 0;
    if (pos > 32767) return 32767;
    return (int16_t)pos;
  }

  // Cleared together with the hardware counter, so a tracked session starts
  // from a known zero. The counter is only read while INITIALIZED and every
  // entry into INITIALIZED passes through here, so no stale m_prev is carried
  // across the ticks not taken in the other states.
  void reset() {
    m_prev = 0;
    m_offset = 0;
    m_wraps = 0;
  }

  // Wraps since the reset, either direction. Diagnostics and tests only.
  uint32_t wraps() const { return m_wraps; }

  // Unclamped displacement from the arming zero. Tests only - the control path
  // uses the clamped value update() returns.
  int32_t position() const { return m_offset + (int32_t)m_prev; }

private:
  int16_t m_prev = 0;
  int32_t m_offset = 0;
  uint32_t m_wraps = 0;
};

#endif
