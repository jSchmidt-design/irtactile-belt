#ifndef COUNTER_H
#define COUNTER_H

#include "driver/pcnt.h"
#include "WrapTracker.h"

// Channel numbering. Indices in code are 0-based throughout and line up with
// dac_sample_t: index 0 is the belt the wiring doc and the log line call
// channel 1, index 1 is channel 2. The 1-based names left are the ones naming
// hardware - the pin and PCNT unit macros below - and the Count1/Count2 log
// labels.


#define CH_1_ENCODER_A_PIN 4   // Pulse input
#define CH_1_ENCODER_B_PIN 5   // Direction input


#define CH_2_ENCODER_A_PIN 26   // Pulse input
#define CH_2_ENCODER_B_PIN 27   // Direction input


#define PCNT_UNIT_CH1 PCNT_UNIT_0

#define PCNT_UNIT_CH2 PCNT_UNIT_1

// Glitch filter: pulses shorter than this many APB (80 MHz) cycles are ignored
// on both the pulse and the control input. 80 cycles = 1 us, which still passes
// a 500 kHz pulse rate - orders above what the mechanics produce. The register
// saturates at 1023 cycles (12.8 us); stay under half the shortest real pulse
// width.
#define PCNT_FILTER_APB_CYCLES 80

struct CounterStatus{
  int16_t ch0=0;
  int16_t ch1=0;
};

class EncoderCounter{


  public:

  static EncoderCounter* GetInstance(){
    static EncoderCounter counter;
    return &counter;
  }

  // Raw hardware counts, signed. The 16 bit PCNT counter *resets to 0* at
  // counter_h_lim / counter_l_lim and cannot be made to saturate, so a reading
  // on its own says nothing about which side of a wrap it is on.
  //
  // Diagnostics only; the control path uses getCountsTracked().
  CounterStatus getCounts(){
    CounterStatus counts;

    pcnt_get_counter_value(PCNT_UNIT_CH1, &counts.ch0);
    pcnt_get_counter_value(PCNT_UNIT_CH2, &counts.ch1);
    return counts;
  }

  // Counts with the wraps folded back in, clamped to 0..32767 - what the torque
  // floor is derived from. See WrapTracker.h.
  //
  // Call from exactly one place, at the DAC tick: the delta test needs a regular
  // sample interval, and a second caller on the other core would race on the
  // tracker state. Use getCounts() for anything else.
  CounterStatus getCountsTracked(){
    const CounterStatus raw = getCounts();

    CounterStatus counts;
    counts.ch0 = m_track0.update(raw.ch0);
    counts.ch1 = m_track1.update(raw.ch1);
    return counts;
  }

  // Wraps per channel since the last reset, for the log. 0 through a normal
  // session; non-zero means a belt was pulled past the wrap.
  uint32_t wrapsCh0() const { return m_track0.wraps(); }
  uint32_t wrapsCh1() const { return m_track1.wraps(); }

  // Called from loop() while dacTask runs on the other core. Safe only because
  // of the ordering at the call site: the reset happens while the state is still
  // INITIALIZING and only the INITIALIZED branch reads the counter, so no tick
  // observes a half-cleared tracker. Keep it that way.
  void reset(){
    m_track0.reset();
    m_track1.reset();

    pcnt_counter_pause(PCNT_UNIT_CH1);
    pcnt_counter_clear(PCNT_UNIT_CH1);
    pcnt_counter_resume(PCNT_UNIT_CH1);

    pcnt_counter_pause(PCNT_UNIT_CH2);
    pcnt_counter_clear(PCNT_UNIT_CH2);
    pcnt_counter_resume(PCNT_UNIT_CH2);
  }


  private:
  WrapTracker m_track0;
  WrapTracker m_track1;

  EncoderCounter(){
    
  // B is a direction input: A's rising edge counts, B's *level* at that edge
  // picks the sign.
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = CH_1_ENCODER_A_PIN,   // Pulse input pin
    .ctrl_gpio_num = CH_1_ENCODER_B_PIN,    // Direction input
    .lctrl_mode = PCNT_MODE_REVERSE,        // B low  -> count down
    .hctrl_mode = PCNT_MODE_KEEP,           // B high -> count up
    .pos_mode = PCNT_COUNT_INC,     // Count on rising edges
    .neg_mode = PCNT_COUNT_DIS,     // Ignore falling edges
    .counter_h_lim = 32767,         // Wraps to 0 here, does not saturate
    .counter_l_lim = -32768,        // and here, when B sends it the other way
    .unit = PCNT_UNIT_CH1,
    .channel = PCNT_CHANNEL_0,
  };

  pcnt_unit_config(&pcnt_config);

  // CH2 is mounted opposite-handed, so its control modes are the mirror of
  // CH1's. Do not "tidy" them to match.
  pcnt_config_t pcnt_config_ch2 = {
    .pulse_gpio_num = CH_2_ENCODER_A_PIN,   // Pulse input pin
    .ctrl_gpio_num = CH_2_ENCODER_B_PIN,    // Direction input
    .lctrl_mode = PCNT_MODE_KEEP,           // B low  -> count up
    .hctrl_mode = PCNT_MODE_REVERSE,        // B high -> count down
    .pos_mode = PCNT_COUNT_INC,     // Count on rising edges
    .neg_mode = PCNT_COUNT_DIS,     // Ignore falling edges
    .counter_h_lim = 32767,         // Wraps to 0 here, does not saturate
    .counter_l_lim = -32768,        // and here, when B sends it the other way
    .unit = PCNT_UNIT_CH2,
    .channel = PCNT_CHANNEL_0,
  };

  pcnt_unit_config(&pcnt_config_ch2);

  // Must be set before the filter is enabled; applies to pulse and control
  // inputs alike.
  pcnt_set_filter_value(PCNT_UNIT_CH1, PCNT_FILTER_APB_CYCLES);
  pcnt_filter_enable(PCNT_UNIT_CH1);

  pcnt_set_filter_value(PCNT_UNIT_CH2, PCNT_FILTER_APB_CYCLES);
  pcnt_filter_enable(PCNT_UNIT_CH2);

  reset();
  }
};

#endif
