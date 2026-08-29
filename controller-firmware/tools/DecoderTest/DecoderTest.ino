#include "driver/pcnt.h"


#define CH_1_ENCODER_A_PIN 4
#define CH_1_ENCODER_B_PIN 5


#define CH_2_ENCODER_A_PIN 26
#define CH_2_ENCODER_B_PIN 27


#define PCNT_UNIT_CH1 PCNT_UNIT_0

#define PCNT_UNIT_CH2 PCNT_UNIT_1

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 Hardware Counter Example");

  // A's rising edge counts, B's level at that edge picks the sign.
  // Must stay in step with Counter.h, which is what the firmware uses.
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = CH_1_ENCODER_A_PIN,
    .ctrl_gpio_num = CH_1_ENCODER_B_PIN,
    .lctrl_mode = PCNT_MODE_REVERSE,        // B low  -> count down
    .hctrl_mode = PCNT_MODE_KEEP,           // B high -> count up
    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_DIS,
    .counter_h_lim = 32767,         // wraps to 0 here, does not saturate
    .counter_l_lim = -32768,
    .unit = PCNT_UNIT_CH1,
    .channel = PCNT_CHANNEL_0,
  };

  pcnt_unit_config(&pcnt_config);

  // CH2 is mounted opposite-handed, so its control modes mirror CH1's.
  pcnt_config_t pcnt_config_ch2 = {
    .pulse_gpio_num = CH_2_ENCODER_A_PIN,
    .ctrl_gpio_num = CH_2_ENCODER_B_PIN,
    .lctrl_mode = PCNT_MODE_KEEP,           // B low  -> count up
    .hctrl_mode = PCNT_MODE_REVERSE,        // B high -> count down
    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_DIS,
    .counter_h_lim = 32767,
    .counter_l_lim = -32768,
    .unit = PCNT_UNIT_CH2,
    .channel = PCNT_CHANNEL_0,
  };

  pcnt_unit_config(&pcnt_config_ch2);
  

  pcnt_counter_pause(PCNT_UNIT_CH1);
  pcnt_counter_clear(PCNT_UNIT_CH1);
  pcnt_counter_resume(PCNT_UNIT_CH1);

    pcnt_counter_pause(PCNT_UNIT_CH2);
  pcnt_counter_clear(PCNT_UNIT_CH2);
  pcnt_counter_resume(PCNT_UNIT_CH2);

}

void loop() {
  int16_t count_ch1 = 0;
  int16_t count_ch2 = 0;
  pcnt_get_counter_value(PCNT_UNIT_CH1, &count_ch1);
   pcnt_get_counter_value(PCNT_UNIT_CH2, &count_ch2);
  Serial.printf("Count1: %d Count2: %d\n", count_ch1, count_ch2);
  delay(500);
}
