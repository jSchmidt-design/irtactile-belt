// GP8413 bench sketch: ramp each channel, then set the two to different levels.
// Includes the real driver header so it exercises GP8413_DAC itself.
//
// SAFETY: this commands the analogue inputs of both drives directly. It never
// touches ENABLE_PIN_1/ENABLE_PIN_2, which reset to floating inputs, so the
// enable relays stay open and the drives stay disabled - the ramps are visible
// on a meter at the DAC outputs and nothing moves. If you deliberately energise
// the drives to watch the belts respond, open the safety switch first and keep
// a hand on it: the ramp goes to full scale.

#include "../../src/irtactile-belt/GP8413_DAC.h"

GP8413_DAC dac;

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!dac.begin(OUTPUT_RANGE_10V)) {
    Serial.println("GP8413 not responding - check SDA 21 / SCL 22 and power");
    return;
  }

  Serial.println("\n--- Starting DAC Test Sequence ---");

  Serial.println("Test 1: Ramping Channel 0 from 0V to max");
  for (uint16_t voltage = 0; voltage <= dac.getMaxVoltage(); voltage += 500) {
    dac.setVoltage(voltage, 0);
    Serial.print("CH0: ");
    Serial.print(voltage);
    Serial.println(" mV");
    delay(200);
  }

  delay(1000);

  Serial.println("Test 2: Ramping Channel 1 from 0V to max");
  for (uint16_t voltage = 0; voltage <= dac.getMaxVoltage(); voltage += 500) {
    dac.setVoltage(voltage, 1);
    Serial.print("CH1: ");
    Serial.print(voltage);
    Serial.println(" mV");
    delay(200);
  }

  delay(1000);

  Serial.println("Test 3: Setting both channels to different values");
  dac.setBothVoltages(2500, 7500);
  Serial.println("CH0: 2500 mV, CH1: 7500 mV");

  delay(2000);

  // Nothing is persisted; the GP8413 comes back at whatever it powers up with.
  Serial.println("Test 4: Resetting outputs to 0V");
  dac.setBothVoltages(0, 0);

  Serial.println("--- Test Sequence Complete ---\n");
}

void loop() {
  delay(1000);
}
