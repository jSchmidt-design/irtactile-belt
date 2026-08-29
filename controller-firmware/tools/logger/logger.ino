// ESP-A: USB serial to the PC, Serial1 (pins 16 RX / 17 TX) to ESP-B.
// Forwards bytes in both directions.

const int RX1_pin = 16;
const int TX1_pin = 17;
const unsigned long BAUD = 115200;

void setup() {
  Serial.begin(115200);
  Serial1.begin(BAUD, SERIAL_8N1, RX1_pin, TX1_pin);
  Serial.println("ESP-A ready: USB serial for logs, Serial1 -> ESP-B for data");
}

void loop() {
  while (Serial1.available()) {
    int b = Serial1.read();
    Serial.write(b);
  }

  while (Serial.available()) {
    int b = Serial.read();
    Serial1.write(b);
  }
}
