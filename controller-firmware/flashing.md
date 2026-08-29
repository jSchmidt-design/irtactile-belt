# Flashing the controller firmware

## Requirements

- ESP32 devkit with CH340 USB-serial adapter
- `arduino-cli` or the Arduino IDE with the **esp32 core 3.x** installed
- A USB cable and the correct COM port

## Steps

1. Open a terminal in [`controller-firmware/`](.).
2. Run the build script:
   ```
   compile.bat
   ```
3. To upload, run:
   ```
   compile.bat -u -p COM6
   ```
   Replace `COM6` with your board's port.

## Troubleshooting

- If the board resets unexpectedly when the host bridge opens the serial port,
  the CH340 auto-reset circuit is being triggered by DTR/RTS. The bridge must
  suppress these line states so streaming does not reboot the ESP32.
- The logging UART on GPIO 17 requires a separate USB-TTL adapter (or a second
  ESP32 running [`tools/logger/logger.ino`](tools/logger/logger.ino))
  to read `Serial1` debug output.
