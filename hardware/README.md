# Hardware setup

This folder contains the hardware documentation for the irTactile belt.

> **Safety.** These drives run on 220 V mains and pull a harness against a
> person. The setup runs on the author's rig, but this is a **v0** and parts of
> the drive-side wiring in [`wiring.md`](wiring.md) are still marked **TBD** —
> not yet written up from the working build. If you rebuild it, verify the drive
> parameters (torque **and speed** limit), the enable path and the safety switch
> against your own hardware. Build and use at your own risk; no warranty. See
> [Safety](../README.md#safety) in the project README.

- [`wiring.md`](wiring.md): pinout and wiring diagrams
- [`images/block-diagram.svg`](images/block-diagram.svg): the overview below
- [`images/wiring-map.svg`](images/wiring-map.svg): every wire, pin to pin
- [`images/`](images/): photos, schematics and scope captures

![The assembled controller on its 3D-printed tray: an 8-channel relay module
along the top, the GP8413 Gravity DAC in the centre, the ESP32 devkit on its
expansion board below it, a DB25 breakout and two RS-485 modules on each side,
mains and low-voltage terminal blocks, and the emergency-stop safety switch on
the left](images/controller.jpeg)

*The assembled controller. Everything in the diagrams below is on this tray; the
emergency-stop button on the left is the safety switch.*

## Overview

A host PC streams two tactile channels over USB to an **ESP32 devkit**. The
ESP32 has no analog output, so it converts each sample to a voltage with a
**GP8413 dual 15-bit DAC**, and those two voltages are the torque command for
two **AASD-15A servo drives** (one per belt channel) running in torque mode.

Everything else exists to close the loop or to make it safe to stand in:

- each drive returns an **incremental encoder** signal, received through
  **RS-485→TTL modules** and counted by the ESP32's hardware pulse counters, so
  the firmware knows how far the belt has been pulled out;
- a **safety switch** cuts the drives' enable path in hardware;
- an **init switch** (the mode button) sets the belt preload and arms the
  system afterwards.

![Block diagram: host PC to ESP32 to GP8413 DAC to two AASD-15A drives, with
encoder feedback through four RS-485 to TTL receivers and a safety switch in the
relay supply](images/block-diagram.svg)

*The safety switch and init switch are the two operator controls; everything
else is fixed wiring. Full pin-level detail is in [`wiring.md`](wiring.md),
which also carries a text version of this diagram.*

## Main components

| Component | Count | Role |
|---|---|---|
| **ESP32 devkit** (CH340) | 1 | The controller. Decodes the serial stream, writes the DAC every tick, counts encoder pulses, runs the arming state machine. USB-powered; the 5 V and 3.3 V rails feed everything else on the control side. |
| **GP8413 dual DAC** | 1 | 15-bit I²C DAC at address `0x58`, both channels set to the 0-10 V range. `VOUT0`/`VOUT1` are the analog torque command (`Vref`) for the two drives. 1 LSB ≈ 0.305 mV; code 32767 = 10.000 V. |
| **RS-485→TTL modules** | 4 | Line receivers for the drives' differential encoder outputs, one per signal (`PA` and `PB` for each drive). The pair goes in on `A+`/`B−`, a single-ended copy comes out on `RXD`. Powered from **3.3 V**, which is what makes the output directly GPIO-safe. |
| **Relay boards** (`SRD-05VDC-SL-C`) | 3 | Relays 1 and 2 are the servo-enable path: they level-shift and isolate the ESP32 from the drives' `SRV-ON` inputs. Relay 3 has no GPIO: it reports the safety switch back to the ESP32. |
| **Safety switch** | 1 | Sits in the **3.3 V supply to the relay boards' input side**: a low-voltage, low-current switch, not in the coil current and not in any mains circuit. Opening it starves all three input optocouplers, so both drives are disabled in hardware regardless of what the firmware is doing. |
| **Init switch** (mode button) | 1 | Momentary switch to GND on GPIO 18. **Held** = `INITIALIZING`: both belts hold a fixed preload so the harness can be taken up. **Released** = `INITIALIZED`: encoders are zeroed and the belt is armed. Re-arming after a safety trip always goes through this button. |
| **AASD-15A servo drives** | 2 | 220 V drives in torque mode, taking the analog command on `CN2` pin 25 and returning a buffered encoder copy on `CN2` pins 17-20. Enabled by a contact closure on `CN2` pin 6. |
| **Servo motors** | 2 | One per belt channel, mounted opposite-handed, which is where channel 2's direction comes from, not from a negative command. |
| **USB-TTL adapter** | 1 | Log output only, on GPIO 17 at 115 200 baud. Required equipment: the device log is the only place some behaviour is observable. |

## How the pieces relate

**Command path.** `host-bridge` → USB → ESP32 → I²C → DAC → `Vref` on each
drive. One-way; the ESP32 never answers the host.

**Feedback path.** Motor encoder → drive → differential `PA`/`PB` on `CN2` →
RS-485→TTL module → ESP32 pulse counter. The count is a *relative* displacement
from the zero set at arming and feeds a floor under the torque command: a
virtual spring, not a position loop. **6667 counts = full scale**; beyond that,
pulling adds no more force.

**Safety path.** The safety switch has two independent effects from one action.
The hardware path opens relays 1 and 2 and disables both drives immediately, and
firmware cannot override it. The firmware path sees relay 3 open on GPIO 19
within one 100 ms `loop()` pass, enters `INACTIVE` and commands the DAC to 0 V.
Hardware first, then the command is withdrawn. **Re-arming is never automatic**;
the firmware waits for the init switch.

**Fail-safe direction.** A de-energised relay, a broken wire, a lost supply or an
unpowered ESP32 all leave the drives' enable input floating, which is *inactive*.
Driving the belt requires the firmware to actively hold a GPIO low.

## Parts I used

The off-the-shelf modules on the control side, with the exact listings this
build was assembled from. Most links are amazon.de and will rot; treat them as a
description of the part, not a shopping list. The drives, motors and switches
are covered in **Main components** above and in [`wiring.md`](wiring.md).

| Part | Role in this build | Listing |
|---|---|---|
| **ESP32 NodeMCU devkit**, USB-C, CH340, 30-pin (QIQIAZI) | The controller. | [B0DHRV7784](https://www.amazon.de/dp/B0DHRV7784) |
| **GP8413 dual 15-bit I²C DAC**, 0-5 V/0-10 V (DFRobot Gravity, DFR1073) | Turns each sample into the analog torque command (`Vref`) for the two drives. Set to the 0-10 V range, address `0x58`. | [DFR1073](https://www.dfrobot.com/product-2756.html) |
| **ESP32 30-pin expansion board**, "1 in 2" GPIO breakout (2-pack) | Carrier for the devkit; doubles each pin so the DAC, relays and RS-485 modules can all tap the same GPIO rail. | [B0DF7HQ8B3](https://www.amazon.de/dp/B0DF7HQ8B3) |
| **TTL↔RS-485 modules**, 3.3 V/5 V (AYWHP, 5-pack) | The four encoder line receivers, `PA`/`PB` per drive. Run from 3.3 V so `RXD` is GPIO-safe. | [B0F7LFVM6Q](https://www.amazon.de/dp/B0F7LFVM6Q) |
| **5 V relay module** with optocouplers, `SRD-05VDC-SL-C` relays (ELEGOO 8-channel) | Three relays needed: relays 1 and 2 are the servo-enable path, relay 3 reports the safety switch back. Any mix of channel counts works (3×1, 2×2, 1×4, …); I used an 8-channel board I had on hand. | [B01M61VVGV](https://www.amazon.de/dp/B01M61VVGV) |
| **DB25 gender changer / breakout board**, male-to-female with screw terminals (CEMYDEYO) | Breaks out each drive's `CN2` (DB25) to screw terminals: the analog command on pin 25 and the encoder copy on pins 17-20. | [B0F5LK9F38](https://www.amazon.de/dp/B0F5LK9F38) |

## Where to look next

| For | See |
|---|---|
| Every GPIO, connector pin and supply rail | [`wiring.md`](wiring.md) |
| Firmware pin map, state machine, bench procedure | [`../controller-firmware/README.md`](../controller-firmware/README.md) |
| Serial frame format | [`../controller-firmware/protocol.md`](../controller-firmware/protocol.md) |
| Flashing the controller | [`../controller-firmware/flashing.md`](../controller-firmware/flashing.md) |
