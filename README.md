# irtactile-belt

Uses irTactile as the signal generator to give the open-source **Hörnle belt
tensioner** torque-based control: ESP32 controller firmware, the host-side
serial bridge that feeds it, and the hardware and protocol documentation.

The tensioner's two AASD-15A servo drives pull the harness belts and take their
torque command as an analog Vref. Build instructions for the tensioner itself:
[English](https://www.overtake.gg/threads/hoernles-belt-tensioner-detailed-build-instructions.249951/),
[German](https://forum.virtualracing.org/threads/hoernles-belt-tensioner-detailierte-bauanleitung.132799/).

```
irTactile engine ──► shared memory ──► host-bridge ──USB/CH340──► ESP32 ──► GP8413 DAC ──► AASD-15A drives ──► belts
```

![The assembled controller: an 8-channel relay module, the GP8413 DAC, an ESP32
devkit and two DB25 breakouts on a 3D-printed tray, with an emergency-stop
button wired in as the safety switch](hardware/images/controller.jpeg)

The ESP32, DAC, relays and switch wiring live on one tray; see
[`hardware/`](hardware/) for the full pin-level reference and the parts list.

## Status

**v0.** Roughly tested and working on the author's rig. Documentation is still a
work in progress — several entries in
[`hardware/wiring.md`](hardware/wiring.md) are marked **TBD**.

## Safety

This project drives two 220 V AASD-15A servo drives that pull a harness against
a person's torso. It runs on the author's rig, but it is not a certified safety
system and carries no warranty (see [`LICENSE`](LICENSE)). **If you build it,
you are responsible for your own setup.**

- Set a torque limit **and a speed limit** in the drive parameters — torque mode
  leaves speed unbounded otherwise.
- The emergency-stop switch is the only thing that disables the drives
  independently of the firmware. Check that it cuts both drives before each
  session, and re-check after any change to the relay wiring (the `VCC`/`JD-VCC`
  jumper must stay removed).
- Confirm the fail-safe behaviour and that 0 V is zero torque on your own drives
  before putting the harness on someone.
- Keep clear of the belt path while the drives are enabled.

## Advantages

**Torque mode needs no calibration.** The drives take a *force* command, not a
position command, so there is nothing to teach the software: hold the init
button until the harness is taken up, release, and the unit is armed. Because
preload is a force, it is identical on every run: a different driver, a seating
position moved by an inch, a thick jacket instead of a shirt all change where
the belt sits, not how hard it pulls. A position-based tensioner has to be
re-zeroed for each of those.

**Smooth signal.** The command is a streamed waveform, not a set point that gets
nudged: 15-bit resolution (1 LSB ≈ 0.305 mV of `Vref`) at up to 6000 samples per
second, with the encoder-derived spring floor applied underneath it. Effects
arrive as continuous force rather than as discrete steps.

**Free mixing.** irTactile does the signal generation, so the belt is not
limited to a fixed table of effects: braking, longitudinal g, ABS, kerbs and
rumble all sum into the two channels, and adding or reshaping one is a change in
irTactile, not in the firmware or the protocol.

**Two belts in one.** The same hardware behaves like the automatic inertia-reel
belt of a passenger car (slack until it is asked to pull) or, held at preload,
like a fixed harness in a race car.


## Components

| Component | Description |
|---|---|
| [`host-bridge/`](host-bridge/) | Windows console app: reads the irTactile shared-memory stream, encodes two channels and writes them to a COM port |
| [`controller-firmware/`](controller-firmware/) | ESP32 Arduino firmware: decodes the serial stream and drives a GP8413 dual DAC |
| [`hardware/`](hardware/) | Wiring map, component diagrams and the pin-level reference for the ESP32, DAC, relays and drives |


## Quick start

1. Wire the hardware: [`hardware/wiring.md`](hardware/wiring.md).
2. Flash the firmware: [`controller-firmware/flashing.md`](controller-firmware/flashing.md),
   [`controller-firmware/README.md`](controller-firmware/README.md).
3. Build and run the bridge: [`host-bridge/README.md`](host-bridge/README.md).
