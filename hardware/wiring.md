# Wiring

An **ESP32 devkit** (CH340 USB-serial) drives **two AASD-15A servo drives** in
torque mode. The ESP32 has no analog output, so the torque command comes from a
**GP8413 dual 15-bit DAC** on I²C; each drive returns an incremental encoder
signal to a hardware pulse counter (PCNT) on the ESP32.

The ESP32-side tables are read out of the firmware and are authoritative.
Drive-side details marked **TBD** run correctly on the author's rig but have not
been written up here yet; if you rebuild this, verify them against your own
hardware.

> **The section 3 and 4 TBDs are the ones that matter for safety** — the analog
> torque command (0 V = zero torque, speed limit set), the enable path and the
> safety switch. Confirm them on your own build before putting the harness on
> someone. See [Safety](../README.md#safety) in the project README.

![Connection map: ESP32 pins on the left; safety switch, three relay boards,
four RS-485 to TTL receivers and the GP8413 DAC in the middle; the two drives'
CN2 pins on the right](images/wiring-map.svg)

Each row is one wire: the ESP32 pin on the left, whatever sits in between, and
the drive `CN2` pin on the right. The sections below repeat the same connections
as tables, with the notes and the **TBD**s that go with them.

---

## 1. ESP32 pin map

Every GPIO the firmware touches. Sources are in
`controller-firmware/src/irtactile-belt/`.

| GPIO | Dir | Function | Configured as |
|---|---|---|---|
| 1 | out | UART0 TX, data link (unused, host→device only) | 1 200 000 baud 8N1 |
| 3 | in | UART0 RX, tactile sample stream from `host-bridge` | 1 200 000 baud 8N1 |
| 4 | in | Encoder ch1 **A** (pulse) | PCNT unit 0, rising edge |
| 5 | in | Encoder ch1 **B** (direction) | PCNT unit 0 control input |
| 13 | out | **Servo enable 2** (relay) | `OUTPUT`; HIGH at boot, LOW after 1 s = drive active |
| 14 | out | Diagnostic / scope trigger | `OUTPUT`, driven LOW, never toggled |
| 16 | in | UART1 RX, logging (unused) | `Serial1`, 115 200 8N1 |
| 17 | out | UART1 TX, log output | `Serial1`, 115 200 8N1 |
| 18 | in | Mode button (momentary, to GND) | `INPUT_PULLUP`, 50 ms debounce |
| 19 | in | Status input | `INPUT_PULLUP`; HIGH ⇒ `INACTIVE` |
| 21 | I/O | I²C **SDA** to GP8413 | internal pull-up, 1 MHz |
| 22 | out | I²C **SCL** to GP8413 | internal pull-up, 1 MHz |
| 25 | out | **Servo enable 1** (relay) | `OUTPUT`; HIGH at boot, LOW after 1 s = drive active |
| 26 | in | Encoder ch2 **A** (pulse) | PCNT unit 1, rising edge |
| 27 | in | Encoder ch2 **B** (direction) | PCNT unit 1 control input |

Boot-time constraints:

- **Leave GPIO 12 unconnected.** It is the `MTDI` strapping pin and must read
  LOW at reset or the chip picks the wrong flash supply voltage. A relay board's
  `IN` pin is not high-impedance: it pulls toward 3.3 V through the input opto,
  which is why drive 1's enable was moved to **GPIO 25**. Do not fit a stronger
  external pull-down instead; it fights the opto LED current.
- **GPIO 1 / 3 are the USB-serial pins**, shared with the CH340. The boot log
  appears on GPIO 1 at every reset, and nothing may drive GPIO 3 while flashing.
- **Both drives are inactive for the first ~1 s of every boot** (enable pins
  HIGH, relays open). They are enabled just *before* `dac.begin()` writes the
  range register. **TBD: confirm the GP8413 powers up at 0 V**, otherwise move
  the enable writes after `dac.begin()`.
- Before the ESP32 boots the pins float, but the relay contacts are normally
  open, so the drives stay inactive through reset and while unpowered.

---

## 2. ESP32 ⇄ GP8413 DAC

The module's input side is just four terminals: `D`, `C`, `+`, `−`. It runs
from a single 5 V rail; there is no separate analog supply.

| GP8413 pin | Connects to | Notes |
|---|---|---|
| `C` (SCL) | GPIO 22 | 1 MHz, above the specified 400 kHz; see the tick budget in `Playout.h` |
| `D` (SDA) | GPIO 21 | internal pull-ups enabled in software. **TBD: external pull-up fitted?** (~45 kΩ internal is marginal at 1 MHz) |
| `−` | ESP32 GND | the module's only ground; also the reference for the analog outputs |
| `+` | **5 V from the USB rail** | the module's only supply (per its datasheet). An on-board boost converter makes the 0-10 V, so there is no separate ≥ 12 V analog rail to feed. |
| VOUT0 / VOUT1 | drive #1 / #2 `Vref` | 0-10 V, 15-bit |

Firmware configuration: address `0x58`, 1 MHz, range register `0x01` = `0x77`
(**0-10 V on both channels**), one auto-incrementing 5-byte write per 6 kHz tick.
Full scale is 15-bit: code `32767` = 10.000 V, **1 LSB ≈ 0.305 mV**.

> **Only half the drive's input range is reachable.** `Vref` accepts −10…+10 V
> but the GP8413 outputs 0…+10 V, so each channel pulls in one direction only.
> That matches the firmware (the command is clamped at zero, never signed) and
> the mechanics: CH2 gets its opposing direction from being mounted
> opposite-handed, not from a negative command.
> **TBD: confirm 0 V is exactly zero torque at the drive, not mid-scale.**

---

## 3. GP8413 → AASD-15A analog torque command

The drive's analog directive input on `CN2` is single-ended and bipolar,
−10…+10 V, entering through 10 kΩ into an op-amp with a 10 kΩ leg to `AGND`.

| Signal | From | To (`CN2`) |
|---|---|---|
| Torque command ch1 | `VOUT0` | drive #1 **pin 25 `Vref`** |
| Analog return ch1 | DAC `GND` | drive #1 **pin 13 `AGND`** |
| Torque command ch2 | `VOUT1` | drive #2 **pin 25 `Vref`** |
| Analog return ch2 | DAC `GND` | drive #2 **pin 13 `AGND`** |

**Return topology: star, not daisy chain.** Each drive has its own `AGND`
conductor back to the DAC ground point, so neither drive's return current flows
through the other's reference. **TBD: shielded pair, shield grounded at one end.**

> **`AGND` is not isolated.** Unlike the enable path (section 4), both drives' pin 13
> and the GP8413 ground are one net, which reaches USB ground and the host PC
> through the devkit: a ground loop between the drive chassis and the host.

Drive parameters that must match this wiring (**TBD: record the actual
parameter numbers**): torque mode with analog command; analog full-scale
voltage, gain, offset (0 V = zero torque) and dead band; torque/current limit;
and a **speed limit**, which torque mode otherwise leaves unbounded.

---

## 4. ESP32 → AASD-15A enable (via relay)

The ESP32 does not drive the servo-on inputs directly. Each enable GPIO drives a
**relay board** (`SRD-05VDC-SL-C`, active-low input, normally-open contact),
which provides both the level shift and the isolation.

| GPIO level | Contact | Drive pin 6 | Drive |
|---|---|---|---|
| **HIGH** | open | floating | **inactive** (first ~1 s of boot) |
| **LOW** | closed | tied to supply common | **active** |

| ESP32 | Signal | Relay | To |
|---|---|---|---|
| GPIO 25 | `ENABLE_PIN_1` | 1 | drive #1 `CN2` pin 6 |
| GPIO 13 | `ENABLE_PIN_2` | 2 | drive #2 `CN2` pin 6 |
| GND | n/a | 3 | ESP32 GPIO 19 (status input, see below) |

Board supplies, per relay board (**three boards in total**):

| Pin | Fed from |
|---|---|
| `JD-VCC` | **USB 5 V**, unswitched; relay coil supply |
| `VCC` | **3.3 V through the safety switch**; input optocoupler supply |
| `GND` | common |
| `IN` | GPIO 25 / GPIO 13 / **tied to GND** (relay 3) |

### Safety switch

The safety switch sits **in the 3.3 V supply to the relay boards' input side**,
not in the coil current and not in any mains circuit, so it is an ordinary
low-voltage, low-current switch. Cutting that 3.3 V starves the input
optocouplers on all three boards and every coil de-energises regardless of what
the GPIOs are doing.

```
   safety switch ──▶ 3.3 V to relay board inputs ──┬──▶ relay 1  (IN ◀ GPIO 25)
                                                   ├──▶ relay 2  (IN ◀ GPIO 13)
                                                   └──▶ relay 3  (IN ◀ GND)

   USB 5 V ─────────▶ coil supply (all three)
```

Relay 3 has no GPIO: its input is wired straight to ground, so it conducts
whenever the switched 3.3 V is present. It is a direct electrical report of the
safety switch, which the firmware reads on GPIO 19.

| Safety switch | Relays 1 & 2 | Relay 3 | GPIO 19 | Result |
|---|---|---|---|---|
| **on** | firmware-controlled | closed | **LOW** | normal operation |
| **off** | **open, drives disabled** | open | **HIGH** | `INACTIVE`, DAC commanded to 0 V |

> **The `VCC`-`JD-VCC` jumper must stay removed.** Separating those rails is what
> puts the switch in the path at all. Refitting it (the state these boards ship
> in) would tie `VCC` to the unswitched 5 V and silently defeat the kill path,
> with no symptom other than a safety switch that no longer does anything. Check
> after any board swap or repair.

> **One switch, two independent effects.** The hardware path disables both drives
> immediately and cannot be overridden by firmware; the firmware path notices on
> its next `loop()` pass (100 ms poll) and zeroes the DAC. Hardware first, then
> the command is withdrawn.
>
> **Re-arming is not automatic.** When the switch returns, the firmware moves to
> `PRE_INITIALIZED` and waits for the mode button (section 6), never straight back to
> driving.

### Drive-side input circuit

The AASD-15A digital inputs are optocouplers sharing one common anode terminal,
with a **5 kΩ series resistor internal to the drive**; no external resistor
wanted. Supply positive goes to pin 9, supply negative to `COM(10)`, and a
contact between the two energises the LED.

```
   +V ──▶ pin 9 ──[ 5 kΩ ]──▶ |◀ opto LED ──▶ pin 6 (SigIn1) ──▶ relay ──▶ supply −
```

| `CN2` pin | Signal | Use |
|---|---|---|
| **9** | `DC12~24V`, input common, anode side | **fed 5 V** |
| **6** | `SigIn1`, SRV-ON | **the enable** |
| **10** | `COM`, input common return | **TBD: wired, or does the relay return straight to USB 0 V?** |

**Why pin 9 runs at 5 V and not 12-24 V.** With the internal 5 kΩ and ~1.2 V LED
drop, forward current tracks the supply: ≈4.6 mA at 24 V, ≈2.2 mA at 12 V,
**≈0.76 mA at 5 V**. That is below the datasheet range on paper, but it is
long-established practice in the **SFX-100** motion-simulator community with
these same drives, and it lets the whole control side run from the single USB
5 V rail with no separate 24 V supply. If enabling ever proves marginal, the
relay contact is dry, so only pin 9's wire and the contact return move to 12 or
24 V; the coil stays at 5 V and the GPIO at 3.3 V.

> **The isolation barrier is the drive's own optocoupler**, so nothing on the
> ESP32/USB side is galvanically tied to the drive's power ground.

> **Fail-safe direction.** A de-energised relay, a broken coil wire, a lost coil
> supply, or an unpowered/resetting ESP32 all leave pin 6 floating and the drives
> **inactive**. Activation requires the firmware to actively hold the GPIO low.

**TBD: the 5 V current budget.** Two coils (~70-80 mA each) plus the ESP32 share
the USB rail, and both energising together is a meaningful step load.

### Unused drive I/O

`SigIn2` (7, alarm reset), `SigIn3` (21), `SigIn4` (8) and the open-collector
outputs `SigOut1` (11, **SRV-READY**), `SigOut2` (23, **alarm detection**),
`SigOut3` (12), `SigOut4` (24) are unwired. Two matter: a faulted drive is
currently invisible to the firmware and to the log, and there is no way to clear
a drive fault without a power cycle. **TBD: how the unused inputs are left**
(floating is the inactive state; the LED needs a return path to conduct).

**TBD: whether GPIO 19 is also meant to be fed from `SigOut1` (`SRV-READY`)**, or
from the safety switch alone.

---

## 5. AASD-15A → ESP32 encoder feedback

Each drive returns its **position pulse output** on `CN2`: a differential
line-driver copy of the motor encoder, referenced to `DGND`. The firmware counts
**rising edges on A only** and uses the level of B at that edge for direction, so
it is a 1× (not 4×) decode.

| `CN2` pin | Signal | Used |
|---|---|---|
| **20 / 19** | `PA+` / `PA−` | **yes** → PCNT A input |
| **18 / 17** | `PB+` / `PB−` | **yes** → PCNT B input |
| 15 / 14 | `PZ+` / `PZ−`, index | no |
| 22 | `OZ`, Z open collector | no |
| **16** | `GND` (`DGND`) | **yes** |

### Differential receivers: RS-485/TTL modules

The pairs do not reach the ESP32 directly. Each is received by a small
**RS-485-to-TTL module** (MAX3485-type transceiver plus a `74HC04` for automatic
direction control) used purely as a line receiver: the pair goes in on `A+`/`B−`,
the single-ended TTL copy comes out on `RXD`. An RS-485 receiver and an encoder
line receiver are the same thing electrically, and these are far cheaper and
easier to source than a dedicated `AM26LS32`.

**One module per signal, four in total:**

| Module | Input `A+` / `B−` | `RXD` → | Carries |
|---|---|---|---|
| 1 | drive #1 `PA+` (20) / `PA−` (19) | GPIO 4 | ch1 A |
| 2 | drive #1 `PB+` (18) / `PB−` (17) | GPIO 5 | ch1 B |
| 3 | drive #2 `PA+` (20) / `PA−` (19) | GPIO 26 | ch2 A |
| 4 | drive #2 `PB+` (18) / `PB−` (17) | GPIO 27 | ch2 B |

Line side (three-way terminal, bottom edge): `接大地` (earth) → **controller
GND**, `B−` → `PA−`/`PB−`, `A+` → `PA+`/`PB+`. Earth goes to controller ground
rather than chassis PE so the receiver's common-mode reference sits on the same
node as `DGND` and the ESP32; an RS-485 receiver's inputs must stay inside the
common-mode window relative to its own ground.

Logic side (four-way header, top edge): `GND` → ESP32 GND (common with drive
`DGND`), `RXD` → the GPIO above, `TXD` → **not connected**, `VCC` → **3.3 V from
the devkit's 3V3 rail**.

Each module carries ~120 Ω termination across `A`/`B`, TVS diodes and resettable
fuses on the line side. Each pair is point-to-point, so one 120 Ω per pair is
correct. **TBD: confirm the exact transceiver marking, and that only the module
terminates.** A termination enabled at the drive as well would halve the load.

> **The 3.3 V supply is what makes `RXD` directly GPIO-safe.** The module's logic
> output cannot exceed the rail the GPIOs are referenced to, so no level shifting
> is needed anywhere in the encoder path. A 5 V-powered module (or a MAX485
> board, which looks identical) would put 5 V on pins rated 3.6 V absolute
> maximum. Rail load is a few mA; the terminations are driven by the drive's line
> driver, not by this rail.

> **`TXD` is deliberately disconnected.** The `74HC04` enables the RS-485 *driver*
> when `TXD` goes low, which would put the module in contention with the drive's
> line driver. With `TXD` open, the board's pull-up holds the driver off; the
> first thing to check if contention is ever suspected.

### ESP32 side

| ESP32 | Signal | PCNT behaviour |
|---|---|---|
| GPIO 4 | ch1 **A** (pulse) | unit 0, rising edge = 1 count |
| GPIO 5 | ch1 **B** (direction) | B **low ⇒ count down**, high ⇒ up |
| GPIO 26 | ch2 **A** (pulse) | unit 1, rising edge = 1 count |
| GPIO 27 | ch2 **B** (direction) | **mirrored:** B low ⇒ **up**, high ⇒ down |
| GND | common | module `GND` + drive `DGND` (pin 16) |

Nothing between the drive and the GPIO inverts polarity, so the PCNT direction
modes in `Counter.h` describe the as-built behaviour. If a counter ever reads
backwards, the cause is mechanical handing or a swapped pair at the terminals.

> **CH2's direction modes are deliberately the mirror of CH1's** because the
> second unit is mounted opposite-handed (`Counter.h:87-88`). Do not "tidy" them
> to match; if the mechanics are ever re-handed, this is the place to change.

Counter behaviour: 1 µs glitch filter (80 APB cycles, passes up to ~500 kHz);
signed 16-bit counters that **wrap** rather than saturate, folded back in
software by `WrapTracker.h`; reset on the button release that arms the unit.

> **The belt has no mechanical stop.** Past the count where the torque command
> saturates, resistance stops rising and the pull becomes constant-force, so
> **32767 counts is reachable** in a lurch or a fall — which is what the wrap
> tracking above is for. Where that saturation point sits is a firmware setting,
> not a wiring fact: see
> [`protocol.md`](../controller-firmware/protocol.md#tuning-header).

**Counts are never converted to distance, and do not need to be.** The drives run
in torque mode, so the count is a relative displacement from the zero set at
arming and feeds a **floor under the torque command**: a virtual spring, not a
position loop. The counts-to-force mapping is a runtime-tunable feel parameter,
so there is no counts-per-revolution figure to record.

**TBD: the travel actually available**, whether the mechanism can physically
reach 32767 counts. Firmware README bench step 8 answers it by watching
`Wrap1`/`Wrap2` on a full pull.

---

## 6. Front panel / operator controls

| ESP32 | Control | Wiring | Behaviour |
|---|---|---|---|
| GPIO 18 | Mode button | momentary switch to **GND**, `INPUT_PULLUP` | **held** = `INITIALIZING` (fixed preload, code 2000 ≈ 0.610 V); **released** = `INITIALIZED` (armed, encoders zeroed) |
| GPIO 19 | Status input | **relay 3 contact to GND**, `INPUT_PULLUP` | **HIGH (open) = `INACTIVE`**, outputs forced to 0 V; LOW permits arming (section 4) |
| GPIO 14 | Diagnostic | test point / scope probe | driven LOW at startup, never toggled |

**TBD: connector, cable and enclosure details for the panel controls.**

> **TBD: which physical drive is "channel 1".** The firmware pairs encoder ch1
> (GPIO 4/5) and stream slot 1 with DAC `CH0`, but nothing ties `ENABLE_PIN_1`
> (GPIO 25) to the drive fed by `VOUT0`; that is purely a wiring convention and
> must be recorded here.

---

## 7. Power and grounding

| Item | Value |
|---|---|
| ESP32 supply | USB (CH340 devkit) |
| Relay coils (×3) | 5 V from the USB rail |
| Relay board input side | 3.3 V, **through the safety switch** (section 4) |
| Drive digital-input supply (pin 9) | **5 V from the USB rail**, below the datasheet 12-24 V, deliberately (section 4) |
| RS-485→TTL receivers (×4) | **3.3 V from the devkit's 3V3 rail** (section 5) |
| GP8413 supply | **5 V from the USB rail**, single rail; on-board boost makes the 0-10 V output (section 2) |
| Drive mains | 220 V into `L1`/`L2`/`L3` + `PE`; **TBD: single or three phase, filter/breaker** |
| Motor cabling | `U`/`V`/`W` + `PE`; **TBD: cable type, gland/shield termination** |
| Motor encoder cabling | `CN3` drive ↔ motor, 15-way; **TBD: supplied cable or made up?** |
| Ground topology | the enable path is opto-isolated at the drive; the **analog command is not**: the GP8413 ground and both drives' `AGND` are one shared net (section 3). Star point TBD. |
| Shielding, earth bonding, fusing | TBD |
| E-stop / kill chain | safety switch in the 3.3 V relay-input supply (section 4); **TBD: where it is mounted** |

---

## 8. Host connections

| Link | Cable | Notes |
|---|---|---|
| Data | USB → CH340 on the devkit | 1 200 000 baud, **host → device only**. DTR/RTS must stay deasserted or the CH340 auto-reset circuit reboots the ESP32 mid-stream. |
| Logging | USB-TTL adapter on **GPIO 17** (or a second ESP32 running `tools/logger/logger.ino`) | 115 200 baud, device → host only. Required equipment; the drift-lock pass criterion is only observable here. |

---

## 9. AASD-15A connector reference

From the drive's *"Speed, torque control wiring diagram"*. **Bold = used.**

### `CN2` control I/O

| Pin | Signal | Function | Used |
|---|---|---|---|
| **9** | `DC12~24V` | digital input common (anode side) | **5 V here** |
| **6** | `SigIn1` | SRV-ON | **enable, via relay** |
| 7 | `SigIn2` | Alarm reset | |
| 21 | `SigIn3` | Position deviation clear | |
| 8 | `SigIn4` | ZEROSPD | |
| 10 | `COM` | common for inputs and outputs | |
| 11 | `SigOut1` | SRV-READY | |
| 23 | `SigOut2` | Alarm detection | |
| 12 | `SigOut3` | Position completed | |
| 24 | `SigOut4` | Zero speed | |
| **25** | `Vref` | analog command, −10…+10 V | **← DAC** |
| **13** | `AGND` | analog ground | **← DAC GND** |
| **20 / 19** | `PA+` / `PA−` | A phase pulse output | **→ PCNT A** |
| **18 / 17** | `PB+` / `PB−` | B phase pulse output | **→ PCNT B** |
| 15 / 14 | `PZ+` / `PZ−` | Z phase (index) pulse output | |
| 22 | `OZ` | Z open-collector output | |
| **16** | `GND` (`DGND`) | pulse output common | **→ ESP32 GND** |

Each `SigIn` has a **5 kΩ** series resistor into its optocoupler; `Vref` enters
through **10 kΩ** into an op-amp with a 10 kΩ leg to analog ground.

### `CN1` communication (unused)

Pin 2 `+5 V`, 4 / 6 RS-485 `A` / `B`, 1 / 3 RS-232 `Tx` / `Rx`, 5 `Gnd`. This is
how drive parameters are set from a PC if the keypad is inconvenient.

### `CN3` motor encoder (factory cable, drive ↔ motor)

| Pin | Signal | | Pin | Signal |
|---|---|---|---|---|
| 8 | +5 V | | 6 / 7 | `U+` / `U−` |
| 3 / 11 | `A+` / `A−` | | 13 / 12 | `V+` / `V−` |
| 10 / 2 | `B+` / `B−` | | 5 / 4 | `W+` / `W−` |
| 1 / 9 | `Z+` / `Z−` | | 15 | `GND` |
| | | | 14 | `PE` |

Not part of the controller wiring; the `PA`/`PB` outputs on `CN2` are the
drive's buffered copy of this.

### Power terminals

`L1` / `L2` / `L3`: 220 V mains via a power filter; `PE`: protective earth;
`U` / `V` / `W` / `PE`: to the servo motor.

---

