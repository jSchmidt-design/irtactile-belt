#ifndef PINS_H
#define PINS_H

#include <Arduino.h>
#include "soc/gpio_reg.h"

// Every GPIO the sketch itself claims. The two peripherals that own their own
// pins keep them with their configuration: encoder A/B in Counter.h, I2C
// SDA/SCL in GP8413_DAC.h.

// Logging UART (UART1) to the second ESP32 running tools/logger. Nothing is
// sent back, but both pins are configured.
#define LOGGING_RX1_PIN 16   // Serial1 RX (input from ESP-B)
#define LOGGING_TX1_PIN 17   // Serial1 TX (output to ESP-B)

// Data link from the host (UART0). Only RX is used; TX is claimed so the
// driver has a complete pin set.
#define UART_RX_PIN 3
#define UART_TX_PIN 1

// Scope trigger, HIGH for the duration of one DAC tick's work: rising edges
// give the tick cadence, pulse width the per-tick cost. Driven through the GPIO
// register rather than digitalWrite() to stay negligible in the 6 kHz tick
// path it measures. See dacTask().
#define DIAG_PIN 14
#define DIAG_HIGH() REG_WRITE(GPIO_OUT_W1TS_REG, 1u << DIAG_PIN)
#define DIAG_LOW()  REG_WRITE(GPIO_OUT_W1TC_REG, 1u << DIAG_PIN)

// Drive enables. LOW closes the relay and enables the drive; HIGH releases it.
//
// Drive 1 must not sit on GPIO 12 (MTDI): that pin is sampled at reset and must
// read LOW, but the relay board's IN pin is the cathode of its input opto LED
// fed from board VCC through ~1 kOhm, so it pulls toward 3.3 V whenever the
// safety switch is on and the chip selects a 1.8 V VDD_SDIO. Symptom: boots
// with the safety switch open, unreliably with it closed. Do not fit a stronger
// external pull-down instead - that fights the opto LED current and makes relay
// 1's turn-on marginal.
#define ENABLE_PIN_1 25
#define ENABLE_PIN_2 13

// Safety-switch sense. HIGH means the switch is open, i.e. INACTIVE.
#define STATUS_PIN 19

// Mode button, INPUT_PULLUP: LOW is pressed. Held-to-preload, see BeltState.h.
#define MODE_BUTTON_PIN 18

#endif
