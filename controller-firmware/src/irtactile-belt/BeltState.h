#ifndef BELT_STATE_H
#define BELT_STATE_H

#include <Arduino.h>

#include "Pins.h"
#include "Counter.h"

// The arming state machine: safety switch, mode button, and the preload level
// the two of them command. Two orderings are load-bearing: the preload level is
// published before the state that makes it live, and the encoder reset happens
// before INITIALIZED.
//
// Runs on core 1, from loop(). g_status and g_preloadLevel are read by the DAC
// path on core 0; both are aligned and read as single words there.

enum Status{
  INACTIVE,
  PRE_INITIALIZED,
  INITIALIZING,
  INITIALIZED,
};

// Written by beltStateUpdate() on core 1, read by dataHandler and processData
// on core 0.
volatile int g_status;

// Button-held preload level, published to processData().
volatile uint16_t g_preloadLevel = 0;

// The preload the belts hold while the mode button is down: the tension felt
// when setting the belt up, and the zero the encoder counts are measured from
// once the button is released. ~0.6 V of the 10 V range; bench-tunable.
#define PRELOAD_DAC_VALUE 2000

// Latched drive cut, set by writeDac() on core 0 after DAC_FAIL_DISABLE_RUN
// consecutive failures. Cleared only here, by a pass through INACTIVE, i.e. by
// the safety switch: a bus dead for a full second is a hardware fault, so
// recovery costs a deliberate re-arm rather than re-energising the drives
// against whatever command arrives first.
volatile bool g_drivesCut = false;

// Mode button on MODE_BUTTON_PIN, held-to-preload: pressed (LOW) is
// INITIALIZING, released (HIGH) is INITIALIZED. The state follows the debounced
// *level*, there is no toggle latch.
static bool s_buttonDown = false;        // debounced level, true = pressed
static bool s_lastButtonReading = HIGH;  // last raw reading (INPUT_PULLUP)
static unsigned long s_lastDebounceTime = 0;
static const unsigned long DEBOUNCE_DELAY_MS = 50;

// What one pass through the state machine did, for the caller to log. Nothing
// in this file prints; the announcements live in Logging.h.
struct BeltUpdate {
  int status;           // state after this pass
  int prevStatus;       // state before it
  bool statusPin;       // raw STATUS_PIN level, HIGH = safety switch open
  bool buttonPin;       // raw MODE_BUTTON_PIN level, LOW = pressed
  bool drivesRestored;  // a latched DAC-failure cut was cleared this pass
};

inline void beltStateBegin() {
  pinMode(STATUS_PIN, INPUT_PULLUP);      // switch connects to GND
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP); // ditto

  g_status = Status::INACTIVE;
  s_lastButtonReading = digitalRead(MODE_BUTTON_PIN);
}

inline BeltUpdate beltStateUpdate(EncoderCounter *counter) {
  BeltUpdate u;
  u.buttonPin = digitalRead(MODE_BUTTON_PIN);
  u.statusPin = digitalRead(STATUS_PIN);
  u.drivesRestored = false;
  u.prevStatus = g_status;

  // Debounce the raw pin: a reading only becomes the button level once it has
  // held still for DEBOUNCE_DELAY_MS.
  if (u.buttonPin != s_lastButtonReading) {
    s_lastButtonReading = u.buttonPin;
    s_lastDebounceTime = millis();
  } else if ((millis() - s_lastDebounceTime) > DEBOUNCE_DELAY_MS) {
    s_buttonDown = (u.buttonPin == LOW);
  }

  if (u.statusPin) {
    g_status = Status::INACTIVE;

    // The deliberate re-arm for a DAC_FAIL_DISABLE_RUN cut. The safety switch
    // cuts the relay input supply in hardware while INACTIVE, so restoring the
    // level here energises nothing on its own. If the bus is still dead,
    // writeDac() cuts the drives again a second after the next arming.
    if (g_drivesCut) {
      digitalWrite(ENABLE_PIN_1, LOW);
      digitalWrite(ENABLE_PIN_2, LOW);
      g_drivesCut = false;
      u.drivesRestored = true;
    }
  }
  else if (g_status == Status::INACTIVE) {
    g_status = Status::PRE_INITIALIZED;
  }

  // Held-to-preload. PRE_INITIALIZED is only left by pressing, so an untouched
  // unit stays at 0 V; releasing from INITIALIZING arms it and resets the
  // encoders, which is the zero the preload level is measured from.
  if (g_status != Status::INACTIVE) {
    if (s_buttonDown) {
      if (g_status != Status::INITIALIZING) {
        g_preloadLevel = PRELOAD_DAC_VALUE; // set the level before arming the
        g_status = Status::INITIALIZING;    // state, or a tick lands in between
      }                                     // and blanks the output for 167 us
    } else if (g_status == Status::INITIALIZING) {
      g_preloadLevel = 0;
      counter->reset();
      g_status = Status::INITIALIZED;
    }
  }

  u.status = g_status;
  return u;
}

#endif
