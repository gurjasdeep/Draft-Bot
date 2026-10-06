#include "pins.h"
#include <Arduino.h>
// ------------------------------------------------------------
// HOMING SETTINGS
// ------------------------------------------------------------

#define HOMING_STEP_DELAY_US 1000

enum HomingDirection

{

    HOME_LEFT,

    HOME_RIGHT

};

// ------------------------------------------------------------
// Home one stepper
// ------------------------------------------------------------

void homeStepper(
    int stepPin,
    int dirPin,
    int limitPin,
    HomingDirection direction
)
{
    // Configure pins
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(limitPin, INPUT_PULLUP);

    // Set the direction in which the motor should move.
    //
    // Change HIGH/LOW here if your motor moves in the
    // opposite direction from what you expect.
    if (direction == HOME_LEFT)
    {
        digitalWrite(dirPin, LOW);
    }
    else
    {
        digitalWrite(dirPin, HIGH);
    }

    // Keep moving until the limit switch is pressed.
    //
    // INPUT_PULLUP means:
    //   HIGH = switch is NOT pressed
    //   LOW  = switch IS pressed
    while (digitalRead(limitPin) == HIGH)
    {
        // STEP pulse
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(HOMING_STEP_DELAY_US);

        digitalWrite(stepPin, LOW);
        delayMicroseconds(HOMING_STEP_DELAY_US);
    }

    // The motor has reached the mechanical limit.
    // Stop sending step pulses.
}


// ------------------------------------------------------------
// Home both motors
// ------------------------------------------------------------

void homeMotors()
{
    // LEFT motor goes to extreme LEFT
    homeStepper(
        LEFT_STEP_PIN,
        LEFT_DIR_PIN,
        LEFT_LIMIT_PIN,
        HOME_LEFT
    );

    // RIGHT motor goes to extreme RIGHT
    homeStepper(
        RIGHT_STEP_PIN,
        RIGHT_DIR_PIN,
        RIGHT_LIMIT_PIN,
        HOME_RIGHT
    );
}
