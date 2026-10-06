#include "pins.h"
#include "stepper.h"
#include <Arduino.h>
#define HOMING_STEP_DELAY_US 1000

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
    Stepper motor(
        stepPin,
        dirPin,
        255,
        200,
        1,
        limitPin
    );

    motor.begin();
    motor.home(direction);
    motor.setStepDelay(HOMING_STEP_DELAY_US);
}


// ------------------------------------------------------------
// Home both motors
// ------------------------------------------------------------

void homeMotors()
{
    Stepper leftMotor(
        LEFT_PULSE_PIN,
        LEFT_DIR_PIN,
        LEFT_ENABLE_PIN,
        200,
        1,
        LEFT_LIMIT_PIN
    );

    Stepper rightMotor(
        RIGHT_PULSE_PIN,
        RIGHT_DIR_PIN,
        RIGHT_ENABLE_PIN,
        200,
        1,
        RIGHT_LIMIT_PIN
    );

    leftMotor.begin();
    rightMotor.begin();

    leftMotor.home(HomingDirection::HOME_LEFT);
    rightMotor.home(HomingDirection::HOME_RIGHT);
}
