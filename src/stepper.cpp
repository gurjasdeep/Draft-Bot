#include "stepper.h"

Stepper::Stepper(
    uint8_t stepPin,
    uint8_t dirPin,
    uint8_t enablePin,
    uint16_t stepsPerRev,
    uint8_t microsteps,
    uint8_t limitPin
)
{
    _stepPin = stepPin;
    _dirPin = dirPin;
    _enablePin = enablePin;
    _limitPin = limitPin;

    _stepsPerRev = stepsPerRev;
    _microsteps = microsteps;

    _position = 0;
    _targetPosition = 0;

    _moving = false;
    _direction = true;

    _stepDelayMicros = 500;

    _lastStepTime = 0;
}


void Stepper::begin()
{
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);

    if (_enablePin != 255)
    {
        pinMode(_enablePin, OUTPUT);
    }

    if (_limitPin != 255)
    {
        pinMode(_limitPin, INPUT_PULLUP);
    }

    digitalWrite(_stepPin, LOW);
    digitalWrite(_dirPin, LOW);

    disable();
}


void Stepper::enable()
{
    if (_enablePin == 255)
    {
        return;
    }

    /*
     * TB6600 ENA polarity depends on how the
     * driver is wired/configured.
     *
     * This assumes:
     *
     * ENA+ -> Arduino output
     * ENA- -> GND
     *
     * and HIGH enables the driver.
     *
     * If your TB6600 behaves opposite, invert this.
     */

    digitalWrite(_enablePin, HIGH);
}


void Stepper::disable()
{
    if (_enablePin == 255)
    {
        return;
    }

    digitalWrite(_enablePin, LOW);
}


void Stepper::home(HomingDirection direction)
{
    if (_limitPin == 255)
    {
        return;
    }

    if (direction == HomingDirection::HOME_LEFT)
    {
        digitalWrite(_dirPin, LOW);
    }
    else
    {
        digitalWrite(_dirPin, HIGH);
    }

    enable();

    while (digitalRead(_limitPin) == HIGH)
    {
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(1000);

        digitalWrite(_stepPin, LOW);
        delayMicroseconds(1000);
    }

    disable();
    zero();
    _moving = false;
    _targetPosition = 0;
}


bool Stepper::isLimitTriggered() const
{
    if (_limitPin == 255)
    {
        return false;
    }

    return digitalRead(_limitPin) == LOW;
}


void Stepper::setPosition(long steps)
{
    _position = steps;
}


void Stepper::zero()
{
    _position = 0;
}


long Stepper::getPosition() const
{
    return _position;
}


long Stepper::angleToSteps(float angle) const
{
    float totalStepsPerRevolution =
        (float)_stepsPerRev * (float)_microsteps;

    return lround(
        angle * totalStepsPerRevolution / 360.0f
    );
}

void Stepper::setAngle(float angle)
{
    _position = angleToSteps(angle);
}

float Stepper::stepsToAngle(long steps) const
{
    float totalStepsPerRevolution =
        (float)_stepsPerRev * (float)_microsteps;

    return (float)steps * 360.0f /
           totalStepsPerRevolution;
}


float Stepper::getAngle() const
{
    return stepsToAngle(_position);
}


void Stepper::moveTo(long targetSteps)
{
    _targetPosition = targetSteps;

    if (_targetPosition == _position)
    {
        _moving = false;
        return;
    }

    _direction = (_targetPosition > _position);

    digitalWrite(
        _dirPin,
        _direction ? HIGH : LOW
    );

    enable();

    _moving = true;
}


void Stepper::moveToAngle(float angle)
{
    moveTo(angleToSteps(angle));
}


void Stepper::update()
{
    if (!_moving)
        return;

    unsigned long now = micros();

    if ((unsigned long)(now - _lastStepTime) <
        _stepDelayMicros)
    {
        return;
    }

    _lastStepTime = now;

    /*
     * Generate one STEP pulse.
     *
     * TB6600 detects the rising edge.
     */
    digitalWrite(_stepPin, HIGH);

    delayMicroseconds(5);

    digitalWrite(_stepPin, LOW);

    // Update our software position.
    if (_direction)
        _position++;
    else
        _position--;

    // Check whether target was reached.
    if (_position == _targetPosition)
    {
        _moving = false;
    }
}


bool Stepper::isMoving() const
{
    return _moving;
}


void Stepper::setStepDelay(unsigned int microseconds)
{
    _stepDelayMicros = microseconds;
}
