#include "stepper.h"

unsigned int STEPPER_DELAY_MICROSECONDS = 800;

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
}


void Stepper::begin(LimitPolarity limitPolarity)
{
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);

    if (_enablePin != 255)
    {
        pinMode(_enablePin, OUTPUT);
    }

    if (_limitPin != 255)
    {
        pinMode(
            _limitPin,
            limitPolarity == LimitPolarity::ACTIVE_LOW
                ? INPUT_PULLUP
                : INPUT
        );
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
     * and LOW enables the driver.
     *
     * If your TB6600 behaves opposite, invert this.
     */

    digitalWrite(_enablePin, LOW);
}


void Stepper::disable()
{
    if (_enablePin == 255)
    {
        return;
    }

    digitalWrite(_enablePin, HIGH);
}


HomingResult Stepper::home(
    HomingDirection direction,
    LimitPolarity limitPolarity,
    unsigned long timeoutMs
)
{
    if (_limitPin == 255)
    {
        return HomingResult::LIMIT_NOT_CONFIGURED;
    }

    const uint8_t homeDirection =
        direction == HomingDirection::HOME_LEFT ? LOW : HIGH;
    const unsigned long startedAt = millis();

    enable();
    digitalWrite(_dirPin, homeDirection);

    while (!isLimitTriggered(limitPolarity))
    {
        if (millis() - startedAt >= timeoutMs)
        {
            disable();
            _moving = false;
            _targetPosition = _position;
            return HomingResult::TIMEOUT;
        }

        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(STEPPER_DELAY_MICROSECONDS);

        digitalWrite(_stepPin, LOW);
        delayMicroseconds(STEPPER_DELAY_MICROSECONDS);
    }

    disable();
    zero();
    _moving = false;
    _targetPosition = 0;
    return HomingResult::SUCCESS;
}


bool Stepper::isLimitTriggered(LimitPolarity limitPolarity) const
{
    if (_limitPin == 255)
    {
        return false;
    }

    const int activeLevel =
        limitPolarity == LimitPolarity::ACTIVE_LOW ? LOW : HIGH;
    return digitalRead(_limitPin) == activeLevel;
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
    {
        return;
    }

    digitalWrite(_stepPin, HIGH);
    delayMicroseconds(STEPPER_DELAY_MICROSECONDS);
    digitalWrite(_stepPin, LOW);
    delayMicroseconds(STEPPER_DELAY_MICROSECONDS);

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
    STEPPER_DELAY_MICROSECONDS = microseconds;
}
