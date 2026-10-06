#pragma once

#include <Arduino.h>

enum class HomingDirection
{
    HOME_LEFT,
    HOME_RIGHT
};

inline float radiansToDegrees(float radians)
{
    constexpr float PI_ = 3.14159265358979323846f;
    return radians * 180.0f / PI_;
}

class Stepper
{
public:
    Stepper(
        uint8_t stepPin,
        uint8_t dirPin,
        uint8_t enablePin,
        uint16_t stepsPerRev,
        uint8_t microsteps,
        uint8_t limitPin = 255
    );

    void begin();

    // TB6600 enable control
    void enable();
    void disable();

    // Homing
    void home(HomingDirection direction = HomingDirection::HOME_LEFT);
    bool isLimitTriggered() const;

    // Position
    void setPosition(long steps);
    void zero();
    long getPosition() const;

    // Angle conversion
    long angleToSteps(float angle) const;
    float stepsToAngle(long steps) const;

    float getAngle() const;
    // Set the current position to a specific angle without moving.
    void setAngle(float angle);

    // Set a new absolute target
    void moveTo(long targetSteps);
    void moveToAngle(float angle);

    // Must be called repeatedly from loop()
    void update();

    // Check whether the motor has reached its target
    bool isMoving() const;

    // Speed
    void setStepDelay(unsigned int microseconds);

private:
    uint8_t _stepPin;
    uint8_t _dirPin;
    uint8_t _enablePin;
    uint8_t _limitPin;

    uint16_t _stepsPerRev;
    uint8_t _microsteps;

    long _position;
    long _targetPosition;

    bool _moving;
    bool _direction;

    unsigned int _stepDelayMicros;

    unsigned long _lastStepTime;
};
