#include "pins.h"
#include "home.h"
#include <Arduino.h>
// -----------------------------
// SETTINGS
// -----------------------------

#define HOME_ACTIVE HIGH

const int STEP_DELAY = 1500;

// Change these if motors move the wrong way
const bool LEFT_HOME_DIR = LOW;
const bool RIGHT_HOME_DIR = HIGH;


// -----------------------------
// STEP ONE TIME
// -----------------------------

void stepMotor(int stepPin)
{
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY);
}


// -----------------------------
// HOME ONE MOTOR
// -----------------------------

void homeMotor(int stepPin,int dirPin,int limitPin,bool homeDirection)
{
  digitalWrite(dirPin, homeDirection);

  while (digitalRead(limitPin) != HOME_ACTIVE)// Move until switch is triggered
  {
    stepMotor(stepPin);
  }
}


// -----------------------------
// SETUP
// -----------------------------

void setup()
{
  Serial.begin(115200);

  pinMode(LEFT_PULSE_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);

  pinMode(RIGHT_PULSE_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  pinMode(LEFT_LIMIT_PIN, INPUT_PULLUP);
  pinMode(RIGHT_LIMIT_PIN, INPUT_PULLUP);


  Serial.println("Homing Motor 1");
  homeMotor(LEFT_PULSE_PIN,LEFT_DIR_PIN,LEFT_LIMIT_PIN,LEFT_HOME_DIR);
  Serial.println("Motor 1 Homed");

  delay(500);


  Serial.println("Homing Motor 2");
  homeMotor( RIGHT_PULSE_PIN, RIGHT_DIR_PIN, RIGHT_LIMIT_PIN, RIGHT_HOME_DIR);
  Serial.println("Motor 2 Homed");
  Serial.println("HOMING COMPLETE");
}
