#include "pins.h"

void setup()
 {
  
  pinMode(LEFT_PULSE_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(LEFT_LIMIT_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENABLE_PIN, OUTPUT);

  pinMode(RIGHT_PULSE_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_LIMIT_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENABLE_PIN, OUTPUT);

  pinMode(SERVO_PIN, OUTPUT);
}
