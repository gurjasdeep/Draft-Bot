#include "pins.h"
#include "home.h"
#include <Servo.h>

// ======================================================
// LIMIT SWITCH CLASS
// ======================================================

class LimitSwitch
{
private:
  int pin;

public:

  LimitSwitch(int inputPin)
  {
    pin = inputPin;
  }

  void begin()
  {
    pinMode(pin, INPUT_PULLUP);
  }

  bool isTriggered()
  {
    return digitalRead(pin) == HIGH;
  }

  void printStatus(const char* name)
  {
    Serial.print(name);
    Serial.print(F(": "));

    if (isTriggered())
    {
      Serial.println(F("TRIGGERED"));
    }
    else
    {
      Serial.println(F("NORMAL"));
    }
  }
};


// ======================================================
// STEPPER MOTOR CLASS
// ======================================================

class Stepper
{
private:
  int stepPin;
  int dirPin;
  int pulsePin;

public:

  Stepperjoint(int stepPinValue,int dirPinValue, int pulsepin)
  {
    stepPin = stepPinValue;
    dirPin = dirPinValue;
    pulsePin=pulsepin;
  }

  void begin()
  {
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

    digitalWrite(stepPin, LOW);
    digitalWrite(dirPin, LOW);
  }

  void setDirection(bool direction)
  {
    digitalWrite(dirPin, direction);

    delayMicroseconds(20);
  }

  void stepOnce()
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5);

    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }

  void moveSteps(long steps, bool direction)
  {
    setDirection(direction);

    for (long i = 0; i < steps; i++)
    {
      stepOnce();
    }
  }

  void test(long steps)
  {
    Serial.println(F("Forward"));

    moveSteps(steps, HIGH);

    delay(500);

    Serial.println(F("Reverse"));

    moveSteps(steps, LOW);
  }
};


// ======================================================
// PEN SERVO CLASS
// ======================================================

class PenServo
{
private:
  Servo servo;

  int pin;

  int upAngle;
  int downAngle;

public:

  PenServo( int servoPin,int penUpAngle,int penDownAngle)
  {
    pin = servoPin;
    upAngle = penUpAngle;
    downAngle = penDownAngle;
  }

  void begin()
  {
    servo.attach(pin);
    penUp();
  }

  void penUp()
  {
    servo.write(upAngle);
  }

  void penDown()
  {
    servo.write(downAngle);
  }
  void test()
  {
    Serial.println(F("Pen UP"));

    penUp();
    delay(700);

    Serial.println(F("Pen DOWN"));

    penDown();
    delay(700);

    Serial.println(F("Pen UP"));

    penUp();
  }
};



const long TEST_STEPS = 400;

const int STEP_DELAY_US = 1000;

const int PEN_UP_ANGLE = 0;
const int PEN_DOWN_ANGLE = 30;

// Motors
Stepper::Stepperjoint motor1(LEFT_DIR_PIN,LEFT_ENABLE_PIN,LEFT_PULSE_PIN);
Stepper::Stepperjoint motor2(RIGHT_DIR_PIN,RIGHT_ENABLE_PIN,RIGHT_PULSE_PIN);



// Limit switches
LimitSwitch limit1(LEFT_LIMIT_PIN);
LimitSwitch limit2(RIGHT_LIMIT_PIN);


// Pen servo
PenServo pen(SERVO_PIN,PEN_UP_ANGLE,PEN_DOWN_ANGLE);


// ======================================================
// BUTTON FUNCTIONS
// ======================================================

bool buttonPressed()
{
  return digitalRead(BUTTON_PIN) == LOW;
}


void printButtonStatus()
{
  Serial.print(F("Push Button: "));

  if (buttonPressed())
  {
    Serial.println(F("PRESSED"));
  }
  else
  {
    Serial.println(F("RELEASED"));
  }
}


// ======================================================
// LIMIT SWITCH TEST
// ======================================================

void testLimitSwitches()
{
  Serial.println();
  Serial.println(F("LIMIT SWITCHES STATUS"));

  limit1.printStatus("Limit 1");
  limit2.printStatus("Limit 2");
  limit3.printStatus("Limit 3");
}


// ======================================================
// ALL INPUTS TEST
// ======================================================

void printAllInputs()
{
  Serial.println(F("INPUT STATUS "));

  limit1.printStatus("Limit 1");
  limit2.printStatus("Limit 2");
  limit3.printStatus("Limit 3");

  printButtonStatus();
}


// MOTOR TEST FUNCTIONS

void testMotor1()
{
  Serial.println();
  Serial.println(F("MOTOR 1 TEST"));

  motor1.test(TEST_STEPS);

  Serial.println(F("Motor 1 test complete"));
}


void testMotor2()
{
  Serial.println();
  Serial.println(F("MOTOR 2 TEST"));

  motor2.test(TEST_STEPS);

  Serial.println(F("Motor 2 test complete"));
}


void testBothMotors()
{
  testMotor1();

  delay(500);

  testMotor2();
}


// SERVO TEST


void testPenServo()
{
  Serial.println();
  Serial.println(F("===== PEN SERVO TEST ====="));

  pen.test();

  Serial.println(F("Servo test complete"));
}


// MENU

void printMenu()
{
  Serial.println();
  Serial.println(F(" DRAFT BOT COMPONENT TESTER"));
  

  Serial.println(F("1 - Test Motor 1"));
  Serial.println(F("2 - Test Motor 2"));
  Serial.println(F("3 - Test Limit Switches"));
  Serial.println(F("4 - Test Pen Servo"));
  Serial.println(F("5 - Test Push Button"));
  Serial.println(F("6 - Test Both Motors"));
  Serial.println(F("7 - Show All Inputs"));
  Serial.println(F("h - HOME"));
  Serial.println(F("m - Show Menu"));

  Serial.println();
}


// COMMAND HANDLER

void handleCommand(char command)
{
  switch (command)
  {
    case '1':

      testMotor1();

      break;


    case '2':

      testMotor2();

      break;


    case '3':

      testLimitSwitches();

      break;


    case '4':

      testPenServo();

      break;


    case '5':

      printButtonStatus();

      break;


    case '6':

      testBothMotors();

      break;


    case '7':

      printAllInputs();

      break;

    case 'h':
    case 'H':

      Serial.println(F("Homing Motors"));

      homeMotor(LEFT_PULSE_PIN,LEFT_DIR_PIN,LEFT_LIMIT_PIN,LOW);
      homeMotor(RIGHT_PULSE_PIN,RIGHT_DIR_PIN,RIGHT_LIMIT_PIN,HIGH);

      Serial.println(F("Motors Homed"));

      break;


    case 'm':
    case 'M':

      printMenu();

      break;


    default:

      Serial.println(F("Unknown command"));

      break;
  }
}


// ======================================================
// SETUP
// ======================================================

void setup()
{
  Serial.begin(115200);


  // Start motors
  motor1.begin();
  motor2.begin();


  // Start limit switches
  limit1.begin();
  limit2.begin();
  limit3.begin();


  // Push button
  pinMode(BUTTON_PIN, INPUT_PULLUP);


  // Start pen servo
  pen.begin();


  delay(500);


  Serial.println(F("System ready"));

  printMenu();
}


// ======================================================
// MAIN LOOP
// ======================================================

void loop()
{
  if (Serial.available() > 0)
  {
    char command = Serial.read();


    // Ignore newline characters
    if (command == '\n' || command == '\r')
    {
      return;
    }


    handleCommand(command);
  }
}