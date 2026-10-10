#ifndef home_h
#define home_h

void stepMotor(int stepPin);
void homeMotor(int stepPin, int dirPin, int limitPin, bool homeDirection);

#endif // home_h