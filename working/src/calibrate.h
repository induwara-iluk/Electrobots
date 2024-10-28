#ifndef CALIBRATE_H
#define CALIBRATE_H

#include <Arduino.h>

extern void setMotorSpeed(int speedLeft, int speedRight);  // Declare the external motor speed function

int calibrate(int sensors[8]);  // Declare the calibrate function

#endif
