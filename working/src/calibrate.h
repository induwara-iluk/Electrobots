#ifndef CALIBRATE_H
#define CALIBRATE_H

#include <Arduino.h>

extern void setMotorSpeed(int speedLeft, int speedRight);  // Declare the external motor speed function

void calibrate( int threshold[12],int mode);  // Declare the calibrate function

#endif
