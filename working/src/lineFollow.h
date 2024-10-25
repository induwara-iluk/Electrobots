// RobotControl.h

#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <Arduino.h> // Include Arduino library for types like int and functions like analogWrite

// Function declarations
void setMotorSpeed(int leftSpeed, int rightSpeed);
int calculatePosition(int sensors[]);
bool allSensorsDetectBlack(int sensors[]);
void processLineFollowing(int sen[]);

// Global variables (extern if defined in .cpp)
extern int Kp;           // Proportional gain for PID control
extern int Kd;           // Derivative gain for PID control
extern int baseSpeed;    // Base speed for motors
extern int lastError;    // Last error for derivative calculation
extern int integral;     // Integral of error for PID control
extern int sensorThresholds; // Threshold for black line detection

#endif // ROBOTCONTROL_H
