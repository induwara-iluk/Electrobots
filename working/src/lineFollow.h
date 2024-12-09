#ifndef LINEFOLLOW_H
#define LINEFOLLOW_H

#include <Arduino.h>
#include <MotorControl.h> // Include the MotorControl library

// Function declarations
int calculatePosition(int sensors[]);
bool allSensorsDetectBlack(int sensors[]);
void processLineFollowing(int sen[]);
void reverse(int time);

// Global variables
extern float Kp;            // Proportional gain for PID control
extern float Kd;            // Derivative gain for PID control
extern int baseSpeed;       // Base speed for motors
extern int lastError;       // Last error for derivative calculation
extern MotorControl motor; // Global pointer to the MotorControl object

#endif
