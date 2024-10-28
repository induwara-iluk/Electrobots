
#ifndef LINEFOLLOW_H
#define LINEFOLLOW_H

#include <Arduino.h> // Include Arduino library for types like int and functions like analogWrite

// Function declarations
void setMotorSpeed(int leftSpeed, int rightSpeed);
int calculatePosition(int sensors[]);
bool allSensorsDetectBlack(int sensors[]);
void processLineFollowing(int sen[]);

// Global variables (extern if defined in .cpp)
extern float Kp;           // Proportional gain for PID control
extern float Kd;           // Derivative gain for PID control
extern int baseSpeed;    // Base speed for motors
extern int lastError;    // Last error for derivative calculation

extern int sensorThresholds; // Threshold for black line detection

#endif 
