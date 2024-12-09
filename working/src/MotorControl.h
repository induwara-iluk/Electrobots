#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>

class MotorControl {
public:
  MotorControl(uint8_t PWML, uint8_t FL, uint8_t B_L, uint8_t PWMR, uint8_t FR, uint8_t B_R);
  void setMotorSpeed(int leftSpeed, int rightSpeed);
  void stopRobot(); // Declaration for the stopRobot function

private:
  uint8_t _PWML, _FL, _B_L;
  uint8_t _PWMR, _FR, _B_R;
};

#endif
