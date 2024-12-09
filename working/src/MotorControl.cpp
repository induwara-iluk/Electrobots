#include "MotorControl.h"

MotorControl::MotorControl(uint8_t PWML, uint8_t FL, uint8_t B_L, uint8_t PWMR, uint8_t FR, uint8_t B_R) {
  _PWML = PWML;
  _FL = FL;
  _B_L = B_L;
  _PWMR = PWMR;
  _FR = FR;
  _B_R = B_R;

  // Set pin modes
  pinMode(_PWML, OUTPUT);
  pinMode(_FL, OUTPUT);
  pinMode(_B_L, OUTPUT);
  pinMode(_PWMR, OUTPUT);
  pinMode(_FR, OUTPUT);
  pinMode(_B_R, OUTPUT);
}

void MotorControl::setMotorSpeed(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Left motor control
  if (leftSpeed < 0) {
    analogWrite(_PWML, -leftSpeed);
    digitalWrite(_FL, LOW);
    digitalWrite(_B_L, HIGH);
  } else {
    analogWrite(_PWML, leftSpeed);
    digitalWrite(_FL, HIGH);
    digitalWrite(_B_L, LOW);
  }

  // Right motor control
  if (rightSpeed < 0) {
    analogWrite(_PWMR, -rightSpeed);
    digitalWrite(_FR, LOW);
    digitalWrite(_B_R, HIGH);
  } else {
    analogWrite(_PWMR, rightSpeed);
    digitalWrite(_FR, HIGH);
    digitalWrite(_B_R, LOW);
  }
}

void MotorControl::stopRobot() {
  analogWrite(_PWML, 0);
  analogWrite(_PWMR, 0);

  digitalWrite(_FL, LOW);
  digitalWrite(_B_L, LOW);

  digitalWrite(_FR, LOW);
  digitalWrite(_B_R, LOW);
}
