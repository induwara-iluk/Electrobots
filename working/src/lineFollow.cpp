#include <Arduino.h>
#include <lineFollow.h>
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Left motor control
  if (leftSpeed < 0) {
    analogWrite(PWML, -leftSpeed);  // Convert negative speed to positive for PWM
    digitalWrite(FL, LOW);          // Set direction to backward
    digitalWrite(B_L, HIGH);        // Set backward pin HIGH
  } else {
    analogWrite(PWML, leftSpeed);   // Apply speed for forward motion
    digitalWrite(FL, HIGH);         // Set direction to forward
    digitalWrite(B_L, LOW);         // Set backward pin LOW
  }
  
  // Right motor control
  if (rightSpeed < 0) {
    analogWrite(PWMR, -rightSpeed); // Convert negative speed to positive for PWM
    digitalWrite(FR, LOW);          // Set direction to backward
    digitalWrite(B_R, HIGH);        // Set backward pin HIGH
  } else {
    analogWrite(PWMR, rightSpeed);  // Apply speed for forward motion
    digitalWrite(FR, HIGH);         // Set direction to forward
    digitalWrite(B_R, LOW);         // Set backward pin LOW
  }
}
int calculatePosition(int sensors[]) {
  int sum = 0;
  int weightedSum = 0;
  
  sum = sensors[0] + sensors[1] + sensors[2] + sensors[3] + sensors[4] + sensors[5] + sensors[6] + sensors[7];
  weightedSum = (10 * sensors[0] + 7 * sensors[1] + 4 * sensors[2] + sensors[3]) - 
                (10 * sensors[7] + 7 * sensors[6] + 4 * sensors[5] + sensors[4]);

  

  if (sum == 0) {
    return 0; // Default to last known position if no line is detected
  } else {
    
    return weightedSum / sum*2;
  }
}

bool allSensorsDetectBlack(int sensors[]) {
  for (int i = 0; i < 8; i++) {
    if (sensors[i] < sensorThresholds) {
      return false; // Return false if any sensor detects below the threshold
    }
  }
  return true; // Return true if all sensors detect values above the threshold
}

void processLineFollowing(int sen[]) {
  int position = calculatePosition(sen);
  int error = position;  // Target position is 0 (center)
  
  integral += error;
  int derivative = error - lastError;

  int correction = Kp * error + Kd * derivative;
  

  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  setMotorSpeed(leftSpeed, rightSpeed);
  lastError = error;
}