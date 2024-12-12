#include <Arduino.h>
#include <lineFollow.h>
#include <MotorControl.h>



int calculatePosition(int sensors[]) {
  int sum = 0;
  int weightedSum = 0;

  sum =sensors[0] +  sensors[1] + sensors[2] + sensors[3] + sensors[4] + sensors[5] + sensors[6] + sensors[7] + sensors[8] + sensors[9] + sensors[10] + sensors[11] ;
  weightedSum = (13 * sensors[0]  + 11 * sensors[1] + 9 * sensors[2] + 5 * sensors[3] + 3 * sensors[4] + sensors[5]) - 
                (13 * sensors[11] + 11 * sensors[10] + 9 * sensors[9] + 5 * sensors[8] + 3* sensors[7] + sensors[6]);

  if (sum == 0) {
    return 0; // Default to last known position if no line is detected
  } else {
    return weightedSum / sum * 2;
  }
}

void reverse(int time) {
  motor.setMotorSpeed(-80, -80);
  delay(time);
}

bool allSensorsDetectBlack(int sen[]) {
  int sensorThresholds = 750;
  for (int i = 0; i < 11; i++) {
    if (sen[i] <= sensorThresholds) {
      return false; // Return false if any sensor detects below the threshold
    }
  }
  return true; // Return true if all sensors detect values above the threshold
}

void processLineFollowing(int sen[]) {
  int position = calculatePosition(sen);
  int error = position; // Target position is 0 (center)

  int derivative = error - lastError;

  int correction = Kp * error + Kd * derivative;

  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  motor.setMotorSpeed(leftSpeed, rightSpeed);


  lastError = error;
}
