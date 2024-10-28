#include <Arduino.h>
#include <lineFollow.h>


int calculatePosition(int sensors[]) {
  int sum = 0;
  int weightedSum = 0;
  
  sum = sensors[0] + sensors[1] + sensors[2] + sensors[3] + sensors[4] + sensors[5] + sensors[6] + sensors[7];
  weightedSum = (7 * sensors[0] + 5 * sensors[1] + 3 * sensors[2] + sensors[3]) - 
                (7 * sensors[7] + 5 * sensors[6] + 3 * sensors[5] + sensors[4]);

  

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
  
  
  int derivative = error - lastError;

  int correction = Kp * error + Kd * derivative;
  

  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  setMotorSpeed(leftSpeed, rightSpeed);
  lastError = error;
}