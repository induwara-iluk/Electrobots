#include <Arduino.h>

const int B_L = 9;
const int B_R = 7;

const int FL = 10;
const int FR = 6;

const int PWMR = 5;
const int PWML = 11;

const int leftIR = A8;
const int rightIR = A9;

int lastError = 0;
int integral = 0;

float Kp = 8;   // Proportional gain
float Ki = 0.00; // Integral gain
float Kd = 3.0;  // De5ivative gain
int baseSpeed = 70;  // Base speed for the motors

int stage = 1; 
const int historySize = 10; // Size of the history buffer
int senHistory[historySize][8]; // History array to store past sensor states
int historyIndex = 0; // Current index for appending in the history array
const int sensorThresholds = 200; // Threshold to distinguish black and white

// Function Definitions
void initializePins() {
  for (int i = A0; i <= A9; i++) {
    pinMode(i, INPUT);
  }

  pinMode(B_L, OUTPUT);
  pinMode(FL, OUTPUT);
  pinMode(B_R, OUTPUT);
  pinMode(FR, OUTPUT);
  pinMode(PWMR, OUTPUT);
  pinMode(PWML, OUTPUT);
}

void readSensors(int sensors[]) {
  for (int i = 0; i < 8; i++) {
    sensors[i] = analogRead(A0 + i);
    Serial.print(sensors[i]);
    Serial.print(" ");
    
  }
  Serial.print(analogRead(A8));
  Serial.print(" ");
  Serial.print(analogRead(A9));
 Serial.println("");
}

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

void turnBend(byte direction){

  // 0 = left , 1 = right
  while (direction == 0){
    setMotorSpeed(-80,80);
    delay(200);
    if ((analogRead(A3) < sensorThresholds) && (analogRead(A4) < sensorThresholds)){
      return;
    }
  }


  while (direction == 1){
    setMotorSpeed(80,-80);
    delay(200);
    if ((analogRead(A3) < sensorThresholds) && (analogRead(A4) < sensorThresholds)){
      return;
    }
  }

}

void convertSensorsToBinary(int sensors[], int sen[]) {
  for (int i = 0; i < 8; i++) {
    sen[i] = (sensors[i] < sensorThresholds) ? 1 : 0;
  }
}

void updateSensorHistory(int sen[]) {
  for (int i = 0; i < 8; i++) {
    senHistory[historyIndex][i] = sen[i];
  }
  historyIndex = (historyIndex + 1) % historySize; // Circular buffer
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

char detectBend() {
  // Check the last few entries in history to detect a bend pattern
  for (int i = 0; i < historySize; i++) {
    // Check if the two center sensors are 1 (black) and all sensors on one side are 0 (white)
         (senHistory[i][0] == 1 && senHistory[i][1] == 1 && senHistory[i][2] == 1) {
      return 'r';  // Use single quotes for character literals
    } else if ((senHistory[i][3] == 1 && senHistory[i][4] == 1) &&  // Center sensors detecting black
               (senHistory[i][7] == 1 && senHistory[i][6] == 1 && senHistory[i][5] == 1)) {
      return 'l';  // Use single quotes for character literals
    } else if ((senHistory[i][3] == 1 && senHistory[i][4] == 1) &&  // Center sensors detecting black
               (senHistory[i][0] == 1 && senHistory[i][1] == 1 && senHistory[i][2] == 1) &&
               (senHistory[i][7] == 1 && senHistory[i][6] == 1 && senHistory[i][5] == 1)) {
      return 'b';  // Use single quotes for character literals
    }
  }
  return ' ';  // Return a default value if no pattern is detected
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

void stopRobot() {
  analogWrite(PWML, 0);
  analogWrite(PWMR, 0);

  digitalWrite(FL, LOW);
  digitalWrite(B_L, LOW);

  digitalWrite(FR, LOW);
  digitalWrite(B_R, LOW);

  
}

void rotateBendUntilLineDetected(char a) {
  if (a == 'l') {
    // Rotate right
    analogWrite(PWML, 100);
    analogWrite(PWMR, 255);

    digitalWrite(FL, LOW);
    digitalWrite(B_L, HIGH);

    digitalWrite(FR, HIGH);
    digitalWrite(B_R, LOW);

    Serial.println("Rotating right to handle detected bend.");
  } else if (a == 'r') {
    // Rotate left
    analogWrite(PWML, 255);
    analogWrite(PWMR, 100);

    digitalWrite(FL, HIGH);
    digitalWrite(B_L, LOW);

    digitalWrite(FR, LOW);
    digitalWrite(B_R, HIGH);

    Serial.println("Rotating left to handle detected bend.");
  } else {
    Serial.println("Invalid direction. Use 'r' for right or 'l' for left.");
    return;  // Exit if invalid direction
  }

  // Continue rotating until the next line is detected
  bool lineDetected = false;
  while (!lineDetected) {
    int sensors[8];
    readSensors(sensors);
    lineDetected = !allSensorsDetectBlack(sensors);
  }

  // Stop the robot once the line is detected
  stopRobot();
}

void BlueLED(bool condition) {
  if (condition) {
    digitalWrite(BlueLED, HIGH);
  } else {
    digitalWrite(BlueLED, LOW);
    
  }
  

}
void setup() {
  initializePins();
  Serial.begin(9600);
}

void loop() {
  int sensors[8];
  readSensors(sensors);

  int sen[8];
  convertSensorsToBinary(sensors, sen);
  
  // updateSensorHistory(sen); // Update sensor history

  if (allSensorsDetectBlack(sensors)) {
     stopRobot();
     return;
   }

  if (analogRead(leftIR)< sensorThresholds){
     turnBend(0);
     BlueLED(true);
     delay(500);
     BlueLED(false);
     
   }

  if (analogRead(rightIR)< sensorThresholds){
     turnBend(1);
     BlueLED(true);
     delay(500);
     BlueLED(false);
   }

  processLineFollowing(sen);
}

