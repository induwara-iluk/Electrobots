const int B_L = 6;
const int B_R = 10;

const int FL = 7;
const int FR = 9;

const int PWMR = 11;
const int PWML = 5;

int lastError = 0;
int integral = 0;

float Kp = 20;  // Proportional gain
float Ki = 2.01;  // Integral gain
float Kd = 0.0;   // Derivative gain

const int sensorThresholds = 100;  // Threshold to distinguish black and white

void setup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  pinMode(B_L, OUTPUT);
  pinMode(FL, OUTPUT);

  pinMode(B_R, OUTPUT);
  pinMode(FR, OUTPUT);

  pinMode(PWMR, OUTPUT);
  pinMode(PWML, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  int sensors[6];
  sensors[0] = analogRead(A0);
  sensors[1] = analogRead(A1);
  sensors[2] = analogRead(A2);
  sensors[3] = analogRead(A3);
  sensors[4] = analogRead(A4);
  sensors[5] = analogRead(A5);

  int sen[6];
  for(int i = 0; i < 6; i++) {
    sen[i] = (sensors[i] < sensorThresholds) ? 1 : 0;
  }

  if (allSensorsDetectBlack(sensors)) {
    stopRobot();  // Stop the robot immediately
    return;
  }

  int position = calculatePosition(sen);
  int error = position;  // Target position is 0 (center)
  
  integral += error;
  int derivative = error - lastError;

  int correction = Kp * error;
  
  int baseSpeed = 100;  // Base speed for the motors
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  setMotorSpeed(leftSpeed, rightSpeed);

  lastError = error;

  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" , Correction: ");
  Serial.print(correction);
  Serial.print(" , Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" , Right Speed: ");
  Serial.println(rightSpeed);

  delay(100);
}

int calculatePosition(int sensors[]) {
  // Calculate a weighted average of sensor readings
  int sum = 0;
  int weightedSum = 0;
  
  sum = sensors[0] + sensors[1] + sensors[2] + sensors[3] + sensors[4] + sensors[5];
  weightedSum = 9 * sensors[0] + 4 * sensors[1] + sensors[2] - (9 * sensors[5] + 4 * sensors[4] + sensors[3]);

  Serial.print("Weighted Sum: ");
  Serial.print(weightedSum);
  

  if (sum == 0) {
    return 0 ; // Default to last known position if no line is detected
  } else {
    return weightedSum/sum ;
  }
}

bool allSensorsDetectBlack(int sensors[]) {
  for (int i = 0; i < 6; i++) {
    if (sensors[i] < sensorThresholds) {
      return false;
    }
  }
  return true;
}

void stopRobot() {
  analogWrite(PWML, 0);
  analogWrite(PWMR, 0);

  digitalWrite(FL, LOW);
  digitalWrite(B_L, LOW);

  digitalWrite(FR, LOW);
  digitalWrite(B_R, LOW);

  Serial.println("Robot stopped due to all sensors detecting black.");
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  analogWrite(PWML, leftSpeed);
  analogWrite(PWMR, rightSpeed);

  digitalWrite(FL, HIGH);
  digitalWrite(B_L, LOW);

  digitalWrite(FR, HIGH);
  digitalWrite(B_R, LOW);
}