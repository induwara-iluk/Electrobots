// Motor control pins
const int pwmL = 5;    // Left motor PWM pin
const int pwmR = 9;    // Right motor PWM pin
#define in1 7 // L298n Motor Driver pins.
#define in2 6
#define in3 10
#define in4 11
#define LED 13

// PID constants
float Kp = 0.5;        // Proportional constant
float Ki = 0.0;        // Integral constant
float Kd = 0.1;        // Derivative constant

int Speed = 150;       // Base speed of the motors
int lastError = 0;     // Variable to store the previous error value
int integral = 0;      // Variable to store the integral of error

void setup() {
  pinMode(A0, INPUT);    // Leftmost sensor
  pinMode(A1, INPUT);    // Left sensor
  pinMode(A2, INPUT);    // Slightly left sensor
  pinMode(A3, INPUT);    // Slightly right sensor
  pinMode(A4, INPUT);    // Right sensor
  pinMode(A5, INPUT);    // Rightmost sensor
  pinMode(pwmL, OUTPUT); // Left motor
  pinMode(pwmR, OUTPUT); // Right motor
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(LED, OUTPUT);  // Set the LED pin.

  Serial.begin(9600);    // Initialize Serial Monitor
}

void loop() {
  int l3 = analogRead(A0);
  int l2 = analogRead(A1);
  int l1 = analogRead(A2);
  int r1 = analogRead(A3);
  int r2 = analogRead(A4);
  int r3 = analogRead(A5);

  // Calculate the error value
  int error = (l3 * -3) + (l2 * -2) + (l1 * -1) + (r1 * 1) + (r2 * 2) + (r3 * 3);

  // Normalize the error value based on sensor values
  int totalValue = l3 + l2 + l1 + r1 + r2 + r3;

  if (totalValue != 0) {
    error /= totalValue;
  }

  // Calculate PID values
  integral += error;
  int derivative = error - lastError;
  int correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Adjust motor speeds
  int motorSpeedL = Speed - correction;
  int motorSpeedR = Speed + correction;

  // Constrain motor speeds to valid range
  motorSpeedL = constrain(motorSpeedL, 0, 255);
  motorSpeedR = constrain(motorSpeedR, 0, 255);

  // Apply motor speeds
  analogWrite(pwmL, motorSpeedL);
  analogWrite(pwmR, motorSpeedR);

  // Set motor directions (forward)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Update lastError for the next loop iteration
  lastError = error;

  // Optional: Print debug information
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" | Correction: ");
  Serial.print(correction);
  Serial.print(" | Motor L: ");
  Serial.print(motorSpeedL);
  Serial.print(" | Motor R: ");
  Serial.println(motorSpeedR);

  delay(50); // Delay for stability
}

void forward(){
  analogWrite(pwmL, 150);
  analogWrite(pwmR, 150);

  // Set motor directions (forward)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

