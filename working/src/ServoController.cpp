#include <ServoController.h>
#include <Arduino.h>

// Constructor to initialize the servo angles
ServoController::ServoController(int gripperOpenAngle, int gripperClosedAngle, 
                                 int armDownAngle, int armUpAngle) {
  gripperOpen = gripperOpenAngle;
  gripperClosed = gripperClosedAngle;
  armDown = armDownAngle;
  armUp = armUpAngle;
}

// Attach the gripper servo to a pin
void ServoController::attachGripper(int pin) {
  gripperServo.attach(pin);
  gripperServo.write(gripperOpen); // Set to initial position
}

// Attach the arm servo to a pin
void ServoController::attachArm(int pin) {
  armServo.attach(pin);
  armServo.write(armDown); // Set to initial position
}

// Function to grab the box
void ServoController::grabBox() {
  for (int pos = gripperServo.read(); pos <= gripperClosed; pos++) {
    gripperServo.write(pos);
    delay(15); // Adjust the delay as needed for smooth movement
  }
  delay(500); // Wait for the servo to move
}

// Function to release the box
void ServoController::releaseBox() {
  for (int pos = gripperServo.read(); pos >= gripperOpen; pos--) {
    gripperServo.write(pos);
    delay(15); // Adjust the delay as needed for smooth movement
  }
  delay(500); // Wait for the servo to move
}

// Function to lift the box
void ServoController::liftBox() {
  for (int pos = armServo.read(); pos <= armUp; pos++) {
    armServo.write(pos);
    delay(15); // Adjust the delay as needed for smooth movement
  }
  delay(500); // Wait for the servo to move
}

// Function to lower the arm
void ServoController::lowerArm() {
  for (int pos = armServo.read(); pos >= armDown; pos--) {
    armServo.write(pos);
    delay(15); // Adjust the delay as needed for smooth movement
  }
  delay(500); // Wait for the servo to move
}