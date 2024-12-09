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
  gripperServo.write(gripperClosed);
  delay(500); // Wait for the servo to move
}

// Function to release the box
void ServoController::releaseBox() {
  gripperServo.write(gripperOpen);
  delay(500); // Wait for the servo to move
}

// Function to lift the box
void ServoController::liftBox() {
  armServo.write(armUp);
  delay(500); // Wait for the servo to move
}

// Function to lower the arm
void ServoController::lowerArm() {
  armServo.write(armDown);
  delay(500); // Wait for the servo to move
}
