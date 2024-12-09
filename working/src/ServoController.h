#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Servo.h>

class ServoController {
  private:
    Servo gripperServo;  // Servo for the gripper
    Servo armServo;      // Servo for the arm
    int gripperOpen;     // Position for gripper to release the box
    int gripperClosed;   // Position for gripper to grab the box
    int armDown;         // Position for arm to lower
    int armUp;           // Position for arm to lift

  public:
    ServoController(int gripperOpenAngle = 0, int gripperClosedAngle = 140, 
                    int armDownAngle = 0, int armUpAngle =20);

    void attachGripper(int pin);
    void attachArm(int pin);
    void grabBox();
    void releaseBox();
    void liftBox();
    void lowerArm();
};

#endif
