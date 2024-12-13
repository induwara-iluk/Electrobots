#include <Arduino.h>
#include <calibrate.h>
#include <lineFollow.h>
#include <oled.h>
#include <TCSColorSensor.h>
#include <MotorControl.h>
#include <readIR.h>
#include <Arduino.h>
#include <ServoController.h>
#include <Wire.h>
#include "VL53L0X_MultiSensor.h"

int xshutPins[] = {49, 47, 45, 43};
uint8_t i2cAddresses[] = {0x30, 0x31, 0x32, 0x33};

VL53L0X_MultiSensor multiSensor(xshutPins, i2cAddresses, 4);

ServoController boxHandler;

int stage = 3;

TCSColorSensor colorSensor;
oled oledDisplay;
readIR irReader;

const int B_R = 10;
const int FR = 9;
const int PWMR = 8;

const int B_L = 11;
const int FL = 12;
const int PWML = 13;

byte decoded_num = 0;

MotorControl motor(PWML, FL, B_L, PWMR, FR, B_R);

int sensors[12];
int binarySensors[12];

const int btn1 = 18;
const int btn2 = 19;

const int leftIR = A4;
const int rightIR = A15;
const int frontIR = A10;

int lastError = 0;
int integral = 0;

float Kp = 4.0;
float Ki = 0.0;
float Kd = 1.5;

// for straght line
float p_gain = 1.2;  // Proportional gain
float i_gain = 0.05; // Integral gain
float d_gain = 0.1;  // Derivative gain

// PID variables
float pid_error = 0;
float pid_previous_error = 0;
float pid_integral = 0;
float pid_derivative = 0;

int baseSpeed = 60; // Base speed for the motors

const int historySize = 3500; // Size of the history buffer
int senHistory[historySize];  // History array to store past sensor states
int historyIndex = 0;         // Current index for appending in the history array

const int R_encoder_A = 2;

const int L_encoder_A = 3;

volatile long R_encoder_ticks = 0;
volatile long L_encoder_ticks = 0;

const float wheel_diameter = 0.067; // Diameter of the wheels in meters
int pulses_per_revolution = 222;

// Variables for speed calculation
long previous_ticks_R = 0;
long previous_ticks_L = 0;

String irValues_str = "";

int sensorThresholds = 200;

bool entered_colour_sq = false;

int left_bend_count = 0;
int right_bend_count = 0;

// variables for task 6

int places[3] = {1, 2, 3};

int boxArrangement[3] = {1, 2, 3};

int box_count = 0;

bool Accending = false;

bool pick_box = false;

int box = 0;

int releases = 0;

int path_1[4] = {2, 0, 9, 9};
int path_2[4] = {2, -1, 1, 9};
int path_3[4] = {2, -1, 0, 1};
int path_4[4] = {2, 1, -1, 9};
int path_5[4] = {2, 1, 0, -1};

int *myarray[5] = {path_5, path_4, path_1, path_2, path_3};

// Define the sizes of lists for each element
int sizes[3][3] = {
    {2, 3, 4}, // Row 0
    {3, 2, 3}, // Row 1
    {4, 3, 2}  // Row 2
};

// Function to retrieve the path based on x and y
int *definepath(int x, int y)
{
  int sub_place = y - x;
  return myarray[sub_place + 2]; // Return pointer to the path array
}

int backpath_1[4] = {0, 5, 9, 9};
int backpath_2[4] = {-1, 1, 5, 9};
int backpath_3[4] = {-1, 0, 1, 5};
int backpath_4[4] = {1, -1, 5, 9};
int backpath_5[4] = {1, 0, -1, 5};

int *backarray[5] = {backpath_3, backpath_2, backpath_1, backpath_4, backpath_5};

int *backpath(int a, int b)
{
  int sub_place = a - b;
  return backarray[sub_place + 2];
}

int currentInstructionIndex = 0;   // Keeps track of the current instruction
int instruction[4] = {0, 3, 0, 3}; // Array of instructions
int instructionCount = 3;          // Total number of instructions
bool stopRobot = false;
int boxcount = 1;




bool allSensorsDetectwhite(int sen[])
{
  int sensorThresholds = 150;
  for (int i = 3; i < 9; i++)
  {
    if (sen[i] >= sensorThresholds)
    {
      return false; // Return false if any sensor detects below the threshold
    }
  }
  return true; // Return true if all sensors detect values above the threshold
}

void initializePins()
{
  for (int i = A0; i <= A15; i++)
  {
    pinMode(i, INPUT);
  }
  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
  pinMode(R_encoder_A, INPUT);
  pinMode(L_encoder_A, INPUT);
}

// Encoder counting functions
void countR()
{
  R_encoder_ticks++;
}

void countL()
{
  L_encoder_ticks++;
}

void kUp()
{
  Kp += 0.1;
}

void kDown()
{
  Kp -= 0.1;
}

void setup()
{

  initializePins();

  boxHandler.attachGripper(4); // Attach the gripper servo to pin 9
  boxHandler.attachArm(7);     // Attach the arm servo to pin 10

  Serial.begin(9600);

  // if (colorSensor.begin()) {
  //   Serial.println("TCS34725 found");
  // } else {
  //   Serial.println("No TCS34725 found ... check your connections");
  //   while (1);
  // }
  attachInterrupt(digitalPinToInterrupt(btn1), kUp, FALLING);
  attachInterrupt(digitalPinToInterrupt(btn2), kDown, FALLING);

  attachInterrupt(digitalPinToInterrupt(R_encoder_A), countR, RISING);
  attachInterrupt(digitalPinToInterrupt(L_encoder_A), countL, RISING);

  multiSensor.begin();

  oledDisplay.begin();
  oledDisplay.displayText("Stage : " + String(stage), 1.5, 0, 0);

  // int sensorThresholds =  calibrate(sensors);
  // oledDisplay.displayText(String(sensorThresholds),1,0,20);
  delay(2000);

  /*
    while (decoded_num == -1)
    {
      //read barcode


    }

    while (entered_colour_sq == false)
    {
      int direction = detectBend();


      if(decoded_num == 0){
        while (left_bend_count < 2)
        {
          irReader.readSensors(sensors);
          irReader.convertSensorsToBinary(sensors, binarySensors);
          direction = detectBend();
          processLineFollowing(binarySensors);
        }
        motor.stopRobot();
        direction = -1;
        //turn on led

        while (direction != 1)
        {
          irReader.readSensors(sensors);
          irReader.convertSensorsToBinary(sensors, binarySensors);
          direction = detectBend();
          processLineFollowing(binarySensors);
        }
        motor.stopRobot();
        //turn off led
        turn180();
        motor.stopRobot();

        //L_encoder_ticks=0;
        //R_encoder_ticks=0;

        entered_colour_sq = true;


      }




    }

  */
}
void moveDistance(float distance, int speed)
{

  // Calculate the number of pulses required for the given distance
  long pulses_needed = (distance / (PI * wheel_diameter)) * pulses_per_revolution;

  // Reset pulse counts
  R_encoder_ticks = 0;
  L_encoder_ticks = 0;

  // Wait until the required pulses are counted
  while (R_encoder_ticks < pulses_needed && L_encoder_ticks < pulses_needed)
  {
    motor.setMotorSpeed(speed, speed);
  }
  motor.stopRobot();
}

void turnBend(byte direction)
{

  L_encoder_ticks = 0;
  R_encoder_ticks = 0;

  moveDistance(0.12, 65);

  L_encoder_ticks = 0;
  R_encoder_ticks = 0;

  // 0 = left , 1 = right
  while (direction == 0)
  {
    oledDisplay.displayText("left turn", 1, 0, 0);
    oledDisplay.displayText(String(L_encoder_ticks) + " " + String(R_encoder_ticks), 1, 0, 50);
    motor.setMotorSpeed(-70, 70);

    if (R_encoder_ticks > 130 && L_encoder_ticks > 130)
    {
      L_encoder_ticks = 0;
      R_encoder_ticks = 0;
      return;
    }
  }

  while (direction == 1)
  {
    oledDisplay.displayText("right turn", 1, 0, 0);
    oledDisplay.displayText(String(L_encoder_ticks) + " " + String(R_encoder_ticks), 1, 0, 50);
    motor.setMotorSpeed(70, -70);
    if (R_encoder_ticks > 120 && L_encoder_ticks > 120)
    {
      L_encoder_ticks = 0;
      R_encoder_ticks = 0;
      return;
    }
  }
}

bool junctionDetected(int binarySensors[])
{
  if ((binarySensors[0] && binarySensors[11]) || (binarySensors[0] || binarySensors[1]) || (binarySensors[10] || binarySensors[11]))
  {
    return true;
  }
  else
  {
    return false;
  }
}

int detectBend()
{
  if (binarySensors[0] && binarySensors[11])
  {
    return 2;
  }
  if (binarySensors[0] || binarySensors[1])
  {
    return 0; // Left bend detected
  }
  else if (binarySensors[10] || binarySensors[11])
  {           // Adjust indices as needed
    return 1; // Right bend detected
  }
  else
  {
    return -1; // No bend detected
  }
}

void turnBendIR(byte direction)
{

  motor.setMotorSpeed(baseSpeed, baseSpeed);
  delay(200);
  // 0 = left , 1 = right
  while (direction == 0)
  {
    oledDisplay.displayText("left turn", 1, 0, 0);
    motor.setMotorSpeed(-100, 70);

    if ((analogRead(A9) < sensorThresholds) && (analogRead(A10) < sensorThresholds))
    {
      direction = -1;
      motor.stopRobot();
      return;
    }
  }

  while (direction == 1)
  {
    oledDisplay.displayText("right turn", 1, 0, 0);

    motor.setMotorSpeed(70, -100);

    if ((analogRead(A9) < sensorThresholds) && (analogRead(A10) < sensorThresholds))
    {
      direction = -1;
      motor.stopRobot();
      return;
    }
  }
}

void turn180left()
{
  L_encoder_ticks = 0;
  R_encoder_ticks = 0;
  while (true)
  {
    motor.setMotorSpeed(-70, 70);
    if (R_encoder_ticks > 300 && L_encoder_ticks > 300)
    {
      motor.stopRobot();
      return;
    }
  }
}

void turn180right()
{
  L_encoder_ticks = 0;
  R_encoder_ticks = 0;
  while (true)
  {
    motor.setMotorSpeed(70, -70);
    if (R_encoder_ticks > 300 && L_encoder_ticks > 300)
    {
      motor.stopRobot();
      return;
    }
  }
}

void straightMove(int base_speed)
{

  // Calculate error (difference between encoder counts)
  pid_error = L_encoder_ticks - R_encoder_ticks;

  pid_derivative = pid_error - pid_previous_error;

  // PID output (correction factor)
  float correction = (p_gain * pid_error) + (d_gain * pid_derivative);

  // Adjust motor speeds
  int left_speed = base_speed + correction;
  int right_speed = base_speed - correction;

  // Constrain motor speeds to valid range
  left_speed = constrain(left_speed, 0, 255);
  right_speed = constrain(right_speed, 0, 255);

  // Set the motor speeds
  motor.setMotorSpeed(left_speed, right_speed);

  pid_previous_error = pid_error;
}

int nextBox(int a, int b)
{
  int sumOfSet = 1 + 2 + 3;
  return sumOfSet - (a + b);
}

void sortArray(int arr[], int size)
{
  for (int i = 0; i < size - 1; i++)
  {
    for (int j = 0; j < size - i - 1; j++)
    {
      if (arr[j] > arr[j + 1])
      {
        // Swap
        int temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

int sensorDistances[4]; // Array to hold distances for 4 sensors

void updateDistances()
{
  multiSensor.readDistances(); // Trigger distance measurement

  // Get the distances from the multi-sensor library
  int *distances = multiSensor.getDistances();

  // Copy the distances to the shared array
  for (int i = 0; i < 4; i++)
  {
    sensorDistances[i] = distances[i];
  }
}

void loop()
{
  switch (stage)
  {
  case 1:
    break;

  case 2:
    // Code for case value2
    break;

  // Add more cases as needed
  case 3:

    if (releases == 3)
    {
      moveDistance(0.04, -65);
      irReader.readSensors(sensors);
      irReader.convertSensorsToBinary(sensors, binarySensors);
      processLineFollowing(binarySensors);
      if (junctionDetected(sensors))
      {
        moveDistance(0.04, 65);
        turnBend(1);

        stage = 5;
      }
    }

    while (releases < 3)
    {

      oledDisplay.displayText("stage 3 " + String(instruction[0]) + " " + String(instruction[1]) + " " + String(instruction[2]) + " " + String(instruction[3]), 1, 0, 0);
      oledDisplay.displayText("stage 3 " + String(instruction[currentInstructionIndex]), 1, 0, 30);
      oledDisplay.displayText("aneem " + String(currentInstructionIndex), 1, 0, 50);
      irReader.readSensors(sensors);
      irReader.convertSensorsToBinary(sensors, binarySensors);

      // Process line following until all sensors detect white
      if (instruction[currentInstructionIndex] == 5 && allSensorsDetectwhite(sensors))
      {

        // Reset the current instruction index
        currentInstructionIndex = 0;

        // Update the instruction array
        int *newInstructions = definepath(boxcount, places[boxcount - 1]);
        for (int i = 0; i < 4; i++)
        {
          instruction[i] = newInstructions[i];
        }

        // Update the instruction count
        instructionCount = sizes[boxcount - 1][places[boxcount] - 1];

        break;
      }

      if (pick_box && allSensorsDetectwhite(sensors))
      {
        motor.stopRobot();
        boxHandler.lowerArm();
        delay(1000);
        boxHandler.releaseBox();
        delay(1000);
        moveDistance(0.04, -65);
        turn180right();
        moveDistance(0.04, -65);

        releases++;
        pick_box = false;
        if (releases == 3){
          stage = 8;
        }


        int *new_instruction = backpath(boxcount + 1, places[boxcount - 1]);
        instructionCount = sizes[places[boxcount - 1] - 1][boxcount - 1];
        for (int i = 0; i < 4; i++)
        {
          instruction[i] = new_instruction[i];
        }
        currentInstructionIndex = 0;
        oledDisplay.displayText("back " + String(currentInstructionIndex), 1, 0, 0);
        oledDisplay.displayText("stage 3 " + String(instruction[0]) + " " + String(instruction[1]) + " " + String(instruction[2]) + " " + String(instruction[3]), 1, 0, 0);
        delay(2000);
        boxcount++;

        break; // Exit the loop
      }

      // Check if a junction or bend is detected
      if (junctionDetected(binarySensors) || instruction[currentInstructionIndex] == 2)
      {

        motor.stopRobot();
        delay(500); // Wait for 0.5 seconds

        // Process the current instruction based on the instruction array
        switch (instruction[currentInstructionIndex])
        {
        case 2:

          boxHandler.grabBox();
          boxHandler.liftBox();
          moveDistance(0.04, -65);
          turn180right();
          moveDistance(0.04, -65);
          oledDisplay.displayText("hiiii " + String(currentInstructionIndex), 1, 0, 20);
          pick_box = true;
          break;
        case -1:
          // Turn left
          moveDistance(0.02, 65);
          turnBend(0);
          break;
        case 0:
          // Go straight

          if (instruction[currentInstructionIndex + 1] == 3)
          {

            motor.stopRobot();
            moveDistance(0.05, 70);

            const int iterations = 7;
            int distancesBottom[iterations];
            int distancesMiddle[iterations];
            int distancesTop[iterations];

            // Collect data for 3 iterations
            for (int i = 0; i < iterations; i++)
            {
              updateDistances();

              distancesBottom[i] = sensorDistances[1];
              distancesMiddle[i] = sensorDistances[2];
              distancesTop[i] = sensorDistances[3];
            }

            sortArray(distancesBottom, iterations);
            sortArray(distancesMiddle, iterations);
            sortArray(distancesTop, iterations);

            // Get the median (middle) value
            int medianBottom = distancesBottom[iterations / 2];
            int medianMiddle = distancesMiddle[iterations / 2];
            int medianTop = distancesTop[iterations / 2];

            oledDisplay.displayText(String(medianBottom) + " " + String(medianMiddle) + " " + String(medianTop), 1, 0, 20);
            delay(4000);
            if (medianTop < 400)
            {
              box = 3;
            }
            else if (medianMiddle < 400)
            {
              box = 2;
            }
            else
            {
              box = 1;
            }

            if (!Accending)
            {
              places[box_count] = 4 - box;
            }
            else
            {
              places[box_count] = box;
            }

            if (box_count == 0)
            {
              moveDistance(0.03, 65);
            }
            oledDisplay.displayText("box " + String(box) + " box count " + String(box_count), 1, 0, 0);
            delay(4000);
            box_count++;

            if (box_count == 2)
            {
              places[box_count] = nextBox(places[0], places[1]);
              motor.stopRobot();
              delay(1000);
              moveDistance(0.03, -65);
              currentInstructionIndex = -1;
              instructionCount = 2;
              instruction[0] = 1;
              instruction[1] = 5;
              instruction[2] = 9;
              instruction[3] = 9;
              oledDisplay.displayText("stage 3 " + String(instruction[0]) + " " + String(instruction[1]) + " " + String(instruction[2]) + " " + String(instruction[3]), 1, 0, 0);
              turn180left();
              moveDistance(0.15, -65);

              delay(500);
              break;
            }

            currentInstructionIndex++;
            break;
          }
          else
          {
            moveDistance(0.04, 65);
          }
          break;
        case 1:
          // Turn right
          moveDistance(0.02, 65);
          turnBend(1);
          break;
        }
        currentInstructionIndex++;
      }
      else
      {
        // Process line-following logic when no junction or bend is detected
        processLineFollowing(binarySensors);
      }
    }
    break;

  
  

    






    

  default:
    irReader.readSensors(sensors);
    irReader.convertSensorsToBinary(sensors, binarySensors);
    processLineFollowing(binarySensors);
    break;
  }

}

/*
while (historyIndex < historySize) {
    straightMove(60);
    // Read sensors
    irReader.readSensors(sensors);
    irReader.convertSensorsToBinary(sensors, binarySensors);

    // Calculate the sum of binary sensors
    int sum = 0;
    for (int i = 0; i < 12; i++) {
        sum += binarySensors[i];
    }

    // Update sensor history based on the sum
    if (binarySensors[5] == 1) {
        senHistory[historyIndex] = 1; // White line
    }  else {
        senHistory[historyIndex] = 0; // Black line
    }


    historyIndex++;
}
*/

// uint16_t distance = tof.readRangeSingleMillimeters();

// if (tof.timeoutOccurred()) {
//   oledDisplay.displayText("Sensor timeout!");
// } else {
//   //oledDisplay.displayText("Distance: " + String(distance) + " mm");
// }
// irReader.readSensors(sensors);
// irReader.convertSensorsToBinary(sensors, binarySensors);

// processLineFollowing(binarySensors);

// if (distance<30){

//   motor.stopRobot();
//   moveDistance(0.05,50);
//   boxHandler.grabBox();
//   boxHandler.liftBox();
//   turn180left();
//   boxHandler.lowerArm();
//    boxHandler.releaseBox();
//   while (true)
//   {

//   }

// }

// oledDisplay.displayText(String(L_encoder_ticks) + " " + String(R_encoder_ticks),1,0,50);

// colorSensor.readColor();
// oledDisplay.displayText(colorSensor.getDetectedColor(),1,0,20);
// readSensors(sensors);
//  updateSensorHistory(sen); // Update sensor history

// oledDisplay.displayText(irValues_str + "        " + "Kp = "+ String(Kp) ,1,0,0);

// int direction = detectBend();

// if (allSensorsDetectBlack(binarySensors) && direction == -1 ) {
// motor.stopRobot();
// }

// boxHandler.grabBox();    // Grab the box
// delay(1000);             // Wait 1 second
// boxHandler.liftBox();    // Lift the box
// delay(1000);             // Wait 1 second
// boxHandler.lowerArm();   // Lower the box
// delay(1000);             // Wait 1 second
// boxHandler.releaseBox(); // Release the box
// delay(2000);

// if (direction == 0 || direction == 1 ){
//   turnBend(direction);
// }
// else{
//    processLineFollowing(binarySensors);
// }

// for (int i = 0; i < historySize; i++) {
//   Serial.print("Element ");
//   Serial.print(i);
//   Serial.print(": ");
//   Serial.println(senHistory[i]); // Print each element on a new line
//}
