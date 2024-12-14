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


#define XSHUT_PIN_1 A15
#define XSHUT_PIN_2 A13
#define XSHUT_PIN_3 A11


ServoController boxHandler;

//////////////////////////////
// stage handler 

#define switch1 33
#define switch2 35
#define switch3 37
#define switch4 39

int stage = 3 ;
int testCount = 0 ;
int colour;

//TCSColorSensor colorSensor;
oled oledDisplay;
readIR irReader;

bool fisrtBendDetected = false;

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
const int LEDpin= 41 ;

const int leftIR = A4;
const int rightIR = A15;
const int frontIR = A10;

int lastError = 0;
int integral = 0;

float Kp = 4.0;  
float Ki = 0.0; 
float Kd = 1.5;  


// for straght line
float p_gain = 1.2;   // Proportional gain
float i_gain = 0.05;  // Integral gain
float d_gain = 0.1;   // Derivative gain

// PID variables
float pid_error = 0;
float pid_previous_error = 0;
float pid_integral = 0;
float pid_derivative = 0;

int blackThreshold = 700 ; 
int whiteThreshold = 250 ;
int baseSpeed = 65 ;  // Base speed for the motors

const int historySize = 180; // Size of the history buffer
int senHistory[historySize]; // History array to store past sensor states
int historyIndex = 0; // Current index for appending in the history array
int barcode = 4 ;
int outputArray[13] = {0};



const int R_encoder_A = 2;

const int L_encoder_A = 3;


volatile long R_encoder_ticks = 0;
volatile long L_encoder_ticks = 0;

const float wheel_diameter = 0.067; // Diameter of the wheels in meters
int pulses_per_revolution = 222;

// Variables for speed calculation
long previous_ticks_R = 0;
long previous_ticks_L = 0;

String irValues_str ="";

int sensorThresholds = 200;

bool entered_colour_sq = false;

int left_bend_count = 0;
int right_bend_count = 0;
/////////////////////////////////////
// variables for task 2 

int wallChecked = -1 ;
int wallDetected= -1 ;
bool wallDetectionFinished = false ;
bool returnedToStart = false ;
bool virtualBoxFinished = false ;
bool braaughtToCheckpoint = false ;



int VirtualInstructionsWall[2][8]={{1,2,3,-1,0,7,0,9},{1,7,1,0,7,0,9}} ;

int VirtualInstructionsReturn[2][14]={{7,7,10},{7,7,7,-1,10}} ;

int VirtualInstructionsFinish[2][5][19] = {
  //close wall open
  {
    {1,-1,4,3,-1,-1,-1,2,11,100},
    {8,1,1,4,7,11,100},
   {-1,7,4,3,-1,-1,-1,4,7,11,100} ,
   {-1,7,7,4,7,3,-1,-1,-1,4,7,11,100} ,
    {-1,7,7,7,4,7,7,3,-1,-1,-1,4,7,11,100}
  },

  //far wall open
  {
  {1,-1,4,7,7,3,-1,-1,-1,2,11,100},
  {-1,2,7,3,1,1,1,4,7,11,100},
  {-1,7,2,3,1,1,1,4,7,11,100},
  {8,1,7,7,1,4,7,11,100},
  {-1,7,7,7,4,3,-1,-1,-1,4,7,11,100}
  
}
} ;


int VirCount = 0 ;
int* currentInstruction = VirtualInstructionsWall[0];





// variables for task 6

int places[3] = {1,2,3};

int boxArrangement[3] = {1,2,3};

int box_count = 0;

bool Accending = false ;

bool pick_box = false;

int box = 0;

int releases = 0;


int path_1[4] = {2, 0, 9, 9};
int path_2[4] = {2, -1, 1, 9};
int path_3[4] = {2, -1, 0, 1};
int path_4[4] = {2, 1, -1, 9};
int path_5[4] = {2, 1, 0, -1};

int* myarray[5] = {path_5, path_4, path_1, path_2, path_3};
int checkZero(int barcode){
  if(barcode==0){
    return 0 ;
  }else{
    return 1 ;
  }
}
// Define the sizes of lists for each element
int sizes[3][3] = {
    {2, 3, 4}, // Row 0
    {3, 2, 3}, // Row 1
    {4, 3, 2}  // Row 2
};

// Function to retrieve the path based on x and y
int* definepath(int x, int y) {
    int sub_place = y - x;
    return myarray[sub_place + 2]; // Return pointer to the path array
}

int backpath_1[4] = { 0, 5, 9,9};
int backpath_2[4] = { -1, 1, 5,9};
int backpath_3[4] = { -1, 0, 1,5};
int backpath_4[4] = { 1, -1, 5,9};
int backpath_5[4] = { 1, 0, -1,5};

int* backarray[5] = {backpath_3,backpath_2,backpath_1,backpath_4,backpath_5};

int* backpath(int a,int b) {
  int sub_place = a-b;
  return backarray[sub_place + 2];
}





int currentInstructionIndex = 0; // Keeps track of the current instruction
int instruction[4] = {0,3,0,3} ;   // Array of instructions
int instructionCount  = 3 ;        // Total number of instructions
bool stopRobot = false;
int boxcount = 1;

bool allSensorsDetectwhite(int sen[]) {
  int sensorThresholds = 350;
  for (int i = 0; i < 11; i++) {
    if (sen[i] >= sensorThresholds) {
      return false; // Return false if any sensor detects below the threshold
    }
  }
  return true; // Return true if all sensors detect values above the threshold
}

bool doted = false;

void initializePins() {
  for (int i = A0; i <= A15; i++) {
    pinMode(i, INPUT);
  }
  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
  pinMode(R_encoder_A, INPUT);
  pinMode(L_encoder_A, INPUT);
  pinMode(LEDpin,OUTPUT);
  pinMode(switch1,INPUT_PULLUP);
  pinMode(switch2,INPUT_PULLUP);
  pinMode(switch3,INPUT_PULLUP);
  pinMode(switch4,INPUT_PULLUP);

}

// Encoder counting functions
void countR() {
    R_encoder_ticks++;
}

void countL() {
    L_encoder_ticks++;
}

void kUp() {
    Kp +=0.1;   
}

void kDown() {
    Kp -=0.1;
}



void setup() {

  initializePins();

  boxHandler.attachGripper(4); // Attach the gripper servo to pin 9
  boxHandler.attachArm(7);    // Attach the arm servo to pin 10


  Serial.begin(9600);



  // if (colorSensor.begin()) {
  //   Serial.println("TCS34725 found");
  // } else {
  //  Serial.println("No TCS34725 found ... check your connections");
  //  while (1);
  // }
  attachInterrupt(digitalPinToInterrupt(btn1), kUp, FALLING);
  attachInterrupt(digitalPinToInterrupt(btn2), kDown, FALLING);

  attachInterrupt(digitalPinToInterrupt(R_encoder_A), countR, RISING);
  attachInterrupt(digitalPinToInterrupt(L_encoder_A), countL, RISING);

  multiSensor.begin();
 
  oledDisplay.begin(); 
  int value1 = digitalRead(switch1);
  int value2 = digitalRead(switch2);
  int value3 = digitalRead(switch3);
  int value4 = digitalRead(switch4);

  stage = value1*1 + value2*2 + value3*3 +value4*4;

  oledDisplay.displayText("Stage : " + String(stage),1.5,0,0);

  // int sensorThresholds =  calibrate(sensors);
  //oledDisplay.displayText(String(sensorThresholds),1,0,20);
  delay(1000);


  
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


*/    
    
  }   
  

void LED(int status){

  if(status){
    digitalWrite(LEDpin,HIGH);

  }else{
    digitalWrite(LEDpin,LOW);
  }
}
  

void moveDistance(float distance, int speed) {

    // Calculate the number of pulses required for the given distance
    long pulses_needed = (distance / (PI * wheel_diameter)) * pulses_per_revolution;

    // Reset pulse counts
    R_encoder_ticks = 0;
    L_encoder_ticks = 0;

    
    // Wait until the required pulses are counted
    while (R_encoder_ticks < pulses_needed && L_encoder_ticks < pulses_needed) {
      motor.setMotorSpeed(speed,speed);
    }
    motor.stopRobot();
}

    
void turnBend(byte direction){

  L_encoder_ticks = 0;
  R_encoder_ticks = 0;

  moveDistance(0.12 , 65);

  L_encoder_ticks = 0;
  R_encoder_ticks = 0;

  // 0 = left , 1 = right
  while (direction == 0){
    oledDisplay.displayText("left turn",1,0,0);  
    oledDisplay.displayText(String(L_encoder_ticks) + " " + String(R_encoder_ticks),1,0,50); 
    motor.setMotorSpeed(-70,70);

    if (R_encoder_ticks > 140 && L_encoder_ticks  >140){
      L_encoder_ticks = 0;
      R_encoder_ticks = 0;
      return;
    }
  
  }

  while (direction == 1){
    oledDisplay.displayText("right turn",1,0,0);  
    oledDisplay.displayText(String(L_encoder_ticks) + " " + String(R_encoder_ticks),1,0,50); 
    motor.setMotorSpeed(70,-70);
    if (R_encoder_ticks > 120 && L_encoder_ticks  > 120){
      L_encoder_ticks = 0;
      R_encoder_ticks = 0;
      return;
    }
    
  }

}

bool junctionDetected(int binarySensors[]) {
  if ((binarySensors[0]&&binarySensors[1])||((binarySensors[11]&&binarySensors[10]))) {
    return true;
  } else {  
    return false;
    
  }
}

int detectBend(){
  if( binarySensors[0]&& binarySensors[11]){
    return 2 ;
  } 
  if (binarySensors[0] || binarySensors[1] ) {
    return 0;  // Left bend detected
  } else if ( binarySensors[10] || binarySensors[11]) {  // Adjust indices as needed
    return 1;  // Right bend detected
  } else {
    return -1; // No bend detected
  }
}


void turnBendIR(byte direction){

  motor.setMotorSpeed(baseSpeed,baseSpeed);
  delay(200);
  // 0 = left , 1 = right
  while (direction == 0){
    oledDisplay.displayText("left turn",1,0,0);  
    motor.setMotorSpeed(-100,70);

    if ((analogRead(A9) < sensorThresholds) && (analogRead(A10) < sensorThresholds )){
      direction = -1;
      motor.stopRobot();
       return;
     }
  }

  while (direction == 1){
    oledDisplay.displayText("right turn",1,0,0);  

    motor.setMotorSpeed(70,-100);
    
   if ((analogRead(A9) < sensorThresholds) && (analogRead(A10) < sensorThresholds)){
       direction  = -1;
        motor.stopRobot();
       return;
   }
  }

}

void turn180left(){
  L_encoder_ticks = 0;
  R_encoder_ticks = 0;
  while (true)
  {
    motor.setMotorSpeed(-70,70);
    if (R_encoder_ticks > 300 && L_encoder_ticks  > 300){
      motor.stopRobot();
      return;
  }
}
}


void turn180right(){
  L_encoder_ticks = 0;
  R_encoder_ticks = 0;
  while (true)
  {
    motor.setMotorSpeed(70,-70);
    if (R_encoder_ticks > 300 && L_encoder_ticks  > 300){
      motor.stopRobot();
      return;
  }
}
}


void straightMove(int base_speed){

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
int moveTilBend(){
  int bendDirection = -1 ;
  while( bendDirection == -1){
    bendDirection = detectBend() ;
    processLineFollowing(binarySensors);
  }
  return bendDirection ; 
}


int processSensorHistory(int senHistory[]) {
  int barcodeNumber = 0 ; 
  int outCount = 0 ;
  int consecutiveCount = 0;

  for (int i = 0; i < historySize; i++) {
    if (senHistory[i] == 1) {
      consecutiveCount++;
    } else {
      if(consecutiveCount > 0  && consecutiveCount < 10){
        outputArray[outCount] = 0;
        outCount++;
      }else if(consecutiveCount > 10){
        outputArray[outCount] = 1;
        outCount++;
      }
        consecutiveCount = 0;
      }
      
      
  }

  for (int k = 0 ; k < 8 ; k++){
    Serial.println("");
    Serial.print(outputArray[k]);
    barcodeNumber = barcodeNumber + outputArray[k] * (1 << (7 - k));

    
  }
  oledDisplay.displayText(String(barcodeNumber));
  delay(2000);
  return (barcodeNumber % 5) ;

}

void decodeBarcode(){
  colour = 0 ;
    irReader.setColour(colour);

    while (historyIndex < historySize) {
    straightMove(60);
    delay(40);
    // Read sensors
    irReader.readSensors(sensors);
    irReader.convertSensorsToBinary(sensors, binarySensors);

    // Calculate the sum of binary sensors
    int sum = 0;
    for (int i = 0; i < 12; i++) {
        sum += binarySensors[i];
    }

    // Update sensor history based on the sum
    if (sum > 10) {
        senHistory[historyIndex] = 1; // White line
    }  else {
      senHistory[historyIndex] = 0;
    }
    Serial.print(senHistory[historyIndex]);

    if ((binarySensors[0] == 0 && binarySensors[11] == 0 ) &&(binarySensors[5] == 1 || binarySensors[6] == 1|| binarySensors[4] == 1 || binarySensors[7] == 1|| binarySensors[3] == 1 || binarySensors[8] == 1|| binarySensors[2] == 1 || binarySensors[9] == 1|| binarySensors[10] == 1 || binarySensors[1] == 1) ) {
    testCount = testCount+1 ;
  }else{
    testCount = 0 ;
  }

    if (testCount > 6 ){
      motor.stopRobot();
      delay(500);
      historyIndex = historySize+1 ;
      break;
    }

    historyIndex++;
}
motor.stopRobot();
barcode = processSensorHistory(senHistory);
oledDisplay.displayText(String(barcode));
delay(500);
irReader.setColour(0);
stage = 2; 
}

int nextBox(int a, int b) {
  int sumOfSet = 1 + 2 + 3;
  return sumOfSet - (a + b);
}


void sortArray(int arr[], int size) {
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        // Swap
        int temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}


int sensorDistances[4]; // Array to hold distances for 4 sensors

void updateDistances() {
    multiSensor.readDistances(); // Trigger distance measurement

    // Get the distances from the multi-sensor library
    int* distances = multiSensor.getDistances(); 

    // Copy the distances to the shared array
    for (int i = 0; i < 4; i++) {
        sensorDistances[i] = distances[i];
    }
}



void followVirtualInstructions(int currentInstruction[14]){
    if(junctionDetected( binarySensors)){
          motor.stopRobot();
          oledDisplay.displayText("Vir count"+ String(VirCount),1,0,0);
          delay(200);
         
          switch(currentInstruction[VirCount]){
            case 2 :
            LED(1);
            moveDistance(0.04,65);
            break;
            case 3:
            moveDistance(0.04,65);
            LED(0);
            moveDistance(0.04,-65);
            turn180right();
            moveDistance(0.05,-65);
            break;
            case -1 :
            moveDistance(0.03,65);
            turnBend(0);
            moveDistance(0.03,-65);
            break;
            case 1:
            moveDistance(0.03,65);
            turnBend(1);
            moveDistance(0.03,-65);
            break;
            case 4 :
            moveDistance(0.03,-65);
            turn180right();
            moveDistance(0.17,-65);
            LED(1);
            moveDistance(0.04,65);
            break;
            case 5:
            moveDistance(0.03,65);
            LED(1);
            moveDistance(0.04,65);
            break;
            case 6:
            moveDistance(0.03,65);
            LED(0);
            motor.stopRobot();
            break;
            case 7:
            moveDistance(0.04,65);
            break;
            case 8 :
            turn180right();
            break;
            case 9:
            motor.stopRobot();
            wallDetectionFinished = true ;
            oledDisplay.displayText(("wall "+String(wallDetected)),1,0,0);
            delay(1000);
            turn180left();
            moveDistance(0.05,-65);
            VirCount=-1;
            break;
            case 10 :
            motor.stopRobot();
            returnedToStart = true ;
            VirCount=-1;
            delay(200);
            break;
            case 100:
            motor.stopRobot();
            VirCount = -1 ;
            virtualBoxFinished=true;
            break;
            case 11:
            moveDistance(0.08,65);
            motor.stopRobot();
            LED(0);
            break ; 
            break ;
            case 0 :// Go straight
            motor.stopRobot();
            moveDistance(0.05,70);
            wallChecked++ ;
            const int iterations = 7;
            int distancesBottom[iterations];
            int distancesMiddle[iterations];
            

            for (int i = 0; i < iterations; i++) {
              updateDistances();
              

              distancesBottom[i] = sensorDistances[1];
              distancesMiddle[i] = sensorDistances[2];
              

            }
              sortArray(distancesBottom, iterations);
            sortArray(distancesMiddle, iterations);

            int medianBottom = distancesBottom[iterations / 2];
            int medianMiddle = distancesMiddle[iterations / 2];
            
            if (medianBottom < 700 && medianMiddle < 700){
            
              wallDetected = wallChecked ;
              oledDisplay.displayText(("wall detected"+String(wallDetected)),1,0,0);
              delay(200);
            }
            break;
            

            
          }
          motor.stopRobot();
          delay(500);
          VirCount ++ ;
        }else{
            processLineFollowing(binarySensors);
        }
}


void loop() {
    switch (stage) {
        case 0:
            irReader.readSensors(sensors);
            irReader.convertSensorsToBinary(sensors, binarySensors);
            oledDisplay.displayText(irValues_str, 1.5, 0, 0);


            // colorSensor.readColor();
            // oledDisplay.displayText(colorSensor.getDetectedColor(),1,0,20);
            break;

        case 1:
            decodeBarcode();
            break;

        // stage 2
        case 2:
            while (true) {
                irReader.readSensors(sensors);
                irReader.convertSensorsToBinary(sensors, binarySensors);
                if (wallDetectionFinished == false) {
                    currentInstruction = VirtualInstructionsWall[checkZero(barcode)];
                } else {
                    if (returnedToStart == false) {
                        currentInstruction = VirtualInstructionsReturn[checkZero(barcode)];
                    } else {
                        currentInstruction = VirtualInstructionsFinish[1 - wallDetected][barcode];

                        if (virtualBoxFinished) {
                            motor.stopRobot();
                            stage = 4;
                            delay(1000);
                            moveDistance(0.15, 65);
                        }
                    }
                }

                followVirtualInstructions(currentInstruction);
            }

        case 3:
            irReader.setColour(1);

            if (releases == 3) {
                irReader.readSensors(sensors);
                irReader.convertSensorsToBinary(sensors, binarySensors);

                if (junctionDetected(sensors)) {
                    motor.stopRobot();
                    delay(500);
                    moveDistance(0.08, 65);
                    turnBend(1);
                    stage = 7; // Transition to stage 8
                } else {
                    processLineFollowing(binarySensors);
                }
            } else {
                while (releases < 3) {
                    oledDisplay.displayText("stage 3 " + String(instruction[0]) + " " + String(instruction[1]) + " " + String(instruction[2]) + " " + String(instruction[3]), 1, 0, 0);
                    oledDisplay.displayText("stage 3 " + String(instruction[currentInstructionIndex]), 1, 0, 30);
                    oledDisplay.displayText("aneem " + String(currentInstructionIndex), 1, 0, 50);
                    irReader.readSensors(sensors);
                    irReader.convertSensorsToBinary(sensors, binarySensors);

                    // Process line following until all sensors detect white
                    if (instruction[currentInstructionIndex] == 5 && allSensorsDetectwhite(sensors)) {
                        // Reset the current instruction index
                        currentInstructionIndex = 0;

                        // Update the instruction array
                        int* newInstructions = definepath(boxcount, places[boxcount - 1]);
                        for (int i = 0; i < 4; i++) {
                            instruction[i] = newInstructions[i];
                        }

                        // Update the instruction count
                        instructionCount = sizes[boxcount - 1][places[boxcount - 1]];

                        break;
                    }

                    if (pick_box && allSensorsDetectwhite(sensors)) {
                        motor.stopRobot();
                        boxHandler.lowerArm();
                        boxHandler.releaseBox();
                        moveDistance(0.04, -65);
                        turn180right();
                        moveDistance(0.04, -65);

                        releases++;
                        pick_box = false;

                        int* new_instruction = backpath(boxcount + 1, places[boxcount - 1]);
                        instructionCount = sizes[places[boxcount - 1] - 1][boxcount - 1];
                        for (int i = 0; i < 4; i++) {
                            instruction[i] = new_instruction[i];
                        }
                        currentInstructionIndex = 0;
                        oledDisplay.displayText("back " + String(currentInstructionIndex), 1, 0, 0);
                        oledDisplay.displayText("stage 3 " + String(instruction[0]) + " " + String(instruction[1]) + " " + String(instruction[2]) + " " + String(instruction[3]), 1, 0, 0);

                        boxcount++;

                        break; // Exit the loop
                    }

                    // Check if a junction or bend is detected
                    if (junctionDetected(binarySensors) || instruction[currentInstructionIndex] == 2) {
                        motor.stopRobot();
                        delay(500); // Wait for 0.5 seconds

                        // Process the current instruction based on the instruction array
                        switch (instruction[currentInstructionIndex]) {
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
                                if (instruction[currentInstructionIndex + 1] == 3) {
                                    motor.stopRobot();
                                    moveDistance(0.05, 70);

                                    const int iterations = 7;
                                    int distancesBottom[iterations];
                                    int distancesMiddle[iterations];
                                    int distancesTop[iterations];

                                    // Collect data for 3 iterations
                                    for (int i = 0; i < iterations; i++) {
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

                                    if (medianTop < 400) {
                                        box = 3;
                                    } else if (medianMiddle < 400) {
                                        box = 2;
                                    } else {
                                        box = 1;
                                    }

                                    if (!Accending) {
                                        places[box_count] = 4 - box;
                                    } else {
                                        places[box_count] = box;
                                    }

                                    if (box_count == 0) {
                                        moveDistance(0.03, 65);
                                    }
                                    oledDisplay.displayText("box " + String(box) + " box count " + String(box_count), 1, 0, 0);

                                    box_count++;

                                    if (box_count == 2) {
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
                                } else {
                                    moveDistance(0.04, 65);
                                }
                                break;
                            case 1:
                                // Turn right
                                moveDistance(0.02, 65);
                                turnBend(1);
                                break;
                            default:
                                processLineFollowing(binarySensors);
                                break;
                        }
                        currentInstructionIndex++;
                    } else {
                        // Process line-following logic when no junction or bend is detected
                        processLineFollowing(binarySensors);
                    }
                }
            }
            break;

       case 5:
        updateDistances();


        oledDisplay.displayText("stage 5",1,0,0);
       
        // Get the distances array
      



       
        oledDisplay.displayText( "Wait for gate close " +String(sensorDistances[0]),1,0,0);
        delay(200); 
        while (sensorDistances[0]>=100) {
          updateDistances();
            motor.stopRobot(); 
            delay(50);   // Short delay before rechecking
        }

        // Now wait for the next time the gate opens
        oledDisplay.displayText("Gate closed",2,0,0);
        delay(200); 
        while (sensorDistances[0]<=100) {
          updateDistances();
            motor.stopRobot(); 
            delay(50);   // Short delay before rechecking
        }

    // Move through the gate quickly
        oledDisplay.displayText("Gate Opened",2,0,0);
        delay(1000);
        moveDistance(0.35,120);
        motor.stopRobot();
        stage = 3;
        break;

        case 8:
            irReader.setColour(1);
            irReader.readSensors(sensors);
            irReader.convertSensorsToBinary(sensors, binarySensors);
            processLineFollowing(binarySensors);
            if (allSensorsDetectwhite(sensors)) {
                motor.stopRobot();
                moveDistance(0.04, -65);
                turnBend(0);
            }
            break;

        default:
            irReader.readSensors(sensors);
            irReader.convertSensorsToBinary(sensors, binarySensors);
            processLineFollowing(binarySensors);
            break;

        case 4:
            irReader.setColour(2);
            irReader.readSensors(sensors);
            irReader.convertSensorsToBinary(sensors, binarySensors);
            processLineFollowing(binarySensors);
            break;


    case 6:
    while (true)
    
    {
      /* code */
    }
    
    
    stage = 7;
    break;
}
}

/*
while (historyIndex < historySize) {
    straightMove(60);
    delay(50);
    // Read sensors
    irReader.readSensors(sensors);
    irReader.convertSensorsToBinary(sensors, binarySensors);

    // Calculate the sum of binary sensors
    int sum = 0;
    for (int i = 0; i < 12; i++) {
        sum += binarySensors[i];
    }

    // Update sensor history based on the sum
    if (sum > 8) {
        senHistory[historyIndex] = 1; // White line
    }  else {
      senHistory[historyIndex] = 0;
    }
    Serial.print(senHistory[historyIndex]);
    if ((binarySensors[0] == 0 && binarySensors[11] == 0 ) &&(binarySensors[5] == 1 || binarySensors[6] == 1|| binarySensors[4] == 1 || binarySensors[7] == 1|| binarySensors[3] == 1 || binarySensors[8] == 1|| binarySensors[2] == 1 || binarySensors[9] == 1) ) {
    testCount = testCount+1 ;
  }else{
    testCount = 0 ;
  }

    if (testCount > 6 ){
      motor.stopRobot();
      delay(5000);
      historyIndex = historySize+1 ;
      break;
    }

    historyIndex++;
}
motor.stopRobot();
      delay(500);
barcode = processSensorHistory(senHistory);
oledDisplay.displayText(String(barcode));
delay(1000);
irReader.setColour(0);
stage = 2; 
}




void loop() {
switch (stage) {
    case 1:
    LED(0);
    decodeBarcode();

        break;
    case 2:
    while (fisrtBendDetected == false){
      irReader.readSensors(sensors);
    irReader.convertSensorsToBinary(sensors, binarySensors);
        processLineFollowing(binarySensors);
        if(detectBend() != -1){
          fisrtBendDetected = true ;
          }
    }
    for(int i=0 ; i<5 ; i++)
{
  switch(VInstructions[barcode][i]){
    case 0 :
    while(detectBend() == -1){
      straightMove(baseSpeed);
          }
    break;
    case 1 :
    turnBend(1);
    break ;
    case -1 :
    turnBend(0);
    break ;
}
        break;
    // Add more cases as needed
    case 3:
    break;
    case 5 :
    break;

    while(binarySensors[0] == 0 && binarySensors[11]==0){
      irReader.setColour(0);
    irReader.readSensors(sensors);
    irReader.convertSensorsToBinary(sensors, binarySensors);
        processLineFollowing(binarySensors); 
        }
        motor.stopRobot();
        delay(1000);
        while(binarySensors[0] != 0 && binarySensors[11]!=0){
      irReader.setColour(0);
    irReader.readSensors(sensors);
    irReader.convertSensorsToBinary(sensors, binarySensors);
        processLineFollowing(binarySensors); 
        }

        stage = 6 ;
       
        break;
    case 6 : 
    irReader.setColour(1);
    irReader.readSensors(sensors);
    irReader.convertSensorsToBinary(sensors, binarySensors);
    processLineFollowing(binarySensors); 

    break;
    


}







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
  
  //colorSensor.readColor();
  //oledDisplay.displayText(colorSensor.getDetectedColor(),1,0,20);
  //readSensors(sensors);
  // updateSensorHistory(sen); // Update sensor history
  
  

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
}
*/

