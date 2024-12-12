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
#include <VL53L0X.h>


#define XSHUT_PIN_1 A15
#define XSHUT_PIN_2 A13
#define XSHUT_PIN_3 A11


ServoController boxHandler;

int stage = 1 ;
int testCount = 0 ;
int colour;

TCSColorSensor colorSensor;
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

int VInstructions[5][6] ={{0,0,0,0,0,0},{0,1,0,0,0,0},{0,0,0,0,0,0},{0,1,0,0,0,0},{0,0,0,0,0,0}}; 

/*
0-move forwarad
1- turn right
-1-turn left 
2 - turn on the bulb 
-2 -turn off the bulb
 

*/

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

int baseSpeed = 65 ;  // Base speed for the motors

const int historySize = 200; // Size of the history buffer
int senHistory[historySize]; // History array to store past sensor states
int historyIndex = 0; // Current index for appending in the history array
int barcode = 0  ;
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





void initializePins() {
  for (int i = A0; i <= A15; i++) {
    pinMode(i, INPUT);
  }
  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
  pinMode(R_encoder_A, INPUT);
  pinMode(L_encoder_A, INPUT);

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
  //   Serial.println("No TCS34725 found ... check your connections");
  //   while (1);
  // }
  attachInterrupt(digitalPinToInterrupt(btn1), kUp, FALLING);
  attachInterrupt(digitalPinToInterrupt(btn2), kDown, FALLING);

  attachInterrupt(digitalPinToInterrupt(R_encoder_A), countR, RISING);
  attachInterrupt(digitalPinToInterrupt(L_encoder_A), countL, RISING);


 
  oledDisplay.begin(); 
  oledDisplay.displayText("ElectroBots",1.5,0,0);  

  // int sensorThresholds =  calibrate(sensors);
  //oledDisplay.displayText(String(sensorThresholds),1,0,20);
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

    if (R_encoder_ticks > 150 && L_encoder_ticks  >150){
      L_encoder_ticks = 0;
      R_encoder_ticks = 0;
      return;
    }
  
  }

  while (direction == 1){
    oledDisplay.displayText("right turn",1,0,0);  
    oledDisplay.displayText(String(L_encoder_ticks) + " " + String(R_encoder_ticks),1,0,50); 
    motor.setMotorSpeed(70,-70);
    if (R_encoder_ticks > 150 && L_encoder_ticks  > 150){
      L_encoder_ticks = 0;
      R_encoder_ticks = 0;
      return;
    }
    
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
    if (R_encoder_ticks > 310 && L_encoder_ticks  > 310){
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
    if (R_encoder_ticks > 290 && L_encoder_ticks  > 290){
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
      if(consecutiveCount >2 && consecutiveCount < 6){
        outputArray[outCount] = 0;
        outCount++;
      }else if(consecutiveCount > 7){
        outputArray[outCount] = 1;
        outCount++;
      }
        consecutiveCount = 0;
      }
      
      
  }

  for (int k = 0 ; k < 9 ; k++){
    Serial.print(outputArray[k]);
    barcodeNumber = barcodeNumber + outputArray[k] * (1 << (8 - k));

    
  }
  return (barcodeNumber % 5) ;

}

void decodeBarcode(){
  colour = 0 ;
    irReader.setColour(colour);

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
        if(detectBend() != 0){
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
    turnBend(0);
    break ;
    case -1 :
    turnBend(1);
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
}