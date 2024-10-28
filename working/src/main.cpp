#include <Arduino.h>

#include <lineFollow.h>
#include <oled.h>
#include <TCSColorSensor.h>

TCSColorSensor colorSensor;
oled oledDisplay;

int sensors[8];
int sen[8];

const int B_L = 8;
const int FL = 9;
const int PWML = 10;

const int B_R = 5;
const int FR = 6;
const int PWMR = 7;

const int btn1 = 18;
const int btn2 = 19;


const int leftIR = A8;
const int rightIR = A9;
const int frontIR = A10;

int lastError = 0;
int integral = 0;

float Kp = 2.8;   // Proportional gain
float Ki = 0.0; // Integral gain
float Kd = 1.5;  // Derivative gain
int baseSpeed = 65 ;  // Base speed for the motors

int stage = 1; 
const int historySize = 10; // Size of the history buffer
int senHistory[historySize][8]; // History array to store past sensor states
int historyIndex = 0; // Current index for appending in the history array

int sensorThresholds = 300; // Threshold to distinguish black and white

const int R_encoder_A = 2;

const int L_encoder_A = 3;




// Variables for encoder counts
volatile long R_encoder_ticks = 0;
volatile long L_encoder_ticks = 0;

const float wheel_diameter = 0.067; // Diameter of the wheels in meters (e.g., 6.5 cm)
int pulses_per_revolution = 222;

// Variables for speed calculation
long previous_ticks_R = 0;
long previous_ticks_L = 0;



void initializePins() {
  for (int i = A0; i <= A10; i++) {
    pinMode(i, INPUT);
  }
  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
  pinMode(R_encoder_A, INPUT);
  pinMode(L_encoder_A, INPUT);

  pinMode(B_L, OUTPUT);
  pinMode(FL, OUTPUT);
  pinMode(B_R, OUTPUT);
  pinMode(FR, OUTPUT);
  pinMode(PWMR, OUTPUT);
  pinMode(PWML, OUTPUT);
}

// Encoder counting functions
void countR() {
    R_encoder_ticks++;
}

void countL() {
    L_encoder_ticks++;
}


void readSensors(int sensors[]) {
  String irValues_str = "";
  for (int i = 0; i < 8; i++) {
    sensors[i] = analogRead(A0 + i);
    Serial.print(sensors[i]);
    Serial.print(" ");
    irValues_str+= sensors[i];  
    irValues_str+=" ";
    
  }
  //oledDisplay.displayText(irValues_str,1,0,0);
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

void convertSensorsToBinary(int sensors[], int sen[]) {
  for (int i = 0; i < 8; i++) {
    sen[i] = (sensors[i] < sensorThresholds) ? 1 : 0;
  }
}

void moveDistance(float distance) {

    // Calculate the number of pulses required for the given distance
    long pulses_needed = (distance / (PI * wheel_diameter)) * pulses_per_revolution;

    // Reset pulse counts
    R_encoder_ticks = 0;
    L_encoder_ticks = 0;

    
    // Wait until the required pulses are counted
    while (R_encoder_ticks < pulses_needed && L_encoder_ticks < pulses_needed) {
      int sensors[8];
      readSensors(sensors);

      int sen[8];
      convertSensorsToBinary(sensors, sen);
      processLineFollowing(sen);
    }
}



void turnBendIR(byte direction){
  moveDistance(0.04);
  // 0 = left , 1 = right
  while (direction == 0){
    oledDisplay.displayText("left turn",1,0,0);  
   
    setMotorSpeed(-100,70);

    if ((analogRead(A3) < sensorThresholds) && (analogRead(A4) < sensorThresholds &&  colorSensor.getDetectedColor() == "Red" )){
       return;
     }
  }

  while (direction == 1){
    oledDisplay.displayText("right turn",1,0,0);  

    setMotorSpeed(70,-65);
    
   if ((analogRead(A3) < sensorThresholds) && (analogRead(A4) < sensorThresholds)){
       return;
   }
  }

}




void turnBend(byte direction){

  String colour_prv = colorSensor.getDetectedColor();
  moveDistance(0.04);
  if (analogRead(frontIR)< sensorThresholds){
    oledDisplay.displayText("T bend or Cross",1,0,0); 
    
    return;
  }

  // 0 = left , 1 = right
  while (direction == 0){
    oledDisplay.displayText("left turn",1,0,0);  
    oledDisplay.displayText(String(L_encoder_ticks) + " " + String(R_encoder_ticks),1,0,50); 
    setMotorSpeed(-100,70);

    if (R_encoder_ticks > 160 && L_encoder_ticks  > 90){
      L_encoder_ticks = 0;
      R_encoder_ticks = 0;
      return;
    }
    // if ((analogRead(A3) < sensorThresholds) && (analogRead(A4) < sensorThresholds &&  colorSensor.getDetectedColor() == "Red" )){
    //   return;
    // }
  }

  while (direction == 1){
    oledDisplay.displayText("right turn",1,0,0);  
    oledDisplay.displayText(String(L_encoder_ticks) + " " + String(R_encoder_ticks),1,0,50); 
    setMotorSpeed(70,-65);
    if (R_encoder_ticks > 90 && L_encoder_ticks  > 110){
      L_encoder_ticks = 0;
      R_encoder_ticks = 0;
      return;
    }
    // if ((analogRead(A3) < sensorThresholds) && (analogRead(A4) < sensorThresholds)){
    //   return;
    // }
  }

}



void updateSensorHistory(int sen[]) {
  for (int i = 0; i < 8; i++) {
    senHistory[historyIndex][i] = sen[i];
  }
  historyIndex = (historyIndex + 1) % historySize; // Circular buffer
}



void stopRobot() {
  analogWrite(PWML, 0);
  analogWrite(PWMR, 0);

  digitalWrite(FL, LOW);
  digitalWrite(B_L, LOW);

  digitalWrite(FR, LOW);
  digitalWrite(B_R, LOW);
  
}

int detectBend(){
  if((!digitalRead(leftIR) || sen[0])&& (sen[7] || !digitalRead(rightIR))){
    return 2 ;
  } 
  if (sen[0] && sen[1] && sen[2] && !digitalRead(leftIR)) {
    return 0;  // Left bend detected
  } else if (sen[7] && sen[6] && sen[5] && !digitalRead(rightIR)) {  // Adjust indices as needed
    return 1;  // Right bend detected
  } else {
    return -1; // No bend detected
  }
}


void turnTicks(int direction , int given_ticks = 265 ){
  
  
    
    if (direction == 1){
      while(L_encoder_ticks < given_ticks)
      {
      setMotorSpeed(100,0);
      }
      stopRobot();
      delay(100);
      }
    if(direction == 0){
      while(R_encoder_ticks < given_ticks){
      setMotorSpeed(0,100);
      }
      stopRobot();
      delay(100);
      }
    if(direction == 2 ){
      stopRobot();
      delay(1000);
    }
    
  
      
      
  
}

void kUp() {
    Kd -=0.1;
}

void kDown() {
    Kp -=0.1;
}



void setup() {
  initializePins();
  Serial.begin(9600);

  if (colorSensor.begin()) {
    Serial.println("TCS34725 found");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  attachInterrupt(digitalPinToInterrupt(btn1), kUp, FALLING);
  attachInterrupt(digitalPinToInterrupt(btn2), kDown, FALLING);

  attachInterrupt(digitalPinToInterrupt(R_encoder_A), countR, RISING);
  attachInterrupt(digitalPinToInterrupt(L_encoder_A), countL, RISING);

  oledDisplay.begin(); 
  oledDisplay.displayText("ElectroBots",1,0,0);  
  delay(200);
}


void loop() {

  //oledDisplay.displayText("Kp :" + String(Kp) + "    Kd :" + String(Kd) ,1,0,30);

  

 // oledDisplay.displayText(String(L_encoder_ticks) + " " + String(R_encoder_ticks),1,0,50);
  

  //colorSensor.readColor();
  //oledDisplay.displayText(colorSensor.getDetectedColor(),1,0,20);
 
  readSensors(sensors);
  convertSensorsToBinary(sensors, sen);
  // updateSensorHistory(sen); // Update sensor history

  if (allSensorsDetectBlack(sensors)) {
     stopRobot();
     return;
   }  
 /*

  if (analogRead(leftIR)< sensorThresholds && analogRead(rightIR)> sensorThresholds ){   
      L_encoder_ticks = 0;
      R_encoder_ticks = 0;
      turnBendIR(0); // left bend
    }

  if (analogRead(rightIR)< sensorThresholds && analogRead(leftIR)> sensorThresholds){
      L_encoder_ticks = 0;
      R_encoder_ticks = 0;
      turnBendIR(1);  //right bend 
    }


  */
  
  
   int direction = detectBend();
   oledDisplay.displayText(String(direction));
   L_encoder_ticks=0;
   R_encoder_ticks=0;
  turnTicks(direction);
  processLineFollowing(sen);

  
}

