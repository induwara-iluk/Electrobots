#include "readIR.h"

void readIR::readSensors(int sensors[]) {
    
    for (int i = 0; i < 12; i++) {
        sensors[i] = analogRead(A15-i);
    }
}

void readIR::setColour(int value) {
    colour = value;  // Assign the value to the private member
}

void readIR::convertSensorsToBinary(int sensors[], int sen[]) {

   if(colour == 0) {
    irValues_str = "";
    for (int i = 0; i < 12; i++) {
        sen[i] = (sensors[i] < whiteLineThresholds[i] ) ? 1 : 0;
        irValues_str += sensors[i];
        irValues_str += " ";
    }} 
    else if(colour == 1) {
    irValues_str = "";
    for (int i = 0; i < 12; i++) {
        sen[i] = (sensors[i] > blackLineThresholds[i]) ? 1 : 0;
        irValues_str += sensors[i];
        irValues_str += " ";
    }} 
}
