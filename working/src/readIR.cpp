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
    int sensorThreshold = 600;
    irValues_str = "";
    for (int i = 0; i < 12; i++) {
        sen[i] = (sensors[i] < sensorThreshold) ? 1 : 0;
        irValues_str += sensors[i];
        irValues_str += " ";
    }} else if(colour == 1) {
    int sensorThreshold = 350;
    irValues_str = "";
    for (int i = 0; i < 12; i++) {
        sen[i] = (sensors[i] > sensorThreshold) ? 1 : 0;
        irValues_str += sensors[i];
        irValues_str += " ";
    }}
}
