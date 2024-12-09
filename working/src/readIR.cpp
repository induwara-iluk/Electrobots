#include "readIR.h"

void readIR::readSensors(int sensors[]) {
    
    for (int i = 0; i < 12; i++) {
        sensors[i] = analogRead(A15-i);
    }
}

void readIR::convertSensorsToBinary(int sensors[], int sen[]) {
    int sensorThreshold = 600;
    irValues_str = "";
    for (int i = 0; i < 12; i++) {
        sen[i] = (sensors[i] < sensorThreshold) ? 1 : 0;
        irValues_str += sensors[i];
        irValues_str += " ";
    }
}
