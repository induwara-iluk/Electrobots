#include "VL53L0X_MultiSensor.h"

VL53L0X_MultiSensor::VL53L0X_MultiSensor(int* xshutPins, uint8_t* i2cAddresses, int numSensors)
    : xshutPins(xshutPins), i2cAddresses(i2cAddresses), numSensors(numSensors) {
    sensors = new Adafruit_VL53L0X[numSensors];
    distances = new int[numSensors];
}

void VL53L0X_MultiSensor::begin() {
    for (int i = 0; i < numSensors; i++) {
        pinMode(xshutPins[i], OUTPUT);
        digitalWrite(xshutPins[i], LOW);
    }
    delay(10);

    for (int i = 0; i < numSensors; i++) {
        initializeSensor(i);
    }
}

void VL53L0X_MultiSensor::initializeSensor(int index) {
    digitalWrite(xshutPins[index], HIGH);
    delay(10);
    if (!sensors[index].begin(i2cAddresses[index])) {
        Serial.print("Failed to initialize sensor ");
        Serial.println(index + 1);
        while (1);
    }
    Serial.print("Sensor ");
    Serial.print(index + 1);
    Serial.println(" initialized");
}

void VL53L0X_MultiSensor::readDistances() {
    for (int i = 0; i < numSensors; i++) {
        VL53L0X_RangingMeasurementData_t measure;
        sensors[i].rangingTest(&measure, false);
        distances[i] = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;
    }
}

int* VL53L0X_MultiSensor::getDistances() {
    return distances; 
}
