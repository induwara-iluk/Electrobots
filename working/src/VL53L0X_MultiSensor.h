#ifndef VL53L0X_MULTISENSOR_H
#define VL53L0X_MULTISENSOR_H

#include <Adafruit_VL53L0X.h>


class VL53L0X_MultiSensor {
public:
    VL53L0X_MultiSensor(int* xshutPins, uint8_t* i2cAddresses, int numSensors);
    void begin();
    void readDistances();
    int* getDistances();

private:
    Adafruit_VL53L0X* sensors;
    int* xshutPins;
    uint8_t* i2cAddresses;
    int numSensors;
    int* distances;
    

    void initializeSensor(int index);
};

#endif // VL53L0X_MULTISENSOR_H
