#ifndef READ_IR_H
#define READ_IR_H

#include <Arduino.h>

class readIR {
public:
    
    void readSensors(int sensors[]);
    void convertSensorsToBinary(int sensors[], int sen[]);

private:
    
};

extern String irValues_str;;

#endif
