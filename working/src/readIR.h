#ifndef READ_IR_H
#define READ_IR_H

#include <Arduino.h>

// Declare irValues_str as an extern global variable
extern String irValues_str;
extern int whiteLineThresholds[12];
extern int blackLineThresholds[12];
class readIR {
public:
    // Public methods
    void readSensors(int sensors[]);
    void convertSensorsToBinary(int sensors[], int sen[]);

    // Method to set the colour value
    void setColour(int value);

private:
    // Private member variable for colour
    int colour = 0;
};

#endif
