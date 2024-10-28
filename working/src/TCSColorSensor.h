#ifndef TCSColorSensor_h
#define TCSColorSensor_h

#include <Arduino.h>
#include <Adafruit_TCS34725.h>

class TCSColorSensor {
public:
  TCSColorSensor();
  bool begin();
  void readColor();
  String getDetectedColor();

private:
  Adafruit_TCS34725 tcs;
  uint16_t red, green, blue, clear;
  String detectedColor;
};

#endif
