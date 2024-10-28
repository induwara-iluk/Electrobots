#include "TCSColorSensor.h"

TCSColorSensor::TCSColorSensor() 
  : tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X),
    red(0), green(0), blue(0), clear(0), detectedColor("Unclear/Other") {}

bool TCSColorSensor::begin() {
  return tcs.begin();
}

void TCSColorSensor::readColor() {
  tcs.getRawData(&red, &green, &blue, &clear);
  
  if (red > green && red > blue) {
    detectedColor = "Red";
  } else if (green > red && green > blue) {
    detectedColor = "Green";
  } else if (blue > red && blue > green) {
    detectedColor = "Blue";
  } else {
    detectedColor = "Unclear/Other";
  }
}

String TCSColorSensor::getDetectedColor() {
  return detectedColor;
}
