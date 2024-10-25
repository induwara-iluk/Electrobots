// MyOLED.h
#ifndef OLED_H
#define OLED_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

class oled {
public:
    oled(uint8_t screenWidth = 128, uint8_t screenHeight = 64, uint8_t i2cAddress = 0x3C);
    void begin();
    void displayText(const String& text, uint8_t textSize = 2, uint8_t x = 0, uint8_t y = 10);
  
private:
    Adafruit_SSD1306 display;
};

#endif
