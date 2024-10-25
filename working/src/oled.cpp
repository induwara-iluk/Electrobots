#include "oled.h"

// Constructor that initializes the display with given width, height, and address
oled::oled(uint8_t screenWidth, uint8_t screenHeight, uint8_t i2cAddress)
    : display(screenWidth, screenHeight, &Wire, -1)  // Reset pin set to -1 (not used)
{
}

void oled::begin() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Initialize with address 0x3C
        Serial.println(F("SSD1306 allocation failed"));
        while (1);  // Halt if the display fails to initialize
    }
    display.clearDisplay();
    
}

void oled::displayText(const String& text, uint8_t textSize, uint8_t x, uint8_t y) {
    display.clearDisplay();
    display.setTextSize(textSize);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(x, y);
    display.println(text);
    display.display();
}
