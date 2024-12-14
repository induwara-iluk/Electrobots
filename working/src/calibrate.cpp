#include <Arduino.h>
#include <calibrate.h>
#include <MotorControl.h>

int history[12][100];  // 2D array to store sensor readings for each sensor

// Function to find the threshold for a single sensor
int findThreshold(int readings[], int size,int mode) {
    // Step 1: Sort the readings array
    for (int i = 0; i < size - 1; i++) {
        for (int j = 0; j < size - i - 1; j++) {
            if (readings[j] > readings[j + 1]) {
                int temp = readings[j];
                readings[j] = readings[j + 1];
                readings[j + 1] = temp;
            }
        }
    }

    // Step 2: Take the middle value between the 50th and 750th values
    if(mode == 0){int threshold = readings[0] ;

    return threshold - 100;}
    else{
    int threshold = readings[95];

    return threshold + 100; 
}}

// Function to calibrate all sensors and update threshold array
void calibrate(int thresholds[12],int mode) {
    int numReadings = 100;

    // Collect sensor readings for each sensor
    for (int sensorIdx = 0; sensorIdx < 12; sensorIdx++) {
        for (int j = 0; j < numReadings; j++) {
            Serial.println(analogRead(A15-sensorIdx));
            history[sensorIdx][j] = analogRead(A15-sensorIdx);
            delay(2);
        }

        // Find the threshold for the current sensor
        thresholds[sensorIdx] = findThreshold(history[sensorIdx], numReadings,mode);

        // Print the threshold for the current sensor
        Serial.print("Sensor ");
        Serial.print(sensorIdx);
        Serial.print(" Threshold: ");
        Serial.println(thresholds[sensorIdx]);
    }
}