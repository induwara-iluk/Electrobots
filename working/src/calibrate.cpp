#include <Arduino.h>
#include <calibrate.h>
#include <MotorControl.h>

int history[800];
int threshold = 0;  // Variable to store the threshold value

int calibrate(int sensors[8]) {

    // Set motor speeds to rotate the robot while taking readings
    // Adjust speeds as needed for rotation

    // Populate history array with sensor readings
   
    for (int j = 0; j < 800; j++) {
        int i = j % 8;
        history[j] = analogRead(A0 + i);
        Serial.print(history[j]);
        Serial.print(" ");
    }
    Serial.println();

    // Stop the motors after readings are complete

    // Sort the history array
    for (int i = 0; i < 800 - 1; i++) {
        for (int j = i + 1; j < 800; j++) {
            if (history[i] > history[j]) {
                int temp = history[i];
                history[i] = history[j];
                history[j] = temp;
            }
        }
    }

    // Find the value at the 25th percentile
    int quarterValue = history[200];

    // Print the quarter value
    Serial.print("Quarter value: ");
    Serial.println(quarterValue);

    // Return 100 less than the quarter value
    return quarterValue - 100;
}
