/***************************************************************************
  This sketch makes a Fine Offset WH1080 compatible remote sensor unit out
  of an ESP32, a professional Thies anemometer and a BMP280 sensor board.
  Refer to https://blixemboschweer.nl/ for more information on the used hardware.

  This file handles the Thies anemometer that I used in my project (type
  3323.21.002).

  This file was written by Tsjakka from The Netherlands.
  MIT license, this comment block must be included in any redistribution.
 ***************************************************************************/
#ifndef _THIES_H
#define _THIES_H

#include "Arduino.h"

const int NUM_SPEED_SAMPLES = 192;              // Number of samples to store for wind speed (one per 0.25s for the last 48 seconds)
const int NUM_GUST_SAMPLES = 12;                // Number of samples to use for wind gust (one per 0.25s for the last 3 seconds)
const int NUM_DIR_SAMPLES = 48;                 // Number of samples to store for wind direction (one per second for the last 48 seconds)

class Thies {
  public:
    void initialize();                          // Start monitoring interrupts from anemometer
    void sampleWindSpeed();                     // Run this function every 0.25s to store a wind speed sample
    float getWindSpeed();                       // Get the average wind speed over the last 10 minutes
    float getWindGust();                        // Get the maximum wind gust since the last call to this method
    void resetWindGust();                       // Reset the wind gust
    int sampleWindDirection();                  // Run this function every second to store a wind direction sample
    byte getWindDirection();                    // Get the average wind direction over the last 10 minutes (0 to 15 for N, NNE, NE, ENE, E, etc.)
    void resetVars();                           // Reset the running variables to prevent faulty data to propagate over time
    void unitTest();                            // Test the methods above

  private:
    int speedPulses[NUM_SPEED_SAMPLES];         // A circular buffer for storing wind pulses
    int speedPulsesIndex = 0;                   // Index of the current element in the array above
    int speedPulsesCount = 0;                   // Number of elements that have a value in the array above (useful at startup)
    long speedPulsesSum = 0L;                   // The sum of all values in the array above
    int gustPulses[NUM_GUST_SAMPLES];           // A circular buffer for storing wind pulses
    int gustPulsesIndex = 0;                    // Index of the current element in the array above
    int gustPulsesCount = 0;                    // Number of elements that have a value in the array above (useful at startup)
    long gustPulsesSum = 0L;                    // The sum of all values in the array above
    float gust;                                 // The gust in m/s
    short dirValues[NUM_DIR_SAMPLES];           // A circular buffer for storing wind direction
    int dirValuesIndex = 0;                     // Index of the current element in the array above
    int dirValuesCount = 0;                     // Number of elements that have a value in the array above (useful at startup)
    long dirValuesSum = 0L;                     // The sum of all values in the array above
    int dirValuePrevious = 0;                   // The previous sample for wind direction

    short calculateWindDirection(int analogValue);  // Returns a value from 0 to 15 depending on the input value
};

#endif
