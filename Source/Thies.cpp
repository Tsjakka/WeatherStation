/***************************************************************************
  This sketch makes a Fine Offset WH1080 compatible remote sensor unit out
  of an ESP32, a professional Thies anemometer and a BMP280 sensor board.
  Refer to https://blixemboschweer.nl/ for more information on the used hardware.

  This file handles the Thies anemometer that I used in my project (type
  3323.21.002).

  Written by Tsjakka from The Netherlands.
  MIT license, this comment block must be included in any redistribution.
 ***************************************************************************/
#include "Thies.h"

const byte windDirPin = 13;               // The analog input used for wind direction
const byte windSpeedPin = 14;             // The interrupt pin used for the wind speed
volatile int windInterruptCounter = 0;    // Number of interrupts generated by the wind speed sensor
const int pulsesOneMeterPerSecond = 20;   // The wind speed meter generates 20 pulses per second when the wind speed is 1 m/s

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Handle pulses from the wind speed sensor
void IRAM_ATTR handleWindInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
  windInterruptCounter++;
  portEXIT_CRITICAL_ISR(&mux);
}

void Thies::initialize() {
  pinMode(windDirPin, INPUT);
  
  Serial.println("Monitoring interrupts from anemometer");
  pinMode(windSpeedPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(windSpeedPin), handleWindInterrupt, RISING);

  // Start with zeroed arrays
  resetVars();
}

/*
  The World Meteorological Organization (WMO) on wind speed measurement: [Use] a pulse generator that
  generates pulses at a frequency proportional to the rotation rate of the anemometer (preferably several
  pulses per rotation), a counting device that counts the pulses at intervals of 0.25 s, and a microprocessor
  that computes averages and standard deviation over 10 min intervals on the basis of 0.25 s samples. The
  extreme (=gust) has to be determined from 3 s averages, namely, by averaging over the last 12 samples.
  This averaging has to be done every 0.25 s (namely, overlapping 3 s averages every 0.25 s).
  See also:
  https://www.wmo.int/pages/prog/www/IMOP/publications/CIMO-Guide/Prelim_2018_ed/8_I_5_en_MR_clean.pdf,
  section 5.8.3.

  The WH1080 base station requires a wind speed measurement every 48s. I assume the base station will
  average those measurements further when storing the historical date in its memory every 10 minutes or so.
  For the wind speed this means the base station will average the speed over the measurements from the last
  10 minutes. For the gust it means taking the maximum of those measurements. On this side it means we have
  to reset the gust every 48s.

  The Thies anemometer gives 800 Hz pulses at its maximum wind speed of 144 km/u or 40 m/s (20 Hz = 1 m/s).
*/

// Samples the number of pulses generated by the wind speed sensor since the last call to this method. 
// Run this function every 0.25 s.
void Thies::sampleWindSpeed() {
  int windPulses = 0;

  // Keep the interrupt count and restart measuring
  portENTER_CRITICAL(&mux);
  windPulses = windInterruptCounter;
  windInterruptCounter = 0;
  portEXIT_CRITICAL(&mux);

  int removePulses = speedPulses[speedPulsesIndex];

  // Store new sample in array
  speedPulses[speedPulsesIndex] = windPulses;

  // Calculate new sum for calculating average speed. I keep a running sum so I don't have
  // to add up all values in the array every time I need the sum.
  speedPulsesSum += windPulses - removePulses;

  // Calculate new sum used in gust calculation.
  // First check if we have enough samples to remove an old value.
  if (speedPulsesCount >= NUM_GUST_SAMPLES) {
    int gustIndexToBeRemoved = speedPulsesIndex - NUM_GUST_SAMPLES;

    // Check for the 'roll over' of the array
    if (gustIndexToBeRemoved < 0) {
      gustIndexToBeRemoved = NUM_SPEED_SAMPLES - NUM_GUST_SAMPLES + speedPulsesIndex;
    }
    int removeGustPulses = speedPulses[gustIndexToBeRemoved];
    gustPulsesSum += windPulses - removeGustPulses;
  }
  else {
    gustPulsesSum += windPulses;
  }

  // Calculate gust over last 12 samples (3s)
  float tempGust = ((float)gustPulsesSum) / 60.0;  // 1 m/s equals 20 pulses per second for 3 seconds
  if (tempGust > gust) {
    gust = tempGust;
  }

  // Update index and count
  speedPulsesIndex++;
  if (speedPulsesIndex >= NUM_SPEED_SAMPLES) speedPulsesIndex = 0;
  if (speedPulsesCount < NUM_SPEED_SAMPLES) speedPulsesCount++;
}

// Return the current average of the wind speed samples
float Thies::getWindSpeed() {
  float speed = (float)speedPulsesSum;
  speed = speed / (speedPulsesCount * 5); // 1 m/s equals 20 pulses per second (= 5 pulses per 0.25s)

  return speed;
}

// Return the current gust value
float Thies::getWindGust() {
  return gust;
}

// Reset the current gust value
void Thies::resetWindGust() {
  gust = 0;
}

int Thies::sampleWindDirection() {
  int sample = analogRead(windDirPin);
  short direction = calculateWindDirection(sample);

  // Code for calibrating the wind direction
  //Serial.print(sample);
  //Serial.print(" ");

  // Store the sample while checking successive samples for continuity (see WMO document).
  // If two successive samples differ by more than 180 degrees (= 8), the difference
  // is decreased by adding or subtracting 360 degrees (= 16) to/from the second sample.
  // Because of this, samples can be negative or larger than 15.
  if (dirValuePrevious != 999) {
    if (abs(direction - dirValuePrevious) > 8) {
      if (abs(direction - 16 - dirValuePrevious) < abs(direction + 16 - dirValuePrevious)) {
        direction -= 16;
      }
      else {
        direction += 16;
      }
    }
  }

  //Serial.print(direction);
  //Serial.print(", ");
  
  // Calculate new sum for calculating average direction. I keep a running sum so I don't
  // have to add up all values in the array every time I need the sum.
  dirValuesSum += direction - dirValues[dirValuesIndex];

  // Store new sample in array
  dirValues[dirValuesIndex] = direction;

  // Save the direction for the next time
  dirValuePrevious = direction;

  dirValuesIndex++;
  if (dirValuesIndex >= NUM_DIR_SAMPLES) {
    dirValuesIndex = 0;
  }

  if (dirValuesCount < NUM_DIR_SAMPLES) {
    dirValuesCount++;
  }

  return sample;
}

// Returns a value from 0 to 15 for N, NNE, NE, ENE, E, etc.
// The values below are the values measured on my own anemometer.
short Thies::calculateWindDirection(int analogValue) {
  short result = 0;

  if      (                       analogValue <   69) result = 0;
  else if (analogValue >=   69 && analogValue <  239) result = 15;
  else if (analogValue >=  239 && analogValue <  460) result = 14;
  else if (analogValue >=  460 && analogValue <  681) result = 13;
  else if (analogValue >=  681 && analogValue <  865) result = 12;
  else if (analogValue >=  865 && analogValue < 1056) result = 11;
  else if (analogValue >= 1056 && analogValue < 1225) result = 10;
  else if (analogValue >= 1225 && analogValue < 1375) result = 9;
  else if (analogValue >= 1375 && analogValue < 1531) result = 8;
  else if (analogValue >= 1531 && analogValue < 1665) result = 7;
  else if (analogValue >= 1665 && analogValue < 1800) result = 6;
  else if (analogValue >= 1800 && analogValue < 1919) result = 5;
  else if (analogValue >= 1919 && analogValue < 2019) result = 4;
  else if (analogValue >= 2019 && analogValue < 2125) result = 3;
  else if (analogValue >= 2125 && analogValue < 2246) result = 2;
  else if (analogValue >= 2246 && analogValue < 2346) result = 1;
  else if (analogValue >= 2346                      ) result = 0;

  return result;
}

// Return the current average of the wind direction samples
byte Thies::getWindDirection() {
  // Use this code if there is a problem with the running sum.
  //long dirSum = 0;
  //for (int i = 0; i < NUM_DIR_SAMPLES; i++) {
  //  dirSum += dirValues[i];
  //}
  //int direction = round(dirSum / dirValuesCount);
  
  float average = dirValuesSum / dirValuesCount;
  int direction = round(average);

  // Push it within the bounds of the weather station (0..15)
  direction = direction % 16;
  if (direction < 0) direction += 16;

  //printf("Sum of dir values: %ld, count: %d, average: %0.3f, running sum: %ld\n", dirSum, dirValuesCount, average, dirValuesSum);
      
  return (byte)direction;
}

// Reset the running sums, so a bad value doesn't propagate over longer periods.
void Thies::resetVars() {
  Serial.println("Resetting wind variables");
  speedPulsesIndex = 0;
  speedPulsesCount = 0;
  speedPulsesSum = 0;
  gustPulsesIndex = 0;
  gustPulsesCount = 0;
  gustPulsesSum = 0;
  dirValuesIndex = 0;
  dirValuesCount = 0;
  dirValuesSum = 0;
  dirValuePrevious = 999;   // 999 means uninitialized

  // Initialize the arrays to zeros
  for (int i = 0; i < NUM_SPEED_SAMPLES; i++) {
    speedPulses[i] = 0;
  }
  for (int i = 0; i < NUM_GUST_SAMPLES; i++) {
    gustPulses[i] = 0;
  }
  for (int i = 0; i < NUM_DIR_SAMPLES; i++) {
    dirValues[i] = 0;
  }  
}

// Test this class by filling in sample data and printing the resulting values.
// Do not call initialize beforehand, so interrupts will not disturb this method.
void Thies::unitTest() {
  Serial.println("i count speed gust");
  Serial.println();

  for (int i = 0; i < NUM_SPEED_SAMPLES + 20; i++) {
    windInterruptCounter = i % 10 + 1;

    Serial.print(i);
    Serial.print(" ");
    Serial.print(windInterruptCounter);
    Serial.print(" ");

    sampleWindSpeed();
    float speed = getWindSpeed();
    Serial.print(speed);
    Serial.print(" ");
    Serial.println(gust);

    // Reset the gust every 48s
    //if (i % 192 == 0) getWindGust();
  }
}
