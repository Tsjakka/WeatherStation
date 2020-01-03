/***************************************************************************
  This sketch makes a Fine Offset WH1080 compatible remote sensor unit out
  of an ESP32, a professional Thies anemometer and a BMP280 sensor board.
  Refer to http://blixemboschweer.nl/ for more information on the used hardware.

  Starting point for this file was the file with the same name written for
  the LaCrosseITPlusReader project (https://github.com/rinie/LaCrosseITPlusReader).

  This file was written by Tsjakka from The Netherlands.
  MIT license, this comment block must be included in any redistribution.
 ***************************************************************************/
#ifndef _WH1080_h
#define _WH1080_h

#include "Arduino.h"
#include "SensorBase.h"

//WH1080 V2 protocol defines
#define MSG_WS4000 1
#define MSG_WS3000 2
#define LEN_WS4000 10
#define LEN_WS3000 9
#define LEN_MAX 10

// This class takes weather data from several sensors and builds WH1080 specific data packets 
// from it. It can then send these packets to the WH1080 base station.
// Also, it can send DCF time data to the base station.
class WH1080 : public SensorBase {
public:
  byte deviceID = 0;

  // Variables for weather measurement
  struct WeatherData {
	  byte deviceID = 0;
	  float temperature = 0;
	  float humidity = 0;
	  float windSpeed = 0;
	  float windGust = 0;
	  byte windDir = 0;
	  unsigned short rain = 0;
  };
  
  // Variables for handling the DCF77 time signal
  int hours = 0;
  int minutes = 0;
  int seconds = 0;
  int years = 0;
  int months = 0;
  int days = 0;

  void initialize();                          // Start monitoring interrupts from anemometer
  unsigned short readRain();                  // Return the number of pulses from the rain gauge since the last reset
  void sampleRainGauge();                     // Check for tipping of the rain bucket
  void resetRain();                           // Rain needs to be reset every 24h
  void setSensorData(struct WH1080::WeatherData* frame);  // Supply weather data to this class
  void transmitSensorData();                  // Send the weather data to the WH1080 base station
  void enableDcfReceiver();                   // Turn the DCF receiver on
  bool updateTime();                          // Update the controller's internal clock with info received through DCF77. Return true if new time available.
  void disableDcfReceiver();                  // Turn the DCF receiver off
  void transmitTimeData();                    // Send the DCF time data to the WH1080 base station
  void frequencyTest();
  void sensorDataUnitTest();                  // Test the code for generating, handling and sending weather data
  void timeDataUnitTest();                    // Test the DCF code by sending a predefined time to the base station
  void printAllRfmRegs();                     // Print all communication parameters for the RFM69
  bool rfmIsValid();                          // Checks if the RFM chip has the right frequency
  void resetRfm();                            // Reset the RFM by triggering the reset line
  void initializeRfm();                       // Set the communication parameters

private:
  WeatherData weatherData;

  // Methods added for the WH1080 remote sensor project
  void fillSensorBuffer(byte *buf);
  void sendWh1080Message(byte *message, byte length);
  byte getLeftShiftMask(int shift);
  void shiftLeft(byte *buf, int start, int msg_len, int shift);
  void fillTimeBuffer(byte *buf);

  // Methods from the LaCrosseITPlusReader project
  static const byte FRAME_LENGTH = LEN_WS4000;
  void updateTime(byte *tbuf);
  void timestamp(bool includeDate = false);
};

// Helper functions
void printDigits(int digits);
uint8_t bin2BCD(int bin);
int BCD2bin(uint8_t BCD);
void printDouble(double val, byte precision=1);
uint8_t crc8(uint8_t *addr, uint8_t len);

#endif
