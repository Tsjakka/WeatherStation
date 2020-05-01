/***************************************************************************
  This sketch makes a Fine Offset WH1080 compatible remote sensor unit out
  of an ESP32, a professional Thies anemometer and a BMP280 sensor board.
  Refer to http://blixemboschweer.nl/ for more information on the used hardware.

  Starting point for this file was the file with the same name written for
  the LaCrosseITPlusReader project (https://github.com/rinie/LaCrosseITPlusReader).

  This file was written by Tsjakka from The Netherlands.
  MIT license, this comment block must be included in any redistribution.
 ***************************************************************************/
#define USE_SPI_H

#include <TimeLib.h>
#include "WH1080.h"
#include "RFMxx.h"
#include "DCF77.h"
#ifdef USE_SPI_H
#include <SPI.h>
#endif

const byte rainSignalPin = 27;            // The pin used for the rain gauge
const byte dcfInterruptPin = 35;          // The interrupt pin used for the DCF device
const byte dcfEnablePin = 33;             // The pin used for switching the DCF device on or off

#ifndef USE_SPI_H
RFMxx rfm(23, 19, 18, 5, 17, 32);
#else
RFMxx rfm(SS, 17, 32);                    // The class that takes care of communicating through the RFM69WH
#endif

volatile short rainDetectedCounter = 0;   // Number of buckets tips counted
volatile short rainRejectedCounter = 0;   // Number of rejected inputs from the rain sensor
volatile unsigned long rainSignalTime = 0;// The moment that a signal was measured on the rain pin
volatile bool rainSignalDetected = false; // True if a signal was measured on the rain pin
volatile bool rainSignalHandled = true;   // True if the rain signal was handled
volatile int rainSample;                  // The value measured on the rain pin

//portMUX_TYPE rainMux = portMUX_INITIALIZER_UNLOCKED;

DCF77 dcf = DCF77(dcfInterruptPin, digitalPinToInterrupt(dcfInterruptPin), true);  // The class that takes care of handling DCF clock messages

// Handle interrupts from the rain sensor.
//void IRAM_ATTR handleRainInterrupt() {
//  portENTER_CRITICAL_ISR(&rainMux);
//  rainInterruptTime = millis();
//  rainInterruptDetected = true;
//  portEXIT_CRITICAL_ISR(&rainMux);
//}

void WH1080::initialize() {
  // Initialize rain gauge
  Serial.println("Monitoring interrupts from rain gauge");
  pinMode(rainSignalPin, INPUT_PULLUP);  
  //attachInterrupt(digitalPinToInterrupt(rainSignalPin), handlerainSignal, FALLING);

  // Initialize DCF
  pinMode(dcfEnablePin, OUTPUT);
  Serial.println("Starting DCF receiver");
  enableDcfReceiver();
  dcf.Start();

  // Initialize RFM69
  Serial.print("Intializing RFM69H");
  rfm.Init();
  rfm.InitializeForWH1080();
  
  Serial.print(", radio: ");
  Serial.print(rfm.GetRadioName());
  Serial.print(" data rate ");
  Serial.print(rfm.GetDataRate());
  Serial.print(" @ ");
  Serial.print(rfm.GetFrequency());
  Serial.println(" kHz");
  
  // If an analog input pin is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(25));

  // Make up a device ID
  deviceID = (byte)random(1, 256);
}

// Return the amount of rain (in mm) that fell since startup / reset.
unsigned short WH1080::readRain() {
  unsigned short rainPulses = 0;

  // Keep the interrupt count and restart measuring
  //portENTER_CRITICAL(&rainMux);
  rainPulses = rainDetectedCounter;
  //portEXIT_CRITICAL(&rainMux);

  return rainPulses;
}

void WH1080::sampleRainGauge() {
  //bool interruptDetected = false;

  // Check if an interrupt was detected
//  portENTER_CRITICAL(&rainMux);
//  interruptDetected = rainInterruptDetected;
//  portEXIT_CRITICAL(&rainMux);

  if (!rainSignalDetected && rainSignalHandled) {
    rainSample = digitalRead(rainSignalPin);
    if (rainSample == LOW) {
      rainSignalTime = millis();
      rainSignalDetected = true;
      rainSignalHandled = false;
    }
  }

  // If a signal was measured from the bucket, check if the signal resembles 
  // the signal of the tip of a bucket.
  // A tip of the bucket in the rain gauge (the orignal one of the Alecto
  // weather station) pulls the input to zero for slightly less than 100ms.
  if (!rainSignalHandled) {
    unsigned long currentTime = millis();

    // The reading should be zero for around 100 ms following the interrupt.
    // Check for this with some margins.
    if (rainSignalDetected &&
        (currentTime - rainSignalTime > 20) &&
        (currentTime - rainSignalTime < 80)) {
      rainSample = digitalRead(rainSignalPin);
      if (rainSample == LOW) {
        rainDetectedCounter++;
        
        //portENTER_CRITICAL(&rainMux);
        rainSignalDetected = false;
        //portEXIT_CRITICAL(&rainMux);
        
        Serial.print("Rain measured: ");
        Serial.println(rainDetectedCounter);
      }
    } 
    else if (currentTime - rainSignalTime > 120) {
      rainRejectedCounter++;
      
      //portENTER_CRITICAL(&rainMux);
      rainSignalDetected = false;
      rainSignalHandled = true;
      //portEXIT_CRITICAL(&rainMux);
      
      Serial.print("Rain interrupt rejected");
      Serial.println(rainRejectedCounter);
    }    
  }
}

void WH1080::resetRain()
{
  rainDetectedCounter = 0;
  rainRejectedCounter = 0;
}


/*
  Message Format: http://www.sevenwatt.com/main/wh1080-protocol-v2-fsk/

  Package definition:
  [
  preample 3 bytes 0xAA    synchron word    payload 10 bytes  postample 11 bits zero
  0xAA    0xAA    0xAA     0x2D    0xD4     nnnnn---nnnnnnnnn 0x00     0x0
  101010101010101010101010 0010110111010100 101.............. 00000000 000
  ]
  repeated six times (identical packages) per transmission every 48 seconds
  There is no or hardly any spacing between the packages.
  Spacing: to be confirmed.

  Payload definition:
  Weather sensor reading Message Format:
  AAAABBBBBBBBCCCCCCCCCCCCDDDDDDDDEEEEEEEEFFFFFFFFGGGGHHHHHHHHHHHHIIIIJJJJKKKKKKKK
  0xA4    0xF0    0x27    0x47    0x00    0x00    0x03    0xC6    0x0C    0xFE
  10100100111100000010011101000111000000000000000000000011110001100000110011111110

  with:
  AAAA = 1010    Message type: 0xA: sensor readings
  BBBBBBBB       Station ID / rolling code: Changes with battery insertion.
  CCCCCCCCCCCC   Temperature * 10 in celsius. Binary format MSB is sign
  DDDDDDDD       Humidity in %. Binary format 0-100. MSB (bit 7) unused.
  EEEEEEEE       Windspeed
  FFFFFFFF       Wind gust
  GGGG           Unknown
  HHHHHHHHHHHH   Rainfall cumulative. Binary format, max = 0x3FF,
  IIII           Status bits: MSB b3=low batt indicator.
  JJJJ           Wind direction
  KKKKKKKK       CRC8 - reverse Dallas One-wire CRC
*/

void WH1080::setSensorData(struct WH1080::WeatherData* frame) {
  memcpy(&weatherData, frame, sizeof(struct WH1080::WeatherData));
}

void WH1080::transmitSensorData() {
  byte buf[20];

  fillSensorBuffer(buf);
  sendWh1080Message(buf, 10);
}

// Fill the buffer pointed to by 'buf' with weather data.
void WH1080::fillSensorBuffer(byte *buf) {
  // Reverse of: unsigned short deviceID = ((unsigned short)buf[0] << 4) | (buf[1] >> 4);
  buf[0] = 0xA0;  // Message type
  buf[0] |= (deviceID & 0xf0) >> 4;
  buf[1] = (deviceID & 0x0f) << 4;

  // Reverse of: float temperature = ((float)temperatureRaw) / 10;
  short temperatureRaw = round(weatherData.temperature * 10.0f);
  short temperaturePos = temperatureRaw >= 0 ? temperatureRaw : -temperatureRaw;

  // Reverse of: int16_t temp = ((sbuf[1] & 0x07) << 8) | sbuf[2];
  buf[1] |= ((temperaturePos & 0x0700) >> 8);
  // Reverse of: uint8_t sign = (sbuf[1] >> 3) & 1;
  if (temperatureRaw < 0) buf[1] |= 0x08;
  buf[2] = (temperaturePos & 0x00ff);

  // Reverse of: int humidity = buf[3];
  short humidityRaw = round(weatherData.humidity);
  buf[3] = (humidityRaw <= 99 ? (humidityRaw & 0xff) : 99);

  // Reverse of: unsigned short windAvgRaw = (unsigned short)buf[4];
  short windAvgRaw = round(weatherData.windSpeed / 0.34f);
  buf[4] = (windAvgRaw & 0xff);

  // Reverse of: unsigned short windGustRaw = (unsigned short)buf[5];
  // float windGustMs = roundf((float)windGustRaw * 34.0f) / 100;
  short windGustRaw = round(weatherData.windGust / 0.34f);
  buf[5] = (windGustRaw & 0xff);

  // Just sending the number of pulses for rain.
  unsigned short rainRaw = weatherData.rain;

  // Reverse of: unsigned short rainRaw = (((unsigned short)buf[6] & 0x0f) << 8) | buf[7];
  buf[6] = (rainRaw & 0x0f00) >> 8;
  buf[7] = (rainRaw & 0x00ff);
  Serial.print("Rain short: ");
  Serial.println(rainRaw);

  // Reverse of: int direction = buf[8] & 0x0f;
  buf[8] = (weatherData.windDir & 0x0f);

  if (weatherData.lowBatt) {
    buf[8] |= 0x10;
  }

  buf[9] = crc8(buf, 9);

//  printf("Buffer: ");
//  for (int i = 0; i < 10; i++) {
//    printf("%02X ", buf[i]);
//  }

  printf("\nRaw data:\n");
  printf("Station ID: %02X, Temperature: %d, Humidity: %d\n", deviceID, temperatureRaw, humidityRaw);
  printf("Wind speed: %d, Gust speed: %d, Direction: %02X\n", windAvgRaw, windGustRaw, buf[8]);
  printf("Total rain: %d pulses\n", rainRaw);
}

void WH1080::sendWh1080Message(byte* message, byte length) {
  const int payload_len = 98;
  byte buf[payload_len];
  int index = 0;

  // Build the full payload out of the message. The message is repeated 6 times.
  for (int i = 0; i < 6; i++) {
    // The first preamble and sync word will be added by the RFM69.
    if (i != 0) {
      index = 17 * i - 5;
      buf[index] = 0xAA;
      buf[index + 1] = 0xAA;
      buf[index + 2] = 0xAA;
      buf[index + 3] = 0x2D;
      buf[index + 4] = 0xD4;
      index += 5;
    }

    // Copy the message
    for (byte j = 0; j < length; j++) {
      buf[index + j] = message[j];
    }

    index += length;
    buf[index] = 0x00;
    buf[index + 1] = 0x00;

    // All messages except the first one must be shifted a multiple of 5 bits to the left to get an 11 bit postamble.
    if (i != 0) {
      shiftLeft(buf, 17 * i - 5, 18, 5 * i);
    }
  }

//  Serial.print("Payload complete: ");
//  for (int i = 0; i < payload_len; i++) {
//    Serial.print(buf[i], HEX);
//    Serial.print(" ");
//  }

  rfm.SendArray(buf, payload_len);
  Serial.println("Payload sent");
}

// Shift a buffer 'shift' bits to the left. When shifting more than 8 bits, the
// left side of the buffer will be shifted into the bytes to the left of buf[start].
void WH1080::shiftLeft(byte* buf, int start, int msg_len, int shift) {
//  Serial.print("Shifting bits left, start = ");
//  Serial.print(start);
//  Serial.print(", msg_len = ");
//  Serial.print(msg_len);
//  Serial.print(", shift) = ");
//  Serial.println(shift);

  // When shifting left one byte or more
  int shift_bytes = shift / 8;
  if (shift_bytes > 0) {
//    Serial.print("Shifting bytes left: ");
//    Serial.println(shift_bytes);
    for (int k = start; k < start + msg_len; k++) {
      buf[k - shift_bytes] = buf[k];
    }
    buf[start + msg_len - 1] = 0x00;
  }

  shift = shift % 8;
  start -= shift_bytes;
//  Serial.print("Shifting bits left: ");
//  Serial.println(shift);
  buf[start - 1] |= ((buf[start] & getLeftShiftMask(shift)) >> (8 - shift));
  for (int k = start; k < start + msg_len; k++) {
    buf[k] <<= shift;
    if (k < start + msg_len - 1) {
      buf[k] |= ((buf[k + 1] & getLeftShiftMask(shift)) >> (8 - shift));
    }
  }
}

byte WH1080::getLeftShiftMask(int shift) {
  byte result = 0;

  switch (shift) {
    case 0:
      result = 0x00;
      break;
    case 1:
      result = 0x80;
      break;
    case 2:
      result = 0xC0;
      break;
    case 3:
      result = 0xE0;
      break;
    case 4:
      result = 0xF0;
      break;
    case 5:
      result = 0xF8;
      break;
    case 6:
      result = 0xFC;
      break;
    case 7:
      result = 0xFE;
      break;
    default:
      Serial.print("Shift argument outside range [0-7]");
  }

  return result;
}

/*
  Example: a1 82 0d 5a 03 06 00 4e 08 d3 crc ok (gap 48s)
  Pulse stats: Hi: 478 - 658   Lo: 1441 - 1687  (88 point)
  Threshold now 1049
  Temperature: 23.3C
  Pressure p0 (sea level): 1006.6 hPa
  Station Id: 0A18
  Temperature: 12.5C, Humidity: 90%
  Wind speed: 1.0 m/s, Gust Speed 2.0 m/s, S
  Wind speed: 2.3 mph, Gust Speed 4.6 mph, S
  Total rain: 23.4 mm
*/
void WH1080::sensorDataUnitTest() {
  byte buf[20];
  static float i = 0;
  deviceID = 0x63;

  weatherData.temperature = 12.5 + i;
  weatherData.humidity = 66;
  weatherData.windSpeed = 1.0;
  weatherData.windGust = 2.0;
  weatherData.windDir = 8;
  weatherData.rain = 15;

  // Fill the buffer with data in 'weatherData'.
  fillSensorBuffer(buf);
  sendWh1080Message(buf, 10);

  i += 0.1;
}

void WH1080::frequencyTest() {
  byte buf[20];
  deviceID = 0x63;
  unsigned long freq = 0;
  
  while (freq <= 11) {
    delay(3000);
    rfm.SetFrequency(867500 + (freq * 100));
  
    weatherData.temperature = 10 + freq;
    weatherData.humidity = 66;
    weatherData.windSpeed = 1.0;
    weatherData.windGust = 2.0;
    weatherData.windDir = 8;
    weatherData.rain = 1.0;
  
    // Fill the buffer with data in 'weatherData'.
    fillSensorBuffer(buf);
    sendWh1080Message(buf, 10);
    freq++;
  }
}

void WH1080::enableDcfReceiver() {
  digitalWrite(dcfEnablePin, LOW);
}

bool WH1080::updateTime() {
  time_t dcfTime = dcf.getTime(); // Check if new DCF77 time is available
  if (dcfTime != 0) {
    Serial.println("Time has been updated");
    setTime(dcfTime);

    // Display the time
    Serial.print(hour());
    Serial.print(":");
    Serial.print(minute());
    Serial.print(":");
    Serial.print(second());
    Serial.print(" ");
    Serial.print(day());
    Serial.print("-");
    Serial.print(month());
    Serial.print("-");
    Serial.print(year()); 
    Serial.println(); 

    return true;
  }

  return false;
}

void WH1080::disableDcfReceiver() {
  digitalWrite(dcfEnablePin, HIGH);  
}

/*
  The transmitter must send 10 bytes of data as follows.

  DCF Time Message Format:
  00000000111111112222222233333333444444445555555566666666777777778888888899999999
                    Hours Minutes Seconds Year       MonthDay     ?       Checksum
  AAAABBBBBBBBCCCCDDEEEEEEFFFFFFFFGGGGGGGGHHHHHHHHIIIJJJJJKKKKKKKKLMMMMMMMNNNNNNNN
  0xB4    0xFA    0x59    0x06    0x42    0x13    0x43    0x02    0x45    0x74

  with:
  AAAA = 1011    Message type: 0xB: DCF77 time stamp
  BBBBBBBB       Station ID / rolling code: Changes with battery insertion.
  CCCC           Unknown
  DD             Unknown
  EEEEEE         Hours, BCD
  FFFFFFFF       Minutes, BCD
  GGGGGGGG       Seconds, BCD
  HHHHHHHH       Year, last two digits, BCD
  III            Unknown
  JJJJJ          Month number, BCD
  KKKKKKKK       Day in month, BCD
  L              Unknown status bit
  MMMMMMM        Unknown
  NNNNNNNN       CRC8 - reverse Dallas One-wire CRC

  In the WH1080, the DCF code is transmitted five times with 48 second intervals
  between 3-6 minutes past a new hour. The sensor data transmission stops in the
  59th minute. Then there are no transmissions for three minutes, apparently to
  be noise free to acquire the DCF77 signal. On similar OOK weather stations the
  DCF77 signal is only transmitted every two hours.  
*/

void WH1080::transmitTimeData() {
  byte buf[10];

  fillTimeBuffer(buf);
  sendWh1080Message(buf, 10);
}

// Fill a WH1080 specific buffer with time data. Use the date / time of the ESP32, since
// they are set using the DCF time data.
void WH1080::fillTimeBuffer(byte *buf) {
  // Reverse of: setTime(BCD2bin(tbuf[2] & 0x3F), BCD2bin(tbuf[3]), BCD2bin(tbuf[4]), BCD2bin(tbuf[7]), BCD2bin(tbuf[6] & 0x1F), BCD2bin(tbuf[5]));
  buf[0] = 0xB0;
  buf[0] |= (deviceID & 0xf0) >> 4;
  buf[1] = ((deviceID & 0x0f) << 4) | 0x0a;
  buf[2] = (bin2BCD(hour() & 0x3f)) | 0x40;
  buf[3] = bin2BCD(minute() & 0xff);
  buf[4] = bin2BCD(second() & 0xff);
  buf[5] = bin2BCD((year() - 2000) & 0xff);
  buf[6] = (bin2BCD(month() & 0x1f)) | 0xc0;
  buf[7] = bin2BCD(day() & 0xff);
  buf[8] = 0x45; // Unknown
  buf[9] = crc8(buf, 9);
}

void WH1080::timeDataUnitTest() {
  byte buf[10];

  // Bi iA 52 3 44 19 C1 5 45 59
  // BB 3A 52 4 32 19 C1 5 45 0 
  buf[0] = 0xB0;
  buf[0] |= (deviceID & 0xf0) >> 4;
  buf[1] = ((deviceID & 0x0f) << 4) | 0x0a;
  buf[2] = 0x52;
  buf[3] = 0x04;
  buf[4] = 0x32;
  buf[5] = 0x19;
  buf[6] = 0xC1;
  buf[7] = 0x05;
  buf[8] = 0x45;
  buf[9] = crc8(buf, 9);

  // Set the current time to 14:27:12, December 14th, 2015
//  buf[0] = 0xB0;
//  buf[0] |= (deviceID & 0xf0) >> 4;
//  buf[1] = (deviceID & 0x0f) << 4;
//  buf[2] = bin2BCD(14 & 0x3f);
//  buf[3] = bin2BCD(27 & 0xff);
//  buf[4] = bin2BCD(12 & 0xff);
//  buf[5] = bin2BCD((2015 - 2000) & 0xff);
//  buf[6] = bin2BCD(12 & 0x1f);
//  buf[7] = bin2BCD(14 & 0x1f);
//  buf[8] = 0x45; // Unknown
//  buf[9] = crc8(buf, 9);
  sendWh1080Message(buf, 10);
}

void WH1080::printAllRfmRegs() {
  rfm.ReadAllRegs();
}

bool WH1080::rfmIsValid() {
  return (rfm.GetFrequency() == 868300);
}

void WH1080::resetRfm() {
  // Reset the RFM69 Radio
  digitalWrite(32, HIGH);
  delayMicroseconds(100);
  digitalWrite(32, LOW);
  
  // Wait until chip ready
  delay(5);
}

void WH1080::initializeRfm() {
  // Initialize RFM69
  Serial.print("Intializing RFM69H");
  rfm.Init();
  rfm.InitializeForWH1080();
  
  Serial.print(", radio: ");
  Serial.print(rfm.GetRadioName());
  Serial.print(" data rate ");
  Serial.print(rfm.GetDataRate());
  Serial.print(" @ ");
  Serial.print(rfm.GetFrequency());
  Serial.println(" kHz");
}

//
// Helper functions
//

// Functions to convert to and from Binary Coded Decimals, as used in the WH1080 data packets
int BCD2bin(uint8_t BCD) {
  return (10 * (BCD >> 4 & 0xF) + (BCD & 0xF));
}

uint8_t bin2BCD(int bin) {
  return ((bin / 10) << 4) + (bin % 10);
}

// Calculate a CRC over 'len' bytes in the buffer pointed to by 'addr'
uint8_t crc8(uint8_t *addr, uint8_t len) {
  uint8_t crc = 0;

  while (len--) {
    uint8_t inbyte = *addr++;
    uint8_t i;
    for (i = 8; i; i--) {
      uint8_t mix = (crc ^ inbyte) & 0x80; // changed from & 0x01
      crc <<= 1; // changed from right shift
      if (mix) crc ^= 0x31;// changed from 0x8C;
      inbyte <<= 1; // changed from right shift
    }
  }
  return crc;
}

void printDigits(int digits) {
  // utility function for digital clock display: leading 0
  if (digits < 10) Serial.print('0');
  Serial.print(digits);
}

void WH1080::timestamp(bool includeDate)
{
  if (includeDate) {
    Serial.print(year());
    Serial.print("-");
    printDigits(month());
    Serial.print("-");
    printDigits(day());
    Serial.print(" ");
  }
  printDigits(hour());
  Serial.print(":");
  printDigits(minute());
  Serial.print(":");
  printDigits(second());
  Serial.print(" ");
}

void WH1080::updateTime(byte *tbuf) {
  static unsigned long lastMillis;
  SensorBase::DisplayFrame(lastMillis, "WH1080Time", true, tbuf, WH1080::FRAME_LENGTH);
  Serial.print(' ');
  setTime(BCD2bin(tbuf[2] & 0x3F), BCD2bin(tbuf[3]), BCD2bin(tbuf[4]), BCD2bin(tbuf[7]), BCD2bin(tbuf[6] & 0x1F), BCD2bin(tbuf[5]));
  timestamp(true);
  Serial.println();
}
