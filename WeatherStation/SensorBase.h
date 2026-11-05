#ifndef _SENSORBASE_h
#define _SENSORBASE_h

#include "Arduino.h"

class SensorBase {
public:
  static byte UpdateCRC(byte res, uint8_t val);
  static byte CalculateCRC(byte *data, byte len);
  static void SetDebugMode(boolean mode);
  static void DisplayFrame(unsigned long &lastMillis, char *device, bool fIsValid, byte *data, byte frameLength);

protected:
  static bool m_debug;
};

#endif
