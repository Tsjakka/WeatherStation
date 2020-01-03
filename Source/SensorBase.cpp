#include "SensorBase.h"

bool SensorBase::m_debug = false;

byte SensorBase::UpdateCRC(byte res, uint8_t val) {
    for (int i = 0; i < 8; i++) {
      uint8_t tmp = (uint8_t)((res ^ val) & 0x80);
      res <<= 1;
      if (0 != tmp) {
        res ^= 0x31;
      }
      val <<= 1;
    }
  return res;
}

byte SensorBase::CalculateCRC(byte *data, byte len) {
  byte res = 0;
  for (int j = 0; j < len; j++) {
    uint8_t val = data[j];
    res = UpdateCRC(res, val);
  }
  return res;
}

void SensorBase::SetDebugMode(boolean mode) {
  m_debug = mode;
}

void SensorBase::DisplayFrame(unsigned long &lastMillis, char *device, bool fIsValid, byte *data, byte frameLength) {
    unsigned long now = millis();
    char div[16];
    if (lastMillis == 0) {
		lastMillis = now;
	}
    sprintf(div, "%06ld ", (unsigned long)(now - lastMillis));
    Serial.print(div);
    lastMillis = now;
	// Show the raw data bytes
	Serial.print(device);
	Serial.print(" [");
	for (int i = 0; i < frameLength; i++) {
	  Serial.print(data[i], HEX);
	  Serial.print(" ");
	}
	Serial.print("]");

	// Check CRC
	if (!fIsValid) {
	  Serial.print(" CRC:WRONG");
	}
	else {
	  Serial.print(" CRC:OK");
    }
}


