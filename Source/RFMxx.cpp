/***************************************************************************
  This file is an adaptation of the file with the same name written for
  the LaCrosseITPlusReader project (https://github.com/rinie/LaCrosseITPlusReader).
 ***************************************************************************/
#include "RFMxx.h"
#include "SensorBase.h"
//#include "JeeLink.h"
//extern JeeLink jeeLink;
#ifdef USE_SPI_H
#include <SPI.h>
#define m_miso MISO
#define m_mosi MOSI
#define m_sck SCK
#define USE_SPI8_H
#define USE_SPI16_H
#endif

#ifndef USE_SPI_H
RFMxx::RFMxx(byte mosi, byte miso, byte sck, byte ss, byte irq, byte rst) {
  m_mosi = mosi;
  m_miso = miso;
  m_sck = sck;
#else
RFMxx::RFMxx(byte ss, byte irq, byte rst) {
#endif
  m_irq = irq;
  m_ss = ss;
  m_rst = rst;

  m_debug = false;
  m_dataRate = 17241;
  m_frequency = 868300;
  m_payloadPointer = 0;
  m_lastReceiveTime = 0;
  m_payloadReady = false;
  m_payload_crc = 0;
#ifndef USE_SPI_H
  Init();
#endif
}

void RFMxx::Init() {
#ifndef USE_SPI_H
  pinMode(m_mosi, OUTPUT);
  pinMode(m_miso, INPUT);
  pinMode(m_sck, OUTPUT);
  pinMode(m_ss, OUTPUT);
  pinMode(m_irq, INPUT);
  pinMode(m_rst, OUTPUT);
  delay(10);

  digitalWrite(m_ss, HIGH);
#else
  pinMode(m_irq, INPUT);
  pinMode(m_ss, OUTPUT);
  pinMode(m_rst, OUTPUT);
  delay(5);

  // Reset the RFM69 Radio
  digitalWrite(m_rst, HIGH);
  delayMicroseconds(100);
  digitalWrite(m_rst, LOW);

  // Wait until chip ready
  delay(10);
  digitalWrite(m_ss, HIGH);
#if 0
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4); // decided to slow down from DIV2 after SPI stalling in some instances, especially visible on mega1284p when RFM69 and FLASH chip both present
#endif
  SPI.begin();
#endif

  m_radioType = RFMxx::RFM69CW;

// I have commented out the code below because a deadlock occurs when
// the ESP32 is not plugged-in to the PCB.
//  m_radioType = RFMxx::RFM12B;
//  WriteReg(REG_PAYLOADLENGTH, 0xA);
//  if (ReadReg(REG_PAYLOADLENGTH) == 0xA) {
//    WriteReg(REG_PAYLOADLENGTH, 0x40);
//    if (ReadReg(REG_PAYLOADLENGTH) == 0x40) {
//      m_radioType = RFMxx::RFM69CW;
//    }
//  }
}

void RFMxx::InitializeForWH1080() {
  if (m_debug) {
    Serial.print("Radio is: ");
    Serial.println(GetRadioName());
  }

  digitalWrite(m_ss, HIGH);
  EnableReceiver(false);

  if (IsRF69) {
    /* 0x01 */
    WriteReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);
    /* 0x02 */
    WriteReg(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00);
    /* 0x05 */
    WriteReg(REG_FDEVMSB, RF_FDEVMSB_90000);
    /* 0x06 */
    WriteReg(REG_FDEVLSB, RF_FDEVLSB_90000);
    /* 0x11 */
    WriteReg(REG_PALEVEL, RF_PALEVEL_PA0_OFF | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | RF_PALEVEL_OUTPUTPOWER_11111);
    /* 0x13 */
    WriteReg(REG_OCP, RF_OCP_ON);
    /* 0x19 */
    WriteReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2);
    /* 0x28 */
    WriteReg(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
    /* 0x29 */
    WriteReg(REG_RSSITHRESH, 220);
    /* 0x2E */
    WriteReg(REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0);
    /* 0x2F */
    WriteReg(REG_SYNCVALUE1, 0x2D);
    /* 0x30 */
    WriteReg(REG_SYNCVALUE2, 0xD4);
    /* 0x37 */
    WriteReg(REG_PACKETCONFIG1, RF_PACKET1_CRCAUTOCLEAR_OFF);
    /* 0x38 */
    WriteReg(REG_PAYLOADLENGTH, PAYLOADSIZE);
    /* 0x3C */
    WriteReg(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE);
    /* 0x3D */
    WriteReg(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF);
    /* 0x6F */
    WriteReg(REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0);
  }
  else {
    spi16(0x8208);              // RX/TX off
    spi16(0x80E8);              // 80e8 CONFIGURATION EL,EF,868 band,12.5pF  (iT+ 915  80f8)
    spi16(0xC26a);              // DATA FILTER
    spi16(0xCA12);              // FIFO AND RESET  8,SYNC,!ff,DR
    spi16(0xCEd4);              // SYNCHRON PATTERN  0x2dd4
    spi16(0xC481);              // AFC during VDI HIGH
    spi16(0x94a0);              // RECEIVER CONTROL VDI Medium 134khz LNA max DRRSI 103 dbm
    spi16(0xCC77);              //
    spi16(0x9850);              // Deviation 90 kHz
    spi16(0xE000);              //
    spi16(0xC800);              //
    spi16(0xC040);              // 1.66MHz,2.2V
  }

  SetFrequency(m_frequency);
  SetDataRate(m_dataRate);

  ClearFifo();
}

void RFMxx::SetDebugMode(boolean mode) {
  m_debug = mode;
}
unsigned long RFMxx::GetDataRate() {
  return m_dataRate;
}

unsigned long RFMxx::GetFrequency() {
  return m_frequency;
}

RFMxx::RadioType RFMxx::GetRadioType() {
  return m_radioType;
}

String RFMxx::GetRadioName() {
  switch (GetRadioType()) {
    case RFMxx::RFM12B:
      return String("RFM12B");
      break;
    case RFMxx::RFM69CW:
      return String("RFM69CW");
      break;
    default:
      return String("None");
  }
}

void RFMxx::SetDataRate(unsigned long dataRate) {
  m_dataRate = dataRate;
  m_payload_max_size = 64;
  m_payload_min_size = (m_dataRate == 17241) ? 10 : 8;

  if (IsRF69) {
    word r = ((32000000UL + (m_dataRate / 2)) / m_dataRate);
    WriteReg(REG_BITRATEMSB, r >> 8);
    WriteReg(REG_BITRATELSB, r & 0xFF);
  }
  else {
    byte bt;
    if (m_dataRate == 17241) {
      bt = 0x13;
    }
    else if (m_dataRate == 8621) {
      bt = 0x28; // rinie see http://www.g-romahn.de/ws1600, https://github.com/rinie/weatherstationFSK/blob/master/weatherstationFSK.ino
    }
    else if (m_dataRate == 9579) {
      bt = 0x23;
    }
    else {
      bt = (byte)(344828UL / m_dataRate) - 1;
    }
    RFMxx::spi16(0xC600 | bt);
  }
}

void RFMxx::SetFrequency(unsigned long kHz) {
  m_frequency = kHz;

  if (IsRF69) {
    unsigned long f = (((kHz * 1000) << 2) / (32000000L >> 11)) << 6;
    WriteReg(REG_FRFMSB, f >> 16);
    WriteReg(REG_FRFMID, f >> 8);
    WriteReg(REG_FRFLSB, f);
  }
  else {
    RFMxx::spi16(40960 + (m_frequency - 860000) / 5);
  }
}

void RFMxx::EnableReceiver(bool enable, bool fClearFifo) {
  if (enable) {
    if (IsRF69) {
      WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
    }
    else {
      spi16(0x82C8);
      spi16(0xCA81);
      spi16(0xCA83);
    }
  }
  else {
    if (IsRF69) {
      WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
    }
    else {
      spi16(0x8208);
    }
  }
  if (fClearFifo /* || IsRF69 */) {
    ClearFifo();
  }
}

void RFMxx::EnableTransmitter(bool enable) {
  if (enable) {
    if (IsRF69) {
      WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
    }
    else {
      spi16(0x8238);
    }
  }
  else {
    if (IsRF69) {
      WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
    }
    else {
      spi16(0x8208);
    }
  }
}

void RFMxx::Receive() {
  if (IsRF69) {
    if (ReadReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) {
      m_lastReceiveTime = millis();
      m_payloadPointer = 0;
      for (int i = 0; i < PAYLOADSIZE; i++) {
        byte bt = GetByteFromFifo();
        m_payload_crc = SensorBase::UpdateCRC(m_payload_crc, bt);
        m_payload[i] = bt;
        m_payloadPointer = i;
        if (m_payloadPointer >= m_payload_min_size && m_payload_crc == 0) {
          //break;
        }
      }
      m_payloadReady = true;
    }
  }
  else {
#if 0
    bool hasData = false;
    digitalWrite(m_ss, LOW);
    asm("nop");
    asm("nop");
    if (digitalRead(m_miso)) {
      hasData = true;
    }
    digitalWrite(m_ss, HIGH);
    if (hasData) {
#else
    // RF12 status bits
#define RF_FIFO_BIT     0x8000
#define RF_POR_BIT      0x4000
#define RF_LBD_BIT      0x0400
#define RF_WDG_BIT      0x1000
#define RF_OVF_BIT      0x2000
#define RF_RSSI_BIT     0x0100

    // try
    bool hasData = digitalRead(m_irq) == 0;
    //	while ((digitalRead(m_irq) == 0) && !m_payloadReady) {{
    while ((spi16(0) & RF_FIFO_BIT) && !m_payloadReady) {
      {
#endif
        byte bt = GetByteFromFifo();
        m_payload[m_payloadPointer++] = bt;
        m_lastReceiveTime = millis();
        m_payload_crc = SensorBase::UpdateCRC(m_payload_crc, bt);
      }
  
      if ((m_payloadPointer >= 8 && m_payload_crc == 0) || (m_payloadPointer > 0 && millis() > m_lastReceiveTime + 50) || m_payloadPointer >= 32) {
        m_payloadReady = true;
      }
    }
#if 1
  }
#endif
}

byte RFMxx::GetPayload(byte *data) {
  byte payloadPointer = m_payloadPointer;
  m_payloadReady = false;
  m_payloadPointer = 0;
  m_payload_crc = 0;
  Serial.print("Payload2: ");
  for (int i = 0; i < PAYLOADSIZE; i++) {
    Serial.print(m_payload[i], HEX);
    Serial.print(" ");
    data[i] = m_payload[i];
  }
  return payloadPointer;
}

bool RFMxx::ReceiveGetPayloadWhenReady(byte *data, byte &length, byte &packetCount) {
  byte payload[PAYLOADSIZE];
  byte payLoadSize;
  //      byte packetCount;
  bool fAgain = false;
  bool fEnableReceiver = true;
  unsigned long lastReceiveTime = 0;
  byte count = 0;
  byte len;
  bool fPayloadIsReady = false;

  do {
    Receive();

    if (PayloadIsReady()) {
      payLoadSize = GetPayload(payload);

      if (!fPayloadIsReady) {
        Serial.print("Payload: ");
        for (int i = 0; i < payLoadSize; i++) {
          Serial.print(payload[i], HEX);
          Serial.print(" ");
          //printf("%02X ", payload[i]);
          data[i] = payload[i];
        }
        length = payLoadSize;
        len = payLoadSize;
        count++;
        fPayloadIsReady = true;
        fAgain = (payLoadSize < 16);
      }
      else if (payLoadSize >= len) { // WH1080 sends 6 repeated packages
        int i;
        for (i = 0; i < len; i++) {
          if (data[i] != payload[i]) {
            break;
          }
        }
        if (i >= len) {
          count++; // matching packet count
          fAgain = true;
        }
      }
      else {
        fAgain = false;
      }
      packetCount = count;

      if (fAgain) {
        lastReceiveTime = millis();
        fEnableReceiver = true;
        EnableReceiver(fEnableReceiver, false);
      }
    }
    else {
      fAgain = fAgain && (fPayloadIsReady) && (count < 8) && (millis() < lastReceiveTime + 50);
    }
  } while (fAgain);

  if (fEnableReceiver && fPayloadIsReady) {
    fEnableReceiver = false;
    EnableReceiver(fEnableReceiver);
  }
  return (fPayloadIsReady);
}

byte RFMxx::GetByteFromFifo() {
  return IsRF69 ? ReadReg(0x00) : (byte)spi16(0xB000);
}

bool RFMxx::PayloadIsReady() {
  return m_payloadReady;
}


void RFMxx::ClearFifo() {
  if (IsRF69) {
    WriteReg(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
  }
  else {
    for (byte i = 0; i < PAYLOADSIZE; i++) {
      spi16(0xB000);
    }
  }
}

void RFMxx::PowerDown() {
  if (IsRF69) {
    WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
  }
  else {
    spi16(0x8201);
  }
}

byte RFMxx::GetTemperature() {
  byte result = 0;
  if (IsRF69) {
    byte receiverWasOn = ReadReg(REG_OPMODE) & 0x10;

    EnableReceiver(false);

    WriteReg(0x4E, 0x08);
    while ((ReadReg(0x4E) & 0x04));
    result = ~ReadReg(0x4F) - 90;

    if (receiverWasOn) {
      EnableReceiver(true);
    }
  }

  return result;
}

#define clrb(pin) (*portOutputRegister(digitalPinToPort(pin)) &= ~digitalPinToBitMask(pin))
#define setb(pin) (*portOutputRegister(digitalPinToPort(pin)) |= digitalPinToBitMask(pin))
#ifndef USE_SPI8_H
byte RFMxx::spi8(byte value) {
  volatile byte *misoPort = portInputRegister(digitalPinToPort(m_miso));
  byte misoBit = digitalPinToBitMask(m_miso);
  for (byte i = 8; i; i--) {
    clrb(m_sck);
    if (value & 0x80) {
      setb(m_mosi);
    }
    else {
      clrb(m_mosi);
    }
    value <<= 1;
    setb(m_sck);
    if (*misoPort & misoBit) {
      value |= 1;
    }
  }
  clrb(m_sck);

  return value;
}
#else
byte RFMxx::spi8(byte value) {
  byte res;
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));
  digitalWrite(m_ss, LOW);
  res = SPI.transfer(value);
  digitalWrite(m_ss, HIGH);
  SPI.endTransaction();
  return res;
}

#endif

#ifndef USE_SPI16_H
unsigned short RFMxx::spi16(unsigned short value) {
  volatile byte *misoPort = portInputRegister(digitalPinToPort(m_miso));
  byte misoBit = digitalPinToBitMask(m_miso);

  clrb(m_ss);
  for (byte i = 0; i < 16; i++) {
    if (value & 32768) {
      setb(m_mosi);
    }
    else {
      clrb(m_mosi);
    }
    value <<= 1;
    if (*misoPort & misoBit) {
      value |= 1;
    }
    setb(m_sck);
    asm("nop");
    asm("nop");
    clrb(m_sck);
  }
  setb(m_ss);
  return value;
}
#else
unsigned short RFMxx::spi16(unsigned short value) {
  unsigned short res;
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));
  digitalWrite(m_ss, LOW);
  res = SPI.transfer16(value);
  digitalWrite(m_ss, HIGH);
  SPI.endTransaction();
  return res;
}
#endif

byte RFMxx::ReadReg(byte addr) {
#ifndef USE_SPI8_H
  digitalWrite(m_ss, LOW);
  spi8(addr & 0x7F);
  byte regval = spi8(0);
  digitalWrite(m_ss, HIGH);
  return regval;
#else
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));
  digitalWrite(m_ss, LOW);
  SPI.transfer(addr & 0x7F);
  uint8_t regval = SPI.transfer(0);
  digitalWrite(m_ss, HIGH);
  SPI.endTransaction();
  return regval;
#endif
}

void RFMxx::WriteReg(byte addr, byte value) {
#ifndef USE_SPI8_H
  digitalWrite(m_ss, LOW);
  spi8(addr | 0x80);
  spi8(value);

  digitalWrite(m_ss, HIGH);
#else
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));
  digitalWrite(m_ss, LOW);
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  digitalWrite(m_ss, HIGH);
  SPI.endTransaction();
#endif
}

void RFMxx::SendByte(byte data) {
  while (!(spi16(0x0000) & 0x8000)) {}
  RFMxx::spi16(0xB800 | data);
}

void RFMxx::SendArray(byte *data, byte length) {
  //int bytes_sent = 0;
  int i = 0;

  if (IsRF69) {
    Serial.print("Transmitting RFM69...");
    WriteReg(REG_PACKETCONFIG2, (ReadReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // Avoid RX deadlocks

    EnableReceiver(false);
    ClearFifo();

    noInterrupts();

    SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));
    digitalWrite(m_ss, LOW);

    SPI.transfer(REG_FIFO | 0x80);

    while ((i < length)) { // && !(ReadReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_FIFOFULL)) {
      SPI.transfer(data[i]);
      i++;
    }
    Serial.print("Bytes written to fifo: ");
    Serial.println(i);

    digitalWrite(m_ss, HIGH);
    SPI.endTransaction();

    interrupts();

    EnableTransmitter(true);

    // Wait until transmission is finished
    unsigned long txStart = millis();
    while (!(ReadReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) && millis() - txStart < 500);

    EnableTransmitter(false);
  }
  else {
    // Transmitter on
    EnableTransmitter(true);

    // Sync, sync, sync ...
    RFMxx::SendByte(0xAA);
    RFMxx::SendByte(0xAA);
    RFMxx::SendByte(0xAA);
    RFMxx::SendByte(0x2D);
    RFMxx::SendByte(0xD4);

    // Send the data
    for (int i = 0; i < length; i++) {
      RFMxx::SendByte(data[i]);
    }

    // Transmitter off
    delay(1);
    EnableTransmitter(false);
  }

  if (m_debug) {
    Serial.print("Sending data: ");
    for (int p = 0; p < length; p++) {
      Serial.print(data[p], DEC);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// For debugging
#define REGISTER_DETAIL 1
#if REGISTER_DETAIL
// SERIAL PRINT
// replace Serial.print("string") with SerialPrint("string")
#define SerialPrint(x) SerialPrint_P(PSTR(x))
void SerialWrite ( uint8_t c ) {
  Serial.write ( c );
}

void SerialPrint_P(PGM_P str, void (*f)(uint8_t) = SerialWrite ) {
  for (uint8_t c; (c = pgm_read_byte(str)); str++) (*f)(c);
}
#endif

void RFMxx::ReadAllRegs() {
  uint8_t regVal;

#if REGISTER_DETAIL
  int capVal;

  //... State Variables for intelligent decoding
  uint8_t modeFSK = 0;
  int bitRate = 0;
  int freqDev = 0;
  long freqCenter = 0;
#endif

  Serial.println("Address - HEX - BIN");
  for (uint8_t regAddr = 1; regAddr <= 0x4F; regAddr++)
  {
    regVal = ReadReg(regAddr);

    //    Select();
    //    SPI.transfer(regAddr & 0x7F); // send address + r/w bit
    //    regVal = SPI.transfer(0);
    //    Unselect();

    Serial.print(regAddr, HEX);
    Serial.print(" - ");
    Serial.print(regVal, HEX);
    Serial.print(" - ");
    Serial.println(regVal, BIN);

#if REGISTER_DETAIL
    switch (regAddr)
    {
      case 0x1 : {
          SerialPrint ( "Controls the automatic Sequencer ( see section 4.2 )\nSequencerOff : " );
          if ( 0x80 & regVal ) {
            SerialPrint ( "1 -> Mode is forced by the user\n" );
          } else {
            SerialPrint ( "0 -> Operating mode as selected with Mode bits in RegOpMode is automatically reached with the Sequencer\n" );
          }

          SerialPrint( "\nEnables Listen mode, should be enabled whilst in Standby mode:\nListenOn : " );
          if ( 0x40 & regVal ) {
            SerialPrint ( "1 -> On\n" );
          } else {
            SerialPrint ( "0 -> Off ( see section 4.3)\n" );
          }

          SerialPrint( "\nAborts Listen mode when set together with ListenOn=0 See section 4.3.4 for details (Always reads 0.)\n" );
          if ( 0x20 & regVal ) {
            SerialPrint ( "ERROR - ListenAbort should NEVER return 1 this is a write only register\n" );
          }

          SerialPrint("\nTransceiver's operating modes:\nMode : ");
          capVal = (regVal >> 2) & 0x7;
          if ( capVal == 0b000 ) {
            SerialPrint ( "000 -> Sleep mode (SLEEP)\n" );
          } else if ( capVal == 0b001 ) {
            SerialPrint ( "001 -> Standby mode (STDBY)\n" );
          } else if ( capVal == 0b010 ) {
            SerialPrint ( "010 -> Frequency Synthesizer mode (FS)\n" );
          } else if ( capVal == 0b011 ) {
            SerialPrint ( "011 -> Transmitter mode (TX)\n" );
          } else if ( capVal == 0b100 ) {
            SerialPrint ( "100 -> Receiver Mode (RX)\n" );
          } else {
            Serial.print( capVal, BIN );
            SerialPrint ( " -> RESERVED\n" );
          }
          SerialPrint ( "\n" );
          break;
        }

      case 0x2 : {

          SerialPrint("Data Processing mode:\nDataMode : ");
          capVal = (regVal >> 5) & 0x3;
          if ( capVal == 0b00 ) {
            SerialPrint ( "00 -> Packet mode\n" );
          } else if ( capVal == 0b01 ) {
            SerialPrint ( "01 -> reserved\n" );
          } else if ( capVal == 0b10 ) {
            SerialPrint ( "10 -> Continuous mode with bit synchronizer\n" );
          } else if ( capVal == 0b11 ) {
            SerialPrint ( "11 -> Continuous mode without bit synchronizer\n" );
          }

          SerialPrint("\nModulation scheme:\nModulation Type : ");
          capVal = (regVal >> 3) & 0x3;
          if ( capVal == 0b00 ) {
            SerialPrint ( "00 -> FSK\n" );
            modeFSK = 1;
          } else if ( capVal == 0b01 ) {
            SerialPrint ( "01 -> OOK\n" );
          } else if ( capVal == 0b10 ) {
            SerialPrint ( "10 -> reserved\n" );
          } else if ( capVal == 0b11 ) {
            SerialPrint ( "11 -> reserved\n" );
          }

          SerialPrint("\nData shaping: ");
          if ( modeFSK ) {
            SerialPrint( "in FSK:\n" );
          } else {
            SerialPrint( "in OOK:\n" );
          }
          SerialPrint ("ModulationShaping : ");
          capVal = regVal & 0x3;
          if ( modeFSK ) {
            if ( capVal == 0b00 ) {
              SerialPrint ( "00 -> no shaping\n" );
            } else if ( capVal == 0b01 ) {
              SerialPrint ( "01 -> Gaussian filter, BT = 1.0\n" );
            } else if ( capVal == 0b10 ) {
              SerialPrint ( "10 -> Gaussian filter, BT = 0.5\n" );
            } else if ( capVal == 0b11 ) {
              SerialPrint ( "11 -> Gaussian filter, BT = 0.3\n" );
            }
          } else {
            if ( capVal == 0b00 ) {
              SerialPrint ( "00 -> no shaping\n" );
            } else if ( capVal == 0b01 ) {
              SerialPrint ( "01 -> filtering with f(cutoff) = BR\n" );
            } else if ( capVal == 0b10 ) {
              SerialPrint ( "10 -> filtering with f(cutoff) = 2*BR\n" );
            } else if ( capVal == 0b11 ) {
              SerialPrint ( "ERROR - 11 is reserved\n" );
            }
          }

          SerialPrint ( "\n" );
          break;
        }

      case 0x3 : {
          bitRate = (regVal << 8);
          break;
        }

      case 0x4 : {
          bitRate |= regVal;
          unsigned long val = 0;
          SerialPrint ( "Bit Rate (Chip Rate when Manchester encoding is enabled)\nBitRate : ");
          if (bitRate > 0) {
            val = 32UL * 1000UL * 1000UL / bitRate;
          } 
          Serial.println( val );
          SerialPrint( "\n" );
          break;
        }

      case 0x5 : {
          freqDev = ( (regVal & 0x3f) << 8 );
          break;
        }

      case 0x6 : {
          freqDev |= regVal;
          SerialPrint( "Frequency deviation\nFdev : " );
          unsigned long val = RF69_FSTEP * freqDev;
          Serial.println( val );
          SerialPrint ( "\n" );
          break;
        }

      case 0x7 : {
          unsigned long tempVal = regVal;
          freqCenter = ( tempVal << 16 );
          break;
        }

      case 0x8 : {
          unsigned long tempVal = regVal;
          freqCenter = freqCenter | ( tempVal << 8 );
          break;
        }

      case 0x9 : {
          freqCenter = freqCenter | regVal;
          SerialPrint ( "RF Carrier frequency\nFRF : " );
          unsigned long val = RF69_FSTEP * freqCenter;
          Serial.println( val );
          SerialPrint( "\n" );
          break;
        }

      case 0xa : {
          SerialPrint ( "RC calibration control & status\nRcCalDone : " );
          if ( 0x40 & regVal ) {
            SerialPrint ( "1 -> RC calibration is over\n" );
          } else {
            SerialPrint ( "0 -> RC calibration is in progress\n" );
          }

          SerialPrint ( "\n" );
          break;
        }

      case 0xb : {
          SerialPrint ( "Improved AFC routine for signals with modulation index lower than 2.  Refer to section 3.4.16 for details\nAfcLowBetaOn : " );
          if ( 0x20 & regVal ) {
            SerialPrint ( "1 -> Improved AFC routine\n" );
          } else {
            SerialPrint ( "0 -> Standard AFC routine\n" );
          }
          SerialPrint ( "\n" );
          break;
        }

      case 0xc : {
          SerialPrint ( "Reserved\n\n" );
          break;
        }

      case 0xd : {
          byte val;
          SerialPrint ( "Resolution of Listen mode Idle time (calibrated RC osc):\nListenResolIdle : " );
          val = regVal >> 6;
          if ( val == 0b00 ) {
            SerialPrint ( "00 -> reserved\n" );
          } else if ( val == 0b01 ) {
            SerialPrint ( "01 -> 64 us\n" );
          } else if ( val == 0b10 ) {
            SerialPrint ( "10 -> 4.1 ms\n" );
          } else if ( val == 0b11 ) {
            SerialPrint ( "11 -> 262 ms\n" );
          }

          SerialPrint ( "\nResolution of Listen mode Rx time (calibrated RC osc):\nListenResolRx : " );
          val = (regVal >> 4) & 0x3;
          if ( val == 0b00 ) {
            SerialPrint ( "00 -> reserved\n" );
          } else if ( val == 0b01 ) {
            SerialPrint ( "01 -> 64 us\n" );
          } else if ( val == 0b10 ) {
            SerialPrint ( "10 -> 4.1 ms\n" );
          } else if ( val == 0b11 ) {
            SerialPrint ( "11 -> 262 ms\n" );
          }

          SerialPrint ( "\nCriteria for packet acceptance in Listen mode:\nListenCriteria : " );
          if ( 0x8 & regVal ) {
            SerialPrint ( "1 -> signal strength is above RssiThreshold and SyncAddress matched\n" );
          } else {
            SerialPrint ( "0 -> signal strength is above RssiThreshold\n" );
          }

          SerialPrint ( "\nAction taken after acceptance of a packet in Listen mode:\nListenEnd : " );
          val = (regVal >> 1 ) & 0x3;
          if ( val == 0b00 ) {
            SerialPrint ( "00 -> chip stays in Rx mode. Listen mode stops and must be disabled (see section 4.3)\n" );
          } else if ( val == 0b01 ) {
            SerialPrint ( "01 -> chip stays in Rx mode until PayloadReady or Timeout interrupt occurs.  It then goes to the mode defined by Mode. Listen mode stops and must be disabled (see section 4.3)\n" );
          } else if ( val == 0b10 ) {
            SerialPrint ( "10 -> chip stays in Rx mode until PayloadReady or Timeout occurs.  Listen mode then resumes in Idle state.  FIFO content is lost at next Rx wakeup.\n" );
          } else if ( val == 0b11 ) {
            SerialPrint ( "11 -> Reserved\n" );
          }


          SerialPrint ( "\n" );
          break;
        }

      default : {
        }
    }
#endif
  }
  //Unselect();
}
