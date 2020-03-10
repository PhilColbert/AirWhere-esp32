// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include "sx1276-regs.h"
#include "..\ogn\manchester.h"

#define RF_IRQ_PreambleDetect 0x0200
                                     // bits in IrqFlags1 and IrfFlags2
#define RF_IRQ_ModeReady      0x8000 // mode change done (between some modes)
#define RF_IRQ_RxReady        0x4000
#define RF_IRQ_TxReady        0x2000 //
#define RF_IRQ_PllLock        0x1000 //
#define RF_IRQ_Rssi           0x0800
#define RF_IRQ_Timeout        0x0400
//
#define RF_IRQ_SyncAddrMatch  0x0100

#define RF_IRQ_FifoFull      0x0080 //
#define RF_IRQ_FifoNotEmpty  0x0040 // at least one byte in the FIFO
#define RF_IRQ_FifoLevel     0x0020 // more bytes than FifoThreshold
#define RF_IRQ_FifoOverrun   0x0010 // write this bit to clear the FIFO
#define RF_IRQ_PacketSent    0x0008 // packet transmission was completed
#define RF_IRQ_PayloadReady  0x0004
#define RF_IRQ_CrcOk         0x0002
#define RF_IRQ_LowBat        0x0001


//v1.1


extern int debug_level;
extern bool receiveIRQ;

#include <Arduino.h>
#include <SPI.h>

#define SX1276_CH_868_200 868200000 //868.200Mhz
#define SX1276_CH_915_040 916040000 //916.04Mhz

#define SX1276_SCK     5    // GPIO5  -- SX1278's SCK
#define SX1276_MISO    19   // GPIO19 -- SX1278's MISO
#define SX1276_MOSI    27   // GPIO27 -- SX1278's MOSI
#define SX1276_SS      18   // GPIO18 -- SX1278's CS
#define SX1276_RST     14   // GPIO14 -- SX1278's RESET
#define SX1276_DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

#define PIN_LED_PCB  GPIO_NUM_25  // status LED on the PCB

#define PIN_RFM_RST  GPIO_NUM_14  // Reset
#define PIN_RFM_IRQ  GPIO_NUM_26  // packet done on receive or transmit
#define PIN_RFM_SS   GPIO_NUM_18  // SPI chip-select
#define PIN_RFM_SCK  GPIO_NUM_5   // SPI clock
#define PIN_RFM_MISO GPIO_NUM_19  // SPI MISO
#define PIN_RFM_MOSI GPIO_NUM_27  // SPI MOSI


#define Sx1276_debug_mode 0

#define LORA_DEFAULT_SS_PIN    18
#define LORA_DEFAULT_RESET_PIN 14
#define LORA_DEFAULT_DIO0_PIN  26

#define PA_OUTPUT_RFO_PIN      0
#define PA_OUTPUT_PA_BOOST_PIN 1

const int LORA_TX_OK = 0;
const int LORA_TX_TX_ONGOING = -1;
const int LORA_TX_RX_ONGOING = -2;

class LoRaClass : public Stream {
public:
  LoRaClass();

  int begin(long frequency);

  //ogn additions

  uint32_t BaseFrequency;            // [32MHz/2^19/2^8] base frequency = channel #0
  int32_t FrequencyCorrection;      // [32MHz/2^19/2^8] frequency correction (due to Xtal offset)
  uint32_t ChannelSpacing;           // [32MHz/2^19/2^8] spacing between channels
  int16_t Channel;                  // [       integer] channel being used

  void RESET(uint8_t On);

  void setBaseFrequency(uint32_t Frequency=868200000);
  void setChannelSpacing(uint32_t  Spacing=   200000);
  void setFrequencyCorrection(int32_t Correction=0);
  void setChannel(int16_t newChannel);
  uint8_t getChannel(void);
  int Configure(int16_t Channel, const uint8_t *Sync);
  void WriteMode(uint8_t Mode=RF_OPMODE_STANDBY);
  uint8_t ReadMode (void);
  uint8_t ModeReady(void);
  uint16_t ReadIrqFlags(void);
  void ClearIrqFlags(void);
  uint8_t ReadVersion(void) ;

  void WriteTxPower(int8_t TxPower=0);

  uint16_t SwapBytes(uint16_t Word) ;

  uint8_t WriteByte(uint8_t Byte, uint8_t Addr=0);

  void WriteWord(uint16_t Word, uint8_t Addr=0);

  uint8_t ReadByte (uint8_t Addr=0);

  uint16_t ReadWord (uint8_t Addr=0);

  void WriteBytes(const uint8_t *Data, uint8_t Len, uint8_t Addr=0);

  void WriteFreq(uint32_t Freq) ;

  void WritePacket(const uint8_t *Data, uint8_t Len=26) ;

  void ReadPacket(uint8_t *Data, uint8_t *Err, uint8_t Len=26);

  void TransferBlock(uint8_t *Data, uint8_t Len);

  uint8_t *Block_Read(uint8_t Len, uint8_t Addr) ;


  uint8_t *Block_Write(const uint8_t *Data, uint8_t Len, uint8_t Addr);

  static uint32_t calcSynthFrequency(uint32_t Frequency);


  void WriteSYNC(uint8_t WriteSize, uint8_t SyncTol, const uint8_t *SyncData);

  uint8_t Transmit(uint8_t TxChan, const uint8_t *PacketByte, uint8_t Thresh, uint8_t MaxWait);


  void SetTxChannel(uint8_t TxChan) ;

  void sb();

  uint8_t ReadRSSI(void);




  uint8_t sx_readRegister_burst(uint8_t address, uint8_t *data, int length);

  int sx_writeRegister_burst(uint8_t address, uint8_t *data, int length);




  void end();

  void handle_irq();

  int beginPacket(int implicitHeader = false);
  int endPacket();

  int parsePacket(int size = 0);
  int packetRssi();
  float packetSnr();

  // from Print
  virtual size_t write(uint8_t byte);
  virtual size_t write(const uint8_t *buffer, size_t size);

  // from Stream
  virtual int available();
  virtual int read();
  virtual int peek();
  virtual void flush();

  void onReceive(void(*callback)(int));

  void receive(int size = 0);
  void idle();
  void sleep();

  void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
  void setFrequency(long frequency);
  void setSpreadingFactor(int sf);
  void setSignalBandwidth(long sbw);
  void setCodingRate4(int denominator);
  void setPreambleLength(long length);
  void setSyncWord(int sw);
  void enableCrc();
  void disableCrc();

  // deprecated
  void crc() { enableCrc(); }
  void noCrc() { disableCrc(); }

  byte random();

  void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN);
  void setSPIFrequency(uint32_t frequency);

  void dumpRegisters(Stream& out);

  int sendFrame(uint8_t *data, int length);



private:
  void explicitHeaderMode();
  void implicitHeaderMode();

  void IRAM_ATTR handleDio0Rise();

  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t address, uint8_t value);

  static void IRAM_ATTR onDio0Rise();
  void setDio0Irq(int mode);
  void select();
  void unselect();
  void writeFifo(uint8_t addr, uint8_t *data, int length);
  uint8_t getOpMode(void);



private:
  SPISettings _spiSettings;
  int _ss;
  int _reset;
  int _dio0;
  int _frequency;
  int _packetIndex;
  int _implicitHeaderMode;
  void (*_onReceive)(int);
};

extern LoRaClass sx1276;

#endif
