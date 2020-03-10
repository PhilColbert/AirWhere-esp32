// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "sx1276.h"
//#define Sx1276_debug_mode 2

//#define DIS
// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_MODEM_STAT			 0x18
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define RFLR_DIOMAPPING1_DIO1_01 0x10
#define REG_VERSION              0x42 

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06
#define MODE_CAD                 0x07


// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40
#define IRQ_OPMODE_MASK            0xF8
#define IRQ_CADDONE                0x04
#define IRQ_CADDETECTED            0x01

#define MAX_PKT_LENGTH           255


  static const size_t MaxBlockLen = 64;
  uint8_t Block_Buffer[MaxBlockLen];

LoRaClass::LoRaClass() :
  _spiSettings(2E6, MSBFIRST, SPI_MODE0),
  _ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN),
  _frequency(0),
  _packetIndex(0),
  _implicitHeaderMode(0),
  _onReceive(NULL)
{
  // overide Stream timeout value
  setTimeout(0);
}

// OGN Additions

void LoRaClass::RESET(uint8_t On)
{

	if(On) gpio_set_level(gpio_num_t(_reset), 1);
	else gpio_set_level(gpio_num_t(_reset), 1);
}

void  LoRaClass::setBaseFrequency(uint32_t Frequency) { BaseFrequency=calcSynthFrequency(Frequency); }
void LoRaClass::setChannelSpacing(uint32_t  Spacing) { ChannelSpacing=calcSynthFrequency(Spacing); }
void LoRaClass::setFrequencyCorrection(int32_t Correction)
{
	if(Correction<0) FrequencyCorrection = -calcSynthFrequency(-Correction);
	else  FrequencyCorrection =  calcSynthFrequency( Correction);
}
void LoRaClass::setChannel(int16_t newChannel)
{
	Channel=newChannel; WriteFreq((BaseFrequency+ChannelSpacing*Channel+FrequencyCorrection+128)>>8);

	/*   Serial.println("BaseFrequency");
	   Serial.println(BaseFrequency);
	   Serial.println("ChannelSpacing");
	   Serial.println(ChannelSpacing);
	   Serial.println("Channel");
	   Serial.println(Channel);
	   Serial.println("FrequencyCorrection");
	   Serial.println(FrequencyCorrection);
*/


}
uint8_t LoRaClass::getChannel(void) { return Channel; }

void LoRaClass::sb()
{
	WriteMode(RF_OPMODE_STANDBY);
}

int LoRaClass::Configure(int16_t Channel, const uint8_t *Sync)
{
	// setup pins
	  pinMode(_ss, OUTPUT);
	  // set SS high
	  digitalWrite(_ss, HIGH);

	  if (_reset != -1) {

		  Serial.println("resetIN GC onfigure ");
	    pinMode(_reset, OUTPUT);

	    // perform reset
	    digitalWrite(_reset, LOW);
	    delay(20);
	    digitalWrite(_reset, HIGH);
	    delay(20);
	  }

	  // start SPI
	  //SPI.begin();
	  SPI.end();
	  delay(10);
	  SPI.begin(SX1276_SCK,SX1276_MISO,SX1276_MOSI,SX1276_SS);

	WriteMode(RF_OPMODE_STANDBY);              // mode: STDBY, modulation: FSK, no LoRa
	// usleep(1000);
	WriteTxPower(17);
	// ClearIrqFlags();
	WriteWord(0x0140, REG_BITRATEMSB);         // bit rate = 100kbps (32MHz/100000)
	// ReadWord(REG_BITRATEMSB);
	WriteWord(0x0333, REG_FDEVMSB);            // FSK deviation = +/-50kHz [32MHz/(1<<19)]
	// ReadWord(REG_FDEVMSB);
	setChannel(Channel);                       // operating channel
	WriteSYNC(8, 7, Sync);                     // SYNC pattern (setup for reception)
	WriteByte(  0x8A, REG_PREAMBLEDETECT);     // preamble detect: 1 byte, page 92
	WriteByte(  0x00, REG_PACKETCONFIG1);      // Fixed size packet, no DC-free encoding, no CRC, no address filtering
	WriteByte(  0x40, REG_PACKETCONFIG2);      // Packet mode
	WriteByte(    51, REG_FIFOTHRESH);         // TxStartCondition=FifoNotEmpty, FIFO threshold = 51 bytes
	WriteByte(  2*26, REG_PAYLOADLENGTH);      // Packet size = 26 bytes Manchester encoded into 52 bytes
	WriteWord(0x3030, REG_DIOMAPPING1);        // DIO signals: DIO0=00, DIO1=11, DIO2=00, DIO3=00, DIO4=00, DIO5=11, => p.64, 99
	WriteByte(  0x4A, REG_RXBW);               // +/-100kHz Rx bandwidth => p.27+67
	WriteByte(  0x49, REG_PARAMP);             // BT=0.5 shaping, 40us ramp up/down
	WriteByte(  0x07, REG_RSSICONFIG);         // 256 samples for RSSI, p.90

	return 0;
}

void    LoRaClass::WriteMode(uint8_t Mode)
{
	WriteByte(Mode, REG_OPMODE);
} // SLEEP/STDBY/FSYNTH/TX/RX
uint8_t LoRaClass::ReadMode (void) { return ReadByte(REG_OPMODE); }
uint8_t LoRaClass::ModeReady(void) { return ReadByte(REG_IRQFLAGS1)&0x80; }

uint16_t LoRaClass::ReadIrqFlags(void) { return ReadWord(REG_IRQFLAGS1); }

void LoRaClass::ClearIrqFlags(void)    { WriteWord(RF_IRQ_FifoOverrun | RF_IRQ_Rssi, REG_IRQFLAGS1); }


uint8_t LoRaClass::ReadVersion(void) { return ReadByte(REG_VERSION); }           // normally returns: 0x12


void LoRaClass::WriteTxPower(int8_t TxPower)
{
	if(TxPower<2) TxPower=2;
	else if(TxPower>17) TxPower=17;
	// if(TxPower<=14)
	// { WriteByte(0x70 | TxPower    , REG_PACONFIG);
	// }
	// else
	{
		WriteByte(0xF0 | (TxPower-2), REG_PACONFIG);
	}
}

 uint16_t LoRaClass::SwapBytes(uint16_t Word) { return (Word>>8) | (Word<<8); }

  uint8_t LoRaClass::WriteByte(uint8_t Byte, uint8_t Addr) // write Byte
  { // printf("WriteByte(0x%02X, 0x%02X)\n", Byte, Addr);
    uint8_t *Ret = Block_Write(&Byte, 1, Addr); return *Ret; }

  void LoRaClass::WriteWord(uint16_t Word, uint8_t Addr) // write Word => two bytes
  { // printf("WriteWord(0x%04X, 0x%02X)\n", Word, Addr);
    uint16_t Swapped = SwapBytes(Word); Block_Write((uint8_t *)&Swapped, 2, Addr); }

  uint8_t LoRaClass::ReadByte (uint8_t Addr)
  { uint8_t *Ret = Block_Read(1, Addr);
    // printf("ReadByte(0x%02X) => 0x%02X\n", Addr, *Ret );
    return *Ret; }

  uint16_t LoRaClass::ReadWord (uint8_t Addr)
  { uint16_t *Ret = (uint16_t *)Block_Read(2, Addr);
    // printf("ReadWord(0x%02X) => 0x%04X\n", Addr, SwapBytes(*Ret) );
    return SwapBytes(*Ret); }

  void LoRaClass::WriteBytes(const uint8_t *Data, uint8_t Len, uint8_t Addr)
  { Block_Write(Data, Len, Addr); }

  void LoRaClass::WriteFreq(uint32_t Freq)                       // [32MHz/2^19] Set center frequency in units of RFM69 synth.
  { const uint8_t Addr = REG_FRFMSB;
    uint8_t Buff[4];
    Buff[0] = Freq>>16;
    Buff[1] = Freq>> 8;
    Buff[2] = Freq    ;
    Buff[3] =        0;
    Block_Write(Buff, 3, Addr); }

  void LoRaClass::WritePacket(const uint8_t *Data, uint8_t Len)         // write the packet data (26 bytes)
  { uint8_t Packet[2*Len];
    uint8_t PktIdx=0;
    for(uint8_t Idx=0; Idx<Len; Idx++)
    { uint8_t Byte=Data[Idx];
      Packet[PktIdx++]=ManchesterEncode[Byte>>4];                               // software manchester encode every byte
      Packet[PktIdx++]=ManchesterEncode[Byte&0x0F];
    }
    Block_Write(Packet, 2*Len, REG_FIFO);
  }

  void LoRaClass::ReadPacket(uint8_t *Data, uint8_t *Err, uint8_t Len)             // read packet data from FIFO
  { uint8_t *Packet = Block_Read(2*Len, REG_FIFO);                         // read 2x26 bytes from the RF chip RxFIFO
    uint8_t PktIdx=0;
    for(uint8_t Idx=0; Idx<Len; Idx++)                                     // loop over packet bytes
    { uint8_t ByteH = Packet[PktIdx++];
      ByteH = ManchesterDecode[ByteH]; uint8_t ErrH=ByteH>>4; ByteH&=0x0F; // decode manchester, detect (some) errors
      uint8_t ByteL = Packet[PktIdx++];
      ByteL = ManchesterDecode[ByteL]; uint8_t ErrL=ByteL>>4; ByteL&=0x0F;
      Data[Idx]=(ByteH<<4) | ByteL;
      Err [Idx]=(ErrH <<4) | ErrL ;
    }
  }

  void LoRaClass::TransferBlock(uint8_t *Data, uint8_t Len)
  {

#ifdef DIS


    Serial.print("Sending data");

  	Serial.print(" ");
  	Serial.print(Len);
  	Serial.print(" [");
  	for (int i = 0; i < Len; i++)
  	{
  		Serial.print(Data[i], HEX);
  		if (i < Len - 1)
  			Serial.print(", ");
  	}
  	Serial.println("]");
#endif


  	uint8_t * out;

  	out=Data;
    select();
  	digitalWrite(SX1276_SS, LOW);
    SPI.beginTransaction(_spiSettings);
  	SPI.transferBytes( Data, out, Len);
  	SPI.endTransaction();
  	digitalWrite(SX1276_SS, HIGH);
  	unselect();

#ifdef DIS
  	Serial.print("Receiving data");

  	Serial.print(" ");
  	Serial.print(Len);
  	Serial.print(" [");
  	for (int i = 0; i < Len; i++)
  	{
  		Serial.print(out[i], HEX);
  		if (i < Len - 1)
  			Serial.print(", ");
  	}
  	Serial.println("]");
#endif

  }

  uint8_t LoRaClass::sx_readRegister_burst(uint8_t address, uint8_t *data, int length)
  {
  	//address |= 0x80;
  	address &= 0x7F;

  	uint8_t * out;

  	out=data;

  	digitalWrite(SX1276_SS, LOW);
      SPI.beginTransaction(_spiSettings);
      SPI.transfer(address);
  	SPI.transferBytes( data, out, length);
  	SPI.endTransaction();
  	digitalWrite(SX1276_SS, HIGH);

/*  	Serial.print("Receiving data");

  	Serial.print(" ");
  	Serial.print(length);
  	Serial.print(" [");
  	for (int i = 0; i < length; i++)
  	{
  		Serial.print(out[i], HEX);
  		if (i < length - 1)
  			Serial.print(", ");
  	}
  	Serial.println("]"); added return 0;
*/
  	return 0;
  }

  int LoRaClass::sx_writeRegister_burst(uint8_t address, uint8_t *data, int length)
  {

  	/* bit 7 set to write registers */
	  address |= 0x80;

	  uint8_t * out;

	  out=data;

	  digitalWrite(SX1276_SS, LOW);
	  SPI.beginTransaction(_spiSettings);
	  SPI.transfer(address);
	  SPI.transferBytes( data, out, length);
	  SPI.endTransaction();
	  digitalWrite(SX1276_SS, HIGH);

	  // added return 0
	  return 0;
  }


  uint8_t *LoRaClass::Block_Read(uint8_t Len, uint8_t Addr)                       // read given number of bytes from given Addr
  {
	  Block_Buffer[0]=Addr; memset(Block_Buffer+1, 0, Len);
	  TransferBlock (Block_Buffer, Len+1);
	  return  Block_Buffer+1;
  }                                          // return the pointer to the data read from the given Addr

  uint8_t *LoRaClass::Block_Write(const uint8_t *Data, uint8_t Len, uint8_t Addr) // write given number of bytes to given Addr
  {
	  Block_Buffer[0] = Addr | 0x80; memcpy(Block_Buffer+1, Data, Len);
	  // printf("Block_Write( [0x%02X, .. ], %d, 0x%02X) .. [0x%02X, 0x%02X, ...]\n", Data[0], Len, Addr, Block_Buffer[0], Block_Buffer[1]);
	  TransferBlock (Block_Buffer, Len+1);
	  return  Block_Buffer+1;
  }


   uint32_t LoRaClass::calcSynthFrequency(uint32_t Frequency) { return (((uint64_t)Frequency<<16)+7812)/15625; }



   void LoRaClass::WriteSYNC(uint8_t WriteSize, uint8_t SyncTol, const uint8_t *SyncData)
   {
	   if(SyncTol>7) SyncTol=7;
	   if(WriteSize>8) WriteSize=8;
	   WriteBytes(SyncData+(8-WriteSize), WriteSize, REG_SYNCVALUE1);        // write the SYNC, skip some initial bytes
	   WriteByte(  0x90 | (WriteSize-1), REG_SYNCCONFIG);                     // write SYNC length [bytes]
	   WriteWord( 9-WriteSize, REG_PREAMBLEMSB);
   }                           // write preamble length [bytes] (page 71)

   uint8_t LoRaClass::ReadRSSI(void)    { return ReadByte(REG_RSSIVALUE); }         // read value: RSS = -Value/2

   uint8_t LoRaClass::Transmit(uint8_t TxChan, const uint8_t *PacketByte, uint8_t Thresh, uint8_t MaxWait)
   {


	   if(PacketByte==0) return 0;                                   // if no packet to send: simply return

	   WriteMode(RF_OPMODE_STANDBY);                              // switch to standby

	   // vTaskPrioritySet(0, tskIDLE_PRIORITY+2);
	   vTaskDelay(1);
	   SetTxChannel(TxChan);

	   ClearIrqFlags();
//	   printf("WritePacket\n");
	   WritePacket(PacketByte);
	//   printf("RF_OPMODE_TRANSMITTER\n");// write packet into FIFO
	   WriteMode(RF_OPMODE_TRANSMITTER);                          // transmit
	   vTaskDelay(5);                                                 // wait 5ms
	   uint8_t Break=0;
	   for(uint16_t Wait=400; Wait; Wait--)                           // wait for transmission to end
	   { // if(!TRX.DIO0_isOn()) break;
		   // uint8_t  Mode=TRX.ReadMode();
		   uint16_t Flags=ReadIrqFlags();
		   // if(Mode!=RF_OPMODE_TRANSMITTER) break;
		   if(Flags&RF_IRQ_PacketSent) Break++;
		   if(Break>=2) break; }
	   WriteMode(RF_OPMODE_STANDBY);                              // switch to standy
	   // vTaskPrioritySet(0, tskIDLE_PRIORITY+2);

	   // SetRxChannel();
	   WriteMode(RF_OPMODE_RECEIVER);                             // back to receive mode
	   return 1;
   }


   void LoRaClass::SetTxChannel(uint8_t TxChan)         // default channel to transmit is same as the receive channel
   {

	   static const uint8_t OGN_SYN1C[8] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A };

     WriteTxPower(14);                    // set TX for transmission
     setChannel(TxChan&0x7F);
     WriteSYNC(8, 7, OGN_SYN1C);
   }

void LoRaClass::select()
{
	noInterrupts();
	digitalWrite(_ss, LOW);
}

void LoRaClass::unselect()
{
	digitalWrite(_ss, HIGH);
	interrupts();
}

void LoRaClass::writeFifo(uint8_t addr, uint8_t *data, int length)
{
#if (Sx1276_debug_mode > 1)
	Serial.println();
	Serial.print(F("## Sx1276 write fifo, length="));
	Serial.print(length, DEC);

	Serial.print(" [");
	for (int i = 0; i < length; i++)
	{
		Serial.print(data[i], HEX);
		if (i < length - 1)
			Serial.print(", ");
	}
	Serial.println("]");
#endif

	/* select location */
	writeRegister(REG_FIFO_ADDR_PTR, addr);

	/* upload data */
	select();
	SPI.transfer(REG_FIFO | 0x80);
	for (int i = 0; i < length; i++)
		SPI.transfer(data[i]);
	unselect();
}

void LoRaClass::setDio0Irq(int mode)
{
	uint8_t map1 = readRegister(REG_DIO_MAPPING_1) & 0x3F;
	writeRegister(REG_DIO_MAPPING_1, map1 | mode);
}

int LoRaClass::begin(long frequency)
{

  // setup pins
  pinMode(_ss, OUTPUT);
  // set SS high
  digitalWrite(_ss, HIGH);

  if (_reset != -1) {
    pinMode(_reset, OUTPUT);
    Serial.println("resetIN begin ");
    // perform reset
    digitalWrite(_reset, LOW);
    delay(10);
    digitalWrite(_reset, HIGH);
    delay(10);
  }

  // start SPI
  //SPI.begin();
  SPI.end();
  delay(10);
  SPI.begin(SX1276_SCK,SX1276_MISO,SX1276_MOSI,SX1276_SS);

  // check version
  uint8_t version = readRegister(REG_VERSION);

  if (version != 0x12) {
    return 0;
  }

  // put in sleep mode
  sleep();

  // set frequency
  setFrequency(frequency);

  // set base addresses
  writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
  writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

  // set LNA boost
  writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

  // set auto AGC
  writeRegister(REG_MODEM_CONFIG_3, 0x04);

  // set output power to 17 dBm
  setTxPower(17);

  // put in standby mode
  idle();

  return 1;
}

void LoRaClass::end()
{
  // put in sleep mode
  sleep();

  // stop SPI
  SPI.end();
}

int LoRaClass::beginPacket(int implicitHeader)
{
  // put in standby mode
  idle();

  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }

  // reset FIFO address and paload length
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, 0);

  return 1;
}


void LoRaClass::handle_irq()
{
	//Serial.println("Hhandle_irq section >>>>>>>>>>>>>><<<<<<<<<<<>>>>>>>>>>>>>>");

    if (receiveIRQ)
    {
    	if (debug_level>0)
    	{
    		Serial.println("*******************************received packet");
    	}
     // Serial.println("Handling sx1276 irq >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
	  int irqFlags = readRegister(REG_IRQ_FLAGS);

	//  Serial.println("Clear IRQ");

	  // clear IRQ's
	  writeRegister(REG_IRQ_FLAGS, irqFlags);

	  if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {

	    // received a packet
	    _packetIndex = 0;

	    // read packet length
	    int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);

	    // set FIFO address to current RX address
	    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

	    if (_onReceive) {
	      _onReceive(packetLength);
	    }

	    // reset FIFO address
	    writeRegister(REG_FIFO_ADDR_PTR, 0);
	  }
	  receiveIRQ=false;
    }
}

int LoRaClass::endPacket()
{
  // put in TX mode
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

  // wait for TX done
  while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
    yield();
  }

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

  return 1;
}

int LoRaClass::parsePacket(int size)
{
  int packetLength = 0;
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  if (size > 0) {
    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;

    // read packet length
    if (_implicitHeaderMode) {
      packetLength = readRegister(REG_PAYLOAD_LENGTH);
    } else {
      packetLength = readRegister(REG_RX_NB_BYTES);
    }

    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    // put in standby mode
    idle();
  } else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // not currently in RX mode

    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);

    // put in single RX mode
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }

  return packetLength;
}

int LoRaClass::packetRssi()
{
  return (readRegister(REG_PKT_RSSI_VALUE) - (_frequency < 868E6 ? 164 : 157));
}

float LoRaClass::packetSnr()
{
  return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

size_t LoRaClass::write(uint8_t byte)
{
  return write(&byte, sizeof(byte));
}

size_t LoRaClass::write(const uint8_t *buffer, size_t size)
{
  int currentLength = readRegister(REG_PAYLOAD_LENGTH);

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }

  // write data
  for (size_t i = 0; i < size; i++) {
    writeRegister(REG_FIFO, buffer[i]);
  }

  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

  return size;
}

int LoRaClass::available()
{
  return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int LoRaClass::read()
{
  if (!available()) {
    return -1;
  }

  _packetIndex++;

  return readRegister(REG_FIFO);
}

int LoRaClass::peek()
{
  if (!available()) {
    return -1;
  }

  // store current FIFO address
  int currentAddress = readRegister(REG_FIFO_ADDR_PTR);

  // read
  uint8_t b = readRegister(REG_FIFO);

  // restore FIFO address
  writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

  return b;
}

void LoRaClass::flush()
{
}

void LoRaClass::onReceive(void(*callback)(int))
{
  _onReceive = callback;

  if (callback) {
    writeRegister(REG_DIO_MAPPING_1, 0x00);

    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
  }
}

void LoRaClass::receive(int size)
{
  if (size > 0) {
    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void LoRaClass::idle()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRaClass::sleep()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRaClass::setTxPower(int level, int outputPin)
{
  if (PA_OUTPUT_RFO_PIN == outputPin) {
    // RFO
    if (level < 0) {
      level = 0;
    } else if (level > 14) {
      level = 14;
    }

    writeRegister(REG_PA_CONFIG, 0x70 | level);
  } else {
    // PA BOOST
    if (level < 2) {
      level = 2;
    } else if (level > 17) {
      level = 17;
    }

    writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
  }
}

void LoRaClass::setFrequency(long frequency)
{
  _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void LoRaClass::setSpreadingFactor(int sf)
{
  if (sf < 6) {
    sf = 6;
  } else if (sf > 12) {
    sf = 12;
  }

  if (sf == 6) {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
  }

  writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

void LoRaClass::setSignalBandwidth(long sbw)
{
  int bw;

  if (sbw <= 7.8E3) {
    bw = 0;
  } else if (sbw <= 10.4E3) {
    bw = 1;
  } else if (sbw <= 15.6E3) {
    bw = 2;
  } else if (sbw <= 20.8E3) {
    bw = 3;
  } else if (sbw <= 31.25E3) {
    bw = 4;
  } else if (sbw <= 41.7E3) {
    bw = 5;
  } else if (sbw <= 62.5E3) {
    bw = 6;
  } else if (sbw <= 125E3) {
    bw = 7;
  } else if (sbw <= 250E3) {
    bw = 8;
  } else /*if (sbw <= 250E3)*/ {
    bw = 9;
  }

  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void LoRaClass::setCodingRate4(int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

  int cr = denominator - 4;

  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void LoRaClass::setPreambleLength(long length)
{
  writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void LoRaClass::setSyncWord(int sw)
{
  writeRegister(REG_SYNC_WORD, sw);
}

void LoRaClass::enableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void LoRaClass::disableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

byte LoRaClass::random()
{
  return readRegister(REG_RSSI_WIDEBAND);
}

void LoRaClass::setPins(int ss, int reset, int dio0)
{
  _ss = ss;
  _reset = reset;
  _dio0 = dio0;
}

void LoRaClass::setSPIFrequency(uint32_t frequency)
{
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

void LoRaClass::dumpRegisters(Stream& out)
{
  for (int i = 0; i < 128; i++) {
    out.print("0x");
    out.print(i, HEX);
    out.print(": 0x");
    out.println(readRegister(i), HEX);
  }
}

void LoRaClass::explicitHeaderMode()
{
  _implicitHeaderMode = 0;

  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRaClass::implicitHeaderMode()
{
  _implicitHeaderMode = 1;

  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void IRAM_ATTR LoRaClass::handleDio0Rise()
{
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;

    // read packet length
    int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);

    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    if (_onReceive) {
      _onReceive(packetLength);
    }

    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);
  }


}



uint8_t LoRaClass::readRegister(uint8_t address)
{
  return singleTransfer(address & 0x7f, 0x00);
}

void LoRaClass::writeRegister(uint8_t address, uint8_t value)
{
  singleTransfer(address | 0x80, value);
}

uint8_t LoRaClass::singleTransfer(uint8_t address, uint8_t value)
{
  uint8_t response;

  digitalWrite(_ss, LOW);

  SPI.beginTransaction(_spiSettings);
  SPI.transfer(address);
  response = SPI.transfer(value);
  SPI.endTransaction();

  digitalWrite(_ss, HIGH);

  return response;
}

void IRAM_ATTR LoRaClass::onDio0Rise()
{

	receiveIRQ=true;
  //  sx1276.handleDio0Rise();

}

int LoRaClass::sendFrame(uint8_t *data, int length)
{
#if (Sx1276_debug_mode > 0)
	Serial.print(F("## Sx1276 send frame..."));
#endif

//	return LORA_TX_OK;

	//Serial.println("in sx1276 sendFrame");

	uint8_t mode = getOpMode();

	/* are we transmitting anyway? */
	if (mode == MODE_TX)
		return LORA_TX_TX_ONGOING;

	/* in case of receiving, is it ongoing? */
	for (int i = 0; i<400; i++)
	{

		if ((mode == MODE_RX_CONTINUOUS || mode == MODE_RX_SINGLE) && (readRegister(REG_MODEM_STAT) & 0x0B))
			return LORA_TX_RX_ONGOING;

		delayMicroseconds(10);
	}
	/*
	* CAD
	*/


	//setOpMode(RFLR_OPMODE_STANDBY);
	idle();
	writeRegister(REG_IRQ_FLAGS, IRQ_CADDONE | IRQ_CADDETECTED);
	//setOpMode(RFLR_OPMODE_CAD);
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_CAD);

	//wait for CAD completion 

	byte iflags;
	while (((iflags = readRegister(REG_IRQ_FLAGS)) & IRQ_CADDONE) == 0) {
		yield();
		delayMicroseconds(1);//original value 1 so replace with yield to feed WDT (sw resets after 3secs)
	}


	if (iflags & IRQ_CADDETECTED)
	{
#if (Sx1276_debug_mode > 1)
		Serial.println("Cad Detected...");
#endif
		// re-establish old mode //
		if (mode == MODE_RX_CONTINUOUS || mode == MODE_RX_SINGLE)
			//setOpMode(mode);
			writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | mode);
		return LORA_TX_RX_ONGOING;
	}


	//setOpMode(RFLR_OPMODE_STANDBY);
	idle();

	  // reset FIFO address and paload length
	  writeRegister(REG_FIFO_ADDR_PTR, 0);
	  writeRegister(REG_PAYLOAD_LENGTH, 0);


	//todo: check fifo is empty, no rx data..

	/* upload frame */
	writeFifo(0x00, data, length);
	
	writeRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
	writeRegister(REG_PAYLOAD_LENGTH, length);

	//setOpMode(RFLR_OPMODE_TRANSMITTER);


	/* prepare irq */
    if (_onReceive)
	{
		/* clear flag  txdone*/

	//	setDio0Irq(REG_DIO_MAPPING_1);
	}

	  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

	  while((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0);

	  // clear IRQ's
	  writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

	  /* bypass waiting */
	  if (_onReceive)
	  {
		  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
		  return LORA_TX_OK;
	  }


	for (int i = 0; i<1000 && getOpMode() == MODE_TX; i++)
	{
#if (Sx1276_debug_mode > 0)
		Serial.print(".Sx1276.");
#endif
		delay(5);
	}
#if (Sx1276_debug_mode > 0)
	Serial.println("(Sx1276_debug_mode *** done");
#endif

	// back into receive mode.

	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);

	return LORA_TX_OK;
}

uint8_t LoRaClass::getOpMode(void)
{
	return readRegister(REG_OP_MODE & IRQ_OPMODE_MASK);
}


LoRaClass sx1276;
