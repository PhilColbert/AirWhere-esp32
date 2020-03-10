/*
 * mac.cpp
 *
 *  Created on: 30 Sep 2016
 *      Author: sid
 */
#include <math.h>

#include "../fanet_stack/fmac.h"
#include "../fanet_stack/sx1272.h"
//v1.1
/*SX1276 support*/
#include "../fanet_stack/sx1276.h"

// OGN
//v1.1

#define SX1276_CORRECTION -20000
#include "..\ogn\freqplan.h"


#include "..\ogn\ogn.h"

static const uint8_t OGN_SYNC[8] = { 0xAA, 0x66, 0x55, 0xA5, 0x96, 0x99, 0x96, 0x5A };
uint8_t   RX_AverRSSI;               // [-0.5dBm] average RSSI
FreqPlan  RF_FreqPlan;               // frequency hopping pattern calculator
uint32_t RX_Random=0x12345678;        // Random number from LSB of RSSI readouts

#define SX_REG_BACKUP_FSK	0
#define SX_REG_BACKUP_LORA	1
uint8_t sx_reg_backup[2][111];

uint8_t packetMsg[255];
bool channelChanged;
int cidx;

void onReceive(int packetSize)
{

	Serial.print("Received packet '");

	// read packet
	int rssi;
	for (int i = 0; i < packetSize; i++)packetMsg[i] = sx1276.read();
	rssi = sx1276.packetRssi();

	for (int i = 0; i < packetSize; i++) Serial.print(packetMsg[i],HEX);
	// print RSSI of packet
	Serial.print(" with RSSI ");
	Serial.println(sx1276.packetRssi());

}


/* get next frame which can be sent out */
//todo: this is potentially dangerous, as frm may be deleted in another place.
Frame* MacFifo::get_nexttx()
{
	int next;
	noInterrupts();
	for(next = 0; next<fifo.size(); next++)
		if(fifo.get(next)->next_tx < millis())
			break;
	Frame *frm;
	if(next == fifo.size())
		frm = NULL;
	else
		frm = fifo.get(next);
	interrupts();

	return frm;
}

Frame* MacFifo::frame_in_list(Frame *frm)
{
	noInterrupts();

	for(int i=0; i<fifo.size(); i++)
	{
		Frame *frm_list = fifo.get(i);
		if(*frm_list == *frm)
		{
			interrupts();
			return frm_list;
		}
	}

	interrupts();

	return NULL;
}

Frame* MacFifo::front()
{
	noInterrupts();
	Frame *frm = fifo.shift();
	interrupts();

	return frm;
}

/* add frame to fifo */
int MacFifo::add(Frame *frm)
{
	noInterrupts();

	/* buffer full */
	/* note: ACKs will always fit */
	if(fifo.size() >= MAC_FIFO_SIZE && frm->type != FRM_TYPE_ACK)
	{
		interrupts();
		return -1;
	}

	/* only one ack_requested from us to a specific address at a time is allowed in the queue */
	//in order not to screw with the awaiting of ACK
	if(frm->ack_requested)
	{
		for(int i=0; i<fifo.size(); i++)
		{
			//note: this never succeeds for received packets -> tx condition only
			Frame *ffrm = fifo.get(i);
			if(frm->ack_requested && ffrm->src == fmac.my_addr && ffrm->dest == frm->dest)
			{
				interrupts();
				return -2;
			}
		}
	}

	if(frm->type == FRM_TYPE_ACK)
		/* add to front */
		fifo.unshift(frm);
	else
		/* add to tail */
		fifo.add(frm);

	interrupts();
	return 0;
}

/* remove frame from linked list and delete it */
bool MacFifo::remove_delete(Frame *frm)
{
	bool found = false;

	noInterrupts();
	for(int i=0; i<fifo.size() && !found; i++)
		if(frm == fifo.get(i))
		{
			delete fifo.remove(i);
			found = true;
		}
	interrupts();

	return found;
}

/* remove any pending frame that waits on an ACK from a host */
bool MacFifo::remove_delete_acked_frame(MacAddr dest)
{
	bool found = false;
	noInterrupts();

	for(int i=0; i<fifo.size(); i++)
	{
		Frame* frm = fifo.get(i);
		if(frm->ack_requested && frm->dest == dest)
		{
			delete fifo.remove(i);
			found = true;
		}
	}
	interrupts();
	return found;
}

/* this is executed in a non-linear fashion */
void FanetMac::frame_received(int length)
{
//	Serial.print("Received packet '");

	// read packet =0
	int rssi =0;
//	for (int i = 0; i < length; i++)packetMsg[i] = sx1276.read();
//	rssi = sx1276.packetRssi();

//	for (int i = 0; i < length; i++) Serial.print(packetMsg[i],HEX);
	// print RSSI of packet
//	Serial.print(" with RSSI ");
//	Serial.println(sx1276.packetRssi());

	/* quickly read registers */
	// read packet
	/*sx1276 support*/
		//int rssi;
		if (lora_or_flarm == 'r'  )
		{

			num_received = length;
			for (int i = 0; i < length; i++)rx_frame[i] = sx1276.read();
			rssi = sx1276.packetRssi();
		}
		if (lora_or_flarm == 'h'  || lora_or_flarm=='t' || lora_or_flarm=='b' )
		{
			num_received = length;
			for (int i = 0; i < length; i++)rx_frame[i] = sx1276.read();
			rssi = sx1276.packetRssi();
			if (debug_level>10)
			{
		     	for (int i = 0; i < length; i++) Serial.print(rx_frame[i],HEX);
			}
			// print RSSI of packet
		//	Serial.print(" with RSSI ");
		//	Serial.println(sx1276.packetRssi());

		}
		if (lora_or_flarm == 'l')
		{
			num_received = sx1272.getFrame(rx_frame, sizeof(rx_frame));
			rssi = sx1272.getRssi();
		}

#if defined(SerialDEBUG) && MAC_debug_mode > 0
	SerialDEBUG.print(millis());
	SerialDEBUG.print(F("### Mac Rx: "));
	SerialDEBUG.print(num_received, DEC);
	SerialDEBUG.print(F(" @ "));
	SerialDEBUG.print(rssi);
	SerialDEBUG.print(F(" "));

	for(int i=0; i<num_received; i++)
	{
		SerialDEBUG.print(rx_frame[i], HEX);
		if(i<num_received-1)
			SerialDEBUG.print(":");
	}
	SerialDEBUG.println();
#endif


	/* build frame from stream */
	Frame *frm = new Frame(num_received, rx_frame);
	frm->rssi = rssi;

	/* add to fifo */
	if(rx_fifo.add(frm) < 0)
		delete frm;

	return;
}

/* wrapper to fit callback into c++ */
void FanetMac::frame_rx_wrapper(int length)
{
	fmac.frame_received(length);
}

FanetMac::FanetMac() : my_timer(MAC_SLOT_MS, state_wrapper)
{
	num_received = 0;
}

bool FanetMac::begin(Fapp &app)
{
	myApp = &app;

	/* configure phy radio */

	if (debug_level>0)
	{
		Serial.println("radio starting");
	}

	if (lora_or_flarm == 'r')
	{
			

		sx1276.setPins(SX1276_SS, SX1276_RST, SX1276_DI0);
		pinMode(SX1276_DI0, INPUT);

		//radio initialisation

		if (loraFrequency=='9')
		{
			if (sx1276.begin(SX1276_CH_915_040) == false)
			{
				return false;
			}
		}
		else
		{
			if (sx1276.begin(SX1276_CH_868_200) == false)
		    {
			  return false;
		    }
		}
		sx1276.setPreambleLength(5);
		sx1276.setSignalBandwidth(250E3);
		sx1276.setSpreadingFactor(7);
		sx1276.setTimeout(5);
		sx1276.setCodingRate4(5);
		sx1276.enableCrc();

		sx1276.setSyncWord(MAC_SYNCWORD);

		// register the receive callback
		sx1276.onReceive(frame_rx_wrapper);

		// put the radio into receive mode
		sx1276.receive(0); //explicit header
	}

	if (lora_or_flarm == 'l')
	{

		if (sx1272.begin() == false)
			return false;
		sx1272.setBandwidth(BW_250);
		sx1272.setSpreadingFactor(SF_7);
		sx1272.setCodingRate(CR_5);
		//v1.1
		sx1272.sx1272_setSyncWord(MAC_SYNCWORD);
		sx1272.setExplicitHeader(true);
		sx1272.setPayloadCrc(true);
		sx1272.setPower(14);
		sx1272.sx1272_setLnaGain(LNAGAIN_G1_MAX, true);

		//sx1276 support

		if (loraFrequency=='9')
		{
			sx1272.setChannel(CH_915_040);
		}
		else
		{
			sx1272.setChannel(CH_868_200);
		}
		sx1272.setIrqReceiver(frame_rx_wrapper);
	}

	if (lora_or_flarm == 'h'  || lora_or_flarm=='t' || lora_or_flarm=='b')
	{

		sx1276.setPins(LORA_DEFAULT_SS_PIN,LORA_DEFAULT_RESET_PIN,LORA_DEFAULT_DIO0_PIN);
	//#define RF_FREQUENCY 916040000 // Hz
//#define RF_FREQUENCY   868200000 // Hz
//		if (sx1276.begin(SX1276_CH_868_200) == 0) {
	//		return false;
	//	}

		if (debug_level>0)
		{
			if (lora_or_flarm == 'h')
			{
			  Serial.println("heltec board starting");
			}
			if (lora_or_flarm == 't')
			{
				Serial.println("TTGO board starting");
			}
			if (lora_or_flarm == 'b')
			{
				Serial.println("TTGO T-beam board starting");
			}
		}

		if (loraFrequency=='9')
		{
			if (sx1276.begin(SX1276_CH_915_040) == false)
			{
				return false;
			}
		}
		else
		{
			if (sx1276.begin(SX1276_CH_868_200) == false)
		    {
			  return false;
		    }
		}


		//radio initialisation

	    pinMode(SX1276_DI0, INPUT);

	//	sx1276.setFrequency(RF_FREQUENCY);
		sx1276.setPreambleLength(5);
		sx1276.setSignalBandwidth(250E3);
		sx1276.setSpreadingFactor(7);
		sx1276.setTimeout(5);
		sx1276.setCodingRate4(5);
		sx1276.enableCrc();

     	sx1276.setSyncWord(MAC_SYNCWORD);

		// register the receive callback
		sx1276.onReceive(frame_rx_wrapper);

		// put the radio into receive mode
		sx1276.receive(0); //explicit header}


	}




	/* start state machine */
	my_timer.Start();

	/* start random machine */
#if defined(ARDUINO_SAMD_ZERO) || defined(ARDUINO_SAMD_VARIANT_COMPLIANCE)
	/* use the device ID */
	volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
	volatile uint32_t *ptr2 = (volatile uint32_t *)0x0080A040;
	volatile uint32_t *ptr3 = (volatile uint32_t *)0x0080A044;
	volatile uint32_t *ptr4 = (volatile uint32_t *)0x0080A048;
	randomSeed(millis() + *ptr1 + *ptr2 + *ptr3 + *ptr4);
#else
	randomSeed(millis());
#endif

	return true;
}

/* wrapper to fit callback into c++ */
void FanetMac::state_wrapper()
{
	fmac.handle_rx();
	fmac.handle_tx();
}

bool FanetMac::isNeighbor(MacAddr addr)
{
	for(int i=0; i<neighbors.size(); i++)
		if(neighbors.get(i)->addr == addr)
			return true;

	return false;
}

/*
 * Generates ACK frame
 */
void FanetMac::ack(Frame* frm)
{
#if defined(SerialDEBUG) && MAC_debug_mode > 0
	SerialDEBUG.print(millis());
	SerialDEBUG.println(F("### generating ACK"));
#endif

	/* generate reply */
	Frame *ack = new Frame(my_addr);
	ack->type = FRM_TYPE_ACK;
	ack->dest = frm->src;

	/* only do a 2 hop ACK in case it was requested and we received it via a two hop link (= forward bit is not set anymore) */
	if(frm->ack_requested == MAC_ACK_TWOHOP && !frm->forward)
		ack->forward = true;

	/* add to front of fifo */
	//note: this will not fail by define
	if(tx_fifo.add(ack) != 0)
		delete ack;
}


uint8_t FanetMac::StartRFchipFSK(void)
{
	RF_FreqPlan.setPlan(0);

/*	sx1276.RESET(1);                                              // RESET active
	vTaskDelay(50);                                            // wait 10ms
	sx1276.RESET(0);                                              // RESET released
	vTaskDelay(50);*/
	sx1276.setBaseFrequency(RF_FreqPlan.BaseFreq);                // set the base frequency (recalculate to RFM69 internal synth. units)
	sx1276.setChannelSpacing(RF_FreqPlan.ChanSepar);              // set the channel separation
	sx1276.setFrequencyCorrection(SX1276_CORRECTION);
	// wait 10ms
	// set TRX base frequency and channel separation after the frequency hopp$

	sx1276.Configure(0, OGN_SYNC);                                // setup RF chip parameters and set to channel #0
	//vTaskDelay(50);
	sx1276.WriteMode(RF_OPMODE_STANDBY);                          // set RF chip mode to STANDBY
	//Serial.println("reading regs after configure.");
	//sx1276.sx_readRegister_burst(REG_BITRATEMSB, sx_reg_backup[SX_REG_BACKUP_FSK], sizeof(sx_reg_backup[SX_REG_BACKUP_FSK]));
	//vTaskDelay(50);
	return sx1276.ReadVersion();
}

void FanetMac::switch_lora()
{
	//sx1276.setPins(SS,RST,DI0);
	//#define RF_FREQUENCY 916040000 // Hz
#define RF_FREQUENCY   868200000 // Hz
	if (sx1276.begin(RF_FREQUENCY) == 0) {
		Serial.println("sx1276 not recognized - LORA ERROR");
		exit(1);
	}
	else
	{
		Serial.println("sx1276 found");
	}

	//radio initialisation

	pinMode(SX1276_DI0, INPUT);

	sx1276.setFrequency(RF_FREQUENCY);
	sx1276.setPreambleLength(5);
	sx1276.setSignalBandwidth(250E3);
	sx1276.setSpreadingFactor(7);
	sx1276.setTimeout(5);
	sx1276.setCodingRate4(5);
	sx1276.enableCrc();

	sx1276.setSyncWord(MAC_SYNCWORD);

	// register the receive callback
//	sx1276.onReceive(onReceive);
	sx1276.onReceive(frame_rx_wrapper);
// changed as this doesnt work yet so no need to read.
	//sx1276.sx_readRegister_burst(REG_BITRATEMSB, sx_reg_backup[SX_REG_BACKUP_LORA], sizeof(sx_reg_backup[SX_REG_BACKUP_LORA]));

	// put the radio into receive mode
	sx1276.receive(0); //explicit header}
}


/*
 * Processes stuff from rx_fifo
 */
void FanetMac::handle_rx()
{


	if (rx_fifo.size()>0)
	{
		//Serial.println("handle_rx");
	  //  Serial.println(rx_fifo.size());
	}

	/* nothing to do */
	if(rx_fifo.size() == 0)
	{
		/* clean neighbors list */
		for(int i=0; i<neighbors.size(); i++)
		{
			if(neighbors.get(i)->isaround() == false)
				delete neighbors.remove(i);
		}

		return;
	}



	Frame *frm = rx_fifo.front();

	/* build up neighbors list */
	bool neighbor_known = false;
	for(int i=0; i<neighbors.size(); i++)
	{
		if(neighbors.get(i)->addr == frm->src)
		{
			/* update presents */
			neighbors.get(i)->seen();
			neighbor_known = true;
			break;
		}
	}
	/* neighbor unknown until now, add to list */
	if(neighbor_known == false)
	{
		/* too many neighbors, delete oldest member */
		if(neighbors.size() > MAC_NEIGHBOR_SIZE)
			delete neighbors.shift();

		neighbors.add(new NeighborNode(frm->src));
	}



	/* is the frame a forwarded one and is it still in the tx queue? */
	Frame *frm_list = tx_fifo.frame_in_list(frm);
	if(frm_list != NULL)
	{
		/* frame already in tx queue */

		if(frm->rssi > frm_list->rssi + MAC_FORWARD_MIN_DB_BOOST)
		{
			/* somebody broadcasted it already towards our direction */
#if defined(SerialDEBUG) && MAC_debug_mode > 0
			SerialDEBUG.print(millis());
			SerialDEBUG.println(F("### rx frame better than org. dropping both."));
#endif
			/* received frame is at least 20dB better than the original -> no need to rebroadcast */
			tx_fifo.remove_delete(frm_list);
		}
		else
		{
#if defined(SerialDEBUG) && MAC_debug_mode > 0
			SerialDEBUG.print(millis());
			SerialDEBUG.println(F("### adjusting tx time"));
#endif
			/* adjusting new departure time */
			frm_list->next_tx = millis() + random(MAC_FORWARD_DELAY_MIN, MAC_FORWARD_DELAY_MAX);
		}
	}
	else
	{
		if((frm->dest == MacAddr() || frm->dest == my_addr) && frm->src != my_addr)
		{
			/* a relevant frame */
			if(frm->type == FRM_TYPE_ACK)
			{
				if(tx_fifo.remove_delete_acked_frame(frm->src) && myApp != NULL)
					myApp->handle_acked(true, frm->src);
			}
			else
			{
				/* generate ACK */
				if(frm->ack_requested)
					ack(frm);

				/* forward frame */
				if(myApp != NULL)
					myApp->handle_frame(frm);
			}
		}


		/* Forward frame */
		if(frm->forward && tx_fifo.size() < MAC_FIFO_SIZE - 3 && frm->rssi <= MAC_FORWARD_MAX_RSSI_DBM &&
				(frm->dest == MacAddr() || isNeighbor(frm->dest)))
		{
#if defined(SerialDEBUG) && MAC_debug_mode > 0
			SerialDEBUG.print(millis());
			SerialDEBUG.println(F("### adding new forward frame"));
#endif
			/* prevent from re-forwarding */
			frm->forward = false;

			/* generate new tx time */
			frm->next_tx = millis() + random(MAC_FORWARD_DELAY_MIN, MAC_FORWARD_DELAY_MAX);

			/* add to list */

			tx_fifo.add(frm);
			return;
		}
	}

	/* discard frame */
	delete frm;
}

/*
 * get a from from tx_fifo (or the app layer) and transmit it
 */
void FanetMac::handle_tx()
{
	/* still in backoff */
	if(millis() < csma_next_tx)
		return;

	// check if time for ogn packet. - only do this for sx1276 boards at the moment, good luck on writing the sx1272 code phil.....

	if(myApp->is_ogn_broadcast_ready(neighbors.size()) && ogn_on=='y' && ( lora_or_flarm=='h' || lora_or_flarm=='r'  || lora_or_flarm=='t' || lora_or_flarm=='b'))
	{
		Serial.print("Starting OGN");Serial.println(millis());
		ogn_tx_packets++;
		OGN_TxPacket PosPacket;

     	uint8_t ChipVersion = StartRFchipFSK();


		if( ChipVersion!=0x12 )
		{
			Serial.print("ERROR - Starting FSK Failed - Please check");

		}
		else
		{
			if (debug_level>0)
			{
				Serial.println("Switch to OGN successful");
			}
		}


	//	printf("Starting encode of ");
	//	Serial.println(AcftID.Address);
//		Serial.println(millis());

		PosPacket.Packet.HeaderWord=0;
		PosPacket.Packet.Header.Address    = AcftID.Address;         // set address
		PosPacket.Packet.Header.AddrType   = AcftID.AddrType;        // address-type
		PosPacket.Packet.calcAddrParity();                               // parity of (part of) the header

		//printf("encoding\n");

		//PosPacket.Packet.Position.Time=1;
	//	int32_t Latitude = (int16_t)53*60 + 41;
	//	Latitude = Latitude*(int32_t)10000 + 9703;
	//	PosPacket.Packet.EncodeLatitude(Latitude);
	//	int32_t Longitude = -1*((int32_t)2*600000 + (int32_t)15*10000 + (int32_t)7208);
	//PosPacket.Packet.EncodeLatitude(Latitude);
	//

		PosPacket.Packet.Position.Time=1;

		int32_t Latitude = (int16_t)myApp->get_lat_deg()*60 + myApp->get_lat_min();
		Latitude = Latitude*(int32_t)10000 + myApp->get_lat_min_frac();

		if (myApp->get_lat_sign_negative()) Latitude=Latitude*-1;

		PosPacket.Packet.EncodeLatitude(Latitude);

		int32_t Longitude = ((int32_t)myApp->get_lon_deg()*600000 + (int32_t)myApp->get_lon_min()*10000 + (int32_t)myApp->get_lon_min_frac());
		if (myApp->get_lon_sign_negative()) Longitude=Longitude*-1;
		PosPacket.Packet.EncodeLongitude(Longitude);

		PosPacket.Packet.EncodeSpeed(myApp->get_speed());
		PosPacket.Packet.EncodeHeading(myApp->get_heading());
		PosPacket.Packet.EncodeClimbRate(myApp->get_climb());
		PosPacket.Packet.EncodeTurnRate(myApp->get_turnrate());
		PosPacket.Packet.EncodeAltitude(myApp->get_altitude());
		// encode position/altitude/speed/etc. from GPS position
		PosPacket.Packet.Position.Stealth  = AcftID.Stealth;
		PosPacket.Packet.Position.AcftType = AcftID.AcftType;        // aircraft-type
		//printf("Starting TxPacket\n");
		PosPacket.Packet.Whiten();
		PosPacket.calcFEC();

		const uint8_t *TxPktData0=0;
		TxPktData0=PosPacket.Byte();

		uint8_t TxChan = 0;
		//sx1276.Transmit(TxChan, TxPktData0, RX_AverRSSI, 8);
		//delay(500);

		Serial.println("Switch to lora");
		//sx1276.sx_writeRegister_burst(REG_BITRATEMSB, sx_reg_backup[SX_REG_BACKUP_LORA], sizeof(sx_reg_backup[SX_REG_BACKUP_LORA]));

	//	switch_lora();

		sx1276.receive(0);
		myApp->ogn_broadcast_successful();
		// for some reason theres a stray packet that appears, or the irq flags get messed up,  just clear down.
		sx1276.ClearIrqFlags();
		receiveIRQ=false;
		Serial.print("end");Serial.println(millis());
		return;
	}

	/* find next send-able packet */
	/* this breaks the layering. however, this approach is much more efficient as the app layer now has a much higher priority */
	Frame* frm;
	bool app_tx = false;
	if(myApp->is_broadcast_ready(neighbors.size()))
//todo: check if first element of txfifo is not an ACK!
	{

#if defined(SerialDEBUG) && MAC_debug_mode > 0
	//		SerialDEBUG.print(millis());
	//		SerialDEBUG.print(F("### broadcast is ready"));

#endif
		/* the app wants to broadcast the glider state */
		frm = myApp->get_frame();
		if(frm == NULL)
			return;

//todo?? set forward bit only if no inet base station is available, this MAY break the layers
		if(neighbors.size() <= MAC_MAXNEIGHBORS_4_TRACKING_2HOP)
			frm->forward = true;
		else
			frm->forward = false;

		app_tx = true;
	}
	else
	{
#if defined(SerialDEBUG) && MAC_debug_mode > 0
		//	SerialDEBUG.print(millis());
			//SerialDEBUG.print(F("### getting frame from fifo"));

#endif
		/* get a from from the fifo */
		frm = tx_fifo.get_nexttx();
		if(frm == NULL)
			return;

		/* frame w/o a received ack and no more re-transmissions left */
		if(frm->ack_requested && frm->num_tx <= 0)
		{
#if defined(SerialDEBUG) && MAC_debug_mode > 0
			SerialDEBUG.print(millis());
			SerialDEBUG.print(F("### Frame, 0x"));
			SerialDEBUG.print(frm->type, HEX);
			SerialDEBUG.println(F(" NACK!"));
#endif
			if(myApp != NULL)
				myApp->handle_acked(false, frm->dest);
			tx_fifo.remove_delete(frm);
			return;
		}

		/* unicast frame w/o forwarding and it is not a direct neighbor */
		if(frm->forward == false && frm->dest != MacAddr() && isNeighbor(frm->dest) == false)
			frm->forward = true;

		app_tx = false;
	}

	/* serialize frame */
	uint8_t* buffer;
	int blength = frm->serialize(buffer);
	if(blength < 0)
	{
#if defined(SerialDEBUG) && MAC_debug_mode > 0
		SerialDEBUG.print(millis());
		SerialDEBUG.print(F("### Problem serialization. removing. :- error code "));
		SerialDEBUG.println(blength);
#endif
		/* problem while assembling the frame */
		if(app_tx)
			delete frm;
		else
			tx_fifo.remove_delete(frm);
		return;
	}


#if defined(SerialDEBUG) && MAC_debug_mode > 0
	SerialDEBUG.print(millis());
	SerialDEBUG.print(F("### Sending, 0x"));
	SerialDEBUG.print(frm->type, HEX);
	SerialDEBUG.print(F("..."));
#endif

#if defined(SerialDEBUG) && MAC_debug_mode > 1
	/* print whole packet */
	SerialDEBUG.print(F(" "));
	for(int i=0; i<blength; i++)
	{
		SerialDEBUG.print(buffer[i], HEX);
		if(i<blength-1)
			SerialDEBUG.print(F(":"));
	}
	SerialDEBUG.print(F(" "));
#endif

	/* for only a few nodes around, increase the coding rate to ensure a more robust transmission - Added =0 */
	int tx_ret=0;
	serial_tx_packets_counter++;

	if (lora_or_flarm == 'r')
	{
		if (neighbors.size() < MAC_CODING48_THRESHOLD)
			sx1276.setCodingRate4(8);
		else
			sx1276.setCodingRate4(5);

		/* channel free and transmit? */
		tx_ret = sx1276.sendFrame(buffer, blength);
		delete[] buffer;
	}
	if (lora_or_flarm == 'h'  || lora_or_flarm=='t' || lora_or_flarm=='b')
	{

		if (neighbors.size() < MAC_CODING48_THRESHOLD)
			sx1276.setCodingRate4(8);
		else
			sx1276.setCodingRate4(5);

		/* channel free and transmit? */
	    tx_ret = sx1276.sendFrame(buffer, blength);
		delete[] buffer;
	}
	if (lora_or_flarm == 'l')
	{
		if (neighbors.size() < MAC_CODING48_THRESHOLD)
			sx1272.setCodingRate(CR_8);
		else
			sx1272.setCodingRate(CR_5);

		/* channel free and transmit? */
		tx_ret = sx1272.sendFrame(buffer, blength);
		delete[] buffer;
	}


	if(tx_ret == TX_OK)
	{
#if defined(SerialDEBUG) && MAC_debug_mode > 0
		SerialDEBUG.println(F("Fmac done."));
#endif

		if(app_tx)
		{
			/* app tx */
			myApp->broadcast_successful(frm->type);
			delete frm;
		}
		else
		{
			/* fifo tx */

			/* transmission successful */
			if(!frm->ack_requested)
			{
				/* remove frame from FIFO */
				tx_fifo.remove_delete(frm);
			}
			else
			{
				/* update next transmission */
				if(--frm->num_tx > 0)
					frm->next_tx = millis() + (MAC_TX_RETRANSMISSION_TIME * (MAC_TX_RETRANSMISSION_RETRYS - frm->num_tx));
				else
					frm->next_tx = millis() + MAC_TX_ACKTIMEOUT;
			}
		}

		/* ready for a new transmission */
		csma_backoff_exp = MAC_TX_BACKOFF_EXP_MIN;
		csma_next_tx = millis() + MAC_TX_MINTIME;
	}
	else if(tx_ret == TX_RX_ONGOING)
	{
#if defined(SerialDEBUG) && MAC_debug_mode > 0
		SerialDEBUG.print(millis());
		SerialDEBUG.println(F("rx, abort."));
#endif

		if(app_tx)
			delete frm;

		/* channel busy, increment backoff exp */
		if(csma_backoff_exp < MAC_TX_BACKOFF_EXP_MAX)
			csma_backoff_exp++;

		/* next tx try */
		csma_next_tx = millis() + random(1<<(MAC_TX_BACKOFF_EXP_MIN-1), 1<<csma_backoff_exp);

#if defined(SerialDEBUG) && MAC_debug_mode > 0
		SerialDEBUG.print(millis());
		SerialDEBUG.print(F("### backoff ("));
		SerialDEBUG.print(csma_next_tx - millis());
		SerialDEBUG.println(F("ms)"));
#endif
	}
	else
	{
		/* ignoring TX_TX_ONGOING */
#if defined(SerialDEBUG) && MAC_debug_mode > 0
		SerialDEBUG.print(millis());
		SerialDEBUG.println(F("### wow."));
#endif

		if(app_tx)
		{
#if defined(SerialDEBUG) && MAC_debug_mode > 0
		SerialDEBUG.print(millis());
		SerialDEBUG.println(F("### Fmac deleting tranmitted Frame."));
#endif
			delete frm;
		}
	}
}




Frame::Frame()
{
	src = fmac.my_addr;
};

FanetMac fmac = FanetMac();
