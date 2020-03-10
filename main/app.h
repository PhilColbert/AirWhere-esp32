/*
 * app.h
 *
 *  Created on: 17 Oct 2016
 *      Author: sid
 */

#ifndef STACK_APP_H_
#define STACK_APP_H_

#include <math.h>

#include "fanet.h"
//#include "com/serial.h"
#include "fanet_stack/fmac.h"
#include "AirWare.h"
#include "fanet_stack/payload.h"
#include "fanet_stack/sx1272.h"
//extern Sx1272 sx1272;
/*SX1276 support*/
#include "fanet_stack/sx1276.h"


#define APP_VALID_STATE_MS			10000
#define FANET_MANUFACTURER 4


//todo... worst case coding...
#define APP_TYPE1_AIRTIME_MS			40		//actually 20-25ms
//MINTAU is the minimum time before it tries a retransmission (in case the channel was busy)
#define	APP_TYPE1_MINTAU_MS			250
//Time between transmissions
#define	APP_TYPE1_TAU_MS			2000
//_SIZE is simply the payload size of a type 1 packet
#define APP_TYPE1_SIZE				12
// freq between ogn packets and airwhere packets
#define OGN_SEND_RATIO 3

//extern bool framesArrayFree[RX_PACKETS];
//extern Frame *frameData[RX_PACKETS];
//v2
//extern Payload payloadList[RX_PACKETS];
//extern bool payloadReady[RX_PACKETS];
extern Payload payloadList[RX_PACKETS];
extern int payloadReady[RX_PACKETS];
extern long int payload_time_to_send[RX_PACKETS];
extern char packet_repeat;


extern IPAddress apFoundIP;
extern char groundStationMode;

//v4

extern long int tx_led_off_time;
extern bool tx_led_on;

class App : public Fapp
{
private:
	/* units are degrees, seconds, and meter */
	float latitude = NAN;
	float longitude = NAN;
	int altitude;
	float speed;
	float climb;
	float heading;
	float turnrate;

	bool lat_sign_negative;
	int lat_deg;
	int lat_min;
	int lat_min_frac;

	bool lon_sign_negative;
	int lon_deg;
	int lon_min;
	int lon_min_frac;

	//Serial_Interface *mySerialInt = NULL;

	/* ensures the broadcasted information are still valid */
	unsigned long valid_until = 0;

	/* determines the tx rate */
	unsigned long last_tx = 0;
	unsigned long aw_packets_sent = 0;
	unsigned long next_tx = 0;

#ifdef FANET_NAME_AUTOBRDCAST
	char name[20] = "\0";
	bool brdcast_name = false;

	int serialize_name(uint8_t*& buffer)
	{
		const int namelength = strlen(name);
		buffer = new uint8_t[namelength];
		memcpy(buffer, name, namelength);
		return namelength;
	}
#endif

	//v1.1

    void buf_absolut2coord(float &lat, float &lon, uint8_t *buf)
    {
      int32_t lat_i = 0;
      int32_t lon_i = 0;

      if(buf == NULL || lat == NULL || lon == NULL)
        return;

      ((uint8_t*)&lat_i)[0] = buf[0];
      ((uint8_t*)&lat_i)[1] = buf[1];
      ((uint8_t*)&lat_i)[2] = buf[2];

      ((uint8_t*)&lon_i)[0] = buf[3];
      ((uint8_t*)&lon_i)[1] = buf[4];
      ((uint8_t*)&lon_i)[2] = buf[5];

      lat = (float) lat_i / 93206.0f;
      lon = (float) lon_i / 46603.0f;
    }
//v1.1 - changes - see older file.

	int serialize_tracking(uint8_t*& buffer)
	{
		buffer = new uint8_t[APP_TYPE1_SIZE];

		Frame::coord2payload_absolut(latitude, longitude, buffer);

		/* altitude set the lower 12bit */
		int alt = constrain(altitude, 0, 8190);
		if(alt > 2047)
			((uint16_t*)buffer)[3] = ((alt+2)/4) | (1<<11);				//set scale factor
		else
			((uint16_t*)buffer)[3] = alt;
		/* online tracking */
		((uint16_t*)buffer)[3] |= !!do_online_tracking<<15;
		/* aircraft type */
		((uint16_t*)buffer)[3] |= (aircraft_type&0x7)<<12;

		/* Speed */
		int speed2 = constrain((int)roundf(speed*2.0f), 0, 635);
		if(speed2 > 127)
			buffer[8] = ((speed2+2)/5) | (1<<7);					//set scale factor
		else
			buffer[8] = speed2;

		/* Climb */
		int climb10 = constrain((int)roundf(climb*10.0f), -315, 315);
	   if(abs(climb10) > 63)
	   		buffer[9] = ((climb10 + (climb10>=0?2:-2))/5) | (1<<7);			//set scale factor
		else
			buffer[9] = climb10 & 0x7F;

		/* Heading */

		buffer[10] = constrain((int)roundf(heading*256.0f)/360.0f, 0, 255);

		/* Turn rate */
		if(!isnan(turnrate))
		{
			int turnr4 = constrain((int)roundf(turnrate*4.0f), 0, 255);
			if(abs(turnr4) > 63)
				buffer[9] = ((turnr4 + (turnr4>=0?2:-2))/4) | (1<<7);			//set scale factor
			else
				buffer[9] = turnr4 & 0x7f;
			return APP_TYPE1_SIZE;
		}
		else
		{
			return APP_TYPE1_SIZE - 1;
		}
	}
public:
	int aircraft_type;
	bool do_online_tracking;
	bool gpslocked=false;

	void set(float lat, float lon, float alt, float speed, float climb, float heading, float turn,
			int lat_sign_negative, int lat_deg,	int lat_min, int lat_min_frac,
			bool lon_sign_negative,	int lon_deg, int lon_min, int lon_min_frac)
	{
		/* currently only used in linear mode */
		//noInterrupts();

	    /*   if (longitude > -2 || longitude < -3)
	       {
	    	   Serial1.println("------------------");
	    	   Serial1.print("lat->");Serial1.print(lat);Serial1.println("<-");
	           Serial1.print("lon->");Serial1.print(lon);Serial1.println("<-");
	           Serial1.print("alt->");Serial1.print(alt);Serial1.println("<-");
	           Serial1.print("speed");Serial1.print(speed);Serial1.println("<-");
	           Serial1.print("heading->");Serial1.print(heading);Serial1.println("<-");
	       }
*/

		latitude = lat;
		longitude = lon;

		// change back
	//	Serial.println("app setting");
		altitude = roundf(alt);
	//	altitude = rand() % 10000;
		this->speed = speed;
		this->climb = climb;
		if(heading < 0.0f)
			heading += 360.0f;
		this->heading = heading;
		turnrate = turn;

		valid_until = millis() + APP_VALID_STATE_MS;

		if (longitude > -2 || longitude < -3)
			       {
		                  //Serial1.print("lat->");Serial1.print(lat);Serial1.print("<-");
				           //Serial1.print("lon->");Serial1.print(lon);Serial1.println("<-");
		//	Serial1.println("###############################################");

			       }

		this->lat_sign_negative=lat_sign_negative;
		this->lat_deg=lat_deg;
		this->lat_min=lat_min;
		this->lat_min_frac=lat_min_frac;

		this->lon_sign_negative=lon_sign_negative;
		this->lon_deg=lon_deg;
		this->lon_min=lon_min;
		this->lon_min_frac=lon_min_frac;



		//interrupts();
	}

	float get_speed(){return speed;}
	float get_heading()	{return heading;}
	float get_climb(){return climb;}
	float get_turnrate(){return turnrate;}
	float get_altitude(){return altitude;}
	float get_lat_sign_negative(){return lat_sign_negative;}
	float get_lat_deg(){return lat_deg;}
	float get_lat_min(){return lat_min;}
	float get_lat_min_frac(){return lat_min_frac;}
	float get_lon_sign_negative(){return lon_sign_negative;}
	float get_lon_deg(){return lon_deg;}
	float get_lon_min(){return lon_min;}
	float get_lon_min_frac(){return lon_min_frac;}



	void set_gps_lock(char gpslockchar)
	{

      if ( gpslockchar == 'A')
      {
    	  gpslocked=true;
      }
      else
      {
    	  gpslocked=false;
	  }

      return;
	}

	void get_params( int &i)
	{
		i=aircraft_type;
	}

	bool is_ogn_broadcast_ready(int num_neighbors)
	{
		// if we are a ground station then dont transmit
		if (groundStationMode == 'y')
		{
			return false;
		}

		// is the state valid?


		if( isnan(latitude) || isnan(longitude) || !gpslocked)
			return false;

		// determine if its time to send something (again)
		const int tau_add = (num_neighbors/10 + 1) * APP_TYPE1_TAU_MS;


		if(aw_packets_sent<OGN_SEND_RATIO || last_tx + tau_add > millis())
		{
			return false;
		}


		return true;
	}



	void ogn_broadcast_successful()
	{
		last_tx = millis();
		aw_packets_sent=0;

		if ( lora_or_flarm == 'h' || lora_or_flarm=='t' || lora_or_flarm=='f' || lora_or_flarm=='r' || lora_or_flarm=='b')
		{
			//Serial.println("Blink*********************");
			gpio_set_level(GPIO_NUM_2, 1);
			tx_led_on=true;
			tx_led_off_time=millis()+TX_LED_ON;
		}


				if ( wiredDirectly == 'y' || wiredDirectly == 'o' || wiredDirectly == 'v' || wiredDirectly == 's' || wiredDirectly == 'f')
				{
					// streamData=("$AWARE,TRANSMITTING-LOCATION\r\n"+streamData);
					//queue stream data up , not just send - needs ordering.
					writeDataToSerial("$AW,TX-OGN,*");

				}
				else
				{
					// network stuff goes straight out as we send out a line at time.
					// need to merge standalone stuff.



					if (navSofware == 'L')
					{
						writeDataToWifi( airwhere_client, "$AW,TX-OGN,*");
					}
					//v1.1
					if (navSofware == 'X' || navSofware == 'A' || navSofware == 'K')
					{
						sendUDPPacket ( "$AW,TX,*", apFoundIP, true );
					}

				}

			}

	/* device -> air */
	bool is_broadcast_ready(int num_neighbors)
	{

		// do not send if we are in ogn mode.
		if(aw_packets_sent>OGN_SEND_RATIO && ogn_on=='y')
		{
			return false;
		}

		/* if we are a ground station then dont transmit */
	    if (groundStationMode == 'y')
        {
	    	return false;
        }


		/* is the state valid? */
		if(millis() > valid_until || isnan(latitude) || isnan(longitude) || !gpslocked)
			return false;

		/* in case of a busy channel, ensure that frames from the fifo get also a change */
		if(next_tx > millis())
			return false;

		/* determine if its time to send something (again) */
		const int tau_add = (num_neighbors/10 + 1) * APP_TYPE1_TAU_MS;
		if(last_tx + tau_add > millis())
			return false;

		if (ogn_on=='y' )
		{
		  aw_packets_sent++;
		}

		return true;

	}

	void broadcast_successful(int type)
	{
		last_tx = millis();
		//Serial.print("bs lasttx");Serial.println(last_tx);

		if ( lora_or_flarm == 'h' || lora_or_flarm=='t' || lora_or_flarm=='f' || lora_or_flarm=='r' || lora_or_flarm=='b')
		{
		//	Serial.println("Blink*********************");
			gpio_set_level(GPIO_NUM_2, 1);
			tx_led_on=true;
			tx_led_off_time=millis()+TX_LED_ON;
		}


		if ( wiredDirectly == 'y' || wiredDirectly == 'o' || wiredDirectly == 'v' || wiredDirectly == 's' || wiredDirectly == 'f')
		{
			// streamData=("$AWARE,TRANSMITTING-LOCATION\r\n"+streamData);
			//queue stream data up , not just send - needs ordering.
			writeDataToSerial("$AW,TX,*");

		}
		else
		{
			// network stuff goes straight out as we send out a line at time.
			// need to merge standalone stuff.



			if (navSofware == 'L')
			{
				writeDataToWifi( airwhere_client, "$AW,TX,*");
			}
			//v1.1
			if (navSofware == 'X' || navSofware == 'A' || navSofware == 'K')
			{
				sendUDPPacket ( "$AW,TX,*", apFoundIP, true );
			}

		}

	}


	Frame *get_frame()
	{
		/* prepare frame */
		Frame *frm = new Frame(fmac.my_addr);
#ifdef FANET_NAME_AUTOBRDCAST
		static uint32_t framecount = 0;
		if(brdcast_name && (framecount & 0x7F) == 0)
		{
			/* broadcast name */
#if defined(SerialDEBUG) && MAC_debug_mode > 0
			SerialDEBUG.println("getting name");
#endif
			frm->type = FRM_TYPE_NAME;
			frm->payload_length = serialize_name(frm->payload);
		}
		else
		{
#endif
#if defined(SerialDEBUG) && MAC_debug_mode > 0
		//	SerialDEBUG.println("serialize_tracking");
#endif
			/* broadcast tracking information */
			frm->type = FRM_TYPE_TRACKING;
			frm->payload_length = serialize_tracking(frm->payload);
#ifdef FANET_NAME_AUTOBRDCAST
		}
		framecount++;
#endif

		/* in case of a busy channel, ensure that frames from the fifo gets also a change */
		next_tx = millis() + APP_TYPE1_MINTAU_MS;

		return frm;
	}

#ifdef FANET_NAME_AUTOBRDCAST
	/* Name automation in case the host application does not know this... */
	void set_name(char *devname) { snprintf(name, sizeof(name), devname); };
	void allow_brdcast_name(boolean value)
	{
		if(value == false)
			brdcast_name = false;
		else
			brdcast_name = (strlen(name)==0?false:true);
	};
#endif

	void begin()
	{
		//mySerialInt = &si;
	}

	//void begin(Serial_Interface &si)
	//{
		//mySerialInt = &si;
	//}

	/* air -> device */
	void handle_acked(boolean ack, MacAddr &addr)
	{
		SerialDEBUG.println("ack_frame");
		//if(mySerialInt == NULL)
		//	return;

		//mySerialInt->handle_acked(ack, addr);
	}

	void handle_frame(Frame *frm)
	{

#if defined(SerialDEBUG) && MAC_debug_mode > 0

		SerialDEBUG.print(millis());
		SerialDEBUG.print ("### Handling frame :- ");
		/* src_manufacturer,src_id,broadcast,signature,type,payloadlength,payload */
		SerialDEBUG.print(frm->src.manufacturer, HEX);
		SerialDEBUG.print(',');
		// needs sending as HEX?!
		SerialDEBUG.print(frm->src.id, DEC);
		SerialDEBUG.print(',');
		SerialDEBUG.print(frm->dest == MacAddr());	//broadcast
		SerialDEBUG.print(',');
		SerialDEBUG.print(frm->signature, HEX);
		SerialDEBUG.print(',');
		SerialDEBUG.print(frm->type, HEX);
		SerialDEBUG.print(',');
		SerialDEBUG.print(frm->payload_length, HEX);
		SerialDEBUG.print(',');
		for(int i=0; i<frm->payload_length; i++)
		{
			char buf[8];
			sprintf(buf, "%02X", frm->payload[i]);
			SerialDEBUG.print(buf);
		}

		SerialDEBUG.println();
		SerialDEBUG.flush();
#endif

				switch (frm->type)
				{
				  //ACK (Type = 0)
				  case 0:
			        break;
			      case 1:
			      //Tracking (Type = 1)
			      // only build parse and process incoming message if we have lock
			    	//  Serial.println("found tracking frame*****");

				    if (gpslocked && !isnan(latitude) && !isnan(longitude))
				    {
				    //    Serial.print("send_latitude-");Serial.println(latitude);
     				//    Serial.print("send_longitude-");Serial.println(longitude);


					  Payload payload(frm->src.manufacturer,
					  frm->src.id,
					  frm->dest.id,
					  frm->signature,
					  frm->type,
				      frm->rssi,
					  frm->payload_length,
					  frm->payload,
					  latitude,
					  longitude);

//v2
					  /*
				      for ( int packetNumber=0; packetNumber <  RX_PACKETS; packetNumber++)
					  {
					  // if its not ready there a free space in the array
					    if (!payloadReady[packetNumber] )
						{
				//	    	Serial.println("Adding FRAME into **********");
				//	    	Serial.println(packetNumber);
						  payloadList[packetNumber]=payload;
						  payloadReady[packetNumber]=true;
						  break;
						}
					  }
					  */



		              if ( packet_repeat=='y')
		              {

		                  // if its not ready there a free space in the array


		                  // first search array, see if we already have this pilot in the array
		                  // then if we do, update this pilots details and set to repeat
		                  // if not, start at the start again and find the first 0 and add into array.

		                  bool pilot_added=false;

		                  for ( int packetNumber=0; packetNumber <  RX_PACKETS; packetNumber++)
		                  {
		                      if (payloadList[packetNumber].manufacturer==payload.manufacturer &&
		                              payloadList[packetNumber].id==payload.id )
		                      {
		                         if (debug_level>0)
		                         {
		                        	 Serial.println("Adding into array - pilot found - packet repeat is on");
		                         }
		                          payloadList[packetNumber]=payload;
		                          payloadReady[packetNumber]=PACKET_REPEAT_NUMBER;
		                          // send immediately.
		                          payload_time_to_send[packetNumber]=0;
		                          pilot_added=true;
		                      }
		                  }

		                  if (!pilot_added)
		                  {
		                      for ( int packetNumber=0; packetNumber <  RX_PACKETS; packetNumber++)
		                      {
		                          if (payloadReady[packetNumber]==0 )
		                          {
		                        	  if (debug_level>0)
		                        	  {
		                        		  Serial.println("Adding into array - new pilot - packet repeat is on");
		                        	  }

		                              payloadList[packetNumber]=payload;
		                              payloadReady[packetNumber]=PACKET_REPEAT_NUMBER;
		                              // send immediately.
		                              payload_time_to_send[packetNumber]=0;
		                              break;
		                          }
		                      }
		                  }
		              }
		              else
		              {
		                  for ( int packetNumber=0; packetNumber <  RX_PACKETS; packetNumber++)
		                  {
		                      if (payloadReady[packetNumber]==0 )
		                      {
		                    	  if (debug_level>0)
		                    	  {
		                    		  Serial.println("Adding into array - packet repeat is off");
		                    	  }


		                          payloadList[packetNumber]=payload;
		                          payloadReady[packetNumber]=1;
		                          payload_time_to_send[packetNumber]=0;
		                          break;
		                      }
		                  }
		              }


				   }

				    break;

				  case 2:
     			  //Name (Type = 2)
					  //Add Support for Name into LK&XCSoar and then complete following to suit.
					  //Payload payload(frm->src.manufacturer, frm->src.id, frm->payload_length,frm->payload);
				  	break;
				  case 3:
				  //Message (Type = 3)
				  	break;
				  case 4:
				  //Service (Type = 4)
				  	break;
				  case 5:
				  //Landmarks (Type = 5)
				  	break;
				}



		//		Serial.println("frm->type:");		Serial.println(frm->type);
	//	Serial.println("payload lat");Serial.println(payloadList[0].latitude*1000);
	//	Serial.println("payload lon");	Serial.println(payloadList[0].longitude*1000);
	//	Serial.println("online_tracking");Serial.println(payloadList[0].online_tracking);


	}
	void handle_irq(char lrh)
	{
		if (lrh=='l')
		{
			sx1272.handle_irq();
		}
		if (lrh=='r')
		{
			sx1276.handle_irq();
		}
		if ( lrh=='h' || lrh=='t' || lrh=='b' )
		{
			sx1276.handle_irq();
		}



	}

};




//extern App app;

#endif /* STACK_APP_H_ */
