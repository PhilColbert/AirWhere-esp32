
#ifndef Payload_H
#define Payload_H

class Payload {
   public:
      Payload();
      ~Payload();
      Payload(int in_manufacturer, int in_id, int in_dest, int in_signature,
    		  int type , int in_rssi, int payload_length, uint8_t *payload , float lat, float lon );

      Payload(int in_manufacturer, int in_id, int bcast, int in_signature,int in_type,
                              int payload_length, uint8_t *payload);

      int manufacturer = NAN;
	  int id = NAN;
	  int dest = NAN;
	  int signature = NAN;
	  int rssi= NAN;

  	  float latitude = NAN;
      float longitude = NAN;
      bool online_tracking = NAN;
      int aircraft_type = NAN;
   	  int altitude = NAN;
  	  float speed = NAN;
  	  float climb = NAN;
  	  float heading = NAN;

  	// set to false if any of the variables are out of scope.
  	bool payload_verified=true;

   private:
  	float local_latitude = NAN;
  	float local_longitude = NAN;

    float buf2coord(uint16_t *buf,float mycoord)
  	{
  	 // decode buffer
  	 bool odd = !!((1<<15) & *buf);
  	 int16_t sub_deg_int = (*buf&0x7FFF) | (1<<14&*buf)<<1;
  	 const float sub_deg = sub_deg_int / 32767.0f;

  	 float mycood_rounded = roundf(mycoord);
  	 bool mycoord_isodd = ((int)mycood_rounded) & 1;

  	 // target outside our segment. estimate where it is in
  	 if(mycoord_isodd != odd)
  	 {
  	  // adjust deg segment
  	  const float mysub_deg = mycoord - mycood_rounded;
  	  if(sub_deg > mysub_deg)
  	   mycood_rounded--;
  	  else
  	   mycood_rounded++;
  	 }

  	 return mycood_rounded + sub_deg;
  	}

    //v1.1
    void buf_absolut2coord(float &lat, float &lon, uint8_t *buf)
    {
    	if(buf == NULL)
    		return;

    	// integer values /
    	int32_t lati = buf[2]<<16 | buf[1]<<8 | buf[0];
    	if(lati & 0x00800000)
    		lati |= 0xFF000000;
    	int32_t loni = buf[5]<<16 | buf[4]<<8 | buf[3];
    	if(loni & 0x00800000)
    		loni |= 0xFF000000;

    	lat = (float)lati / 93206.0f;
    	lon = (float)loni / 46603.0f;

    }

  	float buf2ufloat(uint8_t buf, float scale)
  	{
  		 float value = (float)(buf&0x7F);
  		 if(buf & 1<<7)
  		  return value*scale;
  		 else
  		  return value;
  	}

  	float buf2sfloat(uint8_t buf, float scale)
  	{
  	 int8_t value8 = (buf&0x7F) | (buf&(1<<6))<<1;
  	 if(buf & 1<<7)
  	  return ((float)value8) * scale;
  	 else
  	  return (float) value8;
  	}


  	bool buf2onlinetracking(uint8_t *buf)
  	{
  	 return (*buf>>7);
  	}

  	int buf2aircrafttype(uint16_t *buf)
  	{
  	 return (*buf>>12) & 0x7;
  	}

  	int buf2altitude(uint16_t *buf)
  	{
  	 int alt = *buf&0x7FF;
  	 if(*buf & 1<<11)
  	  alt*=4;

  	 return alt;
  	}


  	void process_payload_type_1(int payload_length,uint8_t *payload)
  	{
//v1.1
        if (payload_length==9)
        {
          // format for 9
        }

  		if (payload_length==10)
  		{

  		  latitude = buf2coord((uint16_t *)&payload[0],local_latitude);
          longitude = buf2coord((uint16_t *)&payload[2],local_longitude);
          online_tracking=buf2onlinetracking((uint8_t *)&payload[5]);
          aircraft_type = buf2aircrafttype((uint16_t *)&payload[4]);
   	   	  altitude = buf2altitude ((uint16_t *)&payload[4]);
   	  	  speed = buf2ufloat(payload[6], 5.0f) / 2.0f;
   	  	  climb = buf2sfloat(payload[7], 5.0f) / 10.0f;;
   	  	  heading = ((((float)payload[8])*360.0f)/256.0f);

  		}
//v1.1
        if (payload_length==11 || payload_length==12)
        {
          buf_absolut2coord(latitude, longitude, ((uint8_t *) &payload[0]));
          online_tracking=buf2onlinetracking((uint8_t *)&payload[7]);
          aircraft_type = buf2aircrafttype((uint16_t *)&payload[6]);
          altitude = buf2altitude ((uint16_t *)&payload[6]);
          speed = buf2ufloat(payload[8], 5.0f) / 2.0f;
          climb = buf2sfloat(payload[9], 5.0f) / 10.0f;;
          heading = ((((float)payload[10])*360.0f)/256.0f);
        }

        if (payload_length==12)
        {
            //sort turn rate;
        }

// NEED to ADD payload length 12 = turnrate.
  	}

};


Payload::Payload()
{

}

Payload::~Payload()
{

}


Payload::Payload(int in_manufacturer, int in_id, int in_dest, int in_signature,
		         int type, int in_rssi, int payload_length, uint8_t *payload, float lat, float lon)
{
//v1.1
	// we only deal with manus of 0-9 at the moment - needs changing, 11hex = 17dec = skytraxx who also is 1.
//v3 fixed.
	/*manufacturer=in_manufacturer;

	if (manufacturer==17)
	{
		manufacturer=1;
	}
*/
	manufacturer=in_manufacturer;
	id=in_id;
	dest=in_dest;
	signature=in_signature;
	rssi=in_rssi;
	local_latitude=lat;
    local_longitude=lon;
    process_payload_type_1(payload_length, payload);

}

Payload::Payload(int in_manufacturer, int in_id, int bcast, int in_signature,int in_type,
                  int payload_length, uint8_t *payload)
{


    manufacturer=in_manufacturer;
//v3 fixed
    //if (manufacturer==17)
    //{
     //   manufacturer=1;
    //}

    id=in_id;
    signature=in_signature;

    process_payload_type_1(payload_length, payload);

}


#endif
