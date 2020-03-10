#include "xmodem.h"

static uint16_t crc_update(uint16_t crc_in, int incr)
{
	uint16_t xor1 = crc_in >> 15;
	uint16_t out = crc_in << 1;

	if (incr) out++;

	if (xor1) out ^= CRC_POLY;

	return out;
}

static uint16_t crc16(const uint8_t *data, uint16_t size)
{
	uint16_t crc, i;

	for (crc = 0; size > 0; size--, data++)
		for (i = 0x80; i; i >>= 1)
			crc = crc_update(crc, *data & i);

	for (i = 0; i < 16; i++)
		crc = crc_update(crc, 0);

	return crc;
}
static uint16_t swap16(uint16_t in)
{
	return (in >> 8) | ((in & 0xff) << 8);
}

//int xmodem_transmit(Serial *serial, const char *filename)
//Serial is Global;

int xmodem_transmit()
{
     Serial.println(ESP.getFreeHeap());

     HTTPClient http;
     http.begin("http://www.airwhere.co.uk/downloads/fanet.xlb" );
     http.setTimeout(8000);

     delay(500);

     int code = http.GET();
     size_t len = http.getSize();

     Serial.printf("[xmodem]  - code: %d\n", code);
     Serial.printf("[xmodem]  - len: %d\n", len);

     if(code <= 0)
     {
         printf("[httpUpdate] HTTP error: %s\n", http.errorToString(code).c_str());
         http.end();
         return -1;

     }

    // pinMode(13, OUTPUT);
   //  digitalWrite(13, HIGH);

     pinMode(FLARM_BOARD_RESET_PIN, OUTPUT);
     digitalWrite(FLARM_BOARD_RESET_PIN, LOW);
     delay(500);
     digitalWrite(FLARM_BOARD_RESET_PIN, HIGH);

	if(!flarmSerial)
		return -1;

	int ret;
	uint8_t answer;

	uint8_t eof = X_EOF;
	struct xmodem_chunk chunk;

	WiFiClient * tcp = http.getStreamPtr();

	//WiFiUDP::stop();
	//WiFiClient::stopAllExcept(tcp);

	Serial.println("Sending Data ...\n");

	chunk.block = 1;
	chunk.start = X_STX;

	int sent_blocks=0;
	int failures = 0;

	flarmSerial.flush();
	flarmSerial.readString();

    noInterrupts();

	bool read_tcp=true;

	while (len)
	{
		size_t z = 0;
		int next = 0;

		z = min(len, sizeof(chunk.payload));

		if (read_tcp)
		{
		  tcp->readBytes(chunk.payload, z);
		}

		memset(chunk.payload + z, 0xff, sizeof(chunk.payload) - z);

		chunk.crc = swap16(crc16(chunk.payload, sizeof(chunk.payload)));
		chunk.block_neg = 0xff - chunk.block;

	    Serial.print(".");

		ret = flarmSerial.write((uint8_t*) &chunk, sizeof(chunk));

		if (ret != sizeof(chunk))
		{
		    Serial.println("Invalid chunk size written");
		    tcp->stop();
		    http.end();
			return -5;
		}


		size_t slen=0;
		slen=flarmSerial.available();

		answer=0;

		if (slen!=0)
		{
		   answer=flarmSerial.read();
		///   Serial.print("data size ");
		//   Serial.print(slen);
		}

		switch (answer)
		{

            case X_C:
               failures++;
               Serial.print("X_C");
               delay(100);
               read_tcp=false;
               break;
			case X_NAK:
				failures++;
				Serial.print("X_NAK");
				delay(100);
	            read_tcp=false;
				break;
			case X_ACK:
				next = 1;
				sent_blocks++;
				Serial.print("X_ACK");
				delay(100);
				read_tcp=true;
				break;
			default:
				failures++;
				Serial.print(">");
			    Serial.print(answer);
				Serial.print(":");
				Serial.print((char)answer);
				Serial.print("<");
				delay(100);
				read_tcp=false;
				break;
		}

		if (next)
		{
			chunk.block++;
			len -= z;
		}

		if(failures > 200)
			break;

	}

	ret = flarmSerial.write(&eof, sizeof(eof));
    if (ret != sizeof(eof))
    {
        Serial.println(" Failure sending EOF ");
        return -7;
    }

    tcp->stop();
    http.end();

    if (failures>200)
    {
        Serial.printf( "Xmodem FAILED. Failure %d. Blocks %d\n", failures, sent_blocks);
    }
    else
    {
	    Serial.printf( "Xmodem done. Failure %d. Blocks %d\n", failures, sent_blocks);
    }

	return failures>100?-2:0;
}
