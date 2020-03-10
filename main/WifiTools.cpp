#include <Arduino.h>
//ESP32 #include <WiFiClient.h>
#include "AirWare.h"
#include "WifiTools.h"
#include "aw_ble.h"


// #include "airwhere_ble.h"



//ESP32 #include <ESP8266WiFi.h>
//ESP32 #include <DNSServer.h>
//#include "sx1272.h"

//ESP32 extern "C" {
//ESP32 #include <user_interface.h>
//ESP32 }
//extern HardwareSerial GPSSerial;

//v1.1

bool is_valid_command(String characters, int &i)
{

    //$GPGGA
	//$GNGGA
	//$GPRMC
	//$GNRMC
	//$GFPALARM
	//$BSD

    if ((characters.charAt(i)=='$') &&  ( ( characters.charAt(i+3)== 'G' || characters.charAt(i+3)== 'R' || characters.charAt(i+3)== 'P') &&
    		                            ( ( characters.charAt(i+4)== 'G' || characters.charAt(i+4)== 'M' || characters.charAt(i+4)== 'A') ||
    		                             ( characters.charAt(i+1)== 'B'  && characters.charAt(i+2)== 'S' ) ) ))
	//if (characters.charAt(i)=='$' && ( characters.charAt(i+3)== 'G' || characters.charAt(i+3)== 'R' ) && ( characters.charAt(i+4)== 'G' || characters.charAt(i+4)== 'M' ) )

    {
      return true;
    }
    else
    {
      return false;
    }
}



String readAllDataOnWifi(WiFiClient LK8000Client, String TCPBuffer , bool &fullLine)
{

  size_t wlen=LK8000Client.available();

  // For some reason if the incoming tcp buffer is too big it crashes the system, limit it to 512, we might miss a few lines now and then
  // but better than crashing the system, seems windows CE doesnt transmit data very well and stalls, kobo is fine :)
  
  if ( wlen > TCP_BUFFER_MAX_LENGTH )
  {
    LK8000Client.flush();
    DEBUG_SERIAL_UART_MAX("[%ld] - %%%%%%%% TCP data bytes available :- %d - Flushing WifiClient Buffer\r\n", millis(),wlen);
    wlen=LK8000Client.available();
    DEBUG_SERIAL_UART_MAX("[%ld] - %%%%%%%% TCP data bytes available :- %d - AFTER Flushing WifiClient Buffer\r\n", millis(),wlen);
    return "";
  }

  if (wlen!=0)
  {
  
    DEBUG_SERIAL_UART_MAX("[%ld] - TCP data bytes available :- %d \r\n", millis(),wlen);
     
    last_data_received=millis();
    uint8_t sbuf[wlen];
    char cbuf[wlen];
    
    LK8000Client.readBytes(sbuf, wlen);        
     
    DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnWifi Point 2 \r\n", millis());
     
    memset(cbuf, 0, wlen);

    DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnWifi Point 2a \r\n", millis());
     
    memcpy(&cbuf,&sbuf, wlen);
   
    DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnWifi Point 3 \r\n", millis());

    TCPBuffer=TCPBuffer+String(cbuf).substring(0,wlen);
  }
  else
  {
     DEBUG_SERIAL_UART_MAX("[%ld] - No Data on socket at this point \r\n", millis());
  }
  
  bool requiredNMEA=false;
  int i;

   
  if ( TCPBuffer.length()<5)
  {
    return TCPBuffer;
  }

  // make 100% sure length>0

  int len=TCPBuffer.length()-4;

  if (len<0)
  {
    len=0;
  }
  
  for (i=0;i<len;i++)
  {
  //v1.1
    //if (TCPBuffer.charAt(i)=='$' && ( TCPBuffer.charAt(i+3)== 'G' || TCPBuffer.charAt(i+3)== 'R' ) && ( TCPBuffer.charAt(i+4)== 'G' || TCPBuffer.charAt(i+4)== 'M' ) )
	if (is_valid_command(TCPBuffer, i) )
    {
     /// requiredNMEA=true;
      break;
    }
  }  

  DEBUG_SERIAL_UART_MAX("[%ld] - Returning data at found character :- %d \r\n", millis(),i);

  fullLine=false;
  
  for (int i=0;i<TCPBuffer.length();i++)
  { 

//	  DEBUG_SERIAL_UART_S("[" + String(millis()) + "] -" + TCPBuffer.charAt(i)+"-");
    if (TCPBuffer.charAt(i)=='\n' || TCPBuffer.charAt(i)=='\r')
    {
      DEBUG_SERIAL_UART_S("[" + String(millis()) + "] -"+"Found end of Line");
      fullLine=true;
      break;
    }
   
  }  
  
  DEBUG_SERIAL_UART_MAX("[%ld] - Returning data at found CR - character :- %d \r\n", millis(),i);

  DEBUG_SERIAL_UART_S("[" + String(millis()) + "] - - Returning data was :-  >>" + TCPBuffer + "\r\n");
  DEBUG_SERIAL_UART_S("[" + String(millis()) + "] - - Returning data is :-  >>" + TCPBuffer.substring(i) + "\r\n");


// i is only effected at the start  - not by the lower if command - as its declared in the if statement so it runs from where it found the valid nmea line.
 // if (requiredNMEA)
 // {
    return TCPBuffer.substring(i);
//  }
//  else
//  {
 //   return "";
 // }
 }

String readAllDataOnWifi( String TCPBuffer , bool &fullLine)
{

	char udp_flight_software_pkt[2056]={};
	int wlen = udp_flight_software.parsePacket();



  // For some reason if the incoming tcp buffer is too big it crashes the system, limit it to 512, we might miss a few lines now and then
  // but better than crashing the system, seems windows CE doesnt transmit data very well and stalls, kobo is fine :)

  if ( wlen > UDP_BUFFER_MAX_LENGTH )
  {
	udp_flight_software.flush();
	udp_flight_software.readString();

	Serial.println(" > 1024 arrived : - Flushing WifiClient Buffer");

    DEBUG_SERIAL_UART_MAX("[%ld] - %%%%%%%% TCP data bytes available :- %d - Flushing WifiClient Buffer\r\n", millis(),wlen);
    wlen = udp_flight_software.parsePacket();
    DEBUG_SERIAL_UART_MAX("[%ld] - %%%%%%%% TCP data bytes available :- %d - AFTER Flushing WifiClient Buffer\r\n", millis(),wlen);
    return "";
  }

  if (wlen!=0)
  {

    DEBUG_SERIAL_UART_MAX("[%ld] - TCP data bytes available :- %d \r\n", millis(),wlen);

    last_data_received=millis();
    //uint8_t sbuf[wlen];
   // char cbuf[wlen];

    int len = udp_flight_software.read(udp_flight_software_pkt, 2056);


    if (len > 0) {
    	udp_flight_software_pkt[len] = '\0';
     }

   // memset(cbuf, 0, wlen);
  //  memcpy(&cbuf,&sbuf, wlen);

    if (debug_level>0)
    {
    	Serial.println("pkt size");
    	Serial.println(wlen);
    	Serial.println("read size");
    	Serial.println(len);

    	Serial.println(udp_flight_software_pkt);
    	Serial.write(udp_flight_software_pkt);
    }
	//Serial.println("here");

	//for (int i=0;i<200;i++)
	//{
	//	Serial.print(udp_flight_software_pkt[i]);
	//	delay(50);
	//}

//	Serial.println("here1");

    TCPBuffer=TCPBuffer+ String(udp_flight_software_pkt);
  }
  else
  {
     DEBUG_SERIAL_UART_MAX("[%ld] - No Data on socket at this point \r\n", millis());
  }

  bool requiredNMEA=false;
  int i;


  if ( TCPBuffer.length()<5)
  {
    return TCPBuffer;
  }

  // make 100% sure length>0

  int len=TCPBuffer.length()-4;

  if (len<0)
  {
    len=0;
  }

  for (i=0;i<len;i++)
  {
  //v1.1
    //if (TCPBuffer.charAt(i)=='$' && ( TCPBuffer.charAt(i+3)== 'G' || TCPBuffer.charAt(i+3)== 'R' ) && ( TCPBuffer.charAt(i+4)== 'G' || TCPBuffer.charAt(i+4)== 'M' ) )
	if (is_valid_command(TCPBuffer, i) )
    {
     /// requiredNMEA=true;
      break;
    }
  }

  DEBUG_SERIAL_UART_MAX("[%ld] - Returning data at found character :- %d \r\n", millis(),i);

  fullLine=false;

  for (int i=0;i<TCPBuffer.length();i++)
  {

//	  DEBUG_SERIAL_UART_S("[" + String(millis()) + "] -" + TCPBuffer.charAt(i)+"-");
    if (TCPBuffer.charAt(i)=='\n' || TCPBuffer.charAt(i)=='\r')
    {
      DEBUG_SERIAL_UART_S("[" + String(millis()) + "] -"+"Found end of Line");
      fullLine=true;
      break;
    }

  }

  DEBUG_SERIAL_UART_MAX("[%ld] - Returning data at found CR - character :- %d \r\n", millis(),i);

  DEBUG_SERIAL_UART_S("[" + String(millis()) + "] - - Returning data was :-  >>" + TCPBuffer + "\r\n");
  DEBUG_SERIAL_UART_S("[" + String(millis()) + "] - - Returning data is :-  >>" + TCPBuffer.substring(i) + "\r\n");


// i is only effected at the start  - not by the lower if command - as its declared in the if statement so it runs from where it found the valid nmea line.
 // if (requiredNMEA)
 // {
    return TCPBuffer.substring(i);
//  }
//  else
//  {
 //   return "";
 // }
 }

void writeDataToWifi(WiFiClient LK8000Client, String data)
{
  char *csum_ptr;
  char UDPpacketBuffer[96];
//   char newLine='\r';
  int p=data.length();

  unsigned char cs = 0;
  data.toCharArray(UDPpacketBuffer,p+1);

  cs = 0; //clear any old checksum
  for (unsigned int n = 1; n < strlen(UDPpacketBuffer) - 1; n++)
  {
    cs ^= UDPpacketBuffer[n]; //calculates the checksum
  }

  csum_ptr = UDPpacketBuffer + strlen(UDPpacketBuffer);
  snprintf(csum_ptr, sizeof(UDPpacketBuffer) - strlen(UDPpacketBuffer), "%02X\r\n", cs);

  size_t len = strlen(UDPpacketBuffer);
  uint8_t  sbuf[len];
  
  memset(sbuf, 0, len);
  memcpy(&sbuf, &UDPpacketBuffer, len);
        
  LK8000Client.write((const uint8_t *)sbuf, len);
}


String add_to_ble_data_cksum( String ble_data_in, String movingpilotData_in )
{
  char *csum_ptr;
  char UDPpacketBuffer[96];
//   char newLine='\r';
  int p=movingpilotData_in.length();

  unsigned char cs = 0;
  movingpilotData_in.toCharArray(UDPpacketBuffer,p+1);

  cs = 0; //clear any old checksum
  for (unsigned int n = 1; n < strlen(UDPpacketBuffer) - 1; n++)
  {
    cs ^= UDPpacketBuffer[n]; //calculates the checksum
  }

  csum_ptr = UDPpacketBuffer + strlen(UDPpacketBuffer);
  snprintf(csum_ptr, sizeof(UDPpacketBuffer) - strlen(UDPpacketBuffer), "%02X\r\n", cs);

  size_t len = strlen(UDPpacketBuffer);
  uint8_t  sbuf[len];

  memset(sbuf, 0, len);
  memcpy(&sbuf, &UDPpacketBuffer, len);

  // Queue up the packet instead.
 // Serial1.write((const uint8_t *)sbuf, len);
 if (ble_data_in=="")
 {
	 ble_data_in=(String) UDPpacketBuffer;
 }
 else
 {
 // streamData=(String) UDPpacketBuffer+"\r\n"+streamData;
	 ble_data_in=ble_data_in + (String)UDPpacketBuffer;
 }

 return ble_data_in;
}




void writeDataToSerial(String data)
{
  char *csum_ptr;
  char UDPpacketBuffer[96];
//   char newLine='\r';
  int p=data.length();

  unsigned char cs = 0;
  data.toCharArray(UDPpacketBuffer,p+1);

  cs = 0; //clear any old checksum
  for (unsigned int n = 1; n < strlen(UDPpacketBuffer) - 1; n++)
  {
    cs ^= UDPpacketBuffer[n]; //calculates the checksum
  }

  csum_ptr = UDPpacketBuffer + strlen(UDPpacketBuffer);
  snprintf(csum_ptr, sizeof(UDPpacketBuffer) - strlen(UDPpacketBuffer), "%02X\r\n", cs);

  size_t len = strlen(UDPpacketBuffer);
  uint8_t  sbuf[len];
  
  memset(sbuf, 0, len);
  memcpy(&sbuf, &UDPpacketBuffer, len);

  // Queue up the packet instead.  
 // Serial1.write((const uint8_t *)sbuf, len);

  //5.95
  if ( serial_type_out=='a')
  {

	  if (streamData=="")
	  {
		  streamData=(String) UDPpacketBuffer;
	  }
	  else
	  {
		  // streamData=(String) UDPpacketBuffer+"\r\n"+streamData;
		  streamData=(String) UDPpacketBuffer+streamData;
	  }
  }
  else
  {
	  Serial.println(UDPpacketBuffer);
  }
 
}

void write_data_serial_no_cksum(String data)
{

 if (streamData=="")
 {
   streamData=data;
 }
 else
 {
 // streamData=(String) UDPpacketBuffer+"\r\n"+streamData;
	 // ERRRRRRRRRRRRRR EH ???? this cant work ? check it out ?!?!?!
   streamData=data+streamData;
 }

}


void addTestPilot(WiFiClient LK8000Client, String Pilot, int Dist, int current_pos)
{
  size_t wlen;
  current_pos=current_pos+50;  
  String cph=String(current_pos);
  String d=String(Dist);
  String movingpilot= "$PFLAA,7,"+d+","+d+",405,2,"+Pilot+"5,"+cph+",,33,0.2,1*";

  size_t lenmp = movingpilot.length();
  uint8_t  sbufmp[lenmp];
  movingpilot.getBytes (sbufmp, lenmp);
  LK8000Client.write((const uint8_t *)sbufmp, lenmp);
  LK8000Client.write("\r");
}     

bool sendUDPPacket ( String packet, IPAddress apFoundIP, bool add_cksum )
{
   char *csum_ptr;
   char UDPpacketBuffer[96];
   char newLine='\r';
   int p=packet.length();

   unsigned char cs = 0;
   packet.toCharArray(UDPpacketBuffer,p+1);
   
   WiFiUDP udp;

   if (add_cksum)
   {
     cs = 0; //clear any old checksum
     for (unsigned int n = 1; n < strlen(UDPpacketBuffer) - 1; n++) {
       cs ^= UDPpacketBuffer[n]; //calculates the checksum
     }

     csum_ptr = UDPpacketBuffer + strlen(UDPpacketBuffer);
     snprintf(csum_ptr, sizeof(UDPpacketBuffer) - strlen(UDPpacketBuffer), "%02X\n", cs);
   }

   if (debug_level>0)
   {

	   Serial.print("\nSending to ");
	   Serial.print(apFoundIP.toString());
	   Serial.print(" chars ");
	   Serial.println(UDPpacketBuffer);
   }

   udp.beginPacket(apFoundIP, XCSOAR_PORT);
   udp.write((uint8_t*)UDPpacketBuffer, strlen(UDPpacketBuffer));
   udp.endPacket();

   return true;
}


bool upLoadtoUDP( String urlToLoad, IPAddress IP, int udpPort)
{
  if (WiFi.status() != WL_CONNECTED)
  {
   return false;
  }
  char UDPpacketBuffer[128]={0};
 // memset(UDPpacketBuffer,0,sizeof(UDPpacketBuffer));

 // Serial1.println ( UDPpacketBuffer);
 // Serial1.println ( urlToLoad.length());
 
  urlToLoad.toCharArray(UDPpacketBuffer,urlToLoad.length()+1);

  WiFiUDP udp;

 
  DEBUG_SERIAL_UART_MAX("[%ld] - Crash Marker 3a\r\n", millis());
//  Serial1.println ( urlToLoad);
//  Serial1.println ( urlToLoad.length());
 
  yield();

  while ( lastUdpPacketTime > millis() )
  {
    DEBUG_SERIAL_UART_MAX("[%ld] - Waiting for Previous UDP Packet to send ( 100 Millis at the moment ) \r\n", millis());
    //Serial1.println(" Waiting for Previous UDP Packet to send ( 100 Millis at the moment ) \r\n");
    
    myDelay(10);
  }
// ************************ NEEDS CHANGING AND LOWERING - HERE FOR TESTING ONLY *********************************
  lastUdpPacketTime=millis()+20;

  DEBUG_SERIAL_UART_MAX("[%ld] - Crash Marker 3b.111\r\n", millis());

    udp.beginPacket(IP, udpPort);
  udp.write((uint8_t*)UDPpacketBuffer, strlen(UDPpacketBuffer));

//  Serial1.println ( urlToLoad);
 // Serial1.println ( urlToLoad.length());
  
  DEBUG_SERIAL_UART_MAX("[%ld] - Crash Marker 3c.111\r\n", millis());

  // need to stop udp packets being sent too quickly, system seems to crash if its done repeatively.
  // 


  
  
 // Serial1.println(millis());
  yield();
//  delay(100);
  udp.endPacket();

  DEBUG_SERIAL_UART_MAX("[%ld] - Crash Marker 3d.2222\r\n", millis());

  return true;
}


String readAllDataOnSerial( HardwareSerial data_in,  String TCPBuffer, bool &fullLine )
{

//v1.1

  unsigned int wlen=data_in.available();


  //Serial.println("Serial1 avail");
 // Serial.println(GPSSerial.available());
//
  // For some reason if the incoming tcp buffer is too big it crashes the system, limit it to 512, we might miss a few lines now and then
  // but better than crashing the system, seems windows CE doesnt transmit data very well and stalls, kobo is fine :)
  
  if ( wlen > TCP_BUFFER_MAX_LENGTH )
  {//v1.1
	data_in.flush();
    DEBUG_SERIAL_UART_MAX("[%ld] - %%%%%%%% TCP data bytes available :- %d - Flushing readAllDataOnSerial Buffer\r\n", millis(),wlen);
    wlen=data_in.available();
    DEBUG_SERIAL_UART_MAX("[%ld] - %%%%%%%% TCP data bytes available :- %d - AFTER Flushing readAllDataOnSerial Buffer\r\n", millis(),wlen);
    return "";
  }


   wlen=data_in.available();

  if (wlen!=0)
  {


    // read plus 2 as have to declare the 1 byte array which starts at 0 plus the EOL char.
    char gps_data_in[wlen+2]="";
    data_in.readBytes(gps_data_in, wlen);
    gps_data_in[wlen+2] = '\0';
    TCPBuffer=TCPBuffer+String(gps_data_in).substring(0,wlen);

    String oS = String(gps_data_in).substring(0,wlen);


  //  Serial.print("wlen ");
 //  Serial.println(wlen);
  // Serial.print(">>");
 //  Serial.print(oS);
  // Serial.println(">>");

   String oSout=oS;

   int searchS=wlen;

   last_data_received=millis();

   if ( (millis()-pflauSendTime) > pflauCounter )
   {
    addpflau=true;
    pflauCounter=millis();
   }
 
  // Serial.println("incoming string" + oS );

   
DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnSerial Point 4 \r\n", millis());
   
   if( addpflau || streamData!="")
   {
//    Serial.println ( "Adding FL");
 //   Serial.println("in  string > " + oS + " < " );
    
    for (int i=0;i<searchS;i++)
    {


   //   Serial.print(">>" );
    //  Serial.print(oS.charAt(i));
    //  Serial.println("<<" );
      
      if (oS.charAt(i)=='$')
      {
      //   Serial.println("Start" + oS.substring(0,i));
      //   Serial.println("End" + oS.substring(i));
         
         if( addpflau)
         {
         // pflaaPacket="\r\n\r\n" + pflaaPacket+"\r\n\r\n$PFLAU,6,1,2,1,0,144,0,235,446*55\r\n\r\n";
            streamData="$PFLAU,6,1,2,1,0,144,0,235,446*55\r\n"+streamData;
         }
         
         oSout=oS.substring(0,i)+streamData+oS.substring(i);
        // Serial.println("outgoing string out of Addition" + oSout);
      //   Serial.println("out  string > " + oSout + " < " );
         
      //   Serial1.flush();
       //  Serial1.print("\r\n"+oS.substring(0,i)+pflaaPacket+oS.substring(i));
         
         addpflau=false;
         streamData="";
         break;
         i=searchS;
      }
     }  
   }
   

  // Serial.flush();
 // Serial.print(oSout);


  /*if (ble_output == 'e' && deviceConnected)
  {
	//  BLESendChunks(oSout);

	  // NEEDS TO ADD A SYSTEM TO SEND FROM THE $ if congested.

	  if (ble_data.length()<512)
	  {
		  Serial.print("BLE-IN>>>>>");
		  	    Serial.print(ble_data);
		  	    Serial.println("<<<<<BLE-IN");


		  	  Serial.print("oSo-IN>>>>>");
		  	  		  	    Serial.print(oSout);
		  	  		  	    Serial.println("<<<<<oSo-IN");
        while(ble_mutex){};
	    ble_data=ble_data+oSout;

//	    Serial.print("BLE-OUT>>>>>");
	//    Serial.print(ble_data);
	//    Serial.println("<<<<<BLE");
	  }
	  else
	  {
		  Serial.println("too much ble...........waiting..................");
		  if (!ble_mutex)
		  {
		    ble_data="";
		   }
	  }
  }

*/

 //  Serial.println("ehere");
 if ( serial_type_out=='a')
 {
	// Serial.println("outputtin serial");
	 Serial.print(oSout);
 }


  if (ble_output == 'u' && SerialBT.hasClient())
  {
	  SerialBT.print(oSout);
  }

    

  //  Serial.println("to kobo start");
 //  Serial.println(oSout);    
  //  Serial.println("to kobo end");

    
   //   Serial1.print(String(cbuf).substring(0,wlen));

  }
  else
  {
     DEBUG_SERIAL_UART_MAX("[%ld] - No Data on socket at this point \r\n", millis());
  }

   DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnSerial Point 5b \r\n", millis());
  bool requiredNMEA=false;
  int i;
   DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnSerial Point 5c \r\n", millis());
  // Serial1.print("before with ");
   // Serial1.println(TCPBuffer.length());

//v1.1
   // moving this here as it can come out early.
      fullLine=false;

//following if statement is because sometimes the next crashes the system when tcpbuffer=0, god knows why.....
  if ( TCPBuffer.length()==0)
  {
    return "";
  }
    
  if ( TCPBuffer.length()<5)
  {
    //Serial1.print("returning with ");
  //  Serial1.println(TCPBuffer.length());
    return TCPBuffer;
  }

DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnSerial Point 6 \r\n", millis());

  // make 100% sure length>0

  int len=TCPBuffer.length()-4;

  if (len<0)
  {
    len=0;
  }
  
  //v1.1
  bool found_valid_nmea=false;

  for (i=0;i<len;i++)
  {
	  if (is_valid_command(TCPBuffer, i) )
	 // if (TCPBuffer.charAt(i)=='$' && ( TCPBuffer.charAt(i+3)== 'G' || TCPBuffer.charAt(i+3)== 'R' ) && ( TCPBuffer.charAt(i+4)== 'G' || TCPBuffer.charAt(i+4)== 'M' ) )
	  {
		  /// requiredNMEA=true;
		//  Serial.println("");
		//  Serial.print("found valid command :- TCPBuffer **************** - char ");
		//  Serial.println(i);
		//  Serial.print("returning :- >>>");
		//  Serial.print(TCPBuffer.substring(i));
		//  Serial.println("<<<<");

		  found_valid_nmea=true;
		  break;
	  }
  //v1.1  if (TCPBuffer.charAt(i)=='$' && ( TCPBuffer.charAt(i+3)== 'G' || TCPBuffer.charAt(i+3)== 'R' ) && ( TCPBuffer.charAt(i+4)== 'G' || TCPBuffer.charAt(i+4)== 'M' ) )
   // {
    	//v1.1
   // 	found_valid_nmea=true;
     /// requiredNMEA=true;
     // break;
   // }
  }  

DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnSerial Point 7 \r\n", millis());

//v1.1
// if it doesnt find a valid line it steps over the 4 digits we had to save for partial lines
// coming in next time, ie $GGP - therefore we need to get from i for valid line and if its a partial
// then i-1;

    if (found_valid_nmea && TCPBuffer.length()>0)
    {
    	TCPBuffer= TCPBuffer.substring(i);
    }
    else
    {
    	TCPBuffer= TCPBuffer.substring(i-1);
    }

//v1.1  TCPBuffer= TCPBuffer.substring(i);

  fullLine=false;
  
  for (int i=0;i<TCPBuffer.length();i++)
  { 
//v1.1
    if (TCPBuffer.charAt(i)=='\n' && i >6)
//v1.1    if (TCPBuffer.charAt(i)=='\n')
    {
     // Serial.println("Found end of Line **********");
      fullLine=true;
      break;
    }
   
  }  
//  Serial.println("REturning :>");
 // Serial.println(TCPBuffer );
 // Serial.println("<<<<<<<");   
  return TCPBuffer;
 }


bool upLoadtoUDP( bool device_connected,  String data, IPAddress IP, int udpPort)
{

	if (!device_connected)
	{
		if (debug_level>0) Serial.println("no device connected");
		return false;
	}

// remove this.....v3
  //if (WiFi.status() != WL_CONNECTED)
  //{
   //return false;
  //}
  char UDPpacketBuffer[1024]={0};
 // memset(UDPpacketBuffer,0,sizeof(UDPpacketBuffer));

 // Serial1.println ( UDPpacketBuffer);
 // Serial1.println ( urlToLoad.length());
  int length_of_data=data.length()+1;

  data.toCharArray(UDPpacketBuffer,data.length());

  WiFiUDP udp;


  DEBUG_SERIAL_UART_MAX("[%ld] - Crash Marker 3a\r\n", millis());
//  Serial1.println ( urlToLoad);
//  Serial1.println ( urlToLoad.length());

  yield();

  while ( lastUdpPacketTime > millis() )
  {
    DEBUG_SERIAL_UART_MAX("[%ld] - Waiting for Previous UDP Packet to send ( 100 Millis at the moment ) \r\n", millis());
    //Serial1.println(" Waiting for Previous UDP Packet to send ( 100 Millis at the moment ) \r\n");
 Serial.println("waiting for packet to send");
    myDelay(10);
    yield();
  }
// ************************ NEEDS CHANGING AND LOWERING - HERE FOR TESTING ONLY *********************************
  lastUdpPacketTime=millis()+250;

  DEBUG_SERIAL_UART_MAX("[%ld] - Crash Marker 3b.111\r\n", millis());

  if (debug_level>0)
  {
    Serial.println("********************************");

	  Serial.println("Doing the udp.write");
    Serial.println(IP);
    Serial.println(udpPort);
    Serial.println("UDPpacketBuffer");
    Serial.println(UDPpacketBuffer);

    Serial.println("strlen(UDPpacketBuffer)");
    Serial.println(strlen(UDPpacketBuffer));

    

    
    Serial.println("********************************");

  }
    udp.beginPacket(IP, udpPort);
  udp.write((uint8_t*)UDPpacketBuffer, length_of_data);
  udp.endPacket();
  yield();


//  Serial1.println ( urlToLoad);
 // Serial1.println ( urlToLoad.length());

  DEBUG_SERIAL_UART_MAX("[%ld] - Crash Marker 3c.111\r\n", millis());

  // need to stop udp packets being sent too quickly, system seems to crash if its done repeatively.
  //




 // Serial1.println(millis());
  yield();
//  delay(100);
  udp.endPacket();

  DEBUG_SERIAL_UART_MAX("[%ld] - Crash Marker 3d.2222\r\n", millis());

  return true;
}


String readAllDataOnSerialTXWifiU( int number_client, bool device_connected, IPAddress connected_ip, HardwareSerial data_in,  String TCPBuffer, bool &fullLine )
{

//v1.1
  size_t wlen=data_in.available();


  //Serial.println("Serial1 avail");
 // Serial.println(GPSSerial.available());
//
  // For some reason if the incoming tcp buffer is too big it crashes the system, limit it to 512, we might miss a few lines now and then
  // but better than crashing the system, seems windows CE doesnt transmit data very well and stalls, kobo is fine :)

  if ( wlen > TCP_BUFFER_MAX_LENGTH )
  {//v1.1
	data_in.flush();
    DEBUG_SERIAL_UART_MAX("[%ld] - %%%%%%%% TCP data bytes available :- %d - Flushing readAllDataOnSerial Buffer\r\n", millis(),wlen);
    wlen=data_in.available();
    DEBUG_SERIAL_UART_MAX("[%ld] - %%%%%%%% TCP data bytes available :- %d - AFTER Flushing readAllDataOnSerial Buffer\r\n", millis(),wlen);
    return "";
  }

  if (wlen!=0)
  {

    DEBUG_SERIAL_UART_MAX("[%ld] - Serial data bytes available :- %d \r\n", millis(),wlen);
    last_data_received=millis();


    uint8_t sbuf[wlen+1]="";
    char cbuf[wlen+1]="";

   // GPSSerial.flush();
    //v1.1
    data_in.readBytes(cbuf, wlen);



    DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnSerial Point 2 \r\n", millis());

 //   memset(cbuf, 0, wlen);

    DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnSerial Point 2a \r\n", millis());

  //  memcpy(&cbuf,&sbuf, wlen);

    DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnSerial Point 3 \r\n", millis());

   // TCPBuffer=TCPBuffer+String(cbuf).substring(0,wlen);// + "\r\n$PFLAU,6,1,2,1,0,144,0,235,446*\r\n";
   //TCPBuffer="START\r\n" + String(cbuf).substring(0,wlen) + "\r\nEND\r\n";

    cbuf[wlen+1] = '\0';

    TCPBuffer=TCPBuffer+String(cbuf).substring(0,wlen);
  //  delay(200);
 //   Serial1.flush();
 //   yieldDelay(100);

   //Serial1.print(String(cbuf).substring(0,wlen));


   String oS = String(cbuf).substring(0,wlen);
   String oSout=oS;

   int searchS=wlen;


   if ( (millis()-pflauSendTime) > pflauCounter )
   {
    addpflau=true;
    pflauCounter=millis();
   }


  // Serial.println("incoming string" + oS );


DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnSerial Point 4 \r\n", millis());

   if( addpflau || streamData!="")
   {
//    Serial.println ( "Adding FL");
 //   Serial.println("in  string > " + oS + " < " );

    for (int i=0;i<searchS;i++)
    {


   //   Serial.print(">>" );
    //  Serial.print(oS.charAt(i));
    //  Serial.println("<<" );

      if (oS.charAt(i)=='$')
      {
      //   Serial.println("Start" + oS.substring(0,i));
      //   Serial.println("End" + oS.substring(i));

         if( addpflau)
         {
         // pflaaPacket="\r\n\r\n" + pflaaPacket+"\r\n\r\n$PFLAU,6,1,2,1,0,144,0,235,446*55\r\n\r\n";
            streamData="$PFLAU,6,1,2,1,0,144,0,235,446*55\r\n"+streamData;
         }

         oSout=oS.substring(0,i)+streamData+oS.substring(i);
        // Serial.println("outgoing string out of Addition" + oSout);
      //   Serial.println("out  string > " + oSout + " < " );

      //   Serial1.flush();
       //  Serial1.print("\r\n"+oS.substring(0,i)+pflaaPacket+oS.substring(i));

         addpflau=false;
         streamData="";
         break;
         i=searchS;
      }
     }
   }


   //Serial.flush();
   //Serial.print(oSout);


   if (number_client>0)
   {

     upLoadtoUDP(device_connected, oSout, connected_ip, XCSOAR_PORT);
     if (debug_level>0)
     {
      Serial.print("uploading with udp, ip :-");
       Serial.print(connected_ip.toString());
      Serial.print("Port :-" );
      Serial.println(XCSOAR_PORT);
	   Serial.println(oSout);
     }
   }


  //  Serial.println("to kobo start");
 //  Serial.println(oSout);
  //  Serial.println("to kobo end");


   //   Serial1.print(String(cbuf).substring(0,wlen));

  }
  else
  {
     DEBUG_SERIAL_UART_MAX("[%ld] - No Data on socket at this point \r\n", millis());
  }

   DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnSerial Point 5b \r\n", millis());
  bool requiredNMEA=false;
  int i;
   DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnSerial Point 5c \r\n", millis());
  // Serial1.print("before with ");
   // Serial1.println(TCPBuffer.length());

//v1.1
   // moving this here as it can come out early.
      fullLine=false;

//following if statement is because sometimes the next crashes the system when tcpbuffer=0, god knows why.....
  if ( TCPBuffer.length()==0)
  {
    return "";
  }

  if ( TCPBuffer.length()<5)
  {
    //Serial1.print("returning with ");
  //  Serial1.println(TCPBuffer.length());
    return TCPBuffer;
  }

DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnSerial Point 6 \r\n", millis());

  // make 100% sure length>0

  int len=TCPBuffer.length()-4;

  if (len<0)
  {
    len=0;
  }

  //v1.1
  bool found_valid_nmea=false;

  for (i=0;i<len;i++)
  {
	  if (is_valid_command(TCPBuffer, i) )
	 // if (TCPBuffer.charAt(i)=='$' && ( TCPBuffer.charAt(i+3)== 'G' || TCPBuffer.charAt(i+3)== 'R' ) && ( TCPBuffer.charAt(i+4)== 'G' || TCPBuffer.charAt(i+4)== 'M' ) )
	  {
		  /// requiredNMEA=true;
		//  Serial.println("");
		//  Serial.print("found valid command :- TCPBuffer **************** - char ");
		//  Serial.println(i);
		//  Serial.print("returning :- >>>");
		//  Serial.print(TCPBuffer.substring(i));
		//  Serial.println("<<<<");

		  found_valid_nmea=true;
		  break;
	  }
  //v1.1  if (TCPBuffer.charAt(i)=='$' && ( TCPBuffer.charAt(i+3)== 'G' || TCPBuffer.charAt(i+3)== 'R' ) && ( TCPBuffer.charAt(i+4)== 'G' || TCPBuffer.charAt(i+4)== 'M' ) )
   // {
    	//v1.1
   // 	found_valid_nmea=true;
     /// requiredNMEA=true;
     // break;
   // }
  }

DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnSerial Point 7 \r\n", millis());

//v1.1
// if it doesnt find a valid line it steps over the 4 digits we had to save for partial lines
// coming in next time, ie $GGP - therefore we need to get from i for valid line and if its a partial
// then i-1;

    if (found_valid_nmea && TCPBuffer.length()>0)
    {
    	TCPBuffer= TCPBuffer.substring(i);
    }
    else
    {
    	TCPBuffer= TCPBuffer.substring(i-1);
    }

//v1.1  TCPBuffer= TCPBuffer.substring(i);

  fullLine=false;

  for (int i=0;i<TCPBuffer.length();i++)
  {
//v1.1
    if (TCPBuffer.charAt(i)=='\n' && i >6)
//v1.1    if (TCPBuffer.charAt(i)=='\n')
    {
     // Serial.println("Found end of Line **********");
      fullLine=true;
      break;
    }

  }
//  Serial.println("REturning :>");
 // Serial.println(TCPBuffer );
 // Serial.println("<<<<<<<");
  return TCPBuffer;
 }


String readAllDataOnWifiTxSerial( WiFiClient LK8000Client, String TCPBuffer, bool &fullLine )
{


  size_t wlen=Serial.available();

  // For some reason if the incoming tcp buffer is too big it crashes the system, limit it to 512, we might miss a few lines now and then
  // but better than crashing the system, seems windows CE doesnt transmit data very well and stalls, kobo is fine :)
  
  if ( wlen > TCP_BUFFER_MAX_LENGTH )
  {
    Serial.flush();
    DEBUG_SERIAL_UART_MAX("[%ld] - %%%%%%%% TCP data bytes available :- %d - Flushing readAllDataOnSerial Buffer\r\n", millis(),wlen);
    wlen=Serial.available();
    DEBUG_SERIAL_UART_MAX("[%ld] - %%%%%%%% TCP data bytes available :- %d - AFTER Flushing readAllDataOnSerial Buffer\r\n", millis(),wlen);
    return "";
  }

  if (wlen!=0)
  {
  
    DEBUG_SERIAL_UART_MAX("[%ld] - Serial data bytes available :- %d \r\n", millis(),wlen);
     
    last_data_received=millis();
    uint8_t sbuf[wlen];
    char cbuf[wlen];
    

    Serial.readBytes(sbuf, wlen);        
     
    DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnSerial Point 2 \r\n", millis());
     
    memset(cbuf, 0, wlen);

    DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnSerial Point 2a \r\n", millis());
     
    memcpy(&cbuf,&sbuf, wlen);
   
    DEBUG_SERIAL_UART_MAX("[%ld] - In  readAllDataOnSerial Point 3 \r\n", millis());

   // TCPBuffer=TCPBuffer+String(cbuf).substring(0,wlen);// + "\r\n$PFLAU,6,1,2,1,0,144,0,235,446*\r\n";
   //TCPBuffer="START\r\n" + String(cbuf).substring(0,wlen) + "\r\nEND\r\n";
   
    TCPBuffer=TCPBuffer+String(cbuf).substring(0,wlen);
  //  delay(200);
 //   Serial1.flush();
 //   yieldDelay(100);

   //Serial1.print(String(cbuf).substring(0,wlen));

   
   String oS = String(cbuf).substring(0,wlen);
   String oSout=oS;
   
   int searchS=wlen;

   
   if ( (millis()-pflauSendTime) > pflauCounter )
   {
    addpflau=true;
    pflauCounter=millis();
   }
 

  // Serial.println("incoming string" + oS );

   

   
   if( addpflau || streamData!="")
   {
//    Serial.println ( "Adding FL");
 //   Serial.println("in  string > " + oS + " < " );
    
    for (int i=0;i<searchS;i++)
    {


   //   Serial.print(">>" );
    //  Serial.print(oS.charAt(i));
    //  Serial.println("<<" );
      
      if (oS.charAt(i)=='$')
      {
      //   Serial.println("Start" + oS.substring(0,i));
      //   Serial.println("End" + oS.substring(i));
         
         if( addpflau)
         {
         // pflaaPacket="\r\n\r\n" + pflaaPacket+"\r\n\r\n$PFLAU,6,1,2,1,0,144,0,235,446*55\r\n\r\n";
            streamData=streamData+"$PFLAU,6,1,2,1,0,144,0,235,446*55\r\n";
         }
         
         oSout=oS.substring(0,i)+streamData+oS.substring(i);
        // Serial.println("outgoing string out of Addition" + oSout);
      //   Serial.println("out  string > " + oSout + " < " );
         
      //   Serial1.flush();
       //  Serial1.print("\r\n"+oS.substring(0,i)+pflaaPacket+oS.substring(i));
         
         addpflau=false;
         streamData="";
         break;
         i=searchS;
      }
     }  
   }
    
  //  Serial1.flush();
  //  Serial1.print(oSout);
   //   Serial1.print(String(cbuf).substring(0,wlen));
   LK8000Client.flush();
   LK8000Client.print(oSout);
 //  Serial.print(oSout);

  }
  else
  {
     DEBUG_SERIAL_UART_MAX("[%ld] - No Data on socket at this point \r\n", millis());
  }

   
  bool requiredNMEA=false;
  int i;

   
  if ( TCPBuffer.length()<5)
  {
    return TCPBuffer;
  }

  // make 100% sure length>0

  int len=TCPBuffer.length()-4;

  if (len<0)
  {
    len=0;
  }
  
  for (i=0;i<len;i++)
  {
  //v1.1
	  if (is_valid_command(TCPBuffer, i) )
	  {
		  /// requiredNMEA=true;
		  break;
	  }
   // if (TCPBuffer.charAt(i)=='$' && ( TCPBuffer.charAt(i+3)== 'G' || TCPBuffer.charAt(i+3)== 'R' ) && ( TCPBuffer.charAt(i+4)== 'G' || TCPBuffer.charAt(i+4)== 'M' ) )
  //  {
     /// requiredNMEA=true;
   //   break;
  //  }
  }  

  TCPBuffer= TCPBuffer.substring(i);

  fullLine=false;
  
  for (int i=0;i<TCPBuffer.length();i++)
  { 


    if (TCPBuffer.charAt(i)=='\n')
    {
   //   Serial.println("Found end of Line");
      fullLine=true;
      break;
    }
   
  }  
 // Serial.println("returning " + TCPBuffer );
  return TCPBuffer;
 }

 /*bool nmea_cksum(String data )
 {
  char *csum_ptr;
  char UDPpacketBuffer[96];
//   char newLine='\r';
  int p=data.length();

  unsigned char cs = 0;
  data.toCharArray(UDPpacketBuffer,p+1);

  cs = 0; //clear any old checksum
  for (unsigned int n = 1; n < strlen(UDPpacketBuffer) - 1; n++)
  {
    cs ^= UDPpacketBuffer[n]; //calculates the checksum
  }

  csum_ptr = UDPpacketBuffer + strlen(UDPpacketBuffer);
  snprintf(csum_ptr, sizeof(UDPpacketBuffer) - strlen(UDPpacketBuffer), "%02X\r\n", cs);

  Serial1.println(UDPpacketBuffer);
  Serial1.println(csum_ptr);

  return true;
 }*/


bool cksum(String data)
{


   // if theres a carriage return on the end of the line then remove.

//	Serial.print(">");Serial.print(data);Serial.println("<");
//	Serial.print(">");Serial.print(data.substring(data.length()));Serial.println("<");
//	Serial.print(">");Serial.print(data.substring(data.length(),data.length()));Serial.println("<");
//	Serial.print(">");Serial.print(data.substring(data.length()-1,data.length()));Serial.println("<");
//	Serial.print(">");Serial.print(data.substring(data.length()-2,data.length()));Serial.println("<");

   if (data.substring(data.length()-1,data.length())=="\n" || data.substring(data.length()-1,data.length())=="\r" || data.substring(data.length()-1,data.length())==" ")
   {
	// Serial.println("found a return ! ");
	 data=data.substring(0,data.length()-1);
	// Serial.print(">");Serial.print(data);Serial.println("<");
   }


   char UDPpacketBuffer[150];

   int p=data.length();

   unsigned char cs = 0;
   data.toCharArray(UDPpacketBuffer,p+1);


   //start at 1 to jump the $ and  remove the 3 digits at the end for the cksum.

   cs = 0; //clear any old checksum

   unsigned int c=0;

   char cksum_value[3];

   for (unsigned int n = 1; n < strlen(UDPpacketBuffer); n++)
   {
	   if (n < strlen(UDPpacketBuffer) - 3)
	   {
		   //Serial.print("Before:");
	//   Serial.println(UDPpacketBuffer[n]);
         cs ^= UDPpacketBuffer[n]; //calculates the checksum
	   }
	   else
	   {
		 if (n != (strlen(UDPpacketBuffer) -3 )) // on the *
	     {
			 // check its a digit - if not line is false;
		   cksum_value[c]=UDPpacketBuffer[n];

		   if (!isalnum (cksum_value[c]))
		   {
			  // Serial.println("fail digit");
			   return false;
		   }
		 //  Serial.print("After:");
		 //  Serial.println(UDPpacketBuffer[n]);
		   c++;
	     }
	   }

   }

  // Serial1.println("cs");Serial1.println(cs);

   cksum_value[2]='\0';
   //unsigned char number = (char)strtol(cksum_value, NULL, 16);

   if (cs==(char)strtol(cksum_value, NULL, 16))
   {
	   return true;
   }
   else
   {
	  // Serial1.println("fail cksum");
	   return false;
   }

}
