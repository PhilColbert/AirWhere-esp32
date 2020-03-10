/* AirWhere ESP32 v1
 * v4.3
 *
 * adding BLE and bluetooth, banging
 *
 v4.1
  - adding serial commands for LK and XCS.
 *
 v4
  - add TX LED for TTGO and FLARM board.
 *
 *
v1.15
- changing id's

v1.14
- adding all the ogn .
v1.13
- adding standalone functionality - just connect like serial and fire data out via
  udp regardless of holding a port -no tcp port failures etc, should be more reliable in keeping alive,
  maybe not in data, but i think will be ok.
v1.12
- add sounds out for bluefly.
v1.11
- standalong switched to udp out and running like serial.
v1.1
- add /r for some android EOL
- add full coord gps to be compatible with skytraxx
- flarm board
- flarm update
- heltec board
- rfm95 board
- otg sorted


 */

#include "AirWare.h"
#ifdef ESP32_CPU
#else
//#include <nRF905.h>
//#include <WiFiClient.h>
//
//

//#include <DNSServer.h>
//extern "C" {
//#include <user_interface.h>
//#include "Esp.h"
//}
//#include "flarm_codec.h"
//#include "fileOps.h"
#endif
#include "driver/gpio.h"
#include "fileOps.h"
#include "WebHelper.h"
#include "WiFiHelper.h"
#include "esp_wifi.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "tcpip_adapter.h"
#include "esp_wifi_types.h"
#include "WiFi.h"
#include <esp_wifi.h>
#include <Arduino.h>
#include "WifiTools.h"

#include "timeFunctions.h"
#include "fanet_stack/payload.h"
#include "fanet.h"
//#include "airPKT.h"
#include "CalcTools.h"
#include "RFdata.h"


//v1.1 - for heltec oled display. ( pins are only used when init or connect is called so ok to create here. )
#include "oled/SSD1306.h"
SSD1306  display(0x3c, 4, 15);
//SSD1306  /(0x3c, 21, 22);
//SSD1306  display;

#include "vario/airwhere_esp32_vario.h"
#define ARDUINO_RUNNING_CORE0 0
#define ARDUINO_RUNNING_CORE1 1

#ifdef ESP32_CPU
#include "sounds.h"
//VARIO #include "MS5611.h"

#define LOG_LOCAL_LEVEL ESP_LOG_NONE
#define LOG_TAG "BLE-"

#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "main";
HardwareSerial GPSSerial(2);

#define INCLUDE_BLE

#ifdef INCLUDE_BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BluetoothSerial.h>

#endif

//5.95

char serial_type_out='a';

#else
#endif

// ************************************************************************************ REmember to change back!
#ifdef LIVE_VERSION
int AwVersion = 6;
#else
int AwVersion = 0;
#endif

#include "app.h"
App app = App();

//v1.1
#include "flarm.h"
Flarm flarm = Flarm();
#define FLARM_BOARD_BAUD 115200
#define FLARM_BOARD_SERIAL_TIMEOUT 0

//v1.1 - to be done later.
#include "serial_in.h"
Serial_In kobo_serial = Serial_In();
Serial_In gps_serial = Serial_In();

//airWarePkt airPKT;
//airWarePkt airPKTRX;

IPAddress airwhere_web_ip(37, 128, 187, 9);
#ifdef LOGGING_ON
IPAddress loggingIP(37, 128, 187, 9);
String loggingS = "";
bool logIt = false;
#endif


bool dhcp_client_connected = false;
bool socket_connected = false;
int current_pos = 1;
#ifdef ESP32_CPU
bool webUpload = true;
char airWhereSsid[32]="";
char airWherePassword[32]="";
char airWhere_ap_password[32]="12345678";
char loraFrequency='8';
char loraFrequencyC[10];

char web_aircraft_type='1';

WiFiClient airwhere_client;
uint16_t airwhere_port=4353;
IPAddress apFoundIP;

#else
DNSServer dnsServer;
WiFiClient airWareClient;
WiFiClient LK8000Client;
IPAddress LK8000iPAddress(192, 168, 4, 2);
const int lk8000TcpServerPort = 4353;
bool ipFound = false;
bool webUpload = false;

#endif

String GPGGAdata = "";
bool GPGGAdataReceived = false;

String GPRMCdata = "";
bool GPRMCdataReceived = false;
bool displayedclientconnected = false;

byte TxBuffer[PKT_SIZE];
byte RxBuffer[PKT_SIZE];

long tcp_rx_weblines_counter = 0;
long serial_rx_packets_counter = 0;
long nrf_rx_packets_counter = 0;
long tcp_tx_weblines_counter = 0;
long serial_tx_packets_counter = 0;
//v1.14
long ogn_tx_packets=0;
int longest_loop_time = 0;
int flush_time_counter = 0;
long web_upload_counter = 0;
long bad_rx_packets = 0;

GPGGAdataLine GPGGAdataIn ("");
GPRMCdataLine GPRMCdataIn("");

bool startLoop = true;
unsigned long t = 0;
int climbRate = 0;

String TCPBuffer = "";

//ufo_t fo;

char navSofware='X';
// L - LK
// X - XCSOAR
char receiveFlarm; // dont receive;

int loopTime = 0;
unsigned int gsTimer=GSTIMERINTERVAL;
//uint32 free_mem;f
unsigned int looking_for_socket_time;
unsigned int last_client_connected_time;
bool set_Connected_Time = false;
unsigned int last_data_received;
int last_serial_send;
unsigned int webTime;

int myChipID;

char groundStationMode = 'n';
char gsLatitude[11];
char gsLongitude[12];
char gsAltitude[6];
bool populatedGSdata=false;

int PFLout=0;

char wiredDirectly = 'o';
char wiredChar[6];

//char wiredDirectly;
//SoftwareSerial swSer(14, 12, false, 1024);

int lastUdpPacketTime=0;

long gpsBaudRate=9600;
long serialBaudRate=115200;
int airWhereID=0;

//v1.15
char awHexID[5]="0000";
char awHexManu[3]="00";

char wifioffSelected='n';
char awPilotName[30];

String cnl;
//v2
//uint8_t loraData[RF_PACKETS][RF_PACKET_SIZE] = {0};
//v2
int payloadReady[RX_PACKETS] = {0};
long int payload_time_to_send[RX_PACKETS] = {0};
Payload payloadList[RX_PACKETS];

char packet_repeat='n';
long int heap_out=0;

//bool payloadReady[RX_PACKETS] = {false};
//Payload payloadList[RX_PACKETS];

unsigned long pflauSendTime=5000;
unsigned long pflauCounter=0;
unsigned long pflauSendTimeble=5000;
unsigned long pflauCounterble=0;


bool addpflau;
bool rfPktToSend=false;
unsigned int spacerTime=0;

String streamData="";

//int16_t _RSSIpacket;
int8_t _SNR;

unsigned int wifiDownTime=0;
bool wifiUp=true;

int16_t _RSSIpackets[RF_PACKETS]= {0};

#ifdef TEST_PILOT
int addPilotTime=10000;
int pilotInterval=1000;
int pilotA=12345;
double pilotARadsDiff=.05;
double pilotADistance=50;
double pilotACurrentTrack=0;
double pilotAX=0;
double pilotAY=0;
double pilotAlt=0;
double pilotAltDiff=5;
#endif

bool receiveIRQ=false;
int loopno=0;

//sounds section
char soundsOn='y';
//int soundsOff=0;
bool airwhere_tones_on=false;
String soundBuffer;
int tuneIs[noBeeps][beepData]={0};
int noTones=0;
int currentTone=0;

//baro section
 char ms5611_attached = 'n';
 char vario_on ='n';
 bool ms5611_online=false;
 // default to buzzer on ( might be overridden from file.)
 char buzzer_on_off='n';
 int turn_on_off_buzzer=2;
 char vario_sounds_on='y';

 //v5

 bool run_vario=true;
 //VARIO MS5611 ms5611;
 int current_pressure =0;
 int current_altitude =0;
 int current_vario =0;
 int current_temperature = 0;
 int current_battery =0;

 unsigned long pressure_temp_output_time=0;
 unsigned long temp_read_time=0;
 #define BARO_OUTPUT_PERIOD 100
 #define TEMP_OUTPUT_PERIOD 10000
 #define PRESSURE_AT_18000_FT 50000
 #define PRESSURE_AT_0 110000
#define TEMP_OFFSET 4

 int climb_threshold=10;
 // set buzzer to off
 int zero_threshold=100;
 int sink_threshold=-200;
 char calibrate_vario = 'y';

//ble section
 char ble_output='n';

#ifdef INCLUDE_BLE
 #include "airwhere_ble.h"
 
 BluetoothSerial SerialBT;
 bool deviceConnected = false;
 char LK8EX1_Sentence[80];
 String ble_data="";
 bool ble_mutex=false;

#endif

 //v1.1 - had to move uart1 to gps as current code base fails with 115200 speed on uart2
 // hopefully fixed in next update.

 // Flarm board
 HardwareSerial flarmSerial(1);
 #define FLARM_RX_PIN 16
 #define FLARM_TX_PIN 17
 bool flarm_update_in_process=false;
 bool main_loop_halted=false;
 int flarm_reset_time;
 #define FLARM_RESET_SPACE 30000;

 char lora_or_flarm='l'; //default to lora
 bool flarm_board_ok=false;
 char manufacturer_code[3]={'\0'};
 char flarm_board_id[5]={'\0'};
 char flarm_board_expiration[14]={'\0'};
 char flarm_board_build[13]={'\0'};
 bool flarm_tracking_on=false;
 bool fanet_tracking_on=false;
 bool flarm_needs_reset=false;

 IPAddress socket_ip;

 //v1.14
 char ogn_id[7]={};
 char ogn_on='n';

//v2

 WiFiUDP udp_flight_software;
 bool udp_flight_software_running=false;
 long int loopy=0;

 //v2.11

 int debug_level=0;
 //v1.1 updating oled screen on heltec


 //v3

 long int waiting_for_ip=0;
 bool ip_assigned=false;

//v4

 long int tx_led_off_time=0;
 bool tx_led_on=false;

 //v5
 unsigned int warning_time=0;
 unsigned long ble_low_heap_timer=0;
 bool restart_ble=false;

 void update_oled()
 {
	 		 display.clear();
		 display.setFont(ArialMT_Plain_10);
		 display.setTextAlignment(TEXT_ALIGN_LEFT);
		 display.drawString(0, 0, ("AirWhere Version : " + String(AwVersion) ) );
		 display.drawString(0, 14, ("ID : " + String(awHexManu) + String(awHexID) + " / Mode " + String(wiredChar))) ;
		 display.setFont(ArialMT_Plain_10);
		 if (webUpload)
		 {
			 display.drawString(0, 28, ("Internet Connected"));
		 }
		 else
         {
			 display.drawString(0, 28, ("Internet Not Connected"));
         }
		 if ( groundStationMode == 'y' )
		 {
			 display.drawString(0, 42, ("RX:  " + String(serial_rx_packets_counter)));
			 display.drawString(0, 54, "Ground Station Mode");
		 }
		 else
		 {
			  if (ogn_on=='y')
			  {

			    display.drawString(0, 42, ("T:" +  String(serial_tx_packets_counter) + " R: " + String(serial_rx_packets_counter) + " O: " + String(ogn_tx_packets)));
			  }
			  else
			  {

				  display.drawString(0, 42, ("T:" +  String(serial_tx_packets_counter) + " R: " + String(serial_rx_packets_counter)));
			  }
			 display.drawString(0, 54, ("Frequency : " + String(loraFrequencyC) ) );
		 }
		 display.display();
 }

 //v1.13

 void WiFiEvent(WiFiEvent_t event)
 {
	 if (debug_level>0)

     switch(event)
     {
         case SYSTEM_EVENT_AP_START:
             Serial.printf("AP started. IP: [%s]\n", WiFi.softAPIP().toString().c_str() );
             break;
         case SYSTEM_EVENT_AP_STOP:
             Serial.println("AP Stopped");
             break;
         case SYSTEM_EVENT_AP_STADISCONNECTED:
             Serial.println("WiFi Client Disconnected\n");
             break;
         case SYSTEM_EVENT_AP_STACONNECTED:
        	Serial.print("SYSTEM_EVENT_AP_STACONNECTED ");
        	break;
         case SYSTEM_EVENT_STA_GOT_IP:
        	 Serial.print("SYSTEM_EVENT_STA_GOT_IP!!!!!!!!!!!! ");
        	 break;

         default:
         //    Serial.printf("Unhandled WiFi Event: %d\n", event );
             break;
     }
 }


 //v1.1  - moved into task to run constantly so web doesnt freeze.
 void web_task(void *pvParameters) {
	 delay(5000);
	 while (1)
	 {
	   if  (wifiUp)
	   {
		  Web_loop();
	   }
	   delay(250);
//	   Serial.print("WEB TASK uxTaskGetStackHighWaterMark ");
	//   Serial.println(uxTaskGetStackHighWaterMark( NULL ));
	 }
  }


 void ble_task(void *pvParameters) {

	// BLEServer *pServer;

	 delay(1000);
	 if (debug_level>0)
	 {
		 Serial.println("INTO TASK BLE TASK");
	 }
	 start_ble( String(awHexManu) + String(awHexID));
	 delay(1000);
	 while (1)
	 {
	   // only send if we have more than 31k free heap space.

	   if (restart_ble)
	   {
		   restart_ble=false;
	   }
	   if (xPortGetFreeHeapSize()>BLE_LOW_HEAP)
	   {
		   ble_low_heap_timer = millis();
		   if (ble_data.length()>0)
           {
			   ble_mutex=true;

			   BLESendChunks(ble_data);

			//   Serial.println("CLEARING BLEOUT");
			   ble_data="";
			   ble_mutex=false;
           }
	   }
	   else
	   {


		   if (debug_level>0)
		   {
			   Serial.printf( "\n BLE congested - Waiting - Current free heap: %d, minimum ever free heap: %d\n", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
		   }
		   ble_mutex=true;
		   Serial.println("CLEARING BLEOUT");
		   ble_data="";
		   ble_mutex=false;
	   }


	   vTaskDelay(100);
	  // Serial.print("BLE TASK uxTaskGetStackHighWaterMark ");
	 //  Serial.println(uxTaskGetStackHighWaterMark( NULL ));

	 }
  }

void vario_task(void *pvParameters) {
 	Serial.println("Starting Task");
     vario_setup();
     vario_loop();
    // for(;;) {
     	//vTaskDelay(1000);
     //	Serial.println("Looping Task");
    //  	yield();
    //     micros(); //update overflow
    //     vario_loop();
 //    }
 }



void setup()
{
	esp_log_level_set("*", ESP_LOG_VERBOSE);


	WiFi.disconnect(true);
	delay(1000);
	WiFi.onEvent(WiFiEvent);

//sounds

	ledcAttachPin(SPEAKER_PIN,SPEAKER_CHANNEL);
	ledcWriteTone(SPEAKER_CHANNEL,400);
	delay(200);
	ledcWriteTone(SPEAKER_CHANNEL, 900);
	delay(200);
	ledcWriteTone(SPEAKER_CHANNEL, 1200);
	delay(200);
	ledcWriteTone(SPEAKER_CHANNEL, 0 );

	//ESP32
	//airWhereID=69;
  //system_restart();
#ifndef LIVE_VERSION
  //system_set_os_print(0); // stops all output to uart
#endif
  // ESP.wdtDisable();

  // wdtFeed() feeds watchdog
#ifdef ESP32_CPU
#else
if ( receiveFlarm=='y')
{
  nRF905_init();
  nRF905_setFrequency(NRF905_BAND_868 , RF_FREQ);
 // nRF905_setFrequency(NRF905_BAND_433 , 433200000UL);
  nRF905_setTransmitPower(NRF905_PWR_10);
  nRF905_setCRC(NRF905_CRC_16);
  byte addr[] = RXADDR;
  nRF905_setRXAddress(addr);
  nRF905_receive();
}
#endif

//Serial.begin(115200);
//v1.15
 // awHexID[0]='\0';
//Serial.begin(115200);
  load_configFile();

  //v4.1

  	if ( buzzer_on_off=='n')
  	{
  	//	turn_on_off_buzzer=2;
  	}
  	else
  	{
  //      turn_on_off_buzzer=0;
  	}

 Serial.flush();
 Serial.begin((unsigned long)serialBaudRate, (uint32_t) SERIAL_8N1,(int8_t) 3,(int8_t) 1);

  Serial.flush();
  Serial.setTimeout(1);

Serial.println(lora_or_flarm);

 

  if ( lora_or_flarm == 'h' || lora_or_flarm=='t' || lora_or_flarm=='r' || lora_or_flarm=='b')
  {
      gpio_pad_select_gpio(GPIO_NUM_2);
	  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
	  gpio_set_level(GPIO_NUM_2, 0);
  }

  //Serial.println("Startup 2");
  //v1.1
  int8_t gps_pin_tx;
  int8_t gps_pin_rx=22;

  if ( lora_or_flarm == 'h' || lora_or_flarm=='t' || lora_or_flarm=='f' || lora_or_flarm=='r')
  {



	  if (lora_or_flarm=='f')
	  {
		  GPSSerial=HardwareSerial(2);

		  gps_pin_tx=23;
	  }
	  else
	  {
		  GPSSerial=HardwareSerial(1);
		  gps_pin_tx=23;
	  }

  }
  if ( lora_or_flarm == 'l')
  {
	 // gps_pin=33;
	  GPSSerial=HardwareSerial(1);
	  gps_pin_tx=33;
	  //v2.11
  }

  if ( lora_or_flarm == 'b')
  {
	 // gps_pin=33;
	  GPSSerial=HardwareSerial(1);
	  gps_pin_tx=12;
	  gps_pin_rx=15;
	  //v2.11
  }



 // Serial.println("Startup 3");

  delay(100);
  GPSSerial.flush();
  GPSSerial.end();

  GPSSerial.begin((unsigned long)gpsBaudRate, (uint32_t) SERIAL_8N1,gps_pin_tx,gps_pin_rx);


  delay(100);

#ifdef SERIAL_UART1_OUT
 // Serial1.begin(57600);
#endif
  Serial.printf("\r\nAirWhere ESP32 - compiled on %s at %s - Version %d.\r\n", __DATE__, __TIME__, AwVersion );


  //v1.1 - for oled display
  switch (wiredDirectly)
  {
    case 'y':
	  strcpy (wiredChar,"HW");
	  break;
    case 'n':
   	  strcpy ( wiredChar, "Wifi");
   	  break;
    case 's':
   	  strcpy (wiredChar, "Wifi S");
   	  break;
    case 'o':
   	  strcpy (wiredChar, "OTG");
   	  break;
    case 'v':
   	  strcpy (wiredChar, "V2");
   	  break;
    case 'f':
   	  strcpy (wiredChar,"FSH" );
   	  break;

  }

  if ( loraFrequency == '8' )
  {
	  strcpy (loraFrequencyC, "868.2 MHz");
  }
  else
  {
	  strcpy (loraFrequencyC, "916.4 MHz");
  }

  //v1.1
  //Setup OLED for heltec board.

  if ( lora_or_flarm == 'h' || lora_or_flarm == 't')
  {
	  if ( lora_or_flarm == 't')
	  {
		  display = SSD1306(0x3c, 21, 22);
		  Serial.println("TTGO - SX1276 LORA - Starting OLED");
	  }
	  else
	  {
		  Serial.println("Heltec - SX1276 LORA - Starting OLED");
	  }

	//  lora_or_flarm = 'h';

	 // SSD1306  display(0x3c, 4, 15);
	  //SSD1306  display(0x3c, 21, 22);


      pinMode(16,OUTPUT);
      digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
      delay(50);
      digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
      display.init();
      display.flipScreenVertically();
      display.clear();
      display.setFont(ArialMT_Plain_24);
      display.setTextAlignment(TEXT_ALIGN_CENTER);
      display.drawString(64, 15, "AirWhere");
      display.setFont(ArialMT_Plain_10);
      display.drawString(80, 45, "loading.....");
      display.display();


  }

  if ( wiredDirectly == 'y' )
  {

   // Serial1.begin(57600);

    // GPS Baud Rate
    //swSer.begin(57600);
    //swSer.begin(9600);
	//  GPSSerial.flush();
	 // GPSSerial.print("\nAirWhere");
	 // GPSSerial.flush();

	//  GPSSerial.print(airWhereID);
	//  GPSSerial.flush();
	 //  GPSSerial.println(" Starting");
  }

  if ( wiredDirectly == 'o' || wiredDirectly == 'v' || wiredDirectly == 'f' )
  {

    Serial.print("\nAirWhere ");
   // Serial.print(airWhereID);
    Serial.println(" Starting");
  }

  delay(100);

  if ( wiredDirectly == 'y'  )
  {
	//  GPSSerial.flush();
	//  GPSSerial.println("AirWhere attempting to connect to Internet");
  }
  if ( wiredDirectly == 'o' || wiredDirectly == 'v' || wiredDirectly == 'f' )
  {
    Serial.println("AirWhere attempting to connect to Internet");
  }
//v1.15 moved
  /*WiFi_setup();
  Web_setup();

  if ( WiFi.status() == WL_CONNECTED)
  {
	  Serial.println("AirWhere connected to Internet");
  }
  else
  {
	  Serial.println("AirWhere failed to connect to Internet");
  }

*/
  webTime = 0;
  DEBUG_SERIAL_UART("[%ld] - Chip ID: %d\r\n", millis(), airWhereID);

  //Serial.println("Startup 4");

  last_client_connected_time = millis();
  last_data_received = millis();
  delay(100);

  //v16

  if (lora_or_flarm=='l' || lora_or_flarm=='r' || lora_or_flarm=='h' || lora_or_flarm=='t' || lora_or_flarm=='b' )
  {

      sprintf(awHexManu, "%02X",FANET_MANUFACTURER);

      app.begin();
      app.do_online_tracking=true;

      switch (web_aircraft_type)
      {
      case '1':
          app.aircraft_type=1;
          break;
      case '2':
          app.aircraft_type=2;
          break;
      case '3':
          app.aircraft_type=3;
          break;
      case '4':
          app.aircraft_type=4;
          break;
      }


      if (fmac.begin(app))
      {
    	  if ( lora_or_flarm == 'l')
          {
              Serial.println("RF Solutions Lora - SX1272 - attached");
          }
    	  if ( lora_or_flarm == 'r')
          {
              Serial.println("RFM95 - SX1276 LORA - attached");
          }
    	  if ( lora_or_flarm == 'h')
          {
              Serial.println("Heltec - SX1276 LORA - attached");
          }
    	  if ( lora_or_flarm == 't')
          {
              Serial.println("TTGO v2.0 - SX1276 LORA - attached");
          }
    	  if ( lora_or_flarm == 'b')
          {
              Serial.println("TTGO T-beam - SX1276 LORA - attached");
          }
      }
      else
      {
    	  if ( lora_or_flarm == 'l')
          {
              Serial.println("RF Solutions Lora - SX1272 - Not attached - PLease check wiring");
          }
    	  if ( lora_or_flarm == 'r')
          {
              Serial.println("RFM95 - SX1276 LORA - Not attached - PLease check wiring");
          }
    	  if ( lora_or_flarm == 'h')
          {
              Serial.println("Heltec - SX1276 LORA - Not attached - PLease check wiring");
          }
    	  if ( lora_or_flarm == 't')
          {
              Serial.println("TTGO v2.0 - Not attached - Please check wiring");
          }
    	  if ( lora_or_flarm == 'b')
          {
              Serial.println("TTGO T-beam - Not attached - Please check wiring");
          }
      }
//v1.15
      char * pEnd;
      fmac.my_addr=MacAddr(strtol( awHexManu, &pEnd, 16),strtol( awHexID, &pEnd, 16));

      // if theres data in the ogn_id assume valid ( website wont allow invalids. )

      if (strlen(ogn_id)!=0)
      {
    	  // see http://wiki.glidernet.org/ogn-tracking-protocol
    	  char * pEnd;
    	 // uint32_t Address=strtoul(ogn_id,&pEnd,16);

    	  fmac.AcftID.Address=strtoul(ogn_id,&pEnd,16);
    	  //ogn type message = 0x3
    	  fmac.AcftID.AddrType=0x3;
    	  fmac.AcftID.Stealth=0x0;

          switch (web_aircraft_type)
          {
          case '1':
        	  fmac.AcftID.AcftType=0x7;
              break;
          case '2':
        	  fmac.AcftID.AcftType=0x6;
              break;
          case '3':
        	  fmac.AcftID.AcftType=0x11;
              break;
          case '4':
        	  fmac.AcftID.AcftType=0x1;
              break;
          }
      }
  }
  else
  {
      //start flarm board.

      flarm.reset();

      flarm.do_online_tracking=true;
      switch (web_aircraft_type)
      {
      case '1':
          flarm.aircraft_type=1;
          break;
      case '2':
          flarm.aircraft_type=2;
          break;
      case '3':
          flarm.aircraft_type=3;
          break;
      case '4':
          flarm.aircraft_type=4;
          break;
      }
      flarmSerial=HardwareSerial(1);

      flarmSerial.flush();

      delay(100);

      flarmSerial.begin(FLARM_BOARD_BAUD, (uint32_t) SERIAL_8N1,(int8_t) FLARM_RX_PIN,(int8_t) FLARM_TX_PIN);

      delay(100);

      flarmSerial.flush();
      flarmSerial.setTimeout(FLARM_BOARD_SERIAL_TIMEOUT);
      delay(100);
      if ( flarm.begin(flarmSerial, manufacturer_code, flarm_board_id, flarm_board_expiration, flarm_board_build, flarm_tracking_on, fanet_tracking_on ) )
      {
          if ( wiredDirectly == 'y')
          {
              Serial.println("Flarm board Found");
          }
          else
          {
              Serial.println("Flarm board Found");
          }
      }
      else
      {

          if ( wiredDirectly == 'y')
          {
              Serial.println("Error :- Flarm board selected but not Found - check wiring.");
          }
          else
          {
              Serial.println("Error :- Flarm board selected but not Found - check wiring.");
          }
      }
    /*  if (!fanet_tracking_on || !flarm_tracking_on )
      {
          flarm_needs_reset=true;
          flarm_reset_time=millis()+FLARM_RESET_SPACE;
          if ( wiredDirectly == 'y')
          {
              Serial.println("Flarm board Found - but failed to initialise properly - resetting.");
          }
          else
          {
              Serial.println("Flarm board Found - but failed to initialise properly - resetting.");
          }
      }
*/
      //v1.15

      strcpy(awHexManu,manufacturer_code);
      strcpy(awHexID,flarm_board_id);

      char * pEnd;
     // fmac.my_addr=MacAddr(strtol( awHexManu, &pEnd, 16),strtol( awHexID, &pEnd, 16));
      fmac.my_addr=MacAddr(flarm.manufacturer,flarm.id);
  }

  //Serial.println("Startup 5");

  //  Serial.println(fmac.my_addr.id,HEX);

  //v1.15
  WiFi_setup();
 // Serial.println("Startup 6");
  Web_setup();


  if ( WiFi.status() == WL_CONNECTED)
  {
	  Serial.println("AirWhere connected to Internet");
  }
  else
  {
	  Serial.println("AirWhere failed to connect to Internet");
  }


  if ( wifioffSelected == 'y')
  {
    wifiDownTime=180000;
  }
  else
  {
    wifiDownTime=0;
  }
  delay(500);

  GPGGAdataIn.isLineValid=false;
  GPRMCdataIn.isLineValid=false;

  //int attempts=0;

  // we need a check here to make sure the vario has found the sensor and its working properly.
  // for now just assume its working if its attached in the web browser.

  if (ms5611_attached=='y')
  {
	  ms5611_online=true;
  }

 /* if (ms5611_attached=='y')
  {
      Serial.println("Initialize MS5611 Sensor");
      if( ms5611.begin() )
      {
    	Serial.print("MS5611 Sensor Online - sampling set to ");
        Serial.println(ms5611.getOversampling());
        ms5611_online=true;
        delay(10);
        current_pressure=ms5611.readPressure();
        delay(10);
        current_temperature=ms5611.readTemperature();

        Serial.println(current_pressure);
        Serial.println(current_temperature);
      }
      else
      {
    	Serial.println("MS5611 Sensor Selected however no sensor found - check wiring");
    	ms5611_online=false;
      }

  }
  else
  {
	  Serial.println("MS5611 Sensor not Attached");
	  ms5611_online=false;
	  ms5611_attached='n';
  }
 */




  xTaskCreatePinnedToCore(web_task, "web_task", 6500, NULL, 1, NULL, ARDUINO_RUNNING_CORE1);
  delay(50);



  if (ms5611_attached=='y' && vario_on=='y')
  {
	  Serial.println("Starting Vario");
	 //xTaskCreate(&vario_task, "vario_task", 8192, NULL, 5, NULL);
	  xTaskCreatePinnedToCore(vario_task, "vario_task", 8192, NULL, 1, NULL, ARDUINO_RUNNING_CORE1);
	  Serial.println("After Vario");
 // delay(10000);
  }

  Serial.printf( "current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());

#ifdef INCLUDE_BLE
  if ( ble_output=='e')
  {
	  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

	if (debug_level>0)
	{
	  Serial.println("Starting BLE");
	  Serial.println("Heap before initializing BLE code");
	  Serial.printf( "current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
	}
 //   start_ble( "AirWhere" + String(awHexManu) + String(awHexID));
  //  SerialBT.begin("AirWhere" + String(awHexManu) + String(awHexID)); //Bluetooth device name
	xTaskCreatePinnedToCore(ble_task, "ble_task", 4096, NULL, 1, NULL, ARDUINO_RUNNING_CORE1);

	if (debug_level>0)
	{
	  Serial.println("Heap after start BLE");
	  Serial.printf("current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
	}
//	BLESendChunks("AirWhere BLE Starting");
  }
  if ( ble_output=='u')
  {
		if (debug_level>0)
		{
	  	  Serial.println("Starting Bluetooth");
		  Serial.println("Heap before initializing Bluetooth code \r\n");
		  Serial.printf( "current free heap: %d, minimum ever free heap: %d\n", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
		}
	   // start_ble( "AirWhere" + String(awHexManu) + String(awHexID));
	    SerialBT.begin("AirWhere" + String(awHexManu) + String(awHexID)); //Bluetooth device name

		if (debug_level>0)
		{
	      Serial.println("Heap after start Bluetooth \r\n");
		  Serial.printf( "current free heap: %d, minimum ever free heap: %d\n", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
		}
		SerialBT.print("AirWhere Bluetooth Starting");

  }
#endif

  Serial.printf( "current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());

//  if ( wiredDirectly == 'y' )
 // {
    Serial.print("AirWhere ");
    Serial.print(awHexManu);
    Serial.print(awHexID);
   Serial.println(" configuration completed, lets go !");
//  }
//  if ( wiredDirectly == 'o' || wiredDirectly == 'v' || wiredDirectly == 'f')
 // {
//    Serial.print("AirWhere ");
 //   Serial.print(awHexManu);
  //  Serial.print(awHexID);
  //  Serial.println(" configuration completed, lets go !");
 // }

  if (groundStationMode == 'y')
  {
      Serial.print("Ground Station ");
      Serial.print(awHexManu);
      Serial.print(awHexID);
      Serial.println("Mode Activated - Waiting for Pilots.... ");
  }

  //v1.1 - leave loading logo on for 2 seconds.
  if ( lora_or_flarm == 'h'|| lora_or_flarm=='t' )
  {
	  delay(500);
	  update_oled();
  }

  if (wiredDirectly == 's')
  {
	  // gps is always connected.
	  socket_connected = true;
  }

  Serial.printf( "current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());

}

void loop()
{
//v5

//  Serial.printf( "current free heap: %d, minimum ever free heap: %d", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
  
	if (xPortGetMinimumEverFreeHeapSize()<100000)
	{
		if (millis()>warning_time)
		{
	      if (debug_level>0)
	      {
		    Serial.printf( "*****LOOP current free heap: %d, minimum ever free heap: %d\n******", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
	      }
		  warning_time=millis()+1000;
		}
	}

	if (xPortGetMinimumEverFreeHeapSize()<5000)
	{
      if (debug_level>0)
	      {
		    Serial.printf( "*****LOOP current free heap: %d, minimum ever free heap: %d\n******", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
	      }
		Serial.println("System Low on Memory - xPortGetMinimumEverFreeHeapSize < 2KB");
		Serial.println("ESP Restarting !");
		esp_restart();
	}

	if (  ble_output=='e' && ble_low_heap_timer + MAX_BLE_LOW_HEAP_TIME <millis() && deviceConnected)
	{
		Serial.println("Running Less than BLE_LOW_HEAP for more than MAX_BLE_LOW_HEAP_TIME milliseconds");
		Serial.println("ESP Restarting !");
		// restart BLE
		restart_ble=true;
		esp_restart();
	}

	//v4 turn tx led off

	if (millis()>tx_led_off_time && tx_led_on)
	{
		gpio_set_level(GPIO_NUM_2, 0);
		tx_led_on=false;
	}


	yield();
  //  Serial.print("Loop :-");
	//Serial.println(millis());
  //v1.1 stop this loop if we are updating flarm board.
  if (flarm_update_in_process)
  {
	  main_loop_halted=true;
	  Serial.println("Stopping main loop to process board update");
	  delay(1000000);
  }


  DEBUG_SERIAL_UART_MAX("[%ld] - Before fmac.handle\r\n", millis());

  //v1.1
  if (lora_or_flarm=='l' || lora_or_flarm=='r' || lora_or_flarm=='h' || lora_or_flarm=='t' || lora_or_flarm=='b')
  {
	  fmac.handle();
  }
  else
  {
	  if (flarm_needs_reset && millis()>flarm_reset_time)
	  {
		  flarm.reset();
		  //flarm.begin(flarmSerial, manufacturer_code, flarm_board_id, flarm_board_expiration, flarm_board_build, flarm_tracking_on, fanet_tracking_on );
	      if ( flarm.begin(flarmSerial, manufacturer_code, flarm_board_id, flarm_board_expiration, flarm_board_build, flarm_tracking_on, fanet_tracking_on ) )
	      {
             Serial.println("Flarm board Found");
             flarm_needs_reset=false;
	      }
	      else
	      {
             Serial.println("Error :- Flarm board reset but not Found - check wiring.");
             flarm_needs_reset=true;
             flarm_reset_time=millis()+FLARM_RESET_SPACE;
          }
	  }
	  else
	  {
		  flarm.handle(flarmSerial);
	  }
  }

  yield();

  //Serial.printf( "1. - current free heap: %d, minimum ever free heap: %d\n", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());

  // the esp32 doesnt handle interrupts properly yet, it falls over so we just set a flag in the interrupt
  // and check it and handle it here instead in the main loop - as its looping 200 times + a second
  // 99% of the time it catches the received packet  - revisit when interrupts are ok.

  DEBUG_SERIAL_UART_MAX("[%ld] - Before handle irq\r\n", millis());


  // this irq handling only requires for the sx1272/sx1276 chip as it falls over in checking for irq.
  //v1.1
  if (lora_or_flarm!='f')
  {
	  app.handle_irq(lora_or_flarm);
  }



  DEBUG_SERIAL_UART_MAX("[%ld] - Before sounds irq\r\n", millis());

  if ( soundsOn=='y' )
  {
	// if its otg mode, sounds are read in the main loop for now.
	// so only process stream when its set to gps inside esp32.

    // if theres beeps in the tuneIs array then play them, if we have got to the end of the array then stop sound.
    process_next_beep( tuneIs, noTones, currentTone);

    // this checks the serial in from RX and fills the tones array, also starts the first tone.
    // if another tone comes in while the tones are playing, the current sequence is stopped and the new one started.
    if (wiredDirectly == 'y')
    {
      process_sound_stream(soundBuffer, tuneIs, noTones, currentTone);
    }
  }

  // barometer section - get a reading every loop - output to kobo every second.

  DEBUG_SERIAL_UART_MAX("[%ld] - Before baro \r\n", millis());

  if (ms5611_online)
  {

    if (millis()>pressure_temp_output_time )
    {

/*
		String LK8EX1 = "$LK8EX1," +
			(String)current_pressure + "," +
			(String)current_altitude + "," +
			(String)current_vario + "," +
			(String)(current_temperature - TEMP_OFFSET) + "," +
			(String)current_battery + "*";
		NEMEA_Checksum(&LK8EX1);

		if ( ble_output=='e')
		{
			//Serial.print("Sending pressure data over ble in lk8000 format; "); Serial.println(LK8EX1.c_str());

		  BLESendChunks(LK8EX1);
		}
		if ( ble_output=='u')
		{
			if (SerialBT.hasClient()) {

					Serial.printf("*** Classic BT Sent Value: %s", LK8EX1.c_str());
					SerialBT.print(LK8EX1);
			}
		}
*/
	//	if (SerialBT.available()) {
	//		Serial.printf("*** Classic BT Sent Value: %s", LK8EX1.c_str());
	//		SerialBT.print(LK8EX1);
	//	}


        // temp is not needed to be updated often so perhaps every 10 secs?
        DEBUG_SERIAL_UART_MAX("[%ld] - Before ReadTemp \r\n", millis());
    //    Serial.println("Before ReadTemp");

      //VARIO  if (millis()>temp_read_time)
        //VARIO  {
        //  delay(5);
      //VARIO    current_temperature=ms5611.readTemperature();
        //VARIO     temp_read_time=millis()+TEMP_OUTPUT_PERIOD;
     //     Serial.println("REEEEEEEEEEEEEEEEEADING TEMP");
        //VARIO    }
    //VARIO     else
        //VARIO     {
         // DEBUG_SERIAL_UART_MAX("[%ld] - Before readPressure \r\n", millis());
        	//VARIO current_pressure=ms5611.readPressure();
        //VARIO     }

  //    Serial.println(current_pressure);
  //    Serial.println(current_temperature);

        // only display valid pressures - some weird values arrive sometimes.

        if( current_pressure>PRESSURE_AT_18000_FT && current_pressure<PRESSURE_AT_0 )
        {

/*#ifdef INCLUDE_BLE
			if (ble_output == 'e' && deviceConnected)
			{//v1.1



				if (navSofware == 'L' || navSofware == 'K')
				{
					String LK8EX1 = "$LK8EX1," +
						(String)current_pressure + "," +
						(String)current_altitude + "," +
						(String)current_vario + "," +
						(String)(current_temperature - TEMP_OFFSET) + "," +
						(String)current_battery + "*";
					NEMEA_Checksum(&LK8EX1);
					Serial.print("Sending pressure data over ble in lk8000 format; "); Serial.println(LK8EX1);
					BLESendChunks(LK8EX1);
				//	if (SerialBT.available()) {
				//		Serial.printf("*** Classic BT Sent Value: %s", LK8EX1.c_str());
				//		SerialBT.print(LK8EX1);
				//	}
				}

				if (navSofware == 'X'|| navSofware == 'A')
				{

					//PRS XXXXX\n
					char hex_pressure[6];
					itoa(current_pressure, hex_pressure, 16);

					int i = 0;

					while (hex_pressure[i])
					{
						hex_pressure[i] = toupper(hex_pressure[i]);
						i++;
					}

					String PRS = "PRS " + (String)hex_pressure + "\n";
					BLESendChunks(PRS);
				//	if (SerialBT.available()) {
				//		//Serial.printf("*** Classic BT Sent Value: %s", PRS);
				//		SerialBT.print(PRS);
				//	}

				}
			}
#endif
*/

			if ( wiredDirectly == 'y' || wiredDirectly == 'o' || wiredDirectly == 'v' || wiredDirectly == 's' || wiredDirectly == 'f')
			{
          //queue stream data up , not just send - needs ordering.
            if (navSofware == 'L' || navSofware == 'K')
            {
              //  $LK8EX1,pressure,altitude,vario,temperature,battery,*checksum\r\n":
            //	sprintf(LK8EX1_Sentence, "$LK8EX1,%d,%d,%d,%d,%d.%d*", (int)baro.paSample_, (int)(kfAltitudeCm/100.0), (int)kfClimbrateCps, (int)baro.celsiusSample_, bv/10,bv%10);

                String LK8EX1="$LK8EX1,"+
                		(String)current_pressure + "," +
						(String)current_altitude + "," +
						(String)current_vario + "," +
						(String)(current_temperature - TEMP_OFFSET)  + "," +
						(String)current_battery+"*";

                writeDataToSerial(LK8EX1);
            }
            if (navSofware == 'X' || navSofware == 'A')
            {
                //PRS XXXXX\n
                char hex_pressure[6];
                itoa(current_pressure, hex_pressure,16);

                int i=0;

                while (hex_pressure[i])
                {
                  hex_pressure[i]=toupper(hex_pressure[i]);
                  i++;
                }

                String PRS="PRS "+(String)hex_pressure +"\n";

                write_data_serial_no_cksum(PRS);
            }

         }
         else
         {
           // network stuff goes straight out as we send out a line at time.
           // need to merge standalone stuff.


           if (navSofware == 'L')
           {
               String LK8EX1="$LK8EX1,"+
               		(String)current_pressure + "," +
						(String)current_altitude + "," +
						(String)current_vario + "," +
						(String)current_temperature + "," +
						(String)current_battery+"*";

               writeDataToWifi( airwhere_client, LK8EX1);
           }
// we send lk android out to udp.

           if (navSofware == 'X' || navSofware == 'A' || navSofware == 'K' )
           {
             char hex_pressure[6];
             itoa(current_pressure, hex_pressure,16);

             int i=0;

             while (hex_pressure[i])
             {
               hex_pressure[i]=toupper(hex_pressure[i]);
               i++;
             }

             String PRS="PRS "+(String)hex_pressure +"\n";
             sendUDPPacket ( PRS, apFoundIP, false );
           }

         }

        }
        pressure_temp_output_time=millis()+BARO_OUTPUT_PERIOD;

     }
  }


 /*if (deviceConnected) {

		sprintf(LK8EX1_Sentence, "$LK8EX1,%d,%d,%d,%d,%d.%d*", 101300, 8500, 2, 20, 3, 7);
		NEMEA_Checksum(LK8EX1_Sentence);
		Serial.printf("******************************** Sent Value: %s", LK8EX1_Sentence);
		BLESendChunks(LK8EX1_Sentence);


	}*/
 // Serial.printf( "2. - current free heap: %d, minimum ever free heap: %d\n", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
  if ( millis() > wifiDownTime && wifiDownTime!=0 && ( wiredDirectly == 'y' || wiredDirectly == 'o' || wiredDirectly == 'v' || wiredDirectly == 'f'))
  {

	  //   WiFi.mode(WIFI_OFF);
	  //   WiFi.forceSleepBegin();
	   //  wifi_set_sleep_type(MODEM_SLEEP_T);
	     wifiDownTime=0;
	     wifiUp=false;

	     esp_wifi_set_mode(WIFI_MODE_NULL);
	         	   esp_wifi_stop();
	         	   Serial.println("******************WEBCONFIG Setting - WIFI STOPPING************************* ");

  }
#ifdef ESP32_CPU

#else


  if (WiFi.status() != WL_CONNECTED)
  {
   DEBUG_SERIAL_UART_MAX("[%ld] - ********************* WIFI DOWN  ************************* \r\n", millis());
   //Serial1.println(" ********************* WIFI DOWN  ******************** \r\n");
  }
  else
  {
   DEBUG_SERIAL_UART_MAX("[%ld] - ********************* WIFI UP :)  ************************* \r\n", millis());
  //    Serial1.println(" **************************** WIFI UP :)  ********************* \r\n");
  }
  DEBUG_SERIAL_UART_MAX("[%ld] - Starting Loop\r\n", millis());
#endif
//  bool startLoop = true;

 // if (millis() > t)
//  {
    int eT = (int)millis();
    int sT = (int)t;



    //free_mem = system_get_free_heap_size();

    loopTime = eT - sT;

   // Serial.println("Looptime " + loopTime);
//#ifdef TIMINGS
  //  if (  loopTime > 1)
 //  {
      DEBUG_SERIAL_UART_MAX("[%ld] - Looptime [%d]\r\n", millis(), loopTime);
  //ESP32    DEBUG_SERIAL_UART_MAX("[%ld] - Free Memory [%ld]\r\n", millis(), free_mem);
     // Serial.println("Looptime " + loopTime);
 //   }
//#endif

    if ( loopTime > longest_loop_time)
    {
      longest_loop_time = loopTime;
      if (longest_loop_time > 1000)
      {
        longest_loop_time = 0;
      }
    }
    t = millis();
 // }

//  Serial.println("b4 Transmit");
  //if (rfPktToSend)
    //v1.1

   /* if (lora_or_flarm=='l' || lora_or_flarm=='r' || lora_or_flarm=='h' || lora_or_flarm=='t')
    {
    	if (app.is_broadcast_ready(NUM_NEIGHBOURS))
    	{
    		//if ( millis() > spacerTime)
    		// {
    		//int tx = sx1272.sendFrame(TxBuffer, PKT_SIZE+1);

    		//  app.broadcast_successful(FANET_PACKET_TYPE);
    		app.get_frame();
    		//  Serial.println("in Transmit");

    		int tx = 0;
    		// if ( wiredDirectly == 'y' )
    		//  {
    		if ( tx==0 )
    		{
    			  if ( lora_or_flarm == 'h' || lora_or_flarm=='t')
    			  {
    				  update_oled();
    			  }

    			if ( wiredDirectly == 'y' || wiredDirectly == 'o' || wiredDirectly == 'v' ||  wiredDirectly == 'f')
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
    				if (navSofware == 'X' || navSofware == 'A' || navSofware == 'K' )
    				{
       					sendUDPPacket ( "$AW,TX,*", apFoundIP, true );
    				}

    			}

    			rfPktToSend=false;
    		}
    		else
    		{
    			if ( wiredDirectly == 'y' || wiredDirectly == 'o' || wiredDirectly == 'v' || wiredDirectly == 'f')
    			{
    				//streamData=("$AWARE,DEVICE FAILURE - CHECK BOARD & WIRING \r\n"+streamData);
    				//queue stream data up , not just send - needs ordering.
    				writeDataToSerial("$AIRWHERE,DEVICE FAILURE - CHECK BOARD & WIRING");
    			}
    			else
    			{

    				if (navSofware == 'L')
    				{
    					writeDataToWifi( airwhere_client, "$AIRWHERE,DEVICE FAILURE - CHECK BOARD & WIRING  ");
    				}
    				//v1.1
    				if (navSofware == 'X' || navSofware == 'A' || navSofware == 'K'  )
    				{
    					sendUDPPacket ( "$AIRWHERE,DEVICE FAILURE - CHECK BOARD & WIRING  ", apFoundIP, true );
    				}
    			}
    			rfPktToSend=false;

    		}

    		if ( tx==-1 )
    		{
    			//    streamData=("$AWARE,CANT TRANSMIT TX ONGOING\r\n"+streamData);
    		}
    		if ( tx==-2 )
    		{
    			//     streamData=("$AWARE,CANT TRANSMIT RX ONGOING\r\n"+streamData);
    		}

    		//   }


    	}
    }
*/
  unsigned int number_client=0;
  wifi_sta_list_t stations;
  tcpip_adapter_sta_list_t infoList;
  wifi_sta_list_t sta_list;
  esp_wifi_ap_get_sta_list(&sta_list);

  if ( wiredDirectly == 'n'  )
  {

  /*    for(int i = 0; i < sta_list.num; i++)
      {
          //Print the mac address of the connected station
          wifi_sta_info_t &sta = sta_list.sta[i];
          DEBUG_SERIAL_UART_MAX("[%ld] - Station %d MAC: %02X:%02X:%02X:%02X:%02X:%02X\n - ", millis(), i,
                  sta.mac[0], sta.mac[1], sta.mac[2], sta.mac[3], sta.mac[4], sta.mac[5]);
       //   printf("Station %d MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", i,
        //      sta.mac[0], sta.mac[1], sta.mac[2], sta.mac[3], sta.mac[4], sta.mac[5]);

      }

         ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list(&stations));

         ESP_ERROR_CHECK(tcpip_adapter_get_sta_list(&stations, &infoList));
         int i = 0;
         number_client=infoList.num;
         while(i < infoList.num) {
             DEBUG_SERIAL_UART_MAX("[%ld] - mac: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x " IPSTR " %d\n", millis(),
                      infoList.sta[i].mac[0],infoList.sta[i].mac[1],infoList.sta[i].mac[2],
                      infoList.sta[i].mac[3],infoList.sta[i].mac[4],infoList.sta[i].mac[5],
                      IP2STR(&(infoList.sta[i].ip)),
                      (uint32_t)(infoList.sta[i].ip.addr));
           //  printf("mac: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x " IPSTR " %d\n",
          //            infoList.sta[i].mac[0],infoList.sta[i].mac[1],infoList.sta[i].mac[2],
           //           infoList.sta[i].mac[3],infoList.sta[i].mac[4],infoList.sta[i].mac[5],
            //          IP2STR(&(infoList.sta[i].ip)),
            //          (uint32_t)(infoList.sta[i].ip.addr));
            i++;


         }
*/
	  ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list(&stations));
	  ESP_ERROR_CHECK(tcpip_adapter_get_sta_list(&stations, &infoList));

	  if (infoList.num>0)
	  {
		  if (!ip_assigned)
		  {
			ip_assigned=true;
			waiting_for_ip=millis()+5000;
		  }

		  if (millis()>loopy)
		  {
			//  Serial.println(infoList.num);
			  loopy=millis()+1000;
		  }
		  for(int i = 0; i < sta_list.num; i++)
		  {
			  //Print the mac address of the connected station
			 // wifi_sta_info_t &sta = sta_list.sta[i];
		  }
		  // for now only assign first connected ip.

	        // for now only assign first connected ip.
		      if (infoList.sta[0].ip.addr == 0 && millis() > waiting_for_ip)
		      {
		    	//  Serial.print("Need to KICK IP");

		    	//  esp_wifi_deauth_sta(0);
		    //	  waiting_for_ip=0;ip_assigned=false;
		    	  waiting_for_ip=0;ip_assigned=true;

		    	  socket_ip=IPAddress(192,168,4,2);
		    	  apFoundIP=IPAddress(192,168,4,2);
		    	 // socket_ip=infoList.sta[0].ip.addr;
		      }
		      else
		      {
				  socket_ip=infoList.sta[0].ip.addr;
				  apFoundIP=infoList.sta[0].ip.addr;
		      }



		  number_client = 1;
		  if (!socket_connected)
		  {
		     udp_flight_software.begin(UDP_FLIGHT_SOFTWARE_PORT);
		     udp_flight_software_running=true;
		  }



		  socket_connected = true;

		  //Serial.println("ap connected on wired directly=n");
	  }
	  else
	  {
		  number_client =0;
		  socket_connected = false;
		  if (udp_flight_software_running)
		  {
			  udp_flight_software.stop();
		  }
		//  udp_flight_software.available();
		  udp_flight_software_running=false;
		  if (debug_level>0)
		  {
			  if ( groundStationMode != 'y' )
			  {
				  Serial.println("UDP STOPPED");
			  }
		  }
		  if ( groundStationMode != 'y' )
		  {
		    GPGGAdataIn.isLineValid=false;

		  }

	  }

  }
  else
  {
	  if (wiredDirectly == 's')
	  { // if we can find a IP then we are away.

	      ESP_ERROR_CHECK(esp_wifi_ap_get_sta_list(&stations));
          ESP_ERROR_CHECK(tcpip_adapter_get_sta_list(&stations, &infoList));


		  if (infoList.num>0)
		  {
			  yield();
			  if (!ip_assigned)
			  {
				ip_assigned=true;
				waiting_for_ip=millis()+5000;
			  }

		      for(int i = 0; i < sta_list.num; i++)
		      {
		          //Print the mac address of the connected station
		          wifi_sta_info_t &sta = sta_list.sta[i];

		      }
	        // for now only assign first connected ip.
		      if (infoList.sta[0].ip.addr == 0 && millis() > waiting_for_ip)
		      {
		    	//  Serial.print("Need to KICK IP");

		    	//  esp_wifi_deauth_sta(0);
		    //	  waiting_for_ip=0;ip_assigned=false;
		    	  waiting_for_ip=0;ip_assigned=true;

		    	  socket_ip=IPAddress(192,168,4,2);

		    	 // socket_ip=infoList.sta[0].ip.addr;
		      }
		      else
		      {
		        socket_ip=infoList.sta[0].ip.addr;
		      }

			  if (millis()>loopy)
			  {
				 // Serial.println(infoList.sta[0].ip.addr);
				//  Serial.println(infoList.num);
				  loopy=millis()+1000;
		//	 Serial.print("infoList.num :-");
				//  					  Serial.print(infoList.num);
				//  					  Serial.print("Client connected :-");
				 // 					  Serial.print(socket_ip.toString());
					if (debug_level>0)
					{
					  Serial.print("infoList.num :-");
					  Serial.print(infoList.num);
					  Serial.print("Client connected :-");
					  Serial.print(socket_ip.toString());
					 }
			  }


		    number_client = 1;

		  }
		  else
		  {
			  if (debug_level>0)
			  {
				  Serial.println("UDP STOPPED");
			  }
			  waiting_for_ip=0;
			  ip_assigned=false;
			 // socket_connected = false;
			  GPGGAdataIn.isLineValid=false;
		  }
	  }
	  else
	  {
	    number_client = 1;
	    socket_connected = true;
	  }
  }


  DEBUG_SERIAL_UART_MAX("[%ld] - Number of Clients =%d \r\n", millis(), number_client);

  if (groundStationMode == 'n')
  {
    if (number_client == 0)
    {
      DEBUG_SERIAL_UART("[%ld] - No clients connected to AP\r\n", millis());

#ifdef ESP32_CPU

#else


      ipFound = false;
#endif
      //socket_connected = false;
    //  myDelay(100);
      //DEBUG_SERIAL_UART("[%ld] - set_Connected_Time is set to FALSE : %d \r\n", millis(),client_connected_time);
      set_Connected_Time = false;

      // no one has connected in 90 secs ( 90000 millis ), restart wifi to make sure theres not something weird gone on with wifi.
      // until we find the problem
      if ( (last_client_connected_time + 45000) < millis())
      {
        DEBUG_SERIAL_UART("[%ld] - Restarting WIFI - No connect on Wifi\r\n", millis());
        last_client_connected_time = millis();
      }
    }
    else
    {
      // last_ is just a marker for restarting the WIFI if no clients connected.
      last_client_connected_time = millis();
      DEBUG_SERIAL_UART("[%ld] - Setting Last Client_connected_time=%d \r\n", millis(), last_client_connected_time);

      // client is for restarting wifi if a client has connected and it cant find a socket.
      if (!set_Connected_Time)
      {
        // delay for the dchp server to work its magic
        yield();
        myDelay(1000);
        yield();
        looking_for_socket_time = millis();
        set_Connected_Time = true;
      }


      if (!displayedclientconnected)
      {
        DEBUG_SERIAL_UART("[%ld] - Client connected on DHCP\r\n", millis());
#ifdef SERIAL_UART_OUT
       //ESP32 client_status();
#endif


        displayedclientconnected = true;
      }
    }

    yield();
    //v1.11
   // Serial.printf( "3. - current free heap: %d, minimum ever free heap: %d\n", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
    //if ( wiredDirectly == 's' || wiredDirectly == 'n')
  //v2  if( wiredDirectly == 'n')
    /*{
      if (socket_connected && !airwhere_client.connected())
      {
     	socket_connected=false;
    	ESP_LOGI(TAG,"Socket was connected - for some reason its been disconnected.");
      }
    }*/

/*
    if (number_client > 0 && !socket_connected)
    {
       At the moment the esp code sometimes adds an IP that you can see in the station list,
         sometimes doesnt, we have set the dhcp to only assign IP's, 2 to 5, so it must be one of those.
         Iterate through the 4 IP's while theres a client connected and stop looking when theres no clients
         Also stop looking after a certain time and reboot and start again as there might be some weird error.
      if ( looking_for_socket_time + 90000 < millis())
      {
        DEBUG_SERIAL_UART("[%ld] - Not released Client - cant find a socket - Restarting Access Point \r\n", millis());
        yield();
        myDelay(500);
        ESP_LOGI(TAG, "Prepare to restart system!");
        esp_restart();
      }

  	  int iPs[4] = {2, 3, 4};

      for ( int ip = 0; ip < 3; ip++)
      {

    	ESP_ERROR_CHECK(tcpip_adapter_get_sta_list(&stations, &infoList));

        if ( infoList.num == 0 )
        {
          socket_connected = false;
          break;
        }


      //  DEBUG_SERIAL_UART("[%ld] - Connecting to found IP %s\r\n", millis(),IP2STR(&(infoList.sta[i].ip)));
        //  DEBUG_SERIAL_UART("[%ld] - Trying connected IP : %d \r\n",millis(),LK8000iPAddress);

    ///      DEBUG_SERIAL_UART_MAX("[%ld] - mac: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x " IPSTR " %d\n", millis(),
     //                         infoList.sta[clientNo].mac[0],infoList.sta[clientNo].mac[1],infoList.sta[clientNo].mac[2],
      //                        infoList.sta[clientNo].mac[3],infoList.sta[clientNo].mac[4],infoList.sta[clientNo].mac[5],
      //                        IP2STR(&(infoList.sta[clientNo].ip)),
       //                       (uint32_t)(infoList.sta[clientNo].ip.addr));

        bool lkConnect1 = false;

       // IPAddress ip1=(uint32_t)(infoList.sta[clientNo].ip.addr);
        IPAddress ip1=IPAddress(192,168,4,iPs[ip]);
        if (millis() > webTime)
        {
            const int ap = 4353;

            Serial.print("Connecting to :-");
            Serial.println(ip1.toString());
         // lkConnect1 = airwhere_client.connect(infoList.sta[clientNo].ip, ap);
            airwhere_client.setNoDelay(5);
            airwhere_client.setTimeout(1);

            lkConnect1 = airwhere_client.connect(ip1, ap);
            // then check the web server to process any incoming requests.
            Web_loop();
        }

        if (lkConnect1)
        {
          DEBUG_SERIAL_UART("[%ld] - *** Connection to Flight Software on Found IP *** \r\n", millis());
          airwhere_client.setNoDelay(true);
#ifdef LOGGING_ON_MAX
          loggingS = String(airPKT.pktPilotID) + "," + String(millis()) + "," + "Connected to LK TCP Server";
          upLoadtoUDP(loggingS, loggingIP, LOGGING_PORT);
#endif
          airwhere_client.setTimeout(1);
          socket_connected = true;
          last_data_received = millis();
          apFoundIP = ip1;
          break;
        }
        else
        {
          DEBUG_SERIAL_UART("[%ld] - No connection on  :- 192.168.4.%d\r\n", millis(),iPs[ip]);
        }
        Web_loop();
      }
    }
        //   WiFi_setup();

        yield();
        //check to see if they are still connected...

        number_client = wifi_softap_get_station_num();

        int iPs[4] = {100, 101, 2, 3};

        for ( int ip = 0; ip < 4; ip++)
        {
          Web_loop();
          yield();
          // client_connected_time is set when a client connected, if either no clients connect or a client has connected and cant find a socket
          // then restart the ESP, also the ESP sometimes doesnt realise a client has disconnected so just sits there trying to find a socket
          // if so this should restart it.
          DEBUG_SERIAL_UART("[%ld] - looking_for_socket_time=%d \r\n", millis(), looking_for_socket_time);

          if ( looking_for_socket_time + 45000 < millis())
          {
            DEBUG_SERIAL_UART("[%ld] - Not released Client - cant find a socket - Restarting Access Point \r\n", millis());
            LK8000Client.stop();
            yield();
            myDelay(500);
            //WiFi_setup();
            ESP.restart();
          }

          number_client = wifi_softap_get_station_num();

          if ( number_client == 0 )
          {
            ipFound = false;
            socket_connected = false;
            break;
          }

          DEBUG_SERIAL_UART("[%ld] - Trying 192.168.4.%d \r\n", millis(), iPs[ip]);

#ifdef LOGGING_ON_MAX
          loggingS = String(airPKT.pktPilotID) + "," + String(millis()) + "," + "Trying 192.168.4." + String (LK8000iPAddress);
          upLoadtoUDP(loggingS, loggingIP, LOGGING_PORT);
#endif

          IPAddress LK8000iPAddress(192, 168, 4, iPs[ip]);

          //LK8000iPAddress=LK8000iPAddress(192, 168, 4, 1);


          bool lkConnect2 = false;

          DEBUG_SERIAL_UART("[%ld] - webTime 2 = %d \r\n", millis(), webTime);

          if (millis() > webTime)
          {
            lkConnect2 = LK8000Client.connect(LK8000iPAddress, lk8000TcpServerPort);
          }

          if (!lkConnect2)
          {
            DEBUG_SERIAL_UART("[%ld] - Connection failed on :- 192.168.4.%d \r\n", millis(), iPs[ip]);
#ifdef LOGGING_ON_MAX
            loggingS = String(airPKT.pktPilotID) + "," + String(millis()) + "," + "Connection failed on :-" + String (LK8000iPAddress);
            upLoadtoUDP(loggingS, loggingIP, LOGGING_PORT);
#endif
          }
          else
          {
            DEBUG_SERIAL_UART("[%ld] - *** Connection to Flight Software on IP :- 192.168.4.%d *** \r\n", millis(), iPs[ip]);
            LK8000Client.setNoDelay(true);

#ifdef LOGGING_ON
            loggingS = String(airPKT.pktPilotID) + "," + String(millis()) + "," + "Connected to LK TCP Server";
            upLoadtoUDP(loggingS, loggingIP, LOGGING_PORT);
#endif
            LK8000Client.setTimeout(1);
            socket_connected = true;
            last_data_received = millis();
            apFoundIP = LK8000iPAddress;
            ipFound = true;
            break;
          }
          Web_loop();
          yield();
          myDelay(100);
        }
        Web_loop();
        yield();
        myDelay(100);
      }
    }
 ESP32   */

    DEBUG_SERIAL_UART_MAX("[%ld] - Checking for data from TCP\r\n", millis());

    if ( socket_connected)
    {
      looking_for_socket_time = millis();



      DEBUG_SERIAL_UART_MAX("[%ld] - Socket Connected\r\n", millis());

     // char c;
      String s;
     // bool lineend = false;
      bool fullLineReturned=false;
//v1.13
      if ( airwhere_client.connected() || wiredDirectly=='y' || wiredDirectly=='o' || wiredDirectly == 'v' || wiredDirectly == 'f' || ( wiredDirectly == 's' && socket_connected) || ( wiredDirectly == 'n' && socket_connected) )
      {

    //	  Serial.println("int ophere wired directly=s");
        if (TCPBuffer.length() > TCP_BUFFER_MAX_LENGTH)
        {
          // bad slow down somewhere, so clear out the buffer and hopefully it will clear it self up.
          DEBUG_SERIAL_UART("[%ld] - +++++++++++++++++++FLUSHING TCP BUFFER++LENGTH IS %d ++++++++++++\r\n", millis(), TCPBuffer.length());
          flush_time_counter++;
          TCPBuffer = "";


        }

        //Serial.printf( "3a. - current free heap: %d, minimum ever free heap: %d\n", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());

        DEBUG_SERIAL_UART_MAX("[%ld] - Before readAllDataOnWifi \r\n", millis());

        if ( wiredDirectly=='n' || wiredDirectly == 's')
        {
          if ( wiredDirectly == 's' )
          {
           // TCPBuffer = readAllDataOnWifiTxSerial(airwhere_client, TCPBuffer, fullLineReturned);
        	 // ( bool device_connected, IPAddress connected_ip, HardwareSerial data_in,  String TCPBuffer, bool &fullLine )
        	  TCPBuffer = readAllDataOnSerialTXWifiU(number_client,socket_connected, socket_ip, GPSSerial, TCPBuffer, fullLineReturned);
          }
          else
          {
        	//  Serial.println("ireadAllDataOnWifi=n");
           // TCPBuffer = readAllDataOnWifi(airwhere_client, TCPBuffer, fullLineReturned);
        	  TCPBuffer = readAllDataOnWifi(TCPBuffer, fullLineReturned);
          }

         // add for wifi tx version
        //  TCPBuffer = readAllDataOnWifiTxSerial(LK8000Client, TCPBuffer, fullLineReturned);

        }
        else
        {
        	/*char c;
        	  while(true)
        	  {
        	  if (GPSSerial.available() > 0) {
        	          // read the incoming byte:
        	          c = GPSSerial.read();
        	          Serial.println("readalldata");
        	          Serial.println(c);
        	  }
        	  }
*/
          // read serial or softserial
           if ( wiredDirectly == 'y' )
           {
        	//   Serial.println("reading gps ****************");
             TCPBuffer = readAllDataOnSerial(GPSSerial, TCPBuffer, fullLineReturned);
           }
           else
           {//Serial.println("reading serial ****************");
        	 TCPBuffer = readAllDataOnSerial(Serial, TCPBuffer, fullLineReturned);
           }
        }


      //  Serial.printf( "3b. - current free heap: %d, minimum ever free heap: %d\n", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());

        DEBUG_SERIAL_UART_S("[" + String(millis()) + "] - TCPBuffer length is :>" + TCPBuffer.length() + "\r\n");
        DEBUG_SERIAL_UART_S("[" + String(millis()) + "] - TCPBuffer >>" + TCPBuffer + "\r\n");
        DEBUG_SERIAL_UART_MAX("[%ld] - After readAllDataOnWifi \r\n", millis());

        //if the socket is connected and we havent receieved data for 60 seconds then sometimes not very well, restart wifi
        // its high at 60 as the kobo doesnt realise its fallen off the AP and will just sit there error transmitting
        // usually rejoins after about 20-30 secs so hopefully it will be long enough

        if ( socket_connected && last_data_received + 30000 < millis())
        {
          DEBUG_SERIAL_UART("[%ld] - AP connected:Socket connected - no data incoming - Restarting Access Point \r\n", millis());
        //  airwhere_client.stop();

        //  Serial.println("AP connected:Socket connected - no data incoming - Restarting Access Point \r\n");
          yield();
          myDelay(200);
          ESP_LOGI(TAG, "Prepare to restart system!");
         // esp_restart();

          udp_flight_software.stop();
          last_data_received=millis()+5000;
          myDelay(200);


         udp_flight_software.begin(UDP_FLIGHT_SOFTWARE_PORT);

        }

        //int nextDollarPos = 0;
        int newLine = 0;
      //  newLine = TCPBuffer.indexOf("\n");

        if (TCPBuffer.indexOf("\n")>0)
        {
        	newLine=TCPBuffer.indexOf("\n");
        }
        else
        {
            if (TCPBuffer.indexOf("\r")>0)
            {
           	  newLine=TCPBuffer.indexOf("\r");
            }
        }

        bool validSoundLine=false;

        DEBUG_SERIAL_UART_MAX("[%ld] - Before TCPBuffer.substring(0, 6) == \"$GPGGA\"  \r\n", millis());

  /*    if ( TCPBuffer.length()>0)
        {
       Serial.println("Start of test");
        Serial.println(TCPBuffer);
        if (fullLineReturned){ Serial.println("fullLineReturned");}else{Serial.println("fullLineReturned NOT returned");}
        delay(1000);
        }
*/

       // Serial.printf( "3c. - current free heap: %d, minimum ever free heap: %d\n", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());

        if ((TCPBuffer.substring(0, 6) == "$GPGGA" || TCPBuffer.substring(0, 6) == "$GNGGA") && fullLineReturned)
        {

     //   	Serial.println("INTO GPGGA ********************************");


          String currentNMEALine = TCPBuffer.substring(0, newLine);
          cnl = currentNMEALine;
          tcp_rx_weblines_counter++;

          DEBUG_SERIAL_UART_MAX("[%ld] - Into $GPGGA  \r\n", millis());
          climbRate = GPGGAdataIn.getPilotAltitude();

          GPGGAdataIn.rePopulate(currentNMEALine);


          if (debug_level>0)
          {
          	if(GPGGAdataIn.isLineValid)
          	{
          		Serial.println("\nGPGGAdataIn is VALID");

          	}
          	else
          	{
          		Serial.println("\nGPGGAdataIn is NOT VALID");
          	}
          }




         // Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>"+currentNMEALine);


          //Serial.println("GPGGA:"+TCPBuffer);
          // GPGGAdataIn.rePopulate(TCPBuffer); - REMOVED



          climbRate = climbRate - GPGGAdataIn.getPilotAltitude();
          GPGGAdataReceived = true;


          if (ble_output == 'e' && deviceConnected)
            {
          	//  BLESendChunks(oSout);

          	  // NEEDS TO ADD A SYSTEM TO SEND FROM THE $ if congested.

          	  if (ble_data.length()<512)
          	  {
          		  ////Serial.print("BLE-IN>>>>>");
          		  //	    Serial.print(ble_data);
          		  //	    Serial.println("<<<<<BLE-IN");


          		  //	  Serial.print("oSo-IN>>>>>");
          		  	//  		  	    Serial.print(oSout);
          		  	//  		  	    Serial.println("<<<<<oSo-IN");
                  while(ble_mutex){};
          	    ble_data=ble_data+currentNMEALine+"\n";

          	   if ( (millis()-pflauSendTimeble) > pflauCounterble )
          	   {
          		 pflauCounterble=millis();
          	    ble_data=ble_data+"$PFLAU,6,1,2,1,0,144,0,235,446*55\n";
          	   }


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


          // v1.11
          //if ( wiredDirectly == 'n' || wiredDirectly == 's' )
          if ( wiredDirectly == 'n')
          {

            if (navSofware == 'L')
            {
              DEBUG_SERIAL_UART_S("[" + String(millis()) + "] - Uploading GPPGA data to LK >>" + currentNMEALine + "\r\n");

              tcp_tx_weblines_counter++;



              writeDataToWifi( airwhere_client, GPGGAdataIn.getDataWithoutCksum());
            //DEBUG_SERIAL_UART_S("[" + String(millis()) + "] - Uploaded GPPGA data to LK >>" + currentNMEALine + "\r\n");
              tcp_tx_weblines_counter++;
              writeDataToWifi( airwhere_client, "$PFLAU,6,1,2,1,0,144,0,235,446*");
            //   DEBUG_SERIAL_UART_S("["+String(millis()) + "] - Uploaded PFLAU data to LK >>" + currentNMEALine + "\r\n");

            }
            //v1.1
            if (navSofware == 'X' || navSofware == 'A' || navSofware == 'K' )
            {

              DEBUG_SERIAL_UART_S("[" + String(millis()) + "] - Uploading GPPGA data to XCS >> >>" + currentNMEALine + "\r\n");

              sendUDPPacket ( GPGGAdataIn.getDataWithoutCksum(), apFoundIP, true  );
              tcp_tx_weblines_counter++;
              sendUDPPacket ( "$PFLAU,6,1,2,1,0,144,0,235,446*", apFoundIP, true  );
              tcp_tx_weblines_counter++;
            }

          }

          TCPBuffer = TCPBuffer.substring(newLine + 1);
        }
        else
        {
          if ((TCPBuffer.substring(0, 6) == "$GPRMC" || TCPBuffer.substring(0, 6) == "$GNRMC") && fullLineReturned)
          {
        	//  Serial.println("INTO $GPRMC ********************************");

            DEBUG_SERIAL_UART_MAX("[%ld] - Into $GPRMC  \r\n", millis());
            tcp_rx_weblines_counter++;
            if (TCPBuffer.indexOf("\n")>0)
            {
            	newLine=TCPBuffer.indexOf("\n");
            }
            else
            {
                if (TCPBuffer.indexOf("\r")>0)
                {
               	  newLine=TCPBuffer.indexOf("\r");
                }
            }
            String currentNMEALine = TCPBuffer.substring(0, newLine);
            GPRMCdataIn.rePopulate(currentNMEALine);


            if (ble_output == 'e' && deviceConnected)
                {
              	//  BLESendChunks(oSout);

              	  // NEEDS TO ADD A SYSTEM TO SEND FROM THE $ if congested.

              	  if (ble_data.length()<512)
              	  {
              		  ////Serial.print("BLE-IN>>>>>");
              		  //	    Serial.print(ble_data);
              		  //	    Serial.println("<<<<<BLE-IN");


              		  //	  Serial.print("oSo-IN>>>>>");
              		  	//  		  	    Serial.print(oSout);
              		  	//  		  	    Serial.println("<<<<<oSo-IN");
                      while(ble_mutex){};
              	    ble_data=ble_data+currentNMEALine+"\n";


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

            if (debug_level>0)
            {
            	if(GPRMCdataIn.isLineValid)
            	{
            		Serial.println("\nGPRMCdataIn is VALID");

            	}
            	else
            	{
            		Serial.println("\nGPRMCdataIn is NOT VALID");
            	}
            }

          //  Serial.printf( "3d. - current free heap: %d, minimum ever free heap: %d\n", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());

            //v1.1
            if (lora_or_flarm=='l' || lora_or_flarm=='r' ||  lora_or_flarm=='h' || lora_or_flarm=='t' || lora_or_flarm=='b')
            {
                app.set_gps_lock(GPRMCdataIn.getGpsActive());
            }
            else
            {
                flarm.set_gps_lock(GPRMCdataIn.getGpsActive());
            }
            GPRMCdataReceived = true;
            TCPBuffer = TCPBuffer.substring(newLine + 1);

          }
          else
          { // bit of a bodge to do sounds here - change when we get proper time.
        	  if ( TCPBuffer.substring(0,9)=="$GFPALARM"  && fullLineReturned)
        	  {

        		  String currS=TCPBuffer.substring(10);
        		  //v1.12

        		  //passthru for sounds for BFV - needs checking
        		  GPSSerial.println(TCPBuffer.substring(0, newLine ));

        		  currS=currS.substring(0, currS.indexOf('*'));
        		  currS.trim();

        		  if (isItNumeric(currS))
        		  {
        			  int toneNumber=TCPBuffer.substring(9).toInt();
        			  validSoundLine=true;
        			  toneNumber=(toneNumber*GFPALARM_SCALE)+GFPALARM_BASE_FREQUENCY;

        			  tuneIs[0][0]=toneNumber;
        			  tuneIs[0][1]=GFPALARM_LENGTH+millis();
        			  // start playing the first.
        			  // tone(speakerPin, tuneIs[0][0]);

        			  // set switch to stop vario for the sounds.
        			  //   Serial.println("Switching airwhere sounds on");
        			  airwhere_tones_on=true;
        			  ledcWriteTone(SPEAKER_CHANNEL, tuneIs[0][0]);
        			  currentTone=0;
        			  noTones=0;
        		  }
        		  else
        		  {
        			  validSoundLine=false;
        		  }
        		  TCPBuffer = TCPBuffer.substring(newLine + 1);
        	  }
        	  else
        	  {
        		  if (TCPBuffer.substring(0,4)=="$BSD" && fullLineReturned)
        		  {

        			  String bsdLine=TCPBuffer.substring(5);
//v1.12
        			  GPSSerial.println(TCPBuffer);

        			  bsdLine=bsdLine.substring(0, bsdLine.indexOf('*'));

        			  int currentChar=0;

        			  int tuneLength=0;
        			  int maxTones=10;

        			  int toneReading=0;
        			  int spaceIndex=0;

        			  validSoundLine=true;

        			  while( spaceIndex>=0 && currentTone<maxTones && validSoundLine)
        			  {

        				  spaceIndex=bsdLine.indexOf(" ",currentChar);

        				  String currS=bsdLine.substring(currentChar,spaceIndex);
        				  currS.trim();

        				  if (isItNumeric(currS))
        				  {
        					  // time to play tone
        					  if ( toneReading == 1 )
        					  {
        						  tuneIs[currentTone][toneReading]=tuneLength+currS.toInt()+millis();
        						  tuneLength=tuneLength+currS.toInt();
        					  }
        					  else
        					  {
        						  tuneIs[currentTone][toneReading]=currS.toInt();
        					  }

        					  currentChar=spaceIndex+1;

        					  if (toneReading==0)
        					  {
        						  toneReading=1;
        						  noTones++;
        					  }
        					  else
        					  {
        						  toneReading=0;
        						  currentTone++;
        					  }
        				  }
        				  else
        				  {
        					  validSoundLine=false;
        				  }
        			  }
        			  TCPBuffer = TCPBuffer.substring(newLine + 1);
        		  }
        		  else
        		  {
        			  // get rid of another $ or the line is empty
        			  DEBUG_SERIAL_UART_MAX("[%ld] - Into Empty  \r\n", millis());
        		//	  TCPBuffer = TCPBuffer.substring(newLine + 1);

        			  // only remove the next line if a full line is returned !
        			  //v1.1
        			  if (fullLineReturned)
        			  {
        				  //         Serial.println("\nRemoving a line no line found");
        				  TCPBuffer = TCPBuffer.substring(newLine + 1);
        			  }
        		  } }
          }
        }

        if (validSoundLine)
        {
        // start playing first tone and then check at top of loop for the next tone to change to.
         // tone(speakerPin, tuneIs[0][0]);
     	   // set switch to stop vario for the sounds.
          // Serial.println("Switching airwhere sounds on - BSD");
           airwhere_tones_on=true;

           ledcWriteTone(SPEAKER_CHANNEL, tuneIs[0][0]);
          currentTone=0;
        // playing the first.
          noTones--;
        }
        /*   int iAt = TCPBuffer.indexOf('$');

           if (iAt == -1)
           {
             TCPBuffer = "";
           }
           else
           {
             TCPBuffer = TCPBuffer.substring(iAt);
           }

        */
      }
      else
      {





        DEBUG_SERIAL_UART("[%ld] - Client Disconnected -  Trying to reconnect\r\n", millis());


        socket_connected = false;
        GPRMCdataReceived = false;
      }
    }

    yield();

    // the esp32 doesnt handle interrupts properly yet, it falls over so we just set a flag in the interrupt
    // and check it and handle it here instead in the main loop - as its looping 200 times + a second
    // 99% of the time it catches the received packet  - revisit when interrupts are ok.

    // check irq again before doing RF.
    // this irq handling only requires for the sx1272/sx1276 chip as it falls over in checking for irq.
    //v1.1
    if (lora_or_flarm!='f')
    {
  	  app.handle_irq(lora_or_flarm);
    }

    DEBUG_SERIAL_UART_MAX("[%ld] - Into RF Section\r\n", millis());
    /*
      RF SECTION

      We have a full set of GPS data to upload to the Air if the GPS is Active

    */

  //  if (GPGGAdataIn.isLineValid) {Serial.print("GPGGAdataIn LineValid");}else{Serial.println("GPGGAdataIn.NOT isLineValid");}
 //   if (GPRMCdataIn.isLineValid) {Serial.print("GPRMCdataIn LineValid");}else{Serial.println("GPRMCdataIn.NOT isLineValid");}
 //   Serial.printf( "4. - current free heap: %d, minimum ever free heap: %d\n", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());


    if (GPRMCdataReceived && GPRMCdataIn.getGpsActive() == 'A' && GPGGAdataIn.isLineValid && GPRMCdataIn.isLineValid)
    {

      DEBUG_SERIAL_UART_MAX("[%ld] - Starting Sending of Serial if data received - GPS Active\r\n", millis());

   /*   airPKT.pktLatitude = GPGGAdataIn.getPilotLatitudeNoDot().toInt();
      airPKT.pktLongitude = GPGGAdataIn.getPilotLongitudeNoDot().toInt();
      airPKT.pktNSEW = isPilotNESW(GPGGAdataIn.getPilotNS(), GPGGAdataIn.getPilotEW() );
      airPKT.pktAltitude = GPGGAdataIn.getPilotAltitude();
      airPKT.pktTrack = GPRMCdataIn.getPilotTrack();
      airPKT.pktSpeed = GPRMCdataIn.getPilotSpeed();
      airPKT.pktCrc = 36;
*/
       float lat=GPGGAdataIn.getPilotLatitude().toFloat();
       if (GPGGAdataIn.getPilotNS()=='S' )
       {
         lat=lat*-1;
       }

       float lon=GPGGAdataIn.getPilotLongitude().toFloat();

       if (GPGGAdataIn.getPilotEW()=='W' )
       {
         lon=lon*-1;
       }

      // Serial1.print("GPGGAdataIn.getPilotLongitude() - ");Serial1.println(GPGGAdataIn.getPilotLongitude());
     //  Serial.print("lon - ");Serial.println(lon);
     //  Serial.print("lat - ");Serial.println(lat);

       ///v1.1
       if (lora_or_flarm=='l' || lora_or_flarm=='r' || lora_or_flarm=='h' || lora_or_flarm=='t' || lora_or_flarm=='b')
       {
           app.set( lat,
                   lon,
                   float(GPGGAdataIn.getPilotAltitude()),
                   float(GPRMCdataIn.getPilotSpeed()),
                   0, float(GPRMCdataIn.getPilotTrack()), 0,
				   GPGGAdataIn.lat_sign_negative,GPGGAdataIn.lat_deg,GPGGAdataIn.lat_min, GPGGAdataIn.lat_min_frac,
				   GPGGAdataIn.lon_sign_negative,GPGGAdataIn.lon_deg, GPGGAdataIn.lon_min, GPGGAdataIn.lon_min_frac);

       }
       else
       {
           //v16
           //update flarm app - this will know when to send packet.
            flarm.set( GPGGAdataIn.getFixTime(), GPRMCdataIn.getPilotDate() ,lat, lon,  float(GPGGAdataIn.getPilotAltitude()),
                   float(GPRMCdataIn.getPilotSpeed()), 0, float(GPRMCdataIn.getPilotTrack()));

           // blink a led here off another pin to show transmission.
       }





      String urlUp;

      //v1.15

      char manuMe[3];
      char idMe[5];

      sprintf(manuMe,"%02X",fmac.my_addr.manufacturer);
      sprintf(idMe,"%04X",fmac.my_addr.id);

      if (app.do_online_tracking && webUpload)
      {

    	// on wifi  - dest and src - same place, rssi = 0;
        // id is manufacturer + ID .

        urlUp = String ( GPGGAdataIn.getFixTime() ) + "," +
                String (manuMe) + String (idMe)  + "," +
                String (manuMe) + String (idMe) + "," +
				app.aircraft_type + "," +
                GPGGAdataIn.getPilotLatitude() + "," +
				GPGGAdataIn.getPilotLongitude() + "," +
                GPGGAdataIn.getPilotNS() + "," +
				GPGGAdataIn.getPilotEW() + "," +
			    GPRMCdataIn.getPilotTrack() + "," +
				GPRMCdataIn.getPilotSpeed()  + "," +
				GPGGAdataIn.getPilotAltitude() + "," +
				0;

    //    Serial.println("");
    //    Serial.println(urlUp);
     //   Serial.println("");
        web_upload_counter++;
       upLoadtoUDP(urlUp, airwhere_web_ip, AIRWHERE_UDP_PORT  );
      }
//v1.14
      if (flarm.do_online_tracking  && webUpload)
      {


        // on wifi  - dest and src - same place, rssi = 0;
        // id is manufacturer + ID .

        urlUp = String ( GPGGAdataIn.getFixTime().toInt() ) + "," +
                String (manuMe) + String (idMe)  + "," +
                String (manuMe) + String (idMe) + "," +
                flarm.aircraft_type + "," +
                GPGGAdataIn.getPilotLatitude() + "," +
                GPGGAdataIn.getPilotLongitude() + "," +
                GPGGAdataIn.getPilotNS() + "," +
                GPGGAdataIn.getPilotEW() + "," +
			    GPRMCdataIn.getPilotTrack() + "," +
				GPRMCdataIn.getPilotSpeed()  + "," +
				GPGGAdataIn.getPilotAltitude() + "," +
                0;

        web_upload_counter++;
        upLoadtoUDP(urlUp, airwhere_web_ip, AIRWHERE_UDP_PORT  );
      }


/*
      unsigned char cs = 0;
      char crcCheck[23];

      memset(crcCheck, 0, 23);
      memcpy(&crcCheck, &airPKT, 23);

      cs = 0; //clear any old checksum

      for (unsigned int n = 0; n < 23; n++) {
        cs ^= crcCheck[n]; //calculates the checksum

      }

      airPKT.pktCRC = cs;



      memset(TxBuffer, 0, sizeof(airPKT));
      memcpy(&TxBuffer, &airPKT, sizeof(airPKT));

      DEBUG_SERIAL_UART("[%ld] - >>>>>>>>>>>>>>> Transmitting Pilot data On RF : Pilot ID %d\r\n", millis(), airPKT.pktPilotID);

#ifdef LOGGING_ON
      String dataUp = String(airPKT.pktPilotID) + "," +  airPKT.pktAircraftType + "," +
                      GPGGAdataIn.getPilotLatitude() + "," + GPGGAdataIn.getPilotLongitude() + "," +
                      GPGGAdataIn.getPilotNS() + "," + GPGGAdataIn.getPilotEW() + "," +
                      airPKT.pktTrack + "," + airPKT.pktSpeed  + "," + airPKT.pktAltitude;
      loggingS = String(airPKT.pktPilotID) + "," + String(millis()) + "," + "Transmitting Pilot data On RF : " + dataUp;
      //ESP32 upLoadtoUDP(loggingS, loggingIP, LOGGING_PORT);
#endif


       	byte addr[] = TXADDR;
        	nRF905_setTXAddress(addr);
        	nRF905_setData(data, NRF905_PAYLOAD_SIZE );
        	while(!nRF905_send()) { yield(); } ;
      */
   //   serial_tx_packets_counter++;

      //v1.1
	  if ( lora_or_flarm == 'h' || lora_or_flarm == 't' )
	  {
		  update_oled();
	  }

     // Serial.flush();
      //Serial.write(TxBuffer, PKT_SIZE);

     // int tx = sx1272.sendFrame((uint8_t *)str.c_str(), str.length()+1);
      rfPktToSend=true;
      spacerTime=millis()+ random(1, 750);

 /*     int tx = sx1272.sendFrame(TxBuffer, PKT_SIZE+1);

      if ( wiredDirectly == 'y' )
      {
        if ( tx==0 )
        {
          streamData=("$AWARE,TRANSMITTING-LOCATION\r\n"+streamData);
        }
        if ( tx==-1 )
        {
          streamData=("$AWARE,CANT TRANSMIT TX ONGOING\r\n"+streamData);
        }
        if ( tx==-2 )
        {
          streamData=("$AWARE,CANT TRANSMIT RX ONGOING\r\n"+streamData);
        }

      }
      */

      GPRMCdataReceived = false;
    //  GPGGAdataIn.isLineValid=false;
   //   GPRMCdataIn.isLineValid=false;
    }
    else
    {
      // Serial sometimes stops for some weird reason so after 3 mins just restart it to make it happy :)

      // NEEDS SORTING
      //  if ( (looking_for_socket_time+180000)<millis() && socket_connected)
      //  {
      //   DEBUG_SERIAL_UART("[%ld] - Client connected but not sending serial - SERIAL Failure\r\n", millis());
      //system_restart();
      //  Serial.end();
      //  Serial.begin(9600);
      // }


    }
  }
  else
  {
      char manuMe[3];
      char idMe[5];

      sprintf(manuMe,"%02X",fmac.my_addr.manufacturer);
      sprintf(idMe,"%04X",fmac.my_addr.id);

      if (!populatedGSdata)
      {
    	  GPGGAdataIn.rePopulateDecimal(airWhereID, gsLatitude, gsLongitude, gsAltitude );



          DEBUG_SERIAL_UART_S("[" + String(millis()) + "] - GPGGAdataIn.getPilotLatitudeNoDot > " + GPGGAdataIn.getPilotLatitudeNoDot() + "\r\n");
          DEBUG_SERIAL_UART_S("[" + String(millis()) + "] - GPGGAdataIn.getPilotLongitudeNoDot > " + GPGGAdataIn.getPilotLongitudeNoDot() + "\r\n");

          float fixLat = GPGGAdataIn.getPilotLatitude().toFloat();
          float fixLong = GPGGAdataIn.getPilotLongitude().toFloat();

          if ( gsLatitude[0]=='-' )
          {
            fixLat = fixLat * -1;
          }
          if (gsLongitude[0]=='-' )
          {
            fixLong = fixLong * -1;
          }

          //this needs adding for flarm board GS
        //  flarm.set( GPGGAdataIn.getFixTime(), GPRMCdataIn.getPilotDate() ,lat, lon,  float(GPGGAdataIn.getPilotAltitude()),
         //                   float(GPRMCdataIn.getPilotSpeed()), 0, float(GPRMCdataIn.getPilotTrack()));



          app.set( fixLat,
        		  fixLong,
    			  GPGGAdataIn.getPilotAltitude(),
    			  0, 0, 0, 0,
    			  0, 0, 0, 0,
    			  0, 0, 0, 0);


          GPRMCdataIn.setGpsActive();

          GPGGAdataIn.isLineValid=true;
          GPRMCdataIn.isLineValid=true;

          //v1.1
          GPGGAdataIn.setFixTime("000000");

          if (lora_or_flarm=='l' || lora_or_flarm=='r' ||  lora_or_flarm=='h' || lora_or_flarm=='t' || lora_or_flarm=='b')
          {
              app.set_gps_lock(GPRMCdataIn.getGpsActive());
          }
          else
          {
              flarm.set_gps_lock(GPRMCdataIn.getGpsActive());
          }
          if (debug_level>0)
          {
      	  Serial.println ("Populated GS data");
      	  delay(1000);
          }
      }


      if (!populatedGSdata )
      {
    	  populatedGSdata=true;
    	  if( WiFi.status() == WL_CONNECTED)
    	  {


    		  for ( int i=0;i<10;i++)
    		  {
    			  if (debug_level>0)
    			  {
    				  Serial.println ("SENDING KEEP ALIVE GS - Startup");
    				  delay(200);
    			  }

    			  String urlUp = ">GS Location," + String( manuMe ) + String( idMe ) + "," + String( gsLatitude ) + "," + String(gsLongitude) + "," + String(gsAltitude);
    			  upLoadtoUDP(urlUp, airwhere_web_ip, AIRWHERE_UDP_PORT);
    			  delay(100);
    		  }

    	  }

      }
      if (millis()>gsTimer)
      {
    	  if (debug_level>0)
    	  {
    		  Serial.println ("SENDING KEEP ALIVE GS");
    	  }

    	  WiFiClient clientAWV;
    	  //   const int httpPort = 80;
    	  if (clientAWV.connect(airWareServer, 80) && WiFi.status() == WL_CONNECTED)
    	  {
    		  // Serial.println("server Found");
    		  String urlUp = ">GS Location," + String( manuMe ) + String( idMe ) + "," + String( gsLatitude ) + "," + String(gsLongitude) + "," + String(gsAltitude);
    		  upLoadtoUDP(urlUp, airwhere_web_ip, AIRWHERE_UDP_PORT);
    		  gsTimer=millis()+GSTIMERINTERVAL;

    	  }
    	  else
    	  {
    		  Serial.println("AirWhere Ground Station - Internet Failed - Can not find server - Restarting System");
    		  yield();
    		  myDelay(500);
    		  ESP_LOGI(TAG, "Prepare to restart system!");
    		  esp_restart();
    	  }

      }

  }

//*********************************************************************
//*******************  Receive Section **************************
//*********************************************************************


 // Serial.printf( "5. - current free heap: %d, minimum ever free heap: %d\n", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());




 // uint8_t data[128] = {0};
 // int rx = sx1272.getFrame(data, sizeof(data));
  //int rx =0;
 /* if ( serialLen > 23 )
  {
    while ( Serial.peek() != '$' && serialLen > 23)
    {
      DEBUG_SERIAL_UART("[%ld] - Reading a byte - %c - Serial Available - %d \r\n", millis(), Serial.peek(), serialLen);
      byte b[1];
      Serial.readBytes(b, 1);
      serialLen = Serial.available();
    }
    // if seriallen is less than 24, we have got some data without a $ so run around and get some more.
    if ( serialLen < 24)
    {
      success = false;
      DEBUG_SERIAL_UART_MAX("[%ld] - Data has come in but we cant find a $ \r\n", millis());
    }
    else
    {
      DEBUG_SERIAL_UART("[%ld] - Serial Packet - Serial Length - %d \r\n", millis(), Serial.available());
      serial_rx_packets_counter++;
      Serial.readBytes(RxBuffer, PKT_SIZE);

      DEBUG_SERIAL_UART("[%ld] - Received some ARF data :- %d\r\n", millis(), airPKTRX.pktPilotID);

#ifdef LOGGING_ON_MAX
      loggingS = String(airPKT.pktPilotID) + "," + String(millis()) + "," + "Received some ARF data";
      upLoadtoUDP(loggingS, loggingIP, LOGGING_PORT);
#endif
   if (lora_rx )
    {
      serial_rx_packets_counter++;

      // hopefully by here we have a packet that starts with a $, so it should be one of ours, but check data is ok before proceeding.
      if (validatePKT(loraData[0], PKT_SIZE ))
      {
        memset(&airPKTRX, 0, sizeof(airPKT));
        memcpy(&airPKTRX, &loraData[0], sizeof(airPKT));

        success = true;
        DEBUG_SERIAL_UART_MAX("[%ld] - Incoming 24 bytes is one of our Packets, processing Pilot :- %d\r\n", millis(), airPKTRX.pktPilotID);
      }
      else
      {
       success = false;
        DEBUG_SERIAL_UART("[%ld] - **** FOUND BAD PACKET - Skipping ***** :- pilotID %d , CRC %d\r\n", millis(), airPKTRX.pktPilotID, airPKTRX.pktCrc );
      }
    loraData[0][RF_PACKET_SIZE] = {0};
    lora_rx=false;
  }
#ifdef TEST_PILOT
  else
  {
    DEBUG_SERIAL_UART_MAX("[%ld] - Setting A Test Pilot\r\n", millis());
    int randomNo = random(1, 30);
    int PId = random(0, 10) * 100000;
    int loc = random(-100000, 100000);
    int loc1 = random(-100000, 100000);

    if  (randomNo == 15)
    {


      airPKTRX.pktPilotID = PId + 12345;
      airPKTRX.pktCrc = 36;

      airPKTRX.pktLatitude = GPGGAdataIn.getPilotLatitudeNoDot().toInt() + loc;
      airPKTRX.pktLongitude = GPGGAdataIn.getPilotLongitudeNoDot().toInt() + loc1;
      airPKTRX.pktAltitude = random(1, 1000);;
      airPKTRX.pktNSEW = 1;
      unsigned int tr = random(0, 360);
      airPKTRX.pktTrack = tr;
      DEBUG_SERIAL_UART_MAX("[%ld] - Adding Test Pilot %d\r\n", millis(), airPKTRX.pktPilotID);
      //success=true;
      // Serial.println("adding");
      unsigned char cs = 0;

      char crcCheck[23];

      memset(crcCheck, 0, 23);
      memcpy(&crcCheck, &airPKTRX, 23);

      cs = 0; //clear any old checksum

      for (unsigned int n = 0; n < 23; n++) {
        cs ^= crcCheck[n]; //calculates the checksum
      }

      airPKTRX.pktCRC = cs;

      memset(TxBuffer, 0, sizeof(airPKT));
      memcpy(&TxBuffer, &airPKTRX, sizeof(airPKT));
      DEBUG_SERIAL_UART_MAX("[%ld] - Writing Test pilot to Serial %d\r\n", millis(), airPKTRX.pktPilotID);
      Serial.flush();
      Serial.write(TxBuffer, PKT_SIZE);
      success = true;
    }
    else
    {
      success = false;
    }
  }
#endif

  yield();
*/
//*********************************************************************
//******************* nrf905 Receive Section **************************
//*********************************************************************
//***** We need to priorise the Nmea strings coming in and serial *****
//***** If we have serial packet to parse, sort that first, use *******
//***** loops with nothing on serial to process nrf905 packets ********
//***** as this wont halt or cause any problem waiting for them *******
#ifdef ESP32_CPU

#else


if ( receiveFlarm=='y')
{

  if ( !success )
  {
    nRF905_receive();
 //   unsigned long sendStartTime = millis();

    airPKTRX.pktCrc = 0;


    success = nRF905_getData(RxBuffer, sizeof(RxBuffer));
    if (success) // Got data
    {
      nrf_rx_packets_counter++;
      //char *q;

      fo.raw = Bin2Hex(RxBuffer);

DEBUG_SERIAL_UART("[%ld] - Received FLarm packet - Processing\r\n", millis());

      if (webUpload) {String urlUp = "Received FLarm packet";upLoadtoUDP(urlUp, airwhere_web_ip, AIRWHERE_UDP_PORT);}



    //fill airPKTRX with flarm data - need to go capture some flarm data and check it out :)


      flarm_decodeAW(airPKTRX,
                   (flarm_packet *) RxBuffer,
                   LATITUDE, LONGTITUDE, ALTITUDE,
                   0,
                   0,
                   0
                  );

      airPKTRX.pktCrc = 36;

      success = true;
    }
  }

}
#endif
  yield();


  // Received some RF data - Upload to LK.


  DEBUG_SERIAL_UART_MAX("[%ld] - Processing received packet\r\n", millis());
  // DEBUG_SERIAL_UART("[%ld] - airPKTRX.pktCrc %d\r\n", millis(), airPKTRX.pktCrc);

//*********************************************************************
//******************* Process Received Packets Section ****************
//*********************************************************************
  for ( int packetNumber=0; packetNumber <  RX_PACKETS; packetNumber++)
  {

	  // only process the packet when we have a valid fix.

	/*  if (GPGGAdataIn.isLineValid)
	  {
		  Serial.println("GPGGAdataIn.isLineValid");
	  }
	  else
	  {
		  Serial.println("NOT GPGGAdataIn.isLineValid");
	  }

	  if (GPRMCdataIn.isLineValid)
	  {
		  Serial.println("GPRMCdataIn.isLineValid");
	  }
	  else
	  {
		  Serial.println("NOT GPRMCdataIn.isLineValid");
	  }

	  */
	  //v2
	  if (payloadReady[packetNumber]>0 && millis()>payload_time_to_send[packetNumber] && GPGGAdataIn.isLineValid && GPRMCdataIn.isLineValid)
	  //if (payloadReady[packetNumber] && GPGGAdataIn.isLineValid && GPRMCdataIn.isLineValid)
	  {
	//	Serial.println("Sorting ");
	//	Serial.println(packetNumber);
       serial_rx_packets_counter++;




      float fixLat = GPGGAdataIn.getPilotLatitude().toFloat();
      float fixLong = GPGGAdataIn.getPilotLongitude().toFloat();

      if ( GPGGAdataIn.getPilotNS() == 'S')
      {
        fixLat = fixLat * -1;
      }
      if ( GPGGAdataIn.getPilotEW() == 'W')
      {
        fixLong = fixLong * -1;
      }


   //   Serial.print("fixLat");Serial.println(fixLat);
   ////   Serial.print("fixLong");Serial.println(fixLong);
   //   Serial.print("payloadList[packetNumber].latitude");Serial.println(payloadList[packetNumber].latitude);
   //   Serial.print("payloadList[packetNumber].longitude");Serial.println(payloadList[packetNumber].longitude);

      float pilotBearing = CalcBearingA( fixLat, fixLong, payloadList[packetNumber].latitude, payloadList[packetNumber].longitude);
      float pilotDistance = distance( fixLat, fixLong, payloadList[packetNumber].latitude, payloadList[packetNumber].longitude, 'K') ;
      float relNorth=0;
	  float relEast=0;

      float rads = deg2rad(pilotBearing);

      relEast = sin(rads) * pilotDistance * 1000;
      relNorth = cos(rads) * pilotDistance * 1000;

     // double speedKM = airPKTRX.pktSpeed * 0.5144;

      float relVert = payloadList[packetNumber].altitude - GPGGAdataIn.getPilotAltitude();

     // char moving_pilot_data[50]={};

     /* snprintf ( moving_pilot_data, 50, "$PFLAA,0,%s,%s,%s,2,%d%d,%s,0,%s,%d,%d*",
    		    relNorth,relEast,relVert,payloadList[packetNumber].manufacturer,payloadList[packetNumber].id,
				payloadList[packetNumber].heading,payloadList[packetNumber].speed,
				climbRate,payloadList[packetNumber].aircraft_type);

      Serial1.println(moving_pilot_data);*/

      float currentSpeed = payloadList[packetNumber].speed/KMPH_TO_MS;

     //PFLAA,<AlarmLevel>,<RelativeNorth>,<RelativeEast>,<RelativeVertical>,<IDType>,<ID>,<Track>,<TurnRate>,<GroundSpeed>,<ClimbRate>,<AcftType>
     // 0 = unknown1 = glider / motor glider2 = tow / tug plane3 = helicopter / rotorcraft4 = skydiver5 = drop plane for skydivers6 = hang glider (hard)7 = paraglider (soft)8 = aircraft with reciprocating engine(s)9 = aircraft with jet/turboprop engine(s)A =unknownB = balloonC = airshipD = unmanned aerial vehicle (UAV)E = unknownF = static objec

      char flarm_aircraft_type='0';

      switch (payloadList[packetNumber].aircraft_type)
      {
        case '1':
        	flarm_aircraft_type='7';
        break;
        case '2':
        	flarm_aircraft_type='6';
        break;
        case '3':
        	flarm_aircraft_type='B';
        break;
        case '4':
        	flarm_aircraft_type='1';
        break;
      }

      char manu[3];
      char id[5];

      sprintf(manu,"%02X",payloadList[packetNumber].manufacturer);
      sprintf(id,"%04X",payloadList[packetNumber].id);

      String movingpilotData = "$PFLAA,0," + String(relNorth) + "," + String(relEast) + "," + String(relVert) + ",2," +
    		                    String(manu) + String(id) + "," + payloadList[packetNumber].heading + ",0,"  +
								 currentSpeed + "," + climbRate + ","+ flarm_aircraft_type + "*";

      if (debug_level>0)
      {
        Serial.println("Sending $PFLAA " + String(manu) + String(id));
      }


      //v1.1 update OLED

	  if ( lora_or_flarm == 'h' || lora_or_flarm=='t')
	  {
	    update_oled();
	  }

DEBUG_SERIAL_UART_MAX("[%ld] - Crash Marker 4\r\n", millis());



      tcp_tx_weblines_counter++;

    //v1.11  if (wiredDirectly=='n' || wiredDirectly == 's')

      if (ble_output == 'e' && deviceConnected)
          {
        	//  BLESendChunks(oSout);

        	  // NEEDS TO ADD A SYSTEM TO SEND FROM THE $ if congested.

        	  if (ble_data.length()<512)
        	  {
        		  ////Serial.print("BLE-IN>>>>>");
        		  //	    Serial.print(ble_data);
        		  //	    Serial.println("<<<<<BLE-IN");


        		  //	  Serial.print("oSo-IN>>>>>");
        		  	//  		  	    Serial.print(oSout);
        		  	//  		  	    Serial.println("<<<<<oSo-IN");
                while(ble_mutex){};
                ble_data = add_to_ble_data_cksum( ble_data, movingpilotData );


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

      if (wiredDirectly=='n')
      {


        if (navSofware == 'L' && groundStationMode =='n')
        {
          DEBUG_SERIAL_UART_S("[" + String(millis()) + "] - Uploading RF Pilot to LK > " + movingpilotData + "\r\n");
          writeDataToWifi( airwhere_client, movingpilotData);
        }
        //v1.1
        if ((navSofware == 'X' || navSofware == 'A' || navSofware == 'K' ) && groundStationMode =='n')
        {
          DEBUG_SERIAL_UART_S("[" + String(millis()) + "] - Uploading RF Pilot to XCS > " + movingpilotData + "\r\n");

        //  Serial.println("Uploading RF Pilot to XCS > " + movingpilotData);

          sendUDPPacket ( movingpilotData, apFoundIP , true );
        }

      }
      else
      {
         if ( groundStationMode =='n')
         {
             writeDataToSerial(movingpilotData);

      //     Serial.print(">");
      //     Serial.print(movingpilotData);
      //     Serial.println("<");
         }
      }


      char stringlat[13];
      char stringlon[14];

      dtostrf(payloadList[packetNumber].latitude,12, 6, stringlat);
      dtostrf(payloadList[packetNumber].longitude,13, 6, stringlon);

      String sla=String (stringlat);
      String slo=String (stringlon);
      sla.trim();
      slo.trim();

      DEBUG_SERIAL_UART_MAX("[%ld] - Crash Marker 2\r\n", millis());

      char manuMe[3];
      char idMe[5];

      sprintf(manuMe,"%02X",fmac.my_addr.manufacturer);
      sprintf(idMe,"%04X",fmac.my_addr.id);

      String urlUp = String ( GPGGAdataIn.getFixTime() ) + "," +
    		  String ( manuMe ) + String ( idMe ) + "," +
			  String ( manu ) + String ( id ) + "," +
			  payloadList[packetNumber].aircraft_type + "," +
			  sla + "," + slo + "," +
			  "0" + "," + "0" + "," +
			  payloadList[packetNumber].heading + "," +
			  payloadList[packetNumber].speed  + "," +
			  payloadList[packetNumber].altitude + "," +
			  payloadList[packetNumber].rssi;

      if (payloadList[packetNumber].online_tracking && webUpload)
      {
          if (debug_level>0)
          {
            Serial.println("Uploading to web found pilot : " + urlUp);
          }
        upLoadtoUDP(urlUp, airwhere_web_ip, AIRWHERE_UDP_PORT);
      }

      if ( groundStationMode =='y')
      {
        Serial.println(urlUp);
  	    if ( lora_or_flarm == 'h' || lora_or_flarm=='t')
  	    {
  	      update_oled();
  	    }
      }

      //v2
      if (packet_repeat=='y')
      {
    	  payload_time_to_send[packetNumber]=millis()+PACKET_REPEAT_FREQ;
    	  if (payloadReady[packetNumber]==0)
    	  {
    		  payloadList[packetNumber].id=0;
    	  }

      }
      payloadReady[packetNumber]=payloadReady[packetNumber]-1;

     // Serial.print("packet number cleared out");
      	// Serial.print(packetNumber);
      // finished sending, clear ID down.
       //payloadReady[packetNumber]=false;
  }
}

  //Serial.printf( "6. - current free heap: %d, minimum ever free heap: %d\n", xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());

  DEBUG_SERIAL_UART_MAX("[%ld] - Starting Webloop\r\n", millis());
  //2.8

  //v1.1
  //if  (wifiUp)
  //{
   // Web_loop();
  //}

  yield();

  #ifdef TEST_PILOT

  if ( millis() > addPilotTime)
  {

    pilotACurrentTrack=pilotACurrentTrack-pilotARadsDiff;
    pilotAX=pilotADistance*sin(pilotACurrentTrack);
    pilotAY=pilotADistance*cos(pilotACurrentTrack);

    addPilotTime=millis()+pilotInterval;
    pilotADistance=pilotADistance+5;

    if (pilotACurrentTrack<-6.2)
    {
      pilotACurrentTrack=0;
    }

    pilotAlt=pilotAlt+random(1,pilotAltDiff);

    String testPilotData = "$PFLAA,0," + String (-1*pilotAX) + "," + String (-1*pilotAY)+ "," + String (pilotAlt) + ",2," + String (pilotA) + "," + String (-1*(rtodA(pilotACurrentTrack+pilotARadsDiff) )) + ",0," + 25 + "," + 0 + ",7*";
    writeDataToSerial(testPilotData);
  }
  #endif

}
