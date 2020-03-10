//ESP32 #include <SoftwareSerial.h>
#include "timeFunctions.h"
#include "WiFi.h"

extern unsigned int last_data_received;

#ifdef ESP32_CPU
extern HardwareSerial Serial1;
#else
extern SoftwareSerial swSer;
#endif


extern int lastUdpPacketTime;
String readAllDataOnWifi( String TCPBuffer , bool &fullLine);
String readAllDataOnWifi(WiFiClient LK8000Client, String TCPBuffer, bool &fullLine );
String readAllDataOnWifiTxSerial( WiFiClient LK8000Client, String TCPBuffer, bool &fullLine );
String readAllDataOnSerialTXWifiU( int number_connected, bool device_connected, IPAddress connected_ip, HardwareSerial data_in,  String TCPBuffer, bool &fullLine );
void writeDataToWifi(WiFiClient LK8000Client, String data);


String readAllDataOnSerial(HardwareSerial data_in, String TCPBuffer, bool &fullLine );
void writeDataToSerial( String data);
void write_data_serial_no_cksum(String data);

bool is_valid_command(String characters, int &i);

#ifdef ESP32_CPU

#else
void addTestPilot(WiFiClient LK8000Client, String Pilot, int Dist, int current_pos);
#endif

bool sendUDPPacket ( String packet, IPAddress apFoundIP, bool add_cksum );
bool upLoadtoUDP(bool device_connected, IPAddress connected_ip, String data, IPAddress airwhere_web_ip,int port);
bool upLoadtoUDP( String urlToLoad, IPAddress IP, int udpPort);

String  add_to_ble_data_cksum( String ble_data, String movingpilotData );

bool cksum(String data);

extern unsigned long pflauSendTime;
extern unsigned long pflauCounter;
extern bool addpflau;
extern String streamData;
extern char wiredDirectly;

extern bool deviceConnected;
extern char ble_output;

//v2
extern WiFiUDP udp_flight_software;

extern int debug_level;
#include <BluetoothSerial.h>


//v5.9
extern String ble_data;
extern bool ble_mutex;
extern BluetoothSerial SerialBT;

//5.95

extern char serial_type_out;

