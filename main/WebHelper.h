
#include "AirWare.h"
//#include "fileOps.h"
//#include "ESP8266httpUpdate.h"
//#include "ESP8266HTTPClient.h"
#include "timeFunctions.h"
#include <WiFiClient.h>
//v1.1
#include "xmodem.h"

void Web_setup(void);
void Web_loop(void);

extern byte TxBuffer[PKT_SIZE];

//#include <DNSServer.h>
//extern DNSServer dnsServer;

extern IPAddress apIP;
extern bool dhcp_client_connected;

extern long tcp_rx_weblines_counter;
extern long serial_rx_packets_counter;
extern long nrf_rx_packets_counter;
extern long tcp_tx_weblines_counter;
extern long serial_tx_packets_counter;
extern int longest_loop_time;
//extern ufo_t fo;
extern int flush_time_counter;
extern long web_upload_counter;
extern long bad_rx_packets;
extern uint32_t free_mem;
extern int loopTime;
extern char groundStationMode;
extern char gsLatitude[11];
extern char gsLongitude[12];
extern char gsAltitude[6];
extern char wiredDirectly;
extern long gpsBaudRate;
extern long serialBaudRate;
extern int airWhereID;
extern char wifioffSelected;
extern char awPilotName[30];


extern char airWhereSsid[32];
extern char airWherePassword[32];
extern char airWhere_ap_password[32];
extern char navSofware;
extern char receiveFlarm;
extern char loraFrequency;
extern char web_aircraft_type;
extern char ble_output;

extern bool webUpload;
extern int AwVersion;
extern WiFiClient airwhere_client;
extern bool socket_connected;
extern unsigned int webTime;
extern int myChipID;
extern char ms5611_attached;
extern char vario_on;

extern int climb_threshold;
extern int zero_threshold;
extern int sink_threshold;
extern char calibrate_vario;

//v1.1
extern char flarm_board_expiration[14];
extern bool flarm_update_in_process;
extern bool main_loop_halted;

extern char lora_or_flarm;

//v1.14

extern char ogn_id[7];
extern char ogn_on;

//v1.15
extern char awHexID[5];
extern char awHexManu[3];

//v2.11

extern int debug_level;

//v5

extern bool run_vario;

//v5.95

extern char serial_type_out;
