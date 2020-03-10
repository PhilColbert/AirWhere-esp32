#include "AirWare.h"
#include "FS.h"
#include <Preferences.h>
extern "C" {
 //   #include "esp_vfs.h"
 //   #include "esp_vfs_fat.h"
    #include "esp_system.h"
    #include "esp_log.h"
    #include "esp_err.h"
}



extern char airWhereSsid[32];
extern char airWherePassword[32];
extern char airWhere_ap_password[32];
extern char navSofware;
extern char receiveFlarm;
extern char groundStationMode;
extern char gsLatitude[11];
extern char gsLongitude[12];
extern char gsAltitude[6];
extern char wiredDirectly;
extern long gpsBaudRate;
extern char wifioffSelected;
extern int airWhereID;
extern char awPilotName[30];
extern char loraFrequency;
extern char ms5611_attached;
extern char vario_on;
extern int myChipID;
//v16
extern char lora_or_flarm;
//v1.14

extern char ogn_id[7];
extern char ogn_on;

//v1.15
extern char awHexID[5];
extern char awHexManu[3];

extern long serialBaudRate;
extern char web_aircraft_type;
extern char ble_output;
extern int climb_threshold;
extern int zero_threshold;
extern int sink_threshold;
extern char calibrate_vario;

//v2

extern char packet_repeat;
extern int debug_level;

//v5
extern char vario_sounds_on;
extern int turn_on_off_buzzer;

bool update_buzzer(int buzzer_val);
bool update_pkt_repeat(char pkt_char);
bool update_navSofware(char navSofware_val);
bool update_climb_threshold(int climb_val);
bool update_sink_threshold(int sink_val);
bool update_zero_threshold(int zero_val);
bool update_vario_sounds(char vario_sounds_on_off);

//5.95

extern char serial_type_out;

void load_configFile(void);
void update_configFile(String param, String value_is);
void update_configFile(String param, int value_is);
void update_configFile(String param, char value_is);
void update_configFile(String flarm);
void update_configFile(int debug_on);
void update_configFile(String UpSsid,String UpPass,String UpNavSW, String GS);
void update_configFile( String GS, String vario_in,String  ct_in,String  st_in, String zt_in, String calibrate );
void update_configFile(String UpNavSW, String GSIn, String hwMode, String gpsBaud,String WOSout,String LfreqIn, String bleIn,String baroIn,String VarioIn,String serialBaudIn, String radio_board , String select_s_in);
void update_configFile(String UpSsid,String UpPass,String UpNavSW, String GS, String hwMode, String gpsBaud, String WOSin, String AwIDin, String awPilotNameIn, String aWappassIn, String LfreqIn , String baro_in, String vario_setting);
// update pilot info. - err
void update_configFile(String UpSsid, String UpPass, String GS, String AwIDin,
        String asPilotNameIn, String aWappassIn, String aircraft_type_in,
		   String ognID_in, String ogn_on_off_in,String packetsel);
// update GS data
void update_configFile(char gs, String gsLatitudeSwitch, String  gsLongitudeSwitch,String  gsAltitudeS,String aWiDIn, String  UpSsid,String  UpPass);

void update_actual_file(String UpSsid,String UpPass,String UpNavSW, String GS, String hwMode, String gpsBaud,
        String WOSin, String AwIDin, String awPilotNameIn, String aWappassIn, String LfreqIn,
		String gsLa, String gsLo, String gsA, String rcF, String baro_in, String vario_setting,
		String airC_type_in,String serialBaud, String ble_out, String climb_thresh, String zero_thresh,
		String sink_thresh, String calibrate, String radB, String packet_sel );

int gpsSelectToBaud(char sel);
char gpsBaudtoChar(long sel);
