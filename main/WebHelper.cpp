#include <WiFi.h>
//v2
//#include "ESP8266WebServer/src/ESP8266WebServer.h"
#include "ESP32WebServer/src/ESP32WebServer.h"
#include "fileOps.h"
#include "Update.h"
extern "C" {
    #include "esp_log.h"
    #include "esp_err.h"
}

#include "WebHelper.h"
#include "airwhere_ota.h"

//v3
//ESP8266WebServer server ( 80 );
ESP32WebServer server ( 80 );

int timeOnWeb = 15000;
static const char *TAG = "WebHelper";

char html_out[2600];

void displayWeb() {

  DEBUG_SERIAL_UART("[%ld] - Disconnecting Nmea (displayWeb)\r\n", millis());

  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  char webConnected[49] =    "<font color=#fc00ff>Not Connected</font>";
  char configWire[30] = "Wifi mode On";

  
  if (webUpload)
  {
    strcpy ( webConnected, "<font color=#fc00ff>Connected</font>");
  }
        
  if (groundStationMode == 'n')
  {
    webTime = millis() + timeOnWeb;
//v1.1
    char navSofwareWeb[16] = "";
    char webConnected[49] =    "<font color=#fc00ff>Not Connected</font>";
    char socketConnected[60] = "<font color=#fc00ff>Flight Software Not Connected</font>";
    char checkVersion[73] = "Please connect your AirWhere to the Internet to check for a new version";
    char flarmEnabled[23] = "Nrf905 is not attached";
    char flarmSubmit[47] = "y'><input type=submit value='Connect nrf905";
    
    if (webUpload)
    {
      strcpy ( webConnected, "<font color=#fc00ff>Connected</font>");
      strcpy ( checkVersion, "<input type=submit class=\"bu\" value='Check for new Version'>");
    }

    if ( socket_connected )
    {
      strcpy ( socketConnected, "<font color=#fc00ff>Flight Software Connected</font>");
    }

    if ( wiredDirectly=='y')
    {
      strcpy ( configWire, "Hardwired Mode");
    }
    if ( wiredDirectly=='n')
    {
      strcpy ( configWire, "Wifi Mode");
    }
    if ( wiredDirectly=='s')
    {
      strcpy ( configWire, "StandAlone Wifi Mode");
    }
    if ( wiredDirectly=='o')
    {
      strcpy ( configWire, "AirWhere OTG Mode");
    }        
    if ( wiredDirectly=='v')
    {
      strcpy ( configWire, "Vertica V2 Mode");
    }
    if ( wiredDirectly=='f')
    {
      strcpy ( configWire, "FlySkyHy Mode");
    }

    
    
    if ( navSofware == 'L')
    {
      strcpy ( navSofwareWeb, "LK8000 - Kobo");
    }
    if ( navSofware == 'X')
    {
      strcpy ( navSofwareWeb, "XCSoar - Kobo");
    }
    if ( navSofware == 'A')
    {
      strcpy ( navSofwareWeb, "XCSoar - Android");
    }
    if ( navSofware == 'K')
    {
      strcpy ( navSofwareWeb, "LK8000 - Android");
    }


    if ( receiveFlarm=='y')
    {
       strcpy ( flarmEnabled, "Nrf905 is attached" );
       strcpy ( flarmSubmit, "n'><input type=submit value='Disconnect Nrf905"); 
    }

    airwhere_client.flush();
    airwhere_client.stop();
    socket_connected = false;


    snprintf ( html_out, 2600,
    	               "<html><head>\
<style>.bu {background-color: #d7d7d7;border: none;color: black;padding: 8px 32px;\
text-decoration: none;font-size: 18px;margin: 4px 2px;}\
body{font: normal 12px Verdana, Arial, sans-serif;background-color:#e6e7e8}</style>\
<table border=0 cellpadding=6><tr><td><h1>AirWhere ID - %s<sup> &nbsp ESP32</sup></h1></td></tr>\
<tr><td>Built</td><td><b>%s at %s</b></td></tr>\
<tr><td>Wifi Router Name (SSID)</td><td><b>%s</b></td></tr>\
<tr><td>Wifi Router Password( SSID )</td><td><b>%s</b></td></tr>\
<tr><td>AirWhere Access Point Password</td><td><b>%s</b></td></tr>\
<tr><td><br><font color=#fc00ff>Internet Status</font></td><td><br>%s</td></tr>\
<tr><td><br><br>Flight Software Configured</td><td><br><br><b>%s</b></td></tr>\
<tr><td>Connection Mode</td><td><b>%s</b></td></tr>\
<tr><td><br><font color=#fc00ff>Status</td><td><br>%s</td></tr>\
<tr><td colspan=3><table><tr><td><form method=post action=\"config\"><input type=hidden name=work1 value=please1><input type=submit class=\"bu\" value='Configure Pilot Info'></form></td>\
<td><form method=post action=\"configh\"><input type=hidden name=work2 value=please2><input type=submit class=\"bu\" value='Configure Hardware'></form></td>\
<td><form method=post action=\"configv\"><input type=hidden name=work3 value=please3><input type=submit class=\"bu\" value='Configure Vario'></form></td></tr>\
<td></td><table></td></tr><tr><td><font color=#fc00ff>AirWhere Version</td><td><font color=#fc00ff>%d</td></tr>\
<tr><td><form method=post action=\"version\"><input type=hidden name=work value=please>%s</form></td></tr>\
<tr><td>Uptime</td><td>%02d:%02d:%02d</td></tr>\
<tr><td><form method=post action=\"stats\"><input type=hidden name=stats1 value=stats1><input type=submit class=\"bu\" value='Statistics'></form></td>\
<tr><td><form method=post action=\"switch\"><input type=hidden name=aWssid value=%s><input type=hidden name=aWpass value=%s>\
<tr><td><input type=submit class=\"bu\" value='Switch to Ground Station Mode'></form></td></tr></table></body></body></html>",
		awHexID, __DATE__, __TIME__, airWhereSsid, airWherePassword,airWhere_ap_password, webConnected, navSofwareWeb, configWire,
    				   socketConnected, AwVersion, checkVersion,
    				   hr, min % 60, sec % 60, airWhereSsid, airWherePassword);
    	  }
  else
  {
    snprintf ( html_out, 2600,
               "<h1 align=left>AirWhere Ground Station - %s</h1>\
<form method=post action=\"updateGS\">\
<br><Br>Current Latitude, Longitude, Altitude of your Ground Station ( DD.DDDDDD, DDD.DDDDDD, M )<br><br>\
For example 51.991646,02.223554,-001.757813,132.223344, Altitude 250 ( integer ) <br><br>\
Please keep the formatting to (-)DD.DDDDDD (-)DDD.DDDDDD<br><br><br><br>\
<b>Latitude =</b> %s <input type=number step=\"0.000001\" name=gsLatitudeSwitch value=%s><br><br>\
<b>Longitude</b> = %s <input type=number step=\"0.000001\" name=gsLongitudeSwitch value=%s><br><br>\
<b>Altitude AGL (metres)</b> = %s <input type=number name=gsAltitudeSwitch value=%s><br><br><br><Br>\
Enter your Ground Station ID,WiFi Routers SSID and Password for AirWhere to upload Pilots<br>\
Please register the Id on the website to allocate your unique ID<br><br>\
GroundStation ID - %s <input type=text class=bu required=required pattern=\"[0-9a-fA-F]{4}\" name=aWiD value=%s><br><br>\
SSID - %s <input type=text name=aWssidSwitch value=%s><br><br>\
Password - %s <input type=text name=aWpassSwitch value=%s><br><br>\
%s<br><hr><br>\
<input type=submit value=Submit></form><hr>\
Uptime - %02d:%02d:%02d<br>\
AirWhere Packets Received - %ld<br>\
<form method=post action=\"switchAW\"><br><br><br><br>\
<input type=hidden name=aWssid value=%s><input type=hidden name=aWpass value=%s>\
<input type=hidden name=gsLatitude value=%s><input type=hidden name=gsLongitude value=%s>\
<input type=submit value='Switch to AirWhere Mode'></form>", awHexID, gsLatitude, gsLatitude, gsLongitude, gsLongitude, gsAltitude, gsAltitude,
awHexID,awHexID, airWhereSsid, airWhereSsid, airWherePassword, airWherePassword, webConnected, hr, min % 60, sec % 60, serial_rx_packets_counter,
               airWhereSsid, airWherePassword, gsLatitude, gsLongitude);

  }
  server.send ( 200, "text/html", html_out );
}

void configureAW() {
  DEBUG_SERIAL_UART("[%ld] - Disconnecting Nmea (configureAW)\r\n", millis());
  webTime = millis() + timeOnWeb;
  airwhere_client.flush();
  airwhere_client.stop();
  socket_connected = false;


  char Root_temp[2500];

  char type_paraglider[9]="";
  char type_hangglider[9]="";
  char type_balloon[9]="";
  char type_glider[9]="";

  switch (web_aircraft_type)
  {
    case '1':
    strcpy ( type_paraglider, "selected");
    break;
    case '2':
    strcpy ( type_hangglider, "selected");
    break;
    case '3':
    strcpy ( type_balloon, "selected");
    break;
    case '4':
    strcpy ( type_glider, "selected");
    break;
  }

  char ognoff[9]="";
  char ognon[9]="";

  if ( ogn_on == 'y')
  {
    strcpy ( ognon, "selected");
  }
  else
  {
    strcpy ( ognoff, "selected");
  }

  //v16.1
  char disabled[18]={};

  if (lora_or_flarm=='f')
  {
      strcpy ( disabled,"disabled=disabled");
  }

  //v2

  char packetselon[9]="";
  char packetseloff[9]="";;

  if ( packet_repeat == 'y')
  {
    strcpy ( packetselon, "selected");
  }
  else
  {
    strcpy ( packetseloff, "selected");
  }


  DEBUG_SERIAL_UART("[%ld] - Point 1 (configureAW)\r\n", millis());
  snprintf ( Root_temp, 2500,"<style>.bu {background-color: #d7d7d7;border: none;color: black;padding: 8px 32px;\
text-decoration: none;font-size: 18px;margin: 4px 2px;}\
body{font: normal 12px Verdana, Arial, sans-serif;background-color:#e6e7e8}</style>\
<table border=0 cellpadding=6><tr><td><h1 align=center>Configure AirWhere %s</h1>\
<form method=post action=\"update\">\
<tr><td>Manufacturer ID</td><td>%s</td></tr>\
<tr><td>AirWhere ID</td><td><input type=text %s class=bu required=required pattern=\"[0-9A-F]{4}\" name=aWiD value=%s></td></tr>\
<tr><td colspan=2>( IMPORTANT - ( 4 digits 0000-FFFF ) ie 4D2D or 0203 - Please register your unit on the website FIRST! )<br>If you are using the FLarm board , this ID is assigned by the board itself and you can not change.</td></tr>\
<tr><td>OGN ID</td><td><input type=text class=bu name=ognID value=%s></td></tr>\
<tr><td>OGN Tracking</td><td><select class=bu name=ogn_on_off><option value=y %s>yes</option><option value=n %s>no</option></select></td></tr>\
<tr><td colspan=2>( IMPORTANT - Please visit <a href=http://ddb.glidernet.org>http://ddb.glidernet.org</a> - and register before using this option. )</td></tr>\
<tr><td>Pilot Name</td><td><input class=bu type=text name=awPilotName value=\"%s\"></td></tr>\
<tr><td>Aircraft Type</td><td><select class=bu name=wat><option value=1 %s>Paraglider</option><option value=2 %s>Hangglider</option><option value=3 %s>Balloon</option><option value=4 %s>Glider</option></select>\
<tr><td>Wifi Router Name (SSID)</td><td><input class=bu type=text name=aWssid value=\"%s\"></td></tr>\
<tr><td>Wifi Router Password( SSID )</td><td><input class=bu type=text name=aWpass value=\"%s\"></td></tr>\
<tr><td>AirWhere Access Point Password ( 8 digits )</td><td><input class=bu type=text required=required pattern=\"[0-9A-Za-z]{8}\" name=aWappass value=%s> (  8 characters or more )</td></tr>\
<tr><td>Packet Repeat ( keep xcsoar pilots on screen for longer )</td><td><select class=bu name=selectpkt><option value=yes %s>yes</option><option value=no %s>no</option></select></td></tr>\
<tr><td><input type=submit class=bu value=Submit></form></td></tr>\
<tr><td><a href=/>Return to AirWhere Home</a></td><tr></a>\</table>",
awHexID , awHexManu,disabled,  awHexID ,ogn_id,ognon, ognoff, awPilotName, type_paraglider, type_hangglider, type_balloon, type_glider, airWhereSsid,
                    airWherePassword,airWhere_ap_password,packetselon,packetseloff);
  DEBUG_SERIAL_UART("[%ld] - Point 2 (configureAW)\r\n", millis());
  server.send ( 200, "text/html", Root_temp );
//Serial.println(navSofwareWeb);

  DEBUG_SERIAL_UART("[%ld] - Sent Page Back (configureAW)\r\n", millis());

}


void configureAWHardware() {
  DEBUG_SERIAL_UART("[%ld] - Disconnecting Nmea (configureAW)\r\n", millis());
  webTime = millis() + timeOnWeb;
  airwhere_client.flush();
  airwhere_client.stop();
  socket_connected = false;

  char Root_temp[3000];
//v1.1
  char navSofwareWeb[17] = "";
  char softwareL[9] = "";
  char softwareX[9] = "";
  char softwareA[9] = "";
  char softwareK[9] = "";

  if ( navSofware == 'L')
  {
    strcpy ( navSofwareWeb, "LK8000 - Kobo");
    strcpy ( softwareL, "selected");
  }
  if ( navSofware == 'X')
  {
    strcpy ( navSofwareWeb, "XCSoar - Kobo");
    strcpy ( softwareX, "selected");
  }
  if ( navSofware == 'A')
  {
    strcpy ( navSofwareWeb, "XCSoar - Android");
    strcpy ( softwareA, "selected");
  }
  if ( navSofware == 'K')
  {
    strcpy ( navSofwareWeb, "XCSoar - Android");
    strcpy ( softwareK, "selected");
  }

  char hw[20] = "";
  char hwNo[9] = "";
  char hwV[9] = "";
  char hwYes[9] = "";
  char hwSW[9] = "";
  char hwOTG[9] = "";
  char hwF[9] = "";

  if ( wiredDirectly == 'y')
  {
    strcpy ( hw, "Hard Wired Mode");
    strcpy ( hwYes, "selected");
  }
  if ( wiredDirectly == 'n')
  {
    strcpy ( hw, "Wifi Mode");
    strcpy ( hwNo, "selected");
  }
  if ( wiredDirectly == 's')
  {
    strcpy ( hw, "StandAlone Wifi Mode");
    strcpy ( hwSW, "selected");
  }
  if ( wiredDirectly == 'o')
  {
    strcpy ( hw, "AirWhere OTG Mode");
    strcpy ( hwOTG, "selected");
  }
  if ( wiredDirectly == 'v')
  {
    strcpy ( hw, "Vertica v2 Mode");
    strcpy ( hwV, "selected");
  }
  if ( wiredDirectly == 'f')
  {
    strcpy ( hw, "FlySkyHy Mode");
    strcpy ( hwF, "selected");
  }

  char gpsBaud[7]="";
  char gps9600[9]="";
  char gps19200[9]="";
  char gps38400[9]="";
  char gps57600[9]="";
  char gps115200[9]="";


  switch (gpsBaudRate)
  {
    case 9600:
     strcpy ( gpsBaud, "9600");
     strcpy ( gps9600, "selected");
     break;
    case 19200:
     strcpy ( gpsBaud, "19200");
     strcpy ( gps19200, "selected");
     break;
    case 38400:
     strcpy ( gpsBaud, "38400");
     strcpy ( gps38400, "selected");
     break;
    case 57600:
     strcpy ( gpsBaud, "57600");
     strcpy ( gps57600, "selected");
     break;
    case 115200:
     strcpy ( gpsBaud, "115200");
     strcpy ( gps115200, "selected");
     break;
  }

  char serialBaud[7]="";
  char serial9600[9]="";
  char serial19200[9]="";
  char serial38400[9]="";
  char serial57600[9]="";
  char serial115200[9]="";


  switch (serialBaudRate)
  {
    case 9600:
     strcpy ( serialBaud, "9600");
     strcpy ( serial9600, "selected");
     break;
    case 19200:
     strcpy ( serialBaud, "19200");
     strcpy ( serial19200, "selected");
     break;
    case 38400:
     strcpy ( serialBaud, "38400");
     strcpy ( serial38400, "selected");
     break;
    case 57600:
     strcpy ( serialBaud, "57600");
     strcpy ( serial57600, "selected");
     break;
    case 115200:
     strcpy ( serialBaud, "115200");
     strcpy ( serial115200, "selected");
     break;
  }

  char wifioff[9]="";
  char wifion[9]="";

  if ( wifioffSelected == 'n')
  {
    strcpy ( wifion, "selected");
  }
  else
  {
    strcpy ( wifioff, "selected");
  }

  char nsw[10]="";
  strcpy ( nsw, navSofwareWeb);

  char eightsel[9]="";
  char ninesel[9]="";;

  if ( loraFrequency == '8')
  {
    strcpy ( eightsel, "selected");
  }
  else
  {
    strcpy ( ninesel, "selected");
  }

  char baro_yes[9]="";
  char baro_no[9]="";

  if ( ms5611_attached == 'y')
  {
    strcpy ( baro_yes, "selected");
  }
  else
  {
    strcpy ( baro_no, "selected");
  }

  char vario_yes[9]="";
  char vario_no[9]="";

  if ( vario_on == 'y')
  {
    strcpy ( vario_yes, "selected");
  }
  else
  {
    strcpy ( vario_no, "selected");
  }

  char ble_ble[9]="";
  char ble_blue[9]="";
  char ble_no[9]="";


  if ( ble_output == 'e')
  {
    strcpy ( ble_ble, "selected");
  }
  if ( ble_output == 'u')
  {
    strcpy ( ble_blue, "selected");
  }
  if ( ble_output == 'n')
  {
    strcpy ( ble_no, "selected");
  }

  //v1.1
    char lorasel[9]="";
    char flarmsel[9]="";
    char sx1276sel[9]="";
    char sx1276heltec[9]="";
    char sx1276ttgo[9]="";
    char sx1276tbeam[9]="";
    char flarmupdate[110]="";

    if ( lora_or_flarm == 'l')
    {
      strcpy ( lorasel, "selected");
    }
    if ( lora_or_flarm == 'f')
    {
      strcpy ( flarmsel, "selected");
      if (webUpload)
      {
        snprintf ( flarmupdate,110, "<tr><td>Flarm Expires : <b>%s</b></td><td><a href=updateflarm>Update Flarm board</a></td></tr>", flarm_board_expiration);
      }
      else
      {
          snprintf ( flarmupdate,110, "<tr><td>Flarm Expires : <b>%s</b></td><td>Connect to Internet to Update Flarm</td></tr>", flarm_board_expiration);
      }
    }
    if ( lora_or_flarm == 'r')
    {
      strcpy ( sx1276sel, "selected");
    }
    if ( lora_or_flarm == 'h')
    {
      strcpy ( sx1276heltec, "selected");
    }
    if ( lora_or_flarm == 't')
    {
      strcpy ( sx1276ttgo, "selected");
    }
    if ( lora_or_flarm == 'b')
    {
      strcpy ( sx1276tbeam, "selected");
    }

    //5.95
    char serial_type_out_a[9]="";
	char serial_type_out_p[9]="";


    if ( serial_type_out == 'a')
    {
    	strcpy ( serial_type_out_a, "selected");
    }
    if ( serial_type_out == 'p')
    {
    	strcpy ( serial_type_out_p, "selected");
    }

  DEBUG_SERIAL_UART("[%ld] - Point 1 (configureAW)\r\n", millis());
  snprintf ( Root_temp, 3200,"<style>.bu {background-color: #d7d7d7;border: none;color: black;padding: 8px 32px;\
text-decoration: none;font-size: 18px;margin: 4px 2px;}\
body{font: normal 12px Verdana, Arial, sans-serif;background-color:#e6e7e8}</style>\
<table border=0 cellpadding=6><tr><td colspan=2><h1 align=center>Configure AirWhere Hardware %s</h1>\
<form method=post action=\"updateh\">\
<tr><td>Radio Frequency (Europe - 868Mhz, US - 915Mhz)</td><td><select class=bu name=Lfreq><option value=8 %s>868Mhz</option><option value=9 %s>915mhz</option></select></td></tr>\
<tr><td>Radio Board</td><td><select class=bu name=rfboard><option value=l %s>RF Lora Board (sx1272)</option><option value=f %s>Flarm Board</option><option value=r %s>RFM95 Board(sx1276)</option><option value=h %s>Heltec (sx1276)</option><option value=t %s>TTGO v2.0 (sx1276)</option><option value=b %s>TTGO T-beam (sx1276)</option></select></td></tr>\
%s<tr><td>Flight Software</td><td><select class=bu name=navsw><option value=L %s>LK8000 - Kobo</option><option value=X %s>XCSoar - Kobo</option>\
<option value=K %s>LK8000 - Android</option><option value=A %s>XCSoar - Android</option></select></td></tr>\
<tr><td>Connection Mode</td><td><select class=bu name=hwMode><option value=y %s>OTG/Serial Mode ( GPS Attached to ESP32 )</option><option value=o %s>OTG/Serial Mode ( GPS Attached to Kobo )</option><option value=n %s>Wifi Mode</option>\
<option value=s %s>Wifi Standalone Mode</option><option value=v %s>Vertica V2 Mode</option><option value=f %s>FlySkyHy Mode</option></select></td></tr>\
<tr><td>BLE / Bluetooth</td><td><select class=bu name=bleO><option value=e %s>BLE Output</option><option value=u %s>Bluetooth Output</option><option value=n %s>Off</option></select></td></tr>\
<tr><td>GY86 Attached</td><td><select class=bu name=baro><option value=y %s>Yes</option><option value=n %s>No</option></select></td></tr>\
<tr><td>Vario</td><td><select class=bu name=vario><option value=y %s>On</option><option value=n %s>Off</option></select></td></tr>\
<tr><td>GPS Baud Rate ( hard wired only )</td><td><select class=bu name=gpsBaud><option value=9 %s>9600 bps</option><option value=2 %s>19200 bps</option><option value=3 %s>38400 bps</option>\
<option value=5 %s>57600 bps</option><option value=1 %s>115200 bps</option></select></td></tr>\
<tr><td>Kobo (Serial/OTG) Baud Rate</td><td><select class=bu name=serialBaud><option value=9 %s>9600 bps</option><option value=2 %s>19200 bps</option><option value=3 %s>38400 bps</option>\
<option value=5 %s>57600 bps</option><option value=1 %s>115200 bps</option></select></td></tr>\
<tr><td>Serial Data Type Out</td><td><select class=bu name=selectserial><option value=a %s>All Serial Data</option><option value=p %s>Pilot Data Only</option></select></td></tr>\
<tr><td>Wifi Off(wifi off after 3 mins to save power<br>Leave no if you dont know what this means)</td><td><select class=bu name=selectwifioff><option value=yes %s>yes</option><option value=no %s>no</option></select></td></tr>\
<tr><td><input type=submit class=bu value=Submit></form></td></tr>\
<tr><td><a href=/>Return to AirWhere Home</a></td><tr></a>\</table>",
awHexID,eightsel,ninesel, lorasel,  flarmsel, sx1276sel, sx1276heltec,sx1276ttgo,sx1276tbeam, flarmupdate, softwareL, softwareX, softwareK,softwareA, hwYes,hwOTG,hwNo,hwSW, hwV,hwF, ble_ble,ble_blue, ble_no, baro_yes, baro_no,vario_yes,vario_no,
gps9600,gps19200, gps38400,gps57600,gps115200,serial9600,serial19200, serial38400,serial57600,serial115200, serial_type_out_a, serial_type_out_p, wifioff,wifion);

  DEBUG_SERIAL_UART("[%ld] - Point 2 (configureAW)\r\n", millis());
  server.send ( 200, "text/html", Root_temp );
//Serial.println(navSofwareWeb);

  DEBUG_SERIAL_UART("[%ld] - Sent Page Back (configureAW)\r\n", millis());

}


void configureAWVario() {
  DEBUG_SERIAL_UART("[%ld] - Disconnecting Nmea (configureAW)\r\n", millis());
  webTime = millis() + timeOnWeb;
  airwhere_client.flush();
  airwhere_client.stop();
  socket_connected = false;

  char Root_temp[2048];

  char vario_yes[9]="";
  char vario_no[9]="";
  char calibrate_on[9]="";
  char calibrate_off[9] = "";

  if ( vario_on == 'y')
  {
    strcpy ( vario_yes, "selected");
  }
  else
  {
    strcpy ( vario_no, "selected");
  }

  if (calibrate_vario == 'y')
  {
	  strcpy(calibrate_on, "selected");
  }
  else
  {
	  strcpy(calibrate_off, "selected");
  }

  //climb threshold, sink threshold, zero threshold,

  DEBUG_SERIAL_UART("[%ld] - Point 1 (configureAW)\r\n", millis());
    snprintf ( Root_temp, 2048,"<style>.bu {background-color: #d7d7d7;border: none;color: black;padding: 8px 32px;\
  text-decoration: none;font-size: 18px;margin: 4px 2px;}\
  body{font: normal 12px Verdana, Arial, sans-serif;background-color:#e6e7e8}</style>\
  <table border=0 cellpadding=6><tr><td><h1 align=center>Configure AirWhere Vario %s</h1>\
  <form method=post action=\"updatev\">\
  <tr><td>Vario</td><td><select class=bu name=vario><option value=y %s>On</option><option value=n %s>Off</option></select>\
  <tr><td>Calibrate</td><td><select class=bu name=calibrate><option value=y %s>At startup</option><option value=n %s>Off</option></select>\
  <tr><td colspan=2>Vario values are in cm/second - ie 0.1m/s climb is 10cm/s<br>Most paragliders have a sink rate in still air of 100cm/s</tr></td>\
  <tr><td>Climb Threshold ( 0 -> 1000 cm/s )</td><td><input type=number class=bu min=0 max=1000 step=1 name=climbthreshold value=%d>cm/s</td></tr>\
  <tr><td>Zero Threshold( -1000 -> 1000 cm/s )</td><td><input type=number class=bu min=-1000 max=1000 step=1 name=zerothreshold value=%d>cm/s</td></tr>\
  <tr><td>Sink Threshold ( -1000 -> 0 cm/s )</td><td><input type=number class=bu min=-1000 max=0 step=1 name=sinkthreshold value=%d>cm/s</td></tr>\
 <tr><td><input type=submit class=bu value=Submit></form></td></tr>\
  <tr><td><a href=/>Return to AirWhere Home</a></td><tr></a>\</table>",
  awHexID,vario_yes,vario_no,calibrate_on,calibrate_off,climb_threshold,zero_threshold, sink_threshold );
    DEBUG_SERIAL_UART("[%ld] - Point 2 (configureAW)\r\n", millis());
    server.send ( 200, "text/html", Root_temp );
  //Serial.println(navSofwareWeb);

    DEBUG_SERIAL_UART("[%ld] - Sent Page Back (configureAW)\r\n", millis());

}


void aWversion() {
  DEBUG_SERIAL_UART("[%ld] - Disconnecting Nmea (aWversion)\r\n", millis());
  webTime = millis() + timeOnWeb;
  airwhere_client.flush();
  airwhere_client.stop();
  socket_connected = false;
  WiFiClient clientAWV;
  //const int httpPort = 80;

  if (!clientAWV.connect(airWareServer, 80))
  {
    server.send ( 200, "text/html", "Failed to access www.air-ware.co.uk, try later" );
    return;
  }
  else
  {

	  clientAWV.print(String("GET /downloads/esp32version.txt")  + " HTTP/1.1\r\n" +
	                 "Host: www.airwhere.co.uk\r\n" +
	                 "Connection: close\r\n\r\n");

 //   clientAWV.write("GET /downloads/esp32version.txt HTTP/1.1\r\nHost: www.airwhere.co.uk\r\nConnection: close\r\n\r\n");
  //  clientAWV.write("\n");
  }

  String line;

 // while (clientAWV.available())
 // {
  for (int l=0;l<11;l++)
  {
    line = clientAWV.readStringUntil('\r');
  }
 // }


  clientAWV.stop();

  String v = line.substring(9);
  int vI = v.toInt();
  v.trim();

  if (vI == AwVersion)
  {
    server.send ( 200, "text/html", "<html><head>\
<style>.bu {background-color: #d7d7d7;border: none;color: black;padding: 8px 32px;\
text-decoration: none;font-size: 18px;margin: 4px 2px;}\
body{font: normal 12px Verdana, Arial, sans-serif;background-color:#e6e7e8}</style>\
<h1>AirWhere is on latest version</h1><br><Br><a href=/>\
Return to AirWhere Home</a>\
<br><Br><br><br><form method=post action=\"devVersion\"><input type=hidden name=work1 value=please1>\
<input type=submit class=bu value='Install Development AirWhere'></form>" );
  }
  else
  {


    snprintf ( html_out, 750,

               "<html><head>\
<style>.bu {background-color: #d7d7d7;border: none;color: black;padding: 8px 32px;\
text-decoration: none;font-size: 18px;margin: 4px 2px;}\
body{font: normal 12px Verdana, Arial, sans-serif;background-color:#e6e7e8}</style>\
<h1>AirWhere %s - Please Update</h1><br>\
<form method=post action=\"updateVersion\">\
<table><tr><td>Current Version</td><td>%d</td></tr>\
<tr><td>New Version</td><td>%d</td></tr>\
<tr><td><br><br><input class=bu type=submit value=\"Update\"></form></td></tr>\
<tr><td><a href=/><br><br>AirWhere home page</a></td></tr>\
<tr><td><br><Br><form method=post action=\"devVersion\"><input type=hidden name=work1 value=please1>\
<input class=bu type=submit value='Install Development AirWhere'></form></td></tr></table>", awHexID, AwVersion, vI);

    server.send ( 200, "text/html", html_out );
  }

}


//v1.1

void updateflarm()
{
  DEBUG_SERIAL_UART("[%ld] - Disconnecting Nmea (stats)\r\n", millis());
  webTime = millis() + timeOnWeb;
  //v16
  airwhere_client.flush();
  airwhere_client.stop();

  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  char Root_temp[1000];

  snprintf ( Root_temp, 1000, "<html><head>\
<style>.bu {background-color: #d7d7d7;border: none;color: black;padding: 8px 32px;\
text-decoration: none;font-size: 18px;margin: 4px 2px;}\
body{font: normal 12px Verdana, Arial, sans-serif;background-color:#e6e7e8}</style>\
<h1>AirWhere %s - Update Flarm Board</h1><br>\
<form method=post action=\"processfupdate\">\
<table cellspacing=10><tr><td>AirWhere Current Version</td><td>%d</td></tr>\
<tr><td>Uptime</td><td>%02d:%02d:%02d</td></tr>\
<tr><td>Flarm board expiration date</td><td>%s</td></tr>\
<tr><td><input type=submit class=bu value=Process Update></form></td><td></td></tr>\
<tr><td><a href=/>Return to AirWhere Home</a></td><tr></a></td></tr></table>",awHexID, AwVersion,hr, min % 60, sec % 60, flarm_board_expiration);

  server.send ( 200, "text/html", Root_temp );

}

//v1.1

void processfupdate()
{
  DEBUG_SERIAL_UART("[%ld] - Disconnecting Nmea (updateVersion)\r\n", millis());
  webTime = millis() + timeOnWeb;
  //v16
  airwhere_client.flush();
  airwhere_client.stop();
  flarm_update_in_process=true;

  yieldDelay(500);
  yield();

  server.send ( 200, "text/html", "<!DOCTYPE html><html><style>#myProgress{position: relative;width: 100%;height: 30px;background-color: #ddd;}\
#myBar {position: absolute;width: 1%;height: 100%;background-color: #4CAF50;}\
.bu {background-color: #d7d7d7;border: none;color: black;padding: 8px 32px;\
text-decoration: none;font-size: 18px;margin: 4px 2px;}\
body{font: normal 12px Verdana, Arial, sans-serif;background-color:#e6e7e8}</style><body onload=\"move()\" width=100%>\
<script>var width = 1;function move() {var elem = document.getElementById(\"myBar\");\
var id = setInterval(frame, 350);function frame(){width++;if (width >= 100){clearInterval(id);\
document.getElementById(\"p1\").innerHTML = \"<br><br><a href=/>AirWhere Home</a>\";}\
elem.style.width = width + '%';}}</script><div>Processing Flarm Update - please wait - if \
the update is successful the board will automatically reboot, please then reconnect , please check for new expiration date.<div id=myProgress><div id=myBar></div>\
<div align=center><p id=\"p1\"><br><br>>>>      Processing Flarm Update     >>></p></div></body></html>" );

  while (!main_loop_halted)
  {
	  delay(500);
  }

  int ret= xmodem_transmit();

  if( ret==0)
  {

    Serial.println("Flarm Update completed - rebooting.");
  }
  else
  {
    server.send ( 200, "text/html", "Failed to access www.airwhere.co.uk, try later" );
    Serial.println("Flarm Update failed - please try again.");
  }


  yieldDelay(500);

  ESP.restart();


}



void updateAW_entered_directly()
{
  DEBUG_SERIAL_UART("[%ld] - updateAW_entered_directly (updateAW)\r\n", millis());
  webTime = millis() + timeOnWeb;
  server.send ( 200, "text/html", "<h1>Entered Update page directly - Error</h1><a href=/>Please return to AirWhere home page</a></h2>" );
  yieldDelay(100);
}

void switchGS()
{
  DEBUG_SERIAL_UART("[%ld] - Disconnecting Nmea (switchGS)\r\n", millis());
  webTime = millis() + timeOnWeb;
  airwhere_client.flush();
  airwhere_client.stop();
  socket_connected = false;

  String UpSsid, UpPass, UpNavSW = "L", GS = "y";

  for ( uint8_t i = 0; i < server.args(); i++ )
  {
    if ( server.argName ( i ) == "aWssid")
    {
      UpSsid = server.arg ( i );
    }
    if ( server.argName ( i ) == "aWpass")
    {
      UpPass = server.arg ( i );
    }
    if ( server.argName ( i ) == "navsw")
    {
      UpNavSW = server.arg ( i );
    }
  }
  DEBUG_SERIAL_UART("[%ld] -( configureGS - y )\r\n", millis());
  update_configFile(UpSsid, UpPass, UpNavSW, GS);

  server.send ( 200, "text/html", "<h1>AirWhere Rebooting into Ground Station Mode - please reconnect to the Access Point again<br><br></h1><br><br><a href=/>Reconnect to AirWhere home page</a></h2>" );
  delay(3000);
  DEBUG_SERIAL_UART("[%ld] -!!!!! Restarting AirWhere !!!!!\r\n", millis());
  ESP.restart();
}

void flarmConf()
{

  DEBUG_SERIAL_UART("[%ld] - Disconnecting Nmea (flarmConf)\r\n", millis());
  webTime = millis() + timeOnWeb;
  airwhere_client.flush();
  airwhere_client.stop();
  socket_connected = false;

  String flarmC;

  for ( uint8_t i = 0; i < server.args(); i++ )
  {
    if ( server.argName ( i ) == "flarmC")
    {
      flarmC = server.arg ( i );
    }
  }
  DEBUG_SERIAL_UART("[%ld] -( flarmConf - n )\r\n", millis());
  
  update_configFile(flarmC);

  server.send ( 200, "text/html", "<h1>AirWhere Changing Nrf905 Setting - please reconnect to the Access Point again<br><br></h1><a href=/>Reconnect to AirWhere home page</a></h2>" );

  delay(3000);
  DEBUG_SERIAL_UART("[%ld] -!!!!! Restarting AirWhere !!!!!\r\n", millis());
  ESP.restart();
  
}

void aw_stats()
{
  DEBUG_SERIAL_UART("[%ld] - Disconnecting Nmea (stats)\r\n", millis());
  webTime = millis() + timeOnWeb;
  airwhere_client.flush();
  airwhere_client.stop();
  socket_connected = false;

  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  char Root_temp[1520];

  snprintf ( Root_temp, 1520, "<html><head>\
<style>.bu {background-color: #d7d7d7;border: none;color: black;padding: 8px 32px;\
text-decoration: none;font-size: 18px;margin: 4px 2px;}\
body{font: normal 12px Verdana, Arial, sans-serif;background-color:#e6e7e8}</style>\
<h1>AirWhere %s - Statistics</h1><br>\
<table cellspacing=10><tr><td>Current Version</td><td>%d</td></tr>\
<tr><td>Uptime</td><td>%02d:%02d:%02d</td></tr>\
<tr><td>NMEA lines received</td><td>%ld</td></tr>\
<tr><td>AirWhere packets received</td><td>%ld</td></tr>\
<tr><td>FLarm packets received</td><td>%ld</td></tr>\
<tr><td>Invalid packets received</td><td>%ld</td></tr>\
<tr><td>NMEA Lines transmitted</td><td>%ld</td></tr>\
<tr><td>AirWhere packets transmitted</td><td>%ld</td></tr>\
<tr><td>Longest Loop time</td><td>%d</td></tr>\
<tr><td>Last Loop time</td><td>%d</td></tr>\
<tr><td>Free Heap</td><td>%d</td></tr>\
<tr><td>Minimum Ever Free Heap</td><td>%d</td></tr>\
<tr><td>CPU Speed</td><td>%d</td></tr>\
<tr><td>SDK Version</td><td>%s</td></tr>\
<tr><td><a href=/>Return to AirWhere Home</a></td><tr></a></td></tr></table>",awHexID, AwVersion,hr, min % 60, sec % 60,tcp_rx_weblines_counter, serial_rx_packets_counter,
nrf_rx_packets_counter, bad_rx_packets, tcp_tx_weblines_counter, serial_tx_packets_counter ,longest_loop_time, loopTime, (int)ESP.getFreeHeap(), (int)xPortGetMinimumEverFreeHeapSize(), (int)ESP.getCpuFreqMHz(), ESP.getSdkVersion());

  server.send ( 200, "text/html", Root_temp );

}


void aw_debug()
{
  DEBUG_SERIAL_UART("[%ld] - Disconnecting Nmea (stats)\r\n", millis());
  webTime = millis() + timeOnWeb;
  airwhere_client.flush();
  airwhere_client.stop();
  socket_connected = false;

  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

 // Serial.println("in debug");
  delay(200);
  for ( uint8_t i = 0; i < server.args(); i++ )
  {
	 // Serial.println(server.argName ( i ));
	  if ( server.argName ( i ) == "debug_on")
	  {
		  if ( server.arg ( i ) == "y" )
		  {
			  Serial.println("Setting  debug y ");
			  debug_level=1;
			  update_configFile(debug_level);
		  }
		  if ( server.arg ( i ) == "n" )
		  {
			  Serial.println("Setting  debug n ");
			  debug_level=0;
			  update_configFile(debug_level);
		  }

	  }
  }

  delay(200);
  char debug_yes[9]="";
  char debug_no[9]="";

  if ( debug_level == 1)
  {
    strcpy ( debug_yes, "selected");
  }
  else
  {
    strcpy ( debug_no, "selected");
  }

  char Root_temp[1250]="";

  snprintf ( Root_temp, 1250, "<html><head>\
<style>.bu {background-color: #d7d7d7;border: none;color: black;padding: 8px 32px;\
text-decoration: none;font-size: 18px;margin: 4px 2px;}\
body{font: normal 12px Verdana, Arial, sans-serif;background-color:#e6e7e8}</style>\
<h1>AirWhere %s - Statistics</h1><br>\
<form method=post action=\"debug\">\
<table cellspacing=10><tr><td>Current Version</td><td>%d</td></tr>\
<tr><td>Uptime</td><td>%02d:%02d:%02d</td></tr>\
<tr><td>NMEA lines received</td><td>%ld</td></tr>\
<tr><td>AirWhere packets received</td><td>%ld</td></tr>\
<tr><td>FLarm packets received</td><td>%ld</td></tr>\
<tr><td>Invalid packets received</td><td>%ld</td></tr>\
<tr><td>NMEA Lines transmitted</td><td>%ld</td></tr>\
<tr><td>AirWhere packets transmitted</td><td>%ld</td></tr>\
<tr><td>Longest Loop time</td><td>%d</td></tr>\
<tr><td>Last Loop time</td><td>%d</td></tr>\
<tr><td>Debug is </td><td><select class=bu name=debug_on><option value=y %s>On</option><option value=n %s>Off</option></select></td></tr>\
<tr><td><input type=submit class=bu value=debug></form></td></tr>\
<tr><td><a href=/>Return to AirWhere Home.</a></td><tr></a></td></tr></table>",awHexID, AwVersion,hr, min % 60, sec % 60,tcp_rx_weblines_counter, serial_rx_packets_counter,
nrf_rx_packets_counter, bad_rx_packets, tcp_tx_weblines_counter, serial_tx_packets_counter ,longest_loop_time, loopTime, debug_yes,debug_no);

  server.send ( 200, "text/html", Root_temp );

  yieldDelay(500);

 // ESP.restart();

}


void switchAW()
{
  DEBUG_SERIAL_UART("[%ld] - Disconnecting Nmea (switchAW)\r\n", millis());
  webTime = millis() + timeOnWeb;
  airwhere_client.flush();
  airwhere_client.stop();
  socket_connected = false;

  String UpSsid, UpPass, UpNavSW = "L", GS = "n";

  for ( uint8_t i = 0; i < server.args(); i++ )
  {
    if ( server.argName ( i ) == "aWssid")
    {
      UpSsid = server.arg ( i );
    }
    if ( server.argName ( i ) == "aWpass")
    {
      UpPass = server.arg ( i );
    }
    if ( server.argName ( i ) == "navsw")
    {
      UpNavSW = server.arg ( i );
    }
  }
  DEBUG_SERIAL_UART("[%ld] -( configureGS - n )\r\n", millis());
  update_configFile(UpSsid, UpPass, UpNavSW, GS);

  server.send ( 200, "text/html", "<h1>AirWhere Rebooting into AirWhere Mode - please reconnect to the Access Point again<br><br></h1><a href=/>Reconnect to AirWhere home page</a></h2>" );

  delay(3000);
  DEBUG_SERIAL_UART("[%ld] -!!!!! Restarting AirWhere !!!!!\r\n", millis());
  ESP.restart();
}

void updateAW()
{
	  DEBUG_SERIAL_UART("[%ld] - Disconnecting Nmea (updateAW)\r\n", millis());
	  webTime = millis() + timeOnWeb;
	  airwhere_client.flush();
	  airwhere_client.stop();
	  socket_connected = false;

	  String UpSsid, UpPass, GS = "n", AwIDin, asPilotNameIn, aWappassIn,aircraft_type_in,ognID_in,ogn_on_off_in,packetsel;

	  for ( uint8_t i = 0; i < server.args(); i++ )
	  {
	    if ( server.argName ( i ) == "aWssid")
	    {
	      UpSsid = server.arg ( i );
	    }
	    if ( server.argName ( i ) == "aWpass")
	    {
	      UpPass = server.arg ( i );
	    }

	    if ( server.argName ( i ) == "aWiD")
	    {
	      AwIDin = server.arg ( i );
	    }
	    if ( server.argName ( i ) == "awPilotName")
	    {
	      asPilotNameIn = server.arg ( i );
	    }
	    if ( server.argName ( i ) == "aWappass")
	    {
	    	aWappassIn = server.arg ( i );
	    }

	    if ( server.argName ( i ) == "wat")
	    {
	        aircraft_type_in = server.arg ( i );
	    }
	    //v1.14

	    if ( server.argName ( i ) == "ognID")
	    {
	        ognID_in = server.arg ( i );
	    }
	    if ( server.argName ( i ) == "ogn_on_off")
	    {
	    	ogn_on_off_in = server.arg ( i );
	    }
	    if ( server.argName ( i ) == "selectpkt")
	    {
	       packetsel = server.arg ( i );
	    }
	  }

	  update_configFile(UpSsid, UpPass, GS, AwIDin,asPilotNameIn,aWappassIn,aircraft_type_in,ognID_in,ogn_on_off_in,packetsel );

	  server.send ( 200, "text/html", "<html><head>\
	<style>.bu {background-color: #d7d7d7;border: none;color: black;padding: 8px 32px;\
	text-decoration: none;font-size: 18px;margin: 4px 2px;}\
	body{font: normal 12px Verdana, Arial, sans-serif;background-color:#e6e7e8}</style>\
	<h1>AirWhere Rebooting<br><br></h1>Please reconnect to the Access Point again<br><br><a href=/>Reconnect to AirWhere home page</a></h2>" );

      delay(2000);
	  DEBUG_SERIAL_UART("[%ld] -!!!!! Restarting AirWhere !!!!!\r\n", millis());
	  ESP.restart();
}

void updateAWHardware()
{
  DEBUG_SERIAL_UART("[%ld] - Disconnecting Nmea (updateAW)\r\n", millis());
  webTime = millis() + timeOnWeb;
  airwhere_client.flush();
  airwhere_client.stop();
  socket_connected = false;

  String UpNavSW, GS = "n", hwMode, gpsBaud,WOSout,LfreqIn,bleIn,baroIn,VarioIn,serialBaudIn,radio_board,selectserial;

  for ( uint8_t i = 0; i < server.args(); i++ )
  {
      if ( server.argName ( i ) == "navsw")
      {
         UpNavSW = server.arg ( i );
      }
      if ( server.argName ( i ) == "hwMode")
      {
         hwMode = server.arg ( i );
      }
      if ( server.argName ( i ) == "gpsBaud")
      {
         gpsBaud = server.arg ( i );
      }
      if ( server.argName ( i ) == "selectwifioff")
      {
         WOSout = server.arg ( i );
      }
      if ( server.argName ( i ) == "Lfreq")
      {
         LfreqIn = server.arg ( i );
      }

      if ( server.argName ( i ) == "bleO")
      {
    	  bleIn = server.arg ( i );
      }
      if ( server.argName ( i ) == "baro")
      {
    	  baroIn = server.arg ( i );
      }
      if ( server.argName ( i ) == "vario")
      {
    	  VarioIn = server.arg ( i );
      }
      if ( server.argName ( i ) == "serialBaud")
      {
    	  serialBaudIn = server.arg ( i );
      }
      if ( server.argName ( i ) == "rfboard")
      {
         radio_board = server.arg ( i );
      }
      if ( server.argName ( i ) == "selectserial")
      {
    	  selectserial = server.arg ( i );
      }

  }


  update_configFile(UpNavSW, GS = "n", hwMode, gpsBaud,WOSout,LfreqIn,bleIn,baroIn,VarioIn,serialBaudIn,radio_board,selectserial );

  server.send ( 200, "text/html", "<html><head>\
<style>.bu {background-color: #d7d7d7;border: none;color: black;padding: 8px 32px;\
text-decoration: none;font-size: 18px;margin: 4px 2px;}\
body{font: normal 12px Verdana, Arial, sans-serif;background-color:#e6e7e8}</style>\
<h1>AirWhere Rebooting<br><br></h1>Please reconnect to the Access Point again<br><br><a href=/>Reconnect to AirWhere home page</a></h2>" );

  delay(2000);
  DEBUG_SERIAL_UART("[%ld] -!!!!! Restarting AirWhere !!!!!\r\n", millis());
  ESP.restart();
}

void updateAWVario()
{
  DEBUG_SERIAL_UART("[%ld] - Disconnecting Nmea (updateAW)\r\n", millis());
  webTime = millis() + timeOnWeb;
  airwhere_client.flush();
  airwhere_client.stop();
  socket_connected = false;

  String  GS = "n", vario_in, ct_in, st_in, zt_in,calibrate_in;

  for ( uint8_t i = 0; i < server.args(); i++ )
  {
      if ( server.argName ( i ) == "vario")
      {
    	  vario_in = server.arg ( i );
      }
      if ( server.argName ( i ) == "climbthreshold")
      {
    	  ct_in = server.arg ( i );
      }

      if ( server.argName ( i ) == "zerothreshold")
      {
    	  zt_in = server.arg ( i );
      }
      if ( server.argName ( i ) == "sinkthreshold")
      {
    	  st_in = server.arg ( i );
      }
	  if (server.argName(i) == "calibrate")
	  {
		  calibrate_in = server.arg(i);
	  }

  }

 update_configFile( GS = "n", vario_in, ct_in, zt_in, st_in, calibrate_in );

  server.send ( 200, "text/html", "<html><head>\
<style>.bu {background-color: #d7d7d7;border: none;color: black;padding: 8px 32px;\
text-decoration: none;font-size: 18px;margin: 4px 2px;}\
body{font: normal 12px Verdana, Arial, sans-serif;background-color:#e6e7e8}</style>\
<h1>AirWhere Rebooting<br><br></h1>Please reconnect to the Access Point again<br><br><a href=/>Reconnect to AirWhere home page</a></h2>" );

  delay(2000);
  DEBUG_SERIAL_UART("[%ld] -!!!!! Restarting AirWhere !!!!!\r\n", millis());
  ESP.restart();
}

void updateGS()
{
  DEBUG_SERIAL_UART("[%ld] - Disconnecting Nmea (updateGS)\r\n", millis());
  webTime = millis() + timeOnWeb;
  airwhere_client.flush();
  airwhere_client.stop();
  socket_connected = false;

  String aWiDIn, gsLatitudeSwitch, gsLongitudeSwitch, gsAltitudeS, UpSsid, UpPass, UpNavSW = "L" , GS = "y";

  for ( uint8_t i = 0; i < server.args(); i++ )
  {
    DEBUG_SERIAL_UART_S("[" + String(millis()) + "] - Server Arg :>" + server.argName (i) + "\r\n");

    if ( server.argName ( i ) == "gsLatitudeSwitch")
    {
      gsLatitudeSwitch = server.arg ( i );
    }
    if ( server.argName ( i ) == "gsLongitudeSwitch")
    {
      gsLongitudeSwitch = server.arg ( i );
    }
    if ( server.argName ( i ) == "gsAltitudeSwitch")
    {
      gsAltitudeS = server.arg ( i );
    }
    if ( server.argName ( i ) == "aWiD")
    {
    	aWiDIn = server.arg ( i );

    }
    if ( server.argName ( i ) == "aWssidSwitch")
    {
      UpSsid = server.arg ( i );
    }
    if ( server.argName ( i ) == "aWpassSwitch")
    {
      UpPass = server.arg ( i );
    }

  }
  char gsu='g';

  update_configFile(gsu, gsLatitudeSwitch, gsLongitudeSwitch, gsAltitudeS, aWiDIn, UpSsid, UpPass);

  DEBUG_SERIAL_UART("[%ld] -!!!!! Changing GS Settings !!!!!\r\n", millis());


  server.send ( 200, "text/html", "<h1>AirWhere Ground Station Rebooting with new settings  - please reconnect to the Access Point again<br><br></h1><a href=/>Reconnect to AirWhere home page</a></h2>" );
  delay(3000);

  DEBUG_SERIAL_UART("[%ld] -!!!!! Restarting AirWhere !!!!!\r\n", millis());
  ESP.restart();
}

void updateVersion()
{
  DEBUG_SERIAL_UART("[%ld] - Disconnecting Nmea (updateVersion)\r\n", millis());
  webTime = millis() + timeOnWeb;
  airwhere_client.flush();
  airwhere_client.stop();
  socket_connected = false;
  yieldDelay(500);
  yield();

  server.send ( 200, "text/html", "<!DOCTYPE html><html><style>#myProgress{position: relative;width: 100%;height: 30px;background-color: #ddd;}\
#myBar {position: absolute;width: 1%;height: 100%;background-color: #4CAF50;}\
.bu {background-color: #d7d7d7;border: none;color: black;padding: 8px 32px;\
text-decoration: none;font-size: 18px;margin: 4px 2px;}\
body{font: normal 12px Verdana, Arial, sans-serif;background-color:#e6e7e8}</style><body onload=\"move()\" width=100%>\
<script>var width = 1;function move() {var elem = document.getElementById(\"myBar\");\
var id = setInterval(frame, 350);function frame(){width++;if (width >= 100){clearInterval(id);\
document.getElementById(\"p1\").innerHTML = \"<br><br><a href=/>AirWhere Home</a>\";}\
elem.style.width = width + '%';}}</script><div>AirWhere updating to NEW VERSION - please wait - if \
the update is successful the board will automatically reboot, please then reconnect , if the update \
fails the update number will still be the same.<div id=myProgress><div id=myBar></div>\
<div align=center><p id=\"p1\"><br><br>>>>      Installing AirWhere     >>></p></div></body></html>" );

  yieldDelay(500);
  Serial.println("Starting AirWhere Update");
  flarm_update_in_process=true;
  delay(500);
  run_vario=false;
  noInterrupts();
  delay(500);
  update_airwhere(true);

}

void devVersion()
{
  DEBUG_SERIAL_UART("[%ld] - Disconnecting Nmea (devVersion)\r\n", millis());
  webTime = millis() + timeOnWeb;
  airwhere_client.flush();
  airwhere_client.stop();
  socket_connected = false;

  //  auto ret = ESPhttpUpdate.update("www.air-ware.co.uk", 80, "/downloads/AirWare.bin");
  yieldDelay(500);

  /*  server.send ( 200, "text/html", "<!DOCTYPE html><html><style>#myProgress{position: relative;width: 100%;height: 30px;background-color: #ddd;}#myBar {position: absolute;width: 1%;height: 100%;background-color: #4CAF50;}\
                 </style><body>Airware updating to DEV VERSION - please wait - if the update is successful the board will automatically reboot, please then reconnect , if the update fails the update number will still be the same.\
                 <div id=myProgress><div id=myBar></div></div><script>var width = 1;setInterval(move, 200);function move(){var elem = document.getElementById(\"myBar\");\
                 width++;var id = setInterval(frame, 10);function frame() {if (width >= 100) {clearInterval(id);} elem.style.width = width + '%';}}</script></body></html>" );
  */
  server.send ( 200, "text/html", "<!DOCTYPE html><html><style>#myProgress{position: relative;width: 100%;height: 30px;background-color: #ddd;}\
#myBar {position: absolute;width: 1%;height: 100%;background-color: #4CAF50;}</style><body onload=\"move()\" width=100%>\
<script>var width = 1;function move() {var elem = document.getElementById(\"myBar\");\
var id = setInterval(frame, 350);function frame(){width++;if (width >= 100){clearInterval(id);\
document.getElementById(\"p1\").innerHTML = \"<br><br><a href=/>AirWhere Home</a>\";}\
elem.style.width = width + '%';}}</script><div>AirWhere updating to Development VERSION - please wait - if \
the update is successful the board will automatically reboot, please then reconnect , if the update \
fails the update number will still be the same.<div id=myProgress><div id=myBar></div>\
<div align=center><p id=\"p1\"><br><br>>>>      Installing AirWhere     >>></p></div></body></html>" );

  yieldDelay(500);
  Serial.println("Starting Development Version AirWhere Update");
  delay(1000);
  flarm_update_in_process=true;
  update_airwhere(false);

}


void Web_setup()
{

  server.on ( "/", HTTP_ANY, displayWeb );

  //http://www.msftncsi.com/ncsi.txt

  server.on ( "/ncsi.txt", HTTP_ANY, []() {
    server.send ( 200, "text/html", "Microsoft NCSI" );
    DEBUG_SERIAL_UART("[%ld] - Received dodgy new firmware kobo connect\n Kobo looking for NSCI site\n Returning spoofed site.\r\n", millis());
    webTime = millis() + timeOnWeb;
    dhcp_client_connected = true;
  } );


  server.on ( "/library/test/success.html", HTTP_ANY, []() {
    server.send ( 200, "text/html", "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>" );
    DEBUG_SERIAL_UART("[%ld] - Received dodgy kobo connect , looking for APPLE site, return spoofed apple site\r\n", millis());
    webTime = millis() + timeOnWeb;
  } );


  server.on ( "/config", HTTP_ANY, configureAW );
  server.on ( "/configh", HTTP_ANY, configureAWHardware );
  server.on ( "/configv", HTTP_ANY, configureAWVario );
  server.on ( "/update", HTTP_POST, updateAW );
  server.on ( "/updateh", HTTP_POST, updateAWHardware );
  server.on ( "/updatev", HTTP_POST, updateAWVario );
  server.on ( "/updateGS", HTTP_POST, updateGS );
  server.on ( "/update", HTTP_ANY, updateAW_entered_directly );
  server.on ( "/version", HTTP_ANY, aWversion );
  server.on ( "/updateVersion", HTTP_ANY, updateVersion );
  server.on ( "/devVersion", HTTP_ANY, devVersion );
  server.on ( "/switch", HTTP_ANY, switchGS );
  server.on ( "/switchAW", HTTP_ANY, switchAW );
  //v1.1
  server.on ( "/updateflarm", HTTP_ANY, updateflarm );
  server.on ( "/processfupdate", HTTP_ANY, processfupdate );
  server.on ( "/flarmConf", HTTP_ANY, flarmConf );
  server.on ( "/stats", HTTP_ANY, aw_stats );
  //v2.11
  server.on ( "/debug", HTTP_ANY, aw_debug );


  server.onNotFound([]() {
    server.send(200, "text/html", "Success");
    DEBUG_SERIAL_UART("[%ld] - Received a Not Found request - Returning \"Success\"\r\n", millis());
    webTime = webTime + timeOnWeb;
  });

  server.begin();
  DEBUG_SERIAL_UART("[%ld] - HTTP server started\r\n", millis());

}

void Web_loop()
{
  //dnsServer.processNextRequest();
  yield();
  //DEBUG_SERIAL_UART_MAX("[%ld] - After DNS Process\r\n", millis());
  server.handleClient();
  yield();
  // DEBUG_SERIAL_UART_MAX("[%ld] - Ending Webloop\r\n", millis());
}

