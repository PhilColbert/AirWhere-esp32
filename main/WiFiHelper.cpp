#include <WiFi.h>
#include "WiFiHelper.h"
#include "esp_wifi.h"

WiFiClient clientwifi;
IPAddress apIP(192, 168, 4, 1);

void WiFi_setup()
{

  WiFi.mode(WIFI_OFF);
  yieldDelay(500);
  set_Connected_Time=false;

  //v1.15
  //String uniqueSSID="AirWhere-"+ String(airWhereID);
  
  char CC[16] ;
  //uniqueSSID.toCharArray(CC,15);
  sprintf(CC,"AirWhere-%s%s",awHexManu,awHexID);
  const char* sid = CC;



  if (airWhereSsid[0]!='\0' && airWhereSsid[0]!=NULL)
  {

   //2.8 - add reconnect if we are on hard wired as we dont need the wifi for comms. 
    if (wiredDirectly == 'y' || wiredDirectly == 'o' || wiredDirectly == 'v' )
    {
    //  wifi_station_set_reconnect_policy(true);
  //    wifi_station_set_auto_connect(true);
        esp_wifi_set_auto_connect(true);
    }
    else
    {
  //    wifi_station_set_reconnect_policy(false);
  //    wifi_station_set_auto_connect(false);
    }
    WiFi.status();
    WiFi.mode(WIFI_MODE_APSTA);
    DEBUG_SERIAL_UART("[%ld] - Wifi  B status : %d\r\n", millis(),WiFi.status());

    const char* ssid     = airWhereSsid;
    const char* password = airWherePassword;
    WiFi.begin(ssid, password);
//    WiFi.begin(airWhereSsid, airWherePassword);

    int timeout=0;

    while (WiFi.status() != WL_CONNECTED) {
      yieldDelay(500);
      timeout++;
      if (debug_level>0)
      {
        Serial.printf("[%ld] - Wifi connect status : %d\r\n", millis(),WiFi.status());

        Serial.println(airWhereSsid);
        Serial.println(airWherePassword);
      }
      if (WiFi.status() == WL_NO_SSID_AVAIL)
	  {
    	  esp_wifi_set_mode(WIFI_MODE_NULL);
    	   esp_wifi_stop();
    	  // esp_wifi_deinit();
    	  // esp_wifi_init();
    	  delay(100);

		  //WiFi.mode(WIFI_OFF);
		  if (debug_level>0)
		        {
		        Serial.println("WL_NO_SSID_AVAIL - rebooting wifi");
		        }

		  WiFi.begin(ssid, password);
	//	  WiFi.begin(airWhereSsid, airWherePassword);

	  }
      if (timeout==20)
      {
        WiFi.mode(WIFI_AP);
        break;
       }
    } 
  }
  else
  {  
     WiFi.mode(WIFI_AP);
  }
  
  if (WiFi.status() == WL_CONNECTED)
  {
    webUpload=true;
    DEBUG_SERIAL_UART("[%ld] - WiFi connected - uploading to Web\r\n", millis());
  }
  else
  {
    webUpload=false;
    DEBUG_SERIAL_UART("[%ld] - Can no connect to router, disabling Web upload\r\n", millis());
  }
//(const char* ssid, const char* passphrase, int channel, int ssid_hidden, int max_connection)


  //if we are using the esp32 in wifi mode then just let it have one connection point, if not let 2 connect.
//v3
  if (wiredDirectly=='n' ||wiredDirectly=='s')
  {
		  WiFi.softAP(sid,airWhere_ap_password,rand() % 12 + 1,0,1);
  }
  else
  {
	  WiFi.softAP(sid,airWhere_ap_password,rand() % 12 + 1,0,2);
  }
  delay(100);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  yieldDelay(100);



  yieldDelay(100);


//  dnsServer.setTTL(300);
 // dnsServer.setErrorReplyCode(DNSReplyCode::ServerFailure);
//  dnsServer.start(DNS_PORT, "www.msftncsi.com", apIP);

 // struct dhcps_lease dhcp_lease;
 // IP4_ADDR(&dhcp_lease.start_ip, 192, 168, 4, 100);
 // IP4_ADDR(&dhcp_lease.end_ip, 192, 168, 4, 101);
 // wifi_softap_set_dhcps_lease(&dhcp_lease);
   
    


}



