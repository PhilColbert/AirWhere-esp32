#include <Arduino.h>
// ESP23extern "C" {
// ESP23#include <user_interface.h>
// ESP23}
#include "AirWare.h"
// ESP23#include <ESP8266WiFi.h>
//#include "airPKT.h"

byte getVal(char c)
{
   if(c >= '0' && c <= '9')
     return (byte)(c - '0');
   else
     return (byte)(toupper(c)-'A'+10);
}

void Hex2Bin(String str, byte *buffer)
{
  char hexdata[2 * PKT_SIZE + 1];
  
  str.toCharArray(hexdata, sizeof(hexdata));
  for(int j = 0; j < PKT_SIZE * 2 ; j+=2)
  {
    buffer[j>>1] = getVal(hexdata[j+1]) + (getVal(hexdata[j]) << 4);
  }
}

String Bin2Hex(byte *buffer)
{
  String str = "";
  for (int i=0; i < PKT_SIZE; i++) {
    byte c = buffer[i];
    str += (c < 0x10 ? "0" : "") + String(c, HEX);
  }
  return str;
}

#ifdef ESP32_CPU

#else


IPAddress getClientNumber ( int clientNo )
{
  unsigned char number_client;
  struct station_info *stat_info;

  struct ip_addr *IPaddress;
  IPAddress address;
  int i=1;

  // this is most probably the IP it will connect to.
  IPAddress returnIP(192, 168, 4, 2);
  
  number_client= wifi_softap_get_station_num(); // Count of stations which are connected to ESP8266 soft-AP
  stat_info = wifi_softap_get_station_info();

  while (stat_info != NULL)
  {
    IPaddress = &stat_info->ip;
    returnIP = IPaddress->addr;
      if (i==clientNo)
      {
        return returnIP;
      }
    
    stat_info = STAILQ_NEXT(stat_info, next);
    i++;
  }

  return returnIP; 
  
}

int client_status( )
{

  unsigned char number_client;
  struct station_info *stat_info;

  struct ip_addr *IPaddress;
  IPAddress address;
  int i=1;

  number_client= wifi_softap_get_station_num(); // Count of stations which are connected to ESP8266 soft-AP
  stat_info = wifi_softap_get_station_info();

  DEBUG_SERIAL_UART("[%ld] - Total connected_client : %d\r\n", millis(),number_client);

  while (stat_info != NULL) 
  {
    IPaddress = &stat_info->ip;
    address = IPaddress->addr;
// Needs sorting

    DEBUG_SERIAL_UART_S("[" + String(millis()) + "] - Client= " + String(i) + " ip address is = ");
  //  Serial.print((address));
   // Serial.print(" with mac adress is = ");

 //   Serial.print(stat_info->bssid[0],HEX);
 //   Serial.print(stat_info->bssid[1],HEX);
 //   Serial.print(stat_info->bssid[2],HEX);
 //   Serial.print(stat_info->bssid[3],HEX);
 //   Serial.print(stat_info->bssid[4],HEX);
  //  Serial.println(stat_info->bssid[5],HEX);

    stat_info = STAILQ_NEXT(stat_info, next);
    i++;
  }
  delay(10);
} 


#endif

#define pi 3.14159265358979323846

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  Function prototypes                                           :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double deg2rad(double);
double rad2deg(double);
#define d2r 0.0174532925199433

double haversine_km(double lat1, double long1, double lat2, double long2)
{
    double dlong = (long2 - long1) * d2r;
    double dlat = (lat2 - lat1) * d2r;
    double a = pow(sin(dlat/2.0), 2) + cos(lat1*d2r) * cos(lat2*d2r) * pow(sin(dlong/2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = 6367 * c;

    return d;
}

double distance(double lat1, double lon1, double lat2, double lon2, char unit) {
  
/*  Serial.println(lat1);
  Serial.println(lat1*100000);
  Serial.println(lon1*100000);
  Serial.println(lat2*100000);
  Serial.println( lon2*100000);

*/
   
  double theta, dist;
  theta = lon1 - lon2;
  dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
  dist = acos(dist);
  dist = rad2deg(dist);
  dist = dist * 60 * 1.1515;
  switch(unit) {
    case 'M':
      break;
    case 'K':
      dist = dist * 1.609344;
      break;
    case 'N':
      dist = dist * 0.8684;
      break;
  }




  return (dist);
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts decimal degrees to radians             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double deg2rad(double deg) {
  return (deg * pi / 180);
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts radians to decimal degrees             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double rad2deg(double rad) {
  return (rad * 180 / pi);
}


double dtorA(double fdegrees)
{
  return(fdegrees * PI / 180);
}

//Convert radians to degrees
double rtodA(double fradians)
{
  return(fradians * 180.0 / PI);
}

int CalcBearingA(double lat1, double lon1, double lat2, double lon2)
{

  
 
  lat1 = dtorA(lat1);
  lon1 = dtorA(lon1);
  lat2 = dtorA(lat2);
  lon2 = dtorA(lon2);
  
  //determine angle
  double bearing = atan2(sin(lon2-lon1)*cos(lat2), (cos(lat1)*sin(lat2))-(sin(lat1)*cos(lat2)*cos(lon2-lon1)));
  //convert to degrees
  bearing = rtodA(bearing);
  //use mod to turn -90 = 270
  //bearing = fmod((bearing + 360.0), 360);
  //return (int) bearing + 0.5;
  return ((int) bearing + 360) % 360;
}

bool isNumeric( String inS)
{

  if (inS.length()==0)
  {
    return false;
  }
  
  for (int i=0;i<inS.length();i++)
  {
    if ( !(isdigit( inS.charAt(i)) || inS.charAt(i) == '.'))
    {
      DEBUG_SERIAL_UART("[%ld] - Bad Lat/Long data\r\n", millis());
      return false;
    }
  }
  return true;
}


int isPilotNESW(char NS, char EW )
{


 if (NS=='N')
 {
  if ( EW=='E')
  {
    return 2;
  }
  else
  {
    return 1;
  }
 }
 else
 {
  if ( EW=='E')
  {
    return 4;
  }
  else
  {
    return 3;
  }
 }

}



String addDot ( String numbersIn, int dotPos)
{

  String returnS="";
#ifdef ESP32_CPU

#else



  for (int i=numbersIn.length();i--;i>0)
  {

     returnS=numbersIn.charAt(i)+returnS;
    if (i==dotPos)
    {
         returnS="."+returnS;
    }
    
  }
#endif

  return returnS;
}



String reformatLatLong ( String numbersIn, int digits)
{
  int len=digits-numbersIn.length();

  for (int i=0;i<len;i++)
  {
       numbersIn="0"+numbersIn;
  }
  
  return numbersIn;
}






  



