#ifndef WIFIHELPER_H
#define WIFIHELPER_H

#include "AirWare.h"
#include "timeFunctions.h"


void WiFi_setup(void);

//#include <DNSServer.h>
//extern DNSServer dnsServer;

extern bool webUpload;
extern char airWhereSsid[32];
extern char airWherePassword[32];
extern bool set_Connected_Time;
extern int client_connected_time;
extern int myChipID;
extern char wiredDirectly;
extern int airWhereID;
extern char airWhere_ap_password[32];

//v1.15
extern char awHexID[5];
extern char awHexManu[3];

bool upLoadtoWeb( String urlToLoad);

//v5.9

extern int debug_level;

#endif /* WIFIHELPER_H */

