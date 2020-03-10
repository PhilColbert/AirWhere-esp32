#ifndef RFDATA_H
#define RFDATA_H
/* 
 *  
 *  $GPGGA - NMEA GPS 3D-fix data
 *  $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
 *  Where:
 *       GGA          Global Positioning System Fix Data
 *       123519       Fix taken at 12:35:19 UTC
 *       4807.038,N   Latitude 48 deg 07.038' N
 *       01131.000,E  Longitude 11 deg 31.000' E
 *       1            Fix quality: 0 = invalid
 *                                 1 = GPS fix (SPS)
 *                                 2 = DGPS fix
 *                                 3 = PPS fix
 *       08           Number of satellites being tracked
 *       0.9          Horizontal dilution of position
 *       545.4,M      Altitude, Meters, above mean sea level
 *       46.9,M       Height of geoid (mean sea level) above WGS84
 *                        ellipsoid
 *       (empty field) time in seconds since last DGPS update
 *       (empty field) DGPS station ID number
 *       *47          the checksum data, always begins with *

*/

class GPGGAdataLine
{
    String GPGGAs;
    //v1.1
    String pilotFixTime;
    String pilotLatitude;
    char pilotNS;
    String pilotLongitude;
    char pilotEW;
    int FixQuality;
    int noSats;
    String horizDilultion;
    int pilotAltitude;
    String unknownData;
    String heightOfGeoid;
    char dgpsUpdate;
    int dgpsID;
    String nmeaCkSumData;

    String dataWithoutCksum;

    //v1.14

  public:
    bool lat_sign_negative;
	int lat_deg;
	int lat_min;
	int lat_min_frac;
	bool lon_sign_negative;
	int lon_deg;
	int lon_min;
	int lon_min_frac;
       

    //GPGGAdataLine (){};
    GPGGAdataLine (String);
    void rePopulate(String);
    void rePopulateDecimal(int , String ,String , String );
//v1.1 change to string.
    void setFixTime(String pTF){ pilotFixTime=pTF;}
    String getFixTime(){ return pilotFixTime; }
    String getPilotLatitude(){ return pilotLatitude; };
    String getPilotLatitudeNoDot(){ return pilotLatitude.substring(0,2)+pilotLatitude.substring(3); };
    char getPilotNS(){ return pilotNS; };
    String getPilotLongitude(){ return pilotLongitude; };
    String getPilotLongitudeNoDot(){ return pilotLongitude.substring(0,3)+pilotLongitude.substring(4); };
    char getPilotEW(){ return pilotEW; };
    int getFixQuality(){ return FixQuality; };
    int getNoSats(){ return noSats; };
    String getHorizDilultion(){ return horizDilultion; };
    int getPilotAltitude(){ return pilotAltitude; };
    String getHeightOfGeoid(){ return heightOfGeoid; };
    char getDgpsUpdate(){ return dgpsUpdate; };
    int getDgpsID(){ return dgpsID; };
    String getNmeaCkSumData(){ return nmeaCkSumData; };
    String getDataWithoutCksum ()
    {
      return ( dataWithoutCksum);
    }

    bool isLineValid=false;

   // bool cksum(String data);

   private:

    void uploadVars(String );
    void uploadVarsDecimal(String );
    String getLatDecimal(String);
    String getLongDecimal(String);
    void splitLatitude(String s);
    void splitLongitude(String s);
};

void GPGGAdataLine::uploadVars(String data)
{


//	Serial.println(data);

   int commaPos1=0;

 // isLineValid=false;

   isLineValid=true;

   if (cksum(data))
   {
  //   Serial.print("cksum ok");
	   isLineValid=true;
   }
   else
   {
	 //  Serial.print("cksum fail");
   }


  GPGGAs=data.substring(0,data.indexOf(',', commaPos1));
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  //GPGGAs="$GPGGA";
  //v1.1 - remove getint
  pilotFixTime=data.substring(commaPos1,data.indexOf(',', commaPos1));
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
//v1.14
  pilotLatitude=getLatDecimal(data.substring(commaPos1,data.indexOf(',', commaPos1)));
  splitLatitude(data.substring(commaPos1,data.indexOf(',', commaPos1)));

  if (data.substring(commaPos1,data.indexOf(',', commaPos1))=="")
  {
	  isLineValid=false;
	//  Serial.print("lat empty");
	  return;

  }
  // check each digit - make sure they 0->9 or a decimal point.

  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  
  pilotNS=data.substring(commaPos1,data.indexOf(',', commaPos1)).charAt(0);
  //v1.14
  if (pilotNS=='N'){lat_sign_negative=false;}else{lat_sign_negative=true;}

  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  //v1.14
  splitLongitude(data.substring(commaPos1,data.indexOf(',', commaPos1)));
  pilotLongitude=getLongDecimal(data.substring(commaPos1,data.indexOf(',', commaPos1)));

  if (data.substring(commaPos1,data.indexOf(',', commaPos1))=="")
  {
	  isLineValid=false;
	//  Serial.print("long empty");
	  return;

  }

  // check each digit - make sure they 0->9 or a decimal point.

  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  pilotEW=data.substring(commaPos1,data.indexOf(',', commaPos1)).charAt(0);
  if (pilotEW=='E'){lon_sign_negative=false;}else{lon_sign_negative=true;}

  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  FixQuality=data.substring(commaPos1,data.indexOf(',', commaPos1)).charAt(0);
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  noSats=data.substring(commaPos1,data.indexOf(',', commaPos1)).toInt();
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  horizDilultion=data.substring(commaPos1,data.indexOf(',', commaPos1));
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  pilotAltitude=data.substring(commaPos1,data.indexOf(',', commaPos1)).toInt();
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  unknownData=data.substring(commaPos1,data.indexOf(',', commaPos1));
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  heightOfGeoid=data.substring(commaPos1,data.indexOf(',', commaPos1));
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  dgpsUpdate=data.substring(commaPos1,data.indexOf(',', commaPos1)).charAt(0);
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  dgpsID=data.substring(commaPos1,data.indexOf(',', commaPos1)).toInt();
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  nmeaCkSumData=data.substring(commaPos1,data.length());
//return with the * on it.
  dataWithoutCksum=data.substring(0,commaPos1+1);

  dataWithoutCksum.setCharAt(2,'P');
  dataWithoutCksum.setCharAt(3,'P');
  
  
 /* if ( pilotFixTime==0 ||
      pilotLatitude=="" ||
      pilotNS==' ' ||
      pilotLongitude=="" ||
      pilotEW==' ' ||
      FixQuality==0 ||
      noSats==0 ||
      horizDilultion=="" ||
      pilotAltitude==0 ||
      unknownData=="" ||
      heightOfGeoid=="" ||
      dgpsUpdate==' ' ||
      dgpsID==0 ||
      nmeaCkSumData=="")
  {
	  isLineValid=false;
  }
*/

  if ( pilotLatitude==""){ isLineValid=false; }

  // for some reason the gps sends over 0 as both lat and long decimal and alt as 0, this
  // is most likely a false line - a pilot will never be flying at 0 !

  //AW.14
  if (pilotLatitude.substring(5,9)=="000000" &&
      pilotLongitude.substring(6,10)=="000000" &&
      pilotAltitude==0 )
  {
      isLineValid=false;
  }

  // seems some gps give lat long incorrect and alt ok, so add this fix.
//AW.15
 if (pilotLatitude.substring(5,9)=="000000" &&
     pilotLongitude.substring(6,10)=="000000" )
 {
     isLineValid=false;
 }

 if (pilotLatitude.substring(0,2)=="00" &&
		 pilotLatitude.substring(3,9)=="000000" )
 {
     isLineValid=false;
 }

 if (pilotLongitude.substring(0,3)=="000" &&
		 pilotLongitude.substring(4,10)=="000000" )
 {
     isLineValid=false;
 }


 // it may happen once every blue moon, but 99 times out of 10 its a failure.
 if (pilotLatitude.substring(3,9)=="000000" )
 {
     isLineValid=false;
 }

 if ( pilotLongitude.substring(4,10)=="000000" )
 {
     isLineValid=false;
 }



 /*Serial.println(pilotLatitude);
 Serial.println(pilotLatitude.substring(0,2));
 Serial.println(pilotLatitude.substring(3,9));

 Serial.println(pilotLongitude);
 Serial.println(pilotLongitude.substring(0,3));
 Serial.println(pilotLongitude.substring(4,10));
*/

}

void GPGGAdataLine::uploadVarsDecimal(String data)
{
   int commaPos1=0;


  GPGGAs=data.substring(0,data.indexOf(',', commaPos1));
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  //GPGGAs="$GPGGA";
  
  pilotFixTime=data.substring(commaPos1,data.indexOf(',', commaPos1)).toInt();
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;

  pilotLatitude=getLatDecimal(data.substring(commaPos1,data.indexOf(',', commaPos1)));
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  
  pilotNS=data.substring(commaPos1,data.indexOf(',', commaPos1)).charAt(0);
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  
  pilotLongitude=getLongDecimal(data.substring(commaPos1,data.indexOf(',', commaPos1)));
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;

  
  pilotEW=data.substring(commaPos1,data.indexOf(',', commaPos1)).charAt(0);
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  FixQuality=data.substring(commaPos1,data.indexOf(',', commaPos1)).charAt(0);
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  noSats=data.substring(commaPos1,data.indexOf(',', commaPos1)).toInt();
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  horizDilultion=data.substring(commaPos1,data.indexOf(',', commaPos1));
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  pilotAltitude=data.substring(commaPos1,data.indexOf(',', commaPos1)).toInt();
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  unknownData=data.substring(commaPos1,data.indexOf(',', commaPos1));
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  heightOfGeoid=data.substring(commaPos1,data.indexOf(',', commaPos1));
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  dgpsUpdate=data.substring(commaPos1,data.indexOf(',', commaPos1)).charAt(0);
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  dgpsID=data.substring(commaPos1,data.indexOf(',', commaPos1)).toInt();
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  nmeaCkSumData=data.substring(commaPos1,data.length());
//return with the * on it.
  dataWithoutCksum=data.substring(0,commaPos1+1);

  dataWithoutCksum.setCharAt(2,'P');
  dataWithoutCksum.setCharAt(3,'P');
  
  
}

GPGGAdataLine::GPGGAdataLine (String data)
{
 uploadVars(data);
}

void GPGGAdataLine::rePopulate (String data)
{
  uploadVars(data);
}

void GPGGAdataLine::rePopulateDecimal(int myChipId, String gsLo,String gsLa, String gsAlt)
{

  if ( gsLo.substring(0,1)=="-" )
  {
    pilotNS='S';
    pilotLatitude=gsLo.substring(1);
  }
  else
  {
    pilotNS='N';
    pilotLatitude=gsLo.substring(0);
  }

  if ( gsLa.substring(0,1)=="-" )
  {
    pilotEW='E';
    pilotLongitude=gsLa.substring(1);
  }
  else
  {
    pilotEW='W';
    pilotLongitude=gsLa.substring(0);
  }
  
  pilotAltitude=gsAlt.toInt();
  
}


void GPGGAdataLine::splitLatitude(String s)
{

//5341.973366 - just deal with 4 decimal places

	lat_deg=s.substring(0,2).toInt();
	lat_min=s.substring(2,4).toInt();
	lat_min_frac=s.substring(5,9).toInt();
}

void GPGGAdataLine::splitLongitude(String s)
{

	lon_deg=s.substring(0,3).toInt();
	lon_min=s.substring(3,5).toInt();
	lon_min_frac=s.substring(6,10).toInt();
}

String GPGGAdataLine::getLatDecimal(String s)
{

//5341.973366 - just deal with 4 decimal places

  String sLatI=s.substring(0,2);
  String sLatI1=s.substring(2,4);
  String sLatD=s.substring(5,9);

  String sLatx=sLatI1+sLatD;
  long l=sLatx.toInt();

  long o=(l*100)/60;

 String sO=String (o);
  
  if ( o > 99999 )
  {
    sO=sO.substring(0,6);
  }
  else
  {
    if ( o > 9999 )
    {
      sO="0"+sO.substring(0,5);
    } 
    else
    {
      if ( o > 999 )
      {
        sO="00"+sO.substring(0,4);
      } 
      else
      {
        if ( o > 99 )
        {
          sO="000"+sO.substring(0,3);
        }
        else
        {
          if ( o > 9 )
          {
            sO="0000"+sO.substring(0,2);
          }
          else
          {
             sO="00000"+sO.substring(0,1);
          }
        }
      }
    }
  }
  return sLatI+"."+sO;
  
}

String GPGGAdataLine::getLongDecimal(String s)
{

//00215.726591

  String sLatI=s.substring(0,3);
  String sLatI1=s.substring(3,5);
  String sLatD=s.substring(6,10);

  String sLatx=sLatI1+sLatD;
  long l=sLatx.toInt();

  long o=(l*100)/60;

  //need to add leading Zeros to return string.

  String sO=String (o);
  
  if ( o > 99999 )
  {
    sO=sO.substring(0,6);
  }
  else
  {
    if ( o > 9999 )
    {
      sO="0"+sO.substring(0,5);
    } 
    else
    {
      if ( o > 999 )
      {
        sO="00"+sO.substring(0,4);
      } 
      else
      {
        if ( o > 99 )
        {
          sO="000"+sO.substring(0,3);
        }
        else
        {
          if ( o > 9 )
          {
            sO="0000"+sO.substring(0,2);
          }
          else
          {
             sO="00000"+sO.substring(0,1);
          }
        }
      }
    }
  }
  return sLatI+"."+sO;
}

/*bool GPGGAdataLine::cksum(String data)
{

   char UDPpacketBuffer[150];

   int p=data.length();

   unsigned char cs = 0;
   data.toCharArray(UDPpacketBuffer,p);

   //start at 1 to jump the $ and  remove the 3 digits at the end for the cksum.

   cs = 0; //clear any old checksum

   unsigned int c=0;

   char cksum_value[3];

   for (unsigned int n = 1; n < strlen(UDPpacketBuffer); n++)
   {
	   if (n < strlen(UDPpacketBuffer) - 3)
	   {
	  // Serial1.println(UDPpacketBuffer[n]);
         cs ^= UDPpacketBuffer[n]; //calculates the checksum
	   }
	   else
	   {
		 if (n != (strlen(UDPpacketBuffer) -3 )) // on the *
	     {
			 // check its a digit - if not line is false;
		   cksum_value[c]=UDPpacketBuffer[n];

		   if (!isalnum (cksum_value[c]))
		   { Serial1.println("fail digit");
			   return false;
		   }
		  // Serial1.println(UDPpacketBuffer[n]);
		   c++;
	     }
	   }

   }

 //  Serial1.println("cs");Serial1.println(cs);

   cksum_value[2]='\0';
   //unsigned char number = (char)strtol(cksum_value, NULL, 16);

   if (cs==(char)strtol(cksum_value, NULL, 16))
   {
	   return true;
   }
   else
   {
	   Serial1.println("fail cksum");
	   return false;
   }

}
*/

/*
$GPRMC,194350.000,A,5341.9792,N,00215.7123,W,0.22,162.94,290216,,,A*77
Where:
     RMC          Recommended Minimum sentence C
     123519       Fix taken at 12:35:19 UTC
     A            Status A=active or V=Void.
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     022.4        Speed over the ground in knots
     084.4        Track angle in degrees True
     230394       Date - 23rd of March 1994
     003.1,W      Magnetic Variation
     *6A          The checksum data, always begins with *

*/

class GPRMCdataLine
{
    int pilotFixTime;
    char gpsActive;    
    String pilotLatitude;
    char pilotNS;
    String pilotLongitude;
    char pilotEW;
    int pilotSpeed;
    int pilotTrack;
    String pilotDate;
    String magnetVar;
    String unknownVar;
    String nmeaCkSumData;

       
  public:
    //GPGGAdataLine (){};
    GPRMCdataLine (String);
    void rePopulate(String);

    int getFixTime(){ return pilotFixTime; }
    char getGpsActive(){ return gpsActive; };
    void setGpsActive(){  gpsActive='A'; };
    String getPilotLatitude(){ return pilotLatitude; };
    char getPilotNS(){ return pilotNS; };
    String getPilotLongitude(){ return pilotLongitude; };
    char getPilotEW(){ return pilotEW; };
    int getPilotSpeed(){ return pilotSpeed; };
    int getPilotTrack(){ return pilotTrack; };
    String getPilotDate(){ return pilotDate; };
    String getMagnetVar(){ return magnetVar; };
    String getUnknownVar(){ return unknownVar; };
    String getNmeaCkSumData(){ return nmeaCkSumData; };

    bool isLineValid=false;
//$GPRMC,180121.000,A,5341.9846,N,00215.7209,W,0.87,193.95,030316,,,A*72<-

   private:

   void uploadVars(String );
   

};

void GPRMCdataLine::uploadVars(String data)
{
   int commaPos1=0;
  //int currentChar=0;

//  Serial.println(data);
  isLineValid=false;
  if (cksum(data))
  {
	   isLineValid=true;


  }

  else
  {
      if (debug_level>0)
        {

        		Serial.println("GPRMCdataLine cksum FAILED");

        }
    return;
  }

  String GPGGAs=data.substring(0,data.indexOf(',', commaPos1));
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  pilotFixTime=data.substring(commaPos1,data.indexOf(',', commaPos1)).toInt();
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  gpsActive=data.substring(commaPos1,data.indexOf(',', commaPos1)).charAt(0);
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  pilotLatitude=data.substring(commaPos1,data.indexOf(',', commaPos1));
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  pilotNS=data.substring(commaPos1,data.indexOf(',', commaPos1)).charAt(0);
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  pilotLongitude=data.substring(commaPos1,data.indexOf(',', commaPos1));
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  pilotEW=data.substring(commaPos1,data.indexOf(',', commaPos1)).charAt(0);
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  pilotSpeed=data.substring(commaPos1,data.indexOf(',', commaPos1)).toInt();
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  pilotTrack=data.substring(commaPos1,data.indexOf(',', commaPos1)).toInt();
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  pilotDate=data.substring(commaPos1,data.indexOf(',', commaPos1));
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  magnetVar=data.substring(commaPos1,data.indexOf(',', commaPos1));
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;
  unknownVar=data.substring(commaPos1,data.indexOf(',', commaPos1));
  commaPos1=data.indexOf(',', commaPos1);
  commaPos1++;  
  nmeaCkSumData=data.substring(commaPos1,data.length());
  
  }

GPRMCdataLine::GPRMCdataLine (String data)
{
 uploadVars(data);
}

void GPRMCdataLine::rePopulate (String data)
{
  uploadVars(data);
}


#endif //RFDATA_H



 


