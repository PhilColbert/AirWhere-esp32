
#include "fileOps.h"

  //  const char *base_path = "/spiflash";
  //  const char *storage_path = "storage";
  //  const char *aw_file = "/spiflash/airwhere.cfg";

static const char *TAG = "FileOPS";
//static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
 //Mount path for the partition
//FILE *f;

Preferences airwhere_preferences;


int gpsSelectToBaud(char sel)
{

  if (sel=='9')
  {
    return 9600;
  }
  if (sel=='2')
  {
    return 19200;
  }
  if (sel=='3')
  {
    return 38400;
  }
  if (sel=='5')
  {
    return 57600;
  }
  if (sel=='1')
  {
    return 115200;
  }

  return 9600;

}

char gpsBaudtoChar(long sel)
{

  if (sel==9600)
  {
    return '9';
  }
  if (sel==19200)
  {
    return '2';
  }
  if (sel==38400)
  {
    return '3';
  }
  if (sel==57600)
  {
    return '5';
  }
  if (sel==115200)
  {
    return '1';
  }
  
  return 9;

}

void update_actual_file(String UpSsid,String UpPass,String UpNavSW, String GS, String hwMode, String gpsBaud,
        String WOSin, String AwIDin, String awPilotNameIn, String aWappassIn, String LfreqIn,
		String gsLa, String gsLo, String gsA, String rcF, String baro_in, String vario_setting,
		String airC_type_in,String serialBaud, String ble_out, String climb_thresh, String zero_thresh,
		String sink_thresh, String calibrate, String radB, String ognID_in, String ogn_on_off_in, String packet_select, String debug_setting, String selectserial  )
{

      noInterrupts();

      delay(500);

	  ESP_LOGI(TAG, "Opening Config File");

	  airwhere_preferences.begin("airwhere", false);

	  airwhere_preferences.putChar("f",'y');
	  airwhere_preferences.putString("ssid",UpSsid);
	  airwhere_preferences.putString("pass",UpPass);
	  airwhere_preferences.putChar("navS",UpNavSW.charAt(0));
	  airwhere_preferences.putChar("flrm",'n');
	  airwhere_preferences.putChar("gs",GS.charAt(0));
	  airwhere_preferences.putChar("hw",hwMode.charAt(0));
	  airwhere_preferences.putString("lat", gsLa );
	  airwhere_preferences.putString("long", gsLo );
	  airwhere_preferences.putString("alt", gsA);
	  airwhere_preferences.putChar("gpsB", gpsBaud.charAt(0));
	  airwhere_preferences.putChar("wo", WOSin.charAt(0) );
	  airwhere_preferences.putString ("awid",AwIDin);
	  airwhere_preferences.putString( "awpn",awPilotNameIn );
	  airwhere_preferences.putString ("awap", aWappassIn);
	  airwhere_preferences.putChar("freq", LfreqIn.charAt(0));
	  airwhere_preferences.putChar("baro", baro_in.charAt(0));
	  airwhere_preferences.putChar("vario_on", vario_setting.charAt(0) );
	  airwhere_preferences.putChar("aT", airC_type_in.charAt(0));
	  airwhere_preferences.putChar("sB",serialBaud.charAt(0));
	  airwhere_preferences.putChar("ble",ble_out.charAt(0));
	  airwhere_preferences.putInt("climb",climb_thresh.toInt());
	  airwhere_preferences.putInt("zero",zero_thresh.toInt());
	  airwhere_preferences.putInt("sink", sink_thresh.toInt() );
	  airwhere_preferences.putChar("calibrate", calibrate.charAt(0));
	  airwhere_preferences.putChar("radB", radB.charAt(0));
	  airwhere_preferences.putChar("psel", packet_select.charAt(0));
	  airwhere_preferences.putString("ognid", ognID_in);
	  airwhere_preferences.putChar("ognoo", ogn_on_off_in.charAt(0));
	  airwhere_preferences.putInt("dbug", debug_setting.toInt());
//v5
	  airwhere_preferences.putInt("buzzer", turn_on_off_buzzer);
	  airwhere_preferences.putChar("vsoundsonoff", vario_sounds_on);

//v5.95

	  airwhere_preferences.putChar("selectserial", selectserial.charAt(0));
	/*
	  fprintf(f, "ssid=%s\n",UpSsid.c_str() );
	  fprintf(f, "pass=%s\n",UpPass.c_str());
	  fprintf(f, "navS=%s\n",UpNavSW.c_str());
	  fprintf(f, "flrm=n\n");
	  fprintf(f, "gs=%s\n",GS.c_str());
	  fprintf(f, "hw=%s\n",hwMode.c_str());
	  fprintf(f, "lat=%s\n",gsLa.c_str());
	  fprintf(f, "long=%s\n",gsLo.c_str());
	  fprintf(f, "alt=%s\n",gsA.c_str());
	  fprintf(f, "gpsB=%s\n",gpsBaud.c_str());
	  fprintf(f, "wo=%s\n",WOSin.c_str());
	  fprintf(f, "awid=%s\n",AwIDin.c_str());
	  fprintf(f, "awpn=%s\n",awPilotNameIn.c_str());
	  fprintf(f, "awap=%s\n",aWappassIn.c_str());
	  fprintf(f, "freq=%s\n",LfreqIn.c_str());
	  fprintf(f, "baro=%s\n",baro_in.c_str());
	  fprintf(f, "vario=%s\n",vario_setting.c_str());
	  fprintf(f, "aT=%s\n",airC_type_in.c_str());
      fprintf(f, "sB=%s\n",serialBaud.c_str());
      fprintf(f, "ble=%s\n",ble_out.c_str());
      fprintf(f, "climb=%s\n",climb_thresh.c_str());
      fprintf(f, "zero=%s\n",zero_thresh.c_str());
      fprintf(f, "sink=%s\n",sink_thresh.c_str());
	  fprintf(f, "calibrate=%s\n", calibrate.c_str());
	  fprintf(f, "radB=%s\n", radB.c_str());
	  fprintf(f, "ognid=%s\n", ognID_in.c_str());
	  fprintf(f, "ognoo=%s\n", ogn_on_off_in.c_str());
	  fprintf(f, "psel=%s\n", packet_select.c_str());
	  fprintf(f, "dbug=%s\n", debug_setting.c_str());

	  fclose(f);
	  */

	  airwhere_preferences.end();

	  ESP_LOGI(TAG, "File written");
	  delay(500);

	  interrupts();
}

void load_configFile()
{
// add file loaded ok.

	//Serial.begin(115200);




	Serial.println("LOAD CONFIG FILE");

	airwhere_preferences.begin("airwhere", false);

    if (airwhere_preferences.getChar("f",'n') == 'y')
	{

		String ssid=airwhere_preferences.getString("ssid","");
		ssid.toCharArray(airWhereSsid,ssid.length()+1);
		Serial.println("airWhereSsid=" + String(airWhereSsid));

		String pw=airwhere_preferences.getString("pass","");
		pw.toCharArray(airWherePassword,pw.length()+1);
		Serial.println("airWherePassword=" + String(airWherePassword));

		navSofware=airwhere_preferences.getChar("navS",'L');
		Serial.println("navSofware=" + String(navSofware));

		receiveFlarm=airwhere_preferences.getChar("flrm",'n');
		Serial.println("receiveFlarm=" + String(receiveFlarm));

		groundStationMode=airwhere_preferences.getChar("gs",'n');
		Serial.println("groundStationMode=" + String(groundStationMode));

		wiredDirectly=airwhere_preferences.getChar("hw",'o');
		Serial.println("wiredDirectly=" + String(wiredDirectly));

		String lat=airwhere_preferences.getString("lat", "00.0000000" );
		lat.toCharArray(gsLatitude,lat.length()+1);
		Serial.println("gsLatitude=" + String(gsLatitude));

		String lon=airwhere_preferences.getString("long", "000.0000000" );
		lon.toCharArray(gsLongitude,lon.length()+1);
		Serial.println("gsLongitude=" + String(gsLongitude));

		String alt=airwhere_preferences.getString("alt", "0");
		alt.toCharArray(gsAltitude,alt.length()+1);
		Serial.println("gsAltitude=" + String(gsAltitude));

		gpsBaudRate=gpsSelectToBaud(airwhere_preferences.getChar("gpsB", 'n'));
		Serial.println("gpsBaudRate=" + String(gpsBaudRate));

		wifioffSelected=airwhere_preferences.getChar("wo", 'n' );
		Serial.println("wifioffSelected=" + String(wifioffSelected));

		String awid=airwhere_preferences.getString ("awid", String ( awHexID));
		awid.toCharArray(awHexID,awid.length()+1);
		Serial.println("awHexID=" + String(awHexID));

		String awpn=airwhere_preferences.getString( "awpn", "" );
		awpn.toCharArray(awPilotName,awpn.length()+1);
		Serial.println("awPilotName=" + String(awPilotName));

		String awap=airwhere_preferences.getString ("awap", "12345678");
		awap.toCharArray(airWhere_ap_password,awap.length()+1);
		Serial.println("airWhere_ap_password=" + String(airWhere_ap_password));

		loraFrequency=airwhere_preferences.getChar("freq", '8');
		Serial.println("loraFrequency=" + String(loraFrequency));

		ms5611_attached=airwhere_preferences.getChar("baro", 'n');
		Serial.println("ms5611_attached=" + String(ms5611_attached));

		vario_on=airwhere_preferences.getChar("vario_on", 'n' );
		Serial.println("vario_on=" + String(vario_on));

		web_aircraft_type= airwhere_preferences.getChar("aT", '1');
		Serial.println("web_aircraft_type=" + String(web_aircraft_type));

		serialBaudRate=gpsSelectToBaud(airwhere_preferences.getChar("sB",'1'));
		Serial.println("serialBaudRate=" + String(serialBaudRate));

		ble_output=airwhere_preferences.getChar("ble",'n');
		Serial.println("ble_output=" + String(ble_output));

		climb_threshold=airwhere_preferences.getInt("climb",10);
		Serial.println("climb_threshold=" + String(climb_threshold));

		zero_threshold=airwhere_preferences.getInt("zero",100);
		Serial.println("zero_threshold=" + String(zero_threshold));

		sink_threshold=airwhere_preferences.getInt("sink", -200 );
		Serial.println("sink_threshold=" + String(sink_threshold));

		calibrate_vario= airwhere_preferences.getChar("calibrate", 'y');
		Serial.println("calibrate_vario=" + String(calibrate_vario));

		lora_or_flarm = airwhere_preferences.getChar("radB", 'l');
		Serial.println("lora_or_flarm=" + String(lora_or_flarm));

		packet_repeat= airwhere_preferences.getChar("psel", 'n');
		Serial.println("packet_repeat=" + String(packet_repeat));

		String ognidS = airwhere_preferences.getString("ognid", "");
		ognidS.toCharArray(ogn_id,ognidS.length()+1);
		Serial.println("ogn_id=" + String(ogn_id));

		ogn_on=airwhere_preferences.getChar("ognoo", 'n');
		Serial.println("ogn_on=" + String(ogn_on));

		debug_level=airwhere_preferences.getInt("dbug", 0);
		Serial.println("debug_level=" + String(debug_level));

//v5

		turn_on_off_buzzer=airwhere_preferences.getInt("buzzer", 2);
		Serial.println("turn_on_off_buzzer=" + String(turn_on_off_buzzer));
		// if buzzer is set to On (1) then as we are starting up, use 2 as this is the setting for switching it on
		// when the system detects movement.
		if (turn_on_off_buzzer==1)
		{
			turn_on_off_buzzer=2;
		}

		vario_sounds_on=airwhere_preferences.getChar("vsoundsonoff", 'y');
		Serial.println("vsoundsonoff=" + String(vario_sounds_on));

		//5.95

		serial_type_out=airwhere_preferences.getChar("selectserial", 'a');
		Serial.println("serial_type_out=" + String(serial_type_out));
	}
	else
	{

		airwhere_preferences.putString("ssid",""); //(f, "ssid=\n");
		airwhere_preferences.putString("pass",""); //fprintf(f, "pass=\n");
		airwhere_preferences.putChar("navS",'L'); //fprintf(f, "navS=L\n"); //LK
		airwhere_preferences.putChar("flrm",'n'); // 	    fprintf(f, "flrm=n\n"); //do not receive flarm
		airwhere_preferences.putChar("gs",'n'); //fprintf(f, "gs=n\n"); // ground station is not set by default
		airwhere_preferences.putChar("hw",'o');//	    fprintf(f, "hw=o\n"); // hard wired not set by default
		airwhere_preferences.putString("lat", "00.0000000" ); //fprintf(f, "lat=00.0000000\n");
		airwhere_preferences.putString("long", "000.0000000" );  //fprintf(f, "long=000.0000000\n");
		airwhere_preferences.putString("alt", "0");  //fprintf(f, "alt=0\n");
		airwhere_preferences.putChar("gpsB", 'n'); // fprintf(f, "gpsB=9\n");
		airwhere_preferences.putChar("wo", 'n' ); // fprintf(f, "wo=n\n");
		airwhere_preferences.putString ("awid", String ( awHexID)); //  fprintf(f, "awid=%s\n",awHexID);
		airwhere_preferences.putString( "awpn", "" ); //fprintf(f, "awpn=\n");
		airwhere_preferences.putString ("awap", "12345678"); //fprintf(f, "awap=12345678\n");
		airwhere_preferences.putChar("freq", '8');  // fprintf(f, "freq=8\n");
		airwhere_preferences.putChar("baro", 'n'); //fprintf(f, "baro=n\n");
		airwhere_preferences.putChar("vario_on", 'n' ); //fprintf(f, "vario_on=n\n");
		airwhere_preferences.putChar("aT", '1'); //"fprintf(f, "aT=1\n");
		airwhere_preferences.putChar("sB",'1'); //fprintf(f, "sB=1\n");
		airwhere_preferences.putChar("ble",'n'); // fprintf(f, "ble=n\n");
		airwhere_preferences.putInt("climb",10); //fprintf(f, "climb=10\n");
		airwhere_preferences.putInt("zero",100); // fprintf(f, "zero=-50\n");
		airwhere_preferences.putInt("sink", -200 ); //fprintf(f, "sink=-200\n");
		airwhere_preferences.putChar("calibrate", 'y'); //fprintf(f, "calibrate=y\n");
		airwhere_preferences.putChar("radB", 'l'); //fprintf(f, "radB=l\n");
		airwhere_preferences.putChar("psel", 'n'); //fprintf(f, "psel=n\n");
		airwhere_preferences.putString("ognid", ""); //fprintf(f, "ognid=\n");
		airwhere_preferences.putChar("ognoo", 'n'); //fprintf(f, "ognoo=n\n");
		airwhere_preferences.putInt("dbug", 0); //fprintf(f, "dbug=0\n");
		airwhere_preferences.putChar("vsoundsonoff", 'y');
		airwhere_preferences.putInt("turn_on_off_buzzer", 2);

		//5.95
		airwhere_preferences.getChar("selectserial", 'a');


		airwhere_preferences.putChar("f",'y');

	}

    airwhere_preferences.end();


}


/*
void load_configFile()
{


    ESP_LOGI(TAG, "Mounting FAT filesystem - load_configFile");
    // To mount device we need name of device partition, define base_path
    // and allow format partition in case if it is new one and was not formated before
    const esp_vfs_fat_mount_config_t mount_config = {
            .format_if_mount_failed = true,
			.max_files = 12
    };

    esp_err_t err = esp_vfs_fat_spiflash_mount(base_path, storage_path, &mount_config, &s_wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (0x%x)", err);
        return;
    }

    struct stat st;
    if (stat(aw_file, &st) == 0)
    {
    	ESP_LOGI(TAG, "File exists - reading");

        // Open file for reading
        ESP_LOGI(TAG, "Reading file");
        f = fopen(aw_file, "rb");
        if (f == NULL)
        {
            ESP_LOGE(TAG, "Failed to open file for reading");
            return;
        }

        char cline[32];
        while (fgets(cline, sizeof (cline), f) != NULL)
        {
        	ESP_LOGI(TAG, "Line :- %s", cline);

       	    String line=String(cline);

            if (line.substring(0,4)=="ssid")
             {

               String ssid=line.substring(5);
               int len=line.length()-5;
               ssid.toCharArray(airWhereSsid,len);
             }

             if (line.substring(0,4)=="pass")
             {
               String pw=line.substring(5);
               int len=line.length()-5;
               pw.toCharArray(airWherePassword,len);
             }
             if (line.substring(0,4)=="navS")
             {
               navSofware=line.charAt(5);
             }
             if (line.substring(0,4)=="flrm")
             {
               receiveFlarm=line.charAt(5);
             }
             if (line.substring(0,2)=="gs")
             {
               groundStationMode=line.charAt(3);
             }
             if (line.substring(0,2)=="hw")
             {
               wiredDirectly=line.charAt(3);
             }

             if (line.substring(0,3)=="lat")
             {
               String ls=line.substring(4);
               if (ls.equals(""))
               {
            	   strcpy( gsLatitude,"0");
               }
               else
               {
                   ls.toCharArray(gsLatitude,ls.length());
               }
             }
             if (line.substring(0,4)=="long")
             {
               String ls=line.substring(5);
               if (ls.equals(""))
               {
            	   strcpy( gsLongitude,"0");
               }
               else
               {
            	   ls.toCharArray(gsLongitude,ls.length());
               }
             }
             if (line.substring(0,3)=="alt")
             {
               String ls=line.substring(4);
               ls.toCharArray(gsAltitude,ls.length());
             }

             if (line.substring(0,2)=="wo")
             {
               wifioffSelected=line.charAt(3);
             }
             if (line.substring(0,4)=="awid")
             {
               //v1.15
            	 String ls=line.substring(5);
                 ls.toCharArray(awHexID,ls.length());
                 // just to sort any issues with older setups.
                 awHexID[4]='\0';
              // airWhereID=line.substring(5).toInt();
             }

             if (line.substring(0,4)=="awpn")
             {
               String ls=line.substring(5);
               ls.toCharArray(awPilotName,ls.length());
             }
             if (line.substring(0,4)=="awap")
             {
               String ls=line.substring(5);
               ls.toCharArray(airWhere_ap_password,ls.length());
       //        Serial.println(airWhere_ap_password);
             }
             if (line.substring(0,4)=="freq")
             {
               loraFrequency=line.charAt(5);
             }
             if (line.substring(0,4)=="gpsB")
             {
                gpsBaudRate=gpsSelectToBaud(line.charAt(5));
             }
             if (line.substring(0,4)=="baro")
             {
            	 ms5611_attached=line.charAt(5);
             }
             if (line.substring(0,5)=="vario")
             {
            	 vario_on=line.charAt(6);
             }
             if (line.substring(0,2)=="aT")
             {
            	 web_aircraft_type=line.charAt(3);
             }
             if (line.substring(0,2)=="sB")
             {
            	 serialBaudRate=gpsSelectToBaud(line.charAt(3));
             }
             if (line.substring(0,3)=="ble")
             {
            	 ble_output=line.charAt(4);
             }
             if (line.substring(0,5)=="climb")
             {
            	 climb_threshold=line.substring(6).toInt();
             }
             if (line.substring(0,4)=="zero")
             {
            	 zero_threshold=line.substring(5).toInt();
             }

             if (line.substring(0,4)=="sink")
             {
            	 sink_threshold=line.substring(5).toInt();
             }

			 if (line.substring(0, 9) == "calibrate")
			 {
				 calibrate_vario = line.charAt(10);
			 }
		     if (line.substring(0,4)=="radB")
		     {
		    	 lora_or_flarm=line.charAt(5);
		     }
		     if (line.substring(0,5)=="ognid")
		     {

		    	 String ogn_idS=line.substring(6);
		    	 int len=line.length()-6;
		    	 ogn_idS.toCharArray(ogn_id,len);

		     }
		     if (line.substring(0,5)=="ognoo")
		     {
		    	 ogn_on=line.charAt(6);
		     }
		     if (line.substring(0,4)=="psel")
		     {
		    	 packet_repeat=line.charAt(5);
		     }
		     if (line.substring(0,4)=="dbug")
		     {
		    	 debug_level=line.substring(5).toInt();

		    	 //ESP_LOGI(TAG, "d :- %d", debug_level);

		    //	 Serial.println(line.substring(5,6));
		    //	 Serial.println(debug_level);
		    //	 delay(1000);

		    	//
		     }

           }
    }
    else
    {
    	ESP_LOGI(TAG, "Opening file2");
    	f = fopen(aw_file, "wb");

    	if (f == NULL) {
    	        ESP_LOGE(TAG, "Failed to open file for writing");
    	        return;
    	    }

    	    fprintf(f, "ssid=\n");
    	    fprintf(f, "pass=\n");
    	    fprintf(f, "navS=L\n"); //LK
    	    fprintf(f, "flrm=n\n"); //do not receive flarm
    	    fprintf(f, "gs=n\n"); // ground station is not set by default
    	    fprintf(f, "hw=o\n"); // hard wired not set by default
    	    fprintf(f, "lat=00.0000000\n");
    	    fprintf(f, "long=000.0000000\n");
    	    fprintf(f, "alt=0\n");
    	    fprintf(f, "gpsB=9\n");
    	    fprintf(f, "wo=n\n");
    	    fprintf(f, "awid=%s\n",awHexID);
    	    fprintf(f, "awpn=\n");
    	    fprintf(f, "awap=12345678\n");
    	    fprintf(f, "freq=8\n");
    	    fprintf(f, "baro=n\n");
    	    fprintf(f, "vario_on=n\n");
    	    fprintf(f, "aT=1\n");
    	    fprintf(f, "sB=1\n");
    	    fprintf(f, "ble=n\n");
    	    fprintf(f, "climb=10\n");
    	    fprintf(f, "zero=-50\n");
    	    fprintf(f, "sink=-200\n");
			fprintf(f, "calibrate=y\n");
			fprintf(f, "radB=l\n");
			fprintf(f, "psel=n\n");
			fprintf(f, "ognid=\n");
			fprintf(f, "ognoo=n\n");
			fprintf(f, "psel=n\n");
			fprintf(f, "dbug=0\n");

    	    fclose(f);
    	    ESP_LOGI(TAG, "File written");

    }

// NEED TO UMOUNT before restart.
    // Unmount FATFS
 //   ESP_LOGI(TAG, "Unmounting FAT filesystem");
  //  esp_err_t  esp_vfs_fat_spiflash_unmount(const char* base_path, wl_handle_t wl_handle);


}

*/

// using operator overloading to choose the right function.
//update single value

void update_configFile(String param, String value_is)
{
	noInterrupts();
	delay(500);
	airwhere_preferences.begin("airwhere", false);

	//airwhere_preferences.putString(param,value_is);

	airwhere_preferences.end();
	delay(500);
	interrupts();
}

void update_configFile(String param, int value_is)
{
	noInterrupts();
	delay(500);
	airwhere_preferences.begin("airwhere", false);
//const c*=param.toCharArray();
//const char*="1";

//	airwhere_preferences.putInt(,value_is);

	airwhere_preferences.end();
	delay(500);
	interrupts();

}

void update_configFile(String param, char value_is)
{
	noInterrupts();
	delay(500);
	airwhere_preferences.begin("airwhere", false);
	//airwhere_preferences.putChar(param,value_is);

	airwhere_preferences.end();
	delay(500);
	interrupts();
}


// updating config pilot info.

void update_configFile(String UpSsid, String UpPass, String GS, String AwIDin,
		               String asPilotNameIn, String aWappassIn, String aircraft_type_in,
					   String ognID_in, String ogn_on_off_in ,String packetsel)
{
	  update_actual_file( UpSsid,
			              UpPass,
						  String(navSofware),
						  GS,
						  String(wiredDirectly),
						  String(gpsBaudtoChar(gpsBaudRate)),
						  String (wifioffSelected),
						  AwIDin,
						  asPilotNameIn,
						  aWappassIn,
						  String(loraFrequency),
						  String(gsLatitude),
						  String(gsLongitude),
						  String(gsAltitude),
						  String(receiveFlarm),
						  String(ms5611_attached),
						  String(vario_on),
						  aircraft_type_in,
						  String(serialBaudRate),
						  String(ble_output),
						  String(climb_threshold),
						  String( zero_threshold),
						  String(sink_threshold),
						  String(calibrate_vario),
						  String(lora_or_flarm),
						  ognID_in,
						  ogn_on_off_in,
						  packetsel,
						  String(debug_level),
						  String(serial_type_out)
                       	  );


	  ESP_LOGI(TAG, "update_configFile - UPdateAW - Completed.");
}
//updating hardware config

void  update_configFile(String UpNavSW, String GSIn, String hwMode, String gpsBaud,String WOSout,String LfreqIn,
		                String bleIn,String baroIn,String VarioIn,String serialBaudIn, String radio_board , String selectserial)
{

	  update_actual_file( String(airWhereSsid),
        	              String(airWherePassword),
						  UpNavSW,
						  GSIn,
						  hwMode,
						  gpsBaud,
						  WOSout,
						  String (awHexID),
						  String (awPilotName),
						  String (airWhere_ap_password),
						  LfreqIn,
						  String(gsLatitude),
						  String(gsLongitude),
						  String(gsAltitude),
						  String(receiveFlarm),
						  baroIn,
						  VarioIn,
						  String(web_aircraft_type),
						  serialBaudIn,
						  bleIn,
						  String(climb_threshold),
						  String( zero_threshold),
						  String(sink_threshold),
						  String(calibrate_vario),
						  radio_board,
						  String (ogn_id),
						  String (ogn_on),
                          String (packet_repeat),
						  String(debug_level),
						  selectserial
						                         	  );


	  ESP_LOGI(TAG, "update_configFile - UPdateAW - Completed.");
}

//updating vario config

void update_configFile( String GS, String vario_in,String  ct_in,String  zt_in, String st_in, String calibrate )
{
	  update_actual_file( String(airWhereSsid),
        	  String(airWherePassword),
			  String(navSofware),
			  GS,
			  String(wiredDirectly),
			  String(gpsBaudtoChar(gpsBaudRate)),
			  String (wifioffSelected),
			  String (awHexID),
			  String (awPilotName),
			  String (airWhere_ap_password),
			  String(loraFrequency),
			  String(gsLatitude),
			  String(gsLongitude),
			  String(gsAltitude),
			  String(receiveFlarm),
			  String(ms5611_attached),
			  vario_in,
			  String(web_aircraft_type),
			  String(serialBaudRate),
			  String(ble_output),
			  ct_in,
			  zt_in,
			  st_in,
		      calibrate,
			  String(lora_or_flarm),
			  String (ogn_id),
			  String (ogn_on),
              String (packet_repeat),
			  String(debug_level),
			  String(serial_type_out)

			  );


	  ESP_LOGI(TAG, "update_configFile - UPdateAW - Completed.");
}

//updating ground station config

void update_configFile(char gs, String gsLatitudeSwitch, String  gsLongitudeSwitch,String  gsAltitudeS,String  awIDIn, String  UpSsid,String  UpPass)
{
	  update_actual_file( UpSsid,
			  UpPass,
			  String(navSofware),
			  String(groundStationMode),
			  String(wiredDirectly),
			  String(gpsBaudtoChar(gpsBaudRate)),
			  String (wifioffSelected),
			  awIDIn,
			  String (awPilotName),
			  String (airWhere_ap_password),
			  String(loraFrequency),
			  gsLatitudeSwitch,
			  gsLongitudeSwitch,
			  gsAltitudeS,
			  String(receiveFlarm),
			  String(ms5611_attached),
			  String(vario_on),
			  String(web_aircraft_type),
			  String(serialBaudRate),
			  String(ble_output),
			  String(climb_threshold),
			  String( zero_threshold),
			  String(sink_threshold),
			  String(calibrate_vario),
			  String(lora_or_flarm),
			  String (ogn_id),
			  String (ogn_on),
              String (packet_repeat),
			  String(debug_level),
			  String(serial_type_out)
			  );


	  ESP_LOGI(TAG, "update_configFile - GS DATA - Completed.");
	  delay(1000);
}



void update_configFile(String UpSsid,String UpPass,String UpNavSW, String GS, String hwMode, String gpsBaud,
		               String WOSin, String AwIDin, String awPilotNameIn, String aWappassIn, String LfreqIn, String baro_in, String vario_setting )
{


  String gsLa=String(gsLatitude);
  String gsLo=String(gsLongitude);
  String gsA=String(gsAltitude);
  String rcF=String(receiveFlarm);

  ESP_LOGI(TAG, "update_configFile - UPdateAW");

  update_actual_file( UpSsid, UpPass, UpNavSW,  GS,  hwMode,  gpsBaud,
           WOSin,  AwIDin,  awPilotNameIn,  aWappassIn,  LfreqIn,
  	       gsLa,  gsLo,  gsA,  rcF, baro_in, vario_setting,
		   String(web_aircraft_type),
		   						  String(serialBaudRate),
		   						  String(ble_output),
		   						  String(climb_threshold),
		   						  String( zero_threshold),
		   						  String(sink_threshold),
								  String(calibrate_vario),
								  String(lora_or_flarm),
								  String (ogn_id),
								  String (ogn_on),
					              String (packet_repeat),
								  String(debug_level),
								  String(serial_type_out));
  ESP_LOGI(TAG, "update_configFile - UPdateAW - Completed.");

}

void update_configFile(String UpSsid,String UpPass,String UpNavSW, String GS)
{

  update_actual_file( UpSsid,
		              UpPass,
					  UpNavSW,
					  GS,
					  String(wiredDirectly),
					  String(gpsBaudtoChar(gpsBaudRate)),
					  String (wifioffSelected),
					  String (awHexID),
					  String (awPilotName),
					  String (airWhere_ap_password),
					  String(loraFrequency),
					  String(gsLatitude),
					  String(gsLongitude),
					  String(gsAltitude),
					  String(receiveFlarm),
					  String(ms5611_attached),
					  String(vario_on),
					  String(web_aircraft_type),
					  String(serialBaudRate),
					  String(ble_output),
					  String(climb_threshold),
					  String( zero_threshold),
					  String(sink_threshold),
					  String(calibrate_vario),
					  String(lora_or_flarm),
					  String (ogn_id),
					  String (ogn_on),
		              String (packet_repeat),
					  String(debug_level),
					  String(serial_type_out));

}

/*void update_configFile(String gsLatitude, String gsLongitude, String gsAltitude, String UpSsid, String UpPass, String UpNavSW, String GS)
{

  String rcF=String(receiveFlarm);
  String gpsB=String(gpsBaudtoChar(gpsBaudRate));
  String wOS=String (wifioffSelected);
  String AwIDin=String (airWhereID);
  String awPilotNameIn=String (awPilotName); 
  String aWappassIn=String (airWhere_ap_password);
  String LfreqIn=String(loraFrequency);


  ESP_LOGI(TAG, "update_configFile - GS");

  update_actual_file( UpSsid,
		              UpPass,
					  UpNavSW,
					  GS,
					  String(wiredDirectly),
					  String(gpsBaudtoChar(gpsBaudRate)),
					  String (wifioffSelected),
					  String (airWhereID),
					  String (awPilotName),
					  String (airWhere_ap_password),
					  String(loraFrequency),
					  String(gsLatitude),
					  String(gsLongitude),
					  String(gsAltitude),
					  String(receiveFlarm),
					  String(ms5611_attached),
					  String(vario_on),
					  String(web_aircraft_type),
					  String(serialBaudRate),
					  String(ble_output),
					  String(climb_threshold),
					  String( zero_threshold),
					  String(sink_threshold));
}
*/
void update_configFile(String flarm)
{
  
 /* String FawS=String(airWhereSsid);
  String FawP=String(airWherePassword);
  String Fns=String(navSofware);
  
  String Fgsm=String(groundStationMode);
  String Fgsla=String(gsLatitude);
  String FgsLo=String(gsLongitude);
  String FgsA=String(gsAltitude);
  String wd=String(wiredDirectly);

  String gpsB=String(gpsBaudtoChar(gpsBaudRate));
  String wOS=String (wifioffSelected); 
  String AwIDin=String (airWhereID);
  String awPilotNameIn=String (awPilotName);
  String aWappassIn=String (airWhere_ap_password);
  String LfreqIn=String(loraFrequency);
  */

  update_actual_file( String(airWhereSsid),
	            	  String(airWherePassword),
					  String(navSofware),
					  String(groundStationMode),
					  String(wiredDirectly),
					  String(gpsBaudtoChar(gpsBaudRate)),
					  String (wifioffSelected),
					  String (awHexID),
					  String (awPilotName),
					  String (airWhere_ap_password),
					  String(loraFrequency),
					  String(gsLatitude),
					  String(gsLongitude),
					  String(gsAltitude),
					  flarm,
					  String(ms5611_attached),
					  String(vario_on),
					  String(web_aircraft_type),
					  String(serialBaudRate),
					  String(ble_output),
					  String(climb_threshold),
					  String( zero_threshold),
					  String(sink_threshold),
					  String(calibrate_vario),
					  String(lora_or_flarm),
					  String (ogn_id),
					  String (ogn_on),
		              String (packet_repeat),
					  String(debug_level),
					  String(serial_type_out));
}

void update_configFile(int debug_setting)
{

 /* String FawS=String(airWhereSsid);
  String FawP=String(airWherePassword);
  String Fns=String(navSofware);

  String Fgsm=String(groundStationMode);
  String Fgsla=String(gsLatitude);
  String FgsLo=String(gsLongitude);
  String FgsA=String(gsAltitude);
  String wd=String(wiredDirectly);

  String gpsB=String(gpsBaudtoChar(gpsBaudRate));
  String wOS=String (wifioffSelected);
  String AwIDin=String (airWhereID);
  String awPilotNameIn=String (awPilotName);
  String aWappassIn=String (airWhere_ap_password);
  String LfreqIn=String(loraFrequency);
  */

  update_actual_file( String(airWhereSsid),
	            	  String(airWherePassword),
					  String(navSofware),
					  String(groundStationMode),
					  String(wiredDirectly),
					  String(gpsBaudtoChar(gpsBaudRate)),
					  String (wifioffSelected),
					  String (awHexID),
					  String (awPilotName),
					  String (airWhere_ap_password),
					  String(loraFrequency),
					  String(gsLatitude),
					  String(gsLongitude),
					  String(gsAltitude),
					  String(receiveFlarm),
					  String(ms5611_attached),
					  String(vario_on),
					  String(web_aircraft_type),
					  String(serialBaudRate),
					  String(ble_output),
					  String(climb_threshold),
					  String( zero_threshold),
					  String(sink_threshold),
					  String(calibrate_vario),
					  String(lora_or_flarm),
					  String (ogn_id),
					  String (ogn_on),
		              String (packet_repeat),
					  String (debug_setting),
					  String(serial_type_out));
}
//v5
bool update_buzzer(int buzzer_val)
{
	noInterrupts();
	delay(100);
	airwhere_preferences.begin("airwhere", false);
	airwhere_preferences.putInt("buzzer", buzzer_val);
	airwhere_preferences.end();
	delay(100);
	interrupts();
	return true;
}

bool update_pkt_repeat(char pkt_char)
{
	noInterrupts();
	delay(100);
	airwhere_preferences.begin("airwhere", false);
	airwhere_preferences.putChar("psel", pkt_char);;
	airwhere_preferences.end();
	delay(100);
	interrupts();
	return true;
}


bool update_navSofware(char navSofware_val)
{
	noInterrupts();
	delay(100);
	airwhere_preferences.begin("airwhere", false);
	airwhere_preferences.putChar("navS", navSofware_val);;
	airwhere_preferences.end();
	delay(100);
	interrupts();
	return true;
}

bool update_climb_threshold(int climb_val)
{
	noInterrupts();
	delay(100);
	airwhere_preferences.begin("airwhere", false);
	airwhere_preferences.putInt("climb", climb_val);;
	airwhere_preferences.end();
	delay(100);
	interrupts();
	return true;
}

bool update_sink_threshold(int sink_val)
{
	noInterrupts();
	delay(100);
	airwhere_preferences.begin("airwhere", false);
	airwhere_preferences.putInt("sink", sink_val);;
	airwhere_preferences.end();
	delay(100);
	interrupts();
	return true;
}
bool update_zero_threshold(int zero_val)
{
	noInterrupts();
	delay(100);
	airwhere_preferences.begin("airwhere", false);
	airwhere_preferences.putInt("zero", zero_val);;
	airwhere_preferences.end();
	delay(100);
	interrupts();
	return true;
}

bool update_vario_sounds(char vario_sounds_on_off)
{
	noInterrupts();
	delay(100);
	airwhere_preferences.begin("airwhere", false);
	airwhere_preferences.putChar("vsoundsonoff", vario_sounds_on_off);
	airwhere_preferences.end();
	delay(100);
	interrupts();
	return true;
}



