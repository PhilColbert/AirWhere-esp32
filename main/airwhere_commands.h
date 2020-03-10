

/*
 * v4.1
 * label=AW Reset Default
event=SendNMEAPort1 RSX

label=Lift Buzzer\nOff/On
event=SendNMEAPort1 BBZ ( 0 / 1 )

event=SendNMEAPort1 BVU
event=StatusMessage Volume UP

event=SendNMEAPort1 BVD
event=StatusMessage Volume Down

event=SendNMEAPort1 BTN
event=StatusMessage Sounds Off/On

event=SendNMEAPort1 AWPKT 1/0
event=StatusMessage Packet Repeat\nOn


event=SendNMEAPort1 AWLT 1/0
event=StatusMessage Live Tracking\nOn

event=SendNMEAPort1 AWT 1/0
event=StatusMessage Tracking\nOn

event=SendNMEAPort1 BOM 0
event=StatusMessage Switching AirWhere to XCSoar Mode

event=SendNMEAPort1 BOM 1
event=StatusMessage Switching AirWhere to LK8000 Mode

event=SendNMEAPort1 AWVC
event=StatusMessage  Calibrating Vario - Do not move instrument.

event=SendNMEAPort1 AWVO
event=StatusMessage Vario\n On/Off

event=SendNMEAPort1 BOL 5/8/10/12/15/20/30
label=LiftThresh 0.12 m/s

event=SendNMEAPort1 BFI 150
label=LiftFreq\n150Hz

event=SendNMEAPort1 BRM 25
label=Lift BeepRate Fast 0.25


event=SendNMEAPort1 BOS 1000
label=SinkTone Off (-10m/s)

event=SendNMEAPort1 BSQ 80
label=Sink Inc\n80Hz


event=SendNMEAPort1 BZT 70 ( 0 1 turns on and off ).
label=Buzzer -0.7 m/s



 *
 */

///turn_on_off_buzzer
// 0 turn off buzzer
// 1 turn on buzzer
// 2 set to turn on when motion sensed

        if (BUFFER.substring(0,4)=="$BBZ")
    	{

        	if (debug_level>0)
        	{
        		Serial.println("Received command : $BBZ");
		        Serial.println(BUFFER);
        	}

			validSoundLine=true;

			tuneIs[0][0]=900;
			tuneIs[0][1]=250+millis();
			airwhere_tones_on=true;
			ledcWriteTone(SPEAKER_CHANNEL, tuneIs[0][0]);
			currentTone=0;
			noTones=0;

        /*	Serial.println("$BBZ :- ");
        	Serial.println("*******");
           Serial.println(BUFFER);
           Serial.println( BUFFER.substring(5,6));
           Serial.println("*******");


           Serial.println(">>>>" + String(BUFFER.length()) + "<<<<");
           Serial.println("*******");
*/

        	if (BUFFER.substring(5,6)=="0" && BUFFER.charAt(4)==' ')
        	{

        		turn_on_off_buzzer=0;
        		update_buzzer(0);

        	}
        	else
        	{
        		if (BUFFER.substring(5,6)=="1" && BUFFER.charAt(4)==' ')
        		{

        			turn_on_off_buzzer=1;
        			update_buzzer(1);

        		}
        		else
        		{
        			if ((turn_on_off_buzzer==0 || turn_on_off_buzzer==2))
        			{
        				turn_on_off_buzzer=1;
        				update_buzzer(1);
        			}
        			else
        			{
        				if (turn_on_off_buzzer==1)
        				{
        					turn_on_off_buzzer=0;
        					update_buzzer(0);

        				}
        			}
        		}
        	}
        	// if (buzzer_on_off=='f' && BUFFER.length()<7)
    	}

//RESET
        if (BUFFER.substring(0,4)=="$RSX" || BUFFER.substring(0,4)=="$RST" )
    	{

        	if (debug_level>0)
        	{
        		Serial.println("Received command : $RSX");
        		 Serial.println(BUFFER);
        	}

			validSoundLine=true;

			tuneIs[0][0]=900;
			tuneIs[0][1]=250+millis();
			airwhere_tones_on=true;
			ledcWriteTone(SPEAKER_CHANNEL, tuneIs[0][0]);
			currentTone=0;
			noTones=0;
			Serial.println("RSX command received - Restarting system.");
			delay(500);

			esp_restart();
    	}
//LK <-> XCS
        if (BUFFER.substring(0,4)=="$BOM")
        {

        	if (debug_level>0)
        	{
        		Serial.println("Received command : $BOM");
        		 Serial.println(BUFFER);
        	}

        //	Serial.println("$BOM");

        	validSoundLine=true;

        	tuneIs[0][0]=900;
        	tuneIs[0][1]=250+millis();
        	airwhere_tones_on=true;
        	ledcWriteTone(SPEAKER_CHANNEL, tuneIs[0][0]);
        	currentTone=0;
        	noTones=0;


        	if (BUFFER.substring(5,6)=="X" && BUFFER.charAt(4)==' ')
        	{
          		navSofware='X';
        		update_navSofware('X');
        	}
        	if (BUFFER.substring(5,6)=="L" && BUFFER.charAt(4)==' ')
        	{
        		navSofware='L';
        		update_navSofware('L');
        	}
        	if (BUFFER.substring(5,6)=="A" && BUFFER.charAt(4)==' ')
        	{
        		navSofware='A';
        		update_navSofware('A');
        	}
        	if (BUFFER.substring(5,6)=="K" && BUFFER.charAt(4)==' ')
        	{

        	//	Serial.println("update K");
        		navSofware='K';
        		update_navSofware('K');
        	}
        }

// VARIO SOUNDS ON OFF

        if (BUFFER.substring(0,5)=="$AWVO")
        {
        	if (debug_level>0)
        	{
        		Serial.println("Received command : $AWVO");
        		 Serial.println(BUFFER);
        	}

        	validSoundLine=true;

        	tuneIs[0][0]=900;
        	tuneIs[0][1]=250+millis();
        	airwhere_tones_on=true;
        	ledcWriteTone(SPEAKER_CHANNEL, tuneIs[0][0]);
        	currentTone=0;
        	noTones=0;


        	if (BUFFER.substring(6,7)=="1" && BUFFER.charAt(5)==' ')
        	{
        		vario_sounds_on='y';
        		update_vario_sounds(vario_sounds_on);
        	}
        	if (BUFFER.substring(6,7)=="0" && BUFFER.charAt(5)==' ')
        	{
        		vario_sounds_on='n';
        		update_vario_sounds(vario_sounds_on);
        	}
        }
        // implement volume up down
        if (BUFFER.substring(0,4)=="$BVD")
        {
        	if (debug_level>0)
        	{
        		Serial.println("Received command : $BVD");
        		 Serial.println(BUFFER);
        	}
        }

        if (BUFFER.substring(0,6)=="$AWPKT")
        {
        	if (debug_level>0)
        	{
        		Serial.println("Received command : $AWPKT");
        		 Serial.println(BUFFER);
        	}

        	validSoundLine=true;

        	tuneIs[0][0]=900;
        	tuneIs[0][1]=250+millis();
        	airwhere_tones_on=true;
        	ledcWriteTone(SPEAKER_CHANNEL, tuneIs[0][0]);
        	currentTone=0;
        	noTones=0;


        	if (BUFFER.substring(7,8)=="1" && BUFFER.charAt(6)==' ')
        	{
        		update_pkt_repeat('y');
        		packet_repeat='y';
        	}
        	if (BUFFER.substring(7,8)=="0" && BUFFER.charAt(6)==' ')
        	{
           		update_pkt_repeat('n');
           		packet_repeat='n';
        	}
        }
        // implement calibrate vario.
        if (BUFFER.substring(0,5)=="$AWVC")
        {
        	if (debug_level>0)
        	{
        		Serial.println("Received command : $AWVC");
        		 Serial.println(BUFFER);
        	}
        }
        //Lift threshold
        if (BUFFER.substring(0,4)=="$BOL")
        {
        	if (debug_level>0)
        	{
        		Serial.println("Received command : $BOL");
        		 Serial.println(BUFFER);
        	}

        	validSoundLine=true;

        	tuneIs[0][0]=900;
        	tuneIs[0][1]=250+millis();
        	airwhere_tones_on=true;
        	ledcWriteTone(SPEAKER_CHANNEL, tuneIs[0][0]);
        	currentTone=0;
        	noTones=0;


        	if (BUFFER.charAt(4)==' ')
        	{
        		climb_threshold=BUFFER.substring(5,7).toInt();
        		update_climb_threshold(climb_threshold);


        	}
        }

        // implement Beep Freq
        if (BUFFER.substring(0,4)=="$BFQ")
        {
        	if (debug_level>0)
        	{
        		Serial.println("Received command : $BFQ");
        		 Serial.println(BUFFER);
        	}
        }
        // implement Beep Rate
        if (BUFFER.substring(0,4)=="$BRM")
        {
        	if (debug_level>0)
        	{
        		Serial.println("Received command : $BRM");
        		 Serial.println(BUFFER);
        	}
        }

        //Sink threshold
        if (BUFFER.substring(0,4)=="$BOS")
        {
        	if (debug_level>0)
        	{
        		Serial.println("Received command : $BOS");
        		 Serial.println(BUFFER);
        	}
        	validSoundLine=true;

        	tuneIs[0][0]=900;
        	tuneIs[0][1]=250+millis();
        	airwhere_tones_on=true;
        	ledcWriteTone(SPEAKER_CHANNEL, tuneIs[0][0]);
        	currentTone=0;
        	noTones=0;


        	if (BUFFER.charAt(4)==' ')
        	{
        		sink_threshold=-1*(BUFFER.substring(5,9).toInt());
        		update_sink_threshold(sink_threshold);

        	}
        }
        // implement Sink tone
        if (BUFFER.substring(0,4)=="$BSQ")
        {
        	if (debug_level>0)
        	{
        		Serial.println("Received command : $BSQ");
        		 Serial.println(BUFFER);
        	}
        }

        if (BUFFER.substring(0,5)=="$BZZL")
        {
        	if (debug_level>0)
        	{
        		Serial.println("Received command : $BZZL");
        		 Serial.println(BUFFER);
        	}
        	validSoundLine=true;

        	tuneIs[0][0]=900;
        	tuneIs[0][1]=250+millis();
        	airwhere_tones_on=true;
        	ledcWriteTone(SPEAKER_CHANNEL, tuneIs[0][0]);
        	currentTone=0;
        	noTones=0;


        	if (BUFFER.charAt(5)==' ')
        	{
        		int zt=BUFFER.substring(6,9).toInt();
        		if (zt>100)
        		{
        			zero_threshold=(zt-100);
        		}
        		else
        		{
        			zero_threshold=(zt*-1);
        		}


        		update_zero_threshold(zero_threshold);

        	}
        }

