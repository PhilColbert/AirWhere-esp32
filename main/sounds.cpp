/*
 * sounds.cpp
 *
 *  Created on: 4 Apr 2017
 *      Author: Fugazi
 */

#include "sounds.h"
#include "fileOps.h"

// store the serial data read into SoundBuffer - it might come in amount of data at a time, so wait till we have a full line
// then fill the array with the data - start the first tone and return to the main loop which will process the rest.
// if another sound command comes in, kill this one straight away and then fill tuneIs with the new data and start that tone.

bool isItNumeric ( String testString )
{

    for ( int i=0;i<testString.length();i++)
    {
      if (testString.charAt(i)=='0' ||
          testString.charAt(i)=='1' ||
          testString.charAt(i)=='2' ||
          testString.charAt(i)=='3' ||
          testString.charAt(i)=='4' ||
          testString.charAt(i)=='5' ||
          testString.charAt(i)=='6' ||
          testString.charAt(i)=='7' ||
          testString.charAt(i)=='8' ||
          testString.charAt(i)=='9')
      {
      }
      else
      {
        return false;
      }
    }
  return true;
}

void process_next_beep(int tuneIs[][beepData] ,int &noTones, int &currentTone)
{

    if (currentTone>=noTones)
     {

     // validSoundLine=false;
     // switch off sound.
       if (  millis() > tuneIs[currentTone][1] && airwhere_tones_on)
       {
         tuneIs[currentTone][1]=0;
         noTones=0;
         currentTone=0;
         ledcWriteTone(SPEAKER_CHANNEL, 0 );
     //    Serial.println("Switching airwhere sounds off");
         airwhere_tones_on=false;
      //   validSoundLine=false;
       }
     }

    // if ( noTones>0 && validSoundLine)
    if ( noTones>0 )
     {
      if ( millis()>tuneIs[currentTone][1])
      {
        currentTone++;
        ledcWriteTone(SPEAKER_CHANNEL, tuneIs[currentTone][0]);
       }
     }
}

void process_sound_stream(String &SoundBuffer, int tuneIs[][beepData] ,int &noTones, int &currentTone)
{

    size_t slen=Serial.available();
    uint8_t soundsbuf[slen];
    char soundscbuf[slen];
    bool soundsFullLine=false;
    int posReturn=0;

    //need to rewrite this properly for multiple lines, but am tired...
    String current_buffer="";

    if (slen!=0)
    {

        Serial.readBytes(soundsbuf, slen);
        memset(soundscbuf, 0, slen);
        memcpy(&soundscbuf,&soundsbuf, slen);
        SoundBuffer=SoundBuffer+String(soundscbuf).substring(0,slen);
    }

    if (SoundBuffer.length()>0)
    {
       // Serial.println("SoundBuffer start 444 " + SoundBuffer + "<<<");


        for ( posReturn=0;posReturn<=SoundBuffer.length();posReturn++)
        {
          if (SoundBuffer.charAt(posReturn)=='\n' || SoundBuffer.charAt(posReturn)=='\r')
          {

             // need to remove the cksum - assume for now we only have a line at a time coming in
             // if in future multiple lines come in at once - need to add code to deal with this - doubtful.
        	  current_buffer=SoundBuffer.substring(0, SoundBuffer.indexOf('*'));
        	  // add the rest into sb, the 5 jumps the cksum and the cr and lf.
        	   if (SoundBuffer.indexOf('*')!=-1)
        	   {
        		   SoundBuffer=SoundBuffer.substring(SoundBuffer.indexOf('*')+5,SoundBuffer.length());
        	   }
        	   else
        	   {
        		   SoundBuffer=="";
        	   }
        	//	   Serial.println("SoundBuffer end" + SoundBuffer + "<<<");
             soundsFullLine=true;
             break;
         }
     }

    }

    bool validSoundLine=false;



    if ( current_buffer!="" && soundsFullLine)
    {
    	//Serial.println("current_buffer" + current_buffer);


    	if ( current_buffer.substring(0,9)=="$GFPALARM" )
    	{

    		String currS=SoundBuffer.substring(10);
    		//v1.12

    		//v3 GPSSerial.println(SoundBuffer);

    		currS.trim();

    		if (isItNumeric(currS))
    		{
    			int toneNumber=SoundBuffer.substring(9).toInt();
    			validSoundLine=true;
    			toneNumber=(toneNumber*GFPALARM_SCALE)+GFPALARM_BASE_FREQUENCY;

    			tuneIs[0][0]=toneNumber;
    			tuneIs[0][1]=GFPALARM_LENGTH+millis();
    			// start playing the first.
				// tone(speakerPin, tuneIs[0][0]);

    			// set switch to stop vario for the sounds.
    			//   Serial.println("Switching airwhere sounds on");
    			airwhere_tones_on=true;
    			ledcWriteTone(SPEAKER_CHANNEL, tuneIs[0][0]);
    			currentTone=0;
    			noTones=0;
    		}
    		else
    		{
    			validSoundLine=false;
    		}

    	}

    	if (current_buffer.substring(0,4)=="$BSD")
    	{

    		String bsdLine=current_buffer.substring(5);
    		//v1.12
    		//v3    GPSSerial.println(SoundBuffer);

    		int currentChar=0;

    		int tuneLength=0;
    		int maxTones=10;

    		int toneReading=0;
    		int spaceIndex=0;

    		validSoundLine=true;

    		while( spaceIndex>=0 && currentTone<maxTones && validSoundLine)
    		{

    			spaceIndex=bsdLine.indexOf(" ",currentChar);

    			String currS=bsdLine.substring(currentChar,spaceIndex);
    			currS.trim();

    			if (isItNumeric(currS))
    			{
    				// time to play tone
    				if ( toneReading == 1 )
    				{
    					tuneIs[currentTone][toneReading]=tuneLength+currS.toInt()+millis();
    					tuneLength=tuneLength+currS.toInt();
    				}
    				else
    				{
    					tuneIs[currentTone][toneReading]=currS.toInt();
    				}

    				currentChar=spaceIndex+1;

    				if (toneReading==0)
    				{
    					toneReading=1;
    					noTones++;
    				}
    				else
    				{
    					toneReading=0;
    					currentTone++;
    				}
    			}
    			else
    			{
    				validSoundLine=false;
    			}
    		}


    		if (validSoundLine)
    		{
    			// start playing first tone and then check at top of loop for the next tone to change to.
    			// tone(speakerPin, tuneIs[0][0]);
    			// set switch to stop vario for the sounds.
    			// Serial.println("Switching airwhere sounds on - BSD");
    			airwhere_tones_on=true;

    			ledcWriteTone(SPEAKER_CHANNEL, tuneIs[0][0]);
    			currentTone=0;
    			// playing the first.
    			noTones--;
    		}
    	}

#define BUFFER current_buffer
#include "airwhere_commands.h"

    	current_buffer="";
    }

}

