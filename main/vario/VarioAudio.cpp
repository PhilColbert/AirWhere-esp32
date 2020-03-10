#include <Arduino.h>
#include "VarioAudio.h"
#include "config.h"
#include "util.h"

extern int turn_on_off_buzzer;

void VarioAudio::analogWrite(int pin, uint32_t value)
{
	ledcAttachPin(pin, audiochannel_);
	ledcWrite(audiochannel_, value);
}


void VarioAudio::analogWriteFreq(double freq)
{
	ledcWriteTone(audiochannel_, freq);
}

	

void VarioAudio::Config(int pinPWM, int audiochannel, int stvalue, int ctvalue, int ztvalue) {
	discrimThreshold_ 	=  CLIMB_DISCRIMINATION_THRESHOLD;
	sinkToneCps_       	= stvalue;
	climbToneCps_      	= ctvalue;
	liftyAirToneCps_   	= ztvalue;
    varioState_ 		= VARIO_STATE_QUIET;
	beepPeriodTicks_	= 0;
	beepEndTick_ 		= 0;
	beepCps_ 			= 0;
	varioCps_ 			= 0;
	freqHz_				= 0; 
	tick_				= 0;
	pinPWM_ 			= pinPWM;
	audiochannel_		= audiochannel;
	vario_buzzer_on_	= false;
	check_for_lifty_	= false;
	analogWrite(pinPWM_, 0);
	
    }

//v5

void VarioAudio::UpdateConfig(int stvalue, int ctvalue, int ztvalue) {

	sinkToneCps_       	= stvalue;
	climbToneCps_      	= ctvalue;
	liftyAirToneCps_   	= ztvalue;

    }

void VarioAudio::VarioBeep(int nCps, int turnonBuzzer) {
	int index = 0;
	
    if (turn_on_off_buzzer==1)
    {
    //	Serial.println("var on" );
    	vario_buzzer_on_ = true;
    	check_for_lifty_ = true;
    }

    if (turn_on_off_buzzer==0)
    {
    	vario_buzzer_on_ = false;
    	check_for_lifty_ = false;
   // 	Serial.println("var off");
    }
/*
    if (vario_buzzer_on_)
    {
    	Serial.println("vario_buzzer_on_");
    }
    if (check_for_lifty_)
      {
      	Serial.println("check_for_lifty_");
      }
*/

    int32_t newFreqHz, freqdelta;
	int newduration, newduty = 0;
	int durationdelta, cmsdelta, dutydelta;

	// check if the buzzer is enabled now and if the climb threshold of 1m/s exceeded..
	// basically delays turning on the buzzer until in flight and >  1m/s or < 1m/s has occured
	if (vario_buzzer_on_ && (ABS(nCps) > 100)) check_for_lifty_ = true;

	
	if (
		(beepPeriodTicks_ <= 0)
		|| ((tick_ >= beepPeriodTicks_ / 2) && (ABS(nCps - varioCps_) > discrimThreshold_))
		|| ((nCps >= climbToneCps_) && (varioCps_ < climbToneCps_))
		|| ((nCps >= liftyAirToneCps_) && (varioCps_ < liftyAirToneCps_))
		) {


		if (nCps > VARIO_MAX_CPS)
			varioCps_ = VARIO_MAX_CPS;
		else if (nCps < -VARIO_MAX_CPS)
			varioCps_ = -VARIO_MAX_CPS;
		else
			varioCps_ = nCps;

		int tonetablesize = sizeof(newbeeptbl_) / sizeof(*newbeeptbl_);

		for (int i = 0; i < tonetablesize; i++) {
			if (varioCps_ > newbeeptbl_[index].cms) index++;
			else
				break;
		}

		
		if ((varioCps_ > liftyAirToneCps_) && (varioCps_ <= climbToneCps_) && turn_on_off_buzzer > 0 ) {

			// wait 10 seconds for vario to settle then only turn on once climb/sink threshold exceeds 1 ms
			if ((!vario_buzzer_on_) && (turnonBuzzer >= 9))
			{
				vario_buzzer_on_ = true;
				turn_on_off_buzzer=1;
			}
		}
		//Serial.printf("Variocps %d index %d sinktone cps %d\r\n", varioCps_, index, sinkToneCps_);
		if ((varioCps_ > climbToneCps_) || varioCps_ <= sinkToneCps_ ) {

					
			if (index != (tonetablesize -1)) {
				tick_ = 0;

				freqdelta = newbeeptbl_[index].freq - newbeeptbl_[index - 1].freq;
				durationdelta = newbeeptbl_[index].duration - newbeeptbl_[index - 1].duration;
				cmsdelta = newbeeptbl_[index].cms - newbeeptbl_[index - 1].cms;
				dutydelta = newbeeptbl_[index].duty - newbeeptbl_[index - 1].duty;

				newFreqHz = (float(freqdelta) / float(cmsdelta))*(varioCps_ - newbeeptbl_[index].cms) + newbeeptbl_[index].freq;
				newduration = (float(durationdelta) / float(cmsdelta))*(varioCps_ - newbeeptbl_[index].cms) + newbeeptbl_[index].duration;
				newduty = (float(dutydelta) / float(cmsdelta))*(varioCps_ - newbeeptbl_[index].cms) + newbeeptbl_[index].duty;
				//Serial.print("New Freq "); Serial.print(newFreqHz); Serial.print(" New Duration "); Serial.print(newduration); Serial.print(" New Duty "); Serial.println(newduty);
				if (newduty == 100)
				{
					beepPeriodTicks_ = 0;
					beepEndTick_ = 1000; //use a large number. 1000 equates to 50 secs so never turn off speaker
				}
					
				else
				{
					beepPeriodTicks_ = newduration / 20;//20 milleseconds sample time we are using
					beepEndTick_ = float(newduty / 100.0) * (newduration / 20); //20 millseonds sample
				}
				
				//Serial.printf(" beep period: %d beep end tick:%d \r\n",beepPeriodTicks_,beepEndTick_);
				freqHz_ = newFreqHz;
				SetFrequency(freqHz_);
				

			}
			else
			{
				tick_ = 0;
				newFreqHz = newbeeptbl_[index].freq;
				if (newduty == 100)
				{
					beepPeriodTicks_ = 0;
					beepEndTick_ = 1000; //use a large number. 1000 equates to 50 secs so never turn off speaker
				}
				else
				{
					beepPeriodTicks_ = newbeeptbl_[index].duration / 20;
					beepEndTick_ = float(newbeeptbl_[index].duty / 100.0) * (newbeeptbl_[index].duration) / 20; //20 millseonds sample
				}
				
				freqHz_ = newFreqHz;
				SetFrequency(freqHz_);
				

			}
		}
		else if ((varioCps_ > liftyAirToneCps_) && vario_buzzer_on_ && check_for_lifty_) {


			tick_ = 0;
			beepPeriodTicks_ = 2;
			beepEndTick_ = 1000;//never turn off tone while in lifty mode
								//newFreqHz = VARIO_TICK_FREQHZ + (beepCps_*(VARIO_XOVER_FREQHZ - VARIO_TICK_FREQHZ))/VARIO_XOVER_CPS;
			newFreqHz = 200 + (varioCps_) * 4;
			CLAMP(newFreqHz, 50, VARIO_MAX_FREQHZ);

			freqHz_ = newFreqHz;
			SetFrequency(freqHz_);  // higher frequency as you approach climb threshold
			
		}
		else //turn off tone
		{			
			beepPeriodTicks_ = 0;
			beepEndTick_ = 0;
			freqHz_ = 0;
			SetFrequency(freqHz_);		
		}
	}	
	else
	{
		tick_++;
		beepPeriodTicks_--;
		newFreqHz = freqHz_;
		if (tick_ >= beepEndTick_) { // shut off tone
			newFreqHz = 0;
		}
		if (newFreqHz != freqHz_) {
			freqHz_ = newFreqHz;
			SetFrequency(freqHz_);
			
		}
	}
}
void VarioAudio::Setthresholds(int stvalue, int ctvalue, int ztvalue) {
	sinkToneCps_ = stvalue;
	climbToneCps_ = ctvalue;
	liftyAirToneCps_ = ztvalue;
	//Serial.printf("New thresholds set: ST %d CT %d ZT %d\r\n",stvalue,ctvalue,ztvalue);
}



void VarioAudio::SetFrequency(int32_t fHz) {
	if (!airwhere_tones_on && vario_sounds_on=='y')
	{
	  if (fHz ) {

		  analogWriteFreq(fHz);
		  analogWrite(pinPWM_, 512);
	    }
	  else {
		  analogWrite(pinPWM_, 0);
		  }
	  }
}


void VarioAudio::GenerateTone(int32_t fHz, int ms) {

	// need to detach pin - attach in main loop, detach again and carry on .,...
	  if (!airwhere_tones_on && vario_sounds_on=='y')
	  {

        SetFrequency(fHz);
        delay(ms);
        SetFrequency(0);
	  }
    }

