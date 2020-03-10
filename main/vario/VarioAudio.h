#ifndef VARIO_AUDIO_H_
#define VARIO_AUDIO_H_


typedef struct NEWBEEP_ {
	float cms;
	int32_t freq;
	int duration;
	int duty;
} NEWBEEP;

#define VARIO_MAX_CPS         1000

// more audio discrimination for climbrates under this threshold
#define VARIO_XOVER_CPS       500



#define VARIO_STATE_SINK    	11
#define VARIO_STATE_QUIET   	22
#define VARIO_STATE_LIFTY_AIR	33
#define VARIO_STATE_CLIMB   	44

#define CLIMB_DISCRIMINATION_THRESHOLD 25

extern bool airwhere_tones_on;
extern char buzzer_on_off;
extern int turn_on_off_buzzer;
extern char vario_sounds_on;


class VarioAudio {
	
public :
VarioAudio(){};

void Config(int pinPWM, int audiochannel, int stvalue, int ctvalue, int ztvalue);
//v5
void UpdateConfig(int stvalue, int ctvalue, int ztvalue);
void VarioBeep(int cps, int turnonBuzzer);
void SetFrequency(int32_t freqHz);
void GenerateTone( int32_t freqHz, int ms);
void Setthresholds(int stvalue, int ctvalue, int ztvalue);


private :


void analogWrite(int pin, uint32_t value);
void analogWriteFreq(double freq);



int discrimThreshold_;
int beepCps_;
int varioCps_;
int32_t freqHz_;
int sinkToneCps_;
int climbToneCps_;
int liftyAirToneCps_;

int beepPeriodTicks_;
int beepEndTick_;
int tick_;
int varioState_;
int pinPWM_;
int audiochannel_;
bool vario_buzzer_on_ ;
bool check_for_lifty_ ;

	
//models the xctracer audio  table shows cm/s, freq Hz, cycle period ms, cycle duty % -  ( resolution used is 20ms)
NEWBEEP newbeeptbl_[10] = {
{-1000,200,10,100},
{-300,250,10,100},
{-200,300,10,100},
{0,400,600,100},
{100,587,560,50 },
{200,600,520,50 },
{300,675,440,50},
{400,725,320,50},
{500,850,240,50},
{1000,1200,200,50}
};
};//end class definition



#endif
