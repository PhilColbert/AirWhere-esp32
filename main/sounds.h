/*
 * sounds.h
 *
 *  Created on: 4 Apr 2017
 *      Author: Fugazi
 */

#ifndef SOUNDS_H_
#define SOUNDS_H_

#include <Arduino.h>
//Sounds - allow for 20 different beeps.
#define noBeeps 20
#define beepData 2
#define GFPALARM_LENGTH 300
#define GFPALARM_BASE_FREQUENCY 200
#define GFPALARM_SCALE 13
#define SPEAKER_PIN 32
#define SPEAKER_CHANNEL 0

bool isItNumeric ( String testString );
void process_next_beep(int tuneIs[][beepData] ,int &noTones, int &currentTone);
void process_sound_stream(String &SoundBuffer, int tuneIs[][beepData], int &noTones, int &currentTone );

//extern HardwareSerial GPSSerial;
extern bool airwhere_tones_on;
extern char buzzer_on_off;
extern int  turn_on_off_buzzer;
extern char navSofware;
extern char vario_sounds_on;


#endif /* SOUNDS_H_ */
