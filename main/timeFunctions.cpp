#include <Arduino.h>
// ESP32 #include "WebHelper.h"
#include "AirWare.h"

void myDelay(int timeToWait)
{
  int timeNow=millis();
  while (timeNow+timeToWait>millis())
  {
    yield();
#ifdef ESP32_CPU

#else
    Web_loop();
#endif


  }
}

void yieldDelay(int timeToWait)
{
  int timeNow=millis();
  while (timeNow+timeToWait>millis())
  {
    yield();
  }

}


