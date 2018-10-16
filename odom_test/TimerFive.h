#ifndef TimerFive_h
#define TimerFive_h

/*
IMPORTANT:
 - TimerFive is destined only to use on Arduino Mega 2560.
 - You cannot use pwm on the pins 44, 45, 46.
 - You cannot use a period greater than 8.39 seconde.
*/

#include <avr/io.h>
#include <avr/interrupt.h>

#define RESOLUTION 65536    // Timer1 is 16 bit

class TimerFive
{
  public:
    void attachInterrupt(void (*isr)(), long microseconds);
    void detachInterrupt();
    void setPeriod(long microseconds);
    void (*isrCallback)();
};

extern TimerFive Timer5;
#endif
