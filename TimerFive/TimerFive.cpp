#ifndef TimerFive_cpp
#define TimerFive_cpp

#include "TimerFive.h"

TimerFive Timer5;              // preinstatiate

ISR(TIMER5_COMPA_vect)          // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  Timer5.isrCallback();
}

void TimerFive::setPeriod(long microseconds)		// AR modified for atomic access
{
  unsigned char clockSelectBits;
  long cycles = (F_CPU / 1000000) * microseconds;
  if(cycles < RESOLUTION)              clockSelectBits = _BV(CS10);              // no prescale, full xtal
  else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11);              // prescale by /8
  else if((cycles >>= 3) < RESOLUTION) clockSelectBits = _BV(CS11) | _BV(CS10);  // prescale by /64
  else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12);              // prescale by /256
  else if((cycles >>= 2) < RESOLUTION) clockSelectBits = _BV(CS12) | _BV(CS10);  // prescale by /1024
  else        cycles = RESOLUTION - 1, clockSelectBits = _BV(CS12) | _BV(CS10);  // request was out of bounds, set as maximum
  
  char oldSREG = SREG;				
  cli();							// Disable interrupts for 16 bit register access
  OCR5A = cycles;                                          // ICR1 is TOP in p & f correct pwm mode
  SREG = oldSREG;
  
  TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
  TCCR5B |= clockSelectBits;                                          // reset clock select register, and starts the clock
}

void TimerFive::attachInterrupt(void (*isr)(), long microseconds)
{
  TCCR5A = 0;                 // clear control register A 
  TCCR5B = _BV(WGM52);        // set mode 12: Clear Timer on Compare Match (CTC), stop the timerZ
  if(microseconds > 0) setPeriod(microseconds);
  isrCallback = isr;                                       // register the user's callback with the real ISR
  TIMSK5 = _BV(OCIE5A);                                     // sets the timer overflow interrupt enable bit
}

void TimerFive::detachInterrupt()
{
  TIMSK5 &= ~_BV(OCIE5A);                                // clears the timer overflow interrupt enable bit 
							// timer continues to count without calling the isr
}

#endif
