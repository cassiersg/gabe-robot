#include <plib.h>
#include "status_led.h"



void status_led_init(void)
{
	// set the pin as output and init it as off
    PORTSetPinsDigitalOut(IOPORT_F, BIT_1);
    mPORTFClearBits(BIT_1);

    // configure Timer 1 using external asynchronous clock, 1:1 prescale, 0x4000 period
    OpenTimer1(T1_ON | T1_SOURCE_EXT | T1_SYNC_EXT_OFF | T1_PS_1_1, 0x4000);
			//Timer1 On, External source, No synchronization, 1:1 Prescaler, Period

    // Set up the timer interrupt with a priority of 2
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);
}


void __ISR(_TIMER_1_VECTOR, ipl2) Timer1Handler(void)
{
    // Toggle the LED
    mPORTFToggleBits(BIT_1);

    // Clear the interrupt flag
	mT1ClearIntFlag();
}
