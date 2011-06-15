#include <plib.h>
#include "motor_control.h"

#define SERVO_PERIOD 20250  // 18 msecs

#define ANGLE_0  3802 // 3.379 msec
#define ANGLE_90 1818 // 1.616 msec

int shortPeriod = 0; // indication in which phase control hase we are.
					 // shortPeriod=1 means we are sending the pulse

int impulseLen = ANGLE_0; // this is the current angle. (expressed in pulse length)

int motor_init(void)
{
	// motor is mapped on RD2. set it to low
    PORTSetPinsDigitalOut(IOPORT_D, BIT_2);
    mPORTDClearBits(BIT_2);

	// once the port is correctly configured, start the timer 
	// and configure the associated interrupt
	/* Timer 3 is configured to run at 1.125 MHz : 
          72MHz/2/32 (we are on Periph clock) /32 (divider)
	*/
    OpenTimer3(T3_ON | T3_PS_1_32, SERVO_PERIOD);
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_4);


}

int motor_setAngle(int angle)
{
	int res = SUCCESS;
	if ((angle >=-20) && (angle <= 110))
	{
		impulseLen = ANGLE_0 + (angle*(ANGLE_90-ANGLE_0))/90 ;
		printf("applying %i\r\n", impulseLen);
	}
	else
		res = FAILURE;
	return res;
}

void __ISR(_TIMER_3_VECTOR, ipl4) Timer3Handler(void)
{
    // Toggle the LED
    mPORTDToggleBits(BIT_2);
	shortPeriod ^= 1;

	// new period
	int duration;
	if (shortPeriod)
		duration = impulseLen;
	else
		duration = SERVO_PERIOD-impulseLen;

	OpenTimer3(T3_ON | T3_PS_1_32, duration);
	
    // Clear the interrupt flag
	mT3ClearIntFlag();
}


