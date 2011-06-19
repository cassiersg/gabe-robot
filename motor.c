#include <plib.h>
#include "motor_control.h"


#define SERVO_MIN_PERIOD 20250  // 18 msecs
#define MIN_DURATION 100 // min wait time
#define ANGLE_0  3802 // 3.379 msec
#define ANGLE_90 1818 // 1.616 msec


// motor control command
#define MAX_MOTOR 4
int motorImpulseLengths[MAX_MOTOR] = {ANGLE_0, ANGLE_0, ANGLE_0, ANGLE_0};

// motor control state
int motorIndex = 0;
int totalDuration = 0;




int motor_init(void)
{
	// motor is mapped on RD2. set it to low
    PORTSetPinsDigitalOut(IOPORT_D, BIT_2);
    mPORTDClearBits(BIT_2);
	PORTSetPinsDigitalOut(IOPORT_D, BIT_3);
    mPORTDClearBits(BIT_3);
	PORTSetPinsDigitalOut(IOPORT_D, BIT_4);
    mPORTDClearBits(BIT_4);
	PORTSetPinsDigitalOut(IOPORT_D, BIT_5);
    mPORTDClearBits(BIT_5);

	// once the port is correctly configured, start the timer 
	// and configure the associated interrupt
	/* Timer 3 is configured to run at 1.125 MHz : 
          72MHz/2/32 (we are on Periph clock) /32 (divider)
	*/
    OpenTimer3(T3_ON | T3_PS_1_32, SERVO_MIN_PERIOD);
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_4);


}

int motor_setAngle(int angle, int motorIndex)
{
	int res = SUCCESS;
	if ((angle >=-20) && (angle <= 110) && (motorIndex>=0) && (motorIndex < MAX_MOTOR))
	{
		motorImpulseLengths[motorIndex] = ANGLE_0 + (angle*(ANGLE_90-ANGLE_0))/90 ;
		printf("applying %i\r\n", motorImpulseLengths[motorIndex]);
	}
	else
		res = FAILURE;
	return res;
}


/*
 the motor active period is set by the timer 3. Period are being set after each other. 
 Once all motors have been activited, we wait the end of a global period.
 At the end of the global period we can automatically adapt the motor period- but this is not yet achieved
*/

void __ISR(_TIMER_3_VECTOR, ipl4) Timer3Handler(void)
{
    // Toggle the LED
    // mPORTDToggleBits(BIT_2);
    #define MOTOR_MASK 0x3C

    // toggle PortD
    *(uint32 *)0xBF8860EC = (6<<motorIndex)&MOTOR_MASK; // we togle the previous motor and the next one 

	int duration;
	if ((motorIndex>=MAX_MOTOR))
	{
		duration = SERVO_MIN_PERIOD-totalDuration;
        if (duration < 0)
			duration = MIN_DURATION;
		motorIndex = 0;
		totalDuration = 0;
	}
	else
	{
		duration = motorImpulseLengths[motorIndex];
		motorIndex++;
		totalDuration += duration;
	}
	
	OpenTimer3(T3_ON | T3_PS_1_32, duration);
	
    // Clear the interrupt flag
	mT3ClearIntFlag();
}


