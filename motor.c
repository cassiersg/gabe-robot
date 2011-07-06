#include <plib.h>
#include <stdio.h>
#include <math.h>
#include "motor_control.h"

#define TICKSPERMS 1125  // 1.125 MHz
#define SERVO_MIN_PERIOD (1125*18)  // 18 msecs
#define MIN_DURATION 100 // min wait time
#define ANGLE_0  3802 // 3.379 msec
#define ANGLE_90 1818 // 1.616 msec
#define D_ANGLE_1 ((ANGLE_0-ANGLE_90)/90) // 1°

#define DEGRES(rad) ((rad)*180/M_PI)


// motor control command
#define MAX_MOTOR 4
int motorImpLenFinal[MAX_MOTOR]={ANGLE_0, ANGLE_0, ANGLE_0, ANGLE_0};
int Dif_ImpLen_PerPeriod[MAX_MOTOR]={D_ANGLE_1*90, D_ANGLE_1*90, D_ANGLE_1*90, D_ANGLE_1*90};
int motorImpulseLengths[MAX_MOTOR] = {ANGLE_0, ANGLE_0, ANGLE_0, ANGLE_0};



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
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_4);
    OpenTimer3(T3_ON | T3_PS_1_32, SERVO_MIN_PERIOD);
}

int motor_setAngle(int angle, int motorIndex, int speed)
{
	int res = SUCCESS;
	if ((angle >=-45) && (angle <= 45) && (motorIndex>=0) && (motorIndex < MAX_MOTOR))
	{
		int wantedImpulse = ANGLE_0 + (angle+45)*D_ANGLE_1;
		//  speed*D_ANGLE_1 => Dif_ImpLen / sec * sec/period <= 18ms/1000
		Dif_ImpLen_PerPeriod[motorIndex]=speed*D_ANGLE_1*SERVO_MIN_PERIOD/1125/1000;
		printf("applying %i\r\n", wantedImpulse);
		if (speed==0)
			Dif_ImpLen_PerPeriod[motorIndex]=D_ANGLE_1*90;
		if (wantedImpulse>motorImpulseLengths[motorIndex]) 
			Dif_ImpLen_PerPeriod[motorIndex]=fabs(Dif_ImpLen_PerPeriod[motorIndex]);
		else
			Dif_ImpLen_PerPeriod[motorIndex]=-fabs(Dif_ImpLen_PerPeriod[motorIndex]);
		motorImpLenFinal[motorIndex] = wantedImpulse; /* set this at the end otherwise the speed is updated too late*/
	}
	else
	{
		printf("angle  %d out of range or motorIndex %d unknown\n", angle, motorIndex);
		res = FAILURE;
	}
	return res;
}


/*
 the motor active period is set by the timer 3. Period are being set after each other.
 Once all motors have been activited, we wait the end of a global period.
 The motor period is automatically adapted before the impulse depending on the speed.
*/

void __ISR(_TIMER_3_VECTOR, ipl4) Timer3Handler(void)
{
	// motor control state
	static int motorIndex = 0;
	static int totalDuration = 0;

    // mPORTDToggleBits(BIT_2);
    #define MOTOR_MASK 0x3C

    // toggle PortD
    *(uint32 *)0xBF8860EC = (0x6<<motorIndex)&MOTOR_MASK; // we toggle the previous motor and the next one

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
		if (fabs(motorImpLenFinal[motorIndex]-motorImpulseLengths[motorIndex])>fabs(Dif_ImpLen_PerPeriod[motorIndex]))
			motorImpulseLengths[motorIndex] += Dif_ImpLen_PerPeriod[motorIndex];
		else
			motorImpulseLengths[motorIndex] = motorImpLenFinal[motorIndex];

		duration = motorImpulseLengths[motorIndex];
		motorIndex++;
		totalDuration += duration;
	}

	OpenTimer3(T3_ON | T3_PS_1_32, duration);

    // Clear the interrupt flag
	mT3ClearIntFlag();
}

int pod_setPosition(int x, int y, int z, int motor1, int motor2, int motor3)
{
	/* point demande hors zone possible */
	if (z<=0 || y<=0 || sqrt(x*x+y*y+z*z)> LEN1+LEN2+LEN3)
        return FAILURE;
	int angle_motor1, angle_motor2, angle_motor3;
	int lenM2Tip;
	angle_motor1=DIRECTION1* ( DEGRES(atan(x/y)+ANGLE1) );
	lenM2Tip=sqrt(z*z + pow(sqrt(x*x + y*y)-LEN1, 2));
	angle_motor3=DIRECTION3*( DEGRES(acos((lenM2Tip*lenM2Tip-LEN2*LEN2-LEN3*LEN3)/(-2*LEN2*LEN3)))+ANGLE2) ;
	angle_motor2=DIRECTION2* ( DEGRES(acos(z/lenM2Tip))+90+ DEGRES(acos((LEN3*LEN3-lenM2Tip*lenM2Tip-LEN2*LEN2)/(-2*lenM2Tip*LEN2)))+ANGLE3 );
	if ( motor_setAngle(angle_motor1, motor1, 0)==SUCCESS && motor_setAngle(angle_motor2, motor2, 0)==SUCCESS && motor_setAngle(angle_motor3, motor3, 0)==SUCCESS )
		return SUCCESS;
	return FAILURE;
}

