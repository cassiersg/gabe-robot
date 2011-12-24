#include <plib.h>
#include <stdio.h>
#include <math.h>

#include "robot_time.h"
#include "motor_control.h"
#include "robot_math.h"

#define SERVO_MIN_PERIOD (TICKS_PER_MSEC *18)  // 18 msecs
#define MIN_DURATION 100 // min wait time
#define ANGLE_0  3802 // 3.379 msec
#define ANGLE_2048 1818 // 1.616 msec
#define D_ANGLE_2048 (ANGLE_2048-ANGLE_0) // 2PI/4

// motor control command
#define MAX_MOTOR 4
int motorImpLenFinal[MAX_MOTOR]={ANGLE_0 + D_ANGLE_2048/2,ANGLE_0 + D_ANGLE_2048/2,ANGLE_0 + D_ANGLE_2048/2,ANGLE_0 + D_ANGLE_2048/2};
int motorImpulseLengths[MAX_MOTOR] = {ANGLE_0 + D_ANGLE_2048/2,ANGLE_0 + D_ANGLE_2048/2,ANGLE_0 + D_ANGLE_2048/2,ANGLE_0 + D_ANGLE_2048/2};
int numberPeriod_toFinal[MAX_MOTOR] = { 0, 0, 0, 0};


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

int motor_setAngle(int angle, int motorIndex, uint32 time)
{
	angle = DEG_RANGLE(angle);
	return m_setAngle(angle, motorIndex, time);
}

int m_setAngle(int angle, int motorIndex, uint32 time)
{
	int res = SUCCESS;
	if ((angle >=-1024) && (angle <= 1024) && (motorIndex>=0) && (motorIndex < MAX_MOTOR))
	{
	    int wantedImpulse = ANGLE_0 + (1024+angle)*D_ANGLE_2048/2048 ;
		numberPeriod_toFinal[motorIndex] = time*TICKS_PER_MSEC/SERVO_MIN_PERIOD +1;
		motorImpLenFinal[motorIndex] = wantedImpulse;
	}
	else
	{
	    printf("angle  %d out of range or motorIndex %d unknown\r\n", angle, motorIndex);
	    res = FAILURE;
	}
	return res;
}

int pod_setPosition(int x, int y, int z, int time, int motor1, int motor2, int motor3)
{
	/* point demande hors zone possible */
	if (z<=0 || y<=0 || r_sqrt(x*x+y*y+z*z)> LEN1+LEN2+LEN3)
        return FAILURE;
	int lenM2Tip= r_sqrt(z*z + pow(r_sqrt(x*x + y*y)-LEN1, 2));
	int angle_motor1=DIRECTION1* (r_atan((x<<12)/y)+ANGLE1);
	int angle_motor3=DIRECTION3*( r_acos(((lenM2Tip*lenM2Tip-LEN2*LEN2-LEN3*LEN3)<<12)/(-2*LEN2*LEN3))+ANGLE3);
	int angle_motor2=DIRECTION2* ( r_acos((z<<12)/lenM2Tip)+2048+ r_acos(((LEN3*LEN3-lenM2Tip*lenM2Tip-LEN2*LEN2)<<12)/(-2*lenM2Tip*LEN2))+ANGLE2 );
	//printf("pod_setPosition: lenM2Tip: %i; angle1: %i; angle2: %i; angle3: %i\r\n", lenM2Tip, angle_motor1, angle_motor2, angle_motor3);
    while (angle_motor1>7168)  angle_motor1-=8192;
	while (angle_motor1<-7168) angle_motor1+=8192;
	while (angle_motor2>7168)  angle_motor2-=8192;
	while (angle_motor2<-7168) angle_motor2+=8192;
	while (angle_motor3>7168)  angle_motor3-=8192;
	while (angle_motor3<-7168) angle_motor3+=8192;
	int res1 = m_setAngle(angle_motor1, motor1, time);
	int res2 = m_setAngle(angle_motor2, motor2, time);
	int res3 = m_setAngle(angle_motor3, motor3, time);
	if ( res1==SUCCESS && res2==SUCCESS && res3==SUCCESS )
		return SUCCESS;
	else
	{
		printf("setPosition error: x:%i, y:%i, z:%i", x, y, z);
		return FAILURE;
	}
}

void show_motors(void)
{
	int angle, i;
	for (i=0; i<MAX_MOTOR; i++)
	{
		angle=RANGLE_DEG(2048*(motorImpulseLengths[i]-ANGLE_0)/D_ANGLE_2048)-45;
		printf("Motor %i : %i degres\r\n", i, angle);
	}
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
		if (numberPeriod_toFinal[motorIndex]==0)
			motorImpulseLengths[motorIndex]=motorImpLenFinal[motorIndex];
		else
		{
			motorImpulseLengths[motorIndex]+=(motorImpLenFinal[motorIndex]-motorImpulseLengths[motorIndex])/numberPeriod_toFinal[motorIndex];
			numberPeriod_toFinal[motorIndex]--;
		}
		duration = motorImpulseLengths[motorIndex];
		// to save this refresh with historical.h
		if (numberPeriod_toFinal[motorIndex]!=0)
			mrefresh_save(motorIndex, duration, numberPeriod_toFinal[motorIndex]);
		motorIndex++;
		totalDuration += duration;
	}

	OpenTimer3(T3_ON | T3_PS_1_32, duration);

    // Clear the interrupt flag
	mT3ClearIntFlag();
}
