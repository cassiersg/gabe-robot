#include <plib.h>
#include <stdio.h>
#include <math.h>

#include "robot_time.h"
#include "motor_control.h"
#include "robot_math.h"
#include "robot_console.h"

#define SERVO_MIN_PERIOD (TICKS_PER_MSEC *18)  // 18 msecs
#define MIN_DURATION     100 // min wait time
#define ANGLE_0          3720 // 3.307 msec
#define ANGLE_RPI2       1818 // 1.616 msec
#define D_ANGLE_RPI2     (ANGLE_RPI2-ANGLE_0) // 2PI/4



static int  setAngle(uint8 *args[], int argc);
static int  setPosition(uint8 *args[], int argc);
static int  msetAngle(uint8 *args[], int argc);
static int  motors_state(uint8 *args[], int argc);
CmdEntry motorConsoleMenu[] = {
	{ "setangle",    setAngle,       4, "setangle <motIdx> <val degre> <time ms>"},
	{ "sa",          setAngle,       2, "setangle <motIdx> <val degre> <time ms>"},
	{ "setposition", setPosition,    8, "setposition <x> <y> <z> <time ms> <motor1> <motor2> <motot3>"},
	{ "sp",          setPosition,    4, "setposition <x> <y> <z> <time ms> <motor1> <motor2> <motot3>"},
	{ "mstate",      motors_state,   1, "angles of the motors"},
	{ "sad",         msetAngle,      4, "setangle <motIdx> <val rangle> <time ms>"},
	{ NULL,       NULL,      0, NULL},
};




// motor control command
#define MAX_MOTOR 4
int motorImpLenFinal[MAX_MOTOR]={ANGLE_0 + D_ANGLE_RPI2/2,ANGLE_0 + D_ANGLE_RPI2/2,ANGLE_0 + D_ANGLE_RPI2/2,ANGLE_0 + D_ANGLE_RPI2/2};
int motorImpulseLengths[MAX_MOTOR] = {ANGLE_0 + D_ANGLE_RPI2/2,ANGLE_0 + D_ANGLE_RPI2/2,ANGLE_0 + D_ANGLE_RPI2/2,ANGLE_0 + D_ANGLE_RPI2/2};
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
	console_addCommandsList(motorConsoleMenu);
}

int motor_setAngle(int angle, int motorIndex, uint32 time)
{
	angle = DEG_RANGLE(angle);
	return m_setAngle(angle, motorIndex, time);
}

int m_setAngle(int angle, int motorIndex, uint32 time)
{
	int res = SUCCESS;
	if ((angle >= -ANGLE_UNIT*50/360) && (angle <= ANGLE_UNIT*50/360) && (motorIndex>=0) && (motorIndex < MAX_MOTOR))
	{
	    int wantedImpulse = ANGLE_0 + (RANGLE_PI4+angle)*D_ANGLE_RPI2/RANGLE_PI2;
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
	if (r_sqrt(x*x+y*y+z*z)> LEN1+LEN2+LEN3)
        return FAILURE;
	int lenM2Tip= r_sqrt(z*z + pow(r_sqrt(x*x + y*y)-LEN1, 2));
	if (lenM2Tip==0)
	{
		printf("setPosition error: x:%i, y:%i, z:%i (lenM2Tip=0)\r\n", x, y, z);
		return FAILURE;
	}
	int angle_motor1;
	if (y==0)
	{
		angle_motor1=DIRECTION1*ANGLE1;
	}
	else
	{
		angle_motor1 = DIRECTION1 * (r_atanD(x, y) + ANGLE1);
	}
	int angle_motor3 = DIRECTION3 * (r_acosD(lenM2Tip*lenM2Tip-LEN2*LEN2-LEN3*LEN3, -2*LEN2*LEN3)+ANGLE3);
	int angle_motor2 = DIRECTION2 * (r_acosD(z, lenM2Tip) + 2048 + r_acosD(LEN3*LEN3-lenM2Tip*lenM2Tip-LEN2*LEN2, -2*lenM2Tip*LEN2) +ANGLE2);
	//printf("pod_setPosition: lenM2Tip: %i; angle1: %i; angle2: %i; angle3: %i\r\n", lenM2Tip, angle_motor1, angle_motor2, angle_motor3);
    while (angle_motor1 >  (ANGLE_UNIT-RANGLE_PI4)) angle_motor1-=ANGLE_UNIT;
	while (angle_motor1 < -(ANGLE_UNIT-RANGLE_PI4)) angle_motor1+=ANGLE_UNIT;
	while (angle_motor2 >  (ANGLE_UNIT-RANGLE_PI4)) angle_motor2-=ANGLE_UNIT;
	while (angle_motor2 < -(ANGLE_UNIT-RANGLE_PI4)) angle_motor2+=ANGLE_UNIT;
	while (angle_motor3 >  (ANGLE_UNIT-RANGLE_PI4)) angle_motor3-=ANGLE_UNIT;
	while (angle_motor3 < -(ANGLE_UNIT-RANGLE_PI4)) angle_motor3+=ANGLE_UNIT;
	int res1 = m_setAngle(angle_motor1, motor1, time);
	int res2 = m_setAngle(angle_motor2, motor2, time);
	int res3 = m_setAngle(angle_motor3, motor3, time);
	if ( res1==SUCCESS && res2==SUCCESS && res3==SUCCESS )
		return SUCCESS;
	else
	{
		printf("setPosition error: x:%i, y:%i, z:%i\r\n", x, y, z);
		return FAILURE;
	}
}

void show_motors(void)
{
	int angle, i;
	for (i=0; i<MAX_MOTOR; i++)
	{
		angle=RANGLE_DEG(RANGLE_PI2*(motorImpulseLengths[i]-ANGLE_0)/D_ANGLE_RPI2)-45;
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



static int setAngle(uint8 *args[], int argc)
{
	int motorIdx = atoi(args[1]);
	int angle=0, time=0;
	if (argc>2)
		angle = atoi(args[2]);
	if (argc>3)
		time = atoi(args[3]);

	printf("setAngle function %i \r\n", angle);
	int res = motor_setAngle(angle,motorIdx, time);
	if (res != SUCCESS)
	{
		printf("could not apply requested angle\r\n");
	}
	return 0;
}

static int setPosition(uint8 *args[], int argc)
{
	int x=atoi(args[1]);
	int y=atoi(args[2]);
	int z=atoi(args[3]);
	int motor1=0, motor2=1, motor3=2, time=500;
	if (argc>4)
	{
		time = atoi(args[4]);
	}
	if (argc>7)
	{
		motor1=atoi(args[5]);
		motor2=atoi(args[6]);
		motor3=atoi(args[7]);
	}
	printf("setPosition fuction %i %i %i, time: %i  \r\n", x, y, z, time);
	if (pod_setPosition(x, y, z, time, motor1, motor2, motor3)!=SUCCESS)
		printf("could not apply requested position\r\n");
	return 0;
}

static int msetAngle(uint8 *args[], int argc)
{
	int motorIdx = atoi(args[1]);
	int angle = atoi(args[2]);
	int speed = atoi(args[3]);

	printf("msetAngle function %i \r\n", angle);
	int res = m_setAngle(angle,motorIdx, speed);
	if (res != SUCCESS)
	{
		printf("could not apply requested angle\r\n");
	}
	return 0;
}
static int motors_state(uint8 *args[], int argc)
{
	printf("motors state:\r\n");
	show_motors();
	return 0;
}