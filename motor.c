#include <plib.h>
#include <stdio.h>
#include <math.h>

#include "robot_time.h"
#include "motor_control.h"
#include "robot_math.h"
#include "robot_console.h"
#include "historical.h"

#define MAX_MOTOR 12
#define NB_PODS   4

#define SERVO_MIN_PERIOD (TICKS_PER_MSEC *18)  // 18 msecs
#define MIN_DURATION     100 // min wait time

uint32 nullVar;

static void m_setAngle_imp(int impulseLength, int motorIndex, uint32 time);
static int  m_testRAngle(int angle, int motorIndex);
static void m_setAngle_ra_noTest(int angle, int motorIndex, uint32 time);

static int c_setAngle_ra(uint8 *args[], int argc);
static int c_setAngle_deg(uint8 *args[], int argc);
static int c_setAngle_imp(uint8 *args[], int argc);
static int c_setPosition(uint8 *args[], int argc);
static int c_motors_state(uint8 *args[], int argc);


//~~~~~~~~~~ MOTORS STATE ~~~~~~~~~~

// description of the features of a motor type
typedef struct MotorType MotorType;
struct MotorType
{
    int impPi2;
    int midImpLen;
    int deltaRangle;
};
#define BM_MIN_IMP 1550
#define BM_MAX_IMP 3850
#define BM_IMP_PI2 1840
#define DM_MIN_IMP 1400
#define DM_MAX_IMP 5000
#define DM_IMP_PI2 2300
MotorType blueMotor = {BM_IMP_PI2, (BM_MAX_IMP+BM_MIN_IMP)/2, (BM_MAX_IMP-BM_MIN_IMP)*RANGLE_PI2/2/BM_IMP_PI2};
MotorType darkMotor = {DM_IMP_PI2, (DM_MAX_IMP+DM_MIN_IMP)/2, (DM_MAX_IMP-DM_MIN_IMP)*RANGLE_PI2/2/DM_IMP_PI2};
#undef BM_MIN_IMP
#undef BM_MAX_IMP
#undef BM_IMP_PI2
#undef DM_MIN_IMP
#undef DM_MAX_IMP
#undef DM_IMP_PI2

// for unused "motors"
MotorType noneMotor = {0, 0, 0};

#define PORT_D_INV ((uint32 *)0xBF8860EC)
#define PORT_E_INV ((uint32 *)0xBF88612C)

// physical interface of a motor
typedef struct Amotor Amotor;
struct Amotor
{
    MotorType *type;
    uint32 *toggleRegister;
    uint32 toggleValue;
};

Amotor motors[MAX_MOTOR+2] = {
    {&darkMotor, PORT_D_INV, 1<<1},
    {&darkMotor, PORT_D_INV, 1<<2},
    {&blueMotor, PORT_D_INV, 1<<3},
    {&darkMotor, PORT_D_INV, 1<<4},
    {&darkMotor, PORT_D_INV, 1<<5},
    {&blueMotor, PORT_D_INV, 1<<6},
    {&darkMotor, PORT_D_INV, 1<<7},
    {&darkMotor, PORT_E_INV, 1<<0},
    {&blueMotor, PORT_E_INV, 1<<1},
    {&darkMotor, PORT_E_INV, 1<<2},
    {&darkMotor, PORT_E_INV, 1<<3},
    {&blueMotor, PORT_E_INV, 1<<4},
    {&noneMotor, &nullVar,   0},
    {&noneMotor, &nullVar,   0},
};

typedef struct Apod Apod;
struct Apod
{
    int m1Idx;
    int m2Idx;
    int m3Idx;
};

Apod pods[NB_PODS] = {
      { 0,  1,  2},
      { 3,  4,  5},
      { 6,  7,  8},
      { 9, 10, 11},
};      

int motorImpLenFinal[MAX_MOTOR+3] = {0};
int motorImpulseLengths[MAX_MOTOR+3] = {0};
int numberPeriod_toFinal[MAX_MOTOR+3] = {0};


//~~~~~~~~~~ CONSOLE APPLICATION ~~~~~~~~~~

CmdEntry motorConsoleMenu[] = {
	{ "setangle",    c_setAngle_deg, 2, "setangle <val degrees> <motIdx> <time ms>"},
	{ "sa",          c_setAngle_deg, 2, "setangle <val degrees> <motIdx> <time ms>"},
	{ "setposition", c_setPosition,  8, "setposition <x> <y> <z> <time ms> <motor1> <motor2> <motot3>"},
	{ "sp",          c_setPosition,  4, "setposition <x> <y> <z> <time ms> <motor1> <motor2> <motot3>"},
	{ "mstate",      c_motors_state, 1, "angles of the motors"},
	{ "sra",         c_setAngle_ra,  2, "setangle <val rangle> <motIdx> <time ms>"},
	{ "simp",        c_setAngle_imp, 2, "setangle <val imp len> <mot Idx> <time ms>"},
	{ NULL,       NULL,      0, NULL},
};

static int c_setAngle_imp(uint8 *args[], int argc)
{
    int motorIdx = atoi(args[1]);
    int angle=0, time=100;
    if (argc>2)
    {
        angle = atoi(args[2]);
        if (argc>3)
            time = atoi(args[3]);
    }
    printf("setAngle function %i impluses\r\n", angle);
    m_setAngle_imp(angle, motorIdx, time);
    return 0;
}

static int c_setAngle_ra(uint8 *args[], int argc)
{
	int motorIdx = atoi(args[1]);
	int angle=0, time=100;
	if (argc>2)
		angle = atoi(args[2]);
	if (argc>3)
		time = atoi(args[3]);

	printf("setAngle function %i RA\r\n", angle);
	int res = m_setAngle_ra(angle,motorIdx, time);
	if (res != SUCCESS)
	{
		printf("could not apply requested angle\r\n");
	}
	return 0;
}

static int c_setAngle_deg(uint8 *args[], int argc)
{
    int motorIdx = atoi(args[1]);
    int angle=0, time=100;
    if (argc>2)
    {
        angle = atoi(args[2]);
        if (argc>3)
            time = atoi(args[3]);
    }
    printf("setAngle function %i degrees\r\n", angle);
	int res = m_setAngle_deg(angle,motorIdx, time);
	if (res != SUCCESS)
	{
		printf("could not apply requested angle\r\n");
	}
	return 0;
}


static int c_setPosition(uint8 *args[], int argc)
{
	int x=atoi(args[1]);
	int y=atoi(args[2]);
	int z=atoi(args[3]);
	int time=500, podIdx=0;
	if (argc>4)
	{
		time = atoi(args[4]);
    }
	if (argc>5)
	{
		podIdx=atoi(args[5]);
	}
	printf("setPosition fuction %i %i %i, time: %i  \r\n", x, y, z, time);
	if (pod_setPosition(x, y, z, time, podIdx)!=SUCCESS)
		printf("could not apply requested position\r\n");
	return 0;
}

static int c_motors_state(uint8 *args[], int argc)
{
    printf("motors state:\r\n");
    show_motors();
    return 0;
}

        
//~~~~~~~~~~ MOTORS COMMAND ~~~~~~~~~~

void motor_init(void)
{
    // init state variables
    int i;
    for (i=0; i<MAX_MOTOR+2; i++)
    {
        motorImpLenFinal[i] = motors[i].type->midImpLen;
        motorImpulseLengths[i] = motors[i].type->midImpLen;
        numberPeriod_toFinal[i] = 0;
    }
    
	// set pins (ports RD and RE)
	PORTSetPinsDigitalOut(IOPORT_D, BIT_1);
    mPORTDClearBits(BIT_1);
    PORTSetPinsDigitalOut(IOPORT_D, BIT_2);
    mPORTDClearBits(BIT_2);
	PORTSetPinsDigitalOut(IOPORT_D, BIT_3);
    mPORTDClearBits(BIT_3);
	PORTSetPinsDigitalOut(IOPORT_D, BIT_4);
    mPORTDClearBits(BIT_4);
	PORTSetPinsDigitalOut(IOPORT_D, BIT_5);
    mPORTDClearBits(BIT_5);
    PORTSetPinsDigitalOut(IOPORT_D, BIT_6);
    mPORTDClearBits(BIT_6);
    PORTSetPinsDigitalOut(IOPORT_D, BIT_7);
    mPORTDClearBits(BIT_7);
    PORTSetPinsDigitalOut(IOPORT_E, BIT_0);
    mPORTEClearBits(BIT_0);
    PORTSetPinsDigitalOut(IOPORT_E, BIT_1);
    mPORTEClearBits(BIT_1);
    PORTSetPinsDigitalOut(IOPORT_E, BIT_2);
    mPORTEClearBits(BIT_2);
    PORTSetPinsDigitalOut(IOPORT_E, BIT_3);
    mPORTEClearBits(BIT_3);
    PORTSetPinsDigitalOut(IOPORT_E, BIT_4);
    mPORTEClearBits(BIT_4);

	// once the port is correctly configured, start the timer
	// and configure the associated interrupt
	/* Timer 3 is configured to run at 1.125 MHz :
          72MHz/2/32 (we are on Periph clock) /32 (divider)
	*/
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_4);
    OpenTimer3(T3_ON | T3_PS_1_32, SERVO_MIN_PERIOD);
	console_addCommandsList(motorConsoleMenu);
}

static void m_setAngle_imp(int impulseLength, int motorIndex, uint32 time)
{
    numberPeriod_toFinal[motorIndex] = time*TICKS_PER_MSEC/SERVO_MIN_PERIOD +1;
    motorImpLenFinal[motorIndex] = impulseLength;
}

static int m_testRAngle(int angle, int motorIndex)
{
    return  0 <= motorIndex && 
            motorIndex < MAX_MOTOR && 
            R_ABS(angle) <= motors[motorIndex].type->deltaRangle;
}


static void m_setAngle_ra_noTest(int angle, int motorIndex, uint32 time)
{
    MotorType *type = motors[motorIndex].type;
    int impLen = type->midImpLen + angle * type->impPi2 / RANGLE_PI2;
    return m_setAngle_imp(impLen, motorIndex, time);
}        

int m_setAngle_ra(int angle, int motorIndex, uint32 time)
{
    if (m_testRAngle(angle, motorIndex))
    {
        m_setAngle_ra_noTest(angle, motorIndex, time);
        return SUCCESS;
    }
    else
    {
        printf("ERROR setAngle: motorIdx %i, angle %i\r\n", motorIndex, angle);
        return FAILURE;
    }
}        

int m_setAngle_deg(int angle, int motorIndex, uint32 time)
{
	angle = DEG_RANGLE(angle);
	return m_setAngle_ra(angle, motorIndex, time);
}
    
void show_motors(void)
{
	int angle, i;
	MotorType *type;
	for (i=0; i<MAX_MOTOR; i++)
	{
    	type = motors[i].type;
		angle=RANGLE_DEG(RANGLE_PI2 * (motorImpulseLengths[i] - type->midImpLen) / type->impPi2);
		printf("Motor %i : %i degrees\r\n", i, angle);
	}
}


//~~~~~~~~~~ PODS MANGEMENT ~~~~~~~~~~

int pod_setPosition(int x, int y, int z, int time, int podIdx)
{
	/* point demande hors zone possible */
	if (r_sqrt(x*x+y*y+z*z)> LEN1+LEN2+LEN3)
        return FAILURE;
	int lenM2Tip = r_sqrt(z*z + SQUARE(r_sqrt(x*x + y*y)-LEN1));
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
	int angle_motor3 = DIRECTION3 * (r_acosD(SQUARE(lenM2Tip)-LEN2*LEN2-LEN3*LEN3, 
                  -2*LEN2*LEN3)+ANGLE3);
	int angle_motor2 = DIRECTION2 * (r_acosD(z, lenM2Tip) + RANGLE_PI2 + 
                  r_acosD(LEN3*LEN3-lenM2Tip*lenM2Tip-LEN2*LEN2, -2*lenM2Tip*LEN2) +ANGLE2);
    RANGLE_NORMALIZE(angle_motor1);
    RANGLE_NORMALIZE(angle_motor2);
    RANGLE_NORMALIZE(angle_motor3);
    //printf("pod_setPosition: lenM2Tip: %i; angle1: %i; angle2: %i; angle3: %i\r\n", lenM2Tip, angle_motor1, angle_motor2, angle_motor3);
	int res1 = m_setAngle_ra(angle_motor1, pods[podIdx].m1Idx, time);
	int res2 = m_setAngle_ra(angle_motor2, pods[podIdx].m2Idx, time);
	int res3 = m_setAngle_ra(angle_motor3, pods[podIdx].m3Idx, time);
	if ( res1==SUCCESS && res2==SUCCESS && res3==SUCCESS )
		return SUCCESS;
	else
	{
		printf("setPosition error: x:%i, y:%i, z:%i\r\n", x, y, z);
		return FAILURE;
	}
}


//~~~~~~~~~~ INTERRUPT MANAGEMENT ~~~~~~~~~~

/*
 the motor active period is set by the timer 3. Period are being set after each other.
 Once all motors have been activited, we wait the end of a global period.
 The motor period is automatically adapted before the impulse depending on the speed.
 
 Two motors are activated together : when a motor is stopped, the other is kept running
 and a third is started. 
*/

void __ISR(_TIMER_3_VECTOR, ipl4) Timer3Handler(void)
{
	// motor control state
	static int motorOn1      = 0;
	static int motorOn2      = 0;
	static int motorNext     = 0;
	static int nextDuration  = MIN_DURATION;
	static int totalDuration = 0;
	
	int duration;
    
	// end of loop
    if (motorNext == MAX_MOTOR + 1)
	{
    	*(motors[motorOn1].toggleRegister) = motors[motorOn1].toggleValue;
    	duration = SERVO_MIN_PERIOD - totalDuration;
    	if (duration < MIN_DURATION)
    	{
        	duration = MIN_DURATION;
        }   	
        //reinit all statics variables
    	motorOn1 = MAX_MOTOR;
    	motorOn2 = MAX_MOTOR;
    	motorNext = 0;
    	nextDuration = MIN_DURATION;
    	totalDuration = 0;
    }
    else
    {
        //toggle previous motor and next motor
	    *(motors[motorOn1].toggleRegister) = motors[motorOn1].toggleValue;
	    *(motors[motorNext].toggleRegister) = motors[motorNext].toggleValue;
	    
	    // duration for next motor
	    if (numberPeriod_toFinal[motorNext] == 0)
	    {
    	    motorImpulseLengths[motorNext] = motorImpLenFinal[motorNext];
    	}
    	else
    	{
        	motorImpulseLengths[motorNext] += (motorImpLenFinal[motorNext] - motorImpulseLengths[motorNext]) / numberPeriod_toFinal[motorNext];
        	numberPeriod_toFinal[motorNext]--;
        }
	    // save refresh in the historical
	    if (numberPeriod_toFinal[motorNext] !=0)
	    {
    	    mrefresh_save(motorNext, motorImpulseLengths[motorNext], numberPeriod_toFinal[motorNext]);
    	}

    	// select nextDuration and update static values
    	if (nextDuration <= motorImpulseLengths[motorNext] || motorNext == MAX_MOTOR)
    	{
        	duration = nextDuration;
        	nextDuration = motorImpulseLengths[motorNext] - nextDuration;
        	motorOn1 = motorOn2;
        	motorOn2 = motorNext;
        }
        else
        {
            duration = motorImpulseLengths[motorNext];
            nextDuration -= motorImpulseLengths[motorNext];
            motorOn1 = motorNext;
        }
        motorNext++;
        totalDuration += duration;
    }
    // duration == 0 => timer OFF
    if (duration < 1)
    {
        duration = 1;
    }
    
    OpenTimer3(T3_ON | T3_PS_1_32, duration);
    
    //clear the interrupt flag
    mT3ClearIntFlag();
}    

