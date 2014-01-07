#include <plib.h>
#include <stdio.h>

#include "motor_control.h"
#include "robot_time.h"
#include "robot_math.h"
#include "robot_console.h"
#include "historical.h"

#define MAX_MOTOR 12

#define SERVO_MIN_PERIOD (TICKS_PER_MSEC *18)  // 18 msecs
#define MIN_DURATION     100 // min wait time


//~~~~~~~~~~ GLOBAL MOTORS VARIABLES ~~~~~~~~~~

// specifications of a motor type
typedef const struct MotorType MotorType;
struct MotorType
{
	int minImpLen;
	int maxImpLen;
    int impPi2;
    int midImpLen;
};

#define INIT_MOTOR_TYPE(minImp, maxImp, impPi2) {(minImp), (maxImp), (impPi2), ((maxImp)+(minImp))/2}
static MotorType blueMotor = INIT_MOTOR_TYPE(1550, 3850, 1840);
static MotorType darkMotor = INIT_MOTOR_TYPE(1400, 5000, 2300);
#undef INIT_MOTOR_TYPE


// for "virtual void motors" -> used at end of loop
static MotorType noneMotor = {0, 0, 0};

typedef struct MotorState MotorState;
struct MotorState
{
    int currentImpLen;
    int finalImpLen;
    int nbPeriods_toFinal;
};

// addresses to toggle IO ports
#define PORT_D_INV ((uint32 *)0xBF8860EC)
#define PORT_E_INV ((uint32 *)0xBF88612C)

// physical interface of a motor
typedef struct Motor Motor;
struct Motor
{
    MotorType *type;
    uint32 *toggleRegister;
    uint32 toggleValue; // bitmask
    MotorState state;
    int motorIdx;
};

// Provide a valid writable address for "virtual motors"
static uint32 nullVar;
/* states are initialized in initialization function */
static Motor motors[MAX_MOTOR+2] = 
{
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


//~~~~~~~~~~ MOTORS COMMAND ~~~~~~~~~~

static int testImpLen(int impLen, Motor *motor);
static int setImpLen(int impLen, Motor *motor, uint32 duration);
static int rAngle2impLen(int angle, Motor *motor);


static int testImpLen(int impLen, Motor *motor)
{
	MotorType *type = motor->type;
	int res = type->minImpLen <= impLen && type->maxImpLen >= impLen;
	return res ? SUCCESS : FAILURE;
}
			
static int setImpLen(int impLen, Motor *motor, uint32 duration)
{
	if (testImpLen(impLen, motor) == SUCCESS)
	{
    	motor->state.nbPeriods_toFinal = duration*TICKS_PER_MSEC/SERVO_MIN_PERIOD +1;
    	motor->state.finalImpLen = impLen;
    	return SUCCESS;
	}
	else
	{
		printf("SetAngle error : impulseLength %i out of range for motor %p\r\n", impLen, motor);
		return FAILURE;
	}
}

static int rAngle2impLen(int angle, Motor *motor)
{
	MotorType *type = motor->type;
    return type->midImpLen + angle * type->impPi2 / RANGLE_PI2;
}

int testMotor(int angle, unsigned int motorIdx)
{
    if (motorIdx >= MAX_MOTOR)
        return FAILURE;
    Motor *motor = &motors[motorIdx];
    return testImpLen(rAngle2impLen(angle, motor), motor);
}

int setMotor(int angle, unsigned int motorIdx, uint32 duration)
{
    if (testMotor(angle, motorIdx) == FAILURE)
        return FAILURE;
    Motor *motor = &motors[motorIdx];
	return setImpLen(rAngle2impLen(angle, motor), motor, duration);
}


//~~~~~~~~~~ CONSOLE APPLICATION ~~~~~~~~~~

#define DEFAULT_MOVE_DURATION 500  // ms
#define DEFAULT_MOTOR_IDX  0

static int c_setImp(char *args[], int argc);
static int c_setAngle_ra(char *args[], int argc);
static int c_setAngle_deg(char *args[], int argc);
static int c_motors_state(char *args[], int argc);
static int c_testRa(char *args[], int argc);
static void show_motors(void);

static CmdEntry motorConsoleMenu[] = {
	{ "setangle",    c_setAngle_deg, 2, "setangle <val degrees> <motIdx>[=0] <duration ms>[=100]"},
	{ "sa",          c_setAngle_deg, 2, "Alias of setangle"},
	{ "mstate",      c_motors_state, 1, "show angles of the motors"},
	{ "sra",         c_setAngle_ra,  2, "setRAngle <val rangle> <motIdx>[=0] <duration ms>[=100]"},
	{ "simp",        c_setImp,       2, "setangle <impLen> <mot Idx>[=0] <time ms>[=0]"},
	{ "tra",         c_testRa,       2, "tra <val rangle> <mot Idx>[=0]"},
	{ NULL,       NULL,      0, NULL},
};

static int c_setImp(char *args[], int argc)
{
    unsigned int motorIdx = DEFAULT_MOTOR_IDX;
    uint32 duration = DEFAULT_MOVE_DURATION;
    unsigned int impLen = atoi(args[1]);
    if (argc>2)
        motorIdx = atoi(args[2]);
        if (argc>3)
        duration = atoi(args[3]);
    printf("setImpLen function %i ms\r\n", impLen);
    if (motorIdx >= MAX_MOTOR || setImpLen(impLen, &motors[motorIdx], duration) != SUCCESS) {
        printf("could not apply requested impLen\r\n");
    }
    return SUCCESS;
}

static int c_setAngle_ra(char *args[], int argc)
{
    int motorIdx = DEFAULT_MOTOR_IDX;
    int duration = DEFAULT_MOVE_DURATION;
    int angle = atoi(args[1]);
	if (argc>2)
        motorIdx = atoi(args[2]);
	if (argc>3)
        duration = atoi(args[3]);
	printf("setAngle function %i RA\r\n", angle);
	int res = setMotor(angle, motorIdx, duration);
	if (res != SUCCESS)
	{
		printf("could not apply requested angle\r\n");
	}
	return SUCCESS;
}

static int c_setAngle_deg(char *args[], int argc)
{
    unsigned int motorIdx = DEFAULT_MOTOR_IDX;
    uint32 duration = DEFAULT_MOVE_DURATION;
    int angle = atoi(args[1]);
    if (argc>2)
        motorIdx = atoi(args[2]);
        if (argc>3)
        duration = atoi(args[3]);
    printf("setAngle function %i degrees\r\n", angle);
	int res = setMotor(DEG2RANGLE(angle), motorIdx, duration);
	if (res != SUCCESS) {
		printf("could not apply requested angle\r\n");
	}
	return SUCCESS;
}

static int c_motors_state(char *args[], int argc)
{
    printf("motors state:\r\n");
    show_motors();
    return SUCCESS;
}

static int c_testRa(char *args[], int argc)
{
    int motorIdx = DEFAULT_MOTOR_IDX;
    int angle = atoi(args[1]);
	if (argc>2)
        motorIdx = atoi(args[2]);
	printf("testRAngle function %i RA\r\n", angle);
	int res = testMotor(angle, motorIdx);
	if (res != SUCCESS)
	{
		printf("could not apply requested angle\r\n");
	}
	else
	{
    	printf("It's OK\r\n");
    }   	
	return SUCCESS;
}

static void show_motors(void)
{
	int impLen, rAngle, angle;
	int i;
	Motor *motor;
	MotorType *type;
	for (i=0; i<MAX_MOTOR; i++)
	{
    	motor = &motors[i];
    	type = motor->type;
    	impLen = motor->state.currentImpLen;
    	rAngle = RANGLE_PI2 * (impLen - type->midImpLen) / type->impPi2;
		angle = RANGLE2DEG(rAngle);
		printf("Motor %2i : %4u mus : %2i RAngle : %2i degrees\r\n", i, impLen, rAngle, angle);
    }
}


//~~~~~~~~~~ INITIALIZATION ~~~~~~~~~~
        
void motor_init(void)
{
    // init state variables
    int i;
    Motor *motor;
    MotorState *mstate;
    unsigned int midImpLen;
    for (i=0; i<MAX_MOTOR+2; i++)
    {
        motor = &motors[i];
        mstate = &(motor->state);
        midImpLen = motor->type->midImpLen;
        mstate->currentImpLen = midImpLen;
        mstate->finalImpLen = midImpLen;
        mstate->nbPeriods_toFinal = 0;
    }
    
	// set pins mode (ports D and E)
	PORTSetPinsDigitalOut(IOPORT_D, BIT_1 | BIT_2 | BIT_3 | BIT_4 | BIT_5 |
									BIT_6 | BIT_7);
	PORTSetPinsDigitalOut(IOPORT_E, BIT_0 | BIT_1 | BIT_2 | BIT_3 | BIT_4);
	// clear all pins
	PORTClearBits(IOPORT_D, BIT_1 | BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6 | BIT_7);
	PORTClearBits(IOPORT_E, BIT_0 | BIT_1 | BIT_2 | BIT_3 | BIT_4);

	// once the ports are correctly configured, start the timer
	// and configure the associated interrupt
	/* Timer 3 is configured to run at 1.125 MHz :
          72MHz/2/32 (we are on Periph clock) /32 (divider)
	*/
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_4);
    OpenTimer3(T3_ON | T3_PS_1_32, SERVO_MIN_PERIOD);

	// motor control by console
	console_addCommandsList(motorConsoleMenu);
}

//~~~~~~~~~~ INTERRUPT HANDLER ~~~~~~~~~~

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
	static int motorOn1Idx   = 0;
	static int motorOn2Idx   = 0;
	static int nextMotorIdx  = 0;
	static int nextDuration  = MIN_DURATION;
	static int totalDuration = 0;
	
	int duration;
    
	// end of loop
    if (nextMotorIdx >= MAX_MOTOR + 1)
	{
    	// turn off last pin
    	*(motors[motorOn1Idx].toggleRegister) = motors[motorOn1Idx].toggleValue;
    	// wait for a complete cycle, with at least MIN_DURATION sleep time
    	duration = SERVO_MIN_PERIOD - totalDuration;
    	if (duration < MIN_DURATION)
    	{
        	duration = MIN_DURATION;
        }   	
        //reinit all statics variables
    	motorOn1Idx = MAX_MOTOR;
    	motorOn2Idx = MAX_MOTOR;
    	nextMotorIdx = 0;
    	nextDuration = MIN_DURATION;
    	totalDuration = 0;
    }
    else
    {
        Motor *nextMotor = &motors[nextMotorIdx];
        MotorState *nextMotorState = &nextMotor->state;
        
        //toggle previous motor and next motor
	    *(motors[motorOn1Idx].toggleRegister) = motors[motorOn1Idx].toggleValue;
	    *(nextMotor->toggleRegister) = nextMotor->toggleValue;
	    
	    // Compute duration for next motor
        if (nextMotorState->nbPeriods_toFinal <= 0)	 // move finished
	    {
    	    nextMotorState->currentImpLen = nextMotorState->finalImpLen;
    	}
    	else
    	{
        	nextMotorState->currentImpLen += (nextMotorState->finalImpLen - nextMotorState->currentImpLen) 
        	                                        / nextMotorState->nbPeriods_toFinal;
        	nextMotorState->nbPeriods_toFinal--;
        }
	    // save refresh in the historical
        if (nextMotorState->nbPeriods_toFinal != 0)
	    {
    	    mrefresh_save(nextMotorIdx, nextMotorState->currentImpLen, nextMotorState->nbPeriods_toFinal);
    	}
    	// Select the next "motorOn1", adapt next sleep duration
        if (nextDuration <= nextMotorState->currentImpLen || nextMotorIdx >= MAX_MOTOR)
    	{
        	duration = nextDuration;
            nextDuration = nextMotorState->currentImpLen - nextDuration;
        	motorOn1Idx = motorOn2Idx;
        	motorOn2Idx = nextMotorIdx;
        }
        else
        {
            duration = nextMotorState->currentImpLen;
            nextDuration -= nextMotorState->currentImpLen;
            motorOn1Idx = nextMotorIdx;
        }
        nextMotorIdx++;
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
