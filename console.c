#include <plib.h>
#include "robot_console.h"

/* include interfaces controlled through the console */
#include "motor_control.h"


/* maximum number of char in one command */
#define MAX_CMD_LEN 512

/* local functions */
static int  splitCmd(uint8 *theCmd, uint8 *args[], int maxArg);
static int  printHelp(uint8 *args[], int argc);
static void processCmd(uint8 *theCmd, int cmdLen);
static int  setAngle(uint8 *args[], int argc);
static int  setPosition(uint8 *args[], int argc);
static int msetAngle(uint8 *args[], int argc);


/* cmd entry is the basic element used to
   construct the menu list
*/
typedef struct CmdEntry CmdEntry;
struct CmdEntry
{
	uint8 *theCmd;
	int (*processingFunction)(uint8 *args[], int argc);
	uint8 minArgs;
	uint8 *theHelp;
};

CmdEntry headMenu[] = {
	{ "help",        printHelp,   1, "this is printing help"},
	{ "setangle",    setAngle,    4, "setangle <motIdx> <val degre> <speed °/sec>"},
	{ "sa",          setAngle,    4, "setangle <motIdx> <val degre> <speed °/sec>"},
	{ "setposition", setPosition, 7, "setposition <x> <y> <z> <motor1> <motor2> <motot3>"},
	{ "sp",          setPosition, 4, "setposition <x> <y> <z>"},
	{ "sad",         msetAngle,   4, "setangle <motIdx> <val rangle> <speed °/sec>"},
	{ NULL,       NULL,      0, NULL},
};

/* maximum number of arguments of a function (+1 : name of the function) */
#define MAX_ARG 7

uint8 theCmd[MAX_CMD_LEN];
int cmdLen = 0;
int totalStr = 0;

void console_init(int pbClk, int desiredBaudRate)
{
	OpenUART2(UART_EN, 					// Module is ON
	UART_RX_ENABLE | UART_TX_ENABLE,	// Enable TX & RX
	pbClk/16/desiredBaudRate-1);		// 9600 bps, 8-N-1.
}


/* function which verify whether
   there is something to do for the console
   If a command is detected, it will immediately
   handle the command
*/
void console_process(void)
{
	if (DataRdyUART2())
	{
       	uint8 newChar = (char)ReadUART2(); // Read data from Rx.
 		theCmd[cmdLen] = newChar;
		cmdLen++;
		if ((newChar == '\n') || (newChar == '\r'))
		{
			putcUART2('\r');
			putcUART2('\n');
			processCmd(theCmd, cmdLen);
			cmdLen = 0;
		}
		else
		{
			// handle special characters here
			putcUART2(newChar);
			if (newChar == 127)  // this is backspace
			{
				cmdLen -=2; // remove the backspace it self and the previous char
				if (cmdLen < 0) // icheck if backspace was not the first char on the line
					cmdLen = 0;
			}
			//printf("last char is %i\r\n",newChar);
			if (cmdLen == MAX_CMD_LEN)
			{
            	totalStr++;
                theCmd[cmdLen-1] = 0;
				printf("unknown command %i %s\n\r", totalStr,theCmd);
				cmdLen = 0;
			}
		}
	}
}


/*
	splitCmd is a function which splits a command into separate args.
	It fills in an array of pointers with pointers to each arg.
    Each space in the cmd will be replaced with 0
	The function returns -1 is there are more args than allowed.
	"theCmd" must be null terminated
*/
static int splitCmd(uint8 *theCmd, uint8 *args[], int maxArg)
{
	int nextArg = 0;
	int idx = 0;

	#define SEARCH_START 0
	#define SEARCH_END 1

	int state = SEARCH_START;
	uint8 nextChar = theCmd[0];
	while (nextChar != 0)
	{
		if ((state==SEARCH_START) && (nextChar != ' '))
		{
			if (nextArg >= maxArg)
			{
				// too many args
				nextArg = -1;
				break;
			}
			args[nextArg] = &theCmd[idx];
			state = SEARCH_END;
			nextArg++;
		}
		if (nextChar == ' ')
		{
			theCmd[idx] = 0;
			state=SEARCH_START;
		}
		idx++;
		nextChar = theCmd[idx];
	}
	// lets point the non existing args to the last NULL char
	int res = nextArg;
	if (nextArg >=0)
	{
		while (nextArg<maxArg)
		{
			args[nextArg] = &theCmd[idx];
			nextArg++;
		}
	}
	return res;
}


static void processCmd(uint8 *theCmd, int cmdLen)
{
	uint8 *args[MAX_ARG];
	if (cmdLen > 1)
	{
		theCmd[cmdLen-1] = 0;
		int res = splitCmd(theCmd, args, MAX_ARG);

		// go through the menu list and select the right entry
		CmdEntry *entry = &headMenu[0];
		while (entry->theCmd != NULL)
		{
			int entryDoesNotMatch = strcasecmp(entry->theCmd, args[0]);
			if (0==entryDoesNotMatch)
			{
				printf("res=%i; arg[0]=%s,arg[1]=%s,arg[2]=%s,arg[3]=%s\r\n",
				   res,args[0],args[1],args[2],args[3]);

				if (res >= entry->minArgs)
					(entry->processingFunction)(args, res);
				else
					printf("too few args for %s : %s\r\n", args[0], entry->theHelp);
				break;
			}
			entry++;
		}
		if (entry->theCmd == NULL)
		{
			// we did not found an valid entry
			printf("INVALID CMD REC: [%s]\r\n", theCmd);
			printf("res=%i; arg[0]=%s,arg[1]=%s,arg[2]=%s,arg[3]=%s\r\n",
			   res,args[0],args[1],args[2],args[3]);
		}
	}
	// print the shell
	putsUART2(">>> ");
}


static int printHelp(uint8 *args[], int argc)
{
	printf("this is the main help function \r\n");
	return 0;
}


static int setAngle(uint8 *args[], int argc)
{
	int motorIdx = atoi(args[1]);
	int angle = atoi(args[2]);
	int speed = atoi(args[3]);

	printf("setAngle function %i \r\n", angle);
	int res = motor_setAngle(angle,motorIdx, speed);
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
	int motor1, motor2, motor3;
	if (argc==7)
	{
		motor1=atoi(args[4]);
		motor2=atoi(args[5]);
		motor3=atoi(args[6]);
	}
	else
	{
		motor1=0;
		motor2=1;
		motor3=2;
	}
	printf("setPosition fuction %i %i %i  \r\n", x, y, z);
	if (pod_setPosition(x, y, z, motor1, motor2, motor3)!=SUCCESS)
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
