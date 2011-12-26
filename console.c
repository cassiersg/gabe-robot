#include <plib.h>
#include <string.h>
#include <stdio.h>
#include "robot_console.h"

/* maximum number of char in one command */
#define MAX_CMD_LEN 512

/* local functions */
static int  splitCmd(uint8 *theCmd, uint8 *args[], int maxArg);
static int  printHelp(uint8 *args[], int argc);
static void processCmd(uint8 *theCmd, int cmdLen);



/* cmd entry is the basic element used to
   construct the menu list
*/
CmdEntry localMenu[] = {
	{ "help",        printHelp,      1, "this is printing help"},
	{ NULL,       NULL,      0, NULL},
};
CmdEntry *headMenu[MAX_COMMAND_LIST];
int numberOfMenu = 0;


/* maximum number of arguments of a function (+1 : name of the function) */
#define MAX_ARG 8

uint8 theCmd[MAX_CMD_LEN];
int cmdLen = 0;
int totalStr = 0;

void console_init(int pbClk, int desiredBaudRate)
{
	// Module is ON, Enable TX & RX, baudrate is determined by the caller, 8-N-1
	OpenUART2(UART_EN, UART_RX_ENABLE | UART_TX_ENABLE, pbClk/16/desiredBaudRate-1);		
	console_addCommandsList(localMenu);
}

// add a list of command to process. Each list is terminated by an entry where theCmd is NULL
void console_addCommandsList(CmdEntry *cmdList)
{
	if (numberOfMenu < MAX_COMMAND_LIST)
		headMenu[numberOfMenu++] = cmdList;
	else
	{
		printf("Error: too many command list\n");
	}
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
		
        int i;
		int notFound = 1;
		CmdEntry *entry = NULL;
		for (i=0;(i<numberOfMenu) && (notFound);i++)
		{
			entry = headMenu[i];
			while ((entry->theCmd != NULL) && notFound)
			{
				int entryDoesNotMatch = strcmp(entry->theCmd, args[0]);
				if (0==entryDoesNotMatch)
				{
					notFound = 0; // we found en entry
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
		}
		if ((entry == NULL) || (entry->theCmd == NULL))
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
	printf("commands:\r\n\r\n");
	int i,j;
	for (i=0; i<numberOfMenu;i++)
	{
		CmdEntry *menu = headMenu[i];
		for (j=0; menu[j].theCmd != NULL; j++)
		{
			printf("%s  ", menu[j].theCmd);
			int k = strlen(menu[j].theCmd);
			while (12-k >0)
			{
				k++;
				printf(" ");
			}
			printf("%s\r\n", menu[j].theHelp);
		}
	}
	return 0;
}

