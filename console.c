#include <plib.h>
#include <string.h>
#include <stdio.h>
#include "robot_console.h"


/* local functions */
static int  splitCmd(char *theCmd, char *args[], int maxArg);
static int  printHelp(char *args[], int argc);
static void processCmd(char *theCmd, int cmdLen);

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

/* maximum number of char in one command */
#define MAX_CMD_LEN 512
char theCmd[MAX_CMD_LEN];
int cmdLen = 0;
int totalStr = 0;

void console_init(int pbClk, int desiredBaudRate)
{
	// Module is ON, Enable TX & RX, baudrate is determined by the caller, 8-N-1
	OpenUART(UART_EN, UART_RX_ENABLE | UART_TX_ENABLE, pbClk/16/desiredBaudRate-1);		

	// STEP 3. Configure UART2 RX Interrupt

	ConfigIntUART(UART_INT_PR3 | UART_RX_INT_EN | UART_TX_INT_DIS);

	console_addCommandsList(localMenu);
}

// add a list of command to process. Each list is terminated by an entry where theCmd is NULL
void console_addCommandsList(CmdEntry *cmdList)
{
	if (numberOfMenu < MAX_COMMAND_LIST)
		headMenu[numberOfMenu++] = cmdList;
	else
	{
		putsUART("Error: too many command list\n\r");
	}
}


/* To be done :
   interprete arrows key in order to retrieve previous commands

   when moving the arrow, the current command is replaced by an existing one.
   Arrows key's are display as 3 consecutive chars : 
    0x1b 0x5b 0x41 = up
    0x1b 0x5b 0x42 = down
    0x1b 0x5b 0x43 = left
    0x1b 0x5b 0x44 = rigth


Replay buffer:
 each time we execute a command, we copy it in the replay buffer.
 The up and down arrows allow to reload previous commands. 
 Each time the up or down arrow is touched, the pointer in 
 the replay buffer is moved and the pointer command is reloaded into the current string command.
 When "enter" is touched, the current string command is moved into the 
 replay buffer and the pointer is moved to the next free place.

 Each cmd string in the buffer is preceeded by the 2 bytes representing the length, 
 then the string itself (padded with 1 byte if the length is odd, and then the length again.
 The second length allows to go back.

*/

#define REPLAY_LENGTH 1500
uint16 replayBuffer[REPLAY_LENGTH];
int bufferNext = 0;
int bufferFirst= 0;
int bufferPointer = 0;

void storeCmdInReplayBuf(char *strCmd, int len)
{
	// to be completed. We should check whether the new command is the same as the last one
	{
		// check if lastLen is the same as the previous one
		int lastCmd = bufferNext-1;
		if (lastCmd<0)
			lastCmd += REPLAY_LENGTH;
		int lastLen = replayBuffer[lastCmd];
		// if so do a memcmp optionally in 2 steps
		if (lastLen == len)
		{
			int cmdIsDifferent = 0;
			char *tempNewCmd = strCmd;
			lastCmd -= (lastLen+1)>>1;
			if (lastCmd < 0)
				lastCmd += REPLAY_LENGTH;

			int spaceTillEnd = (REPLAY_LENGTH - lastCmd)<<1;
			if (spaceTillEnd < lastLen)
			{
				cmdIsDifferent = memcmp(tempNewCmd, &replayBuffer[lastCmd],spaceTillEnd);
				lastLen -= (spaceTillEnd>>1);
				lastCmd = 0;
				tempNewCmd += spaceTillEnd; 
			}
			cmdIsDifferent |= memcmp(tempNewCmd, &replayBuffer[lastCmd], lastLen);
			if (cmdIsDifferent == 0)
			{
				bufferPointer = bufferNext;
				return; // same command no need to store it
			}
		}		
	}
	// check whether there is enough place in the replay buffer
	int occupied = bufferNext - bufferFirst;
	if (occupied < 0)
		occupied += REPLAY_LENGTH;

	int freeSpace = REPLAY_LENGTH - occupied;
	int missingSpace = ((len+5)>>1)-(freeSpace-1);

	#if 0
	printf("copying new command size=%i, freeSpace=%i,missingSpace=%i [%i,%i]\n",
		len, freeSpace, missingSpace, bufferFirst, bufferNext);
	#endif
	while (missingSpace > 0)
	{
		// if not enough place in the replay buffer => drop n first cmd
		int firstLen = (replayBuffer[bufferFirst]+5)>>1;
		bufferFirst += firstLen;
		missingSpace -= firstLen;
		printf("drop %i\n",firstLen);
		if (bufferFirst >= REPLAY_LENGTH)
			bufferFirst -= REPLAY_LENGTH;
	}
	// we now have enough place
	// copy the cmd - could be in two step in case of wrap around, update bufferNext 
	replayBuffer[bufferNext++] = len;
	if (bufferNext >= REPLAY_LENGTH)
		bufferNext -= REPLAY_LENGTH;

	int freeSpaceTillEnd = REPLAY_LENGTH-bufferNext;
	int tempLen = len;
	if (freeSpaceTillEnd < ((len+1)>>1))
	{
		memcpy(&replayBuffer[bufferNext], strCmd, freeSpaceTillEnd<<1);
		bufferNext = 0; 
		tempLen -= freeSpaceTillEnd <<1;
		strCmd += freeSpaceTillEnd<<1;
	}
	memcpy(&replayBuffer[bufferNext], strCmd, tempLen);
	bufferNext += (tempLen+1)>>1;
	if (bufferNext >= REPLAY_LENGTH)
		bufferNext -= REPLAY_LENGTH;
	replayBuffer[bufferNext++] = len;
	if (bufferNext >= REPLAY_LENGTH)
		bufferNext -= REPLAY_LENGTH;
	 
	// and bufferPointer (which is reset each time we store a new command)
	bufferPointer = bufferNext;
}

// return true if a new command has been copied
int movePointer(char *strCmd, int *len, int isUp /* go to a previous command */)
{
	/* if move is possble copy newly pointed command in the strCmd, len */
	int copyFrom = -1; /* offset of the comman to copy -1 indicates no copy possible */
	if (isUp && (bufferPointer != bufferFirst))
	{
		/* we can go up */
		bufferPointer --;
		if (bufferPointer < 0)
			bufferPointer +=  REPLAY_LENGTH;
		int length = replayBuffer[bufferPointer];
		bufferPointer -= (length+3)>>1;
		if (bufferPointer <0)
			bufferPointer += REPLAY_LENGTH;
		copyFrom = bufferPointer;
    }
	if ((isUp==0) && (bufferPointer != bufferNext))
	{
		int lengthOfCurr = replayBuffer[bufferPointer];
		bufferPointer += (lengthOfCurr + 5)>>1;
		if (bufferPointer >= REPLAY_LENGTH)
			bufferPointer -= REPLAY_LENGTH;
		if (bufferPointer != bufferNext)
		{
			copyFrom = bufferPointer;
		}
		else
		{
			*len = 0;
			return 1;
		}
	}
	if (copyFrom != -1)
	{
		/* we want to copy */
		int length = replayBuffer[copyFrom];
		copyFrom ++;
		if (copyFrom >= REPLAY_LENGTH)
			 copyFrom -= REPLAY_LENGTH;
		*len = length;
		if ((copyFrom+((length+1)>>1)) > REPLAY_LENGTH)
		{
			/* wrap around => copy in two steps */
			memcpy(strCmd, &replayBuffer[copyFrom], (REPLAY_LENGTH-copyFrom)<<1);
			length -= (REPLAY_LENGTH-copyFrom)<<1;
			strCmd += (REPLAY_LENGTH-copyFrom)<<1;
			copyFrom = 0;
		}
		memcpy(strCmd, &replayBuffer[copyFrom], length);
		return 1;
	}
	return 0;
}


void console_processCmd(char *cmd)
{
	int len = strlen(cmd);
	putsUART("\n\r");
	processCmd(cmd, len);
}

#define CONSOLE_BUF_LEN 256
char consoleBuffer[CONSOLE_BUF_LEN];
volatile int buffWrite = 0;
volatile int buffRead = 0;


void console_process(void)
{
	if (buffWrite != buffRead)
	{
       	char newChar = (char)consoleBuffer[buffRead++]; // Read data from Rx.
		if (buffRead >= CONSOLE_BUF_LEN)
			buffRead = 0;
 		theCmd[cmdLen] = newChar;
		//printf("char=0x%x\n", newChar);
		// we should process here the up and down arrow in order to easily replay previous command
		// we will maintain a stack with the last 10 commands. pushing on arrow
		// will load in the buffer the content of a previous command
		cmdLen++;
		if ((newChar == '\n') || (newChar == '\r'))
		{
			putsUART("\r\n");
			//putsUART("newLineDetected\r\n");
			
			// copy the command in the command buffer
			theCmd[cmdLen-1] = 0; // transform the string in null terminated string 
			storeCmdInReplayBuf(theCmd, cmdLen-1); // do not copy the null string
			processCmd(theCmd, cmdLen);
			cmdLen = 0;
		}
		else if (newChar == 0x1b)
		{
			cmdLen--; // do not keep control char in the buffer
			// read the 2 next char in order to be sure
			// note this could block the program is we do not get the addiotnal chars on the UART
			// we could avoid this by never xaiting more than x msecs.
			while (buffRead == buffWrite)
				;
			char secondChar = (char)consoleBuffer[buffRead++]; // Read data from Rx.
			if (buffRead >= CONSOLE_BUF_LEN)
				buffRead = 0;

			while (buffRead == buffWrite)
				;
			char thirdChar = (char)consoleBuffer[buffRead++]; // Read data from Rx.
			if (buffRead >= CONSOLE_BUF_LEN)
				buffRead = 0;

			if (((thirdChar == 0x41)||(thirdChar == 0x42)) && (secondChar == 0x5b))
			{
				int prevLen = cmdLen;
				if (movePointer(theCmd, &cmdLen, (thirdChar == 0x41)))
				{
					// back space the complete len
					while (prevLen --)
						putcUART(127);
					// display the new cmd
					//printf("recover previous command len=%i\n",cmdLen);
					theCmd[cmdLen] = 0; // be sure the string is null terminated
					int i;
					for (i=0; i< cmdLen; i++)
						putcUART(theCmd[i]);
				}
			} 

		}
		else
		{
			// handle special characters here
			putcUART(newChar);
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
				printf("unknown command %i %s", totalStr,theCmd);
				putsUART("\r\n");
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
static int splitCmd(char *theCmd, char *args[], int maxArg)
{
	int nextArg = 0;
	int idx = 0;

	#define SEARCH_START 0
	#define SEARCH_END 1

	int state = SEARCH_START;
	char nextChar = theCmd[0];
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


static void processCmd(char *theCmd, int cmdLen)
{
	char *args[MAX_ARG];
	if (cmdLen > 1)
	{
		//printf("(%s)\n\r",theCmd);

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
					//printf("res=%i; arg[0]=%s,arg[1]=%s,arg[2]=%s,arg[3]=%s\r\n",
				    //	res,args[0],args[1],args[2],args[3]);
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
	putsUART(">>> ");
}


static int printHelp(char *args[], int argc)
{
	putsUART("this is the main help function \r\n");
	putsUART("commands:\r\n\r\n");
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
			printf("%s", menu[j].theHelp);
			printf("\r\n");
		}
	}
	return 0;
}


	
void __ISR(_UART_VECTOR, ipl3) IntUartConsoleHandler(void)
{
	// Is this an RX interrupt?
	if(mURXGetIntFlag())
	{
		// Clear the RX interrupt Flag
	    mURXClearIntFlag();
		char theChar = ReadUART();
		consoleBuffer[buffWrite++] = theChar;
		if (buffWrite >= CONSOLE_BUF_LEN)
			buffWrite = 0;
	}
}

#ifdef USE_UART1
void _mon_putc(char c)
{
	putcUART(c);
}

#endif

