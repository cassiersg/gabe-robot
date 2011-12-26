#ifndef ROBOT_CONSOLE_H
#define ROBOT_CONSOLE_H

#include "basic_types.h"

#define MAX_COMMAND_LIST 10

typedef struct CmdEntry CmdEntry;
struct CmdEntry
{
	uint8 *theCmd;
	int (*processingFunction)(uint8 *args[], int argc);
	uint8 minArgs;
	uint8 *theHelp;
};


void console_init(int pbClk, int desiredBaudRate);

// add a list of command to process. Each list is terminated by an entry where theCmd is NULL
void console_addCommandsList(CmdEntry *cmdList);

/* function which verify whether 
   there is something to do for the console
   If a command is detected, it will immediately 
   handle the command.
   Each application can add its own list of command. 
   The maximum number of application is determined by constant MAX_COMMAND_LIST
 
*/
void console_process(void);

#endif
