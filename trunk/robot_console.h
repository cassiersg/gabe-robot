#ifndef ROBOT_CONSOLE_H
#define ROBOT_CONSOLE_H

#include "basic_types.h"

void console_init(int pbClk, int desiredBaudRate);

/* function which verify whether 
   there is something to do for the console
   If a command is detected, it will immediately 
   handle the command
*/
void console_process(void);

#endif
