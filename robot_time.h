#ifndef ROBOT_TIME_H
#define ROBOT_TIME_H

#include "basic_types.h"

/* TIME FUNCTION
   Time is maintained by Timer2. 
   Timer 2 is configured to run at 1.125 MHz 
*/

#define TICKS_PER_SECOND (1125000)
#define TICKS_PER_MSEC (1125)

void time_init(void);
uint32 time_getTime(void);

#endif
