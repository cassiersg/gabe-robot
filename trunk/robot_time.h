#ifndef ROBOT_TIME_H
#define ROBOT_TIME_H

#include "basic_types.h"

/* TIME FUNCTION
   Time is maintained by Timer2. 
   Timer 2 is configured to run at 2.25 MHz
*/

#define TICKS_PER_SECOND (2250000)
#define TICKS_PER_MSEC (2250)

void time_init(void);
uint32 time_getTime(void);

#endif
