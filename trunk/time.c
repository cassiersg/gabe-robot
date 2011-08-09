#include <plib.h>
#include "robot_time.h"

/* TIME FUNCTION
   Time is maintained by Timer2. 
   Timer 2 is configured to run at 2,25 MHz : 72MHz/32 (we are on Periph clock) /32 (divider) 
   The time is composed as followed : 14 LSB are coming from the timer the upper bits are maintained in SW
   To maintain the upper bits, each time we getn an interrupt (wrap of the 14 bits HW counter; 
   we increase the SW counter by 1.
   We keep have a total of (32+14) bits. Note we could easily keep more time.

   Note we get an interrupt every 29 msecs.
   At 2,25 MHz, this allows us to survive more than a year (32 bits give us about 1 hour)
   
   One difficulty in the system is that we do not want to return bad time if we get the 
   interrupt during the processing.
   We will read twice the PB. If the second reading is smaller than the first one, 
   it means we just had a wrap so we will have to reread the SW accumulator

*/

uint32 currentSoftTime = 0;

#define TIME_BITS_IN_HW 15
#define TIME_MAX_HW_VAL (1<<TIME_BITS_IN_HW)


void time_init(void)
{

    OpenTimer2(T2_ON | T2_PS_1_32, TIME_MAX_HW_VAL);
			//Timer2 On,  No synchronization, 1:32 Prescaler, Period

    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_3);

}


uint32 time_getTime(void)
{
    uint32 firstTimerVal = 1; // force at least one read of the counter
	uint32 softTime;
    uint32 secondTimerVal = 0;

    // if the second timer val is lower than the first, 
    // it means we just had a wrap between the two reading
    while (secondTimerVal < firstTimerVal)
    {
        firstTimerVal = ReadTimer2();
        softTime = currentSoftTime;

	    secondTimerVal = ReadTimer2();
	}

	return (currentSoftTime<<(TIME_BITS_IN_HW-1))|secondTimerVal;
}

void __ISR(_TIMER_2_VECTOR, ipl3) Timer2Handler(void)
{
    // just increase the upper bits
    currentSoftTime ++;

    // Clear the interrupt flag
	mT2ClearIntFlag();
}
