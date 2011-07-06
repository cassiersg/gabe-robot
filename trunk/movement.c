#include "movement.h"
#include "robot_time.h"

typedef enum CurrentState { E_LowPosition = 0, E_HighPosition = 1} CurrentState;

typedef struct MoveState MoveState;
struct MoveState
{
	CurrentState currState;
	uint32 nextExitTime;
};

MoveState moveState = {E_LowPosition, 0};

void process_move(void)
{
	uint32 currentTime = time_getTime();

	/* note use int16 iso int32 casting to avoid issues with wrap around */
	if (currentTime > moveState.nextExitTime)
	{
		switch (moveState.currState)
		{
			case E_LowPosition:
				motor_setAngle(45,0,0);
				motor_setAngle(45,1,0);
				moveState.nextExitTime = currentTime + 5*TICKS_PER_SECOND;
				moveState.currState = E_HighPosition;
				break;
			case E_HighPosition:
				motor_setAngle(-45,0,20);
				motor_setAngle(-45,1,20);
				moveState.nextExitTime = currentTime + 10*TICKS_PER_SECOND;
				moveState.currState = E_LowPosition;
				break;
			default:
				/* we should print a warning here --- we could use the led to signal warnings */
				break;
		}
	}
}

