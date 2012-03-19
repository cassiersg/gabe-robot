#include "basic_types.h"
#include "movement.h"
#include "robot_time.h"
#include "historical.h"
#include "motor_control.h"
#include <stdio.h>
#include <math.h>

/* we define sequence of movements;
   A movement is characterized by a position for each motors and an associated speed. 
   In the future we could replace individual setting with a pod index 
   a "position" and a time to reach this position for this pod.
*/

#define MAX_MOTOR_CONTROLLED 3
#define DIS_INTER_POINTS 10
#define MAX_NUM_INTER_M (200/DIS_INTER_POINTS+1)

#define TOTAL_DUR_FACTOR 1000

typedef struct Position Position;
struct Position
{
	int x;
	int y;
	int z;
};

typedef union PodPosition PodPosition;
union PodPosition
{
	Position position;
	int motorAngles[MAX_MOTOR_CONTROLLED];
};

/* E_null = the pod is not moving */
typedef enum PosEntryType { E_coord_xyz = 0 , E_angles = 1, E_null = 2 } PosEntryType;
typedef enum DisplaceType { D_direct = 0, D_straight = 1} DisplaceType;

typedef struct Amov Amov;
struct Amov
{
	int timeInThisState; /* relative to the total duration of the move sequence */
	int timeToState; /* relative to the total duration of the move sequence */
    PodPosition podPosition;
	PosEntryType posType;
	DisplaceType displaceType;
};

Amov firstSeq[] = {
	{ 1,  0, { 45,  45,  45}, E_angles, D_direct },
	{ 2,  0, {-45, -45, -45}, E_angles, D_direct },
	{ 1,  0, {  0,   0,   0}, E_angles, D_direct },
	{-1,  0, { 0,  0,  0}, E_angles, D_direct },
};

Amov secondSeq[]  = {
	{ 0, 1, {-50, 80, 35}, E_coord_xyz, D_straight },
	{ 0, 1, { 50, 80, 35}, E_coord_xyz, D_straight },
	{ 0, 2, {  0,  0,  0}, E_null, D_direct},
	{-1, 0, {  0,  0,  0}, E_angles, D_direct },
};


/* define sequences of sequences
   A sequence entry is a link to sequence of movement. In the future we 
   could extend this with the possibility to point to a nested sequence.

   The move state would contain a stack and there would be a maximum nesting level.
*/
typedef enum SeqEntryType {E_MovSeq = 0, E_SubSeq = 1, E_lastEntry = 0xFF } SeqEntryType;
typedef struct SeqEntry SeqEntry;

typedef union SubSeq SubSeq;
union SubSeq {
	Amov     *moveSeq;
	SeqEntry *seq;
};

struct SeqEntry
{
	SeqEntryType entryType;
	SubSeq       subSeq;
	int          durFactor;
};


SeqEntry seqEntry1[] = {
{E_MovSeq,    firstSeq,  1 },
{E_lastEntry, NULL,      0 },
};

SeqEntry seqEntry2[] = {
{E_MovSeq,    secondSeq, 1 },
{E_lastEntry, NULL,     0 },
};


#define MAX_RECURSION 5

typedef struct SeqState SeqState;
struct SeqState
{
	SeqEntryType seqType;
	SubSeq  seq;
	int seqIndex;
	int duration;
};

typedef struct MoveState MoveState;
struct MoveState
{

	SeqState seqStack[MAX_RECURSION];
	int currentStackLevel;
	uint32 nextExitTime;
	Position currentPos;
	PosEntryType currentE_type;
	Amov interMoves[MAX_NUM_INTER_M];
};

static void stackState(void);
static void interMovState(int level);
static void stack_init(MoveState *stack, SeqEntry *initSeq, int totalDuration);

MoveState moveState; /*= {{{E_SubSeq,(void *)seqEntry2,0,4000},{0,NULL,0,0},{0,NULL,0,0},{0,NULL,0,0},{0,NULL,0,0}}, 
						0, 0, {0, 0, 0}, E_angles };*/

Amov *movSeqArray[] = {firstSeq, secondSeq, NULL};
SeqEntry *seqEntryArray[] = {seqEntry1, seqEntry2, NULL};

void move_init(void)
{
	/*stack_init*/
	stack_init(&moveState, seqEntry2, 4000);
	/* initialization of the relative times */
	int i, duration;
	Amov *moveSeq = movSeqArray[0];
	for (i=0; movSeqArray[i]!=NULL; i++)
	{
		moveSeq = movSeqArray[i];
		duration = 0;
		while (moveSeq->timeInThisState>=0)
		{
			duration += moveSeq->timeInThisState + moveSeq->timeToState;
			moveSeq++;
		}
		moveSeq = movSeqArray[i];
		while (moveSeq->timeInThisState>=0)
		{
			moveSeq->timeInThisState = TOTAL_DUR_FACTOR * moveSeq->timeInThisState / duration;
			moveSeq->timeToState = TOTAL_DUR_FACTOR * moveSeq->timeToState / duration;
			moveSeq++;
		}
	}
	SeqEntry *entrySeq = seqEntryArray[0];
	for (i=0; seqEntryArray[i]!=NULL; i++)
	{
		entrySeq = seqEntryArray[i];
		duration = 0;
		while (entrySeq->entryType != E_lastEntry)
		{
			duration += entrySeq->durFactor;
			entrySeq++;
		}
		entrySeq = seqEntryArray[i];
		while (entrySeq->entryType != E_lastEntry)
		{
			entrySeq->durFactor = TOTAL_DUR_FACTOR * entrySeq->durFactor / duration;
			entrySeq++;
		}
	}
}

static void stack_init(MoveState *stack, SeqEntry *initSeq, int totalDuration)
{
	stack->seqStack[0].seqType = E_SubSeq;
	stack->seqStack[0].seq.seq =  initSeq;
	stack->seqStack[0].seqIndex = 0;
	stack->seqStack[0].duration = totalDuration;
	stack->currentStackLevel = 0;
	stack->nextExitTime = 0;
	stack->currentE_type = E_angles;
	//stackState();
}

void process_move(void)
{
	uint32 currentTime = time_getTime();
	int goUp = 0;
	/* note use int16 iso int32 casting to avoid issues with wrap around */
	if (currentTime > moveState.nextExitTime)
	{
		//stackState();
		SeqState *seqState = &moveState.seqStack[moveState.currentStackLevel];

		/* execute the current entry */
		/* check if entry is a subseq. if this is the case, jump into it at first pos */
		if (seqState->seqType == E_SubSeq)
		{
			SeqEntry *seqp = &seqState->seq.seq[seqState->seqIndex];
			/* move to the new level */
			if (seqp->entryType == E_lastEntry)
			{
				goUp = 1;
			}
			else
			{
				// enter sub sequence
				if (moveState.currentStackLevel < (MAX_RECURSION-1))
				{
					moveState.currentStackLevel++;
					
					SeqState *newSeqState = &moveState.seqStack[moveState.currentStackLevel];
					newSeqState->seqIndex = 0;
					newSeqState->seqType = seqp->entryType;
					newSeqState->seq =  seqp->subSeq;
					newSeqState->duration = (seqState->duration)*(seqp->durFactor)/TOTAL_DUR_FACTOR;
				}
				else
				{
					// too many recursion level - so skip subsequence
					printf("movement stack : too many recursion level\r\n");
					seqState->seqIndex++;
				}
			}
		}
		else 
		{
			Amov *movp = &seqState->seq.moveSeq[seqState->seqIndex];
			/* check if end of sequence */
			if (movp->timeInThisState < 0)
			{
				// end of sequence
				goUp = 1;
			}
			else
			{
				/* command motors */
				if (movp->posType == E_angles)
				{
					int i;
					int timeToState = (seqState->duration)*(movp->timeToState)/TOTAL_DUR_FACTOR;
					int timeInThisState = (seqState->duration)*(movp->timeInThisState)/TOTAL_DUR_FACTOR;
					for (i=0; i<MAX_MOTOR_CONTROLLED; i++)
					{
						motor_setAngle((movp->podPosition.motorAngles)[i],i,timeToState);
					}
					moveState.currentE_type= E_angles;
					/* decide nextTime */
					 moveState.nextExitTime = currentTime + (timeInThisState + timeToState) * TICKS_PER_MSEC;
                    /* jump to the next entry */
                    seqState->seqIndex++;
				}
				else if (movp->posType == 2)
				{
					/* decide nextTime */
					int timeToState = (seqState->duration)*(movp->timeToState)/TOTAL_DUR_FACTOR;
					int timeInThisState = (seqState->duration)*(movp->timeInThisState)/TOTAL_DUR_FACTOR;
					 moveState.nextExitTime = currentTime + (timeInThisState + timeToState) * TICKS_PER_MSEC;
                    /* jump to the next entry */
                    seqState->seqIndex++;
				}
				else if (movp->posType == E_coord_xyz)
				{
					Position *coord = &movp->podPosition.position;
					// if previous movement was E_angles or too many recursion level, do as D_direct
					if (movp->displaceType == D_straight && moveState.currentE_type ==E_coord_xyz && moveState.currentStackLevel < (MAX_RECURSION-1))
					{
						int num_points = abs((coord->x-moveState.currentPos.x)/DIS_INTER_POINTS)+1;
						int i;
						int timeFactor = TOTAL_DUR_FACTOR*(movp->timeToState)/((movp->timeToState + movp->timeInThisState)*num_points);
						int x= (coord->x - moveState.currentPos.x)/num_points;
						int y= (coord->y - moveState.currentPos.y)/num_points;
						int z= (coord->z - moveState.currentPos.z)/num_points;
						Amov *interPointPtr;
						for (i=0; i<num_points; i++)
						{
							interPointPtr = &moveState.interMoves[i];
							interPointPtr->timeInThisState = 0;
							interPointPtr->timeToState = timeFactor;
							interPointPtr->podPosition.position.x = moveState.currentPos.x + x*(i+1);
							interPointPtr->podPosition.position.y = moveState.currentPos.y + y*(i+1);
							interPointPtr->podPosition.position.z = moveState.currentPos.z + z*(i+1);
							interPointPtr->posType = E_coord_xyz;
							interPointPtr->displaceType = D_direct;
						}
						moveState.interMoves[num_points-1].timeInThisState = TOTAL_DUR_FACTOR*(movp->timeInThisState)/(movp->timeToState + movp->timeInThisState);
						moveState.interMoves[num_points].timeInThisState = -1;

						moveState.currentStackLevel++;
						SeqState *newSubSeqState = &moveState.seqStack[moveState.currentStackLevel];
						newSubSeqState->seqIndex = 0;
						newSubSeqState->seqType = E_MovSeq;
						newSubSeqState->seq.moveSeq = moveState.interMoves;
						newSubSeqState->duration = (seqState->duration)*(movp->timeInThisState + movp->timeToState)/TOTAL_DUR_FACTOR;
					}
					else
					{	
						int timeToState = (seqState->duration)*(movp->timeToState)/TOTAL_DUR_FACTOR;
						int timeInThisState = (seqState->duration)*(movp->timeInThisState)/TOTAL_DUR_FACTOR;
						pod_setPosition(coord->x, coord->y, coord->z, timeToState, 0, 1, 2);
						event_save("movement.c, setPosition: x:%i, y:%i, z:%i", coord->x, coord->y, coord->z);
						moveState.currentPos.x = coord->x;
						moveState.currentPos.y = coord->y;
						moveState.currentPos.z = coord->z;
						moveState.currentE_type = E_coord_xyz;
						/* decide nextTime */
						moveState.nextExitTime = currentTime + (timeInThisState + timeToState) * TICKS_PER_MSEC;
                        /* jump to the next entry */
                        seqState->seqIndex++;
					}
				}
			}
		}

		if (goUp)
		{
			// go one level up - next index. if not possible go back to first entry
			if (moveState.currentStackLevel == 0)
			{
				seqState->seqIndex = 0;
			}
			else
			{
				moveState.currentStackLevel--;
				seqState = &moveState.seqStack[moveState.currentStackLevel];
				seqState->seqIndex++;
			}
		}
		//stackState();
	}
}


static void stackState(void)
{
	int i;
	int interPlvl = -1;
	printf("STACK STATE:\r\n");
	printf("   current move type: %i\r\n", moveState.currentE_type);
	int nbVar = 4;
	char *names[]    = {"seqEntry1", "firstSeq", "seqEntry2", "secondSeq"};
	void *adress[] = {&seqEntry1, &firstSeq, &seqEntry2, &secondSeq};
	for (i=0; i<nbVar; i++)
	{
		printf("  &%s: %p\r\n", names[i], adress[i]);
	}
	for (i=0; i<=moveState.currentStackLevel; i++)
	{
		SeqState *curSeq = &moveState.seqStack[i];
		printf("   STACK LEVEL %i\r\n", i);
		printf("      sequence:%p\r\n", curSeq->seq);
		printf("      cur index:%i\r\n", curSeq->seqIndex);
		printf("      duration:%i\r\n", curSeq->duration);
		if (curSeq->seq.moveSeq == moveState.interMoves)
		{
			interPlvl = curSeq->seqIndex ;
		}
	}
	if (interPlvl >=0)
	{
		interMovState(interPlvl);
	}
}

static void interMovState(int level)
{
	printf("INTER MOVE STATE:\r\n");
	int i;
	int end = FALSE;
	Position *position;
	for (i=0; i<MAX_NUM_INTER_M && end==FALSE; i++)
	{
		if (i == level)
		{
			printf(">");
		}
		else
		{
			printf(" ");
		}
		position = &moveState.interMoves[i].podPosition.position; /*&interPoints[i].podPosition.position;*/
		printf("T in this s.: %i  T to s.:%i  x:%i  y:%i  z:%i\r\n", moveState.interMoves[i].timeInThisState, /*interPoints[i].timeInThisState,*/
					moveState.interMoves[i].timeToState, position->x, position->y, position->z);
        if (moveState.interMoves[i].timeInThisState<0)
        {
            end=TRUE;
        }
	}
}
