#include "basic_types.h"
#include "movement.h"
#include "robot_time.h"
#include "historical.h"
#include <stdio.h>

/* we define sequence of movements;
   A movement is characterized by a position for each motors and an associated speed. 
   In the future we could replace individual setting with a pod index 
   a "position" and a time to reach this position for this pod.

*/

static void stackState(void);
static void interMovState(int level);

#define MAX_MOTOR_CONTROLLED 3
#define DIS_INTER_POINTS 10
#define MAX_NUM_INTER_P (200/DIS_INTER_POINTS)

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

typedef enum PosEntryType { E_coord_xyz = 0 , E_angles = 1 } PosEntryType;
typedef enum DisplaceType { D_direct = 0, D_straight = 1} DisplaceType;

typedef struct Amov Amov;
struct Amov
{
	int timeInThisState; /* unit is msec. -1 indicate end of the sequence */
	int timeToState; /* unit is msec. */
    PodPosition podPosition;
	PosEntryType posType;
	DisplaceType displaceType;
};

Amov firstSeq[] = {
	{ 5000,   0, { 45,  45,  45}, E_angles, D_direct },
	{ 10000, 20, {-45, -45, -45}, E_angles, D_direct },
	{ 5000,   0, {  0,   0,   0}, E_angles, D_direct },
	{-1, 0, { 0,  0,  0}, E_angles, D_direct },
};

Amov secondSeq[] = {
	{ 2000, 0, { 15,  15,  15}, E_angles, D_direct },
	{ 2000, 0, { 30,  30,  30}, E_angles, D_direct },
	{ 2000, 0, { 45,  45,  45}, E_angles, D_direct },
	{ 2000, 0, { 30,  30,  30}, E_angles, D_direct },
	{ 2000, 0, { 15,  15,  15}, E_angles, D_direct },
	{ 2000, 0, {  0,   0,   0}, E_angles, D_direct },
	{-1, 0, { 0,  0,  0}, E_angles, D_direct },
};

Amov thirdSeq[] = {
	{ 0, 500, { -50, 80, 35}, E_coord_xyz, D_straight },
	{ 0, 500, {  50, 80, 35}, E_coord_xyz, D_straight },
	{-1, 0, { 0,  0,  0}, E_angles, D_direct },
};


Amov fourthSeq[] = {
	{ 2000, 400, { -50, 80, 55}, E_coord_xyz, D_straight },
	{    0, 400, { -50, 80, 35}, E_coord_xyz, D_straight },
	{    0, 400, {   0, 80, 35}, E_coord_xyz, D_straight },
	{    0, 400, {  50, 80, 35}, E_coord_xyz, D_straight },
	{    0, 400, {  50, 80, 55}, E_coord_xyz, D_straight },
	{    0, 400, {  25, 80, 55}, E_coord_xyz, D_straight },
	{    0, 400, {   0, 80, 55}, E_coord_xyz, D_straight },
	{    0, 400, { -25, 80, 55}, E_coord_xyz, D_straight },
	{    0, 400, { -50, 80, 55}, E_coord_xyz, D_straight },
	{ 2000, 400, {   0,  0,  0}, E_angles,    D_direct },
	{-1, 0, { 0,  0,  0}, E_angles, D_direct },
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

Amov interPoints[MAX_NUM_INTER_P];
SubSeq interMove;

struct SeqEntry

{
	SeqEntryType entryType;
	SubSeq       subSeq;  	
};

//extern SeqEntry seqEntry2[];

SeqEntry seqEntry[] = {
{E_MovSeq,    firstSeq },
{E_MovSeq,    secondSeq },
{E_lastEntry, NULL },
};

SeqEntry seqEntry3[] = {
{E_MovSeq,    thirdSeq },
{E_lastEntry, NULL },
};

/*
SeqEntry seqEntry4[] = {
{E_MovSeq,    fourthSeq },
{E_lastEntry, NULL },
};
*/

#define MAX_RECURSION 5

typedef struct SeqState SeqState;
struct SeqState
{
	SeqEntryType seqType;
	SubSeq  seq;
	int seqIndex;

};

typedef struct MoveState MoveState;
struct MoveState
{

	SeqState seqStack[MAX_RECURSION];
	int currentStackLevel;
	uint32 nextExitTime;
	Position currentPos;
	PosEntryType currentE_type;
};

MoveState moveState = {{{E_SubSeq,(void *)seqEntry3,0},{0,NULL,0},{0,NULL,0},{0,NULL,0},{0,NULL,0}}, 
						0, 0, {0, 0, 0}, E_angles };


void move_init(void)
{
	interMove.moveSeq = interPoints;
}

void process_move(void)
{
	uint32 currentTime = time_getTime();
	int goUp = 0;
	/* note use int16 iso int32 casting to avoid issues with wrap around */
	if (currentTime > moveState.nextExitTime)
	{
		stackState();
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
				}
				else
				{
					// too many recursion level - so skip subsequence
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
					for (i=0; i<MAX_MOTOR_CONTROLLED; i++)
					{
						motor_setAngle((movp->podPosition.motorAngles)[i],i,movp->timeToState);
					}
					moveState.currentE_type= E_angles;
					/* decide nextTime */
					 moveState.nextExitTime = currentTime + ( movp->timeInThisState + movp->timeToState) *TICKS_PER_MSEC;
                    /* jump to the next entry */
                    seqState->seqIndex++;
				}
				else
				{
					Position *coord = &movp->podPosition.position;
					// if previous movement was E_angles or too many recursion level, do as D_direct
					if (movp->displaceType == D_straight && moveState.currentE_type ==E_coord_xyz && moveState.currentStackLevel < (MAX_RECURSION-1))
					{
						int num_points = abs((coord->x-moveState.currentPos.x)/DIS_INTER_POINTS);
						int i;
						int timeToState = (movp->timeToState)/num_points;
						int x= (coord->x - moveState.currentPos.x)/num_points;
						int y= (coord->y - moveState.currentPos.y)/num_points;
						int z= (coord->z - moveState.currentPos.z)/num_points;
						Amov *interPointPtr;
						for (i=0; i<num_points; i++)
						{
							interPointPtr = &interPoints[i];
							interPointPtr->timeInThisState = 0;
							interPointPtr->timeToState = timeToState;
							interPointPtr->podPosition.position.x = moveState.currentPos.x + x*(i+1);
							interPointPtr->podPosition.position.y = moveState.currentPos.y + y*(i+1);
							interPointPtr->podPosition.position.z = moveState.currentPos.z + z*(i+1);
							interPointPtr->posType = E_coord_xyz;
							interPointPtr->displaceType = D_direct;
						}
						interPoints[num_points-1].timeInThisState = movp->timeInThisState;
						interPoints[num_points].timeInThisState = -1;

						moveState.currentStackLevel++;
						SeqState *newSubSeqState = &moveState.seqStack[moveState.currentStackLevel];
						newSubSeqState->seqIndex = 0;
						newSubSeqState->seqType = E_MovSeq;
						newSubSeqState->seq = interMove;
					}
					else
					{	
						pod_setPosition(coord->x, coord->y, coord->z, movp->timeToState, 0, 1, 2);
						event_save("movement.c, setPosition: x:%i, y:%i, z:%i", coord->x, coord->y, coord->z);
						moveState.currentPos.x = coord->x;
						moveState.currentPos.y = coord->y;
						moveState.currentPos.z = coord->z;
						moveState.currentE_type = E_coord_xyz;
						/* decide nextTime */
						moveState.nextExitTime = currentTime + ( movp->timeInThisState + movp->timeToState) *TICKS_PER_MSEC;
                        /* jump to the next entry */
                        seqState->seqIndex++;
					}
				}
			}
		}

		if (goUp)
		{
			printf("goUp\r\n");
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
	printf("   &thirdSeq:%p\r\n", thirdSeq);
	printf("   &seqEntry3:%p\r\n", seqEntry3);
	printf("   &interMove subSeq:%p\r\n", &interMove);
	printf("   &interPoints :%p\r\n", interPoints);
	for (i=0; i<=moveState.currentStackLevel; i++)
	{
		SeqState *curSeq = &moveState.seqStack[i];
		printf("   STACK LEVEL %i\r\n", i);
		printf("      sequence:%p\r\n", curSeq->seq);
		printf("      cur index:%i\r\n", curSeq->seqIndex);
		if ((int)curSeq->seq.moveSeq == (int)interMove.moveSeq)
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
	for (i=0; i<MAX_NUM_INTER_P && end==FALSE; i++)
	{
		if (i == level)
		{
			printf(">");
		}
		else
		{
			printf(" ");
		}
		position = &interPoints[i].podPosition.position;
		printf("T in this s.: %i  T to s.:%i  x:%i  y:%i  z:%i\r\n", interPoints[i].timeInThisState,
					interPoints[i].timeToState, position->x, position->y, position->z);
        if (interPoints[i].timeInThisState<0)
        {
            end=TRUE;
        }
	}
}
