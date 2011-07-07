#include "movement.h"
#include "robot_time.h"

/* we define sequence of movements;
   A movement is characterized by a possition for each motors and an associated speed. 
   In the future we could replace individual setting with a pod index 
   a "position" and a time to reach this position for this pod.

*/

#define MAX_MOTOR_CONTROLLED 3

typedef struct Amov Amov;
struct Amov
{
	int timeInThisState; /* unit is second. -1 indicate end of the sequence */
	int angleSpeed; /* should be derived from "timeInthisState" but this to be done later on */
    int motorAngles[MAX_MOTOR_CONTROLLED];
};

Amov firstSeq[] = {
	{ 5,   0, { 45,  45,  45}},
	{ 10, 20, {-45, -45, -45}},
	{ 5,   0, {  0,   0,   0}},
	{-1, 0, { 0,  0,  0}},
};

Amov secondSeq[] = {
	{ 2,   0, { 15,  15,  15}},
	{ 2,  0, { 30,  30,  30}},
	{ 2,  0, { 45,  45,  45}},
	{ 2,  0, { 30,  30,  30}},
	{ 2,   0, { 15,  15,  15}},
	{ 2,   0, { 0,  0,  0}},
	{-1, 0, { 0,  0,  0}},
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
};

extern SeqEntry seqEntry2[];

SeqEntry seqEntry[] = {
	{E_MovSeq,    firstSeq },
	{E_MovSeq,    secondSeq },
	{E_lastEntry, NULL },
};

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
};

MoveState moveState = {{{E_SubSeq,seqEntry,0},{0,NULL,0},{0,NULL,0},{0,NULL,0},{0,NULL,0}}, 0, 0};

void process_move(void)
{
	uint32 currentTime = time_getTime();
	int goUp = 0;
	/* note use int16 iso int32 casting to avoid issues with wrap around */
	if (currentTime > moveState.nextExitTime)
	{
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
				int i;
				for (i=0; i<MAX_MOTOR_CONTROLLED; i++)
				{
					motor_setAngle(movp->motorAngles[i],i,movp->angleSpeed);
				}

				/* decide nextTime */
				moveState.nextExitTime = currentTime + movp->timeInThisState*TICKS_PER_SECOND;
				/* jum to the next entry */
				seqState->seqIndex++;
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
	}
}

