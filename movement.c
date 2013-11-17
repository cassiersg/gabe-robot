#include "basic_types.h"
#include "movement.h"
#include "robot_time.h"
#include "historical.h"
#include "motor_control.h"
#include "robot_console.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* ~~~~~~~~~~ POD MOVE TYPES DECLARATIONS ~~~~~~~~~~ */

#define DIS_INTER_POINTS 10
#define MAX_NUM_INTER_M (200/DIS_INTER_POINTS+1)

#define TOTAL_DUR_FACTOR 1000

/* we define sequence of movements;
   A movement is characterized by a position for each motors and an associated speed. 
   In the future we could replace individual setting with a pod index 
   a "position" and a time to reach this position for this pod.
*/

typedef struct PodPosition PodPosition;
struct PodPosition
{
	int x;
	int y;
	int z;
};

/* E_null = the pod is not moving */
typedef enum PosEntryType { E_coord_xyz = 0, E_null = 2 } PosEntryType;
typedef enum DisplaceType { D_direct = 0, D_straight = 1} DisplaceType;

typedef struct Amov Amov;
struct Amov
{
	int timeToState; /* relative to the total duration of the move sequence */
    PodPosition podPosition;
	PosEntryType posType;
	DisplaceType displaceType;
};


Amov nullMove[] = {
    { 1, {0, 0, 0}, E_null, D_direct},
    {-1, {0, 0, 0}, E_coord_xyz, D_direct},
};    

Amov firstSeq[]  = {
	{ 1, { 40,  90, 25}, E_coord_xyz, D_straight },
	{ 2, { 40, -10, 25}, E_coord_xyz, D_straight },
	{ 1, {  0,   0,  0}, E_null, D_direct},
	{ 2, {100, 100, 25}, E_coord_xyz, D_straight},
	{-1, {  0,  0,  0}, E_coord_xyz, D_direct },
};

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

SeqEntry nullSeq[] = {
    { E_MovSeq, {nullMove}, 1},
    { E_lastEntry, {NULL}, 0},
};    

SeqEntry seqEntry1[] = {
{E_MovSeq,    {firstSeq},  1 },
{E_lastEntry, {NULL},      0 },
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
	PodPosition currentPos;
	PosEntryType currentE_type;
	Amov interMoves[MAX_NUM_INTER_M];
	int podIdx;
};

MoveState moveStates[4];

typedef enum ProcessMoveState { EndMove, ProcessMove } ProcessMoveState;

Amov *movSeqArray[] = {firstSeq, NULL};
SeqEntry *seqEntryArray[] = {seqEntry1, NULL};

/* ~~~~~~~~~~ GLOBAL MOVE TYPES DECLARATIONS ~~~~~~~~~~ */

#define SIZE_GMOVES_QUEUE 8
#define NB_GMOVES 1

typedef struct AGlobalMove AGlobalMove;
struct AGlobalMove
{
    int duration; /* time in ms */
    SeqEntry *podMoves[4];
};

AGlobalMove firstGMove[] = {
    { 4000, {seqEntry1, nullSeq, nullSeq, nullSeq}},
    {-1, {NULL, NULL, NULL, NULL}},
};

typedef struct GMovesQueue GMovesQueue;
struct GMovesQueue
{
    int readIdx;
    int writeIdx;
    int subMoveIdx;
    AGlobalMove *moves[SIZE_GMOVES_QUEUE];
};    

GMovesQueue gMovesQueue = {0, 0, 0};

AGlobalMove *gMoves[NB_GMOVES] = {firstGMove };

static int c_addGMove(char *args[], int argc);
static int c_showGMove(char *args[], int argc);
static int c_showStacks(char *args[], int argc);

static void stack_init(MoveState *stack, SeqEntry *initSeq, int totalDuration, int podIdx);
static void stacks_init(SeqEntry *initSeq[4], int totalDuration);
static ProcessMoveState processPod_move(MoveState *stack);
static ProcessMoveState processPod_moves(void);
static void stackState(MoveState *stack);
static void interMovState(int level, MoveState *stack);

static int putGMove(AGlobalMove *move);
static void process_GMove(void);
static void show_gMovesQueue(void);

/* ~~~~~~~~~~ CONSOLE APPLICATION ~~~~~~~~~~ */

CmdEntry movementConsoleMenu[] = {
    { "addgmove", c_addGMove,   1, "add a global move in the queue"},
    { "showgmove", c_showGMove, 1, "show the gMoves queue"},
    { "stackstate", c_showStacks, 1, "stackState"},
    { NULL, NULL, 0, NULL},
};

static int c_addGMove(char *args[], int argc)
{
    AGlobalMove *gMove = gMoves[0];
    if (argc > 1)
    {
        int idx = atoi(args[1]);
        if (idx < NB_GMOVES)
        {
            gMove = gMoves[idx];
        }
        else
        {
            return 1;
        }    
    }        
    putGMove(gMove);
    return 0;
}    

static int c_showGMove(char *args[], int argc)
{
    show_gMovesQueue();
    return 0;
}    

static int c_showStacks(char *args[], int argc)
{
    int i;
    printf("StackState:\r\n");
    for (i=0; i<4; i++)
    {
        printf("--- pod %i ---\r\n", i);
        stackState(&moveStates[i]);
    }
    return SUCCESS;
}

/* ~~~~~~~~~~ INIT ~~~~~~~~~~ */
        
void move_init(void)
{
    /* stacks */
    SeqEntry *seq[] = {nullSeq, nullSeq, nullSeq, nullSeq};
    stacks_init(seq, 1);
    /* console */
    console_addCommandsList(movementConsoleMenu);
	/* initialization of the relative times */
	int i, duration;
	Amov *moveSeq = movSeqArray[0];
	for (i=0; movSeqArray[i]!=NULL; i++)
	{
		moveSeq = movSeqArray[i];
		duration = 0;
		while (moveSeq->timeToState>=0)
		{
			duration += moveSeq->timeToState;
			moveSeq++;
		}
		moveSeq = movSeqArray[i];
		while (moveSeq->timeToState>=0)
		{
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


/* ~~~~~~~~~~ POD MOVE PROCESSING ~~~~~~~~~~ */

static void stack_init(MoveState *stack, SeqEntry *initSeq, int totalDuration, int podIdx)
{
	stack->seqStack[0].seqType = E_SubSeq;
	stack->seqStack[0].seq.seq =  initSeq;
	stack->seqStack[0].seqIndex = 0;
	stack->seqStack[0].duration = totalDuration;
	stack->currentStackLevel = 0;
	stack->nextExitTime = 0;
	stack->currentE_type = E_null;
	stack->podIdx = podIdx;
	//stackState();
}

static void stacks_init(SeqEntry *initSeq[4], int totalDuration)
{
    int podIdx;
    for (podIdx=0; podIdx<4; podIdx++)
    {
        stack_init(&moveStates[podIdx], initSeq[podIdx], totalDuration, podIdx);
    }
}        

static ProcessMoveState processPod_move(MoveState *stack)
{
	uint32 currentTime = time_getTime();
	int goUp = 0;
	/* note use int16 iso int32 casting to avoid issues with wrap around */
	if (currentTime > stack->nextExitTime)
	{
		//stackState();
		SeqState *seqState = &stack->seqStack[stack->currentStackLevel];

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
				if (stack->currentStackLevel < (MAX_RECURSION-1))
				{
					stack->currentStackLevel++;
					
					SeqState *newSeqState = &stack->seqStack[stack->currentStackLevel];
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
			if (movp->timeToState < 0)
			{
				// end of sequence
				goUp = 1;
			}
			else
			{
				/* command motors */
				if (movp->posType == E_null)
				{
					/* decide nextTime */
					int timeToState = (seqState->duration)*(movp->timeToState)/TOTAL_DUR_FACTOR;
					 stack->nextExitTime = currentTime + timeToState * TICKS_PER_MSEC;
                    /* jump to the next entry */
                    seqState->seqIndex++;
				}
				else if (movp->posType == E_coord_xyz)
				{
					PodPosition *coord = &movp->podPosition;
					// if too many recursion level, do as D_direct
					if (movp->displaceType == D_straight && stack->currentE_type ==E_coord_xyz && stack->currentStackLevel < (MAX_RECURSION-1))
					{
						int num_points = R_ABS((coord->x - stack->currentPos.x) / DIS_INTER_POINTS) + 1;
						int i;
						int timeFactor = TOTAL_DUR_FACTOR / num_points;
						int x= (coord->x - stack->currentPos.x)/num_points;
						int y= (coord->y - stack->currentPos.y)/num_points;
						int z= (coord->z - stack->currentPos.z)/num_points;
						Amov *interPointPtr;
						for (i=0; i<num_points; i++)
						{
							interPointPtr = &stack->interMoves[i];
							interPointPtr->timeToState = timeFactor;
							interPointPtr->podPosition.x = stack->currentPos.x + x*(i+1);
							interPointPtr->podPosition.y = stack->currentPos.y + y*(i+1);
							interPointPtr->podPosition.z = stack->currentPos.z + z*(i+1);
							interPointPtr->posType = E_coord_xyz;
							interPointPtr->displaceType = D_direct;
						}
						stack->interMoves[num_points].timeToState = -1;

						stack->currentStackLevel++;
						SeqState *newSubSeqState = &stack->seqStack[stack->currentStackLevel];
						newSubSeqState->seqIndex = 0;
						newSubSeqState->seqType = E_MovSeq;
						newSubSeqState->seq.moveSeq = stack->interMoves;
						newSubSeqState->duration = seqState->duration * movp->timeToState / TOTAL_DUR_FACTOR;
					}
					else
					{	
						int timeToState = (seqState->duration)*(movp->timeToState)/TOTAL_DUR_FACTOR;
						pod_setPosition(coord->x, coord->y, coord->z, timeToState, stack->podIdx);
						event_save("movement.c, setPosition: x:%i, y:%i, z:%i", coord->x, coord->y, coord->z);
						stack->currentPos.x = coord->x;
						stack->currentPos.y = coord->y;
						stack->currentPos.z = coord->z;
						stack->currentE_type = E_coord_xyz;
						/* decide nextTime */
						stack->nextExitTime = currentTime + timeToState * TICKS_PER_MSEC;
                        /* jump to the next entry */
                        seqState->seqIndex++;
					}
				}
			}
		}

		if (goUp)
		{
			// go one level up - next index. if not possible go back to first entry
			if (stack->currentStackLevel == 0)
			{
				//seqState->seqIndex = 0;
				return EndMove;
			}
			else
			{
				stack->currentStackLevel--;
				seqState = &stack->seqStack[stack->currentStackLevel];
				seqState->seqIndex++;
			}
		}
		//stackState();
	}
	return ProcessMove;
}

static ProcessMoveState processPod_moves(void)
{
    int i;
    ProcessMoveState state = EndMove;
    for (i=0; i<4; i++)
    {
        if (processPod_move(&moveStates[i]) == ProcessMove)
        {
            state = ProcessMove;
        }    
    }
    return state;
}        



static void stackState(MoveState *stack)
{
	int i;
	int interPlvl = -1;
	printf("STACK STATE:\r\n");
	printf("   current move type: %i\r\n", stack->currentE_type);
	int nbVar = 4;
	char *names[]    = {"seqEntry1", "firstSeq"};
	void *adress[] = {&seqEntry1, &firstSeq};
	for (i=0; i<nbVar; i++)
	{
		printf("  &%s: %p\r\n", names[i], adress[i]);
	}
	for (i=0; i<= stack->currentStackLevel; i++)
	{
		SeqState *curSeq = &stack->seqStack[i];
		printf("   STACK LEVEL %i\r\n", i);
		printf("      sequence:%p\r\n", curSeq->seq.seq);
		printf("      cur index:%i\r\n", curSeq->seqIndex);
		printf("      duration:%i\r\n", curSeq->duration);
		if (curSeq->seq.moveSeq == stack->interMoves)
		{
			interPlvl = curSeq->seqIndex ;
		}
	}
	if (interPlvl >=0)
	{
		interMovState(interPlvl, stack);
	}
}

static void interMovState(int level, MoveState *stack)
{
	printf("INTER MOVE STATE:\r\n");
	int i;
	int end = FALSE;
	PodPosition *position;
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
		position = &stack->interMoves[i].podPosition; /*&interPoints[i].podPosition.position;*/
		printf("T to s.:%i  x:%i  y:%i  z:%i\r\n", stack->interMoves[i].timeToState, position->x, position->y, position->z);
        if (stack->interMoves[i].timeToState<0)
        {
            end=TRUE;
        }
	}
}

void process_move(void)
{
    process_GMove();
}    
  
/* ~~~~~~~~~~ GLOBAL MOVE PROCESSING ~~~~~~~~~~ */

static int putGMove(AGlobalMove *move)
{
    // queue is full
    if ((gMovesQueue.writeIdx + 1) % SIZE_GMOVES_QUEUE == gMovesQueue.readIdx)
    {
        return FAILURE;
    }    
    else
    {
        gMovesQueue.moves[gMovesQueue.writeIdx] = move;
        gMovesQueue.writeIdx++;
        if (gMovesQueue.writeIdx >= SIZE_GMOVES_QUEUE)
        {
            gMovesQueue.writeIdx -= SIZE_GMOVES_QUEUE;
        }
        return SUCCESS;
    }
}            

static void process_GMove(void)
{
    // end of move processing
    if (processPod_moves() == EndMove)
    {
        if (gMovesQueue.readIdx != gMovesQueue.writeIdx)
        {
            // end of array
            if (gMovesQueue.moves[gMovesQueue.readIdx][gMovesQueue.subMoveIdx].duration == -1)
            {
                gMovesQueue.readIdx++;
                if (gMovesQueue.readIdx >= SIZE_GMOVES_QUEUE)
                {
                    gMovesQueue.readIdx -= SIZE_GMOVES_QUEUE;
                }
                gMovesQueue.subMoveIdx = 0;
            }
            else
            {
                AGlobalMove *gMove = &gMovesQueue.moves[gMovesQueue.readIdx][gMovesQueue.subMoveIdx];
                stacks_init(gMove->podMoves, gMove->duration);
                gMovesQueue.subMoveIdx++;
            }    
        }
    }
}

static void show_gMovesQueue(void)
{
    printf("gMovesQueue state:\r\n  readIdx: %i\r\n  writeIdx: %i\r\n  subMoveIdx: %i\r\n", gMovesQueue.readIdx, gMovesQueue.writeIdx, gMovesQueue.subMoveIdx);
    int i;
    printf("   firstGMove : %p\r\n", firstGMove);
    for (i=0; i<SIZE_GMOVES_QUEUE; i++)
    {
        printf("  moves[%i] : %p\r\n", i, gMovesQueue.moves[i]);
    }
}        