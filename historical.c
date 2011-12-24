#include "historical.h"
#include "basic_types.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define N_MREFRESH_SAVED 16
typedef struct e_Mrefresh e_Mrefresh;
struct e_Mrefresh
{
	uint32 time;
	uint8 motorIndex;
	int impLen;
	int perToFinal;
};
e_Mrefresh m_refresh[N_MREFRESH_SAVED];
int mRefIdx;


#define N_EVENT_SAVED 10
#define E_STRLEN 50
#define PAR_NUM 4
typedef struct event event;
struct event
{
	uint32 time;
	uint8 event[E_STRLEN];
	int parameters[PAR_NUM];
};
event eventTable[N_EVENT_SAVED];
int eventT_idx;

int eventSave = FALSE;

static int showEvent(int evIdx);
static int showMrefresh(int mIdx);

void resetHistory(void)
{
	memset(m_refresh,0,N_MREFRESH_SAVED*sizeof(e_Mrefresh));
	memset(eventTable,0,N_EVENT_SAVED*sizeof(event));
	mRefIdx=0;
	eventT_idx=0;
}

void mrefresh_save(uint8 motorIndex, int impLen, int perToFinal)
{
	if (eventSave==TRUE)
	{
		m_refresh[mRefIdx].time       = time_getTime();
		m_refresh[mRefIdx].motorIndex = motorIndex; 
		m_refresh[mRefIdx].impLen     = impLen;
		m_refresh[mRefIdx].perToFinal = perToFinal;
		mRefIdx++;
		if (mRefIdx==N_MREFRESH_SAVED)
			mRefIdx=0;
	}
}

void event_save(uint8 *event, ...)
{
	if (eventSave==TRUE)
	{
		va_list parameters;
		va_start(parameters, event);
		int i;
		eventTable[eventT_idx].time = time_getTime();
 	    memcpy(eventTable[eventT_idx].event,event,E_STRLEN);
		for (i=0; i<PAR_NUM; i++)
			eventTable[eventT_idx].parameters[i]=va_arg(parameters, int);
		eventT_idx++;
		if (eventT_idx==N_EVENT_SAVED)
			eventT_idx=0;
	}
}

void setState_eSave(int state)
{
	eventSave=state;
}

static int showEvent(int evIdx)
{
	event *obj = NULL;
	obj= &eventTable[evIdx];
	printf("t:%11i ", obj->time);
	printf(obj->event, obj->parameters[0], obj->parameters[1], obj->parameters[2], obj->parameters[3]);
	printf("\r\n");
	evIdx++;
	if (evIdx>=N_EVENT_SAVED)
		evIdx=0;
	return evIdx;
}

static int showMrefresh(int mIdx)
{
	e_Mrefresh *obj = NULL;
	obj = &m_refresh[mIdx];
	printf("t:%11i Mrefrsh mIdx:%i impLen:%4i perToFinal:%i\r\n", obj->time, obj->motorIndex, obj->impLen, obj->perToFinal);
	mIdx++;
	if (mIdx>=N_MREFRESH_SAVED)
		mIdx=0;
	return mIdx;
}


void showHistorical(void)
{
	setState_eSave(FALSE);
	int evIndex  = 0;
	int mIndex   = 0;
	int m_state  = TRUE;
	int ev_state = TRUE;

	if (eventTable[N_EVENT_SAVED-1].time!=0)
		evIndex=eventT_idx;
	else
	{
		// table unused
		if (eventT_idx==0 && eventTable[0].time==0)
			ev_state = FALSE;
	}
	if (m_refresh[N_MREFRESH_SAVED-1].time!=0)
		mIndex=mRefIdx;
	else
	{
		// table unused
		if (mRefIdx==0 && m_refresh[0].time==0)
			m_state = FALSE;
	}

	while (m_state==TRUE && ev_state==TRUE)
	{
		if (m_refresh[mIndex].time<eventTable[evIndex].time)
		{
			mIndex = showMrefresh(mIndex);
			if (mIndex==mRefIdx)
				m_state=FALSE;
		}
		else
		{
			evIndex=showEvent(evIndex);
			if (evIndex==eventT_idx)
				ev_state=FALSE;
		}
	}
	while (m_state==TRUE)
	{
		mIndex = showMrefresh(mIndex);
		if (mIndex==mRefIdx)
			m_state=FALSE;
	}
	while (ev_state==TRUE)
	{
		evIndex=showEvent(evIndex);
		if (evIndex==eventT_idx)
			ev_state=FALSE;
	}
}
