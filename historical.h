#ifndef HISTORICAL_H
#define HISTORICAL_H

#include "basic_types.h"

void historical_init(void);
void mrefresh_save(uint8 motorIndex, int impLen, int perToFinal);
void event_save(uint8 *event, ...);

void resetHistory(void);
void setState_eSave(int state);
void showHistorical(void);

#endif
