#ifndef BASIC_TYPES_H
#define BASIC_TYPES_H


#include <stddef.h>  /* needed to get //#define NULL 0 */

#define SUCCESS  0
#define FAILURE -1
#define FALSE    0
#define TRUE     1

typedef short int int16;
typedef unsigned short int uint16;
typedef unsigned int uint32;
typedef int int32;
typedef unsigned char uint8;
/*
typedef union PtrInt PtrInt;
union PtrInt {
	void * ptr;
	int integer;
}
*/

#endif
