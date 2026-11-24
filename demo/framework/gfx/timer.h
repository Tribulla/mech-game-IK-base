#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

uint64_t GetTicks( void );

float GetMilliseconds( uint64_t ticks );

void SleepMs( int milliseconds );

void YieldThread( void );

#ifdef __cplusplus
}
#endif
