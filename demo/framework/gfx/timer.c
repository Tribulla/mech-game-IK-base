#include "gfx/timer.h"

#if defined( _WIN32 )

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

uint64_t GetTicks( void )
{
	LARGE_INTEGER counter;
	QueryPerformanceCounter( &counter );
	return (uint64_t)counter.QuadPart;
}

float GetMilliseconds( uint64_t ticks )
{
	static double s_invFrequency = 0.0;
	if ( s_invFrequency == 0.0 )
	{
		LARGE_INTEGER frequency;
		QueryPerformanceFrequency( &frequency );
		s_invFrequency = 1000.0 / (double)frequency.QuadPart;
	}
	return (float)( (double)( GetTicks() - ticks ) * s_invFrequency );
}

void SleepMs( int milliseconds )
{
	Sleep( (DWORD)milliseconds );
}

void YieldThread( void )
{
	SwitchToThread();
}

#else

#if defined( __linux__ ) && !defined( _POSIX_C_SOURCE )
#define _POSIX_C_SOURCE 200809L
#endif

#include <sched.h>
#include <time.h>

uint64_t GetTicks( void )
{
	struct timespec ts;
	clock_gettime( CLOCK_MONOTONIC, &ts );
	return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

float GetMilliseconds( uint64_t ticks )
{
	return (float)( (double)( GetTicks() - ticks ) * 1e-6 );
}

void SleepMs( int milliseconds )
{
	struct timespec ts;
	ts.tv_sec = milliseconds / 1000;
	ts.tv_nsec = (long)( milliseconds % 1000 ) * 1000000L;
	nanosleep( &ts, NULL );
}

void YieldThread( void )
{
	sched_yield();
}

#endif
