// Persistent Win32 / pthreads worker pool and high-resolution timing helpers.
// Workers sleep between frames on a condition variable keyed to a monotonic generation counter
// and pull work items with an atomic index cursor so per-frame dispatch allocates nothing.

#include "gc_thread.h"
#include "gaitcore.h"

#include <stdlib.h>

#if defined( _WIN32 )
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <malloc.h>
#include <windows.h>
#else
#define _POSIX_C_SOURCE 200809L
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#endif

double gc_time_sec( void )
{
#if defined( _WIN32 )
	static LARGE_INTEGER freq = { 0 };
	if ( freq.QuadPart == 0 )
		QueryPerformanceFrequency( &freq );
	LARGE_INTEGER now;
	QueryPerformanceCounter( &now );
	return (double)now.QuadPart / (double)freq.QuadPart;
#else
	struct timespec ts;
	clock_gettime( CLOCK_MONOTONIC, &ts );
	return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
#endif
}

void* gc_alloc_aligned( size_t size, size_t alignment )
{
	if ( alignment < sizeof( void* ) )
		alignment = sizeof( void* );
#if defined( _WIN32 )
	return _aligned_malloc( size, alignment );
#else
	void* p = NULL;
	if ( posix_memalign( &p, alignment, size ) != 0 )
		return NULL;
	return p;
#endif
}

void gc_free_aligned( void* ptr )
{
	if ( !ptr )
		return;
#if defined( _WIN32 )
	_aligned_free( ptr );
#else
	free( ptr );
#endif
}

int gc_hardware_threads( void )
{
#if defined( _WIN32 )
	DWORD n = GetActiveProcessorCount( ALL_PROCESSOR_GROUPS );
	if ( n == 0 )
	{
		SYSTEM_INFO si;
		GetSystemInfo( &si );
		n = si.dwNumberOfProcessors;
	}
	int r = (int)n;
#else
	long n = sysconf( _SC_NPROCESSORS_ONLN );
	int r = ( n > 0 ) ? (int)n : 1;
#endif
	if ( r < 1 ) r = 1;
	if ( r > 64 ) r = 64;
	return r;
}

#if defined( _WIN32 )

struct GcThreadPool {
	int num_threads;
	int num_workers;
	CRITICAL_SECTION mu;
	CONDITION_VARIABLE work_cv;
	CONDITION_VARIABLE done_cv;
	int shutdown;
	uint64_t generation;
	int active_workers;
	GcWorkFn fn;
	void* user;
	int count;
	volatile LONG next_idx;
	HANDLE* workers;
};

static DWORD WINAPI gc_worker_main( LPVOID arg )
{
	GcThreadPool* p = (GcThreadPool*)arg;
	uint64_t my_gen = 0;
	for ( ;; )
	{
		EnterCriticalSection( &p->mu );
		while ( !p->shutdown && p->generation == my_gen )
		{
			SleepConditionVariableCS( &p->work_cv, &p->mu, INFINITE );
		}
		if ( p->shutdown )
		{
			LeaveCriticalSection( &p->mu );
			return 0;
		}
		my_gen = p->generation;
		GcWorkFn fn = p->fn;
		void* user = p->user;
		int count = p->count;
		LeaveCriticalSection( &p->mu );

		for ( ;; )
		{
			int idx = (int)( InterlockedIncrement( &p->next_idx ) - 1 );
			if ( idx >= count )
				break;
			fn( idx, user );
		}

		EnterCriticalSection( &p->mu );
		p->active_workers--;
		if ( p->active_workers == 0 )
		{
			WakeConditionVariable( &p->done_cv );
		}
		LeaveCriticalSection( &p->mu );
	}
}

GcThreadPool* gc_pool_create( int num_threads )
{
	if ( num_threads <= 0 )
		num_threads = gc_hardware_threads();
	if ( num_threads < 1 )
		num_threads = 1;
	if ( num_threads > 64 )
		num_threads = 64;
	GcThreadPool* p = (GcThreadPool*)calloc( 1, sizeof( GcThreadPool ) );
	if ( !p )
		return NULL;
	p->num_threads = num_threads;
	// The calling thread participates in gc_pool_parallel_for, so spawn (num_threads - 1) workers.
	p->num_workers = num_threads - 1;
	InitializeCriticalSection( &p->mu );
	InitializeConditionVariable( &p->work_cv );
	InitializeConditionVariable( &p->done_cv );
	if ( p->num_workers > 0 )
	{
		p->workers = (HANDLE*)calloc( (size_t)p->num_workers, sizeof( HANDLE ) );
		for ( int i = 0; i < p->num_workers; ++i )
		{
			p->workers[i] = CreateThread( NULL, 0, gc_worker_main, p, 0, NULL );
		}
	}
	return p;
}

void gc_pool_destroy( GcThreadPool* p )
{
	if ( !p )
		return;
	if ( p->num_workers > 0 && p->workers )
	{
		EnterCriticalSection( &p->mu );
		p->shutdown = 1;
		p->generation++;
		WakeAllConditionVariable( &p->work_cv );
		LeaveCriticalSection( &p->mu );
		for ( int i = 0; i < p->num_workers; ++i )
		{
			if ( p->workers[i] )
			{
				WaitForSingleObject( p->workers[i], INFINITE );
				CloseHandle( p->workers[i] );
			}
		}
		free( p->workers );
	}
	DeleteCriticalSection( &p->mu );
	free( p );
}

void gc_pool_parallel_for( GcThreadPool* p, int count, GcWorkFn fn, void* user )
{
	if ( !fn || count <= 0 )
		return;
	if ( !p || p->num_workers <= 0 || count <= 1 )
	{
		for ( int i = 0; i < count; ++i )
			fn( i, user );
		return;
	}
	EnterCriticalSection( &p->mu );
	p->fn = fn;
	p->user = user;
	p->count = count;
	p->next_idx = 0;
	p->active_workers = p->num_workers;
	p->generation++;
	WakeAllConditionVariable( &p->work_cv );
	LeaveCriticalSection( &p->mu );

	// Caller works alongside background workers to eliminate main-thread idle stall.
	for ( ;; )
	{
		int idx = (int)( InterlockedIncrement( &p->next_idx ) - 1 );
		if ( idx >= count )
			break;
		fn( idx, user );
	}

	EnterCriticalSection( &p->mu );
	while ( p->active_workers > 0 )
	{
		SleepConditionVariableCS( &p->done_cv, &p->mu, INFINITE );
	}
	LeaveCriticalSection( &p->mu );
}

#else // POSIX pthreads

struct GcThreadPool {
	int num_threads;
	int num_workers;
	pthread_mutex_t mu;
	pthread_cond_t work_cv;
	pthread_cond_t done_cv;
	int shutdown;
	uint64_t generation;
	int active_workers;
	GcWorkFn fn;
	void* user;
	int count;
	int next_idx;
	pthread_t* workers;
};

static void* gc_worker_main( void* arg )
{
	GcThreadPool* p = (GcThreadPool*)arg;
	uint64_t my_gen = 0;
	for ( ;; )
	{
		pthread_mutex_lock( &p->mu );
		while ( !p->shutdown && p->generation == my_gen )
		{
			pthread_cond_wait( &p->work_cv, &p->mu );
		}
		if ( p->shutdown )
		{
			pthread_mutex_unlock( &p->mu );
			return NULL;
		}
		my_gen = p->generation;
		GcWorkFn fn = p->fn;
		void* user = p->user;
		int count = p->count;
		pthread_mutex_unlock( &p->mu );

		for ( ;; )
		{
			int idx = __atomic_fetch_add( &p->next_idx, 1, __ATOMIC_RELAXED );
			if ( idx >= count )
				break;
			fn( idx, user );
		}

		pthread_mutex_lock( &p->mu );
		p->active_workers--;
		if ( p->active_workers == 0 )
		{
			pthread_cond_signal( &p->done_cv );
		}
		pthread_mutex_unlock( &p->mu );
	}
}

GcThreadPool* gc_pool_create( int num_threads )
{
	if ( num_threads <= 0 )
		num_threads = gc_hardware_threads();
	if ( num_threads < 1 )
		num_threads = 1;
	if ( num_threads > 64 )
		num_threads = 64;
	GcThreadPool* p = (GcThreadPool*)calloc( 1, sizeof( GcThreadPool ) );
	if ( !p )
		return NULL;
	p->num_threads = num_threads;
	// The calling thread participates in gc_pool_parallel_for, so spawn (num_threads - 1) workers.
	p->num_workers = num_threads - 1;
	pthread_mutex_init( &p->mu, NULL );
	pthread_cond_init( &p->work_cv, NULL );
	pthread_cond_init( &p->done_cv, NULL );
	if ( p->num_workers > 0 )
	{
		p->workers = (pthread_t*)calloc( (size_t)p->num_workers, sizeof( pthread_t ) );
		for ( int i = 0; i < p->num_workers; ++i )
		{
			pthread_create( &p->workers[i], NULL, gc_worker_main, p );
		}
	}
	return p;
}

void gc_pool_destroy( GcThreadPool* p )
{
	if ( !p )
		return;
	if ( p->num_workers > 0 && p->workers )
	{
		pthread_mutex_lock( &p->mu );
		p->shutdown = 1;
		p->generation++;
		pthread_cond_broadcast( &p->work_cv );
		pthread_mutex_unlock( &p->mu );
		for ( int i = 0; i < p->num_workers; ++i )
		{
			pthread_join( p->workers[i], NULL );
		}
		free( p->workers );
	}
	pthread_cond_destroy( &p->done_cv );
	pthread_cond_destroy( &p->work_cv );
	pthread_mutex_destroy( &p->mu );
	free( p );
}

void gc_pool_parallel_for( GcThreadPool* p, int count, GcWorkFn fn, void* user )
{
	if ( !fn || count <= 0 )
		return;
	if ( !p || p->num_workers <= 0 || count <= 1 )
	{
		for ( int i = 0; i < count; ++i )
			fn( i, user );
		return;
	}
	pthread_mutex_lock( &p->mu );
	p->fn = fn;
	p->user = user;
	p->count = count;
	p->next_idx = 0;
	p->active_workers = p->num_workers;
	p->generation++;
	pthread_cond_broadcast( &p->work_cv );
	pthread_mutex_unlock( &p->mu );

	for ( ;; )
	{
		int idx = __atomic_fetch_add( &p->next_idx, 1, __ATOMIC_RELAXED );
		if ( idx >= count )
			break;
		fn( idx, user );
	}

	pthread_mutex_lock( &p->mu );
	while ( p->active_workers > 0 )
	{
		pthread_cond_wait( &p->done_cv, &p->mu );
	}
	pthread_mutex_unlock( &p->mu );
}

#endif

int gc_pool_thread_count( const GcThreadPool* pool )
{
	return pool ? pool->num_threads : 1;
}
