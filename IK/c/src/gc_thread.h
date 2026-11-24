// Internal timing, aligned allocation, and persistent work-stealing thread pool.

#ifndef GAITCORE_THREAD_H
#define GAITCORE_THREAD_H

#include <stddef.h>
#include <stdint.h>

typedef struct GcThreadPool GcThreadPool;
typedef void ( *GcWorkFn )( int idx, void* user );

double gc_time_sec( void );
void* gc_alloc_aligned( size_t size, size_t alignment );
void gc_free_aligned( void* ptr );

// Passing num_threads <= 0 queries hardware concurrency (clamped 1..64).
GcThreadPool* gc_pool_create( int num_threads );
void gc_pool_destroy( GcThreadPool* pool );
int gc_pool_thread_count( const GcThreadPool* pool );

// Splits 0..count-1 across the worker pool and blocks until all items finish.
void gc_pool_parallel_for( GcThreadPool* pool, int count, GcWorkFn fn, void* user );

#endif // GAITCORE_THREAD_H
