#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct Arena
{
	uint8_t* base;
	size_t capacity;
	size_t used;
} Arena;

int ArenaInit( Arena* a, size_t capacity );

void ArenaShutdown( Arena* a );

void* ArenaAlloc( Arena* a, size_t size, size_t align );

void ArenaReset( Arena* a );

#ifdef __cplusplus
} 
#endif
