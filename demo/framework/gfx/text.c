#include "gfx/text.h"

#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define TEXT_CAPACITY 1024
#define TEXT_BUFFER_SIZE 64

typedef struct TextSlot
{
	TextSpace space;
	Vec3 worldPos;
	int screenX;
	int screenY;
	Vec4 color;
	char text[TEXT_BUFFER_SIZE];
} TextSlot;

static TextSlot s_slots[TEXT_CAPACITY];
static TextEntry s_entries[TEXT_CAPACITY];
static int s_count;

void ResetTextArena( void )
{
	s_count = 0;
}

static TextSlot* AppendSlot( Vec4 color, const char* text )
{
	if ( text == NULL )
		return NULL;
	if ( s_count >= TEXT_CAPACITY )
		return NULL;

	TextSlot* slot = &s_slots[s_count];
	slot->color = color;
	strncpy( slot->text, text, TEXT_BUFFER_SIZE - 1 );
	slot->text[TEXT_BUFFER_SIZE - 1] = '\0';
	return slot;
}

static void PublishSlot( const TextSlot* slot )
{
	s_entries[s_count].space = slot->space;
	s_entries[s_count].worldPos = slot->worldPos;
	s_entries[s_count].screenX = slot->screenX;
	s_entries[s_count].screenY = slot->screenY;
	s_entries[s_count].color = slot->color;
	s_entries[s_count].text = slot->text;
	s_count += 1;
}

void DrawString( Vec3 worldPos, Vec4 color, const char* text )
{
	TextSlot* slot = AppendSlot( color, text );
	if ( slot == NULL )
		return;
	slot->space = TEXT_SPACE_WORLD;
	slot->worldPos = worldPos;
	slot->screenX = 0;
	slot->screenY = 0;
	PublishSlot( slot );
}

void DrawScreenString( int x, int y, Vec4 color, const char* text )
{
	TextSlot* slot = AppendSlot( color, text );
	if ( slot == NULL )
		return;
	slot->space = TEXT_SPACE_SCREEN;
	slot->worldPos = Vec3_zero;
	slot->screenX = x;
	slot->screenY = y;
	PublishSlot( slot );
}

void DrawScreenStringFormat( int x, int y, Vec4 color, const char* fmt, ... )
{
	if ( fmt == NULL )
		return;
	char buffer[TEXT_BUFFER_SIZE];
	va_list args;
	va_start( args, fmt );
	vsnprintf( buffer, sizeof( buffer ), fmt, args );
	va_end( args );
	DrawScreenString( x, y, color, buffer );
}

int GetTextCount( void )
{
	return s_count;
}

const TextEntry* GetTextAt( int i )
{
	assert( i >= 0 && i < s_count );
	return &s_entries[i];
}
