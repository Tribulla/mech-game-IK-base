#pragma once

#include "gfx/projection.h"
#include "gfx/utility.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum TextSpace
{
	TEXT_SPACE_WORLD = 0,
	TEXT_SPACE_SCREEN = 1,
} TextSpace;

typedef struct TextEntry
{
	TextSpace space;
	Vec3 worldPos;
	int screenX;
	int screenY;
	Vec4 color;
	const char* text;
} TextEntry;

void DrawString( Vec3 worldPos, Vec4 color, const char* text );

void DrawScreenString( int x, int y, Vec4 color, const char* text );
void DrawScreenStringFormat( int x, int y, Vec4 color, const char* fmt, ... );

int GetTextCount( void );
const TextEntry* GetTextAt( int i );

void ResetTextArena( void );

static inline bool ResolveTextScreenPos( const TextEntry* e, Mat4 view, Mat4 proj, int viewportW, int viewportH,
										 float* outScreenX, float* outScreenY )
{
	if ( e->space == TEXT_SPACE_SCREEN )
	{
		*outScreenX = (float)e->screenX;
		*outScreenY = (float)e->screenY;
		return true;
	}
	return ProjectWorldToScreen( view, proj, viewportW, viewportH, e->worldPos, outScreenX, outScreenY );
}

#ifdef __cplusplus
} 
#endif
