#pragma once

#include "sokol_gfx.h"

#ifdef __cplusplus
extern "C"
{
#endif

void InitHighlightTarget( void );
void ShutdownHighlightTarget( void );

void ResizeHighlightTarget( int width, int height );

sg_view GetHighlightTargetMaskAttachView( void );

sg_view GetHighlightTargetMaskSampleView( void );

#ifdef __cplusplus
} 
#endif
