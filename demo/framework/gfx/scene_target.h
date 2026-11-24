#pragma once

#include "sokol_gfx.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define SCENE_SAMPLE_COUNT 4

void InitSceneTarget( const sg_environment* env );
void ShutdownSceneTarget( void );

void ResizeSceneTarget( int width, int height );

sg_attachments GetSceneTargetAttachments( void );

sg_view GetSceneTargetResolveSampleView( void );

sg_view GetSceneTargetResolveColorAttachView( void );

sg_pixel_format GetSceneTargetDepthFormat( void );

#ifdef __cplusplus
} 
#endif
