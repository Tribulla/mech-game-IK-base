#pragma once

#include "gfx/geometry_registry.h"
#include "gfx/utility.h"
#include "sokol_gfx.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum HighlightKind
{
	HIGHLIGHT_KIND_NONE = 0,
	HIGHLIGHT_KIND_HOVER = 1,
	HIGHLIGHT_KIND_SELECT = 2,
} HighlightKind;

void HighlightMaskInit( void );
void HighlightMaskShutdown( void );

void HighlightMaskBeginFrame( void );

void AppendHighlightSphere( Transform transform, float radius, HighlightKind kind );

void AppendHighlightCapsule( Transform transform, float halfLength, float radius, HighlightKind kind );

void AppendHighlightGeometry( MeshHandle handle, Transform transform, Vec3 scale, HighlightKind kind );

bool HasHighlightMaskContent( void );

void SubmitHighlightMask( int width, int height, const Mat4* viewProj, Vec3 cameraPosWorld, sg_view maskAttachView,
						  sg_view depthAttachView );

#ifdef __cplusplus
} 
#endif
