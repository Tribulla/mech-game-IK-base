#pragma once

#include "gfx/utility.h"
#include "sokol_gfx.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct EdgeOverlayParams
{
	bool enabled;
	bool showHulls;
	bool showMeshes;
	bool showHeightfields;
	bool showEdgeConvexity;
	float thicknessPx;
	float zBias;
	Vec4 convexColor;
	Vec4 concaveColor;
	Vec4 flatColor;
} EdgeOverlayParams;

EdgeOverlayParams GetDefaultEdgeParams( void );

void InitEdges( void );
void ShutdownEdges( void );

void RenderEdgesInMSAA( int width, int height, const Mat4* view, const Mat4* proj, sg_view opaqueInstanceView,
						sg_view transparentInstanceView, const EdgeOverlayParams* params );

void RenderEdgesPostTransparent( int width, int height, const Mat4* view, const Mat4* proj, sg_view colorAttachView,
								 sg_view depthAttachView, sg_view opaqueInstanceView, sg_view transparentInstanceView,
								 const EdgeOverlayParams* params );

#ifdef __cplusplus
} 
#endif
