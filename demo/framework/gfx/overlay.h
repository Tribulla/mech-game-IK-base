#pragma once

#include "gfx/draw_overlay.h"
#include "gfx/utility.h"
#include "sokol_gfx.h"

#ifdef __cplusplus
extern "C"
{
#endif

void OverlayInit( void );
void OverlayShutdown( void );

void OverlayResetFrame( void );

void OverlayAppendLine( Vec3 a, Vec3 b, Vec4 linearColor, float thickness, OverlayThicknessUnit thicknessUnit,
						OverlayOcclusionMode occlusionMode );

void OverlayAppendPoint( Vec3 p, Vec4 linearColor, float size, OverlayThicknessUnit sizeUnit, OverlayOcclusionMode occlusionMode );

void OverlaySubmit( int width, int height, const Mat4* view, const Mat4* viewInv, const Mat4* proj, const Mat4* projInv,
					Vec3 cameraPos, float time, sg_view linearDepthView, sg_sampler linearDepthSampler, sg_pixel_format colorFormat,
					sg_pixel_format depthFormat );

int OverlayLineCount( void );
int OverlayPointCount( void );

#ifdef __cplusplus
} 
#endif
