#pragma once

#include "gfx/draw_overlay.h"
#include "gfx/utility.h"

#define DEFAULT_METALLIC 0.0f
#define DEFAULT_ROUGHNESS 0.5f

#define DEFAULT_LINE_THICKNESS_PX 1.5f
#define DEFAULT_POINT_SIZE_PX 4.0f

#define DEFAULT_ARROW_HEAD_FRAC 0.15f

#ifdef __cplusplus
extern "C"
{
#endif

void SetDrawOrigin( Vec3 origin );

Vec3 GetDrawOrigin( void );

void DrawCube( Transform transform, Vec3 scale, Vec4 baseColor );
void DrawCubeEx( Transform transform, Vec3 scale, Vec4 baseColor, float metallic, float roughness,
				 TransparentShadowCast shadowCast );

void DrawSphere( Transform transform, float radius, Vec4 baseColor );
void DrawSphereEx( Transform transform, float radius, Vec4 baseColor, float metallic, float roughness,
				   TransparentShadowCast shadowCast );

void DrawCapsule( Transform transform, float halfLength, float radius, Vec4 baseColor );
void DrawCapsuleEx( Transform transform, float halfLength, float radius, Vec4 baseColor, float metallic, float roughness,
					TransparentShadowCast shadowCast );

void DrawPlane( Vec3 normal, Vec3 point, Vec4 color );

void DrawLine( Vec3 a, Vec3 b, Vec4 color );
void DrawLineEx( Vec3 a, Vec3 b, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				 OverlayOcclusionMode occlusionMode );

void DrawPoint( Vec3 p, float size, Vec4 color );
void DrawPointEx( Vec3 p, Vec4 color, float size, OverlayThicknessUnit sizeUnit, OverlayOcclusionMode occlusionMode );

void DrawArrow( Vec3 a, Vec3 b, Vec4 color );
void DrawArrowEx( Vec3 a, Vec3 b, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				  OverlayOcclusionMode occlusionMode, float headLengthFrac );

void DrawCross( Vec3 center, float size, Vec4 color );
void DrawCrossEx( Vec3 center, float size, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				  OverlayOcclusionMode occlusionMode );

void DrawAabb( Vec3 min, Vec3 max, Vec4 color );
void DrawAabbEx( Vec3 min, Vec3 max, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				 OverlayOcclusionMode occlusionMode );

void DrawBounds( AABB bounds, float extension, Vec4 color );

void DrawAxes( Transform transform, float size );
void DrawAxesEx( Transform transform, float size, float thickness, OverlayThicknessUnit thicknessUnit,
				 OverlayOcclusionMode occlusionMode );

void DrawGrid( Vec3 center, Vec3 normal, float halfExtent, int divisions, Vec4 color );

void DrawGroundGrid( int size );

void DrawTriangle( Transform transform, Vec3 a, Vec3 b, Vec3 c, Vec4 color );

void DrawWireSphere( Transform transform, float radius, int segments, Vec4 color );
void DrawWireCapsule( Vec3 a, Vec3 b, float radius, int segments, Vec4 color );

void DrawString3D( Vec3 point, Vec4 color, const char* format, ... );

#ifdef __cplusplus
} 
#endif
