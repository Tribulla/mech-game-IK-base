#pragma once

#include "gfx/draw_overlay.h"
#include "gfx/edges.h"
#include "gfx/highlight_outline.h"
#include "gfx/utility.h"
#include "sokol_gfx.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct FrameInput
{
	Mat4 view;
	Mat4 viewInv;
	Mat4 proj;
	Mat4 projInv;
	Vec3 cameraPosition;

	Vec3 drawOrigin;

	Vec2 gridWrap;

	float time;

	int debugMode;

	bool disableShadows;

	bool disableAmbientOcclusion;

	bool zUp;
} FrameInput;

typedef struct Sun
{
	Vec3 dirToSun;
	Vec3 color;
	float ambient;
	float strength;
} Sun;

void InitRenderer( const sg_environment* env );

void SetSun( Sun sun );
Sun GetSun( void );

void SetExposure( float ev );
float GetExposure( void );

void SetToneMapSaturation( float saturation );
float GetToneMapSaturation( void );

void SetSkyTurbidity( float t );
float GetSkyTurbidity( void );

void SetIblEnabled( bool enabled );
bool GetIblEnabled( void );

void SetEdgeOverlayParams( const EdgeOverlayParams* settings );
EdgeOverlayParams GetEdgeOverlayParams( void );

void SetHighlightParams( const HighlightParams* params );
HighlightParams GetHighlightParams( void );

typedef struct
{
	int cubeCount;
	int sphereCount;
	int capsuleCount;
	int cubeCountXp;
	int sphereCountXp;
	int capsuleCountXp;
	int geomInstanceCount;
	int geomInstanceCountXp;
	int geomSpanCount;
	int lineCount;
	int pointCount;
	int drawCallCount;
	float frameTimeMs;
} RenderStats;

RenderStats GetRenderStats( void );

typedef struct CameraState
{
	Mat4 view;
	Mat4 viewInv;
	Mat4 proj;
	Mat4 projInv;
	int viewportW;
	int viewportH;
} CameraState;
CameraState GetCameraState( void );

sg_buffer GetCubeProxyVertexBuffer( void );
sg_buffer GetCubeProxyIndexBuffer( void );
int GetCubeProxyIndexCount( void );

void RenderFrame( const sg_swapchain* swapChain, const FrameInput* frame );

void RenderFrameOffscreen( const sg_attachments* attachments, sg_pixel_format outputFormat, int width, int height,
							  const FrameInput* frame );
void ShutdownRenderer( void );

int GetRenderErrorCount( void );

void ResetFrameArena( void );

void AppendCube( Transform transform, Vec3 scale, Vec4 baseColor, float metallic, float roughness, TransparentShadowCast shadowCast );

void AppendSphere( Transform transform, float radius, Vec4 baseColor, float metallic, float roughness,
					  TransparentShadowCast shadowCast );

void AppendCapsule( Transform transform, float halfLength, float radius, Vec4 baseColor, float metallic, float roughness,
					   TransparentShadowCast shadowCast );

#ifdef __cplusplus
} 
#endif
