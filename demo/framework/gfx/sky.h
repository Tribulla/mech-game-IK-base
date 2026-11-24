#pragma once

#include "gfx/utility.h"
#include "sokol_gfx.h"

#ifdef __cplusplus
extern "C"
{
#endif

void InitSky( const sg_environment* env );
void ShutdownSky( void );
float SkySunFadeWeight( Vec3 dirToSun, bool zUp );

void DrawSky( Vec3 dirToSun, float turbidity, Vec3 cameraPos, Mat4 invViewProj, bool zUp );

#ifdef __cplusplus
} 
#endif
