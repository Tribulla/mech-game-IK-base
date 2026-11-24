#pragma once

#include "gfx/utility.h"
#include "sokol_gfx.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define SHADOW_CASCADE_COUNT 3
#define SHADOW_RESOLUTION 4096

#define SHADOW_SPLIT_FAR 50.0f

void InitShadows( void );
void ShutdownShadows( void );

void SetShadowSunDir( Vec3 dirToSun );

void SetShadowSplits( float nearViewZ, float farViewZ );

void SetShadowSplitFar( float farViewZ );

void FitShadows( const Mat4* viewInv, const Mat4* proj );

void BeginShadowPass( int cascade );
void EndShadowPass( void );

Mat4 GetCascadeMatrix( int cascade );

float GetCascadeFarViewZ( int cascade );

float GetCascadePcfScale( int cascade );

sg_view GetShadowSampleView( void );
sg_sampler GetShadowSampler( void );

float GetShadowTexelSize( void );

#ifdef __cplusplus
} 
#endif
