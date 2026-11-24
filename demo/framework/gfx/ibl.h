#pragma once

#include "gfx/utility.h"
#include "sokol_gfx.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define GFX_IBL_CUBE_SIZE 256
#define GFX_IBL_CUBE_MIPS 7
#define GFX_IBL_LUT_SIZE 256

void InitImageBasedLighting( const sg_environment* env );
void ShutdownImageBasedLighting( void );

void MarkIblDirty( void );

void RebuildImageBasedLightingIfDirty( Vec3 dirToSun, float turbidity, bool zUp );

sg_view GetIblSpecularView( void );
sg_sampler GetIblSpecularSampler( void );
float GetIblSpecularMaxLod( void );

sg_view GetIblBrdfLutView( void );
sg_sampler GetIblBrdfLutSampler( void );

void GetIblSphericalHarmonicCoefficients( Vec3 out[9] );

#ifdef __cplusplus
} 
#endif
