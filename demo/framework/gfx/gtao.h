#pragma once

#include "gfx/utility.h"
#include "sokol_gfx.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

void InitAmbientOcclusion( void );
void ShutdownAmbientOcclusion( void );

typedef enum AmbientOcclusionQuality
{
	AO_QUALITY_MEDIUM = 0,

	AO_QUALITY_HIGH = 1,

	AO_QUALITY_ULTRA = 2,
} AmbientOcclusionQuality;

#define GTAO_MAX_DENOISE_PASSES 5

typedef struct GtaoTraceParams
{
	float radius;
	float falloff;
	float radiusMul;
	float finalValuePow;
	float denoiseBlurBeta;
	float sampleDistPow;
	float thinOcclComp;
	float depthMipSampOffset;
	AmbientOcclusionQuality quality;
	int denoisePassCount;
} GtaoTraceParams;

GtaoTraceParams GetGtaoTraceParams( void );
void SetGtaoTraceParams( GtaoTraceParams p );
GtaoTraceParams GetDefaultGtaoTraceParams( void );

AmbientOcclusionQuality GetAOQualityForPixelScale( float dpiScale );

void ResizeGTAO( int width, int height );

sg_view GetLinearDepthAttachmentView( void );
sg_view GetPrepassDepthAttachmentView( void );

sg_view GetLinearDepthSampleView( void );

void PrefilterDepth( const Mat4* proj, int viewportWidth, int viewportHeight );

sg_view GetPrefilterDepthSampleView( void );

void ComputeNoisyResult( const Mat4* proj, int viewportWidth, int viewportHeight );

void Denoise( const Mat4* proj, int viewportWidth, int viewportHeight );

void ClearAmbientOcclusion( void );

sg_view GetAOResultSampleView( void );

sg_sampler GetAOLinearSampler( void );

#ifdef __cplusplus
} 
#endif
