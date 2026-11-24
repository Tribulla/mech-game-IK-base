#include "gfx/shadow.h"

#include <float.h>
#include <math.h>
#include <stdio.h>

#define SHADOW_SPLIT_NEAR 0.1f
#define PSSM_LAMBDA 0.5f
#define PSSM_LAMBDA_MAX 0.9f
#define CASTER_MARGIN_M 50.0f

static struct
{
	Vec3 sunDir;
	float splitNear;
	float splitFar;
	float splitLambda;
	sg_image depthArray;
	sg_view cascadeAttachmentViews[SHADOW_CASCADE_COUNT];
	sg_view sampleView;
	sg_sampler sampler;
	Mat4 cascadeMatrices[SHADOW_CASCADE_COUNT];
	float cascadeFarViewZ[SHADOW_CASCADE_COUNT];
	float cascadeRadius[SHADOW_CASCADE_COUNT];
	bool inPass;
} s_shadow;

static inline float ComputeSplitDistance( int i, int cascadeCount, float nearZ, float farZ )
{
	const float fi = (float)i / (float)cascadeCount;
	const float u = nearZ + ( farZ - nearZ ) * fi;
	const float l = nearZ * powf( farZ / nearZ, fi );
	return s_shadow.splitLambda * l + ( 1.0f - s_shadow.splitLambda ) * u;
}

static inline float SplitLambdaForRange( float nearZ, float farZ )
{
	float decades = log10f( ( farZ / nearZ ) / ( SHADOW_SPLIT_FAR / SHADOW_SPLIT_NEAR ) );
	return ClampFloat( PSSM_LAMBDA + 0.4f * decades, PSSM_LAMBDA, PSSM_LAMBDA_MAX );
}

static void FrustumSliceCornersWorld( const Mat4* viewInv, const Mat4* proj, float nearZ, float farZ, Vec3 cornersOut[8] )
{
	const Vec3 eye = { viewInv->cw.x, viewInv->cw.y, viewInv->cw.z };
	const Vec3 right = { viewInv->cx.x, viewInv->cx.y, viewInv->cx.z };
	const Vec3 up = { viewInv->cy.x, viewInv->cy.y, viewInv->cy.z };
	const Vec3 forward = { -viewInv->cz.x, -viewInv->cz.y, -viewInv->cz.z };

	const float tanHalfFovY = ( proj->cy.y != 0.0f ) ? ( 1.0f / proj->cy.y ) : 1.0f;
	const float aspect = ( proj->cx.x != 0.0f ) ? ( proj->cy.y / proj->cx.x ) : 1.0f;

	const float distances[2] = { nearZ, farZ };
	for ( int s = 0; s < 2; ++s )
	{
		const float d = distances[s];
		const float h = d * tanHalfFovY;
		const float w = h * aspect;
		const Vec3 center = MulAdd( eye, d, forward );
		for ( int j = 0; j < 4; ++j )
		{
			const float sx = ( j & 1 ) ? +w : -w;
			const float sy = ( j & 2 ) ? +h : -h;
			Vec3 c = MulAdd( center, sx, right );
			c = MulAdd( c, sy, up );
			cornersOut[s * 4 + j] = c;
		}
	}
}

static Mat4 FitCascade( const Mat4* viewInv, const Mat4* proj, Vec3 dirToSun, float nearZ, float farZ, float* radiusOut )
{
	Vec3 corners[8];
	FrustumSliceCornersWorld( viewInv, proj, nearZ, farZ, corners );

	Vec3 center = Vec3_zero;
	for ( int i = 0; i < 8; ++i )
	{
		center = Add( center, corners[i] );
	}
	center = MulSV( 1.0f / 8.0f, center );

	float radius = 0.0f;
	for ( int i = 0; i < 8; ++i )
	{
		const Vec3 d = Sub( corners[i], center );
		const float r = sqrtf( d.x * d.x + d.y * d.y + d.z * d.z );
		if ( r > radius )
		{
			radius = r;
		}
	}
	if ( radius < 0.01f )
	{
		radius = 0.01f;
	}
	*radiusOut = radius;

	Vec3 up = Vec3_axisY;
	if ( fabsf( dirToSun.y ) > 0.999f )
	{
		up = Vec3_axisZ;
	}

	const Vec3 eyeWorld = MulAdd( center, radius + CASTER_MARGIN_M, dirToSun );
	const Mat4 lightView = MakeLookAt( eyeWorld, center, up );
	const Mat4 lightProj = MakeOrthographic( -radius, radius, -radius, radius, 0.0f, 2.0f * radius + CASTER_MARGIN_M );
	Mat4 lightVP = MulMM4( lightProj, lightView );

	const Vec4 centerClip = MulMV4( lightVP, MakeVec4( center.x, center.y, center.z, 1.0f ) );
	const float halfRes = (float)SHADOW_RESOLUTION * 0.5f;
	const float texX = centerClip.x * halfRes;
	const float texY = centerClip.y * halfRes;
	const float dx = ( roundf( texX ) - texX ) / halfRes;
	const float dy = ( roundf( texY ) - texY ) / halfRes;

	Mat4 snapMat = MakeIdentity();
	snapMat.cw.x = dx;
	snapMat.cw.y = dy;
	return MulMM4( snapMat, lightVP );
}

void InitShadows( void )
{
	s_shadow.sunDir = Normalize( (Vec3){ 0.5f, 0.8f, 0.4f } );
	s_shadow.splitNear = SHADOW_SPLIT_NEAR;
	s_shadow.splitFar = SHADOW_SPLIT_FAR;
	s_shadow.splitLambda = SplitLambdaForRange( s_shadow.splitNear, s_shadow.splitFar );

	for ( int i = 0; i < SHADOW_CASCADE_COUNT; ++i )
	{
		s_shadow.cascadeRadius[i] = 1.0f;
	}

	sg_image_desc desc = { 0 };
	desc.type = SG_IMAGETYPE_ARRAY;
	desc.usage.depth_stencil_attachment = true;
	desc.width = SHADOW_RESOLUTION;
	desc.height = SHADOW_RESOLUTION;
	desc.num_slices = SHADOW_CASCADE_COUNT;
	desc.pixel_format = SG_PIXELFORMAT_DEPTH;
	desc.sample_count = 1;
	desc.label = "shadow_depth_array";
	s_shadow.depthArray = sg_make_image( &desc );

	for ( int i = 0; i < SHADOW_CASCADE_COUNT; ++i )
	{
		sg_view_desc vdesc = { 0 };
		vdesc.depth_stencil_attachment.image = s_shadow.depthArray;
		vdesc.depth_stencil_attachment.slice = i;
		vdesc.label = "shadow_cascade_attachment";
		s_shadow.cascadeAttachmentViews[i] = sg_make_view( &vdesc );
	}

	sg_view_desc svdesc = { 0 };
	svdesc.texture.image = s_shadow.depthArray;
	svdesc.texture.slices.base = 0;
	svdesc.texture.slices.count = SHADOW_CASCADE_COUNT;
	svdesc.label = "shadow_sample_view";
	s_shadow.sampleView = sg_make_view( &svdesc );

	sg_sampler_desc sdesc = { 0 };
	sdesc.min_filter = SG_FILTER_LINEAR;
	sdesc.mag_filter = SG_FILTER_LINEAR;
	sdesc.mipmap_filter = SG_FILTER_NEAREST;
	sdesc.wrap_u = SG_WRAP_CLAMP_TO_EDGE;
	sdesc.wrap_v = SG_WRAP_CLAMP_TO_EDGE;
	sdesc.wrap_w = SG_WRAP_CLAMP_TO_EDGE;
	sdesc.compare = SG_COMPAREFUNC_LESS_EQUAL;
	sdesc.label = "shadow_compare_sampler";
	s_shadow.sampler = sg_make_sampler( &sdesc );

	s_shadow.inPass = false;
}

void ShutdownShadows( void )
{
	sg_destroy_sampler( s_shadow.sampler );
	sg_destroy_view( s_shadow.sampleView );
	for ( int i = 0; i < SHADOW_CASCADE_COUNT; ++i )
	{
		sg_destroy_view( s_shadow.cascadeAttachmentViews[i] );
	}
	sg_destroy_image( s_shadow.depthArray );
}

void SetShadowSplits( float nearViewZ, float farViewZ )
{
	if ( nearViewZ <= 0.0f || farViewZ <= nearViewZ )
	{
		nearViewZ = SHADOW_SPLIT_NEAR;
		farViewZ = SHADOW_SPLIT_FAR;
	}

	s_shadow.splitNear = nearViewZ;
	s_shadow.splitFar = farViewZ;
	s_shadow.splitLambda = SplitLambdaForRange( nearViewZ, farViewZ );
}

void SetShadowSplitFar( float farViewZ )
{
	SetShadowSplits( s_shadow.splitNear, farViewZ );
}

void SetShadowSunDir( Vec3 dirToSun )
{
	const float lenSq = dirToSun.x * dirToSun.x + dirToSun.y * dirToSun.y + dirToSun.z * dirToSun.z;
	if ( lenSq > 0.0f )
	{
		s_shadow.sunDir = Normalize( dirToSun );
	}
}

void FitShadows( const Mat4* viewInv, const Mat4* proj )
{
	float splits[SHADOW_CASCADE_COUNT + 1];
	for ( int i = 0; i <= SHADOW_CASCADE_COUNT; ++i )
	{
		splits[i] = ComputeSplitDistance( i, SHADOW_CASCADE_COUNT, s_shadow.splitNear, s_shadow.splitFar );
	}

	for ( int i = 0; i < SHADOW_CASCADE_COUNT; ++i )
	{
		s_shadow.cascadeMatrices[i] = FitCascade( viewInv, proj, s_shadow.sunDir, splits[i], splits[i + 1], &s_shadow.cascadeRadius[i] );
		s_shadow.cascadeFarViewZ[i] = splits[i + 1];
	}
}

void BeginShadowPass( int cascade )
{
	if ( cascade < 0 || cascade >= SHADOW_CASCADE_COUNT )
	{
		fprintf( stderr, "cascade index out of range: %d\n", cascade );
		return;
	}
	sg_pass pass = { 0 };
	pass.action.depth.load_action = SG_LOADACTION_CLEAR;
	pass.action.depth.clear_value = 1.0f;
	pass.action.depth.store_action = SG_STOREACTION_STORE;
	pass.attachments.depth_stencil = s_shadow.cascadeAttachmentViews[cascade];
	sg_begin_pass( &pass );
	s_shadow.inPass = true;
}

void EndShadowPass( void )
{
	if ( !s_shadow.inPass )
	{
		return;
	}
	sg_end_pass();
	s_shadow.inPass = false;
}

Mat4 GetCascadeMatrix( int cascade )
{
	if ( cascade < 0 || cascade >= SHADOW_CASCADE_COUNT )
	{
		return MakeIdentity();
	}
	return s_shadow.cascadeMatrices[cascade];
}

float GetCascadeFarViewZ( int cascade )
{
	if ( cascade < 0 || cascade >= SHADOW_CASCADE_COUNT )
	{
		return 0.0f;
	}
	return s_shadow.cascadeFarViewZ[cascade];
}

float GetCascadePcfScale( int cascade )
{
	if ( cascade < 0 || cascade >= SHADOW_CASCADE_COUNT )
	{
		return 1.0f;
	}
	const float scale = s_shadow.cascadeRadius[0] / s_shadow.cascadeRadius[cascade];
	return ClampFloat( scale, 0.25f, 1.0f );
}

sg_view GetShadowSampleView( void )
{
	return s_shadow.sampleView;
}

sg_sampler GetShadowSampler( void )
{
	return s_shadow.sampler;
}

float GetShadowTexelSize( void )
{
	return 1.0f / (float)SHADOW_RESOLUTION;
}
