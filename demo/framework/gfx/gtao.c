#include "gfx/gtao.h"

#include "gtao_denoise.glsl.h"
#include "gtao_main_pass.glsl.h"
#include "gtao_prefilter_depth.glsl.h"

#include <stdio.h>

#define GTAO_PREFILTER_MIP_COUNT 5

static struct
{
	int width;
	int height;

	sg_image prepassDepthImage;
	sg_view prepassDepthAttachView;

	sg_image linearDepthImage;
	sg_view linearDepthAttachView;
	sg_view linearDepthSampleView;

	sg_image prefilterDepthImage;
	sg_view prefilterMipStorageView[GTAO_PREFILTER_MIP_COUNT];
	sg_view prefilterChainSampleView;

	sg_image edgesImage;
	sg_view edgesStorageView;
	sg_view edgesSampleView;

	sg_image noisy0Image;
	sg_view noisy0StorageView;
	sg_view noisy0SampleView;
	sg_image noisy1Image;
	sg_view noisy1StorageView;
	sg_view noisy1SampleView;

	sg_image resultImage;
	sg_view resultAttachView;
	sg_view resultStorageView;
	sg_view resultSampleView;

	sg_sampler linearSampler;
	sg_sampler pointSampler;

	sg_shader prefilterShader;
	sg_pipeline prefilterPipeline;

	sg_shader mainPassShader;
	sg_pipeline mainPassPipeline;

	sg_shader denoiseShader;
	sg_pipeline denoisePipeline;

	GtaoTraceParams params; 
} s_gtao;

static const GtaoTraceParams k_gtao_default_params = {
	.radius = 0.5f,
	.falloff = 0.615f,
	.radiusMul = 1.457f,
	.finalValuePow = 2.2f,
	.denoiseBlurBeta = 1.2f,
	.sampleDistPow = 2.0f,
	.thinOcclComp = 0.0f,
	.depthMipSampOffset = 3.30f,
	.quality = AO_QUALITY_HIGH,
	.denoisePassCount = 3,
};

static void GtaoQualityToCounts( AmbientOcclusionQuality quality, int* sliceCount, int* stepsPerSlice )
{
	switch ( quality )
	{
		case AO_QUALITY_MEDIUM:
			*sliceCount = 2;
			*stepsPerSlice = 2;
			break;
		case AO_QUALITY_ULTRA:
			*sliceCount = 9;
			*stepsPerSlice = 3;
			break;
		case AO_QUALITY_HIGH:
		default:
			*sliceCount = 3;
			*stepsPerSlice = 3;
			break;
	}
}

void InitAmbientOcclusion( void )
{
	s_gtao.width = 0;
	s_gtao.height = 0;
	s_gtao.params = k_gtao_default_params;

	sg_sampler_desc linDesc = { 0 };
	linDesc.min_filter = SG_FILTER_LINEAR;
	linDesc.mag_filter = SG_FILTER_LINEAR;
	linDesc.mipmap_filter = SG_FILTER_NEAREST;
	linDesc.wrap_u = SG_WRAP_CLAMP_TO_EDGE;
	linDesc.wrap_v = SG_WRAP_CLAMP_TO_EDGE;
	linDesc.wrap_w = SG_WRAP_CLAMP_TO_EDGE;
	linDesc.label = "gtao_linear_sampler";
	s_gtao.linearSampler = sg_make_sampler( &linDesc );

	sg_sampler_desc pointDesc = { 0 };
	pointDesc.min_filter = SG_FILTER_NEAREST;
	pointDesc.mag_filter = SG_FILTER_NEAREST;
	pointDesc.mipmap_filter = SG_FILTER_NEAREST;
	pointDesc.wrap_u = SG_WRAP_CLAMP_TO_EDGE;
	pointDesc.wrap_v = SG_WRAP_CLAMP_TO_EDGE;
	pointDesc.wrap_w = SG_WRAP_CLAMP_TO_EDGE;
	pointDesc.label = "gtao_point_sampler";
	s_gtao.pointSampler = sg_make_sampler( &pointDesc );

	s_gtao.prefilterShader = sg_make_shader( gtao_prefilter_depth_prog_shader_desc( sg_query_backend() ) );

	sg_pipeline_desc ppdesc = { 0 };
	ppdesc.compute = true;
	ppdesc.shader = s_gtao.prefilterShader;
	ppdesc.label = "gtao_prefilter_pipeline";
	s_gtao.prefilterPipeline = sg_make_pipeline( &ppdesc );

	s_gtao.mainPassShader = sg_make_shader( gtao_main_pass_prog_shader_desc( sg_query_backend() ) );

	sg_pipeline_desc mpdesc = { 0 };
	mpdesc.compute = true;
	mpdesc.shader = s_gtao.mainPassShader;
	mpdesc.label = "gtao_main_pass_pipeline";
	s_gtao.mainPassPipeline = sg_make_pipeline( &mpdesc );

	s_gtao.denoiseShader = sg_make_shader( gtao_denoise_prog_shader_desc( sg_query_backend() ) );

	sg_pipeline_desc dndesc = { 0 };
	dndesc.compute = true;
	dndesc.shader = s_gtao.denoiseShader;
	dndesc.label = "gtao_denoise_pipeline";
	s_gtao.denoisePipeline = sg_make_pipeline( &dndesc );
}

static void destroyTargets( void )
{
	if ( s_gtao.resultSampleView.id )
	{
		sg_destroy_view( s_gtao.resultSampleView );
		s_gtao.resultSampleView.id = 0;
	}
	if ( s_gtao.resultStorageView.id )
	{
		sg_destroy_view( s_gtao.resultStorageView );
		s_gtao.resultStorageView.id = 0;
	}
	if ( s_gtao.resultAttachView.id )
	{
		sg_destroy_view( s_gtao.resultAttachView );
		s_gtao.resultAttachView.id = 0;
	}
	if ( s_gtao.resultImage.id )
	{
		sg_destroy_image( s_gtao.resultImage );
		s_gtao.resultImage.id = 0;
	}
	if ( s_gtao.noisy1SampleView.id )
	{
		sg_destroy_view( s_gtao.noisy1SampleView );
		s_gtao.noisy1SampleView.id = 0;
	}
	if ( s_gtao.noisy1StorageView.id )
	{
		sg_destroy_view( s_gtao.noisy1StorageView );
		s_gtao.noisy1StorageView.id = 0;
	}
	if ( s_gtao.noisy1Image.id )
	{
		sg_destroy_image( s_gtao.noisy1Image );
		s_gtao.noisy1Image.id = 0;
	}
	if ( s_gtao.noisy0SampleView.id )
	{
		sg_destroy_view( s_gtao.noisy0SampleView );
		s_gtao.noisy0SampleView.id = 0;
	}
	if ( s_gtao.noisy0StorageView.id )
	{
		sg_destroy_view( s_gtao.noisy0StorageView );
		s_gtao.noisy0StorageView.id = 0;
	}
	if ( s_gtao.noisy0Image.id )
	{
		sg_destroy_image( s_gtao.noisy0Image );
		s_gtao.noisy0Image.id = 0;
	}
	if ( s_gtao.edgesSampleView.id )
	{
		sg_destroy_view( s_gtao.edgesSampleView );
		s_gtao.edgesSampleView.id = 0;
	}
	if ( s_gtao.edgesStorageView.id )
	{
		sg_destroy_view( s_gtao.edgesStorageView );
		s_gtao.edgesStorageView.id = 0;
	}
	if ( s_gtao.edgesImage.id )
	{
		sg_destroy_image( s_gtao.edgesImage );
		s_gtao.edgesImage.id = 0;
	}
	if ( s_gtao.prefilterChainSampleView.id )
	{
		sg_destroy_view( s_gtao.prefilterChainSampleView );
		s_gtao.prefilterChainSampleView.id = 0;
	}
	for ( int i = 0; i < GTAO_PREFILTER_MIP_COUNT; ++i )
	{
		if ( s_gtao.prefilterMipStorageView[i].id )
		{
			sg_destroy_view( s_gtao.prefilterMipStorageView[i] );
			s_gtao.prefilterMipStorageView[i].id = 0;
		}
	}
	if ( s_gtao.prefilterDepthImage.id )
	{
		sg_destroy_image( s_gtao.prefilterDepthImage );
		s_gtao.prefilterDepthImage.id = 0;
	}
	if ( s_gtao.linearDepthSampleView.id )
	{
		sg_destroy_view( s_gtao.linearDepthSampleView );
		s_gtao.linearDepthSampleView.id = 0;
	}
	if ( s_gtao.linearDepthAttachView.id )
	{
		sg_destroy_view( s_gtao.linearDepthAttachView );
		s_gtao.linearDepthAttachView.id = 0;
	}
	if ( s_gtao.linearDepthImage.id )
	{
		sg_destroy_image( s_gtao.linearDepthImage );
		s_gtao.linearDepthImage.id = 0;
	}
	if ( s_gtao.prepassDepthAttachView.id )
	{
		sg_destroy_view( s_gtao.prepassDepthAttachView );
		s_gtao.prepassDepthAttachView.id = 0;
	}
	if ( s_gtao.prepassDepthImage.id )
	{
		sg_destroy_image( s_gtao.prepassDepthImage );
		s_gtao.prepassDepthImage.id = 0;
	}
}

void ShutdownAmbientOcclusion( void )
{
	destroyTargets();
	if ( s_gtao.denoisePipeline.id )
	{
		sg_destroy_pipeline( s_gtao.denoisePipeline );
		s_gtao.denoisePipeline.id = 0;
	}
	if ( s_gtao.denoiseShader.id )
	{
		sg_destroy_shader( s_gtao.denoiseShader );
		s_gtao.denoiseShader.id = 0;
	}
	if ( s_gtao.mainPassPipeline.id )
	{
		sg_destroy_pipeline( s_gtao.mainPassPipeline );
		s_gtao.mainPassPipeline.id = 0;
	}
	if ( s_gtao.mainPassShader.id )
	{
		sg_destroy_shader( s_gtao.mainPassShader );
		s_gtao.mainPassShader.id = 0;
	}
	if ( s_gtao.prefilterPipeline.id )
	{
		sg_destroy_pipeline( s_gtao.prefilterPipeline );
		s_gtao.prefilterPipeline.id = 0;
	}
	if ( s_gtao.prefilterShader.id )
	{
		sg_destroy_shader( s_gtao.prefilterShader );
		s_gtao.prefilterShader.id = 0;
	}
	if ( s_gtao.pointSampler.id )
	{
		sg_destroy_sampler( s_gtao.pointSampler );
		s_gtao.pointSampler.id = 0;
	}
	if ( s_gtao.linearSampler.id )
	{
		sg_destroy_sampler( s_gtao.linearSampler );
		s_gtao.linearSampler.id = 0;
	}
	s_gtao.width = 0;
	s_gtao.height = 0;
}

void ResizeGTAO( int width, int height )
{
	if ( width <= 0 || height <= 0 )
	{
		return;
	}
	if ( width == s_gtao.width && height == s_gtao.height && s_gtao.linearDepthImage.id )
	{
		return;
	}

	destroyTargets();

	s_gtao.width = width;
	s_gtao.height = height;

	sg_image_desc didesc = { 0 };
	didesc.usage.depth_stencil_attachment = true;
	didesc.width = width;
	didesc.height = height;
	didesc.pixel_format = SG_PIXELFORMAT_DEPTH;
	didesc.sample_count = 1;
	didesc.label = "gtao_prepass_depth";
	s_gtao.prepassDepthImage = sg_make_image( &didesc );

	sg_view_desc davdesc = { 0 };
	davdesc.depth_stencil_attachment.image = s_gtao.prepassDepthImage;
	davdesc.label = "gtao_prepass_depth_attach";
	s_gtao.prepassDepthAttachView = sg_make_view( &davdesc );

	sg_image_desc ldesc = { 0 };
	ldesc.usage.color_attachment = true;
	ldesc.width = width;
	ldesc.height = height;
	ldesc.pixel_format = SG_PIXELFORMAT_R32F;
	ldesc.sample_count = 1;
	ldesc.label = "gtao_linear_depth";
	s_gtao.linearDepthImage = sg_make_image( &ldesc );

	sg_view_desc lavdesc = { 0 };
	lavdesc.color_attachment.image = s_gtao.linearDepthImage;
	lavdesc.label = "gtao_linear_depth_attach";
	s_gtao.linearDepthAttachView = sg_make_view( &lavdesc );

	sg_view_desc lsvdesc = { 0 };
	lsvdesc.texture.image = s_gtao.linearDepthImage;
	lsvdesc.label = "gtao_linear_depth_sample";
	s_gtao.linearDepthSampleView = sg_make_view( &lsvdesc );

	sg_image_desc pdesc = { 0 };
	pdesc.usage.storage_image = true;
	pdesc.width = width;
	pdesc.height = height;
	pdesc.num_mipmaps = GTAO_PREFILTER_MIP_COUNT;
	pdesc.pixel_format = SG_PIXELFORMAT_R32F;
	pdesc.sample_count = 1;
	pdesc.label = "gtao_prefilter_depth";
	s_gtao.prefilterDepthImage = sg_make_image( &pdesc );

	for ( int i = 0; i < GTAO_PREFILTER_MIP_COUNT; ++i )
	{
		sg_view_desc msvdesc = { 0 };
		msvdesc.storage_image.image = s_gtao.prefilterDepthImage;
		msvdesc.storage_image.mip_level = i;
		msvdesc.label = "gtao_prefilter_mip_storage";
		s_gtao.prefilterMipStorageView[i] = sg_make_view( &msvdesc );
	}

	sg_view_desc pcsvdesc = { 0 };
	pcsvdesc.texture.image = s_gtao.prefilterDepthImage;

	pcsvdesc.label = "gtao_prefilter_chain_sample";
	s_gtao.prefilterChainSampleView = sg_make_view( &pcsvdesc );

	sg_image_desc edesc = { 0 };
	edesc.usage.storage_image = true;
	edesc.width = width;
	edesc.height = height;
	edesc.pixel_format = SG_PIXELFORMAT_R32F;
	edesc.sample_count = 1;
	edesc.label = "gtao_edges";
	s_gtao.edgesImage = sg_make_image( &edesc );

	sg_view_desc esvdesc = { 0 };
	esvdesc.storage_image.image = s_gtao.edgesImage;
	esvdesc.label = "gtao_edges_storage";
	s_gtao.edgesStorageView = sg_make_view( &esvdesc );

	sg_view_desc etxvdesc = { 0 };
	etxvdesc.texture.image = s_gtao.edgesImage;
	etxvdesc.label = "gtao_edges_sample";
	s_gtao.edgesSampleView = sg_make_view( &etxvdesc );

	sg_image_desc ndesc = { 0 };
	ndesc.usage.storage_image = true;
	ndesc.width = width;
	ndesc.height = height;
	ndesc.pixel_format = SG_PIXELFORMAT_R32F;
	ndesc.sample_count = 1;

	ndesc.label = "gtao_noisy0";
	s_gtao.noisy0Image = sg_make_image( &ndesc );
	ndesc.label = "gtao_noisy1";
	s_gtao.noisy1Image = sg_make_image( &ndesc );

	sg_view_desc n0stvdesc = { 0 };
	n0stvdesc.storage_image.image = s_gtao.noisy0Image;
	n0stvdesc.label = "gtao_noisy0_storage";
	s_gtao.noisy0StorageView = sg_make_view( &n0stvdesc );

	sg_view_desc n0txvdesc = { 0 };
	n0txvdesc.texture.image = s_gtao.noisy0Image;
	n0txvdesc.label = "gtao_noisy0_sample";
	s_gtao.noisy0SampleView = sg_make_view( &n0txvdesc );

	sg_view_desc n1stvdesc = { 0 };
	n1stvdesc.storage_image.image = s_gtao.noisy1Image;
	n1stvdesc.label = "gtao_noisy1_storage";
	s_gtao.noisy1StorageView = sg_make_view( &n1stvdesc );

	sg_view_desc n1txvdesc = { 0 };
	n1txvdesc.texture.image = s_gtao.noisy1Image;
	n1txvdesc.label = "gtao_noisy1_sample";
	s_gtao.noisy1SampleView = sg_make_view( &n1txvdesc );

	sg_image_desc rdesc = { 0 };
	rdesc.usage.color_attachment = true;
	rdesc.usage.storage_image = true;
	rdesc.width = width;
	rdesc.height = height;
	rdesc.pixel_format = SG_PIXELFORMAT_R32F;
	rdesc.sample_count = 1;
	rdesc.label = "gtao_result";
	s_gtao.resultImage = sg_make_image( &rdesc );

	sg_view_desc ravdesc = { 0 };
	ravdesc.color_attachment.image = s_gtao.resultImage;
	ravdesc.label = "gtao_result_attach";
	s_gtao.resultAttachView = sg_make_view( &ravdesc );

	sg_view_desc rstvdesc = { 0 };
	rstvdesc.storage_image.image = s_gtao.resultImage;
	rstvdesc.label = "gtao_result_storage";
	s_gtao.resultStorageView = sg_make_view( &rstvdesc );

	sg_view_desc rsvdesc = { 0 };
	rsvdesc.texture.image = s_gtao.resultImage;
	rsvdesc.label = "gtao_result_sample";
	s_gtao.resultSampleView = sg_make_view( &rsvdesc );
}

sg_view GetLinearDepthAttachmentView( void )
{
	return s_gtao.linearDepthAttachView;
}

sg_view GetPrepassDepthAttachmentView( void )
{
	return s_gtao.prepassDepthAttachView;
}

sg_view GetLinearDepthSampleView( void )
{
	return s_gtao.linearDepthSampleView;
}

sg_view GetPrefilterDepthSampleView( void )
{
	return s_gtao.prefilterChainSampleView;
}

sg_view GetAOResultSampleView( void )
{
	return s_gtao.resultSampleView;
}

sg_sampler GetAOLinearSampler( void )
{
	return s_gtao.linearSampler;
}

void ClearAmbientOcclusion( void )
{
	if ( !s_gtao.resultImage.id )
	{
		fprintf( stderr, "[gtao/error] ClearAmbientOcclusion called before ResizeGTAO succeeded\n" );
		return;
	}
	sg_pass pass = { 0 };
	pass.action.colors[0].load_action = SG_LOADACTION_CLEAR;
	pass.action.colors[0].clear_value.r = 1.0f;
	pass.action.colors[0].clear_value.g = 1.0f;
	pass.action.colors[0].clear_value.b = 1.0f;
	pass.action.colors[0].clear_value.a = 1.0f;
	pass.action.colors[0].store_action = SG_STOREACTION_STORE;
	pass.attachments.colors[0] = s_gtao.resultAttachView;
	sg_push_debug_group( "gtao_clear_white" );
	sg_begin_pass( &pass );
	sg_end_pass();
	sg_pop_debug_group();
}

typedef struct
{
	int viewportWidth;
	int viewportHeight;
	Vec2 viewportPixelSize;
	Vec2 ndcToViewMul;
	Vec2 ndcToViewAdd;
	Vec2 ndcToViewMul_x_PixelSize;
	GtaoTraceParams params;
} GtaoUboInputs;

static GtaoUboInputs ComputeUboInputs( const Mat4* proj, int viewportWidth, int viewportHeight )
{
	GtaoUboInputs in;
	in.viewportWidth = viewportWidth;
	in.viewportHeight = viewportHeight;
	in.viewportPixelSize = (Vec2){ 1.0f / (float)viewportWidth, 1.0f / (float)viewportHeight };

	const float tanHalfFOVY = 1.0f / proj->cy.y;
	const float tanHalfFOVX = 1.0f / proj->cx.x;
	in.ndcToViewMul = (Vec2){ tanHalfFOVX * 2.0f, tanHalfFOVY * -2.0f };
	in.ndcToViewAdd = (Vec2){ tanHalfFOVX * -1.0f, tanHalfFOVY * 1.0f };
	in.ndcToViewMul_x_PixelSize = (Vec2){
		in.ndcToViewMul.x * in.viewportPixelSize.x,
		in.ndcToViewMul.y * in.viewportPixelSize.y,
	};

	in.params = s_gtao.params;
	return in;
}

static gtao_prefilter_depth_ub_gtao_t buildPrefilterUbo( const GtaoUboInputs* in )
{
	gtao_prefilter_depth_ub_gtao_t c = { 0 };
	c.ViewportSize[0] = in->viewportWidth;
	c.ViewportSize[1] = in->viewportHeight;
	c.EffectRadius = in->params.radius;
	c.EffectFalloffRange = in->params.falloff;
	c.RadiusMultiplier = in->params.radiusMul;
	return c;
}

static gtao_main_pass_ub_gtao_t MakeMainPassUbo( const GtaoUboInputs* in )
{
	gtao_main_pass_ub_gtao_t c = { 0 };
	c.ViewportSize[0] = in->viewportWidth;
	c.ViewportSize[1] = in->viewportHeight;
	c.ViewportPixelSize = in->viewportPixelSize;
	c.NDCToViewMul = in->ndcToViewMul;
	c.NDCToViewAdd = in->ndcToViewAdd;
	c.NDCToViewMul_x_PixelSize = in->ndcToViewMul_x_PixelSize;
	c.EffectRadius = in->params.radius;
	c.EffectFalloffRange = in->params.falloff;
	c.RadiusMultiplier = in->params.radiusMul;
	c.FinalValuePower = in->params.finalValuePow;
	c.SampleDistributionPower = in->params.sampleDistPow;
	c.ThinOccluderCompensation = in->params.thinOcclComp;
	c.DepthMIPSamplingOffset = in->params.depthMipSampOffset;
	int sliceCount, stepsPerSlice;
	GtaoQualityToCounts( in->params.quality, &sliceCount, &stepsPerSlice );
	c.SliceCount = (float)sliceCount;
	c.StepsPerSlice = (float)stepsPerSlice;
	return c;
}

static gtao_denoise_ub_gtao_t MakeDenoiseUbo( const GtaoUboInputs* in )
{
	gtao_denoise_ub_gtao_t c = { 0 };
	c.ViewportSize[0] = in->viewportWidth;
	c.ViewportSize[1] = in->viewportHeight;
	c.DenoiseBlurBeta = in->params.denoiseBlurBeta;
	return c;
}

GtaoTraceParams GetGtaoTraceParams( void )
{
	return s_gtao.params;
}
void SetGtaoTraceParams( GtaoTraceParams p )
{
	if ( p.denoisePassCount < 1 )
	{
		p.denoisePassCount = 1;
	}
	if ( p.denoisePassCount > GTAO_MAX_DENOISE_PASSES )
	{
		p.denoisePassCount = GTAO_MAX_DENOISE_PASSES;
	}
	s_gtao.params = p;
}

GtaoTraceParams GetDefaultGtaoTraceParams( void )
{
	return k_gtao_default_params;
}

AmbientOcclusionQuality GetAOQualityForPixelScale( float dpiScale )
{
	const float highMinDpiScale = 2.0f;
	return ( dpiScale >= highMinDpiScale ) ? AO_QUALITY_HIGH : AO_QUALITY_ULTRA;
}

void PrefilterDepth( const Mat4* proj, int viewportWidth, int viewportHeight )
{
	if ( !s_gtao.prefilterDepthImage.id )
	{
		fprintf( stderr, "[gtao/error] PrefilterDepth called before ResizeGTAO succeeded\n" );
		return;
	}

	GtaoUboInputs inputs = ComputeUboInputs( proj, viewportWidth, viewportHeight );
	gtao_prefilter_depth_ub_gtao_t consts = buildPrefilterUbo( &inputs );

	sg_pass pass = { 0 };
	pass.compute = true;
	pass.label = "gtao_prefilter_depth";
	sg_push_debug_group( "gtao_prefilter_depth" );
	sg_begin_pass( &pass );

	sg_apply_pipeline( s_gtao.prefilterPipeline );

	sg_bindings bindings = { 0 };
	bindings.views[VIEW_gtao_prefilter_depth_out_depth0] = s_gtao.prefilterMipStorageView[0];
	bindings.views[VIEW_gtao_prefilter_depth_out_depth1] = s_gtao.prefilterMipStorageView[1];
	bindings.views[VIEW_gtao_prefilter_depth_out_depth2] = s_gtao.prefilterMipStorageView[2];
	bindings.views[VIEW_gtao_prefilter_depth_out_depth3] = s_gtao.prefilterMipStorageView[3];
	bindings.views[VIEW_gtao_prefilter_depth_out_depth4] = s_gtao.prefilterMipStorageView[4];
	bindings.views[VIEW_gtao_prefilter_depth_tex_src_depth] = s_gtao.linearDepthSampleView;
	bindings.samplers[SMP_gtao_prefilter_depth_smp_src_depth] = s_gtao.pointSampler;
	sg_apply_bindings( &bindings );

	sg_apply_uniforms( UB_gtao_prefilter_depth_ub_gtao, &SG_RANGE( consts ) );

	const int groupsX = ( viewportWidth + 15 ) / 16;
	const int groupsY = ( viewportHeight + 15 ) / 16;
	sg_dispatch( groupsX, groupsY, 1 );

	sg_end_pass();
	sg_pop_debug_group();
}

void ComputeNoisyResult( const Mat4* proj, int viewportWidth, int viewportHeight )
{
	if ( !s_gtao.noisy0Image.id || !s_gtao.edgesImage.id || !s_gtao.prefilterDepthImage.id )
	{
		fprintf( stderr, "[gtao/error] ComputeNoisyResult called before ResizeGTAO succeeded\n" );
		return;
	}

	GtaoUboInputs inputs = ComputeUboInputs( proj, viewportWidth, viewportHeight );
	gtao_main_pass_ub_gtao_t consts = MakeMainPassUbo( &inputs );

	sg_pass pass = { 0 };
	pass.compute = true;
	pass.label = "gtao_main_pass";
	sg_push_debug_group( "gtao_main_pass" );
	sg_begin_pass( &pass );

	sg_apply_pipeline( s_gtao.mainPassPipeline );

	sg_bindings bindings = { 0 };

	bindings.views[VIEW_gtao_main_pass_out_noisy_ao] = s_gtao.noisy0StorageView;
	bindings.views[VIEW_gtao_main_pass_out_edges] = s_gtao.edgesStorageView;
	bindings.views[VIEW_gtao_main_pass_tex_raw_depth] = s_gtao.linearDepthSampleView;
	bindings.views[VIEW_gtao_main_pass_tex_prefilter_depth] = s_gtao.prefilterChainSampleView;
	bindings.samplers[SMP_gtao_main_pass_smp_point_clamp] = s_gtao.pointSampler;
	sg_apply_bindings( &bindings );

	sg_apply_uniforms( UB_gtao_main_pass_ub_gtao, &SG_RANGE( consts ) );

	const int groupsX = ( viewportWidth + 7 ) / 8;
	const int groupsY = ( viewportHeight + 7 ) / 8;
	sg_dispatch( groupsX, groupsY, 1 );

	sg_end_pass();
	sg_pop_debug_group();
}

static void runDenoiseDispatch( const gtao_denoise_ub_gtao_t* consts, sg_view srcSample, sg_view dstStorage, bool finalApply,
								const char* label )
{
	sg_pass pass = { 0 };
	pass.compute = true;
	pass.label = label;
	sg_push_debug_group( label );
	sg_begin_pass( &pass );

	sg_apply_pipeline( s_gtao.denoisePipeline );

	sg_bindings bindings = { 0 };
	bindings.views[VIEW_gtao_denoise_out_ao] = dstStorage;
	bindings.views[VIEW_gtao_denoise_tex_edges] = s_gtao.edgesSampleView;
	bindings.views[VIEW_gtao_denoise_tex_noisy_ao] = srcSample;
	bindings.samplers[SMP_gtao_denoise_smp_point_clamp] = s_gtao.pointSampler;
	sg_apply_bindings( &bindings );

	sg_apply_uniforms( UB_gtao_denoise_ub_gtao, &SG_RANGE( *consts ) );

	gtao_denoise_ub_denoise_t apply = { finalApply ? 1 : 0, 0, 0, 0 };
	sg_apply_uniforms( UB_gtao_denoise_ub_denoise, &SG_RANGE( apply ) );

	const int groupsX = ( s_gtao.width + 15 ) / 16;
	const int groupsY = ( s_gtao.height + 7 ) / 8;
	sg_dispatch( groupsX, groupsY, 1 );

	sg_end_pass();
	sg_pop_debug_group();
}

void Denoise( const Mat4* proj, int viewportWidth, int viewportHeight )
{
	if ( !s_gtao.noisy0Image.id || !s_gtao.noisy1Image.id || !s_gtao.resultImage.id || !s_gtao.edgesImage.id )
	{
		fprintf( stderr, "[gtao/error] Denoise called before ResizeGTAO succeeded\n" );
		return;
	}

	GtaoUboInputs inputs = ComputeUboInputs( proj, viewportWidth, viewportHeight );
	gtao_denoise_ub_gtao_t consts = MakeDenoiseUbo( &inputs );

	int passCount = s_gtao.params.denoisePassCount;
	if ( passCount < 1 )
	{
		passCount = 1;
	}
	if ( passCount > GTAO_MAX_DENOISE_PASSES )
	{
		passCount = GTAO_MAX_DENOISE_PASSES;
	}

	static const char* k_intermediateLabels[GTAO_MAX_DENOISE_PASSES] = {
		"gtao_denoise_pass0", "gtao_denoise_pass1", "gtao_denoise_pass2",
		"gtao_denoise_pass3", "gtao_denoise_pass4",
	};

	sg_view src = s_gtao.noisy0SampleView;
	sg_view dst = s_gtao.noisy1StorageView;
	sg_view srcAlt = s_gtao.noisy1SampleView;
	sg_view dstAlt = s_gtao.noisy0StorageView;

	for ( int i = 0; i < passCount - 1; ++i )
	{
		runDenoiseDispatch( &consts, src, dst, false, k_intermediateLabels[i] );
		sg_view tmpS = src;
		src = srcAlt;
		srcAlt = tmpS;
		sg_view tmpD = dst;
		dst = dstAlt;
		dstAlt = tmpD;
	}
	runDenoiseDispatch( &consts, src, s_gtao.resultStorageView, true, "gtao_denoise_final" );
}
