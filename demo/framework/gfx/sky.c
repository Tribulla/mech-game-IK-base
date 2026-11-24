#include "gfx/sky.h"

#include "gfx/scene_target.h"
#include "sky.glsl.h"

#define BELOW_HORIZON_FADE_START_SIN 0.0349f
#define BELOW_HORIZON_FADE_END_SIN 0.0872f

static struct
{
	sg_shader shader;
	sg_pipeline pipeline;
	bool ready;
} s_sky;

void InitSky( const sg_environment* env )
{
	s_sky.shader = sg_make_shader( sky_shader_desc( sg_query_backend() ) );

	sg_pipeline_desc pdesc = { 0 };
	pdesc.shader = s_sky.shader;
	pdesc.colors[0].pixel_format = SG_PIXELFORMAT_RGBA16F;
	pdesc.depth.pixel_format = env->defaults.depth_format;
	pdesc.depth.compare = SG_COMPAREFUNC_GREATER_EQUAL;
	pdesc.depth.write_enabled = false;
	pdesc.cull_mode = SG_CULLMODE_NONE;
	pdesc.face_winding = SG_FACEWINDING_CCW;
	pdesc.sample_count = SCENE_SAMPLE_COUNT;
	pdesc.label = "sky_pipeline";
	s_sky.pipeline = sg_make_pipeline( &pdesc );

	s_sky.ready = true;
}

void ShutdownSky( void )
{
	if ( !s_sky.ready )
	{
		return;
	}
	sg_destroy_pipeline( s_sky.pipeline );
	sg_destroy_shader( s_sky.shader );
	s_sky.ready = false;
}

float SkySunFadeWeight( Vec3 dirToSun, bool zUp )
{
	float elevSin = zUp ? dirToSun.z : dirToSun.y;
	float t = ( elevSin - BELOW_HORIZON_FADE_START_SIN ) / ( BELOW_HORIZON_FADE_END_SIN - BELOW_HORIZON_FADE_START_SIN );
	t = ( t < 0.0f ) ? 0.0f : ( t > 1.0f ? 1.0f : t );
	return t * t * ( 3.0f - 2.0f * t );
}

void DrawSky( Vec3 dirToSun, float turbidity, Vec3 cameraPos, Mat4 invViewProj, bool zUp )
{
	if ( !s_sky.ready )
	{
		return;
	}

	const float lenSq = dirToSun.x * dirToSun.x + dirToSun.y * dirToSun.y + dirToSun.z * dirToSun.z;
	Vec3 sun = ( lenSq > 0.0f ) ? Normalize( dirToSun ) : Vec3_axisY;

	const float fade = SkySunFadeWeight( sun, zUp );

	sky_ub_frame_t ub = { 0 };
	ub.inv_view_proj = invViewProj;

	sky_ub_pass_t up = { 0 };
	up.sun_dir_world = MakeVec4( sun.x, sun.y, sun.z, fade );
	up.sky_params = MakeVec4( turbidity, zUp ? 1.0f : 0.0f, 0.0f, 0.0f );
	up.camera_pos = MakeVec4( cameraPos.x, cameraPos.y, cameraPos.z, 0.0f );

	sg_apply_pipeline( s_sky.pipeline );

	sg_apply_uniforms( UB_sky_ub_frame, &SG_RANGE( ub ) );
	sg_apply_uniforms( UB_sky_ub_pass, &SG_RANGE( up ) );
	sg_draw( 0, 3, 1 );
}
