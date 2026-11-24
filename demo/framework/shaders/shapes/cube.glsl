#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype vec3 Vec3
#pragma sokol @ctype vec2 Vec2

#pragma sokol @vs vs

layout( binding = 0 ) uniform ub_frame
{
	mat4 view;
	mat4 proj;
	mat4 view_proj;
	mat4 inv_view_proj;
	vec4 camera_pos; 
	vec4 viewport;	 
};

layout( binding = 2 ) uniform ub_draw
{
	ivec4 inst_base;
};

struct cube_instance
{
	vec4 xform_row0; 
	vec4 xform_row1;
	vec4 xform_row2;
	vec4 base_color; 
	vec4 material;	 

};

layout( binding = 0 ) readonly buffer cube_instances
{
	cube_instance instances[];
};

in vec3 in_pos; 

out vec3 v_view_pos;  
out vec3 v_world_pos; 
flat out vec4 v_base_color;
flat out vec4 v_material;

void main()
{
	cube_instance inst = instances[inst_base.x + gl_InstanceIndex];
	vec4 local_pos = vec4( in_pos, 1.0 );
	vec3 world_pos =
		vec3( dot( inst.xform_row0, local_pos ), dot( inst.xform_row1, local_pos ), dot( inst.xform_row2, local_pos ) );
	vec4 view_pos = view * vec4( world_pos, 1.0 );
	v_view_pos = view_pos.xyz;
	v_world_pos = world_pos;
	v_base_color = inst.base_color;
	v_material = inst.material;
	gl_Position = proj * view_pos;
}
#pragma sokol @end

#pragma sokol @fs fs

#pragma sokol @include ../common/pbr.glsl

layout( binding = 1 ) uniform ub_pass
{
	vec4 sun_dir_view;		  
	vec4 sun_color;			  
	ivec4 flags;			  
	vec4 cascade_far_view_z;  
	mat4 cascade_matrices[3]; 
	vec4 cascade_pcf_scale;	  
	mat4 view;				  
	vec4 camera_pos_world;	  
	vec4 sh[9];				  
	vec4 ibl_params;		  
};

#pragma sokol @image_sample_type tex_shadow depth
layout( binding = 1 ) uniform texture2DArray tex_shadow;
#pragma sokol @sampler_type smp_shadow comparison
layout( binding = 0 ) uniform samplerShadow smp_shadow;

#pragma sokol @image_sample_type tex_ibl_spec float
layout( binding = 2 ) uniform textureCube tex_ibl_spec;
#pragma sokol @sampler_type smp_ibl_spec filtering
layout( binding = 1 ) uniform sampler smp_ibl_spec;
#pragma sokol @image_sample_type tex_brdf_lut float
layout( binding = 3 ) uniform texture2D tex_brdf_lut;
#pragma sokol @sampler_type smp_brdf_lut filtering
layout( binding = 2 ) uniform sampler smp_brdf_lut;

#pragma sokol @image_sample_type tex_ao float
layout( binding = 4 ) uniform texture2D tex_ao;
#pragma sokol @sampler_type smp_ao filtering
layout( binding = 3 ) uniform sampler smp_ao;

in vec3 v_view_pos;
in vec3 v_world_pos;
flat in vec4 v_base_color;
flat in vec4 v_material;
out vec4 out_color;

#pragma sokol @include ../common/shadow_pcf.glsl

float sampleCascade( int cascade, vec3 world_pos, float n_dot_l )
{
	vec4 light_clip = cascade_matrices[cascade] * vec4( world_pos, 1.0 );
	vec3 light_ndc = light_clip.xyz / light_clip.w;

	float ny = light_ndc.y * cascade_far_view_z.w;
	vec3 light_uv = vec3( light_ndc.x * 0.5 + 0.5, ny * 0.5 + 0.5, light_ndc.z );

	if ( any( lessThan( light_uv, vec3( 0.0 ) ) ) || any( greaterThan( light_uv, vec3( 1.0 ) ) ) )
	{
		return 1.0;
	}

	float ndl = clamp( n_dot_l, 0.001, 1.0 );
	float tanAngle = sqrt( 1.0 - ndl * ndl ) / ndl;
	float texelDepthStep = 0.001;
	float bias = clamp( texelDepthStep * tanAngle, 0.0005, 0.05 );

	float pcf = sampleShadowPCF( tex_shadow, smp_shadow, cascade, light_uv, bias, cascade_pcf_scale[cascade] );

	if ( cascade == 2 )
	{
		vec2 edge = min( light_uv.xy, 1.0 - light_uv.xy );
		float fade = clamp( min( edge.x, edge.y ) * 10.0, 0.0, 1.0 );
		pcf = mix( 1.0, pcf, fade );
	}

	return pcf;
}

float sampleShadowCascaded( vec3 world_pos, float n_dot_l, float view_z )
{
	int cascade = 2;
	if ( view_z < cascade_far_view_z.x )
	{
		cascade = 0;
	}
	else if ( view_z < cascade_far_view_z.y )
	{
		cascade = 1;
	}

	float shadow = sampleCascade( cascade, world_pos, n_dot_l );

	if ( cascade < 2 )
	{
		float far_z = ( cascade == 0 ) ? cascade_far_view_z.x : cascade_far_view_z.y;
		float near_z = ( cascade == 0 ) ? 0.0 : cascade_far_view_z.x;
		float blend_start = mix( far_z, near_z, 0.1 );
		if ( view_z > blend_start )
		{
			float alpha = ( view_z - blend_start ) / ( far_z - blend_start );
			shadow = mix( shadow, sampleCascade( cascade + 1, world_pos, n_dot_l ), alpha );
		}
	}

	return shadow;
}

void main()
{

	vec3 dx = dFdx( v_view_pos );

	vec3 dy = dFdy( v_view_pos ) * ( -cascade_far_view_z.w );
	vec3 n_view = normalize( cross( dy, dx ) );

	float view_z = -v_view_pos.z;

	if ( flags.x == 1 )
	{
		float v = clamp( view_z / 50.0, 0.0, 1.0 );
		out_color = vec4( v, v, v, 1.0 );
		return;
	}

	if ( flags.x == 2 )
	{
		int cascade = 2;
		if ( view_z < cascade_far_view_z.x )
		{
			cascade = 0;
		}
		else if ( view_z < cascade_far_view_z.y )
		{
			cascade = 1;
		}

		vec4 light_clip = cascade_matrices[cascade] * vec4( v_world_pos, 1.0 );
		vec3 light_ndc = light_clip.xyz / light_clip.w;
		float ny = light_ndc.y * cascade_far_view_z.w;
		vec3 light_uv = vec3( light_ndc.x * 0.5 + 0.5, ny * 0.5 + 0.5, light_ndc.z );
		bool oob = any( lessThan( light_uv, vec3( 0.0 ) ) ) || any( greaterThan( light_uv, vec3( 1.0 ) ) );
		vec3 tint = vec3( 1.0 );
		if ( !oob )
		{
			if ( cascade == 0 )
			{
				tint = vec3( 1.0, 0.4, 0.4 );
			}
			else if ( cascade == 1 )
			{
				tint = vec3( 0.4, 1.0, 0.4 );
			}
			else
			{
				tint = vec3( 0.4, 0.4, 1.0 );
			}
		}
		out_color = vec4( tint, 1.0 );
		return;
	}

	if ( flags.x == 3 )
	{

		out_color = vec4( n_view * 0.5 + 0.5, 1.0 );
		return;
	}

	vec2 ao_uv = gl_FragCoord.xy / vec2( textureSize( sampler2D( tex_ao, smp_ao ), 0 ) );
	float ao = textureLod( sampler2D( tex_ao, smp_ao ), ao_uv, 0.0 ).x;

	if ( flags.x == 4 )
	{
		out_color = vec4( vec3( ao ), 1.0 );
		return;
	}

	vec3 L = normalize( sun_dir_view.xyz );
	float n_dot_l = max( dot( n_view, L ), 0.0 );
	float shadow = sampleShadowCascaded( v_world_pos, n_dot_l, view_z );

	vec3 V_view = normalize( -v_view_pos );
	vec3 direct = brdf_evaluate( n_view, V_view, L, sun_color.rgb * shadow, v_base_color.rgb, v_material.x, v_material.y );

	mat3 view_to_world = transpose( mat3( view ) );
	vec3 N_world = normalize( view_to_world * n_view );
	vec3 V_world = normalize( camera_pos_world.xyz - v_world_pos );
	vec3 ambient = pbrEvaluateAmbient( ibl_params.y > 0.5, N_world, V_world, v_base_color.rgb, v_material.x, v_material.y,
									   sun_color.a, sh, tex_ibl_spec, smp_ibl_spec, tex_brdf_lut, smp_brdf_lut, ibl_params.x );

	out_color = vec4( ( direct + ambient * ao ) * v_base_color.a, v_base_color.a );
}
#pragma sokol @end

#pragma sokol @program cube vs fs
