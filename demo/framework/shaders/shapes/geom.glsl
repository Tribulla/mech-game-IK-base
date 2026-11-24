#pragma sokol @module geom

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype vec3 Vec3
#pragma sokol @ctype vec2 Vec2

#pragma sokol @vs vs

layout( binding = 0 ) uniform ub_frame
{
	mat4 view_proj;
};

layout( binding = 2 ) uniform ub_draw
{
	ivec4 inst_base; 
};

struct instance
{
	vec4 xform_row0; 
	vec4 xform_row1; 
	vec4 xform_row2; 
	vec4 base_color; 
	vec4 scale;		 
	vec4 material;	 

};

layout( binding = 0 ) readonly buffer instances
{
	instance items[];
};

in vec3 in_pos;
in vec3 in_normal;

out vec3 v_view_pos;	 

out vec3 v_world_normal; 
flat out vec4 v_base_color;
flat out vec4 v_material;

void main()
{
	instance inst = items[inst_base.x + gl_InstanceIndex];

	vec3 scaled_pos = in_pos * inst.scale.xyz;
	vec4 local_pos = vec4( scaled_pos, 1.0 );
	vec3 world_pos =
		vec3( dot( inst.xform_row0, local_pos ), dot( inst.xform_row1, local_pos ), dot( inst.xform_row2, local_pos ) );

	vec3 inv_scaled_n = in_normal / inst.scale.xyz;
	vec3 world_normal = vec3( dot( inst.xform_row0.xyz, inv_scaled_n ), dot( inst.xform_row1.xyz, inv_scaled_n ),
							  dot( inst.xform_row2.xyz, inv_scaled_n ) );

	v_world_normal = world_normal;
	v_base_color = inst.base_color;
	v_material = inst.material;

	gl_Position = view_proj * vec4( world_pos, 1.0 );

	v_view_pos = world_pos; 
}
#pragma sokol @end

#pragma sokol @fs fs

#pragma sokol @include ../common/pbr.glsl

layout( binding = 1 ) uniform ub_pass
{
	vec4 sun_dir_world;		  
	vec4 sun_color;			  
	ivec4 flags;			  
	vec4 camera_pos;		  
	vec4 grid_offset;		  
	mat4 view;				  
	vec4 cascade_far_view_z;  
	mat4 cascade_matrices[3]; 
	vec4 cascade_pcf_scale;	  
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

#pragma sokol @include ../common/shadow_pcf.glsl

float sampleCascade( int cascade, vec3 world_pos, vec3 world_normal )
{
	vec4 light_clip = cascade_matrices[cascade] * vec4( world_pos, 1.0 );
	vec3 light_ndc = light_clip.xyz / light_clip.w;

	float ny = light_ndc.y * cascade_far_view_z.w;
	vec3 light_uv = vec3( light_ndc.x * 0.5 + 0.5, ny * 0.5 + 0.5, light_ndc.z );

	if ( any( lessThan( light_uv, vec3( 0.0 ) ) ) || any( greaterThan( light_uv, vec3( 1.0 ) ) ) )
	{
		return 1.0;
	}

	float n_dot_l = max( dot( world_normal, normalize( sun_dir_world.xyz ) ), 0.0 );
	float bias = mix( 0.0040, 0.0008, n_dot_l );

	float pcf = sampleShadowPCF( tex_shadow, smp_shadow, cascade, light_uv, bias, cascade_pcf_scale[cascade] );

	if ( cascade == 2 )
	{
		vec2 edge = min( light_uv.xy, 1.0 - light_uv.xy );
		float fade = clamp( min( edge.x, edge.y ) * 10.0, 0.0, 1.0 );
		pcf = mix( 1.0, pcf, fade );
	}

	return pcf;
}

float sampleShadowCascaded( vec3 world_pos, vec3 world_normal, float view_z )
{
	int cascade = 2;
	if ( view_z < cascade_far_view_z.x )
		cascade = 0;
	else if ( view_z < cascade_far_view_z.y )
		cascade = 1;

	float shadow = sampleCascade( cascade, world_pos, world_normal );

	if ( cascade < 2 )
	{
		float far_z = ( cascade == 0 ) ? cascade_far_view_z.x : cascade_far_view_z.y;
		float near_z = ( cascade == 0 ) ? 0.0 : cascade_far_view_z.x;
		float blend_start = mix( far_z, near_z, 0.1 );
		if ( view_z > blend_start )
		{
			float alpha = ( view_z - blend_start ) / ( far_z - blend_start );
			shadow = mix( shadow, sampleCascade( cascade + 1, world_pos, world_normal ), alpha );
		}
	}

	return shadow;
}

in vec3 v_view_pos; 
in vec3 v_world_normal;
flat in vec4 v_base_color;
flat in vec4 v_material;

out vec4 out_color;

void main()
{
	vec3 n_world = normalize( v_world_normal );

	if ( dot( n_world, camera_pos.xyz - v_view_pos ) < 0.0 )
	{
		n_world = -n_world;
	}

	if ( flags.x == 1 )
	{
		vec3 hit_view = ( view * vec4( v_view_pos, 1.0 ) ).xyz;
		float view_depth = -hit_view.z;
		float v = clamp( view_depth / 50.0, 0.0, 1.0 );
		out_color = vec4( v, v, v, 1.0 );
		return;
	}
	if ( flags.x == 2 )
	{

		float view_z_dbg = -( view * vec4( v_view_pos, 1.0 ) ).z;
		int cascade = 2;
		if ( view_z_dbg < cascade_far_view_z.x )
			cascade = 0;
		else if ( view_z_dbg < cascade_far_view_z.y )
			cascade = 1;
		vec4 light_clip = cascade_matrices[cascade] * vec4( v_view_pos, 1.0 );
		vec3 light_ndc = light_clip.xyz / light_clip.w;
		float ny = light_ndc.y * cascade_far_view_z.w;
		vec3 light_uv = vec3( light_ndc.x * 0.5 + 0.5, ny * 0.5 + 0.5, light_ndc.z );
		bool oob = any( lessThan( light_uv, vec3( 0.0 ) ) ) || any( greaterThan( light_uv, vec3( 1.0 ) ) );
		vec3 tint = vec3( 1.0 );
		if ( !oob )
		{
			if ( cascade == 0 )
				tint = vec3( 1.0, 0.4, 0.4 );
			else if ( cascade == 1 )
				tint = vec3( 0.4, 1.0, 0.4 );
			else
				tint = vec3( 0.4, 0.4, 1.0 );
		}
		out_color = vec4( tint, 1.0 );
		return;
	}
	if ( flags.x == 3 )
	{

		vec3 n_view = ( view * vec4( n_world, 0.0 ) ).xyz;
		out_color = vec4( normalize( n_view ) * 0.5 + 0.5, 1.0 );
		return;
	}

	vec2 ao_uv = gl_FragCoord.xy / vec2( textureSize( sampler2D( tex_ao, smp_ao ), 0 ) );
	float ao = textureLod( sampler2D( tex_ao, smp_ao ), ao_uv, 0.0 ).x;

	if ( flags.x == 4 )
	{
		out_color = vec4( vec3( ao ), 1.0 );
		return;
	}

	vec3 world_pos = v_view_pos;

	vec3 baseColor = v_base_color.rgb;
	if ( v_material.z > 0.5 )
	{

		vec2 line_xz = world_pos.xz + grid_offset.xy;
		vec2 axis_xz = world_pos.xz + grid_offset.zw;
		vec3 grid_color = proceduralGrid( line_xz, axis_xz, v_base_color.rgb, v_material.w );
		float grid_blend = pow( clamp( n_world.y, 0.0, 1.0 ), 8.0 );
		baseColor = mix( v_base_color.rgb, grid_color, grid_blend );
	}

	vec3 L = normalize( sun_dir_world.xyz );
	float n_dot_l = max( dot( n_world, L ), 0.0 );
	float view_z = -( view * vec4( world_pos, 1.0 ) ).z;
	float shadow = sampleShadowCascaded( world_pos, n_world, view_z );
	vec3 V = normalize( camera_pos.xyz - world_pos );
	vec3 direct = brdf_evaluate( n_world, V, L, sun_color.rgb * shadow, baseColor, v_material.x, v_material.y );
	vec3 ambient = pbrEvaluateAmbient( ibl_params.y > 0.5, n_world, V, baseColor, v_material.x, v_material.y, sun_color.a, sh,
									   tex_ibl_spec, smp_ibl_spec, tex_brdf_lut, smp_brdf_lut, ibl_params.x );

	out_color = vec4( ( direct + ambient * ao ) * v_base_color.a, v_base_color.a );
}
#pragma sokol @end

#pragma sokol @program triangle vs fs
