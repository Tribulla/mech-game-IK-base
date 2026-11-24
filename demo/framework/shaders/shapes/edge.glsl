#pragma sokol @module edge

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4

#pragma sokol @vs vs

layout( binding = 0 ) uniform ub_frame
{
	mat4 view_proj;
	vec4 viewport; 
};

layout( binding = 2 ) uniform ub_draw
{
	ivec4 inst_base;  
	vec4 edge_params; 
};

struct geom_instance
{
	vec4 xform_row0;
	vec4 xform_row1;
	vec4 xform_row2;
	vec4 base_color;
	vec4 scale;
	vec4 material;
};

layout( binding = 0 ) readonly buffer geom_instances
{
	geom_instance gi_items[];
};

struct edge_endpoint
{
	vec4 pos_and_flag;
};

layout( binding = 1 ) readonly buffer edge_endpoints
{
	edge_endpoint ee_items[];
};

noperspective out float v_dist_from_axis_px;

flat out float v_half_width_px;

flat out float v_flag;

void main()
{

	int edge_idx = int( uint( gl_VertexIndex ) / 6u );
	int corner = gl_VertexIndex - edge_idx * 6;

	bool is_b = ( corner == 1 ) || ( corner == 2 ) || ( corner == 4 );
	float side = ( ( corner == 2 ) || ( corner == 4 ) || ( corner == 5 ) ) ? 1.0 : -1.0;

	geom_instance inst = gi_items[inst_base.x + gl_InstanceIndex];
	vec4 ea = ee_items[2 * edge_idx + 0].pos_and_flag;
	vec4 eb = ee_items[2 * edge_idx + 1].pos_and_flag;

	vec3 a_local = ea.xyz * inst.scale.xyz;
	vec3 b_local = eb.xyz * inst.scale.xyz;
	vec3 a_world = vec3( dot( inst.xform_row0, vec4( a_local, 1.0 ) ), dot( inst.xform_row1, vec4( a_local, 1.0 ) ),
						 dot( inst.xform_row2, vec4( a_local, 1.0 ) ) );
	vec3 b_world = vec3( dot( inst.xform_row0, vec4( b_local, 1.0 ) ), dot( inst.xform_row1, vec4( b_local, 1.0 ) ),
						 dot( inst.xform_row2, vec4( b_local, 1.0 ) ) );

	vec4 clip_a = view_proj * vec4( a_world, 1.0 );
	vec4 clip_b = view_proj * vec4( b_world, 1.0 );
	vec4 clip_end = is_b ? clip_b : clip_a;

	vec2 ndc_a = clip_a.xy / clip_a.w;
	vec2 ndc_b = clip_b.xy / clip_b.w;
	vec2 sa = ( ndc_a * 0.5 + 0.5 ) * viewport.xy;
	vec2 sb = ( ndc_b * 0.5 + 0.5 ) * viewport.xy;

	vec2 dir_px = sb - sa;
	float len_px = length( dir_px );
	vec2 dir_n = ( len_px > 1e-6 ) ? ( dir_px / len_px ) : vec2( 1.0, 0.0 );
	vec2 perp_n = vec2( -dir_n.y, dir_n.x );

	float half_width_px = max( edge_params.x * 0.5, 0.5 );

	const float AA_PAD_PX = 1.5;
	float quad_half_px = half_width_px + AA_PAD_PX;

	vec2 offset_px = perp_n * side * quad_half_px;
	vec2 offset_ndc = offset_px * ( 2.0 * viewport.zw );
	vec4 clip_pos = clip_end;
	clip_pos.xy += offset_ndc * clip_end.w;

	clip_pos.z += edge_params.y * clip_pos.w;

	gl_Position = clip_pos;
	v_dist_from_axis_px = side * quad_half_px;
	v_half_width_px = half_width_px;
	v_flag = ea.w; 
}
#pragma sokol @end

#pragma sokol @fs fs

layout( binding = 1 ) uniform ub_pass
{
	vec4 convex_color;	
	vec4 concave_color; 
	vec4 flat_color;	
};

noperspective in float v_dist_from_axis_px;
flat in float v_half_width_px;
flat in float v_flag;

out vec4 out_color;

void main()
{

	float d = abs( v_dist_from_axis_px );
	float aa = fwidth( v_dist_from_axis_px );
	float coverage = 1.0 - smoothstep( v_half_width_px - aa, v_half_width_px + aa, d );

	vec4 col;
	if ( v_flag > 1.5 )
	{
		col = flat_color;
	}
	else if ( v_flag > 0.5 )
	{
		col = convex_color;
	}
	else
	{
		col = concave_color;
	}
	out_color = col * coverage;
}
#pragma sokol @end

#pragma sokol @program edge_overlay vs fs
