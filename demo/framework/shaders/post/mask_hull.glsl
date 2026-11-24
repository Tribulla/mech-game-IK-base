#pragma sokol @module mask_hull

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype vec3 Vec3

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
	vec4 scale_kind; 
};

layout( binding = 0 ) readonly buffer instances
{
	instance items[];
};

in vec3 in_pos;

flat out float v_kind;

void main()
{
	instance inst = items[inst_base.x + gl_InstanceIndex];

	vec3 scaled_pos = in_pos * inst.scale_kind.xyz;
	vec4 local_pos = vec4( scaled_pos, 1.0 );
	vec3 world_pos =
		vec3( dot( inst.xform_row0, local_pos ), dot( inst.xform_row1, local_pos ), dot( inst.xform_row2, local_pos ) );

	v_kind = inst.scale_kind.w;
	gl_Position = view_proj * vec4( world_pos, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs

flat in float v_kind;

layout( location = 0 ) out vec4 out_kind;

void main()
{

	out_kind = vec4( v_kind * 0.5, 0.0, 0.0, 1.0 );
}
#pragma sokol @end

#pragma sokol @program prog vs fs
