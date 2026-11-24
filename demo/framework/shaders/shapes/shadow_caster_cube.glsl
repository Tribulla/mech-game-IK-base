#pragma sokol @module shadow_cube

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype vec3 Vec3
#pragma sokol @ctype vec2 Vec2

#pragma sokol @vs vs

layout( binding = 0 ) uniform ub_frame
{
	mat4 light_view_proj;
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

void main()
{
	cube_instance inst = instances[inst_base.x + gl_InstanceIndex];
	vec4 local_pos = vec4( in_pos, 1.0 );
	vec3 world_pos =
		vec3( dot( inst.xform_row0, local_pos ), dot( inst.xform_row1, local_pos ), dot( inst.xform_row2, local_pos ) );
	gl_Position = light_view_proj * vec4( world_pos, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs

void main()
{
}
#pragma sokol @end

#pragma sokol @program caster vs fs
