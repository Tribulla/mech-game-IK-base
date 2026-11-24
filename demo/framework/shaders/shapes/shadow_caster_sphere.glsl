#pragma sokol @module shadow_sphere

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

struct instance
{
	vec4 xform_row0; 
	vec4 xform_row1; 
	vec4 xform_row2; 
	vec4 base_color; 
	vec4 params;	 
	vec4 material;	 
};

layout( binding = 0 ) readonly buffer instances
{
	instance items[];
};

in vec3 in_pos; 

void main()
{
	instance inst = items[inst_base.x + gl_InstanceIndex];
	vec3 center = vec3( inst.xform_row0.w, inst.xform_row1.w, inst.xform_row2.w );
	float radius = inst.params.x;
	vec3 world_pos = center + radius * in_pos;
	gl_Position = light_view_proj * vec4( world_pos, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs
void main()
{
}
#pragma sokol @end

#pragma sokol @program caster vs fs
