#pragma sokol @module shadow_geom

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
	vec4 scale;		 
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

	vec3 scaled_pos = in_pos * inst.scale.xyz;
	vec4 local_pos = vec4( scaled_pos, 1.0 );
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
