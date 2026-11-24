#pragma sokol @module depth_only_geom

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype vec3 Vec3
#pragma sokol @ctype vec2 Vec2

#pragma sokol @vs vs

layout(binding = 0) uniform ub_frame
{
	mat4 view_proj;
	mat4 view;
};

layout(binding = 2) uniform ub_draw
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

layout(binding = 0) readonly buffer instances
{
	instance items[];
};

in vec3 in_pos;

out float v_linear_view_z;

void main()
{
	instance inst = items[inst_base.x + gl_InstanceIndex];

	vec3 scaled_pos = in_pos * inst.scale.xyz;
	vec4 local_pos = vec4(scaled_pos, 1.0);
	vec3 world_pos = vec3(
		dot(inst.xform_row0, local_pos),
		dot(inst.xform_row1, local_pos),
		dot(inst.xform_row2, local_pos)
	);

	vec4 view_h = view * vec4(world_pos, 1.0);
	v_linear_view_z = -view_h.z;
	gl_Position = view_proj * vec4(world_pos, 1.0);
}

#pragma sokol @end

#pragma sokol @fs fs

in float v_linear_view_z;
out vec4 out_linear_depth;

void main()
{
	out_linear_depth = vec4(v_linear_view_z, 0.0, 0.0, 0.0);
}

#pragma sokol @end

#pragma sokol @program prog vs fs
