#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4
#pragma sokol @ctype vec3 Vec3

#pragma sokol @vs vs

layout(binding = 0) uniform sky_ub_frame
{
	mat4 inv_view_proj;
};

out vec4 v_world_homog; 

void main()
{

	vec2 ndc = vec2(float((gl_VertexIndex << 1) & 2), float(gl_VertexIndex & 2)) * 2.0 - 1.0;

	vec4 clip = vec4(ndc, 0.0, 1.0);
	gl_Position = clip;

	v_world_homog = inv_view_proj * clip;
}
#pragma sokol @end

#pragma sokol @fs fs

#pragma sokol @include ../common/preetham.glsl

layout(binding = 1)uniform sky_ub_pass
{
	vec4 sun_dir_world; 
	vec4 sky_params; 
	vec4 camera_pos; 
};

in vec4 v_world_homog;
out vec4 out_color;

void main()
{

	vec3 world_pos = v_world_homog.xyz / v_world_homog.w;
	vec3 view_dir = normalize(world_pos - camera_pos.xyz);

	vec3 sun_dir = normalize(sun_dir_world.xyz);

	vec3 rgb = preethamSkyScaled(view_dir, sun_dir, sky_params.x, sun_dir_world.w, sky_params.y);

	out_color = vec4(rgb, 1.0);
}
#pragma sokol @end

#pragma sokol @program sky vs fs
