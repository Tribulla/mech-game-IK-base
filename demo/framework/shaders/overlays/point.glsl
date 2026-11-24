#pragma sokol @module overlay_point

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4

#pragma sokol @vs vs

layout(binding = 0)uniform ub_frame
{
	mat4 view;
	mat4 proj;
	mat4 view_proj;
	mat4 inv_view_proj;
	vec4 camera_pos; 
	vec4 viewport; 
};

struct point_instance
{
	vec4 p; 
	vec4 color; 
	uvec4 flags; 

};

layout(binding = 0) readonly buffer point_instances
{
	point_instance instances[];
};

out vec2 v_offset_from_center_px; 
out float v_half_size_px; 
out float v_view_z; 
flat out vec4 v_color;
flat out uint v_occ_mode;

void main()
{
	point_instance inst = instances[gl_InstanceIndex];

	int index = gl_VertexIndex;
	float corner_x = ((index == 1)||(index == 2)||(index == 4)) ? 1.0 : -1.0;
	float corner_y = ((index == 2)||(index == 4)||(index == 5)) ? 1.0 : -1.0;

	vec3 world_p = inst.p.xyz;
	vec4 clip_c = view_proj * vec4(world_p, 1.0);
	vec4 view_c = view * vec4(world_p, 1.0);
	float view_z = -view_c.z;

	float requested = inst.p.w;
	float half_size_px;
	if (inst.flags.x == 0u)
	{
		half_size_px = requested * 0.5;
	}
	else
	{
		float pixels_per_world = viewport.y * proj[1][1] / (2.0 * view_z);
		half_size_px = requested * 0.5 * pixels_per_world;
	}
	half_size_px = max(half_size_px, 0.5);

	const float AA_PAD_PX = 1.5;
	float quad_half_px = half_size_px + AA_PAD_PX;

	vec2 offset_px = vec2(corner_x, corner_y) * quad_half_px;
	vec2 offset_ndc = offset_px * (2.0 * viewport.zw);

	vec4 clip_pos = clip_c;
	clip_pos.xy += offset_ndc * clip_c.w;

	gl_Position = clip_pos;
	v_offset_from_center_px = offset_px;
	v_half_size_px = half_size_px;
	v_view_z = view_z;
	v_color = inst.color;
	v_occ_mode = inst.flags.y;
}
#pragma sokol @end

#pragma sokol @fs fs

#pragma sokol @image_sample_type tex_depth float
layout(binding = 1)uniform texture2D tex_depth;

#pragma sokol @sampler_type smp_depth filtering
layout(binding = 0)uniform sampler smp_depth;

in vec2 v_offset_from_center_px;
in float v_half_size_px;
in float v_view_z;
flat in vec4 v_color;
flat in uint v_occ_mode;
out vec4 out_color;

const uint OCC_HIDE = 0u;

void main()
{

	float d = length(v_offset_from_center_px);
	float aa = fwidth(d);
	float coverage = 1.0 - smoothstep(v_half_size_px - aa, v_half_size_px + aa, d);

	vec2 depth_uv = gl_FragCoord.xy / vec2(textureSize(sampler2D(tex_depth, smp_depth), 0));
	float scene_z = textureLod(sampler2D(tex_depth, smp_depth), depth_uv, 0.0).r;

	const float occlusion_bias_m = 0.001; 
	const float occlusion_slope_k = 2.0;
	float occlusion_bias = occlusion_bias_m + occlusion_slope_k * fwidth(v_view_z);
	bool occluded = v_view_z > scene_z + occlusion_bias;

	vec4 col = v_color;

	if (occluded)
	{
		if (v_occ_mode == OCC_HIDE)
		{
			col = vec4(0.0);
		} else {

			col *= 0.25;
		}
	}

	out_color = col * coverage;
}
#pragma sokol @end

#pragma sokol @program prog vs fs
