#pragma sokol @module overlay_line

#pragma sokol @ctype mat4 Mat4
#pragma sokol @ctype vec4 Vec4

#pragma sokol @vs vs

layout(binding = 0) uniform ub_frame
{
	mat4 view;
	mat4 proj;
	mat4 view_proj;
	mat4 inv_view_proj;
	vec4 camera_pos; 
	vec4 viewport; 
};

struct line_instance
{
	vec4 a; 
	vec4 b; 
	vec4 color; 
	uvec4 flags; 

};

layout(binding = 0) readonly buffer line_instances
{
	line_instance instances[];
};

out vec2 v_screen_a; 
out vec2 v_screen_b; 

noperspective out float v_dist_from_axis_px; 
out float v_half_width_px; 

out float v_view_z; 
flat out vec4 v_color;
flat out uint v_occ_mode;

void main()
{
	line_instance inst = instances[gl_InstanceIndex];

	int index = gl_VertexIndex;
	bool is_b = (index == 1)||(index == 2)||(index == 4);
	float side = ((index == 2)||(index == 4)||(index == 5)) ? 1.0 : -1.0;

	vec3 world_a = inst.a.xyz;
	vec3 world_b = inst.b.xyz;
	vec4 clip_a = view_proj * vec4(world_a, 1.0);
	vec4 clip_b = view_proj * vec4(world_b, 1.0);
	vec4 view_a = view * vec4(world_a, 1.0);
	vec4 view_b = view * vec4(world_b, 1.0);

	vec4 clip_end = is_b ? clip_b : clip_a;
	float view_z_end = -(is_b ? view_b.z : view_a.z);

	vec2 ndc_a = clip_a.xy / clip_a.w;
	vec2 ndc_b = clip_b.xy / clip_b.w;
	v_screen_a = (ndc_a * 0.5 + 0.5) * viewport.xy;
	v_screen_b = (ndc_b * 0.5 + 0.5) * viewport.xy;

	vec2 dir_px = v_screen_b - v_screen_a;
	float len_px = length(dir_px);
	vec2 dir_n = (len_px > 1e-6) ? (dir_px / len_px) : vec2(1.0, 0.0);
	vec2 perp_n = vec2(-dir_n.y, dir_n.x);

	float thickness = inst.a.w;
	float half_width_px;
	if (inst.flags.x == 0u)
	{
		half_width_px = thickness * 0.5;
	}
	else
	{
		float pixels_per_world = viewport.y * proj[1][1] / (2.0 * view_z_end);
		half_width_px = thickness * 0.5 * pixels_per_world;
	}

	half_width_px = max(half_width_px, 0.5);

	const float AA_PAD_PX = 1.5;
	float quad_half_px = half_width_px + AA_PAD_PX;

	vec2 offset_px = perp_n * side * quad_half_px;
	vec2 offset_ndc = offset_px * (2.0 * viewport.zw);
	vec4 clip_pos = clip_end;
	clip_pos.xy += offset_ndc * clip_end.w;

	gl_Position = clip_pos;
	v_dist_from_axis_px = side * quad_half_px;
	v_half_width_px = half_width_px;
	v_view_z = view_z_end;
	v_color = inst.color;
	v_occ_mode = inst.flags.y;
}
#pragma sokol @end

#pragma sokol @fs fs

#pragma sokol @image_sample_type tex_depth float
layout(binding = 1)uniform texture2D tex_depth;

#pragma sokol @sampler_type smp_depth filtering
layout(binding = 0)uniform sampler smp_depth;

in vec2 v_screen_a;
in vec2 v_screen_b;
noperspective in float v_dist_from_axis_px;
in float v_half_width_px;
in float v_view_z;
flat in vec4 v_color;
flat in uint v_occ_mode;
out vec4 out_color;

const uint OCC_HIDE = 0u;
const uint OCC_DIM = 1u;
const uint OCC_DASHED = 2u;

void main() 
{

	float d = abs(v_dist_from_axis_px);
	float aa = fwidth(v_dist_from_axis_px);
	float coverage = 1.0 - smoothstep(v_half_width_px - aa, v_half_width_px + aa, d);

	vec2 depth_uv = gl_FragCoord.xy / vec2(textureSize(sampler2D(tex_depth, smp_depth), 0));
	float scene_z = textureLod(sampler2D(tex_depth, smp_depth), depth_uv, 0.0).r;

	const float occlusion_bias_m = 0.001; 
	const float occlusion_slope_k = 2.0; 
	float occlusion_bias = occlusion_bias_m + occlusion_slope_k * fwidth(v_view_z);
	bool occluded = v_view_z > scene_z + occlusion_bias;

	vec4 col = v_color;

	if (v_occ_mode == OCC_HIDE && occluded)
	{
		col = vec4(0.0);
	}
	else if (v_occ_mode == OCC_DIM && occluded)
	{
		col *= 0.25;
	}
	else if (v_occ_mode == OCC_DASHED && occluded)
	{

		vec2 line_vec = v_screen_b - v_screen_a;
		float line_len = max(length(line_vec), 1e-6);
		vec2 line_dir = line_vec / line_len;
		vec2 rel_px = gl_FragCoord.xy - v_screen_a;
		float along_px = dot(rel_px, line_dir);

		const float DASH_PERIOD_PX = 16.0;
		const float DASH_HALF = 4.0; 
		float phase = mod(along_px, DASH_PERIOD_PX);
		float dash_sd = abs(phase - DASH_HALF) - DASH_HALF;
		float dash_aa = fwidth(along_px);
		float dash_msk = 1.0 - smoothstep(-dash_aa, dash_aa, dash_sd);
		col *= dash_msk;
	}

	out_color = col * coverage;
}
#pragma sokol @end

#pragma sokol @program prog vs fs
