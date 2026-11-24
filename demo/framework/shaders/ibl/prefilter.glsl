#pragma sokol @module prefilter

#pragma sokol @ctype vec4 Vec4

#pragma sokol @vs vs

out vec2 v_uv;

void main()
{
	vec2 ndc = vec2(float((gl_VertexIndex << 1) & 2),
	float(gl_VertexIndex & 2)) * 2.0 - 1.0;
	v_uv = ndc * 0.5 + 0.5;
	gl_Position = vec4(ndc, 0.0, 1.0);
}
#pragma sokol @end

#pragma sokol @fs fs

layout(binding = 0) uniform ub_pass
{
	vec4 face_right; 
	vec4 face_up; 
	vec4 face_forward; 
	vec4 params; 
};

#pragma sokol @image_sample_type tex_sky float
layout(binding = 0)uniform textureCube tex_sky;
#pragma sokol @sampler_type smp_sky filtering
layout(binding = 0)uniform sampler smp_sky;

in vec2 v_uv;
out vec4 out_color;

const float PREFILTER_PI = 3.14159265358979323846;
const uint NUM_SAMPLES = 1024u;

float radicalInverseVdC(uint bits)
{
	bits = (bits << 16u)|(bits >> 16u);
	bits = ((bits & 0x55555555u) << 1u)|((bits & 0xAAAAAAAAu) >> 1u);
	bits = ((bits & 0x33333333u) << 2u)|((bits & 0xCCCCCCCCu) >> 2u);
	bits = ((bits & 0x0F0F0F0Fu) << 4u)|((bits & 0xF0F0F0F0u) >> 4u);
	bits = ((bits & 0x00FF00FFu) << 8u)|((bits & 0xFF00FF00u) >> 8u);
	return float(bits) * 2.3283064365386963e-10;
}

vec2 hammersley(uint i, uint n)
{
	return vec2(float(i) / float(n), radicalInverseVdC(i));
}

vec3 importanceSampleGGX(vec2 Xi, vec3 N, float roughness)
{
	float a = roughness * roughness;
	float phi = 2.0 * PREFILTER_PI * Xi.x;
	float cos_theta = sqrt((1.0 - Xi.y) / (1.0 + (a * a - 1.0) * Xi.y));
	float sin_theta = sqrt(max(1.0 - cos_theta * cos_theta, 0.0));

	vec3 H_tan = vec3(sin_theta * cos(phi), sin_theta * sin(phi), cos_theta);

	vec3 up = abs(N.z) < 0.999 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
	vec3 tangent = normalize(cross(up, N));
	vec3 bitan = cross(N, tangent);

	return normalize(tangent * H_tan.x + bitan * H_tan.y + N * H_tan.z);
}

void main()
{
	vec2 uv = v_uv * 2.0 - 1.0;
	vec3 N = normalize(face_forward.xyz + uv.x * face_right.xyz + uv.y * face_up.xyz);
	vec3 V = N; 

	float roughness = params.x;
	if (roughness < 0.001)
	{

		out_color = vec4(texture(samplerCube(tex_sky, smp_sky), N).rgb, 1.0);
		return;
	}

	vec3 prefiltered = vec3(0.0);
	float total_w = 0.0;

	for(uint i = 0u; i < NUM_SAMPLES; ++i)
	{
		vec2 Xi = hammersley(i, NUM_SAMPLES);
		vec3 H = importanceSampleGGX(Xi, N, roughness);
		vec3 L = normalize(2.0 * dot(V, H) * H - V);
		float n_dot_l = max(dot(N, L), 0.0);
		if (n_dot_l > 0.0)
		{
			prefiltered += texture(samplerCube(tex_sky, smp_sky), L).rgb * n_dot_l;
			total_w += n_dot_l;
		}
	}
	out_color = vec4(prefiltered / max(total_w, 1.0e-4), 1.0);
}
#pragma sokol @end

#pragma sokol @program build vs fs
