#pragma sokol @module brdf_lut

#pragma sokol @ctype vec4 Vec4

#pragma sokol @vs vs

layout(binding = 0)uniform ub_brdf
{

	vec4 params; 
};

out vec2 v_uv;

void main()
{
	vec2 ndc = vec2(float((gl_VertexIndex << 1)& 2),
	float(gl_VertexIndex & 2)) * 2.0 - 1.0;
	v_uv = vec2(ndc.x * 0.5 + 0.5, 0.5 + ndc.y * 0.5 * params.x);
	gl_Position = vec4(ndc, 0.0, 1.0);
}
#pragma sokol @end

#pragma sokol @fs fs

in vec2 v_uv;
out vec2 out_color;

const float BRDF_LUT_PI = 3.14159265358979323846;
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

vec3 importanceSampleGGX(vec2 Xi, float alpha)
{
	float a2 = alpha * alpha;
	float phi = 2.0 * BRDF_LUT_PI * Xi.x;
	float cos_theta = sqrt((1.0 - Xi.y) / (1.0 + (a2 - 1.0) * Xi.y));
	float sin_theta = sqrt(max(1.0 - cos_theta * cos_theta, 0.0));
	return vec3(sin_theta * cos(phi), sin_theta * sin(phi), cos_theta);
}

float smithGggxIbl(float n_dot_v, float n_dot_l, float alpha)
{
	float k = (alpha * alpha) * 0.5;
	float gv = n_dot_v / (n_dot_v * (1.0 - k) + k);
	float gl = n_dot_l / (n_dot_l * (1.0 - k) + k);
	return gv * gl;
}

vec2 integrateBRDF(float n_dot_v, float roughness)
{

	vec3 V = vec3(sqrt(max(1.0 - n_dot_v * n_dot_v, 0.0)), 0.0, n_dot_v);

	float A = 0.0;
	float B = 0.0;
	float alpha = roughness * roughness;

	for (uint i = 0u; i < NUM_SAMPLES; ++i)
	{
		vec2 Xi = hammersley(i, NUM_SAMPLES);
		vec3 H = importanceSampleGGX(Xi, alpha);
		vec3 L = 2.0 * dot(V, H) * H - V;

		float n_dot_l = max(L.z, 0.0);
		float n_dot_h = max(H.z, 0.0);
		float v_dot_h = max(dot(V, H), 0.0);

		if (n_dot_l > 0.0)
		{
			float G = smithGggxIbl(n_dot_v, n_dot_l, alpha);

			float G_Vis = (G * v_dot_h) / max(n_dot_h * n_dot_v, 1.0e-5);

			float Fc = pow(max(0.0, 1.0 - v_dot_h), 5.0);

			A += (1.0 - Fc) * G_Vis;
			B += Fc * G_Vis;
		}
	}

	return vec2(A, B) / float(NUM_SAMPLES);
}

void main()
{

	float n_dot_v = max(v_uv.x, 1.0e-3);
	float roughness = max(v_uv.y, 1.0e-3);
	out_color = integrateBRDF(n_dot_v, roughness);
}
#pragma sokol @end

#pragma sokol @program build vs fs
