const float PBR_PI = 3.14159265358979323846;

float pbrDistributionGGX(float n_dot_h, float alpha)
{
	float a2 = alpha * alpha;
	float d = (n_dot_h * n_dot_h) * (a2 - 1.0) + 1.0;
	return a2 / (PBR_PI * d * d);
}

float pbrVisibilitySmithCorrelated(float n_dot_v, float n_dot_l, float alpha)
{
	float a2 = alpha * alpha;
	float vv = n_dot_l * sqrt(n_dot_v * n_dot_v * (1.0 - a2) + a2);
	float vl = n_dot_v * sqrt(n_dot_l * n_dot_l * (1.0 - a2) + a2);
	return 0.5 / (vv + vl + 1e-5);
}

vec3 pbrFresnelSchlick(float v_dot_h, vec3 f0)
{
	float t = 1.0 - v_dot_h;
	float t2 = t * t;
	return f0 + (vec3(1.0) - f0) * (t2 * t2 * t);
}

vec3 brdf_evaluate(vec3 N, vec3 V, vec3 L, vec3 sun_radiance,
	vec3 base_color, float metallic, float roughness)
{
	float n_dot_l = max(dot(N, L), 0.0);
	float n_dot_v = max(dot(N, V), 1e-4);

	vec3 H = normalize(V + L);
	float n_dot_h = max(dot(N, H), 0.0);
	float v_dot_h = max(dot(V, H), 0.0);

	float perc = max(roughness, 0.045);
	float alpha = perc * perc;

	vec3 f0 = mix(vec3(0.04), base_color, metallic);

	float D = pbrDistributionGGX(n_dot_h, alpha);
	float Vis = pbrVisibilitySmithCorrelated(n_dot_v, n_dot_l, alpha);
	vec3 F = pbrFresnelSchlick(v_dot_h, f0);

	vec3 specular = D * Vis * F;
	vec3 diffuse = (vec3(1.0) - F) * (1.0 - metallic) * base_color / PBR_PI;

	return (diffuse + specular) * sun_radiance * n_dot_l;
}

vec3 pbrEvaluateShIrradiance(vec3 N, vec4 sh[9])
{
	const float Y00 = 0.282094791773878;
	const float Y1n1 = 0.488602511902919;
	const float Y10 = 0.488602511902919;
	const float Y1p1 = 0.488602511902919;
	const float Y2n2 = 1.092548430592079;
	const float Y2n1 = 1.092548430592079;
	const float Y20 = 0.315391565252520;
	const float Y2p1 = 1.092548430592079;
	const float Y2p2 = 0.546274215296039;

	float b0 = Y00;
	float b1 = -Y1n1 * N.y;
	float b2 = Y10 * N.z;
	float b3 = -Y1p1 * N.x;
	float b4 = Y2n2 * N.x * N.y;
	float b5 = -Y2n1 * N.y * N.z;
	float b6 = Y20 * (3.0 * N.z * N.z - 1.0);
	float b7 = -Y2p1 * N.x * N.z;
	float b8 = Y2p2 * (N.x * N.x - N.y * N.y);

	return sh[0].xyz * b0
	+ sh[1].xyz * b1 + sh[2].xyz * b2 + sh[3].xyz * b3
	+ sh[4].xyz * b4 + sh[5].xyz * b5 + sh[6].xyz * b6
	+ sh[7].xyz * b7 + sh[8].xyz * b8;
}

vec3 pbrEvaluateIbl(vec3 N_world, vec3 V_world, vec3 base_color, float metallic,
	float roughness, vec4 sh[9],
	textureCube tex_spec_cube, sampler smp_spec_cube,
	texture2D tex_brdf_lut, sampler smp_brdf_lut,
	float max_lod)
{

	float n_dot_v = clamp(dot(N_world, V_world), 1e-4, 1.0);
	vec3 R = reflect(-V_world, N_world);

	vec3 F0 = mix(vec3(0.04), base_color, metallic);
	vec3 F0m = max(vec3(1.0 - roughness), F0);

	vec3 F = F0 + (F0m - F0) * pow(max(0.0, 1.0 - n_dot_v), 5.0);

	vec3 kS = F;
	vec3 kD = (vec3(1.0) - kS) * (1.0 - metallic);

	vec3 irradiance = max(pbrEvaluateShIrradiance(N_world, sh), vec3(0.0));
	vec3 diffuse = kD * base_color * irradiance / PBR_PI;

	float lod = roughness * max_lod;
	vec3 prefiltered = textureLod(samplerCube(tex_spec_cube, smp_spec_cube), R, lod).rgb;
	vec2 brdf = texture(sampler2D(tex_brdf_lut, smp_brdf_lut),
	vec2(n_dot_v, roughness)).rg;
	vec3 specular = prefiltered * (F0 * brdf.x + brdf.y);

	return diffuse + specular;
}

vec3 pbrEvaluateAmbient(bool iblEnabled,
	vec3 N_world, vec3 V_world,
	vec3 base_color, float metallic, float roughness,
	float flat_strength,
	vec4 sh[9],
	textureCube tex_spec_cube, sampler smp_spec_cube,
	texture2D tex_brdf_lut, sampler smp_brdf_lut,
float max_lod)
{

	vec3 result = base_color * flat_strength;
	if (iblEnabled) {
		result = pbrEvaluateIbl(N_world, V_world,
			base_color, metallic, roughness,
			sh,
			tex_spec_cube, smp_spec_cube,
			tex_brdf_lut, smp_brdf_lut,
		max_lod);
	}
	return result;
}

vec3 proceduralGrid(vec2 line_xz, vec2 axis_xz, vec3 base_color, float cell_size)
{
	vec2 coord_minor = line_xz / cell_size;
	vec2 d_minor = max(fwidth(coord_minor), vec2(1.0e-6));
	vec2 g_minor = abs(fract(coord_minor - 0.5) - 0.5) / d_minor;
	float line_minor = 1.0 - clamp(min(g_minor.x, g_minor.y), 0.0, 1.0);

	vec2 coord_major = line_xz / (cell_size * 10.0);
	vec2 d_major = max(fwidth(coord_major), vec2(1.0e-6));
	vec2 g_major = abs(fract(coord_major - 0.5) - 0.5) / d_major;
	float line_major = 1.0 - clamp(min(g_major.x, g_major.y), 0.0, 1.0);

	vec3 minor_color = base_color * 0.70;
	vec3 major_color = base_color * 0.35;

	vec3 result = mix(base_color, minor_color, line_minor);
	result = mix(result, major_color, line_major);

	vec2 d_axis = max(fwidth(axis_xz), vec2(1.0e-6));
	float axis_x = (1.0 - clamp(abs(axis_xz.y) / d_axis.y, 0.0, 1.0)) * step(0.0, axis_xz.x);
	float axis_z = (1.0 - clamp(abs(axis_xz.x) / d_axis.x, 0.0, 1.0)) * step(0.0, axis_xz.y);
	result = mix(result, vec3(1.0, 0.0, 0.0), axis_x);
	result = mix(result, vec3(0.0, 0.0, 1.0), axis_z);
	return result;
}
