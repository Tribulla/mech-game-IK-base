const float PREETHAM_PI = 3.14159265358979323846;

const float PREETHAM_LUMINANCE_SCALE = 0.06;

float preethamPerez(float cos_theta, float cos_gamma, float gamma,
	float A, float B, float C, float D, float E)
{
	float term1 = 1.0 + A * exp(B / max(cos_theta, 0.01));
	float term2 = 1.0 + C * exp(D * gamma) + E * cos_gamma * cos_gamma;
	return term1 * term2;
}

vec3 preethamXyY_to_XYZ(float x, float y, float Y)
{
	float yy = max(y, 1.0e-5);
	return vec3(Y * x / yy, Y, Y * (1.0 - x - y) / yy);
}

vec3 preethamXYZ_to_linear_srgb(vec3 c)
{
	return vec3(
		dot(c, vec3(3.2404542, - 1.5371385, - 0.4985314)),
		dot(c, vec3(-0.9692660, 1.8760108, 0.0415560)),
		dot(c, vec3(0.0556434, - 0.2040259, 1.0572252))
	);
}

vec3 preethamSky(vec3 view_dir, vec3 sun_dir, float turbidity)
{

	float sun_y = clamp(sun_dir.y, 0.0, 1.0);

	vec3 view_clamped = normalize(vec3(view_dir.x, max(view_dir.y, 0.0) + 0.01, view_dir.z));

	float cos_theta = max(view_clamped.y, 0.0); 
	float cos_gamma = clamp(dot(sun_dir, view_clamped), - 1.0, 1.0); 
	float gamma = acos(cos_gamma);
	float sun_theta = acos(sun_y);

	float T = max(turbidity, 1.0);
	float T2 = T * T;

	float A_Y = 0.1787 * T - 1.4630;
	float B_Y = -0.3554 * T + 0.4275;
	float C_Y = -0.0227 * T + 5.3251;
	float D_Y = 0.1206 * T - 2.5771;
	float E_Y = -0.0670 * T + 0.3703;

	float A_x = -0.0193 * T - 0.2592;
	float B_x = -0.0665 * T + 0.0008;
	float C_x = -0.0004 * T + 0.2125;
	float D_x = -0.0641 * T - 0.8989;
	float E_x = -0.0033 * T + 0.0452;

	float A_y = -0.0167 * T - 0.2608;
	float B_y = -0.0950 * T + 0.0092;
	float C_y = -0.0079 * T + 0.2102;
	float D_y = -0.0441 * T - 1.6537;
	float E_y = -0.0109 * T + 0.0529;

	float ts = sun_theta;
	float ts2 = ts * ts;
	float ts3 = ts2 * ts;

	float x_z =
	(0.00166 * ts3 - 0.00375 * ts2 + 0.00209 * ts) * T2 +
	(-0.02903 * ts3 + 0.06377 * ts2 - 0.03202 * ts + 0.00394) * T +
	(0.11693 * ts3 - 0.21196 * ts2 + 0.06052 * ts + 0.25886);

	float y_z =
	(0.00275 * ts3 - 0.00610 * ts2 + 0.00317 * ts) * T2 +
	(-0.04214 * ts3 + 0.08970 * ts2 - 0.04153 * ts + 0.00516) * T +
	(0.15346 * ts3 - 0.26756 * ts2 + 0.06670 * ts + 0.26688);

	float chi = (4.0 / 9.0 - T / 120.0) * (PREETHAM_PI - 2.0 * sun_theta);
	float Y_z = (4.0453 * T - 4.9710) * tan(chi) - 0.2155 * T + 2.4192;

	float cos_zen_gamma = sun_y; 
	float pY_v = preethamPerez(cos_theta, cos_gamma, gamma, A_Y, B_Y, C_Y, D_Y, E_Y);
	float px_v = preethamPerez(cos_theta, cos_gamma, gamma, A_x, B_x, C_x, D_x, E_x);
	float py_v = preethamPerez(cos_theta, cos_gamma, gamma, A_y, B_y, C_y, D_y, E_y);
	float pY_z = preethamPerez(1.0, cos_zen_gamma, sun_theta, A_Y, B_Y, C_Y, D_Y, E_Y);
	float px_z = preethamPerez(1.0, cos_zen_gamma, sun_theta, A_x, B_x, C_x, D_x, E_x);
	float py_z = preethamPerez(1.0, cos_zen_gamma, sun_theta, A_y, B_y, C_y, D_y, E_y);

	float Y = max(Y_z * pY_v / max(pY_z, 1.0e-5), 0.0);
	float x = x_z * px_v / max(px_z, 1.0e-5);
	float y = y_z * py_v / max(py_z, 1.0e-5);

	vec3 XYZ = preethamXyY_to_XYZ(x, y, Y);
	vec3 rgb = preethamXYZ_to_linear_srgb(XYZ);
	return max(rgb, vec3(0.0));
}

vec3 preethamToYUp(vec3 v, float zUp)
{
	return mix(v, vec3(v.x, v.z, - v.y), zUp);
}

vec3 preethamSkyScaled(vec3 view_dir, vec3 sun_dir, float turbidity, float fade, float zUp)
{
	vec3 v = preethamToYUp(view_dir, zUp);
	vec3 s = preethamToYUp(sun_dir, zUp);
	return preethamSky(v, s, turbidity) * PREETHAM_LUMINANCE_SCALE * fade;
}
