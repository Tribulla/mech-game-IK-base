#pragma sokol @module sky_to_cube

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

#pragma sokol @include ../common/preetham.glsl

layout(binding = 0)uniform ub_face
{

	vec4 face_right; 
	vec4 face_up; 
	vec4 face_forward; 
	vec4 sun_dir_world; 
};

in vec2 v_uv;
out vec4 out_color;

void main()
{
	vec2 uv = v_uv * 2.0 - 1.0; 
	vec3 dir = normalize(face_forward.xyz + uv.x * face_right.xyz + uv.y * face_up.xyz);

	vec3 rgb = preethamSkyScaled(dir, sun_dir_world.xyz, sun_dir_world.w, face_forward.w, face_up.w);
	out_color = vec4(rgb, 1.0);
}
#pragma sokol @end

#pragma sokol @program build vs fs
