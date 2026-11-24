#pragma sokol @module tonemap

#pragma sokol @ctype vec4 Vec4

#pragma sokol @vs vs

out vec2 v_uv;

void main()
{
	vec2 ndc = vec2( float( ( gl_VertexIndex << 1 ) & 2 ), float( gl_VertexIndex & 2 ) ) * 2.0 - 1.0;
	v_uv = ndc * 0.5 + 0.5;
	gl_Position = vec4( ndc, 0.0, 1.0 );
}
#pragma sokol @end

#pragma sokol @fs fs

layout( binding = 0 ) uniform ub_tonemap
{

	vec4 params;
};

layout( binding = 0 ) uniform texture2D tex_scene;
layout( binding = 0 ) uniform sampler smp_scene;

in vec2 v_uv;
out vec4 out_color;

vec3 agxDefaultContrastApprox( vec3 x )
{
	vec3 x2 = x * x;
	vec3 x4 = x2 * x2;
	return +15.5 * x4 * x2 - 40.14 * x4 * x + 31.96 * x4 - 6.868 * x2 * x + 0.4298 * x2 + 0.1191 * x - 0.00232;
}

vec3 agx( vec3 v )
{

	const mat3 inset = mat3( 0.842479062253094, 0.0423282422610123, 0.0423756549057051, 0.0784335999999992, 0.878468636469772,
							 0.0784336, 0.0792237451477643, 0.0791661274605434, 0.879142973793104 );
	const float MIN_EV = -12.47393;
	const float MAX_EV = 4.026069;

	v = inset * v;

	v = max( v, vec3( 1.0e-10 ) );
	v = clamp( log2( v ), MIN_EV, MAX_EV );
	v = ( v - MIN_EV ) / ( MAX_EV - MIN_EV );
	v = agxDefaultContrastApprox( v );
	return v;
}

vec3 agxOutset( vec3 v )
{

	const mat3 outset = mat3( 1.19687900512017, -0.0528968517574562, -0.0529716355144438, -0.0980208811401368, 1.15190312990417,
							  -0.0980434501171241, -0.0990297440797205, -0.0989611768448433, 1.15107367264116 );
	return outset * v;
}

vec3 agxLook( vec3 v, float saturation )
{
	const vec3 LUMA = vec3( 0.2126, 0.7152, 0.0722 );
	float luma = dot( v, LUMA );
	return luma + saturation * ( v - luma );
}

void main()
{

	float sampleY = ( params.y > 0.0 ) ? v_uv.y : ( 1.0 - v_uv.y );
	vec3 hdr = textureLod( sampler2D( tex_scene, smp_scene ), vec2( v_uv.x, sampleY ), 0.0 ).rgb;
	hdr *= pow( 2.0, params.x );
	vec3 ldr = agxOutset( agxLook( agx( hdr ), params.z ) );
	out_color = vec4( clamp( ldr, 0.0, 1.0 ), 1.0 );
}
#pragma sokol @end

#pragma sokol @program prog vs fs
