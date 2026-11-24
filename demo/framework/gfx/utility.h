#pragma once

#include "math3d.h"

#include <math.h>

typedef struct Vec4
{
	float x, y, z, w;
} Vec4;

typedef struct Mat4
{
	Vec4 cx, cy, cz, cw;
} Mat4;

static inline Vec4 MakeVec4( float x, float y, float z, float w )
{
	Vec4 v = { x, y, z, w };
	return v;
}

static inline Mat4 MakeIdentity( void )
{
	Mat4 m = {
		{ 1.0f, 0.0f, 0.0f, 0.0f },
		{ 0.0f, 1.0f, 0.0f, 0.0f },
		{ 0.0f, 0.0f, 1.0f, 0.0f },
		{ 0.0f, 0.0f, 0.0f, 1.0f },
	};
	return m;
}

static inline Vec4 MulMV4( Mat4 m, Vec4 v )
{
	Vec4 r;
	r.x = m.cx.x * v.x + m.cy.x * v.y + m.cz.x * v.z + m.cw.x * v.w;
	r.y = m.cx.y * v.x + m.cy.y * v.y + m.cz.y * v.z + m.cw.y * v.w;
	r.z = m.cx.z * v.x + m.cy.z * v.y + m.cz.z * v.z + m.cw.z * v.w;
	r.w = m.cx.w * v.x + m.cy.w * v.y + m.cz.w * v.z + m.cw.w * v.w;
	return r;
}

static inline Mat4 MulMM4( Mat4 a, Mat4 b )
{
	Mat4 r;
	r.cx = MulMV4( a, b.cx );
	r.cy = MulMV4( a, b.cy );
	r.cz = MulMV4( a, b.cz );
	r.cw = MulMV4( a, b.cw );
	return r;
}

static inline Mat4 Transpose4( Mat4 m )
{
	Mat4 r;
	r.cx = MakeVec4( m.cx.x, m.cy.x, m.cz.x, m.cw.x );
	r.cy = MakeVec4( m.cx.y, m.cy.y, m.cz.y, m.cw.y );
	r.cz = MakeVec4( m.cx.z, m.cy.z, m.cz.z, m.cw.z );
	r.cw = MakeVec4( m.cx.w, m.cy.w, m.cz.w, m.cw.w );
	return r;
}

static inline Mat4 MakePerspective( float fovY, float aspect, float nearPlane, float farPlane )
{
	Mat4 m = { { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f } };

	float cotangent = 1.0f / tanf( fovY / 2.0f );
	m.cx.x = cotangent / aspect;
	m.cy.y = cotangent;
	m.cz.w = -1.0f;
	m.cz.z = farPlane / ( nearPlane - farPlane );
	m.cw.z = ( nearPlane * farPlane ) / ( nearPlane - farPlane );
	return m;
}

static inline void MakePerspectiveAndInverse( Mat4* outProj, Mat4* outProjInv, float fovY, float aspect, float nearPlane,
											  float farPlane )
{
	float tanHalf = tanf( fovY / 2.0f );
	float cotangent = 1.0f / tanHalf;

	float c = farPlane / ( nearPlane - farPlane );
	float d = ( nearPlane * farPlane ) / ( nearPlane - farPlane );

	Mat4 p = { { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f } };
	p.cx.x = cotangent / aspect;
	p.cy.y = cotangent;
	p.cz.z = c;
	p.cz.w = -1.0f;
	p.cw.z = d;
	*outProj = p;

	Mat4 pi = { { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f } };
	pi.cx.x = aspect * tanHalf;
	pi.cy.y = tanHalf;
	pi.cz.w = 1.0f / d;
	pi.cw.z = -1.0f;
	pi.cw.w = c / d;
	*outProjInv = pi;
}

static inline Mat4 MakeOrthographic( float left, float right, float bottom, float top, float nearPlane, float farPlane )
{
	Mat4 m = { { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f } };
	m.cx.x = 2.0f / ( right - left );
	m.cy.y = 2.0f / ( top - bottom );
	m.cz.z = 1.0f / ( nearPlane - farPlane );
	m.cw.w = 1.0f;
	m.cw.x = ( left + right ) / ( left - right );
	m.cw.y = ( bottom + top ) / ( bottom - top );
	m.cw.z = nearPlane / ( nearPlane - farPlane );
	return m;
}

static inline Mat4 MakeLookAt( Vec3 eye, Vec3 center, Vec3 up )
{
	Vec3 f = Normalize( Sub( center, eye ) );
	Vec3 s = Normalize( Cross( f, up ) );
	Vec3 u = Cross( s, f );

	Mat4 m;
	m.cx = MakeVec4( s.x, u.x, -f.x, 0.0f );
	m.cy = MakeVec4( s.y, u.y, -f.y, 0.0f );
	m.cz = MakeVec4( s.z, u.z, -f.z, 0.0f );
	m.cw = MakeVec4( -Dot( s, eye ), -Dot( u, eye ), Dot( f, eye ), 1.0f );
	return m;
}

static inline void MakeViewAndInverse( Mat4* outView, Mat4* outViewInv, Vec3 right, Vec3 up, Vec3 forward, Vec3 eye )
{
	Mat4 v;
	v.cx = MakeVec4( right.x, up.x, forward.x, 0.0f );
	v.cy = MakeVec4( right.y, up.y, forward.y, 0.0f );
	v.cz = MakeVec4( right.z, up.z, forward.z, 0.0f );
	v.cw = MakeVec4( -Dot( right, eye ), -Dot( up, eye ), -Dot( forward, eye ), 1.0f );
	*outView = v;

	Mat4 vi;
	vi.cx = MakeVec4( right.x, right.y, right.z, 0.0f );
	vi.cy = MakeVec4( up.x, up.y, up.z, 0.0f );
	vi.cz = MakeVec4( forward.x, forward.y, forward.z, 0.0f );
	vi.cw = MakeVec4( eye.x, eye.y, eye.z, 1.0f );
	*outViewInv = vi;
}

static inline Mat4 MakeMat4FromTransform( Transform t, Vec3 scale )
{
	Mat3 r = MakeMatrixFromQuat( t.q );
	Mat4 m;
	m.cx = MakeVec4( scale.x * r.cx.x, scale.x * r.cx.y, scale.x * r.cx.z, 0.0f );
	m.cy = MakeVec4( scale.y * r.cy.x, scale.y * r.cy.y, scale.y * r.cy.z, 0.0f );
	m.cz = MakeVec4( scale.z * r.cz.x, scale.z * r.cz.y, scale.z * r.cz.z, 0.0f );
	m.cw = MakeVec4( t.p.x, t.p.y, t.p.z, 1.0f );
	return m;
}

static inline float SrgbToLinear( float c )
{
	return c <= 0.04045f ? c / 12.92f : powf( ( c + 0.055f ) / 1.055f, 2.4f );
}

static inline Vec4 MakeColor( uint32_t hexColor )
{
	uint32_t v = (uint32_t)hexColor;
	float k = 1.0f / 255.0f;
	float r = (float)( ( v >> 16 ) & 0xFFu ) * k;
	float g = (float)( ( v >> 8 ) & 0xFFu ) * k;
	float b = (float)( v & 0xFFu ) * k;
	return MakeVec4( SrgbToLinear( r ), SrgbToLinear( g ), SrgbToLinear( b ), 1.0f );
}

static inline Vec4 MakeColorAlpha( uint32_t hexColor, float alpha )
{
	uint32_t v = (uint32_t)hexColor;
	float k = 1.0f / 255.0f;
	float r = (float)( ( v >> 16 ) & 0xFFu ) * k;
	float g = (float)( ( v >> 8 ) & 0xFFu ) * k;
	float b = (float)( v & 0xFFu ) * k;
	return MakeVec4( SrgbToLinear( r ), SrgbToLinear( g ), SrgbToLinear( b ), alpha );
}
