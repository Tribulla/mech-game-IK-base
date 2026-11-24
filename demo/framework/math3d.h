#pragma once

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define MATH_PI 3.14159265359f
#define MATH_DEG_TO_RAD ( MATH_PI / 180.0f )
#define MATH_RAD_TO_DEG ( 180.0f / MATH_PI )

#ifdef __cplusplus
#define MATH_LITERAL( T ) T
#else
#define MATH_LITERAL( T ) ( T )
#endif

typedef struct Vec2
{
	float x, y;
} Vec2;

typedef struct Vec3
{
	float x, y, z;
} Vec3;

typedef struct Quat
{
	Vec3 v;
	float s;
} Quat;

typedef struct Transform
{
	Vec3 p;
	Quat q;
} Transform;

typedef struct Mat3
{
	Vec3 cx, cy, cz;
} Mat3;

typedef struct AABB
{
	Vec3 lowerBound;
	Vec3 upperBound;
} AABB;

typedef struct CosSin
{
	float cosine;
	float sine;
} CosSin;

static const Vec3 Vec3_zero = { 0.0f, 0.0f, 0.0f };
static const Vec3 Vec3_one = { 1.0f, 1.0f, 1.0f };
static const Vec3 Vec3_axisX = { 1.0f, 0.0f, 0.0f };
static const Vec3 Vec3_axisY = { 0.0f, 1.0f, 0.0f };
static const Vec3 Vec3_axisZ = { 0.0f, 0.0f, 1.0f };
static const Quat Quat_identity = { { 0.0f, 0.0f, 0.0f }, 1.0f };
static const Transform Transform_identity = { { 0.0f, 0.0f, 0.0f }, { { 0.0f, 0.0f, 0.0f }, 1.0f } };

static inline float MinFloat( float a, float b )
{
	return a < b ? a : b;
}
static inline float MaxFloat( float a, float b )
{
	return a > b ? a : b;
}
static inline float ClampFloat( float a, float lo, float hi )
{
	return a < lo ? lo : ( a > hi ? hi : a );
}
static inline float LerpFloat( float a, float b, float t )
{
	return a + t * ( b - a );
}
static inline int MinInt( int a, int b )
{
	return a < b ? a : b;
}
static inline int MaxInt( int a, int b )
{
	return a > b ? a : b;
}
static inline int ClampInt( int a, int lo, int hi )
{
	return a < lo ? lo : ( a > hi ? hi : a );
}

static inline float UnwindAngle( float radians )
{
	return remainderf( radians, 2.0f * MATH_PI );
}

static inline CosSin ComputeCosSin( float radians )
{
	CosSin cs = { cosf( radians ), sinf( radians ) };
	return cs;
}

static inline Vec3 MakeVec3( float x, float y, float z )
{
	Vec3 v = { x, y, z };
	return v;
}
static inline Vec3 Add( Vec3 a, Vec3 b )
{
	Vec3 r = { a.x + b.x, a.y + b.y, a.z + b.z };
	return r;
}
static inline Vec3 Sub( Vec3 a, Vec3 b )
{
	Vec3 r = { a.x - b.x, a.y - b.y, a.z - b.z };
	return r;
}
static inline Vec3 Neg( Vec3 a )
{
	Vec3 r = { -a.x, -a.y, -a.z };
	return r;
}

static inline Vec3 MulSV( float s, Vec3 v )
{
	Vec3 r = { s * v.x, s * v.y, s * v.z };
	return r;
}

static inline Vec3 MulAdd( Vec3 a, float s, Vec3 b )
{
	Vec3 r = { a.x + s * b.x, a.y + s * b.y, a.z + s * b.z };
	return r;
}

static inline Vec3 MulSub( Vec3 a, float s, Vec3 b )
{
	Vec3 r = { a.x - s * b.x, a.y - s * b.y, a.z - s * b.z };
	return r;
}

static inline Vec3 Blend2( float s, Vec3 a, float t, Vec3 b )
{
	Vec3 r = { s * a.x + t * b.x, s * a.y + t * b.y, s * a.z + t * b.z };
	return r;
}

static inline Vec3 Mul( Vec3 a, Vec3 b )
{
	Vec3 r = { a.x * b.x, a.y * b.y, a.z * b.z };
	return r;
}
static inline float Dot( Vec3 a, Vec3 b )
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline Vec3 Cross( Vec3 a, Vec3 b )
{
	Vec3 r = { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
	return r;
}
static inline float LengthSquared( Vec3 v )
{
	return v.x * v.x + v.y * v.y + v.z * v.z;
}
static inline float Length( Vec3 v )
{
	return sqrtf( LengthSquared( v ) );
}
static inline float Distance( Vec3 a, Vec3 b )
{
	return Length( Sub( b, a ) );
}
static inline float DistanceSquared( Vec3 a, Vec3 b )
{
	return LengthSquared( Sub( b, a ) );
}

static inline Vec3 Normalize( Vec3 v )
{
	float length = Length( v );
	if ( length < FLT_EPSILON )
	{
		return Vec3_zero;
	}
	return MulSV( 1.0f / length, v );
}
static inline Vec3 Lerp( Vec3 a, Vec3 b, float t )
{
	Vec3 r = { a.x + t * ( b.x - a.x ), a.y + t * ( b.y - a.y ), a.z + t * ( b.z - a.z ) };
	return r;
}
static inline Vec3 Min( Vec3 a, Vec3 b )
{
	Vec3 r = { MinFloat( a.x, b.x ), MinFloat( a.y, b.y ), MinFloat( a.z, b.z ) };
	return r;
}
static inline Vec3 Max( Vec3 a, Vec3 b )
{
	Vec3 r = { MaxFloat( a.x, b.x ), MaxFloat( a.y, b.y ), MaxFloat( a.z, b.z ) };
	return r;
}
static inline Vec3 Abs( Vec3 a )
{
	Vec3 r = { fabsf( a.x ), fabsf( a.y ), fabsf( a.z ) };
	return r;
}

static inline Vec3 Perp( Vec3 a )
{
	Vec3 p;
	if ( a.x < -0.5f || 0.5f < a.x )
	{
		p = MATH_LITERAL( Vec3 ){ a.y, -a.x, 0.0f };
	}
	else
	{
		p = MATH_LITERAL( Vec3 ){ 0.0f, a.z, -a.y };
	}
	return Normalize( p );
}

static inline Quat MakeQuatFromAxisAngle( Vec3 axis, float radians )
{
	CosSin cs = ComputeCosSin( 0.5f * radians );
	Quat q = { { cs.sine * axis.x, cs.sine * axis.y, cs.sine * axis.z }, cs.cosine };
	return q;
}

static inline Quat MakeQuatFromEuler( float yaw, float pitch, float roll );

static inline Quat MulQuat( Quat a, Quat b )
{
	Quat r;
	r.v.x = a.s * b.v.x + a.v.x * b.s + a.v.y * b.v.z - a.v.z * b.v.y;
	r.v.y = a.s * b.v.y - a.v.x * b.v.z + a.v.y * b.s + a.v.z * b.v.x;
	r.v.z = a.s * b.v.z + a.v.x * b.v.y - a.v.y * b.v.x + a.v.z * b.s;
	r.s = a.s * b.s - a.v.x * b.v.x - a.v.y * b.v.y - a.v.z * b.v.z;
	return r;
}

static inline Quat Conjugate( Quat q )
{
	Quat r = { { -q.v.x, -q.v.y, -q.v.z }, q.s };
	return r;
}

static inline Quat NormalizeQuat( Quat q )
{
	float length = sqrtf( q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z + q.s * q.s );
	if ( length < FLT_EPSILON )
	{
		return Quat_identity;
	}
	float inv = 1.0f / length;
	Quat r = { { inv * q.v.x, inv * q.v.y, inv * q.v.z }, inv * q.s };
	return r;
}

static inline Quat MakeQuatFromEuler( float yaw, float pitch, float roll )
{
	Quat qy = MakeQuatFromAxisAngle( Vec3_axisY, yaw );
	Quat qx = MakeQuatFromAxisAngle( Vec3_axisX, pitch );
	Quat qz = MakeQuatFromAxisAngle( Vec3_axisZ, roll );
	return MulQuat( qy, MulQuat( qx, qz ) );
}

static inline Quat NLerp( Quat a, Quat b, float t )
{
	if ( a.v.x * b.v.x + a.v.y * b.v.y + a.v.z * b.v.z + a.s * b.s < 0.0f )
	{
		b = MATH_LITERAL( Quat ){ { -b.v.x, -b.v.y, -b.v.z }, -b.s };
	}
	Quat r = { { a.v.x + t * ( b.v.x - a.v.x ), a.v.y + t * ( b.v.y - a.v.y ), a.v.z + t * ( b.v.z - a.v.z ) },
			   a.s + t * ( b.s - a.s ) };
	return NormalizeQuat( r );
}

static inline Vec3 RotateVector( Quat q, Vec3 v )
{

	Vec3 t1 = Cross( q.v, v );
	Vec3 t2 = MulAdd( t1, q.s, v );
	Vec3 t3 = Cross( q.v, t2 );
	return MulAdd( v, 2.0f, t3 );
}

static inline Vec3 InvRotateVector( Quat q, Vec3 v )
{
	return RotateVector( Conjugate( q ), v );
}

static inline Quat ComputeQuatBetweenUnitVectors( Vec3 v1, Vec3 v2 )
{
	Quat out;
	Vec3 m = Lerp( v1, v2, 0.5f );
	float tolerance = 100.0f * FLT_EPSILON;
	if ( LengthSquared( m ) > tolerance * tolerance )
	{
		out.v = Cross( v1, m );
		out.s = Dot( v1, m );
	}
	else
	{

		if ( fabsf( v1.x ) > 0.5f )
		{
			out.v = MATH_LITERAL( Vec3 ){ v1.y, -v1.x, 0.0f };
		}
		else
		{
			out.v = MATH_LITERAL( Vec3 ){ 0.0f, v1.z, -v1.y };
		}
		out.s = 0.0f;
	}
	return NormalizeQuat( out );
}

static inline Mat3 MakeMatrixFromQuat( Quat q )
{
	float x = q.v.x, y = q.v.y, z = q.v.z, w = q.s;
	float xx = x * x, yy = y * y, zz = z * z;
	float xy = x * y, xz = x * z, yz = y * z;
	float wx = w * x, wy = w * y, wz = w * z;
	Mat3 m;
	m.cx = MATH_LITERAL( Vec3 ){ 1.0f - 2.0f * ( yy + zz ), 2.0f * ( xy + wz ), 2.0f * ( xz - wy ) };
	m.cy = MATH_LITERAL( Vec3 ){ 2.0f * ( xy - wz ), 1.0f - 2.0f * ( xx + zz ), 2.0f * ( yz + wx ) };
	m.cz = MATH_LITERAL( Vec3 ){ 2.0f * ( xz + wy ), 2.0f * ( yz - wx ), 1.0f - 2.0f * ( xx + yy ) };
	return m;
}

static inline Transform MakeTransform( Vec3 p, Quat q )
{
	Transform t = { p, q };
	return t;
}
static inline Vec3 TransformPoint( Transform t, Vec3 v )
{
	return Add( RotateVector( t.q, v ), t.p );
}
static inline Vec3 InvTransformPoint( Transform t, Vec3 v )
{
	return InvRotateVector( t.q, Sub( v, t.p ) );
}

static inline Transform MulTransforms( Transform a, Transform b )
{
	Transform r = { Add( RotateVector( a.q, b.p ), a.p ), MulQuat( a.q, b.q ) };
	return r;
}
static inline Transform InvertTransform( Transform t )
{
	Quat qi = Conjugate( t.q );
	Transform r = { Neg( RotateVector( qi, t.p ) ), qi };
	return r;
}

static inline Transform ToRelativeTransform( Transform t, Vec3 origin )
{
	Transform r = { Sub( t.p, origin ), t.q };
	return r;
}

static inline bool IsValidAABB( AABB a )
{
	Vec3 d = Sub( a.upperBound, a.lowerBound );
	bool valid = d.x >= 0.0f && d.y >= 0.0f && d.z >= 0.0f;
	valid = valid && isfinite( a.lowerBound.x ) && isfinite( a.lowerBound.y ) && isfinite( a.lowerBound.z );
	valid = valid && isfinite( a.upperBound.x ) && isfinite( a.upperBound.y ) && isfinite( a.upperBound.z );
	return valid;
}
static inline Vec3 AABB_Center( AABB a )
{
	return MulSV( 0.5f, Add( a.lowerBound, a.upperBound ) );
}

static inline Vec3 AABB_Extents( AABB a )
{
	return MulSV( 0.5f, Sub( a.upperBound, a.lowerBound ) );
}
static inline AABB OffsetAABB( AABB box, Vec3 offset )
{
	AABB r = { Add( box.lowerBound, offset ), Add( box.upperBound, offset ) };
	return r;
}
static inline AABB AABB_Union( AABB a, AABB b )
{
	AABB r = { Min( a.lowerBound, b.lowerBound ), Max( a.upperBound, b.upperBound ) };
	return r;
}
static inline bool AABB_ContainsPoint( AABB a, Vec3 p )
{
	return p.x >= a.lowerBound.x && p.y >= a.lowerBound.y && p.z >= a.lowerBound.z && p.x <= a.upperBound.x &&
		   p.y <= a.upperBound.y && p.z <= a.upperBound.z;
}

#ifdef __cplusplus
inline Vec3 operator+( Vec3 a, Vec3 b )
{
	return Add( a, b );
}
inline Vec3 operator-( Vec3 a, Vec3 b )
{
	return Sub( a, b );
}
inline Vec3 operator-( Vec3 a )
{
	return Neg( a );
}
inline Vec3 operator*( float s, Vec3 v )
{
	return MulSV( s, v );
}
inline Vec3 operator*( Vec3 v, float s )
{
	return MulSV( s, v );
}
inline Vec3 operator/( Vec3 v, float s )
{
	return MulSV( 1.0f / s, v );
}
inline Vec3& operator+=( Vec3& a, Vec3 b )
{
	a = Add( a, b );
	return a;
}
inline Vec3& operator-=( Vec3& a, Vec3 b )
{
	a = Sub( a, b );
	return a;
}
inline Vec3& operator*=( Vec3& a, float s )
{
	a = MulSV( s, a );
	return a;
}
inline bool operator==( Vec3 a, Vec3 b )
{
	return a.x == b.x && a.y == b.y && a.z == b.z;
}
inline bool operator!=( Vec3 a, Vec3 b )
{
	return !( a == b );
}
inline Quat operator*( Quat a, Quat b )
{
	return MulQuat( a, b );
}
inline Vec3 operator*( Quat q, Vec3 v )
{
	return RotateVector( q, v );
}
inline Transform operator*( Transform a, Transform b )
{
	return MulTransforms( a, b );
}
#endif
