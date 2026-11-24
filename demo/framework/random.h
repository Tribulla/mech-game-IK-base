#pragma once

#include "math3d.h"

#include <math.h>
#include <stdint.h>

class Rng
{
public:
	explicit Rng( uint64_t seed = 12345u )
	{
		Seed( seed );
	}

	void Seed( uint64_t seed )
	{
		m_state = 0u;
		m_inc = ( seed << 1u ) | 1u;
		NextU32();
		m_state += seed;
		NextU32();
	}

	uint32_t NextU32()
	{
		uint64_t old = m_state;
		m_state = old * 6364136223846793005ULL + m_inc;
		uint32_t xorshifted = (uint32_t)( ( ( old >> 18u ) ^ old ) >> 27u );
		uint32_t rot = (uint32_t)( old >> 59u );
		return ( xorshifted >> rot ) | ( xorshifted << ( ( 0u - rot ) & 31u ) );
	}

	float Float()
	{
		return (float)( NextU32() >> 8 ) * ( 1.0f / 16777216.0f );
	}

	float Range( float lo, float hi )
	{
		return lo + ( hi - lo ) * Float();
	}

	int IntRange( int lo, int hi )
	{
		uint32_t span = (uint32_t)( hi - lo + 1 );
		return lo + (int)( NextU32() % span );
	}

	bool Chance( float probability )
	{
		return Float() < probability;
	}

	Vec3 RangeVec3( Vec3 lo, Vec3 hi )
	{
		return { Range( lo.x, hi.x ), Range( lo.y, hi.y ), Range( lo.z, hi.z ) };
	}

	Vec3 UnitVector()
	{
		float z = Range( -1.0f, 1.0f );
		float a = Range( 0.0f, 2.0f * MATH_PI );
		float r = sqrtf( MaxFloat( 0.0f, 1.0f - z * z ) );
		return { r * cosf( a ), r * sinf( a ), z };
	}

	Quat Rotation()
	{
		float u1 = Float(), u2 = Range( 0.0f, 2.0f * MATH_PI ), u3 = Range( 0.0f, 2.0f * MATH_PI );
		float s1 = sqrtf( 1.0f - u1 ), s2 = sqrtf( u1 );
		Quat q;
		q.v = { s1 * sinf( u2 ), s1 * cosf( u2 ), s2 * sinf( u3 ) };
		q.s = s2 * cosf( u3 );
		return q;
	}

private:
	uint64_t m_state;
	uint64_t m_inc;
};

namespace Noise
{
inline uint32_t Hash( int32_t x, int32_t y, int32_t z, uint32_t seed )
{
	uint32_t h = seed * 0x27d4eb2dU;
	h ^= (uint32_t)x * 0x85ebca6bU;
	h = ( h << 13 ) | ( h >> 19 );
	h ^= (uint32_t)y * 0xc2b2ae35U;
	h = ( h << 13 ) | ( h >> 19 );
	h ^= (uint32_t)z * 0x165667b1U;
	h ^= h >> 16;
	h *= 0x7feb352dU;
	h ^= h >> 15;
	h *= 0x846ca68bU;
	h ^= h >> 16;
	return h;
}

inline float Hash01( int32_t x, int32_t y, int32_t z, uint32_t seed )
{
	return (float)( Hash( x, y, z, seed ) >> 8 ) * ( 1.0f / 16777216.0f );
}

inline float Fade( float t )
{
	return t * t * t * ( t * ( t * 6.0f - 15.0f ) + 10.0f );
}

inline float Grad2( uint32_t h, float x, float y )
{
	switch ( h & 7u )
	{
		case 0: return x + y;
		case 1: return x - y;
		case 2: return -x + y;
		case 3: return -x - y;
		case 4: return x;
		case 5: return -x;
		case 6: return y;
		default: return -y;
	}
}

inline float Grad3( uint32_t h, float x, float y, float z )
{
	switch ( h % 12u )
	{
		case 0: return x + y;
		case 1: return -x + y;
		case 2: return x - y;
		case 3: return -x - y;
		case 4: return x + z;
		case 5: return -x + z;
		case 6: return x - z;
		case 7: return -x - z;
		case 8: return y + z;
		case 9: return -y + z;
		case 10: return y - z;
		default: return -y - z;
	}
}

inline float Perlin2( float x, float y, uint32_t seed = 0 )
{
	int32_t x0 = (int32_t)floorf( x ), y0 = (int32_t)floorf( y );
	float fx = x - (float)x0, fy = y - (float)y0;
	float u = Fade( fx ), v = Fade( fy );
	float n00 = Grad2( Hash( x0, y0, 0, seed ), fx, fy );
	float n10 = Grad2( Hash( x0 + 1, y0, 0, seed ), fx - 1.0f, fy );
	float n01 = Grad2( Hash( x0, y0 + 1, 0, seed ), fx, fy - 1.0f );
	float n11 = Grad2( Hash( x0 + 1, y0 + 1, 0, seed ), fx - 1.0f, fy - 1.0f );
	float nx0 = n00 + u * ( n10 - n00 );
	float nx1 = n01 + u * ( n11 - n01 );
	return 0.7f * ( nx0 + v * ( nx1 - nx0 ) );
}

inline float Perlin3( float x, float y, float z, uint32_t seed = 0 )
{
	int32_t x0 = (int32_t)floorf( x ), y0 = (int32_t)floorf( y ), z0 = (int32_t)floorf( z );
	float fx = x - (float)x0, fy = y - (float)y0, fz = z - (float)z0;
	float u = Fade( fx ), v = Fade( fy ), w = Fade( fz );
	float n[8];
	for ( int i = 0; i < 8; ++i )
	{
		int dx = i & 1, dy = ( i >> 1 ) & 1, dz = ( i >> 2 ) & 1;
		n[i] = Grad3( Hash( x0 + dx, y0 + dy, z0 + dz, seed ), fx - (float)dx, fy - (float)dy, fz - (float)dz );
	}
	float x00 = n[0] + u * ( n[1] - n[0] );
	float x10 = n[2] + u * ( n[3] - n[2] );
	float x01 = n[4] + u * ( n[5] - n[4] );
	float x11 = n[6] + u * ( n[7] - n[6] );
	float y0v = x00 + v * ( x10 - x00 );
	float y1v = x01 + v * ( x11 - x01 );
	return y0v + w * ( y1v - y0v );
}

inline float Fbm2( float x, float y, int octaves, uint32_t seed = 0, float lacunarity = 2.0f, float gain = 0.5f )
{
	float sum = 0.0f, amp = 1.0f, norm = 0.0f;
	for ( int i = 0; i < octaves; ++i )
	{
		sum += amp * Perlin2( x, y, seed + (uint32_t)i * 1013u );
		norm += amp;
		amp *= gain;
		x *= lacunarity;
		y *= lacunarity;
	}
	return norm > 0.0f ? sum / norm : 0.0f;
}

inline float Ridged2( float x, float y, int octaves, uint32_t seed = 0, float lacunarity = 2.0f, float gain = 0.5f )
{
	float sum = 0.0f, amp = 0.5f, norm = 0.0f, prev = 1.0f;
	for ( int i = 0; i < octaves; ++i )
	{
		float n = 1.0f - fabsf( Perlin2( x, y, seed + (uint32_t)i * 7919u ) );
		n *= n;
		sum += n * amp * prev;
		norm += amp;
		prev = n;
		amp *= gain;
		x *= lacunarity;
		y *= lacunarity;
	}
	return norm > 0.0f ? sum / norm : 0.0f;
}
} 
