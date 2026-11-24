#pragma once

#include "mech_bridge.h"

#include "imgui.h"
#include "random.h"
#include "scene.h"

#include <math.h>
#include <vector>

namespace mech
{

inline Vec3 PV( const float* p, int k )
{
	return { p[k], p[k + 1], p[k + 2] };
}

inline Vec3 BodyPoint( const float* p, float x, float y, float z )
{
	return PV( p, PosePos ) + x * PV( p, PoseRight ) + y * PV( p, PoseUp ) + z * PV( p, PoseFwd );
}

inline Quat BodyRotation( const float* p )
{
	return QuatFromAxes( Neg( PV( p, PoseRight ) ), PV( p, PoseUp ), PV( p, PoseFwd ) );
}

inline void BodyBox( const float* p, Quat q, float x, float y, float z, float hx, float hy, float hz, const Material& m )
{
	DrawBox( Transform{ BodyPoint( p, x, y, z ), q }, { 2.0f * hx, 2.0f * hy, 2.0f * hz }, m );
}

inline const char* ModeLabel( const float* p )
{
	int state = (int)p[PoseState], flags = (int)p[PoseFlags];
	if ( flags & FlagWire )
		return "wire climb";
	if ( flags & FlagLeap )
		return "run (leap)";
	if ( state == StateAir )
		return "airborne";
	if ( state == StateMantle )
		return "mantle";
	if ( state == StateCrouchJump )
		return "jump";
	if ( flags & FlagDrift )
		return "slide";
	if ( flags & FlagWall )
		return "climb";
	if ( flags & FlagCrouch )
		return "crouch";
	if ( flags & FlagRun )
		return "run";
	if ( flags & FlagSlow )
		return "slow walk";
	return "walk";
}

enum MechKind
{
	KindLynx = 0,
	KindStriker,
	KindStrider,
	KindGecko,
	KindBastion,
	KindWarden,
	KindMantis,
	KindScarab,
	KindColossus
};

inline Material FactionMaterial( int faction )
{
	return Materials::SeriesMaterial( faction == 1 ? 0 : 1 );
}

inline void Barrel( const float* p, float x, float y, float z, float length, float radius, const Material& m )
{
	DrawRod( BodyPoint( p, x, y, z ), BodyPoint( p, x, y, z + length ), radius, m );
}

inline Vec3 ClampedGunDir( const float* p, const Vec3& aim )
{
	Vec3 r = PV( p, PoseRight ), u = PV( p, PoseUp ), f = PV( p, PoseFwd );
	float lx = Dot( aim, r );
	float ly = Dot( aim, u );
	float lz = Dot( aim, f );
	float t = ClampFloat( ( lz + 0.25f ) * 2.0f, 0.0f, 1.0f );
	float s = t * t * ( 3.0f - 2.0f * t );
	float minY = 0.12f + ( -0.22f - 0.12f ) * s;
	if ( ly < minY || ly > 0.94f )
	{
		ly = ClampFloat( ly, minY, 0.94f );
		float xz2 = lx * lx + lz * lz;
		float want2 = 1.0f - ly * ly;
		if ( xz2 > 1e-8f )
		{
			float s = sqrtf( want2 / xz2 );
			lx *= s;
			lz *= s;
		}
		else
		{
			lz = sqrtf( want2 );
			lx = 0.0f;
		}
	}
	return Normalize( MulAdd( MulAdd( MulSV( lx, r ), ly, u ), lz, f ) );
}

inline Vec3 PlantedGroundUp( const float* p, const MechInfo& info, const Vec3& hullUp )
{
	if ( hullUp.y <= 0.5f )
	{
		return hullUp;
	}
	const int J = info.jointsPerLeg;
	const int segs = J - 1;
	const int stride = info.LegStride();
	Vec3 pf[8];
	int count = 0;
	for ( int l = 0; l < info.legs && count < 8; ++l )
	{
		const float* L = p + kHeaderFloats + l * stride;
		if ( L[J * 3 + 3] > 0.5f )
		{
			pf[count++] = PV( L, segs * 3 );
		}
	}
	if ( count >= 3 )
	{
		Vec3 nSum = { 0.0f, 0.0f, 0.0f };
		for ( int a = 0; a < count; ++a )
		{
			for ( int b = a + 1; b < count; ++b )
			{
				Vec3 ab = Sub( pf[b], pf[a] );
				for ( int c = b + 1; c < count; ++c )
				{
					Vec3 cr = Cross( ab, Sub( pf[c], pf[a] ) );
					if ( Dot( cr, hullUp ) < 0.0f )
					{
						cr = MulSV( -1.0f, cr );
					}
					nSum = Add( nSum, cr );
				}
			}
		}
		if ( LengthSquared( nSum ) > 1e-6f )
		{
			Vec3 gUp = Normalize( nSum );
			if ( gUp.y > 0.45f )
			{
				return gUp;
			}
		}
	}
	return Normalize( Lerp( { 0.0f, 1.0f, 0.0f }, hullUp, 0.20f ) );
}

inline Quat FootRotation( const Vec3& hullUp, const Vec3& hullFwd, const Vec3& groundUp, const Vec3& ankle,
						  const Vec3& foot, bool planted, Vec3* outUp )
{
	Vec3 footUp = hullUp;
	if ( hullUp.y > 0.5f )
	{
		if ( planted )
		{
			footUp = groundUp;
		}
		else
		{
			const Vec3 worldUp = { 0.0f, 1.0f, 0.0f };
			Vec3 shin = Sub( ankle, foot );
			float sl = LengthSquared( shin );
			if ( sl > 1e-6f )
			{
				footUp = Normalize( Lerp( worldUp, MulSV( 1.0f / sqrtf( sl ), shin ), 0.25f ) );
			}
			else
			{
				footUp = worldUp;
			}
		}
	}
	Vec3 ff = Sub( hullFwd, MulSV( Dot( hullFwd, footUp ), footUp ) );
	if ( LengthSquared( ff ) < 1e-6f )
	{
		ff = { 0.0f, 0.0f, 1.0f };
		ff = Sub( ff, MulSV( Dot( ff, footUp ), footUp ) );
	}
	ff = Normalize( ff );
	Vec3 fx = Normalize( Cross( footUp, ff ) );
	*outUp = footUp;
	return QuatFromAxes( fx, footUp, ff );
}

inline void MainGun( const float* p, const Vec3* aim, float x, float y, float z, float length, float radius,
					 const Material& m )
{
	if ( aim == nullptr )
	{
		Barrel( p, x, y, z, length, radius, m );
		return;
	}
	Vec3 a = BodyPoint( p, x, y, z );
	Vec3 dir = ClampedGunDir( p, *aim );
	DrawRod( a, MulAdd( a, length, dir ), radius, m );
}

inline void SubArms( const float* p, float hx, float hz, float reach, const Material& metal )
{
	for ( int s = -1; s <= 1; s += 2 )
	{
		float x = (float)s;
		DrawRod( BodyPoint( p, x * hx * 0.65f, -0.12f, hz * 0.85f ), BodyPoint( p, x * hx * 0.85f, -0.3f, hz + reach ), 0.07f, metal );
		Barrel( p, x * hx * 0.85f, -0.3f, hz + reach - 0.1f, 0.6f, 0.045f, metal );
	}
}

inline void DrawHull( const float* p, Quat q, const MechInfo& info, const Material& body, const Material& metal,
					  const Material& sensor, const Vec3* aim )
{
	const float hx = info.half.x, hy = info.half.y, hz = info.half.z;
	BodyBox( p, q, 0, 0, 0, hx, hy, hz, body );
	switch ( info.kind )
	{
		case KindLynx:
			BodyBox( p, q, 0, -0.05f, hz + 0.18f, hx * 0.72f, hy * 0.75f, 0.22f, body );
			BodyBox( p, q, 0, hy * 0.1f, hz + 0.41f, hx * 0.42f, 0.06f, 0.015f, sensor );
			DrawRod( BodyPoint( p, 0, hy, -hz * 0.55f ), BodyPoint( p, 0, hy + 0.5f, -hz * 0.2f ), 0.1f, metal );
			BodyBox( p, q, 0, hy + 0.55f, 0.0f, 0.2f, 0.18f, 0.55f, body );
			MainGun( p, aim, 0, hy + 0.57f, 0.55f, 2.1f, 0.06f, metal );
			SubArms( p, hx, hz, 0.9f, metal );
			for ( int s = -1; s <= 1; s += 2 )
			{
				BodyBox( p, q, (float)s * ( hx + 0.07f ), 0.1f, hz * 0.45f, 0.07f, 0.11f, 0.2f, metal );
			}
			break;

		case KindStriker:
			BodyBox( p, q, 0, -0.08f, hz + 0.2f, hx * 0.7f, hy * 0.7f, 0.25f, body );
			BodyBox( p, q, 0, hy * 0.05f, hz + 0.46f, hx * 0.45f, 0.06f, 0.015f, sensor );
			DrawRod( BodyPoint( p, 0, hy, -hz * 0.6f ), BodyPoint( p, 0, hy + 0.7f, -hz * 0.25f ), 0.12f, metal );
			BodyBox( p, q, 0, hy + 0.75f, -0.05f, 0.26f, 0.22f, 0.75f, body );
			MainGun( p, aim, 0, hy + 0.77f, 0.7f, 2.8f, 0.085f, metal );
			if ( aim == nullptr )
			{
				BodyBox( p, q, 0, hy + 0.77f, 3.55f, 0.14f, 0.09f, 0.14f, metal );
			}
			else
			{
				DrawBall( MulAdd( BodyPoint( p, 0, hy + 0.77f, 0.7f ), 2.85f, ClampedGunDir( p, *aim ) ), 0.14f, metal );
			}
			SubArms( p, hx, hz, 1.0f, metal );
			for ( int s = -1; s <= 1; s += 2 )
			{
				BodyBox( p, q, (float)s * ( hx + 0.07f ), 0.12f, hz * 0.45f, 0.07f, 0.12f, 0.24f, metal );
			}
			break;

		case KindStrider:
			/* Biped pelvis collar, armored torso/cockpit, visor, rear pack, and twin shoulder cannons. */
			BodyBox( p, q, 0, -hy * 0.65f, 0.0f, hx * 0.75f, hy * 0.35f, hz * 0.85f, metal );
			BodyBox( p, q, 0, hy + 0.28f, 0.05f, hx * 0.92f, 0.36f, hz * 0.88f, body );
			BodyBox( p, q, 0, hy + 0.30f, hz * 0.94f, hx * 0.56f, 0.12f, 0.03f, sensor );
			BodyBox( p, q, 0, hy + 0.16f, -hz - 0.16f, hx * 0.68f, 0.32f, 0.18f, metal );
			for ( int s = -1; s <= 1; s += 2 )
			{
				float sx = (float)s * ( hx + 0.16f );
				BodyBox( p, q, sx, hy + 0.48f, -0.04f, 0.18f, 0.18f, 0.40f, body );
				MainGun( p, aim, sx, hy + 0.48f, 0.36f, 1.25f, 0.055f, metal );
				DrawRod( BodyPoint( p, (float)s * 0.26f, hy + 0.64f, -0.18f ),
						 BodyPoint( p, (float)s * 0.28f, hy + 1.08f, -0.32f ), 0.025f, metal );
			}
			break;

		case KindGecko:
			/* Low-profile wall-climbing spider hull with LiDAR dome and swept antennae. */
			BodyBox( p, q, 0, -0.04f, hz + 0.22f, hx * 0.68f, hy * 0.65f, 0.24f, body );
			BodyBox( p, q, 0, 0.02f, hz + 0.47f, hx * 0.44f, 0.06f, 0.02f, sensor );
			BodyBox( p, q, 0, hy + 0.12f, 0.05f, 0.36f, 0.14f, 0.42f, metal );
			DrawBall( BodyPoint( p, 0, hy + 0.28f, 0.12f ), 0.16f, sensor );
			for ( int s = -1; s <= 1; s += 2 )
			{
				BodyBox( p, q, (float)s * ( hx + 0.05f ), -0.04f, 0.0f, 0.06f, 0.10f, hz * 0.72f, metal );
				DrawRod( BodyPoint( p, (float)s * 0.25f, hy, -hz * 0.5f ),
						 BodyPoint( p, (float)s * 0.45f, hy + 0.55f, -hz - 0.55f ), 0.028f, metal );
			}
			break;

		case KindBastion:
			BodyBox( p, q, 0, hy + 0.08f, -0.25f, hx * 0.85f, 0.1f, hz * 0.75f, body );
			BodyBox( p, q, 0, hy + 0.45f, 0.15f, 0.85f, 0.3f, 1.0f, body );
			BodyBox( p, q, 0, hy + 0.5f, 1.16f, 0.35f, 0.08f, 0.02f, sensor );
			MainGun( p, aim, 0, hy + 0.42f, 1.1f, 3.2f, 0.11f, metal );
			for ( int s = -1; s <= 1; s += 2 )
			{
				Barrel( p, (float)s * 0.7f, hy + 0.82f, 0.3f, 0.8f, 0.04f, metal );
			}
			break;

		case KindWarden:
			BodyBox( p, q, 0, -0.12f, hz + 0.28f, hx * 0.8f, hy * 0.65f, 0.3f, body );
			BodyBox( p, q, 0, hy + 0.38f, -0.15f, 0.85f, 0.36f, 1.05f, body );
			MainGun( p, aim, 0, hy + 0.42f, 0.9f, 2.4f, 0.11f, metal );
			if ( aim == nullptr )
			{
				BodyBox( p, q, 0, hy + 0.58f, 1.4f, 0.06f, 0.06f, 0.25f, metal );
			}
			for ( int s = -1; s <= 1; s += 2 )
			{
				BodyBox( p, q, (float)s * 0.5f, hy + 0.8f, 0.55f, 0.1f, 0.07f, 0.12f, sensor );
			}
			break;

		case KindMantis:
			BodyBox( p, q, 0, -0.05f, hz + 0.35f, hx * 0.7f, hy * 0.7f, 0.45f, body );
			BodyBox( p, q, 0, -0.12f, hz + 0.95f, hx * 0.4f, hy * 0.42f, 0.25f, body );
			BodyBox( p, q, 0, hy * 0.15f, hz + 0.75f, hx * 0.5f, 0.05f, 0.08f, sensor );
			BodyBox( p, q, 0, hy + 0.3f, -0.25f, 0.42f, 0.28f, 0.65f, metal );
			break;

		case KindScarab:
			BodyBox( p, q, 0, 0.05f, -hz - 0.35f, hx * 0.8f, hy * 0.85f, 0.4f, body );
			BodyBox( p, q, 0, 0.12f, hz + 0.3f, 0.32f, 0.26f, 0.3f, body );
			BodyBox( p, q, 0, 0.14f, hz + 0.61f, 0.18f, 0.1f, 0.015f, sensor );
			for ( int s = -1; s <= 1; s += 2 )
			{
				Barrel( p, (float)s * ( hx + 0.06f ), 0.0f, hz * 0.2f, 0.8f, 0.04f, metal );
			}
			break;

		case KindColossus:
			BodyBox( p, q, 0, -0.15f, hz + 0.3f, hx * 0.8f, hy * 0.6f, 0.35f, body );
			BodyBox( p, q, 0, hy + 0.6f, -0.3f, 1.3f, 0.58f, 1.6f, body );
			MainGun( p, aim, 0, hy + 0.62f, 1.25f, 5.0f, 0.19f, metal );
			MainGun( p, aim, 0.75f, hy + 0.5f, 1.25f, 2.4f, 0.1f, metal );
			for ( int s = -1; s <= 1; s += 2 )
			{
				Barrel( p, (float)s * 1.05f, hy + 1.22f, 0.4f, 0.9f, 0.05f, metal );
				BodyBox( p, q, (float)s * 0.45f, hy + 1.2f, 0.9f, 0.14f, 0.08f, 0.12f, sensor );
			}
			BodyBox( p, q, 0, hy * 0.3f, -hz - 0.35f, hx * 0.8f, hy * 0.6f, 0.38f, metal );
			break;

		default:
			break;
	}
}

inline void DrawMech( const float* p, const MechInfo& info, bool driven, const Vec3* aim = nullptr )
{
	const Material body = FactionMaterial( info.faction );
	const Material metal = Materials::Static();
	const Material sensor = driven ? Materials::Highlight() : Materials::Sleeping();
	const float hx = info.half.x, hz = info.half.z;
	const Quat q = BodyRotation( p );
	const Vec3 up = PV( p, PoseUp );
	const Vec3 fwd = PV( p, PoseFwd );

	DrawHull( p, q, info, body, metal, sensor, aim );

	if ( (int)p[PoseFlags] & FlagWire )
	{
		Vec3 anchor = PV( p, PoseAnchor );
		Vec3 right = PV( p, PoseRight );
		for ( int s = -1; s <= 1; s += 2 )
		{
			Vec3 from = BodyPoint( p, (float)s * ( hx + 0.07f ), 0.1f, hz * 0.45f );
			Vec3 to = MulAdd( anchor, (float)s * 0.6f, right );
			DrawRod( from, to, 0.025f, metal );
			DrawBall( to, 0.09f, metal );
		}
	}

	const Vec3 groundUp = PlantedGroundUp( p, info, up );
	const int J = info.jointsPerLeg;
	const int segs = J - 1;
	const bool coxa = segs > info.widthCount;
	const int stride = info.LegStride();
	for ( int l = 0; l < info.legs; ++l )
	{
		const float* L = p + kHeaderFloats + l * stride;
		Vec3 n = PV( L, J * 3 );
		bool planted = L[J * 3 + 3] > 0.5f;
		bool sliding = L[J * 3 + 4] > 0.5f;
		for ( int s = 0; s < segs; ++s )
		{
			Vec3 a = PV( L, s * 3 ), b = PV( L, s * 3 + 3 );
			int wi = s - ( coxa ? 1 : 0 );
			float w = wi < 0 ? info.widths[0] * 1.1f : info.widths[wi < info.widthCount ? wi : info.widthCount - 1];
			DrawRod( a, b, 0.42f * w, body );
			DrawBall( a, 0.55f * w, metal );
			if ( s + 1 < segs && wi >= 0 ) 
			{
				Vec3 c = PV( L, s * 3 + 6 );
				Vec3 pa = MulAdd( Lerp( a, b, 0.3f ), 0.5f * w, n );
				Vec3 pb = MulAdd( Lerp( b, c, 0.55f ), 0.5f * w, n );
				DrawRod( pa, pb, 0.11f * w, metal );
			}
		}
		Vec3 foot = PV( L, segs * 3 );
		Vec3 ankle = PV( L, ( segs - 1 ) * 3 );
		if ( info.kind == KindStriker )
		{
			Vec3 d = Normalize( Sub( foot, ankle ) );
			Vec3 side = MulSV( 0.14f, Normalize( Cross( d, n ) ) );
			DrawRod( Add( Lerp( ankle, foot, 0.1f ), side ), Add( Lerp( ankle, foot, 0.75f ), side ), 0.075f, metal );
		}
		if ( info.kind == KindMantis && l < 2 && J >= 5 )
		{
			Vec3 knee = PV( L, 3 * 3 );
			DrawRod( knee, MulAdd( MulAdd( knee, 1.3f, up ), 0.35f, fwd ), 0.05f, metal );
		}
		float wf = info.widths[info.widthCount - 1];
		Material fm = sliding ? Materials::Warning() : planted ? Materials::Static() : Materials::KinematicIdle();
		Vec3 footUp = up;
		Quat fq = FootRotation( up, fwd, groundUp, ankle, foot, planted, &footUp );
		DrawBox( Transform{ MulAdd( foot, wf * 0.12f, footUp ), fq }, { wf * 1.9f, wf * 0.44f, wf * 2.6f }, fm );
	}
}

inline void DrawMechLod( const float* p, const MechInfo& info, bool driven, int lod, const Vec3* aim = nullptr )
{
	if ( lod <= 0 )
	{
		DrawMech( p, info, driven, aim );
		return;
	}

	const Material body = FactionMaterial( info.faction );
	const Material metal = Materials::Static();
	const float hx = info.half.x, hy = info.half.y, hz = info.half.z;
	const Quat q = BodyRotation( p );
	const Vec3 up = PV( p, PoseUp );

	BodyBox( p, q, 0, 0, 0, hx, hy, hz, body );
	if ( lod == 1 )
	{
		switch ( info.kind )
		{
			case KindLynx:
				Barrel( p, 0, hy + 0.57f, 0.55f, 2.1f, 0.06f, metal );
				break;
			case KindStriker:
				Barrel( p, 0, hy + 0.77f, 0.7f, 2.8f, 0.085f, metal );
				break;
			case KindStrider:
				BodyBox( p, q, 0, hy + 0.28f, 0.05f, hx * 0.92f, 0.36f, hz * 0.88f, body );
				Barrel( p, -( hx + 0.16f ), hy + 0.48f, 0.36f, 1.25f, 0.055f, metal );
				Barrel( p, ( hx + 0.16f ), hy + 0.48f, 0.36f, 1.25f, 0.055f, metal );
				break;
			case KindGecko:
				BodyBox( p, q, 0, hy + 0.12f, 0.05f, 0.36f, 0.14f, 0.42f, metal );
				break;
			case KindBastion:
				BodyBox( p, q, 0, hy + 0.45f, 0.15f, 0.85f, 0.3f, 1.0f, body );
				Barrel( p, 0, hy + 0.42f, 1.1f, 3.2f, 0.11f, metal );
				break;
			case KindWarden:
				BodyBox( p, q, 0, hy + 0.38f, -0.15f, 0.85f, 0.36f, 1.05f, body );
				Barrel( p, 0, hy + 0.42f, 0.9f, 2.4f, 0.11f, metal );
				break;
			case KindMantis:
				BodyBox( p, q, 0, hy + 0.3f, -0.25f, 0.42f, 0.28f, 0.65f, metal );
				break;
			case KindScarab:
				BodyBox( p, q, 0, 0.12f, hz + 0.3f, 0.32f, 0.26f, 0.3f, body );
				break;
			case KindColossus:
				BodyBox( p, q, 0, hy + 0.6f, -0.3f, 1.3f, 0.58f, 1.6f, body );
				Barrel( p, 0, hy + 0.62f, 1.25f, 5.0f, 0.19f, metal );
				break;
			default:
				break;
		}
	}

	const int J = info.jointsPerLeg;
	const int segs = J - 1;
	const bool coxa = segs > info.widthCount;
	const int stride = info.LegStride();
	const float wf = info.widths[info.widthCount - 1];
	for ( int l = 0; l < info.legs; ++l )
	{
		const float* L = p + kHeaderFloats + l * stride;
		if ( lod == 1 )
		{
			for ( int s = 0; s < segs; ++s )
			{
				Vec3 a = PV( L, s * 3 ), b = PV( L, s * 3 + 3 );
				int wi = s - ( coxa ? 1 : 0 );
				float w = wi < 0 ? info.widths[0] * 1.1f : info.widths[wi < info.widthCount ? wi : info.widthCount - 1];
				DrawRod( a, b, 0.42f * w, body );
			}
			Vec3 foot = PV( L, segs * 3 );
			bool planted = L[J * 3 + 3] > 0.5f;
			bool sliding = L[J * 3 + 4] > 0.5f;
			Material fm = sliding ? Materials::Warning() : planted ? Materials::Static() : Materials::KinematicIdle();
			DrawBox( Transform{ MulAdd( foot, wf * 0.12f, up ), q }, { wf * 1.9f, wf * 0.44f, wf * 2.6f }, fm );
		}
		else
		{
			for ( int s = coxa ? 1 : 0; s < segs; ++s )
			{
				Vec3 a = PV( L, s * 3 ), b = PV( L, s * 3 + 3 );
				int wi = s - ( coxa ? 1 : 0 );
				float w = wi < 0 ? info.widths[0] * 1.1f : info.widths[wi < info.widthCount ? wi : info.widthCount - 1];
				DrawRod( a, b, 0.44f * w, body );
			}
		}
	}
}

inline void DrawDrivenMarker( const World& w, const float* p, const MechInfo& info )
{
	Vec3 pos = PV( p, PosePos );
	float t;
	Vec3 hp, hn;
	if ( w.Raycast( pos, { 0.0f, -1.0f, 0.0f }, 40.0f, &t, &hp, &hn, nullptr ) )
	{
		float r = 1.3f * MaxFloat( info.half.x, info.half.z ) + 0.6f;
		DrawCircleXZ( Add( hp, { 0.0f, 0.05f, 0.0f } ), r, Color( Palette::Highlight ), 40 );
	}
}

inline void DrawDebugLines( const float* d, int floats )
{
	for ( int i = 0; i + kDebugFloats <= floats; i += kDebugFloats )
	{
		DrawLineEx( PV( d, i ), PV( d, i + 3 ), DebugRoleColor( (int)d[i + 6] ), Style::LinePx, OVERLAY_THICKNESS_PIXELS,
					OVERLAY_OCCLUSION_DIM );
	}
}

class Effects
{
public:
	void Clear()
	{
		m_sparks.clear();
		m_skids.clear();
		m_skidHead = 0;
	}

	int HandleEvents( const float* ev, int floats )
	{
		int seen = 0;
		for ( int i = 0; i + kEventFloats <= floats; i += kEventFloats )
		{
			const float* e = ev + i;
			int type = (int)e[0];
			Vec3 p = PV( e, 1 ), n = PV( e, 4 ), x = PV( e, 8 );
			float s = e[7];
			seen |= 1 << ( type & 15 );
			switch ( type )
			{
				case EvSpark:
					for ( int k = 0; k < 3; ++k )
					{
						AddSpark( p, 0.35f * x + Vec3{ m_rng.Range( -2.0f, 2.0f ), m_rng.Range( 1.5f, 3.5f ), m_rng.Range( -2.0f, 2.0f ) },
								  0.35f, false );
					}
					break;
				case EvBoost:
					for ( int k = 0; k < 30; ++k )
					{
						AddSpark( p, { m_rng.Range( -4.0f, 4.0f ), m_rng.Range( 1.0f, 3.0f ), m_rng.Range( -4.0f, 4.0f ) }, 0.6f, false );
					}
					break;
				case EvImpact:
					for ( int k = 0; k < 40; ++k )
					{
						AddSpark( p, { m_rng.Range( -7.0f, 7.0f ), m_rng.Range( 2.0f, 7.0f ), m_rng.Range( -7.0f, 7.0f ) }, 0.5f, true );
					}
					break;
				case EvSkid:
				{
					Skid k = { MulAdd( p, 0.03f, n ), MulAdd( x, 0.03f, n ), s * 1.6f, 0.0f };
					if ( (int)m_skids.size() < kMaxSkids )
					{
						m_skids.push_back( k );
					}
					else
					{
						m_skids[m_skidHead] = k;
						m_skidHead = ( m_skidHead + 1 ) % kMaxSkids;
					}
				}
				break;
				default:
					break;
			}
			(void)n;
		}
		return seen;
	}

	void Update( float dt )
	{
		for ( size_t i = 0; i < m_sparks.size(); )
		{
			Spark& s = m_sparks[i];
			s.life -= dt;
			if ( s.life <= 0.0f )
			{
				s = m_sparks.back();
				m_sparks.pop_back();
				continue;
			}
			s.v.y -= 14.0f * dt;
			s.p = MulAdd( s.p, dt, s.v );
			++i;
		}
		for ( Skid& k : m_skids )
		{
			k.age += dt;
		}
	}

	void Draw() const
	{
		for ( const Skid& k : m_skids )
		{
			float a = 0.8f * ClampFloat( 1.0f - k.age / 25.0f, 0.0f, 1.0f );
			if ( a > 0.0f )
			{
				DrawLineEx( k.a, k.b, Color( Palette::LineDim, a ), k.width, OVERLAY_THICKNESS_WORLD, OVERLAY_OCCLUSION_HIDE );
			}
		}
		for ( const Spark& s : m_sparks )
		{
			float a = ClampFloat( s.life / s.max, 0.0f, 1.0f );
			Vec4 c = Color( s.force ? Palette::Force : Palette::Warning, a );
			DrawLineEx( s.p, MulAdd( s.p, -0.035f, s.v ), c, Style::LinePx, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
		}
	}

	int SkidCount() const
	{
		return (int)m_skids.size();
	}

private:
	struct Spark
	{
		Vec3 p, v;
		float life, max;
		bool force;
	};
	struct Skid
	{
		Vec3 a, b;
		float width, age;
	};
	static constexpr int kMaxSkids = 4000;

	void AddSpark( Vec3 p, Vec3 v, float life, bool force )
	{
		if ( m_sparks.size() < 3000 )
		{
			m_sparks.push_back( { p, v, life, life, force } );
		}
	}

	std::vector<Spark> m_sparks;
	std::vector<Skid> m_skids;
	int m_skidHead = 0;
	Rng m_rng{ 99u };
};

class Footfall
{
public:
	static constexpr int kHistory = 240; 
	static constexpr int kMaxLegs = 8;

	void Clear()
	{
		memset( m_hist, 0, sizeof( m_hist ) );
		m_head = 0;
		m_acc = 0.0f;
	}

	void Record( const float* p, const MechInfo& info, float dt )
	{
		m_legs = info.legs < kMaxLegs ? info.legs : kMaxLegs;
		m_acc += dt;
		while ( m_acc >= 1.0f / 60.0f )
		{
			m_acc -= 1.0f / 60.0f;
			int stride = info.LegStride();
			for ( int l = 0; l < m_legs; ++l )
			{
				m_hist[l][m_head] = p[kHeaderFloats + l * stride + info.jointsPerLeg * 3 + 3] > 0.5f ? 1 : 0;
			}
			m_head = ( m_head + 1 ) % kHistory;
		}
	}

	void DrawImGui( float rowHeight ) const
	{
		ImDrawList* dl = ImGui::GetWindowDrawList();
		ImVec2 o = ImGui::GetCursorScreenPos();
		float labelW = ImGui::CalcTextSize( "L00" ).x + 6.0f;
		float w = ImGui::GetContentRegionAvail().x - labelW;
		if ( w < 40.0f )
		{
			w = 40.0f;
		}
		float cw = w / (float)kHistory;
		ImU32 stance = HexToImU32( Palette::Line );
		ImU32 lane = HexToImU32( Palette::LineDim, 70 );
		ImU32 label = HexToImU32( Palette::TextMuted );
		for ( int l = 0; l < m_legs; ++l )
		{
			float y = o.y + (float)l * rowHeight;
			char lab[8];
			snprintf( lab, sizeof( lab ), "%c%d", ( l & 1 ) ? 'R' : 'L', l / 2 );
			dl->AddText( ImVec2( o.x, y + 0.5f * ( rowHeight - ImGui::GetFontSize() ) ), label, lab );
			float x0 = o.x + labelW;
			dl->AddRectFilled( ImVec2( x0, y + 1.0f ), ImVec2( x0 + w, y + rowHeight - 1.0f ), lane );
			int run = -1;
			for ( int c = 0; c <= kHistory; ++c )
			{
				bool on = false;
				if ( c < kHistory )
				{
					int idx = ( m_head + c ) % kHistory;
					on = m_hist[l][idx] != 0;
				}
				if ( on && run < 0 )
				{
					run = c;
				}
				else if ( on == false && run >= 0 )
				{
					dl->AddRectFilled( ImVec2( x0 + (float)run * cw, y + 2.0f ), ImVec2( x0 + (float)c * cw, y + rowHeight - 2.0f ),
									   stance );
					run = -1;
				}
			}
		}
		ImGui::Dummy( ImVec2( labelW + w, (float)m_legs * rowHeight + 2.0f ) );
	}

private:
	unsigned char m_hist[kMaxLegs][kHistory] = {};
	int m_head = 0;
	int m_legs = 0;
	float m_acc = 0.0f;
};

} 
