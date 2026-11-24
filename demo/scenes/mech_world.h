#pragma once

#include "math3d.h"
#include "random.h"
#include "scene.h"

#include <math.h>
#include <string.h>
#include <vector>

namespace mech
{

enum BoxLook
{
	LookStructure = 0, 
	LookRock,		   
	LookMarker,		   
	LookSlate,		   
	LookLight,		   
	LookSand,		   
	LookEarth,		   
	LookPark,		   
};

enum TerrainMode
{
	TerrainNone = 0,
	TerrainArena,
	TerrainFrontline
};

struct Crater
{
	float x, z, r, depth;
};

struct Box
{
	Vec3 c;		
	Vec3 h;		
	Vec3 ax[3]; 
	Quat q;		
	Vec3 bmin, bmax;
	int look;
};

struct Gate
{
	Vec3 p;
	float yaw; 
	float r;   

	std::vector<Vec3> via;
	bool viaWalkersOnly = false;
};

struct Course
{
	const char* name;
	const char* label; 
	const char* desc;
	std::vector<Gate> gates; 
	std::vector<AABB> crouchZones;
};

inline float Smooth01( float t )
{
	t = ClampFloat( t, 0.0f, 1.0f );
	return t * t * ( 3.0f - 2.0f * t );
}

inline Quat QuatFromAxes( Vec3 x, Vec3 y, Vec3 z )
{
	Quat q;
	float tr = x.x + y.y + z.z;
	if ( tr > 0.0f )
	{
		float s = sqrtf( tr + 1.0f ) * 2.0f;
		q.s = 0.25f * s;
		q.v = { ( y.z - z.y ) / s, ( z.x - x.z ) / s, ( x.y - y.x ) / s };
	}
	else if ( x.x > y.y && x.x > z.z )
	{
		float s = sqrtf( 1.0f + x.x - y.y - z.z ) * 2.0f;
		q.s = ( y.z - z.y ) / s;
		q.v = { 0.25f * s, ( y.x + x.y ) / s, ( z.x + x.z ) / s };
	}
	else if ( y.y > z.z )
	{
		float s = sqrtf( 1.0f + y.y - x.x - z.z ) * 2.0f;
		q.s = ( z.x - x.z ) / s;
		q.v = { ( y.x + x.y ) / s, 0.25f * s, ( z.y + y.z ) / s };
	}
	else
	{
		float s = sqrtf( 1.0f + z.z - x.x - y.y ) * 2.0f;
		q.s = ( x.y - y.x ) / s;
		q.v = { ( z.x + x.z ) / s, ( z.y + y.z ) / s, 0.25f * s };
	}
	return NormalizeQuat( q );
}

struct GridCell
{
	int offset = 0;
	int count = 0;
};

class World
{
public:
	std::vector<Box> boxes;
	std::vector<GridCell> gridCells;
	std::vector<int> gridIndices;
	float gridMinX = 0.0f, gridMinZ = 0.0f, gridMaxX = 0.0f, gridMaxZ = 0.0f;
	float gridCellSize = 6.0f, gridInvCell = 1.0f / 6.0f;
	int gridNx = 0, gridNz = 0;
	std::vector<Course> courses;
	bool hills = false; 
	bool pit = false;	
	int terrainMode = TerrainNone;
	uint32_t terrainSeed = 1u;
	float terrainMaxY = 6.0f;
	float groundPlaneY = -9.0f;
	Vec3 spawnPos = { 0.0f, 0.0f, 0.0f };
	float spawnYaw = 0.0f;
	std::vector<Vec3> aiHomes;
	std::vector<Crater> craters;
	float phaseA = 0.0f;
	float phaseB = 0.0f;
	float extent = 250.0f;
	const char* name = "";

	static float HillWeight( float z )
	{
		return Smooth01( ( -z - 40.0f ) / 25.0f );
	}

	float Height( float x, float z ) const
	{
		float h = 0.0f;
		if ( hills )
		{
			float w = HillWeight( z );
			if ( w > 0.0f )
			{
				float hh = 2.2f * sinf( x * 0.09f ) * cosf( z * 0.07f ) + 1.3f * sinf( x * 0.21f + 1.3f ) * sinf( z * 0.17f ) +
						   0.35f * sinf( x * 0.55f + z * 0.43f ) + 1.6f;
				h += w * hh;
			}
		}
		if ( pit )
		{
			float dx = fabsf( x ) - 12.0f, dz = fabsf( z - 93.0f ) - 7.0f;
			float outside = dx > dz ? dx : dz;
			if ( outside < 1.2f )
			{
				h -= 7.0f * Smooth01( ( 1.2f - outside ) / 1.2f );
			}
		}
		if ( terrainMode == TerrainFrontline )
		{
			float swell = 1.3f * sinf( x * 0.042f + phaseA ) * cosf( z * 0.038f + phaseB ) +
						  0.75f * sinf( x * 0.11f - z * 0.09f + phaseB ) + 1.0f;
			float ridge = 3.8f * Smooth01( ( z - 34.0f ) / 28.0f );
			float tz = -8.0f + 5.0f * sinf( x * 0.055f + phaseA );
			float trenchMask = ( 1.0f - Smooth01( ( fabsf( z - tz ) - 2.6f ) / 2.2f ) ) * ( 1.0f - Smooth01( ( fabsf( x ) - 52.0f ) / 12.0f ) );
			float sx = x, sz = z + 34.0f;
			float spMask = Smooth01( ( sqrtf( sx * sx + sz * sz ) - 10.0f ) / 14.0f );
			h = swell * spMask + ridge - 1.85f * trenchMask;
			for ( const Crater& c : craters )
			{
				float dx = x - c.x, dz = z - c.z;
				float d2 = dx * dx + dz * dz;
				float maxR = c.r * 1.35f;
				if ( d2 < maxR * maxR )
				{
					float u = sqrtf( d2 ) / c.r;
					float bowl = u < 1.0f ? ( 1.0f - u * u ) : 0.0f;
					float lip = expf( -( u - 1.05f ) * ( u - 1.05f ) * 16.0f );
					h += -c.depth * bowl * bowl + 0.25f * c.depth * lip;
				}
			}
		}
		return h;
	}

	Vec3 TerrainNormal( float x, float z ) const
	{
		const float e = 0.1f;
		float hx = Height( x + e, z ) - Height( x - e, z );
		float hz = Height( x, z + e ) - Height( x, z - e );
		return Normalize( Vec3{ -hx, 2.0f * e, -hz } );
	}

	bool HasTerrain() const
	{
		return hills || pit || terrainMode != TerrainNone;
	}

	bool Raycast( Vec3 o, Vec3 d, float maxT, float* tOut, Vec3* pOut, Vec3* nOut, int* surface ) const
	{
		float best = maxT, t;
		bool hit = false;
		int sid = 0;
		Vec3 bn = { 0.0f, 1.0f, 0.0f }, n;
		if ( TerrainRay( o, d, best, &t ) )
		{
			best = t;
			hit = true;
			Vec3 p = MulAdd( o, t, d );
			bn = TerrainNormal( p.x, p.z );
		}
		if ( gridCells.empty() )
		{
			for ( size_t i = 0; i < boxes.size(); ++i )
			{
				if ( RayBox( boxes[i], o, d, best, &t, &n ) )
				{
					best = t;
					bn = n;
					hit = true;
					sid = (int)i + 1;
				}
			}
		}
		else
		{
			float tStart = 0.0f;
			float tEnd = best;
			const float oXZ[2] = { o.x, o.z };
			const float dXZ[2] = { d.x, d.z };
			const float gMin[2] = { gridMinX, gridMinZ };
			const float gMax[2] = { gridMaxX, gridMaxZ };
			bool skipBoxes = false;
			for ( int a = 0; a < 2; ++a )
			{
				if ( fabsf( dXZ[a] ) < 1e-8f )
				{
					if ( oXZ[a] < gMin[a] || oXZ[a] > gMax[a] )
					{
						skipBoxes = true;
						break;
					}
				}
				else
				{
					float inv = 1.0f / dXZ[a];
					float t1 = ( gMin[a] - oXZ[a] ) * inv;
					float t2 = ( gMax[a] - oXZ[a] ) * inv;
					if ( t1 > t2 )
					{
						float tmp = t1;
						t1 = t2;
						t2 = tmp;
					}
					tStart = MaxFloat( tStart, t1 );
					tEnd = MinFloat( tEnd, t2 );
					if ( tStart > tEnd )
					{
						skipBoxes = true;
						break;
					}
				}
			}
			if ( skipBoxes == false )
			{
				auto testCell = [&]( int cx, int cz ) {
					const GridCell& gc = gridCells[(size_t)cz * (size_t)gridNx + (size_t)cx];
					const int* idx = gridIndices.data() + gc.offset;
					for ( int k = 0; k < gc.count; ++k )
					{
						int bi = idx[k];
						if ( RayBox( boxes[bi], o, d, best, &t, &n ) )
						{
							if ( hit == false || t < best || ( t == best && ( sid == 0 || bi + 1 < sid ) ) )
							{
								best = t;
								bn = n;
								hit = true;
								sid = bi + 1;
							}
						}
					}
				};
				float sx = o.x + d.x * tStart;
				float sz = o.z + d.z * tStart;
				int cx = ClampInt( (int)floorf( ( sx - gridMinX ) * gridInvCell ), 0, gridNx - 1 );
				int cz = ClampInt( (int)floorf( ( sz - gridMinZ ) * gridInvCell ), 0, gridNz - 1 );
				if ( fabsf( d.x ) < 1e-8f && fabsf( d.z ) < 1e-8f )
				{
					testCell( cx, cz );
				}
				else
				{
					int stepX = d.x > 1e-8f ? 1 : ( d.x < -1e-8f ? -1 : 0 );
					int stepZ = d.z > 1e-8f ? 1 : ( d.z < -1e-8f ? -1 : 0 );
					float tDeltaX = stepX != 0 ? ( gridCellSize / fabsf( d.x ) ) : 1e30f;
					float tDeltaZ = stepZ != 0 ? ( gridCellSize / fabsf( d.z ) ) : 1e30f;
					float nextX = stepX > 0 ? ( gridMinX + (float)( cx + 1 ) * gridCellSize )
											: ( gridMinX + (float)cx * gridCellSize );
					float nextZ = stepZ > 0 ? ( gridMinZ + (float)( cz + 1 ) * gridCellSize )
											: ( gridMinZ + (float)cz * gridCellSize );
					float tMaxX = stepX != 0 ? MaxFloat( tStart, ( nextX - o.x ) / d.x ) : 1e30f;
					float tMaxZ = stepZ != 0 ? MaxFloat( tStart, ( nextZ - o.z ) / d.z ) : 1e30f;
					int maxSteps = gridNx + gridNz + 4;
					for ( int s = 0; s < maxSteps; ++s )
					{
						testCell( cx, cz );
						float tNext = tMaxX < tMaxZ ? tMaxX : tMaxZ;
						if ( tNext > best || tNext > tEnd )
						{
							break;
						}
						if ( tMaxX <= tMaxZ )
						{
							cx += stepX;
							tMaxX += tDeltaX;
						}
						else
						{
							cz += stepZ;
							tMaxZ += tDeltaZ;
						}
						if ( cx < 0 || cx >= gridNx || cz < 0 || cz >= gridNz )
						{
							break;
						}
					}
				}
			}
		}
		if ( hit == false )
		{
			return false;
		}
		*tOut = best;
		*pOut = MulAdd( o, best, d );
		*nOut = bn;
		if ( surface )
		{
			*surface = sid;
		}
		return true;
	}

	static void PushOutOfBox( const Box& b, Vec3* c, float r, Vec3* nn, bool* touched )
	{
		if ( c->x + r < b.bmin.x || c->x - r > b.bmax.x || c->y + r < b.bmin.y || c->y - r > b.bmax.y ||
			 c->z + r < b.bmin.z || c->z - r > b.bmax.z )
		{
			return;
		}
		Vec3 rel = Sub( *c, b.c );
		float l[3] = { Dot( rel, b.ax[0] ), Dot( rel, b.ax[1] ), Dot( rel, b.ax[2] ) };
		float h[3] = { b.h.x, b.h.y, b.h.z };
		float q[3];
		bool inside = true;
		for ( int k = 0; k < 3; ++k )
		{
			q[k] = ClampFloat( l[k], -h[k], h[k] );
			if ( q[k] != l[k] )
			{
				inside = false;
			}
		}
		if ( inside )
		{
			int bestAxis = 0;
			float pen = 1e30f, s = 1.0f;
			for ( int k = 0; k < 3; ++k )
			{
				float p1 = h[k] - l[k], p2 = l[k] + h[k];
				if ( p1 < pen )
				{
					pen = p1;
					bestAxis = k;
					s = 1.0f;
				}
				if ( p2 < pen )
				{
					pen = p2;
					bestAxis = k;
					s = -1.0f;
				}
			}
			Vec3 n = MulSV( s, b.ax[bestAxis] );
			*c = MulAdd( *c, pen + r, n );
			*nn = n;
			*touched = true;
			return;
		}
		Vec3 cp = b.c + q[0] * b.ax[0] + q[1] * b.ax[1] + q[2] * b.ax[2];
		Vec3 dv = Sub( *c, cp );
		float dl = Length( dv );
		if ( dl < r && dl > 1e-6f )
		{
			Vec3 n = MulSV( 1.0f / dl, dv );
			*c = MulAdd( *c, r - dl, n );
			*nn = n;
			*touched = true;
		}
	}

	bool SpherePush( Vec3* c, float r, Vec3* nOut ) const
	{
		bool touched = false;
		Vec3 nn = { 0.0f, 1.0f, 0.0f };
		float th = Height( c->x, c->z );
		if ( c->y - r < th )
		{
			nn = TerrainNormal( c->x, c->z );
			c->y = th + r;
			touched = true;
		}
		if ( gridCells.empty() )
		{
			for ( const Box& b : boxes )
			{
				PushOutOfBox( b, c, r, &nn, &touched );
			}
		}
		else
		{
			float qr = r + 0.5f;
			if ( c->x + qr >= gridMinX && c->x - qr <= gridMaxX && c->z + qr >= gridMinZ && c->z - qr <= gridMaxZ )
			{
				int x0 = ClampInt( (int)floorf( ( c->x - qr - gridMinX ) * gridInvCell ), 0, gridNx - 1 );
				int x1 = ClampInt( (int)floorf( ( c->x + qr - gridMinX ) * gridInvCell ), 0, gridNx - 1 );
				int z0 = ClampInt( (int)floorf( ( c->z - qr - gridMinZ ) * gridInvCell ), 0, gridNz - 1 );
				int z1 = ClampInt( (int)floorf( ( c->z + qr - gridMinZ ) * gridInvCell ), 0, gridNz - 1 );
				int seen[128];
				int nSeen = 0;
				for ( int z = z0; z <= z1; ++z )
				{
					for ( int x = x0; x <= x1; ++x )
					{
						const GridCell& gc = gridCells[(size_t)z * (size_t)gridNx + (size_t)x];
						const int* idx = gridIndices.data() + gc.offset;
						for ( int k = 0; k < gc.count; ++k )
						{
							int bi = idx[k];
							bool dup = false;
							for ( int s = 0; s < nSeen; ++s )
							{
								if ( seen[s] == bi )
								{
									dup = true;
									break;
								}
							}
							if ( dup )
							{
								continue;
							}
							if ( nSeen < 128 )
							{
								seen[nSeen++] = bi;
							}
							PushOutOfBox( boxes[bi], c, r, &nn, &touched );
						}
					}
				}
			}
		}
		if ( nOut )
		{
			*nOut = nn;
		}
		return touched;
	}

	void BuildGrid( float cellSize = 6.0f )
	{
		gridCells.clear();
		gridIndices.clear();
		gridNx = 0;
		gridNz = 0;
		if ( (int)boxes.size() < 32 )
		{
			return;
		}
		float minX = boxes[0].bmin.x, maxX = boxes[0].bmax.x;
		float minZ = boxes[0].bmin.z, maxZ = boxes[0].bmax.z;
		for ( size_t i = 1; i < boxes.size(); ++i )
		{
			minX = MinFloat( minX, boxes[i].bmin.x );
			maxX = MaxFloat( maxX, boxes[i].bmax.x );
			minZ = MinFloat( minZ, boxes[i].bmin.z );
			maxZ = MaxFloat( maxZ, boxes[i].bmax.z );
		}
		minX -= 1.0f;
		minZ -= 1.0f;
		maxX += 1.0f;
		maxZ += 1.0f;

		gridCellSize = MaxFloat( cellSize, MaxFloat( ( maxX - minX ) / 128.0f, ( maxZ - minZ ) / 128.0f ) );
		gridInvCell = 1.0f / gridCellSize;
		gridMinX = minX;
		gridMinZ = minZ;
		gridNx = MaxInt( 1, (int)ceilf( ( maxX - minX ) * gridInvCell ) );
		gridNz = MaxInt( 1, (int)ceilf( ( maxZ - minZ ) * gridInvCell ) );
		gridMaxX = gridMinX + (float)gridNx * gridCellSize;
		gridMaxZ = gridMinZ + (float)gridNz * gridCellSize;

		int numCells = gridNx * gridNz;
		gridCells.assign( (size_t)numCells, GridCell{} );

		auto cellRange = [&]( const Box& b, int* x0, int* x1, int* z0, int* z1 ) {
			*x0 = ClampInt( (int)floorf( ( b.bmin.x - 0.01f - gridMinX ) * gridInvCell ), 0, gridNx - 1 );
			*x1 = ClampInt( (int)floorf( ( b.bmax.x + 0.01f - gridMinX ) * gridInvCell ), 0, gridNx - 1 );
			*z0 = ClampInt( (int)floorf( ( b.bmin.z - 0.01f - gridMinZ ) * gridInvCell ), 0, gridNz - 1 );
			*z1 = ClampInt( (int)floorf( ( b.bmax.z + 0.01f - gridMinZ ) * gridInvCell ), 0, gridNz - 1 );
		};

		for ( size_t i = 0; i < boxes.size(); ++i )
		{
			int x0, x1, z0, z1;
			cellRange( boxes[i], &x0, &x1, &z0, &z1 );
			for ( int z = z0; z <= z1; ++z )
			{
				for ( int x = x0; x <= x1; ++x )
				{
					gridCells[(size_t)z * (size_t)gridNx + (size_t)x].count += 1;
				}
			}
		}

		int totalRefs = 0;
		std::vector<int> writeCursor( (size_t)numCells );
		for ( int c = 0; c < numCells; ++c )
		{
			gridCells[c].offset = totalRefs;
			writeCursor[c] = totalRefs;
			totalRefs += gridCells[c].count;
		}
		gridIndices.resize( (size_t)totalRefs );

		for ( int i = 0; i < (int)boxes.size(); ++i )
		{
			int x0, x1, z0, z1;
			cellRange( boxes[i], &x0, &x1, &z0, &z1 );
			for ( int z = z0; z <= z1; ++z )
			{
				for ( int x = x0; x <= x1; ++x )
				{
					int c = z * gridNx + x;
					gridIndices[writeCursor[c]++] = i;
				}
			}
		}
	}

	float GroundY( float x, float z ) const
	{
		float t;
		Vec3 p, n;
		if ( Raycast( { x, 60.0f, z }, { 0.0f, -1.0f, 0.0f }, 100.0f, &t, &p, &n, nullptr ) )
		{
			return p.y;
		}
		return Height( x, z );
	}

	void Clear()
	{
		boxes.clear();
		gridCells.clear();
		gridIndices.clear();
		gridMinX = 0.0f;
		gridMinZ = 0.0f;
		gridMaxX = 0.0f;
		gridMaxZ = 0.0f;
		gridCellSize = 6.0f;
		gridInvCell = 1.0f / 6.0f;
		gridNx = 0;
		gridNz = 0;
		courses.clear();
		hills = false;
		pit = false;
		terrainMode = TerrainNone;
		terrainSeed = 1u;
		terrainMaxY = 6.0f;
		groundPlaneY = -9.0f;
		spawnPos = { 0.0f, 0.0f, 0.0f };
		spawnYaw = 0.0f;
		aiHomes.clear();
		craters.clear();
		phaseA = 0.0f;
		phaseB = 0.0f;
	}

	void BuildArena()
	{
		Clear();
		name = "arena";
		terrainMode = TerrainArena;
		terrainMaxY = 6.0f;
		groundPlaneY = -9.0f;
		hills = true;
		pit = true;
		extent = 250.0f;
		Rng rng( 1234567u );

		AddAabb( -8, 0, -8, 8, 0.12f, 8, LookStructure );
		const float pylons[4][2] = { { 6, 14 }, { 14, -6 }, { -14, 6 }, { -6, -14 } };
		for ( auto& p : pylons )
		{
			AddAabb( p[0] - 0.4f, 0, p[1] - 0.4f, p[0] + 0.4f, 6, p[1] + 0.4f, LookMarker );
		}

		for ( int i = 0; i < 5; ++i )
		{
			float h = 0.5f * (float)( i + 1 );
			AddAabb( 16 + i * 3.0f, 0, -24, 18.6f + i * 3.0f, h, -18, LookStructure );
		}
		for ( int i = 0; i < 4; ++i )
		{
			float deg = 10.0f + 10.0f * (float)i;
			float len = 6.0f;
			float rise = len * tanf( deg * MATH_DEG_TO_RAD );
			float x = -16.0f - (float)i * 5.0f;
			AddRamp( x, 0, -20, x, rise, -20 - len, 3.5f, LookStructure );
			AddAabb( x - 1.75f, 0, -20 - len - 4, x + 1.75f, rise, -20 - len + 0.1f, LookStructure );
		}
		AddAabb( -26, 0, 18, -24, 5, 22, LookStructure );
		AddAabb( -16, 0, 18, -14, 5, 22, LookStructure );
		AddAabb( -26, 5, 18, -14, 5.8f, 22, LookStructure );
		AddAabb( 20, 0, 20, 30, 12, 22, LookStructure );

		{
			AddStairs( 0, 30, 0, 10, 12, 0.25f, 0.5f, 0 );
			AddAabb( -5, 0, 36, 5, 3, 44, LookStructure );
			for ( int i = 0; i < 5; ++i )
			{
				float hz = 45.2f + (float)i * 2.0f;
				AddAabb( -2.15f, 0, hz - 0.75f, -0.65f, 3.0f + rng.Range( -0.15f, 0.15f ), hz + 0.75f, LookStructure );
				hz += 1.0f;
				AddAabb( 0.65f, 0, hz - 0.75f, 2.15f, 3.0f + rng.Range( -0.15f, 0.15f ), hz + 0.75f, LookStructure );
			}
			AddAabb( -5, 0, 55.8f, 5, 3, 62, LookStructure );
			AddRamp( 0, 3, 62, 0, 0, 71, 10, LookStructure );
			for ( int i = 0; i < 70; ++i ) 
			{
				float x = rng.Range( -8.0f, 8.0f ), z = rng.Range( 72.0f, 84.0f );
				float sx = rng.Range( 0.25f, 0.9f ), sz = rng.Range( 0.25f, 0.9f ), sy = rng.Range( 0.08f, 0.38f );
				AddRot( { x, sy * 0.5f, z }, { sx, sy, sz }, rng.Range( 0.0f, 3.14f ), rng.Range( -0.25f, 0.25f ), LookRock );
			}
			AddAabb( -4.45f, -8, 84.0f, -3.55f, 0, 102.0f, LookStructure ); 
			AddAabb( 0.5f, -8, 84.0f, 6.5f, 0, 102.0f, LookStructure );	 
			AddRamp( 9.5f, -7, 90, 9.5f, 0, 101.5f, 5, LookStructure );	 
			AddAabb( -3.4f, 0, 106, -2.6f, 3.6f, 120, LookStructure );		 
			AddAabb( 2.6f, 0, 106, 3.4f, 3.6f, 120, LookStructure );
			AddAabb( -3.4f, 3.0f, 106, 3.4f, 3.6f, 120, LookStructure );
			AddRamp( 0, 0, 124, 0, 3.6f, 132, 10, LookStructure ); 
			AddAabb( -5, 0, 132, 5, 3.6f, 137, LookStructure );
			AddRamp( 0, 3.6f, 137, 0, 0, 141, 10, LookStructure );

			Course c;
			c.name = "A  Agility Run";
			c.label = "A  Agility";
			c.desc = "Stairs, stepping stones, rubble, pit beam, crouch tunnel, steep hill.";
			AddGate( c, 0, 0, 22, 0, 4.5f );
			AddGate( c, 0, 3, 40, 0, 4.5f );
			AddGate( c, 0, 3, 59, 0, 4.5f );
			AddGate( c, 0, 0, 78, 0, 5.0f );
			AddGate( c, 0, 0, 104, 0, 5.0f );
			c.gates.back().via = { { 3.5f, 0.0f, 83.0f }, { 3.5f, 0.0f, 101.5f } }; 
			AddGate( c, 0, 0, 113, 0, 2.6f );
			AddGate( c, 0, 3.6f, 134.5f, 0, 5.0f );
			AddGate( c, 0, 0, 148, 0, 5.0f );
			c.crouchZones.push_back( { { -3.6f, -1.0f, 103.0f }, { 3.6f, 4.0f, 122.0f } } );
			courses.push_back( c );
		}

		{
			AddAabb( 32, 0, -6, 38, 1.0f, 6, LookStructure );
			AddAabb( 38, 0, -6, 44, 2.2f, 6, LookStructure );
			AddAabb( 44, 0, -14.0f, 50, 3.6f, 6, LookStructure );

			AddRamp( 50, 3.6f, -9.0f, 58, 7.0f, -9.0f, 8, LookStructure );
			AddAabb( 58, 0, -14.0f, 68, 7.0f, 6, LookStructure );
			AddAabb( 68, 6.5f, -1.25f, 88, 7.0f, 1.25f, LookStructure );
			for ( int i = 0; i < 3; ++i )
			{
				float x = 73.0f + (float)i * 6.0f;
				AddAabb( x - 0.3f, 0, -0.3f, x + 0.3f, 6.5f, 0.3f, LookStructure );
			}
			AddRamp( 88, 7.0f, 0, 100, 0, 0, 4, LookStructure );

			Course c;
			c.name = "B  Vertical";
			c.label = "B  Vertical";
			c.desc = "Ledges to mantle, a 7 m tower (wire anchors or the ramp), a sky bridge.";
			const float east = 0.5f * MATH_PI;
			AddGate( c, 26, 0, 0, east, 5.0f );
			AddGate( c, 47, 3.6f, 0, east, 4.5f );
			AddGate( c, 63, 7.0f, 0, east, 5.0f );
			c.gates.back().via = { { 46.0f, 3.6f, -9.0f }, { 60.0f, 7.0f, -9.0f } }; 
			c.gates.back().viaWalkersOnly = true;
			AddGate( c, 78, 7.0f, 0, east, 3.5f );
			AddGate( c, 106, 0, 0, east, 5.0f );
			courses.push_back( c );
		}

		{
			const float cx = -80.0f, rx = 30.0f, rz = 18.0f;
			for ( int ring = 0; ring < 2; ++ring )
			{
				float ex = ring ? rx + 6.0f : rx - 6.0f, ez = ring ? rz + 6.0f : rz - 6.0f;
				int n = ring ? 44 : 28;
				for ( int i = 0; i < n; ++i )
				{
					float t = (float)i / (float)n * 2.0f * MATH_PI;
					AddRot( { cx + ex * cosf( t ), 0.35f, ez * sinf( t ) }, { 0.22f, 0.35f, 0.22f }, t, 0, LookMarker );
				}
			}
			Course c;
			c.name = "C  Drift Ring";
			c.label = "C  Drift";
			c.desc = "One lap of the coned oval. Tap or hold X at speed to slide.";
			for ( int k = 0; k <= 8; ++k )
			{
				float t = (float)( k % 8 ) / 8.0f * 2.0f * MATH_PI;
				float tx = -rx * sinf( t ), tz = rz * cosf( t );
				AddGate( c, cx + rx * cosf( t ), 0, rz * sinf( t ), atan2f( tx, tz ), 6.0f );
			}
			courses.push_back( c );
		}

		{
			for ( int i = 0; i < 45; ++i )
			{
				float x = rng.Range( -45.0f, 45.0f ), z = rng.Range( -180.0f, -55.0f );
				float s = rng.Range( 0.5f, 2.2f );
				AddRot( { x, Height( x, z ) + s * 0.25f, z }, { s, s * 0.6f, s * 0.8f }, rng.Range( 0.0f, 3.14f ),
						rng.Range( -0.4f, 0.4f ), LookRock );
			}
			Course c;
			c.name = "D  Hill Climb";
			c.label = "D  Hills";
			c.desc = "Rolling terrain and boulders: foot placement on uneven ground.";
			const float gp[6][2] = { { 0, -30 }, { 18, -70 }, { -14, -100 }, { 12, -126 }, { -20, -152 }, { 0, -176 } };
			for ( int i = 0; i < 6; ++i )
			{
				float yaw = MATH_PI;
				if ( i < 5 )
				{
					yaw = atan2f( gp[i + 1][0] - gp[i][0], gp[i + 1][1] - gp[i][1] );
				}
				AddGate( c, gp[i][0], Height( gp[i][0], gp[i][1] ), gp[i][1], yaw, 5.5f );
			}
			courses.push_back( c );
		}
	}

	enum LabSurface
	{
		LabFlat = 0,
		LabStairs,
		LabRubble,
		LabSlopes,
		LabLedges,
		LabSurfaceCount
	};

	void BuildLab( int surface, uint64_t seed )
	{
		Clear();
		name = "lab";
		groundPlaneY = 0.0f;
		extent = 420.0f;
		Rng rng( seed );
		const float x0 = -6.0f, x1 = 6.0f;
		switch ( surface )
		{
			case LabStairs:
				for ( int k = 0; k < 4; ++k ) 
				{
					float z = 10.0f + (float)k * 26.0f;
					float rise = 0.2f + 0.05f * (float)k;
					AddStairs( 0, z, 0, x1 - x0, 6, rise, 0.6f, 0 );
					AddAabb( x0, 0, z + 3.6f, x1, rise * 6.0f, z + 8.0f, LookStructure );
					AddStairs( 0, z + 11.6f, MATH_PI, x1 - x0, 6, rise, 0.6f, 0 );
				}
				break;
			case LabRubble:
				for ( int i = 0; i < 260; ++i )
				{
					float x = rng.Range( x0, x1 ), z = rng.Range( 8.0f, 120.0f );
					float sx = rng.Range( 0.2f, 0.8f ), sz = rng.Range( 0.2f, 0.8f ), sy = rng.Range( 0.08f, 0.45f );
					AddRot( { x, sy * 0.5f, z }, { sx, sy, sz }, rng.Range( 0.0f, 3.14f ), rng.Range( -0.25f, 0.25f ), LookRock );
				}
				break;
			case LabSlopes:
				for ( int k = 0; k < 4; ++k ) 
				{
					float z = 10.0f + (float)k * 28.0f;
					float deg = 10.0f + 8.0f * (float)k;
					float rise = 8.0f * tanf( deg * MATH_DEG_TO_RAD );
					AddRamp( 0, 0, z, 0, rise, z + 8.0f, x1 - x0, LookStructure );
					AddAabb( x0, 0, z + 8.0f, x1, rise, z + 12.0f, LookStructure );
					AddRamp( 0, rise, z + 12.0f, 0, 0, z + 20.0f, x1 - x0, LookStructure );
				}
				break;
			case LabLedges:
				for ( int k = 0; k < 5; ++k ) 
				{
					float z = 12.0f + (float)k * 22.0f;
					float h = 0.5f + 0.4f * (float)k;
					AddAabb( x0, 0, z, x1, h, z + 8.0f, LookStructure );
				}
				break;
			default:
				break;
		}

		for ( int i = 0; i <= 40; ++i ) 
		{
			float z = (float)i * 10.0f;
			AddAabb( x0 - 1.0f, 0, z - 0.15f, x0 - 0.7f, 0.5f, z + 0.15f, LookMarker );
			AddAabb( x1 + 0.7f, 0, z - 0.15f, x1 + 1.0f, 0.5f, z + 0.15f, LookMarker );
		}
	}

	static Material BoxMaterial( int look )
	{
		switch ( look )
		{
			case LookRock:
				return Materials::Terrain( Palette::TerrainRock );
			case LookMarker:
			case LookSlate:
				return Materials::Sleeping();
			case LookLight:
				return Materials::KinematicIdle();
			case LookSand:
				return Materials::Terrain( Palette::TerrainSand );
			case LookEarth:
				return Materials::Terrain( Palette::TerrainDirt );
			case LookPark:
				return Materials::Terrain( Palette::TerrainForest );
			default:
				return Materials::Static();
		}
	}

	void DrawBoxes() const
	{
		for ( const Box& b : boxes )
		{
			DrawBox( Transform{ b.c, b.q }, MulSV( 2.0f, b.h ), BoxMaterial( b.look ) );
		}
	}

	Box* AddRot( Vec3 c, Vec3 h, float yaw, float pitch, int look )
	{
		Box b;
		float cy = cosf( yaw ), sy = sinf( yaw ), cp = cosf( pitch ), sp = sinf( pitch );
		Vec3 X = { cy, 0.0f, -sy }, Y = { 0.0f, 1.0f, 0.0f }, Z = { sy, 0.0f, cy };
		b.ax[0] = X;
		b.ax[1] = cp * Y + sp * Z;
		b.ax[2] = cp * Z - sp * Y;
		b.c = c;
		b.h = h;
		b.look = look;
		Finish( &b );
		boxes.push_back( b );
		return &boxes.back();
	}

	void AddAabb( float x0, float y0, float z0, float x1, float y1, float z1, int look )
	{
		AddRot( { ( x0 + x1 ) * 0.5f, ( y0 + y1 ) * 0.5f, ( z0 + z1 ) * 0.5f },
				{ fabsf( x1 - x0 ) * 0.5f, fabsf( y1 - y0 ) * 0.5f, fabsf( z1 - z0 ) * 0.5f }, 0.0f, 0.0f, look );
	}

	void AddRamp( float x0, float y0, float z0, float x1, float y1, float z1, float width, int look )
	{
		float dx = x1 - x0, dz = z1 - z0;
		float lh = sqrtf( dx * dx + dz * dz );
		float rise = y1 - y0;
		float yaw = atan2f( dx, dz );
		float pitch = -atan2f( rise, lh );
		float len = sqrtf( lh * lh + rise * rise ) + 0.6f;
		const float thick = 0.6f;
		Vec3 mid = { ( x0 + x1 ) * 0.5f, ( y0 + y1 ) * 0.5f, ( z0 + z1 ) * 0.5f };
		Box* b = AddRot( mid, { width * 0.5f, thick * 0.5f, len * 0.5f }, yaw, pitch, look );
		b->c = MulAdd( mid, -thick * 0.5f, b->ax[1] );
		Finish( b );
	}

	void AddStairs( float x, float z, float yaw, float width, int n, float rise, float run, float base )
	{
		Vec3 dir = { sinf( yaw ), 0.0f, cosf( yaw ) };
		for ( int i = 0; i < n; ++i )
		{
			float len = (float)( n - i ) * run;
			float hgt = rise * (float)( i + 1 );
			Vec3 c = MulAdd( Vec3{ x, base + hgt * 0.5f, z }, (float)i * run + len * 0.5f, dir );
			AddRot( c, { width * 0.5f, hgt * 0.5f, len * 0.5f }, yaw, 0.0f, LookStructure );
		}
	}

	static void AddGate( Course& c, float x, float y, float z, float yaw, float r )
	{
		Gate g;
		g.p = { x, y, z };
		g.yaw = yaw;
		g.r = r;
		c.gates.push_back( g );
	}

private:
	bool TerrainRay( Vec3 o, Vec3 d, float maxT, float* tOut ) const
	{
		if ( HasTerrain() == false )
		{
			if ( d.y >= -1e-9f || o.y < 0.0f )
			{
				return false;
			}
			float t = -o.y / d.y;
			if ( t > maxT )
			{
				return false;
			}
			*tOut = t;
			return true;
		}
		const float hMax = terrainMaxY;
		float t = 0.0f;
		Vec3 p = o;
		float f = p.y - Height( p.x, p.z );
		if ( f < 0.0f )
		{
			return false; 
		}
		for ( int i = 0; i < 400 && t < maxT; ++i )
		{
			if ( p.y > hMax && d.y >= 0.0f )
			{
				return false;
			}
			float step = ClampFloat( f * 0.35f, 0.04f, 3.0f );
			float tn = MinFloat( t + step, maxT );
			Vec3 q = MulAdd( o, tn, d );
			float fq = q.y - Height( q.x, q.z );
			if ( fq < 0.0f )
			{
				float a = t, b = tn;
				for ( int k = 0; k < 18; ++k )
				{
					float m = 0.5f * ( a + b );
					Vec3 pm = MulAdd( o, m, d );
					if ( pm.y - Height( pm.x, pm.z ) < 0.0f )
					{
						b = m;
					}
					else
					{
						a = m;
					}
				}
				*tOut = b;
				return true;
			}
			if ( tn >= maxT )
			{
				break;
			}
			t = tn;
			p = q;
			f = fq;
		}
		return false;
	}

	static bool RayBox( const Box& b, Vec3 o, Vec3 d, float maxT, float* t, Vec3* n )
	{

		float tmin = 0.0f, tmax = maxT;
		const float* bo = &o.x;
		const float* bd = &d.x;
		const float* mn = &b.bmin.x;
		const float* mx = &b.bmax.x;
		for ( int i = 0; i < 3; ++i )
		{
			if ( fabsf( bd[i] ) < 1e-12f )
			{
				if ( bo[i] < mn[i] || bo[i] > mx[i] )
				{
					return false;
				}
			}
			else
			{
				float inv = 1.0f / bd[i];
				float t1 = ( mn[i] - bo[i] ) * inv, t2 = ( mx[i] - bo[i] ) * inv;
				if ( t1 > t2 )
				{
					float tt = t1;
					t1 = t2;
					t2 = tt;
				}
				tmin = MaxFloat( tmin, t1 );
				tmax = MinFloat( tmax, t2 );
				if ( tmin > tmax )
				{
					return false;
				}
			}
		}
		Vec3 rel = Sub( o, b.c );
		float lo[3] = { Dot( rel, b.ax[0] ), Dot( rel, b.ax[1] ), Dot( rel, b.ax[2] ) };
		float ld[3] = { Dot( d, b.ax[0] ), Dot( d, b.ax[1] ), Dot( d, b.ax[2] ) };
		float h[3] = { b.h.x, b.h.y, b.h.z };
		float t0 = -1e30f, t1 = 1e30f, sgn = 0.0f;
		int axis = -1;
		for ( int i = 0; i < 3; ++i )
		{
			if ( fabsf( ld[i] ) < 1e-12f )
			{
				if ( lo[i] < -h[i] || lo[i] > h[i] )
				{
					return false;
				}
				continue;
			}
			float a = ( -h[i] - lo[i] ) / ld[i], c = ( h[i] - lo[i] ) / ld[i];
			float s = -1.0f;
			if ( a > c )
			{
				float tt = a;
				a = c;
				c = tt;
				s = 1.0f;
			}
			if ( a > t0 )
			{
				t0 = a;
				axis = i;
				sgn = s;
			}
			t1 = MinFloat( t1, c );
			if ( t0 > t1 )
			{
				return false;
			}
		}
		if ( axis < 0 || t0 < 0.0f || t0 > maxT )
		{
			return false; 
		}
		*t = t0;
		*n = MulSV( sgn, b.ax[axis] );
		return true;
	}

	static void Finish( Box* b )
	{
		Vec3 e = { 0.0f, 0.0f, 0.0f };
		const float hh[3] = { b->h.x, b->h.y, b->h.z };
		for ( int i = 0; i < 3; ++i )
		{
			e.x += fabsf( b->ax[i].x ) * hh[i];
			e.y += fabsf( b->ax[i].y ) * hh[i];
			e.z += fabsf( b->ax[i].z ) * hh[i];
		}
		b->bmin = Sub( b->c, e );
		b->bmax = Add( b->c, e );
		b->q = QuatFromAxes( b->ax[0], b->ax[1], b->ax[2] );
	}
};

enum TerrainClass
{
	TerrainFlat = 0, 
	TerrainGrass,
	TerrainSlope,
	TerrainPit,
	TerrainSand,
	TerrainForest,
	TerrainSnow,
	TerrainClassCount
};

inline Material TerrainMaterial( int c )
{
	switch ( c )
	{
		case TerrainGrass:
			return Materials::Terrain( Palette::TerrainGrass );
		case TerrainSlope:
			return Materials::Terrain( Palette::TerrainRock );
		case TerrainPit:
			return Materials::Terrain( Palette::TerrainDirt );
		case TerrainSand:
			return Materials::Terrain( Palette::TerrainSand );
		case TerrainForest:
			return Materials::Terrain( Palette::TerrainForest );
		case TerrainSnow:
			return Materials::Terrain( Palette::TerrainSnow );
		default:
			return MakeMaterial( Palette::Ground, 0.0f, 0.70f );
	}
}

struct TerrainMeshes
{
	Mesh meshes[TerrainClassCount];
	int triangles = 0;

	void Build( const World& w, float cell = 2.0f )
	{
		if ( w.terrainMode != TerrainNone && w.terrainMode != TerrainArena )
		{
			cell = 1.4f;
		}
		triangles = 0;
		int n = (int)( 2.0f * w.extent / cell );
		int stride = n + 1;
		std::vector<Vec3> pos( (size_t)stride * stride ), nrm( (size_t)stride * stride );
		for ( int j = 0; j <= n; ++j )
		{
			for ( int i = 0; i <= n; ++i )
			{
				float x = -w.extent + (float)i * cell, z = -w.extent + (float)j * cell;
				pos[(size_t)j * stride + i] = { x, w.Height( x, z ), z };
				nrm[(size_t)j * stride + i] = w.TerrainNormal( x, z );
			}
		}
		MeshBuilder parts[TerrainClassCount];
		std::vector<int> remap[TerrainClassCount];
		for ( auto& r : remap )
		{
			r.assign( pos.size(), -1 );
		}
		auto vert = [&]( int c, int idx ) {
			if ( remap[c][idx] < 0 )
			{
				remap[c][idx] = parts[c].AddVertex( pos[idx], nrm[idx] );
			}
			return remap[c][idx];
		};
		for ( int j = 0; j < n; ++j )
		{
			for ( int i = 0; i < n; ++i )
			{
				int v00 = j * stride + i, v10 = v00 + 1, v01 = v00 + stride, v11 = v01 + 1;
				const int tris[2][3] = { { v00, v01, v11 }, { v00, v11, v10 } };
				for ( auto& t : tris )
				{
					Vec3 c = MulSV( 1.0f / 3.0f, pos[t[0]] + pos[t[1]] + pos[t[2]] );
					Vec3 fn = Normalize( Cross( pos[t[1]] - pos[t[0]], pos[t[2]] - pos[t[0]] ) );
					int cls = TerrainFlat;
					if ( w.terrainMode == TerrainFrontline )
					{
						if ( c.y < -0.5f )
						{
							cls = TerrainPit;
						}
						else if ( fn.y < 0.88f )
						{
							cls = TerrainSlope;
						}
						else if ( c.y > 4.2f )
						{
							cls = TerrainForest;
						}
						else
						{
							cls = TerrainGrass;
						}
					}
					else
					{
						if ( c.y < -0.3f )
						{
							cls = TerrainPit;
						}
						else if ( fn.y < 0.88f )
						{
							cls = TerrainSlope;
						}
						else if ( w.hills && World::HillWeight( c.z ) > 0.25f )
						{
							cls = TerrainGrass;
						}
					}
					parts[cls].AddTriangle( vert( cls, t[0] ), vert( cls, t[1] ), vert( cls, t[2] ) );
				}
			}
		}
		for ( int c = 0; c < TerrainClassCount; ++c )
		{
			meshes[c].Upload( parts[c], "mech_terrain" );
			triangles += meshes[c].TriangleCount();
		}
	}

	void Draw() const
	{
		Transform xf = { { 0.0f, 0.0f, 0.0f }, Quat_identity };
		for ( int c = 0; c < TerrainClassCount; ++c )
		{
			meshes[c].DrawWithGrid( xf, TerrainMaterial( c ) );
		}
	}
};

} 
