#pragma once

#include "gfx/draw.h"
#include "gfx/geometry_registry.h"
#include "theme.h"

#include <stdint.h>
#include <vector>

void DrawBox( Transform xf, Vec3 size, const Material& m );

void DrawBox( Vec3 center, Vec3 size, const Material& m );

void DrawBall( Vec3 center, float radius, const Material& m );
void DrawBall( Transform xf, float radius, const Material& m );

void DrawRod( Vec3 a, Vec3 b, float radius, const Material& m );

void DrawGroundPlane( float halfExtent = 100.0f, float y = 0.0f );

void DrawPolyline( const Vec3* points, int count, Vec4 color, bool closed = false, float thicknessPx = Style::LinePx );

void DrawCircleXZ( Vec3 center, float radius, Vec4 color, int segments = 48 );

void DrawTargetMarker( Vec3 p, float size, Vec4 color );

class MeshBuilder
{
public:
	void Clear();

	int AddVertex( Vec3 position, Vec3 normal = { 0.0f, 0.0f, 0.0f } );
	void AddTriangle( int a, int b, int c );
	void AddQuad( int a, int b, int c, int d ); 

	void AddBox( Vec3 center, Vec3 halfExtents );

	template <typename HeightFn>
	void AddHeightGrid( float x0, float z0, float x1, float z1, int nx, int nz, HeightFn heightFn )
	{
		int base = (int)vertices.size();
		for ( int j = 0; j <= nz; ++j )
		{
			for ( int i = 0; i <= nx; ++i )
			{
				float x = x0 + ( x1 - x0 ) * (float)i / (float)nx;
				float z = z0 + ( z1 - z0 ) * (float)j / (float)nz;
				AddVertex( { x, heightFn( x, z ), z } );
			}
		}
		int stride = nx + 1;
		for ( int j = 0; j < nz; ++j )
		{
			for ( int i = 0; i < nx; ++i )
			{
				int v00 = base + j * stride + i;
				int v10 = v00 + 1;
				int v01 = v00 + stride;
				int v11 = v01 + 1;
				AddTriangle( v00, v01, v11 );
				AddTriangle( v00, v11, v10 );
			}
		}
	}

	void ComputeSmoothNormals();

	void MakeFlatShaded();

	bool IsEmpty() const
	{
		return indices.empty();
	}

	std::vector<MeshVertex> vertices;
	std::vector<uint32_t> indices;
};

class Mesh
{
public:
	Mesh() = default;
	~Mesh();
	Mesh( const Mesh& ) = delete;
	Mesh& operator=( const Mesh& ) = delete;
	Mesh( Mesh&& other ) noexcept;
	Mesh& operator=( Mesh&& other ) noexcept;

	void Upload( const MeshBuilder& builder, const char* label = "sample_mesh" );
	void Release();
	bool IsValid() const
	{
		return IsMeshHandleValid( m_handle );
	}

	int TriangleCount() const
	{
		return m_triangleCount;
	}

	void Draw( Transform xf, const Material& m, Vec3 scale = { 1.0f, 1.0f, 1.0f } ) const;

	void DrawWithGrid( Transform xf, const Material& m, Vec3 scale = { 1.0f, 1.0f, 1.0f } ) const;

private:
	MeshHandle m_handle = { -1, 0u };
	int m_triangleCount = 0;
};

void ReleaseSceneResources();
