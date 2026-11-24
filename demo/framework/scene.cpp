#include "scene.h"

#include "math3d.h"

#include <math.h>
#include <utility>

static uint32_t s_nextMeshKey = 1;

static uint32_t NextMeshKey()
{
	uint32_t key = 0x80000000u | ( s_nextMeshKey++ & 0x7FFFFFFFu );
	return key;
}

static Transform ToRelative( Transform xf )
{
	return ToRelativeTransform( xf, GetDrawOrigin() );
}

void DrawBox( Transform xf, Vec3 size, const Material& m )
{
	DrawCubeEx( xf, size, m.color, m.metallic, m.roughness, TRANSPARENT_SHADOW_FULL );
}

void DrawBox( Vec3 center, Vec3 size, const Material& m )
{
	Transform xf = { center, Quat_identity };
	DrawBox( xf, size, m );
}

void DrawBall( Vec3 center, float radius, const Material& m )
{
	Transform xf = { center, Quat_identity };
	DrawSphereEx( xf, radius, m.color, m.metallic, m.roughness, TRANSPARENT_SHADOW_FULL );
}

void DrawBall( Transform xf, float radius, const Material& m )
{
	DrawSphereEx( xf, radius, m.color, m.metallic, m.roughness, TRANSPARENT_SHADOW_FULL );
}

void DrawRod( Vec3 a, Vec3 b, float radius, const Material& m )
{
	Vec3 d = Sub( b, a );
	float length = Length( d );
	Vec3 center = Add( a, MulSV( 0.5f, d ) );
	if ( length < 1e-6f )
	{
		DrawBall( center, radius, m );
		return;
	}

	Quat q = ComputeQuatBetweenUnitVectors( { 1.0f, 0.0f, 0.0f }, MulSV( 1.0f / length, d ) );
	Transform xf = { center, q };
	DrawCapsuleEx( xf, 0.5f * length, radius, m.color, m.metallic, m.roughness, TRANSPARENT_SHADOW_FULL );
}

static Mesh s_box;

void ReleaseSceneResources()
{
	s_box.Release();
}

static Mesh& UnitBoxMesh()
{
	if ( s_box.IsValid() == false )
	{
		MeshBuilder b;
		b.AddBox( { 0.0f, 0.0f, 0.0f }, { 0.5f, 0.5f, 0.5f } );
		s_box.Upload( b, "ground_box" );
	}
	return s_box;
}

void DrawGroundPlane( float halfExtent, float y )
{
	const float thickness = 2.0f;
	Transform xf = { { 0.0f, y - 0.5f * thickness, 0.0f }, Quat_identity };
	Material ground = MakeMaterial( Palette::Ground, 0.0f, 0.70f );
	UnitBoxMesh().DrawWithGrid( xf, ground, { 2.0f * halfExtent, thickness, 2.0f * halfExtent } );
}

void DrawPolyline( const Vec3* points, int count, Vec4 color, bool closed, float thicknessPx )
{
	for ( int i = 0; i + 1 < count; ++i )
	{
		DrawLineEx( points[i], points[i + 1], color, thicknessPx, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM );
	}
	if ( closed && count > 2 )
	{
		DrawLineEx( points[count - 1], points[0], color, thicknessPx, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM );
	}
}

void DrawCircleXZ( Vec3 center, float radius, Vec4 color, int segments )
{
	if ( segments < 3 )
	{
		segments = 3;
	}
	Vec3 prev = Add( center, { radius, 0.0f, 0.0f } );
	for ( int i = 1; i <= segments; ++i )
	{
		float a = 2.0f * MATH_PI * (float)i / (float)segments;
		Vec3 p = Add( center, { radius * cosf( a ), 0.0f, radius * sinf( a ) } );
		DrawLineEx( prev, p, color, Style::LinePx, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM );
		prev = p;
	}
}

void DrawTargetMarker( Vec3 p, float size, Vec4 color )
{
	DrawCrossEx( p, size, color, Style::LineBoldPx, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );
	DrawPointEx( p, color, Style::PointBoldPx, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM );
}

void MeshBuilder::Clear()
{
	vertices.clear();
	indices.clear();
}

int MeshBuilder::AddVertex( Vec3 position, Vec3 normal )
{
	MeshVertex v;
	v.position[0] = position.x;
	v.position[1] = position.y;
	v.position[2] = position.z;
	v.normal[0] = normal.x;
	v.normal[1] = normal.y;
	v.normal[2] = normal.z;
	vertices.push_back( v );
	return (int)vertices.size() - 1;
}

void MeshBuilder::AddTriangle( int a, int b, int c )
{
	indices.push_back( (uint32_t)a );
	indices.push_back( (uint32_t)b );
	indices.push_back( (uint32_t)c );
}

void MeshBuilder::AddQuad( int a, int b, int c, int d )
{
	AddTriangle( a, b, c );
	AddTriangle( a, c, d );
}

void MeshBuilder::AddBox( Vec3 c, Vec3 h )
{

	const Vec3 normals[6] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
	const Vec3 us[6] = { { 0, 1, 0 }, { 0, 0, 1 }, { 0, 0, 1 }, { 1, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } };
	for ( int f = 0; f < 6; ++f )
	{
		Vec3 n = normals[f];
		Vec3 u = us[f];
		Vec3 v = Cross( n, u );
		Vec3 center = { c.x + n.x * h.x, c.y + n.y * h.y, c.z + n.z * h.z };
		Vec3 su = { u.x * h.x, u.y * h.y, u.z * h.z };
		Vec3 sv = { v.x * h.x, v.y * h.y, v.z * h.z };
		int i0 = AddVertex( Sub( Sub( center, su ), sv ), n );
		int i1 = AddVertex( Sub( Add( center, su ), sv ), n );
		int i2 = AddVertex( Add( Add( center, su ), sv ), n );
		int i3 = AddVertex( Add( Sub( center, su ), sv ), n );
		AddQuad( i0, i1, i2, i3 );
	}
}

void MeshBuilder::ComputeSmoothNormals()
{
	std::vector<Vec3> acc( vertices.size(), Vec3_zero );
	for ( size_t t = 0; t + 2 < indices.size(); t += 3 )
	{
		const float* pa = vertices[indices[t]].position;
		const float* pb = vertices[indices[t + 1]].position;
		const float* pc = vertices[indices[t + 2]].position;
		Vec3 a = { pa[0], pa[1], pa[2] };
		Vec3 b = { pb[0], pb[1], pb[2] };
		Vec3 c = { pc[0], pc[1], pc[2] };
		Vec3 n = Cross( Sub( b, a ), Sub( c, a ) ); 
		for ( int k = 0; k < 3; ++k )
		{
			acc[indices[t + k]] = Add( acc[indices[t + k]], n );
		}
	}
	for ( size_t i = 0; i < vertices.size(); ++i )
	{
		Vec3 n = Normalize( acc[i] );
		vertices[i].normal[0] = n.x;
		vertices[i].normal[1] = n.y;
		vertices[i].normal[2] = n.z;
	}
}

void MeshBuilder::MakeFlatShaded()
{
	std::vector<MeshVertex> flat;
	std::vector<uint32_t> flatIndices;
	flat.reserve( indices.size() );
	flatIndices.reserve( indices.size() );
	for ( size_t t = 0; t + 2 < indices.size(); t += 3 )
	{
		MeshVertex v[3] = { vertices[indices[t]], vertices[indices[t + 1]], vertices[indices[t + 2]] };
		Vec3 a = { v[0].position[0], v[0].position[1], v[0].position[2] };
		Vec3 b = { v[1].position[0], v[1].position[1], v[1].position[2] };
		Vec3 c = { v[2].position[0], v[2].position[1], v[2].position[2] };
		Vec3 n = Normalize( Cross( Sub( b, a ), Sub( c, a ) ) );
		for ( int k = 0; k < 3; ++k )
		{
			v[k].normal[0] = n.x;
			v[k].normal[1] = n.y;
			v[k].normal[2] = n.z;
			flatIndices.push_back( (uint32_t)flat.size() );
			flat.push_back( v[k] );
		}
	}
	vertices.swap( flat );
	indices.swap( flatIndices );
}

Mesh::~Mesh()
{
	Release();
}

Mesh::Mesh( Mesh&& other ) noexcept
{
	m_handle = other.m_handle;
	m_triangleCount = other.m_triangleCount;
	other.m_handle = InvalidMeshHandle();
	other.m_triangleCount = 0;
}

Mesh& Mesh::operator=( Mesh&& other ) noexcept
{
	if ( this != &other )
	{
		Release();
		m_handle = other.m_handle;
		m_triangleCount = other.m_triangleCount;
		other.m_handle = InvalidMeshHandle();
		other.m_triangleCount = 0;
	}
	return *this;
}

void Mesh::Upload( const MeshBuilder& builder, const char* label )
{
	Release();
	if ( builder.IsEmpty() || builder.vertices.empty() )
	{
		return;
	}

	m_handle = RegisterMesh( NextMeshKey(), builder.vertices.data(), (int)builder.vertices.size(), builder.indices.data(),
							 (int)builder.indices.size(), label );
	if ( IsValid() )
	{
		SetMeshKind( m_handle, MESH_KIND_MESH );
		m_triangleCount = (int)builder.indices.size() / 3;
	}
}

void Mesh::Release()
{
	if ( IsValid() )
	{
		ReleaseMeshReference( m_handle );
	}
	m_handle = InvalidMeshHandle();
	m_triangleCount = 0;
}

void Mesh::Draw( Transform xf, const Material& m, Vec3 scale ) const
{
	if ( IsValid() == false )
	{
		return;
	}
	AppendMesh( m_handle, ToRelative( xf ), scale, m.color, m.metallic, m.roughness, MESH_MATERIAL_MODE_SOLID, 0.0f,
				TRANSPARENT_SHADOW_FULL );
}

void Mesh::DrawWithGrid( Transform xf, const Material& m, Vec3 scale ) const
{
	if ( IsValid() == false )
	{
		return;
	}
	AppendMesh( m_handle, ToRelative( xf ), scale, m.color, m.metallic, m.roughness, MESH_MATERIAL_MODE_GROUND_GRID,
				Style::GridCell, TRANSPARENT_SHADOW_FULL );
}
