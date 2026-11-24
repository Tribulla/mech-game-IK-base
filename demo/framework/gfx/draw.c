#include "gfx/draw.h"

#include "gfx/overlay.h"
#include "gfx/renderer.h"
#include "gfx/text.h"

#include <stdarg.h>
#include <stdio.h>

static Vec3 s_drawOrigin;

static inline Vec3 ToRelative( Vec3 worldPoint )
{
	return Sub( worldPoint, s_drawOrigin );
}

static inline Transform ToRelativeFrame( Transform transform )
{
	return ToRelativeTransform( transform, s_drawOrigin );
}

void SetDrawOrigin( Vec3 origin )
{
	s_drawOrigin = origin;
}

Vec3 GetDrawOrigin( void )
{
	return s_drawOrigin;
}

void DrawCubeEx( Transform transform, Vec3 scale, Vec4 baseColor, float metallic, float roughness,
				 TransparentShadowCast shadowCast )
{
	AppendCube( ToRelativeFrame( transform ), scale, baseColor, metallic, roughness, shadowCast );
}

void DrawCube( Transform transform, Vec3 scale, Vec4 baseColor )
{
	DrawCubeEx( transform, scale, baseColor, DEFAULT_METALLIC, DEFAULT_ROUGHNESS, TRANSPARENT_SHADOW_FULL );
}

void DrawSphereEx( Transform transform, float radius, Vec4 baseColor, float metallic, float roughness,
				   TransparentShadowCast shadowCast )
{
	AppendSphere( ToRelativeFrame( transform ), radius, baseColor, metallic, roughness, shadowCast );
}

void DrawSphere( Transform transform, float radius, Vec4 baseColor )
{
	DrawSphereEx( transform, radius, baseColor, DEFAULT_METALLIC, DEFAULT_ROUGHNESS, TRANSPARENT_SHADOW_FULL );
}

void DrawCapsuleEx( Transform transform, float halfLength, float radius, Vec4 baseColor, float metallic, float roughness,
					TransparentShadowCast shadowCast )
{
	AppendCapsule( ToRelativeFrame( transform ), halfLength, radius, baseColor, metallic, roughness, shadowCast );
}

void DrawCapsule( Transform transform, float halfLength, float radius, Vec4 baseColor )
{
	DrawCapsuleEx( transform, halfLength, radius, baseColor, DEFAULT_METALLIC, DEFAULT_ROUGHNESS, TRANSPARENT_SHADOW_FULL );
}

void DrawPlane( Vec3 normal, Vec3 point, Vec4 color )
{
	Vec3 c = ToRelative( point );
	Vec3 perp1 = Perp( normal );
	Vec3 perp2 = Cross( perp1, normal );
	Vec3 p1 = Add( c, Add( perp1, perp2 ) );
	Vec3 p2 = Add( c, Sub( perp2, perp1 ) );
	Vec3 p3 = Sub( c, Add( perp1, perp2 ) );
	Vec3 p4 = Add( c, Sub( perp1, perp2 ) );
	float th = DEFAULT_LINE_THICKNESS_PX;
	OverlayAppendLine( p1, p2, color, th, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	OverlayAppendLine( p2, p3, color, th, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	OverlayAppendLine( p3, p4, color, th, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	OverlayAppendLine( p4, p1, color, th, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	OverlayAppendLine( c, Add( c, MulSV( 0.5f, normal ) ), color, th, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	OverlayAppendPoint( c, color, 10.0f, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM );
}

void DrawLineEx( Vec3 a, Vec3 b, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				 OverlayOcclusionMode occlusionMode )
{
	OverlayAppendLine( ToRelative( a ), ToRelative( b ), color, thickness, thicknessUnit, occlusionMode );
}

void DrawLine( Vec3 a, Vec3 b, Vec4 color )
{
	DrawLineEx( a, b, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
}

void DrawPointEx( Vec3 p, Vec4 color, float size, OverlayThicknessUnit sizeUnit, OverlayOcclusionMode occlusionMode )
{
	OverlayAppendPoint( ToRelative( p ), color, size, sizeUnit, occlusionMode );
}

void DrawPoint( Vec3 p, float size, Vec4 color )
{
	DrawPointEx( p, color, size, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM );
}

void DrawArrowEx( Vec3 a, Vec3 b, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				  OverlayOcclusionMode occlusionMode, float headLengthFrac )
{
	Vec3 tail = ToRelative( a );
	Vec3 tip = ToRelative( b );
	OverlayAppendLine( tail, tip, color, thickness, thicknessUnit, occlusionMode );

	Vec3 shaft = Sub( tip, tail );
	float shaftLen = Length( shaft );
	if ( shaftLen < 1e-6f )
	{
		return;
	}
	Vec3 dir = { shaft.x / shaftLen, shaft.y / shaftLen, shaft.z / shaftLen };
	Vec3 perp = Perp( dir );
	float headLen = shaftLen * headLengthFrac;

	Vec3 backFromTip = MulSV( -headLen, dir );
	Vec3 sideStep = MulSV( headLen * 0.5f, perp );
	Vec3 tip1 = Add( tip, Add( backFromTip, sideStep ) );
	Vec3 tip2 = Add( tip, Sub( backFromTip, sideStep ) );
	OverlayAppendLine( tip, tip1, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( tip, tip2, color, thickness, thicknessUnit, occlusionMode );
}

void DrawArrow( Vec3 a, Vec3 b, Vec4 color )
{
	DrawArrowEx( a, b, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE,
				 DEFAULT_ARROW_HEAD_FRAC );
}

void DrawCrossEx( Vec3 center, float size, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				  OverlayOcclusionMode occlusionMode )
{
	Vec3 c = ToRelative( center );
	float h = size * 0.5f;
	OverlayAppendLine( (Vec3){ c.x - h, c.y, c.z }, (Vec3){ c.x + h, c.y, c.z }, color, thickness, thicknessUnit,
					   occlusionMode );
	OverlayAppendLine( (Vec3){ c.x, c.y - h, c.z }, (Vec3){ c.x, c.y + h, c.z }, color, thickness, thicknessUnit,
					   occlusionMode );
	OverlayAppendLine( (Vec3){ c.x, c.y, c.z - h }, (Vec3){ c.x, c.y, c.z + h }, color, thickness, thicknessUnit,
					   occlusionMode );
}

void DrawCross( Vec3 center, float size, Vec4 color )
{
	DrawCrossEx( center, size, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
}

void DrawAabbEx( Vec3 mn, Vec3 mx, Vec4 color, float thickness, OverlayThicknessUnit thicknessUnit,
				 OverlayOcclusionMode occlusionMode )
{
	Vec3 c000 = ToRelative( ( (Vec3){ mn.x, mn.y, mn.z } ) );
	Vec3 c100 = ToRelative( ( (Vec3){ mx.x, mn.y, mn.z } ) );
	Vec3 c010 = ToRelative( ( (Vec3){ mn.x, mx.y, mn.z } ) );
	Vec3 c110 = ToRelative( ( (Vec3){ mx.x, mx.y, mn.z } ) );
	Vec3 c001 = ToRelative( ( (Vec3){ mn.x, mn.y, mx.z } ) );
	Vec3 c101 = ToRelative( ( (Vec3){ mx.x, mn.y, mx.z } ) );
	Vec3 c011 = ToRelative( ( (Vec3){ mn.x, mx.y, mx.z } ) );
	Vec3 c111 = ToRelative( ( (Vec3){ mx.x, mx.y, mx.z } ) );

	OverlayAppendLine( c000, c100, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( c100, c101, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( c101, c001, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( c001, c000, color, thickness, thicknessUnit, occlusionMode );

	OverlayAppendLine( c010, c110, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( c110, c111, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( c111, c011, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( c011, c010, color, thickness, thicknessUnit, occlusionMode );

	OverlayAppendLine( c000, c010, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( c100, c110, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( c101, c111, color, thickness, thicknessUnit, occlusionMode );
	OverlayAppendLine( c001, c011, color, thickness, thicknessUnit, occlusionMode );
}

void DrawAabb( Vec3 mn, Vec3 mx, Vec4 color )
{
	DrawAabbEx( mn, mx, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
}

void DrawBounds( AABB bounds, float extension, Vec4 color )
{
	Vec3 e = { extension, extension, extension };
	DrawAabb( Sub( bounds.lowerBound, e ), Add( bounds.upperBound, e ), color );
}

void DrawAxesEx( Transform transform, float size, float thickness, OverlayThicknessUnit thicknessUnit,
				 OverlayOcclusionMode occlusionMode )
{
	Transform rel = ToRelativeFrame( transform );
	Vec3 origin = rel.p;
	Mat3 basis = MakeMatrixFromQuat( rel.q );
	OverlayAppendLine( origin, MulAdd( origin, size, basis.cx ), MakeVec4( 1.0f, 0.0f, 0.0f, 1.0f ), thickness, thicknessUnit,
					   occlusionMode );
	OverlayAppendLine( origin, MulAdd( origin, size, basis.cy ), MakeVec4( 0.0f, 1.0f, 0.0f, 1.0f ), thickness, thicknessUnit,
					   occlusionMode );
	OverlayAppendLine( origin, MulAdd( origin, size, basis.cz ), MakeVec4( 0.0f, 0.0f, 1.0f, 1.0f ), thickness, thicknessUnit,
					   occlusionMode );
}

void DrawAxes( Transform transform, float size )
{
	DrawAxesEx( transform, size, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
}

void DrawGrid( Vec3 center, Vec3 normal, float halfExtent, int divisions, Vec4 color )
{
	if ( divisions < 1 || halfExtent <= 0.0f )
	{
		return;
	}
	Vec3 c = ToRelative( center );

	Vec3 n = Normalize( normal );
	Vec3 u = Normalize( Perp( n ) );
	Vec3 v = Cross( n, u );

	const float step = ( 2.0f * halfExtent ) / (float)divisions;
	for ( int i = 0; i <= divisions; ++i )
	{
		const float o = -halfExtent + (float)i * step;
		Vec3 ua = Add( c, Add( MulSV( -halfExtent, u ), MulSV( o, v ) ) );
		Vec3 ub = Add( c, Add( MulSV( halfExtent, u ), MulSV( o, v ) ) );
		OverlayAppendLine( ua, ub, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
		Vec3 va = Add( c, Add( MulSV( o, u ), MulSV( -halfExtent, v ) ) );
		Vec3 vb = Add( c, Add( MulSV( o, u ), MulSV( halfExtent, v ) ) );
		OverlayAppendLine( va, vb, color, DEFAULT_LINE_THICKNESS_PX, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	}
}

void DrawGroundGrid( int size )
{
	Vec4 color = MakeVec4( 0.3f, 0.3f, 0.3f, 1.0f );
	DrawGrid( Vec3_zero, Vec3_axisY, (float)size, size, color );
}

void DrawTriangle( Transform transform, Vec3 a, Vec3 b, Vec3 c, Vec4 color )
{
	Transform rel = ToRelativeFrame( transform );

	Vec3 ra = TransformPoint( rel, a );
	Vec3 rb = TransformPoint( rel, b );
	Vec3 rc = TransformPoint( rel, c );
	float th = DEFAULT_LINE_THICKNESS_PX;
	OverlayAppendLine( ra, rb, color, th, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	OverlayAppendLine( rb, rc, color, th, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
	OverlayAppendLine( rc, ra, color, th, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_HIDE );
}

static void DrawDisc( Vec3 center, Vec3 normal, float radius, int segments, Vec4 color )
{
	Vec3 tangent1 = Perp( normal );
	Vec3 tangent2 = Cross( normal, tangent1 );

	float delta = 2.0f * MATH_PI / segments;
	float cosine = cosf( delta );
	float sine = sinf( delta );

	float x1 = radius, y1 = 0.0f;
	Vec3 vertex1 = Add( center, Blend2( x1, tangent1, y1, tangent2 ) );

	for ( int i = 0; i < segments; ++i )
	{
		float x2 = cosine * x1 - sine * y1;
		float y2 = sine * x1 + cosine * y1;
		Vec3 vertex2 = Add( center, Blend2( x2, tangent1, y2, tangent2 ) );

		OverlayAppendLine( vertex1, vertex2, color, 2.0f, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DIM );

		x1 = x2;
		y1 = y2;
		vertex1 = vertex2;
	}
}

static void DrawArc( Vec3 center, Vec3 normal, float radius, Vec3 start, float maxDegrees, int segments, Vec4 color )
{
	Vec3 tangent1 = Normalize( start );
	Vec3 tangent2 = Cross( normal, tangent1 );

	float deltaDegrees = maxDegrees / segments;
	CosSin cs = ComputeCosSin( MATH_DEG_TO_RAD * deltaDegrees );
	float x1 = radius, y1 = 0.0f;

	Vec3 vertex1 = Add( center, Blend2( x1, tangent1, y1, tangent2 ) );

	for ( float angle = 0.0f; angle < maxDegrees - 0.001f; angle += deltaDegrees )
	{
		float x2 = cs.cosine * x1 - cs.sine * y1;
		float y2 = cs.sine * x1 + cs.cosine * y1;
		Vec3 vertex2 = Add( center, Blend2( x2, tangent1, y2, tangent2 ) );

		OverlayAppendLine( vertex1, vertex2, color, 2.0f, OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );

		x1 = x2;
		y1 = y2;
		vertex1 = vertex2;
	}
}

void DrawWireSphere( Transform transform, float radius, int segments, Vec4 color )
{
	Transform rel = ToRelativeFrame( transform );
	Vec3 center = rel.p;

	Vec3 axisX = RotateVector( rel.q, (Vec3){ 1.0f, 0.0f, 0.0f } );
	Vec3 axisY = RotateVector( rel.q, (Vec3){ 0.0f, 1.0f, 0.0f } );
	Vec3 axisZ = RotateVector( rel.q, (Vec3){ 0.0f, 0.0f, 1.0f } );

	DrawDisc( center, axisX, radius, segments, color );
	DrawDisc( center, axisY, radius, segments, color );
	DrawDisc( center, axisZ, radius, segments, color );
}

void DrawWireCapsule( Vec3 a, Vec3 b, float radius, int segments, Vec4 color )
{
	Vec3 center1 = ToRelative( a );
	Vec3 center2 = ToRelative( b );

	Vec3 normal = Normalize( Sub( center2, center1 ) );
	Vec3 tangent1 = Perp( normal );
	Vec3 tangent2 = Cross( normal, tangent1 );

	OverlayAppendLine( MulAdd( center1, radius, tangent1 ), MulAdd( center2, radius, tangent1 ), color, 2.0f,
					   OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );
	OverlayAppendLine( MulAdd( center1, radius, tangent2 ), MulAdd( center2, radius, tangent2 ), color, 2.0f,
					   OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );
	OverlayAppendLine( MulSub( center1, radius, tangent1 ), MulSub( center2, radius, tangent1 ), color, 2.0f,
					   OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );
	OverlayAppendLine( MulSub( center1, radius, tangent2 ), MulSub( center2, radius, tangent2 ), color, 2.0f,
					   OVERLAY_THICKNESS_PIXELS, OVERLAY_OCCLUSION_DASHED );

	DrawArc( center1, Neg( tangent1 ), radius, tangent2, 180.0f, segments / 2, color );
	DrawArc( center1, tangent2, radius, tangent1, 180.0f, segments / 2, color );
	DrawArc( center2, tangent1, radius, tangent2, 180.0f, segments / 2, color );
	DrawArc( center2, Neg( tangent2 ), radius, tangent1, 180.0f, segments / 2, color );

	DrawDisc( center1, normal, radius, segments, color );
	DrawDisc( center2, normal, radius, segments, color );
}

void DrawString3D( Vec3 point, Vec4 color, const char* format, ... )
{
	va_list args;
	va_start( args, format );
	char buffer[256];
	vsnprintf( buffer, sizeof( buffer ), format, args );
	va_end( args );
	DrawString( ToRelative( point ), color, buffer );
}
