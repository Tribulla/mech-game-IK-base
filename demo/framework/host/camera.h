#pragma once

#include "gfx/utility.h"

struct sapp_event;

struct PickRay
{
	Vec3 origin;
	Vec3 translation;
};

class Camera
{
public:
	Camera();

	static constexpr float kViewDistance = 1000.0f;

	void OnEvent( const sapp_event* e );

	void Update( float dt, int width, int height );

	Mat4 View() const
	{
		return m_view;
	}

	Mat4 ViewInverse() const
	{
		return m_viewInv;
	}

	Mat4 Proj() const
	{
		return m_proj;
	}

	Mat4 ProjInverse() const
	{
		return m_projInv;
	}

	Vec3 Position() const;

	Vec3 GetPosition() const
	{
		return m_position;
	}

	Vec3 GetForward() const
	{
		return m_forward;
	}

	Vec3 GetRight() const
	{
		return m_right;
	}

	Vec3 GetUp() const
	{
		return m_up;
	}

	void SetOrbit( float yawRadians, float pitchRadians, float radius );

	void SetView( float yawDegrees, float pitchDegrees, float radius, Vec3 pivot );

	void SetPivot( Vec3 pivot )
	{
		m_pivot = pivot;
	}

	void SetTarget( Vec3 target )
	{
		m_pivot = target;
	}

	void Frame( AABB aabb, float aspect, float padding );
	void SetFov( float fovRadians )
	{
		m_fov = fovRadians;
	}
	void SetClip( float near, float far )
	{
		m_near = near;
		m_far = far;
	}

	void SetDrawDistance( float meters )
	{
		m_drawDistance = meters;
	}

	void RebuildProj( float aspect );

	void UpdateTransform();

	PickRay BuildPickRay( float x, float y ) const;

	void SetRenderTransform( float lengthUnitsPerMeter, bool zUp );

	Vec3 DrawOrigin() const
	{
		Vec4 e = MulMV4( m_renderXformInv, MakeVec4( m_worldEye.x, m_worldEye.y, m_worldEye.z, 1.0f ) );
		return Vec3{ e.x, e.y, e.z };
	}

	AABB DrawBounds() const
	{
		float h = m_drawDistance * m_lengthUnitsPerMeter;
		Vec3 r = { h, h, h };
		return OffsetAABB( { Neg( r ), r }, DrawOrigin() );
	}

	Vec3 m_pivot;
	float m_yaw;
	float m_pitch;
	float m_radius;

	float m_fov;
	float m_near;
	float m_far;

	float m_drawDistance;

	float m_speed;

	int m_width;
	int m_height;

	bool m_thirdPerson;

	Vec3 m_position;
	Vec3 m_worldEye;
	Vec3 m_right;
	Vec3 m_up;
	Vec3 m_forward;

	Mat4 m_view;
	Mat4 m_viewInv;
	Mat4 m_proj;
	Mat4 m_projInv;

	Mat4 m_renderXform;
	Mat4 m_renderXformInv;
	float m_lengthUnitsPerMeter;
	bool m_zUp;

	float m_orbitDX = 0.0f;
	float m_orbitDY = 0.0f;
	float m_panDX = 0.0f;
	float m_panDY = 0.0f;
	float m_radialZoomDY = 0.0f;
	float m_scrollAccum = 0.0f;
	float m_speedScrollAccum = 0.0f;

	bool m_leftDown = false;
	bool m_rightDown = false;
	bool m_middleDown = false;
	bool m_altDown = false;
	bool m_wDown = false;
	bool m_aDown = false;
	bool m_sDown = false;
	bool m_dDown = false;
};
