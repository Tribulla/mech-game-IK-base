#include "camera.h"

#include "sokol_app.h"

#include "math3d.h"

#include <math.h>

static constexpr float HALF_PI = MATH_PI * 0.5f;
static constexpr float DEG_TO_RAD = MATH_PI / 180.0f;
static constexpr float ORBIT_SENS = 0.005f;
static constexpr float FLY_LOOK_SENS = 0.005f;
static constexpr float PAN_SENS = 0.005f;
static constexpr float RADIAL_ZOOM_SENS = 0.02f;
static constexpr float FLY_SPEED_STEP = 1.0f;
static constexpr float ZOOM_STEP = 0.9f;
static constexpr float MIN_DIST = 0.1f;
static constexpr float MIN_SPEED = 0.06f;
static constexpr float MAX_SPEED = 30000.0f;
static constexpr float PITCH_LIM = HALF_PI - 0.01f;
static constexpr float THIRD_PERSON_MIN_RADIUS = 0.25f;

static void ReverseZPerspectiveAndInverse( Mat4* outProj, Mat4* outProjInv, float fov, float aspect, float n, float f )
{
	MakePerspectiveAndInverse( outProj, outProjInv, fov, aspect, f, n );
}

static Vec3 ForwardFromAngles( float yaw, float pitch )
{
	const float cp = cosf( pitch );
	return Vec3{ sinf( yaw ) * cp, sinf( pitch ), cosf( yaw ) * cp };
}

static void RebuildBasisAndView( Camera& c )
{
	const Vec3 forward = ForwardFromAngles( c.m_yaw, c.m_pitch );
	const Vec3 worldUp = { 0.0f, 1.0f, 0.0f };
	const Vec3 upUnnorm = Sub( worldUp, MulSV( Dot( worldUp, forward ), forward ) );
	const Vec3 up = Normalize( upUnnorm );
	const Vec3 right = Normalize( Cross( up, forward ) );

	c.m_worldEye = Add( c.m_pivot, MulSV( c.m_radius, forward ) );
	c.m_position = Vec3_zero;
	c.m_right = right;
	c.m_up = up;
	c.m_forward = forward;

	Mat4 viewBasis, viewBasisInv;
	MakeViewAndInverse( &viewBasis, &viewBasisInv, right, up, forward, c.m_position );
	c.m_view = MulMM4( viewBasis, c.m_renderXform );
	c.m_viewInv = MulMM4( c.m_renderXformInv, viewBasisInv );
}

Camera::Camera()
	: m_pivot{ 0.0f, 0.0f, 0.0f }
	, m_yaw( 35.0f * DEG_TO_RAD )
	, m_pitch( -25.0f * DEG_TO_RAD )
	, m_radius( 25.0f )
	, m_fov( 60.0f * DEG_TO_RAD )
	, m_near( 0.1f )
	, m_far( kViewDistance )
	, m_drawDistance( kViewDistance )
	, m_speed( 10.0f )
	, m_width( 0 )
	, m_height( 0 )
	, m_thirdPerson( false )
	, m_position{ 0.0f, 0.0f, 0.0f }
	, m_worldEye{ 0.0f, 0.0f, 0.0f }
	, m_right{ 1.0f, 0.0f, 0.0f }
	, m_up{ 0.0f, 1.0f, 0.0f }
	, m_forward{ 0.0f, 0.0f, 1.0f }
	, m_view( MakeIdentity() )
	, m_viewInv( MakeIdentity() )
	, m_proj( MakeIdentity() )
	, m_projInv( MakeIdentity() )
	, m_renderXform( MakeIdentity() )
	, m_renderXformInv( MakeIdentity() )
	, m_lengthUnitsPerMeter( 1.0f )
	, m_zUp( false )
{
	RebuildBasisAndView( *this );
	RebuildProj( 1.0f );
}

void Camera::RebuildProj( float aspect )
{
	if ( aspect <= 0.0f )
		aspect = 1.0f;
	ReverseZPerspectiveAndInverse( &m_proj, &m_projInv, m_fov, aspect, m_near, m_far );
}

void Camera::SetRenderTransform( float lengthUnitsPerMeter, bool zUp )
{
	m_lengthUnitsPerMeter = lengthUnitsPerMeter > 0.0f ? lengthUnitsPerMeter : 1.0f;
	m_zUp = zUp;

	const float s = 1.0f / m_lengthUnitsPerMeter;
	const float u = m_lengthUnitsPerMeter;

	if ( zUp )
	{
		m_renderXform =
			Mat4{ { s, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, -s, 0.0f }, { 0.0f, s, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 1.0f } };
		m_renderXformInv =
			Mat4{ { u, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, u, 0.0f }, { 0.0f, -u, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 1.0f } };
	}
	else
	{
		m_renderXform =
			Mat4{ { s, 0.0f, 0.0f, 0.0f }, { 0.0f, s, 0.0f, 0.0f }, { 0.0f, 0.0f, s, 0.0f }, { 0.0f, 0.0f, 0.0f, 1.0f } };
		m_renderXformInv =
			Mat4{ { u, 0.0f, 0.0f, 0.0f }, { 0.0f, u, 0.0f, 0.0f }, { 0.0f, 0.0f, u, 0.0f }, { 0.0f, 0.0f, 0.0f, 1.0f } };
	}

	RebuildBasisAndView( *this );
}

static AABB TransformAABBToDisplay( const Mat4& s, AABB a )
{
	Vec4 lo = MulMV4( s, MakeVec4( a.lowerBound.x, a.lowerBound.y, a.lowerBound.z, 1.0f ) );
	Vec4 hi = MulMV4( s, MakeVec4( a.upperBound.x, a.upperBound.y, a.upperBound.z, 1.0f ) );
	AABB out;
	out.lowerBound = { fminf( lo.x, hi.x ), fminf( lo.y, hi.y ), fminf( lo.z, hi.z ) };
	out.upperBound = { fmaxf( lo.x, hi.x ), fmaxf( lo.y, hi.y ), fmaxf( lo.z, hi.z ) };
	return out;
}

void Camera::Frame( AABB aabb, float aspect, float padding )
{
	if ( !IsValidAABB( aabb ) )
	{
		return;
	}

	aabb = TransformAABBToDisplay( m_renderXform, aabb );
	Vec3 center = AABB_Center( aabb );
	Vec3 ext = AABB_Extents( aabb ); 
	float r = sqrtf( ext.x * ext.x + ext.y * ext.y + ext.z * ext.z );
	m_pivot = ( center );
	if ( r < 1.0e-6f )
	{
		RebuildBasisAndView( *this );
		return;
	}
	if ( aspect <= 0.0f )
	{
		aspect = 1.0f;
	}
	float halfFovY = 0.5f * m_fov;
	float invTan = 1.0f / tanf( halfFovY );
	float distV = r * invTan;
	float distH = r * invTan / aspect;
	float d = ( distV > distH ) ? distV : distH;
	d *= padding;
	SetOrbit( m_yaw, m_pitch, d );
}

void Camera::SetOrbit( float yawRadians, float pitchRadians, float radius )
{
	m_yaw = yawRadians;
	m_pitch = ClampFloat( pitchRadians, -PITCH_LIM, PITCH_LIM );
	m_radius = ClampFloat( radius, MIN_DIST, kViewDistance );
	RebuildBasisAndView( *this );
}

void Camera::SetView( float yawDegrees, float pitchDegrees, float radius, Vec3 pivot )
{
	SetPivot( pivot );
	SetOrbit( yawDegrees * DEG_TO_RAD, pitchDegrees * DEG_TO_RAD, radius );
}

void Camera::UpdateTransform()
{
	RebuildBasisAndView( *this );
}

Vec3 Camera::Position() const
{
	return m_position;
}

PickRay Camera::BuildPickRay( float x, float y ) const
{
	PickRay ray;
	ray.origin = Vec3_zero;
	ray.translation = Vec3_zero;

	if ( m_width <= 0 || m_height <= 0 )
	{
		return ray;
	}

	float ndcX = ( 2.0f * x / (float)m_width ) - 1.0f;
	float ndcY = 1.0f - ( 2.0f * y / (float)m_height );

	Mat4 invVP = MulMM4( m_viewInv, m_projInv );

	Vec4 clipN = MakeVec4( ndcX, ndcY, 1.0f, 1.0f );
	Vec4 clipF = MakeVec4( ndcX, ndcY, 0.0f, 1.0f );

	Vec4 wN = MulMV4( invVP, clipN );
	Vec4 wF = MulMV4( invVP, clipF );

	if ( wN.w == 0.0f || wF.w == 0.0f )
	{
		return ray;
	}

	float invWN = 1.0f / wN.w;
	float invWF = 1.0f / wF.w;

	Vec3 nearWorld = { wN.x * invWN, wN.y * invWN, wN.z * invWN };
	Vec3 farWorld = { wF.x * invWF, wF.y * invWF, wF.z * invWF };

	ray.origin = Add( DrawOrigin(), nearWorld );
	ray.translation = Sub( farWorld, nearWorld );
	return ray;
}

void Camera::OnEvent( const sapp_event* e )
{
	uint32_t mods = e->modifiers & ( SAPP_MODIFIER_SHIFT | SAPP_MODIFIER_CTRL | SAPP_MODIFIER_ALT );
	bool canOrbit = mods == SAPP_MODIFIER_ALT;

	switch ( e->type )
	{
		case SAPP_EVENTTYPE_MOUSE_DOWN:
			if ( e->mouse_button == SAPP_MOUSEBUTTON_LEFT )
				m_leftDown = true;
			if ( e->mouse_button == SAPP_MOUSEBUTTON_RIGHT )
				m_rightDown = true;
			if ( e->mouse_button == SAPP_MOUSEBUTTON_MIDDLE )
				m_middleDown = true;
			break;
		case SAPP_EVENTTYPE_MOUSE_UP:
			if ( e->mouse_button == SAPP_MOUSEBUTTON_LEFT )
				m_leftDown = false;
			if ( e->mouse_button == SAPP_MOUSEBUTTON_RIGHT )
				m_rightDown = false;
			if ( e->mouse_button == SAPP_MOUSEBUTTON_MIDDLE )
				m_middleDown = false;
			break;
		case SAPP_EVENTTYPE_MOUSE_MOVE:
			if ( canOrbit && m_leftDown )
			{
				m_orbitDX += e->mouse_dx;
				m_orbitDY += e->mouse_dy;
			}
			else if ( canOrbit && m_middleDown )
			{
				m_panDX += e->mouse_dx;
				m_panDY += e->mouse_dy;
			}
			else if ( canOrbit && m_rightDown )
			{
				m_radialZoomDY += e->mouse_dy;
			}
			else if ( !canOrbit && m_rightDown )
			{
				m_orbitDX += e->mouse_dx;
				m_orbitDY += e->mouse_dy;
			}
			break;
		case SAPP_EVENTTYPE_MOUSE_SCROLL:
			if ( !canOrbit && m_rightDown )
				m_speedScrollAccum += e->scroll_y;
			else
				m_scrollAccum += e->scroll_y;
			break;
		case SAPP_EVENTTYPE_KEY_DOWN:
			switch ( e->key_code )
			{
				case SAPP_KEYCODE_W:
					m_wDown = true;
					break;
				case SAPP_KEYCODE_A:
					m_aDown = true;
					break;
				case SAPP_KEYCODE_S:
					m_sDown = true;
					break;
				case SAPP_KEYCODE_D:
					m_dDown = true;
					break;
				case SAPP_KEYCODE_LEFT_ALT:
				case SAPP_KEYCODE_RIGHT_ALT:
					m_altDown = true;
					break;
				default:
					break;
			}
			break;
		case SAPP_EVENTTYPE_KEY_UP:
			switch ( e->key_code )
			{
				case SAPP_KEYCODE_W:
					m_wDown = false;
					break;
				case SAPP_KEYCODE_A:
					m_aDown = false;
					break;
				case SAPP_KEYCODE_S:
					m_sDown = false;
					break;
				case SAPP_KEYCODE_D:
					m_dDown = false;
					break;
				case SAPP_KEYCODE_LEFT_ALT:
				case SAPP_KEYCODE_RIGHT_ALT:
					m_altDown = false;
					break;
				default:
					break;
			}
			break;
		case SAPP_EVENTTYPE_UNFOCUSED:
			m_leftDown = false;
			m_rightDown = false;
			m_middleDown = false;
			m_wDown = false;
			m_aDown = false;
			m_sDown = false;
			m_dDown = false;
			m_altDown = false;
			break;
		default:
			break;
	}
}

void Camera::Update( float dt, int width, int height )
{
	m_width = width;
	m_height = height;
	const float aspect = ( height > 0 ) ? (float)width / (float)height : 1.0f;

	if ( m_thirdPerson )
	{
		if ( m_scrollAccum != 0.0f )
		{
			m_radius -= m_scrollAccum;
			if ( m_radius < THIRD_PERSON_MIN_RADIUS )
				m_radius = THIRD_PERSON_MIN_RADIUS;
		}
		m_orbitDX = 0.0f;
		m_orbitDY = 0.0f;
		m_panDX = 0.0f;
		m_panDY = 0.0f;
		m_radialZoomDY = 0.0f;
		m_scrollAccum = 0.0f;
		m_speedScrollAccum = 0.0f;
		RebuildBasisAndView( *this );
		RebuildProj( aspect );
		return;
	}

	const bool flyMode = m_rightDown && !m_altDown;

	if ( flyMode )
	{
		const Vec3 eyeBefore = Add( m_pivot, MulSV( m_radius, ForwardFromAngles( m_yaw, m_pitch ) ) );

		if ( m_orbitDX != 0.0f || m_orbitDY != 0.0f )
		{
			m_yaw -= m_orbitDX * FLY_LOOK_SENS;
			m_pitch += m_orbitDY * FLY_LOOK_SENS;
			if ( m_pitch > PITCH_LIM )
				m_pitch = PITCH_LIM;
			if ( m_pitch < -PITCH_LIM )
				m_pitch = -PITCH_LIM;
		}

		const Vec3 forward = ForwardFromAngles( m_yaw, m_pitch );

		float wasdF = 0.0f;
		float wasdR = 0.0f;
		if ( m_wDown )
			wasdF -= 1.0f;
		if ( m_sDown )
			wasdF += 1.0f;
		if ( m_dDown )
			wasdR += 1.0f;
		if ( m_aDown )
			wasdR -= 1.0f;

		Vec3 eye = eyeBefore;
		if ( wasdF != 0.0f || wasdR != 0.0f )
		{
			const Vec3 worldUp = { 0.0f, 1.0f, 0.0f };
			Vec3 right = Cross( worldUp, forward );
			const float rlen = sqrtf( right.x * right.x + right.y * right.y + right.z * right.z );
			if ( rlen > 1.0e-6f )
			{
				right.x /= rlen;
				right.y /= rlen;
				right.z /= rlen;
			}
			const float step = m_speed * dt;
			eye.x += forward.x * wasdF * step + right.x * wasdR * step;
			eye.y += forward.y * wasdF * step + right.y * wasdR * step;
			eye.z += forward.z * wasdF * step + right.z * wasdR * step;
		}

		m_pivot.x = eye.x - forward.x * m_radius;
		m_pivot.y = eye.y - forward.y * m_radius;
		m_pivot.z = eye.z - forward.z * m_radius;

		if ( m_speedScrollAccum != 0.0f )
		{
			m_speed += m_speedScrollAccum * FLY_SPEED_STEP;
			m_speed = ClampFloat( m_speed, MIN_SPEED, MAX_SPEED );
		}
	}
	else
	{
		if ( m_orbitDX != 0.0f || m_orbitDY != 0.0f )
		{
			m_yaw -= m_orbitDX * ORBIT_SENS;
			m_pitch -= m_orbitDY * ORBIT_SENS;
			m_pitch = ClampFloat( m_pitch, -PITCH_LIM, PITCH_LIM );
		}

		if ( m_panDX != 0.0f || m_panDY != 0.0f )
		{
			const float panScale = PAN_SENS * m_radius;
			m_pivot.x -= m_right.x * m_panDX * panScale;
			m_pivot.y -= m_right.y * m_panDX * panScale;
			m_pivot.z -= m_right.z * m_panDX * panScale;
			m_pivot.x += m_up.x * m_panDY * panScale;
			m_pivot.y += m_up.y * m_panDY * panScale;
			m_pivot.z += m_up.z * m_panDY * panScale;
		}

		if ( m_radialZoomDY != 0.0f )
		{
			m_radius -= RADIAL_ZOOM_SENS * m_radialZoomDY;
			m_radius = ClampFloat( m_radius, MIN_DIST, kViewDistance );
		}

		if ( m_scrollAccum != 0.0f )
		{
			m_radius *= powf( ZOOM_STEP, m_scrollAccum );
			m_radius = ClampFloat( m_radius, MIN_DIST, kViewDistance );
		}
	}

	m_yaw = UnwindAngle( m_yaw );

	m_orbitDX = 0.0f;
	m_orbitDY = 0.0f;
	m_panDX = 0.0f;
	m_panDY = 0.0f;
	m_radialZoomDY = 0.0f;
	m_scrollAccum = 0.0f;
	m_speedScrollAccum = 0.0f;

	RebuildBasisAndView( *this );
	RebuildProj( aspect );
}
