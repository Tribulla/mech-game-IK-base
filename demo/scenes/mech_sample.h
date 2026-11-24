#pragma once

#include "mech_draw.h"

#include "gfx/keycodes.h"
#include "imgui.h"
#include "sample.h"
#include "ui.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

namespace mech
{

struct Rig
{
	int handle = -1;
	MechInfo info;
	int offset = 0; 
	bool ai = false;
	Vec3 waypoints[4] = {};
	int waypoint = 0;
	float stuckTime = 0.0f;
	Vec3 stuckPos = { 0.0f, 0.0f, 0.0f };
	float input[kInputStride] = {};
};

enum CameraMode
{
	CamAim = 0, 
	CamSide,	
	CamFree,	
	CamModeCount
};

inline const char* const* CameraModeNames()
{
	static const char* const names[] = { "Aim (third person)", "Side (gait view)", "Free" };
	return names;
}

inline float WrapPi( float a )
{
	while ( a > MATH_PI )
		a -= 2.0f * MATH_PI;
	while ( a < -MATH_PI )
		a += 2.0f * MATH_PI;
	return a;
}

inline float Heading( const float* p )
{
	if ( fabsf( p[PoseFwd + 1] ) > 0.7f )
	{
		return atan2f( p[PoseRight + 2], -p[PoseRight] ); 
	}
	return atan2f( p[PoseFwd], p[PoseFwd + 2] );
}

class MechSample : public Sample
{
public:
	explicit MechSample( SampleContext* context, int cameraSlot )
		: Sample( context )
		, m_cameraSlot( cameraSlot )
	{
		m_logEvery = EnvInt( "MECH_LOG", 0 );
		if ( EnvInt( "MECH_DEBUG", 0 ) != 0 )
		{
			s_debug = true;
		}
	}

	~MechSample() override
	{
		if ( ActiveWorld() == &m_world )
		{
			ActiveWorld() = nullptr;
		}
		SetMouseLook( false );
	}

	void MouseMove( Vec2 p ) override
	{
		(void)p;
		bool rightLook = m_camera->m_rightDown && m_camera->m_altDown == false && CamMode() != CamFree;
		if ( m_mouseLook == false && rightLook == false )
		{
			return;
		}
		const float sens = 0.0035f;
		m_aimYaw -= m_context->mouseDX * sens;
		m_aimPitch = ClampFloat( m_aimPitch - m_context->mouseDY * sens, kAimPitchMin, kAimPitchMax );
		if ( CamMode() == CamSide )
		{
			CamMode() = CamAim;
		}
	}

protected:

	bool StartCore()
	{
		m_world.BuildGrid();
		ActiveWorld() = &m_world;
		if ( m_world.HasTerrain() )
		{
			m_terrain.Build( m_world );
		}
		m_ok = Reset( true );
		if ( m_ok )
		{
			m_kindNames = KindNames();
			SetDebug( s_debug );
		}
		return m_ok;
	}

	int AddRig( int kind, Vec3 ground, float yaw, bool ai, bool relayout = true )
	{
		Rig r;
		r.handle = Spawn( kind, ground, yaw );
		if ( r.handle < 0 || Info( r.handle, &r.info ) == false )
		{
			m_ok = false;
			return -1;
		}
		r.ai = ai;
		SetPatrol( r, ground );
		m_rigs.push_back( r );
		if ( relayout )
		{
			Relayout();
		}
		return (int)m_rigs.size() - 1;
	}

	void RespawnRig( int i, int kind, Vec3 ground, float yaw )
	{
		Rig& r = m_rigs[i];
		if ( Respawn( r.handle, kind, ground, yaw ) && Info( r.handle, &r.info ) )
		{
			Relayout();
			FetchPoses();
			if ( i == m_player )
			{
				m_footfall.Clear();
				m_camYaw = yaw + MATH_PI;
				m_aimYaw = yaw;
			}
		}
	}

	static void SetPatrol( Rig& r, Vec3 home )
	{
		float s = r.info.mass >= 5.0f ? 18.0f : 8.0f;
		r.waypoints[0] = Add( home, { s, 0.0f, 0.0f } );
		r.waypoints[1] = Add( home, { s, 0.0f, s } );
		r.waypoints[2] = Add( home, { -s, 0.0f, s } );
		r.waypoints[3] = Add( home, { -s, 0.0f, 0.0f } );
		r.waypoint = 0;
	}

	void Relayout()
	{
		int off = 0;
		for ( Rig& r : m_rigs )
		{
			r.offset = off;
			off += r.info.floats;
		}
		m_pose.resize( (size_t)off + 1, 0.0f );
		m_inputs.assign( m_rigs.size() * kInputStride, 0.0f );
	}

	void FetchPoses()
	{
		Poses( m_pose.data(), (int)m_pose.size() );
	}

	const float* PoseOf( int i ) const
	{
		return m_pose.data() + m_rigs[i].offset;
	}

	const float* PlayerPose() const
	{
		return PoseOf( m_player );
	}

	Rig& Player()
	{
		return m_rigs[m_player];
	}

	bool Alive() const
	{
		return m_ok && m_rigs.empty() == false;
	}

	void ReadKeys( float* in )
	{
		if ( m_camera->m_altDown || ( m_camera->m_rightDown && CamMode() == CamFree ) )
		{
			return;
		}
		float dt = ClampFloat( ImGui::GetIO().DeltaTime, 0.0f, 0.05f );
		auto key = []( int k ) { return IsKeyDown( k ) ? 1.0f : 0.0f; };
		float look = key( KEY_RIGHT ) + key( KEY_E ) - key( KEY_LEFT ) - key( KEY_Q );
		m_aimPitch = ClampFloat( m_aimPitch + ( key( KEY_UP ) - key( KEY_DOWN ) ) * 1.2f * dt, kAimPitchMin, kAimPitchMax );
		float mvF = key( KEY_W ) - key( KEY_S );
		float mvR = key( KEY_D ) - key( KEY_A );
		const float* p = PlayerPose();
		if ( CamMode() == CamAim && AimFollowsHeading() == false )
		{
			m_aimYaw -= look * 2.2f * dt; 
			Vec3 aimF = { sinf( m_aimYaw ), 0.0f, cosf( m_aimYaw ) };
			Vec3 aimR = { -aimF.z, 0.0f, aimF.x }; 
			Vec3 move = Add( MulSV( mvF, aimF ), MulSV( mvR, aimR ) );
			float ml = Length( move );
			if ( ml > 1.0f )
			{
				move = MulSV( 1.0f / ml, move );
			}
			float hdg = Heading( p );
			Vec3 mf = { sinf( hdg ), 0.0f, cosf( hdg ) };
			Vec3 mr = { -mf.z, 0.0f, mf.x };
			float fwdIn = Dot( move, mf );
			float strafeIn = Dot( move, mr );
			bool onWall = ( (int)p[PoseFlags] & FlagWall ) != 0;
			bool drifting = ( (int)p[PoseFlags] & FlagDrift ) != 0 || key( KEY_X ) > 0.5f;
			if ( onWall == false )
			{

				float turnIn = ClampFloat( -3.0f * WrapPi( m_aimYaw - hdg ), -1.0f, 1.0f );
				float u = ClampFloat( ( fabsf( turnIn ) - 0.25f ) / 0.50f, 0.0f, 1.0f );
				float turnMag = u * u * ( 3.0f - 2.0f * u );
				if ( turnMag > 0.0f && !drifting )
				{
					bool turnAligned = strafeIn * turnIn > 0.0f;
					bool sharpAboutFace = fwdIn < -0.1f && Dot( PV( p, PoseVel ), mf ) > 4.0f && fabsf( turnIn ) > 0.75f;
					if ( turnAligned || sharpAboutFace )
					{
						float inLen = MinFloat( 1.0f, sqrtf( fwdIn * fwdIn + strafeIn * strafeIn ) );
						float alignFwd = ClampFloat( 0.55f + 0.90f * fwdIn, 0.0f, 1.0f );
						float wantFwd = inLen * alignFwd;
						fwdIn += ( wantFwd - fwdIn ) * turnMag;
						strafeIn *= 1.0f - turnMag * ( 1.0f - 0.12f * alignFwd );
					}
				}
				in[InTurn] = turnIn;
			}
			else
			{
				in[InTurn] += look; 
			}
			in[InFwd] += fwdIn;
			in[InStrafe] += strafeIn;
		}
		else
		{
			in[InFwd] += mvF;
			in[InStrafe] += mvR;
			in[InTurn] += look;
		}
		in[InAim] = m_aimPitch;
		in[InFwd] = ClampFloat( in[InFwd], -1.0f, 1.0f );
		in[InStrafe] = ClampFloat( in[InStrafe], -1.0f, 1.0f );
		in[InTurn] = ClampFloat( in[InTurn], -1.0f, 1.0f );
		in[InRun] = MaxFloat( in[InRun], MaxFloat( key( KEY_LEFT_SHIFT ), key( KEY_RIGHT_SHIFT ) ) );
		in[InSlow] = MaxFloat( in[InSlow], MaxFloat( key( KEY_LEFT_CONTROL ), key( KEY_RIGHT_CONTROL ) ) );
		in[InCrouch] = MaxFloat( in[InCrouch], key( KEY_C ) );
		in[InJump] = MaxFloat( in[InJump], key( KEY_SPACE ) );
		in[InDrift] = MaxFloat( in[InDrift], key( KEY_X ) );
		in[InClimb] = MaxFloat( in[InClimb], key( KEY_Z ) );
		in[InImpact] = MaxFloat( in[InImpact], key( KEY_K ) );
		in[InGait] = MaxFloat( in[InGait], key( KEY_G ) );
	}

	Vec3 AimDir() const
	{
		float cp = cosf( m_aimPitch );
		return { sinf( m_aimYaw ) * cp, sinf( m_aimPitch ), cosf( m_aimYaw ) * cp };
	}

	virtual bool AimFollowsHeading() const
	{
		return false;
	}

	void SetMouseLook( bool on )
	{
		if ( on == m_mouseLook )
		{
			return;
		}
		m_mouseLook = on;
		sapp_lock_mouse( on );
		m_camera->m_thirdPerson = on;
	}

	static float SteerTo( const float* p, Vec3 target, float* in, float* distOut = nullptr )
	{
		Vec3 pos = PV( p, PosePos ), fwd = PV( p, PoseFwd );
		Vec3 to = Sub( target, pos );
		to.y = 0.0f;
		float d = Length( to );
		if ( distOut )
		{
			*distOut = d;
		}
		if ( d < 1e-3f )
		{
			return 0.0f;
		}
		Vec3 fh = { fwd.x, 0.0f, fwd.z };
		if ( Length( fh ) < 0.1f )
		{
			fh = { 0.0f, 0.0f, 1.0f };
		}
		fh = Normalize( fh );
		to = MulSV( 1.0f / d, to );

		Vec3 right = { -fh.z, 0.0f, fh.x };
		float ang = atan2f( Dot( right, to ), Dot( fh, to ) );
		in[InTurn] = ClampFloat( ang * 2.0f, -1.0f, 1.0f );
		in[InFwd] = fabsf( ang ) < 1.0f ? 1.0f : 0.25f;
		return ang;
	}

	static float ThrottleFor( const MechInfo& info, float speed )
	{
		return ClampFloat( speed / MaxFloat( info.walkSpeed, 0.1f ), 0.0f, 1.0f );
	}

	void AiInput( int i, float dt )
	{
		Rig& r = m_rigs[i];
		memset( r.input, 0, sizeof( r.input ) );
		const float* p = PoseOf( i );
		float d;
		SteerTo( p, r.waypoints[r.waypoint], r.input, &d );
		r.input[InFwd] *= ThrottleFor( r.info, 3.5f );
		if ( d < 2.5f )
		{
			r.waypoint = ( r.waypoint + 1 ) & 3;
		}
		r.stuckTime += dt;
		if ( r.stuckTime > 5.0f )
		{
			Vec3 pos = PV( p, PosePos );
			if ( Distance( pos, r.stuckPos ) < 1.0f )
			{
				r.waypoint = ( r.waypoint + 1 ) & 3;
			}
			r.stuckTime = 0.0f;
			r.stuckPos = pos;
		}
	}

	void StepCore( float dt )
	{
		float simDt = dt * s_timeScale;
		for ( int k = 0; k < kInputStride; ++k )
		{
			if ( m_pulse[k] > 0.0f ) 
			{
				Player().input[k] = MaxFloat( Player().input[k], m_pulse[k] );
			}
			m_pulse[k] = 0.0f;
		}
		for ( size_t i = 0; i < m_rigs.size(); ++i )
		{
			memcpy( &m_inputs[i * kInputStride], m_rigs[i].input, sizeof( float ) * kInputStride );
		}
		if ( mech::Step( simDt, m_inputs.data(), (int)m_rigs.size() ) == false )
		{
			m_halted = true;
			return;
		}
		FetchPoses();
		m_events.resize( 4096 * kEventFloats );
		int nev = Events( m_events.data(), (int)m_events.size() );
		m_lastEvents = m_fx.HandleEvents( m_events.data(), nev );
		m_fx.Update( simDt );
		m_simTime += simDt;

		const float* p = PlayerPose();
		m_footfall.Record( p, Player().info, simDt );
		if ( ( m_stepCount & 3 ) == 0 || m_stats.mode.empty() )
		{
			Stats( Player().handle, &m_stats );
		}
		m_speedPlot.Add( m_time, { p[PoseSpeed] } );
		m_supportPlot.Add( m_time, { p[PoseSupport] } );
		m_stretchPlot.Add( m_time, { m_stats.maxStretch } );

		m_debugFloats = 0;
		if ( s_debug )
		{
			m_debug.resize( 200000 );
			m_debugFloats = DebugLines( Player().handle, m_debug.data(), (int)m_debug.size() );
		}
		if ( HasError() )
		{
			m_halted = true;
		}
		if ( m_logEvery > 0 && m_stepCount % m_logEvery == 0 )
		{
			printf( "[mech] step %5d %-8s pos (%6.1f %5.1f %6.1f) speed %4.1f %-9s gait %-11s planted %d/%d stretch %.2f "
					"sim %.2f ms py_err %d\n",
					m_stepCount, Player().info.name.c_str(), p[0], p[1], p[2], p[PoseSpeed], ModeLabel( p ), m_stats.gait.c_str(),
					(int)p[PosePlanted], Player().info.legs, m_stats.maxStretch, m_stats.simMs, HasError() ? 1 : 0 );
			fflush( stdout );
		}
	}

	void ReloadPython()
	{
		if ( ActiveBackend() != BackendPython )
		{
			SetMessage( Palette::Info, "Using native C backend (switch to Python for F5 reload)" );
			return;
		}
		if ( m_ok == false )
		{
			m_context->requestRestart = true;
			return;
		}
		if ( HotReload() )
		{
			for ( Rig& r : m_rigs )
			{
				Info( r.handle, &r.info );
			}
			Relayout();
			FetchPoses();
			SetDebug( s_debug );
			m_halted = false;
			SetMessage( Palette::Good, "Python gait code reloaded" );
		}
		else
		{
			m_halted = true;
			SetMessage( Palette::Error, "Python reload failed: see the panel" );
		}
	}

	int& CamMode()
	{
		return s_camMode[m_cameraSlot];
	}

	void InitCamera( Vec3 ground, float yaw, float pitchDeg, float dist )
	{
		m_aimYaw = yaw;

		m_aimPitch = ClampFloat( (float)EnvInt( "MECH_AIM_PITCH", 0 ) * MATH_DEG_TO_RAD, kAimPitchMin, kAimPitchMax );
		m_camYaw = yaw + MATH_PI;
		m_camPitch = pitchDeg * MATH_DEG_TO_RAD;
		m_camDist = dist;
		m_camTarget = ground;
		if ( m_context->restart == false )
		{
			m_camera->SetView( m_camYaw * MATH_RAD_TO_DEG, pitchDeg, dist, ground );
		}
		else
		{

			m_camYaw = m_camera->m_yaw;
			m_camPitch = m_camera->m_pitch;
			m_camDist = m_camera->m_radius;
		}
		m_lastYaw = m_camera->m_yaw;
		m_lastPitch = m_camera->m_pitch;
		m_lastRadius = m_camera->m_radius;
	}

	float DefaultCameraDistance() const
	{
		const MechInfo& I = m_rigs[m_player].info;
		return 1.3f * ( 7.0f + 4.2f * MaxFloat( I.half.x, I.half.z ) );
	}

	void UpdateCamera()
	{
		float dt = ClampFloat( ImGui::GetIO().DeltaTime, 0.0f, 0.05f );
		if ( Alive() == false || CamMode() == CamFree )
		{
			return;
		}
		if ( m_lastRadius > 0.0f && fabsf( m_camera->m_radius - m_lastRadius ) > 1e-3f )
		{
			m_camDist = ClampFloat( m_camDist * m_camera->m_radius / m_lastRadius, 2.0f, 200.0f );
		}

		const float* p = PlayerPose();
		const MechInfo& I = Player().info;

		float above = CamMode() == CamAim ? 1.2f + 0.35f * MaxFloat( I.half.x, I.half.z ) : 0.6f;
		Vec3 target = Add( PV( p, PosePos ), { 0.0f, above + I.half.y, 0.0f } );

		if ( m_context->pause == false )
		{
			m_camTarget = MulAdd( m_camTarget, dt * s_timeScale, PV( p, PoseVel ) );
		}
		m_camTarget = Lerp( m_camTarget, target, 1.0f - expf( -12.0f * dt ) );
		float heading = Heading( p );
		bool onWall = ( (int)p[PoseFlags] & FlagWall ) != 0;
		if ( AimFollowsHeading() || onWall )
		{
			m_aimYaw += WrapPi( heading - m_aimYaw ) * ( 1.0f - expf( -2.5f * dt ) );
		}
		float yaw = m_camYaw, pitch = m_camPitch, dist = m_camDist;
		if ( CamMode() == CamAim )
		{

			yaw = m_aimYaw + MATH_PI;
			pitch = ClampFloat( 0.34f - 0.8f * m_aimPitch, -0.35f, 1.45f );
		}
		if ( CamMode() == CamSide )
		{
			yaw = heading + 0.5f * MATH_PI;
			pitch = 0.12f;
			dist *= 0.9f;
		}
		Vec3 dir = { sinf( yaw ) * cosf( pitch ), sinf( pitch ), cosf( yaw ) * cosf( pitch ) };
		float t;
		Vec3 hp, hn;
		if ( m_world.Raycast( m_camTarget, dir, dist, &t, &hp, &hn, nullptr ) )
		{
			dist = MaxFloat( 1.5f, t - 0.4f );
		}

		m_camera->m_pivot = m_camTarget;
		m_camera->m_yaw = yaw;
		m_camera->m_pitch = pitch;
		m_camera->m_radius = dist;
		m_camera->UpdateTransform();
		m_lastYaw = yaw;
		m_lastPitch = pitch;
		m_lastRadius = dist;
	}

	void DrawScene()
	{
		if ( m_world.HasTerrain() )
		{
			m_terrain.Draw();
			DrawGroundPlane( 450.0f, m_world.groundPlaneY );
		}
		else
		{
			DrawGroundPlane( 450.0f, 0.0f );
		}
		m_world.DrawBoxes();
		if ( Alive() == false )
		{
			return;
		}
		Vec3 aim = AimDir();
		bool aiming = CamMode() == CamAim;
		for ( size_t i = 0; i < m_rigs.size(); ++i )
		{
			bool driven = (int)i == m_player;
			DrawMech( PoseOf( (int)i ), m_rigs[i].info, driven, driven && aiming ? &aim : nullptr );
		}
		DrawDrivenMarker( m_world, PlayerPose(), Player().info );
		if ( aiming )
		{

			const float* p = PlayerPose();
			Vec3 from = Add( PV( p, PosePos ), MulSV( Player().info.half.y + 0.5f, PV( p, PoseUp ) ) );
			float t;
			Vec3 hp, hn;
			if ( m_world.Raycast( from, aim, 250.0f, &t, &hp, &hn, nullptr ) )
			{
				DrawTargetMarker( hp, 0.3f + 0.012f * t, Color( Palette::Highlight ) );
			}
		}
		m_fx.Draw();
		if ( s_debug && m_debugFloats > 0 )
		{
			DrawDebugLines( m_debug.data(), m_debugFloats );
		}
	}

	void SetMessage( uint32_t paletteHex, const char* format, ... )
	{
		va_list args;
		va_start( args, format );
		vsnprintf( m_message, sizeof( m_message ), format, args );
		va_end( args );
		m_messageColor = paletteHex;
		m_messageTime = 3.0f;
		if ( m_logEvery > 0 )
		{
			printf( "[msg] step %5d  %s\n", m_stepCount, m_message );
			fflush( stdout );
		}
	}

	void HudMechLine()
	{
		if ( Alive() == false )
		{
			return;
		}
		const float* p = PlayerPose();
		DrawTextLine( "%s  %s  %4.1f m/s  gait %s%s", Player().info.name.c_str(), ModeLabel( p ), p[PoseSpeed], m_stats.gait.c_str(),
					  m_stats.gaitOverride ? " (forced)" : "" );
	}

	void HudMessages()
	{
		float dt = ClampFloat( ImGui::GetIO().DeltaTime, 0.0f, 0.05f );
		if ( m_halted || HasError() )
		{
			DrawTextLineColored( Palette::Error, "Gait core error: see the panel, then F5 or R (restart)." );
		}
		else if ( m_messageTime > 0.0f )
		{
			DrawTextLineColored( m_messageColor, "%s", m_message );
		}
		m_messageTime -= dt;
	}

	bool KindCombo( int* kind )
	{
		if ( m_kindNames.empty() )
		{
			return false;
		}
		std::vector<const char*> names;
		for ( const std::string& s : m_kindNames )
		{
			names.push_back( s.c_str() );
		}
		return ui::Combo( "Mech type", kind, names.data(), (int)names.size() );
	}

	void PanelErrors()
	{
		if ( m_ok && HasError() == false && m_halted == false )
		{
			return;
		}
		ui::Status( Palette::Error, m_ok ? "Gait core raised an exception" : "Gait core not running" );
		const std::string& e = LastError().empty() ? InitMessage() : LastError();

		size_t start = 0;
		int lines = 0;
		for ( size_t i = e.size(); i-- > 0; )
		{
			if ( e[i] == '\n' && ++lines > 6 )
			{
				start = i + 1;
				break;
			}
		}
		ui::Hint( "%s", e.c_str() + start );
		ui::Hint( "Fix the code in IK/python/gaitcore, then press F5 (reload in place) or R (restart)." );
	}

	void PanelGaitCore()
	{
		if ( ui::Section( "Gait core" ) == false || Alive() == false )
		{
			return;
		}
		const float* p = PlayerPose();
		const MechInfo& I = Player().info;
		ui::Stat( "mode", "%s", m_stats.mode.c_str() );
		ui::Stat( "gait", "%s%s", m_stats.gait.c_str(), m_stats.gaitOverride ? " (forced)" : "" );
		ui::Stat( "state", "%s", m_stats.state.c_str() );
		ui::Stat( "speed", "%.2f m/s", p[PoseSpeed] );
		ui::Stat( "duty / cycle", "%.2f / %.2f s", m_stats.duty, m_stats.cycle );
		ui::Stat( "feet planted", "%d / %d", (int)p[PosePlanted], I.legs );
		if ( m_stats.maxStretch > 1.0f )
		{
			ui::Status( Palette::Warning, "max stretch %.2f (over reach)", m_stats.maxStretch );
		}
		else
		{
			ui::Stat( "max stretch", "%.2f of reach", m_stats.maxStretch );
		}
		if ( p[PoseSupport] >= 0.0f || p[PosePlanted] < 1.0f ) 
		{
			ui::Stat( "support", "%+.2f m", p[PoseSupport] );
		}
		else
		{
			ui::Status( Palette::Warning, "CoM outside support by %.2f m", -p[PoseSupport] );
		}
		if ( I.climber )
		{
			ui::Stat( "wall-climb", "%s", m_stats.climbEnabled ? "on" : "off" );
		}
		ui::Stat( ActiveBackend() == BackendC ? "c step" : "python step", "%.2f ms, %d rays", m_stats.simMs, m_stats.rays );
		ui::Divider( "Footfall (filled = stance)" );
		m_footfall.DrawImGui( ImGui::GetFontSize() * 0.9f );
	}

	void PanelDebug()
	{
		if ( ui::Section( "Debug" ) == false )
		{
			return;
		}
		if ( HasPythonBackend() )
		{
			const char* backends[2] = { "C (Native)", "Python" };
			int b = ActiveBackend() == BackendPython ? 1 : 0;
			if ( ui::Combo( "IK backend", &b, backends, 2 ) )
			{
				SetBackend( b == 1 ? BackendPython : BackendC );
				m_context->requestRestart = true;
			}
		}
		else
		{
			ui::Stat( "IK backend", "C (Native)" );
		}
		if ( ActiveBackend() == BackendC )
		{
			int hw = HardwareThreads();
			int th = WorkerThreads();
			if ( hw > 1 && ui::SliderInt( "C threads", &th, 1, hw ) )
			{
				SetWorkerThreads( th );
			}
		}
		if ( ui::Checkbox( "IK / gait overlay (H)", &s_debug ) )
		{
			SetDebug( s_debug );
		}
		if ( s_debug )
		{
			ui::Hint( "Gray rings: step zones. White cross: predicted foothold. Gold: swing target. Blue: swing path. "
					  "Gold ticks: planted foot normals. Green/red: support polygon. Orange: raycasts this step." );
		}
		ui::Combo( "Camera (V)", &CamMode(), CameraModeNames(), CamModeCount );
		ui::SliderFloat( "Time scale", &s_timeScale, 0.1f, 2.0f, "%.2fx" );
		if ( ActiveBackend() == BackendPython && ui::Button( "Reload Python (F5)" ) )
		{
			ReloadPython();
		}
		if ( m_ok )
		{
			ui::Hint( "%s. Code: %s", InitMessage().c_str(), IkDir().c_str() );
		}
	}

	bool PanelControls()
	{
		if ( ui::Section( "Controls", false ) == false )
		{
			return false;
		}
		ui::KeyHint( "W S / A D", "Move / strafe (toward the view)" );
		ui::KeyHint( "Right drag", "Look around (mech turns; in the air the hull pitches up to 90 deg)" );
		ui::KeyHint( "L / Esc", "Lock / free the mouse for look" );
		ui::KeyHint( "Arrows, Q E", "Turn / pitch the view" );
		ui::KeyHint( "Shift / Ctrl", "Run / slow walk" );
		ui::KeyHint( "C", "Crouch (toggle)" );
		ui::KeyHint( "Space", "Jump" );
		ui::KeyHint( "X", "Quick slide at speed" );
		ui::KeyHint( "Z", "Wall-walk on/off (climber mechs)" );
		ui::KeyHint( "G", "Force next gait" );
		ui::KeyHint( "K", "Shove (balance test)" );
		ui::KeyHint( "1 - 9", "Mech type" );
		ui::KeyHint( "H / V", "IK overlay / camera" );
		ui::KeyHint( "F5", "Reload Python" );
		return true;
	}

	void MetricsTabs()
	{
		if ( ImGui::BeginTabItem( "Footfall" ) )
		{
			float rows = Alive() ? (float)Player().info.legs : 4.0f;
			float h = ImGui::GetContentRegionAvail().y / MaxFloat( rows, 1.0f );
			m_footfall.DrawImGui( ClampFloat( h, 10.0f, 40.0f ) );
			ImGui::EndTabItem();
		}
		if ( ImGui::BeginTabItem( "Gait plots" ) )
		{
			float h = ImGui::GetContentRegionAvail().y / ImGui::GetFontSize() / 3.0f - 0.5f;
			h = MaxFloat( h, 4.0f );
			m_speedPlot.Draw( { "speed" }, h );
			m_supportPlot.Draw( { "support margin" }, h );
			m_stretchPlot.Draw( { "max leg stretch" }, h );
			ImGui::EndTabItem();
		}
	}

	bool CommonKey( int key )
	{
		switch ( key )
		{
			case KEY_H:
				s_debug = !s_debug;
				SetDebug( s_debug );
				return true;
			case KEY_V:
				CamMode() = ( CamMode() + 1 ) % CamModeCount;
				if ( Alive() && CamMode() != CamFree )
				{
					m_camYaw = Heading( PlayerPose() ) + MATH_PI;
				}
				if ( CamMode() == CamFree )
				{
					SetMouseLook( false );
				}
				SetMessage( Palette::TextNormal, "Camera: %s", CameraModeNames()[CamMode()] );
				return true;
			case KEY_L:
				SetMouseLook( m_mouseLook == false );
				if ( m_mouseLook )
				{
					CamMode() = CamAim;
				}
				SetMessage( Palette::TextNormal, m_mouseLook ? "Mouse look on (L or Esc frees the mouse)" : "Mouse look off" );
				return true;
			case KEY_ESCAPE:
				if ( m_mouseLook )
				{
					SetMouseLook( false );
					return true;
				}
				return false;
			case KEY_F5:
				ReloadPython();
				return true;
			default:
				return false;
		}
	}

	AABB MechBounds() const
	{
		if ( Alive() == false )
		{
			return { { -20.0f, 0.0f, -20.0f }, { 20.0f, 10.0f, 20.0f } };
		}
		const float* p = PlayerPose();
		const MechInfo& I = m_rigs[m_player].info;
		float r = 4.0f + 3.0f * MaxFloat( I.half.x, I.half.z );
		Vec3 c = PV( p, PosePos );
		return { Sub( c, { r, r, r } ), Add( c, { r, r, r } ) };
	}

	World m_world;
	TerrainMeshes m_terrain;
	std::vector<Rig> m_rigs;
	int m_player = 0;
	std::vector<float> m_pose, m_inputs, m_events, m_debug;
	int m_debugFloats = 0;
	int m_lastEvents = 0;
	std::vector<std::string> m_kindNames;
	Effects m_fx;
	Footfall m_footfall;
	MechStats m_stats;
	ui::ScrollingPlot m_speedPlot{ "m/s", 8.0f };
	ui::ScrollingPlot m_supportPlot{ "m", 8.0f };
	ui::ScrollingPlot m_stretchPlot{ "of reach", 8.0f };
	float m_pulse[kInputStride] = {}; 
	bool m_ok = false;
	bool m_halted = false;
	double m_simTime = 0.0;
	int m_logEvery = 0;

	int m_cameraSlot;
	float m_camYaw = 0.0f, m_camPitch = 0.3f, m_camDist = 10.0f;
	float m_aimYaw = 0.0f;	 
	float m_aimPitch = 0.0f; 
	bool m_mouseLook = false;
	static constexpr float kAimPitchMin = -1.53f, kAimPitchMax = 1.53f; 
	float m_lastYaw = 0.0f, m_lastPitch = 0.0f, m_lastRadius = -1.0f;
	Vec3 m_camTarget = { 0.0f, 0.0f, 0.0f };

	char m_message[192] = "";
	uint32_t m_messageColor = Palette::TextNormal;
	float m_messageTime = 0.0f;

	static inline bool s_debug = false;
	static inline float s_timeScale = 1.0f;
	static inline int s_camMode[3] = { CamAim, CamSide, CamFree };
};

} 
