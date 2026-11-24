#include "mech_sample.h"

#include <math.h>

using namespace mech;

class GaitLab : public MechSample
{
public:
	explicit GaitLab( SampleContext* context )
		: MechSample( context, 1 )
	{
		s_kind = EnvInt( "MECH_KIND", s_kind );
		s_mode = EnvInt( "MECH_LAB_MODE", s_mode );		  
		s_surface = EnvInt( "MECH_LAB_SURFACE", s_surface ); 
		int throttlePct = EnvInt( "MECH_LAB_THROTTLE", -1 );  
		if ( throttlePct >= 0 )
		{
			s_throttle = ClampFloat( (float)throttlePct * 0.01f, 0.0f, 1.0f );
		}
		m_world.BuildLab( s_surface, (uint64_t)s_seed );
		if ( StartCore() )
		{
			s_kind = ClampInt( s_kind, 0, (int)m_kindNames.size() - 1 );
			m_player = AddRig( s_kind, kStart, 0.0f, false );
			m_player = 0;
			if ( Alive() )
			{
				mech::Step( 1.0f / 60.0f, m_inputs.data(), (int)m_rigs.size() );
				FetchPoses();
			}
		}
		InitCamera( kStart, 0.0f, 10.0f, Alive() ? DefaultCameraDistance() : 12.0f );
	}

	static Sample* Create( SampleContext* context )
	{
		return new GaitLab( context );
	}

	bool AimFollowsHeading() const override
	{
		return true; 
	}

	void Update( float dt ) override
	{
		if ( Alive() == false || m_halted )
		{
			return;
		}
		float simDt = dt * s_timeScale;
		Rig& r = Player();
		memset( r.input, 0, sizeof( r.input ) );
		const float* p = PlayerPose();
		Vec3 pos = PV( p, PosePos );

		SteerTo( p, { 0.0f, 0.0f, pos.z + 15.0f }, r.input );
		r.input[InFwd] = s_throttle;
		r.input[InRun] = s_mode == 2 ? 1.0f : 0.0f;
		r.input[InSlow] = s_mode == 0 ? 1.0f : 0.0f;
		ReadKeys( r.input );

		m_cooldown -= simDt;
		if ( m_cooldown <= 0.0f && m_stats.mode.empty() == false )
		{
			if ( s_crouch != m_stats.crouchToggle )
			{
				m_pulse[InCrouch] = 1.0f;
				m_cooldown = 0.5f;
			}
			else if ( r.info.climber && s_climb != m_stats.climbEnabled )
			{
				m_pulse[InClimb] = 1.0f;
				m_cooldown = 0.5f;
			}
		}

		StepCore( dt );
		if ( m_halted )
		{
			return;
		}
		pos = PV( PlayerPose(), PosePos );
		if ( pos.z > kLaneEnd || pos.y < -20.0f || fabsf( pos.x ) > 30.0f )
		{
			m_laps += pos.z > kLaneEnd ? 1 : 0;
			RespawnRig( m_player, r.info.kind, kStart, 0.0f );
			m_camTarget = kStart;
		}
	}

	void Render() override
	{
		UpdateCamera();
		DrawScene();
		Vec4 dim = Color( Palette::LineDim );
		DrawLine( { -6.0f, 0.02f, kStart.z }, { 6.0f, 0.02f, kStart.z }, dim );
		DrawLine( { -6.0f, 0.02f, kLaneEnd }, { 6.0f, 0.02f, kLaneEnd }, dim );

		HudMechLine();
		if ( Alive() )
		{
			DrawTextLine( "%s lane   %.0f / %.0f m   laps %d", s_surfaceNames[s_surface], PlayerPose()[PosePos + 2], kLaneEnd, m_laps );
		}
		HudMessages();
	}

	void SwitchKind( int kind )
	{
		if ( kind < 0 || kind >= (int)m_kindNames.size() )
		{
			return;
		}
		s_kind = kind;
		RespawnRig( m_player, kind, kStart, 0.0f );
		m_camTarget = kStart;
		m_camDist = DefaultCameraDistance();
		m_stats = MechStats();
		SetMessage( Palette::Highlight, "%s: %s", Player().info.name.c_str(), Player().info.blurb.c_str() );
	}

	void DrawControls() override
	{
		PanelErrors();
		if ( Alive() == false )
		{
			PanelDebug();
			return;
		}
		if ( ui::Section( "Setup" ) )
		{
			int kind = Player().info.kind;
			if ( KindCombo( &kind ) )
			{
				SwitchKind( kind );
			}
			ui::Hint( "%s", Player().info.blurb.c_str() );
			if ( ui::Combo( "Surface", &s_surface, s_surfaceNames, World::LabSurfaceCount ) )
			{
				m_context->requestRestart = true;
			}
			if ( s_surface == World::LabRubble && ui::SeedInput( "Seed", &s_seed ) )
			{
				m_context->requestRestart = true;
			}
		}
		if ( ui::Section( "Drive" ) )
		{
			ui::Combo( "Mode", &s_mode, s_modeNames, 3 );
			ui::SliderFloat( "Throttle", &s_throttle, 0.0f, 1.0f, "%.2f" );
			ui::Checkbox( "Crouch", &s_crouch );
			if ( Player().info.climber )
			{
				ui::Checkbox( "Wall-climb", &s_climb );
			}
			if ( ui::Button( "Force next gait (G)" ) )
			{
				m_pulse[InGait] = 1.0f;
			}
			if ( ui::Button( "Shove (K)" ) )
			{
				m_pulse[InImpact] = 1.0f;
			}
			if ( ui::Button( "Jump (Space)" ) )
			{
				m_pulse[InJump] = 1.0f;
			}
			if ( ui::Button( "Back to the start" ) )
			{
				RespawnRig( m_player, Player().info.kind, kStart, 0.0f );
				m_camTarget = kStart;
			}
			ui::Hint( "The mech holds the lane by itself. Keys add to the throttle; 1 - 9 switch the mech." );
		}
		PanelGaitCore();
		ui::Divider( "Speed (m/s)" );
		m_speedPlot.Draw( { "speed" }, 6.0f );
		PanelDebug();
		PanelControls();
	}

	void DrawMetricsTabs() override
	{
		MetricsTabs();
	}

	void Keyboard( int key, int mods ) override
	{
		(void)mods;
		if ( CommonKey( key ) || Alive() == false )
		{
			return;
		}
		if ( key >= KEY_1 && key <= KEY_9 )
		{
			SwitchKind( key - KEY_1 );
		}
	}

	bool GetBounds( AABB* bounds ) override
	{
		*bounds = MechBounds();
		return true;
	}

	static constexpr Vec3 kStart = { 0.0f, 0.0f, 2.0f };
	static constexpr float kLaneEnd = 400.0f; 
	static inline const char* const s_surfaceNames[] = { "Flat", "Stairs", "Rubble", "Slopes", "Ledges" };
	static inline const char* const s_modeNames[] = { "Slow walk", "Walk", "Run" };

	int m_laps = 0;
	float m_cooldown = 0.0f;

	static inline int s_kind = 0;
	static inline int s_surface = World::LabStairs;
	static inline int s_seed = 7;
	static inline int s_mode = 1;
	static inline float s_throttle = 0.2f; 
	static inline bool s_crouch = false;
	static inline bool s_climb = false;
};

static int sampleGaitLab = RegisterSample( "Mechs", "Gait Lab", GaitLab::Create,
										   "One mech on a test lane (stairs, rubble, slopes, ledges) with a side camera, footfall diagram and gait plots." );
