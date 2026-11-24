#include "mech_sample.h"

#include <math.h>
#include <stdio.h>

using namespace mech;

class MechSandbox : public MechSample
{
public:
	MechSandbox( SampleContext* context )
		: MechSample( context, 0 )
	{
		m_world.BuildArena();
		s_kind = EnvInt( "MECH_KIND", s_kind );
		if ( EnvInt( "MECH_AUTOPILOT", 0 ) != 0 )
		{
			s_autopilot = true;
		}

		Vec3 start = { 0.0f, 0.12f, 0.0f };
		float yaw = 0.0f;
		m_respawnPos = start;
		m_respawnYaw = yaw;

		if ( StartCore() )
		{
			int kinds = (int)m_kindNames.size();
			s_kind = ClampInt( s_kind, 0, kinds - 1 );
			m_player = AddRig( s_kind, start, yaw, false );
			const Vec3 homes[8] = { { 14, 0, 12 },   { -12, 0, 8 },  { -20, 0, 26 }, { 40, 0, -48 },
									{ -30, 0, -6 }, { 28, 0, 30 },  { -16, 0, -28 }, { 16, 0, -34 } };
			for ( int k = 0, h = 0; k < kinds && h < 8; ++k )
			{
				if ( k != s_kind )
				{
					AddRig( k, homes[h], (float)h * 1.3f, true );
					++h;
				}
			}
			m_player = 0;
			if ( Alive() )
			{
				mech::Step( 1.0f / 60.0f, m_inputs.data(), (int)m_rigs.size() );
				FetchPoses();
			}
		}
		InitCamera( start, yaw, 18.0f, Alive() ? DefaultCameraDistance() : 14.0f );
		SetMessage( Palette::Highlight, "Walk into a gold start gate to run a course, or press T" );
	}

	static Sample* Create( SampleContext* c )
	{
		return new MechSandbox( c );
	}

	void Update( float dt ) override
	{
		if ( Alive() == false || m_halted )
		{
			return;
		}
		float simDt = dt * s_timeScale;
		for ( size_t i = 0; i < m_rigs.size(); ++i )
		{
			Rig& r = m_rigs[i];
			if ( (int)i == m_player )
			{
				memset( r.input, 0, sizeof( r.input ) );
				if ( s_autopilot )
				{
					AutopilotInput( r.input, simDt );
				}
				ReadKeys( r.input );
			}
			else if ( s_aiPatrol )
			{
				AiInput( (int)i, simDt );
			}
			else
			{
				memset( r.input, 0, sizeof( r.input ) );
			}
		}
		StepCore( dt );
		if ( m_halted )
		{
			return;
		}
		UpdateCourses( simDt );
		if ( m_restartTimer > 0.0f )
		{
			m_restartTimer -= simDt;
			if ( m_restartTimer <= 0.0f && s_autopilot && m_running == false && m_course >= 0 && m_course != 2 )
			{
				TeleportToCourse( m_course );
			}
		}
	}

	void CourseStart( int c, Vec3* pos, float* yaw ) const
	{
		const Gate& g = m_world.courses[c].gates[0];
		Vec3 back = { sinf( g.yaw ), 0.0f, cosf( g.yaw ) };
		Vec3 p = MulAdd( g.p, -8.0f, back );
		p.y = m_world.GroundY( p.x, p.z );
		*pos = p;
		*yaw = g.yaw;
	}

	void TeleportToCourse( int c )
	{
		Vec3 p;
		float yaw;
		CourseStart( c, &p, &yaw );
		RespawnRig( m_player, Player().info.kind, p, yaw );
		m_course = -1;
		m_running = false;
		m_courseSel = c;
		m_respawnPos = p;
		m_respawnYaw = yaw;
		SetMessage( Palette::Highlight, "%s: walk through the gold start gate", m_world.courses[c].name );
	}

	static bool GateHit( const Gate& g, Vec3 p )
	{
		Vec3 d = Sub( p, g.p );
		float dy = fabsf( d.y );
		d.y = 0.0f;
		return Length( d ) < g.r && dy < 8.0f;
	}

	void UpdateCourses( float dt )
	{
		Vec3 p = PV( PlayerPose(), PosePos );
		if ( m_running )
		{
			m_courseTime += dt;
			const Course& c = m_world.courses[m_course];
			const Gate& g = c.gates[m_nextGate];
			if ( GateHit( g, p ) )
			{
				m_respawnPos = { g.p.x, m_world.GroundY( g.p.x, g.p.z ), g.p.z };
				m_respawnYaw = g.yaw;
				m_nextGate++;
				m_via = 0;
				int mins = (int)( m_courseTime / 60.0 );
				double secs = fmod( m_courseTime, 60.0 );
				if ( m_nextGate >= (int)c.gates.size() )
				{
					m_running = false;
					float& best = s_best[m_course][Player().info.kind & 15];
					bool record = best == 0.0f || m_courseTime < best;
					if ( record )
					{
						best = (float)m_courseTime;
					}
					SetMessage( Palette::Good, "Finish  %02d:%05.2f%s", mins, secs, record ? "  new best" : "" );
					m_restartTimer = 3.0f; 
				}
				else
				{
					SetMessage( Palette::Good, "Checkpoint %d / %d   %02d:%05.2f", m_nextGate - 1, (int)c.gates.size() - 2, mins, secs );
				}
			}
		}
		else
		{
			for ( int i = 0; i < (int)m_world.courses.size(); ++i )
			{
				if ( GateHit( m_world.courses[i].gates[0], p ) )
				{

					if ( m_course == i && m_nextGate == -1 )
					{
						continue;
					}
					m_course = i;
					m_courseSel = i;
					m_nextGate = 1;
					m_via = 0;
					m_running = true;
					m_courseTime = 0.0;
					m_respawnPos = { p.x, m_world.GroundY( p.x, p.z ), p.z };
					m_respawnYaw = m_world.courses[i].gates[0].yaw;
					SetMessage( Palette::Highlight, "Go!  %s", m_world.courses[i].name );
					break;
				}
			}
			if ( m_course >= 0 && m_running == false && m_nextGate >= 0 && GateHit( m_world.courses[m_course].gates[0], p ) == false )
			{
				m_nextGate = -1;
			}
		}
		if ( p.y < -30.0f )
		{
			RespawnAtCheckpoint();
		}
	}

	void RespawnAtCheckpoint()
	{
		RespawnRig( m_player, Player().info.kind, m_respawnPos, m_respawnYaw );
		m_via = 0;
		SetMessage( Palette::Warning, "Respawned at the last checkpoint" );
	}

	bool AimFollowsHeading() const override
	{
		return s_autopilot;
	}

	void AutopilotInput( float* in, float dt )
	{
		m_autoCooldown -= dt;
		const float* p = PlayerPose();
		const MechInfo& I = Player().info;
		int c = m_running ? m_course : m_courseSel;
		if ( c < 0 )
		{
			c = 0;
		}
		const Course& course = m_world.courses[c];
		int gi = m_running ? m_nextGate : 0;
		if ( gi < 0 )
		{
			gi = 0;
		}
		const Gate& g = course.gates[gi];
		Vec3 target = g.p;
		bool useVia = g.via.empty() == false && ( g.viaWalkersOnly == false || ( I.climber == false && I.wireMax < 7.5f ) );
		if ( useVia && m_via < (int)g.via.size() )
		{
			target = g.via[m_via];
			Vec3 next = m_via + 1 < (int)g.via.size() ? g.via[m_via + 1] : g.p;
			Vec3 d = Sub( target, PV( p, PosePos ) );
			d.y = 0.0f;

			Vec3 pos = PV( p, PosePos );
			Vec3 a = { pos.x - next.x, 0.0f, pos.z - next.z }, b = { target.x - next.x, 0.0f, target.z - next.z };
			if ( Length( d ) < 2.5f || Length( a ) < Length( b ) - 1.0f )
			{
				m_via++;
				target = m_via < (int)g.via.size() ? g.via[m_via] : g.p;
			}
		}
		if ( m_running == false && m_nextGate == -1 && m_course == c )
		{

			target = MulAdd( g.p, 10.0f, Vec3{ sinf( g.yaw ), 0.0f, cosf( g.yaw ) } );
		}
		float dist;
		float ang = SteerTo( p, target, in, &dist );

		float fullFwd = in[InFwd];
		in[InFwd] *= ThrottleFor( I, 5.0f );
		if ( useVia && m_via < (int)g.via.size() && dist < 6.0f )
		{
			in[InFwd] *= 0.35f; 
		}

		Vec3 pos = PV( p, PosePos );

		m_autoStuck += dt;
		if ( dist < m_autoBest - 1.0f )
		{
			m_autoBest = dist;
			m_autoStuck = 0.0f;
		}
		if ( m_autoTarget.x != target.x || m_autoTarget.z != target.z )
		{
			m_autoTarget = target;
			m_autoBest = dist;
			m_autoStuck = 0.0f;
		}
		if ( m_autoStuck > 8.0f )
		{
			m_autoStuck = 0.0f;
			m_autoBest = 1e9f;
			RespawnAtCheckpoint();
			return;
		}
		bool inCrouchZone = false;
		for ( const AABB& z : course.crouchZones )
		{
			inCrouchZone |= AABB_ContainsPoint( z, pos );
		}

		if ( fabsf( ang ) < 0.25f && dist > 10.0f + 0.6f * p[PoseSpeed] && inCrouchZone == false )
		{
			in[InRun] = 1.0f;
			in[InFwd] = fullFwd;
		}
		if ( c == 2 && fabsf( ang ) > 0.4f && p[PoseSpeed] > 3.0f )
		{
			in[InDrift] = 1.0f;
		}
		if ( m_autoCooldown <= 0.0f )
		{
			if ( inCrouchZone != m_stats.crouchToggle )
			{
				m_pulse[InCrouch] = 1.0f;
				m_autoCooldown = 0.6f;
			}
			else if ( I.climber && m_stats.climbEnabled == false )
			{
				m_pulse[InClimb] = 1.0f;
				m_autoCooldown = 0.6f;
			}
		}
	}

	void Keyboard( int key, int mods ) override
	{
		(void)mods;
		if ( CommonKey( key ) )
		{
			return;
		}
		if ( Alive() == false )
		{
			return;
		}
		if ( key >= KEY_1 && key <= KEY_9 )
		{
			SwitchKind( key - KEY_1 );
		}
		else if ( key == KEY_T )
		{
			TeleportToCourse( ( m_courseSel + 1 ) % (int)m_world.courses.size() );
		}
		else if ( key == KEY_N )
		{
			DriveNext();
		}
		else if ( key == KEY_BACKSPACE )
		{
			RespawnAtCheckpoint();
		}
	}

	void SwitchKind( int kind )
	{
		if ( kind < 0 || kind >= (int)m_kindNames.size() )
		{
			return;
		}
		const float* p = PlayerPose();
		Vec3 pos = PV( p, PosePos );
		pos.y = m_world.GroundY( pos.x, pos.z );
		RespawnRig( m_player, kind, pos, Heading( p ) );
		s_kind = kind;
		m_camDist = DefaultCameraDistance();
		SetMessage( Palette::Highlight, "%s: %s", Player().info.name.c_str(), Player().info.blurb.c_str() );
	}

	void DriveNext()
	{
		Rig& old = Player();
		old.ai = true;
		SetPatrol( old, PV( PlayerPose(), PosePos ) );
		m_player = ( m_player + 1 ) % (int)m_rigs.size();
		Player().ai = false;
		m_footfall.Clear();
		m_camYaw = Heading( PlayerPose() ) + MATH_PI;
		m_camDist = DefaultCameraDistance();
		s_kind = Player().info.kind;
		SetMessage( Palette::Highlight, "Now driving %s", Player().info.name.c_str() );
	}

	void DrawGates()
	{
		const Vec3 up = { 0.0f, 1.0f, 0.0f };
		const float height = 4.5f;
		for ( int c = 0; c < (int)m_world.courses.size(); ++c )
		{
			const Course& co = m_world.courses[c];
			bool active = m_running && m_course == c;
			for ( int i = 0; i < (int)co.gates.size(); ++i )
			{
				const Gate& g = co.gates[i];
				if ( c == 2 && i == (int)co.gates.size() - 1 && active == false )
				{
					continue; 
				}
				bool next = active && i == m_nextGate;
				bool start = m_running == false && i == 0;
				bool passed = active && i < m_nextGate;
				Vec3 right = { cosf( g.yaw ), 0.0f, -sinf( g.yaw ) };
				Vec3 l = MulAdd( g.p, -g.r, right ), r = MulAdd( g.p, g.r, right );
				Vec3 lt = MulAdd( l, height, up ), rt = MulAdd( r, height, up );
				if ( active == false && start == false )
				{

					Vec4 dim = Color( Palette::LineDim );
					DrawLine( l, lt, dim );
					DrawLine( r, rt, dim );
					DrawLine( lt, rt, dim );
					continue;
				}
				Material m = next || start ? Materials::Highlight() : passed ? Materials::Good() : Materials::Static();
				DrawRod( l, lt, 0.12f, m );
				DrawRod( r, rt, 0.12f, m );
				DrawRod( lt, rt, 0.09f, m );
				if ( next || start )
				{
					DrawLineEx( g.p, MulAdd( g.p, 40.0f, up ), Color( Palette::Highlight, 0.7f ), Style::LinePx, OVERLAY_THICKNESS_PIXELS,
								OVERLAY_OCCLUSION_DIM );
				}
			}
		}
	}

	void Render() override
	{
		UpdateCamera();
		DrawScene();
		DrawGates();

		HudMechLine();
		if ( m_running )
		{
			const Course& c = m_world.courses[m_course];
			DrawTextLine( "%s   %02d:%05.2f   checkpoint %d / %d", c.name, (int)( m_courseTime / 60.0 ), fmod( m_courseTime, 60.0 ),
						  MaxInt( 0, m_nextGate - 1 ), (int)c.gates.size() - 2 );
		}
		else
		{
			DrawTextLine( "Free roam   T: next course   Backspace: respawn%s", s_autopilot ? "   (autopilot on)" : "" );
		}
		HudMessages();
	}

	static void FormatTime( char* buf, size_t n, double t )
	{
		if ( t <= 0.0 )
		{
			snprintf( buf, n, "--:--.--" );
		}
		else
		{
			snprintf( buf, n, "%02d:%05.2f", (int)( t / 60.0 ), fmod( t, 60.0 ) );
		}
	}

	void DrawControls() override
	{
		PanelErrors();
		if ( Alive() == false )
		{
			PanelDebug();
			return;
		}
		if ( ui::Section( "Mech" ) )
		{
			int kind = Player().info.kind;
			if ( KindCombo( &kind ) )
			{
				SwitchKind( kind );
			}
			ui::Hint( "%s", Player().info.blurb.c_str() );
			if ( ui::Button( "Drive next mech (N)" ) )
			{
				DriveNext();
			}
			ui::Checkbox( "Autopilot", &s_autopilot );
			ui::Checkbox( "AI walkers patrol", &s_aiPatrol );
			if ( ui::Button( "Shove (K)" ) )
			{
				m_pulse[InImpact] = 1.0f;
			}
			if ( ui::Button( "Force next gait (G)" ) )
			{
				m_pulse[InGait] = 1.0f;
			}
			if ( Player().info.climber && ui::Button( m_stats.climbEnabled ? "Wall-climb off (Z)" : "Wall-climb on (Z)" ) )
			{
				m_pulse[InClimb] = 1.0f;
			}
		}
		if ( ui::Section( "Course" ) )
		{
			std::vector<const char*> names;
			for ( const Course& c : m_world.courses )
			{
				names.push_back( c.name );
			}
			ui::Combo( "Course", &m_courseSel, names.data(), (int)names.size() );
			ui::Hint( "%s", m_world.courses[m_courseSel].desc );
			if ( ui::Button( "Go to its start gate (T)" ) )
			{
				TeleportToCourse( m_courseSel );
			}
			if ( ui::Button( "Respawn at checkpoint (Backspace)" ) )
			{
				RespawnAtCheckpoint();
			}
			char t[32];
			if ( m_running )
			{
				FormatTime( t, sizeof( t ), m_courseTime );
				ui::Status( Palette::Highlight, "%s  %s", m_world.courses[m_course].name, t );
			}
			ui::Divider( "Best times" );
			for ( int c = 0; c < (int)m_world.courses.size(); ++c )
			{
				FormatTime( t, sizeof( t ), s_best[c][Player().info.kind & 15] );
				ui::Stat( m_world.courses[c].label, "%s", t );
			}
		}
		PanelGaitCore();
		PanelDebug();
		if ( ui::Section( "Map", false ) )
		{
			DrawMap( ImGui::GetContentRegionAvail().x );
		}
		if ( PanelControls() )
		{
			ui::KeyHint( "T", "Next course" );
			ui::KeyHint( "N", "Drive next mech" );
			ui::KeyHint( "Backspace", "Respawn at checkpoint" );
		}
	}

	void DrawMap( float size )
	{
		ImDrawList* dl = ImGui::GetWindowDrawList();
		ImVec2 o = ImGui::GetCursorScreenPos();
		const float range = 200.0f;
		float k = size / ( 2.0f * range );

		auto map = [&]( float x, float z ) { return ImVec2( o.x + 0.5f * size - x * k, o.y + 0.5f * size - z * k ); };
		dl->AddRect( o, ImVec2( o.x + size, o.y + size ), HexToImU32( Palette::LineDim ) );
		for ( const Box& b : m_world.boxes )
		{
			if ( b.bmax.y < 0.5f )
			{
				continue;
			}
			ImVec2 a = map( b.bmax.x, b.bmax.z ), c = map( b.bmin.x, b.bmin.z );
			dl->AddRectFilled( a, c, HexToImU32( Palette::LineDim, 160 ) );
		}
		for ( int c = 0; c < (int)m_world.courses.size(); ++c )
		{
			const Course& co = m_world.courses[c];
			for ( size_t i = 0; i + 1 < co.gates.size(); ++i )
			{
				dl->AddLine( map( co.gates[i].p.x, co.gates[i].p.z ), map( co.gates[i + 1].p.x, co.gates[i + 1].p.z ),
							 HexToImU32( Palette::Info, 200 ), 1.5f );
			}
			int gi = m_running && m_course == c ? m_nextGate : 0;
			gi = ClampInt( gi, 0, (int)co.gates.size() - 1 );
			ImVec2 gp = map( co.gates[gi].p.x, co.gates[gi].p.z );
			dl->AddRectFilled( ImVec2( gp.x - 3, gp.y - 3 ), ImVec2( gp.x + 3, gp.y + 3 ), HexToImU32( Palette::Highlight ) );
		}
		for ( int i = 0; i < (int)m_rigs.size(); ++i )
		{
			const float* p = PoseOf( i );
			ImVec2 mp = map( p[0], p[2] );
			float s = i == m_player ? 4.0f : 3.0f;
			dl->AddCircleFilled( mp, s, HexToImU32( SeriesHex( m_rigs[i].info.kind ) ) );
			if ( i == m_player )
			{
				dl->AddCircle( mp, s + 2.5f, HexToImU32( Palette::Highlight ), 0, 1.5f );
				dl->AddLine( mp, map( p[0] + p[PoseFwd] * 12.0f, p[2] + p[PoseFwd + 2] * 12.0f ), HexToImU32( Palette::Highlight ), 1.5f );
			}
		}
		ImGui::Dummy( ImVec2( size, size ) );
	}

	void DrawMetricsTabs() override
	{
		MetricsTabs();
		if ( ImGui::BeginTabItem( "Map" ) )
		{
			ImVec2 avail = ImGui::GetContentRegionAvail();
			DrawMap( MinFloat( avail.x, avail.y ) );
			ImGui::EndTabItem();
		}
	}

	bool GetBounds( AABB* bounds ) override
	{
		*bounds = MechBounds();
		return true;
	}

	int m_courseSel = 0;
	int m_course = -1;
	int m_nextGate = 0;
	int m_via = 0;
	bool m_running = false;
	double m_courseTime = 0.0;
	Vec3 m_respawnPos = { 0.0f, 0.0f, 0.0f };
	float m_respawnYaw = 0.0f;
	float m_autoCooldown = 0.0f;
	float m_autoStuck = 0.0f;
	float m_restartTimer = 0.0f;
	float m_autoBest = 1e9f;
	Vec3 m_autoTarget = { 0.0f, 0.0f, 0.0f };

	static inline int s_kind = 0;
	static inline bool s_autopilot = false;
	static inline bool s_aiPatrol = true;
	static inline float s_best[8][16] = {};
};

static int sampleMechSandbox = RegisterSample( "Mechs", "Mech Sandbox", MechSandbox::Create,
											   "Drive 9 procedural mech archetypes (bipeds, quadrupeds, wall-climbers, hexapods, and octopods) around a hub and four timed courses." );
