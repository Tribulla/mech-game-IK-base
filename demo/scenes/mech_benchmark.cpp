#include "mech_sample.h"
#include "mech_scene_worlds.h"

#include "gfx/renderer.h"
#include "gfx/timer.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

using namespace mech;

namespace
{

const char* const kTerrainNames[] = { "Flat", "Rubble", "Frontline" };
const char* const kPatternNames[] = { "Patrol", "Orbit ring", "Sprint", "Idle" };
const char* const kMotionNames[] = { "Slow", "Walk", "Run", "Slide" };

constexpr int kStageCount = 13;
const char* const kStageNames[kStageCount] = {
	"IK: Probes & Gait Sel",
	"IK: Body & Balance",
	"IK: Leg Targets & Pred",
	"IK: Swings & Search",
	"IK: Stepping & Foothold",
	"IK: Foot Clearance",
	"IK: Solver",
	"IK: Obstacle Plane-Roll",
	"IK: Overhead / Other",
	"C++: AI & Input Prep",
	"Bridge: Poses & FX",
	"C++: Scene Draw Submit",
	"GPU: Renderer CPU Pass"
};

constexpr int kMaxBenchMechs = 1000;

} 

class MechBenchmark : public MechSample
{
public:
	explicit MechBenchmark( SampleContext* context )
		: MechSample( context, 2 )
	{
		s_mechCount = ClampInt( EnvInt( "MECH_BENCH_COUNT", s_mechCount ), 1, kMaxBenchMechs );
		s_terrain = ClampInt( EnvInt( "MECH_BENCH_TERRAIN", s_terrain ), 0, 2 );
		s_mechMix = ClampInt( EnvInt( "MECH_BENCH_MIX", s_mechMix ), 0, kProfileKinds );
		s_drawLod = EnvInt( "MECH_BENCH_LOD", s_drawLod ? 1 : 0 ) != 0;

		BuildBenchmarkWorld();

		if ( StartCore() )
		{
			s_mechMix = ClampInt( s_mechMix, 0, (int)m_kindNames.size() );
			SetNativeProfile( true );
			SyncBenchConfig();
			ResizeCrowd( s_mechCount );
			if ( Alive() )
			{
				ResetNativeQueryStats();
				mech::Step( 1.0f / (float)s_simHz, m_inputs.data(), (int)m_rigs.size() );
				FetchPoses();
				CollectStepStats( 0.0f, 0.0f, 1 );
			}
		}

		if ( EnvInt( "MECH_BENCH_PROFILER", 0 ) != 0 )
		{
			m_context->showMetrics = true;
		}
		float camDist = s_mechCount > 64 ? ClampFloat( 32.0f + m_crowdRadius * 1.15f, 52.0f, 260.0f ) : 52.0f;
		float camPitch = s_mechCount > 64 ? 32.0f : 28.0f;
		InitCamera( { 0.0f, 2.0f, 0.0f }, 0.0f, camPitch, camDist );
		if ( m_context->restart == false )
		{
			m_camera->SetView( -35.0f, camPitch, camDist, { 0.0f, 2.0f, 0.0f } );
		}
		SetMessage( Palette::Highlight, "IK Benchmark: tune sliders, press M for profiler" );
	}

	~MechBenchmark() override
	{
		SetNativeProfile( false );
		SetBenchConfig( 1.0f / 60.0f, 0.0f, 20.0f, 1.0f, true, false );
	}

	static Sample* Create( SampleContext* context )
	{
		return new MechBenchmark( context );
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
		if ( m_configDirty )
		{
			SyncBenchConfig();
		}

		float stepDt = 1.0f / (float)s_simHz;
		float scaledDt = dt * s_simSpeed;
		m_stepAcc = MinFloat( m_stepAcc + scaledDt, stepDt * (float)s_maxSubsteps );

		float frameAiMs = 0.0f;
		float framePyCallMs = 0.0f;
		float frameBridgeMs = 0.0f;
		float raw[kProfileFloats] = {};
		float sumStages[9] = {};
		float sumPosesMs = 0.0f;
		float sumKindMs[kProfileKinds] = {};
		int sumKindRays[kProfileKinds] = {};
		int sumKindIk[kProfileKinds] = {};
		int kindCounts[kProfileKinds] = {};
		int kindLegs[kProfileKinds] = {};
		int sumSubsteps = 0;
		int sumIkSolves = 0;
		int sumIkSkipped = 0;
		int sumPyRays = 0;
		int stepsRun = 0;

		ResetNativeQueryStats();

		while ( m_stepAcc >= stepDt - 1e-6f && stepsRun < s_maxSubsteps )
		{
			m_stepAcc -= stepDt;
			m_benchSimTime += stepDt;

			uint64_t tAi = GetTicks();
			FillBenchInputs( stepDt );
			for ( size_t i = 0; i < m_rigs.size(); ++i )
			{
				memcpy( &m_inputs[i * kInputStride], m_rigs[i].input, sizeof( float ) * kInputStride );
			}
			frameAiMs += GetMilliseconds( tAi );

			uint64_t tPy = GetTicks();
			if ( mech::Step( stepDt, m_inputs.data(), (int)m_rigs.size() ) == false )
			{
				m_halted = true;
				return;
			}
			framePyCallMs += GetMilliseconds( tPy );

			int nProf = ProfileData( raw, kProfileFloats );
			if ( nProf >= kProfileHeader )
			{
				for ( int s = 0; s < 9; ++s )
				{
					sumStages[s] += raw[s];
				}
				sumSubsteps += (int)( raw[9] + 0.5f );
				sumIkSolves += (int)( raw[10] + 0.5f );
				sumIkSkipped += (int)( raw[11] + 0.5f );
				sumPyRays += (int)( raw[12] + 0.5f );
				sumPosesMs += raw[13];
			}
			if ( nProf >= kProfileFloats )
			{
				for ( int k = 0; k < kProfileKinds; ++k )
				{
					int b = kProfileHeader + k * kProfileKindStride;
					kindCounts[k] = (int)( raw[b] + 0.5f );
					kindLegs[k] = (int)( raw[b + 1] + 0.5f );
					sumKindMs[k] += raw[b + 2];
					sumKindRays[k] += (int)( raw[b + 3] + 0.5f );
					sumKindIk[k] += (int)( raw[b + 4] + 0.5f );
				}
			}
			stepsRun += 1;
		}

		if ( stepsRun > 0 )
		{
			uint64_t tBr = GetTicks();
			FetchPoses();
			int maxEvents = MaxInt( 16384, (int)m_rigs.size() * 64 );
			m_events.resize( (size_t)maxEvents * kEventFloats );
			int nev = Events( m_events.data(), (int)m_events.size() );
			if ( s_drawFx && nev > 0 )
			{
				m_lastEvents = m_fx.HandleEvents( m_events.data(), nev );
			}
			if ( s_debug )
			{
				m_debug.resize( 200000 );
				m_debugFloats = DebugLines( Player().handle, m_debug.data(), (int)m_debug.size() );
			}
			else
			{
				m_debugFloats = 0;
			}
			frameBridgeMs += GetMilliseconds( tBr );
			m_simTime += (double)( stepDt * (float)stepsRun );

			const float* p = PlayerPose();
			if ( ( m_stepCount & 3 ) == 0 || m_stats.mode.empty() )
			{
				Stats( Player().handle, &m_stats );
			}
			m_footfall.Record( p, Player().info, stepDt * (float)stepsRun );
			m_speedPlot.Add( m_time, { p[PoseSpeed] } );
			m_supportPlot.Add( m_time, { p[PoseSupport] } );
			m_stretchPlot.Add( m_time, { m_stats.maxStretch } );
		}

		if ( s_drawFx )
		{
			m_fx.Update( scaledDt );
		}
		else
		{
			m_fx.Clear();
		}

		if ( HasError() )
		{
			m_halted = true;
			return;
		}

		NativeQueryStats q = GetNativeQueryStats();
		m_frameDidUpdate = true;
		m_lastStepsRun = stepsRun;
		m_lastPyCallMs = framePyCallMs;
		m_lastAiMs = frameAiMs;
		m_lastBridgeMs = frameBridgeMs + sumPosesMs;
		m_lastSubsteps = sumSubsteps;
		m_lastIkSolves = sumIkSolves;
		m_lastIkSkipped = sumIkSkipped;
		m_lastPyRays = sumPyRays;
		m_lastNative = q;
		for ( int s = 0; s < 9; ++s )
		{
			m_lastRawStages[s] = sumStages[s];
		}
		for ( int k = 0; k < kProfileKinds; ++k )
		{
			m_lastKindCnt[k] = kindCounts[k];
			m_lastKindLegs[k] = kindLegs[k];
			m_lastKindMs[k] = sumKindMs[k];
			m_lastKindRays[k] = sumKindRays[k];
			m_lastKindIk[k] = sumKindIk[k];
		}
		m_totalLegs = 0;
		for ( const Rig& r : m_rigs )
		{
			m_totalLegs += r.info.legs;
		}
	}

	void Render() override
	{
		if ( CamMode() != CamFree )
		{
			UpdateCamera();
		}

		uint64_t tW = GetTicks();
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
		float drawWorldMs = GetMilliseconds( tW );

		uint64_t tM = GetTicks();
		if ( Alive() && s_drawMechs )
		{
			Vec3 aim = AimDir();
			bool aiming = CamMode() == CamAim;
			Vec3 camEye = m_camera->DrawOrigin();
			for ( size_t i = 0; i < m_rigs.size(); ++i )
			{
				bool driven = (int)i == m_player && CamMode() != CamFree;
				const float* pose = PoseOf( (int)i );
				int lod = 0;
				if ( s_drawLod && driven == false )
				{
					Vec3 d = Sub( PV( pose, PosePos ), camEye );
					float d2 = Dot( d, d );
					lod = d2 < 55.0f * 55.0f ? 0 : ( d2 < 135.0f * 135.0f ? 1 : 2 );
				}
				DrawMechLod( pose, m_rigs[i].info, driven, lod, driven && aiming ? &aim : nullptr );
			}
			if ( CamMode() != CamFree )
			{
				DrawDrivenMarker( m_world, PlayerPose(), Player().info );
			}
		}
		float drawMechsMs = GetMilliseconds( tM );

		uint64_t tFx = GetTicks();
		if ( s_drawFx )
		{
			m_fx.Draw();
		}
		if ( s_debug && m_debugFloats > 0 )
		{
			DrawDebugLines( m_debug.data(), m_debugFloats );
		}
		float drawFxMs = GetMilliseconds( tFx );

		m_lastDrawWorldMs = drawWorldMs;
		m_lastDrawMechsMs = drawMechsMs;
		m_lastDrawFxMs = drawFxMs;

		if ( m_pauseProfile == false && ( m_context->pause == false || m_frameDidUpdate ) )
		{
			AccumulateProfilerFrame();
		}
		m_frameDidUpdate = false;

		int effIkHz = s_lockIkToSim ? s_simHz : s_legIkHz;
		float frameMs = (float)( sapp_frame_duration() * 1000.0 );
		float fps = m_frameMsEma > 0.001f ? ( 1000.0f / m_frameMsEma ) : 0.0f;
		DrawTextLine( "%d mechs (%d legs)  Sim %dHz  IK %dHz  %.2fx", (int)m_rigs.size(), m_totalLegs, s_simHz, effIkHz,
					  s_simSpeed );
		if ( ActiveBackend() == BackendC )
		{
			DrawTextLine( "%.1fms (%.0f FPS) C(%dT) %.2fms Draw %.2fms [M]",
						  m_frameMsEma > 0.0f ? m_frameMsEma : frameMs, fps, WorkerThreads(), m_pyTotalFrameAvg,
						  m_drawTotalAvg );
		}
		else
		{
			DrawTextLine( "%.1fms (%.0f FPS) Py %.2fms Draw %.2fms [M]",
						  m_frameMsEma > 0.0f ? m_frameMsEma : frameMs, fps, m_pyTotalFrameAvg, m_drawTotalAvg );
		}
		HudMessages();
	}

	void DrawControls() override
	{
		PanelErrors();
		if ( Alive() == false )
		{
			PanelDebug();
			return;
		}

		if ( ui::Section( "Performance" ) )
		{
			if ( HasPythonBackend() )
			{
				const char* backends[2] = { "C (Native)", "Python" };
				int b = ActiveBackend() == BackendPython ? 1 : 0;
				if ( ui::Combo( "IK backend", &b, backends, 2 ) )
				{
					SetBackend( b == 1 ? BackendPython : BackendC );
					ResetStats();
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
				if ( hw > 1 && ui::SliderInt( "Worker threads", &th, 1, hw ) )
				{
					SetWorkerThreads( th );
					ResetStats();
				}
			}
			float fps = m_frameMsEma > 0.001f ? 1000.0f / m_frameMsEma : 0.0f;
			float perMech = m_rigs.empty() ? 0.0f : m_pyTotalFrameAvg / (float)m_rigs.size();
			ui::Stat( "frame", "%.2f ms (%.0f FPS)", m_frameMsEma, fps );
			if ( perMech >= 0.1f || m_rigs.empty() )
			{
				ui::Stat( "sim wall", "%.2f ms (%.2f/mech)", m_pyTotalFrameAvg, perMech );
			}
			else
			{
				ui::Stat( "sim wall", "%.2f ms (%.1f us/m)", m_pyTotalFrameAvg, perMech * 1000.0f );
			}
			if ( ActiveBackend() == BackendC )
			{
				float cpu = LastCpuMs();
				float spd = m_lastPyCallMs > 1e-4f ? cpu / m_lastPyCallMs : 1.0f;
				ui::Stat( "worker cpu", "%.2f ms (%.1fx)", cpu, spd );
			}
			ui::Stat( "c++ draw", "%.2f ms", m_drawTotalAvg );
			ui::Stat( "workload", "%d mechs / %d legs", (int)m_rigs.size(), m_totalLegs );
			if ( m_raysAvgFrame >= 10000.0f )
			{
				ui::Stat( "queries", "%.1fk rays, %.0f push", m_raysAvgFrame * 0.001f, m_pushAvgFrame );
			}
			else
			{
				ui::Stat( "queries", "%.0f rays, %.0f push", m_raysAvgFrame, m_pushAvgFrame );
			}
			if ( ui::Button( m_context->showMetrics ? "Hide Profiler (M)" : "Open Profiler (M)" ) )
			{
				m_context->showMetrics = !m_context->showMetrics;
			}
		}

		if ( ui::Section( "Workload" ) )
		{
			if ( ui::SliderInt( "Mech count", &s_mechCount, 1, kMaxBenchMechs ) )
			{
				ResizeCrowd( s_mechCount );
			}
			int mixCount = MinInt( kProfileKinds + 1, (int)m_kindNames.size() + 1 );
			const char* mixNames[kProfileKinds + 1] = { "All 9" };
			for ( int k = 0; k < (int)m_kindNames.size() && k + 1 <= kProfileKinds; ++k )
			{
				mixNames[k + 1] = m_kindNames[k].c_str();
			}
			if ( ui::Combo( "Mech mix", &s_mechMix, mixNames, mixCount ) )
			{
				RebuildCrowd();
			}
			if ( ui::Combo( "Terrain", &s_terrain, kTerrainNames, 3 ) )
			{
				m_context->requestRestart = true;
			}
			if ( s_terrain != 0 && ui::SeedInput( "Seed", &s_seed ) )
			{
				m_context->requestRestart = true;
			}
			ui::Combo( "Pattern", &s_pattern, kPatternNames, 4 );
			ui::Combo( "Motion mode", &s_motionMode, kMotionNames, 4 );
		}

		if ( ui::Section( "Frequencies & Speed" ) )
		{
			if ( ui::SliderInt( "Sim / Gait Hz", &s_simHz, 10, 120 ) )
			{
				m_configDirty = true;
			}
			if ( ui::Checkbox( "Lock Leg IK to Sim", &s_lockIkToSim ) )
			{
				m_configDirty = true;
			}
			if ( s_lockIkToSim == false && ui::SliderInt( "Leg IK Hz", &s_legIkHz, 5, 120 ) )
			{
				m_configDirty = true;
			}
			if ( ui::SliderInt( "Probe Hz", &s_probeHz, 2, 60 ) )
			{
				m_configDirty = true;
			}
			ui::SliderFloat( "Sim speed", &s_simSpeed, 0.0f, 3.0f, "%.2fx" );
			if ( ui::SliderFloat( "Move speed", &s_moveSpeed, 0.0f, 2.0f, "%.2fx" ) )
			{
				m_configDirty = true;
			}
			ui::SliderInt( "Max substeps", &s_maxSubsteps, 1, 8 );
		}

		if ( ui::Section( "Isolation", false ) )
		{
			if ( ui::Checkbox( "Obstacle roll", &s_planeRoll ) )
			{
				m_configDirty = true;
			}
			ui::Checkbox( "Draw mechs", &s_drawMechs );
			ui::Checkbox( "Distance LOD", &s_drawLod );
			ui::Checkbox( "Effects (FX)", &s_drawFx );
			ui::Checkbox( "Shadows", &m_context->enableShadows );
			ui::Checkbox( "GTAO", &m_context->enableGtao );
			if ( ui::Checkbox( "IK overlay (H)", &s_debug ) )
			{
				SetDebug( s_debug );
			}
			ui::Combo( "Camera (V)", &CamMode(), CameraModeNames(), CamModeCount );
			if ( ActiveBackend() == BackendPython && ui::Button( "Reload Python (F5)" ) )
			{
				ReloadPython();
			}
		}

		if ( ui::Section( "Presets", false ) )
		{
			if ( ui::Button( "Light (8 @ 60 Hz)" ) )
			{
				ApplyPreset( 8, 60, true, 20, 20, 1.0f, 1.0f, true );
			}
			if ( ui::Button( "Medium (24 @ 60 Hz)" ) )
			{
				ApplyPreset( 24, 60, true, 20, 20, 1.0f, 1.0f, true );
			}
			if ( ui::Button( "Heavy (48 @ 60 Hz)" ) )
			{
				ApplyPreset( 48, 60, true, 20, 20, 1.0f, 1.0f, true );
			}
			if ( ui::Button( "Extreme (128 @ 60 Hz)" ) )
			{
				ApplyPreset( 128, 60, true, 20, 20, 1.0f, 1.0f, true );
			}
			if ( ui::Button( "Mass (500 @ 60 Hz)" ) )
			{
				ApplyPreset( 500, 60, true, 20, 20, 1.0f, 1.0f, true );
			}
			if ( ui::Button( "1K Stress (1000 @ 60 Hz)" ) )
			{
				ApplyPreset( 1000, 60, true, 20, 20, 1.0f, 1.0f, true );
			}
			if ( ui::Button( "Staggered IK (32 @ 20 Hz)" ) )
			{
				ApplyPreset( 32, 60, false, 20, 10, 1.0f, 1.0f, true );
			}
			if ( ui::Button( "Reset defaults" ) )
			{
				s_terrain = 0;
				s_mechMix = 0;
				s_pattern = 1;
				s_motionMode = 2;
				s_drawMechs = true;
				s_drawFx = true;
				ApplyPreset( 16, 60, true, 20, 20, 1.0f, 1.0f, true );
				m_context->requestRestart = true;
			}
		}

		if ( ui::Section( "Controls", false ) )
		{
			ui::KeyHint( "M", "Toggle Profiler drawer" );
			ui::KeyHint( "Right drag + WASD", "Fly camera (Free mode)" );
			ui::KeyHint( "Alt + drag", "Orbit / pan / zoom" );
			ui::KeyHint( "V / F", "Cycle camera / Frame crowd" );
			ui::KeyHint( "H", "Toggle IK / gait overlay" );
			ui::KeyHint( "P / O", "Pause / single step" );
			ui::KeyHint( "F5", "Reload Python gaitcore" );
		}
	}

	void DrawMetricsTabs() override
	{
		DrawProfilerTab();
		DrawTimelineTab();
		DrawPerMechTab();
		MetricsTabs();
	}

	void Keyboard( int key, int mods ) override
	{
		(void)mods;
		CommonKey( key );
	}

	bool GetBounds( AABB* bounds ) override
	{
		if ( Alive() == false )
		{
			return false;
		}
		Vec3 mn = PV( PoseOf( 0 ), PosePos );
		Vec3 mx = mn;
		for ( int i = 1; i < (int)m_rigs.size(); ++i )
		{
			Vec3 p = PV( PoseOf( i ), PosePos );
			mn.x = MinFloat( mn.x, p.x );
			mn.y = MinFloat( mn.y, p.y );
			mn.z = MinFloat( mn.z, p.z );
			mx.x = MaxFloat( mx.x, p.x );
			mx.y = MaxFloat( mx.y, p.y );
			mx.z = MaxFloat( mx.z, p.z );
		}
		Vec3 pad = { 4.0f, 4.0f, 4.0f };
		*bounds = { Sub( mn, pad ), Add( mx, pad ) };
		return true;
	}

private:

	void BuildBenchmarkWorld()
	{
		if ( s_terrain == 2 )
		{
			BuildFrontline( m_world, (uint64_t)s_seed );
			m_world.extent = 220.0f;
			return;
		}
		m_world.Clear();
		m_world.groundPlaneY = 0.0f;
		m_world.extent = 260.0f;
		if ( s_terrain == 0 )
		{
			m_world.name = "bench_flat";
			for ( int i = 0; i < 8; ++i )
			{
				float a = (float)i * ( 2.0f * MATH_PI / 8.0f );
				float px = 65.0f * sinf( a ), pz = 65.0f * cosf( a );
				m_world.AddAabb( px - 0.45f, 0.0f, pz - 0.45f, px + 0.45f, 1.5f, pz + 0.45f, LookMarker );
			}
			for ( int i = 0; i < 16; ++i )
			{
				float a = (float)i * ( 2.0f * MATH_PI / 16.0f );
				float px = 185.0f * sinf( a ), pz = 185.0f * cosf( a );
				m_world.AddAabb( px - 0.6f, 0.0f, pz - 0.6f, px + 0.6f, 2.2f, pz + 0.6f, LookMarker );
			}
			return;
		}

		m_world.name = "bench_rubble";
		Rng rng( (uint64_t)s_seed + 17u );
		for ( int i = 0; i < 36; ++i )
		{
			float a = (float)i * ( 2.0f * MATH_PI / 36.0f ) + rng.Range( -0.05f, 0.05f );
			float r = 20.0f + (float)( i % 6 ) * 27.0f;
			float x = r * sinf( a ), z = r * cosf( a );
			float tx = x + cosf( a ) * 7.0f, tz = z - sinf( a ) * 7.0f;
			float w = rng.Range( 4.2f, 6.0f );
			float hw = w * 0.5f;
			float h = rng.Range( 0.55f, 1.40f );
			m_world.AddRamp( x, 0.0f, z, tx, h, tz, w, LookStructure );
			m_world.AddAabb( tx - hw, 0.0f, tz - hw, tx + hw, h, tz + hw, LookStructure );
		}
		for ( int i = 0; i < 3200; ++i )
		{
			float ang = rng.Range( 0.0f, 2.0f * MATH_PI );
			float rad = sqrtf( rng.Float() ) * 182.0f;
			float x = rad * sinf( ang ), z = rad * cosf( ang );
			float sx = rng.Range( 0.30f, 1.25f );
			float sy = rng.Range( 0.12f, 0.58f );
			float sz = rng.Range( 0.30f, 1.25f );
			m_world.AddRot( { x, sy * 0.45f, z }, { sx, sy * 0.5f, sz }, rng.Range( 0.0f, MATH_PI ),
							rng.Range( -0.25f, 0.25f ), LookRock );
		}
		for ( int c = 0; c < 180; ++c )
		{
			float cAng = rng.Range( 0.0f, 2.0f * MATH_PI );
			float cRad = sqrtf( rng.Float() ) * 178.0f;
			float cx = cRad * sinf( cAng ), cz = cRad * cosf( cAng );
			for ( int k = 0; k < 10; ++k )
			{
				float rAng = rng.Range( 0.0f, 2.0f * MATH_PI );
				float rDist = rng.Range( 0.8f, 6.0f );
				float x = cx + rDist * sinf( rAng );
				float z = cz + rDist * cosf( rAng );
				bool big = ( k & 1 ) == 0;
				float sx = big ? rng.Range( 0.35f, 1.35f ) : rng.Range( 0.22f, 0.75f );
				float sy = big ? rng.Range( 0.15f, 0.68f ) : rng.Range( 0.10f, 0.42f );
				float sz = big ? rng.Range( 0.35f, 1.35f ) : rng.Range( 0.22f, 0.75f );
				m_world.AddRot( { x, sy * 0.45f, z }, { sx, sy * 0.5f, sz }, rng.Range( 0.0f, MATH_PI ),
								rng.Range( -0.28f, 0.28f ), LookRock );
			}
		}
	}

	void SpawnSlot( int idx, Vec3* pos, float* yaw ) const
	{
		float x = 0.0f, z = 0.0f;
		if ( idx > 0 )
		{
			int rem = idx - 1;
			int ring = 1;
			int cap = 6;
			while ( rem >= cap )
			{
				rem -= cap;
				++ring;
				cap = 6 * ring;
			}
			float r = 9.5f * (float)ring;
			float a = ( (float)rem + 0.35f * (float)( ring & 1 ) ) * ( 2.0f * MATH_PI / (float)cap );
			x = r * sinf( a );
			z = r * cosf( a );
		}
		if ( s_terrain == 2 )
		{
			z -= 10.0f;
		}
		float lim = m_world.extent * 0.85f;
		x = ClampFloat( x, -lim, lim );
		z = ClampFloat( z, -lim, lim );
		float y = m_world.GroundY( x, z );
		*pos = { x, y, z };
		*yaw = atan2f( -z, x );
	}

	int KindForSlot( int idx ) const
	{
		if ( m_kindNames.empty() )
		{
			return 0;
		}
		if ( s_mechMix <= 0 )
		{
			return idx % (int)m_kindNames.size();
		}
		return ClampInt( s_mechMix - 1, 0, (int)m_kindNames.size() - 1 );
	}

	void ResizeCrowd( int target )
	{
		target = ClampInt( target, 1, kMaxBenchMechs );
		int cur = (int)m_rigs.size();
		if ( target > cur )
		{
			m_rigs.reserve( (size_t)target );
			m_homes.reserve( (size_t)target );
			for ( int i = cur; i < target; ++i )
			{
				Vec3 p;
				float yaw;
				SpawnSlot( i, &p, &yaw );
				int idx = AddRig( KindForSlot( i ), p, yaw, true, false );
				if ( idx >= 0 )
				{
					SetBenchHome( idx, p, i );
				}
			}
			Relayout();
			FetchPoses();
		}
		else if ( target < cur )
		{
			DespawnTo( target );
			m_rigs.resize( (size_t)target );
			m_homes.resize( (size_t)target );
			m_player = 0;
			Relayout();
			FetchPoses();
		}
		m_crowdRadius = 0.0f;
		for ( const BenchHome& h : m_homes )
		{
			float r = sqrtf( h.origin.x * h.origin.x + h.origin.z * h.origin.z );
			m_crowdRadius = MaxFloat( m_crowdRadius, r );
		}
	}

	void RebuildCrowd()
	{
		DespawnTo( 0 );
		m_rigs.clear();
		m_homes.clear();
		ResizeCrowd( s_mechCount );
	}

	void SetBenchHome( int idx, Vec3 p, int slot )
	{
		if ( (int)m_homes.size() <= idx )
		{
			m_homes.resize( (size_t)idx + 1 );
		}
		m_homes[idx].origin = p;
		float rad = sqrtf( p.x * p.x + p.z * p.z );
		m_homes[idx].ringRadius = MaxFloat( 12.0f, MinFloat( rad, m_world.extent * 0.82f ) );
		m_homes[idx].laneDir = ( slot & 1 ) ? 1.0f : -1.0f;
	}

	void SyncBenchConfig()
	{
		float maxSub = 1.0f / (float)ClampInt( s_simHz, 10, 120 );
		float ikHz = s_lockIkToSim ? 0.0f : (float)s_legIkHz;
		SetBenchConfig( maxSub, ikHz, (float)s_probeHz, s_moveSpeed, s_planeRoll, true );
		m_configDirty = false;
	}

	void ApplyPreset( int count, int simHz, bool lockIk, int legIkHz, int probeHz, float simSpd, float mvSpd, bool roll )
	{
		s_mechCount = count;
		s_simHz = simHz;
		s_lockIkToSim = lockIk;
		s_legIkHz = legIkHz;
		s_probeHz = probeHz;
		s_simSpeed = simSpd;
		s_moveSpeed = mvSpd;
		s_planeRoll = roll;
		ResizeCrowd( s_mechCount );
		SyncBenchConfig();
		ResetStats();
	}

	void FillBenchInputs( float stepDt )
	{
		int n = (int)m_rigs.size();
		for ( int i = 0; i < n; ++i )
		{
			Rig& r = m_rigs[i];
			memset( r.input, 0, sizeof( r.input ) );
			if ( s_pattern == 3 )
			{
				continue;
			}
			const float* p = PoseOf( i );
			Vec3 pos = PV( p, PosePos );
			if ( s_pattern == 0 )
			{
				float d = 0.0f;
				SteerTo( p, r.waypoints[r.waypoint], r.input, &d );
				if ( d < 3.0f )
				{
					r.waypoint = ( r.waypoint + 1 ) & 3;
				}
				r.stuckTime += stepDt;
				if ( r.stuckTime > 5.0f )
				{
					if ( Distance( pos, r.stuckPos ) < 1.0f )
					{
						r.waypoint = ( r.waypoint + 1 ) & 3;
					}
					r.stuckTime = 0.0f;
					r.stuckPos = pos;
				}
			}
			else if ( s_pattern == 1 )
			{
				float R = ( i < (int)m_homes.size() ) ? m_homes[i].ringRadius : 16.0f;
				float ang = atan2f( pos.x, pos.z );
				float dir = ( i & 1 ) ? 1.0f : -1.0f;
				float lead = 16.0f / MaxFloat( R, 8.0f );
				float tgtAng = ang + dir * lead;
				Vec3 target = { R * sinf( tgtAng ), pos.y, R * cosf( tgtAng ) };
				SteerTo( p, target, r.input );
			}
			else if ( s_pattern == 2 )
			{
				float laneX = ( i < (int)m_homes.size() ) ? m_homes[i].origin.x : (float)( i - n / 2 ) * 5.0f;
				float& laneDir = ( i < (int)m_homes.size() ) ? m_homes[i].laneDir : r.waypoints[0].y;
				float sprintSpan = ClampFloat( MaxFloat( 56.0f, m_crowdRadius + 16.0f ), 56.0f, m_world.extent * 0.82f );
				float turnAt = sprintSpan - 6.0f;
				float endZ = laneDir > 0.0f ? sprintSpan : -sprintSpan;
				if ( ( laneDir > 0.0f && pos.z > turnAt ) || ( laneDir < 0.0f && pos.z < -turnAt ) )
				{
					laneDir = -laneDir;
					endZ = laneDir > 0.0f ? sprintSpan : -sprintSpan;
				}
				SteerTo( p, { laneX, pos.y, endZ }, r.input );
			}

			if ( s_motionMode == 0 )
			{
				r.input[InSlow] = 1.0f;
			}
			else if ( s_motionMode == 1 )
			{
				r.input[InFwd] *= 0.65f;
			}
			else if ( s_motionMode == 2 )
			{
				r.input[InRun] = 1.0f;
			}
			else if ( s_motionMode == 3 )
			{
				r.input[InRun] = 1.0f;
				double phase = fmod( m_benchSimTime + 0.40 * (double)( i % 8 ), 3.2 );
				if ( phase < 0.65 )
				{
					r.input[InDrift] = 1.0f;
					r.input[InTurn] = ClampFloat( r.input[InTurn] + ( ( i & 1 ) ? 0.55f : -0.55f ), -1.0f, 1.0f );
				}
			}
		}
	}

	void ResetStats()
	{
		m_frameMsEma = 0.0f;
		m_pyTotalFrameAvg = 0.0f;
		m_pyTotalStepAvg = 0.0f;
		m_drawTotalAvg = 0.0f;
		m_raysAvgFrame = 0.0f;
		m_pushAvgFrame = 0.0f;
		m_heightAvgFrame = 0.0f;
		m_ikSolvesAvg = 0.0f;
		m_ikSkippedAvg = 0.0f;
		memset( m_stageAvgFrame, 0, sizeof( m_stageAvgFrame ) );
		memset( m_stageAvgStep, 0, sizeof( m_stageAvgStep ) );
		memset( m_stagePeak, 0, sizeof( m_stagePeak ) );
		memset( m_nativeAvgFrame, 0, sizeof( m_nativeAvgFrame ) );
		memset( m_nativePeak, 0, sizeof( m_nativePeak ) );
		memset( m_kindAvgMs, 0, sizeof( m_kindAvgMs ) );
		memset( m_kindAvgRays, 0, sizeof( m_kindAvgRays ) );
		memset( m_kindAvgIk, 0, sizeof( m_kindAvgIk ) );
		m_profTimePlot.Clear();
		m_profCountPlot.Clear();
	}

	void CollectStepStats( float aiMs, float bridgeMs, int stepsRun )
	{
		float raw[kProfileFloats] = {};
		int nProf = ProfileData( raw, kProfileFloats );
		m_frameDidUpdate = true;
		m_lastStepsRun = stepsRun;
		m_lastPyCallMs = nProf >= 1 ? raw[0] : 0.0f;
		m_lastAiMs = aiMs;
		m_lastBridgeMs = bridgeMs + ( nProf >= 14 ? raw[13] : 0.0f );
		for ( int s = 0; s < 9 && s < nProf; ++s )
		{
			m_lastRawStages[s] = raw[s];
		}
		m_totalLegs = 0;
		for ( const Rig& r : m_rigs )
		{
			m_totalLegs += r.info.legs;
		}
	}

	void AccumulateProfilerFrame()
	{
		const float alpha = 0.10f;
		float frameMs = (float)( sapp_frame_duration() * 1000.0 );
		m_frameMsEma = m_frameMsEma <= 0.0f ? frameMs : m_frameMsEma + ( frameMs - m_frameMsEma ) * alpha;

		float samples[kStageCount] = {};
		float pyStageSum = 0.0f;
		if ( m_frameDidUpdate )
		{
			for ( int i = 0; i < 8; ++i )
			{
				samples[i] = m_lastRawStages[1 + i];
				pyStageSum += samples[i];
			}
			samples[8] = MaxFloat( 0.0f, m_lastRawStages[0] - pyStageSum );
			samples[9] = m_lastAiMs;
			samples[10] = m_lastBridgeMs;
		}
		samples[11] = m_lastDrawWorldMs + m_lastDrawMechsMs + m_lastDrawFxMs;
		samples[12] = GetRenderStats().frameTimeMs;

		float pyFrameTotal = m_frameDidUpdate ? m_lastRawStages[0] : 0.0f;
		m_pyTotalFrameAvg += ( pyFrameTotal - m_pyTotalFrameAvg ) * alpha;
		float drawTotal = samples[11];
		m_drawTotalAvg += ( drawTotal - m_drawTotalAvg ) * alpha;

		float rays = m_frameDidUpdate ? (float)m_lastNative.rayCalls : 0.0f;
		float pushes = m_frameDidUpdate ? (float)m_lastNative.pushCalls : 0.0f;
		float heights = m_frameDidUpdate ? (float)m_lastNative.heightCalls : 0.0f;
		float rayMs = m_frameDidUpdate ? m_lastNative.rayMs : 0.0f;
		float pushMs = m_frameDidUpdate ? m_lastNative.pushMs : 0.0f;
		float heightMs = m_frameDidUpdate ? m_lastNative.heightMs : 0.0f;
		float ikSol = m_frameDidUpdate ? (float)m_lastIkSolves : 0.0f;
		float ikSkip = m_frameDidUpdate ? (float)m_lastIkSkipped : 0.0f;

		m_raysAvgFrame += ( rays - m_raysAvgFrame ) * alpha;
		m_pushAvgFrame += ( pushes - m_pushAvgFrame ) * alpha;
		m_heightAvgFrame += ( heights - m_heightAvgFrame ) * alpha;
		m_ikSolvesAvg += ( ikSol - m_ikSolvesAvg ) * alpha;
		m_ikSkippedAvg += ( ikSkip - m_ikSkippedAvg ) * alpha;

		float nativeSmp[3] = { rayMs, pushMs, heightMs };
		for ( int i = 0; i < 3; ++i )
		{
			m_nativeAvgFrame[i] += ( nativeSmp[i] - m_nativeAvgFrame[i] ) * alpha;
			if ( nativeSmp[i] > m_nativePeak[i] )
			{
				m_nativePeak[i] = nativeSmp[i];
			}
		}

		for ( int i = 0; i < kStageCount; ++i )
		{
			m_stageAvgFrame[i] += ( samples[i] - m_stageAvgFrame[i] ) * alpha;
			if ( samples[i] > m_stagePeak[i] )
			{
				m_stagePeak[i] = samples[i];
			}
		}

		if ( m_frameDidUpdate && m_lastStepsRun > 0 )
		{
			float invS = 1.0f / (float)m_lastStepsRun;
			for ( int i = 0; i < 11; ++i )
			{
				float perStep = samples[i] * invS;
				m_stageAvgStep[i] += ( perStep - m_stageAvgStep[i] ) * alpha;
			}
			m_pyTotalStepAvg += ( pyFrameTotal * invS - m_pyTotalStepAvg ) * alpha;
		}
		m_stageAvgStep[11] = m_stageAvgFrame[11];
		m_stageAvgStep[12] = m_stageAvgFrame[12];

		for ( int k = 0; k < kProfileKinds; ++k )
		{
			float kms = ( m_frameDidUpdate && m_lastKindCnt[k] > 0 ) ? m_lastKindMs[k] : 0.0f;
			float kray = ( m_frameDidUpdate && m_lastKindCnt[k] > 0 ) ? (float)m_lastKindRays[k] : 0.0f;
			float kik = ( m_frameDidUpdate && m_lastKindCnt[k] > 0 ) ? (float)m_lastKindIk[k] : 0.0f;
			m_kindAvgMs[k] += ( kms - m_kindAvgMs[k] ) * alpha;
			m_kindAvgRays[k] += ( kray - m_kindAvgRays[k] ) * alpha;
			m_kindAvgIk[k] += ( kik - m_kindAvgIk[k] ) * alpha;
		}

		m_profTimePlot.Add( m_time,
							{ pyFrameTotal, samples[6] + samples[7], samples[4], rayMs + pushMs + heightMs, samples[11], samples[10] } );
		m_profCountPlot.Add( m_time, { rays, pushes, ikSol, ikSkip } );
	}

	void DrawProfilerTab()
	{
		if ( ImGui::BeginTabItem( "Profiler" ) == false )
		{
			return;
		}

		ImGui::Checkbox( "Pause capture", &m_pauseProfile );
		ImGui::SameLine();
		if ( ImGui::SmallButton( "Reset peaks" ) )
		{
			memset( m_stagePeak, 0, sizeof( m_stagePeak ) );
			memset( m_nativePeak, 0, sizeof( m_nativePeak ) );
		}
		ImGui::SameLine();
		ImGui::Checkbox( "Per-step", &m_perStepView );

		const float* vals = m_perStepView ? m_stageAvgStep : m_stageAvgFrame;
		float pySum = 0.0f;
		for ( int i = 0; i <= 8; ++i )
		{
			pySum += vals[i];
		}
		float totalCpu = pySum + vals[9] + vals[10] + vals[11] + vals[12];

		int hotIdx = 0;
		for ( int i = 1; i < kStageCount; ++i )
		{
			if ( vals[i] > vals[hotIdx] )
			{
				hotIdx = i;
			}
		}
		float hotPct = totalCpu > 1e-4f ? ( 100.0f * vals[hotIdx] / totalCpu ) : 0.0f;
		uint32_t budgetCol = totalCpu <= 12.0f ? Palette::Good : ( totalCpu <= 16.67f ? Palette::Warning : Palette::Error );

		ImGui::SameLine();
		ui::Status( budgetCol, "Hotspot: %s (%.1f%%, %.2f ms) | CPU %.2f / 16.67 ms", kStageNames[hotIdx], hotPct, vals[hotIdx],
					totalCpu );

		const float segVals[6] = {
			vals[0],					  
			vals[1] + vals[8],			  
			vals[2] + vals[3] + vals[4] + vals[5], 
			vals[6] + vals[7],			  
			vals[9] + vals[10],			  
			vals[11] + vals[12]			  
		};
		const char* segNames[6] = { "Probes/Gait", "Body/Py", "Stepping/FH", "IK+Roll", "Bridge/AI", "Draw+Render" };
		float barScaleMax = MaxFloat( 16.67f, totalCpu * 1.05f );
		float availW = MaxFloat( 120.0f, ImGui::GetContentRegionAvail().x );
		float barH = ImGui::GetFontSize() * 1.15f;
		ImVec2 p0 = ImGui::GetCursorScreenPos();
		ImDrawList* dl = ImGui::GetWindowDrawList();
		dl->AddRectFilled( p0, ImVec2( p0.x + availW, p0.y + barH ), HexToImU32( Palette::LineDim, 55 ) );
		float x = p0.x;
		for ( int s = 0; s < 6; ++s )
		{
			float sw = availW * ( segVals[s] / barScaleMax );
			if ( sw >= 0.5f )
			{
				dl->AddRectFilled( ImVec2( x, p0.y + 1.0f ), ImVec2( x + sw, p0.y + barH - 1.0f ), HexToImU32( SeriesHex( s ) ) );
				if ( ImGui::IsMouseHoveringRect( ImVec2( x, p0.y ), ImVec2( x + sw, p0.y + barH ) ) )
				{
					float spct = totalCpu > 1e-4f ? 100.0f * segVals[s] / totalCpu : 0.0f;
					ImGui::SetTooltip( "%s: %.2f ms (%.1f%%)", segNames[s], segVals[s], spct );
				}
				x += sw;
			}
		}
		float xBudget = p0.x + availW * ( 16.67f / barScaleMax );
		dl->AddLine( ImVec2( xBudget, p0.y ), ImVec2( xBudget, p0.y + barH ), HexToImU32( Palette::Warning ), 2.0f );
		ImGui::Dummy( ImVec2( availW, barH + 2.0f ) );

		float mechs = MaxFloat( 1.0f, (float)m_rigs.size() );
		float tblH = MaxFloat( ImGui::GetFontSize() * 5.0f, ImGui::GetContentRegionAvail().y );
		ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_ScrollY |
								ImGuiTableFlags_SizingStretchProp;
		if ( ImGui::BeginTable( "##prof_stages", 6, flags, ImVec2( 0.0f, tblH ) ) )
		{
			ImGui::TableSetupScrollFreeze( 0, 1 );
			ImGui::TableSetupColumn( "Stage", ImGuiTableColumnFlags_WidthStretch, 1.9f );
			ImGui::TableSetupColumn( "Avg (ms)", ImGuiTableColumnFlags_WidthStretch, 0.7f );
			ImGui::TableSetupColumn( "Peak (ms)", ImGuiTableColumnFlags_WidthStretch, 0.7f );
			ImGui::TableSetupColumn( "Share", ImGuiTableColumnFlags_WidthStretch, 0.6f );
			ImGui::TableSetupColumn( "us/mech", ImGuiTableColumnFlags_WidthStretch, 0.7f );
			ImGui::TableSetupColumn( "Details", ImGuiTableColumnFlags_WidthStretch, 1.8f );
			ImGui::TableHeadersRow();

			auto drawRow = [&]( const char* name, float avgMs, float peakMs, float sharePct, const char* det, int colorSlot ) {
				ImGui::TableNextRow();
				ImGui::TableSetColumnIndex( 0 );
				if ( colorSlot >= 0 )
				{
					ImVec2 sp = ImGui::GetCursorScreenPos();
					float sz = ImGui::GetFontSize() * 0.75f;
					dl->AddRectFilled( ImVec2( sp.x, sp.y + sz * 0.25f ), ImVec2( sp.x + sz, sp.y + sz * 1.25f ),
									   HexToImU32( SeriesHex( colorSlot ) ) );
					ImGui::Dummy( ImVec2( sz + 4.0f, sz ) );
					ImGui::SameLine();
				}
				ImGui::TextUnformatted( name );

				ImGui::TableSetColumnIndex( 1 );
				ImGui::Text( "%.3f", avgMs );

				ImGui::TableSetColumnIndex( 2 );
				ImGui::Text( "%.2f", peakMs );

				ImGui::TableSetColumnIndex( 3 );
				if ( sharePct >= 0.0f )
				{
					ImGui::Text( "%4.1f%%", sharePct );
				}
				else
				{
					ImGui::TextUnformatted( "(incl.)" );
				}

				ImGui::TableSetColumnIndex( 4 );
				ImGui::Text( "%.1f", avgMs * 1000.0f / mechs );

				ImGui::TableSetColumnIndex( 5 );
				ImGui::TextUnformatted( det );
			};

			const int slotMap[kStageCount] = { 0, 1, 2, 2, 2, 2, 3, 3, 1, 4, 4, 5, 5 };
			for ( int i = 0; i < kStageCount; ++i )
			{
				char det[96] = "-";
				if ( i == 6 )
				{
					snprintf( det, sizeof( det ), "%.0f solves / %.0f cached", m_ikSolvesAvg, m_ikSkippedAvg );
				}
				else if ( i == 7 )
				{
					snprintf( det, sizeof( det ), "%s", s_planeRoll ? "radial clearance active" : "disabled" );
				}
				else if ( i == 11 )
				{
					snprintf( det, sizeof( det ), "w %.2f  m %.2f  fx %.2f ms", m_lastDrawWorldMs, m_lastDrawMechsMs,
							  m_lastDrawFxMs );
				}
				else if ( i == 12 )
				{
					RenderStats rs = GetRenderStats();
					snprintf( det, sizeof( det ), "dc %d  cap %d  box %d  sph %d", rs.drawCallCount, rs.capsuleCount,
							  rs.cubeCount, rs.sphereCount );
				}
				float pct = totalCpu > 1e-4f ? ( 100.0f * vals[i] / totalCpu ) : 0.0f;
				drawRow( kStageNames[i], vals[i], m_stagePeak[i], pct, det, slotMap[i] );
			}

			const char* nNames[3] = { "World::Raycast (incl.)", "World::SpherePush (incl.)", "World::TerrainHeight (incl.)" };
			float nCalls[3] = { m_raysAvgFrame, m_pushAvgFrame, m_heightAvgFrame };
			for ( int i = 0; i < 3; ++i )
			{
				char det[96];
				float usPerCall = nCalls[i] > 0.5f ? ( m_nativeAvgFrame[i] * 1000.0f / nCalls[i] ) : 0.0f;
				snprintf( det, sizeof( det ), "%.0f calls/frame  (%.2f us/call)", nCalls[i], usPerCall );
				drawRow( nNames[i], m_nativeAvgFrame[i], m_nativePeak[i], -1.0f, det, -1 );
			}

			ImGui::EndTable();
		}

		ImGui::EndTabItem();
	}

	void DrawTimelineTab()
	{
		if ( ImGui::BeginTabItem( "Timeline" ) == false )
		{
			return;
		}
		float h = MaxFloat( 4.0f, ImGui::GetContentRegionAvail().y / ImGui::GetFontSize() * 0.48f - 0.5f );
		m_profTimePlot.Draw( { "IK Core", "IK Solver", "Stepping/FH", "World Queries", "C++ Draw", "Bridge" }, h );
		m_profCountPlot.Draw( { "Raycasts", "SpherePush", "IK Solves", "IK Skipped" }, h );
		ImGui::EndTabItem();
	}

	void DrawPerMechTab()
	{
		if ( ImGui::BeginTabItem( "Per-Mech Cost" ) == false )
		{
			return;
		}
		float sumKind = 0.0f;
		for ( int k = 0; k < kProfileKinds; ++k )
		{
			sumKind += m_kindAvgMs[k];
		}

		float tblH = MaxFloat( ImGui::GetFontSize() * 5.0f, ImGui::GetContentRegionAvail().y );
		ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_ScrollY |
								ImGuiTableFlags_SizingStretchProp;
		if ( ImGui::BeginTable( "##prof_kinds", 8, flags, ImVec2( 0.0f, tblH ) ) )
		{
			ImGui::TableSetupScrollFreeze( 0, 1 );
			ImGui::TableSetupColumn( "Mech Type", ImGuiTableColumnFlags_WidthStretch, 1.4f );
			ImGui::TableSetupColumn( "Count", ImGuiTableColumnFlags_WidthStretch, 0.6f );
			ImGui::TableSetupColumn( "Legs", ImGuiTableColumnFlags_WidthStretch, 0.6f );
			ImGui::TableSetupColumn( "Total (ms)", ImGuiTableColumnFlags_WidthStretch, 0.8f );
			ImGui::TableSetupColumn( "ms/mech", ImGuiTableColumnFlags_WidthStretch, 0.8f );
			ImGui::TableSetupColumn( "us/leg", ImGuiTableColumnFlags_WidthStretch, 0.8f );
			ImGui::TableSetupColumn( "Rays/mech", ImGuiTableColumnFlags_WidthStretch, 0.8f );
			ImGui::TableSetupColumn( "Share", ImGuiTableColumnFlags_WidthStretch, 1.3f );
			ImGui::TableHeadersRow();

			ImDrawList* dl = ImGui::GetWindowDrawList();
			for ( int k = 0; k < kProfileKinds && k < (int)m_kindNames.size(); ++k )
			{
				int cnt = m_lastKindCnt[k];
				int legs = m_lastKindLegs[k];
				float ms = m_kindAvgMs[k];
				float perM = cnt > 0 ? ms / (float)cnt : 0.0f;
				float usLeg = legs > 0 ? ( ms * 1000.0f / (float)legs ) : 0.0f;
				float rayM = cnt > 0 ? m_kindAvgRays[k] / (float)cnt : 0.0f;
				float share = sumKind > 1e-4f ? ms / sumKind : 0.0f;

				ImGui::TableNextRow();
				ImGui::TableSetColumnIndex( 0 );
				ImGui::TextUnformatted( m_kindNames[k].c_str() );
				ImGui::TableSetColumnIndex( 1 );
				ImGui::Text( "%d", cnt );
				ImGui::TableSetColumnIndex( 2 );
				ImGui::Text( "%d", legs );
				ImGui::TableSetColumnIndex( 3 );
				ImGui::Text( "%.3f", ms );
				ImGui::TableSetColumnIndex( 4 );
				ImGui::Text( "%.3f", perM );
				ImGui::TableSetColumnIndex( 5 );
				ImGui::Text( "%.1f", usLeg );
				ImGui::TableSetColumnIndex( 6 );
				ImGui::Text( "%.1f", rayM );
				ImGui::TableSetColumnIndex( 7 );
				ImVec2 p = ImGui::GetCursorScreenPos();
				float cw = MaxFloat( 24.0f, ImGui::GetContentRegionAvail().x );
				float ch = ImGui::GetFontSize();
				dl->AddRectFilled( ImVec2( p.x, p.y + 2.0f ), ImVec2( p.x + cw, p.y + ch ), HexToImU32( Palette::LineDim, 50 ) );
				dl->AddRectFilled( ImVec2( p.x, p.y + 2.0f ), ImVec2( p.x + cw * ClampFloat( share, 0.0f, 1.0f ), p.y + ch ),
								   HexToImU32( SeriesHex( k % 6 ), 140 ) );
				ImGui::Text( " %4.1f%%", share * 100.0f );
			}
			ImGui::EndTable();
		}
		ImGui::EndTabItem();
	}

	struct BenchHome
	{
		Vec3 origin = { 0.0f, 0.0f, 0.0f };
		float ringRadius = 12.0f;
		float laneDir = 1.0f;
	};

	std::vector<BenchHome> m_homes;
	float m_crowdRadius = 0.0f;
	float m_stepAcc = 0.0f;
	double m_benchSimTime = 0.0;
	bool m_configDirty = true;
	bool m_pauseProfile = false;
	bool m_perStepView = false;
	int m_totalLegs = 0;

	bool m_frameDidUpdate = false;
	int m_lastStepsRun = 0;
	float m_lastPyCallMs = 0.0f;
	float m_lastAiMs = 0.0f;
	float m_lastBridgeMs = 0.0f;
	int m_lastSubsteps = 0;
	int m_lastIkSolves = 0;
	int m_lastIkSkipped = 0;
	int m_lastPyRays = 0;
	float m_lastRawStages[9] = {};
	NativeQueryStats m_lastNative;
	float m_lastDrawWorldMs = 0.0f;
	float m_lastDrawMechsMs = 0.0f;
	float m_lastDrawFxMs = 0.0f;

	float m_frameMsEma = 0.0f;
	float m_pyTotalFrameAvg = 0.0f;
	float m_pyTotalStepAvg = 0.0f;
	float m_drawTotalAvg = 0.0f;
	float m_raysAvgFrame = 0.0f;
	float m_pushAvgFrame = 0.0f;
	float m_heightAvgFrame = 0.0f;
	float m_ikSolvesAvg = 0.0f;
	float m_ikSkippedAvg = 0.0f;

	float m_stageAvgFrame[kStageCount] = {};
	float m_stageAvgStep[kStageCount] = {};
	float m_stagePeak[kStageCount] = {};
	float m_nativeAvgFrame[3] = {};
	float m_nativePeak[3] = {};

	int m_lastKindCnt[kProfileKinds] = {};
	int m_lastKindLegs[kProfileKinds] = {};
	float m_lastKindMs[kProfileKinds] = {};
	int m_lastKindRays[kProfileKinds] = {};
	int m_lastKindIk[kProfileKinds] = {};
	float m_kindAvgMs[kProfileKinds] = {};
	float m_kindAvgRays[kProfileKinds] = {};
	float m_kindAvgIk[kProfileKinds] = {};

	ui::ScrollingPlot m_profTimePlot{ "ms", 12.0f };
	ui::ScrollingPlot m_profCountPlot{ "count", 12.0f };

	static inline int s_mechCount = 16;
	static inline int s_mechMix = 0;
	static inline int s_terrain = 0;
	static inline int s_seed = 7;
	static inline int s_pattern = 1;
	static inline int s_motionMode = 2;
	static inline int s_simHz = 60;
	static inline bool s_lockIkToSim = true;
	static inline int s_legIkHz = 20;
	static inline int s_probeHz = 20;
	static inline float s_simSpeed = 1.0f;
	static inline float s_moveSpeed = 1.0f;
	static inline int s_maxSubsteps = 4;
	static inline bool s_planeRoll = true;
	static inline bool s_drawMechs = true;
	static inline bool s_drawLod = true;
	static inline bool s_drawFx = true;
};

static int sampleBenchmark = RegisterSample(
	"Mechs", "IK Benchmark", MechBenchmark::Create,
	"Multi-mech IK/gait stress test with live frequency/speed sliders and M-key profiler breakdown." );
