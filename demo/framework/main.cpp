#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "gfx/draw.h"
#include "gfx/keycodes.h"
#include "gfx/renderer.h"
#include "host/gui.h"
#include "sample.h"
#include "scene.h"
#include "screenshot.h"
#include "sokol_app.h"
#include "sokol_glue.h"
#include "ui.h"

#include "gfx/timer.h"
#include "math3d.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined( _WIN32 )
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#include <windows.h>
#include <timeapi.h>

#pragma comment( lib, "winmm.lib" )
#endif

#ifndef APP_TITLE
#define APP_TITLE "Mech IK Core Demo"
#endif

#define GROUND_GRID_CELL_SIZE 1.0f

static SampleContext s_context;
static int s_frame = 0;
static int s_frameLimit = -1;
static const char* s_sampleArg = nullptr;
static const char* s_screenshotPath = nullptr;
static bool s_hideUI = false;
static bool s_startPaused = false;
static int s_width = 1920;
static int s_height = 1080;

static int CompareSamples( const void* a, const void* b )
{
	const SampleEntry* ea = (const SampleEntry*)a;
	const SampleEntry* eb = (const SampleEntry*)b;
	int result = strcmp( ea->category, eb->category );
	if ( result == 0 )
	{
		result = strcmp( ea->name, eb->name );
	}
	return result;
}

static void SortSamples()
{
	qsort( g_sampleEntries, g_sampleCount, sizeof( SampleEntry ), CompareSamples );
}

static void OnDrawUI( void )
{
	DrawUI( &s_context );
}

static int ResolveSampleArg( const char* arg )
{
	if ( arg == nullptr )
	{
		return -1;
	}
	int index = FindSample( arg );
	if ( index >= 0 )
	{
		return index;
	}
	char* end = nullptr;
	long n = strtol( arg, &end, 10 );
	if ( end != arg && *end == 0 && n >= 0 && n < g_sampleCount )
	{
		return (int)n;
	}
	fprintf( stderr, "samples: no sample matches '%s' (use --list)\n", arg );
	return -1;
}

static void OnInit( void )
{
#if defined( _WIN32 )
	timeBeginPeriod( 1 );
#endif

	const sg_environment env = sglue_environment();
	InitRenderer( &env );
	InitUI( &env, OnDrawUI );
	ui::ApplyPlotStyle();

	constexpr float DEG = 3.14159265358979323846f / 180.0f;
	s_context.camera.SetFov( 50.0f * DEG );
	s_context.camera.SetClip( 0.1f, Camera::kViewDistance );

	SortSamples();
	s_context.Load();
	s_context.showControls = s_context.newUser && s_frameLimit < 0;
	s_context.showUI = !s_hideUI;
	s_context.pause = s_startPaused;

	s_context.windowWidth = sapp_width();
	s_context.windowHeight = sapp_height();

	s_context.camera.Update( 0.0f, sapp_width(), sapp_height() );

	int index = s_context.sampleIndex;
	int requested = ResolveSampleArg( s_sampleArg );
	if ( requested >= 0 )
	{
		index = requested;
	}
	if ( index < 0 || index >= g_sampleCount )
	{
		index = 0;
	}
	SelectSample( &s_context, index, false );
}

static void FrameScene()
{
	Sample* sample = s_context.sample;
	AABB bounds;
	sample->ComputeBounds( &bounds );
	sample->FrameBounds( bounds, 1.0f );
}

static void OnEvent( const sapp_event* e )
{
	bool uiCaptured = false;
	if ( s_context.camera.m_thirdPerson == false )
	{
		uiCaptured = HandleEvent( e );
	}

	const bool releaseOrUnfocus = e->type == SAPP_EVENTTYPE_MOUSE_UP || e->type == SAPP_EVENTTYPE_UNFOCUSED;
	if ( uiCaptured == false || releaseOrUnfocus )
	{
		s_context.camera.OnEvent( e );
	}

	if ( uiCaptured )
	{

		if ( e->type == SAPP_EVENTTYPE_MOUSE_UP )
		{
			s_context.sample->HandleMouseUp( { e->mouse_x, e->mouse_y }, e->mouse_button );
		}
		return;
	}

	const int mods = e->modifiers & ( SAPP_MODIFIER_SHIFT | SAPP_MODIFIER_CTRL | SAPP_MODIFIER_ALT | SAPP_MODIFIER_SUPER );
	Sample* sample = s_context.sample;

	switch ( e->type )
	{
		case SAPP_EVENTTYPE_KEY_DOWN:
			SetKeyDown( e->key_code, true );
			if ( e->key_repeat )
			{
				break;
			}
			switch ( e->key_code )
			{
				case SAPP_KEYCODE_TAB:
					s_context.showUI = !s_context.showUI;
					break;

				case SAPP_KEYCODE_ESCAPE:
					if ( s_context.showControls )
					{
						s_context.showControls = false;
					}
					else
					{
						sample->Keyboard( e->key_code, mods );
					}
					break;

				case SAPP_KEYCODE_Q:
					if ( mods & SAPP_MODIFIER_CTRL )
					{
						sapp_request_quit();
						break;
					}
					sample->Keyboard( e->key_code, mods );
					break;

				case SAPP_KEYCODE_O:
					if ( mods & SAPP_MODIFIER_CTRL )
					{
						s_context.showUI = true;
						s_context.openSamplePicker = true;
					}
					else
					{
						s_context.singleStep += ( mods & SAPP_MODIFIER_SHIFT ) ? 5 : 1;
					}
					break;

				case SAPP_KEYCODE_P:
				case SAPP_KEYCODE_PAUSE:
					s_context.pause = !s_context.pause;
					break;

				case SAPP_KEYCODE_M:
					s_context.showMetrics = !s_context.showMetrics;
					break;

				case SAPP_KEYCODE_R:
					if ( mods == 0 )
					{
						s_context.requestRestart = true;
					}
					else
					{
						sample->Keyboard( e->key_code, mods );
					}
					break;

				case SAPP_KEYCODE_LEFT_BRACKET:
					SelectSample( &s_context, MaxInt( 0, s_context.sampleIndex - 1 ), false );
					break;

				case SAPP_KEYCODE_RIGHT_BRACKET:
					SelectSample( &s_context, MinInt( g_sampleCount - 1, s_context.sampleIndex + 1 ), false );
					break;

				case SAPP_KEYCODE_F:
					if ( mods == 0 )
					{
						FrameScene();
					}
					else
					{
						sample->Keyboard( e->key_code, mods );
					}
					break;

				case SAPP_KEYCODE_F12:
					RequestScreenshot( nullptr );
					break;

				default:
					sample->Keyboard( e->key_code, mods );
					break;
			}
			break;

		case SAPP_EVENTTYPE_KEY_UP:
			SetKeyDown( e->key_code, false );
			break;

		case SAPP_EVENTTYPE_CHAR:
			if ( e->char_code == '?' && e->key_repeat == false )
			{
				s_context.showControls = !s_context.showControls;
			}
			break;

		case SAPP_EVENTTYPE_MOUSE_DOWN:
			s_context.mouseX = e->mouse_x;
			s_context.mouseY = e->mouse_y;
			sample->HandleMouseDown( { e->mouse_x, e->mouse_y }, e->mouse_button, mods );
			break;

		case SAPP_EVENTTYPE_MOUSE_UP:
			sample->HandleMouseUp( { e->mouse_x, e->mouse_y }, e->mouse_button );
			break;

		case SAPP_EVENTTYPE_MOUSE_MOVE:
			s_context.mouseX = e->mouse_x;
			s_context.mouseY = e->mouse_y;
			s_context.mouseDX = e->mouse_dx;
			s_context.mouseDY = e->mouse_dy;
			sample->HandleMouseMove( { e->mouse_x, e->mouse_y } );
			break;

		case SAPP_EVENTTYPE_RESIZED:
		{
			int w = sapp_width();
			int h = sapp_height();
			s_context.minimized = ( w == 0 || h == 0 );
			if ( s_context.minimized == false )
			{
				s_context.windowWidth = w;
				s_context.windowHeight = h;
			}
		}
		break;

		default:
			break;
	}
}

static void LimitFrameRate( uint64_t frameStart )
{
	const float targetMs = 1000.0f / 60.0f;
	const float spinMs = 2.0f;
	int sleepMs = (int)( targetMs - spinMs - GetMilliseconds( frameStart ) );
	if ( sleepMs > 0 )
	{
		SleepMs( sleepMs );
	}
	while ( GetMilliseconds( frameStart ) < targetMs )
	{
		YieldThread();
	}
}

static void OnFrame( void )
{
	if ( s_frameLimit >= 0 && s_frame >= s_frameLimit )
	{
		sapp_quit();
		return;
	}

	const uint64_t frameStart = GetTicks();

	if ( s_context.minimized )
	{
		if ( s_frameLimit < 0 )
		{
			LimitFrameRate( frameStart );
		}
		return;
	}

	if ( s_screenshotPath != nullptr && s_frameLimit > 0 && s_frame == s_frameLimit - 1 )
	{
		RequestScreenshot( s_screenshotPath );
	}

	const float dt = (float)sapp_frame_duration();
	const int W = sapp_width();
	const int H = sapp_height();

	Camera& camera = s_context.camera;
	camera.Update( dt, W, H );
	camera.SetRenderTransform( 1.0f, s_context.viewZUp );
	camera.SetDrawDistance( s_context.drawDistance );
	SetDrawOrigin( camera.DrawOrigin() );

	ResetFrameArena();

	Sample* sample = s_context.sample;
	sample->ResetText();
	sample->Step();
	SetDrawOrigin( camera.DrawOrigin() );
	sample->RenderFrame();

	FrameInput fi{};
	fi.view = camera.View();
	fi.viewInv = camera.ViewInverse();
	fi.proj = camera.Proj();
	fi.projInv = camera.ProjInverse();
	fi.cameraPosition = camera.Position();
	Vec3 drawOrigin = GetDrawOrigin();
	fi.drawOrigin = ( drawOrigin );
	double gridPeriod = 10.0 * GROUND_GRID_CELL_SIZE;
	fi.gridWrap.x = (float)fmod( (double)drawOrigin.x, gridPeriod );
	fi.gridWrap.y = (float)fmod( (double)drawOrigin.z, gridPeriod );
	fi.time = (float)sapp_frame_count() / 60.0f;
	fi.debugMode = s_context.debugView;
	fi.disableShadows = !s_context.enableShadows;
	fi.disableAmbientOcclusion = !s_context.enableGtao;
	fi.zUp = s_context.viewZUp;

	const sg_swapchain sc = sglue_swapchain();
	RenderFrame( &sc, &fi );

	StartUIFrame( dt );
	RenderUI( &sc );

	if ( IsScreenshotRequested() )
	{
		CaptureScreenshotIfRequested( &sc, g_sampleEntries[s_context.sampleIndex].name );
	}

	sg_commit();
	++s_frame;

	if ( s_context.requestRestart )
	{
		s_context.requestRestart = false;
		SelectSample( &s_context, s_context.sampleIndex, true );
	}

	if ( s_frameLimit < 0 )
	{
		LimitFrameRate( frameStart );
	}
}

static void OnCleanup( void )
{
	delete s_context.sample;
	s_context.sample = nullptr;

	if ( s_frameLimit < 0 )
	{
		s_context.Save();
	}

	ReleaseSceneResources();
	ShutdownUI();
	ShutdownRenderer();

	const int errors = GetRenderErrorCount();
	fprintf( stderr, "samples: %d frames, %d sokol errors\n", s_frame, errors );

#if defined( _WIN32 )
	timeEndPeriod( 1 );
#endif

	exit( errors == 0 ? 0 : 1 );
}

static void OnAppLog( const char* tag, uint32_t logLevel, uint32_t logItemId, const char* message, uint32_t lineNumber,
					  const char* filename, void* userData )
{
	(void)tag;
	(void)logItemId;
	(void)filename;
	(void)userData;
	(void)lineNumber;

	const char* level = ( logLevel == 0 ) ? "panic" : ( logLevel == 1 ) ? "error" : ( logLevel == 2 ) ? "warn" : "info";
	fprintf( stderr, "sokol(level %s) %s\n", level, message ? message : "(no message)" );
	if ( logLevel == 0 )
	{
		fprintf( stderr, "Ensure you have OpenGL 4.5 if on Linux\n" );
		exit( 1 );
	}
}

sapp_desc sokol_main( int argc, char** argv )
{
	for ( int i = 1; i < argc; ++i )
	{
		if ( strcmp( argv[i], "--frames" ) == 0 && i + 1 < argc )
		{
			s_frameLimit = atoi( argv[++i] );
		}
		else if ( strcmp( argv[i], "--sample" ) == 0 && i + 1 < argc )
		{
			s_sampleArg = argv[++i];
		}
		else if ( strcmp( argv[i], "--screenshot" ) == 0 && i + 1 < argc )
		{
			s_screenshotPath = argv[++i];
		}
		else if ( strcmp( argv[i], "--hide-ui" ) == 0 )
		{
			s_hideUI = true;
		}
		else if ( strcmp( argv[i], "--pause" ) == 0 )
		{
			s_startPaused = true;
		}
		else if ( strcmp( argv[i], "--size" ) == 0 && i + 1 < argc )
		{
			int w = 0, h = 0;
			if ( sscanf( argv[++i], "%dx%d", &w, &h ) == 2 && w > 0 && h > 0 )
			{
				s_width = w;
				s_height = h;
			}
		}
		else if ( strcmp( argv[i], "--zup" ) == 0 )
		{
			s_context.viewZUp = true;
		}
		else if ( strcmp( argv[i], "--list" ) == 0 )
		{
			SortSamples();
			for ( int k = 0; k < g_sampleCount; ++k )
			{
				printf( "%d\t%s/%s\t%s\n", k, g_sampleEntries[k].category, g_sampleEntries[k].name,
						g_sampleEntries[k].description ? g_sampleEntries[k].description : "" );
			}
			exit( 0 );
		}
	}

	if ( s_screenshotPath != nullptr && s_frameLimit < 0 )
	{

		s_frameLimit = 90;
	}

	sapp_desc desc{};
	desc.init_cb = OnInit;
	desc.frame_cb = OnFrame;
	desc.event_cb = OnEvent;
	desc.cleanup_cb = OnCleanup;
	desc.logger.func = OnAppLog;
	desc.gl.major_version = 4;
	desc.gl.minor_version = 5;
	desc.width = s_width;
	desc.height = s_height;
	desc.sample_count = 1;
	desc.window_title = APP_TITLE;
	desc.swap_interval = 0;
	desc.high_dpi = true;
	return desc;
}
