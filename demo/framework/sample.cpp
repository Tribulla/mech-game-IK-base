#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "sample.h"

#include "gfx/draw.h"
#include "gfx/gtao.h"
#include "gfx/keycodes.h"
#include "gfx/renderer.h"
#include "gfx/shadow.h"
#include "gfx/text.h"
#include "imgui.h"
#include "implot.h"
#include "jsmn.h"
#include "scene.h"
#include "screenshot.h"
#include "sokol_app.h"
#include "theme.h"
#include "ui.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef APP_TITLE
#define APP_TITLE "Mech IK Core Demo"
#endif

#define INFO_PANEL_WIDTH_EM 21.0f

static bool s_keyDown[512];

bool IsKeyDown( int key )
{
	return (unsigned)key < 512u ? s_keyDown[key] : false;
}

void SetKeyDown( int key, bool down )
{
	if ( (unsigned)key < 512u )
	{
		s_keyDown[key] = down;
	}
}

static const char* s_settingsFileName = "settings.ini";

static char* ReadWholeFile( int& size, const char* filename )
{
	FILE* file = fopen( filename, "rb" );
	if ( file == nullptr )
	{
		return nullptr;
	}
	fseek( file, 0, SEEK_END );
	size = (int)ftell( file );
	fseek( file, 0, SEEK_SET );
	if ( size <= 0 )
	{
		fclose( file );
		return nullptr;
	}
	char* data = (char*)malloc( size + 1 );
	size_t got = fread( data, 1, size, file );
	fclose( file );
	data[got] = 0;
	size = (int)got;
	return data;
}

void SampleContext::Save()
{
	FILE* file = fopen( s_settingsFileName, "w" );
	if ( file == nullptr )
	{
		return;
	}

	const SampleEntry* entry = ( sampleIndex >= 0 && sampleIndex < g_sampleCount ) ? &g_sampleEntries[sampleIndex] : nullptr;

	fprintf( file, "{\n" );
	fprintf( file, "  \"sample\": \"%s/%s\",\n", entry ? entry->category : "", entry ? entry->name : "" );
	fprintf( file, "  \"enableShadows\": %s,\n", enableShadows ? "true" : "false" );
	fprintf( file, "  \"enableGtao\": %s,\n", enableGtao ? "true" : "false" );
	fprintf( file, "  \"gtaoQuality\": %d,\n", (int)GetGtaoTraceParams().quality );
	fprintf( file, "  \"enableIbl\": %s,\n", GetIblEnabled() ? "true" : "false" );
	fprintf( file, "  \"exposure\": %g,\n", GetExposure() );
	fprintf( file, "  \"sunStrength\": %g,\n", GetSun().strength );
	fprintf( file, "  \"debugView\": %d,\n", debugView );
	fprintf( file, "  \"drawDistance\": %g,\n", drawDistance );
	fprintf( file, "  \"hertz\": %g\n", hertz );
	fprintf( file, "}\n" );
	fclose( file );
}

#define MAX_TOKENS 128

static bool JsonKeyIs( const char* json, const jsmntok_t* tok, const char* s )
{
	return tok->type == JSMN_STRING && (int)strlen( s ) == tok->end - tok->start &&
		   strncmp( json + tok->start, s, tok->end - tok->start ) == 0;
}

static void JsonValue( const char* json, const jsmntok_t* tok, char* out, int outSize )
{
	int count = tok->end - tok->start;
	if ( count > outSize - 1 )
	{
		count = outSize - 1;
	}
	memcpy( out, json + tok->start, count );
	out[count] = 0;
}

static bool JsonBool( const char* json, const jsmntok_t* tok )
{
	return strncmp( json + tok->start, "true", 4 ) == 0;
}

void SampleContext::Load()
{
	int size = 0;
	char* data = ReadWholeFile( size, s_settingsFileName );
	if ( data == nullptr )
	{
		return;
	}
	newUser = false;

	jsmn_parser parser;
	jsmntok_t tokens[MAX_TOKENS];
	jsmn_init( &parser );
	int tokenCount = jsmn_parse( &parser, data, size, tokens, MAX_TOKENS );
	char buffer[256];

	for ( int i = 0; i + 1 < tokenCount; ++i )
	{
		const jsmntok_t* key = &tokens[i];
		const jsmntok_t* val = &tokens[i + 1];
		if ( JsonKeyIs( data, key, "sample" ) )
		{
			JsonValue( data, val, buffer, sizeof( buffer ) );
			int index = FindSample( buffer );
			if ( index >= 0 )
			{
				sampleIndex = index;
			}
		}
		else if ( JsonKeyIs( data, key, "enableShadows" ) )
		{
			enableShadows = JsonBool( data, val );
		}
		else if ( JsonKeyIs( data, key, "enableGtao" ) )
		{
			enableGtao = JsonBool( data, val );
		}
		else if ( JsonKeyIs( data, key, "gtaoQuality" ) )
		{
			JsonValue( data, val, buffer, sizeof( buffer ) );
			GtaoTraceParams p = GetGtaoTraceParams();
			p.quality = (AmbientOcclusionQuality)ClampInt( atoi( buffer ), 0, 2 );
			SetGtaoTraceParams( p );
		}
		else if ( JsonKeyIs( data, key, "enableIbl" ) )
		{
			SetIblEnabled( JsonBool( data, val ) );
		}
		else if ( JsonKeyIs( data, key, "exposure" ) )
		{
			JsonValue( data, val, buffer, sizeof( buffer ) );
			SetExposure( strtof( buffer, nullptr ) );
		}
		else if ( JsonKeyIs( data, key, "sunStrength" ) )
		{
			JsonValue( data, val, buffer, sizeof( buffer ) );
			Sun sun = GetSun();
			sun.strength = strtof( buffer, nullptr );
			SetSun( sun );
		}
		else if ( JsonKeyIs( data, key, "debugView" ) )
		{
			JsonValue( data, val, buffer, sizeof( buffer ) );
			debugView = ClampInt( atoi( buffer ), 0, 4 );
		}
		else if ( JsonKeyIs( data, key, "drawDistance" ) )
		{
			JsonValue( data, val, buffer, sizeof( buffer ) );
			drawDistance = ClampFloat( strtof( buffer, nullptr ), 1.0f, Camera::kViewDistance );
		}
		else if ( JsonKeyIs( data, key, "hertz" ) )
		{
			JsonValue( data, val, buffer, sizeof( buffer ) );
			hertz = ClampFloat( strtof( buffer, nullptr ), 5.0f, 240.0f );
		}
	}

	free( data );
}

Sample::Sample( SampleContext* context )
{
	m_context = context;
	m_camera = &context->camera;

	if ( m_camera->m_thirdPerson )
	{
		sapp_lock_mouse( false );
		m_camera->m_thirdPerson = false;
	}

	if ( context->restart == false )
	{
		m_camera->SetView( -30.0f, 22.0f, 22.0f, { 0.0f, 1.0f, 0.0f } );
	}

	m_stepCount = 0;
	m_time = 0.0;
	m_textLine = 0;
	m_textIncrement = 22;

	m_handleCount = 0;
	m_activeHandle = -1;
	m_hoverHandle = -1;
	m_handleDepth = 0.0f;
	m_handleGrabOffset = Vec3_zero;

	memset( m_frameMs, 0, sizeof( m_frameMs ) );
	m_frameWrite = 0;
}

Sample::~Sample()
{
}

void Sample::ResetText()
{
	float fontSize = ImGui::GetFontSize();
	if ( m_context->showUI )
	{
		m_textLine = (int)( ImGui::GetFrameHeight() + 0.5f * fontSize );
	}
	else
	{
		m_textLine = (int)( 3.0f * fontSize );
	}
	m_textIncrement = (int)( 1.6f * fontSize + 0.5f );
}

void Sample::Step()
{
	m_frameMs[m_frameWrite % m_frameHistory] = (float)( sapp_frame_duration() * 1000.0 );
	m_frameWrite += 1;

	float dt = 0.0f;
	if ( m_context->pause == false || m_context->singleStep > 0 )
	{
		dt = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;
		m_context->singleStep = MaxInt( 0, m_context->singleStep - 1 );
	}

	if ( dt > 0.0f )
	{
		Update( dt );
		m_stepCount += 1;
		m_time += dt;
	}
}

void Sample::RenderFrame()
{
	Render();

	for ( int i = 0; i < m_handleCount; ++i )
	{
		const DragHandle& h = m_handles[i];
		if ( h.visible == false )
		{
			continue;
		}
		Vec3 p = { h.point->x, h.point->y, h.point->z };
		DrawBall( p, h.radius, Materials::Highlight() );
		if ( i == m_activeHandle || ( m_activeHandle < 0 && i == m_hoverHandle ) )
		{
			DrawCrossEx( p, 4.0f * h.radius, Color( Palette::Highlight ), Style::LinePx, OVERLAY_THICKNESS_PIXELS,
						 OVERLAY_OCCLUSION_DIM );
		}
	}
}

void Sample::DrawTextLine( const char* format, ... )
{
	va_list args;
	va_start( args, format );
	char buffer[512];
	vsnprintf( buffer, sizeof( buffer ), format, args );
	va_end( args );
	DrawScreenString( 8, m_textLine, Color( Palette::TextNormal ), buffer );
	m_textLine += m_textIncrement;
}

void Sample::DrawTextLineColored( uint32_t paletteHex, const char* format, ... )
{
	va_list args;
	va_start( args, format );
	char buffer[512];
	vsnprintf( buffer, sizeof( buffer ), format, args );
	va_end( args );
	DrawScreenString( 8, m_textLine, Color( paletteHex ), buffer );
	m_textLine += m_textIncrement;
}

PickRay Sample::MouseRay() const
{
	return m_camera->BuildPickRay( m_context->mouseX, m_context->mouseY );
}

PickRay Sample::ScreenRay( Vec2 p ) const
{
	return m_camera->BuildPickRay( p.x, p.y );
}

bool Sample::RayHitPlaneY( const PickRay& ray, float height, Vec3* hit )
{
	float dy = ray.translation.y;
	if ( fabsf( dy ) < 1e-9f )
	{
		return false;
	}
	float t = ( height - (float)ray.origin.y ) / dy;
	if ( t < 0.0f || t > 1.0f )
	{
		return false;
	}
	hit->x = (float)ray.origin.x + t * ray.translation.x;
	hit->y = height;
	hit->z = (float)ray.origin.z + t * ray.translation.z;
	return true;
}

int Sample::AddHandle( Vec3* point, float radius, bool visible )
{
	if ( m_handleCount >= m_maxHandles || point == nullptr )
	{
		return -1;
	}
	m_handles[m_handleCount] = { point, radius, visible };
	return m_handleCount++;
}

void Sample::FrameBounds( AABB bounds, float padding )
{
	float aspect = m_camera->m_height > 0 ? (float)m_camera->m_width / (float)m_camera->m_height : 1.0f;
	m_camera->Frame( bounds, aspect, padding );
}

static int PickHandle( const DragHandle* handles, int count, const PickRay& ray )
{
	Vec3 o = { (float)ray.origin.x, (float)ray.origin.y, (float)ray.origin.z };
	Vec3 d = ray.translation;
	float dd = Dot( d, d );
	if ( dd <= 0.0f )
	{
		return -1;
	}
	int best = -1;
	float bestT = 2.0f;
	for ( int i = 0; i < count; ++i )
	{
		Vec3 c = *handles[i].point;
		float r = 1.5f * handles[i].radius;
		float t = Dot( Sub( c, o ), d ) / dd;
		if ( t < 0.0f || t > 1.0f )
		{
			continue;
		}
		Vec3 closest = MulAdd( o, t, d );
		if ( DistanceSquared( closest, c ) <= r * r && t < bestT )
		{
			bestT = t;
			best = i;
		}
	}
	return best;
}

void Sample::HandleMouseDown( Vec2 p, int button, int mods )
{
	if ( button == MOUSE_LEFT && mods == 0 && m_handleCount > 0 )
	{
		PickRay ray = ScreenRay( p );
		int hit = PickHandle( m_handles, m_handleCount, ray );
		if ( hit >= 0 )
		{
			m_activeHandle = hit;
			Vec3 eye = { (float)m_camera->m_worldEye.x, (float)m_camera->m_worldEye.y, (float)m_camera->m_worldEye.z };
			Vec3 viewDir = Neg( m_camera->GetForward() );
			Vec3 c = *m_handles[hit].point;
			m_handleDepth = Dot( Sub( c, eye ), viewDir );

			Vec3 o = { (float)ray.origin.x, (float)ray.origin.y, (float)ray.origin.z };
			float denom = Dot( ray.translation, viewDir );
			if ( fabsf( denom ) > 1e-9f )
			{
				float t = ( m_handleDepth - Dot( Sub( o, eye ), viewDir ) ) / denom;
				m_handleGrabOffset = Sub( c, MulAdd( o, t, ray.translation ) );
			}
			else
			{
				m_handleGrabOffset = Vec3_zero;
			}
			return;
		}
	}

	MouseDown( p, button, mods );
}

void Sample::HandleMouseUp( Vec2 p, int button )
{
	if ( m_activeHandle >= 0 && button == MOUSE_LEFT )
	{
		m_activeHandle = -1;
		return;
	}
	MouseUp( p, button );
}

void Sample::HandleMouseMove( Vec2 p )
{
	PickRay ray = ScreenRay( p );

	if ( m_activeHandle >= 0 )
	{
		Vec3 eye = { (float)m_camera->m_worldEye.x, (float)m_camera->m_worldEye.y, (float)m_camera->m_worldEye.z };
		Vec3 viewDir = Neg( m_camera->GetForward() );
		Vec3 o = { (float)ray.origin.x, (float)ray.origin.y, (float)ray.origin.z };
		float denom = Dot( ray.translation, viewDir );
		if ( fabsf( denom ) > 1e-9f )
		{
			float t = ( m_handleDepth - Dot( Sub( o, eye ), viewDir ) ) / denom;
			*m_handles[m_activeHandle].point = Add( MulAdd( o, t, ray.translation ), m_handleGrabOffset );
		}
		return;
	}

	m_hoverHandle = m_handleCount > 0 ? PickHandle( m_handles, m_handleCount, ray ) : -1;

	MouseMove( p );
}

bool Sample::ComputeBounds( AABB* bounds )
{
	if ( GetBounds( bounds ) )
	{
		return true;
	}
	*bounds = { { -10.0f, 0.0f, -10.0f }, { 10.0f, 5.0f, 10.0f } };
	return false;
}

SampleEntry g_sampleEntries[MAX_SAMPLES];
int g_sampleCount = 0;

int RegisterSample( const char* category, const char* name, SampleCreateFcn* fcn, const char* description )
{
	int index = g_sampleCount;
	if ( index < MAX_SAMPLES )
	{
		g_sampleEntries[index] = { category, name, fcn, description };
		g_sampleCount += 1;
		return index;
	}
	return -1;
}

static bool EqualsNoCase( const char* a, const char* b )
{
	while ( *a && *b )
	{
		if ( tolower( (unsigned char)*a ) != tolower( (unsigned char)*b ) )
		{
			return false;
		}
		++a;
		++b;
	}
	return *a == *b;
}

int FindSample( const char* nameOrPath )
{
	if ( nameOrPath == nullptr || nameOrPath[0] == 0 )
	{
		return -1;
	}

	char path[256];
	for ( int i = 0; i < g_sampleCount; ++i )
	{
		snprintf( path, sizeof( path ), "%s/%s", g_sampleEntries[i].category, g_sampleEntries[i].name );
		if ( EqualsNoCase( nameOrPath, path ) || EqualsNoCase( nameOrPath, g_sampleEntries[i].name ) )
		{
			return i;
		}
	}
	return -1;
}

void SelectSample( SampleContext* context, int selection, bool restart )
{
	if ( selection < 0 || selection >= g_sampleCount )
	{
		return;
	}

	delete context->sample;
	context->sample = nullptr;

	context->sampleIndex = selection;
	context->restart = restart;
	context->sample = g_sampleEntries[selection].createFcn( context );

	AABB bounds;
	context->sample->ComputeBounds( &bounds );
	float diagonal = Distance( bounds.lowerBound, bounds.upperBound );
	SetShadowSplitFar( ClampFloat( 1.5f * diagonal, SHADOW_SPLIT_FAR, 300.0f ) );

	context->restart = false;
}

static ImVec4 HexColor( uint32_t hexRgb )
{
	return ImGui::ColorConvertU32ToFloat4( HexToImU32( hexRgb ) );
}

static int FuzzyScore( const char* needle, const char* haystack )
{
	if ( needle == nullptr || needle[0] == '\0' )
	{
		return 0;
	}

	int score = 0;
	int hi = 0;
	int prevMatchHi = -2;
	for ( int ni = 0; needle[ni] != '\0'; ++ni )
	{
		int nc = tolower( (unsigned char)needle[ni] );
		while ( haystack[hi] != '\0' && tolower( (unsigned char)haystack[hi] ) != nc )
		{
			++hi;
		}
		if ( haystack[hi] == '\0' )
		{
			return -1;
		}
		int bonus = 1;
		if ( hi == 0 )
		{
			bonus += 10;
		}
		else if ( !isalnum( (unsigned char)haystack[hi - 1] ) )
		{
			bonus += 5;
		}
		if ( hi == prevMatchHi + 1 )
		{
			bonus += 3;
		}
		score += bonus;
		prevMatchHi = hi;
		++hi;
	}
	return score;
}

static void RebuildPicker( const char* q, int* outFiltered, int* outCount )
{
	struct Scored
	{
		int idx;
		int score;
	};
	static Scored scored[MAX_SAMPLES];
	int n = 0;
	for ( int i = 0; i < g_sampleCount; ++i )
	{
		int nameScore = FuzzyScore( q, g_sampleEntries[i].name );
		int catScore = FuzzyScore( q, g_sampleEntries[i].category );
		int best = -1;
		if ( nameScore >= 0 )
		{
			best = nameScore * 2;
		}
		if ( catScore >= 0 && catScore > best )
		{
			best = catScore;
		}
		if ( best < 0 )
		{
			continue;
		}
		scored[n].idx = i;
		scored[n].score = best;
		n += 1;
	}
	for ( int i = 1; i < n; ++i )
	{
		Scored tmp = scored[i];
		int j = i - 1;
		while ( j >= 0 && scored[j].score < tmp.score )
		{
			scored[j + 1] = scored[j];
			--j;
		}
		scored[j + 1] = tmp;
	}
	for ( int i = 0; i < n; ++i )
	{
		outFiltered[i] = scored[i].idx;
	}
	*outCount = n;
}

static void DrawRow( const char* key, const char* desc )
{
	ImGui::TableNextRow();
	ImGui::TableSetColumnIndex( 0 );
	ImGui::TextColored( HexColor( Palette::TextTitle ), "%s", key );
	ImGui::TableSetColumnIndex( 1 );
	ImGui::TextUnformatted( desc );
}

static void DrawRenderMenu( SampleContext& ctx )
{
	ImGui::MenuItem( "Shadows", nullptr, &ctx.enableShadows );

	if ( ImGui::BeginMenu( "GTAO" ) )
	{
		ImGui::MenuItem( "Enable", nullptr, &ctx.enableGtao );
		GtaoTraceParams p = GetGtaoTraceParams();
		int quality = p.quality;
		static const char* qualityNames[] = { "Medium", "High", "Ultra" };
		if ( ImGui::Combo( "Quality", &quality, qualityNames, 3 ) )
		{
			p.quality = (AmbientOcclusionQuality)quality;
			SetGtaoTraceParams( p );
		}
		ImGui::EndMenu();
	}

	bool ibl = GetIblEnabled();
	if ( ImGui::MenuItem( "IBL", nullptr, &ibl ) )
	{
		SetIblEnabled( ibl );
	}

	ImGui::Separator();
	ImGui::PushItemWidth( 8.0f * ImGui::GetFontSize() );
	float ev = GetExposure();
	if ( ImGui::SliderFloat( "Exposure", &ev, -8.0f, 4.0f, "%.2f" ) )
	{
		SetExposure( ev );
	}
	Sun sun = GetSun();
	if ( ImGui::SliderFloat( "Sun", &sun.strength, 0.0f, 1.0f, "%.2f" ) )
	{
		SetSun( sun );
	}
	ImGui::PopItemWidth();

	ImGui::Separator();
	if ( ImGui::BeginMenu( "Debug View" ) )
	{
		static const char* viewNames[] = {
			"0 - lit", "1 - view-space distance", "2 - CSM cascade", "3 - view-space normal", "4 - raw GTAO",
		};
		for ( int i = 0; i < 5; ++i )
		{
			if ( ImGui::RadioButton( viewNames[i], ctx.debugView == i ) )
			{
				ctx.debugView = i;
			}
		}
		ImGui::EndMenu();
	}
}

static void FrameScene( SampleContext* context )
{
	AABB bounds;
	context->sample->ComputeBounds( &bounds );
	context->sample->FrameBounds( bounds, 1.0f );
}

static void DrawMenuBar( SampleContext* context )
{
	float fontSize = ImGui::GetFontSize();

	if ( ImGui::BeginMainMenuBar() )
	{
		if ( ImGui::BeginMenu( "Sim" ) )
		{
			ImGui::MenuItem( "Pause", "P", &context->pause );
			if ( ImGui::MenuItem( "Single Step", "O" ) )
			{
				context->singleStep += 1;
			}
			if ( ImGui::MenuItem( "Restart", "R" ) )
			{
				SelectSample( context, context->sampleIndex, true );
			}
			ImGui::Separator();
			if ( ImGui::MenuItem( "Previous Sample", "[" ) )
			{
				SelectSample( context, MaxInt( 0, context->sampleIndex - 1 ), false );
			}
			if ( ImGui::MenuItem( "Next Sample", "]" ) )
			{
				SelectSample( context, MinInt( g_sampleCount - 1, context->sampleIndex + 1 ), false );
			}
			ImGui::Separator();
			if ( ImGui::MenuItem( "Save Screenshot", "F12" ) )
			{
				RequestScreenshot( nullptr );
			}
			ImGui::Separator();
			if ( ImGui::MenuItem( "Quit", "Ctrl+Q" ) )
			{
				sapp_request_quit();
			}
			ImGui::EndMenu();
		}

		if ( ImGui::BeginMenu( "View" ) )
		{
			if ( ImGui::MenuItem( "Hide UI", "Tab" ) )
			{
				context->showUI = false;
			}
			if ( ImGui::MenuItem( "Frame Scene", "F" ) )
			{
				FrameScene( context );
			}
			ImGui::MenuItem( "Diagnostics", "M", &context->showMetrics );
			ImGui::PushItemWidth( 8.0f * fontSize );
			ImGui::SliderFloat( "Draw Distance", &context->drawDistance, 10.0f, Camera::kViewDistance, "%.0f m" );
			ImGui::PopItemWidth();
			context->sample->DrawViewMenu();
			ImGui::EndMenu();
		}

		if ( ImGui::BeginMenu( "Render" ) )
		{
			DrawRenderMenu( *context );
			ImGui::EndMenu();
		}

		if ( ImGui::BeginMenu( "Samples" ) )
		{
			if ( ImGui::MenuItem( "Search...", "Ctrl+O" ) )
			{
				context->openSamplePicker = true;
			}
			ImGui::Separator();
			int i = 0;
			while ( i < g_sampleCount )
			{
				const char* category = g_sampleEntries[i].category;
				if ( ImGui::BeginMenu( category ) )
				{
					while ( i < g_sampleCount && strcmp( category, g_sampleEntries[i].category ) == 0 )
					{
						bool selected = ( i == context->sampleIndex );
						if ( ImGui::MenuItem( g_sampleEntries[i].name, nullptr, selected ) )
						{
							SelectSample( context, i, false );
						}
						if ( g_sampleEntries[i].description && ImGui::IsItemHovered() )
						{
							ImGui::SetTooltip( "%s", g_sampleEntries[i].description );
						}
						++i;
					}
					ImGui::EndMenu();
				}
				else
				{
					while ( i < g_sampleCount && strcmp( category, g_sampleEntries[i].category ) == 0 )
					{
						++i;
					}
				}
			}
			ImGui::EndMenu();
		}

		static bool showAbout = false;
		if ( ImGui::BeginMenu( "Help" ) )
		{
			ImGui::MenuItem( "Controls", "?", &context->showControls );
			ImGui::MenuItem( "About", nullptr, &showAbout );
			ImGui::EndMenu();
		}

		ImGui::EndMainMenuBar();

		if ( context->showControls )
		{
			ImGui::SetNextWindowPos( { context->camera.m_width * 0.5f, context->camera.m_height * 0.4f }, ImGuiCond_Appearing,
									 { 0.5f, 0.5f } );
			ImGui::SetNextWindowSize( { 26.0f * fontSize, 0.0f }, ImGuiCond_Appearing );

			if ( ImGui::Begin( "Controls", &context->showControls,
							   ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize ) )
			{
				ImGui::SeparatorText( "Keyboard" );
				if ( ImGui::BeginTable( "keys", 2, ImGuiTableFlags_SizingFixedFit ) )
				{
					DrawRow( "Tab", "Show / hide UI" );
					DrawRow( "M", "Show / hide diagnostics" );
					DrawRow( "P", "Pause / resume" );
					DrawRow( "O", "Single step (Shift: 5)" );
					DrawRow( "R", "Restart sample" );
					DrawRow( "[  ]", "Previous / next sample" );
					DrawRow( "Ctrl+O", "Search samples" );
					DrawRow( "F", "Frame scene" );
					DrawRow( "F12", "Save screenshot" );
					DrawRow( "?", "Show / hide controls" );
					DrawRow( "Esc", "Cancel / close" );
					DrawRow( "Ctrl+Q", "Quit" );
					ImGui::EndTable();
				}

				ImGui::SeparatorText( "Mouse" );
				if ( ImGui::BeginTable( "mouse", 2, ImGuiTableFlags_SizingFixedFit ) )
				{
					DrawRow( "Alt + left drag", "Orbit camera" );
					DrawRow( "Alt + middle drag", "Pan camera" );
					DrawRow( "Alt + right drag", "Zoom (dolly)" );
					DrawRow( "Right drag", "Fly look (WASD to move)" );
					DrawRow( "Scroll", "Zoom" );
					DrawRow( "Left drag (gold ball)", "Move a drag handle" );
					ImGui::EndTable();
				}
			}
			ImGui::End();
		}

		if ( showAbout )
		{
			ImGui::SetNextWindowPos( { context->camera.m_width * 0.5f, context->camera.m_height * 0.5f }, ImGuiCond_Appearing,
									 { 0.5f, 0.5f } );
			ImGui::SetNextWindowSize( { 22.0f * fontSize, 0.0f }, ImGuiCond_Appearing );
			if ( ImGui::Begin( "About", &showAbout, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize ) )
			{
				ImGui::TextColored( HexColor( Palette::TextTitle ), "%s", APP_TITLE );
				ImGui::Spacing();
				ImGui::TextDisabled( "Renderer and UI from the Box3D/Vox3D sample app." );
				ImGui::TextDisabled( "sokol, Dear ImGui, ImPlot" );
			}
			ImGui::End();
		}
	}
}

static void DrawSamplePicker( SampleContext* context )
{
	float fontSize = ImGui::GetFontSize();

	static char query[64] = {};
	static char prevQuery[64] = {};
	static int highlight = 0;
	static int prevHighlight = 0;
	static int filtered[MAX_SAMPLES];
	static int filteredCount = 0;
	static bool justOpened = false;
	static bool forceScroll = false;

	if ( context->openSamplePicker )
	{
		ImGui::OpenPopup( "##sample_picker" );
		context->openSamplePicker = false;
		query[0] = '\0';
		prevQuery[0] = '\0';
		highlight = 0;
		prevHighlight = 0;
		RebuildPicker( query, filtered, &filteredCount );
		justOpened = true;
		forceScroll = true;
	}

	ImGui::SetNextWindowPos( { context->camera.m_width * 0.5f, context->camera.m_height * 0.35f }, ImGuiCond_Appearing,
							 { 0.5f, 0.5f } );
	ImGui::SetNextWindowSize( { 32.0f * fontSize, 0.0f }, ImGuiCond_Appearing );

	if ( ImGui::BeginPopup( "##sample_picker",
							ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings ) )
	{
		if ( justOpened )
		{
			ImGui::SetKeyboardFocusHere();
			justOpened = false;
		}

		ImGui::PushItemWidth( -1.0f );
		ImGui::InputTextWithHint( "##q", "Search by name or category...", query, sizeof( query ) );
		ImGui::PopItemWidth();

		if ( strcmp( query, prevQuery ) != 0 )
		{
			RebuildPicker( query, filtered, &filteredCount );
			strncpy( prevQuery, query, sizeof( prevQuery ) );
			prevQuery[sizeof( prevQuery ) - 1] = '\0';
			highlight = 0;
			forceScroll = true;
		}

		if ( filteredCount > 0 )
		{
			if ( ImGui::IsKeyPressed( ImGuiKey_DownArrow, true ) )
			{
				highlight = ( highlight + 1 ) % filteredCount;
			}
			if ( ImGui::IsKeyPressed( ImGuiKey_UpArrow, true ) )
			{
				highlight = ( highlight + filteredCount - 1 ) % filteredCount;
			}
		}
		bool commit = ImGui::IsKeyPressed( ImGuiKey_Enter, false ) || ImGui::IsKeyPressed( ImGuiKey_KeypadEnter, false );

		ImGui::BeginChild( "##results", { 0.0f, 14.0f * fontSize }, ImGuiChildFlags_Borders );
		for ( int row = 0; row < filteredCount; ++row )
		{
			int i = filtered[row];
			char label[160];
			snprintf( label, sizeof( label ), "%s  >  %s", g_sampleEntries[i].category, g_sampleEntries[i].name );
			bool sel = ( row == highlight );
			if ( ImGui::Selectable( label, sel ) )
			{
				highlight = row;
				commit = true;
			}
			if ( sel && ( forceScroll || highlight != prevHighlight ) )
			{
				ImGui::SetScrollHereY();
			}
		}
		ImGui::EndChild();
		prevHighlight = highlight;
		forceScroll = false;

		if ( commit && filteredCount > 0 )
		{
			SelectSample( context, filtered[highlight], false );
			ImGui::CloseCurrentPopup();
		}

		if ( ImGui::IsKeyPressed( ImGuiKey_Escape, false ) )
		{
			ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}
}

static void DrawInfoPanel( SampleContext* context )
{
	const SampleEntry& entry = g_sampleEntries[context->sampleIndex];
	Sample* sample = context->sample;
	float fontSize = ImGui::GetFontSize();
	float panelWidth = INFO_PANEL_WIDTH_EM * fontSize;
	float menuBarHeight = ImGui::GetFrameHeight();

	ImGui::SetNextWindowPos( { context->camera.m_width - panelWidth - 0.5f * fontSize, menuBarHeight + 0.5f * fontSize } );
	ImGui::SetNextWindowSize( { panelWidth, context->camera.m_height - menuBarHeight - fontSize } );

	ImGui::Begin( "Info", nullptr,
				  ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
					  ImGuiWindowFlags_NoTitleBar );

	ImGui::TextColored( HexColor( Palette::TextTitle ), "%s", entry.name );
	ImGui::TextColored( HexColor( Palette::TextSubtitle ), "%s", entry.category );
	if ( entry.description != nullptr && entry.description[0] != 0 )
	{
		ui::Hint( "%s", entry.description );
	}
	ImGui::Separator();

	if ( context->pause )
	{
		ImGui::TextColored( HexColor( Palette::TextAlert ), "PAUSED" );
		ImGui::SameLine();
		ImGui::TextDisabled( "(p)" );
		ImGui::Separator();
	}

	ImGui::TextColored( HexColor( Palette::TextStat ), "%.1f ms", (float)( sapp_frame_duration() * 1000.0 ) );
	ImGui::TextColored( HexColor( Palette::TextStat ), "step %d   t %.2f s", sample->m_stepCount, sample->m_time );
	ImGui::Separator();

	ImGui::PushItemWidth( 8.0f * fontSize );
	float before = ImGui::GetCursorPosY();
	sample->DrawControls();
	if ( ImGui::GetCursorPosY() != before )
	{
		ImGui::Separator();
	}
	ImGui::PopItemWidth();

	if ( ImGui::CollapsingHeader( "Camera" ) )
	{
		Vec3 p = context->camera.m_pivot;
		ImGui::TextColored( HexColor( Palette::TextStat ), "pivot (%.1f, %.1f, %.1f)", p.x, p.y, p.z );
		ImGui::TextColored( HexColor( Palette::TextStat ), "yaw/pitch (%.1f, %.1f)", MATH_RAD_TO_DEG * context->camera.m_yaw,
							MATH_RAD_TO_DEG * context->camera.m_pitch );
		ImGui::TextColored( HexColor( Palette::TextStat ), "radius %.1f m  speed %.1f m/s", context->camera.m_radius,
							context->camera.m_speed );
	}

	ImGui::Spacing();
	if ( ImGui::Button( "Restart (R)", ImVec2( -1.0f, 0.0f ) ) )
	{
		context->requestRestart = true;
	}

	ImGui::End();
}

static void DrawHud( SampleContext* context )
{
	const SampleEntry& entry = g_sampleEntries[context->sampleIndex];
	float fontSize = ImGui::GetFontSize();

	DrawScreenStringFormat( 8, (int)( 1.0f * fontSize ), Color( Palette::TextTitle ), "%s : %s", entry.category, entry.name );
	if ( context->pause )
	{
		DrawScreenString( 8, (int)( 2.2f * fontSize ), Color( Palette::TextAlert ), "PAUSED" );
	}
	DrawScreenStringFormat( 8, context->camera.m_height - (int)( 1.8f * fontSize ), Color( Palette::TextStat ),
							"%.1f ms  step %d", 1000.0f * (float)sapp_frame_duration(), context->sample->m_stepCount );
}

void Sample::DrawMetrics()
{
	if ( m_context->showMetrics == false )
	{
		return;
	}

	float fontSize = ImGui::GetFontSize();
	float panelWidth = INFO_PANEL_WIDTH_EM * fontSize;
	float margin = 0.5f * fontSize;
	float drawerWidth = m_camera->m_width - panelWidth - 1.5f * fontSize;
	float minHeight = 8.0f * fontSize;
	float maxHeight = MaxFloat( minHeight, m_camera->m_height - 3.0f * fontSize );

	static float s_drawerHeight = 0.0f;
	if ( s_drawerHeight < minHeight )
	{
		s_drawerHeight = 16.0f * fontSize;
	}
	s_drawerHeight = ClampFloat( s_drawerHeight, minHeight, maxHeight );

	ImGui::SetNextWindowPos( { margin, m_camera->m_height - s_drawerHeight - margin } );
	ImGui::SetNextWindowSize( { drawerWidth, s_drawerHeight }, ImGuiCond_FirstUseEver );
	ImGui::SetNextWindowSizeConstraints( { drawerWidth, minHeight }, { drawerWidth, maxHeight } );

	ImGui::Begin( "Metrics", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar );
	s_drawerHeight = ClampFloat( ImGui::GetWindowSize().y, minHeight, maxHeight );

	if ( ImGui::BeginTabBar( "MetricsTabs", ImGuiTabBarFlags_None ) == false )
	{
		ImGui::End();
		return;
	}

	DrawMetricsTabs();

	if ( ImGui::BeginTabItem( "Frame Time" ) )
	{
		int count = m_frameWrite < m_frameHistory ? m_frameWrite : m_frameHistory;
		int start = m_frameWrite - count;
		static float xs[m_frameHistory];
		static float ys[m_frameHistory];
		float maxValue = 1.0f;
		for ( int i = 0; i < count; ++i )
		{
			xs[i] = (float)i / 60.0f;
			ys[i] = m_frameMs[( start + i ) % m_frameHistory];
			maxValue = MaxFloat( maxValue, ys[i] );
		}
		if ( ImPlot::BeginPlot( "##frametime", ImGui::GetContentRegionAvail(), ImPlotFlags_NoTitle | ImPlotFlags_NoMenus ) )
		{
			ImPlot::SetupAxes( "t (s)", "ms" );
			ImPlot::SetupAxisLimits( ImAxis_X1, 0.0, m_frameHistory / 60.0, ImPlotCond_Always );
			ImPlot::SetupAxisLimits( ImAxis_Y1, 0.0, maxValue * 1.1, ImPlotCond_Always );
			ImPlot::PlotLine( "frame", xs, ys, count );
			ImPlot::EndPlot();
		}
		ImGui::EndTabItem();
	}

	if ( ImGui::BeginTabItem( "Renderer" ) )
	{
		const RenderStats st = GetRenderStats();
		const float frameMs = (float)( sapp_frame_duration() * 1000.0 );
		static float smoothedFrameMs = 0.0f;
		static float smoothedCpuMs = 0.0f;
		smoothedFrameMs = ( smoothedFrameMs <= 0.0f ) ? frameMs : smoothedFrameMs * 0.9f + frameMs * 0.1f;
		smoothedCpuMs = ( smoothedCpuMs <= 0.0f ) ? st.frameTimeMs : smoothedCpuMs * 0.9f + st.frameTimeMs * 0.1f;
		const float fps = ( smoothedFrameMs > 0.0f ) ? ( 1000.0f / smoothedFrameMs ) : 0.0f;

		const char* backend = "?";
		switch ( sg_query_backend() )
		{
			case SG_BACKEND_D3D11:
				backend = "D3D11";
				break;
			case SG_BACKEND_METAL_MACOS:
				backend = "Metal";
				break;
			case SG_BACKEND_GLCORE:
				backend = "GLCORE";
				break;
			default:
				break;
		}

		ImGui::Text( "%s   %.2f ms  (%.0f FPS)   CPU %.2f ms", backend, smoothedFrameMs, fps, smoothedCpuMs );
		ImGui::Text( "draw calls %d  (approx)", st.drawCallCount );
		ImGui::Separator();
		ImGui::Text( "opaque   cubes %d  spheres %d  capsules %d  geom %d (%d spans)", st.cubeCount, st.sphereCount,
					 st.capsuleCount, st.geomInstanceCount, st.geomSpanCount );
		ImGui::Text( "transp.  cubes %d  spheres %d  capsules %d  geom %d", st.cubeCountXp, st.sphereCountXp, st.capsuleCountXp,
					 st.geomInstanceCountXp );
		ImGui::Text( "overlays lines %d  points %d", st.lineCount, st.pointCount );
		ImGui::EndTabItem();
	}

	ImGui::EndTabBar();
	ImGui::End();
}

void DrawUI( SampleContext* context )
{
	if ( context->showUI == false )
	{
		DrawHud( context );
		return;
	}

	DrawMenuBar( context );
	DrawSamplePicker( context );
	DrawInfoPanel( context );
	context->sample->DrawWindows();
	context->sample->DrawMetrics();
}
