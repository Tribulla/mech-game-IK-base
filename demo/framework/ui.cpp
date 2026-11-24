#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "ui.h"

#include "imgui.h"
#include "implot.h"

#include <initializer_list>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

namespace ui
{
static ImVec4 ToImVec4( uint32_t hexRgb )
{
	return ImGui::ColorConvertU32ToFloat4( HexToImU32( hexRgb ) );
}

bool Section( const char* label, bool defaultOpen )
{
	return ImGui::CollapsingHeader( label, defaultOpen ? ImGuiTreeNodeFlags_DefaultOpen : 0 );
}

void Divider( const char* label )
{
	ImGui::SeparatorText( label );
}

void Stat( const char* label, const char* valueFormat, ... )
{
	char buffer[256];
	va_list args;
	va_start( args, valueFormat );
	vsnprintf( buffer, sizeof( buffer ), valueFormat, args );
	va_end( args );

	ImGui::TextColored( ToImVec4( Palette::TextMuted ), "%s", label );
	ImGui::SameLine( ImGui::GetFontSize() * 8.0f );
	ImGui::TextColored( ToImVec4( Palette::TextStat ), "%s", buffer );
}

void Status( uint32_t paletteHex, const char* format, ... )
{
	char buffer[256];
	va_list args;
	va_start( args, format );
	vsnprintf( buffer, sizeof( buffer ), format, args );
	va_end( args );
	ImGui::TextColored( ToImVec4( paletteHex ), "%s", buffer );
}

void Hint( const char* format, ... )
{
	char buffer[512];
	va_list args;
	va_start( args, format );
	vsnprintf( buffer, sizeof( buffer ), format, args );
	va_end( args );
	ImGui::PushStyleColor( ImGuiCol_Text, ToImVec4( Palette::TextMuted ) );
	ImGui::TextWrapped( "%s", buffer );
	ImGui::PopStyleColor();
}

void KeyHint( const char* key, const char* description )
{
	ImGui::TextColored( ToImVec4( Palette::TextTitle ), "%s", key );
	ImGui::SameLine( ImGui::GetFontSize() * 6.0f );
	ImGui::PushStyleColor( ImGuiCol_Text, ToImVec4( Palette::TextMuted ) );
	ImGui::TextWrapped( "%s", description );
	ImGui::PopStyleColor();
}

bool SliderFloat( const char* label, float* v, float minValue, float maxValue, const char* format )
{
	return ImGui::SliderFloat( label, v, minValue, maxValue, format );
}

bool SliderInt( const char* label, int* v, int minValue, int maxValue )
{
	return ImGui::SliderInt( label, v, minValue, maxValue );
}

bool Checkbox( const char* label, bool* v )
{
	return ImGui::Checkbox( label, v );
}

bool Button( const char* label )
{
	return ImGui::Button( label, ImVec2( -1.0f, 0.0f ) );
}

bool SeedInput( const char* label, int* seed )
{
	bool changed = ImGui::InputInt( label, seed );
	char buttonLabel[96];
	snprintf( buttonLabel, sizeof( buttonLabel ), "Randomize##%s", label );
	if ( ImGui::Button( buttonLabel, ImVec2( -1.0f, 0.0f ) ) )
	{
		static unsigned s_counter = 0;
		s_counter += 0x9E3779B9u;
		*seed = (int)( ( (unsigned)time( nullptr ) * 2654435761u + s_counter ) % 1000000u );
		changed = true;
	}
	return changed;
}

bool Combo( const char* label, int* current, const char* const names[], int count )
{
	return ImGui::Combo( label, current, names, count );
}

ScrollingPlot::ScrollingPlot( const char* yLabel, float historySeconds )
	: m_yLabel( yLabel )
	, m_historySeconds( historySeconds )
{
	Clear();
}

void ScrollingPlot::Clear()
{
	m_seriesCount = 0;
	m_count = 0;
	m_head = 0;
}

void ScrollingPlot::Add( double t, std::initializer_list<float> values )
{
	int n = 0;
	for ( float v : values )
	{
		if ( n >= kMaxSeries )
		{
			break;
		}
		m_v[n][m_head] = v;
		n += 1;
	}
	if ( n > m_seriesCount )
	{
		m_seriesCount = n;
	}
	m_t[m_head] = (float)t;
	m_head = ( m_head + 1 ) % kCapacity;
	if ( m_count < kCapacity )
	{
		m_count += 1;
	}
}

void ScrollingPlot::Draw( std::initializer_list<const char*> seriesNames, float heightEm )
{
	ImVec2 size( -1.0f, heightEm * ImGui::GetFontSize() );
	char id[128];
	snprintf( id, sizeof( id ), "##plot_%p", (void*)this );
	if ( ImPlot::BeginPlot( id, size, ImPlotFlags_NoTitle | ImPlotFlags_NoMenus ) == false )
	{
		return;
	}

	double tMax = m_count > 0 ? m_t[( m_head - 1 + kCapacity ) % kCapacity] : 0.0;
	ImPlot::SetupAxes( nullptr, m_yLabel, ImPlotAxisFlags_None, ImPlotAxisFlags_AutoFit );
	ImPlot::SetupLegend( ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal );
	ImPlot::SetupAxisLimits( ImAxis_X1, tMax - m_historySeconds, tMax, ImPlotCond_Always );

	int offset = m_count < kCapacity ? 0 : m_head;
	const char* const* names = seriesNames.begin();
	int nameCount = (int)seriesNames.size();
	for ( int s = 0; s < m_seriesCount; ++s )
	{
		const char* name = s < nameCount ? names[s] : "series";
		ImPlot::SetNextLineStyle( ImPlot::GetColormapColor( s ), 1.5f );
		ImPlot::PlotLine( name, m_t, m_v[s], m_count, ImPlotLineFlags_None, offset );
	}
	ImPlot::EndPlot();
}

} 

void ui::ApplyPlotStyle()
{
	static ImU32 colors[6];
	for ( int i = 0; i < 6; ++i )
	{
		colors[i] = HexToImU32( Palette::SeriesColors[i] );
	}
	ImPlotColormap map = ImPlot::AddColormap( "SamplesBase", colors, 6, true );
	ImPlot::GetStyle().Colormap = map;

	ImPlotStyle& style = ImPlot::GetStyle();
	style.LineWeight = 1.5f;
	style.PlotPadding = ImVec2( 6.0f, 6.0f );
	ImVec4* c = style.Colors;
	c[ImPlotCol_FrameBg] = ImVec4( 0.0f, 0.0f, 0.0f, 0.0f );
	c[ImPlotCol_PlotBg] = ImVec4( 0.08f, 0.085f, 0.095f, 1.0f );
	c[ImPlotCol_PlotBorder] = ImVec4( 1.0f, 1.0f, 1.0f, 0.09f );
	c[ImPlotCol_LegendBg] = ImVec4( 0.10f, 0.105f, 0.115f, 0.90f );
	c[ImPlotCol_LegendBorder] = ImVec4( 1.0f, 1.0f, 1.0f, 0.09f );
	c[ImPlotCol_AxisGrid] = ImVec4( 1.0f, 1.0f, 1.0f, 0.07f );
	c[ImPlotCol_AxisText] = ImVec4( 0.60f, 0.62f, 0.66f, 1.0f );
}
