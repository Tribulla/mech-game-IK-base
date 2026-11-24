#pragma once

#include "theme.h"

#include <initializer_list>
#include <stdint.h>

namespace ui
{

bool Section( const char* label, bool defaultOpen = true );

void Divider( const char* label );

void Stat( const char* label, const char* valueFormat, ... );

void Status( uint32_t paletteHex, const char* format, ... );

void Hint( const char* format, ... );

void KeyHint( const char* key, const char* description );

bool SliderFloat( const char* label, float* v, float minValue, float maxValue, const char* format = "%.2f" );
bool SliderInt( const char* label, int* v, int minValue, int maxValue );
bool Checkbox( const char* label, bool* v );

bool Button( const char* label );

bool SeedInput( const char* label, int* seed );

bool Combo( const char* label, int* current, const char* const names[], int count );

class ScrollingPlot
{
public:
	static constexpr int kCapacity = 1024;
	static constexpr int kMaxSeries = 6;

	explicit ScrollingPlot( const char* yLabel = "value", float historySeconds = 10.0f );

	void Clear();
	void Add( double t, std::initializer_list<float> values );

	void Draw( std::initializer_list<const char*> seriesNames, float heightEm = 10.0f );

private:
	const char* m_yLabel;
	float m_historySeconds;
	int m_seriesCount;
	int m_count;
	int m_head;
	float m_t[kCapacity];
	float m_v[kMaxSeries][kCapacity];
};

void ApplyPlotStyle();
} 
