#pragma once

#include "gfx/draw_overlay.h"
#include "gfx/utility.h"

#include <stdint.h>

inline Vec4 Color( uint32_t hexRgb, float alpha = 1.0f )
{
	return MakeColorAlpha( (uint32_t)hexRgb, alpha );
}

inline uint32_t HexToImU32( uint32_t hexRgb, uint32_t alpha = 255 )
{

	return ( alpha << 24 ) | ( ( hexRgb & 0xFF ) << 16 ) | ( hexRgb & 0xFF00 ) | ( ( hexRgb >> 16 ) & 0xFF );
}

namespace Palette
{

constexpr uint32_t Static = 0xA9A9A9;	   
constexpr uint32_t Dynamic = 0xD2B48C;	   
constexpr uint32_t Sleeping = 0x778899;	   
constexpr uint32_t Kinematic = 0x4682B4;   
constexpr uint32_t KinematicIdle = 0xB0C4DE; 
constexpr uint32_t Ground = 0xA9A9A9;	   

constexpr uint32_t Highlight = 0xDAA520; 
constexpr uint32_t Good = 0x3CB371;		 
constexpr uint32_t Warning = 0xFFA500;	 
constexpr uint32_t Error = 0xDC143C;	 
constexpr uint32_t Info = 0x87CEEB;		 

constexpr uint32_t TerrainWater = 0x4F7A96; 
constexpr uint32_t TerrainSand = 0xC9B58E;	
constexpr uint32_t TerrainGrass = 0x8A9A5B; 
constexpr uint32_t TerrainForest = 0x6B7B4B;
constexpr uint32_t TerrainDirt = 0x8B7660; 
constexpr uint32_t TerrainRock = 0x8C8C88; 
constexpr uint32_t TerrainSnow = 0xE6E8EA; 

constexpr uint32_t SeriesColors[6] = {
	0x4682B4, 
	0xD2B48C, 
	0x3CB371, 
	0xCD5C5C, 
	0x9A8FBF, 
	0x5F9EA0, 

};

constexpr uint32_t Line = 0xC8C8C8;		  
constexpr uint32_t LineDim = 0x707070;	  
constexpr uint32_t Velocity = 0x87CEEB;	  
constexpr uint32_t Force = 0xFF00FF;	  
constexpr uint32_t Normal = 0xFFD700;	  
constexpr uint32_t AxisX = 0xE04040;	  
constexpr uint32_t AxisY = 0x40C040;
constexpr uint32_t AxisZ = 0x4060E0;

constexpr uint32_t TextTitle = 0xDAA520;	
constexpr uint32_t TextSubtitle = 0xD3D3D3; 
constexpr uint32_t TextStat = 0x2E8B57;		
constexpr uint32_t TextNormal = 0xE6E8EC;	
constexpr uint32_t TextMuted = 0x7D828C;	
constexpr uint32_t TextAlert = 0xFF0000;	
} 

inline uint32_t SeriesHex( int i )
{
	int n = (int)( sizeof( Palette::SeriesColors ) / sizeof( Palette::SeriesColors[0] ) );
	return Palette::SeriesColors[( ( i % n ) + n ) % n];
}

inline Vec4 Series( int i, float alpha = 1.0f )
{
	return Color( SeriesHex( i ), alpha );
}

struct Material
{
	Vec4 color;
	float metallic;
	float roughness;
};

inline Material MakeMaterial( uint32_t hexRgb, float metallic, float roughness, float alpha = 1.0f )
{
	return Material{ Color( hexRgb, alpha ), metallic, roughness };
}

namespace Materials
{
inline Material Static()
{
	return MakeMaterial( Palette::Static, 0.0f, 0.85f );
}
inline Material Dynamic()
{
	return MakeMaterial( Palette::Dynamic, 0.0f, 0.65f );
}
inline Material Sleeping()
{
	return MakeMaterial( Palette::Sleeping, 0.0f, 0.95f );
}
inline Material Kinematic()
{
	return MakeMaterial( Palette::Kinematic, 0.85f, 0.35f );
}
inline Material KinematicIdle()
{
	return MakeMaterial( Palette::KinematicIdle, 0.0f, 0.85f );
}
inline Material Highlight()
{
	return MakeMaterial( Palette::Highlight, 0.0f, 0.40f );
}
inline Material Good()
{
	return MakeMaterial( Palette::Good, 0.0f, 0.50f );
}
inline Material Warning()
{
	return MakeMaterial( Palette::Warning, 0.0f, 0.30f );
}
inline Material Error()
{
	return MakeMaterial( Palette::Error, 0.0f, 0.50f );
}

inline Material Ghost()
{
	return MakeMaterial( Palette::Info, 0.0f, 0.50f, 0.35f );
}

inline Material SeriesMaterial( int i )
{
	return MakeMaterial( SeriesHex( i ), 0.0f, 0.60f );
}

inline Material Terrain( uint32_t paletteHex )
{
	return MakeMaterial( paletteHex, 0.0f, 0.90f );
}
} 

namespace Style
{
constexpr float LinePx = 1.5f;		 
constexpr float LineBoldPx = 2.5f;	 
constexpr float PointPx = 6.0f;		 
constexpr float PointBoldPx = 10.0f; 
constexpr float GridCell = 1.0f;	 
} 
