#pragma once

#include "mech_world.h"

#include <math.h>

namespace mech
{

inline void BuildFrontline( World& w, uint64_t seed )
{
	w.Clear();
	w.name = "frontline";
	w.terrainMode = TerrainFrontline;
	w.terrainMaxY = 12.0f;
	w.groundPlaneY = -6.0f;
	w.extent = 175.0f;
	Rng rng( seed );
	w.phaseA = rng.Range( -0.6f, 0.6f );
	w.phaseB = rng.Range( 0.0f, 2.0f * MATH_PI );

	const float fixedCraters[8][2] = {
		{ -14.0f, -22.0f }, { 16.0f, -19.0f }, { -20.0f, 8.0f }, { 12.0f, 10.0f },
		{ -9.0f, 32.0f },	{ 22.0f, 35.0f },  { -28.0f, 48.0f }, { 26.0f, 50.0f },
	};
	for ( int i = 0; i < 8; ++i )
	{
		Crater c;
		c.x = fixedCraters[i][0] + rng.Range( -3.0f, 3.0f );
		c.z = fixedCraters[i][1] + rng.Range( -2.0f, 2.0f );
		c.r = rng.Range( 5.5f, 8.8f );
		c.depth = rng.Range( 1.15f, 1.85f );
		w.craters.push_back( c );
	}

	const float bridgeX[3] = { -22.0f, 0.0f, 22.0f };
	for ( float bx : bridgeX )
	{
		float tz = -8.0f + 5.0f * sinf( bx * 0.055f + w.phaseA );
		float yN = w.Height( bx, tz + 5.0f ), yS = w.Height( bx, tz - 5.0f );
		float yDeck = MaxFloat( yN, yS ) + 0.35f;
		w.AddAabb( bx - 2.75f, yDeck - 0.32f, tz - 4.2f, bx + 2.75f, yDeck, tz + 4.2f, LookEarth );
		w.AddAabb( bx - 2.95f, yDeck - 0.42f, tz - 4.2f, bx - 2.65f, yDeck + 0.14f, tz + 4.2f, LookSlate );
		w.AddAabb( bx + 2.65f, yDeck - 0.42f, tz - 4.2f, bx + 2.95f, yDeck + 0.14f, tz + 4.2f, LookSlate );
		w.AddRamp( bx, yS - 0.15f, tz - 6.4f, bx, yDeck, tz - 4.2f, 5.5f, LookEarth );
		w.AddRamp( bx, yDeck, tz + 4.2f, bx, yN - 0.15f, tz + 6.4f, 5.5f, LookEarth );
	}

	for ( int row = 0; row < 2; ++row )
	{
		float rz = 14.0f + (float)row * 8.0f;
		float xOff = row == 0 ? 0.0f : 3.2f;
		for ( float x = -34.0f + xOff; x <= 34.0f; x += 6.4f )
		{
			if ( fabsf( x ) < 2.2f )
			{
				continue; 
			}
			float gy = w.Height( x, rz );
			float th = rng.Range( 0.90f, 1.25f );
			w.AddAabb( x - 0.72f, gy - 0.35f, rz - 0.72f, x + 0.72f, gy + 0.25f, rz + 0.72f, LookSlate );
			w.AddRot( { x, gy + 0.25f + th * 0.45f, rz }, { 0.55f, th * 0.45f, 0.55f }, rng.Range( -0.15f, 0.15f ), 0.0f, LookStructure );
		}
	}

	const float pbPos[3][2] = { { -22.0f, 44.0f }, { 18.0f, 46.0f }, { -38.0f, 14.0f } };
	for ( auto& p : pbPos )
	{
		float px = p[0], pz = p[1];
		float py = w.Height( px, pz );
		w.AddAabb( px - 4.2f, py - 1.5f, pz - 3.6f, px + 4.2f, py + 2.4f, pz + 3.6f, LookStructure );
		w.AddAabb( px - 4.6f, py + 2.4f, pz - 4.0f, px + 4.6f, py + 3.2f, pz + 4.0f, LookSlate );

		w.AddAabb( px - 7.0f, py - 1.5f, pz - 5.0f, px - 4.5f, py + 1.15f, pz + 2.5f, LookEarth );
	}

	float wBase = 2.8f;
	w.AddAabb( -56.0f, wBase, 62.0f, -8.0f, wBase + 9.2f, 67.0f, LookStructure );
	w.AddAabb( -56.0f, wBase + 9.2f, 61.7f, -8.0f, wBase + 9.9f, 67.3f, LookSlate );
	w.AddAabb( 8.0f, wBase, 62.0f, 56.0f, wBase + 9.2f, 67.0f, LookStructure );
	w.AddAabb( 8.0f, wBase + 9.2f, 61.7f, 56.0f, wBase + 9.9f, 67.3f, LookSlate );

	for ( int side = -1; side <= 1; side += 2 )
	{
		for ( int k = 0; k < 4; ++k )
		{
			float px = (float)side * ( 14.0f + (float)k * 11.5f );
			w.AddAabb( px - 0.9f, wBase, 60.8f, px + 0.9f, wBase + 9.2f, 62.0f, LookSlate );
			w.AddAabb( px - 2.0f, wBase + 9.9f, 61.8f, px + 2.0f, wBase + 11.1f, 63.0f, LookStructure );
		}
		float ex = (float)side * 7.2f;
		w.AddAabb( ex - 1.2f, wBase, 62.3f, ex + 1.2f, wBase + 5.2f, 66.7f, LookRock );
	}

	for ( int i = -5; i <= 5; ++i )
	{
		if ( fabsf( (float)i ) < 1.5f )
		{
			continue;
		}
		float bx = (float)i * 9.5f;
		w.AddAabb( bx - 0.8f, wBase, 67.0f, bx + 0.8f, wBase + 8.5f, 70.0f, LookSlate );
	}
	w.AddRamp( -8.0f, w.Height( -24.0f, 75.0f ) - 0.2f, 75.0f, -38.0f, wBase + 9.2f, 75.0f, 7.0f, LookStructure );
	w.AddAabb( -42.0f, wBase, 67.0f, -34.0f, wBase + 9.2f, 78.5f, LookStructure );

	float gBase = w.Height( 0.0f, 64.5f );
	w.AddRamp( 0.0f, w.Height( 0.0f, 54.0f ) - 0.2f, 54.0f, 0.0f, gBase + 1.15f, 64.5f, 11.0f, LookRock );
	w.AddRamp( 0.0f, gBase + 1.15f, 64.5f, 0.0f, w.Height( 0.0f, 75.0f ) - 0.2f, 75.0f, 11.0f, LookRock );

	w.AddAabb( 20.0f, wBase, 72.0f, 28.0f, wBase + 12.2f, 80.0f, LookStructure );
	w.AddAabb( 19.6f, wBase + 12.2f, 71.6f, 28.4f, wBase + 13.0f, 80.4f, LookSlate );

	for ( int i = 0; i < 48; ++i )
	{
		float bx = rng.Range( -48.0f, 48.0f );
		float bz = rng.Range( -22.0f, 88.0f );
		if ( ( fabsf( bx ) < 4.0f && fabsf( bz + 34.0f ) < 12.0f ) || fabsf( bz + 8.0f ) < 7.0f )
		{
			continue;
		}
		float s = rng.Range( 0.45f, 1.35f );
		float by = w.Height( bx, bz );
		w.AddRot( { bx, by + s * 0.22f, bz }, { s, s * 0.45f, s * 0.8f }, rng.Range( 0.0f, 3.14f ), rng.Range( -0.3f, 0.3f ), LookRock );
	}

	w.spawnPos = { 0.0f, w.Height( 0.0f, -34.0f ), -34.0f };
	w.spawnYaw = 0.0f;
	w.aiHomes = {
		{ -34.0f, 0.0f, -24.0f }, { 34.0f, 0.0f, -24.0f }, { -32.0f, 0.0f, 28.0f }, { 32.0f, 0.0f, 28.0f },
		{ 0.0f, 0.0f, 84.0f },	  { -42.0f, 0.0f, 84.0f }, { 42.0f, 0.0f, 84.0f },	{ -44.0f, 0.0f, 12.0f },
	};

	Course c;
	c.name = "Frontline Assault";
	c.label = "Frontline";
	c.desc = "Duckboard bridge -> Dragon's Teeth -> Breach the 8 m wall -> West Parapet -> return.";
	World::AddGate( c, 0.0f, w.Height( 0.0f, -20.0f ), -20.0f, 0.0f, 5.5f );
	float tz0 = -8.0f + 5.0f * sinf( w.phaseA );
	World::AddGate( c, 0.0f, w.Height( 0.0f, tz0 + 4.8f ) + 0.1f, tz0, 0.0f, 5.0f );
	World::AddGate( c, 0.0f, w.Height( 0.0f, 32.0f ), 32.0f, 0.0f, 5.5f );
	World::AddGate( c, 0.0f, gBase + 1.15f, 64.5f, 0.0f, 5.5f );
	World::AddGate( c, -38.0f, wBase + 9.2f, 64.5f, -0.5f * MATH_PI, 4.5f );
	c.gates.back().via = { { -8.0f, w.Height( -8.0f, 75.0f ), 75.0f }, { -38.0f, wBase + 9.2f, 75.0f } };
	World::AddGate( c, -22.0f, w.Height( -22.0f, -22.0f ), -22.0f, MATH_PI, 5.5f );
	c.gates.back().via = { { -48.0f, w.Height( -48.0f, 50.0f ), 50.0f }, { -22.0f, w.Height( -22.0f, 10.0f ), 10.0f } };
	World::AddGate( c, 0.0f, w.Height( 0.0f, -20.0f ), -20.0f, 0.0f, 5.5f );
	w.courses.push_back( c );
}

} 
