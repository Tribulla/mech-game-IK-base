/* custom_mech_example.c - Minimal standalone C11 integration example for gaitcore::c.
 *
 * Shows how a game engine integrates the multithreaded procedural IK & gait core:
 *   1. Implement thread-safe read-only GcWorldCallbacks (raycast, sphere_push, terrain_height).
 *   2. Register custom mech archetypes with GcMechDesc + gc_register_mech_type.
 *   3. Step multiple mechs in parallel via gc_step and read packed poses + events.
 */

#include "gaitcore.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct ExampleBox {
	double min_xyz[3];
	double max_xyz[3];
} ExampleBox;

typedef struct ExampleWorld {
	double ground_y;
	ExampleBox boxes[2];
	int box_count;
} ExampleWorld;

static double world_height( void* user, double x, double z )
{
	(void)x;
	(void)z;
	const ExampleWorld* w = (const ExampleWorld*)user;
	return w ? w->ground_y : 0.0;
}

static int ray_aabb( const ExampleBox* b, double ox, double oy, double oz,
					 double dx, double dy, double dz, double max_d, GcRayHit* out_hit )
{
	const double orig[3] = { ox, oy, oz };
	const double dir[3] = { dx, dy, dz };
	double tmin = 0.0, tmax = max_d, sgn = 0.0;
	int hit_axis = -1;

	for ( int a = 0; a < 3; ++a )
	{
		if ( fabs( dir[a] ) < 1e-12 )
		{
			if ( orig[a] < b->min_xyz[a] || orig[a] > b->max_xyz[a] )
				return 0;
			continue;
		}
		double inv = 1.0 / dir[a];
		double t1 = ( b->min_xyz[a] - orig[a] ) * inv;
		double t2 = ( b->max_xyz[a] - orig[a] ) * inv;
		double s = -1.0;
		if ( t1 > t2 )
		{
			double tmp = t1; t1 = t2; t2 = tmp;
			s = 1.0;
		}
		if ( t1 > tmin )
		{
			tmin = t1;
			hit_axis = a;
			sgn = s;
		}
		if ( t2 < tmax )
			tmax = t2;
		if ( tmin > tmax )
			return 0;
	}

	if ( hit_axis < 0 || tmin <= 0.0 || tmin > max_d )
		return 0;

	out_hit->t = tmin;
	out_hit->px = ox + dx * tmin;
	out_hit->py = oy + dy * tmin;
	out_hit->pz = oz + dz * tmin;
	out_hit->nx = hit_axis == 0 ? sgn : 0.0;
	out_hit->ny = hit_axis == 1 ? sgn : 0.0;
	out_hit->nz = hit_axis == 2 ? sgn : 0.0;
	out_hit->surface = 1;
	return 1;
}

static int world_raycast( void* user, double ox, double oy, double oz,
						  double dx, double dy, double dz, double max_d, GcRayHit* out_hit )
{
	const ExampleWorld* w = (const ExampleWorld*)user;
	if ( !w )
		return 0;

	int hit = 0;
	double best = max_d;
	GcRayHit best_hit;
	memset( &best_hit, 0, sizeof( best_hit ) );

	if ( dy < -1e-9 && oy >= w->ground_y )
	{
		double t = ( w->ground_y - oy ) / dy;
		if ( t >= 0.0 && t <= best )
		{
			best = t;
			best_hit.t = t;
			best_hit.px = ox + dx * t;
			best_hit.py = w->ground_y;
			best_hit.pz = oz + dz * t;
			best_hit.nx = 0.0;
			best_hit.ny = 1.0;
			best_hit.nz = 0.0;
			best_hit.surface = 0;
			hit = 1;
		}
	}

	for ( int i = 0; i < w->box_count; ++i )
	{
		GcRayHit bh;
		if ( ray_aabb( &w->boxes[i], ox, oy, oz, dx, dy, dz, best, &bh ) )
		{
			best = bh.t;
			best_hit = bh;
			best_hit.surface = i + 1;
			hit = 1;
		}
	}

	if ( hit )
		*out_hit = best_hit;
	return hit;
}

static int world_sphere_push( void* user, double cx, double cy, double cz, double r,
							  double* out_xyz, double* out_nxyz )
{
	const ExampleWorld* w = (const ExampleWorld*)user;
	double x = cx, y = cy, z = cz;
	double nx = 0.0, ny = 1.0, nz = 0.0;
	int touched = 0;

	if ( w && y - r < w->ground_y )
	{
		y = w->ground_y + r;
		touched = 1;
	}
	if ( w )
	{
		for ( int i = 0; i < w->box_count; ++i )
		{
			const ExampleBox* b = &w->boxes[i];
			double qx = x < b->min_xyz[0] ? b->min_xyz[0] : ( x > b->max_xyz[0] ? b->max_xyz[0] : x );
			double qy = y < b->min_xyz[1] ? b->min_xyz[1] : ( y > b->max_xyz[1] ? b->max_xyz[1] : y );
			double qz = z < b->min_xyz[2] ? b->min_xyz[2] : ( z > b->max_xyz[2] ? b->max_xyz[2] : z );
			double dx = x - qx, dy = y - qy, dz = z - qz;
			double d = sqrt( dx * dx + dy * dy + dz * dz );
			if ( d > 1e-6 && d < r )
			{
				double inv = 1.0 / d;
				nx = dx * inv; ny = dy * inv; nz = dz * inv;
				x += ( r - d ) * nx;
				y += ( r - d ) * ny;
				z += ( r - d ) * nz;
				touched = 1;
			}
		}
	}

	out_xyz[0] = x; out_xyz[1] = y; out_xyz[2] = z;
	out_nxyz[0] = nx; out_nxyz[1] = ny; out_nxyz[2] = nz;
	return touched;
}

int main( void )
{
	ExampleWorld world;
	memset( &world, 0, sizeof( world ) );
	world.ground_y = 0.0;
	/* Two low step obstacles along +Z */
	world.boxes[0] = (ExampleBox){ { -8.0, 0.0, 8.0 }, { 8.0, 0.35, 11.0 } };
	world.boxes[1] = (ExampleBox){ { -8.0, 0.0, 11.0 }, { 8.0, 0.70, 15.0 } };
	world.box_count = 2;

	GcContext* ctx = gc_create( 0 ); /* 0 = auto hardware concurrency */
	if ( !ctx )
		return 1;

	GcWorldCallbacks cb = { world_raycast, world_sphere_push, world_height, &world };
	gc_set_world( ctx, &cb );

	/* Define and register a custom quadruped and a custom biped */
	GcMechDesc scout_quad, runner_biped;
	gc_mech_desc_init_default( &scout_quad, 4 );
	snprintf( scout_quad.name, sizeof( scout_quad.name ), "MY_QUAD_SCOUT" );
	scout_quad.speed_walk = 16.0f;
	scout_quad.speed_run = 24.0f;
	scout_quad.jump_speed = 12.0f;

	gc_mech_desc_init_default( &runner_biped, 2 );
	snprintf( runner_biped.name, sizeof( runner_biped.name ), "MY_BIPED" );
	runner_biped.speed_walk = 11.0f;
	runner_biped.speed_run = 16.0f;

	int kind_quad = gc_register_mech_type( ctx, &scout_quad );
	int kind_biped = gc_register_mech_type( ctx, &runner_biped );
	if ( kind_quad < 0 || kind_biped < 0 )
	{
		gc_destroy( ctx );
		return 1;
	}

	int h_quad = gc_spawn( ctx, kind_quad, -2.5f, 0.0f, 0.0f, 0.0f );
	int h_biped = gc_spawn( ctx, kind_biped, 2.5f, 0.0f, 0.0f, 0.0f );
	if ( h_quad < 0 || h_biped < 0 )
	{
		gc_destroy( ctx );
		return 1;
	}

	GcMechInfo info_quad, info_biped;
	gc_info( ctx, h_quad, &info_quad );
	gc_info( ctx, h_biped, &info_biped );

	float inputs[2 * GC_INPUT_STRIDE];
	float poses[512];
	float events[256 * GC_EVENT_FLOATS];
	int step_events = 0;

	/* Simulate 3 seconds at 60 Hz: walk 1.5s, sprint 1.0s, quick slide 0.5s */
	const float dt = 1.0f / 60.0f;
	for ( int frame = 0; frame < 180; ++frame )
	{
		memset( inputs, 0, sizeof( inputs ) );
		for ( int i = 0; i < 2; ++i )
		{
			float* in = &inputs[i * GC_INPUT_STRIDE];
			in[GC_IN_FWD] = 1.0f;
			if ( frame >= 90 )
				in[GC_IN_RUN] = 1.0f;
			if ( frame >= 150 )
			{
				in[GC_IN_DRIFT] = 1.0f;
				in[GC_IN_TURN] = 0.35f;
			}
		}

		gc_step( ctx, dt, inputs, 2 );
		gc_poses( ctx, poses, 512 );

		int nf = gc_events( ctx, events, 256 * GC_EVENT_FLOATS );
		for ( int e = 0; e + GC_EVENT_FLOATS <= nf; e += GC_EVENT_FLOATS )
		{
			if ( (int)events[e] == GC_EV_STEP )
				step_events++;
		}
	}

	const float* p_quad = poses;
	const float* p_biped = poses + info_quad.floats;
	printf( "[custom_mech_example] threads=%d step_events=%d\n", gc_get_threads( ctx ), step_events );
	printf( "  %-14s pos=(%.2f, %.2f, %.2f) speed=%.2f m/s planted=%d/%d\n",
			info_quad.name,
			p_quad[GC_POSE_POS], p_quad[GC_POSE_POS + 1], p_quad[GC_POSE_POS + 2],
			p_quad[GC_POSE_SPEED], (int)p_quad[GC_POSE_PLANTED], info_quad.legs );
	printf( "  %-14s pos=(%.2f, %.2f, %.2f) speed=%.2f m/s planted=%d/%d\n",
			info_biped.name,
			p_biped[GC_POSE_POS], p_biped[GC_POSE_POS + 1], p_biped[GC_POSE_POS + 2],
			p_biped[GC_POSE_SPEED], (int)p_biped[GC_POSE_PLANTED], info_biped.legs );

	gc_destroy( ctx );
	return 0;
}
