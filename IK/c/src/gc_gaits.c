// Gait timing library for 2-, 4-, 6-, and 8-legged walkers.
// Legs are ordered front-to-back in left/right pairs: L0, R0, L1, R1, ...
// offsets[leg] shifts a leg on the [0, 1) clock; swing opens when (phase + offset) mod 1 < (1 - duty).
// Keep these tables synchronized with IK/python/gaitcore/gaits.py.

#include "gc_internal.h"

static void set_pair_sync_groups( GcGait* g )
{
	int pairs = g->n_legs / 2;
	g->group_count = pairs;
	for ( int k = 0; k < pairs; ++k )
	{
		g->group_sizes[k] = 2;
		g->groups[k][0] = 2 * k;
		g->groups[k][1] = 2 * k + 1;
	}
}

static GcGait make_gait( const char* name, int n_legs, const double* offsets, double duty,
						 double stride, double cycle_min, double cycle_max, int min_support,
						 int adjacent_lock, int static_stable, double lift, int pair_sync,
						 double reach, double leap, double stride_pitch, int has_duty_fast,
						 double duty_fast, int phase_lock )
{
	GcGait g;
	memset( &g, 0, sizeof( g ) );
	snprintf( g.name, sizeof( g.name ), "%s", name );
	g.n_legs = n_legs;
	for ( int i = 0; i < n_legs; ++i )
		g.offsets[i] = offsets[i];
	g.duty = duty;
	g.stride = stride;
	g.cycle_min = cycle_min;
	g.cycle_max = cycle_max;
	g.min_support = min_support;
	g.adjacent_lock = adjacent_lock;
	g.static_stable = static_stable;
	g.lift = lift;
	g.pair_sync = pair_sync;
	g.reach = reach;
	g.leap = leap;
	g.stride_pitch = stride_pitch;
	g.has_duty_fast = has_duty_fast;
	g.duty_fast = duty_fast;
	g.phase_lock = phase_lock;
	if ( pair_sync )
		set_pair_sync_groups( &g );
	return g;
}

void gc_gaits_biped( GcGait* out, int* out_count )
{
	const double o0[2] = { 0.0, 0.5 };
	out[0] = make_gait( "stalk", 2, o0, 0.78, 1.2, 1.2, 1.8, 1, 1, 0, 0.7, 0, 1.0, 1.0, 1.0, 0, 0.0, 0 );
	out[1] = make_gait( "walk",  2, o0, 0.62, 2.3, 0.85, 1.25, 1, 1, 0, 1.0, 0, 1.0, 1.0, 1.0, 0, 0.0, 0 );
	out[2] = make_gait( "run",   2, o0, 0.36, 3.8, 0.52, 0.72, 0, 0, 0, 1.3, 0, 1.0, 1.0, 1.0, 0, 0.0, 0 );
	*out_count = 3;
}

void gc_gaits_quadruped( GcGait* out, int* out_count )
{
	const double o_crawl[4] = { 0.75, 0.25, 0.0, 0.5 };
	const double o_trot[4] = { 0.0, 0.5, 0.5, 0.0 };
	const double o_gallop[4] = { 0.5, 0.6, 0.0, 0.1 };
	const double o_bound[4] = { 0.6, 0.6, 0.35, 0.35 };

	out[0] = make_gait( "crawl", 4, o_crawl, 0.82, 1.1, 1.3, 2.0, 3, 1, 1, 0.8, 0, 1.0, 1.0, 1.0, 0, 0.0, 0 );
	out[1] = make_gait( "walk",  4, o_crawl, 0.66, 1.9, 0.9, 1.4, 2, 1, 0, 1.0, 0, 1.0, 1.0, 1.0, 0, 0.0, 0 );
	out[2] = make_gait( "trot",  4, o_trot,  0.50, 2.8, 0.55, 0.85, 2, 0, 0, 1.1, 0, 1.0, 1.0, 1.0, 0, 0.0, 0 );

	// Fast trot: diagonal pairs swap synced flight hops (duty_fast = 0.27) when moving at top walking speed.
	out[3] = make_gait( "trot_run", 4, o_trot, 0.5, 3.0, 0.34, 0.6, 0, 0, 0, 1.2, 0, 1.6, 0.06, 0.0, 1, 0.27, 1 );
	out[3].group_count = 2;
	out[3].group_sizes[0] = 2; out[3].groups[0][0] = 0; out[3].groups[0][1] = 3;
	out[3].group_sizes[1] = 2; out[3].groups[1][0] = 1; out[3].groups[1][1] = 2;

	out[4] = make_gait( "gallop", 4, o_gallop, 0.36, 4.0, 0.42, 0.6, 0, 0, 0, 1.3, 0, 1.0, 1.0, 1.0, 0, 0.0, 0 );
	out[5] = make_gait( "bound",  4, o_bound,  0.4,  3.4, 0.5,  0.85, 0, 0, 0, 1.6, 1, 2.2, 1.0, 1.0, 0, 0.0, 0 );
	*out_count = 6;
}

void gc_gaits_hexapod( GcGait* out, int* out_count )
{
	const double o_wave[6]   = { 0.667, 0.167, 0.833, 0.333, 0.0, 0.5 };
	const double o_ripple[6] = { 0.333, 0.833, 0.667, 0.167, 0.0, 0.5 };
	const double o_tripod[6] = { 0.0, 0.5, 0.5, 0.0, 0.0, 0.5 };
	const double o_bound[6]  = { 0.6, 0.6, 0.48, 0.48, 0.35, 0.35 };

	out[0] = make_gait( "wave",   6, o_wave,   0.86, 0.9, 1.0, 1.8, 5, 1, 1, 0.8, 0, 1.0, 1.0, 1.0, 0, 0.0, 0 );
	out[1] = make_gait( "ripple", 6, o_ripple, 0.66, 1.3, 0.6, 1.1, 4, 1, 0, 1.0, 0, 1.0, 1.0, 1.0, 0, 0.0, 0 );
	out[2] = make_gait( "tripod", 6, o_tripod, 0.50, 1.8, 0.34, 0.6, 3, 0, 0, 1.1, 0, 1.0, 1.0, 1.0, 0, 0.0, 0 );

	out[3] = make_gait( "tripod_run", 6, o_tripod, 0.5, 2.6, 0.32, 0.55, 0, 0, 0, 1.2, 0, 1.6, 0.06, 0.0, 1, 0.3, 1 );
	out[3].group_count = 2;
	out[3].group_sizes[0] = 3; out[3].groups[0][0] = 0; out[3].groups[0][1] = 3; out[3].groups[0][2] = 4;
	out[3].group_sizes[1] = 3; out[3].groups[1][0] = 1; out[3].groups[1][1] = 2; out[3].groups[1][2] = 5;

	out[4] = make_gait( "bound", 6, o_bound, 0.4, 2.8, 0.45, 0.8, 0, 0, 0, 1.5, 1, 2.0, 1.0, 1.0, 0, 0.0, 0 );
	*out_count = 5;
}

void gc_gaits_octopod( GcGait* out, int* out_count )
{
	const double o_wave[8]  = { 0.625, 0.125, 0.75, 0.25, 0.875, 0.375, 0.0, 0.5 };
	const double o_meta[8]  = { 0.25, 0.75, 0.5, 0.0, 0.75, 0.25, 0.0, 0.5 };
	const double o_tetra[8] = { 0.0, 0.5, 0.5, 0.0, 0.0, 0.5, 0.5, 0.0 };
	const double o_bound[8] = { 0.6, 0.6, 0.52, 0.52, 0.44, 0.44, 0.35, 0.35 };

	out[0] = make_gait( "wave",        8, o_wave,  0.86, 1.0, 1.1, 2.0, 6, 1, 1, 0.8, 0, 1.0, 1.0, 1.0, 0, 0.0, 0 );
	out[1] = make_gait( "metachronal", 8, o_meta,  0.70, 1.5, 0.6, 1.2, 5, 1, 0, 1.0, 0, 1.0, 1.0, 1.0, 0, 0.0, 0 );
	out[2] = make_gait( "tetrapod",    8, o_tetra, 0.50, 2.0, 0.36, 0.65, 4, 0, 0, 1.1, 0, 1.0, 1.0, 1.0, 0, 0.0, 0 );

	out[3] = make_gait( "tetrapod_run", 8, o_tetra, 0.5, 2.4, 0.4, 0.6, 0, 0, 0, 1.15, 0, 1.5, 0.1, 0.0, 1, 0.32, 1 );
	out[3].group_count = 2;
	out[3].group_sizes[0] = 4; out[3].groups[0][0] = 0; out[3].groups[0][1] = 3; out[3].groups[0][2] = 4; out[3].groups[0][3] = 7;
	out[3].group_sizes[1] = 4; out[3].groups[1][0] = 1; out[3].groups[1][1] = 2; out[3].groups[1][2] = 5; out[3].groups[1][3] = 6;

	out[4] = make_gait( "bound", 8, o_bound, 0.4, 2.4, 0.45, 0.8, 0, 0, 0, 1.3, 1, 1.8, 1.0, 1.0, 0, 0.0, 0 );
	*out_count = 5;
}

int gc_gait_find( const GcGait* gaits, int count, const char* name )
{
	if ( !name )
		return -1;
	for ( int i = 0; i < count; ++i )
	{
		if ( strcmp( gaits[i].name, name ) == 0 )
			return i;
	}
	return -1;
}

// Interpolates normalized phase offsets along the shortest wrap-around path on [0, 1).
double gc_phase_lerp( double a, double b, double t )
{
	double d = gc_mod1( b - a + 0.5 ) - 0.5;
	return gc_mod1( a + d * t );
}
