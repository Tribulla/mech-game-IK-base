#include "gaitcore.h"
#include "../src/gc_internal.h"
#include "../src/gc_thread.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_failures = 0;

#define EXPECT_TRUE( cond ) do { \
	if ( !( cond ) ) { \
		fprintf( stderr, "FAIL [%s:%d]: %s\n", __FILE__, __LINE__, #cond ); \
		g_failures++; \
	} \
} while ( 0 )

#define EXPECT_NEAR( a, b, eps ) do { \
	double _va = (a), _vb = (b), _ve = (eps); \
	if ( fabs( _va - _vb ) > _ve || isnan( _va ) || isnan( _vb ) ) { \
		fprintf( stderr, "FAIL [%s:%d]: %s (%.6f) != %s (%.6f) tol %.6f\n", \
				 __FILE__, __LINE__, #a, _va, #b, _vb, _ve ); \
		g_failures++; \
	} \
} while ( 0 )

static void test_vmath_geometry( void )
{
	GcV2 pts[4] = { { 0, 0 }, { 2, 0 }, { 2, 2 }, { 0, 2 } };
	GcV2 hull[GC_MAX_HULL_PTS];
	int hn = gc_convex_hull_2d( pts, 4, hull );
	EXPECT_TRUE( hn == 4 );
	EXPECT_NEAR( gc_support_margin( hull, hn, (GcV2){ 1.0, 1.0 } ), 1.0, 1e-6 );
	EXPECT_TRUE( gc_support_margin( hull, hn, (GcV2){ 3.0, 1.0 } ) < 0.0 );

	GcV3 pl[6];
	int k = 0;
	for ( int x = -1; x <= 1; ++x )
	{
		for ( int z = -1; z <= 1; z += 2 )
			pl[k++] = gc_v3( (double)x, 0.5 * (double)x, (double)z );
	}
	GcV3 c, n;
	EXPECT_TRUE( gc_fit_plane( pl, NULL, 6, &c, &n ) == 2 );
	EXPECT_NEAR( gc_v3_dot( n, gc_v3_norm( gc_v3( -0.5, 1.0, 0.0 ), GC_WORLD_UP ) ), 1.0, 1e-5 );

	GcFrame fr;
	gc_frame_init( &fr, gc_v3( 0, 0, 1 ), gc_v3( 0, 1, 0 ) );
	EXPECT_NEAR( fr.right.x, -1.0, 1e-6 );
	gc_frame_yaw( &fr, GC_PI * 0.5 );
	EXPECT_NEAR( fr.fwd.x, -1.0, 1e-6 );
}

static int within_limits( const GcLegIK* ik )
{
	double h0 = ik->hip_lim[0] * GC_RAD2DEG, h1 = ik->hip_lim[1] * GC_RAD2DEG;
	if ( !( h0 - 0.5 <= ik->hip_deg && ik->hip_deg <= h1 + 0.5 ) )
		return 0;
	for ( int i = 0; i < ik->n_segs - 1; ++i )
	{
		double b0 = ik->bend_lim[i][0] * GC_RAD2DEG, b1 = ik->bend_lim[i][1] * GC_RAD2DEG;
		if ( !( b0 - 0.5 <= ik->bends_deg[i] && ik->bends_deg[i] <= b1 + 0.5 ) )
			return 0;
	}
	return fabs( ik->yaw_deg ) <= ik->coxa_max * GC_RAD2DEG + 0.5;
}

static void test_ik_solvers_and_limits( void )
{
	/* Two-bone exact + unreachable rigidity */
	const double tbs[4][2] = { { 1.0, -1.0 }, { 0.3, -1.6 }, { 1.5, 0.2 }, { -0.5, -1.2 } };
	for ( int i = 0; i < 4; ++i )
	{
		double kx, ky, fx, fy;
		int ok = gc_solve_two_bone( 1.0, 1.0, tbs[i][0], tbs[i][1], 1, &kx, &ky, &fx, &fy );
		EXPECT_TRUE( ok );
		EXPECT_NEAR( fx, tbs[i][0], 1e-4 );
		EXPECT_NEAR( fy, tbs[i][1], 1e-4 );
		EXPECT_NEAR( hypot( kx, ky ), 1.0, 1e-5 );
	}
	double kx, ky, fx, fy;
	EXPECT_TRUE( !gc_solve_two_bone( 1.0, 1.0, 5.0, 0.0, 1, &kx, &ky, &fx, &fy ) );
	EXPECT_NEAR( hypot( fx - kx, fy - ky ), 1.0, 1e-5 );

	/* FABRIK 3-seg digitigrade */
	const double L[3] = { 1.25, 1.35, 0.72 };
	const int sides[2] = { 1, -1 };
	double pts[4][2];
	gc_initial_pose_2d( L, 3, sides, 0.2, -2.3, pts );
	EXPECT_TRUE( gc_solve_fabrik_2d( L, 3, pts, 0.2, -2.3, sides, 40, 1e-3 ) );
	EXPECT_TRUE( hypot( pts[3][0] - 0.2, pts[3][1] + 2.3 ) < 1e-2 );

	/* 400-step sweep stays inside limits */
	GcLimits lim;
	memset( &lim, 0, sizeof( lim ) );
	lim.has_coxa = 1; lim.coxa = 60.0;
	lim.has_hip = 1; lim.hip[0] = -30.0; lim.hip[1] = 70.0;
	lim.bend_count = 2;
	lim.bend[0][0] = 10.0; lim.bend[0][1] = 140.0;
	lim.bend[1][0] = 15.0; lim.bend[1][1] = 140.0;

	const double l3[3] = { 1.6, 1.5, 0.55 };
	const int s3[2] = { 1, 1 };
	GcLegIK ik;
	gc_legik_init( &ik, GC_PLANE_RADIAL, 0.3, l3, 3, s3, gc_v3( 1, 0, 0 ), &lim );
	GcFrame fr;
	gc_frame_init( &fr, gc_v3( 0, 0, 1 ), gc_v3( 0, 1, 0 ) );
	GcV3 js[GC_MAX_JOINTS];
	for ( int i = 0; i < 400; ++i )
	{
		double a = i * 0.07;
		GcV3 target = gc_v3( 2.4 * cos( a ), 1.5 * sin( a * 1.7 ) - 0.6, 2.4 * sin( a * 0.9 ) );
		gc_legik_solve( &ik, gc_v3_zero(), &fr, target, 0.0, js );
		EXPECT_TRUE( within_limits( &ik ) );
	}

	/* Feasible matches full solve */
	GcLimits lim2;
	memset( &lim2, 0, sizeof( lim2 ) );
	lim2.has_coxa = 1; lim2.coxa = 60.0;
	lim2.has_hip = 1; lim2.hip[0] = -30.0; lim2.hip[1] = 75.0;
	lim2.bend_count = 1; lim2.bend[0][0] = 8.0; lim2.bend[0][1] = 150.0;
	const double l2[2] = { 1.4, 1.6 };
	const int s2[1] = { 1 };
	gc_legik_init( &ik, GC_PLANE_RADIAL, 0.3, l2, 2, s2, gc_v3( 1, 0, 0 ), &lim2 );
	int agree = 0, total = 0;
	for ( int ix = -6; ix <= 6; ++ix )
	{
		for ( int iz = -6; iz <= 6; ++iz )
		{
			GcV3 t = gc_v3( ix * 0.45, -1.15, iz * 0.45 );
			int f = gc_legik_feasible( &ik, gc_v3_zero(), &fr, t, 0.0 );
			GcLegIK probe;
			gc_legik_init( &probe, GC_PLANE_RADIAL, 0.3, l2, 2, s2, gc_v3( 1, 0, 0 ), &lim2 );
			for ( int k = 0; k < 4; ++k )
				gc_legik_solve( &probe, gc_v3_zero(), &fr, t, 0.0, js );
			if ( f == ( probe.err < 0.03 ? 1 : 0 ) )
				agree++;
			total++;
		}
	}
	EXPECT_TRUE( (double)agree / (double)total > 0.9 );

	/* Plane roll preserves foot & lengths */
	const double lp[3] = { 1.4, 1.5, 1.1 };
	GcLimits limp;
	memset( &limp, 0, sizeof( limp ) );
	limp.has_coxa = 1; limp.coxa = 75.0;
	limp.has_hip = 1; limp.hip[0] = -35.0; limp.hip[1] = 80.0;
	limp.bend_count = 1; limp.bend[0][0] = 8.0; limp.bend[0][1] = 150.0;
	GcLegIK ik0, ikp, ikm;
	gc_legik_init( &ik0, GC_PLANE_RADIAL, 0.25, lp, 3, s3, gc_v3( 1, 0, 0 ), &limp );
	gc_legik_init( &ikp, GC_PLANE_RADIAL, 0.25, lp, 3, s3, gc_v3( 1, 0, 0 ), &limp );
	gc_legik_init( &ikm, GC_PLANE_RADIAL, 0.25, lp, 3, s3, gc_v3( 1, 0, 0 ), &limp );
	GcV3 tgt = gc_v3( 2.2, -1.4, 0.2 );
	GcV3 j0[GC_MAX_JOINTS], jp[GC_MAX_JOINTS], jm[GC_MAX_JOINTS];
	int nj0 = gc_legik_solve( &ik0, gc_v3_zero(), &GC_LOCAL_FRAME, tgt, 0.0, j0 );
	GcV3 n0 = ik0.plane_normal;
	int njp = gc_legik_solve( &ikp, gc_v3_zero(), &GC_LOCAL_FRAME, tgt, 0.3, jp );
	int njm = gc_legik_solve( &ikm, gc_v3_zero(), &GC_LOCAL_FRAME, tgt, -0.3, jm );
	EXPECT_TRUE( nj0 == 5 && njp == 5 && njm == 5 );
	EXPECT_TRUE( ikp.err < 1e-4 && ikm.err < 1e-4 );
	EXPECT_TRUE( gc_v3_dist( jp[njp - 1], tgt ) < 1e-4 );
	EXPECT_TRUE( gc_v3_dist( jm[njm - 1], tgt ) < 1e-4 );
	EXPECT_TRUE( gc_v3_dot( gc_v3_sub( jp[2], j0[2] ), n0 ) * gc_v3_dot( gc_v3_sub( jm[2], j0[2] ), n0 ) < -1e-4 );
}

static void test_gaitsel_and_blend( void )
{
	GcGait q[GC_MAX_GAITS];
	int qn = 0;
	gc_gaits_quadruped( q, &qn );
	int iw = gc_gait_find( q, qn, "walk" );
	int itr = gc_gait_find( q, qn, "trot_run" );
	EXPECT_TRUE( iw >= 0 && itr >= 0 );

	GcGaitBlend b;
	gc_blend_init( &b, iw, &q[iw] );
	gc_blend_start( &b, itr, &q[itr], 0.5 );
	char lab[64];
	gc_blend_label( &b, lab, sizeof( lab ) );
	EXPECT_TRUE( strcmp( lab, "walk > trot_run" ) == 0 );
	double prev[4];
	memcpy( prev, b.eff.offsets, 4 * sizeof( double ) );
	while ( b.t < 1.0 )
	{
		gc_blend_update( &b, 1.0 / 60.0 );
		for ( int i = 0; i < 4; ++i )
		{
			double d = fabs( gc_mod1( prev[i] - b.eff.offsets[i] + 0.5 ) - 0.5 );
			EXPECT_TRUE( d < 0.05 );
		}
		if ( b.s < 0.85 )
			EXPECT_TRUE( b.eff.group_count == 0 && b.eff.phase_lock == 0 );
		memcpy( prev, b.eff.offsets, 4 * sizeof( double ) );
	}
	EXPECT_TRUE( b.eff.group_count == q[itr].group_count );
	gc_blend_label( &b, lab, sizeof( lab ) );
	EXPECT_TRUE( strcmp( lab, "trot_run" ) == 0 );
}

static void test_stub_world_obox( void )
{
	GcStubWorld* w = gc_stub_world_create( 0.0, 1 );
	gc_stub_world_add_obox( w, 0.0, 0.5, 0.0, 1.0, 0.5, 1.0, 0.3, 0.2 );
	GcWorldCallbacks cb = gc_stub_world_callbacks( w );
	GcRayHit h;
	EXPECT_TRUE( cb.raycast( cb.user, 0.0, 5.0, 0.0, 0.0, -1.0, 0.0, 10.0, &h ) );
	EXPECT_NEAR( h.py, 0.5 + 0.5 / cos( 0.2 ), 1e-3 );
	EXPECT_NEAR( h.ny, cos( 0.2 ), 1e-3 );

	double xyz[3], nxyz[3];
	EXPECT_TRUE( cb.sphere_push( cb.user, 0.0, 0.9, 0.0, 0.2, xyz, nxyz ) );
	EXPECT_TRUE( nxyz[1] > 0.9 );
	gc_stub_world_destroy( w );
}

static void test_all_mechs_walk_slide_jump( void )
{
	GcStubWorld* w = gc_stub_world_create( 0.0, 1 );
	/* Add a 0.25 m thin wall + a block staircase */
	gc_stub_world_add_box( w, -0.12, 0.0, 12.0, 0.12, 2.2, 22.0 );
	GcWorldCallbacks cb = gc_stub_world_callbacks( w );

	GcContext* ctx = gc_create( 1 );
	gc_set_world( ctx, &cb );
	float pose[256];
	float inp[GC_INPUT_STRIDE];

	for ( int k = 0; k < gc_kind_count(); ++k )
	{
		gc_reset( ctx );
		gc_set_world( ctx, &cb );
		int h = gc_spawn( ctx, k, 0.0f, 0.0f, 0.0f, 0.0f );
		EXPECT_TRUE( h == 0 );
		GcMechInfo info;
		EXPECT_TRUE( gc_info( ctx, h, &info ) );

		/* Run forward for 2.0 s, then slide for 0.6 s, then recover for 1.0 s, then jump */
		for ( int step = 0; step < 260; ++step )
		{
			memset( inp, 0, sizeof( inp ) );
			inp[0] = 1.0f; /* fwd */
			if ( step < 120 )
			{
				inp[3] = 1.0f; /* run */
			}
			else if ( step < 156 )
			{
				inp[3] = 1.0f;
				inp[7] = 1.0f; /* slide */
				inp[2] = 0.4f;
			}
			else if ( step == 216 )
			{
				inp[6] = 1.0f; /* jump */
			}
			gc_step( ctx, 1.0f / 60.0f, inp, 1 );
		}
		int n = gc_poses( ctx, pose, 256 );
		EXPECT_TRUE( n == info.floats );
		for ( int fi = 0; fi < n; ++fi )
		{
			EXPECT_TRUE( isfinite( pose[fi] ) );
		}
		EXPECT_TRUE( pose[2] > 15.0f ); /* moved forward cleanly */
		EXPECT_TRUE( pose[1] > 0.2f && pose[1] < 20.0f );
	}

	gc_destroy( ctx );
	gc_stub_world_destroy( w );
}

static GcStubWorld* build_bench_stub_world( void )
{
	GcStubWorld* w = gc_stub_world_create( 0.0, 1 );
	uint64_t rng = 0x123456789ABCDEF1ULL;
	for ( int i = 0; i < 120; ++i )
	{
		rng ^= rng >> 12; rng ^= rng << 25; rng ^= rng >> 27;
		double u1 = (double)( ( rng * 0x2545F4914F6CDD1DULL ) >> 11 ) / 9007199254740992.0;
		rng ^= rng >> 12; rng ^= rng << 25; rng ^= rng >> 27;
		double u2 = (double)( ( rng * 0x2545F4914F6CDD1DULL ) >> 11 ) / 9007199254740992.0;
		rng ^= rng >> 12; rng ^= rng << 25; rng ^= rng >> 27;
		double u3 = (double)( ( rng * 0x2545F4914F6CDD1DULL ) >> 11 ) / 9007199254740992.0;
		double ang = u1 * GC_TAU;
		double rad = sqrt( u2 ) * 55.0;
		double sx = 0.35 + 0.65 * u3;
		double sy = 0.12 + 0.30 * u1;
		double sz = 0.35 + 0.65 * u2;
		gc_stub_world_add_obox( w, rad * sin( ang ), sy * 0.45, rad * cos( ang ),
								sx, sy * 0.5, sz, ang, ( u3 - 0.5 ) * 0.3 );
	}
	return w;
}

static void run_determinism_pass( GcStubWorld* w, int num_threads, int n_mechs, int steps,
								  float* out_poses, int* out_pose_n, float* out_ev, int* out_ev_n )
{
	GcWorldCallbacks cb = gc_stub_world_callbacks( w );
	GcContext* ctx = gc_create( num_threads );
	gc_set_world( ctx, &cb );

	GcBenchConfig cfg = { (float)( 1.0 / 60.0 ), 0.0f, 20.0f, 1.0f, 1, 1 };
	gc_set_bench_config( ctx, &cfg );

	for ( int i = 0; i < n_mechs; ++i )
	{
		float a = (float)i * (float)( GC_TAU / (double)n_mechs );
		float r = 12.0f + (float)( i / 12 ) * 8.0f;
		gc_spawn( ctx, i % gc_kind_count(), r * sinf( a ), 0.0f, r * cosf( a ), a + 1.57f );
	}

	float* inputs = (float*)calloc( (size_t)n_mechs * GC_INPUT_STRIDE, sizeof( float ) );
	int ev_total = 0;
	for ( int s = 0; s < steps; ++s )
	{
		for ( int i = 0; i < n_mechs; ++i )
		{
			float* in = inputs + i * GC_INPUT_STRIDE;
			in[0] = 1.0f;
			in[2] = ( i & 1 ) ? 0.35f : -0.35f;
			in[3] = ( ( s / 40 ) % 2 == 0 ) ? 1.0f : 0.0f;
			in[7] = ( s >= 60 && s < 85 ) ? 1.0f : 0.0f;
			in[6] = ( s == 110 ) ? 1.0f : 0.0f;
		}
		if ( s == 90 && num_threads > 1 )
			gc_set_threads( ctx, num_threads ); /* verify safe thread-count call mid-sim */
		gc_step( ctx, 1.0f / 60.0f, inputs, n_mechs );
		int ne = gc_events( ctx, out_ev + ev_total, 200000 - ev_total );
		ev_total += ne;
	}
	*out_pose_n = gc_poses( ctx, out_poses, 32768 );
	*out_ev_n = ev_total;
	free( inputs );
	gc_destroy( ctx );
}

static void test_multithread_determinism_and_speedup( void )
{
	GcStubWorld* w = build_bench_stub_world();
	const int n_mechs = 48;
	const int steps = 160;

	float* p1 = (float*)malloc( 32768 * sizeof( float ) );
	float* p4 = (float*)malloc( 32768 * sizeof( float ) );
	float* p8 = (float*)malloc( 32768 * sizeof( float ) );
	float* e1 = (float*)malloc( 200000 * sizeof( float ) );
	float* e4 = (float*)malloc( 200000 * sizeof( float ) );
	float* e8 = (float*)malloc( 200000 * sizeof( float ) );
	int np1 = 0, np4 = 0, np8 = 0, ne1 = 0, ne4 = 0, ne8 = 0;

	run_determinism_pass( w, 1, n_mechs, steps, p1, &np1, e1, &ne1 );
	run_determinism_pass( w, 4, n_mechs, steps, p4, &np4, e4, &ne4 );
	run_determinism_pass( w, 8, n_mechs, steps, p8, &np8, e8, &ne8 );

	EXPECT_TRUE( np1 > 0 && np1 == np4 && np1 == np8 );
	EXPECT_TRUE( ne1 > 0 && ne1 == ne4 && ne1 == ne8 );
	EXPECT_TRUE( memcmp( p1, p4, (size_t)np1 * sizeof( float ) ) == 0 );
	EXPECT_TRUE( memcmp( p1, p8, (size_t)np1 * sizeof( float ) ) == 0 );
	EXPECT_TRUE( memcmp( e1, e4, (size_t)ne1 * sizeof( float ) ) == 0 );
	EXPECT_TRUE( memcmp( e1, e8, (size_t)ne1 * sizeof( float ) ) == 0 );

	for ( int i = 0; i < np1; ++i )
		EXPECT_TRUE( isfinite( p1[i] ) );

	free( p1 ); free( p4 ); free( p8 );
	free( e1 ); free( e4 ); free( e8 );

	/* Speedup benchmark: 64 mechs x 240 steps, 1 thread vs hw threads */
	int hw = gc_hardware_threads();
	GcWorldCallbacks cb = gc_stub_world_callbacks( w );
	const int bm = 64, bs = 240;
	float* binp = (float*)calloc( (size_t)bm * GC_INPUT_STRIDE, sizeof( float ) );
	for ( int i = 0; i < bm; ++i )
	{
		binp[i * GC_INPUT_STRIDE + 0] = 1.0f;
		binp[i * GC_INPUT_STRIDE + 2] = 0.25f;
		binp[i * GC_INPUT_STRIDE + 3] = 1.0f;
	}
	float* bpose = (float*)malloc( 32768 * sizeof( float ) );

	double t1_ms = 0.0, tmt_ms = 0.0;
	for ( int pass = 0; pass < 2; ++pass )
	{
		int nt = pass == 0 ? 1 : hw;
		GcContext* ctx = gc_create( nt );
		gc_set_world( ctx, &cb );
		GcBenchConfig cfg = { (float)( 1.0 / 60.0 ), 0.0f, 20.0f, 1.0f, 1, 1 };
		gc_set_bench_config( ctx, &cfg );
		for ( int i = 0; i < bm; ++i )
			gc_spawn( ctx, i % gc_kind_count(), (float)( i % 8 ) * 6.0f - 21.0f, 0.0f, (float)( i / 8 ) * 6.0f - 21.0f, 0.0f );
		double t0 = gc_time_sec();
		for ( int s = 0; s < bs; ++s )
			gc_step( ctx, 1.0f / 60.0f, binp, bm );
		int bpn = gc_poses( ctx, bpose, 32768 );
		double elapsed = ( gc_time_sec() - t0 ) * 1000.0;
		EXPECT_TRUE( bpn > 0 );
		if ( pass == 0 ) t1_ms = elapsed; else tmt_ms = elapsed;
		gc_destroy( ctx );
	}
	printf( "[gaitcore_c bench] %d mechs x %d steps | 1T: %.2f ms (%.3f ms/step) | %dT: %.2f ms (%.3f ms/step) | speedup %.2fx\n",
			bm, bs, t1_ms, t1_ms / bs, hw, tmt_ms, tmt_ms / bs, t1_ms / ( tmt_ms > 1e-6 ? tmt_ms : 1.0 ) );

	free( binp );
	free( bpose );
	gc_stub_world_destroy( w );
}

static void test_custom_mech_api( void )
{
	GcStubWorld* w = gc_stub_world_create( 0.0, 1 );
	gc_stub_world_add_box( w, -4.0, 0.0, 6.0, 4.0, 0.35, 9.0 );
	GcWorldCallbacks cb = gc_stub_world_callbacks( w );

	GcContext* ctx = gc_create( 1 );
	gc_set_world( ctx, &cb );
	EXPECT_TRUE( gc_context_kind_count( ctx ) == gc_kind_count() );

	/* Validation rejects zeroed/invalid descriptors */
	GcMechDesc bad;
	memset( &bad, 0, sizeof( bad ) );
	EXPECT_TRUE( gc_register_mech_type( ctx, &bad ) == -1 );
	EXPECT_TRUE( gc_spawn( ctx, 999, 0.0f, 0.0f, 0.0f, 0.0f ) == -1 );

	/* Register default biped (2 legs), custom spider (4 legs), and preset-derived hexapod (6 legs) */
	GcMechDesc biped_desc, quad_desc, hex_desc;
	gc_mech_desc_init_default( &biped_desc, 2 );
	snprintf( biped_desc.name, sizeof( biped_desc.name ), "STRIDER_2" );
	int k_biped = gc_register_mech_type( ctx, &biped_desc );

	gc_mech_desc_init_default( &quad_desc, 4 );
	snprintf( quad_desc.name, sizeof( quad_desc.name ), "SCOUT_4" );
	quad_desc.speed_walk = 18.0f;
	quad_desc.speed_run = 28.0f;
	int k_quad = gc_register_mech_type( ctx, &quad_desc );

	EXPECT_TRUE( gc_mech_desc_from_preset( 6, &hex_desc ) == 1 );
	snprintf( hex_desc.name, sizeof( hex_desc.name ), "CARRIER_6" );
	int k_hex = gc_register_mech_type( ctx, &hex_desc );

	EXPECT_TRUE( k_biped == gc_kind_count() );
	EXPECT_TRUE( k_quad == gc_kind_count() + 1 );
	EXPECT_TRUE( k_hex == gc_kind_count() + 2 );
	EXPECT_TRUE( gc_context_kind_count( ctx ) == gc_kind_count() + 3 );
	EXPECT_TRUE( strcmp( gc_context_kind_name( ctx, k_biped ), "STRIDER_2" ) == 0 );
	EXPECT_TRUE( strcmp( gc_context_kind_name( ctx, k_quad ), "SCOUT_4" ) == 0 );
	EXPECT_TRUE( strcmp( gc_context_kind_name( ctx, k_hex ), "CARRIER_6" ) == 0 );

	int h0 = gc_spawn( ctx, k_biped, -5.0f, 0.0f, 0.0f, 0.0f );
	int h1 = gc_spawn( ctx, k_quad, 0.0f, 0.0f, 0.0f, 0.0f );
	int h2 = gc_spawn_custom( ctx, &hex_desc, 5.0f, 0.0f, 0.0f, 0.0f );
	int h3 = gc_spawn_custom( ctx, &hex_desc, 10.0f, 0.0f, 0.0f, 0.0f ); /* dedup check */
	EXPECT_TRUE( h0 == 0 && h1 == 1 && h2 == 2 && h3 == 3 );
	EXPECT_TRUE( gc_context_kind_count( ctx ) == gc_kind_count() + 3 );

	float inputs[4 * GC_INPUT_STRIDE];
	memset( inputs, 0, sizeof( inputs ) );
	for ( int i = 0; i < 4; ++i )
		inputs[i * GC_INPUT_STRIDE + GC_IN_FWD] = 1.0f;

	for ( int s = 0; s < 180; ++s )
		gc_step( ctx, 1.0f / 60.0f, inputs, 4 );

	float poses[1024];
	int total = gc_poses( ctx, poses, 1024 );
	EXPECT_TRUE( total > 0 );
	for ( int i = 0; i < total; ++i )
		EXPECT_TRUE( isfinite( poses[i] ) );

	int off = 0;
	for ( int i = 0; i < 4; ++i )
	{
		GcMechInfo info;
		EXPECT_TRUE( gc_info( ctx, i, &info ) == 1 );
		EXPECT_TRUE( poses[off + GC_POSE_POS + 2] > 6.0f );
		off += info.floats;
	}

	/* gc_reset retains custom archetypes; gc_clear_custom_types removes them */
	gc_reset( ctx );
	EXPECT_TRUE( gc_context_kind_count( ctx ) == gc_kind_count() + 3 );
	gc_clear_custom_types( ctx );
	EXPECT_TRUE( gc_context_kind_count( ctx ) == gc_kind_count() );

	gc_destroy( ctx );
	gc_stub_world_destroy( w );
}

int main( void )
{
	test_vmath_geometry();
	test_ik_solvers_and_limits();
	test_gaitsel_and_blend();
	test_stub_world_obox();
	test_all_mechs_walk_slide_jump();
	test_custom_mech_api();
	test_multithread_determinism_and_speedup();

	if ( g_failures == 0 )
	{
		printf( "gaitcore_c_tests: ALL PASSED\n" );
		return 0;
	}
	printf( "gaitcore_c_tests: %d FAILURES\n", g_failures );
	return 1;
}
