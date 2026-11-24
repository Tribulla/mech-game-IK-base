// Built-in 9-mech roster and custom GcMechDesc -> GcMechDef conversion/validation.
// Keep preset numeric fields aligned with IK/python/gaitcore/mechdefs.py.

#include "gc_internal.h"

// Appends a mirrored left/right leg pair in body-local coordinates (negative X left, positive X right).
static void add_pair( GcMechDef* d, double mx, double my, double mz, double rx, double rz )
{
	int k = d->n_legs;
	GcLegDef* l = &d->legs[k];
	memset( l, 0, sizeof( *l ) );
	l->mount[0] = mx; l->mount[1] = my; l->mount[2] = mz;
	l->rest[0] = rx; l->rest[1] = rz;
	l->side = -1;

	GcLegDef* r = &d->legs[k + 1];
	memset( r, 0, sizeof( *r ) );
	r->mount[0] = -mx; r->mount[1] = my; r->mount[2] = mz;
	r->rest[0] = -rx; r->rest[1] = rz;
	r->side = 1;

	d->n_legs = k + 2;
}

static GcMechDef g_defs[9];
static int g_defs_inited = 0;

static void init_defs( void )
{
	if ( g_defs_inited )
		return;
	memset( g_defs, 0, sizeof( g_defs ) );

	/* 0: LYNX */
	{
		GcMechDef* d = &g_defs[0];
		snprintf( d->name, sizeof( d->name ), "LYNX" );
		d->kind_id = 0;
		d->faction = 0;
		snprintf( d->blurb, sizeof( d->blurb ), "Light recon quadruped - agile spider legs, wire anchors" );
		d->plane = GC_PLANE_RADIAL;
		d->coxa = 0.25;
		d->n_segs = 2;
		d->lengths[0] = 1.4; d->lengths[1] = 1.6;
		d->sides[0] = 1;
		d->widths[0] = 0.22; d->widths[1] = 0.16;
		d->limits.has_coxa = 1; d->limits.coxa = 75.0;
		d->limits.has_hip = 1; d->limits.hip[0] = -30.0; d->limits.hip[1] = 75.0;
		d->limits.bend_count = 1; d->limits.bend[0][0] = 8.0; d->limits.bend[0][1] = 150.0;
		add_pair( d, -0.75, 0.0, 0.95, -2.0, 2.1 );
		add_pair( d, -0.75, 0.0, -0.95, -2.0, -2.0 );
		d->body_half[0] = 0.8; d->body_half[1] = 0.45; d->body_half[2] = 1.25;
		d->ride = 1.15;
		d->speed_slow = 10.5; d->speed_walk = 27.3; d->speed_run = 35.7;
		d->accel = 8.0; d->accel_run = 24.0; d->turn_rate = 3.4;
		gc_gaits_quadruped( d->gaits, &d->gait_count );
		d->gait_slow_idx = gc_gait_find( d->gaits, d->gait_count, "crawl" );
		d->gait_walk_idx = gc_gait_find( d->gaits, d->gait_count, "walk" );
		d->gait_run_idx = gc_gait_find( d->gaits, d->gait_count, "bound" );
		d->gait_walk_fast_idx = gc_gait_find( d->gaits, d->gait_count, "trot_run" );
		d->gait_drift_idx = gc_gait_find( d->gaits, d->gait_count, "trot" );
		d->step_height = 0.45; d->step_max = 0.7; d->mantle_max = 1.6; d->wire_max = 9.0;
		d->climber = 0; d->jump_speed = 14.1; d->max_tilt = 20.0; d->look_tilt = 25.0;
		d->mass = 1.0; d->biped = 0; d->bound = 0.45; d->bound_pitch = 8.0;
		d->zone = 1.0; d->swing_min = 0.14; d->swing_max = 0.42; d->size = 1.0;
	}

	/* 1: STRIKER */
	{
		GcMechDef* d = &g_defs[1];
		snprintf( d->name, sizeof( d->name ), "STRIKER" );
		d->kind_id = 1;
		d->faction = 0;
		snprintf( d->blurb, sizeof( d->blurb ), "Assault quadruped - 3-seg legs, high jump, wire anchors" );
		d->plane = GC_PLANE_RADIAL;
		d->coxa = 0.3;
		d->n_segs = 3;
		d->lengths[0] = 1.6; d->lengths[1] = 1.5; d->lengths[2] = 0.55;
		d->sides[0] = 1; d->sides[1] = 1;
		d->widths[0] = 0.26; d->widths[1] = 0.2; d->widths[2] = 0.18;
		d->limits.has_coxa = 1; d->limits.coxa = 75.0;
		d->limits.has_hip = 1; d->limits.hip[0] = -35.0; d->limits.hip[1] = 70.0;
		d->limits.bend_count = 2;
		d->limits.bend[0][0] = 5.0; d->limits.bend[0][1] = 145.0;
		d->limits.bend[1][0] = 15.0; d->limits.bend[1][1] = 150.0;
		add_pair( d, -0.85, 0.05, 1.1, -2.3, 2.5 );
		add_pair( d, -0.85, 0.05, -1.1, -2.3, -2.3 );
		d->body_half[0] = 0.9; d->body_half[1] = 0.5; d->body_half[2] = 1.45;
		d->ride = 1.45;
		d->speed_slow = 14.0; d->speed_walk = 36.4; d->speed_run = 47.6;
		d->accel = 12.0; d->accel_run = 36.0; d->turn_rate = 4.2;
		gc_gaits_quadruped( d->gaits, &d->gait_count );
		d->gait_slow_idx = gc_gait_find( d->gaits, d->gait_count, "crawl" );
		d->gait_walk_idx = gc_gait_find( d->gaits, d->gait_count, "walk" );
		d->gait_run_idx = gc_gait_find( d->gaits, d->gait_count, "bound" );
		d->gait_walk_fast_idx = gc_gait_find( d->gaits, d->gait_count, "trot_run" );
		d->gait_drift_idx = gc_gait_find( d->gaits, d->gait_count, "trot" );
		d->step_height = 0.55; d->step_max = 0.8; d->mantle_max = 2.2; d->wire_max = 10.0;
		d->climber = 0; d->jump_speed = 17.9; d->max_tilt = 22.0; d->look_tilt = 25.0;
		d->mass = 1.0; d->biped = 0; d->bound = 0.75; d->bound_pitch = 10.0;
		d->zone = 1.15; d->swing_min = 0.12; d->swing_max = 0.4; d->size = 1.0;
	}

	/* 2: STRIDER */
	{
		GcMechDef* d = &g_defs[2];
		snprintf( d->name, sizeof( d->name ), "STRIDER" );
		d->kind_id = 2;
		d->faction = 0;
		snprintf( d->blurb, sizeof( d->blurb ), "Combat biped - sagittal 2-leg strider with shoulder cannons" );
		d->plane = GC_PLANE_SAGITTAL;
		d->coxa = 0.0;
		d->n_segs = 2;
		d->lengths[0] = 1.35; d->lengths[1] = 1.35;
		d->sides[0] = 1;
		d->widths[0] = 0.26; d->widths[1] = 0.22;
		d->limits.has_roll = 1; d->limits.roll = 30.0;
		d->limits.has_hip = 1; d->limits.hip[0] = -75.0; d->limits.hip[1] = 105.0;
		d->limits.bend_count = 1; d->limits.bend[0][0] = 8.0; d->limits.bend[0][1] = 150.0;
		add_pair( d, -0.48, -0.25, 0.0, -0.48, 0.0 );
		d->body_half[0] = 0.55; d->body_half[1] = 0.55; d->body_half[2] = 0.55;
		d->ride = 1.95;
		d->speed_slow = 3.5; d->speed_walk = 6.5; d->speed_run = 13.5;
		d->accel = 9.0; d->accel_run = 20.0; d->turn_rate = 3.6;
		gc_gaits_biped( d->gaits, &d->gait_count );
		d->gait_slow_idx = gc_gait_find( d->gaits, d->gait_count, "stalk" );
		d->gait_walk_idx = gc_gait_find( d->gaits, d->gait_count, "walk" );
		d->gait_run_idx = gc_gait_find( d->gaits, d->gait_count, "run" );
		d->gait_walk_fast_idx = -1;
		d->gait_drift_idx = d->gait_walk_idx;
		d->step_height = 0.45; d->step_max = 0.75; d->mantle_max = 1.6; d->wire_max = 0.0;
		d->climber = 0; d->jump_speed = 12.5; d->max_tilt = 16.0; d->look_tilt = 20.0;
		d->mass = 1.0; d->biped = 1; d->bound = 0.22; d->bound_pitch = 3.0;
		d->zone = 0.9; d->swing_min = 0.12; d->swing_max = 0.36; d->size = 1.0;
	}

	/* 3: GECKO */
	{
		GcMechDef* d = &g_defs[3];
		snprintf( d->name, sizeof( d->name ), "GECKO" );
		d->kind_id = 3;
		d->faction = 0;
		snprintf( d->blurb, sizeof( d->blurb ), "Wall-climbing spider scout - surface-adhesion quadruped" );
		d->plane = GC_PLANE_RADIAL;
		d->coxa = 0.22;
		d->n_segs = 2;
		d->lengths[0] = 1.35; d->lengths[1] = 1.50;
		d->sides[0] = 1;
		d->widths[0] = 0.20; d->widths[1] = 0.14;
		d->limits.has_coxa = 1; d->limits.coxa = 75.0;
		d->limits.has_hip = 1; d->limits.hip[0] = -35.0; d->limits.hip[1] = 80.0;
		d->limits.bend_count = 1; d->limits.bend[0][0] = 8.0; d->limits.bend[0][1] = 155.0;
		add_pair( d, -0.68, -0.05, 0.85, -1.85, 1.85 );
		add_pair( d, -0.68, -0.05, -0.85, -1.85, -1.80 );
		d->body_half[0] = 0.72; d->body_half[1] = 0.36; d->body_half[2] = 1.05;
		d->ride = 1.00;
		d->speed_slow = 10.0; d->speed_walk = 24.0; d->speed_run = 32.0;
		d->accel = 10.0; d->accel_run = 28.0; d->turn_rate = 4.2;
		gc_gaits_quadruped( d->gaits, &d->gait_count );
		d->gait_slow_idx = gc_gait_find( d->gaits, d->gait_count, "crawl" );
		d->gait_walk_idx = gc_gait_find( d->gaits, d->gait_count, "walk" );
		d->gait_run_idx = gc_gait_find( d->gaits, d->gait_count, "bound" );
		d->gait_walk_fast_idx = gc_gait_find( d->gaits, d->gait_count, "trot_run" );
		d->gait_drift_idx = gc_gait_find( d->gaits, d->gait_count, "trot" );
		d->step_height = 0.45; d->step_max = 0.75; d->mantle_max = 1.6; d->wire_max = 0.0;
		d->climber = 1; d->jump_speed = 14.5; d->max_tilt = 28.0; d->look_tilt = 25.0;
		d->mass = 0.85; d->biped = 0; d->bound = 0.40; d->bound_pitch = 7.0;
		d->zone = 0.95; d->swing_min = 0.12; d->swing_max = 0.38; d->size = 1.0;
	}

	/* 4: BASTION */
	{
		GcMechDef* d = &g_defs[4];
		snprintf( d->name, sizeof( d->name ), "BASTION" );
		d->kind_id = 4;
		d->faction = 1;
		snprintf( d->blurb, sizeof( d->blurb ), "Armored sagittal octopod - heavy tank with under-hull legs" );
		d->plane = GC_PLANE_SAGITTAL;
		d->coxa = 0.0;
		d->n_segs = 2;
		d->lengths[0] = 1.0; d->lengths[1] = 1.1;
		d->sides[0] = -1;
		d->widths[0] = 0.34; d->widths[1] = 0.28;
		d->limits.has_roll = 1; d->limits.roll = 45.0;
		d->limits.has_hip = 1; d->limits.hip[0] = -85.0; d->limits.hip[1] = 30.0;
		d->limits.bend_count = 1; d->limits.bend[0][0] = 10.0; d->limits.bend[0][1] = 150.0;
		add_pair( d, -1.35, -0.45, 2.0, -1.75, 2.15 );
		add_pair( d, -1.35, -0.45, 0.7, -1.75, 0.75 );
		add_pair( d, -1.35, -0.45, -0.7, -1.75, -0.75 );
		add_pair( d, -1.35, -0.45, -2.0, -1.75, -2.05 );
		d->body_half[0] = 1.45; d->body_half[1] = 0.7; d->body_half[2] = 2.6;
		d->ride = 1.75;
		d->speed_slow = 9.8; d->speed_walk = 25.5; d->speed_run = 33.3;
		d->accel = 3.0; d->accel_run = 9.0; d->turn_rate = 1.8;
		gc_gaits_octopod( d->gaits, &d->gait_count );
		d->gait_slow_idx = gc_gait_find( d->gaits, d->gait_count, "wave" );
		d->gait_walk_idx = gc_gait_find( d->gaits, d->gait_count, "metachronal" );
		d->gait_run_idx = gc_gait_find( d->gaits, d->gait_count, "bound" );
		d->gait_walk_fast_idx = gc_gait_find( d->gaits, d->gait_count, "tetrapod_run" );
		d->gait_drift_idx = gc_gait_find( d->gaits, d->gait_count, "tetrapod" );
		d->step_height = 0.4; d->step_max = 0.65; d->mantle_max = 1.4; d->wire_max = 0.0;
		d->climber = 0; d->jump_speed = 10.0; d->max_tilt = 14.0; d->look_tilt = 25.0;
		d->mass = 4.0; d->biped = 0; d->bound = 0.30; d->bound_pitch = 4.0;
		d->zone = 0.8; d->swing_min = 0.16; d->swing_max = 0.45; d->size = 1.0;
	}

	/* 5: WARDEN */
	{
		GcMechDef* d = &g_defs[5];
		snprintf( d->name, sizeof( d->name ), "WARDEN" );
		d->kind_id = 5;
		d->faction = 1;
		snprintf( d->blurb, sizeof( d->blurb ), "Artillery octopod - 3-segment radial legs, heavy turret" );
		d->plane = GC_PLANE_RADIAL;
		d->coxa = 0.3;
		d->n_segs = 3;
		d->lengths[0] = 1.8; d->lengths[1] = 1.7; d->lengths[2] = 0.6;
		d->sides[0] = 1; d->sides[1] = 1;
		d->widths[0] = 0.3; d->widths[1] = 0.24; d->widths[2] = 0.16;
		d->limits.has_coxa = 1; d->limits.coxa = 70.0;
		d->limits.has_hip = 1; d->limits.hip[0] = -20.0; d->limits.hip[1] = 65.0;
		d->limits.bend_count = 2;
		d->limits.bend[0][0] = 35.0; d->limits.bend[0][1] = 150.0;
		d->limits.bend[1][0] = 15.0; d->limits.bend[1][1] = 145.0;
		add_pair( d, -1.2, 0.0, 1.5, -2.7, 2.6 );
		add_pair( d, -1.2, 0.0, 0.5, -3.0, 0.9 );
		add_pair( d, -1.2, 0.0, -0.5, -3.0, -0.9 );
		add_pair( d, -1.2, 0.0, -1.5, -2.7, -2.5 );
		d->body_half[0] = 1.3; d->body_half[1] = 0.65; d->body_half[2] = 2.2;
		d->ride = 1.4;
		d->speed_slow = 9.8; d->speed_walk = 25.5; d->speed_run = 33.3;
		d->accel = 4.5; d->accel_run = 13.5; d->turn_rate = 2.0;
		gc_gaits_octopod( d->gaits, &d->gait_count );
		d->gait_slow_idx = gc_gait_find( d->gaits, d->gait_count, "wave" );
		d->gait_walk_idx = gc_gait_find( d->gaits, d->gait_count, "metachronal" );
		d->gait_run_idx = gc_gait_find( d->gaits, d->gait_count, "bound" );
		d->gait_walk_fast_idx = gc_gait_find( d->gaits, d->gait_count, "tetrapod_run" );
		d->gait_drift_idx = gc_gait_find( d->gaits, d->gait_count, "tetrapod" );
		d->step_height = 0.45; d->step_max = 0.75; d->mantle_max = 1.8; d->wire_max = 0.0;
		d->climber = 0; d->jump_speed = 11.0; d->max_tilt = 18.0; d->look_tilt = 25.0;
		d->mass = 4.0; d->biped = 0; d->bound = 0.35; d->bound_pitch = 5.0;
		d->zone = 0.95; d->swing_min = 0.14; d->swing_max = 0.45; d->size = 1.0;
	}

	/* 6: MANTIS */
	{
		GcMechDef* d = &g_defs[6];
		snprintf( d->name, sizeof( d->name ), "MANTIS" );
		d->kind_id = 6;
		d->faction = 1;
		snprintf( d->blurb, sizeof( d->blurb ), "Hunter hexapod - agile 3-seg flanker with foreleg blades" );
		d->plane = GC_PLANE_RADIAL;
		d->coxa = 0.2;
		d->n_segs = 3;
		d->lengths[0] = 1.0; d->lengths[1] = 1.8; d->lengths[2] = 0.7;
		d->sides[0] = 1; d->sides[1] = 1;
		d->widths[0] = 0.18; d->widths[1] = 0.2; d->widths[2] = 0.12;
		d->limits.has_coxa = 1; d->limits.coxa = 75.0;
		d->limits.has_hip = 1; d->limits.hip[0] = -25.0; d->limits.hip[1] = 100.0;
		d->limits.bend_count = 2;
		d->limits.bend[0][0] = 10.0; d->limits.bend[0][1] = 160.0;
		d->limits.bend[1][0] = 10.0; d->limits.bend[1][1] = 155.0;
		add_pair( d, -0.55, -0.35, 0.8, -1.9, 2.0 );
		add_pair( d, -0.55, -0.35, 0.0, -2.1, 0.1 );
		add_pair( d, -0.55, -0.35, -0.8, -1.9, -1.8 );
		d->body_half[0] = 0.6; d->body_half[1] = 0.45; d->body_half[2] = 1.5;
		d->ride = 1.45;
		d->speed_slow = 14.0; d->speed_walk = 36.4; d->speed_run = 47.6;
		d->accel = 12.0; d->accel_run = 36.0; d->turn_rate = 4.4;
		gc_gaits_hexapod( d->gaits, &d->gait_count );
		d->gait_slow_idx = gc_gait_find( d->gaits, d->gait_count, "wave" );
		d->gait_walk_idx = gc_gait_find( d->gaits, d->gait_count, "ripple" );
		d->gait_run_idx = gc_gait_find( d->gaits, d->gait_count, "bound" );
		d->gait_walk_fast_idx = gc_gait_find( d->gaits, d->gait_count, "tripod_run" );
		d->gait_drift_idx = gc_gait_find( d->gaits, d->gait_count, "tripod" );
		d->step_height = 0.5; d->step_max = 0.75; d->mantle_max = 1.8; d->wire_max = 0.0;
		d->climber = 0; d->jump_speed = 16.7; d->max_tilt = 24.0; d->look_tilt = 25.0;
		d->mass = 1.0; d->biped = 0; d->bound = 0.55; d->bound_pitch = 9.0;
		d->zone = 1.0; d->swing_min = 0.11; d->swing_max = 0.38; d->size = 1.0;
	}

	/* 7: SCARAB */
	{
		GcMechDef* d = &g_defs[7];
		snprintf( d->name, sizeof( d->name ), "SCARAB" );
		d->kind_id = 7;
		d->faction = 1;
		snprintf( d->blurb, sizeof( d->blurb ), "Utility scout hexapod - compact frame, sensor head" );
		d->plane = GC_PLANE_RADIAL;
		d->coxa = 0.2;
		d->n_segs = 2;
		d->lengths[0] = 1.2; d->lengths[1] = 1.45;
		d->sides[0] = 1;
		d->widths[0] = 0.17; d->widths[1] = 0.12;
		d->limits.has_coxa = 1; d->limits.coxa = 75.0;
		d->limits.has_hip = 1; d->limits.hip[0] = -25.0; d->limits.hip[1] = 75.0;
		d->limits.bend_count = 1; d->limits.bend[0][0] = 40.0; d->limits.bend[0][1] = 165.0;
		add_pair( d, -0.5, -0.1, 0.6, -1.55, 1.5 );
		add_pair( d, -0.5, -0.1, 0.0, -1.75, 0.0 );
		add_pair( d, -0.5, -0.1, -0.6, -1.55, -1.4 );
		d->body_half[0] = 0.55; d->body_half[1] = 0.38; d->body_half[2] = 0.95;
		d->ride = 0.95;
		d->speed_slow = 11.2; d->speed_walk = 29.1; d->speed_run = 38.1;
		d->accel = 10.0; d->accel_run = 30.0; d->turn_rate = 4.0;
		gc_gaits_hexapod( d->gaits, &d->gait_count );
		d->gait_slow_idx = gc_gait_find( d->gaits, d->gait_count, "wave" );
		d->gait_walk_idx = gc_gait_find( d->gaits, d->gait_count, "ripple" );
		d->gait_run_idx = gc_gait_find( d->gaits, d->gait_count, "bound" );
		d->gait_walk_fast_idx = gc_gait_find( d->gaits, d->gait_count, "tripod_run" );
		d->gait_drift_idx = gc_gait_find( d->gaits, d->gait_count, "tripod" );
		d->step_height = 0.36; d->step_max = 0.6; d->mantle_max = 1.2; d->wire_max = 0.0;
		d->climber = 0; d->jump_speed = 13.4; d->max_tilt = 26.0; d->look_tilt = 25.0;
		d->mass = 0.9; d->biped = 0; d->bound = 0.30; d->bound_pitch = 7.0;
		d->zone = 0.75; d->swing_min = 0.11; d->swing_max = 0.38; d->size = 1.0;
	}

	/* 8: COLOSSUS */
	{
		GcMechDef* d = &g_defs[8];
		snprintf( d->name, sizeof( d->name ), "COLOSSUS" );
		d->kind_id = 8;
		d->faction = 1;
		snprintf( d->blurb, sizeof( d->blurb ), "Siege octopod - super-heavy walker with dual cannons" );
		d->plane = GC_PLANE_RADIAL;
		d->coxa = 0.45;
		d->n_segs = 3;
		d->lengths[0] = 2.6; d->lengths[1] = 2.6; d->lengths[2] = 0.9;
		d->sides[0] = 1; d->sides[1] = 1;
		d->widths[0] = 0.45; d->widths[1] = 0.36; d->widths[2] = 0.24;
		d->limits.has_coxa = 1; d->limits.coxa = 65.0;
		d->limits.has_hip = 1; d->limits.hip[0] = -15.0; d->limits.hip[1] = 55.0;
		d->limits.bend_count = 2;
		d->limits.bend[0][0] = 40.0; d->limits.bend[0][1] = 150.0;
		d->limits.bend[1][0] = 15.0; d->limits.bend[1][1] = 145.0;
		add_pair( d, -1.85, -0.1, 2.3, -4.0, 3.7 );
		add_pair( d, -1.85, -0.1, 0.8, -4.4, 1.3 );
		add_pair( d, -1.85, -0.1, -0.8, -4.4, -1.3 );
		add_pair( d, -1.85, -0.1, -2.3, -4.0, -3.5 );
		d->body_half[0] = 2.0; d->body_half[1] = 1.0; d->body_half[2] = 3.2;
		d->ride = 2.3;
		d->speed_slow = 7.0; d->speed_walk = 18.2; d->speed_run = 23.8;
		d->accel = 2.5; d->accel_run = 7.5; d->turn_rate = 1.2;
		gc_gaits_octopod( d->gaits, &d->gait_count );
		d->gait_slow_idx = gc_gait_find( d->gaits, d->gait_count, "wave" );
		d->gait_walk_idx = gc_gait_find( d->gaits, d->gait_count, "metachronal" );
		d->gait_run_idx = gc_gait_find( d->gaits, d->gait_count, "bound" );
		d->gait_walk_fast_idx = gc_gait_find( d->gaits, d->gait_count, "tetrapod_run" );
		d->gait_drift_idx = gc_gait_find( d->gaits, d->gait_count, "tetrapod" );
		d->step_height = 0.7; d->step_max = 1.2; d->mantle_max = 2.6; d->wire_max = 0.0;
		d->climber = 0; d->jump_speed = 8.9; d->max_tilt = 12.0; d->look_tilt = 25.0;
		d->mass = 6.0; d->biped = 0; d->bound = 0.30; d->bound_pitch = 4.0;
		d->zone = 1.5; d->swing_min = 0.22; d->swing_max = 0.65; d->size = 1.6;
	}

	g_defs_inited = 1;
}

int gc_mechdef_count( void )
{
	return 9;
}

const GcMechDef* gc_mechdef_by_kind( int kind )
{
	init_defs();
	int k = kind % 9;
	if ( k < 0 )
		k += 9;
	return &g_defs[k];
}

int gc_kind_count( void )
{
	return gc_mechdef_count();
}

const char* gc_kind_name( int kind )
{
	return gc_mechdef_by_kind( kind )->name;
}

// Converts a validated host GcMechDesc into an internal GcMechDef and binds the gait table
// matching its leg count (2, 4, 6, or 8). Missing optional tuning values fall back to safe defaults.
int gc_mechdef_from_desc( const GcMechDesc* desc, int kind_id, GcMechDef* out )
{
	if ( !desc || !out )
		return 0;
	if ( desc->num_legs != 2 && desc->num_legs != 4 && desc->num_legs != 6 && desc->num_legs != 8 )
		return 0;
	if ( desc->num_segments < 2 || desc->num_segments > GC_MAX_SEGS )
		return 0;
	if ( !isfinite( (double)desc->coxa ) || desc->coxa < 0.0f )
		return 0;
	if ( !isfinite( (double)desc->ride_height ) || desc->ride_height <= 0.05f )
		return 0;
	for ( int k = 0; k < 3; ++k )
	{
		if ( !isfinite( (double)desc->body_half[k] ) || desc->body_half[k] <= 0.05f )
			return 0;
	}
	for ( int s = 0; s < desc->num_segments; ++s )
	{
		if ( !isfinite( (double)desc->segment_lengths[s] ) || desc->segment_lengths[s] <= 0.01f )
			return 0;
	}
	for ( int i = 0; i < desc->num_legs; ++i )
	{
		for ( int k = 0; k < 3; ++k )
		{
			if ( !isfinite( (double)desc->legs[i].mount[k] ) )
				return 0;
		}
		for ( int k = 0; k < 2; ++k )
		{
			if ( !isfinite( (double)desc->legs[i].rest_xz[k] ) )
				return 0;
		}
	}

	memset( out, 0, sizeof( *out ) );
	snprintf( out->name, sizeof( out->name ), "%s", desc->name[0] ? desc->name : "CUSTOM" );
	snprintf( out->blurb, sizeof( out->blurb ), "%s", desc->blurb );
	out->kind_id = kind_id;
	out->faction = desc->faction;
	out->plane = ( desc->plane_mode == GC_LEG_PLANE_SAGITTAL ) ? GC_PLANE_SAGITTAL : GC_PLANE_RADIAL;
	out->coxa = (double)desc->coxa;
	out->n_segs = desc->num_segments;
	for ( int s = 0; s < out->n_segs; ++s )
	{
		out->lengths[s] = (double)desc->segment_lengths[s];
		out->widths[s] = desc->segment_widths[s] > 0.0f ? (double)desc->segment_widths[s] : 0.20;
	}
	for ( int s = 0; s < out->n_segs - 1; ++s )
		out->sides[s] = desc->bend_sides[s] < 0 ? -1 : 1;

	out->limits.has_coxa = desc->limits.has_coxa ? 1 : 0;
	out->limits.coxa = (double)desc->limits.coxa_deg;
	out->limits.has_roll = desc->limits.has_roll ? 1 : 0;
	out->limits.roll = (double)desc->limits.roll_deg;
	out->limits.has_hip = desc->limits.has_hip ? 1 : 0;
	out->limits.hip[0] = (double)desc->limits.hip_deg[0];
	out->limits.hip[1] = (double)desc->limits.hip_deg[1];
	out->limits.bend_count = desc->limits.bend_count < 0 ? 0
							 : ( desc->limits.bend_count > ( GC_MAX_SEGS - 1 ) ? ( GC_MAX_SEGS - 1 ) : desc->limits.bend_count );
	for ( int s = 0; s < out->limits.bend_count; ++s )
	{
		out->limits.bend[s][0] = (double)desc->limits.bend_deg[s][0];
		out->limits.bend[s][1] = (double)desc->limits.bend_deg[s][1];
	}

	out->n_legs = desc->num_legs;
	for ( int i = 0; i < out->n_legs; ++i )
	{
		GcLegDef* dst = &out->legs[i];
		const GcLegDesc* src = &desc->legs[i];
		dst->mount[0] = (double)src->mount[0];
		dst->mount[1] = (double)src->mount[1];
		dst->mount[2] = (double)src->mount[2];
		dst->rest[0] = (double)src->rest_xz[0];
		dst->rest[1] = (double)src->rest_xz[1];
		dst->side = src->side != 0 ? ( src->side < 0 ? -1 : 1 )
								   : ( ( src->mount[0] < 0.0f || ( src->mount[0] == 0.0f && src->rest_xz[0] < 0.0f ) ) ? -1 : 1 );
	}

	for ( int k = 0; k < 3; ++k )
		out->body_half[k] = (double)desc->body_half[k];
	out->ride = (double)desc->ride_height;
	out->speed_slow = desc->speed_slow > 0.0f ? (double)desc->speed_slow : 8.0;
	out->speed_walk = desc->speed_walk > 0.0f ? (double)desc->speed_walk : 20.0;
	out->speed_run = desc->speed_run > 0.0f ? (double)desc->speed_run : 30.0;
	out->accel = desc->accel > 0.0f ? (double)desc->accel : 8.0;
	out->accel_run = desc->accel_run > 0.0f ? (double)desc->accel_run : 24.0;
	out->turn_rate = desc->turn_rate > 0.0f ? (double)desc->turn_rate : 3.2;
	out->step_height = desc->step_height > 0.0f ? (double)desc->step_height : 0.45;
	out->step_max = desc->step_max > 0.0f ? (double)desc->step_max : 0.7;
	out->mantle_max = desc->mantle_max > 0.0f ? (double)desc->mantle_max : 1.6;
	out->wire_max = desc->wire_max > 0.0f ? (double)desc->wire_max : 0.0;
	out->climber = desc->climber ? 1 : 0;
	out->jump_speed = desc->jump_speed > 0.0f ? (double)desc->jump_speed : 12.0;
	out->max_tilt = desc->max_tilt_deg > 0.0f ? (double)desc->max_tilt_deg : 18.0;
	out->look_tilt = desc->look_tilt_deg > 0.0f ? (double)desc->look_tilt_deg : 25.0;
	out->mass = desc->mass > 0.0f ? (double)desc->mass : 1.0;
	out->bound = desc->bound > 0.0f ? (double)desc->bound : 0.0;
	out->bound_pitch = (double)desc->bound_pitch_deg;
	out->zone = desc->zone > 0.0f ? (double)desc->zone : 1.0;
	out->swing_min = desc->swing_min > 0.0f ? (double)desc->swing_min : 0.12;
	out->swing_max = desc->swing_max > out->swing_min ? (double)desc->swing_max : ( out->swing_min + 0.25 );
	out->size = desc->size > 0.0f ? (double)desc->size : 1.0;

	if ( out->n_legs == 2 )
	{
		out->biped = 1;
		gc_gaits_biped( out->gaits, &out->gait_count );
		out->gait_slow_idx = gc_gait_find( out->gaits, out->gait_count, "stalk" );
		out->gait_walk_idx = gc_gait_find( out->gaits, out->gait_count, "walk" );
		out->gait_run_idx = gc_gait_find( out->gaits, out->gait_count, "run" );
		out->gait_walk_fast_idx = -1;
		out->gait_drift_idx = out->gait_walk_idx;
	}
	else if ( out->n_legs == 4 )
	{
		out->biped = 0;
		gc_gaits_quadruped( out->gaits, &out->gait_count );
		out->gait_slow_idx = gc_gait_find( out->gaits, out->gait_count, "crawl" );
		out->gait_walk_idx = gc_gait_find( out->gaits, out->gait_count, "walk" );
		out->gait_run_idx = gc_gait_find( out->gaits, out->gait_count, "bound" );
		out->gait_walk_fast_idx = gc_gait_find( out->gaits, out->gait_count, "trot_run" );
		out->gait_drift_idx = gc_gait_find( out->gaits, out->gait_count, "trot" );
	}
	else if ( out->n_legs == 6 )
	{
		out->biped = 0;
		gc_gaits_hexapod( out->gaits, &out->gait_count );
		out->gait_slow_idx = gc_gait_find( out->gaits, out->gait_count, "wave" );
		out->gait_walk_idx = gc_gait_find( out->gaits, out->gait_count, "ripple" );
		out->gait_run_idx = gc_gait_find( out->gaits, out->gait_count, "bound" );
		out->gait_walk_fast_idx = gc_gait_find( out->gaits, out->gait_count, "tripod_run" );
		out->gait_drift_idx = gc_gait_find( out->gaits, out->gait_count, "tripod" );
	}
	else
	{
		out->biped = 0;
		gc_gaits_octopod( out->gaits, &out->gait_count );
		out->gait_slow_idx = gc_gait_find( out->gaits, out->gait_count, "wave" );
		out->gait_walk_idx = gc_gait_find( out->gaits, out->gait_count, "metachronal" );
		out->gait_run_idx = gc_gait_find( out->gaits, out->gait_count, "bound" );
		out->gait_walk_fast_idx = gc_gait_find( out->gaits, out->gait_count, "tetrapod_run" );
		out->gait_drift_idx = gc_gait_find( out->gaits, out->gait_count, "tetrapod" );
	}
	return 1;
}

int gc_mech_desc_from_preset( int preset_kind, GcMechDesc* out_desc )
{
	if ( !out_desc || preset_kind < 0 || preset_kind >= gc_mechdef_count() )
		return 0;
	const GcMechDef* d = gc_mechdef_by_kind( preset_kind );
	memset( out_desc, 0, sizeof( *out_desc ) );
	snprintf( out_desc->name, sizeof( out_desc->name ), "%s", d->name );
	snprintf( out_desc->blurb, sizeof( out_desc->blurb ), "%s", d->blurb );
	out_desc->faction = d->faction;
	out_desc->plane_mode = d->plane == GC_PLANE_SAGITTAL ? GC_LEG_PLANE_SAGITTAL : GC_LEG_PLANE_RADIAL;
	out_desc->coxa = (float)d->coxa;
	out_desc->num_segments = d->n_segs;
	for ( int s = 0; s < d->n_segs && s < GC_MAX_SEGS_PUBLIC; ++s )
	{
		out_desc->segment_lengths[s] = (float)d->lengths[s];
		out_desc->segment_widths[s] = (float)d->widths[s];
	}
	for ( int s = 0; s < d->n_segs - 1 && s < GC_MAX_SEGS_PUBLIC - 1; ++s )
		out_desc->bend_sides[s] = d->sides[s];

	out_desc->limits.has_coxa = d->limits.has_coxa;
	out_desc->limits.coxa_deg = (float)d->limits.coxa;
	out_desc->limits.has_roll = d->limits.has_roll;
	out_desc->limits.roll_deg = (float)d->limits.roll;
	out_desc->limits.has_hip = d->limits.has_hip;
	out_desc->limits.hip_deg[0] = (float)d->limits.hip[0];
	out_desc->limits.hip_deg[1] = (float)d->limits.hip[1];
	out_desc->limits.bend_count = d->limits.bend_count;
	for ( int s = 0; s < d->limits.bend_count && s < GC_MAX_SEGS_PUBLIC - 1; ++s )
	{
		out_desc->limits.bend_deg[s][0] = (float)d->limits.bend[s][0];
		out_desc->limits.bend_deg[s][1] = (float)d->limits.bend[s][1];
	}

	out_desc->num_legs = d->n_legs;
	for ( int i = 0; i < d->n_legs && i < GC_MAX_LEGS_PUBLIC; ++i )
	{
		out_desc->legs[i].mount[0] = (float)d->legs[i].mount[0];
		out_desc->legs[i].mount[1] = (float)d->legs[i].mount[1];
		out_desc->legs[i].mount[2] = (float)d->legs[i].mount[2];
		out_desc->legs[i].rest_xz[0] = (float)d->legs[i].rest[0];
		out_desc->legs[i].rest_xz[1] = (float)d->legs[i].rest[1];
		out_desc->legs[i].side = d->legs[i].side;
	}

	for ( int k = 0; k < 3; ++k )
		out_desc->body_half[k] = (float)d->body_half[k];
	out_desc->ride_height = (float)d->ride;
	out_desc->speed_slow = (float)d->speed_slow;
	out_desc->speed_walk = (float)d->speed_walk;
	out_desc->speed_run = (float)d->speed_run;
	out_desc->accel = (float)d->accel;
	out_desc->accel_run = (float)d->accel_run;
	out_desc->turn_rate = (float)d->turn_rate;
	out_desc->step_height = (float)d->step_height;
	out_desc->step_max = (float)d->step_max;
	out_desc->mantle_max = (float)d->mantle_max;
	out_desc->wire_max = (float)d->wire_max;
	out_desc->climber = d->climber;
	out_desc->jump_speed = (float)d->jump_speed;
	out_desc->max_tilt_deg = (float)d->max_tilt;
	out_desc->look_tilt_deg = (float)d->look_tilt;
	out_desc->mass = (float)d->mass;
	out_desc->bound = (float)d->bound;
	out_desc->bound_pitch_deg = (float)d->bound_pitch;
	out_desc->zone = (float)d->zone;
	out_desc->swing_min = (float)d->swing_min;
	out_desc->swing_max = (float)d->swing_max;
	out_desc->size = (float)d->size;
	return 1;
}

void gc_mech_desc_init_default( GcMechDesc* out_desc, int num_legs )
{
	if ( !out_desc )
		return;
	if ( num_legs <= 2 )
	{
		gc_mech_desc_from_preset( 2, out_desc );
		snprintf( out_desc->name, sizeof( out_desc->name ), "BIPED" );
		snprintf( out_desc->blurb, sizeof( out_desc->blurb ), "Default 2-legged sagittal walker" );
	}
	else if ( num_legs <= 4 )
	{
		gc_mech_desc_from_preset( 0, out_desc );
		snprintf( out_desc->name, sizeof( out_desc->name ), "QUADRUPED" );
	}
	else if ( num_legs <= 6 )
	{
		gc_mech_desc_from_preset( 7, out_desc );
		snprintf( out_desc->name, sizeof( out_desc->name ), "HEXAPOD" );
	}
	else
	{
		gc_mech_desc_from_preset( 5, out_desc );
		snprintf( out_desc->name, sizeof( out_desc->name ), "OCTOPOD" );
	}
}
