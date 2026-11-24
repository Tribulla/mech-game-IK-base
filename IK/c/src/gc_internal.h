// Internal types, constants, and module boundaries for the C11 gaitcore engine.
// Keep shared simulation defaults aligned with IK/python/gaitcore/.

#ifndef GAITCORE_INTERNAL_H
#define GAITCORE_INTERNAL_H

#include "gaitcore.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GC_PI 3.14159265358979323846
#define GC_TAU 6.28318530717958647692
#define GC_DEG2RAD ( GC_PI / 180.0 )
#define GC_RAD2DEG ( 180.0 / GC_PI )

#define GC_MAX_LEGS 8
#define GC_MAX_SEGS 4
#define GC_MAX_JOINTS 6
#define GC_MAX_GAITS 8
#define GC_MAX_GROUPS 4
#define GC_MAX_GROUP_SIZE 4
#define GC_MAX_COL_OFFS 8
#define GC_MAX_HULL_PTS 16

// Locomotion tuning constants (m, s, rad) shared with mech_const.py and gaitsel.py.
// Note that GC_G (20 m/s^2) is tuned snappier than Earth gravity so heavy multi-ton walkers
// don't feel floaty during jumps and bounding strides.
#define GC_G 20.0
#define GC_A_LAT 135.0
#define GC_AIM_MAX 1.53
#define GC_TAKEOFF_RAMP_REACH 0.36
#define GC_LANDING_RAMP_REACH 0.68
#define GC_MAX_SUBSTEP_DEFAULT ( 1.0 / 60.0 )
#define GC_MAX_FRAME_DT 0.1

// Terrain harshness and slope cutoffs used to cap the gait ladder.
#define GC_H_GALLOP 0.15
#define GC_H_FAST 0.40
#define GC_H_WALK 0.70
#define GC_SLOPE_FAST 20.0
#define GC_SLOPE_WALK 38.0
#define GC_RELAX_T 1.15
#define GC_RELAX_T_MAX 1.8

enum {
	GC_PLANE_RADIAL = 0,
	GC_PLANE_SAGITTAL = 1
};

enum {
	GC_MODE_SLOW = 0,
	GC_MODE_WALK = 1,
	GC_MODE_RUN = 2
};

typedef struct GcV3 {
	double x, y, z;
} GcV3;

typedef struct GcV2 {
	double x, z;
} GcV2;

// Orthonormal body basis.
// Because right = fwd x up, positive body-local X points toward the left side of a -Z viewer
// and body-local X signs match GcLegDesc symmetrically.
typedef struct GcFrame {
	GcV3 fwd;
	GcV3 up;
	GcV3 right;
	int is_local;
} GcFrame;

extern const GcV3 GC_WORLD_UP;
extern const GcFrame GC_LOCAL_FRAME;

// Vector and scalar helpers (gc_vmath.c)
static inline GcV3 gc_v3( double x, double y, double z )
{
	GcV3 v = { x, y, z };
	return v;
}

static inline GcV3 gc_v3_zero( void )
{
	GcV3 v = { 0.0, 0.0, 0.0 };
	return v;
}

static inline GcV3 gc_v3_add( GcV3 a, GcV3 b )
{
	return gc_v3( a.x + b.x, a.y + b.y, a.z + b.z );
}

static inline GcV3 gc_v3_sub( GcV3 a, GcV3 b )
{
	return gc_v3( a.x - b.x, a.y - b.y, a.z - b.z );
}

static inline GcV3 gc_v3_mul( GcV3 a, double k )
{
	return gc_v3( a.x * k, a.y * k, a.z * k );
}

static inline GcV3 gc_v3_neg( GcV3 a )
{
	return gc_v3( -a.x, -a.y, -a.z );
}

static inline double gc_v3_dot( GcV3 a, GcV3 b )
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline GcV3 gc_v3_cross( GcV3 a, GcV3 b )
{
	return gc_v3( a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x );
}

static inline double gc_v3_len_sq( GcV3 a )
{
	return a.x * a.x + a.y * a.y + a.z * a.z;
}

static inline double gc_v3_len( GcV3 a )
{
	return sqrt( a.x * a.x + a.y * a.y + a.z * a.z );
}

static inline double gc_v3_dist( GcV3 a, GcV3 b )
{
	double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
	return sqrt( dx * dx + dy * dy + dz * dz );
}

static inline GcV3 gc_v3_lerp( GcV3 a, GcV3 b, double t )
{
	return gc_v3( a.x + ( b.x - a.x ) * t, a.y + ( b.y - a.y ) * t, a.z + ( b.z - a.z ) * t );
}

static inline double gc_clamp( double x, double lo, double hi )
{
	return x < lo ? lo : ( x > hi ? hi : x );
}

static inline double gc_smoothstep( double t )
{
	t = gc_clamp( t, 0.0, 1.0 );
	return t * t * ( 3.0 - 2.0 * t );
}

static inline double gc_damp( double rate, double dt )
{
	return 1.0 - exp( -rate * dt );
}

static inline double gc_mod1( double x )
{
	double r = fmod( x, 1.0 );
	if ( r < 0.0 )
		r += 1.0;
	if ( r >= 1.0 )
		r = 0.0;
	return r;
}

static inline double gc_wrap_pi( double a )
{
	double r = fmod( a + GC_PI, GC_TAU );
	if ( r < 0.0 )
		r += GC_TAU;
	return r - GC_PI;
}

GcV3 gc_v3_norm( GcV3 v, GcV3 fallback );
GcV3 gc_project_plane( GcV3 v, GcV3 n );
GcV3 gc_rotate_axis( GcV3 v, GcV3 axis, double angle );
GcV3 gc_turn_displacement( GcV3 travel, GcV3 up, double angle );
double gc_angle_between( GcV3 a, GcV3 b );
GcV3 gc_rotate_towards( GcV3 v, GcV3 target, double max_angle );

void gc_frame_init( GcFrame* fr, GcV3 fwd, GcV3 up );
void gc_frame_set( GcFrame* fr, GcV3 fwd, GcV3 up );
GcV3 gc_frame_to_world( const GcFrame* fr, GcV3 l );
GcV3 gc_frame_to_local( const GcFrame* fr, GcV3 w );
void gc_frame_rotate( GcFrame* fr, GcV3 axis, double angle );
void gc_frame_yaw( GcFrame* fr, double angle );

int gc_convex_hull_2d( const GcV2* pts, int n, GcV2* out_hull );
double gc_support_margin( const GcV2* hull, int n, GcV2 p );
GcV2 gc_nearest_on_hull( const GcV2* hull, int n, GcV2 p );
double gc_dist_to_segment( GcV3 p, GcV3 a, GcV3 b, GcV3 up );
int gc_fit_plane( const GcV3* points, const double* weights, int n, GcV3* out_c, GcV3* out_n );

/* --- world queries (gc_world.c) --- */
typedef struct GcHit {
	double t;
	GcV3 point;
	GcV3 normal;
	int surface;
} GcHit;

// Per-mech query context so worker threads count queries and record debug rays lock-free.
typedef struct GcWorldQueryCtx {
	const GcWorldCallbacks* cb;
	int record;
	int profile_queries;
	int ray_count;
	int push_count;
	int height_count;
	double ray_ms;
	double push_ms;
	double height_ms;
	float* rays;
	int ray_floats;
	int ray_cap_floats;
} GcWorldQueryCtx;

void gc_wq_init( GcWorldQueryCtx* q, const GcWorldCallbacks* cb );
void gc_wq_destroy( GcWorldQueryCtx* q );
void gc_wq_clear_rays( GcWorldQueryCtx* q );
int gc_world_raycast( GcWorldQueryCtx* q, GcV3 origin, GcV3 dir, double max_dist, GcHit* out_hit );
int gc_world_sphere_push( GcWorldQueryCtx* q, GcV3 center, double radius, GcV3* out_center, GcV3* out_normal );

// Joint limits converted into radians inside gc_legik_init for fast solving.
typedef struct GcLimits {
	int has_coxa;
	double coxa;
	int has_roll;
	double roll;
	int has_hip;
	double hip[2];
	int bend_count;
	double bend[GC_MAX_SEGS - 1][2];
} GcLimits;

// Planar IK solver for a single leg chain; holds warm-start state across steps.
typedef struct GcLegIK {
	int mode;
	double coxa;
	int n_segs;
	double lengths[GC_MAX_SEGS];
	int sides[GC_MAX_SEGS - 1];
	double reach;
	double chain_reach;
	GcV3 rest_dir_local;
	int has_pts;
	double pts[GC_MAX_SEGS + 1][2];
	int reached;
	GcV3 plane_normal;
	double err;
	int limited;
	int has_angles;
	double angles[GC_MAX_SEGS]; // [hip, rel1, rel2, ...] kept within legal limits for warm-starts
	double yaw_deg;
	double hip_deg;
	double bends_deg[GC_MAX_SEGS - 1];
	double coxa_ratio;
	double limit_margin_deg;

	double coxa_max;
	double roll_max;
	double hip_lim[2];
	double bend_lim[GC_MAX_SEGS - 1][2];
	double rel_lim[GC_MAX_SEGS - 1][2]; // internal relative angles corresponding to bend_lim
} GcLegIK;

int gc_solve_two_bone( double l1, double l2, double tx, double ty, int side,
					   double* kx, double* ky, double* fx, double* fy );
int gc_solve_fabrik_2d( const double* lengths, int n_segs, double pts[][2], double tx, double ty,
						const int* sides, int iters, double tol );
void gc_initial_pose_2d( const double* lengths, int n_segs, const int* sides, double tx, double ty, double pts[][2] );
void gc_legik_init( GcLegIK* ik, int plane_mode, double coxa, const double* lengths, int n_segs,
					const int* sides, GcV3 rest_dir_local, const GcLimits* lim );
int gc_legik_solve( GcLegIK* ik, GcV3 mount, const GcFrame* frame, GcV3 target, double plane_roll, GcV3* out_joints );
int gc_legik_feasible( GcLegIK* ik, GcV3 mount, const GcFrame* frame, GcV3 target, double slack_deg );

// Gait timing entry in a mech's gait library.
// Legs swing when (gait_phase + offsets[leg]) mod 1 < (1 - duty).
typedef struct GcGait {
	char name[24];
	int n_legs;
	double offsets[GC_MAX_LEGS];
	double duty;                        // fraction of cycle spent planted (>0.5 walking, <0.5 running)
	double stride;                      // nominal step distance at speed (m)
	double cycle_min;
	double cycle_max;
	int min_support;                    // minimum planted legs required before lifting another
	int adjacent_lock;                  // prevent front/back or left/right neighbors lifting together
	int static_stable;                  // keep projected CoM inside support hull before lifting
	double lift;
	int pair_sync;
	double reach;
	double leap;
	double stride_pitch;
	int group_count;
	int group_sizes[GC_MAX_GROUPS];
	int groups[GC_MAX_GROUPS][GC_MAX_GROUP_SIZE];
	int has_duty_fast;
	double duty_fast;                   // duty target at top walking speed for hopping trot/tripod/tetrapod
	int phase_lock;                     // align synced groups on touchdown even if a leg released early
} GcGait;

void gc_gaits_biped( GcGait* out, int* out_count );
void gc_gaits_quadruped( GcGait* out, int* out_count );
void gc_gaits_hexapod( GcGait* out, int* out_count );
void gc_gaits_octopod( GcGait* out, int* out_count );
int gc_gait_find( const GcGait* gaits, int count, const char* name );
double gc_phase_lerp( double a, double b, double t );

/* --- Mechdefs (gc_mechdefs.c) --- */
typedef struct GcLegDef {
	double mount[3];
	double rest[2];
	int has_sides;
	int sides[GC_MAX_SEGS - 1];
	int side;
	int has_limits;
	GcLimits limits;
} GcLegDef;

typedef struct GcMechDef {
	char name[24];
	int kind_id;
	int faction;
	char blurb[128];
	int plane;
	double coxa;
	int n_segs;
	double lengths[GC_MAX_SEGS];
	int sides[GC_MAX_SEGS - 1];
	double widths[GC_MAX_SEGS];
	GcLimits limits;
	int n_legs;
	GcLegDef legs[GC_MAX_LEGS];
	double body_half[3];
	double ride;
	double speed_slow;
	double speed_walk;
	double speed_run;
	double accel;
	double accel_run;
	double turn_rate;
	int gait_count;
	GcGait gaits[GC_MAX_GAITS];
	int gait_slow_idx;
	int gait_walk_idx;
	int gait_run_idx;
	int gait_walk_fast_idx;
	int gait_drift_idx;
	double step_height;
	double step_max;
	double mantle_max;
	double wire_max;
	int climber;
	double jump_speed;
	double max_tilt;
	double look_tilt;
	double mass;
	int biped;
	double bound;
	double bound_pitch;
	double zone;
	double swing_min;
	double swing_max;
	double size;
} GcMechDef;

const GcMechDef* gc_mechdef_by_kind( int kind );
int gc_mechdef_count( void );
int gc_mechdef_from_desc( const GcMechDesc* desc, int kind_id, GcMechDef* out );

// Active swing trajectory using a C1-continuous quintic Hermite curve.
// Retargeting mid-flight preserves current position and velocity while smoothly guiding into end_vel.
typedef struct GcSwing {
	int active;
	GcV3 start_w;
	GcV3 target;
	GcV3 normal;
	double t;
	double dur;
	double lift;
	GcV3 up;
	int has_ground;
	double retarget_cd;
	int has_ideal;
	GcV3 ideal;
	int force_retarget;
	GcV3 origin;
	GcV3 start_velocity;
	GcV3 end_velocity;
	GcV3 bow;
	GcV3 cruise;
	double takeoff_time;
	double landing_time;
	double plan_t;
	double plan_dur;
	int has_start_hip_l;
	GcV3 start_hip_l;
	int has_target_hip_l;
	GcV3 target_hip_l;
	double arc_w;
	double arc_t0;
} GcSwing;

// Runtime state for a single leg (kinematics, stance contact, reactive stretch triggers).
typedef struct GcLeg {
	int idx;
	int pair;
	int side;
	GcV3 mount_local;
	double rest_xz[2];
	GcLegIK ik;
	GcLegIK ik_probe; // disposable IK scratch instance for trial foothold checks
	double reach;
	int is_front;
	double foot_r;
	int force;
	GcV3 foot;
	GcV3 foot_normal;
	int planted;
	int searching;
	double search_cd;
	GcSwing swing;
	int n_joints;
	GcV3 joints[GC_MAX_JOINTS];
	int n_joints_local;
	GcV3 joints_local[GC_MAX_JOINTS];
	GcV3 plane_normal;
	double offset;
	int prev_window;
	int pending;
	double cooldown;
	int sliding;
	double stretch;
	double stretch_rate;
	double limit_t;
	int push;
	int has_carry;
	GcV3 carry;
	double off_path;
	int has_settled_at;
	GcV3 settled_at;
	int has_settled_hull_pos;
	GcV3 settled_hull_pos;
	int has_land_lead;
	GcV3 land_lead;
	double stance_t;
	int has_last_plant;
	GcV3 last_plant_p;
	GcV3 last_plant_n;
	double void_mark;
	double displacement;
	GcV3 mount_world;
	GcV3 ideal_now;
	GcV3 ideal_pred;
	double air_time;
	double plane_roll;
	int near_limit;
	double kc_t;
} GcLeg;

// Per-frame normalized control input.
typedef struct GcInput {
	double fwd, strafe, turn, run, slow, crouch, jump, drift, climb, impact, gait, aim;
} GcInput;

// Scripted obstacle vault / wire-winch animation state.
typedef struct GcMantle {
	int active;
	GcV3 p0;
	GcV3 p2;
	double t;
	double dur;
	double height;
	GcV3 top;
	GcV3 fwd;
	int wire;
	int landed;
} GcMantle;

// Predicted ballistic landing snapshot computed at launch for mid-air leg bracing.
typedef struct GcImpact {
	int valid;
	double t;
	GcV3 point;
	GcV3 normal;
	double speed;
} GcImpact;

// Smoothed ground lookahead and underfoot footing metrics used by the gait selector.
typedef struct GcSituation {
	double ahead_rough;
	double ahead_slope;
	double ahead_void;
	double under_rough;
	double clutter;
	double edge;
	double harsh;
	double slope;
	double stance_slope;
	char reason[32];
	int n;
	double probe_t;
	double probe_raw[4];
	double probe_prev[4];
	double hold[3];
} GcSituation;

// Hysteresis state machine picking the highest gait rung supported by speed and terrain.
typedef struct GcGaitSelector {
	int ladder[4]; // static -> walk -> fast -> gallop indices into def->gaits
	int rung;
	double dwell;
	char reason[32];
	int cap;
	double low_t;
	char cap_why[32];
	double cap_t;
	int has_hold_v;
	double hold_v;
	double relax_wait;
	double since_relax;
	int relaxed;
	double better_t;
	double prev_v;
	double accel_v;
} GcGaitSelector;

// Smooth interpolator between two gait tables; keeps support rules conservative until s >= 0.85.
typedef struct GcGaitBlend {
	GcGait src;
	int dst_idx;
	GcGait dst;
	double t;
	double s;
	double T;
	GcGait eff;
} GcGaitBlend;

typedef struct GcEventBuf {
	float* data;
	int count; // number of events
	int cap;   // event capacity
} GcEventBuf;

// Per-mech simulation state. Each worker owns disjoint GcMech instances with zero shared writes.
typedef struct GcMech {
	const GcMechDef* d;
	double size;
	GcWorldQueryCtx wq;
	GcEventBuf events;

	GcFrame frame;
	double ride;
	GcV3 pos;
	GcV3 vel;
	GcV3 ang_vel;
	double yaw_rate;
	GcLeg legs[GC_MAX_LEGS];
	int n;
	int biped;
	double reach_avg;
	double chord_avg;
	double v_walk;
	double v_slow;
	double jump_sink_max;
	double look_pitch_dn, look_pitch_up;
	double look_pitch;
	int joints_per_leg;

	GcInput inp;
	GcInput prev;
	int mode;
	int gait_override; /* -1 = None, else gait index */
	int gait_idx;
	GcGaitBlend blend;
	GcGait gait;
	GcSituation situation;
	GcGaitSelector selector;
	int has_speed_cap;
	double speed_cap;
	double v_desired;
	double lateral_ratio;
	GcV3 v_cmd;
	GcV3 drive_target;
	double drive_accel;
	GcV3 accel_smooth;
	int calm_gait;
	double foot_rms;
	int fh_count;
	int fh_bad;
	char fh_why[16];
	double duty;
	double phase;
	double cycle;
	double t_ahead;

	int state;
	double state_t;
	int crouch_toggle;
	double crouch;
	int climb_enabled;
	int drifting;
	double drift_t;
	double drift_amt;
	GcV3 slide_dir;
	double slide_cd;
	double slide_buf;
	int slide_lock;
	double unsupported_t;
	int air_belly;
	double void_t;
	double flight_t;
	double flight_dur;
	double flight_h;
	double flight_y0;
	double flight_hold;
	int flight_count;
	GcV3 slide_vel;
	double absorb;
	double jump_sink;
	int braking;
	int has_balance_out;
	GcV3 balance_out;
	double balance_excess;
	double balance_h;
	double unbal_t;
	double jump_sink_cap;
	double jump_r0;
	double jump_y0;
	int jump_npl;
	double brace;
	GcImpact impact;
	int prev_npl;
	int trailing_support;
	double detach_t;
	int has_plane_n_local;
	GcV3 plane_n_local;
	GcV3 ground_n;
	double land_right_t;
	GcV3 land_right_n;
	double support;
	GcV2 hull[GC_MAX_HULL_PTS];
	int hull_n;
	GcV3 vis_offset;
	GcMantle mantle;
	int on_wall;
	GcV3 last_accel;
	double speed;
	GcV3 vt;
	uint64_t rng_state;
	double spark_t;
	double ceil_t;
	int has_ceil_y;
	double ceil_y;
	double pitch_cap;
	double pitch_strain_t;
	double sup_t;
	int has_sup_h;
	GcHit sup_h;
	int has_sup_ha;
	GcHit sup_ha;
	double sup_ha_slope_dz;
	double mantle_t;

	double ik_solve_hz;
	double probe_period;
	double speed_scale;
	int plane_roll_enabled;
	double ik_acc;
	int profile_enabled;
	double prof[10];
	double step_ms;

	double col_r;
	int n_col_offs;
	double col_offs[GC_MAX_COL_OFFS];
	double col_lat;
} GcMech;

/* Forward declarations across C source modules */
void gc_input_set( GcInput* inp, const float* vals );

void gc_leg_init( GcLeg* l, int idx, const GcLegDef* ldef, const GcMechDef* d, double size );
GcV3 gc_leg_rest_local( const GcLeg* l, double ride, double spread );
void gc_leg_begin_swing( GcLeg* l, GcV3 target, GcV3 normal, double dur, double lift, GcV3 up,
						 int has_ground, GcWorldQueryCtx* wq, double clearance_probe,
						 GcV3 start_vel, GcV3 end_vel, GcV3 bow, int has_ceil_y, double max_ceil_y );
void gc_leg_curve_world( const GcLeg* l, int n_pts, GcV3* out_pts );
int gc_leg_update_swing( GcLeg* l, double dt );
double gc_leg_keep_clear( GcLeg* l, GcWorldQueryCtx* wq, GcV3 up, double max_step,
						  int has_ceil_y, double max_ceil_y, double dt );
void gc_leg_retarget( GcLeg* l, GcV3 target, GcV3 normal, int has_ground, GcWorldQueryCtx* wq,
					  double clearance_probe, GcV3 end_vel, GcV3 bow, int has_ceil_y, double max_ceil_y );
void gc_leg_solve( GcLeg* l, GcMech* m, GcV3 mount_l, double dt );
void gc_leg_reproject_cached( GcLeg* l, const GcMech* m );

void gc_situation_init( GcSituation* s );
void gc_situation_update( GcSituation* s, GcMech* m, double dt );
void gc_selector_init( GcGaitSelector* sel, const GcMech* m );
void gc_selector_caps( const GcGaitSelector* sel, const GcMech* m, double caps[4] );
int gc_selector_select( GcGaitSelector* sel, GcMech* m, double dt, int* out_has_cap, double* out_cap );
void gc_blend_init( GcGaitBlend* b, int dst_idx, const GcGait* g );
void gc_blend_start( GcGaitBlend* b, int dst_idx, const GcGait* dst, double T );
void gc_blend_update( GcGaitBlend* b, double dt );
void gc_blend_fit_support( GcGaitBlend* b, double duty );
void gc_blend_label( const GcGaitBlend* b, char* buf, size_t cap );

int gc_mech_climbing_mode( const GcMech* m );
GcV3 gc_mech_probe_up( const GcMech* m );
GcFrame gc_mech_support_frame( const GcMech* m );
GcV3 gc_mech_heading_fwd( const GcMech* m );
double gc_mech_ride_eff( const GcMech* m );
int gc_mech_wire_active( const GcMech* m );
void gc_mech_emit( GcMech* m, int kind, GcV3 p, GcV3 n, double strength, GcV3 extra );

void gc_landing_pose( const GcMech* m, const GcLeg* l, double delay, GcV3* out_mount, GcFrame* out_frame );
int gc_find_foothold( GcMech* m, GcLeg* l, GcV3 ideal, double delay, GcV3* out_pt, GcV3* out_n );
int gc_clear_spot( GcMech* m, const GcLeg* l, GcV3 p, GcV3 up, const GcV3* opt_mount );
int gc_reachable( const GcMech* m, GcLeg* l, GcV3 p, const GcV3* opt_hip, const GcFrame* opt_fr );
GcV3 gc_hang_pose( GcMech* m, const GcLeg* l, double ride );
int gc_futile_step( GcLeg* l, int has_fh, GcV3 fh_p );

void gc_solve_leg( GcMech* m, GcLeg* l, double dt );
void gc_slide_foot_clear( GcMech* m, GcLeg* l, GcV3 up );
void gc_feet_clear( GcMech* m, double dt );
void gc_legs_air( GcMech* m, double dt );
double gc_clearance( const GcMech* m );

int gc_may_flight( const GcMech* m );
int gc_galloping_fast( const GcMech* m, int check_blend );
double gc_v_leg( const GcMech* m );
GcV3 gc_planar_travel( const GcMech* m, double seconds );
int gc_try_takeoff( GcMech* m, GcV3 up, GcV3 vt_new, double* io_v_up, int pushed );
int gc_update_flight( GcMech* m, double dt, int npl, GcV3 vt_new, GcV3 up, double* io_v_up );
void gc_update_legs( GcMech* m, double dt );
int gc_can_lift( GcMech* m, GcLeg* l, int emergency, int check_static );
void gc_lift( GcMech* m, GcLeg* l, double dur, int has_fh, GcV3 fh_p, GcV3 fh_n,
			  const GcV3* opt_start_vel, const GcV3* opt_end_vel );
void gc_on_plant( GcMech* m, GcLeg* l );
void gc_set_swing_arc( GcMech* m, GcLeg* l, GcV3 target, double rem, int reset_start );

void gc_mech_init( GcMech* m, const GcMechDef* d, GcV3 ground_pos, double yaw, const GcWorldCallbacks* cb );
void gc_mech_destroy( GcMech* m );
void gc_mech_update( GcMech* m, double dt );
void gc_mech_enter_air( GcMech* m );
GcV3 gc_mantle_pos( const GcMech* m, double t );
const char* gc_mech_mode_label( const GcMech* m );
int gc_mech_flags( const GcMech* m );
void gc_mech_gait_label( const GcMech* m, char* buf, size_t cap );
int gc_mech_pack( const GcMech* m, float* out_buf, int max_floats );
int gc_mech_debug_lines( const GcMech* m, float* out_buf, int max_floats );
void gc_mech_stats_fill( const GcMech* m, float sim_ms, int total_rays, GcMechStats* out );

#endif /* GAITCORE_INTERNAL_H */
