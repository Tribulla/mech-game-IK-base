// GaitCore public C11 API.
// Multithreaded procedural IK and gait engine for 2-, 4-, 6-, and 8-legged mechs.
// World coordinates are meters and seconds, right-handed, +Y up, +Z forward at yaw = 0.

#ifndef GAITCORE_C_H
#define GAITCORE_C_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Packed binary buffer strides (kept in lockstep with IK/python/gaitcore/api.py).
enum {
	GC_INPUT_STRIDE = 12,
	GC_HEADER_FLOATS = 28,
	GC_LEG_EXTRA_FLOATS = 6,
	GC_EVENT_FLOATS = 11,
	GC_DEBUG_LINE_FLOATS = 9,
	GC_PROFILE_HEADER = 16,
	GC_PROFILE_KINDS = 9,
	GC_PROFILE_KIND_STRIDE = 5,
	GC_PROFILE_FLOATS = GC_PROFILE_HEADER + GC_PROFILE_KINDS * GC_PROFILE_KIND_STRIDE
};

enum {
	GC_MAX_LEGS_PUBLIC = 8,
	GC_MAX_SEGS_PUBLIC = 4
};

// Planar IK constraint mode:
// - radial: coxa rotates around hull up toward the target (spider-like splayed legs)
// - sagittal: leg plane aligns with hull forward with limited hip roll (bipeds, narrow tanks)
enum {
	GC_LEG_PLANE_RADIAL = 0,
	GC_LEG_PLANE_SAGITTAL = 1
};

// Per-mech input channels packed sequentially into gc_step(..., inputs, count).
// Edge-triggered actions (crouch, jump, climb, impact, gait) fire on 0 -> 1 transitions.
enum {
	GC_IN_FWD = 0,    // -1..1 body-relative forward/reverse throttle
	GC_IN_STRAFE,     // -1..1 body-relative lateral strafe
	GC_IN_TURN,       // -1..1 yaw command (+ turns toward body right axis)
	GC_IN_RUN,        // > 0.5 selects run / gallop mode
	GC_IN_SLOW,       // > 0.5 selects careful slow-walk / static gait mode
	GC_IN_CROUCH,     // rising edge toggles crouch
	GC_IN_JUMP,       // rising edge starts crouch-jump (or kicks off a wall when climbing)
	GC_IN_DRIFT,      // > 0.5 at speed enters a directional power slide
	GC_IN_CLIMB,      // rising edge toggles surface wall-climbing (requires climber != 0)
	GC_IN_IMPACT,     // rising edge injects a lateral push impulse to test balance recovery
	GC_IN_GAIT,       // rising edge cycles through forced gait overrides (wraps back to auto)
	GC_IN_AIM         // desired hull aim pitch in radians (+ up, clamped inside hull limits)
};

// Float offsets within each mech's GC_HEADER_FLOATS pose header.
// Note that right = fwd x up, so negate right (-right, up, fwd) for a right-handed basis matrix.
enum {
	GC_POSE_POS = 0,     // 3 floats: hull center in world space
	GC_POSE_RIGHT = 3,   // 3 floats: unit body right axis (fwd x up)
	GC_POSE_UP = 6,      // 3 floats: unit body up axis
	GC_POSE_FWD = 9,     // 3 floats: unit body forward axis
	GC_POSE_SPEED = 12,  // 1 float: current ground/planar speed (m/s)
	GC_POSE_STATE = 13,  // 1 float: GC_STATE_* enum
	GC_POSE_FLAGS = 14,  // 1 float: bitmask of GC_FLAG_* flags
	GC_POSE_PLANTED = 15,// 1 float: number of feet currently planted on geometry
	GC_POSE_VEL = 16,    // 3 floats: linear velocity (m/s)
	GC_POSE_PHASE = 19,  // 1 float: gait clock phase [0, 1)
	GC_POSE_DRIFT = 20,  // 1 float: slide blend weight [0, 1]
	GC_POSE_GAIT = 21,   // 1 float: active gait table index
	GC_POSE_SUPPORT = 22,// 1 float: signed margin from CoM to support polygon (m; negative = outside)
	GC_POSE_CYCLE = 23,  // 1 float: current gait cycle period (s)
	GC_POSE_WIRE = 24,   // 1 float: 1.0 while winching up a wall on wire anchors, else 0.0
	GC_POSE_ANCHOR = 25  // 3 floats: world-space wire anchor point when GC_POSE_WIRE > 0.5
};

// Bitmask flags packed into pose[GC_POSE_FLAGS].
enum {
	GC_FLAG_DRIFT = 1,
	GC_FLAG_WALL = 2,
	GC_FLAG_CROUCH = 4,
	GC_FLAG_CLIMB = 8,
	GC_FLAG_SUPPORTED = 16,
	GC_FLAG_SLOW = 32,
	GC_FLAG_RUN = 64,
	GC_FLAG_WIRE = 128,
	GC_FLAG_LEAP = 256
};

enum {
	GC_STATE_GROUND = 0,
	GC_STATE_AIR = 1,
	GC_STATE_MANTLE = 2,
	GC_STATE_CROUCHJUMP = 3
};

// Packed event kinds emitted by gc_events() (11 floats per event).
enum {
	GC_EV_STEP = 1,
	GC_EV_SPARK = 2,
	GC_EV_LAND = 3,
	GC_EV_BOOST = 4,
	GC_EV_IMPACT = 5,
	GC_EV_SKID = 6,
	GC_EV_MANTLE = 7
};

enum {
	GC_DBG_LINE = 0,
	GC_DBG_LINE_DIM,
	GC_DBG_INFO,
	GC_DBG_HIGHLIGHT,
	GC_DBG_GOOD,
	GC_DBG_WARNING,
	GC_DBG_ERROR,
	GC_DBG_FORCE,
	GC_DBG_NORMAL,
	GC_DBG_VELOCITY,
	GC_DBG_AXIS_X,
	GC_DBG_AXIS_Y,
	GC_DBG_AXIS_Z
};

// Ray hit result populated by the host's GcRaycastFn callback.
typedef struct GcRayHit {
	double t;           // hit distance along unit ray direction (must be > 0)
	double px, py, pz;  // world hit point
	double nx, ny, nz;  // unit outward surface normal
	int surface;        // optional host material tag
} GcRayHit;

// Read-only, thread-safe world query interface implemented by the host engine.
// Worker threads invoke these concurrently inside gc_step() with no internal locks held.
// Filter out the mech colliders themselves so feet do not step on their own hulls,
// and ignore ray intersections at t <= 0 so underground probes can escape cleanly.
typedef int ( *GcRaycastFn )( void* user, double ox, double oy, double oz,
							  double dx, double dy, double dz, double max_dist, GcRayHit* out_hit );
typedef int ( *GcSpherePushFn )( void* user, double cx, double cy, double cz, double radius,
								 double* out_xyz, double* out_nxyz );
typedef double ( *GcTerrainHeightFn )( void* user, double x, double z );

typedef struct GcWorldCallbacks {
	GcRaycastFn raycast;
	GcSpherePushFn sphere_push;
	GcTerrainHeightFn terrain_height;
	void* user;
} GcWorldCallbacks;

typedef struct GcMechInfo {
	const char* name;
	const char* blurb;
	int kind;
	int legs;
	int joints_per_leg;
	float half[3];
	float widths[4];
	int width_count;
	float mass;
	int climber;
	int faction;
	float wire_max;
	float walk_speed;
	float run_speed;
	int floats;
} GcMechInfo;

typedef struct GcMechStats {
	char mode[24];
	char gait[64];
	int gait_override;
	char state[24];
	float duty;
	float cycle;
	float max_stretch;
	float sim_ms;
	int rays;
	int climb_enabled;
	int crouch_toggle;
} GcMechStats;

typedef struct GcQueryStats {
	int ray_calls;
	int push_calls;
	int height_calls;
	float ray_ms;
	float push_ms;
	float height_ms;
} GcQueryStats;

typedef struct GcBenchConfig {
	float max_substep;
	float leg_ik_hz;
	float probe_hz;
	float speed_scale;
	int plane_roll;
	int profile;
} GcBenchConfig;

// Joint angle limits in degrees for the planar IK solver.
// Keep bend_deg[i][0] > 0 to avoid knee hyperextension and < 180 to stop segments folding flat.
typedef struct GcJointLimitsDesc {
	int has_coxa;
	float coxa_deg;                         // max yaw either way from rest direction (radial legs)
	int has_roll;
	float roll_deg;                         // max lateral hip roll from vertical (sagittal legs)
	int has_hip;
	float hip_deg[2];                       // radial: 0=outward, +90=up; sagittal: 0=straight down, +=forward
	int bend_count;
	float bend_deg[GC_MAX_SEGS_PUBLIC - 1][2]; // min/max interior bend toward bend_sides[i]
} GcJointLimitsDesc;

// Body-local attachment and neutral foothold offsets for one leg.
// List legs front-to-back in left/right pairs (L0, R0, L1, R1, ...) so gait tables match.
typedef struct GcLegDesc {
	float mount[3];   // hip mount offset in body-local [right, up, fwd] (m)
	float rest_xz[2]; // default ground target under the hull in body-local [right, fwd] (m)
	int side;         // -1 = left side, +1 = right side, 0 = infer from mount[0] / rest_xz[0]
} GcLegDesc;

// Full archetype definition passed to gc_register_mech_type() or gc_spawn_custom().
// Aim for rest reach around 50-65% of (coxa + sum(segment_lengths)) so legs have room to stretch and fold.
typedef struct GcMechDesc {
	char name[24];
	char blurb[128];
	int faction;
	int plane_mode;                         // GC_LEG_PLANE_RADIAL or GC_LEG_PLANE_SAGITTAL
	float coxa;                             // yawing coxa length (m); set 0 for sagittal legs
	int num_segments;                       // 2..4 segments inside the leg plane
	float segment_lengths[GC_MAX_SEGS_PUBLIC];
	int bend_sides[GC_MAX_SEGS_PUBLIC - 1]; // +1 folds knee up/forward, -1 down/back
	float segment_widths[GC_MAX_SEGS_PUBLIC];
	GcJointLimitsDesc limits;
	int num_legs;                           // 2, 4, 6, or 8
	GcLegDesc legs[GC_MAX_LEGS_PUBLIC];
	float body_half[3];                     // hull collision box half-extents [x, y, z] (m)
	float ride_height;                      // nominal hull center height above feet (m)
	float speed_slow;
	float speed_walk;
	float speed_run;
	float accel;
	float accel_run;
	float turn_rate;                        // max yaw rate (rad/s)
	float step_height;                      // swing lift clearance (m)
	float step_max;                         // tallest step-up obstacle height (m)
	float mantle_max;                       // tallest ledge vault height (m)
	float wire_max;                         // max wall height scaled with wire anchors (m; 0 disables)
	int climber;                            // non-zero lets feet grip and walk up vertical walls
	float jump_speed;                       // vertical launch speed (m/s)
	float max_tilt_deg;                     // ground slope follow limit (deg)
	float look_tilt_deg;                    // max hull pitch offset when aiming up/down (deg)
	float mass;
	float bound;                            // vertical leap amplitude during flight gaits (m; 0 disables)
	float bound_pitch_deg;                  // pitch rock amplitude during flight gaits (deg)
	float zone;                             // multiplier on reactive step trigger radius (~1.0)
	float swing_min;                        // shortest allowed swing duration (s)
	float swing_max;                        // longest allowed swing duration (s)
	float size;                             // gait cycle time scale (1.0 nominal; larger steps slower)
} GcMechDesc;

typedef struct GcContext GcContext;
typedef struct GcStubWorld GcStubWorld;

int gc_hardware_threads( void );

// Creates a simulation context and worker pool. Pass num_threads <= 0 to auto-detect cores.
GcContext* gc_create( int num_threads );
void gc_destroy( GcContext* ctx );
// Despawns all active mechs and clears transient stats while keeping registered custom types.
void gc_reset( GcContext* ctx );
// Binds the host world callbacks. Must remain valid and thread-safe while gc_step() runs.
void gc_set_world( GcContext* ctx, const GcWorldCallbacks* world );
// Resizes the worker pool (call between gc_step() frames, never concurrently with stepping).
void gc_set_threads( GcContext* ctx, int num_threads );
int gc_get_threads( const GcContext* ctx );

// Queries the built-in presets (0 .. gc_kind_count()-1).
int gc_kind_count( void );
const char* gc_kind_name( int kind );

// Populates valid default descriptors for 2, 4, 6, or 8 legs or copies a built-in preset to edit.
void gc_mech_desc_init_default( GcMechDesc* out_desc, int num_legs );
int gc_mech_desc_from_preset( int preset_kind, GcMechDesc* out_desc );
// Validates and registers a custom archetype in ctx, returning its kind ID (>= gc_kind_count()) or -1.
int gc_register_mech_type( GcContext* ctx, const GcMechDesc* desc );
int gc_context_kind_count( const GcContext* ctx );
const char* gc_context_kind_name( const GcContext* ctx, int kind );
// Despawns all mechs and frees custom archetypes registered in ctx.
void gc_clear_custom_types( GcContext* ctx );

// Spawns a mech at ground point (x, y, z) facing yaw (rad). Returns handle 0..N-1 or -1 on error.
int gc_spawn( GcContext* ctx, int kind, float x, float y, float z, float yaw );
// Registers (or reuses an identical matching descriptor) and spawns one instance.
int gc_spawn_custom( GcContext* ctx, const GcMechDesc* desc, float x, float y, float z, float yaw );
int gc_respawn( GcContext* ctx, int handle, int kind, float x, float y, float z, float yaw );
int gc_despawn_to( GcContext* ctx, int target_count );
int gc_count( const GcContext* ctx );
int gc_info( const GcContext* ctx, int handle, GcMechInfo* out );
int gc_floats_per_mech( const GcContext* ctx, int handle );

// Steps all active mechs in parallel across workers (dt clamped <= 0.1 s, substepped by cfg.max_substep).
void gc_step( GcContext* ctx, float dt, const float* inputs, int mech_count );
// Packs all mech poses sequentially into out_buf (parallelized internally when count >= 16).
int gc_poses( GcContext* ctx, float* out_buf, int max_floats );
// Copies accumulated frame events in deterministic handle order and clears the buffer.
int gc_events( GcContext* ctx, float* out_buf, int max_floats );
int gc_debug_lines( const GcContext* ctx, int handle, float* out_buf, int max_floats );
void gc_set_debug( GcContext* ctx, int on );
int gc_stats( const GcContext* ctx, int handle, GcMechStats* out );

// Configures crowd LOD rates (decoupled IK Hz, lookahead probe Hz), speed scale, and stage profiling.
void gc_set_bench_config( GcContext* ctx, const GcBenchConfig* cfg );
int gc_profile_data( const GcContext* ctx, float* out_buf, int max_floats );
void gc_reset_query_stats( GcContext* ctx );
GcQueryStats gc_get_query_stats( const GcContext* ctx );
float gc_last_cpu_ms( const GcContext* ctx );

// Lightweight standalone box/OBB world for unit tests and headless benchmarks (read-only after setup).
GcStubWorld* gc_stub_world_create( double ground_y, int has_ground );
void gc_stub_world_destroy( GcStubWorld* w );
void gc_stub_world_add_box( GcStubWorld* w, double x0, double y0, double z0, double x1, double y1, double z1 );
void gc_stub_world_add_obox( GcStubWorld* w, double cx, double cy, double cz,
							 double hx, double hy, double hz, double yaw, double pitch );
GcWorldCallbacks gc_stub_world_callbacks( const GcStubWorld* w );

#ifdef __cplusplus
}
#endif

#endif // GAITCORE_C_H
