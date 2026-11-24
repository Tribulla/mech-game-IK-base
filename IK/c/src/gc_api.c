// Public C11 entry points: context ownership, custom mech registration, and multithreaded stepping.
// Each mech is cache-line aligned and stepped independently in parallel, then events and stats
// are gathered sequentially in 0..count-1 handle order for bit-identical determinism across thread counts.

#include "gc_internal.h"
#include "gc_thread.h"

struct GcContext {
	GcThreadPool* pool;
	GcWorldCallbacks world_cb;
	GcMech** mechs;
	int count;
	int cap;

	GcMechDef** custom_defs;
	int custom_count;
	int custom_cap;

	float* events_buf;
	int event_floats;
	int event_cap_floats;

	float* dbg_rays_buf;
	int dbg_ray_floats;
	int dbg_ray_cap_floats;

	GcBenchConfig bench;
	int debug;
	float prof_buf[GC_PROFILE_FLOATS];
	GcQueryStats query_stats;
	float last_wall_ms;
	float last_cpu_ms;
	int last_rays;
};

static void apply_bench_cfg( const GcContext* ctx, GcMech* m, int idx )
{
	m->ik_solve_hz = (double)ctx->bench.leg_ik_hz;
	m->probe_period = 1.0 / gc_clamp( (double)ctx->bench.probe_hz, 1.0, 120.0 );
	m->speed_scale = gc_clamp( (double)ctx->bench.speed_scale, 0.0, 3.0 );
	m->plane_roll_enabled = ctx->bench.plane_roll ? 1 : 0;
	m->profile_enabled = ctx->bench.profile ? 1 : 0;
	m->ik_acc = ( m->ik_solve_hz > 0.0 ) ? ( (double)( idx % 8 ) * ( 0.125 / m->ik_solve_hz ) ) : 0.0;
	m->wq.record = ctx->debug ? 1 : 0;
	m->wq.profile_queries = ctx->bench.profile ? 1 : 0;
	m->wq.cb = &ctx->world_cb;
}

static void default_bench_config( GcBenchConfig* cfg )
{
	cfg->max_substep = (float)GC_MAX_SUBSTEP_DEFAULT;
	cfg->leg_ik_hz = 0.0f;
	cfg->probe_hz = 20.0f;
	cfg->speed_scale = 1.0f;
	cfg->plane_roll = 1;
	cfg->profile = 0;
}

GcContext* gc_create( int num_threads )
{
	GcContext* ctx = (GcContext*)calloc( 1, sizeof( GcContext ) );
	if ( !ctx )
		return NULL;
	ctx->pool = gc_pool_create( num_threads );
	default_bench_config( &ctx->bench );
	return ctx;
}

void gc_destroy( GcContext* ctx )
{
	if ( !ctx )
		return;
	gc_clear_custom_types( ctx );
	free( ctx->custom_defs );
	free( ctx->mechs );
	free( ctx->events_buf );
	free( ctx->dbg_rays_buf );
	gc_pool_destroy( ctx->pool );
	free( ctx );
}

static const GcMechDef* ctx_def_by_kind( const GcContext* ctx, int kind )
{
	int base_n = gc_mechdef_count();
	if ( kind >= 0 && kind < base_n )
		return gc_mechdef_by_kind( kind );
	int ci = kind - base_n;
	if ( ctx && ci >= 0 && ci < ctx->custom_count )
		return ctx->custom_defs[ci];
	return NULL;
}

int gc_register_mech_type( GcContext* ctx, const GcMechDesc* desc )
{
	if ( !ctx || !desc )
		return -1;
	int kind_id = gc_mechdef_count() + ctx->custom_count;
	GcMechDef* def = (GcMechDef*)malloc( sizeof( GcMechDef ) );
	if ( !def )
		return -1;
	if ( !gc_mechdef_from_desc( desc, kind_id, def ) )
	{
		free( def );
		return -1;
	}
	if ( ctx->custom_count == ctx->custom_cap )
	{
		int nc = ctx->custom_cap ? ctx->custom_cap * 2 : 8;
		GcMechDef** arr = (GcMechDef**)realloc( ctx->custom_defs, (size_t)nc * sizeof( GcMechDef* ) );
		if ( !arr )
		{
			free( def );
			return -1;
		}
		ctx->custom_defs = arr;
		ctx->custom_cap = nc;
	}
	ctx->custom_defs[ctx->custom_count++] = def;
	return kind_id;
}

int gc_context_kind_count( const GcContext* ctx )
{
	return gc_mechdef_count() + ( ctx ? ctx->custom_count : 0 );
}

const char* gc_context_kind_name( const GcContext* ctx, int kind )
{
	const GcMechDef* d = ctx_def_by_kind( ctx, kind );
	return d ? d->name : "?";
}

void gc_clear_custom_types( GcContext* ctx )
{
	if ( !ctx )
		return;
	gc_despawn_to( ctx, 0 );
	for ( int i = 0; i < ctx->custom_count; ++i )
	{
		free( ctx->custom_defs[i] );
		ctx->custom_defs[i] = NULL;
	}
	ctx->custom_count = 0;
}

void gc_reset( GcContext* ctx )
{
	if ( !ctx )
		return;
	for ( int i = 0; i < ctx->count; ++i )
	{
		gc_mech_destroy( ctx->mechs[i] );
		gc_free_aligned( ctx->mechs[i] );
		ctx->mechs[i] = NULL;
	}
	ctx->count = 0;
	ctx->event_floats = 0;
	ctx->dbg_ray_floats = 0;
	ctx->debug = 0;
	default_bench_config( &ctx->bench );
	memset( ctx->prof_buf, 0, sizeof( ctx->prof_buf ) );
	memset( &ctx->query_stats, 0, sizeof( ctx->query_stats ) );
	ctx->last_wall_ms = 0.0f;
	ctx->last_cpu_ms = 0.0f;
	ctx->last_rays = 0;
}

void gc_set_world( GcContext* ctx, const GcWorldCallbacks* world )
{
	if ( !ctx )
		return;
	if ( world )
		ctx->world_cb = *world;
	else
		memset( &ctx->world_cb, 0, sizeof( ctx->world_cb ) );
	for ( int i = 0; i < ctx->count; ++i )
		ctx->mechs[i]->wq.cb = &ctx->world_cb;
}

void gc_set_threads( GcContext* ctx, int num_threads )
{
	if ( !ctx )
		return;
	if ( num_threads <= 0 )
		num_threads = gc_hardware_threads();
	if ( num_threads < 1 )
		num_threads = 1;
	if ( num_threads > 64 )
		num_threads = 64;
	if ( ctx->pool && gc_pool_thread_count( ctx->pool ) == num_threads )
		return;
	gc_pool_destroy( ctx->pool );
	ctx->pool = gc_pool_create( num_threads );
}

int gc_get_threads( const GcContext* ctx )
{
	return ctx ? gc_pool_thread_count( ctx->pool ) : 1;
}

int gc_spawn( GcContext* ctx, int kind, float x, float y, float z, float yaw )
{
	if ( !ctx )
		return -1;
	const GcMechDef* d = ctx_def_by_kind( ctx, kind );
	if ( !d )
		return -1;
	if ( ctx->count == ctx->cap )
	{
		int nc = ctx->cap ? ctx->cap * 2 : 32;
		GcMech** nm = (GcMech**)realloc( ctx->mechs, (size_t)nc * sizeof( GcMech* ) );
		if ( !nm )
			return -1;
		ctx->mechs = nm;
		ctx->cap = nc;
	}
	// 64-byte alignment prevents false sharing between neighboring mechs stepped on separate cores.
	GcMech* m = (GcMech*)gc_alloc_aligned( sizeof( GcMech ), 64 );
	if ( !m )
		return -1;
	gc_mech_init( m, d, gc_v3( (double)x, (double)y, (double)z ), (double)yaw, &ctx->world_cb );
	int idx = ctx->count;
	apply_bench_cfg( ctx, m, idx );
	ctx->mechs[ctx->count++] = m;
	return idx;
}

// Register once or reuse an identical custom descriptor so spawning 1,000 copies doesn't bloat the type table.
int gc_spawn_custom( GcContext* ctx, const GcMechDesc* desc, float x, float y, float z, float yaw )
{
	if ( !ctx || !desc )
		return -1;
	GcMechDef tmp;
	if ( !gc_mechdef_from_desc( desc, 0, &tmp ) )
		return -1;
	int base_n = gc_mechdef_count();
	for ( int i = 0; i < ctx->custom_count; ++i )
	{
		GcMechDef cmp = *ctx->custom_defs[i];
		cmp.kind_id = 0; // ignore assigned slot ID when checking structural equality
		if ( memcmp( &cmp, &tmp, sizeof( GcMechDef ) ) == 0 )
			return gc_spawn( ctx, base_n + i, x, y, z, yaw );
	}
	int kind = gc_register_mech_type( ctx, desc );
	if ( kind < 0 )
		return -1;
	return gc_spawn( ctx, kind, x, y, z, yaw );
}

int gc_respawn( GcContext* ctx, int handle, int kind, float x, float y, float z, float yaw )
{
	if ( !ctx || handle < 0 || handle >= ctx->count )
		return -1;
	const GcMechDef* d = ctx_def_by_kind( ctx, kind );
	if ( !d )
		return -1;
	GcMech* m = ctx->mechs[handle];
	gc_mech_destroy( m );
	gc_mech_init( m, d, gc_v3( (double)x, (double)y, (double)z ), (double)yaw, &ctx->world_cb );
	apply_bench_cfg( ctx, m, handle );
	return handle;
}

int gc_despawn_to( GcContext* ctx, int target_count )
{
	if ( !ctx )
		return 0;
	if ( target_count < 0 )
		target_count = 0;
	while ( ctx->count > target_count )
	{
		int idx = ctx->count - 1;
		gc_mech_destroy( ctx->mechs[idx] );
		gc_free_aligned( ctx->mechs[idx] );
		ctx->mechs[idx] = NULL;
		ctx->count--;
	}
	return ctx->count;
}

int gc_count( const GcContext* ctx )
{
	return ctx ? ctx->count : 0;
}

int gc_floats_per_mech( const GcContext* ctx, int handle )
{
	if ( !ctx || handle < 0 || handle >= ctx->count )
		return 0;
	const GcMech* m = ctx->mechs[handle];
	return GC_HEADER_FLOATS + m->n * ( m->joints_per_leg * 3 + GC_LEG_EXTRA_FLOATS );
}

int gc_info( const GcContext* ctx, int handle, GcMechInfo* out )
{
	if ( !ctx || !out || handle < 0 || handle >= ctx->count )
		return 0;
	const GcMech* m = ctx->mechs[handle];
	const GcMechDef* d = m->d;
	memset( out, 0, sizeof( *out ) );
	out->name = d->name;
	out->blurb = d->blurb;
	out->kind = d->kind_id;
	out->legs = m->n;
	out->joints_per_leg = m->joints_per_leg;
	out->half[0] = (float)d->body_half[0];
	out->half[1] = (float)d->body_half[1];
	out->half[2] = (float)d->body_half[2];
	out->width_count = d->n_segs <= 4 ? d->n_segs : 4;
	for ( int i = 0; i < out->width_count; ++i )
		out->widths[i] = (float)d->widths[i];
	out->mass = (float)d->mass;
	out->climber = d->climber;
	out->faction = d->faction;
	out->wire_max = (float)d->wire_max;
	out->walk_speed = (float)d->speed_walk;
	out->run_speed = (float)d->speed_run;
	out->floats = gc_floats_per_mech( ctx, handle );
	return 1;
}

void gc_set_bench_config( GcContext* ctx, const GcBenchConfig* cfg )
{
	if ( !ctx || !cfg )
		return;
	ctx->bench.max_substep = (float)gc_clamp( (double)cfg->max_substep, 1.0 / 240.0, GC_MAX_FRAME_DT );
	ctx->bench.leg_ik_hz = (float)gc_clamp( (double)cfg->leg_ik_hz, 0.0, 240.0 );
	ctx->bench.probe_hz = (float)gc_clamp( (double)cfg->probe_hz, 1.0, 120.0 );
	ctx->bench.speed_scale = (float)gc_clamp( (double)cfg->speed_scale, 0.0, 3.0 );
	ctx->bench.plane_roll = cfg->plane_roll ? 1 : 0;
	ctx->bench.profile = cfg->profile ? 1 : 0;
	for ( int i = 0; i < ctx->count; ++i )
		apply_bench_cfg( ctx, ctx->mechs[i], i );
}

typedef struct StepJob {
	GcMech** mechs;
	int substeps;
	double sub_dt;
	int profile;
} StepJob;

static void step_mech_worker( int idx, void* user )
{
	const StepJob* job = (const StepJob*)user;
	GcMech* m = job->mechs[idx];
	gc_wq_clear_rays( &m->wq );
	m->events.count = 0;
	memset( m->prof, 0, sizeof( m->prof ) );
	double mt0 = job->profile ? gc_time_sec() : 0.0;
	for ( int s = 0; s < job->substeps; ++s )
	{
		gc_mech_update( m, job->sub_dt );
	}
	m->step_ms = job->profile ? ( ( gc_time_sec() - mt0 ) * 1000.0 ) : 0.0;
}

static void append_events_buf( GcContext* ctx, const float* src, int n_floats )
{
	if ( n_floats <= 0 )
		return;
	if ( ctx->event_floats + n_floats > ctx->event_cap_floats )
	{
		int nc = ctx->event_cap_floats ? ctx->event_cap_floats * 2 : 1024 * GC_EVENT_FLOATS;
		while ( nc < ctx->event_floats + n_floats )
			nc *= 2;
		float* nd = (float*)realloc( ctx->events_buf, (size_t)nc * sizeof( float ) );
		if ( !nd )
			return;
		ctx->events_buf = nd;
		ctx->event_cap_floats = nc;
	}
	memcpy( ctx->events_buf + ctx->event_floats, src, (size_t)n_floats * sizeof( float ) );
	ctx->event_floats += n_floats;
}

static void append_dbg_rays_buf( GcContext* ctx, const float* src, int n_rays )
{
	if ( n_rays <= 0 )
		return;
	int need = n_rays * GC_DEBUG_LINE_FLOATS;
	if ( ctx->dbg_ray_floats + need > ctx->dbg_ray_cap_floats )
	{
		int nc = ctx->dbg_ray_cap_floats ? ctx->dbg_ray_cap_floats * 2 : 1024 * GC_DEBUG_LINE_FLOATS;
		while ( nc < ctx->dbg_ray_floats + need )
			nc *= 2;
		float* nd = (float*)realloc( ctx->dbg_rays_buf, (size_t)nc * sizeof( float ) );
		if ( !nd )
			return;
		ctx->dbg_rays_buf = nd;
		ctx->dbg_ray_cap_floats = nc;
	}
	float* dst = ctx->dbg_rays_buf + ctx->dbg_ray_floats;
	for ( int i = 0; i < n_rays; ++i )
	{
		const float* r = src + i * 7;
		int role = ( r[6] > 0.5f ) ? GC_DBG_WARNING : GC_DBG_LINE_DIM;
		dst[0] = r[0]; dst[1] = r[1]; dst[2] = r[2];
		dst[3] = r[3]; dst[4] = r[4]; dst[5] = r[5];
		dst[6] = (float)role;
		dst[7] = 0.0f;
		dst[8] = 0.0f;
		dst += GC_DEBUG_LINE_FLOATS;
	}
	ctx->dbg_ray_floats += need;
}

void gc_step( GcContext* ctx, float dt_in, const float* inputs, int mech_count )
{
	if ( !ctx || !isfinite( dt_in ) || dt_in <= 0.0f )
		return;
	double dt = fmin( (double)dt_in, GC_MAX_FRAME_DT );
	int n_mechs = ctx->count < mech_count ? ctx->count : mech_count;
	if ( inputs && n_mechs > 0 )
	{
		for ( int i = 0; i < n_mechs; ++i )
			gc_input_set( &ctx->mechs[i]->inp, inputs + i * GC_INPUT_STRIDE );
	}

	ctx->dbg_ray_floats = 0;
	double max_sub = gc_clamp( (double)ctx->bench.max_substep, 1.0 / 240.0, GC_MAX_FRAME_DT );
	int n_sub = (int)ceil( dt / max_sub - 1e-6 );
	if ( n_sub < 1 )
		n_sub = 1;
	double sub = dt / (double)n_sub;
	int prof = ctx->bench.profile;

	double t0 = gc_time_sec();
	if ( ctx->count > 0 )
	{
		StepJob job;
		job.mechs = ctx->mechs;
		job.substeps = n_sub;
		job.sub_dt = sub;
		job.profile = prof;
		gc_pool_parallel_for( ctx->pool, ctx->count, step_mech_worker, &job );
	}

	double total_rays = 0;
	double prof_sum[10] = { 0 };
	double cpu_ms = 0.0;
	int kind_cnt[GC_PROFILE_KINDS] = { 0 };
	int kind_legs[GC_PROFILE_KINDS] = { 0 };
	double kind_ms[GC_PROFILE_KINDS] = { 0 };
	int kind_rays[GC_PROFILE_KINDS] = { 0 };
	int kind_ik[GC_PROFILE_KINDS] = { 0 };

	for ( int i = 0; i < ctx->count; ++i )
	{
		const GcMech* m = ctx->mechs[i];
		// Drain in handle order after the barrier so output streams match 1-thread runs byte-for-byte.
		if ( m->events.count > 0 )
			append_events_buf( ctx, m->events.data, m->events.count * GC_EVENT_FLOATS );
		if ( ctx->debug && m->wq.ray_floats > 0 )
			append_dbg_rays_buf( ctx, m->wq.rays, m->wq.ray_floats / 7 );

		total_rays += m->wq.ray_count;
		ctx->query_stats.ray_calls += m->wq.ray_count;
		ctx->query_stats.push_calls += m->wq.push_count;
		ctx->query_stats.height_calls += m->wq.height_count;
		ctx->query_stats.ray_ms += (float)m->wq.ray_ms;
		ctx->query_stats.push_ms += (float)m->wq.push_ms;
		ctx->query_stats.height_ms += (float)m->wq.height_ms;

		if ( prof )
		{
			cpu_ms += m->step_ms;
			for ( int s = 0; s < 10; ++s )
				prof_sum[s] += m->prof[s];
			int kid = m->d->kind_id;
			if ( 0 <= kid && kid < GC_PROFILE_KINDS )
			{
				kind_cnt[kid] += 1;
				kind_legs[kid] += m->n;
				kind_ms[kid] += m->step_ms;
				kind_rays[kid] += m->wq.ray_count;
				kind_ik[kid] += (int)m->prof[8];
			}
		}
	}

	double wall_ms = ( gc_time_sec() - t0 ) * 1000.0;
	ctx->last_wall_ms = (float)wall_ms;
	ctx->last_cpu_ms = (float)cpu_ms;
	ctx->last_rays = (int)total_rays;

	if ( prof )
	{
		float prev_poses_ms = ctx->prof_buf[13];
		double stage_sum_ms = 0.0;
		for ( int s = 0; s < 8; ++s )
			stage_sum_ms += prof_sum[s] * 1000.0;
		// Normalize summed per-worker stage timers into wall-clock milliseconds for UI budgeting.
		double wall_scale = ( stage_sum_ms > wall_ms && stage_sum_ms > 1e-6 ) ? ( wall_ms / stage_sum_ms ) : 1.0;
		double kind_scale = ( cpu_ms > wall_ms && cpu_ms > 1e-6 ) ? ( wall_ms / cpu_ms ) : 1.0;

		ctx->prof_buf[0] = (float)wall_ms;
		for ( int s = 0; s < 8; ++s )
			ctx->prof_buf[1 + s] = (float)( prof_sum[s] * 1000.0 * wall_scale );
		ctx->prof_buf[9] = (float)n_sub;
		ctx->prof_buf[10] = (float)prof_sum[8];
		ctx->prof_buf[11] = (float)prof_sum[9];
		ctx->prof_buf[12] = (float)total_rays;
		ctx->prof_buf[13] = prev_poses_ms;

		int total_legs = 0;
		for ( int k = 0; k < GC_PROFILE_KINDS; ++k )
			total_legs += kind_legs[k];
		ctx->prof_buf[14] = (float)total_legs;
		ctx->prof_buf[15] = (float)ctx->count;

		for ( int k = 0; k < GC_PROFILE_KINDS; ++k )
		{
			int base = GC_PROFILE_HEADER + k * GC_PROFILE_KIND_STRIDE;
			ctx->prof_buf[base] = (float)kind_cnt[k];
			ctx->prof_buf[base + 1] = (float)kind_legs[k];
			ctx->prof_buf[base + 2] = (float)( kind_ms[k] * kind_scale );
			ctx->prof_buf[base + 3] = (float)kind_rays[k];
			ctx->prof_buf[base + 4] = (float)kind_ik[k];
		}
	}
}

typedef struct PoseJob {
	GcMech* const* mechs;
	const int* offsets;
	float* out;
	int max_floats;
} PoseJob;

static void pack_pose_worker( int idx, void* user )
{
	const PoseJob* job = (const PoseJob*)user;
	int off = job->offsets[idx];
	if ( off < job->max_floats )
		gc_mech_pack( job->mechs[idx], job->out + off, job->max_floats - off );
}

int gc_poses( GcContext* ctx, float* out_buf, int max_floats )
{
	if ( !ctx || !out_buf || max_floats <= 0 || ctx->count <= 0 )
		return 0;
	int prof = ctx->bench.profile;
	double t0 = prof ? gc_time_sec() : 0.0;
	int offsets_stack[256];
	int* offsets = ctx->count <= 256 ? offsets_stack : (int*)malloc( (size_t)ctx->count * sizeof( int ) );
	int total = 0;
	for ( int i = 0; i < ctx->count; ++i )
	{
		offsets[i] = total;
		total += gc_floats_per_mech( ctx, i );
	}
	if ( ctx->count >= 16 && gc_get_threads( ctx ) > 1 )
	{
		PoseJob job = { ctx->mechs, offsets, out_buf, max_floats };
		gc_pool_parallel_for( ctx->pool, ctx->count, pack_pose_worker, &job );
	}
	else
	{
		for ( int i = 0; i < ctx->count; ++i )
		{
			if ( offsets[i] >= max_floats )
				break;
			gc_mech_pack( ctx->mechs[i], out_buf + offsets[i], max_floats - offsets[i] );
		}
	}
	if ( offsets != offsets_stack )
		free( offsets );
	if ( prof )
		ctx->prof_buf[13] = (float)( ( gc_time_sec() - t0 ) * 1000.0 );
	return total < max_floats ? total : max_floats;
}

int gc_events( GcContext* ctx, float* out_buf, int max_floats )
{
	if ( !ctx || !out_buf || max_floats <= 0 || ctx->event_floats <= 0 )
		return 0;
	int n = ctx->event_floats < max_floats ? ctx->event_floats : max_floats;
	memcpy( out_buf, ctx->events_buf, (size_t)n * sizeof( float ) );
	ctx->event_floats = 0;
	return n;
}

int gc_debug_lines( const GcContext* ctx, int handle, float* out_buf, int max_floats )
{
	if ( !ctx || !out_buf || max_floats <= 0 )
		return 0;
	int n = 0;
	if ( handle >= 0 && handle < ctx->count )
		n = gc_mech_debug_lines( ctx->mechs[handle], out_buf, max_floats );
	if ( ctx->debug && ctx->dbg_ray_floats > 0 && n < max_floats )
	{
		int rem = max_floats - n;
		int copy_n = ctx->dbg_ray_floats < rem ? ctx->dbg_ray_floats : rem;
		memcpy( out_buf + n, ctx->dbg_rays_buf, (size_t)copy_n * sizeof( float ) );
		n += copy_n;
	}
	return n;
}

void gc_set_debug( GcContext* ctx, int on )
{
	if ( !ctx )
		return;
	ctx->debug = on ? 1 : 0;
	for ( int i = 0; i < ctx->count; ++i )
		ctx->mechs[i]->wq.record = ctx->debug;
}

int gc_stats( const GcContext* ctx, int handle, GcMechStats* out )
{
	if ( !ctx || !out || handle < 0 || handle >= ctx->count )
		return 0;
	gc_mech_stats_fill( ctx->mechs[handle], ctx->last_wall_ms, ctx->last_rays, out );
	return 1;
}

int gc_profile_data( const GcContext* ctx, float* out_buf, int max_floats )
{
	if ( !ctx || !out_buf || max_floats <= 0 )
		return 0;
	int n = GC_PROFILE_FLOATS < max_floats ? GC_PROFILE_FLOATS : max_floats;
	memcpy( out_buf, ctx->prof_buf, (size_t)n * sizeof( float ) );
	return n;
}

void gc_reset_query_stats( GcContext* ctx )
{
	if ( !ctx )
		return;
	memset( &ctx->query_stats, 0, sizeof( ctx->query_stats ) );
}

GcQueryStats gc_get_query_stats( const GcContext* ctx )
{
	if ( !ctx )
	{
		GcQueryStats z;
		memset( &z, 0, sizeof( z ) );
		return z;
	}
	return ctx->query_stats;
}

float gc_last_cpu_ms( const GcContext* ctx )
{
	return ctx ? ctx->last_cpu_ms : 0.0f;
}
