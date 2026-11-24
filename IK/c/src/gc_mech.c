#include "gc_internal.h"
#include "gc_thread.h"

// Body-level locomotion controller: drive physics, jump/mantle/air state
// machine, slide momentum, hull orientation, and packed pose export.

// Unpack raw caller control channels in documented order into GcInput.
void gc_input_set( GcInput* inp, const float* vals )
{
	double* dst[12] = {
		&inp->fwd, &inp->strafe, &inp->turn, &inp->run, &inp->slow, &inp->crouch,
		&inp->jump, &inp->drift, &inp->climb, &inp->impact, &inp->gait, &inp->aim
	};
	for ( int i = 0; i < 12; ++i )
	{
		double x = (double)vals[i];
		if ( !isfinite( x ) )
			x = 0.0;
		else if ( i < 3 )
			x = gc_clamp( x, -1.0, 1.0 );
		else if ( i == 11 )
			x = gc_clamp( x, -GC_AIM_MAX, GC_AIM_MAX );
		*dst[i] = x;
	}
}

static double rng_uniform( uint64_t* state, double lo, double hi )
{
	uint64_t x = *state;
	x ^= x >> 12;
	x ^= x << 25;
	x ^= x >> 27;
	*state = x;
	uint64_t r = x * 0x2545F4914F6CDD1DULL;
	double u = (double)( r >> 11 ) * ( 1.0 / 9007199254740992.0 );
	return lo + ( hi - lo ) * u;
}

int gc_mech_climbing_mode( const GcMech* m )
{
	return ( m->d->climber && m->climb_enabled ) ? 1 : 0;
}

GcV3 gc_mech_probe_up( const GcMech* m )
{
	return gc_mech_climbing_mode( m ) ? m->frame.up : GC_WORLD_UP;
}

GcV3 gc_mech_heading_fwd( const GcMech* m )
{
	GcV3 f = m->frame.fwd;
	double fx, fz;
	if ( fabs( f.y ) > 0.7 )
	{
		GcV3 r = m->frame.right;
		fx = r.z;
		fz = -r.x;
	}
	else
	{
		fx = f.x;
		fz = f.z;
	}
	double l2 = fx * fx + fz * fz;
	if ( l2 < 1e-18 )
		return gc_v3( 0.0, 0.0, 1.0 );
	double inv = 1.0 / sqrt( l2 );
	return gc_v3( fx * inv, 0.0, fz * inv );
}

GcFrame gc_mech_support_frame( const GcMech* m )
{
	if ( gc_mech_climbing_mode( m ) )
		return m->frame;
	GcV3 f = gc_mech_heading_fwd( m );
	GcFrame fr;
	fr.fwd = f;
	fr.up = GC_WORLD_UP;
	fr.right = gc_v3( -f.z, 0.0, f.x );
	fr.is_local = 0;
	return fr;
}

double gc_mech_ride_eff( const GcMech* m )
{
	double slide_drop = fmin( 0.26, 0.72 * m->jump_sink_max ) * m->drift_amt;
	double r = m->ride * ( 1.0 - fmax( 0.38 * m->crouch, slide_drop ) );
	if ( m->mode == GC_MODE_SLOW )
		r *= 0.9;
	if ( m->state == GC_STATE_CROUCHJUMP )
		r *= ( 1.0 - m->jump_sink );
	r *= ( 1.0 - m->absorb ) * ( 1.0 - 0.15 * m->brace );
	return r;
}

int gc_mech_wire_active( const GcMech* m )
{
	return ( m->state == GC_STATE_MANTLE && m->mantle.active && m->mantle.wire && !m->mantle.landed ) ? 1 : 0;
}

void gc_mech_emit( GcMech* m, int kind, GcV3 p, GcV3 n, double strength, GcV3 extra )
{
	if ( m->events.count >= m->events.cap )
	{
		int nc = m->events.cap ? m->events.cap * 2 : 64;
		float* nd = (float*)realloc( m->events.data, (size_t)nc * GC_EVENT_FLOATS * sizeof( float ) );
		if ( !nd )
			return;
		m->events.data = nd;
		m->events.cap = nc;
	}
	float* dst = m->events.data + m->events.count * GC_EVENT_FLOATS;
	dst[0] = (float)kind;
	dst[1] = (float)p.x;
	dst[2] = (float)p.y;
	dst[3] = (float)p.z;
	dst[4] = (float)n.x;
	dst[5] = (float)n.y;
	dst[6] = (float)n.z;
	dst[7] = (float)strength;
	dst[8] = (float)extra.x;
	dst[9] = (float)extra.y;
	dst[10] = (float)extra.z;
	m->events.count++;
}

static double fold_ride( GcMech* m )
{
	double lowest = m->ride;
	for ( int k = 1; k < 24; ++k )
	{
		double r = m->ride * ( 1.0 - 0.025 * (double)k );
		for ( int i = 0; i < m->n; ++i )
		{
			GcLeg* l = &m->legs[i];
			GcV3 jl[GC_MAX_JOINTS];
			gc_legik_solve( &l->ik_probe, l->mount_local, &GC_LOCAL_FRAME, gc_leg_rest_local( l, r, 1.0 ), 0.0, jl );
			if ( l->ik_probe.err > 0.01 )
				return lowest;
		}
		lowest = r;
	}
	return lowest;
}

// Precompute safe pitch-down / pitch-up aim angles where all legs remain inside
// 91% reach and pass IK while lifting the chassis just enough for nose clearance.
static void compute_pitch_limits( GcMech* m, double* out_dn, double* out_up )
{
	double cap = m->d->look_tilt;
	double hy = m->d->body_half[1], hz = m->d->body_half[2];
	GcFrame level;
	gc_frame_init( &level, gc_v3( 0.0, 0.0, 1.0 ), GC_WORLD_UP );
	double res[2] = { 0.0, 0.0 };
	static const double sgns[2] = { -1.0, 1.0 };
	int max_deg = (int)ceil( cap );
	for ( int si = 0; si < 2; ++si )
	{
		double best = 0.0;
		for ( int deg = 1; deg <= max_deg; ++deg )
		{
			double a = fmin( (double)deg, cap );
			double rad = a * sgns[si] * GC_DEG2RAD;
			GcV3 up = gc_rotate_axis( GC_WORLD_UP, level.right, rad );
			GcFrame fr;
			gc_frame_init( &fr, gc_v3( 0.0, 0.0, 1.0 ), up );
			double ride = fmax( m->ride, hz * fabs( gc_v3_dot( fr.fwd, GC_WORLD_UP ) ) +
										 hy * fabs( gc_v3_dot( fr.up, GC_WORLD_UP ) ) + 0.06 );
			int ok = 1;
			double r_tests[2] = { ride, ride * 0.92 };
			for ( int ri = 0; ri < 2 && ok; ++ri )
			{
				for ( int li = 0; li < m->n; ++li )
				{
					GcLeg* l = &m->legs[li];
					GcV3 foot_l = gc_frame_to_local( &fr, gc_frame_to_world( &level, gc_leg_rest_local( l, r_tests[ri], 1.0 ) ) );
					if ( gc_v3_dist( foot_l, l->mount_local ) > 0.91 * l->reach ||
						 !gc_legik_feasible( &l->ik_probe, l->mount_local, &GC_LOCAL_FRAME, foot_l, 6.0 ) )
					{
						ok = 0;
						break;
					}
				}
			}
			if ( !ok )
				break;
			best = a;
		}
		res[si] = best * GC_DEG2RAD;
	}
	*out_dn = res[0];
	*out_up = res[1];
}

static double sink_room( GcMech* m )
{
	double r0 = m->jump_r0;
	const GcFrame* fr = &m->frame;
	GcV3 pos = m->pos;
	double dropped = fmax( 0.0, m->jump_y0 - gc_v3_dot( pos, gc_mech_probe_up( m ) ) );
	double step = 0.05 * m->ride;
	double room = fmax( 0.0, m->jump_sink_max * r0 - dropped );
	for ( int i = 0; i < m->n; ++i )
	{
		GcLeg* l = &m->legs[i];
		if ( !l->planted )
			continue;
		GcV3 fl = gc_frame_to_local( fr, gc_v3_sub( l->foot, pos ) );
		double d = 0.0;
		while ( d + step <= room )
		{
			GcV3 jl[GC_MAX_JOINTS];
			gc_legik_solve( &l->ik_probe, l->mount_local, &GC_LOCAL_FRAME,
							gc_v3_add( fl, gc_v3( 0.0, d + step, 0.0 ) ), 0.0, jl );
			if ( l->ik_probe.err > 0.01 )
				break;
			d += step;
		}
		if ( d < room )
			room = d;
	}
	return ( dropped + 0.85 * room ) / r0;
}

static double crouch_time( const GcMech* m )
{
	return ( 0.12 + 0.01 * m->d->jump_speed ) * sqrt( m->size );
}

static double target_speed( const GcMech* m )
{
	double sp = ( m->mode == GC_MODE_SLOW ) ? m->d->speed_slow : ( m->mode == GC_MODE_RUN ? m->d->speed_run : m->d->speed_walk );
	return sp * m->speed_scale;
}

static double crouch_cap( const GcMech* m )
{
	return 0.5 * m->d->speed_slow;
}

static void place_feet_initial( GcMech* m )
{
	const GcFrame* fr = &m->frame;
	for ( int i = 0; i < m->n; ++i )
	{
		GcLeg* l = &m->legs[i];
		l->mount_world = gc_v3_add( m->pos, gc_frame_to_world( fr, l->mount_local ) );
		l->ideal_now = gc_v3_add( m->pos, gc_frame_to_world( fr, gc_leg_rest_local( l, m->ride, 1.0 ) ) );
		l->ideal_pred = l->ideal_now;
		GcV3 fhp, fhn;
		if ( gc_find_foothold( m, l, l->ideal_now, 0.0, &fhp, &fhn ) )
		{
			l->foot = fhp;
			l->foot_normal = fhn;
			l->planted = 1;
		}
		else
		{
			l->foot = l->ideal_now;
			l->planted = 0;
			l->searching = 1;
		}
		gc_solve_leg( m, l, 1.0 / 60.0 );
	}
}

void gc_mech_init( GcMech* m, const GcMechDef* d, GcV3 ground_pos, double yaw, const GcWorldCallbacks* cb )
{
	memset( m, 0, sizeof( *m ) );
	m->d = d;
	m->size = d->size;
	gc_wq_init( &m->wq, cb );
	gc_frame_init( &m->frame, gc_v3( sin( yaw ), 0.0, cos( yaw ) ), GC_WORLD_UP );
	m->ride = d->ride;
	m->pos = gc_v3( ground_pos.x, ground_pos.y + m->ride, ground_pos.z );
	m->n = d->n_legs;
	m->biped = d->biped;

	double reach_sum = 0.0, chord_sum = 0.0;
	for ( int i = 0; i < m->n; ++i )
	{
		gc_leg_init( &m->legs[i], i, &d->legs[i], d, 1.0 );
		reach_sum += m->legs[i].reach;
		double c = 1.15 * m->legs[i].reach;
		if ( m->legs[i].ik.mode == GC_PLANE_RADIAL && m->legs[i].ik.coxa_max < 1.5 )
		{
			double rr = hypot( m->legs[i].rest_xz[0] - m->legs[i].mount_local.x,
							   m->legs[i].rest_xz[1] - m->legs[i].mount_local.z );
			c = fmin( c, 1.85 * rr * sin( m->legs[i].ik.coxa_max ) );
		}
		chord_sum += c;
	}
	m->reach_avg = reach_sum / (double)m->n;
	m->chord_avg = chord_sum / (double)m->n;

	const GcGait* wg = &d->gaits[d->gait_walk_idx];
	m->v_walk = m->reach_avg / fmax( 0.05, wg->duty * wg->cycle_min * sqrt( m->size ) );
	const GcGait* sg = &d->gaits[d->gait_slow_idx];
	m->v_slow = m->reach_avg / fmax( 0.05, sg->duty * sg->cycle_min * sqrt( m->size ) );
	m->jump_sink_max = fmin( 0.45, fmax( 0.0, 1.0 - 1.06 * fold_ride( m ) / m->ride ) );
	compute_pitch_limits( m, &m->look_pitch_dn, &m->look_pitch_up );

	m->joints_per_leg = 1 + ( d->coxa > 0.0 ? 1 : 0 ) + d->n_segs;
	m->mode = GC_MODE_WALK;
	m->gait_override = -1;
	m->gait_idx = d->gait_walk_idx;
	gc_blend_init( &m->blend, m->gait_idx, &d->gaits[m->gait_idx] );
	m->gait = m->blend.eff;
	gc_situation_init( &m->situation );
	gc_selector_init( &m->selector, m );
	m->duty = m->gait.duty;
	for ( int i = 0; i < m->n; ++i )
		m->legs[i].offset = m->gait.offsets[i];
	m->cycle = 1.0;
	m->state = GC_STATE_GROUND;
	m->climb_enabled = d->climber;
	m->slide_dir = gc_v3( 0.0, 0.0, 1.0 );
	m->ground_n = GC_WORLD_UP;
	m->land_right_n = GC_WORLD_UP;
	m->support = 1.0;
	m->rng_state = (uint64_t)( d->kind_id + 1 ) * 0x9E3779B97F4A7C15ULL + 0xBF58476D1CE4E5B9ULL;
	m->ceil_t = 1.0;
	m->pitch_cap = 1.0;
	m->sup_t = 1.0;
	m->mantle_t = 0.05;
	m->probe_period = 0.05;
	m->speed_scale = 1.0;
	m->plane_roll_enabled = 1;

	double hx = d->body_half[0], hy = d->body_half[1], hz = d->body_half[2];
	if ( d->climber )
	{
		m->col_r = hy * 1.15;
		m->n_col_offs = 1;
		m->col_offs[0] = 0.0;
		m->col_lat = 0.0;
	}
	else
	{
		double m_h = fmin( hx, fmin( hz, hy * 1.3 ) );
		m->col_r = m_h * 0.95;
		double off = fmax( 0.0, hz - m->col_r );
		int n = (int)ceil( off / fmax( m->col_r, 1e-3 ) );
		if ( n < 1 ) n = 1;
		if ( off > 0.05 && n + 1 <= GC_MAX_COL_OFFS )
		{
			m->n_col_offs = n + 1;
			for ( int i = 0; i <= n; ++i )
				m->col_offs[i] = off * ( 1.0 - 2.0 * (double)i / (double)n );
		}
		else
		{
			m->n_col_offs = 1;
			m->col_offs[0] = 0.0;
		}
		m->col_lat = fmax( 0.0, hx - m->col_r );
	}
	place_feet_initial( m );
}

void gc_mech_destroy( GcMech* m )
{
	if ( !m )
		return;
	gc_wq_destroy( &m->wq );
	free( m->events.data );
	memset( m, 0, sizeof( *m ) );
}

static void compute_drive_intent( GcMech* m )
{
	const GcInput* inp = &m->inp;
	if ( inp->run > 0.5 )
		m->mode = GC_MODE_RUN;
	else if ( inp->slow > 0.5 )
		m->mode = GC_MODE_SLOW;
	else
		m->mode = GC_MODE_WALK;

	GcFrame heading = gc_mech_support_frame( m );
	GcV3 mv = gc_v3_add( gc_v3_mul( heading.fwd, inp->fwd ), gc_v3_mul( heading.right, inp->strafe * 0.75 ) );
	if ( gc_v3_len_sq( mv ) > 1.0 )
		mv = gc_v3_norm( mv, gc_v3_zero() );
	GcV3 v_cmd = gc_v3_mul( mv, target_speed( m ) );
	double ccap = crouch_cap( m );
	if ( m->crouch > 0.5 && gc_v3_len( v_cmd ) > ccap )
		v_cmd = gc_v3_mul( gc_v3_norm( v_cmd, gc_v3_zero() ), ccap );
	m->v_cmd = v_cmd;
	if ( gc_v3_len_sq( v_cmd ) > 0.01 )
		m->lateral_ratio = fabs( gc_v3_dot( v_cmd, heading.right ) ) / fmax( fabs( gc_v3_dot( v_cmd, heading.fwd ) ), 0.1 );
	else
		m->lateral_ratio = 0.0;
	m->v_desired = gc_v3_len( v_cmd );
}

static double duty_for( const GcMech* m, const GcGait* g )
{
	double duty = g->duty;
	if ( g->has_duty_fast )
	{
		const GcMechDef* d = m->d;
		double v0 = 0.75 * m->v_walk;
		double f = gc_clamp( ( m->speed - v0 ) / fmax( 1.0, d->speed_walk - v0 ), 0.0, 1.0 );
		duty = duty + ( g->duty_fast - duty ) * f;
		if ( !( d->bound * g->leap >= 0.015 ) || m->braking || m->calm_gait )
		{
			if ( duty < 0.5 ) duty = 0.5;
		}
		else
		{
			double b = d->bound * g->leap;
			double vl = m->reach_avg / fmax( 0.05, fmax( duty, 0.25 ) * g->cycle_min * sqrt( m->size ) );
			double k = gc_clamp( ( m->speed - 0.3 * vl ) / ( 0.7 * vl ), 0.0, 1.0 );
			if ( b * k <= 0.016 && duty < 0.5 )
				duty = 0.5;
		}
	}
	return duty;
}

static void select_mode( GcMech* m, double dt )
{
	const GcMechDef* d = m->d;
	if ( m->state == GC_STATE_GROUND )
	{
		gc_situation_update( &m->situation, m, dt );
		int has_cap = 0;
		double cap = 0.0;
		int gidx = gc_selector_select( &m->selector, m, dt, &has_cap, &cap );
		m->has_speed_cap = has_cap;
		m->speed_cap = cap;
		if ( gidx != m->gait_idx )
		{
			m->gait_idx = gidx;
			const GcGait* dst_g = &d->gaits[gidx];
			double dst_T = gc_clamp( dst_g->stride * m->size / fmax( fmax( m->v_desired, m->speed ), 1e-3 ),
									 dst_g->cycle_min * sqrt( m->size ),
									 dst_g->cycle_max * sqrt( m->size ) );
			gc_blend_start( &m->blend, gidx, dst_g, gc_clamp( dst_T, 0.28, 0.50 ) );
		}
	}
	else
	{
		m->selector.low_t = 0.0;
		m->selector.prev_v = m->speed;
		m->selector.accel_v = 0.0;
	}
	gc_blend_update( &m->blend, dt );
	for ( int i = 0; i < m->n; ++i )
		m->legs[i].offset = m->blend.eff.offsets[i];
	double k = gc_damp( 5.0, dt );
	double duty = duty_for( m, &m->blend.dst );
	if ( m->blend.t < 1.0 )
	{
		double src_d = duty_for( m, &m->blend.src );
		duty = src_d + ( duty - src_d ) * m->blend.s;
		if ( m->selector.has_hold_v && m->state == GC_STATE_GROUND )
		{
			double hold = m->selector.hold_v * ( 1.0 + 0.8 * m->blend.s );
			m->speed_cap = m->has_speed_cap ? fmin( m->speed_cap, hold ) : hold;
			m->has_speed_cap = 1;
		}
	}
	m->duty += ( duty - m->duty ) * k;
	gc_blend_fit_support( &m->blend, m->duty );
	m->gait = m->blend.eff;
}

static void cycle_gait_override( GcMech* m )
{
	int gc = m->d->gait_count;
	if ( m->gait_override < 0 )
		m->gait_override = 0;
	else if ( m->gait_override + 1 >= gc )
		m->gait_override = -1;
	else
		m->gait_override++;
}

static void mech_push( GcMech* m, GcV3 dv, int torque )
{
	if ( m->state == GC_STATE_GROUND )
	{
		m->slide_vel = gc_v3_add( m->slide_vel, gc_v3( dv.x, 0.0, dv.z ) );
		m->vel = gc_v3_add( m->vel, dv );
		if ( torque )
		{
			GcV3 dv_h = gc_v3( dv.x, 0.0, dv.z );
			double dhl = gc_v3_len( dv_h );
			if ( dhl > 0.2 )
			{
				GcV3 axis = gc_v3_mul( gc_v3_cross( GC_WORLD_UP, dv_h ), 1.0 / dhl );
				m->ang_vel = gc_v3_add( m->ang_vel, gc_v3_mul( axis, gc_clamp( dhl * 0.45 / sqrt( m->d->mass ), 0.0, 2.6 ) ) );
			}
		}
	}
	else
	{
		m->vel = gc_v3_add( m->vel, dv );
	}
}

static void trigger_impact( GcMech* m )
{
	double a = rng_uniform( &m->rng_state, 0.0, GC_TAU );
	GcV3 axis = gc_v3( cos( a ), 0.0, sin( a ) );
	m->ang_vel = gc_v3_add( m->ang_vel, gc_v3_mul( axis, 2.4 / sqrt( m->d->mass ) ) );
	GcV3 push = gc_v3_mul( gc_v3( sin( a ), 0.0, -cos( a ) ), 4.0 / sqrt( m->d->mass ) );
	mech_push( m, push, 0 );
	m->brace = 1.0;
	gc_mech_emit( m, GC_EV_IMPACT, m->pos, GC_WORLD_UP, 1.0, gc_v3_zero() );
}

static int collide( GcMech* m )
{
	const GcFrame* fr = &m->frame;
	int belly_hit = 0;
	for ( int i = 0; i < m->n_col_offs; ++i )
	{
		GcV3 c = gc_v3_add( m->pos, gc_v3_mul( fr->fwd, m->col_offs[i] ) );
		GcV3 c2, n;
		if ( gc_world_sphere_push( &m->wq, c, m->col_r, &c2, &n ) )
		{
			GcV3 disp = gc_v3_sub( c2, c );
			double dlen = gc_v3_len( disp );
			if ( n.y > 0.65 || ( dlen > 1e-5 && disp.y > 0.65 * dlen ) )
				belly_hit = 1;
			m->pos = gc_v3_add( m->pos, disp );
			if ( dlen > 1e-5 )
			{
				GcV3 dn = gc_v3_mul( disp, 1.0 / dlen );
				double vd = gc_v3_dot( m->vel, dn );
				if ( vd < 0.0 )
					m->vel = gc_v3_sub( m->vel, gc_v3_mul( dn, vd ) );
			}
			double vn = gc_v3_dot( m->vel, n );
			if ( vn < 0.0 )
				m->vel = gc_v3_sub( m->vel, gc_v3_mul( n, vn ) );
		}
	}
	if ( m->col_lat > 0.03 )
	{
		double lat_offs[2];
		int n_lat = 1;
		lat_offs[0] = m->col_offs[0];
		if ( m->n_col_offs > 1 )
		{
			lat_offs[1] = m->col_offs[m->n_col_offs - 1];
			n_lat = 2;
		}
		static const double sgns[2] = { -1.0, 1.0 };
		for ( int li = 0; li < n_lat; ++li )
		{
			for ( int si = 0; si < 2; ++si )
			{
				GcV3 c = gc_v3_add( gc_v3_add( m->pos, gc_v3_mul( fr->fwd, lat_offs[li] ) ),
									gc_v3_mul( fr->right, sgns[si] * m->col_lat ) );
				GcV3 c2, n;
				if ( gc_world_sphere_push( &m->wq, c, m->col_r, &c2, &n ) )
				{
					double dx = c2.x - c.x, dy = c2.y - c.y, dz = c2.z - c.z;
					double hl = hypot( dx, dz );
					if ( hl > 1e-5 && ( fabs( n.y ) < 0.55 || hl > fabs( dy ) * 1.15 ) )
					{
						GcV3 disp = gc_v3( dx, 0.0, dz );
						m->pos = gc_v3_add( m->pos, disp );
						GcV3 dn = gc_v3_mul( disp, 1.0 / hl );
						double vd = gc_v3_dot( m->vel, dn );
						if ( vd < 0.0 )
							m->vel = gc_v3_sub( m->vel, gc_v3_mul( dn, vd ) );
					}
				}
			}
		}
	}
	return belly_hit;
}

static int support_error( GcMech* m, double* out_err )
{
	GcV3 pts[GC_MAX_LEGS];
	double wts[GC_MAX_LEGS];
	int np = 0;
	for ( int i = 0; i < m->n; ++i )
	{
		const GcLeg* l = &m->legs[i];
		if ( l->planted )
		{
			pts[np] = l->foot;
			wts[np] = 1.0;
			np++;
		}
		else if ( l->swing.active && l->swing.has_ground )
		{
			pts[np] = l->swing.target;
			wts[np] = 0.35;
			np++;
		}
	}
	if ( np == 0 )
	{
		m->has_plane_n_local = 0;
		return 0;
	}
	GcFrame sf = gc_mech_support_frame( m );
	GcV3 local[GC_MAX_LEGS];
	for ( int i = 0; i < np; ++i )
		local[i] = gc_frame_to_local( &sf, gc_v3_sub( pts[i], m->pos ) );
	GcV3 c, n;
	int fp = gc_fit_plane( local, wts, np, &c, &n );
	m->has_plane_n_local = ( fp >= 2 ) ? 1 : 0;
	if ( fp >= 2 )
	{
		m->plane_n_local = n;
		double ws = 0.0, acc = 0.0;
		for ( int i = 0; i < np; ++i )
		{
			double d = gc_v3_dot( gc_v3_sub( local[i], c ), n );
			acc += wts[i] * d * d;
			ws += wts[i];
		}
		if ( ws == 0.0 ) ws = 1.0;
		m->foot_rms = sqrt( acc / ws );
	}
	else
	{
		m->foot_rms = 0.0;
	}
	double plane_y = ( fp < 2 || n.y < 0.3 ) ? c.y : ( c.y + ( n.x * c.x + n.z * c.z ) / n.y );
	double err = plane_y + gc_mech_ride_eff( m );
	if ( !gc_mech_climbing_mode( m ) && ( fabs( m->inp.aim ) > 1e-3 || fabs( m->look_pitch ) > 1e-3 ) )
	{
		double hx = m->d->body_half[0], hy = m->d->body_half[1], hz = m->d->body_half[2];
		GcV3 g_up = ( fp >= 2 ) ? gc_v3_norm( gc_frame_to_world( &sf, n ), GC_WORLD_UP ) : GC_WORLD_UP;
		double drop = hz * fabs( gc_v3_dot( m->frame.fwd, g_up ) ) +
					  hy * fabs( gc_v3_dot( m->frame.up, g_up ) ) +
					  hx * fabs( gc_v3_dot( m->frame.right, g_up ) );
		double alt = plane_y + ( drop + 0.06 ) / fmax( 0.5, g_up.y );
		if ( alt > err ) err = alt;
	}
	GcV3 up = sf.up;
	double want = gc_mech_ride_eff( m ) * 0.8;
	if ( m->sup_t >= m->probe_period - 1e-6 )
	{
		m->sup_t = fmod( m->sup_t, m->probe_period );
		m->has_sup_h = gc_world_raycast( &m->wq, m->pos, gc_v3_mul( up, -1.0 ), want, &m->sup_h );
		m->has_sup_ha = 0;
		if ( !gc_mech_climbing_mode( m ) && m->state == GC_STATE_GROUND && m->speed > 0.5 )
		{
			GcV3 v_dir = gc_v3_norm( gc_project_plane( m->vt, up ), gc_v3_zero() );
			if ( gc_v3_len_sq( v_dir ) > 0.5 )
			{
				double lead = gc_clamp( m->speed * 0.10, 0.35, m->d->body_half[2] + 0.35 * m->reach_avg );
				double sm = m->d->step_max;
				GcV3 gn = m->ground_n;
				double slope_dz = -( ( v_dir.x * lead ) * gn.x + ( v_dir.z * lead ) * gn.z ) / fmax( 0.5, gn.y );
				double probe_top = sm + 0.5 + fmax( 0.0, slope_dz );
				GcV3 ahead = gc_v3_add( m->pos, gc_v3_mul( v_dir, lead ) );
				GcHit h_a;
				if ( gc_world_raycast( &m->wq, gc_v3_add( ahead, gc_v3_mul( up, probe_top ) ),
									   gc_v3_mul( up, -1.0 ), probe_top + gc_mech_ride_eff( m ) + 0.5, &h_a ) &&
					 gc_v3_dot( h_a.normal, up ) > 0.64 )
				{
					m->has_sup_ha = 1;
					m->sup_ha = h_a;
					m->sup_ha_slope_dz = slope_dz;
				}
			}
		}
	}
	if ( m->has_sup_h )
		err = fmax( err, want - m->sup_h.t );
	if ( m->has_sup_ha && !gc_mech_climbing_mode( m ) && m->state == GC_STATE_GROUND && m->speed > 0.5 )
	{
		const GcHit* h_a = &m->sup_ha;
		double slope_dz = m->sup_ha_slope_dz;
		GcV3 gn = m->ground_n;
		double plane_ref = ( gc_v3_dot( m->pos, up ) - gc_mech_ride_eff( m ) ) + slope_dz;
		double rise = gc_v3_dot( h_a->point, up ) - plane_ref;
		if ( ( gc_v3_dot( h_a->normal, gn ) < 0.985 && rise > 0.04 && rise <= m->d->step_max + 0.65 ) ||
			 ( h_a->normal.y >= 0.98 && rise > 0.25 && rise <= m->d->step_max + 0.05 ) )
		{
			double ahead_err = ( gc_v3_dot( h_a->point, up ) + 0.88 * gc_mech_ride_eff( m ) ) - gc_v3_dot( m->pos, up );
			if ( ahead_err > err )
			{
				double w_a = gc_clamp( ( m->speed - 0.5 ) / 4.0, 0.0, 0.65 );
				err = err + ( ahead_err - err ) * w_a;
			}
		}
	}
	*out_err = err;
	return 1;
}

// Inspect planted and about-to-land feet to measure COM support margin.
// When resting or near a cliff edge outside the support polygon (and not
// braced by a nearby wall), return an outward push direction for emergency step
// recovery; returns 1.0 when grounded balance is healthy.
static double evaluate_balance( GcMech* m, double dt )
{
	m->has_balance_out = 0;
	m->balance_excess = 0.0;
	int any_edge = 0;
	for ( int i = 0; i < m->n; ++i )
		if ( m->legs[i].searching || m->legs[i].void_mark > 0.0 ) { any_edge = 1; break; }
	int in_fast_gallop = ( gc_galloping_fast( m, 1 ) && ( m->flight_t > 0.0 || !any_edge ) ) ? 1 : 0;
	if ( m->biped || m->on_wall || gc_mech_climbing_mode( m ) || m->drifting ||
		 m->state != GC_STATE_GROUND || m->flight_t > 0.0 || in_fast_gallop || gc_mech_wire_active( m ) )
	{
		m->support = 1.0;
		m->hull_n = 0;
		m->unbal_t = 0.0;
		return 1.0;
	}

	GcV2 pts2[GC_MAX_HULL_PTS];
	double ys[GC_MAX_HULL_PTS];
	int np = 0;
	for ( int i = 0; i < m->n; ++i )
	{
		if ( m->legs[i].planted )
		{
			pts2[np].x = m->legs[i].foot.x;
			pts2[np].z = m->legs[i].foot.z;
			ys[np] = m->legs[i].foot.y;
			np++;
		}
	}
	for ( int i = 0; i < m->n; ++i )
	{
		const GcSwing* sw = &m->legs[i].swing;
		if ( sw->active && sw->has_ground && sw->t > 0.6 && np < GC_MAX_HULL_PTS )
		{
			pts2[np].x = sw->target.x;
			pts2[np].z = sw->target.z;
			ys[np] = sw->target.y;
			np++;
		}
	}
	if ( np == 0 )
	{
		m->support = 1.0;
		m->hull_n = 0;
		if ( !any_edge )
			m->unbal_t = 0.0;
		return 1.0;
	}

	m->hull_n = gc_convex_hull_2d( pts2, np, m->hull );
	GcV2 com = { m->pos.x, m->pos.z };
	double margin = gc_support_margin( m->hull, m->hull_n, com );
	m->support = margin;
	double excess = -margin - 0.15 * m->size;
	int wall_blocked = 0, edge = 0;
	GcV3 out = gc_v3_zero();
	if ( excess > 0.0 )
	{
		GcV2 nearest = gc_nearest_on_hull( m->hull, m->hull_n, com );
		out = gc_v3_norm( gc_v3( com.x - nearest.x, 0.0, com.z - nearest.z ),
						  gc_project_plane( m->frame.fwd, GC_WORLD_UP ) );
		for ( int i = 0; i < m->n; ++i )
		{
			if ( ( m->legs[i].searching || m->legs[i].void_mark > 0.0 ) &&
				 gc_v3_dot( gc_v3_sub( m->legs[i].ideal_now, m->pos ), out ) > 0.0 )
			{
				edge = 1;
				break;
			}
		}
		double half_out = fabs( gc_v3_dot( out, m->frame.right ) ) * m->d->body_half[0] +
						  fabs( gc_v3_dot( out, m->frame.fwd ) ) * m->d->body_half[2];
		double wall_d = half_out + 0.45;
		double y_offs[2] = { 0.0, -0.45 * m->ride };
		for ( int yi = 0; yi < 2; ++yi )
		{
			GcHit hw;
			if ( gc_world_raycast( &m->wq, gc_v3_add( m->pos, gc_v3_mul( GC_WORLD_UP, y_offs[yi] ) ), out, wall_d, &hw ) &&
				 fabs( gc_v3_dot( hw.normal, GC_WORLD_UP ) ) < 0.55 )
			{
				wall_blocked = 1;
				break;
			}
		}
	}
	if ( excess <= 0.0 || wall_blocked || !( ( m->speed < 1.0 && fabs( m->yaw_rate ) < 0.5 ) || edge ) )
	{
		m->unbal_t = fmax( 0.0, m->unbal_t - 2.0 * dt );
		return 1.0;
	}
	m->unbal_t += dt;
	if ( m->unbal_t < 0.2 )
		return 1.0;
	m->has_balance_out = 1;
	m->balance_out = out;
	excess -= 0.35;
	if ( excess <= 0.0 )
		return 1.0;
	m->balance_excess = excess;
	double ysum = 0.0;
	for ( int i = 0; i < np; ++i ) ysum += ys[i];
	m->balance_h = m->pos.y - ysum / (double)np;
	return gc_clamp( 1.0 - excess / ( 0.6 * m->ride ), 0.0, 1.0 );
}

// Latch a brief forward burst slide when crouch/drift is pressed above minimum
// speed, locking out immediate re-triggers while the key stays held so sliding
// feels crisp rather than like continuous ice drifting.
static GcV3 update_drift( GcMech* m, double dt, GcV3 vt )
{
	double spd = gc_v3_len( vt );
	if ( m->slide_cd > 0.0 )
		m->slide_cd = fmax( 0.0, m->slide_cd - dt );
	int want = ( m->inp.drift > 0.5 && m->state == GC_STATE_GROUND && !m->on_wall ) ? 1 : 0;
	if ( !want )
		m->slide_lock = 0;
	int pressed = ( want && !( m->prev.drift > 0.5 ) ) ? 1 : 0;
	if ( pressed && !m->drifting )
		m->slide_buf = 0.22;
	else if ( want && !m->drifting && !m->slide_lock )
		m->slide_buf = fmax( m->slide_buf, dt );
	else
		m->slide_buf = fmax( 0.0, m->slide_buf - dt );

	double start_speed = fmin( 3.0, 0.25 * m->d->speed_run );
	if ( !m->drifting && m->slide_buf > 0.0 && !m->slide_lock &&
		 m->slide_cd <= 0.0 && m->state == GC_STATE_GROUND && !m->on_wall &&
		 spd > start_speed && m->flight_t <= 0.08 )
	{
		m->drifting = 1;
		m->drift_t = 0.0;
		m->slide_buf = 0.0;
		m->flight_t = 0.0;
		m->flight_hold = 0.0;
		GcV3 heading = gc_v3_norm( gc_project_plane( m->frame.fwd, gc_mech_probe_up( m ) ), gc_v3( 0.0, 0.0, 1.0 ) );
		m->slide_dir = gc_v3_norm( vt, heading );
		double kick = fmin( 4.0, 0.14 * m->d->speed_run ) *
					  gc_clamp( spd / fmax( 1.0, 0.5 * m->d->speed_run ), 0.35, 1.0 );
		double new_spd = fmin( spd + kick, m->d->speed_run * 1.08 );
		vt = gc_v3_mul( m->slide_dir, new_spd );
		spd = new_spd;
	}
	else if ( m->drifting )
	{
		m->drift_t += dt;
		double slope_down = gc_v3_dot( m->slide_dir, m->ground_n );
		double min_dur = 0.22;
		double max_dur = 0.95 + 0.55 * gc_clamp( slope_down / 0.35, 0.0, 1.0 );
		double exit_spd = fmax( 2.0, 0.12 * m->d->speed_run );
		if ( ( !want && m->drift_t >= min_dur ) || m->drift_t >= max_dur || spd < exit_spd )
		{
			m->drifting = 0;
			m->slide_cd = 0.35;
			if ( want )
				m->slide_lock = 1;
		}
	}
	double rate = m->drifting ? 14.0 : 9.0;
	m->drift_amt += ( ( m->drifting ? 1.0 : 0.0 ) - m->drift_amt ) * gc_damp( rate, dt );
	return vt;
}

// Clear grounded leg state and slide buffers on entry into free flight.
void gc_mech_enter_air( GcMech* m )
{
	m->state = GC_STATE_AIR;
	m->state_t = 0.0;
	m->slide_vel = gc_v3_zero();
	m->accel_smooth = gc_v3_zero();
	m->has_balance_out = 0;
	m->balance_excess = 0.0;
	m->unbal_t = 0.0;
	m->flight_t = 0.0;
	m->flight_hold = 0.0;
	m->prev_npl = 0;
	m->trailing_support = 0;
	m->unsupported_t = 0.0;
	m->air_belly = 0;
	m->drifting = 0;
	m->slide_buf = 0.0;
	m->detach_t = 0.0;
	m->ground_n = GC_WORLD_UP;
	m->land_right_t = 0.0;
	m->land_right_n = GC_WORLD_UP;
	for ( int i = 0; i < m->n; ++i )
	{
		m->legs[i].planted = 0;
		m->legs[i].swing.active = 0;
		m->legs[i].searching = 0;
	}
}

static void launch_jump( GcMech* m )
{
	const GcMechDef* d = m->d;
	const GcFrame* fr = &m->frame;
	GcV3 jdir = m->on_wall ? fr->up : GC_WORLD_UP;
	GcV3 vj = gc_v3_sub( m->vel, gc_v3_mul( jdir, fmin( 0.0, gc_v3_dot( m->vel, jdir ) ) ) );
	m->vel = gc_v3_add( gc_v3_add( vj, gc_v3_mul( jdir, d->jump_speed ) ),
						gc_v3_mul( fr->fwd, fmax( 0.0, m->inp.fwd ) * 1.5 ) );
	int pushing[GC_MAX_LEGS];
	for ( int i = 0; i < m->n; ++i ) pushing[i] = m->legs[i].planted;
	gc_mech_enter_air( m );
	for ( int i = 0; i < m->n; ++i )
		if ( pushing[i] ) m->legs[i].push = 1;
	gc_mech_emit( m, GC_EV_LAND, gc_v3_sub( m->pos, gc_v3_mul( fr->up, m->ride ) ), fr->up, d->mass * 2.0, gc_v3_zero() );
}

static void predict_impact( GcMech* m )
{
	GcV3 up = GC_WORLD_UP;
	GcV3 p = gc_v3_sub( m->pos, gc_v3_mul( up, m->ride * 1.05 ) );
	GcV3 v = m->vel;
	double t = 0.0;
	const double step = 0.04;
	while ( t < 2.4 )
	{
		GcV3 v2 = gc_v3_sub( v, gc_v3_mul( up, GC_G * step ) );
		GcV3 q = gc_v3_add( p, gc_v3_mul( gc_v3_add( v, v2 ), 0.5 * step ) );
		GcV3 dv = gc_v3_sub( q, p );
		double dl = gc_v3_len( dv );
		if ( dl > 1e-5 )
		{
			GcHit h;
			if ( gc_world_raycast( &m->wq, p, gc_v3_mul( dv, 1.0 / dl ), dl, &h ) && h.normal.y > 0.3 )
			{
				double f = h.t / dl;
				m->impact.valid = 1;
				m->impact.t = t + step * f;
				m->impact.point = h.point;
				m->impact.normal = h.normal;
				m->impact.speed = gc_v3_len( gc_v3_lerp( v, v2, f ) );
				return;
			}
		}
		p = q;
		v = v2;
		t += step;
	}
	m->impact.valid = 0;
}

static int land_on( GcMech* m, const GcHit* h )
{
	int old_state = m->state;
	double old_state_t = m->state_t;
	double old_unsup = m->unsupported_t;
	double old_unbal = m->unbal_t;
	GcV3 old_gn = m->ground_n;
	double old_lp = m->look_pitch;
	double old_abs = m->absorb, old_brace = m->brace;
	GcV3 old_av = m->ang_vel, old_vel = m->vel;
	double old_lrt = m->land_right_t;
	GcV3 old_lrn = m->land_right_n;

	m->state = GC_STATE_GROUND;
	m->state_t = 0.0;
	m->unsupported_t = 0.0;
	GcV3 land_n = gc_v3_norm( h->normal, GC_WORLD_UP );
	double tilt = gc_angle_between( m->frame.up, land_n );
	GcFrame land_fr;
	gc_frame_init( &land_fr, gc_mech_heading_fwd( m ), land_n );
	m->ground_n = land_n;
	m->look_pitch = ( fabs( m->inp.aim ) > 1e-3 ) ? gc_clamp( m->inp.aim, -m->look_pitch_dn, m->look_pitch_up ) : 0.0;
	double impact = -fmin( 0.0, m->vel.y );
	m->absorb = fmax( m->absorb, gc_clamp( ( impact - 1.5 ) / 22.0, 0.0, 0.5 ) );
	m->brace = fmax( m->brace, gc_clamp( impact / 12.0, 0.0, 1.0 ) );
	GcV3 vh = gc_v3( m->vel.x, 0.0, m->vel.z );
	double v_spd = gc_v3_len( vh );
	if ( tilt < 0.15 && impact > 2.5 && v_spd > 1.5 )
	{
		GcV3 axis = gc_v3_mul( gc_v3_cross( GC_WORLD_UP, vh ), 1.0 / v_spd );
		double dip = gc_clamp( impact * fmin( v_spd, 18.0 ) * 0.011 / sqrt( m->d->mass ), 0.0, 1.35 );
		m->ang_vel = gc_v3_add( m->ang_vel, gc_v3_mul( axis, dip ) );
	}
	if ( tilt > 0.10 )
	{
		m->land_right_t = gc_clamp( 0.28 + 0.40 * tilt, 0.28, 0.65 );
		m->land_right_n = land_n;
		if ( tilt >= 0.15 )
			m->ang_vel = gc_v3_mul( m->ang_vel, 0.20 );
	}
	else
	{
		m->land_right_t = 0.0;
	}
	m->vel = gc_v3( m->vel.x, fmax( m->vel.y, -6.0 ), m->vel.z );
	const GcFrame* fr = &m->frame;
	double snap_dist = ( tilt > 0.15 ? 1.35 : 0.95 ) * m->size;

	for ( int i = 0; i < m->n; ++i )
	{
		GcLeg* l = &m->legs[i];
		l->push = 0;
		l->mount_world = gc_v3_add( m->pos, gc_frame_to_world( fr, l->mount_local ) );
		l->ideal_now = gc_v3_add( m->pos, gc_frame_to_world( &land_fr, gc_leg_rest_local( l, m->ride, 1.0 ) ) );
		GcV3 lead_l = ( v_spd > 4.0 ) ? gc_v3_mul( vh, fmin( 0.035, 0.25 * l->reach / v_spd ) ) : gc_v3_zero();
		int has_fh = 0;
		GcV3 fhp = gc_v3_zero(), fhn = GC_WORLD_UP;
		if ( gc_mech_climbing_mode( m ) && tilt > 0.35 )
		{
			GcV3 cands[2] = {
				gc_v3_add( gc_v3_add( gc_project_plane( gc_v3_sub( l->foot, h->point ), land_n ), h->point ),
						   gc_v3_mul( land_n, m->ride + 1.0 ) ),
				gc_v3_add( l->ideal_now, gc_v3_mul( land_n, m->ride + 1.0 ) )
			};
			for ( int ci = 0; ci < 2; ++ci )
			{
				GcHit hc;
				if ( gc_world_raycast( &m->wq, cands[ci], gc_v3_mul( land_n, -1.0 ), m->ride * 2.5 + 1.0, &hc ) &&
					 gc_v3_dot( hc.normal, land_n ) > 0.6 &&
					 gc_v3_dist( hc.point, l->mount_world ) <= 0.98 * l->reach )
				{
					has_fh = 1;
					fhp = hc.point;
					fhn = hc.normal;
					break;
				}
			}
		}
		if ( !has_fh )
		{
			GcV3 p_est = gc_v3_add( gc_v3_add( gc_project_plane( gc_v3_sub( l->foot, h->point ), GC_WORLD_UP ), h->point ), lead_l );
			has_fh = gc_find_foothold( m, l, p_est, 0.0, &fhp, &fhn );
		}
		if ( !has_fh )
			has_fh = gc_find_foothold( m, l, l->ideal_now, 0.0, &fhp, &fhn );
		int reach_ok = has_fh && ( tilt <= 0.15 || gc_v3_dist( fhp, l->mount_world ) <= 0.98 * l->reach );
		if ( has_fh && reach_ok && gc_v3_dist( fhp, l->foot ) < snap_dist )
		{
			l->foot = fhp;
			l->foot_normal = fhn;
			l->planted = 1;
			l->swing.active = 0;
			l->searching = 0;
		}
		else if ( has_fh )
		{
			double d_f = gc_v3_dist( fhp, l->foot );
			double sw_t = ( tilt > 0.15 ) ? gc_clamp( d_f / 18.0, 0.07, 0.14 ) : gc_clamp( d_f / 14.0, 0.11, 0.18 );
			GcV3 sw_up = ( gc_mech_climbing_mode( m ) && tilt > 0.35 ) ? land_n : gc_mech_probe_up( m );
			gc_leg_begin_swing( l, fhp, fhn, sw_t, 0.05, sw_up, 1, &m->wq, 0.0,
								gc_v3_zero(), gc_v3_zero(), gc_v3_zero(), m->has_ceil_y, m->ceil_y );
		}
		else
		{
			l->searching = 1;
		}
	}

	int any_pl = 0;
	for ( int i = 0; i < m->n; ++i )
		if ( m->legs[i].planted ) { any_pl = 1; break; }
	if ( !any_pl )
	{
		GcLeg* best = NULL;
		double best_d = 1e9;
		for ( int i = 0; i < m->n; ++i )
		{
			GcLeg* l = &m->legs[i];
			if ( l->swing.active && l->swing.has_ground && gc_v3_dist( l->swing.target, l->mount_world ) <= 0.96 * l->reach )
			{
				double d_f = gc_v3_dist( l->swing.target, l->foot );
				if ( d_f < best_d )
				{
					best_d = d_f;
					best = l;
				}
			}
		}
		if ( best )
		{
			best->foot = best->swing.target;
			best->foot_normal = best->swing.normal;
			best->planted = 1;
			best->swing.active = 0;
			best->searching = 0;
		}
	}

	int any_ok = 0;
	for ( int i = 0; i < m->n; ++i )
		if ( m->legs[i].planted || ( m->legs[i].swing.active && m->legs[i].swing.has_ground ) ) { any_ok = 1; break; }
	if ( !any_ok )
	{
		m->state = old_state;
		m->state_t = old_state_t;
		m->unsupported_t = old_unsup;
		m->unbal_t = old_unbal;
		m->ground_n = old_gn;
		m->look_pitch = old_lp;
		m->absorb = old_abs;
		m->brace = old_brace;
		m->ang_vel = old_av;
		m->vel = old_vel;
		m->land_right_t = old_lrt;
		m->land_right_n = old_lrn;
		return 0;
	}
	m->unbal_t = 0.0;
	m->air_belly = 0;
	gc_mech_emit( m, GC_EV_LAND, h->point, h->normal, impact * m->d->mass * 0.25, gc_v3_zero() );
	int any_srch = 0;
	for ( int i = 0; i < m->n; ++i )
		if ( m->legs[i].searching ) { any_srch = 1; break; }
	if ( !any_srch )
		m->situation.edge = 0.0;
	return 1;
}

static void update_air( GcMech* m, double dt )
{
	GcFrame* fr = &m->frame;
	const GcInput* inp = &m->inp;
	const GcMechDef* d = m->d;
	m->vel = gc_v3_add( m->vel, gc_v3_mul( GC_WORLD_UP, -GC_G * dt ) );
	GcV3 vh = gc_v3( m->vel.x, 0.0, m->vel.z );
	if ( !m->air_belly )
	{
		GcV3 mv = gc_v3_add( gc_v3_mul( gc_mech_heading_fwd( m ), inp->fwd ), gc_v3_mul( fr->right, inp->strafe * 0.75 ) );
		GcV3 v_des = gc_v3_mul( mv, target_speed( m ) );
		GcV3 vdes_h = gc_v3( v_des.x, 0.0, v_des.z );
		vh = gc_v3_add( vh, gc_v3_mul( gc_v3_sub( vdes_h, vh ), gc_clamp( 1.2 * dt, 0.0, 1.0 ) ) );
	}
	else
	{
		GcV3 fwd_h = gc_v3_norm( gc_project_plane( fr->fwd, GC_WORLD_UP ), gc_v3( 0.0, 0.0, 1.0 ) );
		vh = gc_v3_sub( vh, gc_v3_mul( fwd_h, gc_v3_dot( vh, fwd_h ) * gc_damp( 4.0, dt ) ) );
	}
	m->vel = gc_v3( vh.x, m->vel.y, vh.z );
	m->yaw_rate += ( inp->turn * d->turn_rate - m->yaw_rate ) * gc_damp( 4.0, dt );
	gc_frame_rotate( fr, GC_WORLD_UP, -m->yaw_rate * dt );
	m->pos = gc_v3_add( m->pos, gc_v3_mul( m->vel, dt ) );
	int belly_hit = collide( m );
	predict_impact( m );
	if ( m->vel.y <= 0.0 || m->state_t > 0.4 )
	{
		double depth = 0.0;
		for ( int i = 0; i < m->n; ++i )
		{
			double dp = m->pos.y - m->legs[i].foot.y;
			if ( dp > depth ) depth = dp;
		}
		double reach = fmax( m->ride * 1.02, depth + 0.05 ) + fmax( 0.0, -m->vel.y ) * dt;
		GcHit h;
		double min_ny = gc_mech_climbing_mode( m ) ? 0.3 : 0.55;
		if ( gc_world_raycast( &m->wq, m->pos, gc_v3_mul( GC_WORLD_UP, -1.0 ), reach, &h ) && h.normal.y > min_ny )
			land_on( m, &h );
	}
	m->air_belly = ( m->state == GC_STATE_AIR && belly_hit ) ? 1 : 0;
	if ( m->air_belly && m->state_t > 0.04 )
	{
		GcV3 side_base = gc_v3_norm( gc_project_plane( fr->right, GC_WORLD_UP ), gc_v3( 1.0, 0.0, 0.0 ) );
		double v_lat = gc_v3_dot( m->vel, side_base );
		double sgn = v_lat >= 0.0 ? 1.0 : -1.0;
		GcV3 side_dir = gc_v3_mul( side_base, sgn );
		if ( fabs( v_lat ) < 1.2 )
			m->vel = gc_v3_add( m->vel, gc_v3_mul( side_dir, 1.2 - fabs( v_lat ) ) );
		m->vel = gc_v3_add( m->vel, gc_v3_mul( side_dir, 10.5 * dt ) );
		m->ang_vel = gc_v3_add( m->ang_vel, gc_v3_mul( gc_v3_cross( GC_WORLD_UP, side_dir ), 5.5 * dt ) );
	}
	if ( m->pos.y < -60.0 )
		m->vel = gc_v3_zero();
}

static int try_mantle( GcMech* m )
{
	if ( m->ground_n.y < 0.86 )
		return 0;
	const GcMechDef* d = m->d;
	const GcFrame* fr = &m->frame;
	GcWorldQueryCtx* wq = &m->wq;
	GcHit g_u;
	int has_gu = gc_world_raycast( wq, m->pos, gc_v3_mul( GC_WORLD_UP, -1.0 ), m->ride + 2.5, &g_u );
	if ( has_gu && g_u.normal.y < 0.90 )
		return 0;
	GcV3 fwd_h = gc_v3_norm( gc_v3( fr->fwd.x, 0.0, fr->fwd.z ), gc_v3_zero() );
	if ( gc_v3_len_sq( fwd_h ) < 0.5 )
		return 0;
	double pl_sum = 0.0;
	int pl_cnt = 0;
	for ( int i = 0; i < m->n; ++i )
	{
		if ( m->legs[i].planted )
		{
			pl_sum += m->legs[i].foot.y;
			pl_cnt++;
		}
	}
	double base_y = ( has_gu && fabs( g_u.point.y - ( m->pos.y - m->ride ) ) < 1.2 * m->ride )
						? g_u.point.y
						: ( pl_cnt > 0 ? ( pl_sum / (double)pl_cnt ) : ( m->pos.y - m->ride ) );
	double L = d->body_half[2] + 0.6;
	GcV3 o_hi = gc_v3( m->pos.x, base_y + d->step_max + 0.1, m->pos.z );
	GcHit h;
	if ( !gc_world_raycast( wq, o_hi, fwd_h, L, &h ) || fabs( h.normal.y ) > 0.4 )
		return 0;
	GcV3 o_lo = gc_v3( m->pos.x, base_y + d->step_max * 0.4, m->pos.z );
	GcHit hl;
	if ( gc_world_raycast( wq, o_lo, fwd_h, L, &hl ) && hl.t < h.t - 0.2 )
		return 0;
	double reach_h = fmax( d->mantle_max, d->wire_max );
	GcV3 top_o = gc_v3_add( gc_v3( h.point.x, base_y + reach_h + 0.5, h.point.z ), gc_v3_mul( fwd_h, 0.25 ) );
	GcHit h2;
	if ( !gc_world_raycast( wq, top_o, gc_v3_mul( GC_WORLD_UP, -1.0 ), reach_h + 0.5, &h2 ) || h2.normal.y < 0.8 )
		return 0;
	double height = h2.point.y - base_y;
	if ( height <= d->step_max * 0.9 || height > reach_h )
		return 0;
	GcHit pre;
	if ( gc_world_raycast( wq, gc_v3_sub( gc_v3( h.point.x, h2.point.y + 0.3, h.point.z ), gc_v3_mul( fwd_h, 0.25 ) ),
						   gc_v3_mul( GC_WORLD_UP, -1.0 ), 0.85, &pre ) &&
		 pre.normal.y > 0.64 && h2.point.y - pre.point.y < 0.45 )
		return 0;
	GcV3 ahead = gc_v3_mul( fwd_h, d->body_half[2] * 0.6 );
	GcHit end_hit;
	GcV3 end_ground = h2.point;
	if ( gc_world_raycast( wq, gc_v3_add( top_o, ahead ), gc_v3_mul( GC_WORLD_UP, -1.0 ), reach_h + 0.5, &end_hit ) &&
		 end_hit.normal.y >= 0.8 && h2.point.y - 0.1 <= end_hit.point.y && end_hit.point.y <= base_y + reach_h )
	{
		end_ground = end_hit.point;
	}
	height = fmax( height, end_ground.y - base_y );
	int wire = height > d->mantle_max ? 1 : 0;
	double headroom = m->ride + 2.0 * d->body_half[1];
	GcHit ch1, ch2;
	if ( gc_world_raycast( wq, gc_v3_add( h2.point, gc_v3_mul( GC_WORLD_UP, 0.1 ) ), GC_WORLD_UP, headroom, &ch1 ) ||
		 gc_world_raycast( wq, gc_v3_add( end_ground, gc_v3_mul( GC_WORLD_UP, 0.1 ) ), GC_WORLD_UP, headroom, &ch2 ) )
		return 0;

	m->mantle.active = 1;
	m->mantle.p0 = m->pos;
	m->mantle.p2 = gc_v3_add( end_ground, gc_v3_mul( GC_WORLD_UP, m->ride ) );
	m->mantle.t = 0.0;
	m->mantle.height = height;
	m->mantle.top = h2.point;
	m->mantle.fwd = fwd_h;
	m->mantle.wire = wire;
	m->mantle.landed = !wire;
	m->mantle.dur = ( wire ? ( 0.9 + 0.2 * height ) : ( 0.75 + 0.3 * height ) ) * sqrt( m->size );
	m->state = GC_STATE_MANTLE;
	m->state_t = 0.0;
	m->drifting = 0;
	for ( int i = 0; i < m->n; ++i )
	{
		if ( m->legs[i].is_front || ( m->biped && m->legs[i].idx == 0 ) )
			m->legs[i].force = 1;
	}
	gc_mech_emit( m, GC_EV_MANTLE, h2.point, GC_WORLD_UP, height, gc_v3_zero() );
	return 1;
}

GcV3 gc_mantle_pos( const GcMech* m, double t )
{
	const GcMantle* mn = &m->mantle;
	t = gc_clamp( t, 0.0, 1.0 );
	double s1, s2;
	if ( mn->wire )
	{
		s1 = gc_smoothstep( t / 0.7 );
		s2 = gc_smoothstep( ( t - 0.5 ) / 0.5 );
	}
	else
	{
		s1 = gc_smoothstep( t / 0.6 );
		s2 = gc_smoothstep( ( t - 0.3 ) / 0.7 );
	}
	return gc_v3( mn->p0.x + ( mn->p2.x - mn->p0.x ) * s2,
				  mn->p0.y + ( mn->p2.y + 0.1 - mn->p0.y ) * s1,
				  mn->p0.z + ( mn->p2.z - mn->p0.z ) * s2 );
}

static void update_mantle( GcMech* m, double dt )
{
	GcMantle* mn = &m->mantle;
	double strain = 0.0;
	for ( int i = 0; i < m->n; ++i )
	{
		if ( m->legs[i].planted && m->legs[i].stretch > strain )
			strain = m->legs[i].stretch;
	}
	mn->t += dt / mn->dur * ( strain > 1.0 ? 0.25 : 1.0 );
	GcV3 nw = gc_mantle_pos( m, mn->t );
	m->vel = gc_v3_mul( gc_v3_sub( nw, m->pos ), 1.0 / dt );
	m->pos = nw;
	m->yaw_rate *= exp( -6.3 * dt );
	if ( mn->wire && !mn->landed && mn->t >= 0.72 )
	{
		mn->landed = 1;
		const GcFrame* fr = &m->frame;
		for ( int i = 0; i < m->n; ++i )
		{
			GcLeg* l = &m->legs[i];
			l->mount_world = gc_v3_add( m->pos, gc_frame_to_world( fr, l->mount_local ) );
			l->ideal_now = gc_v3_add( m->pos, gc_frame_to_world( fr, gc_leg_rest_local( l, m->ride, 1.0 ) ) );
			GcV3 fhp, fhn;
			if ( gc_find_foothold( m, l, l->ideal_now, 0.0, &fhp, &fhn ) )
			{
				gc_leg_begin_swing( l, fhp, fhn, 0.12, 0.1, gc_mech_probe_up( m ), 1, &m->wq, 0.0,
									gc_v3_zero(), gc_v3_zero(), gc_v3_zero(), m->has_ceil_y, m->ceil_y );
			}
			else
			{
				l->searching = 1;
			}
		}
	}
	if ( mn->t >= 1.0 )
	{
		m->state = GC_STATE_GROUND;
		m->state_t = 0.0;
		m->vel = gc_v3( m->frame.fwd.x, 0.0, m->frame.fwd.z );
		mn->active = 0;
	}
}

// Grounded locomotion step: crouch-jump telegraph, mantle detection, slide
// latch, yaw-rate clamping, traction/momentum integration, and support ride.
static void update_ground( GcMech* m, double dt, int jump )
{
	const GcMechDef* d = m->d;
	GcFrame* fr = &m->frame;
	GcV3 up = gc_mech_probe_up( m );
	const GcInput* inp = &m->inp;
	int npl = 0;
	for ( int i = 0; i < m->n; ++i )
		if ( m->legs[i].planted ) npl++;

	if ( jump && d->jump_speed > 0.0 && m->state == GC_STATE_GROUND && npl > 0 )
	{
		if ( m->drifting )
		{
			m->drifting = 0;
			m->slide_cd = fmax( m->slide_cd, 0.25 );
		}
		m->state = GC_STATE_CROUCHJUMP;
		m->state_t = 0.0;
		m->jump_sink = 0.0;
		m->jump_r0 = fmax( 0.1, gc_mech_ride_eff( m ) );
		m->jump_y0 = gc_v3_dot( m->pos, up );
		m->jump_npl = npl;
		m->jump_sink_cap = sink_room( m );
	}
	if ( m->state == GC_STATE_CROUCHJUMP )
	{
		double ct = crouch_time( m );
		if ( m->state_t > ct )
		{
			launch_jump( m );
			return;
		}
		if ( npl > m->jump_npl )
		{
			double sr = sink_room( m );
			if ( sr < m->jump_sink_cap ) m->jump_sink_cap = sr;
		}
		m->jump_npl = npl;
		double want = m->jump_sink_cap * gc_smoothstep( m->state_t / fmax( 0.05, ct ) );
		int any_err = 0;
		for ( int i = 0; i < m->n; ++i )
			if ( m->legs[i].planted && m->legs[i].ik.err > 0.02 ) { any_err = 1; break; }
		if ( any_err )
			m->jump_sink = fmax( 0.0, m->jump_sink - 0.6 * dt );
		else if ( want > m->jump_sink )
			m->jump_sink = want;
	}
	if ( m->state == GC_STATE_GROUND && !gc_mech_climbing_mode( m ) &&
		 m->ground_n.y >= 0.86 && m->speed < fmax( 16.0, 1.15 * m->v_walk ) &&
		 ( d->mantle_max > 0.0 || d->wire_max > 0.0 ) &&
		 inp->fwd > 0.05 && m->mantle_t >= m->probe_period - 1e-6 )
	{
		m->mantle_t = fmod( m->mantle_t, m->probe_period );
		if ( try_mantle( m ) )
			return;
	}

	double bal = evaluate_balance( m, dt );
	GcV3 v_des = m->v_cmd;
	double vdl = gc_v3_len( v_des );
	if ( m->has_speed_cap && vdl > m->speed_cap )
	{
		v_des = gc_v3_mul( gc_v3_norm( v_des, gc_v3_zero() ), m->speed_cap );
		vdl = gc_v3_len( v_des );
	}
	m->v_desired = vdl;
	m->drive_target = v_des;
	m->drive_accel = 0.0;
	m->braking = ( m->speed > 1.0 && ( vdl < 0.82 * m->speed || ( vdl > 0.1 && gc_v3_dot( v_des, m->vt ) < -0.1 * m->speed * vdl ) ) ) ? 1 : 0;

	double v_up = gc_v3_dot( m->vel, up );
	GcV3 slide = gc_project_plane( m->slide_vel, up );
	GcV3 vt = gc_v3_sub( gc_v3_sub( m->vel, gc_v3_mul( up, v_up ) ), slide );
	vt = update_drift( m, dt, vt );
	GcV3 vt_raw = vt;

	double spin_scale = 0.62 + 0.38 * gc_smoothstep( ( fmax( m->speed, m->v_desired ) - 1.0 ) / 3.0 );
	double yr_des = inp->turn * d->turn_rate * spin_scale * ( m->mode == GC_MODE_SLOW ? 0.75 : 1.0 );
	double normal_lim = fmin( d->turn_rate, GC_A_LAT / fmax( m->speed, 1e-3 ) );
	double lim, k_yaw;
	if ( m->drifting || m->drift_amt > 0.02 )
	{
		double slide_w = m->drifting ? 1.0 : m->drift_amt;
		double slide_lim = fmin( 0.55 * d->turn_rate, GC_A_LAT / fmax( m->speed, 4.0 ) );
		lim = normal_lim + ( slide_lim - normal_lim ) * slide_w;
		k_yaw = 24.0 - 10.0 * slide_w;
	}
	else
	{
		lim = normal_lim;
		k_yaw = 24.0;
	}
	if ( m->flight_t > 0.0 )
		k_yaw = fmin( k_yaw, 16.0 );
	yr_des = gc_clamp( yr_des, -lim, lim );

	// Throttle in-place pivot yaw when planted legs approach hip yaw or reach
	// limits so the chassis waits slightly instead of twisting past joint stops.
	if ( !m->drifting && !gc_galloping_fast( m, 1 ) && m->speed < 0.45 * m->v_walk && bal >= 1.0 && fabs( yr_des ) > 0.1 )
	{
		double yaw_strain = 0.0;
		for ( int i = 0; i < m->n; ++i )
		{
			const GcLeg* l = &m->legs[i];
			if ( !l->planted ) continue;
			if ( l->ik.mode == GC_PLANE_RADIAL && l->ik.coxa_ratio > 0.96 && l->ik.yaw_deg * yr_des < 0.0 )
				yaw_strain = fmax( yaw_strain, ( l->ik.coxa_ratio - 0.96 ) * 20.0 );
			if ( l->stretch > 0.975 )
				yaw_strain = fmax( yaw_strain, ( l->stretch - 0.975 ) * 20.0 );
		}
		if ( yaw_strain > 0.0 )
			yr_des *= fmax( 0.45, 1.0 - fmin( 0.55, yaw_strain ) );
	}
	double new_yr = m->yaw_rate + ( yr_des - m->yaw_rate ) * gc_damp( k_yaw, dt );
	double yr_mid = 0.5 * ( m->yaw_rate + new_yr );

	int has_ground_swing = 0;
	for ( int i = 0; i < m->n; ++i )
		if ( m->legs[i].swing.active && m->legs[i].swing.has_ground ) { has_ground_swing = 1; break; }
	double traction = ( npl > 0 ) ? 1.0 : ( ( m->flight_t > 0.0 || has_ground_swing ) ? 0.3 : 0.0 );

	GcV3 vt_new;
	if ( m->drifting )
	{
		// Slide mode: preserve linear momentum with light carving, low surface
		// drag, downhill slope acceleration, and active reverse-input braking.
		GcV3 heading = gc_v3_norm( gc_project_plane( fr->fwd, up ), fr->fwd );
		double spd = gc_v3_len( vt );
		GcV3 vdir = gc_v3_norm( vt, m->slide_dir );
		GcV3 steer_dir = ( vdl > 0.2 ) ? gc_v3_norm( gc_project_plane( v_des, up ), heading ) : heading;
		double carve = ( vdl > 0.2 ) ? 0.55 : 0.25;
		vdir = gc_rotate_towards( vdir, steer_dir, carve * dt );
		m->slide_dir = vdir;
		double slope_down = gc_v3_dot( vdir, m->ground_n );
		double base_drag = fmax( 1.8, 0.06 * d->speed_run ) + 0.08 * spd;
		double slope_acc = GC_G * gc_clamp( slope_down, -0.55, 0.55 ) * 2.2;
		double brake_extra = ( vdl > 0.2 && gc_v3_dot( steer_dir, vdir ) < -0.3 ) ? fmax( 8.0, 0.25 * d->speed_run ) : 0.0;
		spd = fmax( 0.0, spd + ( slope_acc - base_drag - brake_extra ) * dt );
		spd = fmin( spd, d->speed_run * ( slope_down > 0.12 ? 1.12 : 1.08 ) );
		vt_new = gc_v3_mul( vdir, spd );
	}
	else
	{
		int fast = ( m->mode == GC_MODE_RUN || m->speed > 0.9 * fmin( m->v_walk, m->mode == GC_MODE_SLOW ? m->v_slow : 1e9 ) );
		double acc = ( ( fast && d->accel_run > 0.0 ) ? d->accel_run : d->accel ) * traction;
		// Rotate existing velocity with yaw during airborne bounds, fast gallops,
		// or smooth banked turns, but not when counter-steering hard across vel.
		int rotate_vt = 0;
		if ( m->speed > 0.5 && fabs( yr_mid ) > 1e-4 )
		{
			if ( m->flight_t > 0.0 || gc_galloping_fast( m, 0 ) )
				rotate_vt = 1;
			else if ( vdl > 0.5 && gc_v3_dot( vt, v_des ) >= -0.25 * m->speed * vdl )
				rotate_vt = 1;
			if ( rotate_vt && vdl > 0.5 )
			{
				double des_yaw = atan2( gc_v3_dot( gc_v3_cross( v_des, vt ), up ), gc_v3_dot( vt, v_des ) );
				if ( des_yaw * yr_mid < -0.03 * fabs( yr_mid ) )
					rotate_vt = 0;
			}
		}
		if ( rotate_vt )
			vt = gc_rotate_axis( vt, up, -yr_mid * dt );
		GcV3 dv = gc_v3_sub( v_des, vt );
		double max_dv = acc * dt * ( gc_v3_dot( dv, vt ) < 0.0 ? 1.6 : 1.0 );
		m->drive_accel = max_dv / dt;
		double dl = gc_v3_len( dv );
		if ( dl > max_dv )
			dv = gc_v3_mul( dv, max_dv / dl );
		vt_new = gc_v3_add( vt, dv );
	}

	const GcLeg* searching[GC_MAX_LEGS];
	int n_srch = 0;
	for ( int i = 0; i < m->n; ++i )
		if ( m->legs[i].searching || m->legs[i].void_mark > 0.0 ) searching[n_srch++] = &m->legs[i];
	if ( n_srch > 0 && npl > 0 && !m->on_wall && !m->drifting )
		m->void_t += dt;
	else
		m->void_t = fmax( 0.0, m->void_t - 0.5 * dt );

	int narrow_ahead = 0;
	GcV3 narrow_void = gc_v3_zero();
	if ( m->void_t > 0.30 && n_srch > 0 && npl > 0 && !m->on_wall && !m->drifting )
	{
		GcV3 vs = gc_v3_zero();
		for ( int i = 0; i < n_srch; ++i )
			vs = gc_v3_add( vs, gc_project_plane( gc_v3_sub( searching[i]->ideal_now, m->pos ), up ) );
		GcV3 hf = gc_v3_norm( gc_project_plane( fr->fwd, up ), fr->fwd );
		double along = gc_v3_dot( vs, hf );
		GcV3 vd_dir = gc_v3_mul( hf, along >= 0.0 ? 1.0 : -1.0 );
		double push = ( vdl > 0.05 ) ? ( gc_v3_dot( v_des, vd_dir ) / fmax( vdl, 1e-6 ) ) : 0.0;
		int center_ground = 0, wide_ground = 0;
		if ( fabs( along ) >= 0.5 * gc_v3_len( vs ) )
		{
			double max_mx = 0.0;
			for ( int i = 0; i < m->n; ++i )
				if ( fabs( m->legs[i].mount_local.x ) > max_mx ) max_mx = fabs( m->legs[i].mount_local.x );
			double lat_check = fmax( 0.52, max_mx * 0.72 );
			GcV3 right_v = gc_v3_cross( up, vd_dir );
			double reach_d = m->ride + d->step_max + 2.0;
			double dists[2] = { d->body_half[2] + 0.6, d->body_half[2] + 1.5 };
			for ( int di = 0; di < 2; ++di )
			{
				GcV3 pa = gc_v3_add( m->pos, gc_v3_mul( vd_dir, dists[di] ) );
				GcHit g;
				if ( gc_world_raycast( &m->wq, gc_v3_add( pa, gc_v3_mul( up, 1.0 ) ), gc_v3_mul( up, -1.0 ), reach_d, &g ) &&
					 gc_v3_dot( g.normal, up ) > 0.64 )
				{
					center_ground = 1;
					if ( lat_check > 0.15 )
					{
						GcHit gl, gr;
						if ( gc_world_raycast( &m->wq, gc_v3_add( gc_v3_sub( pa, gc_v3_mul( right_v, lat_check ) ), gc_v3_mul( up, 1.0 ) ),
											   gc_v3_mul( up, -1.0 ), reach_d, &gl ) &&
							 gc_v3_dot( gl.normal, up ) > 0.64 &&
							 gc_world_raycast( &m->wq, gc_v3_add( gc_v3_add( pa, gc_v3_mul( right_v, lat_check ) ), gc_v3_mul( up, 1.0 ) ),
											   gc_v3_mul( up, -1.0 ), reach_d, &gr ) &&
							 gc_v3_dot( gr.normal, up ) > 0.64 )
							wide_ground = 1;
					}
					else
					{
						wide_ground = 1;
					}
				}
			}
		}
		narrow_ahead = ( center_ground && !wide_ground ) ? 1 : 0;
		if ( narrow_ahead )
		{
			narrow_void = vd_dir;
			double c = gc_v3_dot( vt_new, vd_dir );
			if ( c > 0.0 )
				vt_new = gc_v3_sub( vt_new, gc_v3_mul( vd_dir, c ) );
			if ( bal < 1.0 || m->balance_excess > 0.0 || n_srch >= ( m->n / 2 > 2 ? m->n / 2 : 2 ) )
			{
				double back = -gc_v3_dot( vt_new, vd_dir );
				double want = 1.0 * fmin( 1.0, ( m->void_t - 0.30 ) * 3.0 );
				if ( back < want )
					vt_new = gc_v3_sub( vt_new, gc_v3_mul( vd_dir, want - back ) );
			}
		}
		else if ( m->void_t > 0.4 )
		{
			if ( fabs( along ) < 0.5 * gc_v3_len( vs ) || wide_ground )
			{
			}
			else if ( push > 0.3 )
			{
				if ( m->void_t > 0.9 )
				{
					m->vel = gc_v3_add( gc_v3_add( vt_new, gc_v3_mul( vd_dir, 1.5 ) ), gc_v3_mul( up, v_up ) );
					gc_mech_enter_air( m );
					return;
				}
			}
			else if ( bal < 1.0 && m->balance_excess > 0.15 && m->void_t > 1.5 )
			{
				m->vel = gc_v3_add( gc_v3_add( vt_new, gc_v3_mul( vd_dir, 1.5 ) ), gc_v3_mul( up, v_up ) );
				gc_mech_enter_air( m );
				return;
			}
			else if ( bal >= 1.0 )
			{
				double c = gc_v3_dot( vt_new, vd_dir );
				if ( c > 0.0 )
					vt_new = gc_v3_sub( vt_new, gc_v3_mul( vd_dir, c ) );
				double back = -gc_v3_dot( vt_new, vd_dir );
				double want = 0.8 * fmin( 1.0, ( m->void_t - 0.4 ) * 3.0 );
				if ( back < want )
					vt_new = gc_v3_sub( vt_new, gc_v3_mul( vd_dir, want - back ) );
			}
		}
	}

	if ( !m->drifting && !gc_galloping_fast( m, 1 ) && bal >= 1.0 )
	{
		for ( int i = 0; i < m->n; ++i )
		{
			const GcLeg* l = &m->legs[i];
			if ( l->planted && l->stretch > 0.96 )
			{
				GcV3 away = gc_v3_norm( gc_project_plane( gc_v3_sub( l->mount_world, l->foot ), up ), gc_v3_zero() );
				double c = gc_v3_dot( vt_new, away );
				if ( c > 0.0 )
					vt_new = gc_v3_sub( vt_new, gc_v3_mul( away, c * fmin( 1.0, ( l->stretch - 0.96 ) * 25.0 ) ) );
			}
		}
	}
	m->last_accel = gc_v3_mul( gc_v3_sub( vt_new, vt ), 1.0 / dt );
	m->accel_smooth = gc_v3_lerp( m->accel_smooth, m->last_accel, gc_damp( 14.0, dt ) );

	m->yaw_rate = new_yr;
	gc_frame_yaw( fr, yr_mid * dt );

	double sl = gc_v3_len( slide );
	if ( sl > 1e-4 )
	{
		double mu = ( npl > 0 ) ? 0.55 : 0.12;
		slide = gc_v3_mul( slide, fmax( 0.0, sl - mu * GC_G * dt ) / sl );
	}
	m->slide_vel = slide;

	if ( m->flight_t <= 0.0 && m->gait.pair_sync && m->gait.min_support == 0 &&
		 !m->braking && m->speed > 0.5 * gc_v_leg( m ) && m->trailing_support && !m->on_wall && !m->drifting )
	{
		int rev = gc_v3_dot( vt_new, gc_mech_heading_fwd( m ) ) < 0.0;
		int lead_pair = ( rev ? &m->legs[m->n - 1] : &m->legs[0] )->pair;
		int any_lead = 0;
		for ( int i = 0; i < m->n; ++i )
			if ( m->legs[i].planted && m->legs[i].pair == lead_pair ) { any_lead = 1; break; }
		if ( !any_lead )
			v_up += 0.45 * GC_G * dt;
	}

	if ( gc_update_flight( m, dt, npl, vt_new, up, &v_up ) )
	{
		m->pos = gc_v3_add( gc_v3_add( m->pos, gc_v3_mul( m->vel, dt ) ), gc_v3_mul( up, 0.5 * GC_G * dt * dt ) );
		collide( m );
		return;
	}

	if ( npl > 0 || has_ground_swing )
		m->unsupported_t = 0.0;
	else
		m->unsupported_t += dt;
	int supported = ( npl > 0 || ( m->unsupported_t < 0.12 && gc_may_flight( m ) ) ) ? 1 : 0;
	double err = 0.0;
	int has_err = support_error( m, &err );
	GcV3 grav = gc_v3_zero();
	if ( supported && has_err )
	{
		double f = 3.0 / sqrt( m->size );
		double w = GC_TAU * f;
		v_up += ( w * w * err - 2.0 * 0.75 * w * v_up ) * dt * bal;
		grav = gc_v3_mul( GC_WORLD_UP, -GC_G * dt * ( 1.0 - bal ) );
	}
	else
	{
		grav = gc_v3_mul( GC_WORLD_UP, -GC_G * dt );
	}
	m->vel = gc_v3_add( gc_v3_add( gc_v3_add( vt_new, m->slide_vel ), gc_v3_mul( up, v_up ) ), grav );

	if ( m->has_balance_out && m->balance_excess > 0.0 )
	{
		if ( !( narrow_ahead && gc_v3_dot( m->balance_out, narrow_void ) > 0.2 ) )
		{
			double h = fmax( 0.3, m->balance_h );
			double a = fmin( GC_G, GC_G * m->balance_excess / h );
			double v_cap = fmin( m->v_walk, 6.0 );
			double cur_b = gc_v3_dot( m->vel, m->balance_out );
			double dv_b = fmin( a * dt, fmax( 0.0, v_cap - cur_b ) );
			m->vel = gc_v3_add( m->vel, gc_v3_mul( m->balance_out, dv_b ) );
			if ( dv_b > 0.0 )
				m->ang_vel = gc_v3_add( m->ang_vel, gc_v3_mul( gc_v3_cross( GC_WORLD_UP, m->balance_out ), dv_b / h ) );
		}
		for ( int i = 0; i < m->n; ++i )
		{
			if ( m->legs[i].planted && m->legs[i].stretch > 1.0 )
			{
				m->legs[i].planted = 0;
				m->legs[i].searching = 1;
				m->legs[i].search_cd = 0.15;
			}
		}
		int any_pl_now = 0, any_ok_pl = 0;
		for ( int i = 0; i < m->n; ++i )
		{
			if ( m->legs[i].planted )
			{
				any_pl_now = 1;
				if ( m->legs[i].stretch <= 1.0 ) any_ok_pl = 1;
			}
		}
		if ( ( bal <= 0.0 || !any_pl_now ) && !( narrow_ahead && npl > 0 ) )
		{
			double unbal_lim = ( m->balance_excess > 0.65 * m->size ) ? 0.45 : 2.5;
			if ( !any_ok_pl || m->unbal_t > unbal_lim )
			{
				m->vel = gc_v3_add( m->vel, gc_v3_mul( m->balance_out, 1.0 ) );
				gc_mech_enter_air( m );
			}
		}
	}

	if ( !supported && m->unsupported_t > 0.12 )
		gc_mech_enter_air( m );
	if ( m->on_wall )
	{
		int hold = ( m->n / 2 - 1 ) > 2 ? ( m->n / 2 - 1 ) : 2;
		if ( npl < hold || !gc_mech_climbing_mode( m ) )
		{
			m->detach_t += dt;
			if ( m->detach_t > 0.25 )
				gc_mech_enter_air( m );
		}
		else
		{
			m->detach_t = 0.0;
		}
	}

	GcV3 vt_step = m->drifting ? vt_new : gc_v3_mul( gc_v3_add( vt_raw, vt_new ), 0.5 );
	m->pos = gc_v3_add( m->pos, gc_v3_mul( gc_v3_add( gc_v3_sub( m->vel, vt_new ), vt_step ), dt ) );
	collide( m );
}

static GcV3 climb_probes( GcMech* m, GcV3 target )
{
	if ( m->state != GC_STATE_GROUND )
		return target;
	const GcFrame* fr = &m->frame;
	GcV3 up = fr->up;
	GcV3 mv = gc_v3_add( gc_v3_mul( fr->fwd, m->inp.fwd ), gc_v3_mul( fr->right, m->inp.strafe * 0.75 ) );
	if ( gc_v3_len_sq( mv ) < 0.01 )
		return target;
	GcV3 dirv = gc_v3_norm( mv, fr->fwd );
	double L = m->d->body_half[2] + m->reach_avg * 0.65;
	GcHit h;
	if ( gc_world_raycast( &m->wq, m->pos, dirv, L, &h ) &&
		 gc_v3_dot( h.normal, up ) < 0.7 && gc_v3_dot( h.normal, dirv ) < -0.2 )
	{
		double k = pow( gc_clamp( 1.0 - h.t / L, 0.0, 1.0 ), 0.7 ) * 0.9;
		return gc_v3_norm( gc_v3_add( gc_v3_mul( target, 1.0 - k ), gc_v3_mul( h.normal, k ) ), target );
	}
	GcV3 p = gc_v3_add( m->pos, gc_v3_mul( dirv, m->d->body_half[2] + m->reach_avg * 0.35 ) );
	double depth = m->ride * 2.2;
	GcHit g;
	if ( !gc_world_raycast( &m->wq, p, gc_v3_mul( up, -1.0 ), depth, &g ) )
	{
		GcV3 end = gc_v3_sub( p, gc_v3_mul( up, depth ) );
		GcV3 back = gc_v3_sub( gc_v3_sub( m->pos, gc_v3_mul( up, m->ride ) ), end );
		double bl = gc_v3_len( back );
		if ( bl > 1e-4 )
		{
			GcHit h3;
			if ( gc_world_raycast( &m->wq, end, gc_v3_mul( back, 1.0 / bl ), m->reach_avg * 1.5, &h3 ) &&
				 gc_v3_dot( h3.normal, up ) < 0.9 )
			{
				return gc_v3_norm( gc_v3_add( gc_v3_mul( target, 0.4 ), gc_v3_mul( h3.normal, 0.6 ) ), target );
			}
		}
	}
	return target;
}

// Align chassis up-axis toward ground fit normal, wall climb probes, or air
// tumble/aim orientation, then layer inertia lean and slide counter-lean.
static void update_orientation( GcMech* m, double dt )
{
	GcFrame* fr = &m->frame;
	const GcMechDef* d = m->d;
	int righting = 0;
	GcV3 target = GC_WORLD_UP;
	if ( m->state == GC_STATE_AIR )
	{
		if ( !gc_mech_wire_active( m ) && fabs( m->inp.aim ) > 1e-3 )
		{
			double a = gc_clamp( m->inp.aim, -GC_AIM_MAX, GC_AIM_MAX );
			target = gc_rotate_axis( GC_WORLD_UP, fr->right, a );
		}
		else
		{
			m->ang_vel = gc_v3_mul( m->ang_vel, exp( -3.5 * dt ) );
			double ang = gc_v3_len( m->ang_vel );
			if ( ang * dt > 1e-7 )
				gc_frame_rotate( fr, gc_v3_mul( m->ang_vel, 1.0 / ang ), ang * dt );
			m->on_wall = 0;
			return;
		}
	}
	else
	{
		int any_pl = 0;
		for ( int i = 0; i < m->n; ++i )
			if ( m->legs[i].planted ) { any_pl = 1; break; }
		righting = ( m->state == GC_STATE_GROUND && m->land_right_t > 0.0 && ( any_pl || m->state_t < 0.08 ) ) ? 1 : 0;
		if ( righting )
		{
			m->ground_n = m->land_right_n;
			GcV3 ground_up = m->land_right_n;
			target = ground_up;
			if ( !m->on_wall && fabs( m->inp.aim ) > 1e-3 )
			{
				m->look_pitch = gc_clamp( m->inp.aim, -m->look_pitch_dn, m->look_pitch_up );
				GcV3 pitch_axis = gc_v3_norm( gc_v3_cross( gc_mech_heading_fwd( m ), ground_up ), fr->right );
				target = gc_rotate_axis( target, pitch_axis, m->look_pitch );
			}
		}
		else
		{
			if ( m->state == GC_STATE_MANTLE )
			{
				m->ground_n = gc_v3_norm( gc_v3_lerp( m->ground_n, GC_WORLD_UP, gc_damp( 14.0, dt ) ), GC_WORLD_UP );
			}
			else if ( m->has_plane_n_local )
			{
				GcFrame sf = gc_mech_support_frame( m );
				GcV3 raw_n = gc_v3_norm( gc_frame_to_world( &sf, m->plane_n_local ), GC_WORLD_UP );
				double rate = gc_mech_climbing_mode( m ) ? 22.0 : 14.0;
				m->ground_n = gc_v3_norm( gc_v3_lerp( m->ground_n, raw_n, gc_damp( rate, dt ) ), GC_WORLD_UP );
			}
			else if ( gc_mech_climbing_mode( m ) )
			{
				m->ground_n = fr->up;
			}
			else
			{
				m->ground_n = gc_v3_norm( gc_v3_lerp( m->ground_n, GC_WORLD_UP, gc_damp( 2.0, dt ) ), GC_WORLD_UP );
			}
			GcV3 ground_up;
			if ( gc_mech_climbing_mode( m ) )
			{
				target = climb_probes( m, m->ground_n );
				ground_up = target;
			}
			else
			{
				ground_up = gc_rotate_towards( GC_WORLD_UP, gc_v3_norm( m->ground_n, GC_WORLD_UP ),
											   fmax( d->max_tilt, 40.0 ) * GC_DEG2RAD );
				target = ground_up;
			}
			if ( !m->on_wall )
			{
				GcV3 a = gc_frame_to_local( fr, m->accel_smooth );
				double gal = ( gc_galloping_fast( m, 0 ) || m->flight_t > 0.0 ) ? 0.35 : 1.0;
				double lat_scale = 1.0 / sqrt( fmax( 0.5, d->mass ) );
				GcV3 lean = gc_v3_mul( fr->fwd, gc_clamp( a.z * 0.006 * gal, -0.15, 0.15 ) );
				lean = gc_v3_add( lean, gc_v3_mul( fr->right, gc_clamp( ( a.x * 0.0045 + m->yaw_rate * m->speed * 0.003 ) * lat_scale, -0.20, 0.20 ) ) );
				if ( m->biped )
					lean = gc_v3_add( lean, gc_v3_mul( fr->fwd, fmin( 0.07, m->speed * 0.01 ) ) );
				if ( m->drift_amt > 0.01 )
					lean = gc_v3_sub( lean, gc_v3_mul( m->slide_dir, 0.14 * m->drift_amt ) );
				target = gc_v3_norm( gc_v3_add( target, lean ), GC_WORLD_UP );
			}
			if ( !m->on_wall && ( fabs( m->inp.aim ) > 1e-3 || fabs( m->look_pitch ) > 1e-4 ) )
			{
				double want_a = ( fabs( m->inp.aim ) > 1e-3 ) ? gc_clamp( m->inp.aim, -m->look_pitch_dn, m->look_pitch_up ) : 0.0;
				int strained = 0;
				for ( int i = 0; i < m->n; ++i )
				{
					const GcLeg* l = &m->legs[i];
					if ( l->ik.err > 0.015 ||
						 ( l->planted && l->ik.limit_margin_deg < 1.0 ) ||
						 ( l->planted && l->stretch > 0.96 && m->speed < 2.0 ) )
					{
						strained = 1;
						break;
					}
				}
				if ( strained )
				{
					m->pitch_cap = fmax( 0.35, m->pitch_cap - 1.2 * dt );
					m->pitch_strain_t = 0.45;
				}
				else if ( m->pitch_strain_t > 0.0 )
				{
					m->pitch_strain_t = fmax( 0.0, m->pitch_strain_t - dt );
				}
				else
				{
					m->pitch_cap = fmin( 1.0, m->pitch_cap + 0.65 * dt );
				}
				double capped_a = want_a * m->pitch_cap;
				double rate = ( m->speed > 2.0 ) ? 6.0 : 12.0;
				double d_pitch = ( capped_a - m->look_pitch ) * gc_damp( rate, dt );
				if ( m->speed > 2.0 && ground_up.y < 0.97 )
				{
					double max_dp = 1.8 * dt;
					d_pitch = gc_clamp( d_pitch, -max_dp, max_dp );
				}
				m->look_pitch += d_pitch;
				if ( fabs( m->look_pitch ) > 1e-4 )
				{
					GcV3 pitch_axis = gc_v3_norm( gc_v3_cross( gc_mech_heading_fwd( m ), ground_up ), fr->right );
					target = gc_rotate_axis( target, pitch_axis, m->look_pitch );
				}
			}
			double fwd_v = gc_v3_dot( m->vt, gc_mech_heading_fwd( m ) );
			double fwd_sgn = ( fwd_v < -1.0 ) ? -1.0 : 1.0;
			double fwd_align = gc_clamp( fabs( fwd_v ) / fmax( m->speed, 1.0 ), 0.0, 1.0 );
			if ( m->flight_t > 0.0 && m->flight_dur > 0.0 )
			{
				double prog = 1.0 - m->flight_t / m->flight_dur;
				double pitch = d->bound_pitch * GC_DEG2RAD * m->gait.stride_pitch * fwd_sgn * fwd_align *
							   ( cos( GC_PI * prog ) * 0.92 + 0.08 );
				target = gc_rotate_axis( target, fr->right, pitch );
			}
			else if ( m->gait.pair_sync && m->state == GC_STATE_GROUND && d->bound > 0.0 && m->speed > 0.4 * d->speed_run )
			{
				int front = 0, rear = 0;
				for ( int i = 0; i < m->n; ++i )
				{
					if ( m->legs[i].planted )
					{
						if ( m->legs[i].mount_local.z > 0.05 ) front = 1;
						if ( m->legs[i].mount_local.z < -0.05 ) rear = 1;
					}
				}
				double sgn = ( front && !rear ) ? -0.75 : ( ( rear && !front ) ? 0.85 : 0.0 );
				if ( sgn != 0.0 )
				{
					double pitch = d->bound_pitch * GC_DEG2RAD * m->gait.stride_pitch * fwd_sgn * fwd_align * sgn;
					target = gc_rotate_axis( target, fr->right, pitch );
				}
			}
			if ( m->state == GC_STATE_MANTLE && m->mantle.active )
			{
				double pitch = 16.0 * GC_DEG2RAD * sin( GC_PI * gc_clamp( m->mantle.t, 0.0, 1.0 ) );
				target = gc_rotate_axis( target, fr->right, pitch );
			}
			else if ( !gc_mech_climbing_mode( m ) )
			{
				double lim_rel = ( d->look_tilt + 8.0 ) * GC_DEG2RAD;
				target = gc_rotate_towards( ground_up, gc_v3_norm( target, ground_up ), lim_rel );
			}
		}
	}

	target = gc_v3_norm( target, GC_WORLD_UP );
	GcV3 err = gc_v3_cross( fr->up, target );
	if ( gc_v3_dot( fr->up, target ) < 0.0 )
		err = gc_v3_len_sq( err ) > 1e-9 ? gc_v3_norm( err, fr->right ) : fr->right;
	if ( righting )
	{
		double f = 5.2 / sqrt( m->size );
		double w = fmin( GC_TAU * f, 0.78 / fmax( dt, 1e-4 ) );
		m->ang_vel = gc_v3_mul( gc_v3_add( m->ang_vel, gc_v3_mul( err, ( w * w ) * dt ) ), 1.0 / ( 1.0 + 2.0 * 0.95 * w * dt ) );
	}
	else
	{
		double f = ( gc_mech_climbing_mode( m ) ? 4.5 : 2.6 ) / sqrt( m->size );
		double w = GC_TAU * f;
		m->ang_vel = gc_v3_add( m->ang_vel, gc_v3_mul( gc_v3_sub( gc_v3_mul( err, w * w ), gc_v3_mul( m->ang_vel, 2.0 * 0.7 * w ) ), dt ) );
	}
	double ang = gc_v3_len( m->ang_vel );
	if ( ang * dt > 1e-7 )
		gc_frame_rotate( fr, gc_v3_mul( m->ang_vel, 1.0 / ang ), ang * dt );
	if ( righting )
	{
		m->land_right_t = fmax( 0.0, m->land_right_t - dt );
		if ( gc_angle_between( fr->up, target ) < 0.035 && gc_v3_len( m->ang_vel ) < 0.4 )
			m->land_right_t = 0.0;
	}
	m->on_wall = ( gc_mech_climbing_mode( m ) && m->state != GC_STATE_AIR && fr->up.y < 0.5 ) ? 1 : 0;
}

static void update_vis_offset( GcMech* m, double dt )
{
	GcV3 target = gc_v3_zero();
	if ( m->biped && m->state == GC_STATE_GROUND )
	{
		const GcLeg* sw_legs[GC_MAX_LEGS];
		int n_sw = 0;
		for ( int i = 0; i < m->n; ++i )
			if ( m->legs[i].swing.active ) sw_legs[n_sw++] = &m->legs[i];
		if ( n_sw == 1 )
			target.x = 0.13 * -(double)sw_legs[0]->side * ( 1.0 - 0.5 * fmin( 1.0, m->speed / 6.0 ) );
		target.y = -0.06 * cos( 4.0 * GC_PI * m->phase ) * fmin( 1.0, m->speed / 3.0 );
	}
	m->vis_offset = gc_v3_lerp( m->vis_offset, target, gc_damp( 7.0, dt ) );
}

// Snapshot floating feet in chassis-local space before integrating chassis
// movement so non-planted feet carry naturally with the hull during translation.
void gc_mech_update( GcMech* m, double dt )
{
	if ( !isfinite( dt ) || dt <= 0.0 )
		return;
	int prof = m->profile_enabled;
	double t0 = prof ? gc_time_sec() : 0.0;

	// Measure low ceilings overhead and a half-body-length ahead so leg swing
	// arcs cap their clearance below lintels and overhangs.
	if ( !gc_mech_climbing_mode( m ) )
	{
		m->ceil_t += dt;
		double p_probe = m->probe_period;
		if ( m->ceil_t >= p_probe - 1e-6 )
		{
			m->ceil_t = fmod( m->ceil_t, p_probe );
			double c_vals[2];
			int nc = 0;
			GcHit hc;
			if ( gc_world_raycast( &m->wq, m->pos, GC_WORLD_UP, 4.0, &hc ) )
				c_vals[nc++] = hc.point.y;
			GcV3 look_dir = ( m->speed > 0.5 )
								? gc_v3_norm( gc_project_plane( m->vt, GC_WORLD_UP ), gc_mech_heading_fwd( m ) )
								: gc_mech_heading_fwd( m );
			GcHit ha;
			if ( gc_world_raycast( &m->wq, gc_v3_add( m->pos, gc_v3_mul( look_dir, m->d->body_half[2] + 1.2 ) ),
								   GC_WORLD_UP, 4.0, &ha ) )
				c_vals[nc++] = ha.point.y;
			if ( nc == 0 )
			{
				m->has_ceil_y = 0;
			}
			else
			{
				m->has_ceil_y = 1;
				m->ceil_y = ( nc == 2 && c_vals[1] < c_vals[0] ) ? c_vals[1] : c_vals[0];
			}
		}
	}
	else
	{
		m->has_ceil_y = 0;
	}

	m->sup_t += dt;
	m->mantle_t += dt;
	for ( int i = 0; i < m->n; ++i )
	{
		if ( m->legs[i].planted )
		{
			m->legs[i].has_carry = 0;
		}
		else
		{
			m->legs[i].has_carry = 1;
			m->legs[i].carry = gc_frame_to_local( &m->frame, gc_v3_sub( m->legs[i].foot, m->pos ) );
		}
	}

	if ( m->inp.crouch > 0.5 && m->prev.crouch <= 0.5 )
		m->crouch_toggle = !m->crouch_toggle;
	if ( m->inp.climb > 0.5 && m->prev.climb <= 0.5 && m->d->climber )
		m->climb_enabled = !m->climb_enabled;
	if ( m->inp.gait > 0.5 && m->prev.gait <= 0.5 )
		cycle_gait_override( m );
	if ( m->inp.impact > 0.5 && m->prev.impact <= 0.5 )
		trigger_impact( m );
	int jump = ( m->inp.jump > 0.5 && m->prev.jump <= 0.5 ) ? 1 : 0;

	m->state_t += dt;
	m->absorb = fmax( 0.0, m->absorb - dt * ( 0.9 + 1.5 * m->absorb ) );
	m->brace = fmax( m->brace * exp( -1.4 * dt ), gc_clamp( gc_v3_len( m->slide_vel ) / 5.0, 0.0, 1.0 ) );
	m->crouch += ( ( m->crouch_toggle ? 1.0 : 0.0 ) - m->crouch ) * gc_damp( 6.0, dt );

	compute_drive_intent( m );
	select_mode( m, dt );
	if ( prof )
	{
		double t1 = gc_time_sec();
		m->prof[0] += t1 - t0;
		t0 = t1;
	}

	if ( m->state == GC_STATE_MANTLE )
		update_mantle( m, dt );
	else if ( m->state == GC_STATE_AIR )
		update_air( m, dt );
	else
		update_ground( m, dt, jump );

	update_orientation( m, dt );
	update_vis_offset( m, dt );
	if ( prof )
	{
		double t1 = gc_time_sec();
		m->prof[1] += t1 - t0;
	}
	gc_update_legs( m, dt );
	m->prev = m->inp;
}

const char* gc_mech_mode_label( const GcMech* m )
{
	if ( m->state == GC_STATE_AIR ) return "AIRBORNE";
	if ( gc_mech_wire_active( m ) ) return "WIRE";
	if ( m->state == GC_STATE_MANTLE ) return "MANTLE";
	if ( m->state == GC_STATE_CROUCHJUMP ) return "JUMP";
	if ( m->drifting ) return "SLIDE";
	if ( m->on_wall ) return "CLIMB";
	if ( m->crouch > 0.5 ) return "CROUCH";
	if ( m->mode == GC_MODE_RUN ) return "RUN";
	if ( m->mode == GC_MODE_SLOW ) return "SLOW WALK";
	return "WALK";
}

int gc_mech_flags( const GcMech* m )
{
	int f = 0;
	if ( m->drifting ) f |= 1;
	if ( m->on_wall ) f |= 2;
	if ( m->crouch > 0.5 ) f |= 4;
	if ( m->climb_enabled ) f |= 8;
	if ( m->support >= 0.0 ) f |= 16;
	if ( m->mode == GC_MODE_SLOW ) f |= 32;
	if ( m->mode == GC_MODE_RUN ) f |= 64;
	if ( gc_mech_wire_active( m ) ) f |= 128;
	if ( m->flight_t > 0.0 ) f |= 256;
	return f;
}

void gc_mech_gait_label( const GcMech* m, char* buf, size_t cap )
{
	char b[48];
	gc_blend_label( &m->blend, b, sizeof( b ) );
	const char* r = m->selector.reason;
	if ( r[0] != '\0' &&
		 strcmp( r, "walking" ) != 0 &&
		 strcmp( r, "fast" ) != 0 &&
		 strcmp( r, "galloping" ) != 0 &&
		 strcmp( r, "careful" ) != 0 &&
		 strcmp( r, "forced" ) != 0 )
	{
		snprintf( buf, cap, "%s (%s)", b, r );
	}
	else
	{
		snprintf( buf, cap, "%s", b );
	}
}

// Serialize body pose, state flags, and solved world joint positions into a
// tight float array matching GcPoseHeader + N legs. Returns 0 if buffer is small.
int gc_mech_pack( const GcMech* m, float* out, int max_floats )
{
	int need = GC_HEADER_FLOATS + m->n * ( m->joints_per_leg * 3 + GC_LEG_EXTRA_FLOATS );
	if ( max_floats < need )
		return 0;
	const GcFrame* fr = &m->frame;
	GcV3 p = gc_v3_add( m->pos, gc_frame_to_world( fr, m->vis_offset ) );
	int n_pl = 0;
	for ( int i = 0; i < m->n; ++i )
		if ( m->legs[i].planted ) n_pl++;

	int k = 0;
	out[k++] = (float)p.x; out[k++] = (float)p.y; out[k++] = (float)p.z;
	out[k++] = (float)fr->right.x; out[k++] = (float)fr->right.y; out[k++] = (float)fr->right.z;
	out[k++] = (float)fr->up.x; out[k++] = (float)fr->up.y; out[k++] = (float)fr->up.z;
	out[k++] = (float)fr->fwd.x; out[k++] = (float)fr->fwd.y; out[k++] = (float)fr->fwd.z;
	out[k++] = (float)m->speed;
	out[k++] = (float)m->state;
	out[k++] = (float)gc_mech_flags( m );
	out[k++] = (float)n_pl;
	out[k++] = (float)m->vel.x; out[k++] = (float)m->vel.y; out[k++] = (float)m->vel.z;
	out[k++] = (float)m->phase;
	out[k++] = (float)m->drift_amt;
	out[k++] = (float)m->gait_idx;
	out[k++] = (float)m->support;
	out[k++] = (float)m->cycle;
	if ( gc_mech_wire_active( m ) )
	{
		out[k++] = 1.0f;
		out[k++] = (float)m->mantle.top.x;
		out[k++] = (float)m->mantle.top.y;
		out[k++] = (float)m->mantle.top.z;
	}
	else
	{
		out[k++] = 0.0f; out[k++] = 0.0f; out[k++] = 0.0f; out[k++] = 0.0f;
	}

	int J = m->joints_per_leg;
	for ( int li = 0; li < m->n; ++li )
	{
		const GcLeg* l = &m->legs[li];
		for ( int ji = 0; ji < J; ++ji )
		{
			GcV3 j = ( ji < l->n_joints ) ? l->joints[ji] : l->foot;
			out[k++] = (float)j.x;
			out[k++] = (float)j.y;
			out[k++] = (float)j.z;
		}
		out[k++] = (float)l->plane_normal.x;
		out[k++] = (float)l->plane_normal.y;
		out[k++] = (float)l->plane_normal.z;
		out[k++] = l->planted ? 1.0f : 0.0f;
		out[k++] = l->sliding ? 1.0f : 0.0f;
		out[k++] = l->swing.active ? (float)l->swing.t : -1.0f;
	}
	return k;
}

static int push_dbg_line( float* out, int cur, int max_floats, GcV3 a, GcV3 b, int role )
{
	if ( cur + GC_DEBUG_LINE_FLOATS > max_floats )
		return cur;
	float* d = out + cur;
	d[0] = (float)a.x; d[1] = (float)a.y; d[2] = (float)a.z;
	d[3] = (float)b.x; d[4] = (float)b.y; d[5] = (float)b.z;
	d[6] = (float)role;
	d[7] = 0.0f;
	d[8] = 0.0f;
	return cur + GC_DEBUG_LINE_FLOATS;
}

static int push_dbg_cross( float* out, int cur, int max_floats, const GcFrame* fr, GcV3 p, double sz, int role )
{
	cur = push_dbg_line( out, cur, max_floats, gc_v3_sub( p, gc_v3_mul( fr->right, sz ) ),
						 gc_v3_add( p, gc_v3_mul( fr->right, sz ) ), role );
	cur = push_dbg_line( out, cur, max_floats, gc_v3_sub( p, gc_v3_mul( fr->fwd, sz ) ),
						 gc_v3_add( p, gc_v3_mul( fr->fwd, sz ) ), role );
	return cur;
}

int gc_mech_debug_lines( const GcMech* m, float* out, int max_floats )
{
	int cur = 0;
	const GcFrame* fr = &m->frame;
	GcV3 up_p = gc_mech_probe_up( m );
	GcV3 p = m->pos;
	cur = push_dbg_line( out, cur, max_floats, p, gc_v3_add( p, gc_v3_mul( fr->right, 1.2 ) ), GC_DBG_AXIS_X );
	cur = push_dbg_line( out, cur, max_floats, p, gc_v3_add( p, gc_v3_mul( fr->up, 1.2 ) ), GC_DBG_AXIS_Y );
	cur = push_dbg_line( out, cur, max_floats, p, gc_v3_add( p, gc_v3_mul( fr->fwd, 1.2 ) ), GC_DBG_AXIS_Z );

	GcV3 t1 = gc_v3_norm( gc_project_plane( fr->fwd, up_p ), gc_v3( 0.0, 0.0, 1.0 ) );
	GcV3 t2 = gc_v3_cross( up_p, t1 );
	double zone = m->d->zone;

	for ( int i = 0; i < m->n; ++i )
	{
		const GcLeg* l = &m->legs[i];
		GcV3 c = l->ideal_now;
		GcV3 prev = gc_v3_zero();
		for ( int k = 0; k < 17; ++k )
		{
			double a = (double)k * GC_TAU / 16.0;
			GcV3 q = gc_v3_add( c, gc_v3_add( gc_v3_mul( t1, cos( a ) * zone ), gc_v3_mul( t2, sin( a ) * zone ) ) );
			if ( k > 0 )
				cur = push_dbg_line( out, cur, max_floats, prev, q, GC_DBG_LINE_DIM );
			prev = q;
		}
		cur = push_dbg_cross( out, cur, max_floats, fr, l->ideal_pred, 0.15, GC_DBG_LINE );
		if ( l->swing.active )
		{
			cur = push_dbg_cross( out, cur, max_floats, fr, l->swing.target, 0.2, GC_DBG_HIGHLIGHT );
			GcV3 pts[13];
			gc_leg_curve_world( l, 12, pts );
			for ( int k = 0; k < 12; ++k )
				cur = push_dbg_line( out, cur, max_floats, pts[k], pts[k + 1], GC_DBG_INFO );
		}
		cur = push_dbg_line( out, cur, max_floats, l->foot,
							 gc_v3_add( l->foot, gc_v3_mul( l->foot_normal, 0.5 ) ),
							 l->planted ? GC_DBG_NORMAL : GC_DBG_LINE_DIM );
	}

	double y_sum = 0.0;
	int y_cnt = 0;
	for ( int i = 0; i < m->n; ++i )
	{
		if ( m->legs[i].planted )
		{
			y_sum += m->legs[i].foot.y;
			y_cnt++;
		}
	}
	if ( m->hull_n > 0 && y_cnt > 0 )
	{
		double y = y_sum / (double)y_cnt + 0.05;
		int role = m->support >= 0.0 ? GC_DBG_GOOD : GC_DBG_ERROR;
		for ( int i = 0; i < m->hull_n; ++i )
		{
			GcV2 a = m->hull[i], b = m->hull[( i + 1 ) % m->hull_n];
			cur = push_dbg_line( out, cur, max_floats, gc_v3( a.x, y, a.z ), gc_v3( b.x, y, b.z ), role );
		}
		cur = push_dbg_line( out, cur, max_floats, p, gc_v3( p.x, y, p.z ), GC_DBG_LINE );
		cur = push_dbg_cross( out, cur, max_floats, fr, gc_v3( p.x, y, p.z ), 0.25, GC_DBG_LINE );
	}
	return cur;
}

void gc_mech_stats_fill( const GcMech* m, float sim_ms, int total_rays, GcMechStats* out )
{
	static const char* const state_names[4] = { "ground", "air", "mantle", "jump-crouch" };
	snprintf( out->mode, sizeof( out->mode ), "%s", gc_mech_mode_label( m ) );
	gc_mech_gait_label( m, out->gait, sizeof( out->gait ) );
	out->gait_override = m->gait_override >= 0 ? 1 : 0;
	int st = ( m->state >= 0 && m->state < 4 ) ? m->state : 0;
	snprintf( out->state, sizeof( out->state ), "%s", state_names[st] );
	out->duty = (float)m->duty;
	out->cycle = (float)m->cycle;
	double ms = 0.0;
	for ( int i = 0; i < m->n; ++i )
		if ( m->legs[i].stretch > ms ) ms = m->legs[i].stretch;
	out->max_stretch = (float)ms;
	out->sim_ms = sim_ms;
	out->rays = total_rays;
	out->climb_enabled = m->climb_enabled;
	out->crouch_toggle = m->crouch_toggle;
}
