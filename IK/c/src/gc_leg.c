// Single-leg runtime state, quintic Hermite swing planning/retargeting, and obstacle clearance.

#include "gc_internal.h"

// Quintic blend s(u) with zero first and second derivatives at endpoints and its analytical integral,
// so swing position and velocity remain C1-continuous across retargets.
static void quintic( double u, double* blend, double* integ )
{
	double u2 = u * u;
	double u3 = u2 * u;
	*blend = u3 * ( 10.0 - 15.0 * u + 6.0 * u2 );
	*integ = u2 * u2 * ( 2.5 - 3.0 * u + u2 );
}

static void swing_shaping( const GcSwing* sw, double t, double dur, GcV3* out_pos, GcV3* out_vel )
{
	double q = t * ( 1.0 - t );
	double d = dur > 1e-5 ? dur : 1e-5;
	GcV3 lift = gc_v3_mul( sw->up, sw->lift );
	GcV3 pos = gc_v3_mul( lift, 64.0 * q * q * q );
	GcV3 vel = gc_v3_mul( lift, 192.0 * q * q * ( 1.0 - 2.0 * t ) / d );
	if ( gc_v3_len_sq( sw->bow ) > 1e-10 )
	{
		double span = ( 1.0 - sw->plan_t ) > 1e-5 ? ( 1.0 - sw->plan_t ) : 1e-5;
		double u = gc_clamp( ( t - sw->plan_t ) / span, 0.0, 1.0 );
		double qb = u * ( 1.0 - u );
		pos = gc_v3_add( pos, gc_v3_mul( sw->bow, 16.0 * qb * qb ) );
		vel = gc_v3_add( vel, gc_v3_mul( sw->bow, 32.0 * qb * ( 1.0 - 2.0 * u ) / ( d * span ) ) );
	}
	*out_pos = pos;
	*out_vel = vel;
}

static void swing_plan( GcSwing* sw, GcV3 foot, GcV3 velocity, double reach, const GcV3* opt_end_velocity )
{
	sw->plan_t = sw->t;
	sw->plan_dur = sw->dur;
	if ( opt_end_velocity )
		sw->end_velocity = *opt_end_velocity;
	double duration = fmax( 1e-6, ( 1.0 - sw->t ) * sw->dur );
	GcV3 offset, offset_vel;
	swing_shaping( sw, sw->t, sw->dur, &offset, &offset_vel );
	sw->origin = gc_v3_sub( foot, offset );
	sw->start_velocity = gc_v3_sub( velocity, offset_vel );
	GcV3 travel = gc_v3_sub( sw->target, sw->origin );
	double speed = fmax( gc_v3_len( travel ) / duration, 1e-6 );
	double start_spd = gc_v3_len( sw->start_velocity );
	double end_spd = gc_v3_len( sw->end_velocity );
	sw->takeoff_time = fmin( 0.46 * duration, GC_TAKEOFF_RAMP_REACH * reach / fmax( speed, start_spd ) );
	sw->landing_time = fmin( 0.46 * duration, GC_LANDING_RAMP_REACH * reach / fmax( speed, end_spd ) );
	double cruise_time = fmax( 1e-7, duration - 0.5 * ( sw->takeoff_time + sw->landing_time ) );
	GcV3 num = gc_v3_sub( gc_v3_sub( travel, gc_v3_mul( sw->start_velocity, 0.5 * sw->takeoff_time ) ),
						  gc_v3_mul( sw->end_velocity, 0.5 * sw->landing_time ) );
	sw->cruise = gc_v3_mul( num, 1.0 / cruise_time );
}

static void swing_sample( const GcSwing* sw, double t, GcV3* out_pt, GcV3* out_vel )
{
	if ( t >= 1.0 )
	{
		*out_pt = sw->target;
		*out_vel = sw->end_velocity;
		return;
	}
	double duration = ( 1.0 - sw->plan_t ) * sw->plan_dur;
	double elapsed = fmax( 0.0, ( t - sw->plan_t ) * sw->plan_dur );
	double left = duration - elapsed;
	GcV3 point, velocity;
	if ( sw->takeoff_time > 1e-9 && elapsed < sw->takeoff_time )
	{
		double u = elapsed / sw->takeoff_time;
		double blend, integ;
		quintic( u, &blend, &integ );
		double cruise_w = sw->takeoff_time * integ;
		double initial_w = elapsed - cruise_w;
		point = gc_v3_add( sw->origin,
						   gc_v3_add( gc_v3_mul( sw->start_velocity, initial_w ),
									  gc_v3_mul( sw->cruise, cruise_w ) ) );
		velocity = gc_v3_lerp( sw->start_velocity, sw->cruise, blend );
	}
	else if ( sw->landing_time > 1e-9 && left < sw->landing_time )
	{
		double u = left / sw->landing_time;
		double blend, integ;
		quintic( u, &blend, &integ );
		GcV3 rem_disp = gc_v3_add( gc_v3_mul( sw->end_velocity, left ),
								   gc_v3_mul( gc_v3_sub( sw->cruise, sw->end_velocity ), sw->landing_time * integ ) );
		point = gc_v3_sub( sw->target, rem_disp );
		velocity = gc_v3_lerp( sw->end_velocity, sw->cruise, blend );
	}
	else
	{
		double initial_w = 0.5 * sw->takeoff_time;
		double cruise_w = elapsed - initial_w;
		point = gc_v3_add( sw->origin,
						   gc_v3_add( gc_v3_mul( sw->start_velocity, initial_w ),
									  gc_v3_mul( sw->cruise, cruise_w ) ) );
		velocity = sw->cruise;
	}
	GcV3 offset, offset_vel;
	swing_shaping( sw, t, sw->plan_dur, &offset, &offset_vel );
	*out_pt = gc_v3_add( point, offset );
	*out_vel = gc_v3_add( velocity, offset_vel );
}

void gc_leg_init( GcLeg* l, int idx, const GcLegDef* ldef, const GcMechDef* d, double size )
{
	memset( l, 0, sizeof( *l ) );
	l->idx = idx;
	l->pair = idx / 2;
	l->side = ldef->side;
	double mx = ldef->mount[0], my = ldef->mount[1], mz = ldef->mount[2];
	double rx = ldef->rest[0], rz = ldef->rest[1];
	l->mount_local = gc_v3( mx * size, my * size, mz * size );
	l->rest_xz[0] = rx * size;
	l->rest_xz[1] = rz * size;
	const int* sides = ldef->has_sides ? ldef->sides : d->sides;
	GcV3 rest_dir = gc_v3_norm( gc_v3( rx - mx, 0.0, rz - mz ), gc_v3( (double)l->side, 0.0, 0.0 ) );
	double lengths[GC_MAX_SEGS];
	for ( int i = 0; i < d->n_segs; ++i )
		lengths[i] = d->lengths[i] * size;
	const GcLimits* lim = ldef->has_limits ? &ldef->limits : &d->limits;
	gc_legik_init( &l->ik, d->plane, d->coxa * size, lengths, d->n_segs, sides, rest_dir, lim );
	gc_legik_init( &l->ik_probe, d->plane, d->coxa * size, lengths, d->n_segs, sides, rest_dir, lim );
	l->reach = l->ik.reach;
	l->is_front = mz > 0.05;
	double w_last = d->widths[d->n_segs - 1];
	l->foot_r = fmin( 0.14, w_last * 0.6 );
	l->foot_normal = gc_v3( 0.0, 1.0, 0.0 );
	l->planted = 1;
	l->plane_normal = gc_v3( 1.0, 0.0, 0.0 );
	l->kc_t = 1.0;
}

GcV3 gc_leg_rest_local( const GcLeg* l, double ride, double spread )
{
	return gc_v3( l->rest_xz[0] * spread, -ride, l->rest_xz[1] * ( 1.0 + ( spread - 1.0 ) * 0.5 ) );
}

// Raises swing arc height to clear ledges/rocks along the straight chord without punching
// into low ceilings or doorframes.
static void shape_swing( GcLeg* l, GcWorldQueryCtx* wq, double clearance_probe, GcV3 start,
						 int has_ceil_y, double max_ceil_y )
{
	GcSwing* sw = &l->swing;
	GcV3 p0 = start, p3 = sw->target, up = sw->up;
	double h0 = gc_v3_dot( p0, up );
	double h3 = gc_v3_dot( p3, up );
	int has_min_ceil = has_ceil_y;
	double min_ceil = max_ceil_y;
	double lift = sw->lift;
	if ( clearance_probe > 0.0 )
	{
		GcV3 d = gc_v3_sub( p3, p0 );
		GcV3 side = gc_v3_mul( gc_v3_norm( gc_v3_cross( d, up ), gc_v3( 1.0, 0.0, 0.0 ) ), l->foot_r );
		double d_len = gc_v3_len( d );
		int chord_clear = ( h0 >= h3 - 0.08 && d_len > 1e-4 ) ? 1 : 0;
		if ( chord_clear )
		{
			GcV3 dh = gc_v3_sub( d, gc_v3_mul( up, h3 - h0 ) );
			double dh_len = gc_v3_len( dh );
			if ( dh_len > 1e-4 )
			{
				double base_y = fmin( h0, h3 ) + 0.06;
				GcV3 p0_low = gc_v3_add( p0, gc_v3_mul( up, base_y - h0 ) );
				GcV3 dir_h = gc_v3_mul( dh, 1.0 / dh_len );
				GcV3 offs[3] = { gc_v3_zero(), side, gc_v3_mul( side, -1.0 ) };
				GcHit tmp;
				for ( int oi = 0; oi < 3; ++oi )
				{
					if ( gc_world_raycast( wq, gc_v3_add( p0_low, offs[oi] ), dir_h, dh_len, &tmp ) )
					{
						chord_clear = 0;
						break;
					}
				}
				if ( chord_clear )
				{
					double knee_y = fmin( h0, h3 ) + fmax( 0.35, 0.55 * l->foot_r + 0.28 );
					if ( ( !has_min_ceil || knee_y < min_ceil - 0.08 ) &&
						 gc_world_raycast( wq, gc_v3_add( p0, gc_v3_mul( up, knee_y - h0 ) ), dir_h, dh_len, &tmp ) )
					{
						chord_clear = 0;
					}
				}
			}
			if ( chord_clear && fabs( h0 - h3 ) >= 0.08 )
			{
				GcHit tmp;
				if ( gc_world_raycast( wq, gc_v3_add( p0, gc_v3_mul( up, 0.05 ) ),
									   gc_v3_mul( d, 1.0 / d_len ), d_len, &tmp ) )
				{
					chord_clear = 0;
				}
			}
		}
		if ( !chord_clear )
		{
			static const double ss[7] = { 0.15, 0.27, 0.39, 0.5, 0.61, 0.73, 0.85 };
			GcV3 offs[3] = { gc_v3_zero(), side, gc_v3_mul( side, -1.0 ) };
			for ( int si = 0; si < 7; ++si )
			{
				double s = ss[si];
				GcV3 base = gc_v3_lerp( p0, p3, s );
				for ( int oi = 0; oi < 3; ++oi )
				{
					GcV3 q = gc_v3_add( base, offs[oi] );
					double qy = gc_v3_dot( q, up );
					double cp = clearance_probe;
					if ( has_ceil_y )
						cp = fmin( cp, fmax( 0.0, max_ceil_y - qy - 0.08 ) );
					if ( cp > 0.04 )
					{
						GcHit hc;
						if ( gc_world_raycast( wq, gc_v3_add( q, gc_v3_mul( up, l->foot_r + 0.04 ) ), up, cp, &hc ) )
						{
							double cy = gc_v3_dot( hc.point, up );
							min_ceil = has_min_ceil ? fmin( min_ceil, cy ) : cy;
							has_min_ceil = 1;
							cp = fmin( cp, fmax( 0.0, cy - qy - 0.06 ) );
						}
					}
					if ( cp <= 0.02 )
						continue;
					GcV3 o = gc_v3_add( q, gc_v3_mul( up, cp ) );
					GcHit h;
					if ( gc_world_raycast( wq, o, gc_v3_mul( up, -1.0 ), cp + 0.5, &h ) && gc_v3_dot( h.normal, up ) > 0.45 )
					{
						double above = gc_v3_dot( h.point, up ) - ( h0 + ( h3 - h0 ) * s );
						if ( above > 0.0 && above <= clearance_probe )
							lift = fmax( lift, above + 0.35 * sw->lift + l->foot_r + 0.08 );
					}
				}
			}
		}
		else if ( !has_ceil_y )
		{
			GcV3 mid_q = gc_v3_lerp( p0, p3, 0.5 );
			GcHit hc;
			if ( gc_world_raycast( wq, gc_v3_add( mid_q, gc_v3_mul( up, l->foot_r + 0.04 ) ), up, lift + 0.35, &hc ) )
			{
				min_ceil = gc_v3_dot( hc.point, up );
				has_min_ceil = 1;
			}
		}
	}
	if ( has_min_ceil )
		lift = fmin( lift, fmax( 0.03, min_ceil - fmax( h0, h3 ) - l->foot_r - 0.12 ) );
	sw->lift = lift;
}

void gc_leg_begin_swing( GcLeg* l, GcV3 target, GcV3 normal, double dur, double lift, GcV3 up,
						 int has_ground, GcWorldQueryCtx* wq, double clearance_probe,
						 GcV3 start_vel, GcV3 end_vel, GcV3 bow, int has_ceil_y, double max_ceil_y )
{
	GcSwing* sw = &l->swing;
	memset( sw, 0, sizeof( *sw ) );
	sw->active = 1;
	sw->start_w = l->foot;
	sw->target = target;
	sw->normal = normal;
	sw->dur = dur > 0.05 ? dur : 0.05;
	sw->plan_dur = 1.0;
	sw->up = up;
	sw->has_ground = has_ground;
	sw->retarget_cd = 0.08;
	sw->lift = lift;
	sw->bow = bow;
	sw->end_velocity = end_vel;
	l->planted = 0;
	l->searching = 0;
	l->sliding = 0;
	shape_swing( l, wq, clearance_probe, l->foot, has_ceil_y, max_ceil_y );
	swing_plan( sw, l->foot, start_vel, l->reach, &sw->end_velocity );
}

void gc_leg_curve_world( const GcLeg* l, int n_pts, GcV3* out_pts )
{
	out_pts[0] = l->foot;
	if ( !l->swing.active )
		return;
	for ( int k = 1; k <= n_pts; ++k )
	{
		double t = l->swing.t + ( 1.0 - l->swing.t ) * (double)k / (double)n_pts;
		GcV3 vel_unused;
		swing_sample( &l->swing, t, &out_pts[k], &vel_unused );
	}
}

int gc_leg_update_swing( GcLeg* l, double dt )
{
	GcSwing* sw = &l->swing;
	if ( sw->plan_dur != sw->dur )
	{
		GcV3 pt_unused, velocity;
		swing_sample( sw, sw->t, &pt_unused, &velocity );
		sw->bow = gc_v3_mul( sw->bow, fmax( 0.0, 1.0 - sw->t ) );
		swing_plan( sw, l->foot, velocity, l->reach, NULL );
	}
	sw->t = fmin( 1.0, sw->t + dt / sw->dur );
	GcV3 vel_unused;
	swing_sample( sw, sw->t, &l->foot, &vel_unused );
	if ( sw->t >= 1.0 )
	{
		l->foot = sw->target;
		int hg = sw->has_ground;
		GcV3 n = sw->normal;
		sw->active = 0;
		if ( hg )
		{
			l->planted = 1;
			l->foot_normal = n;
			return 1;
		}
		l->planted = 0;
		l->searching = 1;
		l->search_cd = 0.0;
	}
	return 0;
}

// Keeps a swinging foot from clipping through obstacles, ledges, or ceilings using sphere extraction
// plus a downward surface probe clamped per step so feet rise smoothly instead of snapping.
double gc_leg_keep_clear( GcLeg* l, GcWorldQueryCtx* wq, GcV3 up, double max_step,
						  int has_ceil_y, double max_ceil_y, double dt )
{
	GcSwing* sw = l->swing.active ? &l->swing : NULL;
	double r = l->foot_r;
	int embedded = 0;
	double need_up = 0.0;
	GcV3 c = gc_v3_add( l->foot, gc_v3_mul( up, r ) );
	GcV3 c2, n;
	if ( gc_world_sphere_push( wq, c, r, &c2, &n ) )
	{
		GcV3 push = gc_v3_sub( c2, c );
		if ( gc_v3_len_sq( push ) > 1e-8 )
		{
			embedded = 1;
			double p_up = gc_v3_dot( push, up );
			GcV3 p_lat = gc_v3_sub( push, gc_v3_mul( up, p_up ) );
			if ( ( !sw || sw->t < 0.96 ) && gc_v3_len_sq( p_lat ) > 1e-8 )
				l->foot = gc_v3_add( l->foot, p_lat );
			if ( p_up < -1e-4 )
				l->foot = gc_v3_add( l->foot, gc_v3_mul( up, p_up ) );
			else if ( p_up > need_up )
				need_up = p_up;
		}
	}
	double step_lim = max_step > 0.0 ? max_step : 0.8;
	double probe_up = embedded ? fmin( 0.85, step_lim * 0.75 + 0.15 ) : ( r + 0.28 );
	double fy = gc_v3_dot( l->foot, up );
	int has_local_ceil = has_ceil_y;
	double local_ceil = max_ceil_y;
	int run_ray = 1;
	if ( sw )
	{
		l->kc_t += dt;
		run_ray = ( l->kc_t >= 1.0 / 30.0 - 1e-6 ) || embedded || sw->t > 0.88;
	}
	if ( run_ray )
	{
		l->kc_t = 0.0;
		if ( has_ceil_y || embedded )
		{
			GcHit hc;
			if ( gc_world_raycast( wq, gc_v3_add( l->foot, gc_v3_mul( up, r + 0.02 ) ), up, probe_up + 0.25, &hc ) )
			{
				double cy = gc_v3_dot( hc.point, up );
				local_ceil = has_local_ceil ? fmin( local_ceil, cy ) : cy;
				has_local_ceil = 1;
			}
		}
		if ( has_local_ceil )
			probe_up = fmin( probe_up, fmax( 0.0, local_ceil - fy - 0.06 ) );
		if ( probe_up > r * 0.5 )
		{
			GcHit h;
			if ( gc_world_raycast( wq, gc_v3_add( l->foot, gc_v3_mul( up, probe_up ) ),
								   gc_v3_mul( up, -1.0 ), probe_up + r + 0.04, &h ) &&
				 gc_v3_dot( h.normal, up ) > 0.50 )
			{
				double bell = sw ? ( 4.0 * sw->t * ( 1.0 - sw->t ) ) : 0.0;
				double gap = !sw ? r : ( 0.02 + ( r - 0.02 ) * bell * bell );
				double up_clear = fmax( 0.0, gc_v3_dot( gc_v3_sub( h.point, l->foot ), up ) + gap );
				if ( up_clear > need_up )
					need_up = up_clear;
			}
		}
	}
	if ( has_local_ceil )
		need_up = fmin( need_up, fmax( 0.0, local_ceil - fy - r - 0.06 ) );
	double up_cap;
	if ( embedded )
		up_cap = fmin( step_lim + 0.25, fmax( 0.32, 22.0 * dt ) );
	else
	{
		up_cap = fmin( 0.45, step_lim * 0.6 );
		double m = fmax( 0.22, 16.0 * dt );
		if ( m > up_cap && step_lim * 0.6 >= 0.45 )
			up_cap = fmin( 0.45, m );
		/* Python: min(0.45, step_lim * 0.6, max(0.22, 16.0 * dt)) */
		up_cap = fmin( fmin( 0.45, step_lim * 0.6 ), fmax( 0.22, 16.0 * dt ) );
	}
	if ( need_up > up_cap )
		need_up = up_cap;
	if ( need_up > 0.0 )
		l->foot = gc_v3_add( l->foot, gc_v3_mul( up, need_up ) );
	return need_up;
}

// Re-anchors the remaining swing curve onto a new landing spot while preserving current world velocity.
void gc_leg_retarget( GcLeg* l, GcV3 target, GcV3 normal, int has_ground, GcWorldQueryCtx* wq,
					  double clearance_probe, GcV3 end_vel, GcV3 bow, int has_ceil_y, double max_ceil_y )
{
	GcSwing* sw = &l->swing;
	GcV3 pt_unused, velocity;
	swing_sample( sw, sw->t, &pt_unused, &velocity );
	sw->target = target;
	sw->normal = normal;
	sw->has_ground = has_ground;
	sw->bow = bow;
	sw->end_velocity = end_vel;
	shape_swing( l, wq, clearance_probe, l->foot, has_ceil_y, max_ceil_y );
	swing_plan( sw, l->foot, velocity, l->reach, &sw->end_velocity );
}

void gc_leg_solve( GcLeg* l, GcMech* m, GcV3 mount_l, double dt )
{
	const GcFrame* fr = &m->frame;
	GcV3 pos = m->pos;
	GcV3 r = fr->right, u = fr->up, f = fr->fwd;
	double rx = r.x, ry = r.y, rz = r.z;
	double ux = u.x, uy = u.y, uz = u.z;
	double fx = f.x, fy = f.y, fz = f.z;
	double px = pos.x, py = pos.y, pz = pos.z;
	double dx = l->foot.x - px, dy = l->foot.y - py, dz = l->foot.z - pz;
	GcV3 foot_l = gc_v3( dx * rx + dy * ry + dz * rz,
						 dx * ux + dy * uy + dz * uz,
						 dx * fx + dy * fy + dz * fz );
	int nj = gc_legik_solve( &l->ik, mount_l, &GC_LOCAL_FRAME, foot_l, l->plane_roll, l->joints_local );
	l->n_joints_local = nj;
	l->n_joints = nj;
	for ( int i = 0; i < nj; ++i )
	{
		GcV3 j = l->joints_local[i];
		l->joints[i] = gc_v3( px + rx * j.x + ux * j.y + fx * j.z,
							  py + ry * j.x + uy * j.y + fy * j.z,
							  pz + rz * j.x + uz * j.y + fz * j.z );
	}
	GcV3 pn = l->ik.plane_normal;
	l->plane_normal = gc_v3( -( rx * pn.x + ux * pn.y + fx * pn.z ),
							 -( ry * pn.x + uy * pn.y + fy * pn.z ),
							 -( rz * pn.x + uz * pn.y + fz * pn.z ) );
	l->mount_world = l->joints[0];
	double prev = l->stretch;
	l->stretch = gc_v3_dist( l->mount_world, l->foot ) / l->reach;
	if ( l->ik.err > 0.01 )
	{
		if ( l->sliding )
		{
			l->foot = l->joints[nj - 1];
			l->stretch = gc_v3_dist( l->mount_world, l->foot ) / l->reach;
		}
		else
		{
			double alt = 0.97 + 2.0 * l->ik.err / l->reach;
			if ( alt > l->stretch )
				l->stretch = alt;
		}
	}
	l->near_limit = ( l->ik.limit_margin_deg < 6.0 || l->ik.coxa_ratio > 0.88 ) ? 1 : 0;
	if ( l->planted && l->stance_t > 0.0 )
	{
		double rate = ( l->stretch - prev ) / fmax( dt, 1e-4 );
		l->stretch_rate += ( rate - l->stretch_rate ) * gc_damp( 26.0, dt );
	}
	else
	{
		l->stretch_rate = 0.0;
	}
}

// Reprojects cached local joint positions into the current body transform when decoupled IK skips a tick.
void gc_leg_reproject_cached( GcLeg* l, const GcMech* m )
{
	int nj = l->n_joints_local;
	if ( nj <= 0 )
		return;
	const GcFrame* fr = &m->frame;
	GcV3 pos = m->pos;
	GcV3 r = fr->right, u = fr->up, f = fr->fwd;
	double rx = r.x, ry = r.y, rz = r.z;
	double ux = u.x, uy = u.y, uz = u.z;
	double fx = f.x, fy = f.y, fz = f.z;
	double px = pos.x, py = pos.y, pz = pos.z;
	GcV3 ml = gc_v3_add( l->mount_local, m->vis_offset );
	GcV3 p0 = l->joints_local[0];
	double dx = ml.x - p0.x, dy = ml.y - p0.y, dz = ml.z - p0.z;
	l->n_joints = nj;
	for ( int i = 0; i < nj; ++i )
	{
		GcV3 j = l->joints_local[i];
		double jx = j.x + dx, jy = j.y + dy, jz = j.z + dz;
		l->joints[i] = gc_v3( px + rx * jx + ux * jy + fx * jz,
							  py + ry * jx + uy * jy + fy * jz,
							  pz + rz * jx + uz * jy + fz * jz );
	}
	l->mount_world = l->joints[0];
	l->stretch = gc_v3_dist( l->mount_world, l->foot ) / l->reach;
}
