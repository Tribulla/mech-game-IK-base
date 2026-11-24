// Step scheduling, swing-arc updating, support/stability guards, and flight
// phase timing across all leg configurations (biped through octopod).

#include "gc_internal.h"
#include "gc_thread.h"

typedef struct PredCtx {
	GcFrame fr;
	double ride;
	int dynamic;
	GcV3 travel;
	GcV3 lead;
	int has_dir;
	GcV3 direction;
	double lead_cap;
} PredCtx;

static int fast_hopper( const GcMech* m )
{
	return ( m->d->bound * m->gait.leap >= 0.015 ) ? 1 : 0;
}

double gc_v_leg( const GcMech* m )
{
	return 1.0 * m->reach_avg / fmax( 0.05, m->duty * m->gait.cycle_min * sqrt( m->size ) );
}

int gc_galloping_fast( const GcMech* m, int check_blend )
{
	const GcGaitBlend* b = &m->blend;
	int flying = ( m->gait.min_support == 0 ) ||
				 ( check_blend && b->t < 1.0 && ( b->src.min_support == 0 || b->dst.min_support == 0 ) );
	return ( flying && m->speed >= fmax( 0.5 * gc_v_leg( m ), m->v_walk ) ) ? 1 : 0;
}

int gc_may_flight( const GcMech* m )
{
	if ( m->duty >= 0.48 || m->gait.min_support != 0 || m->calm_gait || m->braking ||
		 m->state != GC_STATE_GROUND || m->on_wall || m->drifting || m->crouch >= 0.5 )
		return 0;
	if ( m->gait.has_duty_fast && !fast_hopper( m ) )
		return 0;
	double b = m->d->bound * m->gait.leap;
	double vl = gc_v_leg( m );
	double k = gc_clamp( ( m->speed - 0.3 * vl ) / ( 0.7 * vl ), 0.0, 1.0 );
	double h_min = m->gait.has_duty_fast ? 0.016 : 0.03;
	return ( b * k > h_min ) ? 1 : 0;
}

static double lead_cap( const GcMech* m )
{
	return fmin( m->d->zone * 1.2 * m->gait.reach, m->reach_avg * 0.75 );
}

GcV3 gc_planar_travel( const GcMech* m, double seconds )
{
	if ( m->drifting )
		return gc_v3_mul( m->vt, seconds );
	GcV3 travel = gc_v3_mul( m->vt, seconds );
	if ( m->state != GC_STATE_GROUND || gc_mech_climbing_mode( m ) )
		return travel;
	GcV3 change = gc_v3_sub( m->drive_target, m->vt );
	double delta = gc_v3_len( change );
	if ( m->drive_accel > 0.0 && delta > 1e-5 )
	{
		double ramp = fmin( seconds, delta / m->drive_accel );
		travel = gc_v3_add( travel, gc_v3_mul( change, m->drive_accel * ramp * ( seconds - 0.5 * ramp ) / delta ) );
	}
	if ( gc_galloping_fast( m, 0 ) || ( m->speed > 4.0 && fabs( m->yaw_rate ) > 0.2 ) )
		travel = gc_turn_displacement( travel, GC_WORLD_UP, -m->yaw_rate * seconds );
	return travel;
}

static double leg_fwd_room( const GcMech* m, const GcLeg* l, GcV3 dir_l, double ride )
{
	double ox = l->rest_xz[0] - l->mount_local.x;
	double oy = -ride - l->mount_local.y;
	double oz = l->rest_xz[1] - l->mount_local.z;
	double dx = dir_l.x, dy = dir_l.y, dz = dir_l.z;
	double along = ox * dx + oy * dy + oz * dz;
	double off2 = ox * ox + oy * oy + oz * oz;
	double rlim = 0.94 * l->reach;
	double room = sqrt( fmax( 0.0, rlim * rlim - off2 + along * along ) ) - along;
	if ( l->ik.mode == GC_PLANE_RADIAL && l->ik.coxa_max < 3.0 )
	{
		double rx = l->ik.rest_dir_local.x, rz = l->ik.rest_dir_local.z;
		double lim = fmax( 0.2, l->ik.coxa_max - 0.10 );
		double cl = cos( lim ), sl = sin( lim );
		static const double sgns[2] = { -1.0, 1.0 };
		for ( int si = 0; si < 2; ++si )
		{
			double sgn = sgns[si];
			double ux = rx * cl + rz * ( sgn * sl );
			double uz = -rx * ( sgn * sl ) + rz * cl;
			double det = dx * uz - dz * ux;
			if ( fabs( det ) > 1e-5 )
			{
				double s = ( ux * oz - uz * ox ) / det;
				double t = ( dx * oz - dz * ox ) / det;
				if ( s > 0.0 && t > 0.0 && s < room )
					room = s;
			}
		}
	}
	if ( fabs( dz ) > 0.1 )
	{
		double adz = fabs( dz );
		for ( int i = 0; i < m->n; ++i )
		{
			const GcLeg* nb = &m->legs[i];
			if ( nb->side == l->side && nb != l )
			{
				double gap = nb->rest_xz[1] - l->rest_xz[1];
				if ( gap * dz > 0.05 )
				{
					double safe_dz = fmax( 0.0, 0.5 * fabs( gap ) - 0.35 );
					double r_c = safe_dz / adz;
					if ( r_c < room )
						room = r_c;
				}
			}
		}
	}
	return fmax( 0.0, room );
}

static GcV3 predict_foot( const GcMech* m, const GcLeg* l, double t_ahead, double t_land, const PredCtx* ctx )
{
	GcFrame fr;
	double ride;
	int dynamic;
	GcV3 travel, lead, direction;
	int has_dir;
	double lcap;

	if ( ctx )
	{
		fr = ctx->fr;
		ride = ctx->ride;
		dynamic = ctx->dynamic;
		travel = ctx->travel;
		lead = ctx->lead;
		has_dir = ctx->has_dir;
		direction = ctx->direction;
		lcap = ctx->lead_cap;
	}
	else
	{
		fr = gc_mech_support_frame( m );
		ride = gc_mech_ride_eff( m );
		dynamic = ( t_land > 0.0 && ( gc_galloping_fast( m, 0 ) || ( ( m->drifting || m->drift_amt > 0.05 ) && m->speed > 4.0 ) ) ) ? 1 : 0;
		lcap = lead_cap( m );
		has_dir = 0;
		direction = gc_v3_zero();
		if ( dynamic )
		{
			travel = gc_planar_travel( m, t_land );
			lead = gc_v3_sub( gc_planar_travel( m, t_ahead ), travel );
			if ( gc_v3_len_sq( lead ) > 1e-8 )
			{
				GcV3 ln = gc_v3_norm( lead, gc_v3_zero() );
				GcV3 lead_dir = ( fabs( m->yaw_rate * t_land ) > 1e-3 )
									? gc_rotate_axis( ln, fr.up, m->yaw_rate * t_land )
									: ln;
				has_dir = 1;
				direction = gc_frame_to_local( &fr, lead_dir );
			}
		}
		else
		{
			travel = gc_v3_zero();
			lead = gc_v3_mul( m->vt, t_ahead );
		}
	}

	double widen = 1.0;
	if ( l->ik.mode == GC_PLANE_RADIAL && fabs( m->yaw_rate ) > 0.5 && m->speed > 6.0 )
		widen = 1.0 + gc_clamp( fabs( m->yaw_rate ) * m->speed / 280.0, 0.0, 0.22 );
	GcV3 rl = gc_frame_to_world( &fr, gc_leg_rest_local( l, ride, widen ) );
	if ( m->state == GC_STATE_MANTLE && m->mantle.active )
	{
		if ( l->force || ( l->is_front && m->mantle.t < 0.5 ) )
			return gc_v3_add( gc_v3_add( m->mantle.top, gc_v3_mul( m->mantle.fwd, 0.4 ) ),
							  gc_v3_mul( fr.right, l->rest_xz[0] ) );
		GcV3 base = gc_mantle_pos( m, m->mantle.t + t_land / m->mantle.dur );
		return gc_v3_add( base, rl );
	}
	if ( m->has_balance_out && gc_v3_dot( rl, m->balance_out ) > 0.0 )
		rl = gc_v3_add( rl, gc_v3_mul( m->balance_out, fmin( 1.5 * m->balance_excess + 0.2, 0.4 * l->reach ) ) );

	double cap;
	if ( dynamic )
	{
		if ( has_dir )
		{
			double room = leg_fwd_room( m, l, direction, ride );
			double reserve = fmin( 0.45 * GC_LANDING_RAMP_REACH * l->reach, 0.25 * m->speed * m->cycle * ( 1.0 - m->duty ) );
			cap = fmin( 0.75 * lcap, fmin( 0.28 * l->reach, fmax( 0.0, room - reserve ) ) );
		}
		else
		{
			cap = 0.0;
		}
	}
	else
	{
		cap = lcap;
	}
	if ( m->has_balance_out )
		lead = gc_v3_sub( lead, gc_v3_mul( m->balance_out, fmax( 0.0, gc_v3_dot( lead, m->balance_out ) ) ) );
	if ( gc_v3_len_sq( lead ) > cap * cap )
		lead = gc_v3_mul( gc_v3_norm( lead, gc_v3_zero() ), cap );
	if ( fabs( m->yaw_rate ) > 1e-3 )
	{
		if ( dynamic )
		{
			double stance_rot = gc_clamp( m->yaw_rate * ( t_ahead - t_land ), -0.6, 0.6 );
			rl = gc_rotate_axis( rl, fr.up, -m->yaw_rate * t_land - stance_rot );
		}
		else
		{
			double yaw_t = ( t_land > 0.0 ) ? ( t_land + 0.5 * fmax( 0.0, t_ahead - t_land ) ) : ( t_ahead * 0.5 );
			rl = gc_rotate_axis( rl, fr.up, -gc_clamp( m->yaw_rate * yaw_t, -0.75, 0.75 ) );
		}
	}
	return gc_v3_add( gc_v3_add( gc_v3_add( m->pos, travel ), rl ), lead );
}

static GcV3 swing_bow( const GcMech* m, const GcLeg* l, GcV3 start, GcV3 target, double dur )
{
	if ( dur < 0.14 )
		return gc_v3_zero();
	GcV3 up_p = gc_mech_probe_up( m );
	GcV3 bow = gc_v3_zero();
	double turn_arc = gc_smoothstep( ( fabs( m->yaw_rate ) * dur - 0.12 ) / 0.25 );
	if ( m->d->plane == GC_PLANE_RADIAL && turn_arc < 0.999 )
	{
		GcV3 hip_mid;
		GcFrame frame_mid;
		gc_landing_pose( m, l, 0.5 * dur, &hip_mid, &frame_mid );
		GcV3 mid = gc_v3_mul( gc_v3_add( start, target ), 0.5 );
		GcV3 to_mid = gc_project_plane( gc_v3_sub( mid, hip_mid ), up_p );
		double rest_r = hypot( l->rest_xz[0] - l->mount_local.x, l->rest_xz[1] - l->mount_local.z );
		GcV3 out = gc_v3_norm( gc_project_plane( gc_frame_to_world( &frame_mid, l->ik.rest_dir_local ), up_p ), gc_v3_zero() );
		double r_mid = gc_v3_len( to_mid );
		double out_mid = gc_v3_dot( to_mid, out );
		if ( r_mid < 0.70 * rest_r || out_mid < 0.20 * rest_r )
		{
			double clearance = fmin( r_mid, out_mid );
			bow = gc_v3_add( bow, gc_v3_mul( out, ( 0.75 * rest_r - clearance ) * ( 1.0 - turn_arc ) ) );
		}
	}
	if ( m->flight_t > 0.0 && m->gait.leap >= 0.4 && m->gait.stride_pitch > 0.2 )
	{
		double scale = fmin( 1.0, m->gait.leap * m->gait.stride_pitch );
		double zmax = 0.0;
		for ( int i = 0; i < m->n; ++i )
			if ( fabs( m->legs[i].mount_local.z ) > zmax ) zmax = fabs( m->legs[i].mount_local.z );
		if ( zmax == 0.0 ) zmax = 1.0;
		double dirz = gc_clamp( l->mount_local.z / zmax, -1.0, 1.0 );
		GcV3 extra = gc_v3_add( gc_v3_mul( gc_mech_heading_fwd( m ), dirz * 0.14 * l->reach ),
								gc_v3_mul( up_p, 0.12 * m->ride ) );
		bow = gc_v3_add( bow, gc_v3_mul( extra, scale ) );
	}
	double max_bow = fmin( 0.90 * l->reach, 2.5 * dur * l->reach );
	double bl = gc_v3_len( bow );
	if ( bl > max_bow )
		bow = gc_v3_mul( bow, max_bow / bl );
	return bow;
}

static GcV3 eval_swing_arc( const GcLeg* l, GcV3 sh, GcV3 th, double s_loc )
{
	if ( l->ik.mode == GC_PLANE_RADIAL )
	{
		GcV3 rd = l->ik.rest_dir_local;
		double px = -rd.z, pz = rd.x;
		double s_out = sh.x * rd.x + sh.z * rd.z, s_side = sh.x * px + sh.z * pz;
		double t_out = th.x * rd.x + th.z * rd.z, t_side = th.x * px + th.z * pz;
		double r0 = hypot( s_out, s_side ), r1 = hypot( t_out, t_side );
		double a0 = atan2( s_side, s_out ), a1 = atan2( t_side, t_out );
		double rad = r0 + ( r1 - r0 ) * s_loc;
		double ang = a0 + ( a1 - a0 ) * s_loc;
		double ca = cos( ang ), sa = sin( ang );
		return gc_v3( ( rd.x * ca + px * sa ) * rad,
					  sh.y + ( th.y - sh.y ) * s_loc,
					  ( rd.z * ca + pz * sa ) * rad );
	}
	return gc_v3_lerp( sh, th, s_loc );
}

// Record or rebase the hip-local polar/linear swing arc used while turning so
// an in-flight foot tracks around the body smoothly without chord cutting.
void gc_set_swing_arc( GcMech* m, GcLeg* l, GcV3 target, double rem, int reset_start )
{
	GcSwing* sw = &l->swing;
	if ( !sw->active )
		return;
	double turn_ang = fabs( m->yaw_rate ) * fmax( rem, sw->dur );
	double w = gc_smoothstep( ( turn_ang - 0.12 ) / 0.25 );
	sw->arc_w = reset_start ? w : fmax( sw->arc_w, w );
	if ( sw->arc_w > 1e-4 )
	{
		GcV3 hip_land;
		GcFrame frame_land;
		gc_landing_pose( m, l, fmax( rem, 0.0 ), &hip_land, &frame_land );
		GcV3 new_th = gc_frame_to_local( &frame_land, gc_v3_sub( target, hip_land ) );
		if ( reset_start || !sw->has_start_hip_l || !sw->has_target_hip_l )
		{
			sw->start_hip_l = gc_frame_to_local( &m->frame, gc_v3_sub( l->foot, l->mount_world ) );
			sw->has_start_hip_l = 1;
			sw->arc_t0 = sw->t;
			sw->target_hip_l = new_th;
			sw->has_target_hip_l = 1;
		}
		else
		{
			double span = fmax( 1e-4, 1.0 - sw->arc_t0 );
			double s_cur = gc_smoothstep( gc_clamp( ( sw->t - sw->arc_t0 ) / span, 0.0, 1.0 ) );
			if ( s_cur <= 1e-4 )
			{
				sw->target_hip_l = new_th;
			}
			else if ( s_cur < 0.88 )
			{
				// Mid-swing retarget: solve for a virtual start point sh_new that passes
				// through the current hip-local foot position at s_cur while ending at new_th.
				double inv = 1.0 / ( 1.0 - s_cur );
				GcV3 sh = sw->start_hip_l, old_th = sw->target_hip_l;
				if ( l->ik.mode == GC_PLANE_RADIAL )
				{
					GcV3 rd = l->ik.rest_dir_local;
					double px = -rd.z, pz = rd.x;
					GcV3 cur = eval_swing_arc( l, sh, old_th, s_cur );
					double c_out = cur.x * rd.x + cur.z * rd.z, c_side = cur.x * px + cur.z * pz;
					double n_out = new_th.x * rd.x + new_th.z * rd.z, n_side = new_th.x * px + new_th.z * pz;
					double r_cur = hypot( c_out, c_side ), r1 = hypot( n_out, n_side );
					double a_cur = atan2( c_side, c_out ), a1 = atan2( n_side, n_out );
					double r0 = gc_clamp( ( r_cur - s_cur * r1 ) * inv, 0.25 * l->reach, 1.15 * l->reach );
					double a0 = gc_clamp( ( a_cur - s_cur * a1 ) * inv, -2.8, 2.8 );
					double y0 = ( cur.y - s_cur * new_th.y ) * inv;
					double ca = cos( a0 ), sa = sin( a0 );
					sw->start_hip_l = gc_v3( ( rd.x * ca + px * sa ) * r0, y0, ( rd.z * ca + pz * sa ) * r0 );
				}
				else
				{
					GcV3 cur = gc_v3_lerp( sh, old_th, s_cur );
					sw->start_hip_l = gc_v3_mul( gc_v3_sub( cur, gc_v3_mul( new_th, s_cur ) ), inv );
				}
				sw->target_hip_l = new_th;
			}
		}
	}
	else if ( reset_start )
	{
		sw->has_start_hip_l = 0;
		sw->has_target_hip_l = 0;
		sw->arc_t0 = 0.0;
	}
}

// Launch a swing toward a planned foothold (or an elevated search point if no
// ground hit was found), shaping bow clearance and slide-matched velocities.
void gc_lift( GcMech* m, GcLeg* l, double dur, int has_fh, GcV3 fh_p, GcV3 fh_n,
			  const GcV3* opt_start_vel, const GcV3* opt_end_vel )
{
	GcV3 up_p = gc_mech_probe_up( m );
	GcV3 target, normal;
	int has_ground;
	if ( has_fh )
	{
		target = fh_p;
		normal = fh_n;
		has_ground = 1;
	}
	else
	{
		target = gc_v3_add( l->ideal_pred, gc_v3_mul( up_p, m->ride * 0.2 ) );
		normal = up_p;
		has_ground = 0;
	}
	double lift = m->d->step_height * m->gait.lift * ( m->crouch > 0.5 ? 0.65 : 1.0 );
	if ( m->state == GC_STATE_MANTLE )
		lift *= 1.6;
	GcV3 sv = opt_start_vel ? *opt_start_vel : gc_v3_zero();
	GcV3 ev = opt_end_vel ? *opt_end_vel : gc_v3_zero();
	if ( m->drifting )
	{
		if ( !opt_start_vel )
			sv = gc_v3_mul( gc_project_plane( m->vt, l->foot_normal ), 0.95 );
		if ( !opt_end_vel )
			ev = gc_v3_mul( gc_project_plane( m->vt, normal ), 0.95 );
	}
	GcV3 bow = swing_bow( m, l, l->foot, target, dur );
	gc_leg_begin_swing( l, target, normal, dur, lift, up_p, has_ground, &m->wq, gc_clearance( m ),
						sv, ev, bow, m->has_ceil_y, m->ceil_y );
	gc_set_swing_arc( m, l, target, dur, 1 );
	l->swing.has_ideal = 1;
	l->swing.ideal = l->ideal_pred;
	l->cooldown = 0.05;
	l->force = 0;
	l->pending = 0;
}

static void plan_step( GcMech* m, GcLeg* l, double dur_in, double* out_dur, int* out_has_fh, GcV3* out_p, GcV3* out_n )
{
	double dur = dur_in;
	if ( m->gait.phase_lock && m->state == GC_STATE_GROUND && m->cycle > 0.0 && !m->braking )
	{
		double p = gc_mod1( m->phase + l->offset );
		double to_stance = gc_mod1( ( 1.0 - m->duty ) - p );
		dur = gc_clamp( to_stance * m->cycle, 0.6 * dur, 1.4 * dur );
	}
	dur = fmax( 0.05, dur );
	if ( m->t_ahead > 0.0 )
		l->ideal_pred = predict_foot( m, l, dur + 0.5 * m->duty * m->cycle, dur, NULL );
	*out_dur = dur;
	*out_has_fh = gc_find_foothold( m, l, l->ideal_pred, dur, out_p, out_n );
}

// Check whether lifting `l` leaves enough valid support (pair sync, minimum
// planted count, adjacent-lock, and static COM polygon margin) for the gait.
int gc_can_lift( GcMech* m, GcLeg* l, int emergency, int check_static )
{
	const GcGait* g = &m->gait;
	if ( l->stance_t == 0.0 )
		return 0;
	const GcLeg* others[GC_MAX_LEGS];
	int n_after = 0;
	for ( int i = 0; i < m->n; ++i )
	{
		const GcLeg* x = &m->legs[i];
		if ( x->planted && x != l && x->stretch <= 1.04 )
			others[n_after++] = x;
	}
	if ( m->state == GC_STATE_GROUND && !m->biped )
	{
		double cur_str = gc_v3_dist( l->mount_world, l->foot ) / l->reach;
		int over = ( cur_str > 0.99 ||
					 ( ( cur_str > 0.86 || l->ik.coxa_ratio > 0.72 || l->near_limit || m->speed > 18.0 ) &&
					   !gc_legik_feasible( &l->ik, l->mount_world, &m->frame, l->foot, 0.0 ) ) ) ? 1 : 0;
		// Require another planted pair unless this leg is already forced past reach/IK
		// or the whole trailing wave is being pulled forward together.
		if ( !over && g->pair_sync && !m->trailing_support )
		{
			int any_diff_pair = 0;
			for ( int i = 0; i < n_after; ++i )
				if ( others[i]->pair != l->pair ) { any_diff_pair = 1; break; }
			if ( !any_diff_pair )
				return 0;
		}
		if ( n_after == 0 && !over && !gc_may_flight( m ) )
		{
			for ( int i = 0; i < m->n; ++i )
			{
				if ( m->legs[i].swing.active && m->legs[i].swing.has_ground )
					return 0;
			}
		}
	}
	if ( emergency && ( l->stretch > 1.04 || l->limit_t > 0.1 ) )
		return 1;
	if ( m->biped )
	{
		int need = ( g->min_support == 0 && m->speed > 1.0 ) ? 0 : 1;
		if ( emergency && m->speed > 1.0 )
			need = 0;
		return n_after >= need ? 1 : 0;
	}
	int need = g->min_support;
	if ( emergency )
		need = need < ( m->n / 2 > 1 ? m->n / 2 : 1 ) ? need : ( m->n / 2 > 1 ? m->n / 2 : 1 );
	if ( m->state == GC_STATE_MANTLE )
	{
		int cap_m = ( m->n / 2 - 1 ) > 1 ? ( m->n / 2 - 1 ) : 1;
		if ( cap_m < need ) need = cap_m;
	}
	if ( m->on_wall )
	{
		int nw = ( m->n / 2 > 3 ) ? ( m->n / 2 ) : 3;
		if ( nw > need ) need = nw;
	}
	if ( m->state == GC_STATE_GROUND && !gc_may_flight( m ) && !emergency )
	{
		if ( 1 > need ) need = 1;
	}
	if ( n_after < ( need < ( m->n - 1 ) ? need : ( m->n - 1 ) ) )
		return 0;
	if ( !emergency && g->adjacent_lock )
	{
		int pairs[3][2] = { { l->pair - 1, l->idx % 2 }, { l->pair + 1, l->idx % 2 }, { l->pair, 1 - l->idx % 2 } };
		for ( int pi = 0; pi < 3; ++pi )
		{
			int pair = pairs[pi][0], side = pairs[pi][1];
			int j = pair * 2 + side;
			if ( 0 <= pair && 0 <= j && j < m->n )
			{
				if ( !m->legs[j].planted )
					return 0;
			}
		}
	}
	if ( check_static && g->static_stable && !emergency && m->state == GC_STATE_GROUND && !m->on_wall )
	{
		GcV2 pts2[GC_MAX_HULL_PTS], hull[GC_MAX_HULL_PTS];
		for ( int i = 0; i < n_after; ++i )
		{
			pts2[i].x = others[i]->foot.x;
			pts2[i].z = others[i]->foot.z;
		}
		int hn = gc_convex_hull_2d( pts2, n_after, hull );
		GcV2 com = { m->pos.x, m->pos.z };
		if ( gc_support_margin( hull, hn, com ) < -0.04 * m->size )
			return 0;
	}
	return 1;
}

// Commit a touch-down foot to stance: re-solve IK to verify plantability,
// arm lift cooldown, cache landing lead, and kick chassis weight for feel.
void gc_on_plant( GcMech* m, GcLeg* l )
{
	gc_solve_leg( m, l, 1.0 / 60.0 );
	if ( !l->planted )
		return;
	l->pending = 0;
	l->stance_t = 0.0;
	if ( gc_galloping_fast( m, 0 ) )
		l->cooldown = fmin( 0.32 * m->duty * m->cycle, 0.38 * l->reach / m->speed );
	else
		l->cooldown = fmax( l->cooldown, 0.16 * sqrt( m->size ) );
	GcV3 up = gc_mech_probe_up( m );
	GcFrame sf = gc_mech_support_frame( m );
	l->has_land_lead = 1;
	l->land_lead = gc_frame_to_local( &sf, gc_project_plane( gc_v3_sub( l->foot, l->ideal_now ), up ) );
	l->has_last_plant = 1;
	l->last_plant_p = l->foot;
	l->last_plant_n = l->foot_normal;
	const GcMechDef* d = m->d;
	double strength = d->mass * ( 0.5 + m->speed * 0.12 );
	gc_mech_emit( m, GC_EV_STEP, l->foot, l->foot_normal, strength, gc_v3_zero() );
	// Add a subtle vertical settle and roll/pitch kick on heavy rigs so steps
	// read as weighted; suppress angular kicks while aiming so sightlines hold.
	if ( d->mass >= 2.0 && m->state == GC_STATE_GROUND )
	{
		m->vel = gc_v3_sub( m->vel, gc_v3_mul( m->frame.up, 0.05 * d->mass ) );
		if ( fabs( m->inp.aim ) < 0.05 )
		{
			GcV3 lever = gc_project_plane( gc_v3_sub( l->foot, m->pos ), m->frame.up );
			double ll = gc_v3_len( lever );
			if ( ll > 0.2 )
			{
				GcV3 axis = gc_v3_cross( m->frame.up, gc_v3_mul( lever, 1.0 / ll ) );
				double kick = fmin( 0.14, 0.022 * d->mass * gc_clamp( 1.0 - m->speed / fmax( 1.0, d->speed_walk ), 0.25, 1.0 ) );
				m->ang_vel = gc_v3_add( m->ang_vel, gc_v3_mul( axis, kick ) );
			}
		}
	}
}

// Initiate a ballistic bounding flight phase when all legs leave the ground,
// choosing initial vertical velocity to land the earliest swing foot on grade.
int gc_try_takeoff( GcMech* m, GcV3 up, GcV3 vt_new, double* io_v_up, int pushed )
{
	(void)up;
	if ( m->flight_t > 0.0 || !gc_may_flight( m ) )
		return 0;
	int paired = m->gait.pair_sync;
	int reverse = gc_v3_dot( vt_new, gc_mech_heading_fwd( m ) ) < 0.0;
	const GcLeg* lead = reverse ? &m->legs[m->n - 1] : &m->legs[0];
	const GcLeg* tail = reverse ? &m->legs[0] : &m->legs[m->n - 1];
	GcLeg* swings[GC_MAX_LEGS];
	int n_sw = 0;
	for ( int i = 0; i < m->n; ++i )
	{
		if ( m->legs[i].swing.active )
		{
			if ( !m->legs[i].swing.has_ground )
				return 0;
			swings[n_sw++] = &m->legs[i];
		}
	}
	if ( n_sw == 0 )
		return 0;
	int recovery = 0;
	if ( paired && !pushed )
	{
		int found = 0;
		for ( int i = 0; i < n_sw; ++i )
		{
			if ( swings[i]->pair == tail->pair && swings[i]->swing.t < 0.30 )
			{
				found = 1;
				break;
			}
		}
		if ( !found )
			return 0;
		recovery = 1;
	}
	double b = m->d->bound * m->gait.leap * ( recovery ? 0.55 : 1.0 );
	double vl = gc_v_leg( m );
	double k = gc_clamp( ( m->speed - 0.3 * vl ) / ( 0.7 * vl ), 0.0, 1.0 );
	double h = b * k;
	double t_land = 1e18;
	const GcLeg* nxt = NULL;
	for ( int i = 0; i < n_sw; ++i )
	{
		double rem_t = ( 1.0 - swings[i]->swing.t ) * swings[i]->swing.dur;
		if ( rem_t < t_land )
		{
			t_land = rem_t;
			nxt = swings[i];
		}
	}
	double t_fl = fmax( t_land, 2.0 * sqrt( 2.0 * h / GC_G ) );
	double extra = t_fl - t_land;
	for ( int i = 0; i < n_sw; ++i )
	{
		GcLeg* l = swings[i];
		double rem = 1.0 - l->swing.t;
		if ( rem > 1e-3 )
		{
			if ( paired )
			{
				double delay = fabs( lead->offset - l->offset ) * m->cycle;
				l->swing.dur = ( t_fl + delay ) / rem;
			}
			else if ( m->gait.group_count > 0 && nxt )
			{
				double d1 = gc_mod1( l->offset - nxt->offset );
				double d2 = gc_mod1( nxt->offset - l->offset );
				double d_off = d1 < d2 ? d1 : d2;
				if ( d_off < 0.08 )
					l->swing.dur = t_fl / rem;
				else
				{
					double min_rem = t_fl + 0.04;
					if ( rem * l->swing.dur < min_rem )
						l->swing.dur = min_rem / rem;
				}
			}
			else
			{
				l->swing.dur += extra / rem;
			}
			l->swing.retarget_cd = 0.0;
			l->swing.force_retarget = 1;
		}
	}
	if ( paired )
	{
		double to_stance = gc_mod1( 1.0 - m->duty - lead->offset - m->phase );
		m->flight_hold = fmax( 0.0, t_fl - to_stance * m->cycle );
	}
	else if ( m->gait.group_count > 0 && nxt )
	{
		double to_stance = gc_mod1( 1.0 - m->duty - nxt->offset - m->phase );
		m->flight_hold = fmax( 0.0, t_fl - to_stance * m->cycle );
	}
	else
	{
		m->flight_hold = extra;
	}
	m->flight_t = m->flight_dur = t_fl;
	m->flight_y0 = m->pos.y;
	m->flight_h = GC_G * t_fl * t_fl / 8.0;
	m->flight_count += 1;
	GcV3 lp = gc_v3_add( m->pos, gc_planar_travel( m, t_fl ) );
	GcHit lh;
	if ( gc_world_raycast( &m->wq, gc_v3( lp.x, m->pos.y + 4.0, lp.z ), gc_v3_mul( GC_WORLD_UP, -1.0 ), m->ride + 8.0, &lh ) &&
		 lh.normal.y > 0.64 )
	{
		double dy = gc_clamp( ( lh.point.y + gc_mech_ride_eff( m ) ) - m->pos.y, -2.5, 2.5 );
		*io_v_up = gc_clamp( dy / t_fl + 0.5 * GC_G * t_fl, 0.25 * GC_G * t_fl, 0.85 * GC_G * t_fl );
	}
	else
	{
		*io_v_up = 0.5 * GC_G * t_fl;
	}
	return 1;
}

// Advance active bounding flight under gravity and trigger takeoff when a
// fast gallop or pronk clears its last planted trailing support.
int gc_update_flight( GcMech* m, double dt, int npl, GcV3 vt_new, GcV3 up, double* io_v_up )
{
	int took_off = 0;
	int reverse = gc_v3_dot( vt_new, gc_mech_heading_fwd( m ) ) < 0.0;
	const GcLeg* tail = reverse ? &m->legs[0] : &m->legs[m->n - 1];
	int pushed = m->trailing_support;
	int ts = 0;
	for ( int i = 0; i < m->n; ++i )
	{
		if ( m->legs[i].planted && m->legs[i].pair == tail->pair )
		{
			ts = 1;
			break;
		}
	}
	m->trailing_support = ts;
	if ( m->flight_t <= 0.0 && npl == 0 && m->prev_npl > 0 )
		took_off = gc_try_takeoff( m, up, vt_new, io_v_up, pushed );
	m->prev_npl = npl;
	if ( m->flight_t <= 0.0 )
		return 0;
	if ( npl > 0 && !took_off )
	{
		m->flight_t = 0.0;
		m->absorb = fmax( m->absorb, gc_clamp( -( *io_v_up ) / 36.0, 0.0, 0.16 ) );
		return 0;
	}
	m->flight_t -= dt;
	*io_v_up -= GC_G * dt;
	m->vel = gc_v3_add( gc_v3_add( vt_new, m->slide_vel ), gc_v3_mul( up, *io_v_up ) );
	m->unsupported_t = 0.0;
	return 1;
}

// Extend slightly faster legs in a swing group so the whole group touches
// down in unison instead of drifting out of phase across uneven terrain.
static double align_swing_group( GcLeg** sw, int n_sw )
{
	double rems[GC_MAX_LEGS];
	double t_land = -1e18;
	for ( int i = 0; i < n_sw; ++i )
	{
		rems[i] = ( 1.0 - sw[i]->swing.t ) * sw[i]->swing.dur;
		if ( rems[i] > t_land )
			t_land = rems[i];
	}
	for ( int i = 0; i < n_sw; ++i )
	{
		double extra = t_land - rems[i];
		double rem = 1.0 - sw[i]->swing.t;
		if ( 0.01 < extra && extra <= 0.15 && rem > 1e-3 )
		{
			sw[i]->swing.dur = ( rem * sw[i]->swing.dur + extra ) / rem;
			sw[i]->swing.force_retarget = 1;
		}
	}
	return t_land;
}

static int group_may_lift( const GcMech* m, const int* grp, int g_sz )
{
	int others = 0;
	for ( int i = 0; i < m->n; ++i )
	{
		if ( !m->legs[i].planted )
			continue;
		int in_g = 0;
		for ( int gi = 0; gi < g_sz; ++gi )
			if ( grp[gi] == m->legs[i].idx ) { in_g = 1; break; }
		if ( !in_g )
			others++;
	}
	if ( m->gait.pair_sync && !m->trailing_support && others == 0 )
		return 0;
	if ( m->duty < 0.48 && gc_may_flight( m ) )
		return 1;
	int opp = m->n - g_sz;
	return ( others >= ( g_sz < opp ? g_sz : opp ) ) ? 1 : 0;
}

static void presync_groups( GcMech* m, const GcGait* dst_gait )
{
	for ( int gi = 0; gi < dst_gait->group_count; ++gi )
	{
		GcLeg* sw[GC_MAX_LEGS];
		int n_sw = 0;
		for ( int k = 0; k < dst_gait->group_sizes[gi]; ++k )
		{
			int j = dst_gait->groups[gi][k];
			if ( j < m->n && m->legs[j].swing.active )
				sw[n_sw++] = &m->legs[j];
		}
		if ( n_sw >= 2 )
			align_swing_group( sw, n_sw );
	}
}

static void sync_groups( GcMech* m, const GcGait* gait, double sw_dur )
{
	for ( int gi = 0; gi < gait->group_count; ++gi )
	{
		const int* grp = gait->groups[gi];
		int g_sz = gait->group_sizes[gi];
		GcLeg* gl[GC_MAX_LEGS];
		int n_gl = 0;
		GcLeg* sw[GC_MAX_LEGS];
		int n_sw = 0;
		for ( int k = 0; k < g_sz; ++k )
		{
			int j = grp[k];
			if ( j < m->n )
			{
				gl[n_gl++] = &m->legs[j];
				if ( m->legs[j].swing.active )
					sw[n_sw++] = &m->legs[j];
			}
		}
		if ( n_sw == 0 || ( n_sw == n_gl && n_gl < 2 ) )
			continue;
		double t_land = align_swing_group( sw, n_sw );
		if ( n_sw < n_gl && t_land > fmax( 0.08, 0.5 * sw_dur ) && group_may_lift( m, grp, g_sz ) )
		{
			for ( int k = 0; k < n_gl; ++k )
			{
				GcLeg* x = gl[k];
				if ( x->planted && x->cooldown <= 0.0 && gc_can_lift( m, x, 1, 1 ) )
				{
					double dur;
					int has_fh;
					GcV3 fhp, fhn;
					plan_step( m, x, t_land, &dur, &has_fh, &fhp, &fhn );
					gc_lift( m, x, dur, has_fh, fhp, fhn, NULL, NULL );
				}
			}
		}
	}
}

static void update_leg_targets( GcMech* m, double dt, const GcFrame* fr, const GcFrame* sf, GcV3 vis_pos,
								GcV3 up_p, double ride, double sw_dur, int moving, int driven )
{
	PredCtx ctx;
	int use_ctx = 0;
	if ( moving && !( m->state == GC_STATE_MANTLE && m->mantle.active ) )
	{
		use_ctx = 1;
		ctx.fr = *sf;
		ctx.ride = ride;
		ctx.dynamic = ( sw_dur > 0.0 && ( gc_galloping_fast( m, 0 ) || ( ( m->drifting || m->drift_amt > 0.05 ) && m->speed > 4.0 ) ) ) ? 1 : 0;
		ctx.lead_cap = lead_cap( m );
		ctx.has_dir = 0;
		ctx.direction = gc_v3_zero();
		if ( ctx.dynamic )
		{
			ctx.travel = gc_planar_travel( m, sw_dur );
			ctx.lead = gc_v3_sub( gc_planar_travel( m, m->t_ahead ), ctx.travel );
			if ( gc_v3_len_sq( ctx.lead ) > 1e-8 )
			{
				GcV3 ln = gc_v3_norm( ctx.lead, gc_v3_zero() );
				GcV3 lead_dir = ( fabs( m->yaw_rate * sw_dur ) > 1e-3 ) ? gc_rotate_axis( ln, sf->up, m->yaw_rate * sw_dur ) : ln;
				ctx.has_dir = 1;
				ctx.direction = gc_frame_to_local( sf, lead_dir );
			}
		}
		else
		{
			ctx.travel = gc_v3_zero();
			ctx.lead = gc_v3_mul( m->vt, m->t_ahead );
		}
	}

	double rest_scale = 1.0 + 0.2 * m->brace + 0.22 * m->drift_amt;
	int flat_up = ( up_p.x == 0.0 && up_p.y == 1.0 && up_p.z == 0.0 );
	int yaw_active = fabs( m->yaw_rate ) > 0.25;

	for ( int i = 0; i < m->n; ++i )
	{
		GcLeg* l = &m->legs[i];
		l->mount_world = gc_v3_add( vis_pos, gc_frame_to_world( fr, l->mount_local ) );
		l->ideal_now = gc_v3_add( m->pos, gc_frame_to_world( sf, gc_leg_rest_local( l, ride, rest_scale ) ) );
		l->ideal_pred = moving ? predict_foot( m, l, m->t_ahead, sw_dur, use_ctx ? &ctx : NULL ) : l->ideal_now;
		if ( flat_up )
			l->displacement = hypot( l->foot.x - l->ideal_now.x, l->foot.z - l->ideal_now.z );
		else
			l->displacement = gc_v3_len( gc_project_plane( gc_v3_sub( l->foot, l->ideal_now ), up_p ) );
		l->off_path = gc_dist_to_segment( l->foot, l->ideal_now, l->ideal_pred, up_p );
		if ( l->has_land_lead )
		{
			double alt = gc_dist_to_segment( l->foot, l->ideal_now,
											 gc_v3_add( l->ideal_now, gc_frame_to_world( sf, l->land_lead ) ), up_p );
			if ( alt < l->off_path ) l->off_path = alt;
		}
		l->cooldown -= dt;
		l->stance_t = l->planted ? ( l->stance_t + dt ) : 0.0;
		l->void_mark -= dt;
		l->sliding = 0;
		if ( driven || yaw_active ||
			 ( l->has_settled_hull_pos &&
			   hypot( m->pos.x - l->settled_hull_pos.x, m->pos.z - l->settled_hull_pos.z ) > 0.25 * m->size ) )
		{
			l->has_settled_at = 0;
		}
	}

	for ( int i = 0; i < m->n; ++i )
	{
		GcLeg* l = &m->legs[i];
		if ( l->has_carry && !l->planted && !l->swing.active )
			l->foot = gc_v3_add( m->pos, gc_frame_to_world( fr, l->carry ) );
	}

	double sl = gc_v3_len( m->slide_vel );
	if ( sl > 0.05 && m->state == GC_STATE_GROUND )
	{
		for ( int i = 0; i < m->n; ++i )
		{
			GcLeg* l = &m->legs[i];
			if ( l->planted )
			{
				GcV3 old = l->foot;
				l->foot = gc_v3_add( l->foot, gc_v3_mul( gc_project_plane( m->slide_vel, l->foot_normal ), dt ) );
				l->sliding = 1;
				gc_slide_foot_clear( m, l, up_p );
				if ( sl > 0.8 )
					gc_mech_emit( m, GC_EV_SKID, old, l->foot_normal, m->d->widths[m->d->n_segs - 1] * 0.8, l->foot );
			}
		}
	}
}

static uint32_t update_swings( GcMech* m, double dt, GcV3 up_p, double t_stance, int moving )
{
	uint32_t just_planted = 0;
	for ( int i = 0; i < m->n; ++i )
	{
		GcLeg* l = &m->legs[i];
		if ( !l->swing.active )
			continue;
		GcSwing* sw = &l->swing;
		double rem = fmax( 0.0, ( 1.0 - sw->t ) * sw->dur - dt );
		if ( ( rem > 0.05 || sw->force_retarget ) && m->state != GC_STATE_CROUCHJUMP )
		{
			sw->retarget_cd -= dt;
			if ( sw->retarget_cd <= 0.0 || sw->force_retarget )
			{
				sw->retarget_cd = m->braking ? 0.03 : ( ( gc_galloping_fast( m, 0 ) || m->drifting ) ? 0.05 : 0.1 );
				GcV3 pred = moving ? predict_foot( m, l, rem + 0.5 * t_stance, rem, NULL ) : l->ideal_now;
				GcV3 ref = sw->has_ideal ? sw->ideal : sw->target;
				int forced = sw->force_retarget;
				sw->force_retarget = 0;
				double ev_l2 = gc_v3_len_sq( sw->end_velocity );
				if ( gc_v3_dist( pred, ref ) > 0.25 || forced ||
					 ( m->drifting && ev_l2 > 0.0 ) ||
					 ( !m->drifting && ev_l2 > 0.25 ) )
				{
					GcV3 fhp, fhn;
					int has_fh = gc_find_foothold( m, l, pred, rem, &fhp, &fhn );
					sw->has_ideal = 1;
					sw->ideal = pred;
					if ( has_fh )
					{
						GcV3 end_vel = m->drifting ? gc_v3_mul( gc_project_plane( m->vt, fhn ), 0.95 ) : gc_v3_zero();
						if ( forced || !sw->has_ground || gc_v3_dist( fhp, sw->target ) > 0.15 ||
							 gc_v3_dist( end_vel, sw->end_velocity ) > 0.8 )
						{
							gc_leg_retarget( l, fhp, fhn, 1, &m->wq, gc_clearance( m ),
											 end_vel, swing_bow( m, l, l->foot, fhp, rem ), m->has_ceil_y, m->ceil_y );
							gc_set_swing_arc( m, l, fhp, rem, 0 );
						}
					}
					else if ( forced )
					{
						GcV3 end_vel = m->drifting ? gc_v3_mul( gc_project_plane( m->vt, up_p ), 0.95 ) : gc_v3_zero();
						gc_leg_retarget( l, pred, up_p, 0, &m->wq, 0.0,
										 end_vel, swing_bow( m, l, l->foot, pred, rem ), m->has_ceil_y, m->ceil_y );
						gc_set_swing_arc( m, l, pred, rem, 0 );
					}
				}
			}
		}
		if ( gc_leg_update_swing( l, dt ) )
		{
			gc_on_plant( m, l );
			just_planted |= ( 1u << l->idx );
		}
		else if ( l->swing.active && l->swing.arc_w > 1e-4 && l->swing.has_start_hip_l && l->swing.has_target_hip_l )
		{
			double rem_s = ( 1.0 - sw->t ) * sw->dur;
			double fade_end = fmin( 0.08, 0.16 * sw->dur );
			double w_arc = sw->arc_w * gc_smoothstep( sw->t / 0.12 ) * gc_smoothstep( rem_s / fmax( 1e-4, fade_end ) );
			if ( w_arc > 1e-4 )
			{
				double span = fmax( 1e-4, 1.0 - sw->arc_t0 );
				double s_loc = gc_smoothstep( gc_clamp( ( sw->t - sw->arc_t0 ) / span, 0.0, 1.0 ) );
				GcV3 rel_l = eval_swing_arc( l, sw->start_hip_l, sw->target_hip_l, s_loc );
				GcV3 p_body = gc_v3_add( l->mount_world, gc_frame_to_world( &m->frame, rel_l ) );
				l->foot = gc_v3_add( l->foot, gc_v3_mul( gc_project_plane( gc_v3_sub( p_body, l->foot ), up_p ), w_arc ) );
			}
		}
	}
	return just_planted;
}

static void update_searching( GcMech* m, double dt, GcV3 up_p, double sw_dur, double ride )
{
	for ( int i = 0; i < m->n; ++i )
	{
		GcLeg* l = &m->legs[i];
		if ( l->searching )
		{
			l->search_cd -= dt;
			if ( l->search_cd <= 0.0 )
			{
				l->search_cd = 0.12;
				double dur;
				int has_fh;
				GcV3 fhp, fhn;
				plan_step( m, l, sw_dur * 0.7, &dur, &has_fh, &fhp, &fhn );
				if ( !has_fh && m->state == GC_STATE_GROUND )
				{
					GcV3 sum_p = gc_v3_zero();
					int n_pl = 0;
					for ( int k = 0; k < m->n; ++k )
					{
						if ( m->legs[k].planted )
						{
							sum_p = gc_v3_add( sum_p, m->legs[k].foot );
							n_pl++;
						}
					}
					if ( n_pl > 0 )
					{
						GcV3 safe = gc_v3_mul( sum_p, 1.0 / (double)n_pl );
						static const double alphas[2] = { 0.60, 0.90 };
						for ( int ai = 0; ai < 2; ++ai )
						{
							if ( gc_find_foothold( m, l, gc_v3_lerp( l->ideal_now, safe, alphas[ai] ), dur, &fhp, &fhn ) )
							{
								has_fh = 1;
								l->void_mark = 1.0;
								break;
							}
						}
					}
				}
				if ( !has_fh && l->has_last_plant && m->state == GC_STATE_GROUND )
				{
					GcV3 p = l->last_plant_p, n = l->last_plant_n;
					if ( gc_v3_dist( p, l->mount_world ) < 0.93 * l->reach &&
						 gc_reachable( m, l, p, NULL, NULL ) &&
						 gc_clear_spot( m, l, p, up_p, NULL ) )
					{
						has_fh = 1;
						fhp = p;
						fhn = n;
						l->void_mark = 1.0;
					}
				}
				if ( has_fh )
				{
					gc_lift( m, l, dur, has_fh, fhp, fhn, NULL, NULL );
					continue;
				}
			}
			GcV3 target = gc_hang_pose( m, l, ride );
			const GcFrame* fr = &m->frame;
			GcV3 fl = gc_frame_to_local( fr, gc_v3_sub( l->foot, m->pos ) );
			GcV3 tl = gc_frame_to_local( fr, gc_v3_sub( target, m->pos ) );
			l->foot = gc_v3_add( m->pos, gc_frame_to_world( fr, gc_v3_lerp( fl, tl, gc_damp( 5.0, dt ) ) ) );
		}
	}
}

static void step_drift_legs( GcMech* m, double dt, double sw_dur, GcV3 up_p, const GcMechDef* d )
{
	int spark_now = 0;
	m->spark_t += dt;
	if ( m->spark_t >= 1.0 / 36.0 )
	{
		m->spark_t = fmod( m->spark_t, 1.0 / 36.0 );
		spark_now = 1;
	}
	for ( int i = 0; i < m->n; ++i )
	{
		GcLeg* l = &m->legs[i];
		if ( l->planted )
		{
			GcV3 slide = gc_v3_mul( gc_project_plane( m->vt, l->foot_normal ), 0.95 );
			GcV3 old = l->foot;
			l->foot = gc_v3_add( l->foot, gc_v3_mul( slide, dt ) );
			l->sliding = 1;
			gc_slide_foot_clear( m, l, up_p );
			if ( spark_now )
				gc_mech_emit( m, GC_EV_SPARK, l->foot, l->foot_normal, m->speed, slide );
			gc_mech_emit( m, GC_EV_SKID, old, l->foot_normal, d->widths[d->n_segs - 1], l->foot );
		}
	}
	int order[GC_MAX_LEGS];
	for ( int i = 0; i < m->n; ++i ) order[i] = i;
	for ( int i = 1; i < m->n; ++i )
	{
		int key = order[i], j = i - 1;
		while ( j >= 0 && m->legs[order[j]].stretch < m->legs[key].stretch )
		{
			order[j + 1] = order[j];
			j--;
		}
		order[j + 1] = key;
	}
	for ( int oi = 0; oi < m->n; ++oi )
	{
		GcLeg* l = &m->legs[order[oi]];
		if ( l->planted && ( l->stretch > 0.96 || l->ik.err > 0.02 || ( l->cooldown <= 0.0 && ( l->stretch > 0.90 || l->near_limit ) ) ) &&
			 gc_can_lift( m, l, 1, 1 ) )
		{
			double dur;
			int has_fh;
			GcV3 fhp, fhn;
			plan_step( m, l, sw_dur * 0.8, &dur, &has_fh, &fhp, &fhn );
			gc_lift( m, l, dur, has_fh, fhp, fhn, NULL, NULL );
		}
	}
}

static void step_reactive_legs( GcMech* m, double dt, const GcFrame* fr, GcV3 up_p, const GcGait* gait,
								double swing_frac, double sw_dur, double eff_spd, int moving, int driven, const GcMechDef* d )
{
	double t_stance = m->cycle * m->duty;
	for ( int i = 0; i < m->n; ++i )
	{
		GcLeg* l = &m->legs[i];
		double p = gc_mod1( m->phase + l->offset );
		int in_w = p < swing_frac;
		if ( in_w && !l->prev_window )
			l->pending = 1;
		if ( !in_w )
			l->pending = 0;
		l->prev_window = in_w;
	}

	double zone = d->zone * ( m->state == GC_STATE_MANTLE ? 0.5 : 1.0 );
	if ( m->state == GC_STATE_GROUND && !m->drifting && fabs( m->yaw_rate ) < 0.35 && eff_spd < 0.35 * m->v_walk && driven )
	{
		double posture_s = gc_smoothstep( ( eff_spd - 0.15 ) / fmax( 0.5, 0.35 * m->v_walk - 0.15 ) );
		zone *= 0.55 + 0.45 * posture_s;
	}
	int has_vdir = ( m->speed > 0.1 );
	GcV3 vdir = has_vdir ? gc_v3_norm( m->vt, gc_v3_zero() ) : gc_v3_zero();
	int has_t_since = ( !m->drifting && m->drift_amt > 0.05 && m->slide_cd > 0.0 && m->speed > 4.0 );
	double t_since = has_t_since ? fmax( 0.0, 0.35 - m->slide_cd ) : 0.0;
	double t_exit_look = fmax( 0.036, sw_dur * 0.85 );
	int has_stale = 0;
	if ( has_t_since )
	{
		for ( int i = 0; i < m->n; ++i )
		{
			if ( m->legs[i].planted && m->legs[i].stance_t + 0.02 >= t_since )
			{
				has_stale = 1;
				break;
			}
		}
	}
	double walk_min_st = ( gait->min_support > 0 ) ? 0.155 : 0.0;

	int order[GC_MAX_LEGS];
	double key_val[GC_MAX_LEGS];
	int key_pri[GC_MAX_LEGS];
	for ( int i = 0; i < m->n; ++i )
	{
		order[i] = i;
		const GcLeg* x = &m->legs[i];
		if ( has_stale )
		{
			key_pri[i] = ( x->near_limit || x->stretch > 0.94 || x->ik.err > 0.01 ) ? 0 : 1;
			key_val[i] = -gc_v3_dist( gc_v3_add( x->mount_world, gc_v3_mul( m->vt, t_exit_look ) ), x->foot ) / x->reach;
		}
		else
		{
			key_pri[i] = 0;
			key_val[i] = -x->displacement;
		}
	}
	for ( int i = 1; i < m->n; ++i )
	{
		int oi = order[i], j = i - 1;
		while ( j >= 0 )
		{
			int oj = order[j];
			int gt = ( key_pri[oj] > key_pri[oi] ) || ( key_pri[oj] == key_pri[oi] && key_val[oj] > key_val[oi] );
			if ( !gt ) break;
			order[j + 1] = oj;
			j--;
		}
		order[j + 1] = oi;
	}

	for ( int oi = 0; oi < m->n; ++oi )
	{
		GcLeg* l = &m->legs[order[oi]];
		if ( !l->planted )
			continue;
		double t_look = ( has_t_since && l->stance_t + 0.02 >= t_since ) ? t_exit_look : 0.036;
		double disp = l->off_path;
		int l_grp_idx = -1;
		for ( int gi = 0; gi < gait->group_count && l_grp_idx < 0; ++gi )
		{
			for ( int k = 0; k < gait->group_sizes[gi]; ++k )
				if ( gait->groups[gi][k] == l->idx ) { l_grp_idx = gi; break; }
		}
		int grp_blocked = ( l_grp_idx >= 0 && !group_may_lift( m, gait->groups[l_grp_idx], gait->group_sizes[l_grp_idx] ) ) ? 1 : 0;
		double cur_str = gc_v3_dist( l->mount_world, l->foot ) / l->reach;
		int cur_infeas = ( cur_str > 0.99 ||
						   ( ( cur_str > 0.86 || l->ik.coxa_ratio > 0.72 || l->near_limit || m->speed > 18.0 ) &&
							 !gc_legik_feasible( &l->ik, l->mount_world, fr, l->foot, 0.0 ) ) ) ? 1 : 0;
		double release = 0.97;
		if ( cur_infeas )
			release = fmin( release, l->stretch - 1e-3 );
		if ( has_vdir && gc_v3_dot( gc_v3_sub( l->foot, l->mount_world ), vdir ) < 0.0 )
		{
			release -= fmin( 0.06, 0.03 * m->speed / l->reach );
			double ahead = gc_v3_dist( gc_v3_add( l->mount_world, gc_v3_mul( m->vt, t_look ) ), l->foot ) / l->reach;
			if ( ahead > 0.97 && ( !grp_blocked || l->stretch > 0.94 ) )
				release = fmin( release, l->stretch - 1e-3 );
		}
		if ( moving && ( m->speed > 4.0 || fabs( m->yaw_rate ) > 0.5 ) )
		{
			GcV3 hip_next;
			GcFrame frame_next;
			if ( fabs( m->yaw_rate ) > 0.5 )
				gc_landing_pose( m, l, t_look, &hip_next, &frame_next );
			else
			{
				hip_next = gc_v3_add( l->mount_world, gc_v3_mul( m->vt, t_look ) );
				frame_next = *fr;
			}
			int allow_pred = cur_infeas || !grp_blocked || l->stretch > 0.95 || l->ik.coxa_ratio > 0.94 || l->ik.err > 0.01;
			if ( allow_pred &&
				 ( l->stance_t >= fmax( walk_min_st, fmin( 0.08, 0.30 * t_stance ) ) ||
				   l->stretch > 0.94 || l->ik.err > 0.01 || cur_infeas ) &&
				 !gc_legik_feasible( &l->ik, hip_next, &frame_next, l->foot, 2.0 ) )
			{
				release = fmin( release, l->stretch - 1e-3 );
			}
			else if ( allow_pred && l->ik.mode == GC_PLANE_RADIAL && m->speed > 10.0 && fabs( m->yaw_rate ) > 0.4 && l->stance_t >= 0.03 )
			{
				GcV3 rel_next = gc_frame_to_local( &frame_next, gc_v3_sub( l->foot, hip_next ) );
				double rest_r = hypot( l->rest_xz[0] - l->mount_local.x, l->rest_xz[1] - l->mount_local.z );
				double r_xz = hypot( rel_next.x, rel_next.z );
				if ( r_xz < 0.48 * rest_r )
				{
					release = fmin( release, l->stretch - 1e-3 );
				}
				else if ( r_xz > 1e-5 )
				{
					GcV3 rd = l->ik.rest_dir_local;
					double inv_r = 1.0 / r_xz;
					double ux = rel_next.x * inv_r, uz = rel_next.z * inv_r;
					double next_yaw = atan2( rd.z * ux - rd.x * uz, rd.x * ux + rd.z * uz );
					double cur_yaw = l->ik.yaw_deg * GC_DEG2RAD;
					double diff_y = fabs( gc_wrap_pi( next_yaw - cur_yaw ) );
					if ( diff_y > 28.0 * GC_DEG2RAD )
						release = fmin( release, l->stretch - 1e-3 );
				}
			}
		}
		if ( moving && !grp_blocked && fabs( m->yaw_rate ) > 0.25 && l->stance_t >= fmax( 0.06, walk_min_st ) )
		{
			double side_exc = fabs( gc_frame_to_local( fr, gc_v3_sub( l->foot, m->pos ) ).x - l->rest_xz[0] );
			if ( side_exc > 0.46 * l->reach )
				release = fmin( release, l->stretch - 1e-3 );
		}
		double soon = ( l->stance_t >= fmax( 0.06, walk_min_st ) && l->stretch > 0.88 )
						  ? ( l->stretch + fmax( 0.0, l->stretch_rate ) * fmin( 0.05, sw_dur * 0.35 ) )
						  : 0.0;
		if ( l->cooldown > 0.0 && l->stretch <= release )
			continue;

		int hard_emg, zone_trig;
		if ( moving )
		{
			int near_req = grp_blocked ? ( l->stretch > 0.94 || l->ik.err > 0.01 )
									   : ( l->stretch > 0.90 || l->stretch_rate > 0.1 || l->ik.err > 0.01 ||
										   ( l->ik.coxa_ratio > 0.88 && fabs( m->yaw_rate ) > 0.25 ) );
			int near_trig = ( l->near_limit && l->cooldown <= 0.0 &&
							  ( l->stance_t >= fmax( 0.06, walk_min_st ) || l->stretch > 0.94 || l->ik.err > 0.01 ) &&
							  ( driven || m->speed > 2.0 ) && near_req );
			hard_emg = ( l->force || l->stretch > release || soon > release || near_trig ) ? 1 : 0;
			zone_trig = ( grp_blocked || l->stance_t < walk_min_st ) ? 0 : ( disp > zone * gait->reach ? 1 : 0 );
		}
		else
		{
			hard_emg = ( l->force || l->stretch > 0.995 ) ? 1 : 0;
			zone_trig = 0;
		}
		int can = hard_emg ? gc_can_lift( m, l, 1, 1 ) : ( zone_trig ? gc_can_lift( m, l, 0, 0 ) : 0 );
		if ( can )
		{
			double dur;
			int has_fh;
			GcV3 fhp, fhn;
			plan_step( m, l, sw_dur * 0.85, &dur, &has_fh, &fhp, &fhn );
			if ( !moving && !l->force && has_fh )
			{
				double cur_d = gc_v3_dist( l->foot, l->mount_world );
				double new_d = gc_v3_dist( fhp, l->mount_world );
				double new_disp = gc_v3_len( gc_project_plane( gc_v3_sub( fhp, l->ideal_now ), up_p ) );
				if ( new_d >= cur_d - 0.05 && new_disp >= l->displacement - 0.05 )
				{
					l->cooldown = fmax( l->cooldown, 0.3 );
					continue;
				}
			}
			if ( gc_futile_step( l, has_fh, fhp ) )
				continue;
			gc_lift( m, l, dur, has_fh, fhp, fhn, NULL, NULL );
			for ( int gi = 0; gi < gait->group_count; ++gi )
			{
				const int* grp = gait->groups[gi];
				int g_sz = gait->group_sizes[gi];
				int in_grp = 0;
				for ( int k = 0; k < g_sz; ++k )
					if ( grp[k] == l->idx ) { in_grp = 1; break; }
				if ( in_grp && group_may_lift( m, grp, g_sz ) )
				{
					for ( int k = 0; k < g_sz; ++k )
					{
						int j = grp[k];
						if ( j < m->n )
						{
							GcLeg* mate = &m->legs[j];
							if ( mate != l && mate->planted && gc_can_lift( m, mate, 1, 1 ) )
							{
								double mdur;
								int mfh;
								GcV3 mp, mn;
								plan_step( m, mate, sw_dur * 0.85, &mdur, &mfh, &mp, &mn );
								gc_lift( m, mate, mdur, mfh, mp, mn, NULL, NULL );
							}
						}
					}
				}
			}
		}
	}

	if ( gc_galloping_fast( m, 1 ) )
	{
		for ( int i = 0; i < m->n; ++i )
		{
			GcLeg* l = &m->legs[i];
			if ( l->planted && l->stretch > 1.0 )
			{
				GcV3 old = l->foot;
				l->foot = gc_v3_add( l->foot, gc_v3_mul( gc_project_plane( m->vt, l->foot_normal ), dt ) );
				l->sliding = 1;
				gc_slide_foot_clear( m, l, up_p );
				gc_mech_emit( m, GC_EV_SKID, old, l->foot_normal, d->widths[d->n_segs - 1] * 0.6, l->foot );
			}
		}
	}
}

static void step_rhythmic_groups( GcMech* m, const GcGait* gait, double sw_dur, double s )
{
	for ( int gi = 0; gi < gait->group_count; ++gi )
	{
		const int* grp = gait->groups[gi];
		int g_sz = gait->group_sizes[gi];
		GcLeg* gl[GC_MAX_LEGS];
		int n_gl = 0;
		for ( int k = 0; k < g_sz; ++k )
		{
			if ( grp[k] < m->n )
				gl[n_gl++] = &m->legs[grp[k]];
		}
		int any_pend = 0, all_planted = 1, any_cd = 0;
		double max_disp = -1e18;
		for ( int k = 0; k < n_gl; ++k )
		{
			if ( gl[k]->pending ) any_pend = 1;
			if ( !gl[k]->planted ) all_planted = 0;
			if ( gl[k]->cooldown > 0.0 ) any_cd = 1;
			if ( gl[k]->displacement > max_disp ) max_disp = gl[k]->displacement;
		}
		if ( !any_pend || !all_planted || any_cd || max_disp <= 0.05 * s )
			continue;
		int others = 0;
		for ( int i = 0; i < m->n; ++i )
		{
			if ( !m->legs[i].planted ) continue;
			int in_g = 0;
			for ( int k = 0; k < n_gl; ++k )
				if ( gl[k] == &m->legs[i] ) { in_g = 1; break; }
			if ( !in_g ) others++;
		}
		int need_min = gait->min_support < ( m->n - n_gl ) ? gait->min_support : ( m->n - n_gl );
		if ( others < need_min || !group_may_lift( m, grp, g_sz ) )
			continue;
		double durs[GC_MAX_LEGS];
		int fhs[GC_MAX_LEGS];
		GcV3 fhps[GC_MAX_LEGS], fhns[GC_MAX_LEGS];
		for ( int k = 0; k < n_gl; ++k )
		{
			plan_step( m, gl[k], sw_dur, &durs[k], &fhs[k], &fhps[k], &fhns[k] );
			gl[k]->pending = 0;
		}
		int all_futile = 1;
		for ( int k = 0; k < n_gl; ++k )
		{
			if ( !gc_futile_step( gl[k], fhs[k], fhps[k] ) )
				all_futile = 0;
		}
		if ( all_futile )
			continue;
		for ( int k = 0; k < n_gl; ++k )
			gc_lift( m, gl[k], durs[k], fhs[k], fhps[k], fhns[k], NULL, NULL );
	}
}

static void step_rhythmic_legs( GcMech* m, double t_stance, double sw_dur, double s )
{
	double min_st = fmax( 0.33 * t_stance, m->gait.min_support > 0 ? 0.155 : 0.0 );
	for ( int i = 0; i < m->n; ++i )
	{
		GcLeg* l = &m->legs[i];
		if ( l->pending && l->planted && l->cooldown <= 0.0 && l->stance_t >= min_st &&
			 l->displacement > 0.05 * s && gc_can_lift( m, l, 0, 1 ) )
		{
			l->pending = 0;
			double dur;
			int has_fh;
			GcV3 fhp, fhn;
			plan_step( m, l, sw_dur, &dur, &has_fh, &fhp, &fhn );
			if ( !gc_futile_step( l, has_fh, fhp ) )
				gc_lift( m, l, dur, has_fh, fhp, fhn, NULL, NULL );
		}
	}
}

static void settle_at_rest( GcMech* m, const GcFrame* sf, GcV3 up_p, double ride, double s, const GcMechDef* d )
{
	if ( m->speed > 0.25 )
		return;
	double c_dist[GC_MAX_LEGS];
	GcV3 c_rest[GC_MAX_LEGS];
	int c_idx[GC_MAX_LEGS];
	int nc = 0;
	for ( int i = 0; i < m->n; ++i )
	{
		if ( m->legs[i].planted )
		{
			GcV3 rc = gc_v3_add( m->pos, gc_frame_to_world( sf, gc_leg_rest_local( &m->legs[i], ride, 1.0 ) ) );
			c_dist[nc] = gc_v3_len( gc_project_plane( gc_v3_sub( m->legs[i].foot, rc ), up_p ) );
			c_rest[nc] = rc;
			c_idx[nc] = i;
			nc++;
		}
	}
	int order[GC_MAX_LEGS];
	for ( int i = 0; i < nc; ++i ) order[i] = i;
	for ( int i = 1; i < nc; ++i )
	{
		int key = order[i], j = i - 1;
		while ( j >= 0 && c_dist[order[j]] < c_dist[key] )
		{
			order[j + 1] = order[j];
			j--;
		}
		order[j + 1] = key;
	}
	for ( int oi = 0; oi < nc; ++oi )
	{
		int ci = order[oi];
		double cur_d = c_dist[ci];
		if ( cur_d <= 0.06 + 0.05 * s )
			break;
		GcLeg* cand = &m->legs[c_idx[ci]];
		if ( cand->cooldown > 0.0 || cand->has_settled_at )
			continue;
		cand->has_settled_at = 1;
		cand->settled_at = c_rest[ci];
		cand->has_settled_hull_pos = 1;
		cand->settled_hull_pos = m->pos;
		if ( !gc_can_lift( m, cand, 0, 1 ) )
			continue;
		GcV3 fhp, fhn;
		if ( !gc_find_foothold( m, cand, c_rest[ci], 0.0, &fhp, &fhn ) || gc_v3_dist( fhp, cand->foot ) < 0.15 )
			continue;
		double dn = gc_v3_len( gc_project_plane( gc_v3_sub( fhp, c_rest[ci] ), up_p ) );
		if ( dn > cur_d - 0.05 )
			continue;
		double settle_dur = gc_clamp( d->swing_min * 1.05 + 0.18 * cur_d, d->swing_min * 1.05, d->swing_max * 0.55 );
		gc_lift( m, cand, settle_dur, 1, fhp, fhn, NULL, NULL );
		break;
	}
}

// Advance gait phase, compute swing/stance timings, predict upcoming footholds,
// schedule leg lifts (air, drift, reactive, or rhythmic), and solve leg IK.
void gc_update_legs( GcMech* m, double dt )
{
	int prof = m->profile_enabled;
	double t0 = prof ? gc_time_sec() : 0.0;
	const GcFrame* fr = &m->frame;
	const GcMechDef* d = m->d;
	double s = m->size;
	GcV3 up_p = gc_mech_probe_up( m );
	GcV3 vis_pos = gc_v3_add( m->pos, gc_frame_to_world( fr, m->vis_offset ) );
	double ride = gc_mech_ride_eff( m );
	GcFrame sf = gc_mech_support_frame( m );

	m->vt = gc_project_plane( m->vel, up_p );
	m->speed = gc_v3_len( m->vt );
	int driven = ( fabs( m->inp.fwd ) > 1e-3 || fabs( m->inp.strafe ) > 1e-3 ||
				   fabs( m->inp.turn ) > 1e-3 || m->drifting || m->state == GC_STATE_MANTLE ) ? 1 : 0;
	int moving = ( m->speed > ( driven ? 0.12 : 0.45 ) * sqrt( s ) || fabs( m->yaw_rate ) > 0.12 ||
				   m->drifting || m->state == GC_STATE_MANTLE ) ? 1 : 0;

	const GcGait* gait = &m->gait;
	double swing_frac = 1.0 - m->duty;
	double stride = gait->stride * s * ( m->crouch > 0.5 ? 0.75 : 1.0 );
	double turn_spd = fabs( m->yaw_rate ) * m->reach_avg * 0.65;
	double eff_spd = m->drifting ? fmax( m->speed * 0.2, fabs( m->yaw_rate ) * m->reach_avg ) : fmax( m->speed, turn_spd );
	int grounded_walk = ( m->state == GC_STATE_GROUND && !gc_may_flight( m ) && !m->drifting ) ? 1 : 0;
	double contact_span = grounded_walk ? m->chord_avg : ( 1.25 * m->reach_avg );
	double t_contact = contact_span / fmax( eff_spd, 1e-3 );
	// Clamp minimum gait cycle against reachable contact time so high-speed
	// grounded rigs finish swings fast enough rather than trailing behind hips.
	double cmin = fmin( gait->cycle_min * sqrt( s ), t_contact / fmax( m->duty, 0.2 ) );
	if ( grounded_walk && eff_spd > 4.0 )
	{
		double cmin_sw = fmax( 0.095, 0.82 * d->swing_min ) / fmax( swing_frac, 0.2 );
		cmin = fmax( cmin_sw, fmin( cmin, t_contact / fmax( swing_frac, 0.2 ) ) );
	}
	double T = gc_clamp( stride / fmax( eff_spd, 1e-3 ), cmin, fmax( cmin, gait->cycle_max * sqrt( s ) ) );
	m->cycle = T;
	if ( m->flight_hold > 0.0 )
		m->flight_hold -= dt;
	else if ( moving && ( m->state == GC_STATE_GROUND || m->state == GC_STATE_MANTLE ) )
		m->phase = gc_mod1( m->phase + dt / T );

	double sw_floor = grounded_walk ? fmax( 0.095, fmin( d->swing_min, T * swing_frac ) )
									: fmax( 0.05, fmin( d->swing_min, T * swing_frac ) );
	double sw_dur = gc_clamp( T * swing_frac, sw_floor, d->swing_max );
	// Exiting a slide at speed leaves old planted feet behind the COM; trim
	// swing durations briefly until slide-era plants cycle out.
	if ( !m->drifting && m->drift_amt > 0.05 && m->slide_cd > 0.0 && grounded_walk && eff_spd > 4.0 )
	{
		double t_since = fmax( 0.0, 0.35 - m->slide_cd );
		int any_stale = 0;
		for ( int i = 0; i < m->n; ++i )
		{
			if ( m->legs[i].planted && m->legs[i].stance_t + 0.02 >= t_since )
			{
				any_stale = 1;
				break;
			}
		}
		if ( any_stale )
		{
			double rec_floor = fmax( 0.095, 0.78 * d->swing_min );
			sw_dur = fmax( rec_floor, sw_dur * ( 1.0 - 0.45 * m->drift_amt ) );
			double decay = exp( -8.0 * dt );
			for ( int i = 0; i < m->n; ++i )
			{
				if ( m->legs[i].swing.active && m->legs[i].swing.dur > sw_dur )
					m->legs[i].swing.dur = fmax( sw_dur, m->legs[i].swing.dur * decay );
			}
		}
	}
	double t_stance = T * m->duty;
	m->t_ahead = moving ? ( sw_dur + 0.5 * t_stance ) : 0.0;

	update_leg_targets( m, dt, fr, &sf, vis_pos, up_p, ride, sw_dur, moving, driven );
	if ( prof )
	{
		double t1 = gc_time_sec();
		m->prof[2] += t1 - t0;
		t0 = t1;
	}

	if ( m->state == GC_STATE_AIR || gc_mech_wire_active( m ) )
	{
		gc_legs_air( m, dt );
		if ( prof )
		{
			double t1 = gc_time_sec();
			m->prof[3] += t1 - t0;
			t0 = t1;
		}
		gc_feet_clear( m, dt );
		if ( prof )
		{
			double t1 = gc_time_sec();
			m->prof[5] += t1 - t0;
		}
		for ( int i = 0; i < m->n; ++i )
			gc_solve_leg( m, &m->legs[i], dt );
		return;
	}

	int npl_pre = 0;
	for ( int i = 0; i < m->n; ++i )
		if ( m->legs[i].planted ) npl_pre++;
	int pushed_pre = m->trailing_support;
	double ik_pre = prof ? ( m->prof[6] + m->prof[7] ) : 0.0;
	uint32_t just_planted = update_swings( m, dt, up_p, t_stance, moving );
	update_searching( m, dt, up_p, sw_dur, ride );
	if ( prof )
	{
		double t1 = gc_time_sec();
		double delta = ( t1 - t0 ) - ( ( m->prof[6] + m->prof[7] ) - ik_pre );
		if ( delta > 0.0 )
			m->prof[3] += delta;
		t0 = t1;
	}

	if ( m->drifting )
	{
		step_drift_legs( m, dt, sw_dur, up_p, d );
	}
	else if ( m->state != GC_STATE_CROUCHJUMP )
	{
		step_reactive_legs( m, dt, fr, up_p, gait, swing_frac, sw_dur, eff_spd, moving, driven, d );
		if ( moving && m->flight_t <= 0.0 )
		{
			if ( gait->group_count > 0 )
				sync_groups( m, gait, sw_dur );
			else if ( m->blend.t < 1.0 && m->blend.s >= 0.70 && m->blend.dst.group_count > 0 )
				presync_groups( m, &m->blend.dst );
		}
		int any_swing = 0;
		for ( int i = 0; i < m->n; ++i )
			if ( m->legs[i].swing.active ) { any_swing = 1; break; }
		if ( moving && gait->group_count > 0 )
			step_rhythmic_groups( m, gait, sw_dur, s );
		else if ( moving )
			step_rhythmic_legs( m, t_stance, sw_dur, s );
		else if ( !any_swing )
			settle_at_rest( m, &sf, up_p, ride, s, d );
	}
	if ( prof )
	{
		double t1 = gc_time_sec();
		m->prof[4] += t1 - t0;
		t0 = t1;
	}

	gc_feet_clear( m, dt );
	if ( prof )
	{
		double t1 = gc_time_sec();
		m->prof[5] += t1 - t0;
	}

	int npl_post = 0;
	for ( int i = 0; i < m->n; ++i )
		if ( m->legs[i].planted ) npl_post++;
	int stressed_grounded = 0;
	int may_fl = gc_may_flight( m );
	// Optional IK solver downsampling saves CPU in large crowds; bypass the
	// accumulator on touch-down or airborne/mantle transitions so joints snap
	// immediately on impact.
	int run_ik = 1;
	if ( m->ik_solve_hz > 0.0 )
	{
		double period = 1.0 / m->ik_solve_hz;
		m->ik_acc += dt;
		if ( m->ik_acc + 1e-6 >= period || just_planted != 0u || m->state != GC_STATE_GROUND )
		{
			m->ik_acc = ( m->ik_acc >= period ) ? fmod( m->ik_acc, period ) : 0.0;
			run_ik = 1;
		}
		else
		{
			run_ik = 0;
		}
	}
	for ( int i = 0; i < m->n; ++i )
	{
		GcLeg* l = &m->legs[i];
		if ( !( just_planted & ( 1u << l->idx ) ) || l->sliding )
		{
			if ( run_ik )
				gc_solve_leg( m, l, dt );
			else
			{
				gc_leg_reproject_cached( l, m );
				if ( prof )
					m->prof[9] += 1.0;
			}
		}
		int stressed = ( !m->drifting && l->planted &&
						 ( l->stretch > 0.95 || l->ik.coxa_ratio > 0.84 || l->near_limit ) ) ? 1 : 0;
		l->limit_t = stressed ? ( l->limit_t + dt ) : 0.0;
		if ( stressed && !may_fl && npl_post <= ( m->n / 2 > 1 ? m->n / 2 : 1 ) )
			stressed_grounded = 1;
	}

	int any_limit_t = 0;
	for ( int i = 0; i < m->n; ++i )
		if ( m->legs[i].limit_t > 0.05 ) { any_limit_t = 1; break; }
	if ( !m->drifting && ( stressed_grounded || any_limit_t ) )
	{
		double rate = stressed_grounded ? 4.5 : 2.5;
		double decay_dur = exp( -rate * dt );
		double min_dur = fmax( 0.095, 0.78 * d->swing_min );
		for ( int i = 0; i < m->n; ++i )
		{
			if ( m->legs[i].swing.active )
				m->legs[i].swing.dur = fmax( min_dur, m->legs[i].swing.dur * decay_dur );
		}
	}

	if ( m->state == GC_STATE_GROUND && m->flight_t <= 0.0 && npl_pre > 0 )
	{
		int npl_now = 0;
		for ( int i = 0; i < m->n; ++i )
			if ( m->legs[i].planted ) npl_now++;
		if ( npl_now == 0 )
		{
			double v_up_new = gc_v3_dot( m->vel, up_p );
			if ( gc_try_takeoff( m, up_p, m->vt, &v_up_new, pushed_pre ) )
			{
				m->vel = gc_v3_add( gc_v3_add( m->vt, m->slide_vel ), gc_v3_mul( up_p, v_up_new ) );
				m->prev_npl = 0;
			}
		}
	}

	if ( m->state == GC_STATE_GROUND && m->flight_t <= 0.0 )
	{
		int any_pl = 0;
		for ( int i = 0; i < m->n; ++i )
			if ( m->legs[i].planted ) { any_pl = 1; break; }
		if ( !any_pl )
		{
			GcLeg* best = NULL;
			double best_t = -1.0;
			for ( int i = 0; i < m->n; ++i )
			{
				GcLeg* x = &m->legs[i];
				if ( x->swing.active && x->swing.has_ground && x->swing.t >= 0.90 && x->swing.t > best_t )
				{
					best_t = x->swing.t;
					best = x;
				}
			}
			if ( best &&
				 gc_v3_dist( best->foot, best->swing.target ) < 0.18 &&
				 gc_v3_dist( best->swing.target, best->mount_world ) <= best->reach * 0.99 &&
				 gc_legik_feasible( &best->ik, best->mount_world, &m->frame, best->swing.target, 3.0 ) )
			{
				best->foot = best->swing.target;
				best->foot_normal = best->swing.normal;
				best->planted = 1;
				best->swing.active = 0;
				best->searching = 0;
				gc_on_plant( m, best );
			}
		}
	}
}
