// Knee obstacle avoidance (radial leg-plane roll), swing corridor spacing, and airborne landing brace poses.

#include "gc_internal.h"
#include "gc_thread.h"

// Checks intermediate leg joints (thigh/shin) against world colliders; ignores harmless ankle grazing
// right at the planted floor surface so flat ground never triggers false knee roll.
static void joints_clear_roll( GcMech* m, const GcV3* pts, int nj, int start_j, GcV3 foot, double r, GcV3 up,
							   int planted, int* out_clear, double* out_min_dist )
{
	int last_int = nj - 2;
	int clear = 1;
	double min_dist = 1e9;
	for ( int j_idx = start_j; j_idx <= last_int; ++j_idx )
	{
		GcV3 q = pts[j_idx];
		GcV3 c2, n;
		if ( gc_world_sphere_push( &m->wq, q, r, &c2, &n ) && gc_v3_len_sq( gc_v3_sub( c2, q ) ) > 1e-6 )
		{
			double pen = gc_v3_len( gc_v3_sub( c2, q ) );
			double surf_dy = gc_v3_dot( gc_v3_sub( gc_v3_sub( q, gc_v3_mul( n, r - pen ) ), foot ), up );
			if ( planted && j_idx == last_int && gc_v3_dot( n, up ) > 0.96 && pen < 0.70 * r && fabs( surf_dy ) < 0.08 )
			{
				if ( 0.05 < min_dist ) min_dist = 0.05;
			}
			else
			{
				clear = 0;
				if ( -pen < min_dist ) min_dist = -pen;
			}
		}
		else
		{
			if ( 0.05 < min_dist ) min_dist = 0.05;
		}
	}
	*out_clear = clear;
	*out_min_dist = min_dist;
}

void gc_solve_leg( GcMech* m, GcLeg* l, double dt )
{
	int prof = m->profile_enabled;
	double t0 = prof ? gc_time_sec() : 0.0;
	GcV3 mount_l = gc_v3_add( l->mount_local, m->vis_offset );
	int has_prev_roll = 0;
	double prev_roll = 0.0;

	if ( m->d->plane == GC_PLANE_RADIAL && fabs( l->plane_roll ) > 1e-4 )
	{
		has_prev_roll = 1;
		prev_roll = l->plane_roll;
		double relaxed = prev_roll * exp( -5.0 * dt );
		l->plane_roll = ( fabs( relaxed ) < 1e-3 ) ? 0.0 : relaxed;
	}
	gc_leg_solve( l, m, mount_l, dt );
	if ( l->planted && !l->sliding && l->ik.err > 0.02 )
	{
		// Planted foot pulled out of reach/limits: drop stance cleanly and re-search rather than tearing the mesh.
		l->planted = 0;
		l->searching = 1;
		l->search_cd = 0.0;
		l->foot = l->joints[l->n_joints - 1];
	}
	if ( !l->planted && l->ik.err > 0.02 )
	{
		for ( int it = 0; it < 2; ++it )
		{
			GcV3 e = l->joints[l->n_joints - 1];
			GcV3 c2, n;
			if ( !gc_world_sphere_push( &m->wq, e, l->foot_r, &c2, &n ) ||
				 gc_v3_len_sq( gc_v3_sub( c2, e ) ) < 1e-6 )
				break;
			l->foot = c2;
			gc_leg_solve( l, m, mount_l, dt );
		}
	}
	if ( prof )
	{
		double t1 = gc_time_sec();
		m->prof[6] += t1 - t0;
		m->prof[8] += 1.0;
		t0 = t1;
	}

	if ( m->plane_roll_enabled && m->d->plane == GC_PLANE_RADIAL && l->n_joints >= 4 )
	{
		GcV3 up = gc_mech_probe_up( m );
		int start_j = l->ik.coxa > 1e-6 ? 2 : 1;
		int last_int = l->n_joints - 2;
		double r = l->foot_r * 0.85;
		int has_hit = 0;
		GcV3 hit_push = gc_v3_zero();
		double best_pen = 0.0;
		for ( int j_idx = start_j; j_idx <= last_int; ++j_idx )
		{
			GcV3 j = l->joints[j_idx];
			GcV3 c2, n;
			if ( gc_world_sphere_push( &m->wq, j, r, &c2, &n ) )
			{
				if ( l->planted && j_idx == last_int && gc_v3_dot( n, up ) > 0.96 )
				{
					double pen = gc_v3_len( gc_v3_sub( c2, j ) );
					double surf_dy = gc_v3_dot( gc_v3_sub( gc_v3_sub( j, gc_v3_mul( n, r - pen ) ), l->foot ), up );
					if ( pen < 0.70 * r && fabs( surf_dy ) < 0.08 )
						continue;
				}
				GcV3 dv = gc_v3_sub( c2, j );
				double p2 = gc_v3_len_sq( dv );
				if ( p2 > best_pen && p2 > 1e-6 )
				{
					best_pen = p2;
					hit_push = dv;
					has_hit = 1;
				}
			}
		}
		if ( has_hit )
		{
			const GcFrame* fr = &m->frame;
			GcV3 pos = m->pos;
			GcV3 foot_l = gc_frame_to_local( fr, gc_v3_sub( l->foot, pos ) );
			GcLegIK* probe = &l->ik_probe;
			probe->has_angles = l->ik.has_angles;
			if ( probe->has_angles )
				memcpy( probe->angles, l->ik.angles, sizeof( probe->angles ) );
			probe->has_pts = l->ik.has_pts;
			if ( probe->has_pts )
				memcpy( probe->pts, l->ik.pts, sizeof( probe->pts ) );

			if ( has_prev_roll )
			{
				GcV3 jl_prev[GC_MAX_JOINTS];
				int nj = gc_legik_solve( probe, mount_l, &GC_LOCAL_FRAME, foot_l, prev_roll, jl_prev );
				if ( probe->err <= fmax( 0.02, l->ik.err + 0.01 ) )
				{
					GcV3 pts_prev[GC_MAX_JOINTS];
					for ( int i = 0; i < nj; ++i )
						pts_prev[i] = gc_v3_add( pos, gc_frame_to_world( fr, jl_prev[i] ) );
					int prev_clear;
					double unused_md;
					joints_clear_roll( m, pts_prev, nj, start_j, l->foot, r, up, l->planted, &prev_clear, &unused_md );
					if ( prev_clear )
					{
						l->plane_roll = prev_roll;
						gc_leg_solve( l, m, mount_l, dt );
						if ( prof )
							m->prof[7] += gc_time_sec() - t0;
						return;
					}
				}
			}
			double base_roll = has_prev_roll ? prev_roll : l->plane_roll;
			double best_roll = base_roll;
			double best_score = -sqrt( best_pen ) - 0.5 * fabs( base_roll );
			double side_sgn = gc_v3_dot( hit_push, l->plane_normal ) >= 0.0 ? 1.0 : -1.0;
			static const double mags[5] = { 0.20, 0.42, 0.65, 0.85, 1.10 };
			double cands[10];
			for ( int i = 0; i < 5; ++i )
			{
				cands[i] = side_sgn * mags[i];
				cands[5 + i] = -side_sgn * mags[i];
			}
			for ( int ci = 0; ci < 10; ++ci )
			{
				double cand_roll = cands[ci];
				if ( fabs( cand_roll - base_roll ) < 1e-4 )
					continue;
				GcV3 jl[GC_MAX_JOINTS];
				int nj = gc_legik_solve( probe, mount_l, &GC_LOCAL_FRAME, foot_l, cand_roll, jl );
				if ( probe->err > fmax( 0.02, l->ik.err + 0.01 ) )
					continue;
				GcV3 pts[GC_MAX_JOINTS];
				for ( int i = 0; i < nj; ++i )
					pts[i] = gc_v3_add( pos, gc_frame_to_world( fr, jl[i] ) );
				int clear;
				double min_dist;
				joints_clear_roll( m, pts, nj, start_j, l->foot, r, up, l->planted, &clear, &min_dist );
				double flip_pen = ( fabs( base_roll ) > 0.05 && cand_roll * base_roll < 0.0 ) ? 0.30 : 0.0;
				double sc = ( clear ? 10.0 : min_dist ) - 0.5 * fabs( cand_roll ) - flip_pen;
				if ( sc > best_score )
				{
					best_score = sc;
					best_roll = cand_roll;
				}
				if ( clear && flip_pen == 0.0 && cand_roll * side_sgn > 0.0 )
					break;
			}
			if ( has_prev_roll )
				l->plane_roll = prev_roll;
			if ( fabs( best_roll - l->plane_roll ) > 1e-4 )
			{
				double max_dr = 1200.0 * GC_DEG2RAD * dt;
				l->plane_roll += gc_clamp( best_roll - l->plane_roll, -max_dr, max_dr );
				gc_leg_solve( l, m, mount_l, dt );
			}
			else if ( has_prev_roll )
			{
				gc_leg_solve( l, m, mount_l, dt );
			}
		}
		if ( prof )
			m->prof[7] += gc_time_sec() - t0;
	}
}

void gc_slide_foot_clear( GcMech* m, GcLeg* l, GcV3 up )
{
	double r = l->foot_r;
	GcV3 c = gc_v3_add( l->foot, gc_v3_mul( up, r ) );
	GcV3 c2, n;
	if ( gc_world_sphere_push( &m->wq, c, r, &c2, &n ) && gc_v3_len_sq( gc_v3_sub( c2, c ) ) > 1e-6 )
	{
		l->foot = gc_v3_sub( c2, gc_v3_mul( up, r ) );
		if ( gc_v3_dot( n, up ) < 0.55 )
			l->force = 1;
	}
}

static void clamp_swing_lat( GcMech* m, GcLeg* l, const GcFrame* fr )
{
	if ( m->drifting )
		return;
	GcV3 fl = gc_frame_to_local( fr, gc_v3_sub( l->foot, m->pos ) );
	double max_dx = 0.47 * l->reach;
	double dx = fl.x - l->rest_xz[0];
	if ( fabs( dx ) > max_dx )
	{
		fl.x = l->rest_xz[0] + copysign( max_dx, dx );
		l->foot = gc_v3_add( m->pos, gc_frame_to_world( fr, fl ) );
	}
}

// Keeps swinging feet outside neighboring legs and the hull belly so strides never self-intersect.
void gc_feet_clear( GcMech* m, double dt )
{
	GcV3 up = gc_mech_probe_up( m );
	const GcFrame* fr = &m->frame;
	int radial = ( m->d->plane == GC_PLANE_RADIAL );
	for ( int i = 0; i < m->n; ++i )
	{
		GcLeg* l = &m->legs[i];
		if ( !l->planted )
		{
			if ( radial && l->swing.active )
			{
				const GcSwing* sw = &l->swing;
				double w = gc_smoothstep( sw->t / 0.15 ) * gc_smoothstep( ( 0.94 - sw->t ) / 0.14 );
				if ( w > 1e-4 )
				{
					GcV3 rel = gc_frame_to_local( fr, gc_v3_sub( l->foot, l->mount_world ) );
					GcV3 rd = l->ik.rest_dir_local;
					double out_d = rel.x * rd.x + rel.z * rd.z;
					double side_d = fabs( rel.x * -rd.z + rel.z * rd.x );
					double rest_r = hypot( l->rest_xz[0] - l->mount_local.x, l->rest_xz[1] - l->mount_local.z );
					double cone_out = ( l->ik.coxa_max < 1.5 ) ? ( side_d / tan( fmax( 0.25, l->ik.coxa_max - 0.08 ) ) ) : 0.0;
					double need_out = fmax( 0.46 * rest_r, fmin( 0.85 * rest_r, cone_out ) );
					if ( out_d < need_out )
						l->foot = gc_v3_add( l->foot, gc_v3_mul( gc_frame_to_world( fr, rd ), w * ( need_out - out_d ) ) );
					GcV3 foot_l = gc_frame_to_local( fr, gc_v3_sub( l->foot, m->pos ) );
					double side_sign = l->rest_xz[0] >= 0.0 ? 1.0 : -1.0;
					double min_lat = fmax( 0.35, 0.70 * fabs( l->mount_local.x ) );
					if ( foot_l.x * side_sign < min_lat )
					{
						l->foot = gc_v3_add( l->foot,
											 gc_v3_mul( gc_frame_to_world( fr, gc_v3( side_sign, 0.0, 0.0 ) ),
														w * ( min_lat - foot_l.x * side_sign ) ) );
					}
				}
			}
			if ( l->swing.active )
				clamp_swing_lat( m, l, fr );
			if ( l->swing.active && l->swing.t > 0.05 && l->swing.t < 0.92 )
			{
				const GcSwing* sw = &l->swing;
				double ws = gc_smoothstep( sw->t / 0.12 ) * gc_smoothstep( ( 0.94 - sw->t ) / 0.14 );
				double min_sib = fmax( 0.42, 2.8 * l->foot_r );
				for ( int ni = 0; ni < m->n; ++ni )
				{
					const GcLeg* nb = &m->legs[ni];
					if ( nb->side != l->side || nb == l )
						continue;
					if ( l->ik.mode == GC_PLANE_SAGITTAL )
					{
						double df = gc_v3_dot( gc_v3_sub( l->foot, nb->foot ), fr->fwd );
						double ad = fabs( df );
						if ( ad < min_sib )
						{
							double sgn = ( ad > 1e-4 ) ? ( df >= 0.0 ? 1.0 : -1.0 ) : ( l->rest_xz[1] >= nb->rest_xz[1] ? 1.0 : -1.0 );
							double push_d = ( min_sib - ad ) * ( !nb->planted ? 0.55 : 0.90 ) * ws;
							l->foot = gc_v3_add( l->foot, gc_v3_mul( fr->fwd, sgn * push_d ) );
						}
					}
					else
					{
						GcV3 dv = gc_project_plane( gc_v3_sub( l->foot, nb->foot ), up );
						double dl2 = gc_v3_len_sq( dv );
						if ( 1e-8 < dl2 && dl2 < min_sib * min_sib )
						{
							double dl = sqrt( dl2 );
							double push_d = ( min_sib - dl ) * ( !nb->planted ? 0.55 : 0.90 ) * ws;
							l->foot = gc_v3_add( l->foot, gc_v3_mul( dv, push_d / dl ) );
							clamp_swing_lat( m, l, fr );
						}
					}
				}
			}
			gc_leg_keep_clear( l, &m->wq, up, m->d->step_max, m->has_ceil_y, m->ceil_y, dt );
		}
	}
}

static void ease_foot( const GcMech* m, GcLeg* l, GcV3 target, double k )
{
	const GcFrame* fr = &m->frame;
	GcV3 fl = gc_frame_to_local( fr, gc_v3_sub( l->foot, m->pos ) );
	GcV3 tl = gc_frame_to_local( fr, gc_v3_sub( target, m->pos ) );
	l->foot = gc_v3_add( m->pos, gc_frame_to_world( fr, gc_v3_lerp( fl, tl, k ) ) );
}

static double brace_weight( const GcMech* m )
{
	if ( !m->impact.valid || ( m->vel.y > 0.0 && m->impact.t > 0.5 ) )
		return 0.0;
	double tilt = gc_angle_between( m->frame.up, m->impact.normal );
	double window = 0.45 * sqrt( m->size ) * ( 1.0 + 0.35 * gc_clamp( tilt / 0.8, 0.0, 1.0 ) );
	return gc_clamp( 1.0 - ( m->impact.t - 0.08 ) / window, 0.0, 1.0 );
}

// Blends airborne legs between a tucked flight pose and an extended impact-brace stance
// aligned to the predicted landing normal right before touchdown.
void gc_legs_air( GcMech* m, double dt )
{
	const GcFrame* fr = &m->frame;
	double b = ( m->state == GC_STATE_AIR ) ? brace_weight( m ) : 0.0;
	double k = ( b > 0.0 ) ? gc_damp( 12.0 + 24.0 * b, dt ) : gc_damp( 10.0, dt );
	double hx = m->d->body_half[0], hy = m->d->body_half[1], hz = m->d->body_half[2];
	int has_land_fr = 0;
	GcFrame land_fr;
	GcV3 n = GC_WORLD_UP;
	double spread = 1.0;
	if ( b > 0.0 && m->impact.valid )
	{
		n = m->impact.normal;
		gc_frame_init( &land_fr, gc_mech_heading_fwd( m ), n );
		has_land_fr = 1;
		double tilt = gc_angle_between( fr->up, n );
		spread = 1.0 + 0.25 * gc_clamp( m->impact.speed / 15.0, 0.0, 1.0 ) + 0.18 * gc_clamp( tilt / 0.8, 0.0, 1.0 );
	}
	for ( int i = 0; i < m->n; ++i )
	{
		GcLeg* l = &m->legs[i];
		l->swing.active = 0;
		l->planted = 0;
		l->searching = 0;
		if ( l->push )
		{
			if ( l->stretch < 0.95 && m->state_t < 0.25 )
				continue;
			l->push = 0;
		}
		GcV3 tuck = gc_v3( l->rest_xz[0] * 0.8 + l->mount_local.x * 0.2, -m->ride * 0.72, l->rest_xz[1] * 0.85 );
		GcV3 pose = gc_v3_add( m->pos, gc_frame_to_world( fr, tuck ) );
		if ( has_land_fr )
		{
			GcV3 rel_land = gc_v3( l->rest_xz[0] * spread, -m->ride * 1.05, l->rest_xz[1] * ( 1.0 + ( spread - 1.0 ) * 0.5 ) );
			GcV3 brace = gc_v3_add( m->pos, gc_frame_to_world( &land_fr, rel_land ) );
			GcV3 bl = gc_frame_to_local( fr, gc_v3_sub( brace, m->pos ) );
			if ( fabs( bl.x ) < hx + 0.15 && fabs( bl.y ) < hy + 0.15 && fabs( bl.z ) < hz + 0.15 )
			{
				if ( m->d->plane == GC_PLANE_SAGITTAL )
				{
					bl.y = fmin( bl.y, -( hy + 0.18 ) );
				}
				else
				{
					double sgn_x = l->mount_local.x >= 0.0 ? 1.0 : -1.0;
					bl.x = sgn_x * fmax( fabs( bl.x ), hx + 0.22 );
				}
				brace = gc_v3_add( m->pos, gc_frame_to_world( fr, bl ) );
			}
			GcV3 mount_w = gc_v3_add( m->pos, gc_frame_to_world( fr, l->mount_local ) );
			GcV3 dv = gc_v3_sub( brace, mount_w );
			double dist = gc_v3_len( dv );
			double min_r = 0.30 * l->reach;
			double max_r = 0.94 * l->reach;
			if ( dist < min_r )
			{
				GcV3 dir_v = ( dist > 1e-4 ) ? gc_v3_mul( dv, 1.0 / dist ) : gc_v3_mul( n, -1.0 );
				brace = gc_v3_add( mount_w, gc_v3_mul( dir_v, min_r ) );
			}
			else if ( dist > max_r )
			{
				brace = gc_v3_add( mount_w, gc_v3_mul( dv, max_r / dist ) );
			}
			pose = gc_v3_lerp( pose, brace, b );
		}
		ease_foot( m, l, pose, k );
	}
}

double gc_clearance( const GcMech* m )
{
	if ( m->state == GC_STATE_MANTLE )
		return m->d->mantle_max + 0.6;
	return m->d->step_max + m->d->step_height + 0.4;
}
