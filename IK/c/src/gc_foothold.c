// Foothold selection: predicts the touchdown hip pose, filters cliffs/thin walls/ceilings,
// tries small local nudges around rock tops first, then falls back to ringed searches.

#include "gc_internal.h"

// Projects where the hip mount will be after delay seconds of planar travel and turn yaw.
// Testing reachability from the touchdown hip prevents fast strides from planting out of reach.
void gc_landing_pose( const GcMech* m, const GcLeg* l, double delay, GcV3* out_mount, GcFrame* out_frame )
{
	GcFrame frame = m->frame;
	if ( fabs( m->yaw_rate * delay ) > 1e-6 )
		gc_frame_rotate( &frame, gc_mech_probe_up( m ), -m->yaw_rate * delay );
	GcV3 hull;
	if ( m->state == GC_STATE_MANTLE && m->mantle.active )
		hull = gc_mantle_pos( m, m->mantle.t + delay / m->mantle.dur );
	else
		hull = gc_v3_add( m->pos, gc_planar_travel( m, delay ) );
	if ( m->state == GC_STATE_GROUND && !gc_mech_climbing_mode( m ) )
	{
		if ( m->flight_t > 0.0 )
		{
			double t = m->flight_t < delay ? m->flight_t : delay;
			hull.y += m->vel.y * t - 0.5 * GC_G * t * t;
		}
		else if ( 0.5 < m->ground_n.y && m->ground_n.y < 0.995 )
		{
			double dh_x = hull.x - m->pos.x, dh_z = hull.z - m->pos.z;
			double dy_slope = gc_clamp( -( dh_x * m->ground_n.x + dh_z * m->ground_n.z ) / m->ground_n.y, -3.0, 3.0 );
			if ( dy_slope >= 0.0 || m->speed < 3.0 )
				hull.y += dy_slope;
			else if ( m->vel.y < -0.5 )
				hull.y += fmax( dy_slope, m->vel.y * delay - 0.35 );
		}
	}
	*out_mount = gc_v3_add( hull, gc_frame_to_world( &frame, gc_v3_add( l->mount_local, m->vis_offset ) ) );
	*out_frame = frame;
}

static double sibling_sep_penalty( const GcMech* m, const GcLeg* l, GcV3 p, GcV3 up )
{
	double min_sep = fmax( 0.36, 2.2 * l->foot_r );
	double pen = 0.0;
	for ( int i = 0; i < m->n; ++i )
	{
		const GcLeg* nb = &m->legs[i];
		if ( nb->side != l->side || nb == l )
			continue;
		int has_q = 0;
		GcV3 q = gc_v3_zero();
		if ( nb->planted )
		{
			q = nb->foot;
			has_q = 1;
		}
		else if ( nb->swing.active && nb->swing.has_ground )
		{
			q = nb->swing.target;
			has_q = 1;
		}
		if ( has_q )
		{
			double d = gc_v3_len( gc_project_plane( gc_v3_sub( p, q ), up ) );
			if ( d < min_sep )
				pen += 2.5 * ( min_sep - d );
		}
	}
	return pen;
}

static int surface_ok( const GcMech* m, const GcHit* h, const GcLeg* l )
{
	if ( gc_mech_climbing_mode( m ) )
	{
		return ( gc_v3_dot( h->normal, m->frame.up ) > -0.2 ||
				 gc_v3_dot( h->normal, gc_v3_sub( l->mount_world, h->point ) ) > 0.05 ) ? 1 : 0;
	}
	return h->normal.y > 0.64 ? 1 : 0;
}

int gc_reachable( const GcMech* m, GcLeg* l, GcV3 p, const GcV3* opt_hip, const GcFrame* opt_fr )
{
	GcV3 hip = opt_hip ? *opt_hip : l->mount_world;
	const GcFrame* fr = opt_fr ? opt_fr : &m->frame;
	double d = gc_v3_dist( hip, p );
	if ( !( l->reach * 0.08 <= d && d <= l->reach * 0.96 ) )
		return 0;
	return gc_legik_feasible( &l->ik, hip, fr, p, 0.0 );
}

int gc_clear_spot( GcMech* m, const GcLeg* l, GcV3 p, GcV3 up, const GcV3* opt_mount )
{
	double r = l->foot_r;
	GcV3 c = gc_v3_add( p, gc_v3_mul( up, r + 0.03 ) );
	GcV3 c2, n;
	if ( gc_world_sphere_push( &m->wq, c, r, &c2, &n ) && gc_v3_len_sq( gc_v3_sub( c2, c ) ) > 1e-6 )
		return 0;
	double len_last = l->ik.lengths[l->ik.n_segs - 1];
	GcV3 tgt = gc_v3_add( p, gc_v3_mul( up, fmax( r + 0.1, 0.55 * len_last ) ) );
	GcV3 mount = opt_mount ? *opt_mount : l->mount_world;
	GcV3 dv = gc_v3_sub( tgt, mount );
	double dl = gc_v3_len( dv );
	if ( dl > 0.3 )
	{
		GcHit h;
		if ( gc_world_raycast( &m->wq, mount, gc_v3_mul( dv, 1.0 / dl ), dl - 0.15, &h ) && !gc_mech_climbing_mode( m ) )
			return 0;
	}
	return 1;
}

// Trial-solves the leg chain when nearby colliders threaten knee/shin clearance on a candidate spot.
static int joints_clear( GcMech* m, GcLeg* l, GcV3 p, const GcV3* opt_mount, const GcFrame* opt_fr )
{
	GcV3 up = gc_mech_probe_up( m );
	double last_two = l->ik.lengths[l->ik.n_segs - 1] + ( l->ik.n_segs >= 2 ? l->ik.lengths[l->ik.n_segs - 2] : 0.0 );
	double R = fmax( 0.95, 0.55 * last_two );
	GcV3 c = gc_v3_add( p, gc_v3_mul( up, R + 0.05 ) );
	GcV3 c2, n;
	if ( !gc_world_sphere_push( &m->wq, c, R, &c2, &n ) || gc_v3_len_sq( gc_v3_sub( c2, c ) ) < 1e-6 )
		return 1;

	GcV3 mount = opt_mount ? *opt_mount : l->mount_world;
	const GcFrame* fr = opt_fr ? opt_fr : &m->frame;
	GcV3 ml = gc_v3_add( l->mount_local, m->vis_offset );
	GcV3 hull = gc_v3_sub( mount, gc_frame_to_world( fr, ml ) );
	l->ik_probe.has_angles = 0;
	l->ik_probe.has_pts = 0;
	GcV3 jl[GC_MAX_JOINTS];
	int nj = gc_legik_solve( &l->ik_probe, ml, &GC_LOCAL_FRAME, gc_frame_to_local( fr, gc_v3_sub( p, hull ) ), 0.0, jl );
	double r = l->foot_r * 0.55;
	GcV3 pts[GC_MAX_JOINTS];
	for ( int i = 0; i < nj; ++i )
		pts[i] = gc_v3_add( hull, gc_frame_to_world( fr, jl[i] ) );
	int last_int = nj - 2;
	for ( int j_idx = 1; j_idx <= last_int; ++j_idx )
	{
		GcV3 cc = pts[j_idx];
		if ( gc_world_sphere_push( &m->wq, cc, r, &c2, &n ) && gc_v3_len_sq( gc_v3_sub( c2, cc ) ) > 1e-6 )
		{
			if ( j_idx == last_int && gc_v3_dot( n, up ) > 0.96 )
			{
				double pen = gc_v3_len( gc_v3_sub( c2, cc ) );
				double surf_dy = gc_v3_dot( gc_v3_sub( gc_v3_sub( cc, gc_v3_mul( n, r - pen ) ), p ), up );
				if ( pen < 0.70 * r && fabs( surf_dy ) < 0.08 )
					continue;
			}
			return 0;
		}
	}
	for ( int i = 1; i < nj - 1; ++i )
	{
		GcV3 a = pts[i];
		GcV3 b = ( i == nj - 2 ) ? gc_v3_add( p, gc_v3_mul( up, 2.0 * l->foot_r + 0.05 ) ) : pts[i + 1];
		GcV3 dv = gc_v3_sub( b, a );
		double dl = gc_v3_len( dv );
		GcHit h;
		if ( dl > 0.05 && gc_world_raycast( &m->wq, a, gc_v3_mul( dv, 1.0 / dl ), dl, &h ) )
			return 0;
	}
	return 1;
}

// Clamps vertical ray origin beneath roofs/overpasses and nudges out of solid brush tops.
static void probe_origin( GcMech* m, GcV3 o, GcV3 up, int has_base_y, double base_y, int check_ceil,
						  GcV3* out_o, int* out_has_ceil, double* out_ceil )
{
	int has_local_ceil = 0;
	double local_ceil = 0.0;
	if ( !gc_mech_climbing_mode( m ) )
	{
		has_local_ceil = m->has_ceil_y;
		local_ceil = m->ceil_y;
		double slope_o = 0.0;
		if ( 0.5 < m->ground_n.y && m->ground_n.y < 0.995 )
		{
			slope_o = gc_clamp( -( ( o.x - m->pos.x ) * m->ground_n.x + ( o.z - m->pos.z ) * m->ground_n.z ) / m->ground_n.y,
								-3.0, 3.0 );
		}
		double pos_slope_y = gc_v3_dot( m->pos, up ) + slope_o;
		double start_y = has_base_y ? fmax( base_y, pos_slope_y ) : pos_slope_y;
		double oy = gc_v3_dot( o, up );
		if ( ( check_ceil || has_local_ceil ) && oy > start_y + 0.05 )
		{
			GcV3 start_pt = gc_v3_add( o, gc_v3_mul( up, start_y - oy ) );
			GcHit hc;
			if ( gc_world_raycast( &m->wq, start_pt, up, ( oy - start_y ) + 0.35, &hc ) && gc_v3_dot( hc.normal, up ) < -0.5 )
			{
				double cy = gc_v3_dot( hc.point, up );
				local_ceil = has_local_ceil ? fmin( local_ceil, cy ) : cy;
				has_local_ceil = 1;
			}
		}
		if ( has_local_ceil && oy > local_ceil - 0.12 )
			o = gc_v3_add( o, gc_v3_mul( up, ( local_ceil - 0.12 ) - oy ) );
	}
	for ( int it = 0; it < 4; ++it )
	{
		GcV3 c2, n;
		if ( !gc_world_sphere_push( &m->wq, o, 0.05, &c2, &n ) || gc_v3_len_sq( gc_v3_sub( c2, o ) ) < 1e-8 )
		{
			*out_o = o;
			*out_has_ceil = has_local_ceil;
			*out_ceil = local_ceil;
			return;
		}
		if ( gc_v3_dot( n, up ) < -0.3 )
		{
			*out_o = gc_v3_sub( c2, gc_v3_mul( up, 0.08 ) );
			*out_has_ceil = has_local_ceil;
			*out_ceil = local_ceil;
			return;
		}
		o = gc_v3_add( o, gc_v3_mul( up, 0.6 ) );
		if ( has_local_ceil && gc_v3_dot( o, up ) > local_ceil - 0.10 )
		{
			o = gc_v3_add( o, gc_v3_mul( up, ( local_ceil - 0.10 ) - gc_v3_dot( o, up ) ) );
			*out_o = o;
			*out_has_ceil = has_local_ceil;
			*out_ceil = local_ceil;
			return;
		}
	}
	*out_o = o;
	*out_has_ceil = has_local_ceil;
	*out_ceil = local_ceil;
}

static double score_cand( const GcMech* m, const GcHit* h, GcV3 ideal, GcV3 up, double expected,
						  int obstructed, int has_fwd, GcV3 fwd )
{
	GcV3 dp = gc_v3_sub( h->point, ideal );
	double dv = gc_v3_dot( dp, up );
	double dh = gc_v3_len( gc_v3_sub( dp, gc_v3_mul( up, dv ) ) );
	double dev = gc_v3_dot( h->point, up ) - expected;
	double sm = m->d->step_max;
	double hc = ( dev >= 0.0 ) ? ( 0.8 * fmax( 0.0, dev - sm ) ) : ( 0.4 * -dev );
	double sc = dh + hc + 2.0 * ( 1.0 - gc_v3_dot( h->normal, up ) );
	if ( obstructed && dev > 0.06 && dev <= sm )
		sc -= fmin( 0.28, 0.45 * dev );
	if ( has_fwd && m->speed > 0.3 )
	{
		double back = -gc_v3_dot( gc_v3_sub( h->point, ideal ), fwd );
		if ( back > 0.0 )
			sc += 0.18 * back;
	}
	return sc;
}

static void horiz_axes( const GcMech* m, GcV3 up, GcV3 axes[4] )
{
	GcV3 t1 = gc_v3_norm( gc_project_plane( m->frame.fwd, up ), gc_v3( 0.0, 0.0, 1.0 ) );
	GcV3 t2 = gc_v3_norm( gc_v3_cross( up, t1 ), gc_v3( 1.0, 0.0, 0.0 ) );
	axes[0] = t1;
	axes[1] = t2;
	axes[2] = gc_v3_norm( gc_v3_add( t1, t2 ), t1 );
	axes[3] = gc_v3_norm( gc_v3_sub( t1, t2 ), t1 );
}

static double sheer_drop_dist( const GcMech* m )
{
	return gc_clamp( fmax( m->ride * 0.78, m->d->step_max * 1.35 ), 1.05, 1.55 );
}

static int probe_sheer_drop( GcMech* m, GcV3 pt, GcV3 up, double ref_y, double sheer_drop )
{
	GcV3 o = gc_v3_add( pt, gc_v3_mul( up, 0.45 ) );
	GcHit hh;
	if ( !gc_world_raycast( &m->wq, o, gc_v3_mul( up, -1.0 ), 0.45 + sheer_drop, &hh ) )
	{
		GcV3 c2, hn;
		return !gc_world_sphere_push( &m->wq, o, 0.04, &c2, &hn );
	}
	return ( ref_y - gc_v3_dot( hh.point, up ) ) > sheer_drop ? 1 : 0;
}

static int sheer_overhang( GcMech* m, const GcHit* h, GcV3 up, double r )
{
	double sheer = sheer_drop_dist( m );
	double ref_y = gc_v3_dot( h->point, up );
	double r_chk = fmax( r, 0.08 );
	GcV3 axes[4];
	horiz_axes( m, up, axes );
	for ( int ai = 0; ai < 4; ++ai )
	{
		if ( probe_sheer_drop( m, gc_v3_add( h->point, gc_v3_mul( axes[ai], r_chk ) ), up, ref_y, sheer ) )
			return 1;
		if ( probe_sheer_drop( m, gc_v3_sub( h->point, gc_v3_mul( axes[ai], r_chk ) ), up, ref_y, sheer ) )
			return 1;
	}
	return 0;
}

// Detects narrow beams and thin walls so wide mechs don't perch on knifeblade ridges and get stuck.
static int is_thin_wall( GcMech* m, const GcHit* h, GcV3 up )
{
	double sheer = sheer_drop_dist( m );
	double ref_y = gc_v3_dot( h->point, up );
	const double r1 = 0.52, r2 = 0.96;
	GcV3 axes[4];
	horiz_axes( m, up, axes );
	for ( int ai = 0; ai < 4; ++ai )
	{
		int d_pos = probe_sheer_drop( m, gc_v3_add( h->point, gc_v3_mul( axes[ai], r1 ) ), up, ref_y, sheer );
		int d_neg = probe_sheer_drop( m, gc_v3_sub( h->point, gc_v3_mul( axes[ai], r1 ) ), up, ref_y, sheer );
		if ( d_pos && d_neg )
			return 1;
		if ( d_pos && probe_sheer_drop( m, gc_v3_sub( h->point, gc_v3_mul( axes[ai], r2 ) ), up, ref_y, sheer ) )
			return 1;
		if ( d_neg && probe_sheer_drop( m, gc_v3_add( h->point, gc_v3_mul( axes[ai], r2 ) ), up, ref_y, sheer ) )
			return 1;
	}
	return 0;
}

static int near_edge( GcMech* m, const GcHit* h, GcV3 up, double r )
{
	GcV3 t1 = gc_v3_norm( gc_project_plane( m->frame.fwd, up ), gc_v3( 0.0, 0.0, 1.0 ) );
	GcV3 t2 = gc_v3_cross( up, t1 );
	GcV3 dvs[4] = { t1, gc_v3_mul( t1, -1.0 ), t2, gc_v3_mul( t2, -1.0 ) };
	for ( int i = 0; i < 4; ++i )
	{
		GcV3 o = gc_v3_add( gc_v3_add( h->point, gc_v3_mul( dvs[i], r ) ), gc_v3_mul( up, 0.35 ) );
		GcHit hh;
		if ( !gc_world_raycast( &m->wq, o, gc_v3_mul( up, -1.0 ), 0.7, &hh ) ||
			 fabs( gc_v3_dot( gc_v3_sub( hh.point, h->point ), up ) ) > 0.2 )
		{
			return 1;
		}
	}
	return 0;
}

typedef struct FhCand {
	double sc;
	GcHit h;
} FhCand;

static int cmp_fhcand( const void* a, const void* b )
{
	double sa = ( (const FhCand*)a )->sc;
	double sb = ( (const FhCand*)b )->sc;
	return sa < sb ? -1 : ( sa > sb ? 1 : 0 );
}

// Offsets the planted foot slightly out along the surface normal so the ankle/pad clears geometry.
static void fh_out( const GcHit* h, GcV3* out_pt, GcV3* out_n )
{
	*out_pt = gc_v3_add( h->point, gc_v3_mul( h->normal, 0.02 ) );
	*out_n = h->normal;
}

int gc_find_foothold( GcMech* m, GcLeg* l, GcV3 ideal, double delay, GcV3* out_pt, GcV3* out_n )
{
	const GcMechDef* d = m->d;
	GcWorldQueryCtx* wq = &m->wq;
	int cm = gc_mech_climbing_mode( m );
	GcV3 up = cm ? m->frame.up : GC_WORLD_UP;

	double slope_rise = 0.0;
	if ( !cm && 0.5 < m->ground_n.y && m->ground_n.y < 0.995 )
	{
		double raw_sr = gc_clamp(
			-( ( ideal.x - m->pos.x ) * m->ground_n.x + ( ideal.z - m->pos.z ) * m->ground_n.z ) / m->ground_n.y,
			-3.0, 3.0 );
		if ( raw_sr >= 0.0 || m->speed < 3.0 || m->vel.y < -0.5 )
			slope_rise = raw_sr;
	}
	double probe_up = ( m->state == GC_STATE_MANTLE )
						  ? ( d->mantle_max + 0.6 )
						  : ( fmax( d->step_max, d->step_height ) * 1.5 + 0.3 + fmax( 0.0, slope_rise ) );
	double probe_down = m->ride * 1.3 + d->step_max + 0.5 + fmax( 0.0, -slope_rise );
	GcV3 origin = gc_v3_add( ideal, gc_v3_mul( up, probe_up ) );

	int has_pose = ( delay > 0.0 );
	GcV3 pose_mount;
	GcFrame pose_frame;
	if ( has_pose )
		gc_landing_pose( m, l, delay, &pose_mount, &pose_frame );
	GcV3 mount = has_pose ? pose_mount : l->mount_world;
	const GcV3* opt_hip = has_pose ? &pose_mount : NULL;
	const GcFrame* opt_fr = has_pose ? &pose_frame : NULL;

	int has_base_y = !cm;
	double base_y = has_base_y ? fmax( gc_v3_dot( mount, up ), gc_v3_dot( m->pos, up ) + slope_rise ) : 0.0;
	int has_ceil_lim = ( !cm && m->has_ceil_y );
	double ceil_lim = m->ceil_y;
	if ( !cm )
	{
		GcHit m_hc;
		if ( gc_world_raycast( wq, mount, up, probe_up + 1.5, &m_hc ) && gc_v3_dot( m_hc.normal, up ) < -0.5 )
		{
			double my_c = gc_v3_dot( m_hc.point, up );
			ceil_lim = has_ceil_lim ? fmin( ceil_lim, my_c ) : my_c;
			has_ceil_lim = 1;
		}
	}
	double lift_o = gc_v3_dot( gc_v3_sub( mount, origin ), up );
	if ( lift_o > 0.0 )
		origin = gc_v3_add( origin, gc_v3_mul( up, lift_o ) );
	GcV3 los = gc_v3_sub( origin, mount );
	double L = gc_v3_len( los );
	if ( L > 1e-4 )
	{
		GcHit h0;
		if ( gc_world_raycast( wq, mount, gc_v3_mul( los, 1.0 / L ), L, &h0 ) )
		{
			if ( cm && surface_ok( m, &h0, l ) && gc_reachable( m, l, h0.point, opt_hip, opt_fr ) )
			{
				fh_out( &h0, out_pt, out_n );
				return 1;
			}
			origin = gc_v3_add( mount, gc_v3_mul( los, fmax( 0.0, h0.t - 0.25 ) / L ) );
		}
	}

	double expected = gc_v3_dot( gc_v3_sub( m->pos, gc_v3_mul( up, gc_mech_ride_eff( m ) ) ), up ) + slope_rise;
	FhCand cands[32];
	int n_cands = 0;

	int has_c_o = 0;
	double c_o = 0.0;
	probe_origin( m, origin, up, has_base_y, base_y, 1, &origin, &has_c_o, &c_o );
	if ( has_c_o )
	{
		ceil_lim = has_ceil_lim ? fmin( ceil_lim, c_o ) : c_o;
		has_ceil_lim = 1;
	}

	GcHit h1;
	int has_h1 = gc_world_raycast( wq, origin, gc_v3_mul( up, -1.0 ),
								   fmax( 1.0, gc_v3_dot( gc_v3_sub( origin, ideal ), up ) + probe_down ), &h1 );
	if ( has_h1 && has_ceil_lim && gc_v3_dot( h1.point, up ) >= ceil_lim - 0.08 )
		has_h1 = 0;
	int ok1 = has_h1 && surface_ok( m, &h1, l );
	int ok2 = ok1 && gc_reachable( m, l, h1.point, opt_hip, opt_fr );
	int ok3 = ok2 && gc_clear_spot( m, l, h1.point, up, &mount );

	m->fh_count += 1;
	m->fh_bad += 1;
	const char* why = !has_h1 ? "none" : ( !ok1 ? "surface" : ( !ok2 ? "reach" : ( !ok3 ? "clear" : "ok" ) ) );
	snprintf( m->fh_why, sizeof( m->fh_why ), "%s", why );
	if ( strcmp( why, "reach" ) == 0 || strcmp( why, "none" ) == 0 )
		m->fh_bad -= 1;

	int h1_cand = 0, is_near = 0;
	double sep_pen = 0.0;
	if ( ok3 )
	{
		if ( cm )
		{
			fh_out( &h1, out_pt, out_n );
			return 1;
		}
		is_near = near_edge( m, &h1, up, l->foot_r );
		sep_pen = sibling_sep_penalty( m, l, h1.point, up );
		int overhang = sheer_overhang( m, &h1, up, l->foot_r );
		int bad_cliff = overhang || is_thin_wall( m, &h1, up );
		if ( !is_near && !bad_cliff && sep_pen <= 0.25 && joints_clear( m, l, h1.point, opt_hip, opt_fr ) )
		{
			m->fh_bad -= 1;
			fh_out( &h1, out_pt, out_n );
			return 1;
		}
		if ( !is_near && !bad_cliff && sep_pen > 0.25 )
			m->fh_bad -= 1;
		snprintf( m->fh_why, sizeof( m->fh_why ), "%s",
				  ( is_near || bad_cliff ) ? "edge" : ( sep_pen > 0.25 ? "sibling" : "joints" ) );
		h1_cand = ( ( is_near || sep_pen > 0.25 ) && !bad_cliff ) ? 1 : 0;
	}
	else if ( cm && !has_h1 )
	{
		GcV3 end = gc_v3_sub( origin, gc_v3_mul( up, probe_up + probe_down * 0.6 ) );
		GcV3 tgt = gc_v3_sub( m->pos, gc_v3_mul( up, m->ride ) );
		GcV3 dv = gc_v3_sub( tgt, end );
		double dl = gc_v3_len( dv );
		if ( dl > 1e-4 )
		{
			GcHit h2;
			if ( gc_world_raycast( wq, end, gc_v3_mul( dv, 1.0 / dl ), dl + l->reach, &h2 ) &&
				 surface_ok( m, &h2, l ) && gc_reachable( m, l, h2.point, opt_hip, opt_fr ) )
			{
				fh_out( &h2, out_pt, out_n );
				return 1;
			}
		}
	}

	GcV3 t1 = gc_v3_norm( gc_project_plane( m->frame.fwd, up ), gc_v3( 0.0, 0.0, 1.0 ) );
	GcV3 t2 = gc_v3_cross( up, t1 );
	double zone = d->zone;
	int has_fwd_d = 0;
	GcV3 fwd_d = gc_v3_zero();
	int obstructed = 0;
	if ( !cm )
	{
		has_fwd_d = 1;
		fwd_d = ( m->speed > 0.3 ) ? gc_v3_norm( gc_project_plane( m->vt, up ), t1 ) : t1;
		GcV3 obs_o = gc_v3( ideal.x, expected + 0.30 * d->step_max, ideal.z );
		GcHit obs_h;
		obstructed = gc_world_raycast( wq, obs_o, fwd_d, fmax( 0.55, 0.8 * zone ), &obs_h );
	}
	if ( h1_cand )
	{
		cands[n_cands].sc = score_cand( m, &h1, ideal, up, expected, obstructed, has_fwd_d, fwd_d ) +
							( is_near ? 0.6 : 0.0 ) + sep_pen;
		cands[n_cands].h = h1;
		n_cands++;
	}

	double nudge = fmax( 0.2, 2.0 * l->foot_r );
	int has_ceil = has_ceil_lim;
	// Try tight local nudges around the ideal spot before searching outer zone rings;
	// this keeps feet stepping onto boulders instead of dodging widely around them.
	double radii[3] = { nudge, zone * 0.4, zone * 0.8 };
	double ang0[3] = { GC_PI / 8.0, 0.0, GC_PI / 8.0 };

	for ( int ring = 0; ring < 3; ++ring )
	{
		if ( ring == 1 && n_cands > 0 )
		{
			qsort( cands, (size_t)n_cands, sizeof( FhCand ), cmp_fhcand );
			int lim = n_cands < 3 ? n_cands : 3;
			for ( int ci = 0; ci < lim; ++ci )
			{
				const GcHit* h = &cands[ci].h;
				if ( cands[ci].sc < 0.5 * zone &&
					 !near_edge( m, h, up, l->foot_r ) &&
					 sibling_sep_penalty( m, l, h->point, up ) <= 0.25 &&
					 joints_clear( m, l, h->point, opt_hip, opt_fr ) &&
					 ( cm || ( !sheer_overhang( m, h, up, l->foot_r ) && !is_thin_wall( m, h, up ) ) ) )
				{
					if ( !cm )
						m->fh_bad -= 1;
					fh_out( h, out_pt, out_n );
					return 1;
				}
			}
		}
		double rad = radii[ring], a0 = ang0[ring];
		for ( int k = 0; k < 8; ++k )
		{
			double a = a0 + (double)k * ( GC_PI / 4.0 );
			GcV3 p = gc_v3_add( ideal, gc_v3_add( gc_v3_mul( t1, cos( a ) * rad ), gc_v3_mul( t2, sin( a ) * rad ) ) );
			GcV3 o;
			int has_co = 0;
			double co = 0.0;
			probe_origin( m, gc_v3_add( p, gc_v3_mul( up, probe_up ) ), up, has_base_y, base_y, has_ceil, &o, &has_co, &co );
			GcHit h;
			if ( !gc_world_raycast( wq, o, gc_v3_mul( up, -1.0 ), fmax( 1.0, gc_v3_dot( gc_v3_sub( o, p ), up ) + probe_down ), &h ) )
				continue;
			int has_cl = has_co || has_ceil_lim;
			double c_lim = !has_co ? ceil_lim : ( !has_ceil_lim ? co : fmin( ceil_lim, co ) );
			if ( ( has_cl && gc_v3_dot( h.point, up ) >= c_lim - 0.08 ) ||
				 !surface_ok( m, &h, l ) ||
				 !gc_reachable( m, l, h.point, opt_hip, opt_fr ) ||
				 !gc_clear_spot( m, l, h.point, up, &mount ) )
				continue;
			double sp = cm ? 0.0 : sibling_sep_penalty( m, l, h.point, up );
			cands[n_cands].sc = score_cand( m, &h, ideal, up, expected, obstructed, has_fwd_d, fwd_d ) + sp;
			cands[n_cands].h = h;
			n_cands++;
		}
	}

	qsort( cands, (size_t)n_cands, sizeof( FhCand ), cmp_fhcand );
	int has_edge_fb = 0;
	GcHit edge_fb;
	int after_fb = 0;
	for ( int ci = 0; ci < n_cands; ++ci )
	{
		const GcHit* h = &cands[ci].h;
		if ( cm )
		{
			if ( joints_clear( m, l, h->point, opt_hip, opt_fr ) )
			{
				fh_out( h, out_pt, out_n );
				return 1;
			}
			continue;
		}
		if ( near_edge( m, h, up, l->foot_r ) )
		{
			if ( !has_edge_fb && joints_clear( m, l, h->point, opt_hip, opt_fr ) &&
				 !sheer_overhang( m, h, up, l->foot_r ) && !is_thin_wall( m, h, up ) )
			{
				has_edge_fb = 1;
				edge_fb = *h;
			}
			if ( has_edge_fb )
			{
				after_fb++;
				if ( after_fb >= 6 )
					break;
			}
			continue;
		}
		if ( joints_clear( m, l, h->point, opt_hip, opt_fr ) &&
			 !sheer_overhang( m, h, up, l->foot_r ) && !is_thin_wall( m, h, up ) )
		{
			fh_out( h, out_pt, out_n );
			return 1;
		}
	}
	if ( has_edge_fb )
	{
		fh_out( &edge_fb, out_pt, out_n );
		return 1;
	}

	if ( !cm )
	{
		const GcFrame* fr_use = has_pose ? &pose_frame : &m->frame;
		GcV3 right_u = gc_v3_norm( gc_project_plane( fr_use->right, up ), t2 );
		double lat_ref = fabs( l->mount_local.x ) > 1e-4 ? l->mount_local.x : l->rest_xz[0];
		double lat_sgn = lat_ref >= 0.0 ? 1.0 : -1.0;
		GcV3 hull_c = has_pose ? gc_v3_sub( mount, gc_frame_to_world( &pose_frame, gc_v3_add( l->mount_local, m->vis_offset ) ) ) : m->pos;
		GcHit hw_side;
		int has_hw = gc_world_raycast( wq, hull_c, gc_v3_mul( right_u, lat_sgn ), fabs( l->rest_xz[0] ) + 0.25, &hw_side );
		int wall_side = has_hw && fabs( gc_v3_dot( hw_side.normal, up ) ) < 0.55;
		double min_lat = wall_side ? 0.18 : fmax( 0.52, 0.52 * fabs( l->mount_local.x ) );

		GcV3 bases[2];
		int n_bases = 1;
		bases[0] = ideal;
		GcV3 rest = !has_pose ? l->ideal_now : gc_v3_add( l->ideal_now, gc_v3_sub( mount, l->mount_world ) );
		if ( gc_v3_dist( rest, ideal ) > 0.3 )
			bases[n_bases++] = rest;

		GcV3 pts[64];
		int n_pts = 0;
		static const double fs1[4] = { 0.38, 0.62, 0.74, 0.85 };
		static const double fs2[3] = { 0.35, 0.58, 0.82 };
		for ( int bi = 0; bi < n_bases; ++bi )
		{
			GcV3 base = bases[bi];
			GcV3 hub = gc_project_plane( gc_v3_sub( hull_c, base ), up );
			GcV3 lat = gc_v3_mul( t2, gc_v3_dot( hub, t2 ) );
			double fwd_sgn = l->rest_xz[1] >= 0.0 ? 1.0 : -1.0;
			for ( int fi = 0; fi < 4; ++fi )
			{
				GcV3 pt = gc_v3_add( base, gc_v3_mul( lat, fs1[fi] ) );
				pts[n_pts++] = pt;
				pts[n_pts++] = gc_v3_add( pt, gc_v3_mul( t1, fwd_sgn * 0.85 ) );
				pts[n_pts++] = gc_v3_sub( pt, gc_v3_mul( t1, fwd_sgn * 0.55 ) );
			}
			for ( int fi = 0; fi < 3; ++fi )
			{
				GcV3 pt = gc_v3_add( base, gc_v3_mul( hub, fs2[fi] ) );
				pts[n_pts++] = pt;
				if ( fi == 1 )
				{
					pts[n_pts++] = gc_v3_add( pt, gc_v3_mul( t1, 0.65 ) );
					pts[n_pts++] = gc_v3_sub( pt, gc_v3_mul( t1, 0.65 ) );
				}
			}
		}

		for ( int pi = 0; pi < n_pts; ++pi )
		{
			GcV3 p = pts[pi];
			if ( gc_v3_dot( gc_project_plane( gc_v3_sub( p, hull_c ), up ), right_u ) * lat_sgn < min_lat )
				continue;
			GcV3 o;
			int has_co = 0;
			double co = 0.0;
			probe_origin( m, gc_v3_add( p, gc_v3_mul( up, probe_up ) ), up, has_base_y, base_y, has_ceil, &o, &has_co, &co );
			GcHit h;
			if ( !gc_world_raycast( wq, o, gc_v3_mul( up, -1.0 ), fmax( 1.0, gc_v3_dot( gc_v3_sub( o, p ), up ) + probe_down ), &h ) )
				continue;
			int has_cl = has_co || has_ceil_lim;
			double c_lim = !has_co ? ceil_lim : ( !has_ceil_lim ? co : fmin( ceil_lim, co ) );
			if ( ( !has_cl || gc_v3_dot( h.point, up ) < c_lim - 0.08 ) &&
				 surface_ok( m, &h, l ) &&
				 gc_reachable( m, l, h.point, opt_hip, opt_fr ) &&
				 gc_clear_spot( m, l, h.point, up, &mount ) &&
				 joints_clear( m, l, h.point, opt_hip, opt_fr ) &&
				 !sheer_overhang( m, &h, up, l->foot_r ) &&
				 !is_thin_wall( m, &h, up ) )
			{
				fh_out( &h, out_pt, out_n );
				return 1;
			}
		}
	}

	return 0;
}

GcV3 gc_hang_pose( GcMech* m, const GcLeg* l, double ride )
{
	GcV3 up = gc_mech_probe_up( m );
	GcV3 hip = l->mount_world;
	GcV3 rest = l->ideal_now;
	GcV3 dv = gc_project_plane( gc_v3_sub( rest, hip ), up );
	double dh = gc_v3_len( dv );
	double R = 0.9 * l->reach;
	if ( dh > R )
	{
		rest = gc_v3_sub( gc_v3_add( hip, gc_v3_mul( dv, R / dh ) ),
						  gc_v3_mul( up, gc_v3_dot( gc_v3_sub( hip, rest ), up ) ) );
		dh = R;
	}
	double y0 = gc_v3_dot( gc_v3_sub( hip, rest ), up );
	double t = sqrt( fmax( 0.0, R * R - dh * dh ) ) - y0;
	GcHit g;
	if ( gc_world_raycast( &m->wq, gc_v3_add( rest, gc_v3_mul( up, ride * 0.5 ) ),
						   gc_v3_mul( up, -1.0 ), ride * 0.5 + fmax( t, 0.0 ) + 0.5, &g ) )
	{
		t = fmin( t, gc_v3_dot( gc_v3_sub( rest, g.point ), up ) - l->foot_r );
	}
	return gc_v3_sub( rest, gc_v3_mul( up, fmax( t, -ride * 0.3 ) ) );
}

// Rejects redundant re-steps when the best found foothold sits right next to the planted foot,
// stopping idle jitter on uneven rubble.
int gc_futile_step( GcLeg* l, int has_fh, GcV3 fh_p )
{
	if ( has_fh && gc_v3_dist( fh_p, l->foot ) < 0.12 )
	{
		l->cooldown = fmax( l->cooldown, 0.25 );
		l->force = 0;
		return 1;
	}
	return 0;
}
