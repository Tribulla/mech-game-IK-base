// Forward terrain lookahead, speed/ground gait-ladder hysteresis, and phase
// blending.

#include "gc_internal.h"

void gc_situation_init( GcSituation* s )
{
	memset( s, 0, sizeof( *s ) );
	s->probe_t = 1.0;
}

static int cmp_double_asc( const void* a, const void* b )
{
	double da = *(const double*)a;
	double db = *(const double*)b;
	return ( da < db ) ? -1 : ( ( da > db ) ? 1 : 0 );
}

typedef struct ProbeHitRec {
	int pt_idx;
	GcV3 point;
	GcV3 normal;
} ProbeHitRec;

// Sample terrain in front of the mech along its travel direction so gait
// selection can downshift before hitting rubble clusters or cliff edges.
static void probe_ahead( GcSituation* sit, GcMech* m, double* out_rough, double* out_slope, double* out_void, double* out_l )
{
	(void)sit;
	GcWorldQueryCtx* wq = &m->wq;
	GcV3 up = GC_WORLD_UP;
	GcV3 fwd = ( m->speed > 1.0 ) ? gc_v3_norm( gc_project_plane( m->vt, up ), gc_mech_heading_fwd( m ) )
								  : gc_mech_heading_fwd( m );
	double curvature = ( m->speed > 1.0 && gc_galloping_fast( m, 0 ) && !m->drifting ) ? ( -m->yaw_rate / m->speed ) : 0.0;
	double hz = m->d->body_half[2];
	double sm = m->d->step_max;
	double acc_r = m->d->accel_run > 0.0 ? m->d->accel_run : m->d->accel;
	double decel = fmax( 6.0, 1.4 * acc_r );
	double brake_d = ( m->speed * m->speed ) / ( 2.0 * decel );
	double min_l = ( m->selector.cap < 3 ) ? fmax( 2.5 * hz, 9.0 ) : ( 2.5 * hz );
	double L = gc_clamp( fmax( m->speed * 0.65, brake_d + 0.35 * m->speed ), min_l, 42.0 );
	double spread = gc_clamp( 0.45 * hz + 0.40, 0.70, 1.15 );

	GcHit g0;
	int has_g0 = gc_world_raycast( wq, gc_v3( m->pos.x, m->pos.y + 1.0, m->pos.z ), gc_v3_mul( up, -1.0 ), m->ride + 3.0, &g0 );
	double base = ( has_g0 && gc_v3_dot( g0.normal, up ) > 0.64 ) ? gc_v3_dot( g0.point, up ) : ( gc_v3_dot( m->pos, up ) - m->ride );

	GcHit ceil_hit;
	int has_max_o_y = gc_world_raycast( wq, m->pos, up, 3.5, &ceil_hit );
	double max_o_y = has_max_o_y ? ( gc_v3_dot( ceil_hit.point, up ) - 0.15 ) : 0.0;

	int void_cnt = 0;
	int n_probes = 0;
	double cl_rough[3];
	int n_cl_rough = 0;
	double intra_slopes[8];
	int n_intra = 0;
	double inter_slopes[4];
	int n_inter = 0;

	int has_prev_h = 1;
	double prev_h = base;
	double prev_d = 0.0;
	int prev_flat = ( !has_g0 || gc_v3_dot( g0.normal, up ) >= 0.96 ) ? 1 : 0;

	static const double alphas[3] = { 0.22, 0.56, 1.0 };
	static const double offs[5][2] = { { 0.0, 0.0 }, { 1.0, 0.0 }, { -1.0, 0.0 }, { 0.0, 1.0 }, { 0.0, -1.0 } };

	for ( int ai = 0; ai < 3; ++ai )
	{
		double dist = hz + alphas[ai] * L;
		double angle = curvature * dist;
		GcV3 centre = gc_v3_add( m->pos, gc_turn_displacement( gc_v3_mul( fwd, dist ), up, angle ) );
		GcV3 tangent = ( fabs( curvature ) > 0.0 ) ? gc_rotate_axis( fwd, up, angle ) : fwd;
		GcV3 right = gc_v3_cross( up, tangent );

		ProbeHitRec hits[5];
		int n_hits = 0;
		int steep_hits = 0;

		for ( int oi = 0; oi < 5; ++oi )
		{
			double dx_o = offs[oi][0], dz_o = offs[oi][1];
			GcV3 p = gc_v3_add( centre, gc_v3_add( gc_v3_mul( right, dx_o * spread ), gc_v3_mul( tangent, dz_o * spread ) ) );
			double o_y = ( has_prev_h ? prev_h : base ) + 2.6 + sm;
			if ( has_max_o_y && o_y > max_o_y )
				o_y = max_o_y;
			GcV3 o = gc_v3( p.x, o_y, p.z );
			double ray_d = fmax( 1.0, o_y - ( base - 2.0 * sm - 0.5 * dist ) );
			GcHit h;
			int got = gc_world_raycast( wq, o, gc_v3_mul( up, -1.0 ), ray_d, &h );
			n_probes++;
			if ( !got || gc_v3_dot( h.point, up ) < base - 2.0 * sm - 0.5 * dist )
			{
				void_cnt++;
				continue;
			}
			if ( gc_v3_dot( h.normal, up ) < 0.55 )
			{
				if ( gc_v3_dot( h.point, up ) > base + 0.55 * sm )
					steep_hits++;
				continue;
			}
			hits[n_hits].pt_idx = oi;
			hits[n_hits].point = h.point;
			hits[n_hits].normal = h.normal;
			n_hits++;
		}

		if ( n_hits > 0 )
		{
			double ys_all[5];
			for ( int i = 0; i < n_hits; ++i )
				ys_all[i] = gc_v3_dot( hits[i].point, up );
			qsort( ys_all, (size_t)n_hits, sizeof( double ), cmp_double_asc );
			double ref_y = ( ys_all[n_hits - 1] - ys_all[0] > 3.5 * sm ) ? ys_all[n_hits - 1] : ys_all[n_hits / 2];
			int write_i = 0;
			for ( int i = 0; i < n_hits; ++i )
			{
				if ( gc_v3_dot( hits[i].point, up ) >= ref_y - 2.0 * sm )
					hits[write_i++] = hits[i];
			}
			void_cnt += ( n_hits - write_i );
			n_hits = write_i;
		}

		int two_level = 0;
		double step_h = 0.0;
		if ( n_hits >= 2 )
		{
			double y_lo = 1e18, y_hi = -1e18;
			int all_flat_n = 1;
			for ( int i = 0; i < n_hits; ++i )
			{
				double y = gc_v3_dot( hits[i].point, up );
				if ( y < y_lo ) y_lo = y;
				if ( y > y_hi ) y_hi = y;
				if ( gc_v3_dot( hits[i].normal, up ) < 0.98 )
					all_flat_n = 0;
			}
			step_h = y_hi - y_lo;
			if ( all_flat_n )
			{
				int ok_levels = 1;
				for ( int i = 0; i < n_hits; ++i )
				{
					double y = gc_v3_dot( hits[i].point, up );
					if ( !( ( y - y_lo <= 0.05 ) || ( y_hi - y <= 0.05 ) ) )
					{
						ok_levels = 0;
						break;
					}
				}
				two_level = ok_levels;
			}
		}

		GcV3 cl[5];
		for ( int i = 0; i < n_hits; ++i )
			cl[i] = gc_v3( hits[i].point.x - m->pos.x, hits[i].point.y, hits[i].point.z - m->pos.z );

		if ( n_hits < 4 )
		{
			if ( steep_hits >= 2 )
				cl_rough[n_cl_rough++] = 0.24 * sm;
			else if ( steep_hits == 1 )
				cl_rough[n_cl_rough++] = 0.15 * sm;
			else if ( n_hits >= 2 )
			{
				if ( two_level )
					cl_rough[n_cl_rough++] = ( step_h <= sm + 0.05 ) ? 0.0 : fmin( 0.155 * sm, 0.5 * step_h );
				else if ( step_h > 0.16 * sm )
					cl_rough[n_cl_rough++] = 0.5 * step_h;
			}
			has_prev_h = 0;
			prev_d = dist;
			prev_flat = 0;
			continue;
		}

		int has_pt[5] = { 0, 0, 0, 0, 0 };
		double pt_y[5] = { 0, 0, 0, 0, 0 };
		GcV3 pt_n[5];
		double ny_vals[5], min_ny = 1e9, nr_min = 1e9, nr_max = -1e9;
		for ( int i = 0; i < n_hits; ++i )
		{
			int pid = hits[i].pt_idx;
			has_pt[pid] = 1;
			pt_y[pid] = gc_v3_dot( hits[i].point, up );
			pt_n[pid] = hits[i].normal;
			double ny = gc_v3_dot( hits[i].normal, up );
			ny_vals[i] = ny;
			if ( ny < min_ny ) min_ny = ny;
			double nr = gc_v3_dot( hits[i].normal, right );
			if ( nr < nr_min ) nr_min = nr;
			if ( nr > nr_max ) nr_max = nr;
		}
		qsort( ny_vals, (size_t)n_hits, sizeof( double ), cmp_double_asc );
		double med_ny = ny_vals[n_hits / 2];

		// Smooth ramps have tilted normals, so a blind vertical height variance
		// check mistakes them for rubble. Require three aligned, coplanar
		// samples with gentle local roughness before discounting the slope.
		int is_smooth_ramp = (
			has_pt[0] && has_pt[1] && has_pt[2] &&
			fabs( pt_y[1] - pt_y[0] ) < 0.04 &&
			fabs( pt_y[2] - pt_y[0] ) < 0.04 &&
			gc_v3_dot( pt_n[1], pt_n[0] ) > 0.995 &&
			gc_v3_dot( pt_n[2], pt_n[0] ) > 0.995 &&
			min_ny < 0.985 &&
			( nr_max - nr_min ) < 0.08
		) ? 1 : 0;

		GcV3 c, nrm;
		if ( gc_fit_plane( cl, NULL, n_hits, &c, &nrm ) < 2 )
			continue;

		double res_sum = 0.0;
		for ( int i = 0; i < n_hits; ++i )
		{
			double d_pl = gc_v3_dot( gc_v3_sub( cl[i], c ), nrm );
			res_sum += d_pl * d_pl;
		}
		double res = sqrt( res_sum / (double)n_hits );
		if ( two_level && steep_hits == 0 )
			res = ( step_h <= sm + 0.05 ) ? 0.0 : fmin( res, 0.155 * sm );
		if ( steep_hits >= 2 )
			res = fmax( res, 0.24 * sm );
		else if ( steep_hits == 1 )
			res = fmax( res, 0.15 * sm );
		cl_rough[n_cl_rough++] = ( is_smooth_ramp && steep_hits == 0 ) ? 0.0 : res;

		if ( is_smooth_ramp && med_ny < 0.985 )
			intra_slopes[n_intra++] = acos( gc_clamp( med_ny, -1.0, 1.0 ) ) * GC_RAD2DEG;

		if ( has_pt[4] && has_pt[0] && has_pt[3] )
		{
			double h_back = pt_y[4], h_mid = pt_y[0], h_fwd = pt_y[3];
			double dh = h_fwd - h_back;
			if ( 0.45 <= fabs( dh ) && fabs( dh ) <= fmax( 1.2 * sm, 1.6 * spread ) )
			{
				double frac = ( h_mid - h_back ) / dh;
				if ( 0.20 <= frac && frac <= 0.80 )
					intra_slopes[n_intra++] = atan2( fabs( dh ), 2.0 * spread ) * GC_RAD2DEG;
			}
		}

		double ys_cl[5];
		for ( int i = 0; i < n_hits; ++i )
			ys_cl[i] = cl[i].y;
		qsort( ys_cl, (size_t)n_hits, sizeof( double ), cmp_double_asc );
		double hm = ys_cl[n_hits / 2];
		int cur_flat = ( med_ny >= 0.985 && min_ny >= 0.98 && res < 0.05 ) ? 1 : 0;
		if ( has_prev_h )
		{
			// Flag a sharp drop between two flat patches (cliff edge or trench lip)
			// more strongly than gradual slope so we downshift before stepping off.
			if ( prev_flat && cur_flat && ( prev_h - hm ) > 2.5 * sm )
			{
				void_cnt += 2;
			}
			else
			{
				double dx_step = fmax( 2.5, dist - prev_d );
				double dh_inter = fabs( hm - prev_h );
				if ( dh_inter <= 0.76 * dx_step )
					inter_slopes[n_inter++] = atan2( dh_inter, dx_step ) * GC_RAD2DEG;
			}
		}
		has_prev_h = 1;
		prev_h = hm;
		prev_d = dist;
		prev_flat = cur_flat;
	}

	double slope = 0.0;
	for ( int i = 0; i < n_inter; ++i )
		if ( inter_slopes[i] > slope ) slope = inter_slopes[i];
	for ( int i = 0; i < n_intra; ++i )
		if ( intra_slopes[i] > slope ) slope = intra_slopes[i];

	double rough = 0.0;
	if ( n_cl_rough > 0 )
	{
		qsort( cl_rough, (size_t)n_cl_rough, sizeof( double ), cmp_double_asc );
		double mid = ( n_cl_rough >= 3 ) ? cl_rough[n_cl_rough / 2] : ( n_cl_rough == 2 ? cl_rough[0] : 0.0 );
		double peek_cap = fmin( 0.055, 0.075 * sm ) + 0.36 * 0.32 * sm;
		rough = fmax( mid, fmin( 0.45 * cl_rough[n_cl_rough - 1], peek_cap ) );
	}

	*out_rough = rough;
	*out_slope = slope;
	*out_void = (double)void_cnt / (double)( n_probes > 0 ? n_probes : 1 );
	*out_l = hz + L;
}

static double ema_step( double cur, double x, double dt, double up_tau, double down_tau )
{
	double tau = ( x > cur ) ? up_tau : down_tau;
	return cur + ( x - cur ) * ( 1.0 - exp( -dt / tau ) );
}

void gc_situation_update( GcSituation* sit, GcMech* m, double dt )
{
	sit->n++;
	double p = m->probe_period;
	sit->probe_t += dt;
	if ( sit->probe_t >= p - 1e-6 )
	{
		sit->probe_t = ( sit->probe_t >= 2.0 * p ) ? 0.0 : fmod( sit->probe_t, p );
		memcpy( sit->probe_prev, sit->probe_raw, sizeof( sit->probe_raw ) );
		probe_ahead( sit, m, &sit->probe_raw[0], &sit->probe_raw[1], &sit->probe_raw[2], &sit->probe_raw[3] );
	}

	double hold = fmin( 0.65, ( sit->probe_raw[3] + m->d->body_half[2] ) / fmax( m->speed, 3.0 ) );
	double* targets[3] = { &sit->ahead_rough, &sit->ahead_slope, &sit->ahead_void };
	for ( int i = 0; i < 3; ++i )
	{
		double cur = *targets[i];
		double seen = fmin( sit->probe_raw[i], sit->probe_prev[i] );
		if ( seen >= cur )
		{
			cur = ( i == 0 ) ? ema_step( cur, seen, dt, 0.06, 0.45 ) : seen;
			sit->hold[i] = hold;
		}
		else
		{
			if ( sit->hold[i] > 0.0 )
			{
				double decay = ( fabs( m->yaw_rate ) > 0.45 || sit->probe_raw[i] < 0.5 * cur ) ? ( 3.0 * dt ) : dt;
				sit->hold[i] -= decay;
			}
			if ( sit->hold[i] <= 0.0 || ( i != 0 && sit->probe_raw[i] > cur ) )
				cur = ema_step( cur, sit->probe_raw[i], dt, 0.3, 0.45 );
		}
		*targets[i] = cur;
	}

	sit->under_rough = ema_step( sit->under_rough, m->foot_rms, dt, 0.45, 0.45 );
	int tried = m->fh_count, bad = m->fh_bad;
	m->fh_count = 0;
	m->fh_bad = 0;
	if ( tried > 0 )
		sit->clutter = ema_step( sit->clutter, (double)bad / (double)tried, dt, 0.25, 0.45 );
	else
		sit->clutter = ema_step( sit->clutter, 0.0, dt, 0.25, 0.70 );

	int any_edge = 0;
	for ( int i = 0; i < m->n; ++i )
	{
		if ( m->legs[i].searching || m->legs[i].void_mark > 0.0 )
		{
			any_edge = 1;
			break;
		}
	}
	sit->edge = ema_step( sit->edge, any_edge ? 1.0 : 0.0, dt, 0.1, 0.45 );

	double stance_slope = 0.0;
	if ( m->has_plane_n_local )
		stance_slope = acos( gc_clamp( m->plane_n_local.y, -1.0, 1.0 ) ) * GC_RAD2DEG;
	sit->stance_slope = ema_step( sit->stance_slope, stance_slope, dt, 0.3, 0.45 );
	sit->slope = fmax( sit->ahead_slope, sit->stance_slope );

	double sm = m->d->step_max;
	double base_r = fmin( 0.055, 0.075 * sm );
	double ahead_n = gc_clamp( ( sit->ahead_rough - base_r ) / ( 0.32 * sm ), 0.0, 1.0 );
	double under_n = gc_clamp( ( 0.85 * sit->under_rough - base_r ) / ( 0.32 * sm ), 0.0, 0.38 );
	double rough_n = fmax( ahead_n, under_n );
	double clutter_n = gc_clamp( ( sit->clutter - 0.30 ) / 0.65, 0.0, 0.38 );
	if ( clutter_n > rough_n )
	{
		sit->harsh = clutter_n;
		snprintf( sit->reason, sizeof( sit->reason ), "cluttered footing" );
	}
	else
	{
		sit->harsh = rough_n;
		snprintf( sit->reason, sizeof( sit->reason ), "rough ground" );
	}
}

void gc_selector_init( GcGaitSelector* sel, const GcMech* m )
{
	memset( sel, 0, sizeof( *sel ) );
	const GcMechDef* d = m->d;
	int fast = d->gait_walk_fast_idx >= 0 ? d->gait_walk_fast_idx
			 : ( d->gait_drift_idx >= 0 ? d->gait_drift_idx : d->gait_walk_idx );
	sel->ladder[0] = d->gait_slow_idx;
	sel->ladder[1] = d->gait_walk_idx;
	sel->ladder[2] = fast;
	sel->ladder[3] = d->gait_run_idx;
	sel->rung = 1;
	sel->dwell = 10.0;
	sel->cap = 3;
	sel->cap_t = 10.0;
	sel->relax_wait = GC_RELAX_T;
	sel->since_relax = 10.0;
}

void gc_selector_caps( const GcGaitSelector* sel, const GcMech* m, double caps[4] )
{
	(void)sel;
	caps[0] = m->v_slow;
	caps[1] = m->v_walk;
	caps[2] = m->d->speed_walk;
	caps[3] = m->d->speed_run;
}

static void selector_cap( const GcSituation* sit, double relax, int* out_cap, char* out_why, size_t why_cap )
{
	double h = sit->harsh + 0.12 * relax;
	int cap_r = 3;
	out_why[0] = '\0';
	if ( sit->harsh + 0.28 * relax > GC_H_WALK )
	{
		cap_r = 0;
		snprintf( out_why, why_cap, "%s", sit->reason );
	}
	else if ( h > GC_H_FAST )
	{
		cap_r = 1;
		snprintf( out_why, why_cap, "%s", sit->reason );
	}
	else if ( h > GC_H_GALLOP )
	{
		cap_r = 2;
		snprintf( out_why, why_cap, "%s", sit->reason );
	}
	if ( sit->slope + 4.0 * relax > GC_SLOPE_WALK && cap_r > 1 )
	{
		cap_r = 1;
		snprintf( out_why, why_cap, "steep slope" );
	}
	else if ( sit->slope + 4.0 * relax > GC_SLOPE_FAST && cap_r > 2 )
	{
		cap_r = 2;
		snprintf( out_why, why_cap, "steep slope" );
	}
	double drop_sig = fmax( sit->edge, sit->ahead_void );
	if ( ( drop_sig + 0.15 * relax > 0.3 ) && cap_r > 2 )
	{
		cap_r = 2;
		snprintf( out_why, why_cap, "%s", ( sit->ahead_void >= sit->edge ) ? "drop ahead" : "footing lost" );
	}
	*out_cap = cap_r;
}

// Pick the highest allowed gait cap from terrain, slope, speed, and ground
// contact. Outputs both whether a speed cap applies and the m/s cap value.
int gc_selector_select( GcGaitSelector* sel, GcMech* m, double dt, int* out_has_cap, double* out_cap )
{
	const GcMechDef* d = m->d;
	sel->dwell += dt;
	const GcSituation* sit = &m->situation;
	double v = m->speed;
	double inst_a = gc_clamp( ( v - sel->prev_v ) / fmax( dt, 1e-4 ), -30.0, 30.0 );
	sel->prev_v = v;
	sel->accel_v += ( inst_a - sel->accel_v ) * ( 1.0 - exp( -dt / 0.15 ) );

	*out_has_cap = 0;
	*out_cap = 0.0;
	if ( m->gait_override >= 0 || m->drifting || gc_mech_climbing_mode( m ) )
		sel->has_hold_v = 0;
	if ( m->gait_override >= 0 )
	{
		snprintf( sel->reason, sizeof( sel->reason ), "forced" );
		return m->gait_override;
	}
	if ( m->drifting )
	{
		snprintf( sel->reason, sizeof( sel->reason ), "slide" );
		return d->gait_drift_idx >= 0 ? d->gait_drift_idx : sel->ladder[1];
	}
	if ( gc_mech_climbing_mode( m ) )
	{
		snprintf( sel->reason, sizeof( sel->reason ), "climbing" );
		return sel->ladder[1];
	}

	double caps[4];
	gc_selector_caps( sel, m, caps );
	int lo = ( m->mode == GC_MODE_SLOW ) ? 0 : 1;
	int hi = ( m->mode == GC_MODE_RUN ) ? 3 : 2;

	double up_thr[3] = { 0.9 * caps[0], 0.9 * caps[1], 0.55 * d->speed_run };
	double down_thr[3] = { 0.75 * caps[0], 0.75 * caps[1], 0.42 * d->speed_run };
	int r = (int)gc_clamp( (double)sel->rung, (double)lo, (double)hi );
	v = m->speed;

	if ( r > lo && v < down_thr[r - 1] )
	{
		if ( r < 3 && m->v_desired >= down_thr[r - 1] && sel->accel_v > 1.0 )
			sel->low_t = fmax( 0.0, sel->low_t - dt );
		else
			sel->low_t += dt;
	}
	else
	{
		sel->low_t = 0.0;
	}
	double low_lim = ( r == 3 && v < 0.40 * d->speed_run ) ? 0.15 : 0.85;
	double v_dn = ( sel->low_t > low_lim ) ? v : fmax( v, 0.9 * m->v_desired );
	while ( r < hi && v > up_thr[r] )
		r++;
	while ( r > lo && v_dn < down_thr[r - 1] )
		r--;

	char reason[32];
	snprintf( reason, sizeof( reason ), "%s",
			  r == 3 ? "galloping" : ( r == 2 ? "fast" : ( r == 1 ? "walking" : "careful" ) ) );
	int turning = fabs( m->inp.turn ) > 0.35;
	if ( m->lateral_ratio > 1.6 && !turning && r > 2 )
	{
		r = 2;
		snprintf( reason, sizeof( reason ), "strafing" );
	}

	int enter_cap, exit_cap;
	char why_in[32], why_ex[32];
	selector_cap( sit, 0.0, &enter_cap, why_in, sizeof( why_in ) );
	selector_cap( sit, 1.0, &exit_cap, why_ex, sizeof( why_ex ) );

	sel->cap_t += dt;
	sel->since_relax += dt;
	sel->better_t = ( exit_cap > sel->cap ) ? ( sel->better_t + dt ) : 0.0;
	if ( sel->since_relax > 2.0 * GC_RELAX_T_MAX )
		sel->relax_wait = GC_RELAX_T;
	double wait = ( sel->cap == 0 && exit_cap >= 2 && sit->harsh < 0.20 ) ? ( 0.60 * sel->relax_wait ) : sel->relax_wait;
	// Downshift immediately for safety; if we already downshifted recently
	// under relax_wait, double the hold-off so rough terrain stops flapping
	// between gaits. Upshifts must survive a dwell timer (`wait`) first.
	if ( enter_cap < sel->cap )
	{
		if ( sel->relaxed && sel->since_relax < 3.0 )
			sel->relax_wait = fmin( GC_RELAX_T_MAX, 2.0 * sel->relax_wait );
		sel->cap = enter_cap;
		snprintf( sel->cap_why, sizeof( sel->cap_why ), "%s", why_in );
		sel->cap_t = 0.0;
		sel->relaxed = 0;
		sel->better_t = 0.0;
	}
	else if ( sel->better_t > wait )
	{
		sel->cap += 1;
		sel->cap_t = 0.0;
		sel->since_relax = 0.0;
		sel->relaxed = 1;
		sel->better_t = 0.0;
		if ( sel->cap >= 3 )
			sel->cap_why[0] = '\0';
	}

	int cap_r = sel->cap;
	const char* why = sel->cap_why;
	int has_speed_cap = 0;
	double speed_cap = 0.0;

	if ( r > sel->rung && r > exit_cap )
		r = sel->rung > exit_cap ? sel->rung : exit_cap;
	if ( r > cap_r )
	{
		if ( sel->rung > cap_r && enter_cap > cap_r )
		{
			if ( sel->rung < r )
				r = sel->rung;
		}
		else
		{
			r = cap_r;
		}
		snprintf( reason, sizeof( reason ), "%s", why );
	}

	int floor_r = 0;
	while ( floor_r < 3 && v > 1.1 * caps[floor_r] )
		floor_r++;
	if ( r < sel->rung && floor_r > r )
		r = sel->rung < floor_r ? sel->rung : floor_r;

	m->calm_gait = ( ( cap_r < 3 || sit->edge > 0.3 || m->crouch > 0.5 ) && r < 3 ) ? 1 : 0;
	if ( m->crouch > 0.5 )
		m->calm_gait = 1;

	if ( cap_r < 3 )
	{
		has_speed_cap = 1;
		if ( cap_r < 2 )
		{
			speed_cap = 0.75 * caps[cap_r];
		}
		else
		{
			double grounded_cap = 0.85 * m->chord_avg / fmax( d->swing_min, 0.10 );
			speed_cap = fmin( caps[2], fmax( 1.35 * caps[1], grounded_cap ) );
			if ( sit->slope > GC_SLOPE_FAST )
			{
				double slope_k = ( sit->harsh < 0.08 ) ? 15.5 : 10.5;
				double slope_cap = ( slope_k * d->step_max ) / fmax( 0.25, tan( sit->slope * GC_DEG2RAD ) );
				speed_cap = fmin( speed_cap, fmax( 1.35 * caps[1], slope_cap ) );
			}
		}
	}

	if ( r != sel->rung )
	{
		double min_dwell = gc_clamp( 0.9 * m->cycle, 0.55, 0.85 );
		int safety = ( ( r < sel->rung && r == cap_r && why[0] != '\0' )
					   || ( sel->rung == 3 && r < 3 && v < 0.40 * d->speed_run ) ) ? 1 : 0;
		if ( sel->dwell < min_dwell && !( safety && sel->dwell > 0.15 ) )
		{
			r = sel->rung;
		}
		else
		{
			if ( r > sel->rung && sel->rung < 2 )
			{
				sel->has_hold_v = 1;
				sel->hold_v = caps[sel->rung];
			}
			else
			{
				sel->has_hold_v = 0;
			}
			sel->rung = r;
			sel->dwell = 0.0;
		}
	}

	snprintf( sel->reason, sizeof( sel->reason ), "%s", reason );
	*out_has_cap = has_speed_cap;
	*out_cap = speed_cap;
	return sel->ladder[r];
}

void gc_blend_init( GcGaitBlend* b, int dst_idx, const GcGait* g )
{
	b->src = *g;
	b->dst_idx = dst_idx;
	b->dst = *g;
	b->t = 1.0;
	b->s = 1.0;
	b->T = 0.5;
	b->eff = *g;
}

// Start a timed transition from the currently effective gait parameters to `dst`.
void gc_blend_start( GcGaitBlend* b, int dst_idx, const GcGait* dst, double T )
{
	b->src = b->eff;
	b->dst_idx = dst_idx;
	b->dst = *dst;
	b->t = 0.0;
	b->T = fmax( T, 0.05 );
}

void gc_blend_update( GcGaitBlend* b, double dt )
{
	b->t = fmin( 1.0, b->t + dt / b->T );
	double s = b->s = gc_smoothstep( b->t );
	GcGait* e = &b->eff;
	const GcGait* a = &b->src;
	const GcGait* dst = &b->dst;

	e->duty = a->duty + ( dst->duty - a->duty ) * s;
	e->stride = a->stride + ( dst->stride - a->stride ) * s;
	e->cycle_min = a->cycle_min + ( dst->cycle_min - a->cycle_min ) * s;
	e->cycle_max = a->cycle_max + ( dst->cycle_max - a->cycle_max ) * s;
	e->lift = a->lift + ( dst->lift - a->lift ) * s;
	e->reach = a->reach + ( dst->reach - a->reach ) * s;
	e->leap = a->leap + ( dst->leap - a->leap ) * s;
	e->stride_pitch = a->stride_pitch + ( dst->stride_pitch - a->stride_pitch ) * s;

	for ( int i = 0; i < e->n_legs; ++i )
		e->offsets[i] = gc_phase_lerp( a->offsets[i], dst->offsets[i], s );

	int done = ( s >= 0.85 ) ? 1 : 0;
	snprintf( e->name, sizeof( e->name ), "%s", done ? dst->name : a->name );
	e->min_support = done ? dst->min_support : ( a->min_support > dst->min_support ? a->min_support : dst->min_support );
	e->static_stable = done ? dst->static_stable : ( a->static_stable || dst->static_stable );
	e->adjacent_lock = done ? dst->adjacent_lock : ( a->adjacent_lock && dst->adjacent_lock );
	if ( done )
	{
		e->group_count = dst->group_count;
		memcpy( e->group_sizes, dst->group_sizes, sizeof( dst->group_sizes ) );
		memcpy( e->groups, dst->groups, sizeof( dst->groups ) );
	}
	else
	{
		e->group_count = 0;
	}
	e->pair_sync = done ? dst->pair_sync : 0;
	e->phase_lock = done ? dst->phase_lock : 0;
	e->has_duty_fast = dst->has_duty_fast;
	e->duty_fast = dst->duty_fast;
}

// Keep min_support tracking actual running duty mid-blend so the scheduler does
// not abort real transitions early; snap to dst->min_support once nearly done.
void gc_blend_fit_support( GcGaitBlend* b, double duty )
{
	if ( b->t >= 1.0 || gc_smoothstep( b->t ) >= 0.85 )
		return;
	int lo = b->src.min_support < b->dst.min_support ? b->src.min_support : b->dst.min_support;
	int hi = b->src.min_support > b->dst.min_support ? b->src.min_support : b->dst.min_support;
	int val = (int)( duty * (double)b->eff.n_legs + 0.05 );
	b->eff.min_support = (int)gc_clamp( (double)val, (double)lo, (double)hi );
}

void gc_blend_label( const GcGaitBlend* b, char* buf, size_t cap )
{
	if ( b->t >= 1.0 )
		snprintf( buf, cap, "%s", b->dst.name );
	else
		snprintf( buf, cap, "%s > %s", b->src.name, b->dst.name );
}
