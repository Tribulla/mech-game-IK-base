// Planar leg inverse kinematics: radial coxa / sagittal hip-roll framing, 2-bone analytic,
// shared-bend 3-bone fast path, warm-started 2D FABRIK, and active-set DLS joint-limit clamp.

#include "gc_internal.h"

// Analytic 2-bone law of cosines in the 2D leg plane.
// Clamps target distance slightly inside [|l1-l2|, l1+l2] so trig never divides by zero
// or flips a straightened limb backward.
int gc_solve_two_bone( double l1, double l2, double tx, double ty, int side,
					   double* kx_out, double* ky_out, double* fx_out, double* fy_out )
{
	double d = hypot( tx, ty );
	int reached = 1;
	double dmax = l1 + l2 - 1e-5;
	double dmin = fabs( l1 - l2 ) + 1e-5;
	if ( d > dmax )
	{
		reached = ( d <= dmax + 1e-3 ) ? 1 : 0;
		d = dmax;
	}
	else if ( d < dmin )
	{
		d = dmin;
	}
	double base = atan2( ty, tx );
	double cos_a = gc_clamp( ( l1 * l1 + d * d - l2 * l2 ) / ( 2.0 * l1 * d ), -1.0, 1.0 );
	double a = base + (double)side * acos( cos_a );
	double kx = l1 * cos( a );
	double ky = l1 * sin( a );
	double dx = tx - kx, dy = ty - ky;
	double dl = hypot( dx, dy );
	if ( dl == 0.0 )
		dl = 1e-9;
	*kx_out = kx;
	*ky_out = ky;
	*fx_out = kx + dx / dl * l2;
	*fy_out = ky + dy / dl * l2;
	return reached;
}

static void reflect_across_line( double px, double py, double lx, double ly, double* ox, double* oy )
{
	double d = px * lx + py * ly;
	*ox = 2.0 * d * lx - px;
	*oy = 2.0 * d * ly - py;
}

// Warm-started 2D FABRIK solver for general multi-segment chains.
// Interior joints are reflected onto their configured bend side before each forward/backward pass.
int gc_solve_fabrik_2d( const double* lengths, int n, double pts[][2], double tx, double ty,
						const int* sides, int iters, double tol )
{
	double total = 0.0;
	for ( int i = 0; i < n; ++i )
		total += lengths[i];
	double d = hypot( tx, ty );
	if ( d < 1e-6 )
	{
		tx = 1e-6;
		ty = 0.0;
		d = 1e-6;
	}
	double lx = tx / d, ly = ty / d;

	if ( d >= total * 0.999 )
	{
		double acc = 0.0;
		for ( int i = 0; i < n; ++i )
		{
			acc += lengths[i];
			pts[i + 1][0] = lx * acc;
			pts[i + 1][1] = ly * acc;
		}
		return ( d <= total * 1.001 ) ? 1 : 0;
	}

	double tol2 = tol * tol;
	for ( int it = 0; it < iters; ++it )
	{
		for ( int j = 1; j < n; ++j )
		{
			double px = pts[j][0], py = pts[j][1];
			double s = lx * py - ly * px;
			if ( s * (double)sides[j - 1] < 0.0 )
			{
				reflect_across_line( px, py, lx, ly, &pts[j][0], &pts[j][1] );
			}
		}
		pts[n][0] = tx;
		pts[n][1] = ty;
		for ( int i = n - 1; i >= 0; --i )
		{
			double dx = pts[i][0] - pts[i + 1][0];
			double dy = pts[i][1] - pts[i + 1][1];
			double l = hypot( dx, dy );
			if ( l == 0.0 )
				l = 1e-9;
			double k = lengths[i] / l;
			pts[i][0] = pts[i + 1][0] + dx * k;
			pts[i][1] = pts[i + 1][1] + dy * k;
		}
		pts[0][0] = 0.0;
		pts[0][1] = 0.0;
		for ( int i = 0; i < n; ++i )
		{
			double dx = pts[i + 1][0] - pts[i][0];
			double dy = pts[i + 1][1] - pts[i][1];
			double l = hypot( dx, dy );
			if ( l == 0.0 )
				l = 1e-9;
			double k = lengths[i] / l;
			pts[i + 1][0] = pts[i][0] + dx * k;
			pts[i + 1][1] = pts[i][1] + dy * k;
		}
		double ex = pts[n][0] - tx;
		double ey = pts[n][1] - ty;
		if ( ex * ex + ey * ey < tol2 )
			break;
	}
	return 1;
}

void gc_initial_pose_2d( const double* lengths, int n, const int* sides, double tx, double ty, double pts[][2] )
{
	double d = hypot( tx, ty );
	if ( d == 0.0 )
		d = 1e-6;
	double lx = tx / d, ly = ty / d;
	double nx = -ly, ny = lx;
	pts[0][0] = 0.0;
	pts[0][1] = 0.0;
	double x = 0.0, y = 0.0;
	for ( int i = 0; i < n; ++i )
	{
		double L = lengths[i];
		int s = ( i < n - 1 ) ? sides[i] : ( i > 0 ? -sides[i - 1] : 0 );
		double ang = 0.6 * (double)s;
		double ca = cos( ang ), sa = sin( ang );
		double dx = lx * ca + nx * sa;
		double dy = ly * ca + ny * sa;
		x += dx * L;
		y += dy * L;
		pts[i + 1][0] = x;
		pts[i + 1][1] = y;
	}
}

void gc_legik_init( GcLegIK* ik, int plane_mode, double coxa, const double* lengths, int n_segs,
					const int* sides, GcV3 rest_dir_local, const GcLimits* lim )
{
	memset( ik, 0, sizeof( *ik ) );
	ik->mode = plane_mode;
	ik->coxa = coxa;
	ik->n_segs = n_segs;
	ik->chain_reach = 0.0;
	for ( int i = 0; i < n_segs; ++i )
	{
		ik->lengths[i] = lengths[i];
		ik->chain_reach += lengths[i];
	}
	for ( int i = 0; i < n_segs - 1; ++i )
		ik->sides[i] = sides[i];
	ik->reach = coxa + ik->chain_reach;
	ik->rest_dir_local = rest_dir_local;
	ik->reached = 1;
	ik->plane_normal = gc_v3( 1.0, 0.0, 0.0 );
	ik->limit_margin_deg = 90.0;

	ik->coxa_max = ( ( lim && lim->has_coxa ) ? lim->coxa : 180.0 ) * GC_DEG2RAD;
	ik->roll_max = ( ( lim && lim->has_roll ) ? lim->roll : 60.0 ) * GC_DEG2RAD;
	if ( !lim || !lim->has_hip )
	{
		ik->hip_lim[0] = -GC_PI;
		ik->hip_lim[1] = GC_PI;
	}
	else
	{
		double h0 = lim->hip[0];
		double h1 = lim->hip[1];
		if ( plane_mode == GC_PLANE_SAGITTAL )
		{
			// Sagittal hip limits are authored with 0 deg straight down; internal plane angles measure 0 deg forward.
			h0 -= 90.0;
			h1 -= 90.0;
		}
		ik->hip_lim[0] = h0 * GC_DEG2RAD;
		ik->hip_lim[1] = h1 * GC_DEG2RAD;
	}

	int n_int = n_segs - 1;
	for ( int i = 0; i < n_int; ++i )
	{
		double b0 = -180.0, b1 = 180.0;
		if ( lim && lim->bend_count > 0 )
		{
			int bi = i < lim->bend_count ? i : ( lim->bend_count - 1 );
			b0 = lim->bend[bi][0];
			b1 = lim->bend[bi][1];
		}
		ik->bend_lim[i][0] = b0 * GC_DEG2RAD;
		ik->bend_lim[i][1] = b1 * GC_DEG2RAD;
		double s = (double)ik->sides[i];
		double r0 = -s * ik->bend_lim[i][0];
		double r1 = -s * ik->bend_lim[i][1];
		if ( r0 <= r1 )
		{
			ik->rel_lim[i][0] = r0;
			ik->rel_lim[i][1] = r1;
		}
		else
		{
			ik->rel_lim[i][0] = r1;
			ik->rel_lim[i][1] = r0;
		}
	}
}

static void plane_scalars( const GcLegIK* ik, GcV3 mount, const GcFrame* frame, GcV3 target, int clamp_yaw,
						   double* rx_o, double* ry_o, double* rz_o,
						   double* ux_o, double* uy_o, double* uz_o,
						   double* vx_o, double* vy_o, double* vz_o, double* yaw_o )
{
	GcV3 up = frame->up;
	double upx = up.x, upy = up.y, upz = up.z;
	double mx = mount.x, my = mount.y, mz = mount.z;
	double dx = target.x - mx, dy = target.y - my, dz = target.z - mz;

	if ( ik->mode == GC_PLANE_RADIAL )
	{
		double rdx, rdy, rdz;
		GcV3 rd = ik->rest_dir_local;
		if ( frame->is_local )
		{
			rdx = rd.x; rdy = rd.y; rdz = rd.z;
		}
		else
		{
			GcV3 r_ax = frame->right, u_ax = up, f_ax = frame->fwd;
			rdx = r_ax.x * rd.x + u_ax.x * rd.y + f_ax.x * rd.z;
			rdy = r_ax.y * rd.x + u_ax.y * rd.y + f_ax.y * rd.z;
			rdz = r_ax.z * rd.x + u_ax.z * rd.y + f_ax.z * rd.z;
		}
		double du = dx * upx + dy * upy + dz * upz;
		double tx = dx - upx * du, ty = dy - upy * du, tz = dz - upz * du;
		double tl2 = tx * tx + ty * ty + tz * tz;
		double ux, uy, uz;
		if ( tl2 < 1e-18 )
		{
			ux = rdx; uy = rdy; uz = rdz;
		}
		else
		{
			double inv = 1.0 / sqrt( tl2 );
			ux = tx * inv; uy = ty * inv; uz = tz * inv;
		}
		double cx = rdy * uz - rdz * uy;
		double cy = rdz * ux - rdx * uz;
		double cz = rdx * uy - rdy * ux;
		double yaw = atan2( cx * upx + cy * upy + cz * upz, rdx * ux + rdy * uy + rdz * uz );
		if ( clamp_yaw && fabs( yaw ) > ik->coxa_max )
		{
			if ( ik->has_angles )
			{
				// Hold the coxa against the side stop it was already pushing toward when a target wraps
				// behind +/-180 deg, preventing sudden opposite-side snap flips.
				double prev_yaw = ik->yaw_deg * GC_DEG2RAD;
				if ( fabs( prev_yaw ) > 0.5 * ik->coxa_max && yaw * prev_yaw < 0.0 && fabs( yaw ) > GC_PI - 0.85 )
					yaw = copysign( ik->coxa_max, prev_yaw );
				else
					yaw = gc_clamp( yaw, -ik->coxa_max, ik->coxa_max );
			}
			else
			{
				yaw = gc_clamp( yaw, -ik->coxa_max, ik->coxa_max );
			}
			double cy_a = cos( yaw ), sy_a = sin( yaw );
			double crx = upy * rdz - upz * rdy;
			double cry = upz * rdx - upx * rdz;
			double crz = upx * rdy - upy * rdx;
			ux = rdx * cy_a + crx * sy_a;
			uy = rdy * cy_a + cry * sy_a;
			uz = rdz * cy_a + crz * sy_a;
			double ul2 = ux * ux + uy * uy + uz * uz;
			if ( ul2 < 1e-18 )
			{
				ux = rdx; uy = rdy; uz = rdz;
			}
			else
			{
				double inv = 1.0 / sqrt( ul2 );
				ux *= inv; uy *= inv; uz *= inv;
			}
		}
		double coxa = ik->coxa;
		*rx_o = mx + ux * coxa;
		*ry_o = my + uy * coxa;
		*rz_o = mz + uz * coxa;
		*ux_o = ux; *uy_o = uy; *uz_o = uz;
		*vx_o = upx; *vy_o = upy; *vz_o = upz;
		*yaw_o = yaw;
		return;
	}

	GcV3 f = frame->fwd;
	double fx = f.x, fy = f.y, fz = f.z;
	double df = dx * fx + dy * fy + dz * fz;
	double px = dx - fx * df, py = dy - fy * df, pz = dz - fz * df;
	double pl2 = px * px + py * py + pz * pz;
	double vx, vy, vz, roll = 0.0;
	if ( pl2 < 1e-8 )
	{
		vx = upx; vy = upy; vz = upz;
	}
	else
	{
		double inv = -1.0 / sqrt( pl2 );
		vx = px * inv; vy = py * inv; vz = pz * inv;
		double c = cos( ik->roll_max );
		double vdu = vx * upx + vy * upy + vz * upz;
		if ( vdu < c )
		{
			double sdx = vx - upx * vdu, sdy = vy - upy * vdu, sdz = vz - upz * vdu;
			double sdl2 = sdx * sdx + sdy * sdy + sdz * sdz;
			if ( sdl2 < 1e-18 )
			{
				GcV3 fr = frame->right;
				sdx = fr.x; sdy = fr.y; sdz = fr.z;
			}
			else
			{
				double invs = 1.0 / sqrt( sdl2 );
				sdx *= invs; sdy *= invs; sdz *= invs;
			}
			double sr = sin( ik->roll_max );
			vx = upx * c + sdx * sr;
			vy = upy * c + sdy * sr;
			vz = upz * c + sdz * sr;
			double vl2 = vx * vx + vy * vy + vz * vz;
			if ( vl2 < 1e-18 )
			{
				vx = upx; vy = upy; vz = upz;
			}
			else
			{
				double invv = 1.0 / sqrt( vl2 );
				vx *= invv; vy *= invv; vz *= invv;
			}
		}
		roll = acos( gc_clamp( vx * upx + vy * upy + vz * upz, -1.0, 1.0 ) );
	}
	*rx_o = mx; *ry_o = my; *rz_o = mz;
	*ux_o = fx; *uy_o = fy; *uz_o = fz;
	*vx_o = vx; *vy_o = vy; *vz_o = vz;
	*yaw_o = roll;
}

static void to_angles( const double pts2[][2], int n, double* ang )
{
	double px = 0.0, py = 0.0, prev = 0.0;
	for ( int i = 0; i < n; ++i )
	{
		double a = atan2( pts2[i][1] - py, pts2[i][0] - px );
		ang[i] = ( i == 0 ) ? a : gc_wrap_pi( a - prev );
		prev = a;
		px = pts2[i][0];
		py = pts2[i][1];
	}
}

static void fk_pts( const GcLegIK* ik, const double* ang, double pts2[][2] )
{
	double a = 0.0, x = 0.0, y = 0.0;
	for ( int i = 0; i < ik->n_segs; ++i )
	{
		a = ( i == 0 ) ? ang[0] : ( a + ang[i] );
		x += ik->lengths[i] * cos( a );
		y += ik->lengths[i] * sin( a );
		pts2[i][0] = x;
		pts2[i][1] = y;
	}
}

static int is_legal( const GcLegIK* ik, const double* ang, double tol )
{
	if ( !( ik->hip_lim[0] - tol <= ang[0] && ang[0] <= ik->hip_lim[1] + tol ) )
		return 0;
	for ( int i = 0; i < ik->n_segs - 1; ++i )
	{
		if ( !( ik->rel_lim[i][0] - tol <= ang[i + 1] && ang[i + 1] <= ik->rel_lim[i][1] + tol ) )
			return 0;
	}
	return 1;
}

static void clamp_angles( const GcLegIK* ik, const double* ang, double* out )
{
	out[0] = gc_clamp( ang[0], ik->hip_lim[0], ik->hip_lim[1] );
	for ( int i = 0; i < ik->n_segs - 1; ++i )
	{
		out[i + 1] = gc_clamp( ang[i + 1], ik->rel_lim[i][0], ik->rel_lim[i][1] );
	}
}

// Active-set damped least squares solver used when the unconstrained chain hits a joint stop.
// Locks joints that push outward against an active limit and re-solves with remaining DOFs.
static void solve_dls( const GcLegIK* ik, const double* ang_in, double tx, double ty, int iters, double* out_ang )
{
	int n = ik->n_segs;
	double lims[GC_MAX_SEGS][2];
	lims[0][0] = ik->hip_lim[0];
	lims[0][1] = ik->hip_lim[1];
	for ( int i = 0; i < n - 1; ++i )
	{
		lims[i + 1][0] = ik->rel_lim[i][0];
		lims[i + 1][1] = ik->rel_lim[i][1];
	}

	double ang[GC_MAX_SEGS];
	clamp_angles( ik, ang_in, ang );
	double pts[GC_MAX_SEGS][2];
	fk_pts( ik, ang, pts );
	double ex = pts[n - 1][0], ey = pts[n - 1][1];
	double cur_err2 = ( tx - ex ) * ( tx - ex ) + ( ty - ey ) * ( ty - ey );
	double best_ang[GC_MAX_SEGS];
	memcpy( best_ang, ang, (size_t)n * sizeof( double ) );
	double lam = 0.08;

	for ( int it = 0; it < iters; ++it )
	{
		if ( cur_err2 < 1e-7 )
			break;
		double rx = tx - ex, ry = ty - ey;
		double Jx[GC_MAX_SEGS], Jy[GC_MAX_SEGS];
		for ( int k = 0; k < n; ++k )
		{
			Jx[k] = -( ey - ( k == 0 ? 0.0 : pts[k - 1][1] ) );
			Jy[k] = ex - ( k == 0 ? 0.0 : pts[k - 1][0] );
		}
		int active[GC_MAX_SEGS];
		for ( int k = 0; k < n; ++k )
			active[k] = 1;
		double step[GC_MAX_SEGS];
		int have_step = 0;

		for ( int pass = 0; pass < n; ++pass )
		{
			double lam2 = lam * lam;
			double a = lam2, c = lam2, b = 0.0;
			for ( int k = 0; k < n; ++k )
			{
				if ( active[k] )
				{
					a += Jx[k] * Jx[k];
					b += Jx[k] * Jy[k];
					c += Jy[k] * Jy[k];
				}
			}
			double det = a * c - b * b;
			if ( det <= 1e-12 )
			{
				have_step = 0;
				break;
			}
			double vx = ( c * rx - b * ry ) / det;
			double vy = ( a * ry - b * rx ) / det;
			for ( int k = 0; k < n; ++k )
				step[k] = active[k] ? ( Jx[k] * vx + Jy[k] * vy ) : 0.0;
			have_step = 1;
			int pinned = 0, any_act = 0;
			for ( int k = 0; k < n; ++k )
			{
				if ( !active[k] )
					continue;
				double lo = lims[k][0], hi = lims[k][1];
				double sk = step[k], ak = ang[k], nk = ak + sk;
				double clamped = ( nk < lo ) ? lo : ( ( nk > hi ) ? hi : nk );
				double applied = clamped - ak;
				if ( fabs( sk ) > 1e-6 && fabs( applied ) <= 1e-6 )
				{
					active[k] = 0;
					pinned = 1;
				}
				else
				{
					any_act = 1;
				}
			}
			if ( !pinned )
				break;
			if ( !any_act )
			{
				have_step = 0;
				break;
			}
		}
		if ( !have_step )
		{
			lam = fmin( 4.0, lam * 2.5 );
			continue;
		}
		double m = 0.0;
		for ( int k = 0; k < n; ++k )
			if ( fabs( step[k] ) > m )
				m = fabs( step[k] );
		if ( m > 0.55 )
		{
			double sc = 0.55 / m;
			for ( int k = 0; k < n; ++k )
				step[k] *= sc;
		}
		static const double scales[4] = { 1.0, 0.5, 0.25, 0.125 };
		int accepted = 0;
		for ( int si = 0; si < 4; ++si )
		{
			double sc = scales[si];
			double trial[GC_MAX_SEGS];
			double aa = 0.0, tex = 0.0, tey = 0.0;
			for ( int k = 0; k < n; ++k )
			{
				double lo = lims[k][0], hi = lims[k][1];
				double v = ang[k] + step[k] * sc;
				double tk = ( v < lo ) ? lo : ( ( v > hi ) ? hi : v );
				trial[k] = tk;
				aa = ( k == 0 ) ? tk : ( aa + tk );
				double lk = ik->lengths[k];
				tex += lk * cos( aa );
				tey += lk * sin( aa );
			}
			double terr2 = ( tx - tex ) * ( tx - tex ) + ( ty - tey ) * ( ty - tey );
			if ( terr2 < cur_err2 - 1e-10 )
			{
				memcpy( ang, trial, (size_t)n * sizeof( double ) );
				fk_pts( ik, trial, pts );
				cur_err2 = terr2;
				ex = pts[n - 1][0];
				ey = pts[n - 1][1];
				memcpy( best_ang, ang, (size_t)n * sizeof( double ) );
				lam = fmax( 0.01, lam / 2.5 );
				accepted = 1;
				break;
			}
		}
		if ( !accepted )
			lam = fmin( 4.0, lam * 2.5 );
	}
	memcpy( out_ang, best_ang, (size_t)n * sizeof( double ) );
}

// Closed-form 3-segment spider solve: picks the ankle angle via law of cosines so the knee and
// ankle share equal bend curvature, then finishes with a 2-bone solve for hip and knee.
static void three_link_pose( const GcLegIK* ik, double tx, double ty, double pts2[][2] )
{
	double thigh = ik->lengths[0], shin = ik->lengths[1], foot = ik->lengths[2];
	double a = 4.0 * thigh * foot;
	double b = 2.0 * shin * ( thigh + foot );
	double c = ( thigh - foot ) * ( thigh - foot ) + shin * shin - tx * tx - ty * ty;
	double disc = b * b - 4.0 * a * c;
	double cosine = ( -b + sqrt( disc > 0.0 ? disc : 0.0 ) ) / ( 2.0 * a );
	double ankle = -(double)ik->sides[1] * acos( gc_clamp( cosine, -1.0, 1.0 ) );
	ankle = gc_clamp( ankle, ik->rel_lim[1][0], ik->rel_lim[1][1] );

	double dx = shin + foot * cos( ankle );
	double dy = foot * sin( ankle );
	double kx, ky, ex, ey;
	gc_solve_two_bone( thigh, hypot( dx, dy ), tx, ty, ik->sides[0], &kx, &ky, &ex, &ey );
	double hip = atan2( ky, kx );
	double knee = gc_wrap_pi( atan2( ey - ky, ex - kx ) - atan2( dy, dx ) - hip );
	double ang[3] = { hip, knee, ankle };
	fk_pts( ik, ang, pts2 );
}

static int three_link_clamped_candidates( const GcLegIK* ik, double tx, double ty, const double* ang,
										  double out_cands[2][3], double out_err[2] )
{
	double thigh = ik->lengths[0], shin = ik->lengths[1], foot = ik->lengths[2];
	int count = 0;
	if ( !( ik->hip_lim[0] - 1e-4 <= ang[0] && ang[0] <= ik->hip_lim[1] + 1e-4 ) )
	{
		double hip = gc_clamp( ang[0], ik->hip_lim[0], ik->hip_lim[1] );
		double kx = thigh * cos( hip ), ky = thigh * sin( hip );
		double kx2, ky2, fx2, fy2;
		gc_solve_two_bone( shin, foot, tx - kx, ty - ky, ik->sides[1], &kx2, &ky2, &fx2, &fy2 );
		double a1 = atan2( ky2, kx2 );
		double a2 = atan2( fy2 - ky2, fx2 - kx2 );
		out_cands[count][0] = hip;
		out_cands[count][1] = gc_wrap_pi( a1 - hip );
		out_cands[count][2] = gc_wrap_pi( a2 - a1 );
		out_err[count] = hypot( kx + fx2 - tx, ky + fy2 - ty );
		count++;
	}
	if ( !( ik->rel_lim[0][0] - 1e-4 <= ang[1] && ang[1] <= ik->rel_lim[0][1] + 1e-4 ) )
	{
		double knee = gc_clamp( ang[1], ik->rel_lim[0][0], ik->rel_lim[0][1] );
		double dx = thigh + shin * cos( knee ), dy = shin * sin( knee );
		double l_up = hypot( dx, dy );
		if ( l_up > 1e-6 )
		{
			double off = atan2( dy, dx );
			double ax, ay, fx, fy;
			gc_solve_two_bone( l_up, foot, tx, ty, ik->sides[1], &ax, &ay, &fx, &fy );
			double hip = gc_wrap_pi( atan2( ay, ax ) - off );
			double ankle = gc_wrap_pi( atan2( fy - ay, fx - ax ) - ( hip + knee ) );
			out_cands[count][0] = hip;
			out_cands[count][1] = knee;
			out_cands[count][2] = ankle;
			out_err[count] = hypot( fx - tx, fy - ty );
			count++;
		}
	}
	return count;
}

int gc_legik_solve( GcLegIK* ik, GcV3 mount, const GcFrame* frame, GcV3 target, double plane_roll, GcV3* out_joints )
{
	double rx, ry, rz, ux, uy, uz, vx, vy, vz, yaw;
	plane_scalars( ik, mount, frame, target, 1, &rx, &ry, &rz, &ux, &uy, &uz, &vx, &vy, &vz, &yaw );
	double dx = target.x - rx, dy = target.y - ry, dz = target.z - rz;
	if ( ik->mode == GC_PLANE_RADIAL && fabs( plane_roll ) > 1e-5 )
	{
		double dl2 = dx * dx + dy * dy + dz * dz;
		if ( dl2 > 1e-6 )
		{
			double inva = 1.0 / sqrt( dl2 );
			double ax = dx * inva, ay = dy * inva, az = dz * inva;
			double cp = cos( plane_roll ), sp = sin( plane_roll );
			double mc = 1.0 - cp;
			double du = ( ax * ux + ay * uy + az * uz ) * mc;
			double cux = ay * uz - az * uy, cuy = az * ux - ax * uz, cuz = ax * uy - ay * ux;
			double uxn = ux * cp + cux * sp + ax * du;
			double uyn = uy * cp + cuy * sp + ay * du;
			double uzn = uz * cp + cuz * sp + az * du;
			ux = uxn; uy = uyn; uz = uzn;
			double dv = ( ax * vx + ay * vy + az * vz ) * mc;
			double cvx = ay * vz - az * vy, cvy = az * vx - ax * vz, cvz = ax * vy - ay * vx;
			double vxn = vx * cp + cvx * sp + ax * dv;
			double vyn = vy * cp + cvy * sp + ay * dv;
			double vzn = vz * cp + cvz * sp + az * dv;
			vx = vxn; vy = vyn; vz = vzn;
		}
	}

	double tx = dx * ux + dy * uy + dz * uz;
	double ty = dx * vx + dy * vy + dz * vz;
	double nx = uy * vz - uz * vy;
	double ny = uz * vx - ux * vz;
	double nz = ux * vy - uy * vx;
	double nl2 = nx * nx + ny * ny + nz * nz;
	if ( nl2 < 1e-18 )
	{
		nx = frame->right.x;
		ny = frame->right.y;
		nz = frame->right.z;
	}
	else
	{
		double invn = 1.0 / sqrt( nl2 );
		nx *= invn;
		ny *= invn;
		nz *= invn;
	}
	ik->plane_normal = gc_v3( nx, ny, nz );

	int nj = 0;
	out_joints[nj++] = mount;
	if ( ik->mode == GC_PLANE_RADIAL && ik->coxa > 1e-6 )
		out_joints[nj++] = gc_v3( rx, ry, rz );

	int three_same = ( ik->n_segs == 3 && ik->sides[0] == ik->sides[1] && ik->lengths[0] > 0.0 && ik->lengths[2] > 0.0 );
	double pts2[GC_MAX_SEGS][2];
	if ( ik->n_segs == 2 )
	{
		double kx, ky, fx, fy;
		ik->reached = gc_solve_two_bone( ik->lengths[0], ik->lengths[1], tx, ty, ik->sides[0], &kx, &ky, &fx, &fy );
		pts2[0][0] = kx; pts2[0][1] = ky;
		pts2[1][0] = fx; pts2[1][1] = fy;
	}
	else if ( three_same )
	{
		three_link_pose( ik, tx, ty, pts2 );
		ik->reached = 1;
	}
	else
	{
		if ( !ik->has_pts || !ik->reached )
		{
			gc_initial_pose_2d( ik->lengths, ik->n_segs, ik->sides, tx, ty, ik->pts );
			ik->has_pts = 1;
		}
		ik->reached = gc_solve_fabrik_2d( ik->lengths, ik->n_segs, ik->pts, tx, ty, ik->sides, 10, 1e-3 );
		for ( int i = 0; i < ik->n_segs; ++i )
		{
			pts2[i][0] = ik->pts[i + 1][0];
			pts2[i][1] = ik->pts[i + 1][1];
		}
	}

	double ang[GC_MAX_SEGS];
	to_angles( pts2, ik->n_segs, ang );
	ik->limited = !is_legal( ik, ang, 1e-3 );
	if ( ik->limited )
	{
		int solved_exact = 0;
		if ( three_same )
		{
			double cands[2][3];
			double cerr[2];
			int nc = three_link_clamped_candidates( ik, tx, ty, ang, cands, cerr );
			for ( int ci = 0; ci < nc; ++ci )
			{
				if ( cerr[ci] < 2e-3 && is_legal( ik, cands[ci], 1e-3 ) )
				{
					memcpy( ang, cands[ci], 3 * sizeof( double ) );
					fk_pts( ik, ang, pts2 );
					solved_exact = 1;
					break;
				}
			}
		}
		if ( !solved_exact )
		{
			int was_reachable = ik->has_angles && ik->err <= 0.02;
			double start[GC_MAX_SEGS];
			if ( was_reachable )
				memcpy( start, ik->angles, (size_t)ik->n_segs * sizeof( double ) );
			else
				clamp_angles( ik, ang, start );
			double ang_dls[GC_MAX_SEGS], pts_dls[GC_MAX_SEGS][2];
			solve_dls( ik, start, tx, ty, 12, ang_dls );
			fk_pts( ik, ang_dls, pts_dls );
			if ( was_reachable )
			{
				double ex = pts_dls[ik->n_segs - 1][0], ey = pts_dls[ik->n_segs - 1][1];
				if ( ( ex - tx ) * ( ex - tx ) + ( ey - ty ) * ( ey - ty ) > 1e-4 )
				{
					double clamped_start[GC_MAX_SEGS], ang_alt[GC_MAX_SEGS], pts_alt[GC_MAX_SEGS][2];
					clamp_angles( ik, ang, clamped_start );
					solve_dls( ik, clamped_start, tx, ty, 12, ang_alt );
					fk_pts( ik, ang_alt, pts_alt );
					double ax = pts_alt[ik->n_segs - 1][0], ay = pts_alt[ik->n_segs - 1][1];
					if ( ( ax - tx ) * ( ax - tx ) + ( ay - ty ) * ( ay - ty ) < ( ex - tx ) * ( ex - tx ) + ( ey - ty ) * ( ey - ty ) )
					{
						memcpy( ang_dls, ang_alt, (size_t)ik->n_segs * sizeof( double ) );
						memcpy( pts_dls, pts_alt, sizeof( pts_alt ) );
					}
				}
			}
			memcpy( ang, ang_dls, (size_t)ik->n_segs * sizeof( double ) );
			memcpy( pts2, pts_dls, sizeof( pts_dls ) );
		}
		if ( ik->n_segs > 2 )
		{
			ik->pts[0][0] = 0.0;
			ik->pts[0][1] = 0.0;
			for ( int i = 0; i < ik->n_segs; ++i )
			{
				ik->pts[i + 1][0] = pts2[i][0];
				ik->pts[i + 1][1] = pts2[i][1];
			}
			ik->has_pts = 1;
		}
	}

	ik->has_angles = 1;
	memcpy( ik->angles, ang, (size_t)ik->n_segs * sizeof( double ) );

	double fx = pts2[ik->n_segs - 1][0], fy = pts2[ik->n_segs - 1][1];
	double off = dx * nx + dy * ny + dz * nz;
	ik->err = sqrt( ( fx - tx ) * ( fx - tx ) + ( fy - ty ) * ( fy - ty ) + off * off );
	ik->reached = ( ik->reached && ik->err < 0.02 ) ? 1 : 0;

	for ( int i = 0; i < ik->n_segs; ++i )
	{
		double px = pts2[i][0], py = pts2[i][1];
		out_joints[nj++] = gc_v3( rx + ux * px + vx * py, ry + uy * px + vy * py, rz + uz * px + vz * py );
	}
	ik->yaw_deg = yaw * GC_RAD2DEG;
	ik->hip_deg = ang[0] * GC_RAD2DEG;
	for ( int i = 0; i < ik->n_segs - 1; ++i )
		ik->bends_deg[i] = -(double)ik->sides[i] * ang[i + 1] * GC_RAD2DEG;

	if ( ik->mode == GC_PLANE_RADIAL && ik->coxa_max < 3.0 )
		ik->coxa_ratio = fabs( yaw ) / fmax( ik->coxa_max, 1e-6 );
	else if ( ik->mode == GC_PLANE_SAGITTAL && ik->roll_max < 1.5 )
		ik->coxa_ratio = fabs( yaw ) / fmax( ik->roll_max, 1e-6 );
	else
		ik->coxa_ratio = 0.0;

	double m_ang = fmin( ang[0] - ik->hip_lim[0], ik->hip_lim[1] - ang[0] );
	for ( int i = 0; i < ik->n_segs - 1; ++i )
	{
		m_ang = fmin( m_ang, ang[i + 1] - ik->rel_lim[i][0] );
		m_ang = fmin( m_ang, ik->rel_lim[i][1] - ang[i + 1] );
	}
	ik->limit_margin_deg = m_ang * GC_RAD2DEG;
	return nj;
}

// Fast analytical reachability test used to filter candidate footholds without disturbing warm-start state.
int gc_legik_feasible( GcLegIK* ik, GcV3 mount, const GcFrame* frame, GcV3 target, double slack_deg )
{
	double slack = slack_deg * GC_DEG2RAD;
	if ( ik->mode == GC_PLANE_RADIAL )
	{
		GcV3 up = frame->up;
		double upx = up.x, upy = up.y, upz = up.z;
		double dx = target.x - mount.x, dy = target.y - mount.y, dz = target.z - mount.z;
		double du = dx * upx + dy * upy + dz * upz;
		double dtx = dx - upx * du, dty = dy - upy * du, dtz = dz - upz * du;
		double dtl2 = dtx * dtx + dty * dty + dtz * dtz;
		if ( dtl2 > 1e-6 )
		{
			GcV3 rd = ik->rest_dir_local;
			double rdx, rdy, rdz;
			if ( frame->is_local )
			{
				rdx = rd.x; rdy = rd.y; rdz = rd.z;
			}
			else
			{
				GcV3 r_ax = frame->right, f_ax = frame->fwd;
				rdx = r_ax.x * rd.x + upx * rd.y + f_ax.x * rd.z;
				rdy = r_ax.y * rd.x + upy * rd.y + f_ax.y * rd.z;
				rdz = r_ax.z * rd.x + upz * rd.y + f_ax.z * rd.z;
			}
			double inv = 1.0 / sqrt( dtl2 );
			double ux0 = dtx * inv, uy0 = dty * inv, uz0 = dtz * inv;
			double cx = rdy * uz0 - rdz * uy0;
			double cy = rdz * ux0 - rdx * uz0;
			double cz = rdx * uy0 - rdy * ux0;
			double yaw = atan2( cx * upx + cy * upy + cz * upz, rdx * ux0 + rdy * uy0 + rdz * uz0 );
			if ( fabs( yaw ) > ik->coxa_max + slack )
				return 0;
		}
	}

	double rx, ry, rz, ux, uy, uz, vx, vy, vz, unused_yaw;
	plane_scalars( ik, mount, frame, target, 1, &rx, &ry, &rz, &ux, &uy, &uz, &vx, &vy, &vz, &unused_yaw );
	double dx = target.x - rx, dy = target.y - ry, dz = target.z - rz;
	double nx = uy * vz - uz * vy;
	double ny = uz * vx - ux * vz;
	double nz = ux * vy - uy * vx;
	double off = dx * nx + dy * ny + dz * nz;
	double max_off = ( ik->mode == GC_PLANE_SAGITTAL ) ? 0.018 : ( 0.025 + 0.04 * ik->reach * ( slack_deg / 6.0 ) );
	if ( fabs( off ) > max_off )
		return 0;

	double tx = dx * ux + dy * uy + dz * uz;
	double ty = dx * vx + dy * vy + dz * vz;
	double d = hypot( tx, ty );
	double h0 = ik->hip_lim[0], h1 = ik->hip_lim[1];

	if ( ik->n_segs == 3 && ik->sides[0] == ik->sides[1] && ik->lengths[0] > 0.0 && ik->lengths[2] > 0.0 )
	{
		if ( d < 1e-4 || d > ik->chain_reach + 1e-3 )
			return 0;
		double pts2[3][2], ang[3];
		three_link_pose( ik, tx, ty, pts2 );
		to_angles( pts2, 3, ang );
		double tip_err = hypot( pts2[2][0] - tx, pts2[2][1] - ty );
		if ( tip_err <= 0.03 )
		{
			double wh = gc_wrap_pi( ang[0] );
			if ( h0 - slack <= wh && wh <= h1 + slack &&
				 ik->rel_lim[0][0] - slack <= ang[1] && ang[1] <= ik->rel_lim[0][1] + slack &&
				 ik->rel_lim[1][0] - slack <= ang[2] && ang[2] <= ik->rel_lim[1][1] + slack )
				return 1;
		}
		double cands[2][3], cerr[2];
		int nc = three_link_clamped_candidates( ik, tx, ty, ang, cands, cerr );
		for ( int ci = 0; ci < nc; ++ci )
		{
			double wh = gc_wrap_pi( cands[ci][0] );
			if ( cerr[ci] <= 0.03 &&
				 h0 - slack <= wh && wh <= h1 + slack &&
				 ik->rel_lim[0][0] - slack <= cands[ci][1] && cands[ci][1] <= ik->rel_lim[0][1] + slack &&
				 ik->rel_lim[1][0] - slack <= cands[ci][2] && cands[ci][2] <= ik->rel_lim[1][1] + slack )
				return 1;
		}
		return 0;
	}

	double l1 = ik->lengths[0], l2;
	double b_lo = ik->bend_lim[0][0], b_hi = ik->bend_lim[0][1];
	if ( ik->n_segs == 2 )
	{
		l2 = ik->lengths[1];
	}
	else
	{
		double la = ik->lengths[1], lb = ik->lengths[2];
		double bm = 0.5 * ( ik->bend_lim[1][0] + ik->bend_lim[1][1] );
		l2 = sqrt( fmax( 1e-6, la * la + lb * lb + 2.0 * la * lb * cos( bm ) ) );
		slack += 15.0 * GC_DEG2RAD;
	}
	if ( d < 1e-4 || d > l1 + l2 )
		return 0;
	double cb = ( d * d - l1 * l1 - l2 * l2 ) / ( 2.0 * l1 * l2 );
	if ( fabs( cb ) > 1.0 )
		return 0;
	double bend = acos( cb );
	if ( bend < fmax( 0.0, b_lo ) - slack || bend > b_hi + slack )
		return 0;
	double alpha = acos( gc_clamp( ( l1 * l1 + d * d - l2 * l2 ) / ( 2.0 * l1 * d ), -1.0, 1.0 ) );
	double hip = gc_wrap_pi( atan2( ty, tx ) + (double)ik->sides[0] * alpha );
	return ( h0 - slack <= hip && hip <= h1 + slack ) ? 1 : 0;
}
