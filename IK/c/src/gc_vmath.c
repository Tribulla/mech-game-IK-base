// Vector math, body frames, 2D convex hull support margin, and least-squares plane fitting.

#include "gc_internal.h"

const GcV3 GC_WORLD_UP = { 0.0, 1.0, 0.0 };

// GC_LOCAL_FRAME flips right = +X so body-local mounts can use standard -X left / +X right signs,
// whereas world frames compute right = fwd x up.
const GcFrame GC_LOCAL_FRAME = {
	{ 0.0, 0.0, 1.0 },
	{ 0.0, 1.0, 0.0 },
	{ 1.0, 0.0, 0.0 },
	1
};

GcV3 gc_v3_norm( GcV3 v, GcV3 fallback )
{
	double l2 = v.x * v.x + v.y * v.y + v.z * v.z;
	if ( l2 < 1e-18 )
		return fallback;
	double inv = 1.0 / sqrt( l2 );
	return gc_v3( v.x * inv, v.y * inv, v.z * inv );
}

GcV3 gc_project_plane( GcV3 v, GcV3 n )
{
	double d = v.x * n.x + v.y * n.y + v.z * n.z;
	return gc_v3( v.x - n.x * d, v.y - n.y * d, v.z - n.z * d );
}

GcV3 gc_rotate_axis( GcV3 v, GcV3 axis, double angle )
{
	double c = cos( angle );
	double s = sin( angle );
	double d = ( axis.x * v.x + axis.y * v.y + axis.z * v.z ) * ( 1.0 - c );
	double cx = axis.y * v.z - axis.z * v.y;
	double cy = axis.z * v.x - axis.x * v.z;
	double cz = axis.x * v.y - axis.y * v.x;
	return gc_v3( v.x * c + cx * s + axis.x * d,
				  v.y * c + cy * s + axis.y * d,
				  v.z * c + cz * s + axis.z * d );
}

// Integrates constant-speed turning along a circular arc rather than shooting straight along the tangent.
// Over a swing duration this prevents landing feet from drifting outside the curved path.
GcV3 gc_turn_displacement( GcV3 travel, GcV3 up, double angle )
{
	if ( fabs( angle ) < 1e-5 )
		return travel;
	double half = 0.5 * angle;
	return gc_v3_mul( gc_rotate_axis( travel, up, half ), sin( half ) / half );
}

double gc_angle_between( GcV3 a, GcV3 b )
{
	GcV3 na = gc_v3_norm( a, gc_v3_zero() );
	GcV3 nb = gc_v3_norm( b, gc_v3_zero() );
	return acos( gc_clamp( gc_v3_dot( na, nb ), -1.0, 1.0 ) );
}

GcV3 gc_rotate_towards( GcV3 v, GcV3 target, double max_angle )
{
	double ang = acos( gc_clamp( gc_v3_dot( v, target ), -1.0, 1.0 ) );
	if ( ang < 1e-6 )
		return target;
	GcV3 axis = gc_v3_cross( v, target );
	if ( gc_v3_len_sq( axis ) < 1e-12 )
	{
		axis = gc_v3_cross( v, gc_v3( 1.0, 0.0, 0.0 ) );
		if ( gc_v3_len_sq( axis ) < 1e-12 )
			axis = gc_v3_cross( v, gc_v3( 0.0, 0.0, 1.0 ) );
	}
	axis = gc_v3_norm( axis, gc_v3( 1.0, 0.0, 0.0 ) );
	return gc_rotate_axis( v, axis, ang < max_angle ? ang : max_angle );
}

void gc_frame_init( GcFrame* fr, GcV3 fwd, GcV3 up )
{
	fr->fwd = gc_v3( 0.0, 0.0, 1.0 );
	fr->up = gc_v3( 0.0, 1.0, 0.0 );
	fr->right = gc_v3( -1.0, 0.0, 0.0 );
	fr->is_local = 0;
	gc_frame_set( fr, fwd, up );
}

// Re-orthonormalizes the basis around the requested up vector.
// When fwd aligns nearly parallel to up (e.g. vertical wall transitions), fall back to the prior heading
// or flipped up axis so the horizontal facing never snaps or flips by 180 degrees.
void gc_frame_set( GcFrame* fr, GcV3 fwd, GcV3 up )
{
	GcV3 u = gc_v3_norm( up, GC_WORLD_UP );
	GcV3 f = gc_project_plane( fwd, u );
	if ( gc_v3_len_sq( f ) < 1e-10 )
	{
		f = gc_project_plane( fr->fwd, u );
		if ( gc_v3_len_sq( f ) < 1e-10 )
			f = gc_project_plane( gc_v3_mul( fr->up, -1.0 ), u );
	}
	fr->up = u;
	fr->fwd = gc_v3_norm( f, gc_v3( 0.0, 0.0, 1.0 ) );
	fr->right = gc_v3_norm( gc_v3_cross( fr->fwd, fr->up ), gc_v3( -1.0, 0.0, 0.0 ) );
	fr->is_local = 0;
}

GcV3 gc_frame_to_world( const GcFrame* fr, GcV3 l )
{
	if ( fr->is_local )
		return l;
	GcV3 r = fr->right, u = fr->up, f = fr->fwd;
	return gc_v3( r.x * l.x + u.x * l.y + f.x * l.z,
				  r.y * l.x + u.y * l.y + f.y * l.z,
				  r.z * l.x + u.z * l.y + f.z * l.z );
}

GcV3 gc_frame_to_local( const GcFrame* fr, GcV3 w )
{
	if ( fr->is_local )
		return w;
	GcV3 r = fr->right, u = fr->up, f = fr->fwd;
	return gc_v3( w.x * r.x + w.y * r.y + w.z * r.z,
				  w.x * u.x + w.y * u.y + w.z * u.z,
				  w.x * f.x + w.y * f.y + w.z * f.z );
}

void gc_frame_rotate( GcFrame* fr, GcV3 axis, double angle )
{
	if ( fabs( angle ) < 1e-9 )
		return;
	gc_frame_set( fr, gc_rotate_axis( fr->fwd, axis, angle ), gc_rotate_axis( fr->up, axis, angle ) );
}

void gc_frame_yaw( GcFrame* fr, double angle )
{
	double c = cos( angle ), s = sin( angle );
	GcV3 nf = gc_v3_add( gc_v3_mul( fr->fwd, c ), gc_v3_mul( fr->right, s ) );
	gc_frame_set( fr, nf, fr->up );
}

static int cmp_v2( const void* pa, const void* pb )
{
	const GcV2* a = (const GcV2*)pa;
	const GcV2* b = (const GcV2*)pb;
	if ( a->x < b->x ) return -1;
	if ( a->x > b->x ) return 1;
	if ( a->z < b->z ) return -1;
	if ( a->z > b->z ) return 1;
	return 0;
}

static double cross2( GcV2 o, GcV2 a, GcV2 b )
{
	return ( a.x - o.x ) * ( b.z - o.z ) - ( a.z - o.z ) * ( b.x - o.x );
}

// Andrew's monotone chain convex hull in the XZ ground/support plane.
int gc_convex_hull_2d( const GcV2* pts, int n, GcV2* out_hull )
{
	if ( n <= 0 )
		return 0;
	if ( n > GC_MAX_HULL_PTS )
		n = GC_MAX_HULL_PTS;
	GcV2 s[GC_MAX_HULL_PTS];
	memcpy( s, pts, (size_t)n * sizeof( GcV2 ) );
	qsort( s, (size_t)n, sizeof( GcV2 ), cmp_v2 );
	int m = 0;
	for ( int i = 0; i < n; ++i )
	{
		if ( m == 0 || s[i].x != s[m - 1].x || s[i].z != s[m - 1].z )
			s[m++] = s[i];
	}
	if ( m <= 2 )
	{
		for ( int i = 0; i < m; ++i )
			out_hull[i] = s[i];
		return m;
	}
	GcV2 lower[GC_MAX_HULL_PTS], upper[GC_MAX_HULL_PTS];
	int nl = 0, nu = 0;
	for ( int i = 0; i < m; ++i )
	{
		while ( nl >= 2 && cross2( lower[nl - 2], lower[nl - 1], s[i] ) <= 0.0 )
			nl--;
		lower[nl++] = s[i];
	}
	for ( int i = m - 1; i >= 0; --i )
	{
		while ( nu >= 2 && cross2( upper[nu - 2], upper[nu - 1], s[i] ) <= 0.0 )
			nu--;
		upper[nu++] = s[i];
	}
	int total = 0;
	for ( int i = 0; i < nl - 1; ++i )
		out_hull[total++] = lower[i];
	for ( int i = 0; i < nu - 1; ++i )
		out_hull[total++] = upper[i];
	return total;
}

static double dist_point_seg_2d( GcV2 p, GcV2 a, GcV2 b )
{
	double ex = b.x - a.x, ez = b.z - a.z;
	double l2 = ex * ex + ez * ez;
	double t = ( l2 < 1e-12 ) ? 0.0 : gc_clamp( ( ( p.x - a.x ) * ex + ( p.z - a.z ) * ez ) / l2, 0.0, 1.0 );
	return hypot( p.x - ( a.x + ex * t ), p.z - ( a.z + ez * t ) );
}

// Signed distance from point p to the support polygon in the XZ plane.
// Positive means inside with clearance to the nearest edge; negative means outside by that distance.
double gc_support_margin( const GcV2* hull, int n, GcV2 p )
{
	if ( n == 0 )
		return -1e9;
	if ( n == 1 )
		return -hypot( p.x - hull[0].x, p.z - hull[0].z );
	if ( n == 2 )
		return -dist_point_seg_2d( p, hull[0], hull[1] );
	int inside = 1;
	double best = 1e9;
	for ( int i = 0; i < n; ++i )
	{
		GcV2 a = hull[i];
		GcV2 b = hull[( i + 1 ) % n];
		double ex = b.x - a.x, ez = b.z - a.z;
		double el = hypot( ex, ez );
		if ( el == 0.0 )
			el = 1e-9;
		double d = ( ex * ( p.z - a.z ) - ez * ( p.x - a.x ) ) / el;
		if ( d < 0.0 )
			inside = 0;
		if ( d < best )
			best = d;
	}
	if ( inside )
		return best;
	double min_d = 1e18;
	for ( int i = 0; i < n; ++i )
	{
		double ds = dist_point_seg_2d( p, hull[i], hull[( i + 1 ) % n] );
		if ( ds < min_d )
			min_d = ds;
	}
	return -min_d;
}

GcV2 gc_nearest_on_hull( const GcV2* hull, int n, GcV2 p )
{
	if ( n <= 0 )
	{
		GcV2 z = { 0.0, 0.0 };
		return z;
	}
	if ( n == 1 )
		return hull[0];
	GcV2 best = hull[0];
	double bd = 1e18;
	int edges = n > 2 ? n : 1;
	for ( int i = 0; i < edges; ++i )
	{
		GcV2 a = hull[i], b = hull[( i + 1 ) % n];
		double ex = b.x - a.x, ez = b.z - a.z;
		double l2 = ex * ex + ez * ez;
		double t = ( l2 < 1e-12 ) ? 0.0 : gc_clamp( ( ( p.x - a.x ) * ex + ( p.z - a.z ) * ez ) / l2, 0.0, 1.0 );
		GcV2 q = { a.x + ex * t, a.z + ez * t };
		double dd = ( q.x - p.x ) * ( q.x - p.x ) + ( q.z - p.z ) * ( q.z - p.z );
		if ( dd < bd )
		{
			best = q;
			bd = dd;
		}
	}
	return best;
}

double gc_dist_to_segment( GcV3 p, GcV3 a, GcV3 b, GcV3 up )
{
	if ( up.x == 0.0 && up.y == 1.0 && up.z == 0.0 )
	{
		double abx = b.x - a.x, abz = b.z - a.z;
		double apx = p.x - a.x, apz = p.z - a.z;
		double l2 = abx * abx + abz * abz;
		if ( l2 <= 1e-9 )
			return hypot( apx, apz );
		double t = gc_clamp( ( apx * abx + apz * abz ) / l2, 0.0, 1.0 );
		return hypot( apx - abx * t, apz - abz * t );
	}
	double ux = up.x, uy = up.y, uz = up.z;
	double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
	double d = dx * ux + dy * uy + dz * uz;
	double abx = dx - ux * d, aby = dy - uy * d, abz = dz - uz * d;
	double px = p.x - a.x, py = p.y - a.y, pz = p.z - a.z;
	double dp = px * ux + py * uy + pz * uz;
	double apx = px - ux * dp, apy = py - uy * dp, apz = pz - uz * dp;
	double l2 = abx * abx + aby * aby + abz * abz;
	if ( l2 <= 1e-9 )
		return sqrt( apx * apx + apy * apy + apz * apz );
	double t = gc_clamp( ( apx * abx + apy * aby + apz * abz ) / l2, 0.0, 1.0 );
	double rx = apx - abx * t, ry = apy - aby * t, rz = apz - abz * t;
	return sqrt( rx * rx + ry * ry + rz * rz );
}

// Fits plane y = a*x + b*z + c through weighted world points via normal equations.
// Returns 0 if empty, 1 if only a centroid was computed (degenerate or < 3 points), 2 if valid normal.
int gc_fit_plane( const GcV3* points, const double* weights, int n, GcV3* out_c, GcV3* out_n )
{
	if ( n == 0 )
		return 0;
	double wsum = 0.0, cx = 0.0, cy = 0.0, cz = 0.0;
	for ( int i = 0; i < n; ++i )
	{
		double w = weights ? weights[i] : 1.0;
		wsum += w;
		cx += points[i].x * w;
		cy += points[i].y * w;
		cz += points[i].z * w;
	}
	if ( wsum == 0.0 )
		wsum = 1.0;
	cx /= wsum;
	cy /= wsum;
	cz /= wsum;
	*out_c = gc_v3( cx, cy, cz );
	if ( n < 3 )
		return 1; // fewer than 3 points: centroid only, no reliable normal
	double sxx = 0.0, sxz = 0.0, szz = 0.0, sxy = 0.0, szy = 0.0;
	for ( int i = 0; i < n; ++i )
	{
		double w = weights ? weights[i] : 1.0;
		double dx = points[i].x - cx;
		double dy = points[i].y - cy;
		double dz = points[i].z - cz;
		sxx += w * dx * dx;
		sxz += w * dx * dz;
		szz += w * dz * dz;
		sxy += w * dx * dy;
		szy += w * dz * dy;
	}
	double det = sxx * szz - sxz * sxz;
	if ( fabs( det ) < 1e-6 )
		return 1;
	double a = ( sxy * szz - szy * sxz ) / det;
	double b = ( szy * sxx - sxy * sxz ) / det;
	*out_n = gc_v3_norm( gc_v3( -a, 1.0, -b ), GC_WORLD_UP );
	return 2;
}
