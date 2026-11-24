// Per-mech world query dispatch (raycasts, sphere pushes) and standalone GcStubWorld geometry.
// Keeping query counters and debug rays per-mech lets worker threads run without shared locks.

#include "gc_internal.h"
#include "gc_thread.h"

void gc_wq_init( GcWorldQueryCtx* q, const GcWorldCallbacks* cb )
{
	memset( q, 0, sizeof( *q ) );
	q->cb = cb;
}

void gc_wq_destroy( GcWorldQueryCtx* q )
{
	free( q->rays );
	memset( q, 0, sizeof( *q ) );
}

void gc_wq_clear_rays( GcWorldQueryCtx* q )
{
	q->ray_count = 0;
	q->push_count = 0;
	q->height_count = 0;
	q->ray_ms = 0.0;
	q->push_ms = 0.0;
	q->height_ms = 0.0;
	q->ray_floats = 0;
}

static void wq_push_ray( GcWorldQueryCtx* q, GcV3 o, GcV3 end, double hit )
{
	if ( q->ray_floats + 7 > q->ray_cap_floats )
	{
		int new_cap = q->ray_cap_floats ? q->ray_cap_floats * 2 : 512 * 7;
		float* np = (float*)realloc( q->rays, (size_t)new_cap * sizeof( float ) );
		if ( !np )
			return;
		q->rays = np;
		q->ray_cap_floats = new_cap;
	}
	float* dst = q->rays + q->ray_floats;
	dst[0] = (float)o.x;
	dst[1] = (float)o.y;
	dst[2] = (float)o.z;
	dst[3] = (float)end.x;
	dst[4] = (float)end.y;
	dst[5] = (float)end.z;
	dst[6] = (float)hit;
	q->ray_floats += 7;
}

int gc_world_raycast( GcWorldQueryCtx* q, GcV3 origin, GcV3 dir, double max_dist, GcHit* out_hit )
{
	q->ray_count += 1;
	int hit = 0;
	GcRayHit rh;
	if ( q->cb && q->cb->raycast )
	{
		double t0 = q->profile_queries ? gc_time_sec() : 0.0;
		hit = q->cb->raycast( q->cb->user, origin.x, origin.y, origin.z,
							  dir.x, dir.y, dir.z, max_dist, &rh );
		if ( q->profile_queries )
			q->ray_ms += ( gc_time_sec() - t0 ) * 1000.0;
	}
	if ( hit )
	{
		out_hit->t = rh.t;
		out_hit->point = gc_v3( rh.px, rh.py, rh.pz );
		out_hit->normal = gc_v3( rh.nx, rh.ny, rh.nz );
		out_hit->surface = rh.surface;
	}
	if ( q->record )
	{
		GcV3 end = hit ? out_hit->point : gc_v3_add( origin, gc_v3_mul( dir, max_dist ) );
		wq_push_ray( q, origin, end, hit ? 1.0 : 0.0 );
	}
	return hit;
}

int gc_world_sphere_push( GcWorldQueryCtx* q, GcV3 center, double radius, GcV3* out_center, GcV3* out_normal )
{
	q->push_count += 1;
	double xyz[3] = { center.x, center.y, center.z };
	double nxyz[3] = { 0.0, 1.0, 0.0 };
	int hit = 0;
	if ( q->cb && q->cb->sphere_push )
	{
		double t0 = q->profile_queries ? gc_time_sec() : 0.0;
		hit = q->cb->sphere_push( q->cb->user, center.x, center.y, center.z, radius, xyz, nxyz );
		if ( q->profile_queries )
			q->push_ms += ( gc_time_sec() - t0 ) * 1000.0;
	}
	*out_center = gc_v3( xyz[0], xyz[1], xyz[2] );
	if ( hit && out_normal )
		*out_normal = gc_v3( nxyz[0], nxyz[1], nxyz[2] );
	return hit;
}

// Standalone read-only test/benchmark world supporting flat ground, AABBs, and rotated OBB ramps.
// Thread-safe for concurrent queries once setup finishes.

typedef struct GcStubBox {
	GcV3 mn;
	GcV3 mx;
} GcStubBox;

typedef struct GcStubOBox {
	GcV3 c;
	double h[3];
	GcV3 ax[3];
	GcV3 bmin;
	GcV3 bmax;
} GcStubOBox;

struct GcStubWorld {
	double ground_y;
	int has_ground;
	GcStubBox* boxes;
	int n_boxes;
	int cap_boxes;
	GcStubOBox* oboxes;
	int n_oboxes;
	int cap_oboxes;
};

GcStubWorld* gc_stub_world_create( double ground_y, int has_ground )
{
	GcStubWorld* w = (GcStubWorld*)calloc( 1, sizeof( GcStubWorld ) );
	if ( !w )
		return NULL;
	w->ground_y = ground_y;
	w->has_ground = has_ground ? 1 : 0;
	return w;
}

void gc_stub_world_destroy( GcStubWorld* w )
{
	if ( !w )
		return;
	free( w->boxes );
	free( w->oboxes );
	free( w );
}

void gc_stub_world_add_box( GcStubWorld* w, double x0, double y0, double z0, double x1, double y1, double z1 )
{
	if ( !w )
		return;
	if ( w->n_boxes == w->cap_boxes )
	{
		int nc = w->cap_boxes ? w->cap_boxes * 2 : 32;
		GcStubBox* nb = (GcStubBox*)realloc( w->boxes, (size_t)nc * sizeof( GcStubBox ) );
		if ( !nb )
			return;
		w->boxes = nb;
		w->cap_boxes = nc;
	}
	GcStubBox b;
	b.mn = gc_v3( x0 < x1 ? x0 : x1, y0 < y1 ? y0 : y1, z0 < z1 ? z0 : z1 );
	b.mx = gc_v3( x0 > x1 ? x0 : x1, y0 > y1 ? y0 : y1, z0 > z1 ? z0 : z1 );
	w->boxes[w->n_boxes++] = b;
}

void gc_stub_world_add_obox( GcStubWorld* w, double cx, double cy, double cz,
							 double hx, double hy, double hz, double yaw, double pitch )
{
	if ( !w )
		return;
	if ( w->n_oboxes == w->cap_oboxes )
	{
		int nc = w->cap_oboxes ? w->cap_oboxes * 2 : 32;
		GcStubOBox* nb = (GcStubOBox*)realloc( w->oboxes, (size_t)nc * sizeof( GcStubOBox ) );
		if ( !nb )
			return;
		w->oboxes = nb;
		w->cap_oboxes = nc;
	}
	double cy_v = cos( yaw ), sy_v = sin( yaw ), cp = cos( pitch ), sp = sin( pitch );
	GcV3 X = gc_v3( cy_v, 0.0, -sy_v );
	GcV3 Y = gc_v3( 0.0, 1.0, 0.0 );
	GcV3 Z = gc_v3( sy_v, 0.0, cy_v );
	GcStubOBox ob;
	ob.c = gc_v3( cx, cy, cz );
	ob.h[0] = hx;
	ob.h[1] = hy;
	ob.h[2] = hz;
	ob.ax[0] = X;
	ob.ax[1] = gc_v3_add( gc_v3_mul( Y, cp ), gc_v3_mul( Z, sp ) );
	ob.ax[2] = gc_v3_sub( gc_v3_mul( Z, cp ), gc_v3_mul( Y, sp ) );
	double ex = fabs( ob.ax[0].x ) * hx + fabs( ob.ax[1].x ) * hy + fabs( ob.ax[2].x ) * hz;
	double ey = fabs( ob.ax[0].y ) * hx + fabs( ob.ax[1].y ) * hy + fabs( ob.ax[2].y ) * hz;
	double ez = fabs( ob.ax[0].z ) * hx + fabs( ob.ax[1].z ) * hy + fabs( ob.ax[2].z ) * hz;
	ob.bmin = gc_v3( cx - ex, cy - ey, cz - ez );
	ob.bmax = gc_v3( cx + ex, cy + ey, cz + ez );
	w->oboxes[w->n_oboxes++] = ob;
}

static int stub_raycast( void* user, double ox, double oy, double oz,
						 double dx, double dy, double dz, double max_dist, GcRayHit* out_hit )
{
	const GcStubWorld* w = (const GcStubWorld*)user;
	int has_hit = 0;
	double best_t = max_dist;
	GcV3 best_p = gc_v3_zero(), best_n = gc_v3( 0.0, 1.0, 0.0 );
	int best_sid = 0;

	if ( w->has_ground && dy < -1e-9 && oy > w->ground_y )
	{
		double t = ( w->ground_y - oy ) / dy;
		if ( t <= best_t )
		{
			has_hit = 1;
			best_t = t;
			best_p = gc_v3( ox + dx * t, w->ground_y, oz + dz * t );
			best_n = gc_v3( 0.0, 1.0, 0.0 );
			best_sid = 0;
		}
	}

	for ( int i = 0; i < w->n_boxes; ++i )
	{
		GcV3 mn = w->boxes[i].mn, mx = w->boxes[i].mx;
		double tmin = -1e30, tmax = 1e30, sgn = 0.0;
		int axis = -1;
		double o_arr[3] = { ox, oy, oz };
		double d_arr[3] = { dx, dy, dz };
		double mn_arr[3] = { mn.x, mn.y, mn.z };
		double mx_arr[3] = { mx.x, mx.y, mx.z };
		int ok = 1;
		for ( int a = 0; a < 3; ++a )
		{
			if ( fabs( d_arr[a] ) < 1e-12 )
			{
				if ( o_arr[a] < mn_arr[a] || o_arr[a] > mx_arr[a] )
				{
					ok = 0;
					break;
				}
			}
			else
			{
				double inv = 1.0 / d_arr[a];
				double t1 = ( mn_arr[a] - o_arr[a] ) * inv;
				double t2 = ( mx_arr[a] - o_arr[a] ) * inv;
				double s = -1.0;
				if ( t1 > t2 )
				{
					double tmp = t1;
					t1 = t2;
					t2 = tmp;
					s = 1.0;
				}
				if ( t1 > tmin )
				{
					tmin = t1;
					axis = a;
					sgn = s;
				}
				if ( t2 < tmax )
					tmax = t2;
				if ( tmin > tmax )
				{
					ok = 0;
					break;
				}
			}
		}
		if ( !ok || axis < 0 || tmin < 0.0 || tmin > max_dist )
			continue;
		if ( !has_hit || tmin < best_t )
		{
			has_hit = 1;
			best_t = tmin;
			best_p = gc_v3( ox + dx * tmin, oy + dy * tmin, oz + dz * tmin );
			best_n = gc_v3( axis == 0 ? sgn : 0.0, axis == 1 ? sgn : 0.0, axis == 2 ? sgn : 0.0 );
			best_sid = i + 1;
		}
	}

	double rx0 = ox, rx1 = ox + dx * ( max_dist < 1e4 ? max_dist : 1e4 );
	double rz0 = oz, rz1 = oz + dz * ( max_dist < 1e4 ? max_dist : 1e4 );
	if ( rx0 > rx1 ) { double tmp = rx0; rx0 = rx1; rx1 = tmp; }
	if ( rz0 > rz1 ) { double tmp = rz0; rz0 = rz1; rz1 = tmp; }

	for ( int i = 0; i < w->n_oboxes; ++i )
	{
		const GcStubOBox* ob = &w->oboxes[i];
		if ( ob->bmax.x < rx0 || ob->bmin.x > rx1 || ob->bmax.z < rz0 || ob->bmin.z > rz1 )
			continue;
		double rx = ox - ob->c.x, ry = oy - ob->c.y, rz = oz - ob->c.z;
		double tmin = -1e30, tmax = 1e30, sgn = 0.0;
		int axis = -1, ok = 1;
		for ( int a = 0; a < 3; ++a )
		{
			GcV3 ax = ob->ax[a];
			double oi = rx * ax.x + ry * ax.y + rz * ax.z;
			double di = dx * ax.x + dy * ax.y + dz * ax.z;
			double hi = ob->h[a];
			if ( fabs( di ) < 1e-12 )
			{
				if ( oi < -hi || oi > hi )
				{
					ok = 0;
					break;
				}
				continue;
			}
			double inv = 1.0 / di;
			double t1 = ( -hi - oi ) * inv;
			double t2 = ( hi - oi ) * inv;
			double s = -1.0;
			if ( t1 > t2 )
			{
				double tmp = t1;
				t1 = t2;
				t2 = tmp;
				s = 1.0;
			}
			if ( t1 > tmin )
			{
				tmin = t1;
				axis = a;
				sgn = s;
			}
			if ( t2 < tmax )
				tmax = t2;
			if ( tmin > tmax )
			{
				ok = 0;
				break;
			}
		}
		if ( !ok || axis < 0 || tmin < 0.0 || tmin > max_dist )
			continue;
		if ( !has_hit || tmin < best_t )
		{
			has_hit = 1;
			best_t = tmin;
			best_p = gc_v3( ox + dx * tmin, oy + dy * tmin, oz + dz * tmin );
			best_n = gc_v3_mul( ob->ax[axis], sgn );
			best_sid = w->n_boxes + i + 1;
		}
	}

	if ( !has_hit )
		return 0;
	out_hit->t = best_t;
	out_hit->px = best_p.x;
	out_hit->py = best_p.y;
	out_hit->pz = best_p.z;
	out_hit->nx = best_n.x;
	out_hit->ny = best_n.y;
	out_hit->nz = best_n.z;
	out_hit->surface = best_sid;
	return 1;
}

static int stub_sphere_push( void* user, double cx, double cy, double cz, double r,
							 double* out_xyz, double* out_nxyz )
{
	const GcStubWorld* w = (const GcStubWorld*)user;
	int hit = 0;
	GcV3 normal = gc_v3( 0.0, 1.0, 0.0 );
	if ( w->has_ground && cy - r < w->ground_y )
	{
		cy = w->ground_y + r;
		normal = gc_v3( 0.0, 1.0, 0.0 );
		hit = 1;
	}
	double r2 = r * r;
	for ( int i = 0; i < w->n_boxes; ++i )
	{
		GcV3 mn = w->boxes[i].mn, mx = w->boxes[i].mx;
		double px = cx < mn.x ? mn.x : ( cx > mx.x ? mx.x : cx );
		double py = cy < mn.y ? mn.y : ( cy > mx.y ? mx.y : cy );
		double pz = cz < mn.z ? mn.z : ( cz > mx.z ? mx.z : cz );
		double dx = cx - px, dy = cy - py, dz = cz - pz;
		double l2 = dx * dx + dy * dy + dz * dz;
		if ( l2 < 1e-18 )
		{
			// Sphere center sits inside the AABB: push out across the shallowest face.
			double pens[6] = { cx - mn.x, mx.x - cx, cy - mn.y, mx.y - cy, cz - mn.z, mx.z - cz };
			static const double nxv[6] = { -1.0, 1.0, 0.0, 0.0, 0.0, 0.0 };
			static const double nyv[6] = { 0.0, 0.0, -1.0, 1.0, 0.0, 0.0 };
			static const double nzv[6] = { 0.0, 0.0, 0.0, 0.0, -1.0, 1.0 };
			int best_k = 0;
			for ( int k = 1; k < 6; ++k )
			{
				if ( pens[k] < pens[best_k] )
					best_k = k;
			}
			double kk = pens[best_k] + r;
			cx += nxv[best_k] * kk;
			cy += nyv[best_k] * kk;
			cz += nzv[best_k] * kk;
			normal = gc_v3( nxv[best_k], nyv[best_k], nzv[best_k] );
			hit = 1;
		}
		else if ( l2 < r2 )
		{
			double l = sqrt( l2 );
			double inv = 1.0 / l;
			double nx = dx * inv, ny = dy * inv, nz = dz * inv;
			double kk = r - l;
			cx += nx * kk;
			cy += ny * kk;
			cz += nz * kk;
			normal = gc_v3( nx, ny, nz );
			hit = 1;
		}
	}
	for ( int i = 0; i < w->n_oboxes; ++i )
	{
		const GcStubOBox* ob = &w->oboxes[i];
		if ( cx + r < ob->bmin.x || cx - r > ob->bmax.x ||
			 cy + r < ob->bmin.y || cy - r > ob->bmax.y ||
			 cz + r < ob->bmin.z || cz - r > ob->bmax.z )
			continue;
		double rx = cx - ob->c.x, ry = cy - ob->c.y, rz = cz - ob->c.z;
		GcV3 ax0 = ob->ax[0], ax1 = ob->ax[1], ax2 = ob->ax[2];
		double l0 = rx * ax0.x + ry * ax0.y + rz * ax0.z;
		double l1 = rx * ax1.x + ry * ax1.y + rz * ax1.z;
		double l2 = rx * ax2.x + ry * ax2.y + rz * ax2.z;
		double h0 = ob->h[0], h1 = ob->h[1], h2 = ob->h[2];
		double q0 = l0 < -h0 ? -h0 : ( l0 > h0 ? h0 : l0 );
		double q1 = l1 < -h1 ? -h1 : ( l1 > h1 ? h1 : l1 );
		double q2 = l2 < -h2 ? -h2 : ( l2 > h2 ? h2 : l2 );
		double d0 = l0 - q0, d1 = l1 - q1, d2 = l2 - q2;
		double dl2 = d0 * d0 + d1 * d1 + d2 * d2;
		if ( dl2 < 1e-18 )
		{
			// Center lies inside the OBB: resolve along the minimum-penetration local axis.
			double pens[6] = { l0 + h0, h0 - l0, l1 + h1, h1 - l1, l2 + h2, h2 - l2 };
			int best_k = 0;
			for ( int k = 1; k < 6; ++k )
			{
				if ( pens[k] < pens[best_k] )
					best_k = k;
			}
			GcV3 nax = ob->ax[best_k / 2];
			double ns = ( best_k % 2 == 0 ) ? -1.0 : 1.0;
			normal = gc_v3_mul( nax, ns );
			double kk = pens[best_k] + r;
			cx += normal.x * kk;
			cy += normal.y * kk;
			cz += normal.z * kk;
			hit = 1;
		}
		else if ( dl2 < r2 )
		{
			double dl = sqrt( dl2 );
			double inv = 1.0 / dl;
			double nx = ( ax0.x * d0 + ax1.x * d1 + ax2.x * d2 ) * inv;
			double ny = ( ax0.y * d0 + ax1.y * d1 + ax2.y * d2 ) * inv;
			double nz = ( ax0.z * d0 + ax1.z * d1 + ax2.z * d2 ) * inv;
			normal = gc_v3( nx, ny, nz );
			double kk = r - dl;
			cx += nx * kk;
			cy += ny * kk;
			cz += nz * kk;
			hit = 1;
		}
	}
	out_xyz[0] = cx;
	out_xyz[1] = cy;
	out_xyz[2] = cz;
	out_nxyz[0] = normal.x;
	out_nxyz[1] = normal.y;
	out_nxyz[2] = normal.z;
	return hit;
}

static double stub_terrain_height( void* user, double x, double z )
{
	(void)x;
	(void)z;
	const GcStubWorld* w = (const GcStubWorld*)user;
	return w->has_ground ? w->ground_y : 0.0;
}

GcWorldCallbacks gc_stub_world_callbacks( const GcStubWorld* w )
{
	GcWorldCallbacks cb;
	cb.raycast = stub_raycast;
	cb.sphere_push = stub_sphere_push;
	cb.terrain_height = stub_terrain_height;
	cb.user = (void*)w;
	return cb;
}
