package gaitcore;

import java.util.Arrays;

/**
 * Immutable 3D vector, orthonormal body frame, 2D convex hull margin, and least-squares plane fit.
 * Mirrors {@code gc_vmath.c} value semantics so callers never hit accidental aliasing bugs.
 */
public final class Vec3 {

    public static final Vec3 ZERO = new Vec3(0.0, 0.0, 0.0);
    public static final Vec3 WORLD_UP = new Vec3(0.0, 1.0, 0.0);

    /** Local frame flips right = +X so body-local mounts use standard -X left / +X right signs. */
    public static final Frame LOCAL_FRAME = new Frame(
            new Vec3(0.0, 0.0, 1.0),
            new Vec3(0.0, 1.0, 0.0),
            new Vec3(1.0, 0.0, 0.0),
            true);

    public final double x;
    public final double y;
    public final double z;

    public Vec3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public static Vec3 of(double x, double y, double z) {
        return new Vec3(x, y, z);
    }

    public Vec3 add(Vec3 b) {
        return new Vec3(x + b.x, y + b.y, z + b.z);
    }

    public Vec3 sub(Vec3 b) {
        return new Vec3(x - b.x, y - b.y, z - b.z);
    }

    public Vec3 mul(double k) {
        return new Vec3(x * k, y * k, z * k);
    }

    public Vec3 neg() {
        return new Vec3(-x, -y, -z);
    }

    public double dot(Vec3 b) {
        return x * b.x + y * b.y + z * b.z;
    }

    public Vec3 cross(Vec3 b) {
        return new Vec3(
                y * b.z - z * b.y,
                z * b.x - x * b.z,
                x * b.y - y * b.x);
    }

    public double lenSq() {
        return x * x + y * y + z * z;
    }

    public double len() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public double dist(Vec3 b) {
        double dx = x - b.x;
        double dy = y - b.y;
        double dz = z - b.z;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    public Vec3 lerp(Vec3 b, double t) {
        return new Vec3(x + (b.x - x) * t, y + (b.y - y) * t, z + (b.z - z) * t);
    }

    public Vec3 norm(Vec3 fallback) {
        double l2 = x * x + y * y + z * z;
        if (l2 < 1e-18) {
            return fallback;
        }
        double inv = 1.0 / Math.sqrt(l2);
        return new Vec3(x * inv, y * inv, z * inv);
    }

    public Vec3 projectPlane(Vec3 n) {
        double d = x * n.x + y * n.y + z * n.z;
        return new Vec3(x - n.x * d, y - n.y * d, z - n.z * d);
    }

    public Vec3 rotateAxis(Vec3 axis, double angle) {
        double c = Math.cos(angle);
        double s = Math.sin(angle);
        double d = (axis.x * x + axis.y * y + axis.z * z) * (1.0 - c);
        double cx = axis.y * z - axis.z * y;
        double cy = axis.z * x - axis.x * z;
        double cz = axis.x * y - axis.y * x;
        return new Vec3(
                x * c + cx * s + axis.x * d,
                y * c + cy * s + axis.y * d,
                z * c + cz * s + axis.z * d);
    }

    /**
     * Integrates constant-speed turning along a circular arc rather than shooting straight along the
     * tangent so landing feet do not drift outside a curved path during swings.
     */
    public static Vec3 turnDisplacement(Vec3 travel, Vec3 up, double angle) {
        if (Math.abs(angle) < 1e-5) {
            return travel;
        }
        double half = 0.5 * angle;
        return travel.rotateAxis(up, half).mul(Math.sin(half) / half);
    }

    public static double angleBetween(Vec3 a, Vec3 b) {
        Vec3 na = a.norm(ZERO);
        Vec3 nb = b.norm(ZERO);
        return Math.acos(GaitCore.clamp(na.dot(nb), -1.0, 1.0));
    }

    public static Vec3 rotateTowards(Vec3 v, Vec3 target, double maxAngle) {
        double ang = Math.acos(GaitCore.clamp(v.dot(target), -1.0, 1.0));
        if (ang < 1e-6) {
            return target;
        }
        Vec3 axis = v.cross(target);
        if (axis.lenSq() < 1e-12) {
            axis = v.cross(new Vec3(1.0, 0.0, 0.0));
            if (axis.lenSq() < 1e-12) {
                axis = v.cross(new Vec3(0.0, 0.0, 1.0));
            }
        }
        axis = axis.norm(new Vec3(1.0, 0.0, 0.0));
        return v.rotateAxis(axis, ang < maxAngle ? ang : maxAngle);
    }

    public static double distToSegment(Vec3 p, Vec3 a, Vec3 b, Vec3 up) {
        if (up.x == 0.0 && up.y == 1.0 && up.z == 0.0) {
            double abx = b.x - a.x;
            double abz = b.z - a.z;
            double apx = p.x - a.x;
            double apz = p.z - a.z;
            double l2 = abx * abx + abz * abz;
            if (l2 <= 1e-9) {
                return Math.hypot(apx, apz);
            }
            double t = GaitCore.clamp((apx * abx + apz * abz) / l2, 0.0, 1.0);
            return Math.hypot(apx - abx * t, apz - abz * t);
        }
        double ux = up.x;
        double uy = up.y;
        double uz = up.z;
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double dz = b.z - a.z;
        double d = dx * ux + dy * uy + dz * uz;
        double abx = dx - ux * d;
        double aby = dy - uy * d;
        double abz = dz - uz * d;
        double px = p.x - a.x;
        double py = p.y - a.y;
        double pz = p.z - a.z;
        double dp = px * ux + py * uy + pz * uz;
        double apx = px - ux * dp;
        double apy = py - uy * dp;
        double apz = pz - uz * dp;
        double l2 = abx * abx + aby * aby + abz * abz;
        if (l2 <= 1e-9) {
            return Math.sqrt(apx * apx + apy * apy + apz * apz);
        }
        double t = GaitCore.clamp((apx * abx + apy * aby + apz * abz) / l2, 0.0, 1.0);
        double rx = apx - abx * t;
        double ry = apy - aby * t;
        double rz = apz - abz * t;
        return Math.sqrt(rx * rx + ry * ry + rz * rz);
    }

    /** 2D point in the XZ support plane. */
    public static final class Vec2 {
        public final double x;
        public final double z;

        public Vec2(double x, double z) {
            this.x = x;
            this.z = z;
        }
    }

    /** Output holder for {@link #fitPlane}. */
    public static final class PlaneFit {
        public Vec3 c = ZERO;
        public Vec3 n = WORLD_UP;
    }

    /**
     * Orthonormal body basis.
     * Because {@code right = fwd x up}, at yaw 0 {@code right = (-1, 0, 0)} in world space.
     */
    public static final class Frame {
        public Vec3 fwd;
        public Vec3 up;
        public Vec3 right;
        public boolean isLocal;

        public Frame() {
            this.fwd = new Vec3(0.0, 0.0, 1.0);
            this.up = new Vec3(0.0, 1.0, 0.0);
            this.right = new Vec3(-1.0, 0.0, 0.0);
            this.isLocal = false;
        }

        private Frame(Vec3 fwd, Vec3 up, Vec3 right, boolean isLocal) {
            this.fwd = fwd;
            this.up = up;
            this.right = right;
            this.isLocal = isLocal;
        }

        public Frame(Vec3 fwd, Vec3 up) {
            this();
            init(fwd, up);
        }

        public void init(Vec3 fwd, Vec3 up) {
            this.fwd = new Vec3(0.0, 0.0, 1.0);
            this.up = new Vec3(0.0, 1.0, 0.0);
            this.right = new Vec3(-1.0, 0.0, 0.0);
            this.isLocal = false;
            set(fwd, up);
        }

        /**
         * Re-orthonormalizes around the requested up axis. Falls back to prior forward or flipped up
         * when {@code fwd} is nearly parallel to {@code up} (e.g. vertical wall transitions).
         */
        public void set(Vec3 fwd, Vec3 up) {
            Vec3 u = up.norm(WORLD_UP);
            Vec3 f = fwd.projectPlane(u);
            if (f.lenSq() < 1e-10) {
                f = this.fwd.projectPlane(u);
                if (f.lenSq() < 1e-10) {
                    f = this.up.mul(-1.0).projectPlane(u);
                }
            }
            this.up = u;
            this.fwd = f.norm(new Vec3(0.0, 0.0, 1.0));
            this.right = this.fwd.cross(this.up).norm(new Vec3(-1.0, 0.0, 0.0));
            this.isLocal = false;
        }

        public Frame copy() {
            return new Frame(fwd, up, right, isLocal);
        }

        public void copyFrom(Frame o) {
            this.fwd = o.fwd;
            this.up = o.up;
            this.right = o.right;
            this.isLocal = o.isLocal;
        }

        public Vec3 toWorld(Vec3 l) {
            if (isLocal) {
                return l;
            }
            return new Vec3(
                    right.x * l.x + up.x * l.y + fwd.x * l.z,
                    right.y * l.x + up.y * l.y + fwd.y * l.z,
                    right.z * l.x + up.z * l.y + fwd.z * l.z);
        }

        public Vec3 toLocal(Vec3 w) {
            if (isLocal) {
                return w;
            }
            return new Vec3(
                    w.x * right.x + w.y * right.y + w.z * right.z,
                    w.x * up.x + w.y * up.y + w.z * up.z,
                    w.x * fwd.x + w.y * fwd.y + w.z * fwd.z);
        }

        public void rotate(Vec3 axis, double angle) {
            if (Math.abs(angle) < 1e-9) {
                return;
            }
            set(fwd.rotateAxis(axis, angle), up.rotateAxis(axis, angle));
        }

        public void yaw(double angle) {
            double c = Math.cos(angle);
            double s = Math.sin(angle);
            Vec3 nf = fwd.mul(c).add(right.mul(s));
            set(nf, up);
        }
    }

    private static double cross2(Vec2 o, Vec2 a, Vec2 b) {
        return (a.x - o.x) * (b.z - o.z) - (a.z - o.z) * (b.x - o.x);
    }

    /** Andrew's monotone chain convex hull in the XZ ground/support plane. */
    public static int convexHull2d(Vec2[] pts, int n, Vec2[] outHull) {
        if (n <= 0) {
            return 0;
        }
        if (n > GaitCore.MAX_HULL_PTS) {
            n = GaitCore.MAX_HULL_PTS;
        }
        Vec2[] s = new Vec2[n];
        System.arraycopy(pts, 0, s, 0, n);
        Arrays.sort(s, 0, n, (a, b) -> {
            int cx = Double.compare(a.x, b.x);
            return cx != 0 ? cx : Double.compare(a.z, b.z);
        });
        int m = 0;
        for (int i = 0; i < n; i++) {
            if (m == 0 || s[i].x != s[m - 1].x || s[i].z != s[m - 1].z) {
                s[m++] = s[i];
            }
        }
        if (m <= 2) {
            for (int i = 0; i < m; i++) {
                outHull[i] = s[i];
            }
            return m;
        }
        Vec2[] lower = new Vec2[m];
        Vec2[] upper = new Vec2[m];
        int nl = 0;
        int nu = 0;
        for (int i = 0; i < m; i++) {
            while (nl >= 2 && cross2(lower[nl - 2], lower[nl - 1], s[i]) <= 0.0) {
                nl--;
            }
            lower[nl++] = s[i];
        }
        for (int i = m - 1; i >= 0; i--) {
            while (nu >= 2 && cross2(upper[nu - 2], upper[nu - 1], s[i]) <= 0.0) {
                nu--;
            }
            upper[nu++] = s[i];
        }
        int total = 0;
        for (int i = 0; i < nl - 1; i++) {
            outHull[total++] = lower[i];
        }
        for (int i = 0; i < nu - 1; i++) {
            outHull[total++] = upper[i];
        }
        return total;
    }

    private static double distPointSeg2d(Vec2 p, Vec2 a, Vec2 b) {
        double ex = b.x - a.x;
        double ez = b.z - a.z;
        double l2 = ex * ex + ez * ez;
        double t = (l2 < 1e-12) ? 0.0 : GaitCore.clamp(((p.x - a.x) * ex + (p.z - a.z) * ez) / l2, 0.0, 1.0);
        return Math.hypot(p.x - (a.x + ex * t), p.z - (a.z + ez * t));
    }

    /** Signed distance from point p to the convex support polygon in the XZ plane (+ inside, - outside). */
    public static double supportMargin(Vec2[] hull, int n, Vec2 p) {
        if (n == 0) {
            return -1e9;
        }
        if (n == 1) {
            return -Math.hypot(p.x - hull[0].x, p.z - hull[0].z);
        }
        if (n == 2) {
            return -distPointSeg2d(p, hull[0], hull[1]);
        }
        boolean inside = true;
        double best = 1e9;
        for (int i = 0; i < n; i++) {
            Vec2 a = hull[i];
            Vec2 b = hull[(i + 1) % n];
            double ex = b.x - a.x;
            double ez = b.z - a.z;
            double el = Math.hypot(ex, ez);
            if (el == 0.0) {
                el = 1e-9;
            }
            double d = (ex * (p.z - a.z) - ez * (p.x - a.x)) / el;
            if (d < 0.0) {
                inside = false;
            }
            if (d < best) {
                best = d;
            }
        }
        if (inside) {
            return best;
        }
        double minD = 1e18;
        for (int i = 0; i < n; i++) {
            double ds = distPointSeg2d(p, hull[i], hull[(i + 1) % n]);
            if (ds < minD) {
                minD = ds;
            }
        }
        return -minD;
    }

    public static Vec2 nearestOnHull(Vec2[] hull, int n, Vec2 p) {
        if (n <= 0) {
            return new Vec2(0.0, 0.0);
        }
        if (n == 1) {
            return hull[0];
        }
        Vec2 best = hull[0];
        double bd = 1e18;
        int edges = n > 2 ? n : 1;
        for (int i = 0; i < edges; i++) {
            Vec2 a = hull[i];
            Vec2 b = hull[(i + 1) % n];
            double ex = b.x - a.x;
            double ez = b.z - a.z;
            double l2 = ex * ex + ez * ez;
            double t = (l2 < 1e-12) ? 0.0 : GaitCore.clamp(((p.x - a.x) * ex + (p.z - a.z) * ez) / l2, 0.0, 1.0);
            Vec2 q = new Vec2(a.x + ex * t, a.z + ez * t);
            double dd = (q.x - p.x) * (q.x - p.x) + (q.z - p.z) * (q.z - p.z);
            if (dd < bd) {
                best = q;
                bd = dd;
            }
        }
        return best;
    }

    /**
     * Fits plane {@code y = a*x + b*z + c} through weighted world points via normal equations.
     * Returns 0 if empty, 1 if only centroid computed (&lt; 3 points or degenerate), 2 if normal valid.
     */
    public static int fitPlane(Vec3[] points, double[] weights, int n, PlaneFit out) {
        if (n == 0) {
            return 0;
        }
        double wsum = 0.0;
        double cx = 0.0;
        double cy = 0.0;
        double cz = 0.0;
        for (int i = 0; i < n; i++) {
            double w = weights != null ? weights[i] : 1.0;
            wsum += w;
            cx += points[i].x * w;
            cy += points[i].y * w;
            cz += points[i].z * w;
        }
        if (wsum == 0.0) {
            wsum = 1.0;
        }
        cx /= wsum;
        cy /= wsum;
        cz /= wsum;
        out.c = new Vec3(cx, cy, cz);
        if (n < 3) {
            return 1;
        }
        double sxx = 0.0;
        double sxz = 0.0;
        double szz = 0.0;
        double sxy = 0.0;
        double szy = 0.0;
        for (int i = 0; i < n; i++) {
            double w = weights != null ? weights[i] : 1.0;
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
        if (Math.abs(det) < 1e-6) {
            return 1;
        }
        double a = (sxy * szz - szy * sxz) / det;
        double b = (szy * sxx - sxy * sxz) / det;
        out.n = new Vec3(-a, 1.0, -b).norm(WORLD_UP);
        return 2;
    }
}
