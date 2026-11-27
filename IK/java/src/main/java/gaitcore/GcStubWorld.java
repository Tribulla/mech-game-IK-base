package gaitcore;

import java.util.ArrayList;
import java.util.List;

/**
 * Lightweight standalone test/benchmark world supporting flat ground, AABBs, and rotated OBBs.
 * Mirrors {@code GcStubWorld} in {@code gc_world.c} and is strictly read-only once geometry setup finishes.
 */
public final class GcStubWorld implements GcWorldCallbacks {

    private static final class Box {
        final Vec3 mn;
        final Vec3 mx;

        Box(Vec3 mn, Vec3 mx) {
            this.mn = mn;
            this.mx = mx;
        }
    }

    private static final class OBox {
        final Vec3 c;
        final double h0, h1, h2;
        final Vec3 ax0, ax1, ax2;
        final Vec3 bmin;
        final Vec3 bmax;

        OBox(Vec3 c, double h0, double h1, double h2, Vec3 ax0, Vec3 ax1, Vec3 ax2, Vec3 bmin, Vec3 bmax) {
            this.c = c;
            this.h0 = h0;
            this.h1 = h1;
            this.h2 = h2;
            this.ax0 = ax0;
            this.ax1 = ax1;
            this.ax2 = ax2;
            this.bmin = bmin;
            this.bmax = bmax;
        }
    }

    private final double groundY;
    private final boolean hasGround;
    private final List<Box> boxes = new ArrayList<>();
    private final List<OBox> oboxes = new ArrayList<>();

    public GcStubWorld(double groundY, boolean hasGround) {
        this.groundY = groundY;
        this.hasGround = hasGround;
    }

    public void addBox(double x0, double y0, double z0, double x1, double y1, double z1) {
        Vec3 mn = new Vec3(Math.min(x0, x1), Math.min(y0, y1), Math.min(z0, z1));
        Vec3 mx = new Vec3(Math.max(x0, x1), Math.max(y0, y1), Math.max(z0, z1));
        boxes.add(new Box(mn, mx));
    }

    public void addOBox(
            double cx, double cy, double cz,
            double hx, double hy, double hz,
            double yaw, double pitch) {
        double cyV = Math.cos(yaw);
        double syV = Math.sin(yaw);
        double cp = Math.cos(pitch);
        double sp = Math.sin(pitch);
        Vec3 xAxis = new Vec3(cyV, 0.0, -syV);
        Vec3 yAxis = new Vec3(0.0, 1.0, 0.0);
        Vec3 zAxis = new Vec3(syV, 0.0, cyV);
        Vec3 ax0 = xAxis;
        Vec3 ax1 = yAxis.mul(cp).add(zAxis.mul(sp));
        Vec3 ax2 = zAxis.mul(cp).sub(yAxis.mul(sp));
        double ex = Math.abs(ax0.x) * hx + Math.abs(ax1.x) * hy + Math.abs(ax2.x) * hz;
        double ey = Math.abs(ax0.y) * hx + Math.abs(ax1.y) * hy + Math.abs(ax2.y) * hz;
        double ez = Math.abs(ax0.z) * hx + Math.abs(ax1.z) * hy + Math.abs(ax2.z) * hz;
        oboxes.add(new OBox(
                new Vec3(cx, cy, cz),
                hx, hy, hz,
                ax0, ax1, ax2,
                new Vec3(cx - ex, cy - ey, cz - ez),
                new Vec3(cx + ex, cy + ey, cz + ez)));
    }

    @Override
    public boolean raycast(
            double ox, double oy, double oz,
            double dx, double dy, double dz,
            double maxDist,
            RayHit outHit) {
        boolean hasHit = false;
        double bestT = maxDist;
        double bestPx = 0.0, bestPy = 0.0, bestPz = 0.0;
        double bestNx = 0.0, bestNy = 1.0, bestNz = 0.0;
        int bestSid = 0;

        if (hasGround && dy < -1e-9 && oy > groundY) {
            double t = (groundY - oy) / dy;
            if (t <= bestT) {
                hasHit = true;
                bestT = t;
                bestPx = ox + dx * t;
                bestPy = groundY;
                bestPz = oz + dz * t;
                bestNx = 0.0;
                bestNy = 1.0;
                bestNz = 0.0;
                bestSid = 0;
            }
        }

        int nBoxes = boxes.size();
        for (int i = 0; i < nBoxes; i++) {
            Box b = boxes.get(i);
            Vec3 mn = b.mn;
            Vec3 mx = b.mx;
            double tmin = -1e30;
            double tmax = 1e30;
            double sgn = 0.0;
            int axis = -1;
            boolean ok = true;
            for (int a = 0; a < 3; a++) {
                double oa = a == 0 ? ox : (a == 1 ? oy : oz);
                double da = a == 0 ? dx : (a == 1 ? dy : dz);
                double mna = a == 0 ? mn.x : (a == 1 ? mn.y : mn.z);
                double mxa = a == 0 ? mx.x : (a == 1 ? mx.y : mx.z);
                if (Math.abs(da) < 1e-12) {
                    if (oa < mna || oa > mxa) {
                        ok = false;
                        break;
                    }
                } else {
                    double inv = 1.0 / da;
                    double t1 = (mna - oa) * inv;
                    double t2 = (mxa - oa) * inv;
                    double s = -1.0;
                    if (t1 > t2) {
                        double tmp = t1;
                        t1 = t2;
                        t2 = tmp;
                        s = 1.0;
                    }
                    if (t1 > tmin) {
                        tmin = t1;
                        axis = a;
                        sgn = s;
                    }
                    if (t2 < tmax) {
                        tmax = t2;
                    }
                    if (tmin > tmax) {
                        ok = false;
                        break;
                    }
                }
            }
            if (!ok || axis < 0 || tmin < 0.0 || tmin > maxDist) {
                continue;
            }
            if (!hasHit || tmin < bestT) {
                hasHit = true;
                bestT = tmin;
                bestPx = ox + dx * tmin;
                bestPy = oy + dy * tmin;
                bestPz = oz + dz * tmin;
                bestNx = axis == 0 ? sgn : 0.0;
                bestNy = axis == 1 ? sgn : 0.0;
                bestNz = axis == 2 ? sgn : 0.0;
                bestSid = i + 1;
            }
        }

        double clampD = Math.min(maxDist, 1e4);
        double rx0 = ox;
        double rx1 = ox + dx * clampD;
        double rz0 = oz;
        double rz1 = oz + dz * clampD;
        if (rx0 > rx1) {
            double tmp = rx0;
            rx0 = rx1;
            rx1 = tmp;
        }
        if (rz0 > rz1) {
            double tmp = rz0;
            rz0 = rz1;
            rz1 = tmp;
        }

        int nOBoxes = oboxes.size();
        for (int i = 0; i < nOBoxes; i++) {
            OBox ob = oboxes.get(i);
            if (ob.bmax.x < rx0 || ob.bmin.x > rx1 || ob.bmax.z < rz0 || ob.bmin.z > rz1) {
                continue;
            }
            double rx = ox - ob.c.x;
            double ry = oy - ob.c.y;
            double rz = oz - ob.c.z;
            double tmin = -1e30;
            double tmax = 1e30;
            double sgn = 0.0;
            int axis = -1;
            boolean ok = true;
            for (int a = 0; a < 3; a++) {
                Vec3 ax = a == 0 ? ob.ax0 : (a == 1 ? ob.ax1 : ob.ax2);
                double hi = a == 0 ? ob.h0 : (a == 1 ? ob.h1 : ob.h2);
                double oi = rx * ax.x + ry * ax.y + rz * ax.z;
                double di = dx * ax.x + dy * ax.y + dz * ax.z;
                if (Math.abs(di) < 1e-12) {
                    if (oi < -hi || oi > hi) {
                        ok = false;
                        break;
                    }
                    continue;
                }
                double inv = 1.0 / di;
                double t1 = (-hi - oi) * inv;
                double t2 = (hi - oi) * inv;
                double s = -1.0;
                if (t1 > t2) {
                    double tmp = t1;
                    t1 = t2;
                    t2 = tmp;
                    s = 1.0;
                }
                if (t1 > tmin) {
                    tmin = t1;
                    axis = a;
                    sgn = s;
                }
                if (t2 < tmax) {
                    tmax = t2;
                }
                if (tmin > tmax) {
                    ok = false;
                    break;
                }
            }
            if (!ok || axis < 0 || tmin < 0.0 || tmin > maxDist) {
                continue;
            }
            if (!hasHit || tmin < bestT) {
                hasHit = true;
                bestT = tmin;
                bestPx = ox + dx * tmin;
                bestPy = oy + dy * tmin;
                bestPz = oz + dz * tmin;
                Vec3 hitAx = axis == 0 ? ob.ax0 : (axis == 1 ? ob.ax1 : ob.ax2);
                bestNx = hitAx.x * sgn;
                bestNy = hitAx.y * sgn;
                bestNz = hitAx.z * sgn;
                bestSid = nBoxes + i + 1;
            }
        }

        if (!hasHit) {
            return false;
        }
        outHit.t = bestT;
        outHit.px = bestPx;
        outHit.py = bestPy;
        outHit.pz = bestPz;
        outHit.nx = bestNx;
        outHit.ny = bestNy;
        outHit.nz = bestNz;
        outHit.surface = bestSid;
        return true;
    }

    @Override
    public boolean spherePush(
            double cx, double cy, double cz,
            double r,
            double[] outXyz,
            double[] outNxyz) {
        boolean hit = false;
        double nxOut = 0.0;
        double nyOut = 1.0;
        double nzOut = 0.0;
        if (hasGround && cy - r < groundY) {
            cy = groundY + r;
            nxOut = 0.0;
            nyOut = 1.0;
            nzOut = 0.0;
            hit = true;
        }
        double r2 = r * r;
        int nBoxes = boxes.size();
        for (int i = 0; i < nBoxes; i++) {
            Box b = boxes.get(i);
            Vec3 mn = b.mn;
            Vec3 mx = b.mx;
            double px = cx < mn.x ? mn.x : (cx > mx.x ? mx.x : cx);
            double py = cy < mn.y ? mn.y : (cy > mx.y ? mx.y : cy);
            double pz = cz < mn.z ? mn.z : (cz > mx.z ? mx.z : cz);
            double dx = cx - px;
            double dy = cy - py;
            double dz = cz - pz;
            double l2 = dx * dx + dy * dy + dz * dz;
            if (l2 < 1e-18) {
                // Center lies inside the AABB: resolve across the shallowest face.
                double p0 = cx - mn.x;
                double p1 = mx.x - cx;
                double p2 = cy - mn.y;
                double p3 = mx.y - cy;
                double p4 = cz - mn.z;
                double p5 = mx.z - cz;
                int bestK = 0;
                double bestPen = p0;
                if (p1 < bestPen) { bestPen = p1; bestK = 1; }
                if (p2 < bestPen) { bestPen = p2; bestK = 2; }
                if (p3 < bestPen) { bestPen = p3; bestK = 3; }
                if (p4 < bestPen) { bestPen = p4; bestK = 4; }
                if (p5 < bestPen) { bestPen = p5; bestK = 5; }
                double nx = bestK == 0 ? -1.0 : (bestK == 1 ? 1.0 : 0.0);
                double ny = bestK == 2 ? -1.0 : (bestK == 3 ? 1.0 : 0.0);
                double nz = bestK == 4 ? -1.0 : (bestK == 5 ? 1.0 : 0.0);
                double kk = bestPen + r;
                cx += nx * kk;
                cy += ny * kk;
                cz += nz * kk;
                nxOut = nx;
                nyOut = ny;
                nzOut = nz;
                hit = true;
            } else if (l2 < r2) {
                double l = Math.sqrt(l2);
                double inv = 1.0 / l;
                double nx = dx * inv;
                double ny = dy * inv;
                double nz = dz * inv;
                double kk = r - l;
                cx += nx * kk;
                cy += ny * kk;
                cz += nz * kk;
                nxOut = nx;
                nyOut = ny;
                nzOut = nz;
                hit = true;
            }
        }

        int nOBoxes = oboxes.size();
        for (int i = 0; i < nOBoxes; i++) {
            OBox ob = oboxes.get(i);
            if (cx + r < ob.bmin.x || cx - r > ob.bmax.x
                    || cy + r < ob.bmin.y || cy - r > ob.bmax.y
                    || cz + r < ob.bmin.z || cz - r > ob.bmax.z) {
                continue;
            }
            double rx = cx - ob.c.x;
            double ry = cy - ob.c.y;
            double rz = cz - ob.c.z;
            Vec3 ax0 = ob.ax0;
            Vec3 ax1 = ob.ax1;
            Vec3 ax2 = ob.ax2;
            double l0 = rx * ax0.x + ry * ax0.y + rz * ax0.z;
            double l1 = rx * ax1.x + ry * ax1.y + rz * ax1.z;
            double l2 = rx * ax2.x + ry * ax2.y + rz * ax2.z;
            double h0 = ob.h0;
            double h1 = ob.h1;
            double h2 = ob.h2;
            double q0 = l0 < -h0 ? -h0 : (l0 > h0 ? h0 : l0);
            double q1 = l1 < -h1 ? -h1 : (l1 > h1 ? h1 : l1);
            double q2 = l2 < -h2 ? -h2 : (l2 > h2 ? h2 : l2);
            double d0 = l0 - q0;
            double d1 = l1 - q1;
            double d2 = l2 - q2;
            double dl2 = d0 * d0 + d1 * d1 + d2 * d2;
            if (dl2 < 1e-18) {
                double p0 = l0 + h0;
                double p1 = h0 - l0;
                double p2 = l1 + h1;
                double p3 = h1 - l1;
                double p4 = l2 + h2;
                double p5 = h2 - l2;
                int bestK = 0;
                double bestPen = p0;
                if (p1 < bestPen) { bestPen = p1; bestK = 1; }
                if (p2 < bestPen) { bestPen = p2; bestK = 2; }
                if (p3 < bestPen) { bestPen = p3; bestK = 3; }
                if (p4 < bestPen) { bestPen = p4; bestK = 4; }
                if (p5 < bestPen) { bestPen = p5; bestK = 5; }
                Vec3 nax = (bestK / 2) == 0 ? ax0 : ((bestK / 2) == 1 ? ax1 : ax2);
                double ns = (bestK % 2 == 0) ? -1.0 : 1.0;
                nxOut = nax.x * ns;
                nyOut = nax.y * ns;
                nzOut = nax.z * ns;
                double kk = bestPen + r;
                cx += nxOut * kk;
                cy += nyOut * kk;
                cz += nzOut * kk;
                hit = true;
            } else if (dl2 < r2) {
                double dl = Math.sqrt(dl2);
                double inv = 1.0 / dl;
                nxOut = (ax0.x * d0 + ax1.x * d1 + ax2.x * d2) * inv;
                nyOut = (ax0.y * d0 + ax1.y * d1 + ax2.y * d2) * inv;
                nzOut = (ax0.z * d0 + ax1.z * d1 + ax2.z * d2) * inv;
                double kk = r - dl;
                cx += nxOut * kk;
                cy += nyOut * kk;
                cz += nzOut * kk;
                hit = true;
            }
        }
        outXyz[0] = cx;
        outXyz[1] = cy;
        outXyz[2] = cz;
        outNxyz[0] = nxOut;
        outNxyz[1] = nyOut;
        outNxyz[2] = nzOut;
        return hit;
    }

    @Override
    public double terrainHeight(double x, double z) {
        return hasGround ? groundY : 0.0;
    }
}
