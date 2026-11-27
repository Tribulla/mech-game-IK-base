package gaitcore;

import java.util.Arrays;

/**
 * Planar leg inverse kinematics: radial coxa / sagittal hip-roll framing, analytic 2-bone,
 * shared-bend 3-bone fast path, warm-started 2D FABRIK, and active-set DLS joint-limit clamp.
 * Mirrors {@code gc_ik.c}.
 */
final class LegIK {

    int mode;
    double coxa;
    int nSegs;
    final double[] lengths = new double[GaitCore.MAX_SEGS];
    final int[] sides = new int[GaitCore.MAX_SEGS - 1];
    double reach;
    double chainReach;
    Vec3 restDirLocal = new Vec3(1.0, 0.0, 0.0);
    boolean hasPts;
    final double[][] pts = new double[GaitCore.MAX_SEGS + 1][2];
    boolean reached = true;
    Vec3 planeNormal = new Vec3(1.0, 0.0, 0.0);
    double err;
    boolean limited;
    boolean hasAngles;
    final double[] angles = new double[GaitCore.MAX_SEGS];
    double yawDeg;
    double hipDeg;
    final double[] bendsDeg = new double[GaitCore.MAX_SEGS - 1];
    double coxaRatio;
    double limitMarginDeg = 90.0;

    double coxaMax;
    double rollMax;
    final double[] hipLim = new double[2];
    final double[][] bendLim = new double[GaitCore.MAX_SEGS - 1][2];
    final double[][] relLim = new double[GaitCore.MAX_SEGS - 1][2];

    // Pre-allocated scratch buffers so solving creates zero per-call garbage.
    private final double[] tmpPlane = new double[10]; // rx,ry,rz, ux,uy,uz, vx,vy,vz, yaw
    private final double[] tmpTb = new double[4];     // kx,ky, fx,fy
    private final double[][] tmpPts2 = new double[GaitCore.MAX_SEGS][2];
    private final double[] tmpAng = new double[GaitCore.MAX_SEGS];
    private final double[][] tmpCands = new double[2][3];
    private final double[] tmpCerr = new double[2];
    private final double[] tmpStart = new double[GaitCore.MAX_SEGS];
    private final double[] tmpAngDls = new double[GaitCore.MAX_SEGS];
    private final double[][] tmpPtsDls = new double[GaitCore.MAX_SEGS][2];
    private final double[] tmpClamped = new double[GaitCore.MAX_SEGS];
    private final double[] tmpAngAlt = new double[GaitCore.MAX_SEGS];
    private final double[][] tmpPtsAlt = new double[GaitCore.MAX_SEGS][2];
    private final double[][] dlsLims = new double[GaitCore.MAX_SEGS][2];
    private final double[] dlsAng = new double[GaitCore.MAX_SEGS];
    private final double[][] dlsPts = new double[GaitCore.MAX_SEGS][2];
    private final double[] dlsBestAng = new double[GaitCore.MAX_SEGS];
    private final double[] dlsJx = new double[GaitCore.MAX_SEGS];
    private final double[] dlsJy = new double[GaitCore.MAX_SEGS];
    private final boolean[] dlsActive = new boolean[GaitCore.MAX_SEGS];
    private final double[] dlsStep = new double[GaitCore.MAX_SEGS];
    private final double[] dlsTrial = new double[GaitCore.MAX_SEGS];
    private final double[] feasibleTb = new double[4];

    /**
     * Analytic 2-bone law of cosines in the 2D leg plane.
     * {@code out4} receives {@code [kx, ky, fx, fy]}.
     */
    static boolean solveTwoBone(double l1, double l2, double tx, double ty, int side, double[] out4) {
        double d = Math.hypot(tx, ty);
        boolean reached = true;
        double dmax = l1 + l2 - 1e-5;
        double dmin = Math.abs(l1 - l2) + 1e-5;
        if (d > dmax) {
            reached = d <= dmax + 1e-3;
            d = dmax;
        } else if (d < dmin) {
            d = dmin;
        }
        double base = Math.atan2(ty, tx);
        double cosA = GaitCore.clamp((l1 * l1 + d * d - l2 * l2) / (2.0 * l1 * d), -1.0, 1.0);
        double a = base + side * Math.acos(cosA);
        double kx = l1 * Math.cos(a);
        double ky = l1 * Math.sin(a);
        double dx = tx - kx;
        double dy = ty - ky;
        double dl = Math.hypot(dx, dy);
        if (dl == 0.0) {
            dl = 1e-9;
        }
        out4[0] = kx;
        out4[1] = ky;
        out4[2] = kx + dx / dl * l2;
        out4[3] = ky + dy / dl * l2;
        return reached;
    }

    /**
     * Warm-started 2D FABRIK solver for multi-segment chains. Reflects interior joints onto their
     * configured bend side before each forward/backward pass.
     */
    static boolean solveFabrik2d(
            double[] lengths, int n, double[][] pts,
            double tx, double ty, int[] sides, int iters, double tol) {
        double total = 0.0;
        for (int i = 0; i < n; i++) {
            total += lengths[i];
        }
        double d = Math.hypot(tx, ty);
        if (d < 1e-6) {
            tx = 1e-6;
            ty = 0.0;
            d = 1e-6;
        }
        double lx = tx / d;
        double ly = ty / d;

        if (d >= total * 0.999) {
            double acc = 0.0;
            for (int i = 0; i < n; i++) {
                acc += lengths[i];
                pts[i + 1][0] = lx * acc;
                pts[i + 1][1] = ly * acc;
            }
            return d <= total * 1.001;
        }

        double tol2 = tol * tol;
        for (int it = 0; it < iters; it++) {
            for (int j = 1; j < n; j++) {
                double px = pts[j][0];
                double py = pts[j][1];
                double s = lx * py - ly * px;
                if (s * sides[j - 1] < 0.0) {
                    double proj = px * lx + py * ly;
                    pts[j][0] = 2.0 * proj * lx - px;
                    pts[j][1] = 2.0 * proj * ly - py;
                }
            }
            pts[n][0] = tx;
            pts[n][1] = ty;
            for (int i = n - 1; i >= 0; i--) {
                double dx = pts[i][0] - pts[i + 1][0];
                double dy = pts[i][1] - pts[i + 1][1];
                double l = Math.hypot(dx, dy);
                if (l == 0.0) {
                    l = 1e-9;
                }
                double k = lengths[i] / l;
                pts[i][0] = pts[i + 1][0] + dx * k;
                pts[i][1] = pts[i + 1][1] + dy * k;
            }
            pts[0][0] = 0.0;
            pts[0][1] = 0.0;
            for (int i = 0; i < n; i++) {
                double dx = pts[i + 1][0] - pts[i][0];
                double dy = pts[i + 1][1] - pts[i][1];
                double l = Math.hypot(dx, dy);
                if (l == 0.0) {
                    l = 1e-9;
                }
                double k = lengths[i] / l;
                pts[i + 1][0] = pts[i][0] + dx * k;
                pts[i + 1][1] = pts[i][1] + dy * k;
            }
            double ex = pts[n][0] - tx;
            double ey = pts[n][1] - ty;
            if (ex * ex + ey * ey < tol2) {
                break;
            }
        }
        return true;
    }

    static void initialPose2d(double[] lengths, int n, int[] sides, double tx, double ty, double[][] pts) {
        double d = Math.hypot(tx, ty);
        if (d == 0.0) {
            d = 1e-6;
        }
        double lx = tx / d;
        double ly = ty / d;
        double nx = -ly;
        double ny = lx;
        pts[0][0] = 0.0;
        pts[0][1] = 0.0;
        double x = 0.0;
        double y = 0.0;
        for (int i = 0; i < n; i++) {
            double L = lengths[i];
            int s = (i < n - 1) ? sides[i] : (i > 0 ? -sides[i - 1] : 0);
            double ang = 0.6 * s;
            double ca = Math.cos(ang);
            double sa = Math.sin(ang);
            double dx = lx * ca + nx * sa;
            double dy = ly * ca + ny * sa;
            x += dx * L;
            y += dy * L;
            pts[i + 1][0] = x;
            pts[i + 1][1] = y;
        }
    }

    void init(
            int planeMode, double coxa, double[] lengths, int nSegs,
            int[] sides, Vec3 restDirLocal, MechDef.Limits lim) {
        this.mode = planeMode;
        this.coxa = coxa;
        this.nSegs = nSegs;
        this.chainReach = 0.0;
        Arrays.fill(this.lengths, 0.0);
        for (int i = 0; i < nSegs; i++) {
            this.lengths[i] = lengths[i];
            this.chainReach += lengths[i];
        }
        Arrays.fill(this.sides, 0);
        for (int i = 0; i < nSegs - 1; i++) {
            this.sides[i] = sides[i];
        }
        this.reach = coxa + this.chainReach;
        this.restDirLocal = restDirLocal;
        this.hasPts = false;
        for (double[] row : this.pts) {
            Arrays.fill(row, 0.0);
        }
        this.reached = true;
        this.planeNormal = new Vec3(1.0, 0.0, 0.0);
        this.err = 0.0;
        this.limited = false;
        this.hasAngles = false;
        Arrays.fill(this.angles, 0.0);
        this.yawDeg = 0.0;
        this.hipDeg = 0.0;
        Arrays.fill(this.bendsDeg, 0.0);
        this.coxaRatio = 0.0;
        this.limitMarginDeg = 90.0;

        this.coxaMax = ((lim != null && lim.hasCoxa) ? lim.coxa : 180.0) * GaitCore.DEG2RAD;
        this.rollMax = ((lim != null && lim.hasRoll) ? lim.roll : 60.0) * GaitCore.DEG2RAD;
        if (lim == null || !lim.hasHip) {
            this.hipLim[0] = -GaitCore.PI;
            this.hipLim[1] = GaitCore.PI;
        } else {
            double h0 = lim.hip[0];
            double h1 = lim.hip[1];
            if (planeMode == GaitCore.PLANE_SAGITTAL) {
                // Sagittal hip limits use 0 deg straight down; internal plane angles measure 0 deg forward.
                h0 -= 90.0;
                h1 -= 90.0;
            }
            this.hipLim[0] = h0 * GaitCore.DEG2RAD;
            this.hipLim[1] = h1 * GaitCore.DEG2RAD;
        }

        int nInt = nSegs - 1;
        for (int i = 0; i < nInt; i++) {
            double b0 = -180.0;
            double b1 = 180.0;
            if (lim != null && lim.bendCount > 0) {
                int bi = i < lim.bendCount ? i : (lim.bendCount - 1);
                b0 = lim.bend[bi][0];
                b1 = lim.bend[bi][1];
            }
            this.bendLim[i][0] = b0 * GaitCore.DEG2RAD;
            this.bendLim[i][1] = b1 * GaitCore.DEG2RAD;
            double s = this.sides[i];
            double r0 = -s * this.bendLim[i][0];
            double r1 = -s * this.bendLim[i][1];
            if (r0 <= r1) {
                this.relLim[i][0] = r0;
                this.relLim[i][1] = r1;
            } else {
                this.relLim[i][0] = r1;
                this.relLim[i][1] = r0;
            }
        }
    }

    private void planeScalars(Vec3 mount, Vec3.Frame frame, Vec3 target, boolean clampYaw, double[] out10) {
        Vec3 up = frame.up;
        double upx = up.x;
        double upy = up.y;
        double upz = up.z;
        double mx = mount.x;
        double my = mount.y;
        double mz = mount.z;
        double dx = target.x - mx;
        double dy = target.y - my;
        double dz = target.z - mz;

        if (mode == GaitCore.PLANE_RADIAL) {
            double rdx, rdy, rdz;
            Vec3 rd = restDirLocal;
            if (frame.isLocal) {
                rdx = rd.x;
                rdy = rd.y;
                rdz = rd.z;
            } else {
                Vec3 rAx = frame.right;
                Vec3 fAx = frame.fwd;
                rdx = rAx.x * rd.x + up.x * rd.y + fAx.x * rd.z;
                rdy = rAx.y * rd.x + up.y * rd.y + fAx.y * rd.z;
                rdz = rAx.z * rd.x + up.z * rd.y + fAx.z * rd.z;
            }
            double du = dx * upx + dy * upy + dz * upz;
            double tx = dx - upx * du;
            double ty = dy - upy * du;
            double tz = dz - upz * du;
            double tl2 = tx * tx + ty * ty + tz * tz;
            double ux, uy, uz;
            if (tl2 < 1e-18) {
                ux = rdx;
                uy = rdy;
                uz = rdz;
            } else {
                double inv = 1.0 / Math.sqrt(tl2);
                ux = tx * inv;
                uy = ty * inv;
                uz = tz * inv;
            }
            double cx = rdy * uz - rdz * uy;
            double cy = rdz * ux - rdx * uz;
            double cz = rdx * uy - rdy * ux;
            double yaw = Math.atan2(cx * upx + cy * upy + cz * upz, rdx * ux + rdy * uy + rdz * uz);
            if (clampYaw && Math.abs(yaw) > coxaMax) {
                if (hasAngles) {
                    // Hold coxa against the side stop it was pressing toward when wrapped behind +/-180 deg.
                    double prevYaw = yawDeg * GaitCore.DEG2RAD;
                    if (Math.abs(prevYaw) > 0.5 * coxaMax && yaw * prevYaw < 0.0 && Math.abs(yaw) > GaitCore.PI - 0.85) {
                        yaw = Math.copySign(coxaMax, prevYaw);
                    } else {
                        yaw = GaitCore.clamp(yaw, -coxaMax, coxaMax);
                    }
                } else {
                    yaw = GaitCore.clamp(yaw, -coxaMax, coxaMax);
                }
                double cyA = Math.cos(yaw);
                double syA = Math.sin(yaw);
                double crx = upy * rdz - upz * rdy;
                double cry = upz * rdx - upx * rdz;
                double crz = upx * rdy - upy * rdx;
                ux = rdx * cyA + crx * syA;
                uy = rdy * cyA + cry * syA;
                uz = rdz * cyA + crz * syA;
                double ul2 = ux * ux + uy * uy + uz * uz;
                if (ul2 < 1e-18) {
                    ux = rdx;
                    uy = rdy;
                    uz = rdz;
                } else {
                    double inv = 1.0 / Math.sqrt(ul2);
                    ux *= inv;
                    uy *= inv;
                    uz *= inv;
                }
            }
            out10[0] = mx + ux * coxa;
            out10[1] = my + uy * coxa;
            out10[2] = mz + uz * coxa;
            out10[3] = ux; out10[4] = uy; out10[5] = uz;
            out10[6] = upx; out10[7] = upy; out10[8] = upz;
            out10[9] = yaw;
            return;
        }

        Vec3 f = frame.fwd;
        double fx = f.x;
        double fy = f.y;
        double fz = f.z;
        double df = dx * fx + dy * fy + dz * fz;
        double px = dx - fx * df;
        double py = dy - fy * df;
        double pz = dz - fz * df;
        double pl2 = px * px + py * py + pz * pz;
        double vx, vy, vz;
        double roll = 0.0;
        if (pl2 < 1e-8) {
            vx = upx;
            vy = upy;
            vz = upz;
        } else {
            double inv = -1.0 / Math.sqrt(pl2);
            vx = px * inv;
            vy = py * inv;
            vz = pz * inv;
            double c = Math.cos(rollMax);
            double vdu = vx * upx + vy * upy + vz * upz;
            if (vdu < c) {
                double sdx = vx - upx * vdu;
                double sdy = vy - upy * vdu;
                double sdz = vz - upz * vdu;
                double sdl2 = sdx * sdx + sdy * sdy + sdz * sdz;
                if (sdl2 < 1e-18) {
                    Vec3 fr = frame.right;
                    sdx = fr.x;
                    sdy = fr.y;
                    sdz = fr.z;
                } else {
                    double invs = 1.0 / Math.sqrt(sdl2);
                    sdx *= invs;
                    sdy *= invs;
                    sdz *= invs;
                }
                double sr = Math.sin(rollMax);
                vx = upx * c + sdx * sr;
                vy = upy * c + sdy * sr;
                vz = upz * c + sdz * sr;
                double vl2 = vx * vx + vy * vy + vz * vz;
                if (vl2 < 1e-18) {
                    vx = upx;
                    vy = upy;
                    vz = upz;
                } else {
                    double invv = 1.0 / Math.sqrt(vl2);
                    vx *= invv;
                    vy *= invv;
                    vz *= invv;
                }
            }
            roll = Math.acos(GaitCore.clamp(vx * upx + vy * upy + vz * upz, -1.0, 1.0));
        }
        out10[0] = mx; out10[1] = my; out10[2] = mz;
        out10[3] = fx; out10[4] = fy; out10[5] = fz;
        out10[6] = vx; out10[7] = vy; out10[8] = vz;
        out10[9] = roll;
    }

    private static void toAngles(double[][] pts2, int n, double[] ang) {
        double px = 0.0;
        double py = 0.0;
        double prev = 0.0;
        for (int i = 0; i < n; i++) {
            double a = Math.atan2(pts2[i][1] - py, pts2[i][0] - px);
            ang[i] = (i == 0) ? a : GaitCore.wrapPi(a - prev);
            prev = a;
            px = pts2[i][0];
            py = pts2[i][1];
        }
    }

    private void fkPts(double[] ang, double[][] pts2) {
        double a = 0.0;
        double x = 0.0;
        double y = 0.0;
        for (int i = 0; i < nSegs; i++) {
            a = (i == 0) ? ang[0] : (a + ang[i]);
            x += lengths[i] * Math.cos(a);
            y += lengths[i] * Math.sin(a);
            pts2[i][0] = x;
            pts2[i][1] = y;
        }
    }

    private boolean isLegal(double[] ang, double tol) {
        if (!(hipLim[0] - tol <= ang[0] && ang[0] <= hipLim[1] + tol)) {
            return false;
        }
        for (int i = 0; i < nSegs - 1; i++) {
            if (!(relLim[i][0] - tol <= ang[i + 1] && ang[i + 1] <= relLim[i][1] + tol)) {
                return false;
            }
        }
        return true;
    }

    private void clampAngles(double[] ang, double[] out) {
        out[0] = GaitCore.clamp(ang[0], hipLim[0], hipLim[1]);
        for (int i = 0; i < nSegs - 1; i++) {
            out[i + 1] = GaitCore.clamp(ang[i + 1], relLim[i][0], relLim[i][1]);
        }
    }

    /** Active-set damped least squares solver used when the unconstrained chain hits a joint stop. */
    private void solveDls(double[] angIn, double tx, double ty, int iters, double[] outAng) {
        int n = nSegs;
        dlsLims[0][0] = hipLim[0];
        dlsLims[0][1] = hipLim[1];
        for (int i = 0; i < n - 1; i++) {
            dlsLims[i + 1][0] = relLim[i][0];
            dlsLims[i + 1][1] = relLim[i][1];
        }

        clampAngles(angIn, dlsAng);
        fkPts(dlsAng, dlsPts);
        double ex = dlsPts[n - 1][0];
        double ey = dlsPts[n - 1][1];
        double curErr2 = (tx - ex) * (tx - ex) + (ty - ey) * (ty - ey);
        System.arraycopy(dlsAng, 0, dlsBestAng, 0, n);
        double lam = 0.08;

        for (int it = 0; it < iters; it++) {
            if (curErr2 < 1e-7) {
                break;
            }
            double rx = tx - ex;
            double ry = ty - ey;
            for (int k = 0; k < n; k++) {
                dlsJx[k] = -(ey - (k == 0 ? 0.0 : dlsPts[k - 1][1]));
                dlsJy[k] = ex - (k == 0 ? 0.0 : dlsPts[k - 1][0]);
                dlsActive[k] = true;
            }
            boolean haveStep = false;

            for (int pass = 0; pass < n; pass++) {
                double lam2 = lam * lam;
                double a = lam2;
                double c = lam2;
                double b = 0.0;
                for (int k = 0; k < n; k++) {
                    if (dlsActive[k]) {
                        a += dlsJx[k] * dlsJx[k];
                        b += dlsJx[k] * dlsJy[k];
                        c += dlsJy[k] * dlsJy[k];
                    }
                }
                double det = a * c - b * b;
                if (det <= 1e-12) {
                    haveStep = false;
                    break;
                }
                double vx = (c * rx - b * ry) / det;
                double vy = (a * ry - b * rx) / det;
                for (int k = 0; k < n; k++) {
                    dlsStep[k] = dlsActive[k] ? (dlsJx[k] * vx + dlsJy[k] * vy) : 0.0;
                }
                haveStep = true;
                boolean pinned = false;
                boolean anyAct = false;
                for (int k = 0; k < n; k++) {
                    if (!dlsActive[k]) {
                        continue;
                    }
                    double lo = dlsLims[k][0];
                    double hi = dlsLims[k][1];
                    double sk = dlsStep[k];
                    double ak = dlsAng[k];
                    double nk = ak + sk;
                    double clamped = (nk < lo) ? lo : ((nk > hi) ? hi : nk);
                    double applied = clamped - ak;
                    if (Math.abs(sk) > 1e-6 && Math.abs(applied) <= 1e-6) {
                        dlsActive[k] = false;
                        pinned = true;
                    } else {
                        anyAct = true;
                    }
                }
                if (!pinned) {
                    break;
                }
                if (!anyAct) {
                    haveStep = false;
                    break;
                }
            }
            if (!haveStep) {
                lam = Math.min(4.0, lam * 2.5);
                continue;
            }
            double m = 0.0;
            for (int k = 0; k < n; k++) {
                if (Math.abs(dlsStep[k]) > m) {
                    m = Math.abs(dlsStep[k]);
                }
            }
            if (m > 0.55) {
                double sc = 0.55 / m;
                for (int k = 0; k < n; k++) {
                    dlsStep[k] *= sc;
                }
            }
            boolean accepted = false;
            for (int si = 0; si < 4; si++) {
                double sc = si == 0 ? 1.0 : (si == 1 ? 0.5 : (si == 2 ? 0.25 : 0.125));
                double aa = 0.0;
                double tex = 0.0;
                double tey = 0.0;
                for (int k = 0; k < n; k++) {
                    double lo = dlsLims[k][0];
                    double hi = dlsLims[k][1];
                    double v = dlsAng[k] + dlsStep[k] * sc;
                    double tk = (v < lo) ? lo : ((v > hi) ? hi : v);
                    dlsTrial[k] = tk;
                    aa = (k == 0) ? tk : (aa + tk);
                    double lk = lengths[k];
                    tex += lk * Math.cos(aa);
                    tey += lk * Math.sin(aa);
                }
                double terr2 = (tx - tex) * (tx - tex) + (ty - tey) * (ty - tey);
                if (terr2 < curErr2 - 1e-10) {
                    System.arraycopy(dlsTrial, 0, dlsAng, 0, n);
                    fkPts(dlsTrial, dlsPts);
                    curErr2 = terr2;
                    ex = dlsPts[n - 1][0];
                    ey = dlsPts[n - 1][1];
                    System.arraycopy(dlsAng, 0, dlsBestAng, 0, n);
                    lam = Math.max(0.01, lam / 2.5);
                    accepted = true;
                    break;
                }
            }
            if (!accepted) {
                lam = Math.min(4.0, lam * 2.5);
            }
        }
        System.arraycopy(dlsBestAng, 0, outAng, 0, n);
    }

    private void threeLinkPose(double tx, double ty, double[][] pts2) {
        double thigh = lengths[0];
        double shin = lengths[1];
        double foot = lengths[2];
        double a = 4.0 * thigh * foot;
        double b = 2.0 * shin * (thigh + foot);
        double c = (thigh - foot) * (thigh - foot) + shin * shin - tx * tx - ty * ty;
        double disc = b * b - 4.0 * a * c;
        double cosine = (-b + Math.sqrt(disc > 0.0 ? disc : 0.0)) / (2.0 * a);
        double ankle = -sides[1] * Math.acos(GaitCore.clamp(cosine, -1.0, 1.0));
        ankle = GaitCore.clamp(ankle, relLim[1][0], relLim[1][1]);

        double dx = shin + foot * Math.cos(ankle);
        double dy = foot * Math.sin(ankle);
        solveTwoBone(thigh, Math.hypot(dx, dy), tx, ty, sides[0], tmpTb);
        double kx = tmpTb[0], ky = tmpTb[1], ex = tmpTb[2], ey = tmpTb[3];
        double hip = Math.atan2(ky, kx);
        double knee = GaitCore.wrapPi(Math.atan2(ey - ky, ex - kx) - Math.atan2(dy, dx) - hip);
        double[] ang = tmpStart;
        ang[0] = hip;
        ang[1] = knee;
        ang[2] = ankle;
        fkPts(ang, pts2);
    }

    private int threeLinkClampedCandidates(
            double tx, double ty, double[] ang, double[][] outCands, double[] outErr) {
        double thigh = lengths[0];
        double shin = lengths[1];
        double foot = lengths[2];
        int count = 0;
        if (!(hipLim[0] - 1e-4 <= ang[0] && ang[0] <= hipLim[1] + 1e-4)) {
            double hip = GaitCore.clamp(ang[0], hipLim[0], hipLim[1]);
            double kx = thigh * Math.cos(hip);
            double ky = thigh * Math.sin(hip);
            solveTwoBone(shin, foot, tx - kx, ty - ky, sides[1], tmpTb);
            double kx2 = tmpTb[0], ky2 = tmpTb[1], fx2 = tmpTb[2], fy2 = tmpTb[3];
            double a1 = Math.atan2(ky2, kx2);
            double a2 = Math.atan2(fy2 - ky2, fx2 - kx2);
            outCands[count][0] = hip;
            outCands[count][1] = GaitCore.wrapPi(a1 - hip);
            outCands[count][2] = GaitCore.wrapPi(a2 - a1);
            outErr[count] = Math.hypot(kx + fx2 - tx, ky + fy2 - ty);
            count++;
        }
        if (!(relLim[0][0] - 1e-4 <= ang[1] && ang[1] <= relLim[0][1] + 1e-4)) {
            double knee = GaitCore.clamp(ang[1], relLim[0][0], relLim[0][1]);
            double dx = thigh + shin * Math.cos(knee);
            double dy = shin * Math.sin(knee);
            double lUp = Math.hypot(dx, dy);
            if (lUp > 1e-6) {
                double off = Math.atan2(dy, dx);
                solveTwoBone(lUp, foot, tx, ty, sides[1], tmpTb);
                double ax = tmpTb[0], ay = tmpTb[1], fx = tmpTb[2], fy = tmpTb[3];
                double hip = GaitCore.wrapPi(Math.atan2(ay, ax) - off);
                double ankle = GaitCore.wrapPi(Math.atan2(fy - ay, fx - ax) - (hip + knee));
                outCands[count][0] = hip;
                outCands[count][1] = knee;
                outCands[count][2] = ankle;
                outErr[count] = Math.hypot(fx - tx, fy - ty);
                count++;
            }
        }
        return count;
    }

    int solve(Vec3 mount, Vec3.Frame frame, Vec3 target, double planeRoll, Vec3[] outJoints) {
        planeScalars(mount, frame, target, true, tmpPlane);
        double rx = tmpPlane[0], ry = tmpPlane[1], rz = tmpPlane[2];
        double ux = tmpPlane[3], uy = tmpPlane[4], uz = tmpPlane[5];
        double vx = tmpPlane[6], vy = tmpPlane[7], vz = tmpPlane[8];
        double yaw = tmpPlane[9];

        double dx = target.x - rx;
        double dy = target.y - ry;
        double dz = target.z - rz;
        if (mode == GaitCore.PLANE_RADIAL && Math.abs(planeRoll) > 1e-5) {
            double dl2 = dx * dx + dy * dy + dz * dz;
            if (dl2 > 1e-6) {
                double inva = 1.0 / Math.sqrt(dl2);
                double ax = dx * inva;
                double ay = dy * inva;
                double az = dz * inva;
                double cp = Math.cos(planeRoll);
                double sp = Math.sin(planeRoll);
                double mc = 1.0 - cp;
                double du = (ax * ux + ay * uy + az * uz) * mc;
                double cux = ay * uz - az * uy;
                double cuy = az * ux - ax * uz;
                double cuz = ax * uy - ay * ux;
                double uxn = ux * cp + cux * sp + ax * du;
                double uyn = uy * cp + cuy * sp + ay * du;
                double uzn = uz * cp + cuz * sp + az * du;
                ux = uxn;
                uy = uyn;
                uz = uzn;
                double dv = (ax * vx + ay * vy + az * vz) * mc;
                double cvx = ay * vz - az * vy;
                double cvy = az * vx - ax * vz;
                double cvz = ax * vy - ay * vx;
                double vxn = vx * cp + cvx * sp + ax * dv;
                double vyn = vy * cp + cvy * sp + ay * dv;
                double vzn = vz * cp + cvz * sp + az * dv;
                vx = vxn;
                vy = vyn;
                vz = vzn;
            }
        }

        double tx = dx * ux + dy * uy + dz * uz;
        double ty = dx * vx + dy * vy + dz * vz;
        double nx = uy * vz - uz * vy;
        double ny = uz * vx - ux * vz;
        double nz = ux * vy - uy * vx;
        double nl2 = nx * nx + ny * ny + nz * nz;
        if (nl2 < 1e-18) {
            nx = frame.right.x;
            ny = frame.right.y;
            nz = frame.right.z;
        } else {
            double invn = 1.0 / Math.sqrt(nl2);
            nx *= invn;
            ny *= invn;
            nz *= invn;
        }
        planeNormal = new Vec3(nx, ny, nz);

        int nj = 0;
        outJoints[nj++] = mount;
        if (mode == GaitCore.PLANE_RADIAL && coxa > 1e-6) {
            outJoints[nj++] = new Vec3(rx, ry, rz);
        }

        boolean threeSame = nSegs == 3 && sides[0] == sides[1] && lengths[0] > 0.0 && lengths[2] > 0.0;
        double[][] pts2 = tmpPts2;
        if (nSegs == 2) {
            reached = solveTwoBone(lengths[0], lengths[1], tx, ty, sides[0], tmpTb);
            pts2[0][0] = tmpTb[0];
            pts2[0][1] = tmpTb[1];
            pts2[1][0] = tmpTb[2];
            pts2[1][1] = tmpTb[3];
        } else if (threeSame) {
            threeLinkPose(tx, ty, pts2);
            reached = true;
        } else {
            if (!hasPts || !reached) {
                initialPose2d(lengths, nSegs, sides, tx, ty, pts);
                hasPts = true;
            }
            reached = solveFabrik2d(lengths, nSegs, pts, tx, ty, sides, 10, 1e-3);
            for (int i = 0; i < nSegs; i++) {
                pts2[i][0] = pts[i + 1][0];
                pts2[i][1] = pts[i + 1][1];
            }
        }

        double[] ang = tmpAng;
        toAngles(pts2, nSegs, ang);
        limited = !isLegal(ang, 1e-3);
        if (limited) {
            boolean solvedExact = false;
            if (threeSame) {
                int nc = threeLinkClampedCandidates(tx, ty, ang, tmpCands, tmpCerr);
                for (int ci = 0; ci < nc; ci++) {
                    if (tmpCerr[ci] < 2e-3 && isLegal(tmpCands[ci], 1e-3)) {
                        System.arraycopy(tmpCands[ci], 0, ang, 0, 3);
                        fkPts(ang, pts2);
                        solvedExact = true;
                        break;
                    }
                }
            }
            if (!solvedExact) {
                boolean wasReachable = hasAngles && err <= 0.02;
                double[] start = tmpStart;
                if (wasReachable) {
                    System.arraycopy(angles, 0, start, 0, nSegs);
                } else {
                    clampAngles(ang, start);
                }
                solveDls(start, tx, ty, 12, tmpAngDls);
                fkPts(tmpAngDls, tmpPtsDls);
                if (wasReachable) {
                    double ex = tmpPtsDls[nSegs - 1][0];
                    double ey = tmpPtsDls[nSegs - 1][1];
                    if ((ex - tx) * (ex - tx) + (ey - ty) * (ey - ty) > 1e-4) {
                        clampAngles(ang, tmpClamped);
                        solveDls(tmpClamped, tx, ty, 12, tmpAngAlt);
                        fkPts(tmpAngAlt, tmpPtsAlt);
                        double ax = tmpPtsAlt[nSegs - 1][0];
                        double ay = tmpPtsAlt[nSegs - 1][1];
                        if ((ax - tx) * (ax - tx) + (ay - ty) * (ay - ty)
                                < (ex - tx) * (ex - tx) + (ey - ty) * (ey - ty)) {
                            System.arraycopy(tmpAngAlt, 0, tmpAngDls, 0, nSegs);
                            for (int i = 0; i < nSegs; i++) {
                                tmpPtsDls[i][0] = tmpPtsAlt[i][0];
                                tmpPtsDls[i][1] = tmpPtsAlt[i][1];
                            }
                        }
                    }
                }
                System.arraycopy(tmpAngDls, 0, ang, 0, nSegs);
                for (int i = 0; i < nSegs; i++) {
                    pts2[i][0] = tmpPtsDls[i][0];
                    pts2[i][1] = tmpPtsDls[i][1];
                }
            }
            if (nSegs > 2) {
                pts[0][0] = 0.0;
                pts[0][1] = 0.0;
                for (int i = 0; i < nSegs; i++) {
                    pts[i + 1][0] = pts2[i][0];
                    pts[i + 1][1] = pts2[i][1];
                }
                hasPts = true;
            }
        }

        hasAngles = true;
        System.arraycopy(ang, 0, angles, 0, nSegs);

        double fx = pts2[nSegs - 1][0];
        double fy = pts2[nSegs - 1][1];
        double off = dx * nx + dy * ny + dz * nz;
        err = Math.sqrt((fx - tx) * (fx - tx) + (fy - ty) * (fy - ty) + off * off);
        reached = reached && err < 0.02;

        for (int i = 0; i < nSegs; i++) {
            double px = pts2[i][0];
            double py = pts2[i][1];
            outJoints[nj++] = new Vec3(
                    rx + ux * px + vx * py,
                    ry + uy * px + vy * py,
                    rz + uz * px + vz * py);
        }
        yawDeg = yaw * GaitCore.RAD2DEG;
        hipDeg = ang[0] * GaitCore.RAD2DEG;
        for (int i = 0; i < nSegs - 1; i++) {
            bendsDeg[i] = -sides[i] * ang[i + 1] * GaitCore.RAD2DEG;
        }

        if (mode == GaitCore.PLANE_RADIAL && coxaMax < 3.0) {
            coxaRatio = Math.abs(yaw) / Math.max(coxaMax, 1e-6);
        } else if (mode == GaitCore.PLANE_SAGITTAL && rollMax < 1.5) {
            coxaRatio = Math.abs(yaw) / Math.max(rollMax, 1e-6);
        } else {
            coxaRatio = 0.0;
        }

        double mAng = Math.min(ang[0] - hipLim[0], hipLim[1] - ang[0]);
        for (int i = 0; i < nSegs - 1; i++) {
            mAng = Math.min(mAng, ang[i + 1] - relLim[i][0]);
            mAng = Math.min(mAng, relLim[i][1] - ang[i + 1]);
        }
        limitMarginDeg = mAng * GaitCore.RAD2DEG;
        return nj;
    }

    /** Fast analytical reachability test for foothold filtering without disturbing warm-start state. */
    boolean feasible(Vec3 mount, Vec3.Frame frame, Vec3 target, double slackDeg) {
        double slack = slackDeg * GaitCore.DEG2RAD;
        if (mode == GaitCore.PLANE_RADIAL) {
            Vec3 up = frame.up;
            double upx = up.x;
            double upy = up.y;
            double upz = up.z;
            double dx = target.x - mount.x;
            double dy = target.y - mount.y;
            double dz = target.z - mount.z;
            double du = dx * upx + dy * upy + dz * upz;
            double dtx = dx - upx * du;
            double dty = dy - upy * du;
            double dtz = dz - upz * du;
            double dtl2 = dtx * dtx + dty * dty + dtz * dtz;
            if (dtl2 > 1e-6) {
                Vec3 rd = restDirLocal;
                double rdx, rdy, rdz;
                if (frame.isLocal) {
                    rdx = rd.x;
                    rdy = rd.y;
                    rdz = rd.z;
                } else {
                    Vec3 rAx = frame.right;
                    Vec3 fAx = frame.fwd;
                    rdx = rAx.x * rd.x + upx * rd.y + fAx.x * rd.z;
                    rdy = rAx.y * rd.x + upy * rd.y + fAx.y * rd.z;
                    rdz = rAx.z * rd.x + upz * rd.y + fAx.z * rd.z;
                }
                double inv = 1.0 / Math.sqrt(dtl2);
                double ux0 = dtx * inv;
                double uy0 = dty * inv;
                double uz0 = dtz * inv;
                double cx = rdy * uz0 - rdz * uy0;
                double cy = rdz * ux0 - rdx * uz0;
                double cz = rdx * uy0 - rdy * ux0;
                double yaw = Math.atan2(cx * upx + cy * upy + cz * upz, rdx * ux0 + rdy * uy0 + rdz * uz0);
                if (Math.abs(yaw) > coxaMax + slack) {
                    return false;
                }
            }
        }

        planeScalars(mount, frame, target, true, tmpPlane);
        double rx = tmpPlane[0], ry = tmpPlane[1], rz = tmpPlane[2];
        double ux = tmpPlane[3], uy = tmpPlane[4], uz = tmpPlane[5];
        double vx = tmpPlane[6], vy = tmpPlane[7], vz = tmpPlane[8];
        double dx = target.x - rx;
        double dy = target.y - ry;
        double dz = target.z - rz;
        double nx = uy * vz - uz * vy;
        double ny = uz * vx - ux * vz;
        double nz = ux * vy - uy * vx;
        double off = dx * nx + dy * ny + dz * nz;
        double maxOff = (mode == GaitCore.PLANE_SAGITTAL) ? 0.018 : (0.025 + 0.04 * reach * (slackDeg / 6.0));
        if (Math.abs(off) > maxOff) {
            return false;
        }

        double tx = dx * ux + dy * uy + dz * uz;
        double ty = dx * vx + dy * vy + dz * vz;
        double d = Math.hypot(tx, ty);
        double h0 = hipLim[0];
        double h1 = hipLim[1];

        if (nSegs == 3 && sides[0] == sides[1] && lengths[0] > 0.0 && lengths[2] > 0.0) {
            if (d < 1e-4 || d > chainReach + 1e-3) {
                return false;
            }
            threeLinkPose(tx, ty, tmpPts2);
            toAngles(tmpPts2, 3, tmpAng);
            double tipErr = Math.hypot(tmpPts2[2][0] - tx, tmpPts2[2][1] - ty);
            if (tipErr <= 0.03) {
                double wh = GaitCore.wrapPi(tmpAng[0]);
                if (h0 - slack <= wh && wh <= h1 + slack
                        && relLim[0][0] - slack <= tmpAng[1] && tmpAng[1] <= relLim[0][1] + slack
                        && relLim[1][0] - slack <= tmpAng[2] && tmpAng[2] <= relLim[1][1] + slack) {
                    return true;
                }
            }
            int nc = threeLinkClampedCandidates(tx, ty, tmpAng, tmpCands, tmpCerr);
            for (int ci = 0; ci < nc; ci++) {
                double wh = GaitCore.wrapPi(tmpCands[ci][0]);
                if (tmpCerr[ci] <= 0.03
                        && h0 - slack <= wh && wh <= h1 + slack
                        && relLim[0][0] - slack <= tmpCands[ci][1] && tmpCands[ci][1] <= relLim[0][1] + slack
                        && relLim[1][0] - slack <= tmpCands[ci][2] && tmpCands[ci][2] <= relLim[1][1] + slack) {
                    return true;
                }
            }
            return false;
        }

        double l1 = lengths[0];
        double l2;
        double bLo = bendLim[0][0];
        double bHi = bendLim[0][1];
        if (nSegs == 2) {
            l2 = lengths[1];
        } else {
            double la = lengths[1];
            double lb = lengths[2];
            double bm = 0.5 * (bendLim[1][0] + bendLim[1][1]);
            l2 = Math.sqrt(Math.max(1e-6, la * la + lb * lb + 2.0 * la * lb * Math.cos(bm)));
            slack += 15.0 * GaitCore.DEG2RAD;
        }
        if (d < 1e-4 || d > l1 + l2) {
            return false;
        }
        double cb = (d * d - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
        if (Math.abs(cb) > 1.0) {
            return false;
        }
        double bend = Math.acos(cb);
        if (bend < Math.max(0.0, bLo) - slack || bend > bHi + slack) {
            return false;
        }
        double alpha = Math.acos(GaitCore.clamp((l1 * l1 + d * d - l2 * l2) / (2.0 * l1 * d), -1.0, 1.0));
        double hip = GaitCore.wrapPi(Math.atan2(ty, tx) + sides[0] * alpha);
        return h0 - slack <= hip && hip <= h1 + slack;
    }
}
