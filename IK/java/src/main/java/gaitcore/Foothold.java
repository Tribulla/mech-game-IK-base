package gaitcore;

import java.util.Arrays;

/**
 * Foothold selection: predicts touchdown hip pose, filters cliffs/thin walls/ceilings, tries local
 * nudges around rock tops first, then falls back to ringed searches. Mirrors {@code gc_foothold.c}.
 */
final class Foothold {

    private Foothold() {
    }

    static final class PoseOut {
        Vec3 mount = Vec3.ZERO;
        final Vec3.Frame frame = new Vec3.Frame();
    }

    static final class Result {
        Vec3 pt = Vec3.ZERO;
        Vec3 n = Vec3.WORLD_UP;
    }

    static final class Cand {
        double sc;
        double t;
        Vec3 point = Vec3.ZERO;
        Vec3 normal = Vec3.WORLD_UP;
        int surface;

        void set(double sc, WorldQueryCtx.Hit h) {
            this.sc = sc;
            this.t = h.t;
            this.point = h.point;
            this.normal = h.normal;
            this.surface = h.surface;
        }
    }

    /** Projects where the hip mount will be after {@code delay} seconds of travel and turn yaw. */
    static void landingPose(Mech m, Leg l, double delay, PoseOut out) {
        out.frame.copyFrom(m.frame);
        if (Math.abs(m.yawRate * delay) > 1e-6) {
            out.frame.rotate(m.probeUp(), -m.yawRate * delay);
        }
        Vec3 hull;
        if (m.state == GaitCore.STATE_MANTLE && m.mantle.active) {
            hull = m.mantlePos(m.mantle.t + delay / m.mantle.dur);
        } else {
            hull = m.pos.add(Scheduler.planarTravel(m, delay));
        }
        if (m.state == GaitCore.STATE_GROUND && !m.climbingMode()) {
            if (m.flightT > 0.0) {
                double t = Math.min(m.flightT, delay);
                hull = new Vec3(hull.x, hull.y + m.vel.y * t - 0.5 * GaitCore.G * t * t, hull.z);
            } else if (0.5 < m.groundN.y && m.groundN.y < 0.995) {
                double dhX = hull.x - m.pos.x;
                double dhZ = hull.z - m.pos.z;
                double dySlope = GaitCore.clamp(-(dhX * m.groundN.x + dhZ * m.groundN.z) / m.groundN.y, -3.0, 3.0);
                if (dySlope >= 0.0 || m.speed < 3.0) {
                    hull = new Vec3(hull.x, hull.y + dySlope, hull.z);
                } else if (m.vel.y < -0.5) {
                    hull = new Vec3(hull.x, hull.y + Math.max(dySlope, m.vel.y * delay - 0.35), hull.z);
                }
            }
        }
        out.mount = hull.add(out.frame.toWorld(l.mountLocal.add(m.visOffset)));
    }

    private static double siblingSepPenalty(Mech m, Leg l, Vec3 p, Vec3 up) {
        double minSep = Math.max(0.36, 2.2 * l.footR);
        double pen = 0.0;
        for (int i = 0; i < m.n; i++) {
            Leg nb = m.legs[i];
            if (nb.side != l.side || nb == l) {
                continue;
            }
            Vec3 q = null;
            if (nb.planted) {
                q = nb.foot;
            } else if (nb.swing.active && nb.swing.hasGround) {
                q = nb.swing.target;
            }
            if (q != null) {
                double d = p.sub(q).projectPlane(up).len();
                if (d < minSep) {
                    pen += 2.5 * (minSep - d);
                }
            }
        }
        return pen;
    }

    private static boolean surfaceOk(Mech m, Vec3 normal, Vec3 point, Leg l) {
        if (m.climbingMode()) {
            return normal.dot(m.frame.up) > -0.2 || normal.dot(l.mountWorld.sub(point)) > 0.05;
        }
        return normal.y > 0.64;
    }

    static boolean reachable(Mech m, Leg l, Vec3 p, Vec3 optHip, Vec3.Frame optFr) {
        Vec3 hip = optHip != null ? optHip : l.mountWorld;
        Vec3.Frame fr = optFr != null ? optFr : m.frame;
        double d = hip.dist(p);
        if (!(l.reach * 0.08 <= d && d <= l.reach * 0.96)) {
            return false;
        }
        return l.ik.feasible(hip, fr, p, 0.0);
    }

    static boolean clearSpot(Mech m, Leg l, Vec3 p, Vec3 up, Vec3 optMount) {
        double r = l.footR;
        Vec3 c = p.add(up.mul(r + 0.03));
        WorldQueryCtx.PushResult pr = m.tmpPush;
        if (m.wq.spherePush(c, r, pr) && pr.center.sub(c).lenSq() > 1e-6) {
            return false;
        }
        double lenLast = l.ik.lengths[l.ik.nSegs - 1];
        Vec3 tgt = p.add(up.mul(Math.max(r + 0.1, 0.55 * lenLast)));
        Vec3 mount = optMount != null ? optMount : l.mountWorld;
        Vec3 dv = tgt.sub(mount);
        double dl = dv.len();
        if (dl > 0.3) {
            WorldQueryCtx.Hit h = m.tmpHit;
            if (m.wq.raycast(mount, dv.mul(1.0 / dl), dl - 0.15, h) && !m.climbingMode()) {
                return false;
            }
        }
        return true;
    }

    /** Trial-solves the leg chain when nearby colliders threaten knee/shin clearance. */
    private static boolean jointsClear(Mech m, Leg l, Vec3 p, Vec3 optMount, Vec3.Frame optFr) {
        Vec3 up = m.probeUp();
        double lastTwo = l.ik.lengths[l.ik.nSegs - 1]
                + (l.ik.nSegs >= 2 ? l.ik.lengths[l.ik.nSegs - 2] : 0.0);
        double R = Math.max(0.95, 0.55 * lastTwo);
        Vec3 c = p.add(up.mul(R + 0.05));
        WorldQueryCtx.PushResult pr = m.tmpPush;
        if (!m.wq.spherePush(c, R, pr) || pr.center.sub(c).lenSq() < 1e-6) {
            return true;
        }

        Vec3 mount = optMount != null ? optMount : l.mountWorld;
        Vec3.Frame fr = optFr != null ? optFr : m.frame;
        Vec3 ml = l.mountLocal.add(m.visOffset);
        Vec3 hull = mount.sub(fr.toWorld(ml));
        l.ikProbe.hasAngles = false;
        l.ikProbe.hasPts = false;
        Vec3[] jl = m.tmpJoints1;
        int nj = l.ikProbe.solve(ml, Vec3.LOCAL_FRAME, fr.toLocal(p.sub(hull)), 0.0, jl);
        double r = l.footR * 0.55;
        Vec3[] pts = m.tmpJoints2;
        for (int i = 0; i < nj; i++) {
            pts[i] = hull.add(fr.toWorld(jl[i]));
        }
        int lastInt = nj - 2;
        for (int jIdx = 1; jIdx <= lastInt; jIdx++) {
            Vec3 cc = pts[jIdx];
            if (m.wq.spherePush(cc, r, pr) && pr.center.sub(cc).lenSq() > 1e-6) {
                if (jIdx == lastInt && pr.normal.dot(up) > 0.96) {
                    double pen = pr.center.sub(cc).len();
                    double surfDy = cc.sub(pr.normal.mul(r - pen)).sub(p).dot(up);
                    if (pen < 0.70 * r && Math.abs(surfDy) < 0.08) {
                        continue;
                    }
                }
                return false;
            }
        }
        WorldQueryCtx.Hit h = m.tmpHit;
        for (int i = 1; i < nj - 1; i++) {
            Vec3 a = pts[i];
            Vec3 b = (i == nj - 2) ? p.add(up.mul(2.0 * l.footR + 0.05)) : pts[i + 1];
            Vec3 dv = b.sub(a);
            double dl = dv.len();
            if (dl > 0.05 && m.wq.raycast(a, dv.mul(1.0 / dl), dl, h)) {
                return false;
            }
        }
        return true;
    }

    static final class ProbeOrigOut {
        Vec3 o = Vec3.ZERO;
        boolean hasCeil;
        double ceil;
    }

    private static void probeOrigin(
            Mech m, Vec3 o, Vec3 up, boolean hasBaseY, double baseY, boolean checkCeil, ProbeOrigOut out) {
        boolean hasLocalCeil = false;
        double localCeil = 0.0;
        WorldQueryCtx.Hit hc = m.tmpHit;
        if (!m.climbingMode()) {
            hasLocalCeil = m.hasCeilY;
            localCeil = m.ceilY;
            double slopeO = 0.0;
            if (0.5 < m.groundN.y && m.groundN.y < 0.995) {
                slopeO = GaitCore.clamp(
                        -((o.x - m.pos.x) * m.groundN.x + (o.z - m.pos.z) * m.groundN.z) / m.groundN.y,
                        -3.0, 3.0);
            }
            double posSlopeY = m.pos.dot(up) + slopeO;
            double startY = hasBaseY ? Math.max(baseY, posSlopeY) : posSlopeY;
            double oy = o.dot(up);
            if ((checkCeil || hasLocalCeil) && oy > startY + 0.05) {
                Vec3 startPt = o.add(up.mul(startY - oy));
                if (m.wq.raycast(startPt, up, (oy - startY) + 0.35, hc) && hc.normal.dot(up) < -0.5) {
                    double cy = hc.point.dot(up);
                    localCeil = hasLocalCeil ? Math.min(localCeil, cy) : cy;
                    hasLocalCeil = true;
                }
            }
            if (hasLocalCeil && oy > localCeil - 0.12) {
                o = o.add(up.mul((localCeil - 0.12) - oy));
            }
        }
        WorldQueryCtx.PushResult pr = m.tmpPush;
        for (int it = 0; it < 4; it++) {
            if (!m.wq.spherePush(o, 0.05, pr) || pr.center.sub(o).lenSq() < 1e-8) {
                out.o = o;
                out.hasCeil = hasLocalCeil;
                out.ceil = localCeil;
                return;
            }
            if (pr.normal.dot(up) < -0.3) {
                out.o = pr.center.sub(up.mul(0.08));
                out.hasCeil = hasLocalCeil;
                out.ceil = localCeil;
                return;
            }
            o = o.add(up.mul(0.6));
            if (hasLocalCeil && o.dot(up) > localCeil - 0.10) {
                o = o.add(up.mul((localCeil - 0.10) - o.dot(up)));
                out.o = o;
                out.hasCeil = hasLocalCeil;
                out.ceil = localCeil;
                return;
            }
        }
        out.o = o;
        out.hasCeil = hasLocalCeil;
        out.ceil = localCeil;
    }

    private static double scoreCand(
            Mech m, Vec3 pt, Vec3 nrm, Vec3 ideal, Vec3 up, double expected,
            boolean obstructed, boolean hasFwd, Vec3 fwd) {
        Vec3 dp = pt.sub(ideal);
        double dv = dp.dot(up);
        double dh = dp.sub(up.mul(dv)).len();
        double dev = pt.dot(up) - expected;
        double sm = m.d.stepMax;
        double hc = (dev >= 0.0) ? (0.8 * Math.max(0.0, dev - sm)) : (0.4 * -dev);
        double sc = dh + hc + 2.0 * (1.0 - nrm.dot(up));
        if (obstructed && dev > 0.06 && dev <= sm) {
            sc -= Math.min(0.28, 0.45 * dev);
        }
        if (hasFwd && m.speed > 0.3) {
            double back = -pt.sub(ideal).dot(fwd);
            if (back > 0.0) {
                sc += 0.18 * back;
            }
        }
        return sc;
    }

    private static void horizAxes(Mech m, Vec3 up, Vec3[] axes) {
        Vec3 t1 = m.frame.fwd.projectPlane(up).norm(new Vec3(0.0, 0.0, 1.0));
        Vec3 t2 = up.cross(t1).norm(new Vec3(1.0, 0.0, 0.0));
        axes[0] = t1;
        axes[1] = t2;
        axes[2] = t1.add(t2).norm(t1);
        axes[3] = t1.sub(t2).norm(t1);
    }

    private static double sheerDropDist(Mech m) {
        return GaitCore.clamp(Math.max(m.ride * 0.78, m.d.stepMax * 1.35), 1.05, 1.55);
    }

    private static boolean probeSheerDrop(Mech m, Vec3 pt, Vec3 up, double refY, double sheerDrop) {
        Vec3 o = pt.add(up.mul(0.45));
        WorldQueryCtx.Hit hh = m.tmpHit2;
        if (!m.wq.raycast(o, up.mul(-1.0), 0.45 + sheerDrop, hh)) {
            return !m.wq.spherePush(o, 0.04, m.tmpPush);
        }
        return (refY - hh.point.dot(up)) > sheerDrop;
    }

    private static boolean sheerOverhang(Mech m, Vec3 pt, Vec3 up, double r) {
        double sheer = sheerDropDist(m);
        double refY = pt.dot(up);
        double rChk = Math.max(r, 0.08);
        Vec3[] axes = m.tmpAxes;
        horizAxes(m, up, axes);
        for (int ai = 0; ai < 4; ai++) {
            if (probeSheerDrop(m, pt.add(axes[ai].mul(rChk)), up, refY, sheer)) {
                return true;
            }
            if (probeSheerDrop(m, pt.sub(axes[ai].mul(rChk)), up, refY, sheer)) {
                return true;
            }
        }
        return false;
    }

    /** Detects narrow beams and thin walls so wide mechs do not perch on knife ridges and stall. */
    private static boolean isThinWall(Mech m, Vec3 pt, Vec3 up) {
        double sheer = sheerDropDist(m);
        double refY = pt.dot(up);
        double r1 = 0.52;
        double r2 = 0.96;
        Vec3[] axes = m.tmpAxes;
        horizAxes(m, up, axes);
        for (int ai = 0; ai < 4; ai++) {
            boolean dPos = probeSheerDrop(m, pt.add(axes[ai].mul(r1)), up, refY, sheer);
            boolean dNeg = probeSheerDrop(m, pt.sub(axes[ai].mul(r1)), up, refY, sheer);
            if (dPos && dNeg) {
                return true;
            }
            if (dPos && probeSheerDrop(m, pt.sub(axes[ai].mul(r2)), up, refY, sheer)) {
                return true;
            }
            if (dNeg && probeSheerDrop(m, pt.add(axes[ai].mul(r2)), up, refY, sheer)) {
                return true;
            }
        }
        return false;
    }

    private static boolean nearEdge(Mech m, Vec3 pt, Vec3 up, double r) {
        Vec3 t1 = m.frame.fwd.projectPlane(up).norm(new Vec3(0.0, 0.0, 1.0));
        Vec3 t2 = up.cross(t1);
        WorldQueryCtx.Hit hh = m.tmpHit2;
        for (int i = 0; i < 4; i++) {
            Vec3 dv = (i == 0) ? t1 : ((i == 1) ? t1.mul(-1.0) : ((i == 2) ? t2 : t2.mul(-1.0)));
            Vec3 o = pt.add(dv.mul(r)).add(up.mul(0.35));
            if (!m.wq.raycast(o, up.mul(-1.0), 0.7, hh)
                    || Math.abs(hh.point.sub(pt).dot(up)) > 0.2) {
                return true;
            }
        }
        return false;
    }

    private static void fhOut(Vec3 point, Vec3 normal, Result out) {
        out.pt = point.add(normal.mul(0.02));
        out.n = normal;
    }

    static boolean findFoothold(Mech m, Leg l, Vec3 ideal, double delay, Result out) {
        MechDef d = m.d;
        WorldQueryCtx wq = m.wq;
        boolean cm = m.climbingMode();
        Vec3 up = cm ? m.frame.up : Vec3.WORLD_UP;

        double slopeRise = 0.0;
        if (!cm && 0.5 < m.groundN.y && m.groundN.y < 0.995) {
            double rawSr = GaitCore.clamp(
                    -((ideal.x - m.pos.x) * m.groundN.x + (ideal.z - m.pos.z) * m.groundN.z) / m.groundN.y,
                    -3.0, 3.0);
            if (rawSr >= 0.0 || m.speed < 3.0 || m.vel.y < -0.5) {
                slopeRise = rawSr;
            }
        }
        double probeUp = (m.state == GaitCore.STATE_MANTLE)
                ? (d.mantleMax + 0.6)
                : (Math.max(d.stepMax, d.stepHeight) * 1.5 + 0.3 + Math.max(0.0, slopeRise));
        double probeDown = m.ride * 1.3 + d.stepMax + 0.5 + Math.max(0.0, -slopeRise);
        Vec3 origin = ideal.add(up.mul(probeUp));

        boolean hasPose = delay > 0.0;
        PoseOut poseOut = m.tmpPoseOut;
        if (hasPose) {
            landingPose(m, l, delay, poseOut);
        }
        Vec3 mount = hasPose ? poseOut.mount : l.mountWorld;
        Vec3 optHip = hasPose ? poseOut.mount : null;
        Vec3.Frame optFr = hasPose ? poseOut.frame : null;

        boolean hasBaseY = !cm;
        double baseY = hasBaseY ? Math.max(mount.dot(up), m.pos.dot(up) + slopeRise) : 0.0;
        boolean hasCeilLim = (!cm && m.hasCeilY);
        double ceilLim = m.ceilY;
        WorldQueryCtx.Hit mHc = m.tmpHit;
        if (!cm) {
            if (wq.raycast(mount, up, probeUp + 1.5, mHc) && mHc.normal.dot(up) < -0.5) {
                double myC = mHc.point.dot(up);
                ceilLim = hasCeilLim ? Math.min(ceilLim, myC) : myC;
                hasCeilLim = true;
            }
        }
        double liftO = mount.sub(origin).dot(up);
        if (liftO > 0.0) {
            origin = origin.add(up.mul(liftO));
        }
        Vec3 los = origin.sub(mount);
        double L = los.len();
        if (L > 1e-4) {
            if (wq.raycast(mount, los.mul(1.0 / L), L, mHc)) {
                if (cm && surfaceOk(m, mHc.normal, mHc.point, l) && reachable(m, l, mHc.point, optHip, optFr)) {
                    fhOut(mHc.point, mHc.normal, out);
                    return true;
                }
                origin = mount.add(los.mul(Math.max(0.0, mHc.t - 0.25) / L));
            }
        }

        double expected = m.pos.sub(up.mul(m.rideEff())).dot(up) + slopeRise;
        Cand[] cands = m.tmpFhCands;
        int nCands = 0;

        ProbeOrigOut poOut = m.tmpProbeOrig;
        probeOrigin(m, origin, up, hasBaseY, baseY, true, poOut);
        origin = poOut.o;
        if (poOut.hasCeil) {
            ceilLim = hasCeilLim ? Math.min(ceilLim, poOut.ceil) : poOut.ceil;
            hasCeilLim = true;
        }

        WorldQueryCtx.Hit h1 = m.tmpHit;
        boolean hasH1 = wq.raycast(origin, up.mul(-1.0), Math.max(1.0, origin.sub(ideal).dot(up) + probeDown), h1);
        if (hasH1 && hasCeilLim && h1.point.dot(up) >= ceilLim - 0.08) {
            hasH1 = false;
        }
        boolean ok1 = hasH1 && surfaceOk(m, h1.normal, h1.point, l);
        boolean ok2 = ok1 && reachable(m, l, h1.point, optHip, optFr);
        boolean ok3 = ok2 && clearSpot(m, l, h1.point, up, mount);

        m.fhCount += 1;
        m.fhBad += 1;
        String why = !hasH1 ? "none" : (!ok1 ? "surface" : (!ok2 ? "reach" : (!ok3 ? "clear" : "ok")));
        m.fhWhy = why;
        if ("reach".equals(why) || "none".equals(why)) {
            m.fhBad -= 1;
        }

        boolean h1Cand = false;
        boolean isNear = false;
        double sepPen = 0.0;
        if (ok3) {
            if (cm) {
                fhOut(h1.point, h1.normal, out);
                return true;
            }
            isNear = nearEdge(m, h1.point, up, l.footR);
            sepPen = siblingSepPenalty(m, l, h1.point, up);
            boolean overhang = sheerOverhang(m, h1.point, up, l.footR);
            boolean badCliff = overhang || isThinWall(m, h1.point, up);
            if (!isNear && !badCliff && sepPen <= 0.25 && jointsClear(m, l, h1.point, optHip, optFr)) {
                m.fhBad -= 1;
                fhOut(h1.point, h1.normal, out);
                return true;
            }
            if (!isNear && !badCliff && sepPen > 0.25) {
                m.fhBad -= 1;
            }
            m.fhWhy = (isNear || badCliff) ? "edge" : (sepPen > 0.25 ? "sibling" : "joints");
            h1Cand = (isNear || sepPen > 0.25) && !badCliff;
        } else if (cm && !hasH1) {
            Vec3 end = origin.sub(up.mul(probeUp + probeDown * 0.6));
            Vec3 tgt = m.pos.sub(up.mul(m.ride));
            Vec3 dv = tgt.sub(end);
            double dl = dv.len();
            if (dl > 1e-4) {
                WorldQueryCtx.Hit h2 = m.tmpHit2;
                if (wq.raycast(end, dv.mul(1.0 / dl), dl + l.reach, h2)
                        && surfaceOk(m, h2.normal, h2.point, l) && reachable(m, l, h2.point, optHip, optFr)) {
                    fhOut(h2.point, h2.normal, out);
                    return true;
                }
            }
        }

        Vec3 t1 = m.frame.fwd.projectPlane(up).norm(new Vec3(0.0, 0.0, 1.0));
        Vec3 t2 = up.cross(t1);
        double zone = d.zone;
        boolean hasFwdD = false;
        Vec3 fwdD = Vec3.ZERO;
        boolean obstructed = false;
        if (!cm) {
            hasFwdD = true;
            fwdD = (m.speed > 0.3) ? m.vt.projectPlane(up).norm(t1) : t1;
            Vec3 obsO = new Vec3(ideal.x, expected + 0.30 * d.stepMax, ideal.z);
            obstructed = wq.raycast(obsO, fwdD, Math.max(0.55, 0.8 * zone), m.tmpHit2);
        }
        if (h1Cand) {
            double sc = scoreCand(m, h1.point, h1.normal, ideal, up, expected, obstructed, hasFwdD, fwdD)
                    + (isNear ? 0.6 : 0.0) + sepPen;
            cands[nCands++].set(sc, h1);
        }

        double nudge = Math.max(0.2, 2.0 * l.footR);
        boolean hasCeil = hasCeilLim;
        double[] radii = { nudge, zone * 0.4, zone * 0.8 };
        double[] ang0 = { GaitCore.PI / 8.0, 0.0, GaitCore.PI / 8.0 };

        for (int ring = 0; ring < 3; ring++) {
            if (ring == 1 && nCands > 0) {
                Arrays.sort(cands, 0, nCands, (a, b) -> Double.compare(a.sc, b.sc));
                int lim = Math.min(nCands, 3);
                for (int ci = 0; ci < lim; ci++) {
                    Cand c = cands[ci];
                    if (c.sc < 0.5 * zone
                            && !nearEdge(m, c.point, up, l.footR)
                            && siblingSepPenalty(m, l, c.point, up) <= 0.25
                            && jointsClear(m, l, c.point, optHip, optFr)
                            && (cm || (!sheerOverhang(m, c.point, up, l.footR) && !isThinWall(m, c.point, up)))) {
                        if (!cm) {
                            m.fhBad -= 1;
                        }
                        fhOut(c.point, c.normal, out);
                        return true;
                    }
                }
            }
            double rad = radii[ring];
            double a0 = ang0[ring];
            for (int k = 0; k < 8; k++) {
                double a = a0 + k * (GaitCore.PI / 4.0);
                Vec3 p = ideal.add(t1.mul(Math.cos(a) * rad)).add(t2.mul(Math.sin(a) * rad));
                probeOrigin(m, p.add(up.mul(probeUp)), up, hasBaseY, baseY, hasCeil, poOut);
                Vec3 o = poOut.o;
                if (!wq.raycast(o, up.mul(-1.0), Math.max(1.0, o.sub(p).dot(up) + probeDown), mHc)) {
                    continue;
                }
                boolean hasCl = poOut.hasCeil || hasCeilLim;
                double cLim = !poOut.hasCeil ? ceilLim : (!hasCeilLim ? poOut.ceil : Math.min(ceilLim, poOut.ceil));
                if ((hasCl && mHc.point.dot(up) >= cLim - 0.08)
                        || !surfaceOk(m, mHc.normal, mHc.point, l)
                        || !reachable(m, l, mHc.point, optHip, optFr)
                        || !clearSpot(m, l, mHc.point, up, mount)) {
                    continue;
                }
                double sp = cm ? 0.0 : siblingSepPenalty(m, l, mHc.point, up);
                double sc = scoreCand(m, mHc.point, mHc.normal, ideal, up, expected, obstructed, hasFwdD, fwdD) + sp;
                cands[nCands++].set(sc, mHc);
            }
        }

        Arrays.sort(cands, 0, nCands, (a, b) -> Double.compare(a.sc, b.sc));
        boolean hasEdgeFb = false;
        Vec3 edgeFbP = Vec3.ZERO;
        Vec3 edgeFbN = Vec3.WORLD_UP;
        int afterFb = 0;
        for (int ci = 0; ci < nCands; ci++) {
            Cand c = cands[ci];
            if (cm) {
                if (jointsClear(m, l, c.point, optHip, optFr)) {
                    fhOut(c.point, c.normal, out);
                    return true;
                }
                continue;
            }
            if (nearEdge(m, c.point, up, l.footR)) {
                if (!hasEdgeFb && jointsClear(m, l, c.point, optHip, optFr)
                        && !sheerOverhang(m, c.point, up, l.footR) && !isThinWall(m, c.point, up)) {
                    hasEdgeFb = true;
                    edgeFbP = c.point;
                    edgeFbN = c.normal;
                }
                if (hasEdgeFb) {
                    afterFb++;
                    if (afterFb >= 6) {
                        break;
                    }
                }
                continue;
            }
            if (jointsClear(m, l, c.point, optHip, optFr)
                    && !sheerOverhang(m, c.point, up, l.footR) && !isThinWall(m, c.point, up)) {
                fhOut(c.point, c.normal, out);
                return true;
            }
        }
        if (hasEdgeFb) {
            fhOut(edgeFbP, edgeFbN, out);
            return true;
        }

        if (!cm) {
            Vec3.Frame frUse = hasPose ? poseOut.frame : m.frame;
            Vec3 rightU = frUse.right.projectPlane(up).norm(t2);
            double latRef = Math.abs(l.mountLocal.x) > 1e-4 ? l.mountLocal.x : l.restXz[0];
            double latSgn = latRef >= 0.0 ? 1.0 : -1.0;
            Vec3 hullC = hasPose ? mount.sub(poseOut.frame.toWorld(l.mountLocal.add(m.visOffset))) : m.pos;
            boolean hasHw = wq.raycast(hullC, rightU.mul(latSgn), Math.abs(l.restXz[0]) + 0.25, mHc);
            boolean wallSide = hasHw && Math.abs(mHc.normal.dot(up)) < 0.55;
            double minLat = wallSide ? 0.18 : Math.max(0.52, 0.52 * Math.abs(l.mountLocal.x));

            Vec3[] bases = m.tmpFhBases;
            int nBases = 1;
            bases[0] = ideal;
            Vec3 rest = !hasPose ? l.idealNow : l.idealNow.add(mount.sub(l.mountWorld));
            if (rest.dist(ideal) > 0.3) {
                bases[nBases++] = rest;
            }

            Vec3[] pts = m.tmpFhPts;
            int nPts = 0;
            double[] fs1 = { 0.38, 0.62, 0.74, 0.85 };
            double[] fs2 = { 0.35, 0.58, 0.82 };
            for (int bi = 0; bi < nBases; bi++) {
                Vec3 base = bases[bi];
                Vec3 hub = hullC.sub(base).projectPlane(up);
                Vec3 lat = t2.mul(hub.dot(t2));
                double fwdSgn = l.restXz[1] >= 0.0 ? 1.0 : -1.0;
                for (int fi = 0; fi < 4; fi++) {
                    Vec3 pt = base.add(lat.mul(fs1[fi]));
                    pts[nPts++] = pt;
                    pts[nPts++] = pt.add(t1.mul(fwdSgn * 0.85));
                    pts[nPts++] = pt.sub(t1.mul(fwdSgn * 0.55));
                }
                for (int fi = 0; fi < 3; fi++) {
                    Vec3 pt = base.add(hub.mul(fs2[fi]));
                    pts[nPts++] = pt;
                    if (fi == 1) {
                        pts[nPts++] = pt.add(t1.mul(0.65));
                        pts[nPts++] = pt.sub(t1.mul(0.65));
                    }
                }
            }

            for (int pi = 0; pi < nPts; pi++) {
                Vec3 p = pts[pi];
                if (p.sub(hullC).projectPlane(up).dot(rightU) * latSgn < minLat) {
                    continue;
                }
                probeOrigin(m, p.add(up.mul(probeUp)), up, hasBaseY, baseY, hasCeil, poOut);
                Vec3 o = poOut.o;
                if (!wq.raycast(o, up.mul(-1.0), Math.max(1.0, o.sub(p).dot(up) + probeDown), mHc)) {
                    continue;
                }
                boolean hasCl = poOut.hasCeil || hasCeilLim;
                double cLim = !poOut.hasCeil ? ceilLim : (!hasCeilLim ? poOut.ceil : Math.min(ceilLim, poOut.ceil));
                if ((!hasCl || mHc.point.dot(up) < cLim - 0.08)
                        && surfaceOk(m, mHc.normal, mHc.point, l)
                        && reachable(m, l, mHc.point, optHip, optFr)
                        && clearSpot(m, l, mHc.point, up, mount)
                        && jointsClear(m, l, mHc.point, optHip, optFr)
                        && !sheerOverhang(m, mHc.point, up, l.footR)
                        && !isThinWall(m, mHc.point, up)) {
                    fhOut(mHc.point, mHc.normal, out);
                    return true;
                }
            }
        }
        return false;
    }

    static Vec3 hangPose(Mech m, Leg l, double ride) {
        Vec3 up = m.probeUp();
        Vec3 hip = l.mountWorld;
        Vec3 rest = l.idealNow;
        Vec3 dv = rest.sub(hip).projectPlane(up);
        double dh = dv.len();
        double R = 0.9 * l.reach;
        if (dh > R) {
            rest = hip.add(dv.mul(R / dh)).sub(up.mul(hip.sub(rest).dot(up)));
            dh = R;
        }
        double y0 = hip.sub(rest).dot(up);
        double t = Math.sqrt(Math.max(0.0, R * R - dh * dh)) - y0;
        WorldQueryCtx.Hit g = m.tmpHit;
        if (m.wq.raycast(rest.add(up.mul(ride * 0.5)), up.mul(-1.0), ride * 0.5 + Math.max(t, 0.0) + 0.5, g)) {
            t = Math.min(t, rest.sub(g.point).dot(up) - l.footR);
        }
        return rest.sub(up.mul(Math.max(t, -ride * 0.3)));
    }

    /** Rejects redundant re-steps when the best found foothold sits right next to the planted foot. */
    static boolean futileStep(Leg l, boolean hasFh, Vec3 fhP) {
        if (hasFh && fhP.dist(l.foot) < 0.12) {
            l.cooldown = Math.max(l.cooldown, 0.25);
            l.force = false;
            return true;
        }
        return false;
    }

    /** Pre-allocated scratch pool element factory for {@link Mech}. */
    static Cand[] allocCandPool() {
        Cand[] arr = new Cand[32];
        for (int i = 0; i < arr.length; i++) {
            arr[i] = new Cand();
        }
        return arr;
    }

    static Vec3[] allocVecPool(int n) {
        Vec3[] arr = new Vec3[n];
        Arrays.fill(arr, Vec3.ZERO);
        return arr;
    }
}
