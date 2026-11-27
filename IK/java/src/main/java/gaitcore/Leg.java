package gaitcore;

import java.util.Arrays;

/**
 * Single-leg runtime state, quintic Hermite swing planning/retargeting, and obstacle clearance.
 * Mirrors {@code gc_leg.c}.
 */
final class Leg {

    static final class Swing {
        boolean active;
        Vec3 startW = Vec3.ZERO;
        Vec3 target = Vec3.ZERO;
        Vec3 normal = Vec3.WORLD_UP;
        double t;
        double dur;
        double lift;
        Vec3 up = Vec3.WORLD_UP;
        boolean hasGround;
        double retargetCd;
        boolean hasIdeal;
        Vec3 ideal = Vec3.ZERO;
        boolean forceRetarget;
        Vec3 origin = Vec3.ZERO;
        Vec3 startVelocity = Vec3.ZERO;
        Vec3 endVelocity = Vec3.ZERO;
        Vec3 bow = Vec3.ZERO;
        Vec3 cruise = Vec3.ZERO;
        double takeoffTime;
        double landingTime;
        double planT;
        double planDur = 1.0;
        boolean hasStartHipL;
        Vec3 startHipL = Vec3.ZERO;
        boolean hasTargetHipL;
        Vec3 targetHipL = Vec3.ZERO;
        double arcW;
        double arcT0;

        void clear() {
            active = false;
            startW = Vec3.ZERO;
            target = Vec3.ZERO;
            normal = Vec3.WORLD_UP;
            t = 0.0;
            dur = 0.0;
            lift = 0.0;
            up = Vec3.WORLD_UP;
            hasGround = false;
            retargetCd = 0.0;
            hasIdeal = false;
            ideal = Vec3.ZERO;
            forceRetarget = false;
            origin = Vec3.ZERO;
            startVelocity = Vec3.ZERO;
            endVelocity = Vec3.ZERO;
            bow = Vec3.ZERO;
            cruise = Vec3.ZERO;
            takeoffTime = 0.0;
            landingTime = 0.0;
            planT = 0.0;
            planDur = 1.0;
            hasStartHipL = false;
            startHipL = Vec3.ZERO;
            hasTargetHipL = false;
            targetHipL = Vec3.ZERO;
            arcW = 0.0;
            arcT0 = 0.0;
        }
    }

    int idx;
    int pair;
    int side;
    Vec3 mountLocal = Vec3.ZERO;
    final double[] restXz = new double[2];
    final LegIK ik = new LegIK();
    final LegIK ikProbe = new LegIK();
    double reach;
    boolean isFront;
    double footR;
    boolean force;
    Vec3 foot = Vec3.ZERO;
    Vec3 footNormal = Vec3.WORLD_UP;
    boolean planted;
    boolean searching;
    double searchCd;
    final Swing swing = new Swing();
    int nJoints;
    final Vec3[] joints = new Vec3[GaitCore.MAX_JOINTS];
    int nJointsLocal;
    final Vec3[] jointsLocal = new Vec3[GaitCore.MAX_JOINTS];
    Vec3 planeNormal = new Vec3(1.0, 0.0, 0.0);
    double offset;
    boolean prevWindow;
    boolean pending;
    double cooldown;
    boolean sliding;
    double stretch;
    double stretchRate;
    double limitT;
    boolean push;
    boolean hasCarry;
    Vec3 carry = Vec3.ZERO;
    double offPath;
    boolean hasSettledAt;
    Vec3 settledAt = Vec3.ZERO;
    boolean hasSettledHullPos;
    Vec3 settledHullPos = Vec3.ZERO;
    boolean hasLandLead;
    Vec3 landLead = Vec3.ZERO;
    double stanceT;
    boolean hasLastPlant;
    Vec3 lastPlantP = Vec3.ZERO;
    Vec3 lastPlantN = Vec3.WORLD_UP;
    double voidMark;
    double displacement;
    Vec3 mountWorld = Vec3.ZERO;
    Vec3 idealNow = Vec3.ZERO;
    Vec3 idealPred = Vec3.ZERO;
    double airTime;
    double planeRoll;
    boolean nearLimit;
    double kcT;

    Leg() {
        Arrays.fill(joints, Vec3.ZERO);
        Arrays.fill(jointsLocal, Vec3.ZERO);
    }

    private static double quinticBlend(double u) {
        double u2 = u * u;
        return u2 * u * (10.0 - 15.0 * u + 6.0 * u2);
    }

    private static double quinticInteg(double u) {
        double u2 = u * u;
        return u2 * u2 * (2.5 - 3.0 * u + u2);
    }

    private static void swingShaping(Swing sw, double t, double dur, Vec3[] outPV) {
        double q = t * (1.0 - t);
        double d = dur > 1e-5 ? dur : 1e-5;
        Vec3 lift = sw.up.mul(sw.lift);
        Vec3 pos = lift.mul(64.0 * q * q * q);
        Vec3 vel = lift.mul(192.0 * q * q * (1.0 - 2.0 * t) / d);
        if (sw.bow.lenSq() > 1e-10) {
            double span = (1.0 - sw.planT) > 1e-5 ? (1.0 - sw.planT) : 1e-5;
            double u = GaitCore.clamp((t - sw.planT) / span, 0.0, 1.0);
            double qb = u * (1.0 - u);
            pos = pos.add(sw.bow.mul(16.0 * qb * qb));
            vel = vel.add(sw.bow.mul(32.0 * qb * (1.0 - 2.0 * u) / (d * span)));
        }
        outPV[0] = pos;
        outPV[1] = vel;
    }

    private static void swingPlan(Swing sw, Vec3 foot, Vec3 velocity, double reach, Vec3 optEndVelocity, Vec3[] tmpPV) {
        sw.planT = sw.t;
        sw.planDur = sw.dur;
        if (optEndVelocity != null) {
            sw.endVelocity = optEndVelocity;
        }
        double duration = Math.max(1e-6, (1.0 - sw.t) * sw.dur);
        swingShaping(sw, sw.t, sw.dur, tmpPV);
        sw.origin = foot.sub(tmpPV[0]);
        sw.startVelocity = velocity.sub(tmpPV[1]);
        Vec3 travel = sw.target.sub(sw.origin);
        double speed = Math.max(travel.len() / duration, 1e-6);
        double startSpd = sw.startVelocity.len();
        double endSpd = sw.endVelocity.len();
        sw.takeoffTime = Math.min(0.46 * duration, GaitCore.TAKEOFF_RAMP_REACH * reach / Math.max(speed, startSpd));
        sw.landingTime = Math.min(0.46 * duration, GaitCore.LANDING_RAMP_REACH * reach / Math.max(speed, endSpd));
        double cruiseTime = Math.max(1e-7, duration - 0.5 * (sw.takeoffTime + sw.landingTime));
        Vec3 num = sw.target.sub(sw.origin)
                .sub(sw.startVelocity.mul(0.5 * sw.takeoffTime))
                .sub(sw.endVelocity.mul(0.5 * sw.landingTime));
        sw.cruise = num.mul(1.0 / cruiseTime);
    }

    private static void swingSample(Swing sw, double t, Vec3[] outPV, Vec3[] tmpPV) {
        if (t >= 1.0) {
            outPV[0] = sw.target;
            outPV[1] = sw.endVelocity;
            return;
        }
        double duration = (1.0 - sw.planT) * sw.planDur;
        double elapsed = Math.max(0.0, (t - sw.planT) * sw.planDur);
        double left = duration - elapsed;
        Vec3 point;
        Vec3 velocity;
        if (sw.takeoffTime > 1e-9 && elapsed < sw.takeoffTime) {
            double u = elapsed / sw.takeoffTime;
            double blend = quinticBlend(u);
            double integ = quinticInteg(u);
            double cruiseW = sw.takeoffTime * integ;
            double initialW = elapsed - cruiseW;
            point = sw.origin.add(sw.startVelocity.mul(initialW)).add(sw.cruise.mul(cruiseW));
            velocity = sw.startVelocity.lerp(sw.cruise, blend);
        } else if (sw.landingTime > 1e-9 && left < sw.landingTime) {
            double u = left / sw.landingTime;
            double blend = quinticBlend(u);
            double integ = quinticInteg(u);
            Vec3 remDisp = sw.endVelocity.mul(left).add(sw.cruise.sub(sw.endVelocity).mul(sw.landingTime * integ));
            point = sw.target.sub(remDisp);
            velocity = sw.endVelocity.lerp(sw.cruise, blend);
        } else {
            double initialW = 0.5 * sw.takeoffTime;
            double cruiseW = elapsed - initialW;
            point = sw.origin.add(sw.startVelocity.mul(initialW)).add(sw.cruise.mul(cruiseW));
            velocity = sw.cruise;
        }
        swingShaping(sw, t, sw.planDur, tmpPV);
        outPV[0] = point.add(tmpPV[0]);
        outPV[1] = velocity.add(tmpPV[1]);
    }

    private final Vec3[] tmpPV1 = new Vec3[2];
    private final Vec3[] tmpPV2 = new Vec3[2];
    private final WorldQueryCtx.Hit tmpHit = new WorldQueryCtx.Hit();
    private final WorldQueryCtx.PushResult tmpPush = new WorldQueryCtx.PushResult();

    void init(int idx, MechDef.LegDef ldef, MechDef d, double size) {
        this.idx = idx;
        this.pair = idx / 2;
        this.side = ldef.side;
        double mx = ldef.mount[0];
        double my = ldef.mount[1];
        double mz = ldef.mount[2];
        double rx = ldef.rest[0];
        double rz = ldef.rest[1];
        this.mountLocal = new Vec3(mx * size, my * size, mz * size);
        this.restXz[0] = rx * size;
        this.restXz[1] = rz * size;
        int[] sides = ldef.hasSides ? ldef.sides : d.sides;
        Vec3 restDir = new Vec3(rx - mx, 0.0, rz - mz).norm(new Vec3(this.side, 0.0, 0.0));
        double[] lengths = new double[GaitCore.MAX_SEGS];
        for (int i = 0; i < d.nSegs; i++) {
            lengths[i] = d.lengths[i] * size;
        }
        MechDef.Limits lim = ldef.hasLimits ? ldef.limits : d.limits;
        ik.init(d.plane, d.coxa * size, lengths, d.nSegs, sides, restDir, lim);
        ikProbe.init(d.plane, d.coxa * size, lengths, d.nSegs, sides, restDir, lim);
        reach = ik.reach;
        isFront = mz > 0.05;
        double wLast = d.widths[d.nSegs - 1];
        footR = Math.min(0.14, wLast * 0.6);
        foot = Vec3.ZERO;
        footNormal = Vec3.WORLD_UP;
        planted = true;
        searching = false;
        searchCd = 0.0;
        swing.clear();
        nJoints = 0;
        nJointsLocal = 0;
        planeNormal = new Vec3(1.0, 0.0, 0.0);
        offset = 0.0;
        prevWindow = false;
        pending = false;
        cooldown = 0.0;
        sliding = false;
        stretch = 0.0;
        stretchRate = 0.0;
        limitT = 0.0;
        push = false;
        hasCarry = false;
        carry = Vec3.ZERO;
        offPath = 0.0;
        hasSettledAt = false;
        settledAt = Vec3.ZERO;
        hasSettledHullPos = false;
        settledHullPos = Vec3.ZERO;
        hasLandLead = false;
        landLead = Vec3.ZERO;
        stanceT = 0.0;
        hasLastPlant = false;
        lastPlantP = Vec3.ZERO;
        lastPlantN = Vec3.WORLD_UP;
        voidMark = 0.0;
        displacement = 0.0;
        mountWorld = Vec3.ZERO;
        idealNow = Vec3.ZERO;
        idealPred = Vec3.ZERO;
        airTime = 0.0;
        planeRoll = 0.0;
        nearLimit = false;
        kcT = 1.0;
    }

    Vec3 restLocal(double ride, double spread) {
        return new Vec3(restXz[0] * spread, -ride, restXz[1] * (1.0 + (spread - 1.0) * 0.5));
    }

    /** Raises swing arc height to clear ledges/rocks along the chord without hitting low ceilings. */
    private void shapeSwing(WorldQueryCtx wq, double clearanceProbe, Vec3 start,
            boolean hasCeilY, double maxCeilY) {
        Swing sw = swing;
        Vec3 p0 = start;
        Vec3 p3 = sw.target;
        Vec3 up = sw.up;
        double h0 = p0.dot(up);
        double h3 = p3.dot(up);
        boolean hasMinCeil = hasCeilY;
        double minCeil = maxCeilY;
        double lift = sw.lift;
        if (clearanceProbe > 0.0) {
            Vec3 d = p3.sub(p0);
            Vec3 sideVec = d.cross(up).norm(new Vec3(1.0, 0.0, 0.0)).mul(footR);
            double dLen = d.len();
            boolean chordClear = (h0 >= h3 - 0.08 && dLen > 1e-4);
            if (chordClear) {
                Vec3 dh = d.sub(up.mul(h3 - h0));
                double dhLen = dh.len();
                if (dhLen > 1e-4) {
                    double baseY = Math.min(h0, h3) + 0.06;
                    Vec3 p0Low = p0.add(up.mul(baseY - h0));
                    Vec3 dirH = dh.mul(1.0 / dhLen);
                    for (int oi = 0; oi < 3; oi++) {
                        Vec3 off = oi == 0 ? Vec3.ZERO : (oi == 1 ? sideVec : sideVec.mul(-1.0));
                        if (wq.raycast(p0Low.add(off), dirH, dhLen, tmpHit)) {
                            chordClear = false;
                            break;
                        }
                    }
                    if (chordClear) {
                        double kneeY = Math.min(h0, h3) + Math.max(0.35, 0.55 * footR + 0.28);
                        if ((!hasMinCeil || kneeY < minCeil - 0.08)
                                && wq.raycast(p0.add(up.mul(kneeY - h0)), dirH, dhLen, tmpHit)) {
                            chordClear = false;
                        }
                    }
                }
                if (chordClear && Math.abs(h0 - h3) >= 0.08) {
                    if (wq.raycast(p0.add(up.mul(0.05)), d.mul(1.0 / dLen), dLen, tmpHit)) {
                        chordClear = false;
                    }
                }
            }
            if (!chordClear) {
                double[] ss = { 0.15, 0.27, 0.39, 0.5, 0.61, 0.73, 0.85 };
                for (int si = 0; si < 7; si++) {
                    double s = ss[si];
                    Vec3 base = p0.lerp(p3, s);
                    for (int oi = 0; oi < 3; oi++) {
                        Vec3 off = oi == 0 ? Vec3.ZERO : (oi == 1 ? sideVec : sideVec.mul(-1.0));
                        Vec3 q = base.add(off);
                        double qy = q.dot(up);
                        double cp = clearanceProbe;
                        if (hasCeilY) {
                            cp = Math.min(cp, Math.max(0.0, maxCeilY - qy - 0.08));
                        }
                        if (cp > 0.04) {
                            if (wq.raycast(q.add(up.mul(footR + 0.04)), up, cp, tmpHit)) {
                                double cy = tmpHit.point.dot(up);
                                minCeil = hasMinCeil ? Math.min(minCeil, cy) : cy;
                                hasMinCeil = true;
                                cp = Math.min(cp, Math.max(0.0, cy - qy - 0.06));
                            }
                        }
                        if (cp <= 0.02) {
                            continue;
                        }
                        Vec3 o = q.add(up.mul(cp));
                        if (wq.raycast(o, up.mul(-1.0), cp + 0.5, tmpHit) && tmpHit.normal.dot(up) > 0.45) {
                            double above = tmpHit.point.dot(up) - (h0 + (h3 - h0) * s);
                            if (above > 0.0 && above <= clearanceProbe) {
                                lift = Math.max(lift, above + 0.35 * sw.lift + footR + 0.08);
                            }
                        }
                    }
                }
            } else if (!hasCeilY) {
                Vec3 midQ = p0.lerp(p3, 0.5);
                if (wq.raycast(midQ.add(up.mul(footR + 0.04)), up, lift + 0.35, tmpHit)) {
                    minCeil = tmpHit.point.dot(up);
                    hasMinCeil = true;
                }
            }
        }
        if (hasMinCeil) {
            lift = Math.min(lift, Math.max(0.03, minCeil - Math.max(h0, h3) - footR - 0.12));
        }
        sw.lift = lift;
    }

    void beginSwing(
            Vec3 target, Vec3 normal, double dur, double lift, Vec3 up,
            boolean hasGround, WorldQueryCtx wq, double clearanceProbe,
            Vec3 startVel, Vec3 endVel, Vec3 bow, boolean hasCeilY, double maxCeilY) {
        swing.clear();
        swing.active = true;
        swing.startW = foot;
        swing.target = target;
        swing.normal = normal;
        swing.dur = dur > 0.05 ? dur : 0.05;
        swing.planDur = 1.0;
        swing.up = up;
        swing.hasGround = hasGround;
        swing.retargetCd = 0.08;
        swing.lift = lift;
        swing.bow = bow;
        swing.endVelocity = endVel;
        planted = false;
        searching = false;
        sliding = false;
        shapeSwing(wq, clearanceProbe, foot, hasCeilY, maxCeilY);
        swingPlan(swing, foot, startVel, reach, swing.endVelocity, tmpPV1);
    }

    void curveWorld(int nPts, Vec3[] outPts) {
        outPts[0] = foot;
        if (!swing.active) {
            return;
        }
        for (int k = 1; k <= nPts; k++) {
            double t = swing.t + (1.0 - swing.t) * k / nPts;
            swingSample(swing, t, tmpPV1, tmpPV2);
            outPts[k] = tmpPV1[0];
        }
    }

    boolean updateSwing(double dt) {
        if (swing.planDur != swing.dur) {
            swingSample(swing, swing.t, tmpPV1, tmpPV2);
            swing.bow = swing.bow.mul(Math.max(0.0, 1.0 - swing.t));
            swingPlan(swing, foot, tmpPV1[1], reach, null, tmpPV2);
        }
        swing.t = Math.min(1.0, swing.t + dt / swing.dur);
        swingSample(swing, swing.t, tmpPV1, tmpPV2);
        foot = tmpPV1[0];
        if (swing.t >= 1.0) {
            foot = swing.target;
            boolean hg = swing.hasGround;
            Vec3 n = swing.normal;
            swing.active = false;
            if (hg) {
                planted = true;
                footNormal = n;
                return true;
            }
            planted = false;
            searching = true;
            searchCd = 0.0;
        }
        return false;
    }

    /**
     * Keeps a swinging foot from clipping obstacles, ledges, or ceilings via sphere extraction and a
     * per-step-clamped downward ground probe.
     */
    double keepClear(WorldQueryCtx wq, Vec3 up, double maxStep, boolean hasCeilY, double maxCeilY, double dt) {
        Swing sw = swing.active ? swing : null;
        double r = footR;
        boolean embedded = false;
        double needUp = 0.0;
        Vec3 c = foot.add(up.mul(r));
        if (wq.spherePush(c, r, tmpPush)) {
            Vec3 push = tmpPush.center.sub(c);
            if (push.lenSq() > 1e-8) {
                embedded = true;
                double pUp = push.dot(up);
                Vec3 pLat = push.sub(up.mul(pUp));
                if ((sw == null || sw.t < 0.96) && pLat.lenSq() > 1e-8) {
                    foot = foot.add(pLat);
                }
                if (pUp < -1e-4) {
                    foot = foot.add(up.mul(pUp));
                } else if (pUp > needUp) {
                    needUp = pUp;
                }
            }
        }
        double stepLim = maxStep > 0.0 ? maxStep : 0.8;
        double probeUp = embedded ? Math.min(0.85, stepLim * 0.75 + 0.15) : (r + 0.28);
        double fy = foot.dot(up);
        boolean hasLocalCeil = hasCeilY;
        double localCeil = maxCeilY;
        boolean runRay = true;
        if (sw != null) {
            kcT += dt;
            runRay = (kcT >= 1.0 / 30.0 - 1e-6) || embedded || sw.t > 0.88;
        }
        if (runRay) {
            kcT = 0.0;
            if (hasCeilY || embedded) {
                if (wq.raycast(foot.add(up.mul(r + 0.02)), up, probeUp + 0.25, tmpHit)) {
                    double cy = tmpHit.point.dot(up);
                    localCeil = hasLocalCeil ? Math.min(localCeil, cy) : cy;
                    hasLocalCeil = true;
                }
            }
            if (hasLocalCeil) {
                probeUp = Math.min(probeUp, Math.max(0.0, localCeil - fy - 0.06));
            }
            if (probeUp > r * 0.5) {
                if (wq.raycast(foot.add(up.mul(probeUp)), up.mul(-1.0), probeUp + r + 0.04, tmpHit)
                        && tmpHit.normal.dot(up) > 0.50) {
                    double bell = sw != null ? (4.0 * sw.t * (1.0 - sw.t)) : 0.0;
                    double gap = sw == null ? r : (0.02 + (r - 0.02) * bell * bell);
                    double upClear = Math.max(0.0, tmpHit.point.sub(foot).dot(up) + gap);
                    if (upClear > needUp) {
                        needUp = upClear;
                    }
                }
            }
        }
        if (hasLocalCeil) {
            needUp = Math.min(needUp, Math.max(0.0, localCeil - fy - r - 0.06));
        }
        double upCap;
        if (embedded) {
            upCap = Math.min(stepLim + 0.25, Math.max(0.32, 22.0 * dt));
        } else {
            upCap = Math.min(Math.min(0.45, stepLim * 0.6), Math.max(0.22, 16.0 * dt));
        }
        if (needUp > upCap) {
            needUp = upCap;
        }
        if (needUp > 0.0) {
            foot = foot.add(up.mul(needUp));
        }
        return needUp;
    }

    /** Re-anchors remaining swing curve onto a new landing spot while preserving current velocity. */
    void retarget(
            Vec3 target, Vec3 normal, boolean hasGround, WorldQueryCtx wq,
            double clearanceProbe, Vec3 endVel, Vec3 bow, boolean hasCeilY, double maxCeilY) {
        swingSample(swing, swing.t, tmpPV1, tmpPV2);
        Vec3 velocity = tmpPV1[1];
        swing.target = target;
        swing.normal = normal;
        swing.hasGround = hasGround;
        swing.bow = bow;
        swing.endVelocity = endVel;
        shapeSwing(wq, clearanceProbe, foot, hasCeilY, maxCeilY);
        swingPlan(swing, foot, velocity, reach, swing.endVelocity, tmpPV2);
    }

    void solve(Mech m, Vec3 mountL, double dt) {
        Vec3.Frame fr = m.frame;
        Vec3 pos = m.pos;
        Vec3 r = fr.right;
        Vec3 u = fr.up;
        Vec3 f = fr.fwd;
        double dx = foot.x - pos.x;
        double dy = foot.y - pos.y;
        double dz = foot.z - pos.z;
        Vec3 footL = new Vec3(
                dx * r.x + dy * r.y + dz * r.z,
                dx * u.x + dy * u.y + dz * u.z,
                dx * f.x + dy * f.y + dz * f.z);
        int nj = ik.solve(mountL, Vec3.LOCAL_FRAME, footL, planeRoll, jointsLocal);
        nJointsLocal = nj;
        nJoints = nj;
        for (int i = 0; i < nj; i++) {
            Vec3 j = jointsLocal[i];
            joints[i] = new Vec3(
                    pos.x + r.x * j.x + u.x * j.y + f.x * j.z,
                    pos.y + r.y * j.x + u.y * j.y + f.y * j.z,
                    pos.z + r.z * j.x + u.z * j.y + f.z * j.z);
        }
        Vec3 pn = ik.planeNormal;
        planeNormal = new Vec3(
                -(r.x * pn.x + u.x * pn.y + f.x * pn.z),
                -(r.y * pn.x + u.y * pn.y + f.y * pn.z),
                -(r.z * pn.x + u.z * pn.y + f.z * pn.z));
        mountWorld = joints[0];
        double prev = stretch;
        stretch = mountWorld.dist(foot) / reach;
        if (ik.err > 0.01) {
            if (sliding) {
                foot = joints[nj - 1];
                stretch = mountWorld.dist(foot) / reach;
            } else {
                double alt = 0.97 + 2.0 * ik.err / reach;
                if (alt > stretch) {
                    stretch = alt;
                }
            }
        }
        nearLimit = ik.limitMarginDeg < 6.0 || ik.coxaRatio > 0.88;
        if (planted && stanceT > 0.0) {
            double rate = (stretch - prev) / Math.max(dt, 1e-4);
            stretchRate += (rate - stretchRate) * GaitCore.damp(26.0, dt);
        } else {
            stretchRate = 0.0;
        }
    }

    /** Reprojects cached local joint positions into current body transform when decoupled IK skips a tick. */
    void reprojectCached(Mech m) {
        int nj = nJointsLocal;
        if (nj <= 0) {
            return;
        }
        Vec3.Frame fr = m.frame;
        Vec3 pos = m.pos;
        Vec3 r = fr.right;
        Vec3 u = fr.up;
        Vec3 f = fr.fwd;
        Vec3 ml = mountLocal.add(m.visOffset);
        Vec3 p0 = jointsLocal[0];
        double dx = ml.x - p0.x;
        double dy = ml.y - p0.y;
        double dz = ml.z - p0.z;
        nJoints = nj;
        for (int i = 0; i < nj; i++) {
            Vec3 j = jointsLocal[i];
            double jx = j.x + dx;
            double jy = j.y + dy;
            double jz = j.z + dz;
            joints[i] = new Vec3(
                    pos.x + r.x * jx + u.x * jy + f.x * jz,
                    pos.y + r.y * jx + u.y * jy + f.y * jz,
                    pos.z + r.z * jx + u.z * jy + f.z * jz);
        }
        mountWorld = joints[0];
        stretch = mountWorld.dist(foot) / reach;
    }
}
