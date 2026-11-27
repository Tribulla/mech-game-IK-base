package gaitcore;

import java.util.Arrays;

/**
 * Body-level locomotion controller: drive physics, jump/mantle/air state machine, quick-slide
 * momentum, hull orientation, and packed pose export. Mirrors {@code gc_mech.c}.
 */
final class Mech {

    static final class Input {
        double fwd, strafe, turn, run, slow, crouch, jump, drift, climb, impact, gait, aim;

        void set(float[] vals, int base) {
            for (int i = 0; i < 12; i++) {
                double x = vals[base + i];
                if (!Double.isFinite(x)) {
                    x = 0.0;
                } else if (i < 3) {
                    x = GaitCore.clamp(x, -1.0, 1.0);
                } else if (i == 11) {
                    x = GaitCore.clamp(x, -GaitCore.AIM_MAX, GaitCore.AIM_MAX);
                }
                switch (i) {
                    case 0 -> fwd = x;
                    case 1 -> strafe = x;
                    case 2 -> turn = x;
                    case 3 -> run = x;
                    case 4 -> slow = x;
                    case 5 -> crouch = x;
                    case 6 -> jump = x;
                    case 7 -> drift = x;
                    case 8 -> climb = x;
                    case 9 -> impact = x;
                    case 10 -> gait = x;
                    case 11 -> aim = x;
                    default -> { }
                }
            }
        }

        void copyFrom(Input o) {
            fwd = o.fwd;
            strafe = o.strafe;
            turn = o.turn;
            run = o.run;
            slow = o.slow;
            crouch = o.crouch;
            jump = o.jump;
            drift = o.drift;
            climb = o.climb;
            impact = o.impact;
            gait = o.gait;
            aim = o.aim;
        }
    }

    static final class Mantle {
        boolean active;
        Vec3 p0 = Vec3.ZERO;
        Vec3 p2 = Vec3.ZERO;
        double t;
        double dur;
        double height;
        Vec3 top = Vec3.ZERO;
        Vec3 fwd = Vec3.ZERO;
        boolean wire;
        boolean landed;
    }

    static final class Impact {
        boolean valid;
        double t;
        Vec3 point = Vec3.ZERO;
        Vec3 normal = Vec3.WORLD_UP;
        double speed;
    }

    final MechDef d;
    final double size;
    final WorldQueryCtx wq;

    float[] eventData = new float[0];
    int eventCount;

    final Vec3.Frame frame = new Vec3.Frame();
    final double ride;
    Vec3 pos;
    Vec3 vel = Vec3.ZERO;
    Vec3 angVel = Vec3.ZERO;
    double yawRate;
    final Leg[] legs = new Leg[GaitCore.MAX_LEGS];
    final int n;
    final boolean biped;
    final double reachAvg;
    final double chordAvg;
    final double vWalk;
    final double vSlow;
    final double jumpSinkMax;
    double lookPitchDn;
    double lookPitchUp;
    double lookPitch;
    final int jointsPerLeg;

    final Input inp = new Input();
    final Input prev = new Input();
    int mode;
    int gaitOverride = -1;
    int gaitIdx;
    final GaitSelector.Blend blend = new GaitSelector.Blend();
    final Gait gait = new Gait();
    final GaitSelector.Situation situation = new GaitSelector.Situation();
    final GaitSelector selector = new GaitSelector();
    boolean hasSpeedCap;
    double speedCap;
    double vDesired;
    double lateralRatio;
    Vec3 vCmd = Vec3.ZERO;
    Vec3 driveTarget = Vec3.ZERO;
    double driveAccel;
    Vec3 accelSmooth = Vec3.ZERO;
    boolean calmGait;
    double footRms;
    int fhCount;
    int fhBad;
    String fhWhy = "";
    double duty;
    double phase;
    double cycle;
    double tAhead;

    int state;
    double stateT;
    boolean crouchToggle;
    double crouch;
    boolean climbEnabled;
    boolean drifting;
    double driftT;
    double driftAmt;
    Vec3 slideDir = new Vec3(0.0, 0.0, 1.0);
    double slideCd;
    double slideBuf;
    boolean slideLock;
    double unsupportedT;
    boolean airBelly;
    double voidT;
    double flightT;
    double flightDur;
    double flightH;
    double flightY0;
    double flightHold;
    int flightCount;
    Vec3 slideVel = Vec3.ZERO;
    double absorb;
    double jumpSink;
    boolean braking;
    boolean hasBalanceOut;
    Vec3 balanceOut = Vec3.ZERO;
    double balanceExcess;
    double balanceH;
    double unbalT;
    double jumpSinkCap;
    double jumpR0;
    double jumpY0;
    int jumpNpl;
    double brace;
    final Impact impact = new Impact();
    int prevNpl;
    boolean trailingSupport;
    double detachT;
    boolean hasPlaneNLocal;
    Vec3 planeNLocal = Vec3.WORLD_UP;
    Vec3 groundN = Vec3.WORLD_UP;
    double landRightT;
    Vec3 landRightN = Vec3.WORLD_UP;
    double support = 1.0;
    final Vec3.Vec2[] hull = new Vec3.Vec2[GaitCore.MAX_HULL_PTS];
    int hullN;
    Vec3 visOffset = Vec3.ZERO;
    final Mantle mantle = new Mantle();
    boolean onWall;
    Vec3 lastAccel = Vec3.ZERO;
    double speed;
    Vec3 vt = Vec3.ZERO;
    long rngState;
    double sparkT;
    double ceilT = 1.0;
    boolean hasCeilY;
    double ceilY;
    double pitchCap = 1.0;
    double pitchStrainT;
    double supT = 1.0;
    boolean hasSupH;
    final WorldQueryCtx.Hit supH = new WorldQueryCtx.Hit();
    boolean hasSupHa;
    final WorldQueryCtx.Hit supHa = new WorldQueryCtx.Hit();
    double supHaSlopeDz;
    double mantleT = 0.05;

    double ikSolveHz;
    double probePeriod = 0.05;
    double speedScale = 1.0;
    boolean planeRollEnabled = true;
    double ikAcc;
    boolean profileEnabled;
    final double[] prof = new double[10];
    double stepMs;

    final double colR;
    final int nColOffs;
    final double[] colOffs = new double[GaitCore.MAX_COL_OFFS];
    final double colLat;

    // Pre-allocated scratch shared across worker-local subsystems (zero GC pressure).
    final WorldQueryCtx.PushResult tmpPush = new WorldQueryCtx.PushResult();
    final WorldQueryCtx.Hit tmpHit = new WorldQueryCtx.Hit();
    final WorldQueryCtx.Hit tmpHit2 = new WorldQueryCtx.Hit();
    final Vec3[] tmpJoints1 = Foothold.allocVecPool(GaitCore.MAX_JOINTS);
    final Vec3[] tmpJoints2 = Foothold.allocVecPool(GaitCore.MAX_JOINTS);
    final boolean[] tmpBool = new boolean[1];
    final Vec3.Frame tmpFrame = new Vec3.Frame();
    final Foothold.PoseOut tmpPoseOut = new Foothold.PoseOut();
    final Foothold.ProbeOrigOut tmpProbeOrig = new Foothold.ProbeOrigOut();
    final Foothold.Cand[] tmpFhCands = Foothold.allocCandPool();
    final Vec3[] tmpFhBases = Foothold.allocVecPool(2);
    final Vec3[] tmpFhPts = Foothold.allocVecPool(64);
    final Vec3[] tmpAxes = Foothold.allocVecPool(4);
    final Foothold.Result tmpFhOut = new Foothold.Result();
    final Leg[] tmpLegs1 = new Leg[GaitCore.MAX_LEGS];
    final Leg[] tmpLegs2 = new Leg[GaitCore.MAX_LEGS];
    final Vec3.Vec2[] tmpVec2A = new Vec3.Vec2[GaitCore.MAX_HULL_PTS];
    final Vec3.Vec2[] tmpVec2B = new Vec3.Vec2[GaitCore.MAX_HULL_PTS];
    final Scheduler.PredCtx tmpPredCtx = new Scheduler.PredCtx();
    final int[] tmpOrder = new int[GaitCore.MAX_LEGS];
    final double[] tmpKeyVal = new double[GaitCore.MAX_LEGS];
    final int[] tmpKeyPri = new int[GaitCore.MAX_LEGS];
    final double[] tmpRems = new double[GaitCore.MAX_LEGS];
    final boolean[] tmpGroupFh = new boolean[GaitCore.MAX_LEGS];
    final Vec3[] tmpGroupPts = Foothold.allocVecPool(GaitCore.MAX_LEGS);
    final Vec3[] tmpGroupNorms = Foothold.allocVecPool(GaitCore.MAX_LEGS);
    final double[] tmpVUp = new double[1];
    final GaitSelector.SelectOut tmpSelOut = new GaitSelector.SelectOut();
    final Vec3[] supPts = Foothold.allocVecPool(GaitCore.MAX_LEGS);
    final double[] supWts = new double[GaitCore.MAX_LEGS];
    final Vec3[] supLocal = Foothold.allocVecPool(GaitCore.MAX_LEGS);
    final Vec3.PlaneFit supFit = new Vec3.PlaneFit();
    final Vec3.Vec2[] balPts2 = new Vec3.Vec2[GaitCore.MAX_HULL_PTS];
    final double[] balYs = new double[GaitCore.MAX_HULL_PTS];
    final boolean[] tmpPushing = new boolean[GaitCore.MAX_LEGS];
    final Vec3[] tmpCurvePts = Foothold.allocVecPool(13);

    private double rngUniform(double lo, double hi) {
        long x = rngState;
        x ^= x >>> 12;
        x ^= x << 25;
        x ^= x >>> 27;
        rngState = x;
        long r = x * 0x2545F4914F6CDD1DL;
        double u = (r >>> 11) * (1.0 / 9007199254740992.0);
        return lo + (hi - lo) * u;
    }

    boolean climbingMode() {
        return d.climber && climbEnabled;
    }

    Vec3 probeUp() {
        return climbingMode() ? frame.up : Vec3.WORLD_UP;
    }

    Vec3 headingFwd() {
        Vec3 f = frame.fwd;
        double fx, fz;
        if (Math.abs(f.y) > 0.7) {
            Vec3 r = frame.right;
            fx = r.z;
            fz = -r.x;
        } else {
            fx = f.x;
            fz = f.z;
        }
        double l2 = fx * fx + fz * fz;
        if (l2 < 1e-18) {
            return new Vec3(0.0, 0.0, 1.0);
        }
        double inv = 1.0 / Math.sqrt(l2);
        return new Vec3(fx * inv, 0.0, fz * inv);
    }

    Vec3.Frame supportFrame() {
        if (climbingMode()) {
            return frame;
        }
        Vec3 f = headingFwd();
        tmpFrame.fwd = f;
        tmpFrame.up = Vec3.WORLD_UP;
        tmpFrame.right = new Vec3(-f.z, 0.0, f.x);
        tmpFrame.isLocal = false;
        return tmpFrame;
    }

    double rideEff() {
        double slideDrop = Math.min(0.26, 0.72 * jumpSinkMax) * driftAmt;
        double r = ride * (1.0 - Math.max(0.38 * crouch, slideDrop));
        if (mode == GaitCore.MODE_SLOW) {
            r *= 0.9;
        }
        if (state == GaitCore.STATE_CROUCHJUMP) {
            r *= (1.0 - jumpSink);
        }
        r *= (1.0 - absorb) * (1.0 - 0.15 * brace);
        return r;
    }

    boolean wireActive() {
        return state == GaitCore.STATE_MANTLE && mantle.active && mantle.wire && !mantle.landed;
    }

    void emit(int kind, Vec3 p, Vec3 n, double strength, Vec3 extra) {
        int needed = (eventCount + 1) * GaitCore.EVENT_FLOATS;
        if (needed > eventData.length) {
            int nc = eventData.length > 0 ? eventData.length * 2 : 64 * GaitCore.EVENT_FLOATS;
            while (nc < needed) {
                nc *= 2;
            }
            eventData = Arrays.copyOf(eventData, nc);
        }
        int base = eventCount * GaitCore.EVENT_FLOATS;
        eventData[base] = (float) kind;
        eventData[base + 1] = (float) p.x;
        eventData[base + 2] = (float) p.y;
        eventData[base + 3] = (float) p.z;
        eventData[base + 4] = (float) n.x;
        eventData[base + 5] = (float) n.y;
        eventData[base + 6] = (float) n.z;
        eventData[base + 7] = (float) strength;
        eventData[base + 8] = (float) extra.x;
        eventData[base + 9] = (float) extra.y;
        eventData[base + 10] = (float) extra.z;
        eventCount++;
    }

    private double foldRide() {
        double lowest = ride;
        for (int k = 1; k < 24; k++) {
            double r = ride * (1.0 - 0.025 * k);
            for (int i = 0; i < n; i++) {
                Leg l = legs[i];
                l.ikProbe.solve(l.mountLocal, Vec3.LOCAL_FRAME, l.restLocal(r, 1.0), 0.0, tmpJoints1);
                if (l.ikProbe.err > 0.01) {
                    return lowest;
                }
            }
            lowest = r;
        }
        return lowest;
    }

    private void computePitchLimits() {
        double cap = d.lookTilt;
        double hy = d.bodyHalf[1];
        double hz = d.bodyHalf[2];
        Vec3.Frame level = new Vec3.Frame(new Vec3(0.0, 0.0, 1.0), Vec3.WORLD_UP);
        Vec3.Frame fr = new Vec3.Frame();
        double[] res = { 0.0, 0.0 };
        int maxDeg = (int) Math.ceil(cap);
        for (int si = 0; si < 2; si++) {
            double sgn = si == 0 ? -1.0 : 1.0;
            double best = 0.0;
            for (int deg = 1; deg <= maxDeg; deg++) {
                double a = Math.min(deg, cap);
                double rad = a * sgn * GaitCore.DEG2RAD;
                Vec3 up = Vec3.WORLD_UP.rotateAxis(level.right, rad);
                fr.init(new Vec3(0.0, 0.0, 1.0), up);
                double rTest = Math.max(ride, hz * Math.abs(fr.fwd.dot(Vec3.WORLD_UP))
                        + hy * Math.abs(fr.up.dot(Vec3.WORLD_UP)) + 0.06);
                boolean ok = true;
                for (int ri = 0; ri < 2 && ok; ri++) {
                    double rt = ri == 0 ? rTest : rTest * 0.92;
                    for (int li = 0; li < n; li++) {
                        Leg l = legs[li];
                        Vec3 footL = fr.toLocal(level.toWorld(l.restLocal(rt, 1.0)));
                        if (footL.dist(l.mountLocal) > 0.91 * l.reach
                                || !l.ikProbe.feasible(l.mountLocal, Vec3.LOCAL_FRAME, footL, 6.0)) {
                            ok = false;
                            break;
                        }
                    }
                }
                if (!ok) {
                    break;
                }
                best = a;
            }
            res[si] = best * GaitCore.DEG2RAD;
        }
        lookPitchDn = res[0];
        lookPitchUp = res[1];
    }

    private double sinkRoom() {
        double r0 = jumpR0;
        Vec3.Frame fr = frame;
        double dropped = Math.max(0.0, jumpY0 - pos.dot(probeUp()));
        double step = 0.05 * ride;
        double room = Math.max(0.0, jumpSinkMax * r0 - dropped);
        for (int i = 0; i < n; i++) {
            Leg l = legs[i];
            if (!l.planted) {
                continue;
            }
            Vec3 fl = fr.toLocal(l.foot.sub(pos));
            double dv = 0.0;
            while (dv + step <= room) {
                l.ikProbe.solve(l.mountLocal, Vec3.LOCAL_FRAME, fl.add(new Vec3(0.0, dv + step, 0.0)), 0.0, tmpJoints1);
                if (l.ikProbe.err > 0.01) {
                    break;
                }
                dv += step;
            }
            if (dv < room) {
                room = dv;
            }
        }
        return (dropped + 0.85 * room) / r0;
    }

    private double crouchTime() {
        return (0.12 + 0.01 * d.jumpSpeed) * Math.sqrt(size);
    }

    private double targetSpeed() {
        double sp = (mode == GaitCore.MODE_SLOW) ? d.speedSlow
                : (mode == GaitCore.MODE_RUN ? d.speedRun : d.speedWalk);
        return sp * speedScale;
    }

    private double crouchCap() {
        return 0.5 * d.speedSlow;
    }

    private void placeFeetInitial() {
        for (int i = 0; i < n; i++) {
            Leg l = legs[i];
            l.mountWorld = pos.add(frame.toWorld(l.mountLocal));
            l.idealNow = pos.add(frame.toWorld(l.restLocal(ride, 1.0)));
            l.idealPred = l.idealNow;
            if (Foothold.findFoothold(this, l, l.idealNow, 0.0, tmpFhOut)) {
                l.foot = tmpFhOut.pt;
                l.footNormal = tmpFhOut.n;
                l.planted = true;
            } else {
                l.foot = l.idealNow;
                l.planted = false;
                l.searching = true;
            }
            Clearance.solveLeg(this, l, 1.0 / 60.0);
        }
    }

    Mech(MechDef d, Vec3 groundPos, double yaw, GcWorldCallbacks cb) {
        this.d = d;
        this.size = d.size;
        this.wq = new WorldQueryCtx(cb);
        this.frame.init(new Vec3(Math.sin(yaw), 0.0, Math.cos(yaw)), Vec3.WORLD_UP);
        this.ride = d.ride;
        this.pos = new Vec3(groundPos.x, groundPos.y + ride, groundPos.z);
        this.n = d.nLegs;
        this.biped = d.biped;

        double reachSum = 0.0;
        double chordSum = 0.0;
        for (int i = 0; i < n; i++) {
            Leg l = new Leg();
            l.init(i, d.legs[i], d, 1.0);
            legs[i] = l;
            reachSum += l.reach;
            double c = 1.15 * l.reach;
            if (l.ik.mode == GaitCore.PLANE_RADIAL && l.ik.coxaMax < 1.5) {
                double rr = Math.hypot(l.restXz[0] - l.mountLocal.x, l.restXz[1] - l.mountLocal.z);
                c = Math.min(c, 1.85 * rr * Math.sin(l.ik.coxaMax));
            }
            chordSum += c;
        }
        this.reachAvg = reachSum / n;
        this.chordAvg = chordSum / n;

        Gait wg = d.gaits[d.gaitWalkIdx];
        this.vWalk = reachAvg / Math.max(0.05, wg.duty * wg.cycleMin * Math.sqrt(size));
        Gait sg = d.gaits[d.gaitSlowIdx];
        this.vSlow = reachAvg / Math.max(0.05, sg.duty * sg.cycleMin * Math.sqrt(size));
        this.jumpSinkMax = Math.min(0.45, Math.max(0.0, 1.0 - 1.06 * foldRide() / ride));
        computePitchLimits();

        this.jointsPerLeg = 1 + (d.coxa > 0.0 ? 1 : 0) + d.nSegs;
        this.mode = GaitCore.MODE_WALK;
        this.gaitOverride = -1;
        this.gaitIdx = d.gaitWalkIdx;
        this.blend.init(gaitIdx, d.gaits[gaitIdx]);
        this.gait.copyFrom(blend.eff);
        this.situation.init();
        this.selector.init(this);
        this.duty = gait.duty;
        for (int i = 0; i < n; i++) {
            legs[i].offset = gait.offsets[i];
        }
        this.cycle = 1.0;
        this.state = GaitCore.STATE_GROUND;
        this.climbEnabled = d.climber;
        this.rngState = (d.kindId + 1L) * 0x9E3779B97F4A7C15L + 0xBF58476D1CE4E5B9L;

        double hx = d.bodyHalf[0];
        double hy = d.bodyHalf[1];
        double hz = d.bodyHalf[2];
        if (d.climber) {
            this.colR = hy * 1.15;
            this.nColOffs = 1;
            this.colOffs[0] = 0.0;
            this.colLat = 0.0;
        } else {
            double mH = Math.min(hx, Math.min(hz, hy * 1.3));
            this.colR = mH * 0.95;
            double off = Math.max(0.0, hz - colR);
            int nc = (int) Math.ceil(off / Math.max(colR, 1e-3));
            if (nc < 1) nc = 1;
            if (off > 0.05 && nc + 1 <= GaitCore.MAX_COL_OFFS) {
                this.nColOffs = nc + 1;
                for (int i = 0; i <= nc; i++) {
                    this.colOffs[i] = off * (1.0 - 2.0 * i / nc);
                }
            } else {
                this.nColOffs = 1;
                this.colOffs[0] = 0.0;
            }
            this.colLat = Math.max(0.0, hx - colR);
        }
        placeFeetInitial();
    }

    private void computeDriveIntent() {
        if (inp.run > 0.5) mode = GaitCore.MODE_RUN;
        else if (inp.slow > 0.5) mode = GaitCore.MODE_SLOW;
        else mode = GaitCore.MODE_WALK;

        Vec3.Frame heading = supportFrame();
        Vec3 mv = heading.fwd.mul(inp.fwd).add(heading.right.mul(inp.strafe * 0.75));
        if (mv.lenSq() > 1.0) {
            mv = mv.norm(Vec3.ZERO);
        }
        Vec3 cmd = mv.mul(targetSpeed());
        double ccap = crouchCap();
        if (crouch > 0.5 && cmd.len() > ccap) {
            cmd = cmd.norm(Vec3.ZERO).mul(ccap);
        }
        vCmd = cmd;
        if (cmd.lenSq() > 0.01) {
            lateralRatio = Math.abs(cmd.dot(heading.right)) / Math.max(Math.abs(cmd.dot(heading.fwd)), 0.1);
        } else {
            lateralRatio = 0.0;
        }
        vDesired = cmd.len();
    }

    private double dutyFor(Gait g) {
        double dv = g.duty;
        if (g.hasDutyFast) {
            double v0 = 0.75 * vWalk;
            double f = GaitCore.clamp((speed - v0) / Math.max(1.0, d.speedWalk - v0), 0.0, 1.0);
            dv = dv + (g.dutyFast - dv) * f;
            if (!(d.bound * g.leap >= 0.015) || braking || calmGait) {
                if (dv < 0.5) dv = 0.5;
            } else {
                double b = d.bound * g.leap;
                double vl = reachAvg / Math.max(0.05, Math.max(dv, 0.25) * g.cycleMin * Math.sqrt(size));
                double k = GaitCore.clamp((speed - 0.3 * vl) / (0.7 * vl), 0.0, 1.0);
                if (b * k <= 0.016 && dv < 0.5) {
                    dv = 0.5;
                }
            }
        }
        return dv;
    }

    private void selectMode(double dt) {
        if (state == GaitCore.STATE_GROUND) {
            situation.update(this, dt);
            int gidx = selector.select(this, dt, tmpSelOut);
            hasSpeedCap = tmpSelOut.hasCap;
            speedCap = tmpSelOut.cap;
            if (gidx != gaitIdx) {
                gaitIdx = gidx;
                Gait dstG = d.gaits[gidx];
                double dstT = GaitCore.clamp(
                        dstG.stride * size / Math.max(Math.max(vDesired, speed), 1e-3),
                        dstG.cycleMin * Math.sqrt(size),
                        dstG.cycleMax * Math.sqrt(size));
                blend.start(gidx, dstG, GaitCore.clamp(dstT, 0.28, 0.50));
            }
        } else {
            selector.lowT = 0.0;
            selector.prevV = speed;
            selector.accelV = 0.0;
        }
        blend.update(dt);
        for (int i = 0; i < n; i++) {
            legs[i].offset = blend.eff.offsets[i];
        }
        double k = GaitCore.damp(5.0, dt);
        double dVal = dutyFor(blend.dst);
        if (blend.t < 1.0) {
            double srcD = dutyFor(blend.src);
            dVal = srcD + (dVal - srcD) * blend.s;
            if (selector.hasHoldV && state == GaitCore.STATE_GROUND) {
                double hold = selector.holdV * (1.0 + 0.8 * blend.s);
                speedCap = hasSpeedCap ? Math.min(speedCap, hold) : hold;
                hasSpeedCap = true;
            }
        }
        duty += (dVal - duty) * k;
        blend.fitSupport(duty);
        gait.copyFrom(blend.eff);
    }

    private void cycleGaitOverride() {
        int gc = d.gaitCount;
        if (gaitOverride < 0) {
            gaitOverride = 0;
        } else if (gaitOverride + 1 >= gc) {
            gaitOverride = -1;
        } else {
            gaitOverride++;
        }
    }

    private void mechPush(Vec3 dv, boolean torque) {
        if (state == GaitCore.STATE_GROUND) {
            slideVel = slideVel.add(new Vec3(dv.x, 0.0, dv.z));
            vel = vel.add(dv);
            if (torque) {
                Vec3 dvH = new Vec3(dv.x, 0.0, dv.z);
                double dhl = dvH.len();
                if (dhl > 0.2) {
                    Vec3 axis = Vec3.WORLD_UP.cross(dvH).mul(1.0 / dhl);
                    angVel = angVel.add(axis.mul(GaitCore.clamp(dhl * 0.45 / Math.sqrt(d.mass), 0.0, 2.6)));
                }
            }
        } else {
            vel = vel.add(dv);
        }
    }

    private void triggerImpact() {
        double a = rngUniform(0.0, GaitCore.TAU);
        Vec3 axis = new Vec3(Math.cos(a), 0.0, Math.sin(a));
        angVel = angVel.add(axis.mul(2.4 / Math.sqrt(d.mass)));
        Vec3 push = new Vec3(Math.sin(a), 0.0, -Math.cos(a)).mul(4.0 / Math.sqrt(d.mass));
        mechPush(push, false);
        brace = 1.0;
        emit(GaitCore.EV_IMPACT, pos, Vec3.WORLD_UP, 1.0, Vec3.ZERO);
    }

    private boolean collide() {
        Vec3.Frame fr = frame;
        boolean bellyHit = false;
        WorldQueryCtx.PushResult pr = tmpPush;
        for (int i = 0; i < nColOffs; i++) {
            Vec3 c = pos.add(fr.fwd.mul(colOffs[i]));
            if (wq.spherePush(c, colR, pr)) {
                Vec3 disp = pr.center.sub(c);
                double dlen = disp.len();
                if (pr.normal.y > 0.65 || (dlen > 1e-5 && disp.y > 0.65 * dlen)) {
                    bellyHit = true;
                }
                pos = pos.add(disp);
                if (dlen > 1e-5) {
                    Vec3 dn = disp.mul(1.0 / dlen);
                    double vd = vel.dot(dn);
                    if (vd < 0.0) {
                        vel = vel.sub(dn.mul(vd));
                    }
                }
                double vn = vel.dot(pr.normal);
                if (vn < 0.0) {
                    vel = vel.sub(pr.normal.mul(vn));
                }
            }
        }
        if (colLat > 0.03) {
            int nLat = 1;
            double lo0 = colOffs[0];
            double lo1 = 0.0;
            if (nColOffs > 1) {
                lo1 = colOffs[nColOffs - 1];
                nLat = 2;
            }
            for (int li = 0; li < nLat; li++) {
                double latOff = li == 0 ? lo0 : lo1;
                for (int si = 0; si < 2; si++) {
                    double sgn = si == 0 ? -1.0 : 1.0;
                    Vec3 c = pos.add(fr.fwd.mul(latOff)).add(fr.right.mul(sgn * colLat));
                    if (wq.spherePush(c, colR, pr)) {
                        double dx = pr.center.x - c.x;
                        double dy = pr.center.y - c.y;
                        double dz = pr.center.z - c.z;
                        double hl = Math.hypot(dx, dz);
                        if (hl > 1e-5 && (Math.abs(pr.normal.y) < 0.55 || hl > Math.abs(dy) * 1.15)) {
                            Vec3 disp = new Vec3(dx, 0.0, dz);
                            pos = pos.add(disp);
                            Vec3 dn = disp.mul(1.0 / hl);
                            double vd = vel.dot(dn);
                            if (vd < 0.0) {
                                vel = vel.sub(dn.mul(vd));
                            }
                        }
                    }
                }
            }
        }
        return bellyHit;
    }

    private boolean supportError(double[] outErr) {
        Vec3[] pts = supPts;
        double[] wts = supWts;
        int np = 0;
        for (int i = 0; i < n; i++) {
            Leg l = legs[i];
            if (l.planted) {
                pts[np] = l.foot;
                wts[np] = 1.0;
                np++;
            } else if (l.swing.active && l.swing.hasGround) {
                pts[np] = l.swing.target;
                wts[np] = 0.35;
                np++;
            }
        }
        if (np == 0) {
            hasPlaneNLocal = false;
            return false;
        }
        Vec3.Frame sf = supportFrame();
        Vec3[] local = supLocal;
        for (int i = 0; i < np; i++) {
            local[i] = sf.toLocal(pts[i].sub(pos));
        }
        int fp = Vec3.fitPlane(local, wts, np, supFit);
        hasPlaneNLocal = fp >= 2;
        if (fp >= 2) {
            planeNLocal = supFit.n;
            double ws = 0.0;
            double acc = 0.0;
            for (int i = 0; i < np; i++) {
                double dv = local[i].sub(supFit.c).dot(supFit.n);
                acc += wts[i] * dv * dv;
                ws += wts[i];
            }
            if (ws == 0.0) ws = 1.0;
            footRms = Math.sqrt(acc / ws);
        } else {
            footRms = 0.0;
        }
        double planeY = (fp < 2 || supFit.n.y < 0.3)
                ? supFit.c.y
                : (supFit.c.y + (supFit.n.x * supFit.c.x + supFit.n.z * supFit.c.z) / supFit.n.y);
        double err = planeY + rideEff();
        if (!climbingMode() && (Math.abs(inp.aim) > 1e-3 || Math.abs(lookPitch) > 1e-3)) {
            double hx = d.bodyHalf[0];
            double hy = d.bodyHalf[1];
            double hz = d.bodyHalf[2];
            Vec3 gUp = (fp >= 2) ? sf.toWorld(supFit.n).norm(Vec3.WORLD_UP) : Vec3.WORLD_UP;
            double drop = hz * Math.abs(frame.fwd.dot(gUp))
                    + hy * Math.abs(frame.up.dot(gUp))
                    + hx * Math.abs(frame.right.dot(gUp));
            double alt = planeY + (drop + 0.06) / Math.max(0.5, gUp.y);
            if (alt > err) err = alt;
        }
        Vec3 up = sf.up;
        double want = rideEff() * 0.8;
        if (supT >= probePeriod - 1e-6) {
            supT = supT % probePeriod;
            hasSupH = wq.raycast(pos, up.mul(-1.0), want, supH);
            hasSupHa = false;
            if (!climbingMode() && state == GaitCore.STATE_GROUND && speed > 0.5) {
                Vec3 vDir = vt.projectPlane(up).norm(Vec3.ZERO);
                if (vDir.lenSq() > 0.5) {
                    double lead = GaitCore.clamp(speed * 0.10, 0.35, d.bodyHalf[2] + 0.35 * reachAvg);
                    double sm = d.stepMax;
                    Vec3 gn = groundN;
                    double slopeDz = -((vDir.x * lead) * gn.x + (vDir.z * lead) * gn.z) / Math.max(0.5, gn.y);
                    double probeTop = sm + 0.5 + Math.max(0.0, slopeDz);
                    Vec3 ahead = pos.add(vDir.mul(lead));
                    if (wq.raycast(ahead.add(up.mul(probeTop)), up.mul(-1.0), probeTop + rideEff() + 0.5, supHa)
                            && supHa.normal.dot(up) > 0.64) {
                        hasSupHa = true;
                        supHaSlopeDz = slopeDz;
                    }
                }
            }
        }
        if (hasSupH) {
            err = Math.max(err, want - supH.t);
        }
        if (hasSupHa && !climbingMode() && state == GaitCore.STATE_GROUND && speed > 0.5) {
            double slopeDz = supHaSlopeDz;
            Vec3 gn = groundN;
            double planeRef = (pos.dot(up) - rideEff()) + slopeDz;
            double rise = supHa.point.dot(up) - planeRef;
            if ((supHa.normal.dot(gn) < 0.985 && rise > 0.04 && rise <= d.stepMax + 0.65)
                    || (supHa.normal.y >= 0.98 && rise > 0.25 && rise <= d.stepMax + 0.05)) {
                double aheadErr = (supHa.point.dot(up) + 0.88 * rideEff()) - pos.dot(up);
                if (aheadErr > err) {
                    double wA = GaitCore.clamp((speed - 0.5) / 4.0, 0.0, 0.65);
                    err = err + (aheadErr - err) * wA;
                }
            }
        }
        outErr[0] = err;
        return true;
    }

    private double evaluateBalance(double dt) {
        hasBalanceOut = false;
        balanceExcess = 0.0;
        boolean anyEdge = false;
        for (int i = 0; i < n; i++) {
            if (legs[i].searching || legs[i].voidMark > 0.0) {
                anyEdge = true;
                break;
            }
        }
        boolean inFastGallop = Scheduler.gallopingFast(this, true) && (flightT > 0.0 || !anyEdge);
        if (biped || onWall || climbingMode() || drifting || state != GaitCore.STATE_GROUND
                || flightT > 0.0 || inFastGallop || wireActive()) {
            support = 1.0;
            hullN = 0;
            unbalT = 0.0;
            return 1.0;
        }

        Vec3.Vec2[] pts2 = balPts2;
        double[] ys = balYs;
        int np = 0;
        for (int i = 0; i < n; i++) {
            if (legs[i].planted) {
                pts2[np] = new Vec3.Vec2(legs[i].foot.x, legs[i].foot.z);
                ys[np] = legs[i].foot.y;
                np++;
            }
        }
        for (int i = 0; i < n; i++) {
            Leg.Swing sw = legs[i].swing;
            if (sw.active && sw.hasGround && sw.t > 0.6 && np < GaitCore.MAX_HULL_PTS) {
                pts2[np] = new Vec3.Vec2(sw.target.x, sw.target.z);
                ys[np] = sw.target.y;
                np++;
            }
        }
        if (np == 0) {
            support = 1.0;
            hullN = 0;
            if (!anyEdge) {
                unbalT = 0.0;
            }
            return 1.0;
        }

        hullN = Vec3.convexHull2d(pts2, np, hull);
        Vec3.Vec2 com = new Vec3.Vec2(pos.x, pos.z);
        double margin = Vec3.supportMargin(hull, hullN, com);
        support = margin;
        double excess = -margin - 0.15 * size;
        boolean wallBlocked = false;
        boolean edge = false;
        Vec3 out = Vec3.ZERO;
        if (excess > 0.0) {
            Vec3.Vec2 nearest = Vec3.nearestOnHull(hull, hullN, com);
            out = new Vec3(com.x - nearest.x, 0.0, com.z - nearest.z).norm(frame.fwd.projectPlane(Vec3.WORLD_UP));
            for (int i = 0; i < n; i++) {
                if ((legs[i].searching || legs[i].voidMark > 0.0) && legs[i].idealNow.sub(pos).dot(out) > 0.0) {
                    edge = true;
                    break;
                }
            }
            double halfOut = Math.abs(out.dot(frame.right)) * d.bodyHalf[0]
                    + Math.abs(out.dot(frame.fwd)) * d.bodyHalf[2];
            double wallD = halfOut + 0.45;
            for (int yi = 0; yi < 2; yi++) {
                double yOff = yi == 0 ? 0.0 : -0.45 * ride;
                if (wq.raycast(pos.add(Vec3.WORLD_UP.mul(yOff)), out, wallD, tmpHit)
                        && Math.abs(tmpHit.normal.dot(Vec3.WORLD_UP)) < 0.55) {
                    wallBlocked = true;
                    break;
                }
            }
        }
        if (excess <= 0.0 || wallBlocked || !((speed < 1.0 && Math.abs(yawRate) < 0.5) || edge)) {
            unbalT = Math.max(0.0, unbalT - 2.0 * dt);
            return 1.0;
        }
        unbalT += dt;
        if (unbalT < 0.2) {
            return 1.0;
        }
        hasBalanceOut = true;
        balanceOut = out;
        excess -= 0.35;
        if (excess <= 0.0) {
            return 1.0;
        }
        balanceExcess = excess;
        double ysum = 0.0;
        for (int i = 0; i < np; i++) {
            ysum += ys[i];
        }
        balanceH = pos.y - ysum / np;
        return GaitCore.clamp(1.0 - excess / (0.6 * ride), 0.0, 1.0);
    }

    private Vec3 updateDrift(double dt, Vec3 vt) {
        double spd = vt.len();
        if (slideCd > 0.0) {
            slideCd = Math.max(0.0, slideCd - dt);
        }
        boolean want = inp.drift > 0.5 && state == GaitCore.STATE_GROUND && !onWall;
        if (!want) {
            slideLock = false;
        }
        boolean pressed = want && !(prev.drift > 0.5);
        if (pressed && !drifting) {
            slideBuf = 0.22;
        } else if (want && !drifting && !slideLock) {
            slideBuf = Math.max(slideBuf, dt);
        } else {
            slideBuf = Math.max(0.0, slideBuf - dt);
        }

        double startSpeed = Math.min(3.0, 0.25 * d.speedRun);
        if (!drifting && slideBuf > 0.0 && !slideLock && slideCd <= 0.0
                && state == GaitCore.STATE_GROUND && !onWall && spd > startSpeed && flightT <= 0.08) {
            drifting = true;
            driftT = 0.0;
            slideBuf = 0.0;
            flightT = 0.0;
            flightHold = 0.0;
            Vec3 heading = frame.fwd.projectPlane(probeUp()).norm(new Vec3(0.0, 0.0, 1.0));
            slideDir = vt.norm(heading);
            double kick = Math.min(4.0, 0.14 * d.speedRun)
                    * GaitCore.clamp(spd / Math.max(1.0, 0.5 * d.speedRun), 0.35, 1.0);
            double newSpd = Math.min(spd + kick, d.speedRun * 1.08);
            vt = slideDir.mul(newSpd);
            spd = newSpd;
        } else if (drifting) {
            driftT += dt;
            double slopeDown = slideDir.dot(groundN);
            double minDur = 0.22;
            double maxDur = 0.95 + 0.55 * GaitCore.clamp(slopeDown / 0.35, 0.0, 1.0);
            double exitSpd = Math.max(2.0, 0.12 * d.speedRun);
            if ((!want && driftT >= minDur) || driftT >= maxDur || spd < exitSpd) {
                drifting = false;
                slideCd = 0.35;
                if (want) {
                    slideLock = true;
                }
            }
        }
        double rate = drifting ? 14.0 : 9.0;
        driftAmt += ((drifting ? 1.0 : 0.0) - driftAmt) * GaitCore.damp(rate, dt);
        return vt;
    }

    void enterAir() {
        state = GaitCore.STATE_AIR;
        stateT = 0.0;
        slideVel = Vec3.ZERO;
        accelSmooth = Vec3.ZERO;
        hasBalanceOut = false;
        balanceExcess = 0.0;
        unbalT = 0.0;
        flightT = 0.0;
        flightHold = 0.0;
        prevNpl = 0;
        trailingSupport = false;
        unsupportedT = 0.0;
        airBelly = false;
        drifting = false;
        slideBuf = 0.0;
        detachT = 0.0;
        groundN = Vec3.WORLD_UP;
        landRightT = 0.0;
        landRightN = Vec3.WORLD_UP;
        for (int i = 0; i < n; i++) {
            legs[i].planted = false;
            legs[i].swing.active = false;
            legs[i].searching = false;
        }
    }

    private void launchJump() {
        Vec3 jdir = onWall ? frame.up : Vec3.WORLD_UP;
        Vec3 vj = vel.sub(jdir.mul(Math.min(0.0, vel.dot(jdir))));
        vel = vj.add(jdir.mul(d.jumpSpeed)).add(frame.fwd.mul(Math.max(0.0, inp.fwd) * 1.5));
        for (int i = 0; i < n; i++) {
            tmpPushing[i] = legs[i].planted;
        }
        enterAir();
        for (int i = 0; i < n; i++) {
            if (tmpPushing[i]) {
                legs[i].push = true;
            }
        }
        emit(GaitCore.EV_LAND, pos.sub(frame.up.mul(ride)), frame.up, d.mass * 2.0, Vec3.ZERO);
    }

    private void predictImpact() {
        Vec3 up = Vec3.WORLD_UP;
        Vec3 p = pos.sub(up.mul(ride * 1.05));
        Vec3 v = vel;
        double t = 0.0;
        double step = 0.04;
        while (t < 2.4) {
            Vec3 v2 = v.sub(up.mul(GaitCore.G * step));
            Vec3 q = p.add(v.add(v2).mul(0.5 * step));
            Vec3 dv = q.sub(p);
            double dl = dv.len();
            if (dl > 1e-5) {
                if (wq.raycast(p, dv.mul(1.0 / dl), dl, tmpHit) && tmpHit.normal.y > 0.3) {
                    double f = tmpHit.t / dl;
                    impact.valid = true;
                    impact.t = t + step * f;
                    impact.point = tmpHit.point;
                    impact.normal = tmpHit.normal;
                    impact.speed = v.lerp(v2, f).len();
                    return;
                }
            }
            p = q;
            v = v2;
            t += step;
        }
        impact.valid = false;
    }

    private boolean landOn(Vec3 hitPoint, Vec3 hitNormal) {
        int oldState = state;
        double oldStateT = stateT;
        double oldUnsup = unsupportedT;
        double oldUnbal = unbalT;
        Vec3 oldGn = groundN;
        double oldLp = lookPitch;
        double oldAbs = absorb;
        double oldBrace = brace;
        Vec3 oldAv = angVel;
        Vec3 oldVel = vel;
        double oldLrt = landRightT;
        Vec3 oldLrn = landRightN;

        state = GaitCore.STATE_GROUND;
        stateT = 0.0;
        unsupportedT = 0.0;
        Vec3 landN = hitNormal.norm(Vec3.WORLD_UP);
        double tilt = Vec3.angleBetween(frame.up, landN);
        Vec3.Frame landFr = tmpFrame;
        landFr.init(headingFwd(), landN);
        groundN = landN;
        lookPitch = (Math.abs(inp.aim) > 1e-3) ? GaitCore.clamp(inp.aim, -lookPitchDn, lookPitchUp) : 0.0;
        double impSpd = -Math.min(0.0, vel.y);
        absorb = Math.max(absorb, GaitCore.clamp((impSpd - 1.5) / 22.0, 0.0, 0.5));
        brace = Math.max(brace, GaitCore.clamp(impSpd / 12.0, 0.0, 1.0));
        Vec3 vh = new Vec3(vel.x, 0.0, vel.z);
        double vSpd = vh.len();
        if (tilt < 0.15 && impSpd > 2.5 && vSpd > 1.5) {
            Vec3 axis = Vec3.WORLD_UP.cross(vh).mul(1.0 / vSpd);
            double dip = GaitCore.clamp(impSpd * Math.min(vSpd, 18.0) * 0.011 / Math.sqrt(d.mass), 0.0, 1.35);
            angVel = angVel.add(axis.mul(dip));
        }
        if (tilt > 0.10) {
            landRightT = GaitCore.clamp(0.28 + 0.40 * tilt, 0.28, 0.65);
            landRightN = landN;
            if (tilt >= 0.15) {
                angVel = angVel.mul(0.20);
            }
        } else {
            landRightT = 0.0;
        }
        vel = new Vec3(vel.x, Math.max(vel.y, -6.0), vel.z);
        double snapDist = (tilt > 0.15 ? 1.35 : 0.95) * size;

        for (int i = 0; i < n; i++) {
            Leg l = legs[i];
            l.push = false;
            l.mountWorld = pos.add(frame.toWorld(l.mountLocal));
            l.idealNow = pos.add(landFr.toWorld(l.restLocal(ride, 1.0)));
            Vec3 leadL = (vSpd > 4.0) ? vh.mul(Math.min(0.035, 0.25 * l.reach / vSpd)) : Vec3.ZERO;
            boolean hasFh = false;
            Vec3 fhp = Vec3.ZERO;
            Vec3 fhn = Vec3.WORLD_UP;
            if (climbingMode() && tilt > 0.35) {
                for (int ci = 0; ci < 2; ci++) {
                    Vec3 cand = ci == 0
                            ? l.foot.sub(hitPoint).projectPlane(landN).add(hitPoint).add(landN.mul(ride + 1.0))
                            : l.idealNow.add(landN.mul(ride + 1.0));
                    if (wq.raycast(cand, landN.mul(-1.0), ride * 2.5 + 1.0, tmpHit)
                            && tmpHit.normal.dot(landN) > 0.6
                            && tmpHit.point.dist(l.mountWorld) <= 0.98 * l.reach) {
                        hasFh = true;
                        fhp = tmpHit.point;
                        fhn = tmpHit.normal;
                        break;
                    }
                }
            }
            if (!hasFh) {
                Vec3 pEst = l.foot.sub(hitPoint).projectPlane(Vec3.WORLD_UP).add(hitPoint).add(leadL);
                hasFh = Foothold.findFoothold(this, l, pEst, 0.0, tmpFhOut);
                if (hasFh) {
                    fhp = tmpFhOut.pt;
                    fhn = tmpFhOut.n;
                }
            }
            if (!hasFh) {
                hasFh = Foothold.findFoothold(this, l, l.idealNow, 0.0, tmpFhOut);
                if (hasFh) {
                    fhp = tmpFhOut.pt;
                    fhn = tmpFhOut.n;
                }
            }
            boolean reachOk = hasFh && (tilt <= 0.15 || fhp.dist(l.mountWorld) <= 0.98 * l.reach);
            if (hasFh && reachOk && fhp.dist(l.foot) < snapDist) {
                l.foot = fhp;
                l.footNormal = fhn;
                l.planted = true;
                l.swing.active = false;
                l.searching = false;
            } else if (hasFh) {
                double dF = fhp.dist(l.foot);
                double swT = (tilt > 0.15)
                        ? GaitCore.clamp(dF / 18.0, 0.07, 0.14)
                        : GaitCore.clamp(dF / 14.0, 0.11, 0.18);
                Vec3 swUp = (climbingMode() && tilt > 0.35) ? landN : probeUp();
                l.beginSwing(fhp, fhn, swT, 0.05, swUp, true, wq, 0.0,
                        Vec3.ZERO, Vec3.ZERO, Vec3.ZERO, hasCeilY, ceilY);
            } else {
                l.searching = true;
            }
        }

        boolean anyPl = false;
        for (int i = 0; i < n; i++) {
            if (legs[i].planted) {
                anyPl = true;
                break;
            }
        }
        if (!anyPl) {
            Leg best = null;
            double bestD = 1e9;
            for (int i = 0; i < n; i++) {
                Leg l = legs[i];
                if (l.swing.active && l.swing.hasGround && l.swing.target.dist(l.mountWorld) <= 0.96 * l.reach) {
                    double dF = l.swing.target.dist(l.foot);
                    if (dF < bestD) {
                        bestD = dF;
                        best = l;
                    }
                }
            }
            if (best != null) {
                best.foot = best.swing.target;
                best.footNormal = best.swing.normal;
                best.planted = true;
                best.swing.active = false;
                best.searching = false;
            }
        }

        boolean anyOk = false;
        for (int i = 0; i < n; i++) {
            if (legs[i].planted || (legs[i].swing.active && legs[i].swing.hasGround)) {
                anyOk = true;
                break;
            }
        }
        if (!anyOk) {
            state = oldState;
            stateT = oldStateT;
            unsupportedT = oldUnsup;
            unbalT = oldUnbal;
            groundN = oldGn;
            lookPitch = oldLp;
            absorb = oldAbs;
            brace = oldBrace;
            angVel = oldAv;
            vel = oldVel;
            landRightT = oldLrt;
            landRightN = oldLrn;
            return false;
        }
        unbalT = 0.0;
        airBelly = false;
        emit(GaitCore.EV_LAND, hitPoint, hitNormal, impSpd * d.mass * 0.25, Vec3.ZERO);
        boolean anySrch = false;
        for (int i = 0; i < n; i++) {
            if (legs[i].searching) {
                anySrch = true;
                break;
            }
        }
        if (!anySrch) {
            situation.edge = 0.0;
        }
        return true;
    }

    private void updateAir(double dt) {
        vel = vel.add(Vec3.WORLD_UP.mul(-GaitCore.G * dt));
        Vec3 vh = new Vec3(vel.x, 0.0, vel.z);
        if (!airBelly) {
            Vec3 mv = headingFwd().mul(inp.fwd).add(frame.right.mul(inp.strafe * 0.75));
            Vec3 vDes = mv.mul(targetSpeed());
            Vec3 vdesH = new Vec3(vDes.x, 0.0, vDes.z);
            vh = vh.add(vdesH.sub(vh).mul(GaitCore.clamp(1.2 * dt, 0.0, 1.0)));
        } else {
            Vec3 fwdH = frame.fwd.projectPlane(Vec3.WORLD_UP).norm(new Vec3(0.0, 0.0, 1.0));
            vh = vh.sub(fwdH.mul(vh.dot(fwdH) * GaitCore.damp(4.0, dt)));
        }
        vel = new Vec3(vh.x, vel.y, vh.z);
        yawRate += (inp.turn * d.turnRate - yawRate) * GaitCore.damp(4.0, dt);
        frame.rotate(Vec3.WORLD_UP, -yawRate * dt);
        pos = pos.add(vel.mul(dt));
        boolean bellyHit = collide();
        predictImpact();
        if (vel.y <= 0.0 || stateT > 0.4) {
            double depth = 0.0;
            for (int i = 0; i < n; i++) {
                double dp = pos.y - legs[i].foot.y;
                if (dp > depth) depth = dp;
            }
            double reach = Math.max(ride * 1.02, depth + 0.05) + Math.max(0.0, -vel.y) * dt;
            double minNy = climbingMode() ? 0.3 : 0.55;
            if (wq.raycast(pos, Vec3.WORLD_UP.mul(-1.0), reach, tmpHit) && tmpHit.normal.y > minNy) {
                landOn(tmpHit.point, tmpHit.normal);
            }
        }
        airBelly = (state == GaitCore.STATE_AIR && bellyHit);
        if (airBelly && stateT > 0.04) {
            Vec3 sideBase = frame.right.projectPlane(Vec3.WORLD_UP).norm(new Vec3(1.0, 0.0, 0.0));
            double vLat = vel.dot(sideBase);
            double sgn = vLat >= 0.0 ? 1.0 : -1.0;
            Vec3 sideDir = sideBase.mul(sgn);
            if (Math.abs(vLat) < 1.2) {
                vel = vel.add(sideDir.mul(1.2 - Math.abs(vLat)));
            }
            vel = vel.add(sideDir.mul(10.5 * dt));
            angVel = angVel.add(Vec3.WORLD_UP.cross(sideDir).mul(5.5 * dt));
        }
        if (pos.y < -60.0) {
            vel = Vec3.ZERO;
        }
    }

    private boolean tryMantle() {
        if (groundN.y < 0.86) {
            return false;
        }
        boolean hasGu = wq.raycast(pos, Vec3.WORLD_UP.mul(-1.0), ride + 2.5, tmpHit);
        if (hasGu && tmpHit.normal.y < 0.90) {
            return false;
        }
        Vec3 fwdH = new Vec3(frame.fwd.x, 0.0, frame.fwd.z).norm(Vec3.ZERO);
        if (fwdH.lenSq() < 0.5) {
            return false;
        }
        double plSum = 0.0;
        int plCnt = 0;
        for (int i = 0; i < n; i++) {
            if (legs[i].planted) {
                plSum += legs[i].foot.y;
                plCnt++;
            }
        }
        double baseY = (hasGu && Math.abs(tmpHit.point.y - (pos.y - ride)) < 1.2 * ride)
                ? tmpHit.point.y
                : (plCnt > 0 ? (plSum / plCnt) : (pos.y - ride));
        double L = d.bodyHalf[2] + 0.6;
        Vec3 oHi = new Vec3(pos.x, baseY + d.stepMax + 0.1, pos.z);
        if (!wq.raycast(oHi, fwdH, L, tmpHit) || Math.abs(tmpHit.normal.y) > 0.4) {
            return false;
        }
        Vec3 hitWallPt = tmpHit.point;
        double hitWallT = tmpHit.t;
        Vec3 oLo = new Vec3(pos.x, baseY + d.stepMax * 0.4, pos.z);
        if (wq.raycast(oLo, fwdH, L, tmpHit) && tmpHit.t < hitWallT - 0.2) {
            return false;
        }
        double reachH = Math.max(d.mantleMax, d.wireMax);
        Vec3 topO = new Vec3(hitWallPt.x, baseY + reachH + 0.5, hitWallPt.z).add(fwdH.mul(0.25));
        if (!wq.raycast(topO, Vec3.WORLD_UP.mul(-1.0), reachH + 0.5, tmpHit) || tmpHit.normal.y < 0.8) {
            return false;
        }
        Vec3 topPt = tmpHit.point;
        double height = topPt.y - baseY;
        if (height <= d.stepMax * 0.9 || height > reachH) {
            return false;
        }
        if (wq.raycast(new Vec3(hitWallPt.x, topPt.y + 0.3, hitWallPt.z).sub(fwdH.mul(0.25)),
                Vec3.WORLD_UP.mul(-1.0), 0.85, tmpHit)
                && tmpHit.normal.y > 0.64 && topPt.y - tmpHit.point.y < 0.45) {
            return false;
        }
        Vec3 ahead = fwdH.mul(d.bodyHalf[2] * 0.6);
        Vec3 endGround = topPt;
        if (wq.raycast(topO.add(ahead), Vec3.WORLD_UP.mul(-1.0), reachH + 0.5, tmpHit)
                && tmpHit.normal.y >= 0.8 && topPt.y - 0.1 <= tmpHit.point.y && tmpHit.point.y <= baseY + reachH) {
            endGround = tmpHit.point;
        }
        height = Math.max(height, endGround.y - baseY);
        boolean wire = height > d.mantleMax;
        double headroom = ride + 2.0 * d.bodyHalf[1];
        if (wq.raycast(topPt.add(Vec3.WORLD_UP.mul(0.1)), Vec3.WORLD_UP, headroom, tmpHit)
                || wq.raycast(endGround.add(Vec3.WORLD_UP.mul(0.1)), Vec3.WORLD_UP, headroom, tmpHit2)) {
            return false;
        }

        mantle.active = true;
        mantle.p0 = pos;
        mantle.p2 = endGround.add(Vec3.WORLD_UP.mul(ride));
        mantle.t = 0.0;
        mantle.height = height;
        mantle.top = topPt;
        mantle.fwd = fwdH;
        mantle.wire = wire;
        mantle.landed = !wire;
        mantle.dur = (wire ? (0.9 + 0.2 * height) : (0.75 + 0.3 * height)) * Math.sqrt(size);
        state = GaitCore.STATE_MANTLE;
        stateT = 0.0;
        drifting = false;
        for (int i = 0; i < n; i++) {
            if (legs[i].isFront || (biped && legs[i].idx == 0)) {
                legs[i].force = true;
            }
        }
        emit(GaitCore.EV_MANTLE, topPt, Vec3.WORLD_UP, height, Vec3.ZERO);
        return true;
    }

    Vec3 mantlePos(double t) {
        t = GaitCore.clamp(t, 0.0, 1.0);
        double s1, s2;
        if (mantle.wire) {
            s1 = GaitCore.smoothstep(t / 0.7);
            s2 = GaitCore.smoothstep((t - 0.5) / 0.5);
        } else {
            s1 = GaitCore.smoothstep(t / 0.6);
            s2 = GaitCore.smoothstep((t - 0.3) / 0.7);
        }
        return new Vec3(
                mantle.p0.x + (mantle.p2.x - mantle.p0.x) * s2,
                mantle.p0.y + (mantle.p2.y + 0.1 - mantle.p0.y) * s1,
                mantle.p0.z + (mantle.p2.z - mantle.p0.z) * s2);
    }

    private void updateMantle(double dt) {
        double strain = 0.0;
        for (int i = 0; i < n; i++) {
            if (legs[i].planted && legs[i].stretch > strain) {
                strain = legs[i].stretch;
            }
        }
        mantle.t += dt / mantle.dur * (strain > 1.0 ? 0.25 : 1.0);
        Vec3 nw = mantlePos(mantle.t);
        vel = nw.sub(pos).mul(1.0 / dt);
        pos = nw;
        yawRate *= Math.exp(-6.3 * dt);
        if (mantle.wire && !mantle.landed && mantle.t >= 0.72) {
            mantle.landed = true;
            for (int i = 0; i < n; i++) {
                Leg l = legs[i];
                l.mountWorld = pos.add(frame.toWorld(l.mountLocal));
                l.idealNow = pos.add(frame.toWorld(l.restLocal(ride, 1.0)));
                if (Foothold.findFoothold(this, l, l.idealNow, 0.0, tmpFhOut)) {
                    l.beginSwing(tmpFhOut.pt, tmpFhOut.n, 0.12, 0.1, probeUp(), true, wq, 0.0,
                            Vec3.ZERO, Vec3.ZERO, Vec3.ZERO, hasCeilY, ceilY);
                } else {
                    l.searching = true;
                }
            }
        }
        if (mantle.t >= 1.0) {
            state = GaitCore.STATE_GROUND;
            stateT = 0.0;
            vel = new Vec3(frame.fwd.x, 0.0, frame.fwd.z);
            mantle.active = false;
        }
    }

    private void updateGround(double dt, boolean jump) {
        Vec3.Frame fr = frame;
        Vec3 up = probeUp();
        int npl = 0;
        for (int i = 0; i < n; i++) {
            if (legs[i].planted) npl++;
        }

        if (jump && d.jumpSpeed > 0.0 && state == GaitCore.STATE_GROUND && npl > 0) {
            if (drifting) {
                drifting = false;
                slideCd = Math.max(slideCd, 0.25);
            }
            state = GaitCore.STATE_CROUCHJUMP;
            stateT = 0.0;
            jumpSink = 0.0;
            jumpR0 = Math.max(0.1, rideEff());
            jumpY0 = pos.dot(up);
            jumpNpl = npl;
            jumpSinkCap = sinkRoom();
        }
        if (state == GaitCore.STATE_CROUCHJUMP) {
            double ct = crouchTime();
            if (stateT > ct) {
                launchJump();
                return;
            }
            if (npl > jumpNpl) {
                double sr = sinkRoom();
                if (sr < jumpSinkCap) jumpSinkCap = sr;
            }
            jumpNpl = npl;
            double want = jumpSinkCap * GaitCore.smoothstep(stateT / Math.max(0.05, ct));
            boolean anyErr = false;
            for (int i = 0; i < n; i++) {
                if (legs[i].planted && legs[i].ik.err > 0.02) {
                    anyErr = true;
                    break;
                }
            }
            if (anyErr) {
                jumpSink = Math.max(0.0, jumpSink - 0.6 * dt);
            } else if (want > jumpSink) {
                jumpSink = want;
            }
        }
        if (state == GaitCore.STATE_GROUND && !climbingMode()
                && groundN.y >= 0.86 && speed < Math.max(16.0, 1.15 * vWalk)
                && (d.mantleMax > 0.0 || d.wireMax > 0.0)
                && inp.fwd > 0.05 && mantleT >= probePeriod - 1e-6) {
            mantleT = mantleT % probePeriod;
            if (tryMantle()) {
                return;
            }
        }

        double bal = evaluateBalance(dt);
        Vec3 vDes = vCmd;
        double vdl = vDes.len();
        if (hasSpeedCap && vdl > speedCap) {
            vDes = vDes.norm(Vec3.ZERO).mul(speedCap);
            vdl = vDes.len();
        }
        vDesired = vdl;
        driveTarget = vDes;
        driveAccel = 0.0;
        braking = speed > 1.0 && (vdl < 0.82 * speed || (vdl > 0.1 && vDes.dot(vt) < -0.1 * speed * vdl));

        double vUp = vel.dot(up);
        Vec3 slide = slideVel.projectPlane(up);
        Vec3 vtCur = vel.sub(up.mul(vUp)).sub(slide);
        vtCur = updateDrift(dt, vtCur);
        Vec3 vtRaw = vtCur;

        double spinScale = 0.62 + 0.38 * GaitCore.smoothstep((Math.max(speed, vDesired) - 1.0) / 3.0);
        double yrDes = inp.turn * d.turnRate * spinScale * (mode == GaitCore.MODE_SLOW ? 0.75 : 1.0);
        double normalLim = Math.min(d.turnRate, GaitCore.A_LAT / Math.max(speed, 1e-3));
        double lim, kYaw;
        if (drifting || driftAmt > 0.02) {
            double slideW = drifting ? 1.0 : driftAmt;
            double slideLim = Math.min(0.55 * d.turnRate, GaitCore.A_LAT / Math.max(speed, 4.0));
            lim = normalLim + (slideLim - normalLim) * slideW;
            kYaw = 24.0 - 10.0 * slideW;
        } else {
            lim = normalLim;
            kYaw = 24.0;
        }
        if (flightT > 0.0) {
            kYaw = Math.min(kYaw, 16.0);
        }
        yrDes = GaitCore.clamp(yrDes, -lim, lim);

        if (!drifting && !Scheduler.gallopingFast(this, true) && speed < 0.45 * vWalk
                && bal >= 1.0 && Math.abs(yrDes) > 0.1) {
            double yawStrain = 0.0;
            for (int i = 0; i < n; i++) {
                Leg l = legs[i];
                if (!l.planted) continue;
                if (l.ik.mode == GaitCore.PLANE_RADIAL && l.ik.coxaRatio > 0.96 && l.ik.yawDeg * yrDes < 0.0) {
                    yawStrain = Math.max(yawStrain, (l.ik.coxaRatio - 0.96) * 20.0);
                }
                if (l.stretch > 0.975) {
                    yawStrain = Math.max(yawStrain, (l.stretch - 0.975) * 20.0);
                }
            }
            if (yawStrain > 0.0) {
                yrDes *= Math.max(0.45, 1.0 - Math.min(0.55, yawStrain));
            }
        }
        double newYr = yawRate + (yrDes - yawRate) * GaitCore.damp(kYaw, dt);
        double yrMid = 0.5 * (yawRate + newYr);

        boolean hasGroundSwing = false;
        for (int i = 0; i < n; i++) {
            if (legs[i].swing.active && legs[i].swing.hasGround) {
                hasGroundSwing = true;
                break;
            }
        }
        double traction = (npl > 0) ? 1.0 : ((flightT > 0.0 || hasGroundSwing) ? 0.3 : 0.0);

        Vec3 vtNew;
        if (drifting) {
            Vec3 heading = fr.fwd.projectPlane(up).norm(fr.fwd);
            double spd = vtCur.len();
            Vec3 vdir = vtCur.norm(slideDir);
            Vec3 steerDir = (vdl > 0.2) ? vDes.projectPlane(up).norm(heading) : heading;
            double carve = (vdl > 0.2) ? 0.55 : 0.25;
            vdir = Vec3.rotateTowards(vdir, steerDir, carve * dt);
            slideDir = vdir;
            double slopeDown = vdir.dot(groundN);
            double baseDrag = Math.max(1.8, 0.06 * d.speedRun) + 0.08 * spd;
            double slopeAcc = GaitCore.G * GaitCore.clamp(slopeDown, -0.55, 0.55) * 2.2;
            double brakeExtra = (vdl > 0.2 && steerDir.dot(vdir) < -0.3)
                    ? Math.max(8.0, 0.25 * d.speedRun)
                    : 0.0;
            spd = Math.max(0.0, spd + (slopeAcc - baseDrag - brakeExtra) * dt);
            spd = Math.min(spd, d.speedRun * (slopeDown > 0.12 ? 1.12 : 1.08));
            vtNew = vdir.mul(spd);
        } else {
            boolean fast = mode == GaitCore.MODE_RUN
                    || speed > 0.9 * Math.min(vWalk, mode == GaitCore.MODE_SLOW ? vSlow : 1e9);
            double acc = ((fast && d.accelRun > 0.0) ? d.accelRun : d.accel) * traction;
            boolean rotateVt = false;
            if (speed > 0.5 && Math.abs(yrMid) > 1e-4) {
                if (flightT > 0.0 || Scheduler.gallopingFast(this, false)) {
                    rotateVt = true;
                } else if (vdl > 0.5 && vtCur.dot(vDes) >= -0.25 * speed * vdl) {
                    rotateVt = true;
                }
                if (rotateVt && vdl > 0.5) {
                    double desYaw = Math.atan2(vDes.cross(vtCur).dot(up), vtCur.dot(vDes));
                    if (desYaw * yrMid < -0.03 * Math.abs(yrMid)) {
                        rotateVt = false;
                    }
                }
            }
            if (rotateVt) {
                vtCur = vtCur.rotateAxis(up, -yrMid * dt);
            }
            Vec3 dv = vDes.sub(vtCur);
            double maxDv = acc * dt * (dv.dot(vtCur) < 0.0 ? 1.6 : 1.0);
            driveAccel = maxDv / dt;
            double dl = dv.len();
            if (dl > maxDv) {
                dv = dv.mul(maxDv / dl);
            }
            vtNew = vtCur.add(dv);
        }

        Leg[] searching = tmpLegs1;
        int nSrch = 0;
        for (int i = 0; i < n; i++) {
            if (legs[i].searching || legs[i].voidMark > 0.0) {
                searching[nSrch++] = legs[i];
            }
        }
        if (nSrch > 0 && npl > 0 && !onWall && !drifting) {
            voidT += dt;
        } else {
            voidT = Math.max(0.0, voidT - 0.5 * dt);
        }

        boolean narrowAhead = false;
        Vec3 narrowVoid = Vec3.ZERO;
        if (voidT > 0.30 && nSrch > 0 && npl > 0 && !onWall && !drifting) {
            Vec3 vs = Vec3.ZERO;
            for (int i = 0; i < nSrch; i++) {
                vs = vs.add(searching[i].idealNow.sub(pos).projectPlane(up));
            }
            Vec3 hf = fr.fwd.projectPlane(up).norm(fr.fwd);
            double along = vs.dot(hf);
            Vec3 vdDir = hf.mul(along >= 0.0 ? 1.0 : -1.0);
            double push = (vdl > 0.05) ? (vDes.dot(vdDir) / Math.max(vdl, 1e-6)) : 0.0;
            boolean centerGround = false;
            boolean wideGround = false;
            if (Math.abs(along) >= 0.5 * vs.len()) {
                double maxMx = 0.0;
                for (int i = 0; i < n; i++) {
                    double ax = Math.abs(legs[i].mountLocal.x);
                    if (ax > maxMx) maxMx = ax;
                }
                double latCheck = Math.max(0.52, maxMx * 0.72);
                Vec3 rightV = up.cross(vdDir);
                double reachD = ride + d.stepMax + 2.0;
                for (int di = 0; di < 2; di++) {
                    double distVal = d.bodyHalf[2] + (di == 0 ? 0.6 : 1.5);
                    Vec3 pa = pos.add(vdDir.mul(distVal));
                    if (wq.raycast(pa.add(up.mul(1.0)), up.mul(-1.0), reachD, tmpHit)
                            && tmpHit.normal.dot(up) > 0.64) {
                        centerGround = true;
                        if (latCheck > 0.15) {
                            if (wq.raycast(pa.sub(rightV.mul(latCheck)).add(up.mul(1.0)), up.mul(-1.0), reachD, tmpHit)
                                    && tmpHit.normal.dot(up) > 0.64
                                    && wq.raycast(pa.add(rightV.mul(latCheck)).add(up.mul(1.0)), up.mul(-1.0), reachD, tmpHit2)
                                    && tmpHit2.normal.dot(up) > 0.64) {
                                wideGround = true;
                            }
                        } else {
                            wideGround = true;
                        }
                    }
                }
            }
            narrowAhead = centerGround && !wideGround;
            if (narrowAhead) {
                narrowVoid = vdDir;
                double c = vtNew.dot(vdDir);
                if (c > 0.0) {
                    vtNew = vtNew.sub(vdDir.mul(c));
                }
                if (bal < 1.0 || balanceExcess > 0.0 || nSrch >= Math.max(n / 2, 2)) {
                    double back = -vtNew.dot(vdDir);
                    double want = 1.0 * Math.min(1.0, (voidT - 0.30) * 3.0);
                    if (back < want) {
                        vtNew = vtNew.sub(vdDir.mul(want - back));
                    }
                }
            } else if (voidT > 0.4) {
                if (Math.abs(along) < 0.5 * vs.len() || wideGround) {
                    // safe straddle or lateral ground
                } else if (push > 0.3) {
                    if (voidT > 0.9) {
                        vel = vtNew.add(vdDir.mul(1.5)).add(up.mul(vUp));
                        enterAir();
                        return;
                    }
                } else if (bal < 1.0 && balanceExcess > 0.15 && voidT > 1.5) {
                    vel = vtNew.add(vdDir.mul(1.5)).add(up.mul(vUp));
                    enterAir();
                    return;
                } else if (bal >= 1.0) {
                    double c = vtNew.dot(vdDir);
                    if (c > 0.0) {
                        vtNew = vtNew.sub(vdDir.mul(c));
                    }
                    double back = -vtNew.dot(vdDir);
                    double want = 0.8 * Math.min(1.0, (voidT - 0.4) * 3.0);
                    if (back < want) {
                        vtNew = vtNew.sub(vdDir.mul(want - back));
                    }
                }
            }
        }

        if (!drifting && !Scheduler.gallopingFast(this, true) && bal >= 1.0) {
            for (int i = 0; i < n; i++) {
                Leg l = legs[i];
                if (l.planted && l.stretch > 0.96) {
                    Vec3 away = l.mountWorld.sub(l.foot).projectPlane(up).norm(Vec3.ZERO);
                    double c = vtNew.dot(away);
                    if (c > 0.0) {
                        vtNew = vtNew.sub(away.mul(c * Math.min(1.0, (l.stretch - 0.96) * 25.0)));
                    }
                }
            }
        }
        lastAccel = vtNew.sub(vtCur).mul(1.0 / dt);
        accelSmooth = accelSmooth.lerp(lastAccel, GaitCore.damp(14.0, dt));

        yawRate = newYr;
        fr.yaw(yrMid * dt);

        double sl = slide.len();
        if (sl > 1e-4) {
            double mu = (npl > 0) ? 0.55 : 0.12;
            slide = slide.mul(Math.max(0.0, sl - mu * GaitCore.G * dt) / sl);
        }
        slideVel = slide;

        if (flightT <= 0.0 && gait.pairSync && gait.minSupport == 0
                && !braking && speed > 0.5 * Scheduler.vLeg(this) && trailingSupport && !onWall && !drifting) {
            boolean rev = vtNew.dot(headingFwd()) < 0.0;
            int leadPair = (rev ? legs[n - 1] : legs[0]).pair;
            boolean anyLead = false;
            for (int i = 0; i < n; i++) {
                if (legs[i].planted && legs[i].pair == leadPair) {
                    anyLead = true;
                    break;
                }
            }
            if (!anyLead) {
                vUp += 0.45 * GaitCore.G * dt;
            }
        }

        tmpVUp[0] = vUp;
        if (Scheduler.updateFlight(this, dt, npl, vtNew, up, tmpVUp)) {
            pos = pos.add(vel.mul(dt)).add(up.mul(0.5 * GaitCore.G * dt * dt));
            collide();
            return;
        }
        vUp = tmpVUp[0];

        if (npl > 0 || hasGroundSwing) {
            unsupportedT = 0.0;
        } else {
            unsupportedT += dt;
        }
        boolean supported = npl > 0 || (unsupportedT < 0.12 && Scheduler.mayFlight(this));
        double[] errOut = tmpVUp;
        errOut[0] = 0.0;
        boolean hasErr = supportError(errOut);
        Vec3 grav;
        if (supported && hasErr) {
            double f = 3.0 / Math.sqrt(size);
            double w = GaitCore.TAU * f;
            vUp += (w * w * errOut[0] - 2.0 * 0.75 * w * vUp) * dt * bal;
            grav = Vec3.WORLD_UP.mul(-GaitCore.G * dt * (1.0 - bal));
        } else {
            grav = Vec3.WORLD_UP.mul(-GaitCore.G * dt);
        }
        vel = vtNew.add(slideVel).add(up.mul(vUp)).add(grav);

        if (hasBalanceOut && balanceExcess > 0.0) {
            if (!(narrowAhead && balanceOut.dot(narrowVoid) > 0.2)) {
                double h = Math.max(0.3, balanceH);
                double a = Math.min(GaitCore.G, GaitCore.G * balanceExcess / h);
                double vCap = Math.min(vWalk, 6.0);
                double curB = vel.dot(balanceOut);
                double dvB = Math.min(a * dt, Math.max(0.0, vCap - curB));
                vel = vel.add(balanceOut.mul(dvB));
                if (dvB > 0.0) {
                    angVel = angVel.add(Vec3.WORLD_UP.cross(balanceOut).mul(dvB / h));
                }
            }
            for (int i = 0; i < n; i++) {
                if (legs[i].planted && legs[i].stretch > 1.0) {
                    legs[i].planted = false;
                    legs[i].searching = true;
                    legs[i].searchCd = 0.15;
                }
            }
            boolean anyPlNow = false;
            boolean anyOkPl = false;
            for (int i = 0; i < n; i++) {
                if (legs[i].planted) {
                    anyPlNow = true;
                    if (legs[i].stretch <= 1.0) anyOkPl = true;
                }
            }
            if ((bal <= 0.0 || !anyPlNow) && !(narrowAhead && npl > 0)) {
                double unbalLim = (balanceExcess > 0.65 * size) ? 0.45 : 2.5;
                if (!anyOkPl || unbalT > unbalLim) {
                    vel = vel.add(balanceOut.mul(1.0));
                    enterAir();
                }
            }
        }

        if (!supported && unsupportedT > 0.12) {
            enterAir();
        }
        if (onWall) {
            int hold = Math.max(n / 2 - 1, 2);
            if (npl < hold || !climbingMode()) {
                detachT += dt;
                if (detachT > 0.25) {
                    enterAir();
                }
            } else {
                detachT = 0.0;
            }
        }

        Vec3 vtStep = drifting ? vtNew : vtRaw.add(vtNew).mul(0.5);
        pos = pos.add(vel.sub(vtNew).add(vtStep).mul(dt));
        collide();
    }

    private Vec3 climbProbes(Vec3 target) {
        if (state != GaitCore.STATE_GROUND) {
            return target;
        }
        Vec3 up = frame.up;
        Vec3 mv = frame.fwd.mul(inp.fwd).add(frame.right.mul(inp.strafe * 0.75));
        if (mv.lenSq() < 0.01) {
            return target;
        }
        Vec3 dirv = mv.norm(frame.fwd);
        double L = d.bodyHalf[2] + reachAvg * 0.65;
        if (wq.raycast(pos, dirv, L, tmpHit) && tmpHit.normal.dot(up) < 0.7 && tmpHit.normal.dot(dirv) < -0.2) {
            double k = Math.pow(GaitCore.clamp(1.0 - tmpHit.t / L, 0.0, 1.0), 0.7) * 0.9;
            return target.mul(1.0 - k).add(tmpHit.normal.mul(k)).norm(target);
        }
        Vec3 p = pos.add(dirv.mul(d.bodyHalf[2] + reachAvg * 0.35));
        double depth = ride * 2.2;
        if (!wq.raycast(p, up.mul(-1.0), depth, tmpHit)) {
            Vec3 end = p.sub(up.mul(depth));
            Vec3 back = pos.sub(up.mul(ride)).sub(end);
            double bl = back.len();
            if (bl > 1e-4) {
                if (wq.raycast(end, back.mul(1.0 / bl), reachAvg * 1.5, tmpHit2)
                        && tmpHit2.normal.dot(up) < 0.9) {
                    return target.mul(0.4).add(tmpHit2.normal.mul(0.6)).norm(target);
                }
            }
        }
        return target;
    }

    private void updateOrientation(double dt) {
        Vec3.Frame fr = frame;
        boolean righting = false;
        Vec3 target = Vec3.WORLD_UP;
        if (state == GaitCore.STATE_AIR) {
            if (!wireActive() && Math.abs(inp.aim) > 1e-3) {
                double a = GaitCore.clamp(inp.aim, -GaitCore.AIM_MAX, GaitCore.AIM_MAX);
                target = Vec3.WORLD_UP.rotateAxis(fr.right, a);
            } else {
                angVel = angVel.mul(Math.exp(-3.5 * dt));
                double ang = angVel.len();
                if (ang * dt > 1e-7) {
                    fr.rotate(angVel.mul(1.0 / ang), ang * dt);
                }
                onWall = false;
                return;
            }
        } else {
            boolean anyPl = false;
            for (int i = 0; i < n; i++) {
                if (legs[i].planted) {
                    anyPl = true;
                    break;
                }
            }
            righting = state == GaitCore.STATE_GROUND && landRightT > 0.0 && (anyPl || stateT < 0.08);
            if (righting) {
                groundN = landRightN;
                Vec3 groundUp = landRightN;
                target = groundUp;
                if (!onWall && Math.abs(inp.aim) > 1e-3) {
                    lookPitch = GaitCore.clamp(inp.aim, -lookPitchDn, lookPitchUp);
                    Vec3 pitchAxis = headingFwd().cross(groundUp).norm(fr.right);
                    target = target.rotateAxis(pitchAxis, lookPitch);
                }
            } else {
                if (state == GaitCore.STATE_MANTLE) {
                    groundN = groundN.lerp(Vec3.WORLD_UP, GaitCore.damp(14.0, dt)).norm(Vec3.WORLD_UP);
                } else if (hasPlaneNLocal) {
                    Vec3.Frame sf = supportFrame();
                    Vec3 rawN = sf.toWorld(planeNLocal).norm(Vec3.WORLD_UP);
                    double rate = climbingMode() ? 22.0 : 14.0;
                    groundN = groundN.lerp(rawN, GaitCore.damp(rate, dt)).norm(Vec3.WORLD_UP);
                } else if (climbingMode()) {
                    groundN = fr.up;
                } else {
                    groundN = groundN.lerp(Vec3.WORLD_UP, GaitCore.damp(2.0, dt)).norm(Vec3.WORLD_UP);
                }
                Vec3 groundUp;
                if (climbingMode()) {
                    target = climbProbes(groundN);
                    groundUp = target;
                } else {
                    groundUp = Vec3.rotateTowards(Vec3.WORLD_UP, groundN.norm(Vec3.WORLD_UP),
                            Math.max(d.maxTilt, 40.0) * GaitCore.DEG2RAD);
                    target = groundUp;
                }
                if (!onWall) {
                    Vec3 a = fr.toLocal(accelSmooth);
                    double gal = (Scheduler.gallopingFast(this, false) || flightT > 0.0) ? 0.35 : 1.0;
                    double latScale = 1.0 / Math.sqrt(Math.max(0.5, d.mass));
                    Vec3 lean = fr.fwd.mul(GaitCore.clamp(a.z * 0.006 * gal, -0.15, 0.15));
                    lean = lean.add(fr.right.mul(
                            GaitCore.clamp((a.x * 0.0045 + yawRate * speed * 0.003) * latScale, -0.20, 0.20)));
                    if (biped) {
                        lean = lean.add(fr.fwd.mul(Math.min(0.07, speed * 0.01)));
                    }
                    if (driftAmt > 0.01) {
                        lean = lean.sub(slideDir.mul(0.14 * driftAmt));
                    }
                    target = target.add(lean).norm(Vec3.WORLD_UP);
                }
                if (!onWall && (Math.abs(inp.aim) > 1e-3 || Math.abs(lookPitch) > 1e-4)) {
                    double wantA = (Math.abs(inp.aim) > 1e-3)
                            ? GaitCore.clamp(inp.aim, -lookPitchDn, lookPitchUp)
                            : 0.0;
                    boolean strained = false;
                    for (int i = 0; i < n; i++) {
                        Leg l = legs[i];
                        if (l.ik.err > 0.015
                                || (l.planted && l.ik.limitMarginDeg < 1.0)
                                || (l.planted && l.stretch > 0.96 && speed < 2.0)) {
                            strained = true;
                            break;
                        }
                    }
                    if (strained) {
                        pitchCap = Math.max(0.35, pitchCap - 1.2 * dt);
                        pitchStrainT = 0.45;
                    } else if (pitchStrainT > 0.0) {
                        pitchStrainT = Math.max(0.0, pitchStrainT - dt);
                    } else {
                        pitchCap = Math.min(1.0, pitchCap + 0.65 * dt);
                    }
                    double cappedA = wantA * pitchCap;
                    double rate = (speed > 2.0) ? 6.0 : 12.0;
                    double dPitch = (cappedA - lookPitch) * GaitCore.damp(rate, dt);
                    if (speed > 2.0 && groundUp.y < 0.97) {
                        double maxDp = 1.8 * dt;
                        dPitch = GaitCore.clamp(dPitch, -maxDp, maxDp);
                    }
                    lookPitch += dPitch;
                    if (Math.abs(lookPitch) > 1e-4) {
                        Vec3 pitchAxis = headingFwd().cross(groundUp).norm(fr.right);
                        target = target.rotateAxis(pitchAxis, lookPitch);
                    }
                }
                double fwdV = vt.dot(headingFwd());
                double fwdSgn = (fwdV < -1.0) ? -1.0 : 1.0;
                double fwdAlign = GaitCore.clamp(Math.abs(fwdV) / Math.max(speed, 1.0), 0.0, 1.0);
                if (flightT > 0.0 && flightDur > 0.0) {
                    double prog = 1.0 - flightT / flightDur;
                    double pitch = d.boundPitch * GaitCore.DEG2RAD * gait.stridePitch * fwdSgn * fwdAlign
                            * (Math.cos(GaitCore.PI * prog) * 0.92 + 0.08);
                    target = target.rotateAxis(fr.right, pitch);
                } else if (gait.pairSync && state == GaitCore.STATE_GROUND && d.bound > 0.0 && speed > 0.4 * d.speedRun) {
                    boolean front = false;
                    boolean rear = false;
                    for (int i = 0; i < n; i++) {
                        if (legs[i].planted) {
                            if (legs[i].mountLocal.z > 0.05) front = true;
                            if (legs[i].mountLocal.z < -0.05) rear = true;
                        }
                    }
                    double sgn = (front && !rear) ? -0.75 : ((rear && !front) ? 0.85 : 0.0);
                    if (sgn != 0.0) {
                        double pitch = d.boundPitch * GaitCore.DEG2RAD * gait.stridePitch * fwdSgn * fwdAlign * sgn;
                        target = target.rotateAxis(fr.right, pitch);
                    }
                }
                if (state == GaitCore.STATE_MANTLE && mantle.active) {
                    double pitch = 16.0 * GaitCore.DEG2RAD * Math.sin(GaitCore.PI * GaitCore.clamp(mantle.t, 0.0, 1.0));
                    target = target.rotateAxis(fr.right, pitch);
                } else if (!climbingMode()) {
                    double limRel = (d.lookTilt + 8.0) * GaitCore.DEG2RAD;
                    target = Vec3.rotateTowards(groundUp, target.norm(groundUp), limRel);
                }
            }
        }

        target = target.norm(Vec3.WORLD_UP);
        Vec3 err = fr.up.cross(target);
        if (fr.up.dot(target) < 0.0) {
            err = err.lenSq() > 1e-9 ? err.norm(fr.right) : fr.right;
        }
        if (righting) {
            double f = 5.2 / Math.sqrt(size);
            double w = Math.min(GaitCore.TAU * f, 0.78 / Math.max(dt, 1e-4));
            angVel = angVel.add(err.mul((w * w) * dt)).mul(1.0 / (1.0 + 2.0 * 0.95 * w * dt));
        } else {
            double f = (climbingMode() ? 4.5 : 2.6) / Math.sqrt(size);
            double w = GaitCore.TAU * f;
            angVel = angVel.add(err.mul(w * w).sub(angVel.mul(2.0 * 0.7 * w)).mul(dt));
        }
        double ang = angVel.len();
        if (ang * dt > 1e-7) {
            fr.rotate(angVel.mul(1.0 / ang), ang * dt);
        }
        if (righting) {
            landRightT = Math.max(0.0, landRightT - dt);
            if (Vec3.angleBetween(fr.up, target) < 0.035 && angVel.len() < 0.4) {
                landRightT = 0.0;
            }
        }
        onWall = climbingMode() && state != GaitCore.STATE_AIR && fr.up.y < 0.5;
    }

    private void updateVisOffset(double dt) {
        double tx = 0.0;
        double ty = 0.0;
        if (biped && state == GaitCore.STATE_GROUND) {
            Leg firstSw = null;
            int nSw = 0;
            for (int i = 0; i < n; i++) {
                if (legs[i].swing.active) {
                    if (firstSw == null) firstSw = legs[i];
                    nSw++;
                }
            }
            if (nSw == 1 && firstSw != null) {
                tx = 0.13 * -firstSw.side * (1.0 - 0.5 * Math.min(1.0, speed / 6.0));
            }
            ty = -0.06 * Math.cos(4.0 * GaitCore.PI * phase) * Math.min(1.0, speed / 3.0);
        }
        visOffset = visOffset.lerp(new Vec3(tx, ty, 0.0), GaitCore.damp(7.0, dt));
    }

    void update(double dt) {
        if (!Double.isFinite(dt) || dt <= 0.0) {
            return;
        }
        boolean profEn = profileEnabled;
        double t0 = profEn ? GaitCore.timeSec() : 0.0;

        if (!climbingMode()) {
            ceilT += dt;
            double pProbe = probePeriod;
            if (ceilT >= pProbe - 1e-6) {
                ceilT = ceilT % pProbe;
                double c0 = 0.0;
                double c1 = 0.0;
                int nc = 0;
                if (wq.raycast(pos, Vec3.WORLD_UP, 4.0, tmpHit)) {
                    c0 = tmpHit.point.y;
                    nc++;
                }
                Vec3 lookDir = (speed > 0.5)
                        ? vt.projectPlane(Vec3.WORLD_UP).norm(headingFwd())
                        : headingFwd();
                if (wq.raycast(pos.add(lookDir.mul(d.bodyHalf[2] + 1.2)), Vec3.WORLD_UP, 4.0, tmpHit)) {
                    if (nc == 0) c0 = tmpHit.point.y;
                    else c1 = tmpHit.point.y;
                    nc++;
                }
                if (nc == 0) {
                    hasCeilY = false;
                } else {
                    hasCeilY = true;
                    ceilY = (nc == 2 && c1 < c0) ? c1 : c0;
                }
            }
        } else {
            hasCeilY = false;
        }

        supT += dt;
        mantleT += dt;
        for (int i = 0; i < n; i++) {
            Leg l = legs[i];
            if (l.planted) {
                l.hasCarry = false;
            } else {
                l.hasCarry = true;
                l.carry = frame.toLocal(l.foot.sub(pos));
            }
        }

        if (inp.crouch > 0.5 && prev.crouch <= 0.5) crouchToggle = !crouchToggle;
        if (inp.climb > 0.5 && prev.climb <= 0.5 && d.climber) climbEnabled = !climbEnabled;
        if (inp.gait > 0.5 && prev.gait <= 0.5) cycleGaitOverride();
        if (inp.impact > 0.5 && prev.impact <= 0.5) triggerImpact();
        boolean jump = inp.jump > 0.5 && prev.jump <= 0.5;

        stateT += dt;
        absorb = Math.max(0.0, absorb - dt * (0.9 + 1.5 * absorb));
        brace = Math.max(brace * Math.exp(-1.4 * dt), GaitCore.clamp(slideVel.len() / 5.0, 0.0, 1.0));
        crouch += ((crouchToggle ? 1.0 : 0.0) - crouch) * GaitCore.damp(6.0, dt);

        computeDriveIntent();
        selectMode(dt);
        if (profEn) {
            double t1 = GaitCore.timeSec();
            prof[0] += t1 - t0;
            t0 = t1;
        }

        if (state == GaitCore.STATE_MANTLE) {
            updateMantle(dt);
        } else if (state == GaitCore.STATE_AIR) {
            updateAir(dt);
        } else {
            updateGround(dt, jump);
        }

        updateOrientation(dt);
        updateVisOffset(dt);
        if (profEn) {
            double t1 = GaitCore.timeSec();
            prof[1] += t1 - t0;
        }
        Scheduler.updateLegs(this, dt);
        prev.copyFrom(inp);
    }

    String modeLabel() {
        if (state == GaitCore.STATE_AIR) return "AIRBORNE";
        if (wireActive()) return "WIRE";
        if (state == GaitCore.STATE_MANTLE) return "MANTLE";
        if (state == GaitCore.STATE_CROUCHJUMP) return "JUMP";
        if (drifting) return "SLIDE";
        if (onWall) return "CLIMB";
        if (crouch > 0.5) return "CROUCH";
        if (mode == GaitCore.MODE_RUN) return "RUN";
        if (mode == GaitCore.MODE_SLOW) return "SLOW WALK";
        return "WALK";
    }

    int flags() {
        int f = 0;
        if (drifting) f |= 1;
        if (onWall) f |= 2;
        if (crouch > 0.5) f |= 4;
        if (climbEnabled) f |= 8;
        if (support >= 0.0) f |= 16;
        if (mode == GaitCore.MODE_SLOW) f |= 32;
        if (mode == GaitCore.MODE_RUN) f |= 64;
        if (wireActive()) f |= 128;
        if (flightT > 0.0) f |= 256;
        return f;
    }

    String gaitLabel() {
        String b = blend.label();
        String r = selector.reason;
        if (!r.isEmpty()
                && !"walking".equals(r)
                && !"fast".equals(r)
                && !"galloping".equals(r)
                && !"careful".equals(r)
                && !"forced".equals(r)) {
            return b + " (" + r + ")";
        }
        return b;
    }

    int floatsNeeded() {
        return GaitCore.HEADER_FLOATS + n * (jointsPerLeg * 3 + GaitCore.LEG_EXTRA_FLOATS);
    }

    int pack(float[] out, int off, int maxFloats) {
        int need = floatsNeeded();
        if (maxFloats < need) {
            return 0;
        }
        Vec3.Frame fr = frame;
        Vec3 p = pos.add(fr.toWorld(visOffset));
        int nPl = 0;
        for (int i = 0; i < n; i++) {
            if (legs[i].planted) nPl++;
        }

        int k = off;
        out[k++] = (float) p.x; out[k++] = (float) p.y; out[k++] = (float) p.z;
        out[k++] = (float) fr.right.x; out[k++] = (float) fr.right.y; out[k++] = (float) fr.right.z;
        out[k++] = (float) fr.up.x; out[k++] = (float) fr.up.y; out[k++] = (float) fr.up.z;
        out[k++] = (float) fr.fwd.x; out[k++] = (float) fr.fwd.y; out[k++] = (float) fr.fwd.z;
        out[k++] = (float) speed;
        out[k++] = (float) state;
        out[k++] = (float) flags();
        out[k++] = (float) nPl;
        out[k++] = (float) vel.x; out[k++] = (float) vel.y; out[k++] = (float) vel.z;
        out[k++] = (float) phase;
        out[k++] = (float) driftAmt;
        out[k++] = (float) gaitIdx;
        out[k++] = (float) support;
        out[k++] = (float) cycle;
        if (wireActive()) {
            out[k++] = 1.0f;
            out[k++] = (float) mantle.top.x;
            out[k++] = (float) mantle.top.y;
            out[k++] = (float) mantle.top.z;
        } else {
            out[k++] = 0.0f; out[k++] = 0.0f; out[k++] = 0.0f; out[k++] = 0.0f;
        }

        int J = jointsPerLeg;
        for (int li = 0; li < n; li++) {
            Leg l = legs[li];
            for (int ji = 0; ji < J; ji++) {
                Vec3 j = (ji < l.nJoints) ? l.joints[ji] : l.foot;
                out[k++] = (float) j.x;
                out[k++] = (float) j.y;
                out[k++] = (float) j.z;
            }
            out[k++] = (float) l.planeNormal.x;
            out[k++] = (float) l.planeNormal.y;
            out[k++] = (float) l.planeNormal.z;
            out[k++] = l.planted ? 1.0f : 0.0f;
            out[k++] = l.sliding ? 1.0f : 0.0f;
            out[k++] = l.swing.active ? (float) l.swing.t : -1.0f;
        }
        return k - off;
    }

    private static int pushDbgLine(float[] out, int cur, int maxFloats, Vec3 a, Vec3 b, int role) {
        if (cur + GaitCore.DEBUG_LINE_FLOATS > maxFloats) {
            return cur;
        }
        out[cur] = (float) a.x; out[cur + 1] = (float) a.y; out[cur + 2] = (float) a.z;
        out[cur + 3] = (float) b.x; out[cur + 4] = (float) b.y; out[cur + 5] = (float) b.z;
        out[cur + 6] = (float) role;
        out[cur + 7] = 0.0f;
        out[cur + 8] = 0.0f;
        return cur + GaitCore.DEBUG_LINE_FLOATS;
    }

    private static int pushDbgCross(float[] out, int cur, int maxFloats, Vec3.Frame fr, Vec3 p, double sz, int role) {
        cur = pushDbgLine(out, cur, maxFloats, p.sub(fr.right.mul(sz)), p.add(fr.right.mul(sz)), role);
        cur = pushDbgLine(out, cur, maxFloats, p.sub(fr.fwd.mul(sz)), p.add(fr.fwd.mul(sz)), role);
        return cur;
    }

    int debugLines(float[] out, int maxFloats) {
        int cur = 0;
        Vec3.Frame fr = frame;
        Vec3 upP = probeUp();
        Vec3 p = pos;
        cur = pushDbgLine(out, cur, maxFloats, p, p.add(fr.right.mul(1.2)), GaitCore.DBG_AXIS_X);
        cur = pushDbgLine(out, cur, maxFloats, p, p.add(fr.up.mul(1.2)), GaitCore.DBG_AXIS_Y);
        cur = pushDbgLine(out, cur, maxFloats, p, p.add(fr.fwd.mul(1.2)), GaitCore.DBG_AXIS_Z);

        Vec3 t1 = fr.fwd.projectPlane(upP).norm(new Vec3(0.0, 0.0, 1.0));
        Vec3 t2 = upP.cross(t1);
        double zone = d.zone;

        for (int i = 0; i < n; i++) {
            Leg l = legs[i];
            Vec3 c = l.idealNow;
            Vec3 prevPt = Vec3.ZERO;
            for (int k = 0; k < 17; k++) {
                double a = k * GaitCore.TAU / 16.0;
                Vec3 q = c.add(t1.mul(Math.cos(a) * zone)).add(t2.mul(Math.sin(a) * zone));
                if (k > 0) {
                    cur = pushDbgLine(out, cur, maxFloats, prevPt, q, GaitCore.DBG_LINE_DIM);
                }
                prevPt = q;
            }
            cur = pushDbgCross(out, cur, maxFloats, fr, l.idealPred, 0.15, GaitCore.DBG_LINE);
            if (l.swing.active) {
                cur = pushDbgCross(out, cur, maxFloats, fr, l.swing.target, 0.2, GaitCore.DBG_HIGHLIGHT);
                l.curveWorld(12, tmpCurvePts);
                for (int k = 0; k < 12; k++) {
                    cur = pushDbgLine(out, cur, maxFloats, tmpCurvePts[k], tmpCurvePts[k + 1], GaitCore.DBG_INFO);
                }
            }
            cur = pushDbgLine(out, cur, maxFloats, l.foot, l.foot.add(l.footNormal.mul(0.5)),
                    l.planted ? GaitCore.DBG_NORMAL : GaitCore.DBG_LINE_DIM);
        }

        double ySum = 0.0;
        int yCnt = 0;
        for (int i = 0; i < n; i++) {
            if (legs[i].planted) {
                ySum += legs[i].foot.y;
                yCnt++;
            }
        }
        if (hullN > 0 && yCnt > 0) {
            double y = ySum / yCnt + 0.05;
            int role = support >= 0.0 ? GaitCore.DBG_GOOD : GaitCore.DBG_ERROR;
            for (int i = 0; i < hullN; i++) {
                Vec3.Vec2 a = hull[i];
                Vec3.Vec2 b = hull[(i + 1) % hullN];
                cur = pushDbgLine(out, cur, maxFloats, new Vec3(a.x, y, a.z), new Vec3(b.x, y, b.z), role);
            }
            cur = pushDbgLine(out, cur, maxFloats, p, new Vec3(p.x, y, p.z), GaitCore.DBG_LINE);
            cur = pushDbgCross(out, cur, maxFloats, fr, new Vec3(p.x, y, p.z), 0.25, GaitCore.DBG_LINE);
        }
        return cur;
    }

    private static final String[] STATE_NAMES = { "ground", "air", "mantle", "jump-crouch" };

    void fillStats(float simMs, int totalRays, GcMechStats out) {
        out.mode = modeLabel();
        out.gait = gaitLabel();
        out.gaitOverride = gaitOverride >= 0 ? 1 : 0;
        int st = (state >= 0 && state < 4) ? state : 0;
        out.state = STATE_NAMES[st];
        out.duty = (float) duty;
        out.cycle = (float) cycle;
        double ms = 0.0;
        for (int i = 0; i < n; i++) {
            if (legs[i].stretch > ms) ms = legs[i].stretch;
        }
        out.maxStretch = (float) ms;
        out.simMs = simMs;
        out.rays = totalRays;
        out.climbEnabled = climbEnabled ? 1 : 0;
        out.crouchToggle = crouchToggle ? 1 : 0;
    }
}
