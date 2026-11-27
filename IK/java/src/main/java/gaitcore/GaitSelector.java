package gaitcore;

import java.util.Arrays;

/**
 * Forward terrain lookahead, speed/ground gait-ladder hysteresis, and phase blending.
 * Mirrors {@code gc_gaitsel.c}.
 */
final class GaitSelector {

    static final class Situation {
        double aheadRough;
        double aheadSlope;
        double aheadVoid;
        double underRough;
        double clutter;
        double edge;
        double harsh;
        double slope;
        double stanceSlope;
        String reason = "";
        int n;
        double probeT = 1.0;
        final double[] probeRaw = new double[4];
        final double[] probePrev = new double[4];
        final double[] hold = new double[3];

        void init() {
            aheadRough = 0.0;
            aheadSlope = 0.0;
            aheadVoid = 0.0;
            underRough = 0.0;
            clutter = 0.0;
            edge = 0.0;
            harsh = 0.0;
            slope = 0.0;
            stanceSlope = 0.0;
            reason = "";
            n = 0;
            probeT = 1.0;
            Arrays.fill(probeRaw, 0.0);
            Arrays.fill(probePrev, 0.0);
            Arrays.fill(hold, 0.0);
        }

        private static final class ProbeHitRec {
            int ptIdx;
            Vec3 point = Vec3.ZERO;
            Vec3 normal = Vec3.WORLD_UP;
        }

        private final ProbeHitRec[] tmpHits = {
                new ProbeHitRec(), new ProbeHitRec(), new ProbeHitRec(), new ProbeHitRec(), new ProbeHitRec()
        };
        private final double[] tmpYsAll = new double[5];
        private final Vec3[] tmpCl = new Vec3[5];
        private final double[] tmpClRough = new double[3];
        private final double[] tmpIntra = new double[8];
        private final double[] tmpInter = new double[4];
        private final boolean[] tmpHasPt = new boolean[5];
        private final double[] tmpPtY = new double[5];
        private final Vec3[] tmpPtN = new Vec3[5];
        private final double[] tmpNyVals = new double[5];
        private final double[] tmpYsCl = new double[5];
        private final Vec3.PlaneFit tmpFit = new Vec3.PlaneFit();

        private void probeAhead(Mech m, double[] out4) {
            WorldQueryCtx wq = m.wq;
            Vec3 up = Vec3.WORLD_UP;
            Vec3 fwd = (m.speed > 1.0) ? m.vt.projectPlane(up).norm(m.headingFwd()) : m.headingFwd();
            double curvature = (m.speed > 1.0 && Scheduler.gallopingFast(m, false) && !m.drifting)
                    ? (-m.yawRate / m.speed)
                    : 0.0;
            double hz = m.d.bodyHalf[2];
            double sm = m.d.stepMax;
            double accR = m.d.accelRun > 0.0 ? m.d.accelRun : m.d.accel;
            double decel = Math.max(6.0, 1.4 * accR);
            double brakeD = (m.speed * m.speed) / (2.0 * decel);
            double minL = (m.selector.cap < 3) ? Math.max(2.5 * hz, 9.0) : (2.5 * hz);
            double L = GaitCore.clamp(Math.max(m.speed * 0.65, brakeD + 0.35 * m.speed), minL, 42.0);
            double spread = GaitCore.clamp(0.45 * hz + 0.40, 0.70, 1.15);

            WorldQueryCtx.Hit g0 = m.tmpHit;
            boolean hasG0 = wq.raycast(new Vec3(m.pos.x, m.pos.y + 1.0, m.pos.z), up.mul(-1.0), m.ride + 3.0, g0);
            double base = (hasG0 && g0.normal.dot(up) > 0.64) ? g0.point.dot(up) : (m.pos.dot(up) - m.ride);

            WorldQueryCtx.Hit ceilHit = m.tmpHit2;
            boolean hasMaxOY = wq.raycast(m.pos, up, 3.5, ceilHit);
            double maxOY = hasMaxOY ? (ceilHit.point.dot(up) - 0.15) : 0.0;

            int voidCnt = 0;
            int nProbes = 0;
            int nClRough = 0;
            int nIntra = 0;
            int nInter = 0;

            boolean hasPrevH = true;
            double prevH = base;
            double prevD = 0.0;
            boolean prevFlat = !hasG0 || g0.normal.dot(up) >= 0.96;

            double[] alphas = { 0.22, 0.56, 1.0 };
            double[][] offs = { { 0.0, 0.0 }, { 1.0, 0.0 }, { -1.0, 0.0 }, { 0.0, 1.0 }, { 0.0, -1.0 } };

            for (int ai = 0; ai < 3; ai++) {
                double dist = hz + alphas[ai] * L;
                double angle = curvature * dist;
                Vec3 centre = m.pos.add(Vec3.turnDisplacement(fwd.mul(dist), up, angle));
                Vec3 tangent = (Math.abs(curvature) > 0.0) ? fwd.rotateAxis(up, angle) : fwd;
                Vec3 right = up.cross(tangent);

                int nHits = 0;
                int steepHits = 0;

                for (int oi = 0; oi < 5; oi++) {
                    double dxO = offs[oi][0];
                    double dzO = offs[oi][1];
                    Vec3 p = centre.add(right.mul(dxO * spread)).add(tangent.mul(dzO * spread));
                    double oY = (hasPrevH ? prevH : base) + 2.6 + sm;
                    if (hasMaxOY && oY > maxOY) {
                        oY = maxOY;
                    }
                    Vec3 o = new Vec3(p.x, oY, p.z);
                    double rayD = Math.max(1.0, oY - (base - 2.0 * sm - 0.5 * dist));
                    boolean got = wq.raycast(o, up.mul(-1.0), rayD, g0);
                    nProbes++;
                    if (!got || g0.point.dot(up) < base - 2.0 * sm - 0.5 * dist) {
                        voidCnt++;
                        continue;
                    }
                    if (g0.normal.dot(up) < 0.55) {
                        if (g0.point.dot(up) > base + 0.55 * sm) {
                            steepHits++;
                        }
                        continue;
                    }
                    ProbeHitRec hr = tmpHits[nHits++];
                    hr.ptIdx = oi;
                    hr.point = g0.point;
                    hr.normal = g0.normal;
                }

                if (nHits > 0) {
                    for (int i = 0; i < nHits; i++) {
                        tmpYsAll[i] = tmpHits[i].point.dot(up);
                    }
                    Arrays.sort(tmpYsAll, 0, nHits);
                    double refY = (tmpYsAll[nHits - 1] - tmpYsAll[0] > 3.5 * sm)
                            ? tmpYsAll[nHits - 1]
                            : tmpYsAll[nHits / 2];
                    int writeI = 0;
                    for (int i = 0; i < nHits; i++) {
                        if (tmpHits[i].point.dot(up) >= refY - 2.0 * sm) {
                            if (writeI != i) {
                                ProbeHitRec dst = tmpHits[writeI];
                                ProbeHitRec src = tmpHits[i];
                                dst.ptIdx = src.ptIdx;
                                dst.point = src.point;
                                dst.normal = src.normal;
                            }
                            writeI++;
                        }
                    }
                    voidCnt += (nHits - writeI);
                    nHits = writeI;
                }

                boolean twoLevel = false;
                double stepH = 0.0;
                if (nHits >= 2) {
                    double yLo = 1e18;
                    double yHi = -1e18;
                    boolean allFlatN = true;
                    for (int i = 0; i < nHits; i++) {
                        double y = tmpHits[i].point.dot(up);
                        if (y < yLo) yLo = y;
                        if (y > yHi) yHi = y;
                        if (tmpHits[i].normal.dot(up) < 0.98) {
                            allFlatN = false;
                        }
                    }
                    stepH = yHi - yLo;
                    if (allFlatN) {
                        boolean okLevels = true;
                        for (int i = 0; i < nHits; i++) {
                            double y = tmpHits[i].point.dot(up);
                            if (!((y - yLo <= 0.05) || (yHi - y <= 0.05))) {
                                okLevels = false;
                                break;
                            }
                        }
                        twoLevel = okLevels;
                    }
                }

                for (int i = 0; i < nHits; i++) {
                    Vec3 hp = tmpHits[i].point;
                    tmpCl[i] = new Vec3(hp.x - m.pos.x, hp.y, hp.z - m.pos.z);
                }

                if (nHits < 4) {
                    if (steepHits >= 2) {
                        tmpClRough[nClRough++] = 0.24 * sm;
                    } else if (steepHits == 1) {
                        tmpClRough[nClRough++] = 0.15 * sm;
                    } else if (nHits >= 2) {
                        if (twoLevel) {
                            tmpClRough[nClRough++] = (stepH <= sm + 0.05) ? 0.0 : Math.min(0.155 * sm, 0.5 * stepH);
                        } else if (stepH > 0.16 * sm) {
                            tmpClRough[nClRough++] = 0.5 * stepH;
                        }
                    }
                    hasPrevH = false;
                    prevD = dist;
                    prevFlat = false;
                    continue;
                }

                Arrays.fill(tmpHasPt, false);
                Arrays.fill(tmpPtY, 0.0);
                double minNy = 1e9;
                double nrMin = 1e9;
                double nrMax = -1e9;
                for (int i = 0; i < nHits; i++) {
                    int pid = tmpHits[i].ptIdx;
                    tmpHasPt[pid] = true;
                    tmpPtY[pid] = tmpHits[i].point.dot(up);
                    tmpPtN[pid] = tmpHits[i].normal;
                    double ny = tmpHits[i].normal.dot(up);
                    tmpNyVals[i] = ny;
                    if (ny < minNy) minNy = ny;
                    double nr = tmpHits[i].normal.dot(right);
                    if (nr < nrMin) nrMin = nr;
                    if (nr > nrMax) nrMax = nr;
                }
                Arrays.sort(tmpNyVals, 0, nHits);
                double medNy = tmpNyVals[nHits / 2];

                // Require aligned, coplanar normals before discounting tilted smooth ramps as flat ground.
                boolean isSmoothRamp = tmpHasPt[0] && tmpHasPt[1] && tmpHasPt[2]
                        && Math.abs(tmpPtY[1] - tmpPtY[0]) < 0.04
                        && Math.abs(tmpPtY[2] - tmpPtY[0]) < 0.04
                        && tmpPtN[1].dot(tmpPtN[0]) > 0.995
                        && tmpPtN[2].dot(tmpPtN[0]) > 0.995
                        && minNy < 0.985
                        && (nrMax - nrMin) < 0.08;

                if (Vec3.fitPlane(tmpCl, null, nHits, tmpFit) < 2) {
                    continue;
                }

                double resSum = 0.0;
                for (int i = 0; i < nHits; i++) {
                    double dPl = tmpCl[i].sub(tmpFit.c).dot(tmpFit.n);
                    resSum += dPl * dPl;
                }
                double res = Math.sqrt(resSum / nHits);
                if (twoLevel && steepHits == 0) {
                    res = (stepH <= sm + 0.05) ? 0.0 : Math.min(res, 0.155 * sm);
                }
                if (steepHits >= 2) {
                    res = Math.max(res, 0.24 * sm);
                } else if (steepHits == 1) {
                    res = Math.max(res, 0.15 * sm);
                }
                tmpClRough[nClRough++] = (isSmoothRamp && steepHits == 0) ? 0.0 : res;

                if (isSmoothRamp && medNy < 0.985) {
                    tmpIntra[nIntra++] = Math.acos(GaitCore.clamp(medNy, -1.0, 1.0)) * GaitCore.RAD2DEG;
                }
                if (tmpHasPt[4] && tmpHasPt[0] && tmpHasPt[3]) {
                    double hBack = tmpPtY[4];
                    double hMid = tmpPtY[0];
                    double hFwd = tmpPtY[3];
                    double dh = hFwd - hBack;
                    if (0.45 <= Math.abs(dh) && Math.abs(dh) <= Math.max(1.2 * sm, 1.6 * spread)) {
                        double frac = (hMid - hBack) / dh;
                        if (0.20 <= frac && frac <= 0.80) {
                            tmpIntra[nIntra++] = Math.atan2(Math.abs(dh), 2.0 * spread) * GaitCore.RAD2DEG;
                        }
                    }
                }

                for (int i = 0; i < nHits; i++) {
                    tmpYsCl[i] = tmpCl[i].y;
                }
                Arrays.sort(tmpYsCl, 0, nHits);
                double hm = tmpYsCl[nHits / 2];
                boolean curFlat = medNy >= 0.985 && minNy >= 0.98 && res < 0.05;
                if (hasPrevH) {
                    if (prevFlat && curFlat && (prevH - hm) > 2.5 * sm) {
                        voidCnt += 2;
                    } else {
                        double dxStep = Math.max(2.5, dist - prevD);
                        double dhInter = Math.abs(hm - prevH);
                        if (dhInter <= 0.76 * dxStep) {
                            tmpInter[nInter++] = Math.atan2(dhInter, dxStep) * GaitCore.RAD2DEG;
                        }
                    }
                }
                hasPrevH = true;
                prevH = hm;
                prevD = dist;
                prevFlat = curFlat;
            }

            double slopeVal = 0.0;
            for (int i = 0; i < nInter; i++) {
                if (tmpInter[i] > slopeVal) slopeVal = tmpInter[i];
            }
            for (int i = 0; i < nIntra; i++) {
                if (tmpIntra[i] > slopeVal) slopeVal = tmpIntra[i];
            }

            double rough = 0.0;
            if (nClRough > 0) {
                Arrays.sort(tmpClRough, 0, nClRough);
                double mid = (nClRough >= 3) ? tmpClRough[nClRough / 2] : (nClRough == 2 ? tmpClRough[0] : 0.0);
                double peekCap = Math.min(0.055, 0.075 * sm) + 0.36 * 0.32 * sm;
                rough = Math.max(mid, Math.min(0.45 * tmpClRough[nClRough - 1], peekCap));
            }

            out4[0] = rough;
            out4[1] = slopeVal;
            out4[2] = (double) voidCnt / (nProbes > 0 ? nProbes : 1);
            out4[3] = hz + L;
        }

        private static double emaStep(double cur, double x, double dt, double upTau, double downTau) {
            double tau = (x > cur) ? upTau : downTau;
            return cur + (x - cur) * (1.0 - Math.exp(-dt / tau));
        }

        void update(Mech m, double dt) {
            n++;
            double p = m.probePeriod;
            probeT += dt;
            if (probeT >= p - 1e-6) {
                probeT = (probeT >= 2.0 * p) ? 0.0 : (probeT % p);
                System.arraycopy(probeRaw, 0, probePrev, 0, 4);
                probeAhead(m, probeRaw);
            }

            double holdLim = Math.min(0.65, (probeRaw[3] + m.d.bodyHalf[2]) / Math.max(m.speed, 3.0));
            for (int i = 0; i < 3; i++) {
                double cur = (i == 0) ? aheadRough : ((i == 1) ? aheadSlope : aheadVoid);
                double seen = Math.min(probeRaw[i], probePrev[i]);
                if (seen >= cur) {
                    cur = (i == 0) ? emaStep(cur, seen, dt, 0.06, 0.45) : seen;
                    hold[i] = holdLim;
                } else {
                    if (hold[i] > 0.0) {
                        double decay = (Math.abs(m.yawRate) > 0.45 || probeRaw[i] < 0.5 * cur) ? (3.0 * dt) : dt;
                        hold[i] -= decay;
                    }
                    if (hold[i] <= 0.0 || (i != 0 && probeRaw[i] > cur)) {
                        cur = emaStep(cur, probeRaw[i], dt, 0.3, 0.45);
                    }
                }
                if (i == 0) aheadRough = cur;
                else if (i == 1) aheadSlope = cur;
                else aheadVoid = cur;
            }

            underRough = emaStep(underRough, m.footRms, dt, 0.45, 0.45);
            int tried = m.fhCount;
            int bad = m.fhBad;
            m.fhCount = 0;
            m.fhBad = 0;
            if (tried > 0) {
                clutter = emaStep(clutter, (double) bad / tried, dt, 0.25, 0.45);
            } else {
                clutter = emaStep(clutter, 0.0, dt, 0.25, 0.70);
            }

            boolean anyEdge = false;
            for (int i = 0; i < m.n; i++) {
                if (m.legs[i].searching || m.legs[i].voidMark > 0.0) {
                    anyEdge = true;
                    break;
                }
            }
            edge = emaStep(edge, anyEdge ? 1.0 : 0.0, dt, 0.1, 0.45);

            double stSlope = 0.0;
            if (m.hasPlaneNLocal) {
                stSlope = Math.acos(GaitCore.clamp(m.planeNLocal.y, -1.0, 1.0)) * GaitCore.RAD2DEG;
            }
            stanceSlope = emaStep(stanceSlope, stSlope, dt, 0.3, 0.45);
            slope = Math.max(aheadSlope, stanceSlope);

            double sm = m.d.stepMax;
            double baseR = Math.min(0.055, 0.075 * sm);
            double aheadN = GaitCore.clamp((aheadRough - baseR) / (0.32 * sm), 0.0, 1.0);
            double underN = GaitCore.clamp((0.85 * underRough - baseR) / (0.32 * sm), 0.0, 0.38);
            double roughN = Math.max(aheadN, underN);
            double clutterN = GaitCore.clamp((clutter - 0.30) / 0.65, 0.0, 0.38);
            if (clutterN > roughN) {
                harsh = clutterN;
                reason = "cluttered footing";
            } else {
                harsh = roughN;
                reason = "rough ground";
            }
        }
    }

    static final class Blend {
        final Gait src = new Gait();
        int dstIdx;
        final Gait dst = new Gait();
        double t = 1.0;
        double s = 1.0;
        double T = 0.5;
        final Gait eff = new Gait();

        void init(int dstIdx, Gait g) {
            src.copyFrom(g);
            this.dstIdx = dstIdx;
            dst.copyFrom(g);
            t = 1.0;
            s = 1.0;
            T = 0.5;
            eff.copyFrom(g);
        }

        void start(int dstIdx, Gait dstGait, double duration) {
            src.copyFrom(eff);
            this.dstIdx = dstIdx;
            dst.copyFrom(dstGait);
            t = 0.0;
            T = Math.max(duration, 0.05);
        }

        void update(double dt) {
            t = Math.min(1.0, t + dt / T);
            double sv = s = GaitCore.smoothstep(t);
            Gait e = eff;
            Gait a = src;
            Gait d = dst;

            e.duty = a.duty + (d.duty - a.duty) * sv;
            e.stride = a.stride + (d.stride - a.stride) * sv;
            e.cycleMin = a.cycleMin + (d.cycleMin - a.cycleMin) * sv;
            e.cycleMax = a.cycleMax + (d.cycleMax - a.cycleMax) * sv;
            e.lift = a.lift + (d.lift - a.lift) * sv;
            e.reach = a.reach + (d.reach - a.reach) * sv;
            e.leap = a.leap + (d.leap - a.leap) * sv;
            e.stridePitch = a.stridePitch + (d.stridePitch - a.stridePitch) * sv;

            for (int i = 0; i < e.nLegs; i++) {
                e.offsets[i] = Gait.phaseLerp(a.offsets[i], d.offsets[i], sv);
            }

            boolean done = sv >= 0.85;
            e.name = done ? d.name : a.name;
            e.minSupport = done ? d.minSupport : Math.max(a.minSupport, d.minSupport);
            e.staticStable = done ? d.staticStable : (a.staticStable || d.staticStable);
            e.adjacentLock = done ? d.adjacentLock : (a.adjacentLock && d.adjacentLock);
            if (done) {
                e.groupCount = d.groupCount;
                System.arraycopy(d.groupSizes, 0, e.groupSizes, 0, GaitCore.MAX_GROUPS);
                for (int gi = 0; gi < GaitCore.MAX_GROUPS; gi++) {
                    System.arraycopy(d.groups[gi], 0, e.groups[gi], 0, GaitCore.MAX_GROUP_SIZE);
                }
            } else {
                e.groupCount = 0;
            }
            e.pairSync = done && d.pairSync;
            e.phaseLock = done && d.phaseLock;
            e.hasDutyFast = d.hasDutyFast;
            e.dutyFast = d.dutyFast;
        }

        /** Keeps minSupport tracking live duty mid-blend so transitions do not stall early. */
        void fitSupport(double duty) {
            if (t >= 1.0 || GaitCore.smoothstep(t) >= 0.85) {
                return;
            }
            int lo = Math.min(src.minSupport, dst.minSupport);
            int hi = Math.max(src.minSupport, dst.minSupport);
            int val = (int) (duty * eff.nLegs + 0.05);
            eff.minSupport = (int) GaitCore.clamp(val, lo, hi);
        }

        String label() {
            if (t >= 1.0) {
                return dst.name;
            }
            return src.name + " > " + dst.name;
        }
    }

    final int[] ladder = new int[4];
    int rung = 1;
    double dwell = 10.0;
    String reason = "";
    int cap = 3;
    double lowT;
    String capWhy = "";
    double capT = 10.0;
    boolean hasHoldV;
    double holdV;
    double relaxWait = GaitCore.RELAX_T;
    double sinceRelax = 10.0;
    boolean relaxed;
    double betterT;
    double prevV;
    double accelV;

    private final double[] tmpCaps = new double[4];

    void init(Mech m) {
        MechDef d = m.d;
        int fast = d.gaitWalkFastIdx >= 0 ? d.gaitWalkFastIdx
                : (d.gaitDriftIdx >= 0 ? d.gaitDriftIdx : d.gaitWalkIdx);
        ladder[0] = d.gaitSlowIdx;
        ladder[1] = d.gaitWalkIdx;
        ladder[2] = fast;
        ladder[3] = d.gaitRunIdx;
        rung = 1;
        dwell = 10.0;
        reason = "";
        cap = 3;
        lowT = 0.0;
        capWhy = "";
        capT = 10.0;
        hasHoldV = false;
        holdV = 0.0;
        relaxWait = GaitCore.RELAX_T;
        sinceRelax = 10.0;
        relaxed = false;
        betterT = 0.0;
        prevV = 0.0;
        accelV = 0.0;
    }

    void caps(Mech m, double[] out4) {
        out4[0] = m.vSlow;
        out4[1] = m.vWalk;
        out4[2] = m.d.speedWalk;
        out4[3] = m.d.speedRun;
    }

    private String capWhyOut = "";

    private int selectorCap(Situation sit, double relax) {
        double h = sit.harsh + 0.12 * relax;
        int capR = 3;
        String why = "";
        if (sit.harsh + 0.28 * relax > GaitCore.H_WALK) {
            capR = 0;
            why = sit.reason;
        } else if (h > GaitCore.H_FAST) {
            capR = 1;
            why = sit.reason;
        } else if (h > GaitCore.H_GALLOP) {
            capR = 2;
            why = sit.reason;
        }
        if (sit.slope + 4.0 * relax > GaitCore.SLOPE_WALK && capR > 1) {
            capR = 1;
            why = "steep slope";
        } else if (sit.slope + 4.0 * relax > GaitCore.SLOPE_FAST && capR > 2) {
            capR = 2;
            why = "steep slope";
        }
        double dropSig = Math.max(sit.edge, sit.aheadVoid);
        if ((dropSig + 0.15 * relax > 0.3) && capR > 2) {
            capR = 2;
            why = (sit.aheadVoid >= sit.edge) ? "drop ahead" : "footing lost";
        }
        capWhyOut = why;
        return capR;
    }

    static final class SelectOut {
        boolean hasCap;
        double cap;
    }

    int select(Mech m, double dt, SelectOut out) {
        MechDef d = m.d;
        dwell += dt;
        Situation sit = m.situation;
        double v = m.speed;
        double instA = GaitCore.clamp((v - prevV) / Math.max(dt, 1e-4), -30.0, 30.0);
        prevV = v;
        accelV += (instA - accelV) * (1.0 - Math.exp(-dt / 0.15));

        out.hasCap = false;
        out.cap = 0.0;
        if (m.gaitOverride >= 0 || m.drifting || m.climbingMode()) {
            hasHoldV = false;
        }
        if (m.gaitOverride >= 0) {
            reason = "forced";
            return m.gaitOverride;
        }
        if (m.drifting) {
            reason = "slide";
            return d.gaitDriftIdx >= 0 ? d.gaitDriftIdx : ladder[1];
        }
        if (m.climbingMode()) {
            reason = "climbing";
            return ladder[1];
        }

        double[] caps = tmpCaps;
        caps(m, caps);
        int lo = (m.mode == GaitCore.MODE_SLOW) ? 0 : 1;
        int hi = (m.mode == GaitCore.MODE_RUN) ? 3 : 2;

        double[] upThr = { 0.9 * caps[0], 0.9 * caps[1], 0.55 * d.speedRun };
        double[] downThr = { 0.75 * caps[0], 0.75 * caps[1], 0.42 * d.speedRun };
        int r = (int) GaitCore.clamp(rung, lo, hi);
        v = m.speed;

        if (r > lo && v < downThr[r - 1]) {
            if (r < 3 && m.vDesired >= downThr[r - 1] && accelV > 1.0) {
                lowT = Math.max(0.0, lowT - dt);
            } else {
                lowT += dt;
            }
        } else {
            lowT = 0.0;
        }
        double lowLim = (r == 3 && v < 0.40 * d.speedRun) ? 0.15 : 0.85;
        double vDn = (lowT > lowLim) ? v : Math.max(v, 0.9 * m.vDesired);
        while (r < hi && v > upThr[r]) {
            r++;
        }
        while (r > lo && vDn < downThr[r - 1]) {
            r--;
        }

        String rsn = r == 3 ? "galloping" : (r == 2 ? "fast" : (r == 1 ? "walking" : "careful"));
        boolean turning = Math.abs(m.inp.turn) > 0.35;
        if (m.lateralRatio > 1.6 && !turning && r > 2) {
            r = 2;
            rsn = "strafing";
        }

        int enterCap = selectorCap(sit, 0.0);
        String whyIn = capWhyOut;
        int exitCap = selectorCap(sit, 1.0);

        capT += dt;
        sinceRelax += dt;
        betterT = (exitCap > cap) ? (betterT + dt) : 0.0;
        if (sinceRelax > 2.0 * GaitCore.RELAX_T_MAX) {
            relaxWait = GaitCore.RELAX_T;
        }
        double wait = (cap == 0 && exitCap >= 2 && sit.harsh < 0.20) ? (0.60 * relaxWait) : relaxWait;
        if (enterCap < cap) {
            if (relaxed && sinceRelax < 3.0) {
                relaxWait = Math.min(GaitCore.RELAX_T_MAX, 2.0 * relaxWait);
            }
            cap = enterCap;
            capWhy = whyIn;
            capT = 0.0;
            relaxed = false;
            betterT = 0.0;
        } else if (betterT > wait) {
            cap += 1;
            capT = 0.0;
            sinceRelax = 0.0;
            relaxed = true;
            betterT = 0.0;
            if (cap >= 3) {
                capWhy = "";
            }
        }

        int capR = cap;
        String why = capWhy;
        boolean hasSpeedCap = false;
        double speedCap = 0.0;

        if (r > rung && r > exitCap) {
            r = Math.max(rung, exitCap);
        }
        if (r > capR) {
            if (rung > capR && enterCap > capR) {
                if (rung < r) {
                    r = rung;
                }
            } else {
                r = capR;
            }
            rsn = why;
        }

        int floorR = 0;
        while (floorR < 3 && v > 1.1 * caps[floorR]) {
            floorR++;
        }
        if (r < rung && floorR > r) {
            r = Math.min(rung, floorR);
        }

        m.calmGait = (capR < 3 || sit.edge > 0.3 || m.crouch > 0.5) && r < 3;
        if (m.crouch > 0.5) {
            m.calmGait = true;
        }

        if (capR < 3) {
            hasSpeedCap = true;
            if (capR < 2) {
                speedCap = 0.75 * caps[capR];
            } else {
                double groundedCap = 0.85 * m.chordAvg / Math.max(d.swingMin, 0.10);
                speedCap = Math.min(caps[2], Math.max(1.35 * caps[1], groundedCap));
                if (sit.slope > GaitCore.SLOPE_FAST) {
                    double slopeK = (sit.harsh < 0.08) ? 15.5 : 10.5;
                    double slopeCap = (slopeK * d.stepMax) / Math.max(0.25, Math.tan(sit.slope * GaitCore.DEG2RAD));
                    speedCap = Math.min(speedCap, Math.max(1.35 * caps[1], slopeCap));
                }
            }
        }

        if (r != rung) {
            double minDwell = GaitCore.clamp(0.9 * m.cycle, 0.55, 0.85);
            boolean safety = (r < rung && r == capR && !why.isEmpty())
                    || (rung == 3 && r < 3 && v < 0.40 * d.speedRun);
            if (dwell < minDwell && !(safety && dwell > 0.15)) {
                r = rung;
            } else {
                if (r > rung && rung < 2) {
                    hasHoldV = true;
                    holdV = caps[rung];
                } else {
                    hasHoldV = false;
                }
                rung = r;
                dwell = 0.0;
            }
        }

        reason = rsn;
        out.hasCap = hasSpeedCap;
        out.cap = speedCap;
        return ladder[r];
    }
}
