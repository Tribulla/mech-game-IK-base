package gaitcore;

/**
 * Knee obstacle avoidance (radial leg-plane roll), swing corridor spacing, and air landing braces.
 * Mirrors {@code gc_clearance.c}.
 */
final class Clearance {

    private Clearance() {
    }

    /**
     * Checks intermediate joints against world colliders, ignoring harmless ankle grazing right at
     * the planted floor surface so flat terrain never triggers false knee roll.
     */
    private static double jointsClearRoll(
            Mech m, Vec3[] pts, int nj, int startJ, Vec3 foot, double r, Vec3 up,
            boolean planted, boolean[] outClear) {
        int lastInt = nj - 2;
        boolean clear = true;
        double minDist = 1e9;
        WorldQueryCtx.PushResult pr = m.tmpPush;
        for (int jIdx = startJ; jIdx <= lastInt; jIdx++) {
            Vec3 q = pts[jIdx];
            if (m.wq.spherePush(q, r, pr) && pr.center.sub(q).lenSq() > 1e-6) {
                double pen = pr.center.sub(q).len();
                double surfDy = q.sub(pr.normal.mul(r - pen)).sub(foot).dot(up);
                if (planted && jIdx == lastInt && pr.normal.dot(up) > 0.96 && pen < 0.70 * r && Math.abs(surfDy) < 0.08) {
                    if (0.05 < minDist) {
                        minDist = 0.05;
                    }
                } else {
                    clear = false;
                    if (-pen < minDist) {
                        minDist = -pen;
                    }
                }
            } else {
                if (0.05 < minDist) {
                    minDist = 0.05;
                }
            }
        }
        outClear[0] = clear;
        return minDist;
    }

    static void solveLeg(Mech m, Leg l, double dt) {
        boolean prof = m.profileEnabled;
        double t0 = prof ? GaitCore.timeSec() : 0.0;
        Vec3 mountL = l.mountLocal.add(m.visOffset);
        boolean hasPrevRoll = false;
        double prevRoll = 0.0;

        if (m.d.plane == GaitCore.PLANE_RADIAL && Math.abs(l.planeRoll) > 1e-4) {
            hasPrevRoll = true;
            prevRoll = l.planeRoll;
            double relaxed = prevRoll * Math.exp(-5.0 * dt);
            l.planeRoll = (Math.abs(relaxed) < 1e-3) ? 0.0 : relaxed;
        }
        l.solve(m, mountL, dt);
        if (l.planted && !l.sliding && l.ik.err > 0.02) {
            // Planted foot pulled out of reach: drop stance cleanly and re-search.
            l.planted = false;
            l.searching = true;
            l.searchCd = 0.0;
            l.foot = l.joints[l.nJoints - 1];
        }
        WorldQueryCtx.PushResult pr = m.tmpPush;
        if (!l.planted && l.ik.err > 0.02) {
            for (int it = 0; it < 2; it++) {
                Vec3 e = l.joints[l.nJoints - 1];
                if (!m.wq.spherePush(e, l.footR, pr) || pr.center.sub(e).lenSq() < 1e-6) {
                    break;
                }
                l.foot = pr.center;
                l.solve(m, mountL, dt);
            }
        }
        if (prof) {
            double t1 = GaitCore.timeSec();
            m.prof[6] += t1 - t0;
            m.prof[8] += 1.0;
            t0 = t1;
        }

        if (m.planeRollEnabled && m.d.plane == GaitCore.PLANE_RADIAL && l.nJoints >= 4) {
            Vec3 up = m.probeUp();
            int startJ = l.ik.coxa > 1e-6 ? 2 : 1;
            int lastInt = l.nJoints - 2;
            double r = l.footR * 0.85;
            boolean hasHit = false;
            Vec3 hitPush = Vec3.ZERO;
            double bestPen = 0.0;
            for (int jIdx = startJ; jIdx <= lastInt; jIdx++) {
                Vec3 j = l.joints[jIdx];
                if (m.wq.spherePush(j, r, pr)) {
                    if (l.planted && jIdx == lastInt && pr.normal.dot(up) > 0.96) {
                        double pen = pr.center.sub(j).len();
                        double surfDy = j.sub(pr.normal.mul(r - pen)).sub(l.foot).dot(up);
                        if (pen < 0.70 * r && Math.abs(surfDy) < 0.08) {
                            continue;
                        }
                    }
                    Vec3 dv = pr.center.sub(j);
                    double p2 = dv.lenSq();
                    if (p2 > bestPen && p2 > 1e-6) {
                        bestPen = p2;
                        hitPush = dv;
                        hasHit = true;
                    }
                }
            }
            if (hasHit) {
                Vec3.Frame fr = m.frame;
                Vec3 pos = m.pos;
                Vec3 footL = fr.toLocal(l.foot.sub(pos));
                LegIK probe = l.ikProbe;
                probe.hasAngles = l.ik.hasAngles;
                if (probe.hasAngles) {
                    System.arraycopy(l.ik.angles, 0, probe.angles, 0, probe.angles.length);
                }
                probe.hasPts = l.ik.hasPts;
                if (probe.hasPts) {
                    for (int i = 0; i < probe.pts.length; i++) {
                        probe.pts[i][0] = l.ik.pts[i][0];
                        probe.pts[i][1] = l.ik.pts[i][1];
                    }
                }

                Vec3[] jlPrev = m.tmpJoints1;
                Vec3[] ptsPrev = m.tmpJoints2;
                boolean[] outClr = m.tmpBool;

                if (hasPrevRoll) {
                    int nj = probe.solve(mountL, Vec3.LOCAL_FRAME, footL, prevRoll, jlPrev);
                    if (probe.err <= Math.max(0.02, l.ik.err + 0.01)) {
                        for (int i = 0; i < nj; i++) {
                            ptsPrev[i] = pos.add(fr.toWorld(jlPrev[i]));
                        }
                        jointsClearRoll(m, ptsPrev, nj, startJ, l.foot, r, up, l.planted, outClr);
                        if (outClr[0]) {
                            l.planeRoll = prevRoll;
                            l.solve(m, mountL, dt);
                            if (prof) {
                                m.prof[7] += GaitCore.timeSec() - t0;
                            }
                            return;
                        }
                    }
                }

                double baseRoll = hasPrevRoll ? prevRoll : l.planeRoll;
                double bestRoll = baseRoll;
                double bestScore = -Math.sqrt(bestPen) - 0.5 * Math.abs(baseRoll);
                double sideSgn = hitPush.dot(l.planeNormal) >= 0.0 ? 1.0 : -1.0;
                double[] mags = { 0.20, 0.42, 0.65, 0.85, 1.10 };
                double[] cands = new double[10];
                for (int i = 0; i < 5; i++) {
                    cands[i] = sideSgn * mags[i];
                    cands[5 + i] = -sideSgn * mags[i];
                }
                for (int ci = 0; ci < 10; ci++) {
                    double candRoll = cands[ci];
                    if (Math.abs(candRoll - baseRoll) < 1e-4) {
                        continue;
                    }
                    int nj = probe.solve(mountL, Vec3.LOCAL_FRAME, footL, candRoll, jlPrev);
                    if (probe.err > Math.max(0.02, l.ik.err + 0.01)) {
                        continue;
                    }
                    for (int i = 0; i < nj; i++) {
                        ptsPrev[i] = pos.add(fr.toWorld(jlPrev[i]));
                    }
                    double minDist = jointsClearRoll(m, ptsPrev, nj, startJ, l.foot, r, up, l.planted, outClr);
                    double flipPen = (Math.abs(baseRoll) > 0.05 && candRoll * baseRoll < 0.0) ? 0.30 : 0.0;
                    double sc = (outClr[0] ? 10.0 : minDist) - 0.5 * Math.abs(candRoll) - flipPen;
                    if (sc > bestScore) {
                        bestScore = sc;
                        bestRoll = candRoll;
                    }
                    if (outClr[0] && flipPen == 0.0 && candRoll * sideSgn > 0.0) {
                        break;
                    }
                }
                if (hasPrevRoll) {
                    l.planeRoll = prevRoll;
                }
                if (Math.abs(bestRoll - l.planeRoll) > 1e-4) {
                    double maxDr = 1200.0 * GaitCore.DEG2RAD * dt;
                    l.planeRoll += GaitCore.clamp(bestRoll - l.planeRoll, -maxDr, maxDr);
                    l.solve(m, mountL, dt);
                } else if (hasPrevRoll) {
                    l.solve(m, mountL, dt);
                }
            }
            if (prof) {
                m.prof[7] += GaitCore.timeSec() - t0;
            }
        }
    }

    static void slideFootClear(Mech m, Leg l, Vec3 up) {
        double r = l.footR;
        Vec3 c = l.foot.add(up.mul(r));
        WorldQueryCtx.PushResult pr = m.tmpPush;
        if (m.wq.spherePush(c, r, pr) && pr.center.sub(c).lenSq() > 1e-6) {
            l.foot = pr.center.sub(up.mul(r));
            if (pr.normal.dot(up) < 0.55) {
                l.force = true;
            }
        }
    }

    private static void clampSwingLat(Mech m, Leg l, Vec3.Frame fr) {
        if (m.drifting) {
            return;
        }
        Vec3 fl = fr.toLocal(l.foot.sub(m.pos));
        double maxDx = 0.47 * l.reach;
        double dx = fl.x - l.restXz[0];
        if (Math.abs(dx) > maxDx) {
            Vec3 adj = new Vec3(l.restXz[0] + Math.copySign(maxDx, dx), fl.y, fl.z);
            l.foot = m.pos.add(fr.toWorld(adj));
        }
    }

    /** Keeps swinging feet clear of neighboring legs and the hull belly so strides never intersect. */
    static void feetClear(Mech m, double dt) {
        Vec3 up = m.probeUp();
        Vec3.Frame fr = m.frame;
        boolean radial = (m.d.plane == GaitCore.PLANE_RADIAL);
        for (int i = 0; i < m.n; i++) {
            Leg l = m.legs[i];
            if (!l.planted) {
                if (radial && l.swing.active) {
                    Leg.Swing sw = l.swing;
                    double w = GaitCore.smoothstep(sw.t / 0.15) * GaitCore.smoothstep((0.94 - sw.t) / 0.14);
                    if (w > 1e-4) {
                        Vec3 rel = fr.toLocal(l.foot.sub(l.mountWorld));
                        Vec3 rd = l.ik.restDirLocal;
                        double outD = rel.x * rd.x + rel.z * rd.z;
                        double sideD = Math.abs(rel.x * -rd.z + rel.z * rd.x);
                        double restR = Math.hypot(l.restXz[0] - l.mountLocal.x, l.restXz[1] - l.mountLocal.z);
                        double coneOut = (l.ik.coxaMax < 1.5)
                                ? (sideD / Math.tan(Math.max(0.25, l.ik.coxaMax - 0.08)))
                                : 0.0;
                        double needOut = Math.max(0.46 * restR, Math.min(0.85 * restR, coneOut));
                        if (outD < needOut) {
                            l.foot = l.foot.add(fr.toWorld(rd).mul(w * (needOut - outD)));
                        }
                        Vec3 footL = fr.toLocal(l.foot.sub(m.pos));
                        double sideSign = l.restXz[0] >= 0.0 ? 1.0 : -1.0;
                        double minLat = Math.max(0.35, 0.70 * Math.abs(l.mountLocal.x));
                        if (footL.x * sideSign < minLat) {
                            l.foot = l.foot.add(fr.toWorld(new Vec3(sideSign, 0.0, 0.0)).mul(w * (minLat - footL.x * sideSign)));
                        }
                    }
                }
                if (l.swing.active) {
                    clampSwingLat(m, l, fr);
                }
                if (l.swing.active && l.swing.t > 0.05 && l.swing.t < 0.92) {
                    Leg.Swing sw = l.swing;
                    double ws = GaitCore.smoothstep(sw.t / 0.12) * GaitCore.smoothstep((0.94 - sw.t) / 0.14);
                    double minSib = Math.max(0.42, 2.8 * l.footR);
                    for (int ni = 0; ni < m.n; ni++) {
                        Leg nb = m.legs[ni];
                        if (nb.side != l.side || nb == l) {
                            continue;
                        }
                        if (l.ik.mode == GaitCore.PLANE_SAGITTAL) {
                            double df = l.foot.sub(nb.foot).dot(fr.fwd);
                            double ad = Math.abs(df);
                            if (ad < minSib) {
                                double sgn = (ad > 1e-4) ? (df >= 0.0 ? 1.0 : -1.0)
                                        : (l.restXz[1] >= nb.restXz[1] ? 1.0 : -1.0);
                                double pushD = (minSib - ad) * (!nb.planted ? 0.55 : 0.90) * ws;
                                l.foot = l.foot.add(fr.fwd.mul(sgn * pushD));
                            }
                        } else {
                            Vec3 dv = l.foot.sub(nb.foot).projectPlane(up);
                            double dl2 = dv.lenSq();
                            if (1e-8 < dl2 && dl2 < minSib * minSib) {
                                double dl = Math.sqrt(dl2);
                                double pushD = (minSib - dl) * (!nb.planted ? 0.55 : 0.90) * ws;
                                l.foot = l.foot.add(dv.mul(pushD / dl));
                                clampSwingLat(m, l, fr);
                            }
                        }
                    }
                }
                l.keepClear(m.wq, up, m.d.stepMax, m.hasCeilY, m.ceilY, dt);
            }
        }
    }

    private static void easeFoot(Mech m, Leg l, Vec3 target, double k) {
        Vec3.Frame fr = m.frame;
        Vec3 fl = fr.toLocal(l.foot.sub(m.pos));
        Vec3 tl = fr.toLocal(target.sub(m.pos));
        l.foot = m.pos.add(fr.toWorld(fl.lerp(tl, k)));
    }

    private static double braceWeight(Mech m) {
        if (!m.impact.valid || (m.vel.y > 0.0 && m.impact.t > 0.5)) {
            return 0.0;
        }
        double tilt = Vec3.angleBetween(m.frame.up, m.impact.normal);
        double window = 0.45 * Math.sqrt(m.size) * (1.0 + 0.35 * GaitCore.clamp(tilt / 0.8, 0.0, 1.0));
        return GaitCore.clamp(1.0 - (m.impact.t - 0.08) / window, 0.0, 1.0);
    }

    /**
     * Blends airborne legs between a tucked flight pose and an extended impact-brace stance aligned
     * to the predicted landing normal right before touchdown.
     */
    static void legsAir(Mech m, double dt) {
        Vec3.Frame fr = m.frame;
        double b = (m.state == GaitCore.STATE_AIR) ? braceWeight(m) : 0.0;
        double k = (b > 0.0) ? GaitCore.damp(12.0 + 24.0 * b, dt) : GaitCore.damp(10.0, dt);
        double hx = m.d.bodyHalf[0];
        double hy = m.d.bodyHalf[1];
        double hz = m.d.bodyHalf[2];
        boolean hasLandFr = false;
        Vec3.Frame landFr = m.tmpFrame;
        Vec3 n = Vec3.WORLD_UP;
        double spread = 1.0;
        if (b > 0.0 && m.impact.valid) {
            n = m.impact.normal;
            landFr.init(m.headingFwd(), n);
            hasLandFr = true;
            double tilt = Vec3.angleBetween(fr.up, n);
            spread = 1.0 + 0.25 * GaitCore.clamp(m.impact.speed / 15.0, 0.0, 1.0)
                    + 0.18 * GaitCore.clamp(tilt / 0.8, 0.0, 1.0);
        }
        for (int i = 0; i < m.n; i++) {
            Leg l = m.legs[i];
            l.swing.active = false;
            l.planted = false;
            l.searching = false;
            if (l.push) {
                if (l.stretch < 0.95 && m.stateT < 0.25) {
                    continue;
                }
                l.push = false;
            }
            Vec3 tuck = new Vec3(l.restXz[0] * 0.8 + l.mountLocal.x * 0.2, -m.ride * 0.72, l.restXz[1] * 0.85);
            Vec3 pose = m.pos.add(fr.toWorld(tuck));
            if (hasLandFr) {
                Vec3 relLand = new Vec3(l.restXz[0] * spread, -m.ride * 1.05, l.restXz[1] * (1.0 + (spread - 1.0) * 0.5));
                Vec3 brace = m.pos.add(landFr.toWorld(relLand));
                Vec3 bl = fr.toLocal(brace.sub(m.pos));
                if (Math.abs(bl.x) < hx + 0.15 && Math.abs(bl.y) < hy + 0.15 && Math.abs(bl.z) < hz + 0.15) {
                    if (m.d.plane == GaitCore.PLANE_SAGITTAL) {
                        bl = new Vec3(bl.x, Math.min(bl.y, -(hy + 0.18)), bl.z);
                    } else {
                        double sgnX = l.mountLocal.x >= 0.0 ? 1.0 : -1.0;
                        bl = new Vec3(sgnX * Math.max(Math.abs(bl.x), hx + 0.22), bl.y, bl.z);
                    }
                    brace = m.pos.add(fr.toWorld(bl));
                }
                Vec3 mountW = m.pos.add(fr.toWorld(l.mountLocal));
                Vec3 dv = brace.sub(mountW);
                double dist = dv.len();
                double minR = 0.30 * l.reach;
                double maxR = 0.94 * l.reach;
                if (dist < minR) {
                    Vec3 dirV = (dist > 1e-4) ? dv.mul(1.0 / dist) : n.mul(-1.0);
                    brace = mountW.add(dirV.mul(minR));
                } else if (dist > maxR) {
                    brace = mountW.add(dv.mul(maxR / dist));
                }
                pose = pose.lerp(brace, b);
            }
            easeFoot(m, l, pose, k);
        }
    }

    static double clearance(Mech m) {
        if (m.state == GaitCore.STATE_MANTLE) {
            return m.d.mantleMax + 0.6;
        }
        return m.d.stepMax + m.d.stepHeight + 0.4;
    }
}
