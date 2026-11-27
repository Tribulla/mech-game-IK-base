package gaitcore;

/**
 * Step scheduling, swing-arc tracking, support/stability guards, and bounding flight timing across
 * 2-, 4-, 6-, and 8-legged rigs. Mirrors {@code gc_scheduler.c}.
 */
final class Scheduler {

    private Scheduler() {
    }

    static final class PredCtx {
        final Vec3.Frame fr = new Vec3.Frame();
        double ride;
        boolean dynamic;
        Vec3 travel = Vec3.ZERO;
        Vec3 lead = Vec3.ZERO;
        boolean hasDir;
        Vec3 direction = Vec3.ZERO;
        double leadCap;
    }

    private static boolean fastHopper(Mech m) {
        return m.d.bound * m.gait.leap >= 0.015;
    }

    static double vLeg(Mech m) {
        return 1.0 * m.reachAvg / Math.max(0.05, m.duty * m.gait.cycleMin * Math.sqrt(m.size));
    }

    static boolean gallopingFast(Mech m, boolean checkBlend) {
        GaitSelector.Blend b = m.blend;
        boolean flying = (m.gait.minSupport == 0)
                || (checkBlend && b.t < 1.0 && (b.src.minSupport == 0 || b.dst.minSupport == 0));
        return flying && m.speed >= Math.max(0.5 * vLeg(m), m.vWalk);
    }

    static boolean mayFlight(Mech m) {
        if (m.duty >= 0.48 || m.gait.minSupport != 0 || m.calmGait || m.braking
                || m.state != GaitCore.STATE_GROUND || m.onWall || m.drifting || m.crouch >= 0.5) {
            return false;
        }
        if (m.gait.hasDutyFast && !fastHopper(m)) {
            return false;
        }
        double b = m.d.bound * m.gait.leap;
        double vl = vLeg(m);
        double k = GaitCore.clamp((m.speed - 0.3 * vl) / (0.7 * vl), 0.0, 1.0);
        double hMin = m.gait.hasDutyFast ? 0.016 : 0.03;
        return b * k > hMin;
    }

    private static double leadCap(Mech m) {
        return Math.min(m.d.zone * 1.2 * m.gait.reach, m.reachAvg * 0.75);
    }

    static Vec3 planarTravel(Mech m, double seconds) {
        if (m.drifting) {
            return m.vt.mul(seconds);
        }
        Vec3 travel = m.vt.mul(seconds);
        if (m.state != GaitCore.STATE_GROUND || m.climbingMode()) {
            return travel;
        }
        Vec3 change = m.driveTarget.sub(m.vt);
        double delta = change.len();
        if (m.driveAccel > 0.0 && delta > 1e-5) {
            double ramp = Math.min(seconds, delta / m.driveAccel);
            travel = travel.add(change.mul(m.driveAccel * ramp * (seconds - 0.5 * ramp) / delta));
        }
        if (gallopingFast(m, false) || (m.speed > 4.0 && Math.abs(m.yawRate) > 0.2)) {
            travel = Vec3.turnDisplacement(travel, Vec3.WORLD_UP, -m.yawRate * seconds);
        }
        return travel;
    }

    private static double legFwdRoom(Mech m, Leg l, Vec3 dirL, double ride) {
        double ox = l.restXz[0] - l.mountLocal.x;
        double oy = -ride - l.mountLocal.y;
        double oz = l.restXz[1] - l.mountLocal.z;
        double dx = dirL.x;
        double dy = dirL.y;
        double dz = dirL.z;
        double along = ox * dx + oy * dy + oz * dz;
        double off2 = ox * ox + oy * oy + oz * oz;
        double rlim = 0.94 * l.reach;
        double room = Math.sqrt(Math.max(0.0, rlim * rlim - off2 + along * along)) - along;
        if (l.ik.mode == GaitCore.PLANE_RADIAL && l.ik.coxaMax < 3.0) {
            double rx = l.ik.restDirLocal.x;
            double rz = l.ik.restDirLocal.z;
            double lim = Math.max(0.2, l.ik.coxaMax - 0.10);
            double cl = Math.cos(lim);
            double sl = Math.sin(lim);
            for (int si = 0; si < 2; si++) {
                double sgn = si == 0 ? -1.0 : 1.0;
                double ux = rx * cl + rz * (sgn * sl);
                double uz = -rx * (sgn * sl) + rz * cl;
                double det = dx * uz - dz * ux;
                if (Math.abs(det) > 1e-5) {
                    double s = (ux * oz - uz * ox) / det;
                    double t = (dx * oz - dz * ox) / det;
                    if (s > 0.0 && t > 0.0 && s < room) {
                        room = s;
                    }
                }
            }
        }
        if (Math.abs(dz) > 0.1) {
            double adz = Math.abs(dz);
            for (int i = 0; i < m.n; i++) {
                Leg nb = m.legs[i];
                if (nb.side == l.side && nb != l) {
                    double gap = nb.restXz[1] - l.restXz[1];
                    if (gap * dz > 0.05) {
                        double safeDz = Math.max(0.0, 0.5 * Math.abs(gap) - 0.35);
                        double rC = safeDz / adz;
                        if (rC < room) {
                            room = rC;
                        }
                    }
                }
            }
        }
        return Math.max(0.0, room);
    }

    private static Vec3 predictFoot(Mech m, Leg l, double tAhead, double tLand, PredCtx ctx) {
        Vec3.Frame fr;
        double ride;
        boolean dynamic;
        Vec3 travel, lead, direction;
        boolean hasDir;
        double lcap;

        if (ctx != null) {
            fr = ctx.fr;
            ride = ctx.ride;
            dynamic = ctx.dynamic;
            travel = ctx.travel;
            lead = ctx.lead;
            hasDir = ctx.hasDir;
            direction = ctx.direction;
            lcap = ctx.leadCap;
        } else {
            fr = m.supportFrame();
            ride = m.rideEff();
            dynamic = tLand > 0.0 && (gallopingFast(m, false) || ((m.drifting || m.driftAmt > 0.05) && m.speed > 4.0));
            lcap = leadCap(m);
            hasDir = false;
            direction = Vec3.ZERO;
            if (dynamic) {
                travel = planarTravel(m, tLand);
                lead = planarTravel(m, tAhead).sub(travel);
                if (lead.lenSq() > 1e-8) {
                    Vec3 ln = lead.norm(Vec3.ZERO);
                    Vec3 leadDir = (Math.abs(m.yawRate * tLand) > 1e-3) ? ln.rotateAxis(fr.up, m.yawRate * tLand) : ln;
                    hasDir = true;
                    direction = fr.toLocal(leadDir);
                }
            } else {
                travel = Vec3.ZERO;
                lead = m.vt.mul(tAhead);
            }
        }

        double widen = 1.0;
        if (l.ik.mode == GaitCore.PLANE_RADIAL && Math.abs(m.yawRate) > 0.5 && m.speed > 6.0) {
            widen = 1.0 + GaitCore.clamp(Math.abs(m.yawRate) * m.speed / 280.0, 0.0, 0.22);
        }
        Vec3 rl = fr.toWorld(l.restLocal(ride, widen));
        if (m.state == GaitCore.STATE_MANTLE && m.mantle.active) {
            if (l.force || (l.isFront && m.mantle.t < 0.5)) {
                return m.mantle.top.add(m.mantle.fwd.mul(0.4)).add(fr.right.mul(l.restXz[0]));
            }
            Vec3 base = m.mantlePos(m.mantle.t + tLand / m.mantle.dur);
            return base.add(rl);
        }
        if (m.hasBalanceOut && rl.dot(m.balanceOut) > 0.0) {
            rl = rl.add(m.balanceOut.mul(Math.min(1.5 * m.balanceExcess + 0.2, 0.4 * l.reach)));
        }

        double cap;
        if (dynamic) {
            if (hasDir) {
                double room = legFwdRoom(m, l, direction, ride);
                double reserve = Math.min(0.45 * GaitCore.LANDING_RAMP_REACH * l.reach,
                        0.25 * m.speed * m.cycle * (1.0 - m.duty));
                cap = Math.min(0.75 * lcap, Math.min(0.28 * l.reach, Math.max(0.0, room - reserve)));
            } else {
                cap = 0.0;
            }
        } else {
            cap = lcap;
        }
        if (m.hasBalanceOut) {
            lead = lead.sub(m.balanceOut.mul(Math.max(0.0, lead.dot(m.balanceOut))));
        }
        if (lead.lenSq() > cap * cap) {
            lead = lead.norm(Vec3.ZERO).mul(cap);
        }
        if (Math.abs(m.yawRate) > 1e-3) {
            if (dynamic) {
                double stanceRot = GaitCore.clamp(m.yawRate * (tAhead - tLand), -0.6, 0.6);
                rl = rl.rotateAxis(fr.up, -m.yawRate * tLand - stanceRot);
            } else {
                double yawT = (tLand > 0.0) ? (tLand + 0.5 * Math.max(0.0, tAhead - tLand)) : (tAhead * 0.5);
                rl = rl.rotateAxis(fr.up, -GaitCore.clamp(m.yawRate * yawT, -0.75, 0.75));
            }
        }
        return m.pos.add(travel).add(rl).add(lead);
    }

    private static Vec3 swingBow(Mech m, Leg l, Vec3 start, Vec3 target, double dur) {
        if (dur < 0.14) {
            return Vec3.ZERO;
        }
        Vec3 upP = m.probeUp();
        Vec3 bow = Vec3.ZERO;
        double turnArc = GaitCore.smoothstep((Math.abs(m.yawRate) * dur - 0.12) / 0.25);
        if (m.d.plane == GaitCore.PLANE_RADIAL && turnArc < 0.999) {
            Foothold.PoseOut po = m.tmpPoseOut;
            Foothold.landingPose(m, l, 0.5 * dur, po);
            Vec3 mid = start.add(target).mul(0.5);
            Vec3 toMid = mid.sub(po.mount).projectPlane(upP);
            double restR = Math.hypot(l.restXz[0] - l.mountLocal.x, l.restXz[1] - l.mountLocal.z);
            Vec3 out = po.frame.toWorld(l.ik.restDirLocal).projectPlane(upP).norm(Vec3.ZERO);
            double rMid = toMid.len();
            double outMid = toMid.dot(out);
            if (rMid < 0.70 * restR || outMid < 0.20 * restR) {
                double clearance = Math.min(rMid, outMid);
                bow = bow.add(out.mul((0.75 * restR - clearance) * (1.0 - turnArc)));
            }
        }
        if (m.flightT > 0.0 && m.gait.leap >= 0.4 && m.gait.stridePitch > 0.2) {
            double scale = Math.min(1.0, m.gait.leap * m.gait.stridePitch);
            double zmax = 0.0;
            for (int i = 0; i < m.n; i++) {
                double az = Math.abs(m.legs[i].mountLocal.z);
                if (az > zmax) {
                    zmax = az;
                }
            }
            if (zmax == 0.0) {
                zmax = 1.0;
            }
            double dirz = GaitCore.clamp(l.mountLocal.z / zmax, -1.0, 1.0);
            Vec3 extra = m.headingFwd().mul(dirz * 0.14 * l.reach).add(upP.mul(0.12 * m.ride));
            bow = bow.add(extra.mul(scale));
        }
        double maxBow = Math.min(0.90 * l.reach, 2.5 * dur * l.reach);
        double bl = bow.len();
        if (bl > maxBow) {
            bow = bow.mul(maxBow / bl);
        }
        return bow;
    }

    private static Vec3 evalSwingArc(Leg l, Vec3 sh, Vec3 th, double sLoc) {
        if (l.ik.mode == GaitCore.PLANE_RADIAL) {
            Vec3 rd = l.ik.restDirLocal;
            double px = -rd.z;
            double pz = rd.x;
            double sOut = sh.x * rd.x + sh.z * rd.z;
            double sSide = sh.x * px + sh.z * pz;
            double tOut = th.x * rd.x + th.z * rd.z;
            double tSide = th.x * px + th.z * pz;
            double r0 = Math.hypot(sOut, sSide);
            double r1 = Math.hypot(tOut, tSide);
            double a0 = Math.atan2(sSide, sOut);
            double a1 = Math.atan2(tSide, tOut);
            double rad = r0 + (r1 - r0) * sLoc;
            double ang = a0 + (a1 - a0) * sLoc;
            double ca = Math.cos(ang);
            double sa = Math.sin(ang);
            return new Vec3((rd.x * ca + px * sa) * rad, sh.y + (th.y - sh.y) * sLoc, (rd.z * ca + pz * sa) * rad);
        }
        return sh.lerp(th, sLoc);
    }

    /** Records or rebases hip-local polar/linear swing arc during turns so swinging feet avoid chord cutting. */
    static void setSwingArc(Mech m, Leg l, Vec3 target, double rem, boolean resetStart) {
        Leg.Swing sw = l.swing;
        if (!sw.active) {
            return;
        }
        double turnAng = Math.abs(m.yawRate) * Math.max(rem, sw.dur);
        double w = GaitCore.smoothstep((turnAng - 0.12) / 0.25);
        sw.arcW = resetStart ? w : Math.max(sw.arcW, w);
        if (sw.arcW > 1e-4) {
            Foothold.PoseOut po = m.tmpPoseOut;
            Foothold.landingPose(m, l, Math.max(rem, 0.0), po);
            Vec3 newTh = po.frame.toLocal(target.sub(po.mount));
            if (resetStart || !sw.hasStartHipL || !sw.hasTargetHipL) {
                sw.startHipL = m.frame.toLocal(l.foot.sub(l.mountWorld));
                sw.hasStartHipL = true;
                sw.arcT0 = sw.t;
                sw.targetHipL = newTh;
                sw.hasTargetHipL = true;
            } else {
                double span = Math.max(1e-4, 1.0 - sw.arcT0);
                double sCur = GaitCore.smoothstep(GaitCore.clamp((sw.t - sw.arcT0) / span, 0.0, 1.0));
                if (sCur <= 1e-4) {
                    sw.targetHipL = newTh;
                } else if (sCur < 0.88) {
                    double inv = 1.0 / (1.0 - sCur);
                    Vec3 sh = sw.startHipL;
                    Vec3 oldTh = sw.targetHipL;
                    if (l.ik.mode == GaitCore.PLANE_RADIAL) {
                        Vec3 rd = l.ik.restDirLocal;
                        double px = -rd.z;
                        double pz = rd.x;
                        Vec3 cur = evalSwingArc(l, sh, oldTh, sCur);
                        double cOut = cur.x * rd.x + cur.z * rd.z;
                        double cSide = cur.x * px + cur.z * pz;
                        double nOut = newTh.x * rd.x + newTh.z * rd.z;
                        double nSide = newTh.x * px + newTh.z * pz;
                        double rCur = Math.hypot(cOut, cSide);
                        double r1 = Math.hypot(nOut, nSide);
                        double aCur = Math.atan2(cSide, cOut);
                        double a1 = Math.atan2(nSide, nOut);
                        double r0 = GaitCore.clamp((rCur - sCur * r1) * inv, 0.25 * l.reach, 1.15 * l.reach);
                        double a0 = GaitCore.clamp((aCur - sCur * a1) * inv, -2.8, 2.8);
                        double y0 = (cur.y - sCur * newTh.y) * inv;
                        double ca = Math.cos(a0);
                        double sa = Math.sin(a0);
                        sw.startHipL = new Vec3((rd.x * ca + px * sa) * r0, y0, (rd.z * ca + pz * sa) * r0);
                    } else {
                        Vec3 cur = sh.lerp(oldTh, sCur);
                        sw.startHipL = cur.sub(newTh.mul(sCur)).mul(inv);
                    }
                    sw.targetHipL = newTh;
                }
            }
        } else if (resetStart) {
            sw.hasStartHipL = false;
            sw.hasTargetHipL = false;
            sw.arcT0 = 0.0;
        }
    }

    static void lift(
            Mech m, Leg l, double dur, boolean hasFh, Vec3 fhP, Vec3 fhN,
            Vec3 optStartVel, Vec3 optEndVel) {
        Vec3 upP = m.probeUp();
        Vec3 target, normal;
        boolean hasGround;
        if (hasFh) {
            target = fhP;
            normal = fhN;
            hasGround = true;
        } else {
            target = l.idealPred.add(upP.mul(m.ride * 0.2));
            normal = upP;
            hasGround = false;
        }
        double lift = m.d.stepHeight * m.gait.lift * (m.crouch > 0.5 ? 0.65 : 1.0);
        if (m.state == GaitCore.STATE_MANTLE) {
            lift *= 1.6;
        }
        Vec3 sv = optStartVel != null ? optStartVel : Vec3.ZERO;
        Vec3 ev = optEndVel != null ? optEndVel : Vec3.ZERO;
        if (m.drifting) {
            if (optStartVel == null) {
                sv = m.vt.projectPlane(l.footNormal).mul(0.95);
            }
            if (optEndVel == null) {
                ev = m.vt.projectPlane(normal).mul(0.95);
            }
        }
        Vec3 bow = swingBow(m, l, l.foot, target, dur);
        l.beginSwing(target, normal, dur, lift, upP, hasGround, m.wq, Clearance.clearance(m),
                sv, ev, bow, m.hasCeilY, m.ceilY);
        setSwingArc(m, l, target, dur, true);
        l.swing.hasIdeal = true;
        l.swing.ideal = l.idealPred;
        l.cooldown = 0.05;
        l.force = false;
        l.pending = false;
    }

    private static double planStep(Mech m, Leg l, double durIn, boolean[] outHasFh, Foothold.Result outFh) {
        double dur = durIn;
        if (m.gait.phaseLock && m.state == GaitCore.STATE_GROUND && m.cycle > 0.0 && !m.braking) {
            double p = GaitCore.mod1(m.phase + l.offset);
            double toStance = GaitCore.mod1((1.0 - m.duty) - p);
            dur = GaitCore.clamp(toStance * m.cycle, 0.6 * dur, 1.4 * dur);
        }
        dur = Math.max(0.05, dur);
        if (m.tAhead > 0.0) {
            l.idealPred = predictFoot(m, l, dur + 0.5 * m.duty * m.cycle, dur, null);
        }
        outHasFh[0] = Foothold.findFoothold(m, l, l.idealPred, dur, outFh);
        return dur;
    }

    static boolean canLift(Mech m, Leg l, boolean emergency, boolean checkStatic) {
        Gait g = m.gait;
        if (l.stanceT == 0.0) {
            return false;
        }
        Leg[] others = m.tmpLegs1;
        int nAfter = 0;
        for (int i = 0; i < m.n; i++) {
            Leg x = m.legs[i];
            if (x.planted && x != l && x.stretch <= 1.04) {
                others[nAfter++] = x;
            }
        }
        if (m.state == GaitCore.STATE_GROUND && !m.biped) {
            double curStr = l.mountWorld.dist(l.foot) / l.reach;
            boolean over = curStr > 0.99
                    || ((curStr > 0.86 || l.ik.coxaRatio > 0.72 || l.nearLimit || m.speed > 18.0)
                    && !l.ik.feasible(l.mountWorld, m.frame, l.foot, 0.0));
            if (!over && g.pairSync && !m.trailingSupport) {
                boolean anyDiffPair = false;
                for (int i = 0; i < nAfter; i++) {
                    if (others[i].pair != l.pair) {
                        anyDiffPair = true;
                        break;
                    }
                }
                if (!anyDiffPair) {
                    return false;
                }
            }
            if (nAfter == 0 && !over && !mayFlight(m)) {
                for (int i = 0; i < m.n; i++) {
                    if (m.legs[i].swing.active && m.legs[i].swing.hasGround) {
                        return false;
                    }
                }
            }
        }
        if (emergency && (l.stretch > 1.04 || l.limitT > 0.1)) {
            return true;
        }
        if (m.biped) {
            int need = (g.minSupport == 0 && m.speed > 1.0) ? 0 : 1;
            if (emergency && m.speed > 1.0) {
                need = 0;
            }
            return nAfter >= need;
        }
        int need = g.minSupport;
        if (emergency) {
            need = Math.min(need, Math.max(m.n / 2, 1));
        }
        if (m.state == GaitCore.STATE_MANTLE) {
            int capM = Math.max(m.n / 2 - 1, 1);
            if (capM < need) {
                need = capM;
            }
        }
        if (m.onWall) {
            int nw = Math.max(m.n / 2, 3);
            if (nw > need) {
                need = nw;
            }
        }
        if (m.state == GaitCore.STATE_GROUND && !mayFlight(m) && !emergency) {
            if (1 > need) {
                need = 1;
            }
        }
        if (nAfter < Math.min(need, m.n - 1)) {
            return false;
        }
        if (!emergency && g.adjacentLock) {
            int side = l.idx % 2;
            for (int pi = 0; pi < 3; pi++) {
                int pair = pi == 0 ? (l.pair - 1) : (pi == 1 ? (l.pair + 1) : l.pair);
                int sd = pi == 2 ? (1 - side) : side;
                int j = pair * 2 + sd;
                if (0 <= pair && 0 <= j && j < m.n) {
                    if (!m.legs[j].planted) {
                        return false;
                    }
                }
            }
        }
        if (checkStatic && g.staticStable && !emergency && m.state == GaitCore.STATE_GROUND && !m.onWall) {
            Vec3.Vec2[] pts2 = m.tmpVec2A;
            Vec3.Vec2[] hull = m.tmpVec2B;
            for (int i = 0; i < nAfter; i++) {
                pts2[i] = new Vec3.Vec2(others[i].foot.x, others[i].foot.z);
            }
            int hn = Vec3.convexHull2d(pts2, nAfter, hull);
            Vec3.Vec2 com = new Vec3.Vec2(m.pos.x, m.pos.z);
            if (Vec3.supportMargin(hull, hn, com) < -0.04 * m.size) {
                return false;
            }
        }
        return true;
    }

    static void onPlant(Mech m, Leg l) {
        Clearance.solveLeg(m, l, 1.0 / 60.0);
        if (!l.planted) {
            return;
        }
        l.pending = false;
        l.stanceT = 0.0;
        if (gallopingFast(m, false)) {
            l.cooldown = Math.min(0.32 * m.duty * m.cycle, 0.38 * l.reach / m.speed);
        } else {
            l.cooldown = Math.max(l.cooldown, 0.16 * Math.sqrt(m.size));
        }
        Vec3 up = m.probeUp();
        Vec3.Frame sf = m.supportFrame();
        l.hasLandLead = true;
        l.landLead = sf.toLocal(l.foot.sub(l.idealNow).projectPlane(up));
        l.hasLastPlant = true;
        l.lastPlantP = l.foot;
        l.lastPlantN = l.footNormal;
        MechDef d = m.d;
        double strength = d.mass * (0.5 + m.speed * 0.12);
        m.emit(GaitCore.EV_STEP, l.foot, l.footNormal, strength, Vec3.ZERO);
        if (d.mass >= 2.0 && m.state == GaitCore.STATE_GROUND) {
            m.vel = m.vel.sub(m.frame.up.mul(0.05 * d.mass));
            if (Math.abs(m.inp.aim) < 0.05) {
                Vec3 lever = l.foot.sub(m.pos).projectPlane(m.frame.up);
                double ll = lever.len();
                if (ll > 0.2) {
                    Vec3 axis = m.frame.up.cross(lever.mul(1.0 / ll));
                    double kick = Math.min(0.14, 0.022 * d.mass
                            * GaitCore.clamp(1.0 - m.speed / Math.max(1.0, d.speedWalk), 0.25, 1.0));
                    m.angVel = m.angVel.add(axis.mul(kick));
                }
            }
        }
    }

    static boolean tryTakeoff(Mech m, Vec3 up, Vec3 vtNew, double[] ioVUp, boolean pushed) {
        if (m.flightT > 0.0 || !mayFlight(m)) {
            return false;
        }
        boolean paired = m.gait.pairSync;
        boolean reverse = vtNew.dot(m.headingFwd()) < 0.0;
        Leg lead = reverse ? m.legs[m.n - 1] : m.legs[0];
        Leg tail = reverse ? m.legs[0] : m.legs[m.n - 1];
        Leg[] swings = m.tmpLegs2;
        int nSw = 0;
        for (int i = 0; i < m.n; i++) {
            if (m.legs[i].swing.active) {
                if (!m.legs[i].swing.hasGround) {
                    return false;
                }
                swings[nSw++] = m.legs[i];
            }
        }
        if (nSw == 0) {
            return false;
        }
        boolean recovery = false;
        if (paired && !pushed) {
            boolean found = false;
            for (int i = 0; i < nSw; i++) {
                if (swings[i].pair == tail.pair && swings[i].swing.t < 0.30) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                return false;
            }
            recovery = true;
        }
        double b = m.d.bound * m.gait.leap * (recovery ? 0.55 : 1.0);
        double vl = vLeg(m);
        double k = GaitCore.clamp((m.speed - 0.3 * vl) / (0.7 * vl), 0.0, 1.0);
        double h = b * k;
        double tLand = 1e18;
        Leg nxt = null;
        for (int i = 0; i < nSw; i++) {
            double remT = (1.0 - swings[i].swing.t) * swings[i].swing.dur;
            if (remT < tLand) {
                tLand = remT;
                nxt = swings[i];
            }
        }
        double tFl = Math.max(tLand, 2.0 * Math.sqrt(2.0 * h / GaitCore.G));
        double extra = tFl - tLand;
        for (int i = 0; i < nSw; i++) {
            Leg l = swings[i];
            double rem = 1.0 - l.swing.t;
            if (rem > 1e-3) {
                if (paired) {
                    double delay = Math.abs(lead.offset - l.offset) * m.cycle;
                    l.swing.dur = (tFl + delay) / rem;
                } else if (m.gait.groupCount > 0 && nxt != null) {
                    double d1 = GaitCore.mod1(l.offset - nxt.offset);
                    double d2 = GaitCore.mod1(nxt.offset - l.offset);
                    double dOff = Math.min(d1, d2);
                    if (dOff < 0.08) {
                        l.swing.dur = tFl / rem;
                    } else {
                        double minRem = tFl + 0.04;
                        if (rem * l.swing.dur < minRem) {
                            l.swing.dur = minRem / rem;
                        }
                    }
                } else {
                    l.swing.dur += extra / rem;
                }
                l.swing.retargetCd = 0.0;
                l.swing.forceRetarget = true;
            }
        }
        if (paired) {
            double toStance = GaitCore.mod1(1.0 - m.duty - lead.offset - m.phase);
            m.flightHold = Math.max(0.0, tFl - toStance * m.cycle);
        } else if (m.gait.groupCount > 0 && nxt != null) {
            double toStance = GaitCore.mod1(1.0 - m.duty - nxt.offset - m.phase);
            m.flightHold = Math.max(0.0, tFl - toStance * m.cycle);
        } else {
            m.flightHold = extra;
        }
        m.flightT = m.flightDur = tFl;
        m.flightY0 = m.pos.y;
        m.flightH = GaitCore.G * tFl * tFl / 8.0;
        m.flightCount += 1;
        Vec3 lp = m.pos.add(planarTravel(m, tFl));
        WorldQueryCtx.Hit lh = m.tmpHit;
        if (m.wq.raycast(new Vec3(lp.x, m.pos.y + 4.0, lp.z), Vec3.WORLD_UP.mul(-1.0), m.ride + 8.0, lh)
                && lh.normal.y > 0.64) {
            double dy = GaitCore.clamp((lh.point.y + m.rideEff()) - m.pos.y, -2.5, 2.5);
            ioVUp[0] = GaitCore.clamp(dy / tFl + 0.5 * GaitCore.G * tFl, 0.25 * GaitCore.G * tFl, 0.85 * GaitCore.G * tFl);
        } else {
            ioVUp[0] = 0.5 * GaitCore.G * tFl;
        }
        return true;
    }

    static boolean updateFlight(Mech m, double dt, int npl, Vec3 vtNew, Vec3 up, double[] ioVUp) {
        boolean tookOff = false;
        boolean reverse = vtNew.dot(m.headingFwd()) < 0.0;
        Leg tail = reverse ? m.legs[0] : m.legs[m.n - 1];
        boolean pushed = m.trailingSupport;
        boolean ts = false;
        for (int i = 0; i < m.n; i++) {
            if (m.legs[i].planted && m.legs[i].pair == tail.pair) {
                ts = true;
                break;
            }
        }
        m.trailingSupport = ts;
        if (m.flightT <= 0.0 && npl == 0 && m.prevNpl > 0) {
            tookOff = tryTakeoff(m, up, vtNew, ioVUp, pushed);
        }
        m.prevNpl = npl;
        if (m.flightT <= 0.0) {
            return false;
        }
        if (npl > 0 && !tookOff) {
            m.flightT = 0.0;
            m.absorb = Math.max(m.absorb, GaitCore.clamp(-ioVUp[0] / 36.0, 0.0, 0.16));
            return false;
        }
        m.flightT -= dt;
        ioVUp[0] -= GaitCore.G * dt;
        m.vel = vtNew.add(m.slideVel).add(up.mul(ioVUp[0]));
        m.unsupportedT = 0.0;
        return true;
    }

    private static double alignSwingGroup(Mech m, Leg[] sw, int nSw) {
        double[] rems = m.tmpRems;
        double tLand = -1e18;
        for (int i = 0; i < nSw; i++) {
            rems[i] = (1.0 - sw[i].swing.t) * sw[i].swing.dur;
            if (rems[i] > tLand) {
                tLand = rems[i];
            }
        }
        for (int i = 0; i < nSw; i++) {
            double extra = tLand - rems[i];
            double rem = 1.0 - sw[i].swing.t;
            if (0.01 < extra && extra <= 0.15 && rem > 1e-3) {
                sw[i].swing.dur = (rem * sw[i].swing.dur + extra) / rem;
                sw[i].swing.forceRetarget = true;
            }
        }
        return tLand;
    }

    private static boolean groupMayLift(Mech m, int[] grp, int gSz) {
        int others = 0;
        for (int i = 0; i < m.n; i++) {
            if (!m.legs[i].planted) {
                continue;
            }
            boolean inG = false;
            for (int gi = 0; gi < gSz; gi++) {
                if (grp[gi] == m.legs[i].idx) {
                    inG = true;
                    break;
                }
            }
            if (!inG) {
                others++;
            }
        }
        if (m.gait.pairSync && !m.trailingSupport && others == 0) {
            return false;
        }
        if (m.duty < 0.48 && mayFlight(m)) {
            return true;
        }
        int opp = m.n - gSz;
        return others >= Math.min(gSz, opp);
    }

    private static void presyncGroups(Mech m, Gait dstGait) {
        Leg[] sw = m.tmpLegs2;
        for (int gi = 0; gi < dstGait.groupCount; gi++) {
            int nSw = 0;
            for (int k = 0; k < dstGait.groupSizes[gi]; k++) {
                int j = dstGait.groups[gi][k];
                if (j < m.n && m.legs[j].swing.active) {
                    sw[nSw++] = m.legs[j];
                }
            }
            if (nSw >= 2) {
                alignSwingGroup(m, sw, nSw);
            }
        }
    }

    private static void syncGroups(Mech m, Gait gait, double swDur) {
        Leg[] gl = m.tmpLegs1;
        Leg[] sw = m.tmpLegs2;
        for (int gi = 0; gi < gait.groupCount; gi++) {
            int[] grp = gait.groups[gi];
            int gSz = gait.groupSizes[gi];
            int nGl = 0;
            int nSw = 0;
            for (int k = 0; k < gSz; k++) {
                int j = grp[k];
                if (j < m.n) {
                    gl[nGl++] = m.legs[j];
                    if (m.legs[j].swing.active) {
                        sw[nSw++] = m.legs[j];
                    }
                }
            }
            if (nSw == 0 || (nSw == nGl && nGl < 2)) {
                continue;
            }
            double tLand = alignSwingGroup(m, sw, nSw);
            if (nSw < nGl && tLand > Math.max(0.08, 0.5 * swDur) && groupMayLift(m, grp, gSz)) {
                for (int k = 0; k < nGl; k++) {
                    Leg x = gl[k];
                    if (x.planted && x.cooldown <= 0.0 && canLift(m, x, true, true)) {
                        double dur = planStep(m, x, tLand, m.tmpBool, m.tmpFhOut);
                        lift(m, x, dur, m.tmpBool[0], m.tmpFhOut.pt, m.tmpFhOut.n, null, null);
                    }
                }
            }
        }
    }

    private static void updateLegTargets(
            Mech m, double dt, Vec3.Frame fr, Vec3.Frame sf, Vec3 visPos,
            Vec3 upP, double ride, double swDur, boolean moving, boolean driven) {
        PredCtx ctx = null;
        if (moving && !(m.state == GaitCore.STATE_MANTLE && m.mantle.active)) {
            ctx = m.tmpPredCtx;
            ctx.fr.copyFrom(sf);
            ctx.ride = ride;
            ctx.dynamic = swDur > 0.0 && (gallopingFast(m, false) || ((m.drifting || m.driftAmt > 0.05) && m.speed > 4.0));
            ctx.leadCap = leadCap(m);
            ctx.hasDir = false;
            ctx.direction = Vec3.ZERO;
            if (ctx.dynamic) {
                ctx.travel = planarTravel(m, swDur);
                ctx.lead = planarTravel(m, m.tAhead).sub(ctx.travel);
                if (ctx.lead.lenSq() > 1e-8) {
                    Vec3 ln = ctx.lead.norm(Vec3.ZERO);
                    Vec3 leadDir = (Math.abs(m.yawRate * swDur) > 1e-3)
                            ? ln.rotateAxis(sf.up, m.yawRate * swDur)
                            : ln;
                    ctx.hasDir = true;
                    ctx.direction = sf.toLocal(leadDir);
                }
            } else {
                ctx.travel = Vec3.ZERO;
                ctx.lead = m.vt.mul(m.tAhead);
            }
        }

        double restScale = 1.0 + 0.2 * m.brace + 0.22 * m.driftAmt;
        boolean flatUp = upP.x == 0.0 && upP.y == 1.0 && upP.z == 0.0;
        boolean yawActive = Math.abs(m.yawRate) > 0.25;

        for (int i = 0; i < m.n; i++) {
            Leg l = m.legs[i];
            l.mountWorld = visPos.add(fr.toWorld(l.mountLocal));
            l.idealNow = m.pos.add(sf.toWorld(l.restLocal(ride, restScale)));
            l.idealPred = moving ? predictFoot(m, l, m.tAhead, swDur, ctx) : l.idealNow;
            if (flatUp) {
                l.displacement = Math.hypot(l.foot.x - l.idealNow.x, l.foot.z - l.idealNow.z);
            } else {
                l.displacement = l.foot.sub(l.idealNow).projectPlane(upP).len();
            }
            l.offPath = Vec3.distToSegment(l.foot, l.idealNow, l.idealPred, upP);
            if (l.hasLandLead) {
                double alt = Vec3.distToSegment(l.foot, l.idealNow, l.idealNow.add(sf.toWorld(l.landLead)), upP);
                if (alt < l.offPath) {
                    l.offPath = alt;
                }
            }
            l.cooldown -= dt;
            l.stanceT = l.planted ? (l.stanceT + dt) : 0.0;
            l.voidMark -= dt;
            l.sliding = false;
            if (driven || yawActive
                    || (l.hasSettledHullPos
                    && Math.hypot(m.pos.x - l.settledHullPos.x, m.pos.z - l.settledHullPos.z) > 0.25 * m.size)) {
                l.hasSettledAt = false;
            }
        }

        for (int i = 0; i < m.n; i++) {
            Leg l = m.legs[i];
            if (l.hasCarry && !l.planted && !l.swing.active) {
                l.foot = m.pos.add(fr.toWorld(l.carry));
            }
        }

        double sl = m.slideVel.len();
        if (sl > 0.05 && m.state == GaitCore.STATE_GROUND) {
            for (int i = 0; i < m.n; i++) {
                Leg l = m.legs[i];
                if (l.planted) {
                    Vec3 old = l.foot;
                    l.foot = l.foot.add(m.slideVel.projectPlane(l.footNormal).mul(dt));
                    l.sliding = true;
                    Clearance.slideFootClear(m, l, upP);
                    if (sl > 0.8) {
                        m.emit(GaitCore.EV_SKID, old, l.footNormal, m.d.widths[m.d.nSegs - 1] * 0.8, l.foot);
                    }
                }
            }
        }
    }

    private static int updateSwings(Mech m, double dt, Vec3 upP, double tStance, boolean moving) {
        int justPlanted = 0;
        for (int i = 0; i < m.n; i++) {
            Leg l = m.legs[i];
            if (!l.swing.active) {
                continue;
            }
            Leg.Swing sw = l.swing;
            double rem = Math.max(0.0, (1.0 - sw.t) * sw.dur - dt);
            if ((rem > 0.05 || sw.forceRetarget) && m.state != GaitCore.STATE_CROUCHJUMP) {
                sw.retargetCd -= dt;
                if (sw.retargetCd <= 0.0 || sw.forceRetarget) {
                    sw.retargetCd = m.braking ? 0.03 : ((gallopingFast(m, false) || m.drifting) ? 0.05 : 0.1);
                    Vec3 pred = moving ? predictFoot(m, l, rem + 0.5 * tStance, rem, null) : l.idealNow;
                    Vec3 ref = sw.hasIdeal ? sw.ideal : sw.target;
                    boolean forced = sw.forceRetarget;
                    sw.forceRetarget = false;
                    double evL2 = sw.endVelocity.lenSq();
                    if (pred.dist(ref) > 0.25 || forced
                            || (m.drifting && evL2 > 0.0)
                            || (!m.drifting && evL2 > 0.25)) {
                        boolean hasFh = Foothold.findFoothold(m, l, pred, rem, m.tmpFhOut);
                        sw.hasIdeal = true;
                        sw.ideal = pred;
                        if (hasFh) {
                            Vec3 fhp = m.tmpFhOut.pt;
                            Vec3 fhn = m.tmpFhOut.n;
                            Vec3 endVel = m.drifting ? m.vt.projectPlane(fhn).mul(0.95) : Vec3.ZERO;
                            if (forced || !sw.hasGround || fhp.dist(sw.target) > 0.15
                                    || endVel.dist(sw.endVelocity) > 0.8) {
                                l.retarget(fhp, fhn, true, m.wq, Clearance.clearance(m),
                                        endVel, swingBow(m, l, l.foot, fhp, rem), m.hasCeilY, m.ceilY);
                                setSwingArc(m, l, fhp, rem, false);
                            }
                        } else if (forced) {
                            Vec3 endVel = m.drifting ? m.vt.projectPlane(upP).mul(0.95) : Vec3.ZERO;
                            l.retarget(pred, upP, false, m.wq, 0.0,
                                    endVel, swingBow(m, l, l.foot, pred, rem), m.hasCeilY, m.ceilY);
                            setSwingArc(m, l, pred, rem, false);
                        }
                    }
                }
            }
            if (l.updateSwing(dt)) {
                onPlant(m, l);
                justPlanted |= (1 << l.idx);
            } else if (l.swing.active && l.swing.arcW > 1e-4 && l.swing.hasStartHipL && l.swing.hasTargetHipL) {
                double remS = (1.0 - sw.t) * sw.dur;
                double fadeEnd = Math.min(0.08, 0.16 * sw.dur);
                double wArc = sw.arcW * GaitCore.smoothstep(sw.t / 0.12)
                        * GaitCore.smoothstep(remS / Math.max(1e-4, fadeEnd));
                if (wArc > 1e-4) {
                    double span = Math.max(1e-4, 1.0 - sw.arcT0);
                    double sLoc = GaitCore.smoothstep(GaitCore.clamp((sw.t - sw.arcT0) / span, 0.0, 1.0));
                    Vec3 relL = evalSwingArc(l, sw.startHipL, sw.targetHipL, sLoc);
                    Vec3 pBody = l.mountWorld.add(m.frame.toWorld(relL));
                    l.foot = l.foot.add(pBody.sub(l.foot).projectPlane(upP).mul(wArc));
                }
            }
        }
        return justPlanted;
    }

    private static void updateSearching(Mech m, double dt, Vec3 upP, double swDur, double ride) {
        for (int i = 0; i < m.n; i++) {
            Leg l = m.legs[i];
            if (l.searching) {
                l.searchCd -= dt;
                if (l.searchCd <= 0.0) {
                    l.searchCd = 0.12;
                    double dur = planStep(m, l, swDur * 0.7, m.tmpBool, m.tmpFhOut);
                    boolean hasFh = m.tmpBool[0];
                    if (!hasFh && m.state == GaitCore.STATE_GROUND) {
                        Vec3 sumP = Vec3.ZERO;
                        int nPl = 0;
                        for (int k = 0; k < m.n; k++) {
                            if (m.legs[k].planted) {
                                sumP = sumP.add(m.legs[k].foot);
                                nPl++;
                            }
                        }
                        if (nPl > 0) {
                            Vec3 safe = sumP.mul(1.0 / nPl);
                            for (int ai = 0; ai < 2; ai++) {
                                double alpha = ai == 0 ? 0.60 : 0.90;
                                if (Foothold.findFoothold(m, l, l.idealNow.lerp(safe, alpha), dur, m.tmpFhOut)) {
                                    hasFh = true;
                                    l.voidMark = 1.0;
                                    break;
                                }
                            }
                        }
                    }
                    if (!hasFh && l.hasLastPlant && m.state == GaitCore.STATE_GROUND) {
                        Vec3 p = l.lastPlantP;
                        Vec3 n = l.lastPlantN;
                        if (p.dist(l.mountWorld) < 0.93 * l.reach
                                && Foothold.reachable(m, l, p, null, null)
                                && Foothold.clearSpot(m, l, p, upP, null)) {
                            hasFh = true;
                            m.tmpFhOut.pt = p;
                            m.tmpFhOut.n = n;
                            l.voidMark = 1.0;
                        }
                    }
                    if (hasFh) {
                        lift(m, l, dur, true, m.tmpFhOut.pt, m.tmpFhOut.n, null, null);
                        continue;
                    }
                }
                Vec3 target = Foothold.hangPose(m, l, ride);
                Vec3.Frame fr = m.frame;
                Vec3 fl = fr.toLocal(l.foot.sub(m.pos));
                Vec3 tl = fr.toLocal(target.sub(m.pos));
                l.foot = m.pos.add(fr.toWorld(fl.lerp(tl, GaitCore.damp(5.0, dt))));
            }
        }
    }

    private static void stepDriftLegs(Mech m, double dt, double swDur, Vec3 upP, MechDef d) {
        boolean sparkNow = false;
        m.sparkT += dt;
        if (m.sparkT >= 1.0 / 36.0) {
            m.sparkT = m.sparkT % (1.0 / 36.0);
            sparkNow = true;
        }
        for (int i = 0; i < m.n; i++) {
            Leg l = m.legs[i];
            if (l.planted) {
                Vec3 slide = m.vt.projectPlane(l.footNormal).mul(0.95);
                Vec3 old = l.foot;
                l.foot = l.foot.add(slide.mul(dt));
                l.sliding = true;
                Clearance.slideFootClear(m, l, upP);
                if (sparkNow) {
                    m.emit(GaitCore.EV_SPARK, l.foot, l.footNormal, m.speed, slide);
                }
                m.emit(GaitCore.EV_SKID, old, l.footNormal, d.widths[d.nSegs - 1], l.foot);
            }
        }
        int[] order = m.tmpOrder;
        for (int i = 0; i < m.n; i++) {
            order[i] = i;
        }
        for (int i = 1; i < m.n; i++) {
            int key = order[i];
            int j = i - 1;
            while (j >= 0 && m.legs[order[j]].stretch < m.legs[key].stretch) {
                order[j + 1] = order[j];
                j--;
            }
            order[j + 1] = key;
        }
        for (int oi = 0; oi < m.n; oi++) {
            Leg l = m.legs[order[oi]];
            if (l.planted
                    && (l.stretch > 0.96 || l.ik.err > 0.02
                    || (l.cooldown <= 0.0 && (l.stretch > 0.90 || l.nearLimit)))
                    && canLift(m, l, true, true)) {
                double dur = planStep(m, l, swDur * 0.8, m.tmpBool, m.tmpFhOut);
                lift(m, l, dur, m.tmpBool[0], m.tmpFhOut.pt, m.tmpFhOut.n, null, null);
            }
        }
    }

    private static void stepReactiveLegs(
            Mech m, double dt, Vec3.Frame fr, Vec3 upP, Gait gait,
            double swingFrac, double swDur, double effSpd, boolean moving, boolean driven, MechDef d) {
        double tStance = m.cycle * m.duty;
        for (int i = 0; i < m.n; i++) {
            Leg l = m.legs[i];
            double p = GaitCore.mod1(m.phase + l.offset);
            boolean inW = p < swingFrac;
            if (inW && !l.prevWindow) {
                l.pending = true;
            }
            if (!inW) {
                l.pending = false;
            }
            l.prevWindow = inW;
        }

        double zone = d.zone * (m.state == GaitCore.STATE_MANTLE ? 0.5 : 1.0);
        if (m.state == GaitCore.STATE_GROUND && !m.drifting && Math.abs(m.yawRate) < 0.35
                && effSpd < 0.35 * m.vWalk && driven) {
            double postureS = GaitCore.smoothstep((effSpd - 0.15) / Math.max(0.5, 0.35 * m.vWalk - 0.15));
            zone *= 0.55 + 0.45 * postureS;
        }
        boolean hasVdir = m.speed > 0.1;
        Vec3 vdir = hasVdir ? m.vt.norm(Vec3.ZERO) : Vec3.ZERO;
        boolean hasTSince = !m.drifting && m.driftAmt > 0.05 && m.slideCd > 0.0 && m.speed > 4.0;
        double tSince = hasTSince ? Math.max(0.0, 0.35 - m.slideCd) : 0.0;
        double tExitLook = Math.max(0.036, swDur * 0.85);
        boolean hasStale = false;
        if (hasTSince) {
            for (int i = 0; i < m.n; i++) {
                if (m.legs[i].planted && m.legs[i].stanceT + 0.02 >= tSince) {
                    hasStale = true;
                    break;
                }
            }
        }
        double walkMinSt = (gait.minSupport > 0) ? 0.155 : 0.0;

        int[] order = m.tmpOrder;
        double[] keyVal = m.tmpKeyVal;
        int[] keyPri = m.tmpKeyPri;
        for (int i = 0; i < m.n; i++) {
            order[i] = i;
            Leg x = m.legs[i];
            if (hasStale) {
                keyPri[i] = (x.nearLimit || x.stretch > 0.94 || x.ik.err > 0.01) ? 0 : 1;
                keyVal[i] = -x.mountWorld.add(m.vt.mul(tExitLook)).dist(x.foot) / x.reach;
            } else {
                keyPri[i] = 0;
                keyVal[i] = -x.displacement;
            }
        }
        for (int i = 1; i < m.n; i++) {
            int oi = order[i];
            int j = i - 1;
            while (j >= 0) {
                int oj = order[j];
                boolean gt = (keyPri[oj] > keyPri[oi]) || (keyPri[oj] == keyPri[oi] && keyVal[oj] > keyVal[oi]);
                if (!gt) {
                    break;
                }
                order[j + 1] = oj;
                j--;
            }
            order[j + 1] = oi;
        }

        Foothold.PoseOut po = m.tmpPoseOut;
        for (int oi = 0; oi < m.n; oi++) {
            Leg l = m.legs[order[oi]];
            if (!l.planted) {
                continue;
            }
            double tLook = (hasTSince && l.stanceT + 0.02 >= tSince) ? tExitLook : 0.036;
            double disp = l.offPath;
            int lGrpIdx = -1;
            for (int gi = 0; gi < gait.groupCount && lGrpIdx < 0; gi++) {
                for (int k = 0; k < gait.groupSizes[gi]; k++) {
                    if (gait.groups[gi][k] == l.idx) {
                        lGrpIdx = gi;
                        break;
                    }
                }
            }
            boolean grpBlocked = lGrpIdx >= 0 && !groupMayLift(m, gait.groups[lGrpIdx], gait.groupSizes[lGrpIdx]);
            double curStr = l.mountWorld.dist(l.foot) / l.reach;
            boolean curInfeas = curStr > 0.99
                    || ((curStr > 0.86 || l.ik.coxaRatio > 0.72 || l.nearLimit || m.speed > 18.0)
                    && !l.ik.feasible(l.mountWorld, fr, l.foot, 0.0));
            double release = 0.97;
            if (curInfeas) {
                release = Math.min(release, l.stretch - 1e-3);
            }
            if (hasVdir && l.foot.sub(l.mountWorld).dot(vdir) < 0.0) {
                release -= Math.min(0.06, 0.03 * m.speed / l.reach);
                double ahead = l.mountWorld.add(m.vt.mul(tLook)).dist(l.foot) / l.reach;
                if (ahead > 0.97 && (!grpBlocked || l.stretch > 0.94)) {
                    release = Math.min(release, l.stretch - 1e-3);
                }
            }
            if (moving && (m.speed > 4.0 || Math.abs(m.yawRate) > 0.5)) {
                Vec3 hipNext;
                Vec3.Frame frameNext;
                if (Math.abs(m.yawRate) > 0.5) {
                    Foothold.landingPose(m, l, tLook, po);
                    hipNext = po.mount;
                    frameNext = po.frame;
                } else {
                    hipNext = l.mountWorld.add(m.vt.mul(tLook));
                    frameNext = fr;
                }
                boolean allowPred = curInfeas || !grpBlocked || l.stretch > 0.95 || l.ik.coxaRatio > 0.94 || l.ik.err > 0.01;
                if (allowPred
                        && (l.stanceT >= Math.max(walkMinSt, Math.min(0.08, 0.30 * tStance))
                        || l.stretch > 0.94 || l.ik.err > 0.01 || curInfeas)
                        && !l.ik.feasible(hipNext, frameNext, l.foot, 2.0)) {
                    release = Math.min(release, l.stretch - 1e-3);
                } else if (allowPred && l.ik.mode == GaitCore.PLANE_RADIAL && m.speed > 10.0
                        && Math.abs(m.yawRate) > 0.4 && l.stanceT >= 0.03) {
                    Vec3 relNext = frameNext.toLocal(l.foot.sub(hipNext));
                    double restR = Math.hypot(l.restXz[0] - l.mountLocal.x, l.restXz[1] - l.mountLocal.z);
                    double rXz = Math.hypot(relNext.x, relNext.z);
                    if (rXz < 0.48 * restR) {
                        release = Math.min(release, l.stretch - 1e-3);
                    } else if (rXz > 1e-5) {
                        Vec3 rd = l.ik.restDirLocal;
                        double invR = 1.0 / rXz;
                        double ux = relNext.x * invR;
                        double uz = relNext.z * invR;
                        double nextYaw = Math.atan2(rd.z * ux - rd.x * uz, rd.x * ux + rd.z * uz);
                        double curYaw = l.ik.yawDeg * GaitCore.DEG2RAD;
                        double diffY = Math.abs(GaitCore.wrapPi(nextYaw - curYaw));
                        if (diffY > 28.0 * GaitCore.DEG2RAD) {
                            release = Math.min(release, l.stretch - 1e-3);
                        }
                    }
                }
            }
            if (moving && !grpBlocked && Math.abs(m.yawRate) > 0.25 && l.stanceT >= Math.max(0.06, walkMinSt)) {
                double sideExc = Math.abs(fr.toLocal(l.foot.sub(m.pos)).x - l.restXz[0]);
                if (sideExc > 0.46 * l.reach) {
                    release = Math.min(release, l.stretch - 1e-3);
                }
            }
            double soon = (l.stanceT >= Math.max(0.06, walkMinSt) && l.stretch > 0.88)
                    ? (l.stretch + Math.max(0.0, l.stretchRate) * Math.min(0.05, swDur * 0.35))
                    : 0.0;
            if (l.cooldown > 0.0 && l.stretch <= release) {
                continue;
            }

            boolean hardEmg, zoneTrig;
            if (moving) {
                boolean nearReq = grpBlocked ? (l.stretch > 0.94 || l.ik.err > 0.01)
                        : (l.stretch > 0.90 || l.stretchRate > 0.1 || l.ik.err > 0.01
                        || (l.ik.coxaRatio > 0.88 && Math.abs(m.yawRate) > 0.25));
                boolean nearTrig = l.nearLimit && l.cooldown <= 0.0
                        && (l.stanceT >= Math.max(0.06, walkMinSt) || l.stretch > 0.94 || l.ik.err > 0.01)
                        && (driven || m.speed > 2.0) && nearReq;
                hardEmg = l.force || l.stretch > release || soon > release || nearTrig;
                zoneTrig = !grpBlocked && l.stanceT >= walkMinSt && disp > zone * gait.reach;
            } else {
                hardEmg = l.force || l.stretch > 0.995;
                zoneTrig = false;
            }
            boolean can = hardEmg ? canLift(m, l, true, true) : (zoneTrig && canLift(m, l, false, false));
            if (can) {
                double dur = planStep(m, l, swDur * 0.85, m.tmpBool, m.tmpFhOut);
                boolean hasFh = m.tmpBool[0];
                Vec3 fhp = m.tmpFhOut.pt;
                Vec3 fhn = m.tmpFhOut.n;
                if (!moving && !l.force && hasFh) {
                    double curD = l.foot.dist(l.mountWorld);
                    double newD = fhp.dist(l.mountWorld);
                    double newDisp = fhp.sub(l.idealNow).projectPlane(upP).len();
                    if (newD >= curD - 0.05 && newDisp >= l.displacement - 0.05) {
                        l.cooldown = Math.max(l.cooldown, 0.3);
                        continue;
                    }
                }
                if (Foothold.futileStep(l, hasFh, fhp)) {
                    continue;
                }
                lift(m, l, dur, hasFh, fhp, fhn, null, null);
                for (int gi = 0; gi < gait.groupCount; gi++) {
                    int[] grp = gait.groups[gi];
                    int gSz = gait.groupSizes[gi];
                    boolean inGrp = false;
                    for (int k = 0; k < gSz; k++) {
                        if (grp[k] == l.idx) {
                            inGrp = true;
                            break;
                        }
                    }
                    if (inGrp && groupMayLift(m, grp, gSz)) {
                        for (int k = 0; k < gSz; k++) {
                            int j = grp[k];
                            if (j < m.n) {
                                Leg mate = m.legs[j];
                                if (mate != l && mate.planted && canLift(m, mate, true, true)) {
                                    double mdur = planStep(m, mate, swDur * 0.85, m.tmpBool, m.tmpFhOut);
                                    lift(m, mate, mdur, m.tmpBool[0], m.tmpFhOut.pt, m.tmpFhOut.n, null, null);
                                }
                            }
                        }
                    }
                }
            }
        }

        if (gallopingFast(m, true)) {
            for (int i = 0; i < m.n; i++) {
                Leg l = m.legs[i];
                if (l.planted && l.stretch > 1.0) {
                    Vec3 old = l.foot;
                    l.foot = l.foot.add(m.vt.projectPlane(l.footNormal).mul(dt));
                    l.sliding = true;
                    Clearance.slideFootClear(m, l, upP);
                    m.emit(GaitCore.EV_SKID, old, l.footNormal, d.widths[d.nSegs - 1] * 0.6, l.foot);
                }
            }
        }
    }

    private static void stepRhythmicGroups(Mech m, Gait gait, double swDur, double s) {
        Leg[] gl = m.tmpLegs1;
        double[] durs = m.tmpRems;
        boolean[] fhs = m.tmpGroupFh;
        Vec3[] fhps = m.tmpGroupPts;
        Vec3[] fhns = m.tmpGroupNorms;
        for (int gi = 0; gi < gait.groupCount; gi++) {
            int[] grp = gait.groups[gi];
            int gSz = gait.groupSizes[gi];
            int nGl = 0;
            for (int k = 0; k < gSz; k++) {
                if (grp[k] < m.n) {
                    gl[nGl++] = m.legs[grp[k]];
                }
            }
            boolean anyPend = false;
            boolean allPlanted = true;
            boolean anyCd = false;
            double maxDisp = -1e18;
            for (int k = 0; k < nGl; k++) {
                if (gl[k].pending) anyPend = true;
                if (!gl[k].planted) allPlanted = false;
                if (gl[k].cooldown > 0.0) anyCd = true;
                if (gl[k].displacement > maxDisp) maxDisp = gl[k].displacement;
            }
            if (!anyPend || !allPlanted || anyCd || maxDisp <= 0.05 * s) {
                continue;
            }
            int others = 0;
            for (int i = 0; i < m.n; i++) {
                if (!m.legs[i].planted) {
                    continue;
                }
                boolean inG = false;
                for (int k = 0; k < nGl; k++) {
                    if (gl[k] == m.legs[i]) {
                        inG = true;
                        break;
                    }
                }
                if (!inG) {
                    others++;
                }
            }
            int needMin = Math.min(gait.minSupport, m.n - nGl);
            if (others < needMin || !groupMayLift(m, grp, gSz)) {
                continue;
            }
            for (int k = 0; k < nGl; k++) {
                durs[k] = planStep(m, gl[k], swDur, m.tmpBool, m.tmpFhOut);
                fhs[k] = m.tmpBool[0];
                fhps[k] = m.tmpFhOut.pt;
                fhns[k] = m.tmpFhOut.n;
                gl[k].pending = false;
            }
            boolean allFutile = true;
            for (int k = 0; k < nGl; k++) {
                if (!Foothold.futileStep(gl[k], fhs[k], fhps[k])) {
                    allFutile = false;
                }
            }
            if (allFutile) {
                continue;
            }
            for (int k = 0; k < nGl; k++) {
                lift(m, gl[k], durs[k], fhs[k], fhps[k], fhns[k], null, null);
            }
        }
    }

    private static void stepRhythmicLegs(Mech m, double tStance, double swDur, double s) {
        double minSt = Math.max(0.33 * tStance, m.gait.minSupport > 0 ? 0.155 : 0.0);
        for (int i = 0; i < m.n; i++) {
            Leg l = m.legs[i];
            if (l.pending && l.planted && l.cooldown <= 0.0 && l.stanceT >= minSt
                    && l.displacement > 0.05 * s && canLift(m, l, false, true)) {
                l.pending = false;
                double dur = planStep(m, l, swDur, m.tmpBool, m.tmpFhOut);
                if (!Foothold.futileStep(l, m.tmpBool[0], m.tmpFhOut.pt)) {
                    lift(m, l, dur, m.tmpBool[0], m.tmpFhOut.pt, m.tmpFhOut.n, null, null);
                }
            }
        }
    }

    private static void settleAtRest(Mech m, Vec3.Frame sf, Vec3 upP, double ride, double s, MechDef d) {
        if (m.speed > 0.25) {
            return;
        }
        double[] cDist = m.tmpRems;
        Vec3[] cRest = m.tmpGroupPts;
        int[] cIdx = m.tmpKeyPri;
        int nc = 0;
        for (int i = 0; i < m.n; i++) {
            if (m.legs[i].planted) {
                Vec3 rc = m.pos.add(sf.toWorld(m.legs[i].restLocal(ride, 1.0)));
                cDist[nc] = m.legs[i].foot.sub(rc).projectPlane(upP).len();
                cRest[nc] = rc;
                cIdx[nc] = i;
                nc++;
            }
        }
        int[] order = m.tmpOrder;
        for (int i = 0; i < nc; i++) {
            order[i] = i;
        }
        for (int i = 1; i < nc; i++) {
            int key = order[i];
            int j = i - 1;
            while (j >= 0 && cDist[order[j]] < cDist[key]) {
                order[j + 1] = order[j];
                j--;
            }
            order[j + 1] = key;
        }
        for (int oi = 0; oi < nc; oi++) {
            int ci = order[oi];
            double curD = cDist[ci];
            if (curD <= 0.06 + 0.05 * s) {
                break;
            }
            Leg cand = m.legs[cIdx[ci]];
            if (cand.cooldown > 0.0 || cand.hasSettledAt) {
                continue;
            }
            cand.hasSettledAt = true;
            cand.settledAt = cRest[ci];
            cand.hasSettledHullPos = true;
            cand.settledHullPos = m.pos;
            if (!canLift(m, cand, false, true)) {
                continue;
            }
            if (!Foothold.findFoothold(m, cand, cRest[ci], 0.0, m.tmpFhOut)
                    || m.tmpFhOut.pt.dist(cand.foot) < 0.15) {
                continue;
            }
            double dn = m.tmpFhOut.pt.sub(cRest[ci]).projectPlane(upP).len();
            if (dn > curD - 0.05) {
                continue;
            }
            double settleDur = GaitCore.clamp(d.swingMin * 1.05 + 0.18 * curD, d.swingMin * 1.05, d.swingMax * 0.55);
            lift(m, cand, settleDur, true, m.tmpFhOut.pt, m.tmpFhOut.n, null, null);
            break;
        }
    }

    static void updateLegs(Mech m, double dt) {
        boolean prof = m.profileEnabled;
        double t0 = prof ? GaitCore.timeSec() : 0.0;
        Vec3.Frame fr = m.frame;
        MechDef d = m.d;
        double s = m.size;
        Vec3 upP = m.probeUp();
        Vec3 visPos = m.pos.add(fr.toWorld(m.visOffset));
        double ride = m.rideEff();
        Vec3.Frame sf = m.supportFrame();

        m.vt = m.vel.projectPlane(upP);
        m.speed = m.vt.len();
        boolean driven = Math.abs(m.inp.fwd) > 1e-3 || Math.abs(m.inp.strafe) > 1e-3
                || Math.abs(m.inp.turn) > 1e-3 || m.drifting || m.state == GaitCore.STATE_MANTLE;
        boolean moving = m.speed > (driven ? 0.12 : 0.45) * Math.sqrt(s)
                || Math.abs(m.yawRate) > 0.12 || m.drifting || m.state == GaitCore.STATE_MANTLE;

        Gait gait = m.gait;
        double swingFrac = 1.0 - m.duty;
        double stride = gait.stride * s * (m.crouch > 0.5 ? 0.75 : 1.0);
        double turnSpd = Math.abs(m.yawRate) * m.reachAvg * 0.65;
        double effSpd = m.drifting
                ? Math.max(m.speed * 0.2, Math.abs(m.yawRate) * m.reachAvg)
                : Math.max(m.speed, turnSpd);
        boolean groundedWalk = m.state == GaitCore.STATE_GROUND && !mayFlight(m) && !m.drifting;
        double contactSpan = groundedWalk ? m.chordAvg : (1.25 * m.reachAvg);
        double tContact = contactSpan / Math.max(effSpd, 1e-3);
        double cmin = Math.min(gait.cycleMin * Math.sqrt(s), tContact / Math.max(m.duty, 0.2));
        if (groundedWalk && effSpd > 4.0) {
            double cminSw = Math.max(0.095, 0.82 * d.swingMin) / Math.max(swingFrac, 0.2);
            cmin = Math.max(cminSw, Math.min(cmin, tContact / Math.max(swingFrac, 0.2)));
        }
        double T = GaitCore.clamp(stride / Math.max(effSpd, 1e-3), cmin, Math.max(cmin, gait.cycleMax * Math.sqrt(s)));
        m.cycle = T;
        if (m.flightHold > 0.0) {
            m.flightHold -= dt;
        } else if (moving && (m.state == GaitCore.STATE_GROUND || m.state == GaitCore.STATE_MANTLE)) {
            m.phase = GaitCore.mod1(m.phase + dt / T);
        }

        double swFloor = groundedWalk
                ? Math.max(0.095, Math.min(d.swingMin, T * swingFrac))
                : Math.max(0.05, Math.min(d.swingMin, T * swingFrac));
        double swDur = GaitCore.clamp(T * swingFrac, swFloor, d.swingMax);
        if (!m.drifting && m.driftAmt > 0.05 && m.slideCd > 0.0 && groundedWalk && effSpd > 4.0) {
            double tSince = Math.max(0.0, 0.35 - m.slideCd);
            boolean anyStale = false;
            for (int i = 0; i < m.n; i++) {
                if (m.legs[i].planted && m.legs[i].stanceT + 0.02 >= tSince) {
                    anyStale = true;
                    break;
                }
            }
            if (anyStale) {
                double recFloor = Math.max(0.095, 0.78 * d.swingMin);
                swDur = Math.max(recFloor, swDur * (1.0 - 0.45 * m.driftAmt));
                double decay = Math.exp(-8.0 * dt);
                for (int i = 0; i < m.n; i++) {
                    if (m.legs[i].swing.active && m.legs[i].swing.dur > swDur) {
                        m.legs[i].swing.dur = Math.max(swDur, m.legs[i].swing.dur * decay);
                    }
                }
            }
        }
        double tStance = T * m.duty;
        m.tAhead = moving ? (swDur + 0.5 * tStance) : 0.0;

        updateLegTargets(m, dt, fr, sf, visPos, upP, ride, swDur, moving, driven);
        if (prof) {
            double t1 = GaitCore.timeSec();
            m.prof[2] += t1 - t0;
            t0 = t1;
        }

        if (m.state == GaitCore.STATE_AIR || m.wireActive()) {
            Clearance.legsAir(m, dt);
            if (prof) {
                double t1 = GaitCore.timeSec();
                m.prof[3] += t1 - t0;
                t0 = t1;
            }
            Clearance.feetClear(m, dt);
            if (prof) {
                double t1 = GaitCore.timeSec();
                m.prof[5] += t1 - t0;
            }
            for (int i = 0; i < m.n; i++) {
                Clearance.solveLeg(m, m.legs[i], dt);
            }
            return;
        }

        int nplPre = 0;
        for (int i = 0; i < m.n; i++) {
            if (m.legs[i].planted) nplPre++;
        }
        boolean pushedPre = m.trailingSupport;
        double ikPre = prof ? (m.prof[6] + m.prof[7]) : 0.0;
        int justPlanted = updateSwings(m, dt, upP, tStance, moving);
        updateSearching(m, dt, upP, swDur, ride);
        if (prof) {
            double t1 = GaitCore.timeSec();
            double delta = (t1 - t0) - ((m.prof[6] + m.prof[7]) - ikPre);
            if (delta > 0.0) {
                m.prof[3] += delta;
            }
            t0 = t1;
        }

        if (m.drifting) {
            stepDriftLegs(m, dt, swDur, upP, d);
        } else if (m.state != GaitCore.STATE_CROUCHJUMP) {
            stepReactiveLegs(m, dt, fr, upP, gait, swingFrac, swDur, effSpd, moving, driven, d);
            if (moving && m.flightT <= 0.0) {
                if (gait.groupCount > 0) {
                    syncGroups(m, gait, swDur);
                } else if (m.blend.t < 1.0 && m.blend.s >= 0.70 && m.blend.dst.groupCount > 0) {
                    presyncGroups(m, m.blend.dst);
                }
            }
            boolean anySwing = false;
            for (int i = 0; i < m.n; i++) {
                if (m.legs[i].swing.active) {
                    anySwing = true;
                    break;
                }
            }
            if (moving && gait.groupCount > 0) {
                stepRhythmicGroups(m, gait, swDur, s);
            } else if (moving) {
                stepRhythmicLegs(m, tStance, swDur, s);
            } else if (!anySwing) {
                settleAtRest(m, sf, upP, ride, s, d);
            }
        }
        if (prof) {
            double t1 = GaitCore.timeSec();
            m.prof[4] += t1 - t0;
            t0 = t1;
        }

        Clearance.feetClear(m, dt);
        if (prof) {
            double t1 = GaitCore.timeSec();
            m.prof[5] += t1 - t0;
        }

        int nplPost = 0;
        for (int i = 0; i < m.n; i++) {
            if (m.legs[i].planted) nplPost++;
        }
        boolean stressedGrounded = false;
        boolean mayFl = mayFlight(m);
        boolean runIk = true;
        if (m.ikSolveHz > 0.0) {
            double period = 1.0 / m.ikSolveHz;
            m.ikAcc += dt;
            if (m.ikAcc + 1e-6 >= period || justPlanted != 0 || m.state != GaitCore.STATE_GROUND) {
                m.ikAcc = (m.ikAcc >= period) ? (m.ikAcc % period) : 0.0;
                runIk = true;
            } else {
                runIk = false;
            }
        }
        for (int i = 0; i < m.n; i++) {
            Leg l = m.legs[i];
            if ((justPlanted & (1 << l.idx)) == 0 || l.sliding) {
                if (runIk) {
                    Clearance.solveLeg(m, l, dt);
                } else {
                    l.reprojectCached(m);
                    if (prof) {
                        m.prof[9] += 1.0;
                    }
                }
            }
            boolean stressed = !m.drifting && l.planted
                    && (l.stretch > 0.95 || l.ik.coxaRatio > 0.84 || l.nearLimit);
            l.limitT = stressed ? (l.limitT + dt) : 0.0;
            if (stressed && !mayFl && nplPost <= Math.max(m.n / 2, 1)) {
                stressedGrounded = true;
            }
        }

        boolean anyLimitT = false;
        for (int i = 0; i < m.n; i++) {
            if (m.legs[i].limitT > 0.05) {
                anyLimitT = true;
                break;
            }
        }
        if (!m.drifting && (stressedGrounded || anyLimitT)) {
            double rate = stressedGrounded ? 4.5 : 2.5;
            double decayDur = Math.exp(-rate * dt);
            double minDur = Math.max(0.095, 0.78 * d.swingMin);
            for (int i = 0; i < m.n; i++) {
                if (m.legs[i].swing.active) {
                    m.legs[i].swing.dur = Math.max(minDur, m.legs[i].swing.dur * decayDur);
                }
            }
        }

        if (m.state == GaitCore.STATE_GROUND && m.flightT <= 0.0 && nplPre > 0) {
            int nplNow = 0;
            for (int i = 0; i < m.n; i++) {
                if (m.legs[i].planted) nplNow++;
            }
            if (nplNow == 0) {
                double[] vUpArr = m.tmpVUp;
                vUpArr[0] = m.vel.dot(upP);
                if (tryTakeoff(m, upP, m.vt, vUpArr, pushedPre)) {
                    m.vel = m.vt.add(m.slideVel).add(upP.mul(vUpArr[0]));
                    m.prevNpl = 0;
                }
            }
        }

        if (m.state == GaitCore.STATE_GROUND && m.flightT <= 0.0) {
            boolean anyPl = false;
            for (int i = 0; i < m.n; i++) {
                if (m.legs[i].planted) {
                    anyPl = true;
                    break;
                }
            }
            if (!anyPl) {
                Leg best = null;
                double bestT = -1.0;
                for (int i = 0; i < m.n; i++) {
                    Leg x = m.legs[i];
                    if (x.swing.active && x.swing.hasGround && x.swing.t >= 0.90 && x.swing.t > bestT) {
                        bestT = x.swing.t;
                        best = x;
                    }
                }
                if (best != null
                        && best.foot.dist(best.swing.target) < 0.18
                        && best.swing.target.dist(best.mountWorld) <= best.reach * 0.99
                        && best.ik.feasible(best.mountWorld, m.frame, best.swing.target, 3.0)) {
                    best.foot = best.swing.target;
                    best.footNormal = best.swing.normal;
                    best.planted = true;
                    best.swing.active = false;
                    best.searching = false;
                    onPlant(m, best);
                }
            }
        }
    }
}
