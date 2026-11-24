"""Gait scheduler, CPG+reactive stepping, flight takeoff/landing, and prediction."""
import math
import time

from .vmath import (V3, WORLD_UP, clamp, smoothstep, damp_factor, project_on_plane,
                    rotate_axis, turn_displacement, convex_hull_2d, support_margin)
from .leg import LANDING_RAMP_REACH
from .mech_const import (G, GROUND, AIR, MANTLE, CROUCHJUMP, EV_STEP, EV_SPARK,
                         EV_SKID, _dist_to_segment)


class SchedulerMixin:
    """Mixin supplying CPG+reactive step scheduling, prediction, and flight control."""

    def _may_flight(self):
        """Whether the current gait & state permit an unsupported ballistic hop."""
        if (self.duty >= 0.48 or self.gait.min_support != 0 or self.calm_gait or self.braking
                or self.state != GROUND or self.on_wall or self.drifting or self.crouch >= 0.5):
            return False
        if self.gait.duty_fast is not None and not self._fast_hopper():
            return False
        b = self.d.get("bound", 0.0) * self.gait.leap
        v_leg = self.v_leg()
        k = clamp((self.speed - 0.3 * v_leg) / (0.7 * v_leg), 0.0, 1.0)
        h_min = 0.016 if self.gait.duty_fast is not None else 0.03
        return b * k > h_min

    def _try_takeoff(self, up, vt_new, v_up, pushed):
        """Launch a bounding/running flight arc when all feet leave the ground."""
        if self.flight_t > 0.0 or not self._may_flight():
            return False, v_up
        paired = self.gait.pair_sync
        reverse = vt_new.dot(self.heading_fwd()) < 0.0
        lead = self.legs[-1] if reverse else self.legs[0]
        tail = self.legs[0] if reverse else self.legs[-1]
        swings = [l for l in self.legs if l.swing is not None]
        if not swings or not all(l.swing.has_ground for l in swings):
            return False, v_up
        recovery = False
        if paired and not pushed:
            # Inverted contact order left the front pair pushing last; launch a corrective
            # re-phasing hop rather than locking into zero-flight hovercraft levitation.
            if not any(l.pair == tail.pair and l.swing.t < 0.30 for l in swings):
                return False, v_up
            recovery = True
        b = self.d.get("bound", 0.0) * self.gait.leap * (0.55 if recovery else 1.0)
        v_leg = self.v_leg()
        k = clamp((self.speed - 0.3 * v_leg) / (0.7 * v_leg), 0.0, 1.0)
        h = b * k
        t_land = min((1.0 - l.swing.t) * l.swing.dur for l in swings)
        t_fl = max(t_land, 2.0 * math.sqrt(2.0 * h / G))
        extra = t_fl - t_land
        nxt = min(swings, key=lambda l: (1.0 - l.swing.t) * l.swing.dur) if (not paired and self.gait.groups) else None
        for l in swings:
            rem = 1.0 - l.swing.t
            if rem > 1e-3:
                if paired:
                    delay = abs(lead.offset - l.offset) * self.cycle
                    l.swing.dur = (t_fl + delay) / rem
                elif self.gait.groups:
                    d_off = min((l.offset - nxt.offset) % 1.0, (nxt.offset - l.offset) % 1.0)
                    if d_off < 0.08:
                        l.swing.dur = t_fl / rem
                    else:
                        min_rem = t_fl + 0.04
                        if rem * l.swing.dur < min_rem:
                            l.swing.dur = min_rem / rem
                else:
                    l.swing.dur += extra / rem
                l.swing.retarget_cd = 0.0
                l.swing.force_retarget = True
        if paired:
            to_stance = (1.0 - self.duty - lead.offset - self.phase) % 1.0
            self.flight_hold = max(0.0, t_fl - to_stance * self.cycle)
        elif self.gait.groups:
            to_stance = (1.0 - self.duty - nxt.offset - self.phase) % 1.0
            self.flight_hold = max(0.0, t_fl - to_stance * self.cycle)
        else:
            self.flight_hold = extra
        self.flight_t = self.flight_dur = t_fl
        self.flight_y0 = self.pos.y
        self.flight_h = G * t_fl * t_fl / 8.0
        self.flight_count += 1
        lp = self.pos + self.planar_travel(t_fl)
        lh = self.world.raycast(V3(lp.x, self.pos.y + 4.0, lp.z), WORLD_UP * -1.0, self.ride + 8.0)
        if lh is not None and lh.normal.y > 0.64:
            dy = clamp((lh.point.y + self.ride_eff()) - self.pos.y, -2.5, 2.5)
            v_up = clamp(dy / t_fl + 0.5 * G * t_fl, 0.25 * G * t_fl, 0.85 * G * t_fl)
        else:
            v_up = 0.5 * G * t_fl
        return True, v_up

    def _update_flight(self, dt, npl, vt_new, up, v_up):
        """Gallop flight trajectory and touchdown handoff."""
        took_off = False
        reverse = vt_new.dot(self.heading_fwd()) < 0.0
        tail = self.legs[0] if reverse else self.legs[-1]
        pushed = self.trailing_support
        self.trailing_support = any(l.planted and l.pair == tail.pair for l in self.legs)
        if self.flight_t <= 0.0 and npl == 0 and self.prev_npl > 0:
            took_off, v_up = self._try_takeoff(up, vt_new, v_up, pushed)
        self.prev_npl = npl
        if self.flight_t <= 0.0:
            return False
        if npl > 0 and not took_off:
            self.flight_t = 0.0
            self.absorb = max(self.absorb, clamp(-v_up / 36.0, 0.0, 0.16))
            return False
        self.flight_t -= dt
        v_up -= G * dt
        self.vel = vt_new + self.slide_vel + up * v_up
        self.unsupported_t = 0.0
        return True

    @staticmethod
    def _align_swing_group(sw):
        rems = [(1.0 - x.swing.t) * x.swing.dur for x in sw]
        t_land = max(rems)
        for x, r in zip(sw, rems):
            extra = t_land - r
            rem = 1.0 - x.swing.t
            if 0.01 < extra <= 0.15 and rem > 1e-3:
                x.swing.dur = (rem * x.swing.dur + extra) / rem
                x.swing.force_retarget = True
        return t_land

    def _presync_groups(self, dst_gait):
        """Late in a blend, align active swings to the incoming gait's groups."""
        for grp in dst_gait.groups:
            gl = [self.legs[j] for j in grp if j < self.n]
            sw = [x for x in gl if x.swing is not None]
            if len(sw) >= 2:
                self._align_swing_group(sw)

    def _sync_groups(self, gait, sw_dur):
        """Legs of one group land together."""
        for grp in gait.groups:
            gl = [self.legs[j] for j in grp if j < self.n]
            sw = [x for x in gl if x.swing is not None]
            if not sw or len(sw) == len(gl) and len(gl) < 2:
                continue
            t_land = self._align_swing_group(sw)
            late = [x for x in gl if x.planted and x.cooldown <= 0.0]
            if late and len(sw) < len(gl) and t_land > max(0.08, 0.5 * sw_dur) and self._group_may_lift(grp):
                for x in late:
                    if self._can_lift(x, emergency=True):
                        self._lift(x, *self._plan_step(x, t_land))

    def _group_may_lift(self, grp):
        """Whether a leg group may leave the ground without dropping below support."""
        others = sum(1 for x in self.legs if x.planted and x.idx not in grp)
        if self.gait.pair_sync and not self.trailing_support and others == 0:
            return False
        if self.duty < 0.48 and self._may_flight():
            return True
        return others >= min(len(grp), self.n - len(grp))

    def _fast_hopper(self):
        return self.d.get("bound", 0.0) * self.gait.leap >= 0.015

    def galloping_fast(self, blend=False):
        b = self.blend
        flying = self.gait.min_support == 0 or (blend and b.blending and (b.src.min_support == 0 or b.dst.min_support == 0))
        return flying and self.speed >= max(0.5 * self.v_leg(), self.v_walk)

    def v_leg(self):
        g = self.gait
        return 1.0 * self.reach_avg / max(0.05, self.duty * g.cycle_min * math.sqrt(self.size))

    def _lead_cap(self):
        return min(self.d["zone"] * 1.2 * self.gait.reach, self.reach_avg * 0.75)

    def planar_travel(self, seconds):
        if self.drifting:
            return self.vt * seconds
        travel = self.vt * seconds
        if self.state != GROUND or self.climbing_mode:
            return travel
        change = self.drive_target - self.vt
        delta = change.length()
        if self.drive_accel > 0.0 and delta > 1e-5:
            ramp = min(seconds, delta / self.drive_accel)
            travel = travel + change * (self.drive_accel * ramp * (seconds - 0.5 * ramp) / delta)
        if self.galloping_fast() or (self.speed > 4.0 and abs(self.yaw_rate) > 0.2):
            travel = turn_displacement(travel, WORLD_UP, -self.yaw_rate * seconds)
        return travel

    def _leg_fwd_room(self, l, direction_local, ride):
        """Workspace travel ahead of rest position along local direction."""
        ox = l.rest_xz[0] - l.mount_local.x
        oy = -ride - l.mount_local.y
        oz = l.rest_xz[1] - l.mount_local.z
        dx, dy, dz = direction_local.x, direction_local.y, direction_local.z
        along = ox * dx + oy * dy + oz * dz
        off2 = ox * ox + oy * oy + oz * oz
        rlim = 0.94 * l.reach
        room = math.sqrt(max(0.0, rlim * rlim - off2 + along * along)) - along
        if l.ik.mode == "radial" and l.ik.coxa_max < 3.0:
            rx, rz = l.ik.rest_dir_local.x, l.ik.rest_dir_local.z
            lim = max(0.2, l.ik.coxa_max - 0.10)
            cl, sl = math.cos(lim), math.sin(lim)
            for sgn in (-1.0, 1.0):
                ux = rx * cl + rz * (sgn * sl)
                uz = -rx * (sgn * sl) + rz * cl
                det = dx * uz - dz * ux
                if abs(det) > 1e-5:
                    s = (ux * oz - uz * ox) / det
                    t = (dx * oz - dz * ox) / det
                    if s > 0.0 and t > 0.0:
                        room = min(room, s)
        if abs(dz) > 0.1:
            adz = abs(dz)
            for nb in self.legs:
                if nb.side == l.side and nb is not l:
                    gap = nb.rest_xz[1] - l.rest_xz[1]
                    if gap * dz > 0.05:
                        safe_dz = max(0.0, 0.5 * abs(gap) - 0.35)
                        room = min(room, safe_dz / adz)
        return max(0.0, room)

    def _predict(self, l, t_ahead, t_land=0.0, ctx=None):
        """Place the foot so the hip passes over its rest position during stance."""
        if ctx is not None:
            fr, ride, dynamic, travel, lead, direction, lead_cap = ctx
        else:
            fr = self.support_frame()
            ride = self.ride_eff()
            dynamic = False
            travel = lead = direction = None
            lead_cap = 0.0
        widen = 1.0
        if l.ik.mode == "radial" and abs(self.yaw_rate) > 0.5 and self.speed > 6.0:
            widen = 1.0 + clamp(abs(self.yaw_rate) * self.speed / 280.0, 0.0, 0.22)
        rl = fr.to_world(l.rest_local(ride, widen))
        if self.state == MANTLE and self.mantle is not None:
            m = self.mantle
            if l.force or (l.is_front and m.t < 0.5):
                return m.top + m.fwd * 0.4 + fr.right * l.rest_xz[0]
            base = self._mantle_pos(m.t + t_land / m.dur)
            return V3(base.x, base.y, base.z) + rl
        if self.balance_out is not None and rl.dot(self.balance_out) > 0.0:
            rl = rl + self.balance_out * min(1.5 * self.balance_excess + 0.2, 0.4 * l.reach)
        if ctx is None:
            dynamic = t_land > 0.0 and (self.galloping_fast() or ((self.drifting or self.drift_amt > 0.05) and self.speed > 4.0))
            lead_cap = self._lead_cap()
            if dynamic:
                travel = self.planar_travel(t_land)
                lead = self.planar_travel(t_ahead) - travel
                if lead.length_sq() > 1e-8:
                    lead_dir = (rotate_axis(lead.normalized(), fr.up, self.yaw_rate * t_land)
                                if abs(self.yaw_rate * t_land) > 1e-3 else lead.normalized())
                    direction = fr.to_local(lead_dir)
                else:
                    direction = None
            else:
                travel = V3()
                lead = self.vt * t_ahead
        if dynamic:
            if direction is not None:
                room = self._leg_fwd_room(l, direction, ride)
                reserve = min(0.45 * LANDING_RAMP_REACH * l.reach, 0.25 * self.speed * self.cycle * (1.0 - self.duty))
                cap = min(0.75 * lead_cap, 0.28 * l.reach, max(0.0, room - reserve))
            else:
                cap = 0.0
        else:
            cap = lead_cap
        if self.balance_out is not None:
            lead = lead - self.balance_out * max(0.0, lead.dot(self.balance_out))
        if lead.length_sq() > cap * cap:
            lead = lead.normalized() * cap
        if abs(self.yaw_rate) > 1e-3:
            if dynamic:
                stance_rot = clamp(self.yaw_rate * (t_ahead - t_land), -0.6, 0.6)
                rl = rotate_axis(rl, fr.up, -self.yaw_rate * t_land - stance_rot)
            else:
                yaw_t = (t_land + 0.5 * max(0.0, t_ahead - t_land)) if t_land > 0.0 else (t_ahead * 0.5)
                rl = rotate_axis(rl, fr.up, -clamp(self.yaw_rate * yaw_t, -0.75, 0.75))
        return self.pos + travel + rl + lead

    def _swing_bow(self, l, start, target, dur):
        """Horizontal/apex swing shaping."""
        if dur < 0.14:
            return V3()
        up_p = self.probe_up()
        bow = V3()
        turn_arc = smoothstep((abs(self.yaw_rate) * dur - 0.12) / 0.25)
        if self.d["plane"] == "radial" and turn_arc < 0.999:
            hip_mid, frame_mid = self._landing_pose(l, 0.5 * dur)
            to_mid = project_on_plane(0.5 * (start + target) - hip_mid, up_p)
            rest_r = math.hypot(l.rest_xz[0] - l.mount_local.x, l.rest_xz[1] - l.mount_local.z)
            out = project_on_plane(frame_mid.to_world(l.ik.rest_dir_local), up_p).normalized()
            r_mid = to_mid.length()
            out_mid = to_mid.dot(out)
            if r_mid < 0.70 * rest_r or out_mid < 0.20 * rest_r:
                clearance = min(r_mid, out_mid)
                bow = bow + out * ((0.75 * rest_r - clearance) * (1.0 - turn_arc))
        if self.flight_t > 0.0 and self.gait.leap >= 0.4 and self.gait.stride_pitch > 0.2:
            scale = min(1.0, self.gait.leap * self.gait.stride_pitch)
            zmax = max(abs(x.mount_local.z) for x in self.legs) or 1.0
            dirz = clamp(l.mount_local.z / zmax, -1.0, 1.0)
            bow = bow + (self.heading_fwd() * (dirz * 0.14 * l.reach) + up_p * (0.12 * self.ride)) * scale
        max_bow = min(0.90 * l.reach, 2.5 * dur * l.reach)
        bl = bow.length()
        if bl > max_bow:
            bow = bow * (max_bow / bl)
        return bow

    def _update_leg_targets(self, dt, fr, sf, vis_pos, up_p, ride, sw_dur, moving, driven):
        pred_ctx = None
        if moving and not (self.state == MANTLE and self.mantle is not None):
            dynamic = sw_dur > 0.0 and (self.galloping_fast() or ((self.drifting or self.drift_amt > 0.05) and self.speed > 4.0))
            lead_cap = self._lead_cap()
            if dynamic:
                travel = self.planar_travel(sw_dur)
                lead = self.planar_travel(self.t_ahead) - travel
                if lead.length_sq() > 1e-8:
                    lead_dir = (rotate_axis(lead.normalized(), sf.up, self.yaw_rate * sw_dur)
                                if abs(self.yaw_rate * sw_dur) > 1e-3 else lead.normalized())
                    direction = sf.to_local(lead_dir)
                else:
                    direction = None
            else:
                travel = V3()
                lead = self.vt * self.t_ahead
                direction = None
            pred_ctx = (sf, ride, dynamic, travel, lead, direction, lead_cap)

        rest_scale = 1.0 + 0.2 * self.brace + 0.22 * self.drift_amt
        flat_up = up_p is WORLD_UP
        yaw_active = abs(self.yaw_rate) > 0.25
        for l in self.legs:
            l.mount_world = vis_pos + fr.to_world(l.mount_local)
            l.ideal_now = self.pos + sf.to_world(l.rest_local(ride, rest_scale))
            l.ideal_pred = self._predict(l, self.t_ahead, sw_dur, pred_ctx) if moving else l.ideal_now
            if flat_up:
                l.displacement = math.hypot(l.foot.x - l.ideal_now.x, l.foot.z - l.ideal_now.z)
            else:
                dv = l.foot - l.ideal_now
                dv = dv - up_p * dv.dot(up_p)
                l.displacement = dv.length()
            l.off_path = _dist_to_segment(l.foot, l.ideal_now, l.ideal_pred, up_p)
            if l.land_lead is not None:
                l.off_path = min(l.off_path, _dist_to_segment(
                    l.foot, l.ideal_now, l.ideal_now + sf.to_world(l.land_lead), up_p))
            l.cooldown -= dt
            l.stance_t = l.stance_t + dt if l.planted else 0.0
            l.void_mark -= dt
            l.sliding = False
            if (driven or yaw_active
                    or (l.settled_hull_pos is not None
                        and math.hypot(self.pos.x - l.settled_hull_pos.x, self.pos.z - l.settled_hull_pos.z) > 0.25 * self.size)):
                l.settled_at = None

        for l in self.legs:
            if l.carry is not None and not l.planted and l.swing is None:
                l.foot = self.pos + fr.to_world(l.carry)

        sl = self.slide_vel.length()
        if sl > 0.05 and self.state == GROUND:
            for l in self.legs:
                if l.planted:
                    old = l.foot
                    l.foot = l.foot + project_on_plane(self.slide_vel, l.foot_normal) * dt
                    l.sliding = True
                    self._slide_foot_clear(l, up_p)
                    if sl > 0.8:
                        self.emit(EV_SKID, old, l.foot_normal, strength=self.d["widths"][-1] * 0.8, extra=l.foot)

    def _update_swings(self, dt, up_p, t_stance, moving):
        just_planted = set()
        for l in self.legs:
            if l.swing is None:
                continue
            sw = l.swing
            rem = max(0.0, (1.0 - sw.t) * sw.dur - dt)
            if (rem > 0.05 or sw.force_retarget) and self.state != CROUCHJUMP:
                sw.retarget_cd -= dt
                if sw.retarget_cd <= 0.0 or sw.force_retarget:
                    sw.retarget_cd = 0.03 if self.braking else (0.05 if (self.galloping_fast() or self.drifting) else 0.1)
                    pred = self._predict(l, rem + 0.5 * t_stance, rem) if moving else l.ideal_now
                    ref = sw.ideal if sw.ideal is not None else sw.target
                    forced = sw.force_retarget
                    sw.force_retarget = False
                    if (pred.dist(ref) > 0.25 or forced
                            or (self.drifting and sw.end_velocity.length_sq() > 0.0)
                            or (not self.drifting and sw.end_velocity.length_sq() > 0.25)):
                        fh = self._find_foothold(l, pred, rem)
                        sw.ideal = pred
                        if fh is not None:
                            end_vel = (project_on_plane(self.vt, fh[1]) * 0.95) if self.drifting else V3()
                            if (forced or not sw.has_ground or fh[0].dist(sw.target) > 0.15
                                    or end_vel.dist(sw.end_velocity) > 0.8):
                                l.retarget(fh[0], fh[1], True, self.world, self._clearance(), self,
                                           end_vel=end_vel, bow=self._swing_bow(l, l.foot, fh[0], rem),
                                           max_ceil_y=self.ceil_y)
                                self._set_swing_arc(l, fh[0], rem)
                        elif forced:
                            end_vel = (project_on_plane(self.vt, up_p) * 0.95) if self.drifting else V3()
                            l.retarget(pred, up_p, False, self.world, 0.0, self,
                                       end_vel=end_vel, bow=self._swing_bow(l, l.foot, pred, rem),
                                       max_ceil_y=self.ceil_y)
                            self._set_swing_arc(l, pred, rem)
            if l.update_swing(dt, self, self.world):
                self._on_plant(l)
                just_planted.add(l.idx)
            elif (l.swing is not None and l.swing.arc_w > 1e-4
                    and l.swing.start_hip_l is not None and l.swing.target_hip_l is not None):
                sw = l.swing
                rem_s = (1.0 - sw.t) * sw.dur
                fade_end = min(0.08, 0.16 * sw.dur)
                w_arc = sw.arc_w * smoothstep(sw.t / 0.12) * smoothstep(rem_s / max(1e-4, fade_end))
                if w_arc > 1e-4:
                    span = max(1e-4, 1.0 - sw.arc_t0)
                    s_loc = smoothstep(clamp((sw.t - sw.arc_t0) / span, 0.0, 1.0))
                    rel_l = self._eval_swing_arc(l, sw.start_hip_l, sw.target_hip_l, s_loc)
                    p_body = l.mount_world + self.frame.to_world(rel_l)
                    l.foot = l.foot + project_on_plane(p_body - l.foot, up_p) * w_arc
        return just_planted

    def _update_searching(self, dt, up_p, sw_dur, ride):
        for l in self.legs:
            if l.searching:
                l.search_cd -= dt
                if l.search_cd <= 0.0:
                    l.search_cd = 0.12
                    dur, fh = self._plan_step(l, sw_dur * 0.7)
                    if fh is None and self.state == GROUND:
                        planted_legs = [x for x in self.legs if x.planted]
                        if planted_legs:
                            safe = sum((x.foot for x in planted_legs), V3()) * (1.0 / len(planted_legs))
                            for alpha in (0.60, 0.90):
                                fh = self._find_foothold(l, l.ideal_now.lerp(safe, alpha), dur)
                                if fh is not None:
                                    l.void_mark = 1.0
                                    break
                    if fh is None and l.last_plant is not None and self.state == GROUND:
                        p, n = l.last_plant
                        if p.dist(l.mount_world) < 0.93 * l.reach and self._reachable(l, p) and self._clear_spot(l, p, up_p):
                            fh = (p, n)
                            l.void_mark = 1.0
                    if fh is not None:
                        self._lift(l, dur, fh)
                        continue
                self._ease_foot(l, self._hang_pose(l, ride), damp_factor(5.0, dt))

    def _step_drift_legs(self, dt, sw_dur, up_p, d):
        spark_now = False
        self._spark_t += dt
        if self._spark_t >= 1.0 / 36.0:
            self._spark_t %= (1.0 / 36.0)
            spark_now = True
        for l in self.legs:
            if l.planted:
                slide = project_on_plane(self.vt, l.foot_normal) * 0.95
                old = l.foot
                l.foot = l.foot + slide * dt
                l.sliding = True
                self._slide_foot_clear(l, up_p)
                if spark_now:
                    self.emit(EV_SPARK, l.foot, l.foot_normal, strength=self.speed, extra=slide)
                self.emit(EV_SKID, old, l.foot_normal, strength=d["widths"][-1], extra=l.foot)
        for l in sorted(self.legs, key=lambda x: -x.stretch):
            if (l.planted and (l.stretch > 0.96 or l.ik.err > 0.02 or (l.cooldown <= 0.0 and (l.stretch > 0.90 or l.near_limit)))
                    and self._can_lift(l, emergency=True)):
                self._lift(l, *self._plan_step(l, sw_dur * 0.8))

    def _step_reactive_legs(self, dt, fr, up_p, gait, swing_frac, sw_dur, eff_spd, moving, driven, d):
        t_stance = self.cycle * self.duty
        for l in self.legs:
            p = (self.phase + l.offset) % 1.0
            in_w = p < swing_frac
            if in_w and not l.prev_window:
                l.pending = True
            if not in_w:
                l.pending = False
            l.prev_window = in_w

        zone = d["zone"] * (0.5 if self.state == MANTLE else 1.0)
        if self.state == GROUND and not self.drifting and abs(self.yaw_rate) < 0.35 and eff_spd < 0.35 * self.v_walk and driven:
            posture_s = smoothstep((eff_spd - 0.15) / max(0.5, 0.35 * self.v_walk - 0.15))
            zone *= 0.55 + 0.45 * posture_s
        vdir = self.vt.normalized() if self.speed > 0.1 else None
        t_since = (max(0.0, 0.35 - self.slide_cd)
                   if (not self.drifting and self.drift_amt > 0.05 and self.slide_cd > 0.0 and self.speed > 4.0)
                   else None)
        t_exit_look = max(0.036, sw_dur * 0.85)
        has_stale = t_since is not None and any(l.planted and l.stance_t + 0.02 >= t_since for l in self.legs)
        groups = gait.groups or ()
        walk_min_st = 0.155 if gait.min_support > 0 else 0.0

        order = (sorted(self.legs, key=lambda x: (
            0 if (x.near_limit or x.stretch > 0.94 or x.ik.err > 0.01) else 1,
            -(x.mount_world + self.vt * t_exit_look).dist(x.foot) / x.reach))
                 if has_stale else sorted(self.legs, key=lambda x: -x.displacement))
        for l in order:
            if not l.planted:
                continue
            t_look = t_exit_look if (t_since is not None and l.stance_t + 0.02 >= t_since) else 0.036
            disp = l.off_path
            l_grp = None
            for grp in groups:
                if l.idx in grp:
                    l_grp = grp
                    break
            grp_blocked = l_grp is not None and not self._group_may_lift(l_grp)
            cur_str = l.mount_world.dist(l.foot) / l.reach
            cur_infeas = cur_str > 0.99 or (
                (cur_str > 0.86 or l.ik.coxa_ratio > 0.72 or l.near_limit or self.speed > 18.0)
                and not l.ik.feasible(l.mount_world, fr, l.foot, slack_deg=0.0)
            )
            release = 0.97
            if cur_infeas:
                release = min(release, l.stretch - 1e-3)
            if vdir is not None and (l.foot - l.mount_world).dot(vdir) < 0.0:
                release -= min(0.06, 0.03 * self.speed / l.reach)
                ahead = (l.mount_world + self.vt * t_look).dist(l.foot) / l.reach
                if ahead > 0.97 and (not grp_blocked or l.stretch > 0.94):
                    release = min(release, l.stretch - 1e-3)
            if moving and (self.speed > 4.0 or abs(self.yaw_rate) > 0.5):
                if abs(self.yaw_rate) > 0.5:
                    hip_next, frame_next = self._landing_pose(l, t_look)
                else:
                    hip_next, frame_next = l.mount_world + self.vt * t_look, fr
                allow_pred = cur_infeas or not grp_blocked or l.stretch > 0.95 or l.ik.coxa_ratio > 0.94 or l.ik.err > 0.01
                if (allow_pred
                        and (l.stance_t >= max(walk_min_st, min(0.08, 0.30 * t_stance))
                             or l.stretch > 0.94 or l.ik.err > 0.01 or cur_infeas)
                        and not l.ik.feasible(hip_next, frame_next, l.foot, slack_deg=2.0)):
                    release = min(release, l.stretch - 1e-3)
                elif allow_pred and l.ik.mode == "radial" and self.speed > 10.0 and abs(self.yaw_rate) > 0.4 and l.stance_t >= 0.03:
                    rel_next = frame_next.to_local(l.foot - hip_next)
                    rest_r = math.hypot(l.rest_xz[0] - l.mount_local.x, l.rest_xz[1] - l.mount_local.z)
                    if math.hypot(rel_next.x, rel_next.z) < 0.48 * rest_r:
                        release = min(release, l.stretch - 1e-3)
                    else:
                        rd = l.ik.rest_dir_local
                        r_xz = math.hypot(rel_next.x, rel_next.z)
                        if r_xz > 1e-5:
                            inv_r = 1.0 / r_xz
                            ux, uz = rel_next.x * inv_r, rel_next.z * inv_r
                            next_yaw = math.atan2(rd.z * ux - rd.x * uz, rd.x * ux + rd.z * uz)
                            cur_yaw = math.radians(l.ik.yaw_deg)
                            diff_y = abs((next_yaw - cur_yaw + math.pi) % math.tau - math.pi)
                            if diff_y > math.radians(28.0):
                                release = min(release, l.stretch - 1e-3)
            if moving and not grp_blocked and abs(self.yaw_rate) > 0.25 and l.stance_t >= max(0.06, walk_min_st):
                side_exc = abs(fr.to_local(l.foot - self.pos).x - l.rest_xz[0])
                if side_exc > 0.46 * l.reach:
                    release = min(release, l.stretch - 1e-3)
            soon = (l.stretch + max(0.0, l.stretch_rate) * min(0.05, sw_dur * 0.35)) if (l.stance_t >= max(0.06, walk_min_st) and l.stretch > 0.88) else 0.0
            if l.cooldown > 0.0 and l.stretch <= release:
                continue
            if moving:
                if grp_blocked:
                    near_req = l.stretch > 0.94 or l.ik.err > 0.01
                else:
                    near_req = (l.stretch > 0.90 or l.stretch_rate > 0.1 or l.ik.err > 0.01
                                or (l.ik.coxa_ratio > 0.88 and abs(self.yaw_rate) > 0.25))
                near_trig = (l.near_limit and l.cooldown <= 0.0
                             and (l.stance_t >= max(0.06, walk_min_st) or l.stretch > 0.94 or l.ik.err > 0.01)
                             and (driven or self.speed > 2.0) and near_req)
                hard_emg = l.force or l.stretch > release or soon > release or near_trig
                zone_trig = False if (grp_blocked or l.stance_t < walk_min_st) else (disp > zone * gait.reach)
            else:
                hard_emg = l.force or l.stretch > 0.995
                zone_trig = False
            can = (self._can_lift(l, emergency=True) if hard_emg
                   else (self._can_lift(l, emergency=False, check_static=False) if zone_trig else False))
            if can:
                dur, fh = self._plan_step(l, sw_dur * 0.85)
                if not moving and not l.force and fh is not None:
                    cur_d = l.foot.dist(l.mount_world)
                    new_d = fh[0].dist(l.mount_world)
                    new_disp = project_on_plane(fh[0] - l.ideal_now, up_p).length()
                    if new_d >= cur_d - 0.05 and new_disp >= l.displacement - 0.05:
                        l.cooldown = max(l.cooldown, 0.3)
                        continue
                if self._futile_step(l, fh):
                    continue
                self._lift(l, dur, fh)
                for grp in groups:
                    if l.idx in grp and self._group_may_lift(grp):
                        for j in grp:
                            mate = self.legs[j] if j < self.n else None
                            if (mate is not None and mate is not l and mate.planted
                                    and self._can_lift(mate, emergency=True)):
                                self._lift(mate, *self._plan_step(mate, sw_dur * 0.85))

        if self.galloping_fast(blend=True):
            for l in self.legs:
                if l.planted and l.stretch > 1.0:
                    old = l.foot
                    l.foot = l.foot + project_on_plane(self.vt, l.foot_normal) * dt
                    l.sliding = True
                    self._slide_foot_clear(l, up_p)
                    self.emit(EV_SKID, old, l.foot_normal, strength=d["widths"][-1] * 0.6, extra=l.foot)

    def _step_rhythmic_groups(self, gait, sw_dur, s):
        for grp in gait.groups:
            gl = [self.legs[j] for j in grp if j < self.n]
            if not any(x.pending for x in gl):
                continue
            if not all(x.planted for x in gl) or any(x.cooldown > 0.0 for x in gl):
                continue
            if max(x.displacement for x in gl) <= 0.05 * s:
                continue
            others = sum(1 for x in self.legs if x.planted and x not in gl)
            if others < min(gait.min_support, self.n - len(gl)) or not self._group_may_lift(grp):
                continue
            plans = [self._plan_step(x, sw_dur) for x in gl]
            for x in gl:
                x.pending = False
            if all(self._futile_step(x, fh) for x, (_, fh) in zip(gl, plans)):
                continue
            for x, plan in zip(gl, plans):
                self._lift(x, *plan)

    def _step_rhythmic_legs(self, t_stance, sw_dur, s):
        min_st = max(0.33 * t_stance, 0.155 if self.gait.min_support > 0 else 0.0)
        for l in self.legs:
            if (l.pending and l.planted and l.cooldown <= 0.0 and l.stance_t >= min_st
                    and l.displacement > 0.05 * s and self._can_lift(l)):
                l.pending = False
                dur, fh = self._plan_step(l, sw_dur)
                if not self._futile_step(l, fh):
                    self._lift(l, dur, fh)

    def _settle_at_rest(self, sf, up_p, ride, s, d):
        if self.speed > 0.25:
            return
        cands = []
        for x in self.legs:
            if x.planted:
                rc = self.pos + sf.to_world(x.rest_local(ride, 1.0))
                cands.append((project_on_plane(x.foot - rc, up_p).length(), rc, x))
        for cur_d, rest_calm, cand in sorted(cands, key=lambda item: -item[0]):
            if cur_d <= 0.06 + 0.05 * s:
                break
            if cand.cooldown > 0.0 or cand.settled_at is not None:
                continue
            cand.settled_at = rest_calm.copy()
            cand.settled_hull_pos = self.pos.copy()
            if not self._can_lift(cand):
                continue
            fh = self._find_foothold(cand, rest_calm)
            if fh is None or fh[0].dist(cand.foot) < 0.15:
                continue
            dn = project_on_plane(fh[0] - rest_calm, up_p).length()
            if dn > cur_d - 0.05:
                continue
            settle_dur = clamp(d["swing_min"] * 1.05 + 0.18 * cur_d,
                               d["swing_min"] * 1.05, d["swing_max"] * 0.55)
            self._lift(cand, settle_dur, fh)
            break

    def _update_legs(self, dt):
        """Advance gait phase, pick swing timing, schedule lifts, and run leg IK."""
        prof = self._prof
        perf = time.perf_counter if prof is not None else None
        t0 = perf() if perf is not None else 0.0
        fr, d, s = self.frame, self.d, self.size
        up_p = self.probe_up()
        vis_pos = self.pos + fr.to_world(self.vis_offset)
        ride = self.ride_eff()
        sf = self.support_frame()
        self.vt = project_on_plane(self.vel, up_p)
        self.speed = self.vt.length()
        driven = (abs(self.inp.fwd) > 1e-3 or abs(self.inp.strafe) > 1e-3
                  or abs(self.inp.turn) > 1e-3 or self.drifting or self.state == MANTLE)
        moving = (self.speed > (0.12 if driven else 0.45) * math.sqrt(s) or abs(self.yaw_rate) > 0.12
                  or self.drifting or self.state == MANTLE)
        gait = self.gait
        swing_frac = 1.0 - self.duty
        stride = gait.stride * s * (0.75 if self.crouch > 0.5 else 1.0)
        turn_spd = abs(self.yaw_rate) * self.reach_avg * 0.65
        eff_spd = max(self.speed * 0.2, abs(self.yaw_rate) * self.reach_avg) if self.drifting else max(self.speed, turn_spd)
        grounded_walk = self.state == GROUND and not self._may_flight() and not self.drifting
        contact_span = self.chord_avg if grounded_walk else 1.25 * self.reach_avg
        t_contact = contact_span / max(eff_spd, 1e-3)
        # Clamp minimum cycle against reachable contact time so high-speed
        # grounded rigs finish swings fast enough instead of dragging behind.
        cmin = min(gait.cycle_min * math.sqrt(s), t_contact / max(self.duty, 0.2))
        if grounded_walk and eff_spd > 4.0:
            cmin_sw = max(0.095, 0.82 * d["swing_min"]) / max(swing_frac, 0.2)
            cmin = max(cmin_sw, min(cmin, t_contact / max(swing_frac, 0.2)))
        T = clamp(stride / max(eff_spd, 1e-3), cmin, max(cmin, gait.cycle_max * math.sqrt(s)))
        self.cycle = T
        if self.flight_hold > 0.0:
            self.flight_hold -= dt
        elif moving and self.state in (GROUND, MANTLE):
            self.phase = (self.phase + dt / T) % 1.0
        sw_floor = max(0.095, min(d["swing_min"], T * swing_frac)) if grounded_walk else max(0.05, min(d["swing_min"], T * swing_frac))
        sw_dur = clamp(T * swing_frac, sw_floor, d["swing_max"])
        # Exiting a slide at speed leaves old planted feet behind the COM; trim
        # swing duration briefly until slide-era plants cycle through.
        if not self.drifting and self.drift_amt > 0.05 and self.slide_cd > 0.0 and grounded_walk and eff_spd > 4.0:
            t_since = max(0.0, 0.35 - self.slide_cd)
            if any(l.planted and l.stance_t + 0.02 >= t_since for l in self.legs):
                rec_floor = max(0.095, 0.78 * d["swing_min"])
                sw_dur = max(rec_floor, sw_dur * (1.0 - 0.45 * self.drift_amt))
                decay = math.exp(-8.0 * dt)
                for l in self.legs:
                    if l.swing is not None and l.swing.dur > sw_dur:
                        l.swing.dur = max(sw_dur, l.swing.dur * decay)
        t_stance = T * self.duty
        self.t_ahead = (sw_dur + 0.5 * t_stance) if moving else 0.0

        self._update_leg_targets(dt, fr, sf, vis_pos, up_p, ride, sw_dur, moving, driven)
        if perf is not None:
            t1 = perf()
            prof[2] += t1 - t0
            t0 = t1

        if self.state == AIR or self.wire_active():
            self._legs_air(dt)
            if perf is not None:
                t1 = perf()
                prof[3] += t1 - t0
                t0 = t1
            self._feet_clear(dt)
            if perf is not None:
                t1 = perf()
                prof[5] += t1 - t0
            for l in self.legs:
                self._solve_leg(l, dt)
            return

        npl_pre_legs = sum(1 for l in self.legs if l.planted)
        pushed_pre = self.trailing_support
        ik_pre = (prof[6] + prof[7]) if prof is not None else 0.0
        just_planted = self._update_swings(dt, up_p, t_stance, moving)
        self._update_searching(dt, up_p, sw_dur, ride)
        if perf is not None:
            t1 = perf()
            prof[3] += max(0.0, (t1 - t0) - ((prof[6] + prof[7]) - ik_pre))
            t0 = t1

        if self.drifting:
            self._step_drift_legs(dt, sw_dur, up_p, d)
        elif self.state != CROUCHJUMP:
            self._step_reactive_legs(dt, fr, up_p, gait, swing_frac, sw_dur, eff_spd, moving, driven, d)
            if moving and self.flight_t <= 0.0:
                if gait.groups:
                    self._sync_groups(gait, sw_dur)
                elif self.blend.blending and self.blend.s >= 0.70 and self.blend.dst.groups:
                    self._presync_groups(self.blend.dst)
            if moving and gait.groups:
                self._step_rhythmic_groups(gait, sw_dur, s)
            elif moving:
                self._step_rhythmic_legs(t_stance, sw_dur, s)
            elif not any(l.swing is not None for l in self.legs):
                self._settle_at_rest(sf, up_p, ride, s, d)
        if perf is not None:
            t1 = perf()
            prof[4] += t1 - t0
            t0 = t1

        self._feet_clear(dt)
        if perf is not None:
            t1 = perf()
            prof[5] += t1 - t0
        npl_post = sum(1 for l in self.legs if l.planted)
        stressed_grounded = False
        may_fl = self._may_flight()
        run_ik = True
        if self.ik_solve_hz > 0.0:
            period = 1.0 / self.ik_solve_hz
            self._ik_acc += dt
            if self._ik_acc + 1e-6 >= period or just_planted or self.state != GROUND:
                self._ik_acc = self._ik_acc % period if self._ik_acc >= period else 0.0
                run_ik = True
            else:
                run_ik = False
        for l in self.legs:
            if l.idx not in just_planted or l.sliding:
                if run_ik:
                    self._solve_leg(l, dt)
                else:
                    l.reproject_cached(self)
                    if prof is not None:
                        prof[9] += 1.0
            stressed = not self.drifting and l.planted and (l.stretch > 0.95 or l.ik.coxa_ratio > 0.84 or l.near_limit)
            l.limit_t = l.limit_t + dt if stressed else 0.0
            if stressed and not may_fl and npl_post <= max(1, self.n // 2):
                stressed_grounded = True
        if not self.drifting and (stressed_grounded or any(l.limit_t > 0.05 for l in self.legs)):
            rate = 4.5 if stressed_grounded else 2.5
            decay_dur = math.exp(-rate * dt)
            min_dur = max(0.095, 0.78 * d["swing_min"])
            for l in self.legs:
                if l.swing is not None:
                    l.swing.dur = max(min_dur, l.swing.dur * decay_dur)
        if self.state == GROUND and self.flight_t <= 0.0 and npl_pre_legs > 0:
            npl_now = sum(1 for l in self.legs if l.planted)
            if npl_now == 0:
                took_off, v_up_new = self._try_takeoff(up_p, self.vt, self.vel.dot(up_p), pushed_pre)
                if took_off:
                    self.vel = self.vt + self.slide_vel + up_p * v_up_new
                    self.prev_npl = 0
        if self.state == GROUND and self.flight_t <= 0.0 and not any(l.planted for l in self.legs):
            best = max((l for l in self.legs if l.swing is not None and l.swing.has_ground and l.swing.t >= 0.90),
                       key=lambda x: x.swing.t, default=None)
            if (best is not None and best.foot.dist(best.swing.target) < 0.18
                    and best.swing.target.dist(best.mount_world) <= best.reach * 0.99
                    and best.ik.feasible(best.mount_world, self.frame, best.swing.target, slack_deg=3.0)):
                best.foot = best.swing.target.copy()
                best.foot_normal = best.swing.normal
                best.planted = True
                best.swing = None
                best.searching = False
                self._on_plant(best)

    def _plan_step(self, l, dur):
        if self.gait.phase_lock and self.state == GROUND and self.cycle > 0.0 and not self.braking:
            p = (self.phase + l.offset) % 1.0
            to_stance = ((1.0 - self.duty) - p) % 1.0
            dur = clamp(to_stance * self.cycle, 0.6 * dur, 1.4 * dur)
        dur = max(0.05, dur)
        if self.t_ahead > 0.0:
            l.ideal_pred = self._predict(l, dur + 0.5 * self.duty * self.cycle, dur)
        return dur, self._find_foothold(l, l.ideal_pred, dur)

    @staticmethod
    def _eval_swing_arc(l, sh, th, s_loc):
        if l.ik.mode == "radial":
            rd = l.ik.rest_dir_local
            px, pz = -rd.z, rd.x
            s_out, s_side = sh.x * rd.x + sh.z * rd.z, sh.x * px + sh.z * pz
            t_out, t_side = th.x * rd.x + th.z * rd.z, th.x * px + th.z * pz
            r0, r1 = math.hypot(s_out, s_side), math.hypot(t_out, t_side)
            a0, a1 = math.atan2(s_side, s_out), math.atan2(t_side, t_out)
            rad = r0 + (r1 - r0) * s_loc
            ang = a0 + (a1 - a0) * s_loc
            ca, sa = math.cos(ang), math.sin(ang)
            return V3((rd.x * ca + px * sa) * rad,
                      sh.y + (th.y - sh.y) * s_loc,
                      (rd.z * ca + pz * sa) * rad)
        return sh.lerp(th, s_loc)

    def _set_swing_arc(self, l, target, rem, reset_start=False):
        sw = l.swing
        if sw is None:
            return
        turn_ang = abs(self.yaw_rate) * max(rem, sw.dur)
        w = smoothstep((turn_ang - 0.12) / 0.25)
        sw.arc_w = w if reset_start else max(sw.arc_w, w)
        if sw.arc_w > 1e-4:
            hip_land, frame_land = self._landing_pose(l, max(rem, 0.0))
            new_th = frame_land.to_local(target - hip_land)
            if reset_start or sw.start_hip_l is None or sw.target_hip_l is None:
                sw.start_hip_l = self.frame.to_local(l.foot - l.mount_world)
                sw.arc_t0 = sw.t
                sw.target_hip_l = new_th
            else:
                span = max(1e-4, 1.0 - sw.arc_t0)
                s_cur = smoothstep(clamp((sw.t - sw.arc_t0) / span, 0.0, 1.0))
                if s_cur <= 1e-4:
                    sw.target_hip_l = new_th
                elif s_cur < 0.88:
                    inv = 1.0 / (1.0 - s_cur)
                    sh, old_th = sw.start_hip_l, sw.target_hip_l
                    if l.ik.mode == "radial":
                        rd = l.ik.rest_dir_local
                        px, pz = -rd.z, rd.x
                        cur = self._eval_swing_arc(l, sh, old_th, s_cur)
                        c_out, c_side = cur.x * rd.x + cur.z * rd.z, cur.x * px + cur.z * pz
                        n_out, n_side = new_th.x * rd.x + new_th.z * rd.z, new_th.x * px + new_th.z * pz
                        r_cur, r1 = math.hypot(c_out, c_side), math.hypot(n_out, n_side)
                        a_cur, a1 = math.atan2(c_side, c_out), math.atan2(n_side, n_out)
                        r0 = clamp((r_cur - s_cur * r1) * inv, 0.25 * l.reach, 1.15 * l.reach)
                        a0 = clamp((a_cur - s_cur * a1) * inv, -2.8, 2.8)
                        y0 = (cur.y - s_cur * new_th.y) * inv
                        ca, sa = math.cos(a0), math.sin(a0)
                        sw.start_hip_l = V3((rd.x * ca + px * sa) * r0, y0, (rd.z * ca + pz * sa) * r0)
                    else:
                        cur = sh.lerp(old_th, s_cur)
                        sw.start_hip_l = (cur - new_th * s_cur) * inv
                    sw.target_hip_l = new_th
        elif reset_start:
            sw.start_hip_l = None
            sw.target_hip_l = None
            sw.arc_t0 = 0.0

    def _lift(self, l, dur, fh, start_vel=None, end_vel=None):
        """Launch a swing arc toward `fh`, or an elevated search point if None."""
        up_p = self.probe_up()
        if fh is not None:
            target, normal, has = fh[0], fh[1], True
        else:
            target, normal, has = l.ideal_pred + up_p * (self.ride * 0.2), up_p, False
        lift = self.d["step_height"] * self.gait.lift * (0.65 if self.crouch > 0.5 else 1.0)
        if self.state == MANTLE:
            lift *= 1.6
        if self.drifting:
            if start_vel is None:
                start_vel = project_on_plane(self.vt, l.foot_normal) * 0.95
            if end_vel is None:
                end_vel = project_on_plane(self.vt, normal) * 0.95
        bow = self._swing_bow(l, l.foot, target, dur)
        l.begin_swing(target, normal, dur, lift, up_p, has, self.world, self._clearance(), self,
                      start_vel=start_vel, end_vel=end_vel, bow=bow, max_ceil_y=self.ceil_y)
        self._set_swing_arc(l, target, dur, reset_start=True)
        l.swing.ideal = l.ideal_pred.copy()
        l.cooldown = 0.05
        l.force = False
        l.pending = False

    def _can_lift(self, l, emergency=False, check_static=True):
        """True if lifting `l` preserves support count, sync, and COM margin."""
        g = self.gait
        if l.stance_t == 0.0:
            return False
        others = [x for x in self.legs if x.planted and x is not l and x.stretch <= 1.04]
        n_after = len(others)
        if self.state == GROUND and not self.biped:
            cur_str = l.mount_world.dist(l.foot) / l.reach
            over = cur_str > 0.99 or (
                (cur_str > 0.86 or l.ik.coxa_ratio > 0.72 or l.near_limit or self.speed > 18.0)
                and not l.ik.feasible(l.mount_world, self.frame, l.foot, slack_deg=0.0)
            )
            if not over and g.pair_sync and not self.trailing_support and not any(x.pair != l.pair for x in others):
                return False
            if n_after == 0 and not over and not self._may_flight() and any(x.swing is not None and x.swing.has_ground for x in self.legs):
                return False
        if emergency and (l.stretch > 1.04 or l.limit_t > 0.1):
            return True
        if self.biped:
            need = 0 if (g.min_support == 0 and self.speed > 1.0) else 1
            if emergency and self.speed > 1.0:
                need = 0
            return n_after >= need
        need = g.min_support
        if emergency:
            need = min(need, max(1, self.n // 2))
        if self.state == MANTLE:
            need = min(need, max(1, self.n // 2 - 1))
        if self.on_wall:
            need = max(need, max(3, self.n // 2))
        if self.state == GROUND and not self._may_flight() and not emergency:
            need = max(need, 1)
        if n_after < min(need, self.n - 1):
            return False
        if not emergency and g.adjacent_lock:
            for nb in self._neighbors(l):
                if not nb.planted:
                    return False
        if check_static and g.static_stable and not emergency and self.state == GROUND and not self.on_wall:
            hull = convex_hull_2d([(x.foot.x, x.foot.z) for x in others])
            if support_margin(hull, (self.pos.x, self.pos.z)) < -0.04 * self.size:
                return False
        return True

    def _on_plant(self, l):
        """Commit touchdown: verify IK plantability, cooldown, and chassis weight kick."""
        self._solve_leg(l)
        if not l.planted:
            return
        l.pending = False
        l.stance_t = 0.0
        if self.galloping_fast():
            l.cooldown = min(0.32 * self.duty * self.cycle, 0.38 * l.reach / self.speed)
        else:
            l.cooldown = max(l.cooldown, 0.16 * math.sqrt(self.size))
        up = self.probe_up()
        l.land_lead = self.support_frame().to_local(project_on_plane(l.foot - l.ideal_now, up))
        l.last_plant = (l.foot.copy(), l.foot_normal.copy())
        d = self.d
        strength = d["mass"] * (0.5 + self.speed * 0.12)
        self.emit(EV_STEP, l.foot, l.foot_normal, strength=strength)
        if d["mass"] >= 2.0 and self.state == GROUND:
            self.vel = self.vel - self.frame.up * (0.05 * d["mass"])
            if abs(self.inp.aim) < 0.05:
                lever = project_on_plane(l.foot - self.pos, self.frame.up)
                ll = lever.length()
                if ll > 0.2:
                    axis = self.frame.up.cross(lever * (1.0 / ll))
                    kick = min(0.14, 0.022 * d["mass"] * clamp(1.0 - self.speed / max(1.0, d["speeds"]["walk"]), 0.25, 1.0))
                    self.ang_vel = self.ang_vel + axis * kick
