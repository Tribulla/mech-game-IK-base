"""Mech body simulation: modes, ground/air/mantle integration, orientation, and packing.

Foothold search, obstacle clearance, and gait scheduling live in `foothold.py`,
`clearance.py`, and `scheduler.py`; shared constants/helpers live in `mech_const.py`.
"""
import math
import random
import time

from .vmath import (V3, Frame, WORLD_UP, LOCAL_FRAME, clamp, smoothstep, damp_factor,
                    project_on_plane, rotate_axis, rotate_towards, angle_between,
                    convex_hull_2d, support_margin, fit_plane)
from . import world as world_mod
from .leg import Leg
from .gaitsel import Situation, GaitSelector, GaitBlend
from .mech_const import (G, A_LAT, AIM_MAX, EV_STEP, EV_SPARK, EV_LAND, EV_BOOST,
                         EV_IMPACT, EV_SKID, EV_MANTLE, DBG_LINE, DBG_LINE_DIM,
                         DBG_INFO, DBG_HIGHLIGHT, DBG_GOOD, DBG_WARNING, DBG_ERROR,
                         DBG_FORCE, DBG_NORMAL, DBG_VELOCITY, DBG_AXIS_X, DBG_AXIS_Y,
                         DBG_AXIS_Z, GROUND, AIR, MANTLE, CROUCHJUMP, STATE_NAMES,
                         INPUT_FIELDS, Input, Mantle, _nearest_on_hull, _dist_to_segment)
from .foothold import FootholdMixin
from .clearance import ClearanceMixin
from .scheduler import SchedulerMixin


class Mech(FootholdMixin, ClearanceMixin, SchedulerMixin):
    def __init__(self, d, ground_pos, yaw, events):
        self.d = d
        self.size = d.get("size", 1.0)
        self.events = events
        self.world = world_mod.get_world()
        self.frame = Frame(V3(math.sin(yaw), 0.0, math.cos(yaw)), WORLD_UP)
        self.ride = d["ride"]
        self.pos = V3(ground_pos.x, ground_pos.y + self.ride, ground_pos.z)
        self.vel = V3()
        self.ang_vel = V3()
        self.yaw_rate = 0.0
        self.legs = [Leg(i, ld, d) for i, ld in enumerate(d["legs"])]
        self.n = len(self.legs)
        self.biped = d.get("biped", False)
        self.reach_avg = sum(l.reach for l in self.legs) / self.n
        chords = []
        for l in self.legs:
            c = 1.15 * l.reach
            if l.ik.mode == "radial" and l.ik.coxa_max < 1.5:
                rr = math.hypot(l.rest_xz[0] - l.mount_local.x, l.rest_xz[1] - l.mount_local.z)
                c = min(c, 1.85 * rr * math.sin(l.ik.coxa_max))
            chords.append(c)
        self.chord_avg = sum(chords) / self.n
        wg = d["gaits"][d["gait_by_mode"]["walk"]]
        self.v_walk = self.reach_avg / max(0.05, wg.duty * wg.cycle_min * math.sqrt(self.size))
        sg = d["gaits"][d["gait_by_mode"]["slow"]]
        self.v_slow = self.reach_avg / max(0.05, sg.duty * sg.cycle_min * math.sqrt(self.size))
        self.jump_sink_max = min(0.45, max(0.0, 1.0 - 1.06 * self._fold_ride() / self.ride))
        self.look_pitch_dn, self.look_pitch_up = self._pitch_limits()
        self.look_pitch = 0.0
        self.joints_per_leg = 1 + (1 if d["coxa"] > 0 else 0) + len(d["lengths"])
        self.inp = Input()
        self.prev = Input()
        self.mode = "walk"
        self.gait_override = None
        self.gait_name = d["gait_by_mode"]["walk"]
        self.blend = GaitBlend(d["gaits"][self.gait_name])
        self.gait = self.blend.eff
        self.situation = Situation()
        self.selector = GaitSelector(self)
        self.speed_cap = None
        self.v_desired = 0.0
        self.lateral_ratio = 0.0
        self._v_cmd = V3()
        self.drive_target = V3()
        self.drive_accel = 0.0
        self.accel_smooth = V3()
        self.calm_gait = False
        self.foot_rms = 0.0
        self.fh_count = 0
        self.fh_bad = 0
        self.duty = self.gait.duty
        for l in self.legs:
            l.offset = self.gait.offsets[l.idx]
        self.phase = 0.0
        self.cycle = 1.0
        self.t_ahead = 0.0
        self.state = GROUND
        self.state_t = 0.0
        self.crouch_toggle = False
        self.crouch = 0.0
        self.climb_enabled = d["climber"]
        self.drifting = False
        self.drift_t = 0.0
        self.drift_amt = 0.0
        self.slide_dir = V3(0.0, 0.0, 1.0)
        self.slide_cd = 0.0
        self.slide_buf = 0.0
        self.slide_lock = False
        self.unsupported_t = 0.0
        self.air_belly = False
        self.void_t = 0.0
        self.flight_t = 0.0
        self.flight_dur = 0.0
        self.flight_h = 0.0
        self.flight_y0 = 0.0
        self.flight_hold = 0.0
        self.flight_count = 0
        self.slide_vel = V3()
        self.absorb = 0.0
        self.jump_sink = 0.0
        self.braking = False
        self.balance_out = None
        self.balance_excess = 0.0
        self.balance_h = 1.0
        self.unbal_t = 0.0
        self.jump_sink_cap = 0.0
        self.jump_r0 = self.ride
        self.jump_y0 = 0.0
        self.jump_npl = 0
        self.brace = 0.0
        self.impact = None
        self.prev_npl = 0
        self.trailing_support = False
        self.detach_t = 0.0
        self.plane_n_local = None
        self.ground_n = WORLD_UP.copy()
        self.land_right_t = 0.0
        self.land_right_n = WORLD_UP.copy()
        self.support = 1.0
        self.hull = []
        self.vis_offset = V3()
        self.mantle = None
        self.on_wall = False
        self.last_accel = V3()
        self.speed = 0.0
        self.vt = V3()
        self._rng = random.Random(d["kind_id"] + 1)
        self._spark_t = 0.0
        self._ceil_t = 1.0
        self.ceil_y = None
        self._pitch_cap = 1.0
        self._pitch_strain_t = 0.0
        self._sup_t = 1.0
        self._sup_h = None
        self._sup_ha = None
        self._sup_vdir = V3(0, 0, 1)
        self._mantle_t = 0.05
        self.ik_solve_hz = 0.0
        self.probe_period = 0.05
        self.speed_scale = 1.0
        self.plane_roll_enabled = True
        self._ik_acc = 0.0
        self._prof = None
        hx, hy, hz = d["body_half"]
        if d["climber"]:
            self.col_r = hy * 1.15
            self.col_offs = (0.0,)
            self.col_lat = 0.0
        else:
            self.col_r = min(hx, hz, hy * 1.3) * 0.95
            off = max(0.0, hz - self.col_r)
            n = max(1, int(math.ceil(off / max(self.col_r, 1e-3))))
            self.col_offs = tuple(off * (1.0 - 2.0 * i / n) for i in range(n + 1)) if off > 0.05 else (0.0,)
            self.col_lat = max(0.0, hx - self.col_r)
        self._place_feet_initial()

    # ------------------------------------------------------------------ utils
    @property
    def climbing_mode(self):
        return self.d["climber"] and self.climb_enabled

    def probe_up(self):
        return self.frame.up if self.climbing_mode else WORLD_UP

    def support_frame(self):
        """Frame used for foot rest positions and the support spring."""
        if self.climbing_mode:
            return self.frame
        f = self.heading_fwd()
        fr = Frame.__new__(Frame)
        fr.fwd = f
        fr.up = WORLD_UP
        fr.right = V3(-f.z, 0.0, f.x)
        return fr

    def heading_fwd(self):
        """Level forward direction, falling back to right when steeply pitched."""
        f = self.frame.fwd
        if abs(f.y) > 0.7:
            r = self.frame.right
            fx, fz = r.z, -r.x
        else:
            fx, fz = f.x, f.z
        l2 = fx * fx + fz * fz
        if l2 < 1e-18:
            return V3(0.0, 0.0, 1.0)
        inv = 1.0 / math.sqrt(l2)
        return V3(fx * inv, 0.0, fz * inv)

    def emit(self, kind, p, n=None, strength=1.0, extra=None):
        n = n or WORLD_UP
        e = extra or V3()
        self.events.append((kind, p.x, p.y, p.z, n.x, n.y, n.z, strength, e.x, e.y, e.z))

    def ride_eff(self):
        slide_drop = min(0.26, 0.72 * self.jump_sink_max) * self.drift_amt
        r = self.ride * (1.0 - max(0.38 * self.crouch, slide_drop))
        if self.mode == "slow":
            r *= 0.9
        if self.state == CROUCHJUMP:
            r *= 1.0 - self.jump_sink
        r *= (1.0 - self.absorb) * (1.0 - 0.15 * self.brace)
        return r

    def _fold_ride(self):
        """Lowest ride height at which every leg still reaches its rest foot within limits."""
        lowest = self.ride
        for k in range(1, 24):
            r = self.ride * (1.0 - 0.025 * k)
            for l in self.legs:
                l.ik_probe.solve(l.mount_local, LOCAL_FRAME, l.rest_local(r))
                if l.ik_probe.err > 0.01:
                    return lowest
            lowest = r
        return lowest

    def _pitch_limits(self):
        """Maximum down/up hull look pitch (rad) relative to the ground plane."""
        cap = float(self.d.get("look_tilt", 25.0))
        _, hy, hz = self.d["body_half"]
        level = Frame(V3(0.0, 0.0, 1.0), WORLD_UP)
        out = []
        for sgn in (-1.0, 1.0):
            best = 0.0
            for deg in range(1, int(math.ceil(cap)) + 1):
                a = min(float(deg), cap)
                rad = math.radians(a * sgn)
                up = rotate_axis(WORLD_UP, level.right, rad)
                fr = Frame(V3(0.0, 0.0, 1.0), up)
                ride = max(self.ride, hz * abs(fr.fwd.dot(WORLD_UP)) + hy * abs(fr.up.dot(WORLD_UP)) + 0.06)
                ok = True
                for r_test in (ride, ride * 0.92):
                    for l in self.legs:
                        foot_l = fr.to_local(level.to_world(l.rest_local(r_test)))
                        if ((foot_l - l.mount_local).length() > 0.91 * l.reach
                                or not l.ik_probe.feasible(l.mount_local, LOCAL_FRAME, foot_l, slack_deg=6.0)):
                            ok = False
                            break
                    if not ok:
                        break
                if not ok:
                    break
                best = a
            out.append(math.radians(best))
        return out[0], out[1]

    def _sink_room(self, planted):
        """Fraction of ride height the hull can sink before a planted leg runs out of fold."""
        r0 = self.jump_r0
        fr, pos = self.frame, self.pos
        dropped = max(0.0, self.jump_y0 - pos.dot(self.probe_up()))
        step = 0.05 * self.ride
        room = max(0.0, self.jump_sink_max * r0 - dropped)
        for l in planted:
            fl = fr.to_local(l.foot - pos)
            d = 0.0
            while d + step <= room:
                l.ik_probe.solve(l.mount_local, LOCAL_FRAME, fl + V3(0.0, d + step, 0.0))
                if l.ik_probe.err > 0.01:
                    break
                d += step
            room = min(room, d)
        return (dropped + 0.85 * room) / r0

    def _crouch_time(self):
        return (0.12 + 0.01 * self.d["jump_speed"]) * math.sqrt(self.size)

    def target_speed(self):
        return self.d["speeds"][self.mode] * self.speed_scale

    def crouch_cap(self):
        return 0.5 * self.d["speeds"]["slow"]

    def _neighbors(self, l):
        out = []
        for pair, side in ((l.pair - 1, l.idx % 2), (l.pair + 1, l.idx % 2), (l.pair, 1 - l.idx % 2)):
            j = pair * 2 + side
            if 0 <= pair and 0 <= j < self.n:
                out.append(self.legs[j])
        return out

    def _place_feet_initial(self):
        fr = self.frame
        for l in self.legs:
            l.mount_world = self.pos + fr.to_world(l.mount_local)
            l.ideal_now = self.pos + fr.to_world(l.rest_local(self.ride))
            l.ideal_pred = l.ideal_now.copy()
            fh = self._find_foothold(l, l.ideal_now)
            if fh:
                l.foot, l.foot_normal = fh
                l.planted = True
            else:
                l.foot = l.ideal_now.copy()
                l.planted = False
                l.searching = True
            self._solve_leg(l)

    # ------------------------------------------------------------------ main
    def update(self, dt):
        """Advance body motion, state machine, chassis tilt, and leg IK by dt."""
        if not math.isfinite(dt) or dt <= 0.0:
            return
        prof = self._prof
        perf = time.perf_counter if prof is not None else None
        t0 = perf() if perf is not None else 0.0
        self.world = world_mod.get_world()
        # Measure low lintels overhead and half a body-length ahead so leg
        # swing arcs cap lift height underneath overhangs.
        if not self.climbing_mode:
            self._ceil_t += dt
            p_probe = self.probe_period
            if self._ceil_t >= p_probe - 1e-6:
                self._ceil_t %= p_probe
                c_vals = []
                hc = self.world.raycast(self.pos, WORLD_UP, 4.0)
                if hc is not None:
                    c_vals.append(hc.point.y)
                look_dir = project_on_plane(self.vt, WORLD_UP).normalized(self.heading_fwd()) if self.speed > 0.5 else self.heading_fwd()
                ha = self.world.raycast(self.pos + look_dir * (self.d["body_half"][2] + 1.2), WORLD_UP, 4.0)
                if ha is not None:
                    c_vals.append(ha.point.y)
                self.ceil_y = min(c_vals) if c_vals else None
        else:
            self.ceil_y = None
        self._sup_t += dt
        self._mantle_t += dt
        inp, prev = self.inp, self.prev
        # Save carried foot offsets in chassis space before updating hull motion
        # so unplanted legs move naturally alongside the torso.
        for l in self.legs:
            l.carry = None if l.planted else self.frame.to_local(l.foot - self.pos)

        def edge(name):
            return getattr(inp, name) > 0.5 >= getattr(prev, name)

        if edge("crouch"):
            self.crouch_toggle = not self.crouch_toggle
        if edge("climb") and self.d["climber"]:
            self.climb_enabled = not self.climb_enabled
        if edge("gait"):
            self._cycle_gait_override()
        if edge("impact"):
            self._impact()
        jump = edge("jump")

        self.state_t += dt
        self.absorb = max(0.0, self.absorb - dt * (0.9 + 1.5 * self.absorb))
        self.brace = max(self.brace * math.exp(-1.4 * dt), clamp(self.slide_vel.length() / 5.0, 0.0, 1.0))
        self.crouch += ((1.0 if self.crouch_toggle else 0.0) - self.crouch) * damp_factor(6.0, dt)
        self._compute_drive_intent()
        self._select_mode(dt)
        if perf is not None:
            t1 = perf()
            prof[0] += t1 - t0
            t0 = t1

        if self.state == MANTLE:
            self._update_mantle(dt)
        elif self.state == AIR:
            self._update_air(dt)
        else:
            self._update_ground(dt, jump)

        self._update_orientation(dt)
        self._update_vis_offset(dt)
        if perf is not None:
            t1 = perf()
            prof[1] += t1 - t0
        self._update_legs(dt)
        prev.copy_from(inp)

    # ------------------------------------------------------------------ modes
    def _compute_drive_intent(self):
        """Map raw stick/key inputs into desired planar velocity and lateral tilt."""
        inp = self.inp
        if inp.run > 0.5:
            self.mode = "run"
        elif inp.slow > 0.5:
            self.mode = "slow"
        else:
            self.mode = "walk"
        heading = self.support_frame()
        mv = heading.fwd * inp.fwd + heading.right * (inp.strafe * 0.75)
        if mv.length_sq() > 1.0:
            mv = mv.normalized()
        v_cmd = mv * self.target_speed()
        if self.crouch > 0.5 and v_cmd.length() > self.crouch_cap():
            v_cmd = v_cmd.normalized() * self.crouch_cap()
        self._v_cmd = v_cmd
        if v_cmd.length_sq() > 0.01:
            self.lateral_ratio = abs(v_cmd.dot(heading.right)) / max(abs(v_cmd.dot(heading.fwd)), 0.1)
        else:
            self.lateral_ratio = 0.0
        self.v_desired = v_cmd.length()

    def _select_mode(self, dt):
        """Pick the active gait from speed, crouch/climb state, and terrain lookahead."""
        d = self.d
        if self.state == GROUND:
            self.situation.update(self, dt)
            name, reason, cap = self.selector.select(self, dt)
            self.speed_cap = cap
            if name != self.gait_name:
                self.gait_name = name
                dst_g = d["gaits"][name]
                dst_T = clamp(
                    dst_g.stride * self.size / max(self.v_desired, self.speed, 1e-3),
                    dst_g.cycle_min * math.sqrt(self.size),
                    dst_g.cycle_max * math.sqrt(self.size),
                )
                self.blend.start(dst_g, clamp(dst_T, 0.28, 0.50))
        else:
            self.selector.low_t = 0.0
            self.selector._prev_v = self.speed
            self.selector._accel_v = 0.0
        self.gait = self.blend.update(dt)
        for l in self.legs:
            l.offset = self.gait.offsets[l.idx]
        k = damp_factor(5.0, dt)
        b = self.blend
        duty = self._duty_for(b.dst)
        if b.blending:
            duty = self._duty_for(b.src) + (duty - self._duty_for(b.src)) * b.s
            hold = self.selector.hold_v
            if hold is not None and self.state == GROUND:
                hold *= 1.0 + 0.8 * b.s
                self.speed_cap = hold if self.speed_cap is None else min(self.speed_cap, hold)
        self.duty += (duty - self.duty) * k
        b.fit_support(self.duty)

    def _duty_for(self, g):
        """Duty factor gait g runs at the current speed."""
        duty = g.duty
        if g.duty_fast is not None:
            d = self.d
            v0 = 0.75 * self.v_walk
            f = clamp((self.speed - v0) / max(1.0, d["speeds"]["walk"] - v0), 0.0, 1.0)
            duty = duty + (g.duty_fast - duty) * f
            if not self._fast_hopper() or self.braking or self.calm_gait:
                duty = max(duty, 0.5)
            else:
                b = d.get("bound", 0.0) * g.leap
                vl = self.reach_avg / max(0.05, max(duty, 0.25) * g.cycle_min * math.sqrt(self.size))
                k = clamp((self.speed - 0.3 * vl) / (0.7 * vl), 0.0, 1.0)
                if b * k <= 0.016:
                    duty = max(duty, 0.5)
        return duty

    def gait_label(self):
        lab = self.blend.label()
        r = self.selector.reason
        if r and r not in ("walking", "fast", "galloping", "careful", "forced"):
            lab += " (%s)" % r
        return lab

    def _cycle_gait_override(self):
        names = [None] + list(self.d["gaits"].keys())
        i = names.index(self.gait_override) if self.gait_override in names else 0
        self.gait_override = names[(i + 1) % len(names)]

    def _impact(self):
        a = self._rng.uniform(0, math.tau)
        axis = V3(math.cos(a), 0.0, math.sin(a))
        self.ang_vel = self.ang_vel + axis * (2.4 / math.sqrt(self.d["mass"]))
        push = V3(math.sin(a), 0.0, -math.cos(a)) * (4.0 / math.sqrt(self.d["mass"]))
        self.push(push, torque=False)
        self.brace = 1.0
        self.emit(EV_IMPACT, self.pos, strength=1.0)

    def push(self, dv, torque=True):
        """External shove / knockback: hull slides and feet skid under friction."""
        if self.state == GROUND:
            self.slide_vel = self.slide_vel + V3(dv.x, 0.0, dv.z)
            self.vel = self.vel + dv
            if torque:
                dv_h = V3(dv.x, 0.0, dv.z)
                dhl = dv_h.length()
                if dhl > 0.2:
                    axis = WORLD_UP.cross(dv_h) * (1.0 / dhl)
                    self.ang_vel = self.ang_vel + axis * clamp(dhl * 0.45 / math.sqrt(self.d["mass"]), 0.0, 2.6)
        else:
            self.vel = self.vel + dv

    # ------------------------------------------------------------------ ground
    def _update_ground(self, dt, jump):
        d, fr = self.d, self.frame
        up = self.probe_up()
        inp = self.inp
        planted = [l for l in self.legs if l.planted]
        npl = len(planted)

        if jump and d["jump_speed"] > 0 and self.state == GROUND and npl > 0:
            if self.drifting:
                self.drifting = False
                self.slide_cd = max(self.slide_cd, 0.25)
            self.state, self.state_t = CROUCHJUMP, 0.0
            self.jump_sink = 0.0
            self.jump_r0 = max(0.1, self.ride_eff())
            self.jump_y0 = self.pos.dot(up)
            self.jump_npl = npl
            self.jump_sink_cap = self._sink_room(planted)
        if self.state == CROUCHJUMP:
            if self.state_t > self._crouch_time():
                self._launch()
                return
            if npl > self.jump_npl:
                self.jump_sink_cap = min(self.jump_sink_cap, self._sink_room(planted))
            self.jump_npl = npl
            want = self.jump_sink_cap * smoothstep(self.state_t / max(0.05, self._crouch_time()))
            if any(l.ik.err > 0.02 for l in planted):
                self.jump_sink = max(0.0, self.jump_sink - 0.6 * dt)
            elif want > self.jump_sink:
                self.jump_sink = want
        if (self.state == GROUND and not self.climbing_mode
                and self.ground_n.y >= 0.86 and self.speed < max(16.0, 1.15 * self.v_walk)
                and (d["mantle_max"] > 0 or d.get("wire_max", 0.0) > 0)
                and inp.fwd > 0.05 and self._mantle_t >= self.probe_period - 1e-6):
            self._mantle_t %= self.probe_period
            if self._try_mantle():
                return

        bal = self._balance(dt, planted)
        v_des = self._v_cmd
        if self.speed_cap is not None and v_des.length() > self.speed_cap:
            v_des = v_des.normalized() * self.speed_cap
        self.v_desired = v_des.length()
        self.drive_target = v_des
        self.drive_accel = 0.0
        v_des_len = v_des.length()
        self.braking = (self.speed > 1.0 and (
            v_des_len < 0.82 * self.speed
            or (v_des_len > 0.1 and v_des.dot(self.vt) < -0.1 * self.speed * v_des_len)
        ))
        v_up = self.vel.dot(up)
        slide = project_on_plane(self.slide_vel, up)
        vt = self.vel - up * v_up - slide
        vt = self._update_drift(dt, vt)
        vt_raw = vt
        spin_scale = 0.62 + 0.38 * smoothstep((max(self.speed, self.v_desired) - 1.0) / 3.0)
        yr_des = (inp.turn * d["turn_rate"] * spin_scale
                  * (0.75 if self.mode == "slow" else 1.0))
        normal_lim = min(d["turn_rate"], A_LAT / max(self.speed, 1e-3))
        if self.drifting or self.drift_amt > 0.02:
            slide_w = 1.0 if self.drifting else self.drift_amt
            slide_lim = min(0.55 * d["turn_rate"], A_LAT / max(self.speed, 4.0))
            lim = normal_lim + (slide_lim - normal_lim) * slide_w
            k_yaw = 24.0 - 10.0 * slide_w
        else:
            lim = normal_lim
            k_yaw = 24.0
        if self.flight_t > 0.0:
            k_yaw = min(k_yaw, 16.0)
        yr_des = clamp(yr_des, -lim, lim)
        if (not self.drifting and not self.galloping_fast(blend=True)
                and self.speed < 0.45 * self.v_walk and bal >= 1.0 and abs(yr_des) > 0.1):
            yaw_strain = 0.0
            for l in planted:
                if l.ik.mode == "radial" and l.ik.coxa_ratio > 0.96 and l.ik.yaw_deg * yr_des < 0.0:
                    yaw_strain = max(yaw_strain, (l.ik.coxa_ratio - 0.96) * 20.0)
                if l.stretch > 0.975:
                    yaw_strain = max(yaw_strain, (l.stretch - 0.975) * 20.0)
            if yaw_strain > 0.0:
                yr_des *= max(0.45, 1.0 - min(0.55, yaw_strain))
        new_yr = self.yaw_rate + (yr_des - self.yaw_rate) * damp_factor(k_yaw, dt)
        yr_mid = 0.5 * (self.yaw_rate + new_yr)
        has_ground_swing = any(l.swing is not None and l.swing.has_ground for l in self.legs)
        traction = 1.0 if npl > 0 else (0.3 if (self.flight_t > 0.0 or has_ground_swing) else 0.0)
        if self.drifting:
            heading = project_on_plane(fr.fwd, up).normalized(fr.fwd)
            spd = vt.length()
            vdir = vt.normalized(self.slide_dir)
            steer_dir = project_on_plane(v_des, up).normalized(heading) if v_des_len > 0.2 else heading
            carve = 0.55 if v_des_len > 0.2 else 0.25
            vdir = rotate_towards(vdir, steer_dir, carve * dt)
            self.slide_dir = vdir
            slope_down = vdir.dot(self.ground_n)
            base_drag = max(1.8, 0.06 * d["speeds"]["run"]) + 0.08 * spd
            slope_acc = G * clamp(slope_down, -0.55, 0.55) * 2.2
            brake_extra = max(8.0, 0.25 * d["speeds"]["run"]) if (v_des_len > 0.2 and steer_dir.dot(vdir) < -0.3) else 0.0
            spd = max(0.0, spd + (slope_acc - base_drag - brake_extra) * dt)
            spd = min(spd, d["speeds"]["run"] * (1.12 if slope_down > 0.12 else 1.08))
            vt_new = vdir * spd
        else:
            fast = self.mode == "run" or self.speed > 0.9 * min(self.v_walk, self.v_slow if self.mode == "slow" else 1e9)
            acc = (d.get("accel_run", d["accel"]) if fast else d["accel"]) * traction
            rotate_vt = False
            if self.speed > 0.5 and abs(yr_mid) > 1e-4:
                if self.flight_t > 0.0 or self.galloping_fast():
                    rotate_vt = True
                elif v_des_len > 0.5 and vt.dot(v_des) >= -0.25 * self.speed * v_des_len:
                    rotate_vt = True
                if rotate_vt and v_des_len > 0.5:
                    des_yaw = math.atan2(v_des.cross(vt).dot(up), vt.dot(v_des))
                    if des_yaw * yr_mid < -0.03 * abs(yr_mid):
                        rotate_vt = False
            if rotate_vt:
                vt = rotate_axis(vt, up, -yr_mid * dt)
            dv = v_des - vt
            max_dv = acc * dt * (1.6 if dv.dot(vt) < 0 else 1.0)
            self.drive_accel = max_dv / dt
            dl = dv.length()
            if dl > max_dv:
                dv = dv * (max_dv / dl)
            vt_new = vt + dv

        searching = [l for l in self.legs if l.searching or l.void_mark > 0.0]
        if searching and npl > 0 and not self.on_wall and not self.drifting:
            self.void_t += dt
        else:
            self.void_t = max(0.0, self.void_t - 0.5 * dt)
        narrow_ahead = False
        narrow_void = None
        if self.void_t > 0.30 and searching and npl > 0 and not self.on_wall and not self.drifting:
            vs = V3()
            for l in searching:
                vs = vs + project_on_plane(l.ideal_now - self.pos, up)
            hf = project_on_plane(fr.fwd, up).normalized(fr.fwd)
            along = vs.dot(hf)
            void = hf * (1.0 if along >= 0.0 else -1.0)
            push = v_des.dot(void) / max(v_des.length(), 1e-6) if v_des.length() > 0.05 else 0.0
            center_ground = False
            wide_ground = False
            if abs(along) >= 0.5 * vs.length():
                lat_check = max(0.52, max(abs(l.mount_local.x) for l in self.legs) * 0.72)
                right_v = up.cross(void)
                reach_d = self.ride + d["step_max"] + 2.0
                for dist_a in (d["body_half"][2] + 0.6, d["body_half"][2] + 1.5):
                    pa = self.pos + void * dist_a
                    g = self.world.raycast(pa + up * 1.0, up * -1.0, reach_d)
                    if g is not None and g.normal.dot(up) > 0.64:
                        center_ground = True
                        if lat_check > 0.15:
                            gl = self.world.raycast(pa - right_v * lat_check + up * 1.0, up * -1.0, reach_d)
                            gr = self.world.raycast(pa + right_v * lat_check + up * 1.0, up * -1.0, reach_d)
                            if gl is not None and gl.normal.dot(up) > 0.64 and gr is not None and gr.normal.dot(up) > 0.64:
                                wide_ground = True
                        else:
                            wide_ground = True
            narrow_ahead = center_ground and not wide_ground
            if narrow_ahead:
                narrow_void = void
                c = vt_new.dot(void)
                if c > 0.0:
                    vt_new = vt_new - void * c
                if bal < 1.0 or self.balance_excess > 0.0 or len(searching) >= max(2, self.n // 2):
                    back = -vt_new.dot(void)
                    want = 1.0 * min(1.0, (self.void_t - 0.30) * 3.0)
                    if back < want:
                        vt_new = vt_new - void * (want - back)
            elif self.void_t > 0.4:
                ground_ahead = wide_ground
                if abs(along) < 0.5 * vs.length() or ground_ahead:
                    pass
                elif push > 0.3:
                    if self.void_t > 0.9:
                        self.vel = vt_new + void * 1.5 + up * v_up
                        self._enter_air()
                        return
                elif bal < 1.0 and self.balance_excess > 0.15 and self.void_t > 1.5:
                    self.vel = vt_new + void * 1.5 + up * v_up
                    self._enter_air()
                    return
                elif bal >= 1.0:
                    c = vt_new.dot(void)
                    if c > 0.0:
                        vt_new = vt_new - void * c
                    back = -vt_new.dot(void)
                    want = 0.8 * min(1.0, (self.void_t - 0.4) * 3.0)
                    if back < want:
                        vt_new = vt_new - void * (want - back)

        if not self.drifting and not self.galloping_fast(blend=True) and bal >= 1.0:
            for l in planted:
                if l.stretch > 0.96:
                    away = project_on_plane(l.mount_world - l.foot, up).normalized()
                    c = vt_new.dot(away)
                    if c > 0:
                        vt_new = vt_new - away * (c * min(1.0, (l.stretch - 0.96) * 25.0))
        self.last_accel = (vt_new - vt) * (1.0 / dt)
        self.accel_smooth = self.accel_smooth.lerp(self.last_accel, damp_factor(14.0, dt))

        self.yaw_rate = new_yr
        fr.yaw(yr_mid * dt)

        sl = slide.length()
        if sl > 1e-4:
            mu = 0.55 if npl > 0 else 0.12
            slide = slide * (max(0.0, sl - mu * G * dt) / sl)
        self.slide_vel = slide

        if (self.flight_t <= 0.0 and self.gait.pair_sync and self.gait.min_support == 0
                and not self.braking and self.speed > 0.5 * self.v_leg()
                and self.trailing_support and not self.on_wall and not self.drifting):
            lead_pair = (self.legs[-1] if vt_new.dot(self.heading_fwd()) < 0.0 else self.legs[0]).pair
            if not any(l.planted and l.pair == lead_pair for l in self.legs):
                v_up += 0.45 * G * dt

        if self._update_flight(dt, npl, vt_new, up, v_up):
            self.pos = self.pos + self.vel * dt + up * (0.5 * G * dt * dt)
            self._collide()
            return

        if npl > 0 or any(l.swing is not None and l.swing.has_ground for l in self.legs):
            self.unsupported_t = 0.0
        else:
            self.unsupported_t += dt
        supported = npl > 0 or (self.unsupported_t < 0.12 and self._may_flight())
        err = self._support_error()
        grav = V3()
        if supported and err is not None:
            f = 3.0 / math.sqrt(self.size)
            w = math.tau * f
            v_up += (w * w * err - 2.0 * 0.75 * w * v_up) * dt * bal
            grav = WORLD_UP * (-G * dt * (1.0 - bal))
        else:
            grav = WORLD_UP * (-G * dt)
        self.vel = vt_new + self.slide_vel + up * v_up + grav
        if self.balance_out is not None and self.balance_excess > 0.0:
            if not (narrow_ahead and self.balance_out.dot(narrow_void) > 0.2):
                h = max(0.3, self.balance_h)
                a = min(G, G * self.balance_excess / h)
                v_cap = min(self.v_walk, 6.0)
                cur_b = self.vel.dot(self.balance_out)
                dv_b = min(a * dt, max(0.0, v_cap - cur_b))
                self.vel = self.vel + self.balance_out * dv_b
                if dv_b > 0.0:
                    self.ang_vel = self.ang_vel + WORLD_UP.cross(self.balance_out) * (dv_b / h)
            for l in planted:
                if l.stretch > 1.0:
                    l.planted = False
                    l.searching = True
                    l.search_cd = 0.15
            if (bal <= 0.0 or not any(l.planted for l in self.legs)) and not (narrow_ahead and npl > 0):
                unbal_lim = 0.45 if self.balance_excess > 0.65 * self.size else 2.5
                if not any(l.planted and l.stretch <= 1.0 for l in self.legs) or self.unbal_t > unbal_lim:
                    self.vel = self.vel + self.balance_out * 1.0
                    self._enter_air()

        if not supported and self.unsupported_t > 0.12:
            self._enter_air()
        if self.on_wall:
            hold = max(2, self.n // 2 - 1)
            if npl < hold or not self.climbing_mode:
                self.detach_t += dt
                if self.detach_t > 0.25:
                    self._enter_air()
            else:
                self.detach_t = 0.0

        vt_step = (vt_raw + vt_new) * 0.5 if not self.drifting else vt_new
        self.pos = self.pos + (self.vel - vt_new + vt_step) * dt
        self._collide()

    def _support_error(self):
        pts, wts = [], []
        for l in self.legs:
            if l.planted:
                pts.append(l.foot)
                wts.append(1.0)
            elif l.swing is not None and l.swing.has_ground:
                pts.append(l.swing.target)
                wts.append(0.35)
        if not pts:
            self.plane_n_local = None
            return None
        fr = self.support_frame()
        local = [fr.to_local(p - self.pos) for p in pts]
        c, n = fit_plane(local, wts)
        self.plane_n_local = n
        if n is not None:
            ws = sum(wts) or 1.0
            self.foot_rms = math.sqrt(sum(w * ((p - c).dot(n)) ** 2 for p, w in zip(local, wts)) / ws)
        else:
            self.foot_rms = 0.0
        if n is None or n.y < 0.3:
            plane_y = c.y
        else:
            plane_y = c.y + (n.x * c.x + n.z * c.z) / n.y
        err = plane_y + self.ride_eff()
        if not self.climbing_mode and (abs(self.inp.aim) > 1e-3 or abs(self.look_pitch) > 1e-3):
            hx, hy, hz = self.d["body_half"]
            g_up = fr.to_world(n).normalized(WORLD_UP) if n is not None else WORLD_UP
            drop = (hz * abs(self.frame.fwd.dot(g_up))
                    + hy * abs(self.frame.up.dot(g_up))
                    + hx * abs(self.frame.right.dot(g_up)))
            err = max(err, plane_y + (drop + 0.06) / max(0.5, g_up.y))
        up = fr.up
        want = self.ride_eff() * 0.8
        refresh_sup = self._sup_t >= self.probe_period - 1e-6
        if refresh_sup:
            self._sup_t %= self.probe_period
            self._sup_h = self.world.raycast(self.pos, up * -1.0, want)
            self._sup_ha = None
            if not self.climbing_mode and self.state == GROUND and self.speed > 0.5:
                v_dir = project_on_plane(self.vt, up).normalized()
                if v_dir.length_sq() > 0.5:
                    lead = clamp(self.speed * 0.10, 0.35, self.d["body_half"][2] + 0.35 * self.reach_avg)
                    sm = self.d["step_max"]
                    gn = self.ground_n
                    slope_dz = -((v_dir.x * lead) * gn.x + (v_dir.z * lead) * gn.z) / max(0.5, gn.y)
                    probe_top = sm + 0.5 + max(0.0, slope_dz)
                    ahead = self.pos + v_dir * lead
                    h_a = self.world.raycast(ahead + up * probe_top, up * -1.0, probe_top + self.ride_eff() + 0.5)
                    if h_a is not None and h_a.normal.dot(up) > 0.64:
                        self._sup_ha = (h_a, slope_dz)
        if self._sup_h is not None:
            err = max(err, want - self._sup_h.t)
        if self._sup_ha is not None and not self.climbing_mode and self.state == GROUND and self.speed > 0.5:
            h_a, slope_dz = self._sup_ha
            gn = self.ground_n
            plane_ref = (self.pos.dot(up) - self.ride_eff()) + slope_dz
            rise = h_a.point.dot(up) - plane_ref
            if ((h_a.normal.dot(gn) < 0.985 and 0.04 < rise <= self.d["step_max"] + 0.65)
                    or (h_a.normal.y >= 0.98 and 0.25 < rise <= self.d["step_max"] + 0.05)):
                ahead_err = (h_a.point.dot(up) + 0.88 * self.ride_eff()) - self.pos.dot(up)
                if ahead_err > err:
                    w_a = clamp((self.speed - 0.5) / 4.0, 0.0, 0.65)
                    err = err + (ahead_err - err) * w_a
        return err

    def _balance(self, dt, planted):
        """Evaluate centre of mass against the foot support polygon."""
        self.balance_out = None
        self.balance_excess = 0.0
        in_fast_gallop = self.galloping_fast(blend=True) and (
            self.flight_t > 0.0 or not any(l.searching or l.void_mark > 0.0 for l in self.legs)
        )
        if (self.biped or self.on_wall or self.climbing_mode or self.drifting or self.state != GROUND
                or self.flight_t > 0.0 or in_fast_gallop or self.wire_active()):
            self.support, self.hull = 1.0, []
            self.unbal_t = 0.0
            return 1.0
        pts = [(l.foot.x, l.foot.z) for l in planted]
        ys = [l.foot.y for l in planted]
        for l in self.legs:
            sw = l.swing
            if sw is not None and sw.has_ground and sw.t > 0.6:
                pts.append((sw.target.x, sw.target.z))
                ys.append(sw.target.y)
        if not pts:
            self.support, self.hull = 1.0, []
            if not any(l.searching or l.void_mark > 0.0 for l in self.legs):
                self.unbal_t = 0.0
            return 1.0
        hull = convex_hull_2d(pts)
        com = (self.pos.x, self.pos.z)
        m = support_margin(hull, com)
        self.support, self.hull = m, hull
        excess = -m - 0.15 * self.size
        wall_blocked = False
        if excess > 0.0:
            nx, nz = _nearest_on_hull(hull, com)
            out = V3(com[0] - nx, 0.0, com[1] - nz).normalized(project_on_plane(self.frame.fwd, WORLD_UP))
            edge = any((l.searching or l.void_mark > 0.0) and (l.ideal_now - self.pos).dot(out) > 0.0
                       for l in self.legs)
            half_out = (abs(out.dot(self.frame.right)) * self.d["body_half"][0]
                        + abs(out.dot(self.frame.fwd)) * self.d["body_half"][2])
            wall_d = half_out + 0.45
            for y_off in (0.0, -0.45 * self.ride):
                hw = self.world.raycast(self.pos + WORLD_UP * y_off, out, wall_d)
                if hw is not None and abs(hw.normal.dot(WORLD_UP)) < 0.55:
                    wall_blocked = True
                    break
        if excess <= 0.0 or wall_blocked or not ((self.speed < 1.0 and abs(self.yaw_rate) < 0.5) or edge):
            self.unbal_t = max(0.0, self.unbal_t - 2.0 * dt)
            return 1.0
        self.unbal_t += dt
        if self.unbal_t < 0.2:
            return 1.0
        self.balance_out = out
        excess -= 0.35
        if excess <= 0.0:
            return 1.0
        self.balance_excess = excess
        self.balance_h = self.pos.y - sum(ys) / len(ys)
        return clamp(1.0 - excess / (0.6 * self.ride), 0.0, 1.0)

    def _update_drift(self, dt, vt):
        """Latch a brief forward burst slide above minimum speed, locking out
        re-triggers while held so slides feel deliberate rather than icy.
        """
        spd = vt.length()
        if self.slide_cd > 0.0:
            self.slide_cd = max(0.0, self.slide_cd - dt)
        want = self.inp.drift > 0.5 and self.state == GROUND and not self.on_wall
        if not want:
            self.slide_lock = False
        pressed = want and not (self.prev.drift > 0.5)
        if pressed and not self.drifting:
            self.slide_buf = 0.22
        elif want and not self.drifting and not self.slide_lock:
            self.slide_buf = max(self.slide_buf, dt)
        else:
            self.slide_buf = max(0.0, self.slide_buf - dt)

        start_speed = min(3.0, 0.25 * self.d["speeds"]["run"])
        if (not self.drifting and self.slide_buf > 0.0 and not self.slide_lock
                and self.slide_cd <= 0.0 and self.state == GROUND and not self.on_wall
                and spd > start_speed and self.flight_t <= 0.08):
            self.drifting = True
            self.drift_t = 0.0
            self.slide_buf = 0.0
            self.flight_t = 0.0
            self.flight_hold = 0.0
            heading = project_on_plane(self.frame.fwd, self.probe_up()).normalized(V3(0.0, 0.0, 1.0))
            self.slide_dir = vt.normalized(heading)
            kick = (min(4.0, 0.14 * self.d["speeds"]["run"])
                    * clamp(spd / max(1.0, 0.5 * self.d["speeds"]["run"]), 0.35, 1.0))
            new_spd = min(spd + kick, self.d["speeds"]["run"] * 1.08)
            vt = self.slide_dir * new_spd
            spd = new_spd
        elif self.drifting:
            self.drift_t += dt
            slope_down = self.slide_dir.dot(self.ground_n)
            min_dur = 0.22
            max_dur = 0.95 + 0.55 * clamp(slope_down / 0.35, 0.0, 1.0)
            exit_spd = max(2.0, 0.12 * self.d["speeds"]["run"])
            if (not want and self.drift_t >= min_dur) or self.drift_t >= max_dur or spd < exit_spd:
                self.drifting = False
                self.slide_cd = 0.35
                if want:
                    self.slide_lock = True
        rate = 14.0 if self.drifting else 9.0
        self.drift_amt += ((1.0 if self.drifting else 0.0) - self.drift_amt) * damp_factor(rate, dt)
        return vt

    def _collide(self):
        fr = self.frame
        belly_hit = False
        for off in self.col_offs:
            c = self.pos + fr.fwd * off
            c2, n = self.world.sphere_push(c, self.col_r)
            if n is not None:
                disp = c2 - c
                dlen = disp.length()
                if n.y > 0.65 or (dlen > 1e-5 and disp.y > 0.65 * dlen):
                    belly_hit = True
                self.pos = self.pos + disp
                if dlen > 1e-5:
                    dn = disp * (1.0 / dlen)
                    vd = self.vel.dot(dn)
                    if vd < 0:
                        self.vel = self.vel - dn * vd
                vn = self.vel.dot(n)
                if vn < 0:
                    self.vel = self.vel - n * vn
        if self.col_lat > 0.03:
            lat_offs = (self.col_offs[0], self.col_offs[-1]) if len(self.col_offs) > 1 else self.col_offs
            for off in lat_offs:
                for sgn in (-1.0, 1.0):
                    c = self.pos + fr.fwd * off + fr.right * (sgn * self.col_lat)
                    c2, n = self.world.sphere_push(c, self.col_r)
                    if n is not None:
                        dx, dy, dz = c2.x - c.x, c2.y - c.y, c2.z - c.z
                        hl = math.hypot(dx, dz)
                        if hl > 1e-5 and (abs(n.y) < 0.55 or hl > abs(dy) * 1.15):
                            disp = V3(dx, 0.0, dz)
                            self.pos = self.pos + disp
                            dn = disp * (1.0 / hl)
                            vd = self.vel.dot(dn)
                            if vd < 0:
                                self.vel = self.vel - dn * vd
        return belly_hit

    # ------------------------------------------------------------------ jump / air
    def _launch(self):
        d, fr = self.d, self.frame
        jdir = fr.up if self.on_wall else WORLD_UP
        vj = self.vel - jdir * min(0.0, self.vel.dot(jdir))
        self.vel = vj + jdir * d["jump_speed"] + fr.fwd * (max(0.0, self.inp.fwd) * 1.5)
        pushing = [l for l in self.legs if l.planted]
        self._enter_air()
        for l in pushing:
            l.push = True
        self.emit(EV_LAND, self.pos - fr.up * self.ride, fr.up, strength=d["mass"] * 2.0)

    def _enter_air(self):
        self.state, self.state_t = AIR, 0.0
        self.slide_vel = V3()
        self.accel_smooth = V3()
        self.balance_out = None
        self.balance_excess = 0.0
        self.unbal_t = 0.0
        self.flight_t = 0.0
        self.flight_hold = 0.0
        self.prev_npl = 0
        self.trailing_support = False
        self.unsupported_t = 0.0
        self.air_belly = False
        self.drifting = False
        self.slide_buf = 0.0
        self.detach_t = 0.0
        self.ground_n = WORLD_UP.copy()
        self.land_right_t = 0.0
        self.land_right_n = WORLD_UP.copy()
        for l in self.legs:
            l.planted = False
            l.swing = None
            l.searching = False

    def _update_air(self, dt):
        fr, inp, d = self.frame, self.inp, self.d
        self.vel = self.vel + WORLD_UP * (-G * dt)
        vh = V3(self.vel.x, 0.0, self.vel.z)
        if not self.air_belly:
            mv = self.heading_fwd() * inp.fwd + fr.right * (inp.strafe * 0.75)
            v_des = mv * self.target_speed()
            vdes_h = V3(v_des.x, 0.0, v_des.z)
            vh = vh + (vdes_h - vh) * clamp(1.2 * dt, 0.0, 1.0)
        else:
            fwd_h = project_on_plane(fr.fwd, WORLD_UP).normalized(V3(0.0, 0.0, 1.0))
            vh = vh - fwd_h * (vh.dot(fwd_h) * damp_factor(4.0, dt))
        self.vel = V3(vh.x, self.vel.y, vh.z)
        self.yaw_rate += (inp.turn * d["turn_rate"] - self.yaw_rate) * damp_factor(4.0, dt)
        fr.rotate(WORLD_UP, -self.yaw_rate * dt)
        self.pos = self.pos + self.vel * dt
        belly_hit = self._collide()
        self.impact = self._predict_impact()
        if self.vel.y <= 0.0 or self.state_t > 0.4:
            depth = max([self.pos.y - l.foot.y for l in self.legs] + [0.0])
            reach = max(self.ride * 1.02, depth + 0.05) + max(0.0, -self.vel.y) * dt
            h = self.world.raycast(self.pos, WORLD_UP * -1.0, reach)
            min_ny = 0.3 if self.climbing_mode else 0.55
            if h is not None and h.normal.y > min_ny:
                self._land(h)
        self.air_belly = (self.state == AIR and belly_hit)
        if self.air_belly and self.state_t > 0.04:
            side_base = project_on_plane(fr.right, WORLD_UP).normalized(V3(1, 0, 0))
            v_lat = self.vel.dot(side_base)
            sgn = 1.0 if v_lat >= 0.0 else -1.0
            side_dir = side_base * sgn
            if abs(v_lat) < 1.2:
                self.vel = self.vel + side_dir * (1.2 - abs(v_lat))
            self.vel = self.vel + side_dir * (10.5 * dt)
            self.ang_vel = self.ang_vel + WORLD_UP.cross(side_dir) * (5.5 * dt)
        if self.pos.y < -60:
            self.vel = V3()

    def _predict_impact(self):
        up = WORLD_UP
        p = self.pos - up * (self.ride * 1.05)
        v = self.vel.copy()
        t = 0.0
        step = 0.04
        while t < 2.4:
            v2 = v - up * (G * step)
            q = p + (v + v2) * (0.5 * step)
            dv = q - p
            dl = dv.length()
            if dl > 1e-5:
                h = self.world.raycast(p, dv * (1.0 / dl), dl)
                if h is not None and h.normal.y > 0.3:
                    f = h.t / dl
                    return (t + step * f, h.point, h.normal, (v + (v2 - v) * f).length())
            p, v, t = q, v2, t + step
        return None

    def _brace_weight(self):
        imp = self.impact
        if imp is None or (self.vel.y > 0.0 and imp[0] > 0.5):
            return 0.0
        tilt = angle_between(self.frame.up, imp[2])
        window = 0.45 * math.sqrt(self.size) * (1.0 + 0.35 * clamp(tilt / 0.8, 0.0, 1.0))
        return clamp(1.0 - (imp[0] - 0.08) / window, 0.0, 1.0)

    def _land(self, h):
        old_state, old_state_t = self.state, self.state_t
        old_unsup = self.unsupported_t
        old_unbal = self.unbal_t
        old_gn = self.ground_n.copy()
        old_lp = self.look_pitch
        old_abs, old_brace = self.absorb, self.brace
        old_av, old_vel = self.ang_vel.copy(), self.vel.copy()
        old_lrt, old_lrn = self.land_right_t, self.land_right_n.copy()

        self.state, self.state_t = GROUND, 0.0
        self.unsupported_t = 0.0
        land_n = h.normal.normalized(WORLD_UP)
        tilt = angle_between(self.frame.up, land_n)
        land_fr = Frame(self.heading_fwd(), land_n)
        self.ground_n = land_n.copy()
        self.look_pitch = clamp(self.inp.aim, -self.look_pitch_dn, self.look_pitch_up) if abs(self.inp.aim) > 1e-3 else 0.0
        impact = -min(0.0, self.vel.y)
        self.absorb = max(self.absorb, clamp((impact - 1.5) / 22.0, 0.0, 0.5))
        self.brace = max(self.brace, clamp(impact / 12.0, 0.0, 1.0))
        vh = V3(self.vel.x, 0.0, self.vel.z)
        v_spd = vh.length()
        if tilt < 0.15 and impact > 2.5 and v_spd > 1.5:
            axis = WORLD_UP.cross(vh) * (1.0 / v_spd)
            dip = clamp(impact * min(v_spd, 18.0) * 0.011 / math.sqrt(self.d["mass"]), 0.0, 1.35)
            self.ang_vel = self.ang_vel + axis * dip
        if tilt > 0.10:
            self.land_right_t = clamp(0.28 + 0.40 * tilt, 0.28, 0.65)
            self.land_right_n = land_n.copy()
            if tilt >= 0.15:
                self.ang_vel = self.ang_vel * 0.20
        else:
            self.land_right_t = 0.0
        self.vel = V3(self.vel.x, max(self.vel.y, -6.0), self.vel.z)
        fr = self.frame
        snap_dist = (1.35 if tilt > 0.15 else 0.95) * self.size
        for l in self.legs:
            l.push = False
            l.mount_world = self.pos + fr.to_world(l.mount_local)
            l.ideal_now = self.pos + land_fr.to_world(l.rest_local(self.ride))
            lead_l = vh * min(0.035, 0.25 * l.reach / v_spd) if v_spd > 4.0 else V3()
            fh = None
            if self.climbing_mode and tilt > 0.35:
                for cand in (project_on_plane(l.foot - h.point, land_n) + h.point + land_n * (self.ride + 1.0),
                             l.ideal_now + land_n * (self.ride + 1.0)):
                    hc = self.world.raycast(cand, land_n * -1.0, self.ride * 2.5 + 1.0)
                    if hc is not None and hc.normal.dot(land_n) > 0.6 and hc.point.dist(l.mount_world) <= 0.98 * l.reach:
                        fh = (hc.point, hc.normal)
                        break
            if fh is None:
                fh = self._find_foothold(l, project_on_plane(l.foot - h.point, WORLD_UP) + h.point + lead_l)
            if fh is None:
                fh = self._find_foothold(l, l.ideal_now)
            reach_ok = fh is not None and (tilt <= 0.15 or fh[0].dist(l.mount_world) <= 0.98 * l.reach)
            if fh and reach_ok and fh[0].dist(l.foot) < snap_dist:
                l.foot, l.foot_normal = fh[0], fh[1]
                l.planted = True
                l.swing = None
                l.searching = False
            elif fh:
                sw_t = (clamp(fh[0].dist(l.foot) / 18.0, 0.07, 0.14)
                        if tilt > 0.15 else clamp(fh[0].dist(l.foot) / 14.0, 0.11, 0.18))
                sw_up = land_n if (self.climbing_mode and tilt > 0.35) else self.probe_up()
                l.begin_swing(fh[0], fh[1], sw_t, 0.05, sw_up, True, self.world, 0.0, self)
            else:
                l.searching = True
        if not any(l.planted for l in self.legs):
            best = None
            best_d = 1e9
            for l in self.legs:
                if l.swing is not None and l.swing.has_ground and l.swing.target.dist(l.mount_world) <= 0.96 * l.reach:
                    d_f = l.swing.target.dist(l.foot)
                    if d_f < best_d:
                        best_d = d_f
                        best = l
            if best is not None:
                best.foot = best.swing.target.copy()
                best.foot_normal = best.swing.normal
                best.planted = True
                best.swing = None
                best.searching = False
        if not any(l.planted or (l.swing is not None and l.swing.has_ground) for l in self.legs):
            self.state, self.state_t = old_state, old_state_t
            self.unsupported_t = old_unsup
            self.unbal_t = old_unbal
            self.ground_n = old_gn
            self.look_pitch = old_lp
            self.absorb, self.brace = old_abs, old_brace
            self.ang_vel, self.vel = old_av, old_vel
            self.land_right_t, self.land_right_n = old_lrt, old_lrn
            return False
        self.unbal_t = 0.0
        self.air_belly = False
        self.emit(EV_LAND, h.point, h.normal, strength=impact * self.d["mass"] * 0.25)
        if not any(l.searching for l in self.legs):
            self.situation.edge = 0.0
        return True

    # ------------------------------------------------------------------ mantle
    def _try_mantle(self):
        if self.ground_n.y < 0.86:
            return False
        d, fr, w = self.d, self.frame, self.world
        g_u = w.raycast(self.pos, WORLD_UP * -1.0, self.ride + 2.5)
        if g_u is not None and g_u.normal.y < 0.90:
            return False
        fwd_h = V3(fr.fwd.x, 0.0, fr.fwd.z).normalized()
        if fwd_h.length_sq() < 0.5:
            return False
        planted = [l.foot.y for l in self.legs if l.planted]
        if g_u is not None and abs(g_u.point.y - (self.pos.y - self.ride)) < 1.2 * self.ride:
            base_y = g_u.point.y
        else:
            base_y = sum(planted) / len(planted) if planted else self.pos.y - self.ride
        L = d["body_half"][2] + 0.6
        o_hi = V3(self.pos.x, base_y + d["step_max"] + 0.1, self.pos.z)
        h = w.raycast(o_hi, fwd_h, L)
        if h is None or abs(h.normal.y) > 0.4:
            return False
        o_lo = V3(self.pos.x, base_y + d["step_max"] * 0.4, self.pos.z)
        hl = w.raycast(o_lo, fwd_h, L)
        if hl is not None and hl.t < h.t - 0.2:
            return False
        reach_h = max(d["mantle_max"], d.get("wire_max", 0.0))
        top_o = V3(h.point.x, base_y + reach_h + 0.5, h.point.z) + fwd_h * 0.25
        h2 = w.raycast(top_o, WORLD_UP * -1.0, reach_h + 0.5)
        if h2 is None or h2.normal.y < 0.8:
            return False
        height = h2.point.y - base_y
        if height <= d["step_max"] * 0.9 or height > reach_h:
            return False
        pre = w.raycast(V3(h.point.x, h2.point.y + 0.3, h.point.z) - fwd_h * 0.25, WORLD_UP * -1.0, 0.85)
        if pre is not None and pre.normal.y > 0.64 and h2.point.y - pre.point.y < 0.45:
            return False
        ahead = fwd_h * (d["body_half"][2] * 0.6)
        end_hit = w.raycast(top_o + ahead, WORLD_UP * -1.0, reach_h + 0.5)
        end_ground = h2.point
        if (end_hit is not None and end_hit.normal.y >= 0.8
                and h2.point.y - 0.1 <= end_hit.point.y <= base_y + reach_h):
            end_ground = end_hit.point
        height = max(height, end_ground.y - base_y)
        wire = height > d["mantle_max"]
        headroom = self.ride + 2.0 * d["body_half"][1]
        if any(w.raycast(p + WORLD_UP * 0.1, WORLD_UP, headroom) for p in (h2.point, end_ground)):
            return False
        m = Mantle()
        m.p0 = self.pos.copy()
        m.p2 = end_ground + WORLD_UP * self.ride
        m.t = 0.0
        m.height = height
        m.top = h2.point
        m.fwd = fwd_h
        m.wire = wire
        m.landed = not wire
        m.dur = ((0.9 + 0.2 * height) if wire else (0.75 + 0.3 * height)) * math.sqrt(self.size)
        self.mantle = m
        self.state, self.state_t = MANTLE, 0.0
        self.drifting = False
        for l in self.legs:
            if l.is_front or (self.biped and l.idx == 0):
                l.force = True
        self.emit(EV_MANTLE, h2.point, WORLD_UP, strength=height)
        return True

    def _mantle_pos(self, t):
        m = self.mantle
        t = clamp(t, 0.0, 1.0)
        if m.wire:
            s1 = smoothstep(t / 0.7)
            s2 = smoothstep((t - 0.5) / 0.5)
        else:
            s1 = smoothstep(t / 0.6)
            s2 = smoothstep((t - 0.3) / 0.7)
        return V3(m.p0.x + (m.p2.x - m.p0.x) * s2,
                  m.p0.y + (m.p2.y + 0.1 - m.p0.y) * s1,
                  m.p0.z + (m.p2.z - m.p0.z) * s2)

    def _update_mantle(self, dt):
        m = self.mantle
        strain = max((l.stretch for l in self.legs if l.planted), default=0.0)
        m.t += dt / m.dur * (0.25 if strain > 1.0 else 1.0)
        new = self._mantle_pos(m.t)
        self.vel = (new - self.pos) * (1.0 / dt)
        self.pos = new
        self.yaw_rate *= math.exp(-6.3 * dt)
        if m.wire and not m.landed and m.t >= 0.72:
            m.landed = True
            fr = self.frame
            for l in self.legs:
                l.mount_world = self.pos + fr.to_world(l.mount_local)
                l.ideal_now = self.pos + fr.to_world(l.rest_local(self.ride))
                fh = self._find_foothold(l, l.ideal_now)
                if fh:
                    l.begin_swing(fh[0], fh[1], 0.12, 0.1, self.probe_up(), True, self.world, 0.0, self)
                else:
                    l.searching = True
        if m.t >= 1.0:
            self.state, self.state_t = GROUND, 0.0
            self.vel = V3(self.frame.fwd.x, 0.0, self.frame.fwd.z) * 1.0
            self.mantle = None

    # ------------------------------------------------------------------ orientation
    def _update_orientation(self, dt):
        """Align chassis up-axis to terrain slope, wall climb probes, or air
        tumble/aim pitch, then layer acceleration lean and slide counter-lean.
        """
        fr, d = self.frame, self.d
        righting = False
        if self.state == AIR:
            if not self.wire_active() and abs(self.inp.aim) > 1e-3:
                a = clamp(self.inp.aim, -AIM_MAX, AIM_MAX)
                target = rotate_axis(WORLD_UP, fr.right, a)
            else:
                self.ang_vel = self.ang_vel * math.exp(-3.5 * dt)
                ang = self.ang_vel.length()
                if ang * dt > 1e-7:
                    fr.rotate(self.ang_vel * (1.0 / ang), ang * dt)
                self.on_wall = False
                return
        else:
            righting = (self.state == GROUND and self.land_right_t > 0.0
                        and (any(l.planted for l in self.legs) or self.state_t < 0.08))
            if righting:
                self.ground_n = self.land_right_n.copy()
                ground_up = self.land_right_n.copy()
                target = ground_up
                if not self.on_wall and abs(self.inp.aim) > 1e-3:
                    self.look_pitch = clamp(self.inp.aim, -self.look_pitch_dn, self.look_pitch_up)
                    pitch_axis = self.heading_fwd().cross(ground_up).normalized(fr.right)
                    target = rotate_axis(target, pitch_axis, self.look_pitch)
            else:
                n_l = self.plane_n_local
                if self.state == MANTLE:
                    self.ground_n = self.ground_n.lerp(WORLD_UP, damp_factor(14.0, dt)).normalized(WORLD_UP)
                elif n_l is not None:
                    raw_n = self.support_frame().to_world(n_l).normalized(WORLD_UP)
                    rate = 22.0 if self.climbing_mode else 14.0
                    self.ground_n = self.ground_n.lerp(raw_n, damp_factor(rate, dt)).normalized(WORLD_UP)
                elif self.climbing_mode:
                    self.ground_n = fr.up.copy()
                else:
                    self.ground_n = self.ground_n.lerp(WORLD_UP, damp_factor(2.0, dt)).normalized(WORLD_UP)
                n_w = self.ground_n
                if self.climbing_mode:
                    target = self._climb_probes(n_w)
                    ground_up = target
                else:
                    ground_up = rotate_towards(WORLD_UP, n_w.normalized(), math.radians(max(d["max_tilt"], 40.0)))
                    target = ground_up
                if not self.on_wall:
                    a = fr.to_local(self.accel_smooth)
                    gal = 0.35 if (self.galloping_fast() or self.flight_t > 0.0) else 1.0
                    lat_scale = 1.0 / math.sqrt(max(0.5, d["mass"]))
                    lean = fr.fwd * clamp(a.z * 0.006 * gal, -0.15, 0.15)
                    lean = lean + fr.right * clamp((a.x * 0.0045 + self.yaw_rate * self.speed * 0.003) * lat_scale, -0.20, 0.20)
                    if self.biped:
                        lean = lean + fr.fwd * min(0.07, self.speed * 0.01)
                    if self.drift_amt > 0.01:
                        lean = lean - self.slide_dir * (0.14 * self.drift_amt)
                    target = (target + lean).normalized()
                if not self.on_wall and (abs(self.inp.aim) > 1e-3 or abs(self.look_pitch) > 1e-4):
                    want_a = clamp(self.inp.aim, -self.look_pitch_dn, self.look_pitch_up) if abs(self.inp.aim) > 1e-3 else 0.0
                    strained = any(
                        l.ik.err > 0.015
                        or (l.planted and l.ik.limit_margin_deg < 1.0)
                        or (l.planted and l.stretch > 0.96 and self.speed < 2.0)
                        for l in self.legs
                    )
                    if strained:
                        self._pitch_cap = max(0.35, self._pitch_cap - 1.2 * dt)
                        self._pitch_strain_t = 0.45
                    elif self._pitch_strain_t > 0.0:
                        self._pitch_strain_t = max(0.0, self._pitch_strain_t - dt)
                    else:
                        self._pitch_cap = min(1.0, self._pitch_cap + 0.65 * dt)
                    capped_a = want_a * self._pitch_cap
                    rate = 6.0 if self.speed > 2.0 else 12.0
                    d_pitch = (capped_a - self.look_pitch) * damp_factor(rate, dt)
                    if self.speed > 2.0 and ground_up.y < 0.97:
                        max_dp = 1.8 * dt
                        d_pitch = clamp(d_pitch, -max_dp, max_dp)
                    self.look_pitch += d_pitch
                    if abs(self.look_pitch) > 1e-4:
                        pitch_axis = self.heading_fwd().cross(ground_up).normalized(fr.right)
                        target = rotate_axis(target, pitch_axis, self.look_pitch)
                fwd_v = self.vt.dot(self.heading_fwd())
                fwd_sgn = -1.0 if fwd_v < -1.0 else 1.0
                fwd_align = clamp(abs(fwd_v) / max(self.speed, 1.0), 0.0, 1.0)
                if self.flight_t > 0.0 and self.flight_dur > 0.0:
                    prog = 1.0 - self.flight_t / self.flight_dur
                    pitch = (math.radians(d.get("bound_pitch", 7.0)) * self.gait.stride_pitch
                             * fwd_sgn * fwd_align * (math.cos(math.pi * prog) * 0.92 + 0.08))
                    target = rotate_axis(target, fr.right, pitch)
                elif (self.gait.pair_sync and self.state == GROUND and d.get("bound", 0.0) > 0.0
                      and self.speed > 0.4 * d["speeds"]["run"]):
                    front = any(l.planted for l in self.legs if l.mount_local.z > 0.05)
                    rear = any(l.planted for l in self.legs if l.mount_local.z < -0.05)
                    sgn = -0.75 if (front and not rear) else (0.85 if (rear and not front) else 0.0)
                    if sgn:
                        pitch = math.radians(d.get("bound_pitch", 7.0)) * self.gait.stride_pitch * fwd_sgn * fwd_align * sgn
                        target = rotate_axis(target, fr.right, pitch)
                if self.state == MANTLE and self.mantle:
                    pitch = math.radians(16.0) * math.sin(math.pi * clamp(self.mantle.t, 0.0, 1.0))
                    target = rotate_axis(target, fr.right, pitch)
                elif not self.climbing_mode:
                    lim_rel = math.radians(d.get("look_tilt", 25.0) + 8.0)
                    target = rotate_towards(ground_up, target.normalized(), lim_rel)
        target = target.normalized()
        err = fr.up.cross(target)
        if fr.up.dot(target) < 0:
            err = err.normalized() if err.length_sq() > 1e-9 else fr.right
        if righting:
            f = 5.2 / math.sqrt(self.size)
            w = min(math.tau * f, 0.78 / max(dt, 1e-4))
            self.ang_vel = (self.ang_vel + err * (w * w) * dt) / (1.0 + 2.0 * 0.95 * w * dt)
        else:
            f = (4.5 if self.climbing_mode else 2.6) / math.sqrt(self.size)
            w = math.tau * f
            self.ang_vel = self.ang_vel + (err * (w * w) - self.ang_vel * (2.0 * 0.7 * w)) * dt
        ang = self.ang_vel.length()
        if ang * dt > 1e-7:
            fr.rotate(self.ang_vel * (1.0 / ang), ang * dt)
        if righting:
            self.land_right_t = max(0.0, self.land_right_t - dt)
            if angle_between(fr.up, target) < 0.035 and self.ang_vel.length() < 0.4:
                self.land_right_t = 0.0
        self.on_wall = self.climbing_mode and self.state != AIR and fr.up.dot(WORLD_UP) < 0.5

    def _climb_probes(self, target):
        if self.state != GROUND:
            return target
        fr, w, inp = self.frame, self.world, self.inp
        up = fr.up
        mv = fr.fwd * inp.fwd + fr.right * (inp.strafe * 0.75)
        if mv.length_sq() < 0.01:
            return target
        dirv = mv.normalized()
        L = self.d["body_half"][2] + self.reach_avg * 0.65
        h = w.raycast(self.pos, dirv, L)
        if h is not None and h.normal.dot(up) < 0.7 and h.normal.dot(dirv) < -0.2:
            k = clamp(1.0 - h.t / L, 0.0, 1.0) ** 0.7 * 0.9
            return (target * (1.0 - k) + h.normal * k).normalized()
        p = self.pos + dirv * (self.d["body_half"][2] + self.reach_avg * 0.35)
        depth = self.ride * 2.2
        if w.raycast(p, up * -1.0, depth) is None:
            end = p - up * depth
            back = (self.pos - up * self.ride) - end
            bl = back.length()
            if bl > 1e-4:
                h3 = w.raycast(end, back * (1.0 / bl), self.reach_avg * 1.5)
                if h3 is not None and h3.normal.dot(up) < 0.9:
                    return (target * 0.4 + h3.normal * 0.6).normalized()
        return target

    def _update_vis_offset(self, dt):
        target = V3()
        if self.biped and self.state == GROUND:
            swinging = [l for l in self.legs if l.swing is not None]
            if len(swinging) == 1:
                target.x = 0.13 * -swinging[0].side * (1.0 - 0.5 * min(1.0, self.speed / 6.0))
            target.y = -0.06 * math.cos(4.0 * math.pi * self.phase) * min(1.0, self.speed / 3.0)
        self.vis_offset = self.vis_offset.lerp(target, damp_factor(7.0, dt))

    # ------------------------------------------------------------------ output
    def wire_active(self):
        m = self.mantle
        return self.state == MANTLE and m is not None and m.wire and not m.landed

    def mode_label(self):
        if self.state == AIR:
            return "AIRBORNE"
        if self.wire_active():
            return "WIRE"
        if self.state == MANTLE:
            return "MANTLE"
        if self.state == CROUCHJUMP:
            return "JUMP"
        if self.drifting:
            return "SLIDE"
        if self.on_wall:
            return "CLIMB"
        if self.crouch > 0.5:
            return "CROUCH"
        return {"run": "RUN", "walk": "WALK", "slow": "SLOW WALK"}[self.mode]

    def flags(self):
        f = 0
        if self.drifting:
            f |= 1
        if self.on_wall:
            f |= 2
        if self.crouch > 0.5:
            f |= 4
        if self.climb_enabled:
            f |= 8
        if self.support >= 0.0:
            f |= 16
        if self.mode == "slow":
            f |= 32
        if self.mode == "run":
            f |= 64
        if self.wire_active():
            f |= 128
        if self.flight_t > 0.0:
            f |= 256
        return f

    def pack(self, out):
        """Serialize body frame, state flags, and world leg joint positions into
        a flat float list matching C GcPoseHeader + N leg records.
        """
        fr = self.frame
        p = self.pos + fr.to_world(self.vis_offset)
        gnames = list(self.d["gaits"].keys())
        out.extend((p.x, p.y, p.z, fr.right.x, fr.right.y, fr.right.z,
                    fr.up.x, fr.up.y, fr.up.z, fr.fwd.x, fr.fwd.y, fr.fwd.z,
                    self.speed, float(self.state), float(self.flags()),
                    float(sum(1 for l in self.legs if l.planted)),
                    self.vel.x, self.vel.y, self.vel.z, self.phase, self.drift_amt,
                    float(gnames.index(self.gait_name)), self.support, self.cycle))
        m = self.mantle
        if self.wire_active():
            out.extend((1.0, m.top.x, m.top.y, m.top.z))
        else:
            out.extend((0.0, 0.0, 0.0, 0.0))
        J = self.joints_per_leg
        for l in self.legs:
            js = l.joints
            for i in range(J):
                j = js[i] if i < len(js) else l.foot
                out.extend((j.x, j.y, j.z))
            n = l.plane_normal
            out.extend((n.x, n.y, n.z, 1.0 if l.planted else 0.0, 1.0 if l.sliding else 0.0,
                        l.swing.t if l.swing is not None else -1.0))

    def status(self):
        d = self.d
        npl = sum(1 for l in self.legs if l.planted)
        ov = " [override]" if self.gait_override else ""
        climb = ("ON" if self.climb_enabled else "off") if d["climber"] else "n/a"
        return "\n".join((
            "%s  -  %s" % (d["name"], d["blurb"]),
            "mode %-9s gait %s%s   duty %.2f   cycle %.2fs" % (self.mode_label(), self.gait_name, ov, self.duty, self.cycle),
            "speed %4.1f m/s   feet planted %d/%d   max stretch %.2f" % (
                self.speed, npl, self.n, max(l.stretch for l in self.legs)),
            "state %s   wall-climb %s   support margin %+.2f m" % (
                STATE_NAMES[self.state], climb, self.support),
        ))

    def debug_lines(self, out):
        """Debug overlay: 9 floats per line (a.xyz, b.xyz, role, 0, 0)."""
        fr = self.frame
        up_p = self.probe_up()

        def line(a, b, role):
            out.extend((a.x, a.y, a.z, b.x, b.y, b.z, float(role), 0.0, 0.0))

        def cross(p, sz, role):
            line(p - fr.right * sz, p + fr.right * sz, role)
            line(p - fr.fwd * sz, p + fr.fwd * sz, role)

        p = self.pos
        line(p, p + fr.right * 1.2, DBG_AXIS_X)
        line(p, p + fr.up * 1.2, DBG_AXIS_Y)
        line(p, p + fr.fwd * 1.2, DBG_AXIS_Z)
        t1 = project_on_plane(fr.fwd, up_p).normalized(V3(0, 0, 1))
        t2 = up_p.cross(t1)
        zone = self.d["zone"]
        for l in self.legs:
            c = l.ideal_now
            prev = None
            for k in range(17):
                a = k * math.tau / 16
                q = c + t1 * (math.cos(a) * zone) + t2 * (math.sin(a) * zone)
                if prev is not None:
                    line(prev, q, DBG_LINE_DIM)
                prev = q
            cross(l.ideal_pred, 0.15, DBG_LINE)
            if l.swing is not None:
                sw = l.swing
                cross(sw.target, 0.2, DBG_HIGHLIGHT)
                pts = l.curve_world(self, 12)
                for a, b in zip(pts, pts[1:]):
                    line(a, b, DBG_INFO)
            line(l.foot, l.foot + l.foot_normal * 0.5, DBG_NORMAL if l.planted else DBG_LINE_DIM)
        ys = [l.foot.y for l in self.legs if l.planted]
        if self.hull and ys:
            y = sum(ys) / len(ys) + 0.05
            role = DBG_GOOD if self.support >= 0 else DBG_ERROR
            for i in range(len(self.hull)):
                a = self.hull[i]
                b = self.hull[(i + 1) % len(self.hull)]
                line(V3(a[0], y, a[1]), V3(b[0], y, b[1]), role)
            line(V3(p.x, p.y, p.z), V3(p.x, y, p.z), DBG_LINE)
            cross(V3(p.x, y, p.z), 0.25, DBG_LINE)
