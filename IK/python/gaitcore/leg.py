"""Per-leg state: stance / swing / searching, swing trajectory, IK.

The swing path uses world coordinates so the foot can stop at contact.
The joint solver uses body coordinates.
"""
import math
from .vmath import V3, LOCAL_FRAME, clamp, damp_factor
from .ik import LegIK


TAKEOFF_RAMP_REACH = 0.36
LANDING_RAMP_REACH = 0.68


def _quintic(u):
    """C2 minimum-jerk ramp: (blend(u), integral_0^u blend(s) ds)."""
    u2 = u * u
    u3 = u2 * u
    return u3 * (10.0 - 15.0 * u + 6.0 * u2), u2 * u2 * (2.5 - 3.0 * u + u2)


class Swing:
    __slots__ = ("start_w", "target", "normal", "t", "dur", "lift", "up",
                 "has_ground", "retarget_cd", "ideal", "force_retarget", "origin",
                 "start_velocity", "end_velocity", "bow", "cruise",
                 "takeoff_time", "landing_time", "plan_t", "plan_dur",
                 "start_hip_l", "target_hip_l", "arc_w", "arc_t0")

    def __init__(self):
        self.t = 0.0
        self.force_retarget = False
        self.end_velocity = V3()
        self.bow = V3()
        self.plan_t = 0.0
        self.plan_dur = 1.0
        self.start_hip_l = None
        self.target_hip_l = None
        self.arc_w = 0.0
        self.arc_t0 = 0.0

    def plan(self, foot, velocity, reach, end_velocity=None):
        """Join the current motion to the target contact at the end of the swing."""
        self.plan_t = self.t
        self.plan_dur = self.dur
        if end_velocity is not None:
            self.end_velocity = end_velocity.copy()
        duration = max(1e-6, (1.0 - self.t) * self.dur)
        offset, offset_vel = self.shaping(self.t, self.dur)
        self.origin = foot - offset
        self.start_velocity = velocity - offset_vel
        travel = self.target - self.origin
        speed = max(travel.length() / duration, 1e-6)
        # A long flight needs a smooth acceleration period at each end.
        # Limit the distance the hip travels while the foot changes speed.
        self.takeoff_time = min(0.46 * duration, TAKEOFF_RAMP_REACH * reach / max(speed, self.start_velocity.length()))
        self.landing_time = min(0.46 * duration, LANDING_RAMP_REACH * reach / max(speed, self.end_velocity.length()))
        cruise_time = max(1e-7, duration - 0.5 * (self.takeoff_time + self.landing_time))
        self.cruise = (travel - self.start_velocity * (0.5 * self.takeoff_time)
                       - self.end_velocity * (0.5 * self.landing_time)) / cruise_time

    def shaping(self, t, dur):
        # Vertical clearance + horizontal bow; pos, vel, and acc are zero at endpoints.
        q = t * (1.0 - t)
        d = max(dur, 1e-5)
        lift = self.up * self.lift
        pos = lift * (64.0 * q**3)
        vel = lift * (192.0 * q * q * (1.0 - 2.0 * t) / d)
        if self.bow.length_sq() > 1e-10:
            span = max(1e-5, 1.0 - self.plan_t)
            u = clamp((t - self.plan_t) / span, 0.0, 1.0)
            qb = u * (1.0 - u)
            pos = pos + self.bow * (16.0 * qb * qb)
            vel = vel + self.bow * (32.0 * qb * (1.0 - 2.0 * u) / (d * span))
        return pos, vel

    def sample(self, t):
        """Return the world position and velocity at this swing phase."""
        if t >= 1.0:
            return self.target, self.end_velocity
        duration = (1.0 - self.plan_t) * self.plan_dur
        elapsed = max(0.0, (t - self.plan_t) * self.plan_dur)
        left = duration - elapsed
        # Integrate a C2 quintic velocity transition at each end of the path.
        if self.takeoff_time > 1e-9 and elapsed < self.takeoff_time:
            u = elapsed / self.takeoff_time
            blend, integ = _quintic(u)
            cruise_weight = self.takeoff_time * integ
            initial_weight = elapsed - cruise_weight
            point = self.origin + self.start_velocity * initial_weight + self.cruise * cruise_weight
            velocity = self.start_velocity.lerp(self.cruise, blend)
        elif self.landing_time > 1e-9 and left < self.landing_time:
            u = left / self.landing_time
            blend, integ = _quintic(u)
            rem_disp = self.end_velocity * left + (self.cruise - self.end_velocity) * (self.landing_time * integ)
            point = self.target - rem_disp
            velocity = self.end_velocity.lerp(self.cruise, blend)
        else:
            initial_weight = 0.5 * self.takeoff_time
            cruise_weight = elapsed - initial_weight
            point = self.origin + self.start_velocity * initial_weight + self.cruise * cruise_weight
            velocity = self.cruise
        offset, offset_vel = self.shaping(t, self.plan_dur)
        return point + offset, velocity + offset_vel


class Leg:
    def __init__(self, idx, ldef, d, size=1.0):
        # NOTE: `size` scales geometry; mech defs are authored in metres so the
        # Mech passes 1.0 (mechdefs 'size' only scales gait timing).
        self.idx = idx
        self.pair = idx // 2
        self.side = ldef["side"]
        mx, my, mz = ldef["mount"]
        rx, rz = ldef["rest"]
        self.mount_local = V3(mx * size, my * size, mz * size)
        self.rest_xz = (rx * size, rz * size)
        sides = ldef["sides"] or d["sides"]
        rest_dir = V3(rx - mx, 0.0, rz - mz).normalized(V3(self.side, 0, 0))
        self.ik = LegIK(d["plane"], d["coxa"] * size, [l * size for l in d["lengths"]],
                        sides, rest_dir, ldef.get("limits") or d.get("limits"))
        # a spare solver for trial solves during foothold search (keeps the
        # real one's warm start untouched)
        self.ik_probe = LegIK(d["plane"], d["coxa"] * size, [l * size for l in d["lengths"]],
                              sides, rest_dir, ldef.get("limits") or d.get("limits"))
        self.reach = self.ik.reach
        self.is_front = mz > 0.05
        self.foot_r = min(0.14, d["widths"][-1] * 0.6)
        self.force = False
        # dynamic state
        self.foot = V3()            # world position (planted: the ground spot)
        self.foot_normal = V3(0, 1, 0)
        self.planted = True
        self.searching = False
        self.search_cd = 0.0
        self.swing = None
        self.joints = []
        self.joints_local = []
        self.plane_normal = V3(1, 0, 0)
        self.offset = 0.0
        self.prev_window = False
        self.pending = False
        self.cooldown = 0.0
        self.sliding = False
        self.stretch = 0.0
        self.stretch_rate = 0.0
        self.limit_t = 0.0          # how long this planted foot has sat at its limit
        self.push = False           # jump: still pushing off the ground
        self.carry = None           # body-space position of a lifted foot (start of frame)
        self.off_path = 0.0         # distance from the stance path (rest -> predicted spot)
        self.settled_at = None      # rest spot when this leg last settled (at rest)
        self.settled_hull_pos = None
        self.land_lead = None       # body-space lead (foot - rest spot) it landed with
        self.stance_t = 0.0         # time since this foot was planted
        self.last_plant = None      # (point, normal) where it last stood
        self.void_mark = 0.0        # > 0: just found nothing to stand on (edge rule)
        self.displacement = 0.0
        self.mount_world = V3()
        self.ideal_now = V3()
        self.ideal_pred = V3()
        self.air_time = 0.0
        self.plane_roll = 0.0
        self.near_limit = False
        self._kc_t = 1.0

    # ------------------------------------------------------------------
    def rest_local(self, ride, spread=1.0):
        """Rest foot position in body space; spread > 1 widens the stance (bracing)."""
        return V3(self.rest_xz[0] * spread, -ride, self.rest_xz[1] * (1.0 + (spread - 1.0) * 0.5))

    def begin_swing(self, target, normal, dur, lift, up, has_ground, world, clearance_probe, body,
                    start_vel=None, end_vel=None, bow=None, max_ceil_y=None):
        sw = Swing()
        sw.start_w = self.foot.copy()
        sw.target = target
        sw.normal = normal
        sw.dur = max(dur, 0.05)
        sw.up = up
        sw.has_ground = has_ground
        sw.retarget_cd = 0.08
        sw.ideal = None
        sw.lift = lift
        if bow is not None:
            sw.bow = bow.copy()
        if end_vel is not None:
            sw.end_velocity = end_vel.copy()
        self.swing = sw
        self.planted = False
        self.searching = False
        self.sliding = False
        self._shape(world, clearance_probe, self.foot, max_ceil_y=max_ceil_y)
        sw.plan(self.foot, start_vel if start_vel is not None else V3(), self.reach, sw.end_velocity)

    def _shape(self, world, clearance_probe, start, max_ceil_y=None):
        """Raise the swing's lift to clear obstacles while respecting overhead ceilings."""
        sw = self.swing
        p0, p3, up = start, sw.target, sw.up
        h0 = p0.dot(up)
        h3 = p3.dot(up)
        min_ceil = max_ceil_y
        lift = sw.lift
        if clearance_probe > 0.0:
            d = p3 - p0
            side = d.cross(up).normalized(V3(1, 0, 0)) * self.foot_r
            d_len = d.length()
            chord_clear = h0 >= h3 - 0.08 and d_len > 1e-4
            if chord_clear:
                dh = d - up * (h3 - h0)
                dh_len = dh.length()
                if dh_len > 1e-4:
                    base_y = min(h0, h3) + 0.06
                    p0_low = p0 + up * (base_y - h0)
                    dir_h = dh * (1.0 / dh_len)
                    for off in (V3(), side, side * -1.0):
                        if world.raycast(p0_low + off, dir_h, dh_len) is not None:
                            chord_clear = False
                            break
                    if chord_clear:
                        knee_y = min(h0, h3) + max(0.35, 0.55 * self.foot_r + 0.28)
                        if (min_ceil is None or knee_y < min_ceil - 0.08) and world.raycast(p0 + up * (knee_y - h0), dir_h, dh_len) is not None:
                            chord_clear = False
                if chord_clear and abs(h0 - h3) >= 0.08:
                    if world.raycast(p0 + up * 0.05, d * (1.0 / d_len), d_len) is not None:
                        chord_clear = False
            if not chord_clear:
                for s in (0.15, 0.27, 0.39, 0.5, 0.61, 0.73, 0.85):
                    base = p0.lerp(p3, s)
                    for off in (V3(), side, side * -1.0):
                        q = base + off
                        qy = q.dot(up)
                        cp = clearance_probe
                        if max_ceil_y is not None:
                            cp = min(cp, max(0.0, max_ceil_y - qy - 0.08))
                        if cp > 0.04:
                            hc = world.raycast(q + up * (self.foot_r + 0.04), up, cp)
                            if hc is not None:
                                ceil_y = hc.point.dot(up)
                                min_ceil = ceil_y if min_ceil is None else min(min_ceil, ceil_y)
                                cp = min(cp, max(0.0, ceil_y - qy - 0.06))
                        if cp <= 0.02:
                            continue
                        o = q + up * cp
                        h = world.raycast(o, up * -1.0, cp + 0.5)
                        if h is not None and h.normal.dot(up) > 0.45:
                            above = h.point.dot(up) - (h0 + (h3 - h0) * s)
                            if 0.0 < above <= clearance_probe:
                                lift = max(lift, above + 0.35 * sw.lift + self.foot_r + 0.08)
            elif max_ceil_y is None:
                mid_q = p0.lerp(p3, 0.5)
                hc = world.raycast(mid_q + up * (self.foot_r + 0.04), up, lift + 0.35)
                if hc is not None:
                    min_ceil = hc.point.dot(up)
        if min_ceil is not None:
            lift = min(lift, max(0.03, min_ceil - max(h0, h3) - self.foot_r - 0.12))
        sw.lift = lift

    def curve_world(self, body, n=12):
        """World points of the remaining swing arc (debug overlay)."""
        sw = self.swing
        return [self.foot] + [sw.sample(sw.t + (1.0 - sw.t) * k / n)[0] for k in range(1, n + 1)]

    def update_swing(self, dt, body, world=None):
        """Advance the swing. Return True when the foot touches the ground."""
        sw = self.swing
        if sw.plan_dur != sw.dur:
            _, velocity = sw.sample(sw.t)
            sw.bow = sw.bow * max(0.0, 1.0 - sw.t)
            sw.plan(self.foot, velocity, self.reach)
        sw.t = min(1.0, sw.t + dt / sw.dur)
        self.foot, _ = sw.sample(sw.t)
        if sw.t >= 1.0:
            self.foot = sw.target.copy()
            self.swing = None
            if sw.has_ground:
                self.planted = True
                self.foot_normal = sw.normal
                return True
            self.planted = False
            self.searching = True
            self.search_cd = 0.0
        return False

    def keep_clear(self, world, up, max_step=None, max_ceil_y=None, dt=1.0 / 60.0):
        """Lift the sole clear of terrain below without probing through ceilings, and push out of obstacles."""
        sw = self.swing
        r = self.foot_r
        embedded = False
        need_up = 0.0
        c2, n = world.sphere_push(self.foot + up * r, r)
        if n is not None:
            push = c2 - (self.foot + up * r)
            if push.length_sq() > 1e-8:
                embedded = True
                p_up = push.dot(up)
                p_lat = push - up * p_up
                if (sw is None or sw.t < 0.96) and p_lat.length_sq() > 1e-8:
                    self.foot = self.foot + p_lat
                if p_up < -1e-4:
                    self.foot = self.foot + up * p_up
                elif p_up > need_up:
                    need_up = p_up
        step_lim = max_step if max_step is not None else 0.8
        probe_up = min(0.85, step_lim * 0.75 + 0.15) if embedded else (r + 0.28)
        fy = self.foot.dot(up)
        local_ceil = max_ceil_y
        if sw is not None:
            self._kc_t += dt
            run_ray = (self._kc_t >= 1.0 / 30.0 - 1e-6) or embedded or sw.t > 0.88
        else:
            run_ray = True
        if run_ray:
            self._kc_t = 0.0
            if max_ceil_y is not None or embedded:
                hc = world.raycast(self.foot + up * (r + 0.02), up, probe_up + 0.25)
                if hc is not None:
                    cy = hc.point.dot(up)
                    local_ceil = cy if local_ceil is None else min(local_ceil, cy)
            if local_ceil is not None:
                probe_up = min(probe_up, max(0.0, local_ceil - fy - 0.06))
            if probe_up > r * 0.5:
                h = world.raycast(self.foot + up * probe_up, up * -1.0, probe_up + r + 0.04)
                if h is not None and h.normal.dot(up) > 0.50:
                    gap = r if sw is None else 0.02 + (r - 0.02) * (4.0 * sw.t * (1.0 - sw.t)) ** 2
                    up_clear = max(0.0, (h.point - self.foot).dot(up) + gap)
                    need_up = max(need_up, up_clear)
        if local_ceil is not None:
            need_up = min(need_up, max(0.0, local_ceil - fy - r - 0.06))
        if embedded:
            up_cap = min(step_lim + 0.25, max(0.32, 22.0 * dt))
        else:
            up_cap = min(0.45, step_lim * 0.6, max(0.22, 16.0 * dt))
        if need_up > up_cap:
            need_up = up_cap
        if need_up > 0.0:
            self.foot = self.foot + up * need_up
        return need_up

    def retarget(self, target, normal, has_ground, world, clearance_probe, body,
                 end_vel=None, bow=None, max_ceil_y=None):
        """Change the contact point. Keep the foot velocity and the landing time."""
        sw = self.swing
        _, velocity = sw.sample(sw.t)
        sw.target = target
        sw.normal = normal
        sw.has_ground = has_ground
        if bow is not None:
            sw.bow = bow.copy()
        else:
            sw.bow = sw.bow * max(0.0, 1.0 - sw.t)
        if end_vel is not None:
            sw.end_velocity = end_vel.copy()
        self._shape(world, clearance_probe, self.foot, max_ceil_y=max_ceil_y)
        sw.plan(self.foot, velocity, self.reach, sw.end_velocity)

    def solve(self, body, mount_l, dt=1.0 / 60.0):
        """Solve the leg in body space; joints come back as world positions."""
        fr, pos = body.frame, body.pos
        r, u, f = fr.right, fr.up, fr.fwd
        rx, ry, rz = r.x, r.y, r.z
        ux, uy, uz = u.x, u.y, u.z
        fx, fy, fz = f.x, f.y, f.z
        px, py, pz = pos.x, pos.y, pos.z
        dx, dy, dz = self.foot.x - px, self.foot.y - py, self.foot.z - pz
        foot_l = V3(dx * rx + dy * ry + dz * rz,
                    dx * ux + dy * uy + dz * uz,
                    dx * fx + dy * fy + dz * fz)
        jl = self.ik.solve(mount_l, LOCAL_FRAME, foot_l, plane_roll=self.plane_roll)
        self.joints_local = jl
        self.joints = [V3(px + rx * j.x + ux * j.y + fx * j.z,
                          py + ry * j.x + uy * j.y + fy * j.z,
                          pz + rz * j.x + uz * j.y + fz * j.z) for j in jl]
        pn = self.ik.plane_normal
        # body space has x = right (a mirror of the world's handedness), so a
        # cross product taken there comes back flipped
        self.plane_normal = V3(-(rx * pn.x + ux * pn.y + fx * pn.z),
                               -(ry * pn.x + uy * pn.y + fy * pn.z),
                               -(rz * pn.x + uz * pn.y + fz * pn.z))
        self.mount_world = self.joints[0]
        prev = self.stretch
        self.stretch = self.mount_world.dist(self.foot) / self.reach
        # A foot the leg can't reach within its joint limits counts as
        # over-stretched, so the gait steps it (and holds the body back)
        # instead of the drawn foot drifting off its planted spot.
        if self.ik.err > 0.01:
            if self.sliding:
                # a skating foot (drift / slide) just slides to where the leg can reach
                self.foot = self.joints[-1].copy()
                self.stretch = self.mount_world.dist(self.foot) / self.reach
            else:
                self.stretch = max(self.stretch, 0.97 + 2.0 * self.ik.err / self.reach)
        self.near_limit = (self.ik.limit_margin_deg < 6.0) or (self.ik.coxa_ratio > 0.88)
        if self.planted and self.stance_t > 0.0:
            rate = (self.stretch - prev) / max(dt, 1e-4)
            self.stretch_rate += (rate - self.stretch_rate) * damp_factor(26.0, dt)
        else:
            self.stretch_rate = 0.0
        return self.joints

    def reproject_cached(self, body):
        """Re-transform cached body-space joints into world space when an IK solve is skipped."""
        pts = self.joints_local
        if not pts:
            return
        fr, pos = body.frame, body.pos
        r, u, f = fr.right, fr.up, fr.fwd
        rx, ry, rz = r.x, r.y, r.z
        ux, uy, uz = u.x, u.y, u.z
        fx, fy, fz = f.x, f.y, f.z
        px, py, pz = pos.x, pos.y, pos.z
        ml = self.mount_local + body.vis_offset
        p0 = pts[0]
        dx, dy, dz = ml.x - p0.x, ml.y - p0.y, ml.z - p0.z
        self.joints = [V3(px + rx * (j.x + dx) + ux * (j.y + dy) + fx * (j.z + dz),
                          py + ry * (j.x + dx) + uy * (j.y + dy) + fy * (j.z + dz),
                          pz + rz * (j.x + dx) + uz * (j.y + dy) + fz * (j.z + dz)) for j in pts]
        self.mount_world = self.joints[0]
        self.stretch = self.mount_world.dist(self.foot) / self.reach
