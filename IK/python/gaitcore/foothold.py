"""Foothold search, surface scoring, obstacle clearance probes, and edge logic."""
import math

from .vmath import V3, WORLD_UP, LOCAL_FRAME, clamp, project_on_plane
from .mech_const import G, GROUND, MANTLE


class FootholdMixin:
    """Mixin supplying foothold selection and placement queries for Mech."""

    def _landing_pose(self, l, delay):
        """Estimate the hip position and orientation at contact."""
        frame = self.frame
        if abs(self.yaw_rate * delay) > 1e-6:
            frame = frame.copy()
            frame.rotate(self.probe_up(), -self.yaw_rate * delay)
        if self.state == MANTLE:
            hull = self._mantle_pos(self.mantle.t + delay / self.mantle.dur)
        else:
            hull = self.pos + self.planar_travel(delay)
        if self.state == GROUND and not self.climbing_mode:
            if self.flight_t > 0.0:
                t = min(self.flight_t, delay)
                hull.y += self.vel.y * t - 0.5 * G * t * t
            elif 0.5 < self.ground_n.y < 0.995:
                dh_x, dh_z = hull.x - self.pos.x, hull.z - self.pos.z
                dy_slope = clamp(-(dh_x * self.ground_n.x + dh_z * self.ground_n.z) / self.ground_n.y, -3.0, 3.0)
                if dy_slope >= 0.0 or self.speed < 3.0:
                    hull.y += dy_slope
                elif self.vel.y < -0.5:
                    hull.y += max(dy_slope, self.vel.y * delay - 0.35)
        return hull + frame.to_world(l.mount_local + self.vis_offset), frame

    def _sibling_sep_penalty(self, l, p, up):
        """Penalize footholds that land too close to a same-side sibling leg."""
        min_sep = max(0.36, 2.2 * l.foot_r)
        pen = 0.0
        for nb in self.legs:
            if nb.side != l.side or nb is l:
                continue
            q = nb.foot if nb.planted else (nb.swing.target if nb.swing is not None and nb.swing.has_ground else None)
            if q is not None:
                d = project_on_plane(p - q, up).length()
                if d < min_sep:
                    pen += 2.5 * (min_sep - d)
        return pen

    def _find_foothold(self, l, ideal, delay=0.0):
        """Pick where a foot should land near `ideal`.  Returns (point, normal) or None.

        1. line-of-sight from the hip to the probe origin (feet must not land
           inside walls; climbers *use* the wall they hit)
        2. probe down along the support up vector
        3. climbers: wrap probe around convex edges
        4. nudge inward (about two foot widths) off rock brims and away from
           steep rock faces before committing
        5. score a two-ring candidate fan by distance, elevation delta, slope,
           and edge margin; stepping up onto walkable ledges costs zero extra
           penalty so climbing onto low cover stays natural.
        """
        w, d = self.world, self.d
        cm = self.climbing_mode
        up = self.frame.up if cm else WORLD_UP
        slope_rise = 0.0
        if not cm and 0.5 < self.ground_n.y < 0.995:
            raw_sr = clamp(-((ideal.x - self.pos.x) * self.ground_n.x
                             + (ideal.z - self.pos.z) * self.ground_n.z) / self.ground_n.y, -3.0, 3.0)
            if raw_sr >= 0.0 or self.speed < 3.0 or self.vel.y < -0.5:
                slope_rise = raw_sr
        if self.state == MANTLE:
            probe_up = d["mantle_max"] + 0.6
        else:
            probe_up = max(d["step_max"], d["step_height"]) * 1.5 + 0.3 + max(0.0, slope_rise)
        probe_down = self.ride * 1.3 + d["step_max"] + 0.5 + max(0.0, -slope_rise)
        origin = ideal + up * probe_up
        pose = self._landing_pose(l, delay) if delay > 0.0 else None
        mount = pose[0] if pose is not None else l.mount_world
        base_y = None if cm else max(mount.dot(up), self.pos.dot(up) + slope_rise)
        ceil_lim = None if cm else self.ceil_y
        if not cm:
            m_hc = w.raycast(mount, up, probe_up + 1.5)
            if m_hc is not None and m_hc.normal.dot(up) < -0.5:
                my_c = m_hc.point.dot(up)
                ceil_lim = my_c if ceil_lim is None else min(ceil_lim, my_c)
        lift_o = (mount - origin).dot(up)
        if lift_o > 0.0:
            # never probe from below the hip (e.g. body sagging after a fall)
            origin = origin + up * lift_o
        los = origin - mount
        L = los.length()
        if L > 1e-4:
            h0 = w.raycast(mount, los * (1.0 / L), L)
            if h0 is not None:
                if cm and self._surface_ok(h0, l, up) and self._reachable(l, h0.point, pose):
                    return self._fh(h0)
                origin = mount + los * (max(0.0, h0.t - 0.25) / L)

        expected = (self.pos - up * self.ride_eff()).dot(up) + slope_rise
        cands = []
        origin, c_o = self._probe_origin(origin, up, base_y, check_ceil=True)
        if c_o is not None:
            ceil_lim = c_o if ceil_lim is None else min(ceil_lim, c_o)
        h1 = w.raycast(origin, up * -1.0, max(1.0, (origin - ideal).dot(up) + probe_down))
        if h1 is not None and ceil_lim is not None and h1.point.dot(up) >= ceil_lim - 0.08:
            h1 = None
        ok1 = h1 is not None and self._surface_ok(h1, l, up)
        ok2 = ok1 and self._reachable(l, h1.point, pose)
        ok3 = ok2 and self._clear_spot(l, h1.point, up, mount)
        self.fh_count += 1
        self.fh_bad += 1             # undone below if the spot under the ideal is taken
        self.fh_why = ("none" if h1 is None else "surface" if not ok1 else "reach" if not ok2
                       else "clear" if not ok3 else "ok")   # read by tools/foothold_check.py
        if self.fh_why in ("reach", "none"):
            # out of reach (down a ramp at speed) is the slope, and no ground
            # under the ideal (a narrow bridge, a ledge) is a drop: neither is
            # clutter (gaitsel has its own drop signals)
            self.fh_bad -= 1
        h1_cand = False
        near = False
        sep_pen = 0.0
        if ok3:
            if cm:
                return self._fh(h1)
            # the spot straight under the ideal is preferred unless near an edge or crowding a sibling
            near = self._near_edge(h1, up, l.foot_r)
            sep_pen = self._sibling_sep_penalty(l, h1.point, up)
            overhang = self._sheer_overhang(h1, up, l.foot_r)
            bad_cliff = overhang or self._is_thin_wall(h1, up)
            if not near and not bad_cliff and sep_pen <= 0.25 and self._joints_clear(l, h1.point, pose):
                self.fh_bad -= 1
                return self._fh(h1)
            if not near and not bad_cliff and sep_pen > 0.25:
                # Sibling spacing is geometry/kinematic separation, not rough/cluttered terrain.
                self.fh_bad -= 1
            self.fh_why = "edge" if (near or bad_cliff) else ("sibling" if sep_pen > 0.25 else "joints")
            h1_cand = (near or sep_pen > 0.25) and not bad_cliff
        elif cm and h1 is None:
            end = origin - up * (probe_up + probe_down * 0.6)
            tgt = self.pos - up * self.ride
            dv = tgt - end
            dl = dv.length()
            if dl > 1e-4:
                h2 = w.raycast(end, dv * (1.0 / dl), dl + l.reach)
                if h2 is not None and self._surface_ok(h2, l, up) and self._reachable(l, h2.point, pose):
                    return self._fh(h2)

        t1 = project_on_plane(self.frame.fwd, up).normalized(V3(0, 0, 1))
        t2 = up.cross(t1)
        zone = d["zone"]
        fwd_d = None
        obstructed = False
        if not cm:
            fwd_d = project_on_plane(self.vt, up).normalized(t1) if self.speed > 0.3 else t1
            obs_o = V3(ideal.x, expected + 0.30 * d["step_max"], ideal.z)
            obstructed = w.raycast(obs_o, fwd_d, max(0.55, 0.8 * zone)) is not None
        if h1_cand:
            cands.append((self._score(h1, ideal, up, expected, obstructed=obstructed, fwd=fwd_d)
                          + (0.6 if near else 0.0) + sep_pen, h1))
        nudge = max(0.2, 2.0 * l.foot_r)
        has_ceil = ceil_lim is not None
        for ring, (r, a0) in enumerate(((nudge, math.pi / 8), (zone * 0.4, 0.0), (zone * 0.8, math.pi / 8))):
            if ring == 1:
                # a clean spot a foot or two away (onto the rock top, off its
                # edge, or out from against its side) beats a different step
                for sc, h in sorted(cands, key=lambda c: c[0])[:3]:
                    if (sc < 0.5 * zone and not self._near_edge(h, up, l.foot_r)
                            and self._sibling_sep_penalty(l, h.point, up) <= 0.25
                            and self._joints_clear(l, h.point, pose)
                            and (cm or (not self._sheer_overhang(h, up, l.foot_r)
                                        and not self._is_thin_wall(h, up)))):
                        if not cm:
                            self.fh_bad -= 1
                        return self._fh(h)
            for k in range(8):
                a = a0 + k * math.pi / 4
                p = ideal + t1 * (math.cos(a) * r) + t2 * (math.sin(a) * r)
                o, c_o = self._probe_origin(p + up * probe_up, up, base_y, check_ceil=has_ceil)
                h = w.raycast(o, up * -1.0, max(1.0, (o - p).dot(up) + probe_down))
                c_lim = ceil_lim if c_o is None else (c_o if ceil_lim is None else min(ceil_lim, c_o))
                if (h is None or (c_lim is not None and h.point.dot(up) >= c_lim - 0.08)
                        or not self._surface_ok(h, l, up) or not self._reachable(l, h.point, pose)
                        or not self._clear_spot(l, h.point, up, mount)):
                    continue
                sc = self._score(h, ideal, up, expected, obstructed=obstructed, fwd=fwd_d) + (0.0 if cm else self._sibling_sep_penalty(l, h.point, up))
                cands.append((sc, h))
        # Check each candidate in score order, deferring _near_edge until needed.
        cands.sort(key=lambda c: c[0])
        edge_fb = None
        after_fb = 0
        for sc, h in cands:
            if cm:
                if self._joints_clear(l, h.point, pose):
                    return self._fh(h)
                continue
            if self._near_edge(h, up, l.foot_r):
                if (edge_fb is None and self._joints_clear(l, h.point, pose)
                        and not self._sheer_overhang(h, up, l.foot_r)
                        and not self._is_thin_wall(h, up)):
                    edge_fb = h
                if edge_fb is not None:
                    after_fb += 1
                    if after_fb >= 6:
                        break
                continue
            if (self._joints_clear(l, h.point, pose)
                    and not self._sheer_overhang(h, up, l.foot_r)
                    and not self._is_thin_wall(h, up)):
                return self._fh(h)
        if edge_fb is not None:
            return self._fh(edge_fb)
        if not cm:
            # nothing near the ideal (a beam narrower than the stance, a foot
            # over an edge): bring the foot in towards the hull, from the ideal
            # and from the rest spot (plus fore/aft shifts along narrow beams)
            fr_use = pose[1] if pose is not None else self.frame
            right_u = project_on_plane(fr_use.right, up).normalized(t2)
            lat_ref = l.mount_local.x if abs(l.mount_local.x) > 1e-4 else l.rest_xz[0]
            lat_sgn = 1.0 if lat_ref >= 0.0 else -1.0
            hull_c = (mount - pose[1].to_world(l.mount_local + self.vis_offset)) if pose is not None else self.pos
            hw_side = w.raycast(hull_c, right_u * lat_sgn, abs(l.rest_xz[0]) + 0.25)
            wall_side = hw_side is not None and abs(hw_side.normal.dot(up)) < 0.55
            min_lat = 0.18 if wall_side else max(0.52, 0.52 * abs(l.mount_local.x))
            bases = [ideal]
            rest = l.ideal_now if pose is None else l.ideal_now + (mount - l.mount_world)
            if rest.dist(ideal) > 0.3:
                bases.append(rest)
            pts = []
            for base in bases:
                hub = project_on_plane(hull_c - base, up)
                lat = t2 * hub.dot(t2)
                fwd_sgn = 1.0 if l.rest_xz[1] >= 0.0 else -1.0
                for f in (0.38, 0.62, 0.74, 0.85):
                    pt = base + lat * f
                    pts.append(pt)
                    pts.append(pt + t1 * (fwd_sgn * 0.85))
                    pts.append(pt - t1 * (fwd_sgn * 0.55))
                for f in (0.35, 0.58, 0.82):
                    pt = base + hub * f
                    pts.append(pt)
                    if f == 0.58:
                        pts.append(pt + t1 * 0.65)
                        pts.append(pt - t1 * 0.65)
            for p in pts:
                if project_on_plane(p - hull_c, up).dot(right_u) * lat_sgn < min_lat:
                    continue
                o, c_o = self._probe_origin(p + up * probe_up, up, base_y, check_ceil=has_ceil)
                h = w.raycast(o, up * -1.0, max(1.0, (o - p).dot(up) + probe_down))
                c_lim = ceil_lim if c_o is None else (c_o if ceil_lim is None else min(ceil_lim, c_o))
                if (h is not None and (c_lim is None or h.point.dot(up) < c_lim - 0.08)
                        and self._surface_ok(h, l, up) and self._reachable(l, h.point, pose)
                        and self._clear_spot(l, h.point, up, mount) and self._joints_clear(l, h.point, pose)
                        and not self._sheer_overhang(h, up, l.foot_r) and not self._is_thin_wall(h, up)):
                    return self._fh(h)
        return None

    def _joints_clear(self, l, p, pose=None):
        """Trial-solve the leg (spare solver, body space, at the hull's
        position at touchdown) and check its knee / ankle aren't inside solid."""
        up = self.probe_up()
        R = max(0.95, 0.55 * sum(l.ik.lengths[-2:]))
        c = p + up * (R + 0.05)
        c2, n = self.world.sphere_push(c, R)
        if n is None or (c2 - c).length_sq() < 1e-6:
            return True
        mount, fr = pose if pose is not None else (l.mount_world, self.frame)
        hull = mount - fr.to_world(l.mount_local + self.vis_offset)
        probe = l.ik_probe
        probe.angles = None
        probe.pts = None
        jl = probe.solve(l.mount_local + self.vis_offset, LOCAL_FRAME, fr.to_local(p - hull))
        r = l.foot_r * 0.55
        pts = [hull + fr.to_world(j) for j in jl]
        last_int = len(pts) - 2
        for j_idx in range(1, last_int + 1):
            cc = pts[j_idx]
            c2, n = self.world.sphere_push(cc, r)
            if n is not None and (c2 - cc).length_sq() > 1e-6:
                if j_idx == last_int and n.dot(up) > 0.96:
                    pen = (c2 - cc).length()
                    surf_dy = ((cc - n * (r - pen)) - p).dot(up)
                    if pen < 0.70 * r and abs(surf_dy) < 0.08:
                        continue
                return False
        for i in range(1, len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            if i == len(pts) - 2:
                b = p + up * (2.0 * l.foot_r + 0.05)
            dv = b - a
            dl = dv.length()
            if dl > 0.05 and self.world.raycast(a, dv * (1.0 / dl), dl) is not None:
                return False
        return True

    def _fh(self, h):
        return (h.point + h.normal * 0.02, h.normal)

    def _probe_origin(self, o, up, base_y=None, check_ceil=True):
        """Move a downward probe's origin up out of any rock it starts inside,
        without pushing it above any overhead ceiling slab."""
        local_ceil = None
        if not self.climbing_mode:
            local_ceil = self.ceil_y
            slope_o = 0.0
            if 0.5 < self.ground_n.y < 0.995:
                slope_o = clamp(-((o.x - self.pos.x) * self.ground_n.x
                                  + (o.z - self.pos.z) * self.ground_n.z) / self.ground_n.y, -3.0, 3.0)
            pos_slope_y = self.pos.dot(up) + slope_o
            start_y = max(base_y, pos_slope_y) if base_y is not None else pos_slope_y
            oy = o.dot(up)
            if (check_ceil or local_ceil is not None) and oy > start_y + 0.05:
                start_pt = o + up * (start_y - oy)
                hc = self.world.raycast(start_pt, up, (oy - start_y) + 0.35)
                if hc is not None and hc.normal.dot(up) < -0.5:
                    cy = hc.point.dot(up)
                    local_ceil = cy if local_ceil is None else min(local_ceil, cy)
            if local_ceil is not None and oy > local_ceil - 0.12:
                o = o + up * ((local_ceil - 0.12) - oy)
        for _ in range(4):
            c2, n = self.world.sphere_push(o, 0.05)
            if n is None or (c2 - o).length_sq() < 1e-8:
                return o, local_ceil
            if n.dot(up) < -0.3:
                o = c2 - up * 0.08
                return o, local_ceil
            o = o + up * 0.6
            if local_ceil is not None and o.dot(up) > local_ceil - 0.10:
                o = o + up * ((local_ceil - 0.10) - o.dot(up))
                return o, local_ceil
        return o, local_ceil

    def _clear_spot(self, l, p, up, mount=None):
        """A foot can stand at p without rock embedment or wall obstruction."""
        r = l.foot_r
        c = p + up * (r + 0.03)
        c2, n = self.world.sphere_push(c, r)
        if n is not None and (c2 - c).length_sq() > 1e-6:
            return False
        tgt = p + up * max(r + 0.1, 0.55 * l.ik.lengths[-1])
        if mount is None:
            mount = l.mount_world
        dv = tgt - mount
        dl = dv.length()
        if dl > 0.3:
            h = self.world.raycast(mount, dv * (1.0 / dl), dl - 0.15)
            if h is not None and not self.climbing_mode:
                return False
        return True

    def _surface_ok(self, h, l, up):
        if self.climbing_mode:
            return h.normal.dot(self.frame.up) > -0.2 or h.normal.dot(l.mount_world - h.point) > 0.05
        return h.normal.y > 0.64

    def _reachable(self, l, p, pose=None):
        hip, frame = pose if pose is not None else (l.mount_world, self.frame)
        return l.reach * 0.08 <= hip.dist(p) <= l.reach * 0.96 and l.ik.feasible(hip, frame, p, slack_deg=0.0)

    def _score(self, h, ideal, up, expected, obstructed=False, fwd=None):
        """Lower is better: distance from ideal, slope, and height change (up-step bias)."""
        dp = h.point - ideal
        dv = dp.dot(up)
        dh = (dp - up * dv).length()
        dev = h.point.dot(up) - expected
        sm = self.d["step_max"]
        if dev >= 0.0:
            hc = 0.8 * max(0.0, dev - sm)
        else:
            hc = 0.4 * -dev
        sc = dh + hc + 2.0 * (1.0 - h.normal.dot(up))
        if obstructed and 0.06 < dev <= sm:
            sc -= min(0.28, 0.45 * dev)
        if fwd is not None and self.speed > 0.3:
            back = -(h.point - ideal).dot(fwd)
            if back > 0.0:
                sc += 0.18 * back
        return sc

    def _horiz_axes(self, up):
        """Four horizontal unit axes (0, 90, +45, -45 deg) orthogonal to `up`."""
        t1 = project_on_plane(self.frame.fwd, up).normalized(V3(0, 0, 1))
        t2 = up.cross(t1).normalized(V3(1, 0, 0))
        return t1, t2, (t1 + t2).normalized(), (t1 - t2).normalized()

    def _sheer_drop_dist(self):
        """Vertical drop depth considered a sheer wall/cliff rather than a steppable obstacle."""
        return clamp(max(self.ride * 0.78, self.d["step_max"] * 1.35), 1.05, 1.55)

    def _probe_sheer_drop(self, pt, up, ref_y, sheer_drop):
        """True if probing down at `pt` reveals a sheer drop > `sheer_drop` below `ref_y`."""
        o = pt + up * 0.45
        hh = self.world.raycast(o, up * -1.0, 0.45 + sheer_drop)
        if hh is None:
            _, hn = self.world.sphere_push(o, 0.04)
            return hn is None
        return (ref_y - hh.point.dot(up)) > sheer_drop

    def _sheer_overhang(self, h, up, r=0.2):
        """True if part of a foot of radius `r` at `h.point` hangs over a sheer cliff/wall drop."""
        sheer = self._sheer_drop_dist()
        ref_y = h.point.dot(up)
        r_chk = max(r, 0.08)
        for ax in self._horiz_axes(up):
            for sgn in (1.0, -1.0):
                if self._probe_sheer_drop(h.point + ax * (sgn * r_chk), up, ref_y, sheer):
                    return True
        return False

    def _is_thin_wall(self, h, up):
        """True if `h.point` sits on a thin wall/ridge <= 0.95 m wide at any yaw angle."""
        sheer = self._sheer_drop_dist()
        ref_y = h.point.dot(up)
        r1 = 0.52
        r2 = 0.96
        for ax in self._horiz_axes(up):
            d_pos = self._probe_sheer_drop(h.point + ax * r1, up, ref_y, sheer)
            d_neg = self._probe_sheer_drop(h.point - ax * r1, up, ref_y, sheer)
            if d_pos and d_neg:
                return True
            if d_pos and self._probe_sheer_drop(h.point - ax * r2, up, ref_y, sheer):
                return True
            if d_neg and self._probe_sheer_drop(h.point + ax * r2, up, ref_y, sheer):
                return True
        return False

    def _near_edge(self, h, up, r=0.2):
        """True if part of a foot of radius r would hang over a drop or butt into a step > 0.2 m."""
        t1 = project_on_plane(self.frame.fwd, up).normalized(V3(0, 0, 1))
        t2 = up.cross(t1)
        for dv in (t1, t1 * -1.0, t2, t2 * -1.0):
            o = h.point + dv * r + up * 0.35
            hh = self.world.raycast(o, up * -1.0, 0.7)
            if hh is None or abs((hh.point - h.point).dot(up)) > 0.2:
                return True
        return False

    def _hang_pose(self, l, ride):
        """Where a searching foot waits: reaching down from the hip under its rest spot."""
        up = self.probe_up()
        hip = l.mount_world
        rest = l.ideal_now
        dv = project_on_plane(rest - hip, up)
        dh = dv.length()
        R = 0.9 * l.reach
        if dh > R:
            rest = hip + dv * (R / dh) - up * (hip - rest).dot(up)
            dh = R
        y0 = (hip - rest).dot(up)
        t = math.sqrt(max(0.0, R * R - dh * dh)) - y0
        g = self.world.raycast(rest + up * (ride * 0.5), up * -1.0, ride * 0.5 + max(t, 0.0) + 0.5)
        if g is not None:
            t = min(t, (rest - g.point).dot(up) - l.foot_r)
        return rest - up * max(t, -ride * 0.3)

    def _futile_step(self, l, fh):
        """Delay a step if the foot would land where it already stands."""
        if fh is not None and fh[0].dist(l.foot) < 0.12:
            l.cooldown = max(l.cooldown, 0.25)
            l.force = False
            return True
        return False
