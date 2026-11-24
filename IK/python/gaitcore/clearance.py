"""Leg solve, obstacle plane-roll clearance, foot anti-clip, and air brace poses."""
import math
import time

from .vmath import V3, Frame, WORLD_UP, LOCAL_FRAME, clamp, smoothstep, damp_factor, project_on_plane, angle_between
from .mech_const import AIR, MANTLE


class ClearanceMixin:
    """Mixin supplying IK solving, joint-obstacle rolling, and air leg posing."""

    def _joints_clear_roll(self, pts, start_j, foot, r, up, planted=True):
        """Check whether interior joints clear obstacles (skipping flat-floor graze at lowest joint)."""
        last_int = len(pts) - 2
        clear = True
        min_dist = 1e9
        for j_idx in range(start_j, last_int + 1):
            q = pts[j_idx]
            c2, n = self.world.sphere_push(q, r)
            if n is not None and (c2 - q).length_sq() > 1e-6:
                pen = (c2 - q).length()
                surf_dy = ((q - n * (r - pen)) - foot).dot(up)
                # Let the lowest planted ankle graze flat ground normally; only
                # flag real penetrations into rocks or raised obstacle sides.
                if planted and j_idx == last_int and n.dot(up) > 0.96 and pen < 0.70 * r and abs(surf_dy) < 0.08:
                    min_dist = min(min_dist, 0.05)
                else:
                    clear = False
                    min_dist = min(min_dist, -(c2 - q).length())
            else:
                min_dist = min(min_dist, 0.05)
        return clear, min_dist

    def _solve_leg(self, l, dt=1.0 / 60.0):
        """Solve one leg; roll radial knee plane around hip->foot to clear rocks."""
        prof = self._prof
        t0 = time.perf_counter() if prof is not None else 0.0
        mount_l = l.mount_local + self.vis_offset
        prev_roll = None
        # Gradually relax any temporary obstacle roll back toward upright.
        if self.d["plane"] == "radial" and abs(l.plane_roll) > 1e-4:
            prev_roll = l.plane_roll
            relaxed = prev_roll * math.exp(-5.0 * dt)
            l.plane_roll = 0.0 if abs(relaxed) < 1e-3 else relaxed
        l.solve(self, mount_l, dt)
        if l.planted and not l.sliding and l.ik.err > 0.02:
            # A foot that has lost contact cannot support the hull.
            l.planted = False
            l.searching = True
            l.search_cd = 0.0
            l.foot = l.joints[-1].copy()
        if not l.planted and l.ik.err > 0.02:
            for _ in range(2):
                e = l.joints[-1]
                c2, n = self.world.sphere_push(e, l.foot_r)
                if n is None or (c2 - e).length_sq() < 1e-6:
                    break
                l.foot = c2
                l.solve(self, mount_l, dt)
        if prof is not None:
            t1 = time.perf_counter()
            prof[6] += t1 - t0
            prof[8] += 1.0
            t0 = t1
        if self.plane_roll_enabled and self.d["plane"] == "radial" and len(l.joints) >= 4:
            up = self.probe_up()
            start_j = 2 if l.ik.coxa > 1e-6 else 1
            last_int = len(l.joints) - 2
            r = l.foot_r * 0.85
            hit_push = None
            best_pen = 0.0
            for j_idx in range(start_j, last_int + 1):
                j = l.joints[j_idx]
                c2, n = self.world.sphere_push(j, r)
                if n is not None:
                    if l.planted and j_idx == last_int and n.dot(up) > 0.96:
                        pen = (c2 - j).length()
                        surf_dy = ((j - n * (r - pen)) - l.foot).dot(up)
                        if pen < 0.70 * r and abs(surf_dy) < 0.08:
                            continue
                    p2 = (c2 - j).length_sq()
                    if p2 > best_pen and p2 > 1e-6:
                        best_pen = p2
                        hit_push = c2 - j
            if hit_push is not None:
                fr, pos = self.frame, self.pos
                foot_l = fr.to_local(l.foot - pos)
                probe = l.ik_probe
                probe.angles = l.ik.angles
                probe.pts = l.ik.pts
                if prev_roll is not None:
                    jl_prev = probe.solve(mount_l, LOCAL_FRAME, foot_l, plane_roll=prev_roll)
                    if probe.err <= max(0.02, l.ik.err + 0.01):
                        pts_prev = [pos + fr.to_world(q) for q in jl_prev]
                        prev_clear, _ = self._joints_clear_roll(pts_prev, start_j, l.foot, r, up, l.planted)
                        if prev_clear:
                            l.plane_roll = prev_roll
                            l.solve(self, mount_l, dt)
                            if prof is not None:
                                prof[7] += time.perf_counter() - t0
                            return
                base_roll = prev_roll if prev_roll is not None else l.plane_roll
                best_roll = base_roll
                best_score = -math.sqrt(best_pen) - 0.5 * abs(base_roll)
                side_sgn = 1.0 if hit_push.dot(l.plane_normal) >= 0.0 else -1.0
                cands = (
                    side_sgn * 0.20, side_sgn * 0.42, side_sgn * 0.65, side_sgn * 0.85, side_sgn * 1.10,
                    -side_sgn * 0.20, -side_sgn * 0.42, -side_sgn * 0.65, -side_sgn * 0.85, -side_sgn * 1.10,
                )
                for cand_roll in cands:
                    if abs(cand_roll - base_roll) < 1e-4:
                        continue
                    jl = probe.solve(mount_l, LOCAL_FRAME, foot_l, plane_roll=cand_roll)
                    if probe.err > max(0.02, l.ik.err + 0.01):
                        continue
                    pts = [pos + fr.to_world(q) for q in jl]
                    clear, min_dist = self._joints_clear_roll(pts, start_j, l.foot, r, up, l.planted)
                    flip_pen = 0.30 if (abs(base_roll) > 0.05 and cand_roll * base_roll < 0.0) else 0.0
                    sc = (10.0 if clear else min_dist) - 0.5 * abs(cand_roll) - flip_pen
                    if sc > best_score:
                        best_score = sc
                        best_roll = cand_roll
                    if clear and flip_pen == 0.0 and cand_roll * side_sgn > 0.0:
                        break
                if prev_roll is not None:
                    l.plane_roll = prev_roll
                if abs(best_roll - l.plane_roll) > 1e-4:
                    # Fast obstacle escape on active penetration; smooth return is handled by pre-relaxation.
                    max_dr = math.radians(1200.0) * dt
                    l.plane_roll += clamp(best_roll - l.plane_roll, -max_dr, max_dr)
                    l.solve(self, mount_l, dt)
                elif prev_roll is not None:
                    l.solve(self, mount_l, dt)
            if prof is not None:
                prof[7] += time.perf_counter() - t0

    def _slide_foot_clear(self, l, up):
        """Keep a skidding planted foot out of boulders/walls and lift immediately on frontal hits."""
        r = l.foot_r
        c = l.foot + up * r
        c2, n = self.world.sphere_push(c, r)
        if n is not None and (c2 - c).length_sq() > 1e-6:
            l.foot = c2 - up * r
            if n.dot(up) < 0.55:
                l.force = True

    def _clamp_swing_lat(self, l, fr):
        """Keep lateral swing excursion bounded relative to rest stance."""
        if self.drifting:
            return
        fl = fr.to_local(l.foot - self.pos)
        max_dx = 0.47 * l.reach
        dx = fl.x - l.rest_xz[0]
        if abs(dx) > max_dx:
            fl = V3(l.rest_xz[0] + math.copysign(max_dx, dx), fl.y, fl.z)
            l.foot = self.pos + fr.to_world(fl)

    def _feet_clear(self, dt=1.0 / 60.0):
        """No lifted foot ends the frame inside a rock (swing ends, gallop
        poses, mid-air tuck / brace poses all included)."""
        up = self.probe_up()
        fr = self.frame
        radial = self.d["plane"] == "radial"
        for l in self.legs:
            if not l.planted:
                if radial and l.swing is not None:
                    sw = l.swing
                    w = smoothstep(sw.t / 0.15) * smoothstep((0.94 - sw.t) / 0.14)
                    if w > 1e-4:
                        # Keep radial swinging feet outside the mount's coxa
                        # cone and lateral torso clearance so legs do not fold
                        # underneath the belly when pivoting quickly.
                        rel = fr.to_local(l.foot - l.mount_world)
                        rd = l.ik.rest_dir_local
                        out_d = rel.x * rd.x + rel.z * rd.z
                        side_d = abs(rel.x * -rd.z + rel.z * rd.x)
                        rest_r = math.hypot(l.rest_xz[0] - l.mount_local.x, l.rest_xz[1] - l.mount_local.z)
                        cone_out = side_d / math.tan(max(0.25, l.ik.coxa_max - 0.08)) if l.ik.coxa_max < 1.5 else 0.0
                        need_out = max(0.46 * rest_r, min(0.85 * rest_r, cone_out))
                        if out_d < need_out:
                            l.foot = l.foot + fr.to_world(rd) * (w * (need_out - out_d))
                        foot_l = fr.to_local(l.foot - self.pos)
                        side_sign = 1.0 if l.rest_xz[0] >= 0.0 else -1.0
                        min_lat = max(0.35, 0.70 * abs(l.mount_local.x))
                        if foot_l.x * side_sign < min_lat:
                            l.foot = l.foot + fr.to_world(V3(side_sign, 0.0, 0.0)) * (w * (min_lat - foot_l.x * side_sign))
                if l.swing is not None:
                    self._clamp_swing_lat(l, fr)
                if l.swing is not None and 0.05 < l.swing.t < 0.92:
                    sw = l.swing
                    ws = smoothstep(sw.t / 0.12) * smoothstep((0.94 - sw.t) / 0.14)
                    min_sib = max(0.42, 2.8 * l.foot_r)
                    for nb in self.legs:
                        if nb.side != l.side or nb is l:
                            continue
                        if l.ik.mode == "sagittal":
                            # Keep sagittal legs in their forward/back plane without lateral sideways kicks.
                            df = (l.foot - nb.foot).dot(fr.fwd)
                            ad = abs(df)
                            if ad < min_sib:
                                sgn = (1.0 if df >= 0.0 else -1.0) if ad > 1e-4 else (1.0 if l.rest_xz[1] >= nb.rest_xz[1] else -1.0)
                                push_d = (min_sib - ad) * (0.55 if not nb.planted else 0.90) * ws
                                l.foot = l.foot + fr.fwd * (sgn * push_d)
                        else:
                            dv = project_on_plane(l.foot - nb.foot, up)
                            dl2 = dv.length_sq()
                            if 1e-8 < dl2 < min_sib * min_sib:
                                dl = math.sqrt(dl2)
                                push_d = (min_sib - dl) * (0.55 if not nb.planted else 0.90) * ws
                                l.foot = l.foot + dv * (push_d / dl)
                                self._clamp_swing_lat(l, fr)
                l.keep_clear(self.world, up, max_step=self.d["step_max"], max_ceil_y=self.ceil_y, dt=dt)

    def _ease_foot(self, l, target, k):
        """Ease a lifted foot towards a pose in body space."""
        fr = self.frame
        fl = fr.to_local(l.foot - self.pos)
        tl = fr.to_local(target - self.pos)
        l.foot = self.pos + fr.to_world(fl.lerp(tl, k))

    def _legs_air(self, dt):
        """Jump / fall legs: push off, tuck in mid-air, and brace onto landing slope."""
        fr = self.frame
        b = self._brace_weight() if self.state == AIR else 0.0
        k = damp_factor(12.0 + 24.0 * b, dt) if b > 0.0 else damp_factor(10.0, dt)
        imp = self.impact
        hx, hy, hz = self.d["body_half"]
        land_fr = None
        n = WORLD_UP
        spread = 1.0
        if b > 0.0 and imp is not None:
            n = imp[2]
            land_fr = Frame(self.heading_fwd(), n)
            tilt = angle_between(fr.up, n)
            spread = 1.0 + 0.25 * clamp(imp[3] / 15.0, 0.0, 1.0) + 0.18 * clamp(tilt / 0.8, 0.0, 1.0)
        for l in self.legs:
            l.swing = None
            l.planted = False
            l.searching = False
            if l.push:
                if l.stretch < 0.95 and self.state_t < 0.25:
                    continue
                l.push = False
            tuck = V3(l.rest_xz[0] * 0.8 + l.mount_local.x * 0.2, -self.ride * 0.72, l.rest_xz[1] * 0.85)
            pose = self.pos + fr.to_world(tuck)
            if land_fr is not None:
                rel_land = V3(l.rest_xz[0] * spread, -self.ride * 1.05, l.rest_xz[1] * (1.0 + (spread - 1.0) * 0.5))
                brace = self.pos + land_fr.to_world(rel_land)
                bl = fr.to_local(brace - self.pos)
                if abs(bl.x) < hx + 0.15 and abs(bl.y) < hy + 0.15 and abs(bl.z) < hz + 0.15:
                    if self.d["plane"] == "sagittal":
                        bl.y = min(bl.y, -(hy + 0.18))
                    else:
                        sgn_x = 1.0 if l.mount_local.x >= 0.0 else -1.0
                        bl.x = sgn_x * max(abs(bl.x), hx + 0.22)
                    brace = self.pos + fr.to_world(bl)
                mount_w = self.pos + fr.to_world(l.mount_local)
                dv = brace - mount_w
                dist = dv.length()
                min_r = 0.30 * l.reach
                max_r = 0.94 * l.reach
                if dist < min_r:
                    dir_v = (dv * (1.0 / dist)) if dist > 1e-4 else (n * -1.0)
                    brace = mount_w + dir_v * min_r
                elif dist > max_r:
                    brace = mount_w + dv * (max_r / dist)
                pose = pose.lerp(brace, b)
            self._ease_foot(l, pose, k)

    def _clearance(self):
        d = self.d
        if self.state == MANTLE:
            return d["mantle_max"] + 0.6
        return d["step_max"] + d["step_height"] + 0.4
