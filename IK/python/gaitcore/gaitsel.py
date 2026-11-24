"""Situation-aware gait selection and smooth gait transitions.

The gait is picked from the situation, not from the mode alone
(docs/MECH_GAIT_DESIGN.md, section 3a).

Ladder (GaitSelector)
    Each machine has a ladder of gaits from the steadiest to the fastest:
    static (crawl / wave) -> walk (walk / ripple / metachronal) -> fast (fast
    trot / tripod / tetrapod) -> gallop (bound).  Speed picks the rung: a gait
    hands over to the next one above 90 % of what its legs carry and takes
    over again below 75 % (like an animal: walk, trot, gallop).  The gallop
    is only entered running and fast: above 55 % of the run speed (back below
    42 %).  Slow mode may use every rung but the gallop, walk mode starts at
    the walk, run mode climbs from the walk through the fast gait to the
    gallop as it speeds up.

Situation
    Rough ground ahead or underfoot, a slope over 20 / 28 deg, a drop ahead
    or lost footing cap the rung, and the speed with it (a
    gait never runs past what its legs carry): a running mech brakes for
    rubble, changes down, and picks up again after it.  The look-ahead grows
    with speed and what it sees is held until the mech gets there, so a fast
    mech changes down before the stairs, not on them.  It never changes down
    into a gait that can't carry the speed it still has: the gait follows the
    speed down a rung at a time.

No flapping
    Speed thresholds have hysteresis; a gait is held for at least about a
    cycle (a downshift forced by the terrain may come sooner); a change up
    needs the ground to clear the relaxed thresholds; a cap lifts
    one rung at a time, only after the ground has been better for 1.5 s
    without a break, and waits longer each time it comes straight back.

Smooth transitions (GaitBlend)
    A change blends over about one gait cycle: the legs' phase offsets slide
    around the cycle, duty, stride, cycle limits, step height, reach, hop
    height and stride pitch interpolate.  Leg groups, phase locking and pair
    sync take the new gait's values near the end of the blend; until then the
    feet that must stay down follow the duty the legs actually run.  Changing
    up holds the speed near what the old gait carries until the new one has
    blended in; Mech._sync_groups pulls the new gait's groups together.
"""
import math
from .gaits import Gait, phase_lerp
from .vmath import V3, WORLD_UP, clamp, smoothstep, fit_plane, project_on_plane, rotate_axis, turn_displacement

# situation -> highest rung allowed (harsh is 0..1, see Situation)
H_GALLOP = 0.15     # above: no gallop, no hops in the fast gait
H_FAST = 0.40       # above: no fast gait (walk at most)
H_WALK = 0.70       # above: static gait only
SLOPE_FAST = 20.0   # deg: steeper than this, no gallop (stairs; a 16 deg ramp is fine)
SLOPE_WALK = 38.0   # deg: steeper than this, walk at most
RELAX_T = 1.15      # s: the ground must be better this long before a cap lifts a rung ...
RELAX_T_MAX = 1.8   # ... doubling up to this when the cap keeps coming back

# ------------------------------------------------------------------ helpers
_GAIT_FIELDS = Gait.__slots__


def _copy_gait(g):
    c = Gait.__new__(Gait)
    for f in _GAIT_FIELDS:
        v = getattr(g, f)
        setattr(c, f, list(v) if isinstance(v, list) else v)
    return c


def _lerp(a, b, s):
    return a + (b - a) * s


# ------------------------------------------------------------------ situation
class Situation:
    """Smoothed picture of the ground and the mech's footing, 0 = easy."""

    def __init__(self):
        self.ahead_rough = 0.0   # m: unevenness of the ground ahead (residual about a plane)
        self.ahead_slope = 0.0   # deg
        self.ahead_void = 0.0    # share of probes ahead that found no ground near the stance
        self.under_rough = 0.0   # m: unevenness of the feet about the stance plane
        self.clutter = 0.0       # share of foothold searches that needed a fallback
        self.edge = 0.0          # legs finding nothing to stand on (0..1)
        self.harsh = 0.0         # combined, 0..1
        self.slope = 0.0         # deg, stance plane and ahead, the larger
        self.stance_slope = 0.0  # deg, plane through the feet
        self.reason = ""
        self._n = 0
        self._probe_t = 1.0
        self._probe_raw = (0.0, 0.0, 0.0, 0.0)
        self._probe_prev = (0.0, 0.0, 0.0, 0.0)
        self._hold = [0.0, 0.0, 0.0]

    def _probe_ahead(self, m):
        """Ground probes ahead along the travel (0.8 s worth at speed, at
        least ~2.5 hull lengths): three small clusters of five probes at a
        third, two thirds and the full look-ahead distance.  Roughness is the
        median of the unevenness *within* each cluster (rubble is uneven
        everywhere; a single ledge or stair face only spoils one cluster).  The
        slope is the steepest local climb: the rise across a cluster along
        the travel when it is 0.45 m or more, or the rise between neighbouring
        clusters.  (Not a plane through a cluster's five probes: that can't
        tell a tilted rubble slab from a staircase, both come out flat-ish and
        20-27 deg.)  Probes that find no ground, or ground further down than a
        walkable slope from the stance explains, mean a drop."""
        w = m.world
        up = WORLD_UP
        if m.speed > 1.0:
            fwd = project_on_plane(m.vt, up).normalized(m.heading_fwd())
        else:
            fwd = m.heading_fwd()
        curvature = -m.yaw_rate / m.speed if m.speed > 1.0 and m.galloping_fast() and not m.drifting else 0.0
        hz = m.d["body_half"][2]
        sm = m.d["step_max"]
        decel = max(6.0, 1.4 * m.d.get("accel_run", m.d["accel"]))
        brake_d = (m.speed * m.speed) / (2.0 * decel)
        min_l = max(2.5 * hz, 9.0) if m.selector.cap < 3 else (2.5 * hz)
        L = clamp(max(m.speed * 0.65, brake_d + 0.35 * m.speed), min_l, 42.0)
        spread = clamp(0.45 * hz + 0.40, 0.70, 1.15)    # cluster radius ~ a foot's working area
        g0 = w.raycast(V3(m.pos.x, m.pos.y + 1.0, m.pos.z), up * -1.0, m.ride + 3.0)
        base = g0.point.dot(up) if (g0 is not None and g0.normal.dot(up) > 0.64) else (m.pos.dot(up) - m.ride)
        ceil_hit = w.raycast(m.pos, up, 3.5)
        max_o_y = (ceil_hit.point.dot(up) - 0.15) if ceil_hit is not None else None
        void = 0
        n = 0
        cl_rough = []
        intra_slopes = []
        inter_slopes = []
        prev_h, prev_d = base, 0.0
        prev_flat = g0 is None or g0.normal.dot(up) >= 0.96
        for a in (0.22, 0.56, 1.0):
            dist = hz + a * L
            angle = curvature * dist
            centre = m.pos + turn_displacement(fwd * dist, up, angle)
            tangent = rotate_axis(fwd, up, angle) if curvature else fwd
            right = up.cross(tangent)
            hits = []
            steep_hits = 0
            for dx, dz in ((0.0, 0.0), (1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)):
                p = centre + right * (dx * spread) + tangent * (dz * spread)
                o_y = (prev_h if prev_h is not None else base) + 2.6 + sm
                if max_o_y is not None and o_y > max_o_y:
                    o_y = max_o_y
                o = V3(p.x, o_y, p.z)
                ray_d = max(1.0, o_y - (base - 2.0 * sm - 0.5 * dist))
                h = w.raycast(o, up * -1.0, ray_d)
                n += 1
                # a drop: no ground, or ground further down than a walkable
                # slope from here explains (a ramp down 30 m ahead isn't a cliff)
                if h is None or h.point.dot(up) < base - 2.0 * sm - 0.5 * dist:
                    void += 1
                    continue
                if h.normal.dot(up) < 0.55:
                    if h.point.dot(up) > base + 0.55 * sm:
                        steep_hits += 1
                    continue
                hits.append((dx, dz, h.point, h.normal))
            # ... or far below the rest of its cluster: a probe off the side of
            # a narrow bridge hits the floor 7 m down; that is the bridge's
            # edge, not rough ground
            if hits:
                ys_all = sorted(hq[2].dot(up) for hq in hits)
                ref_y = ys_all[-1] if (ys_all[-1] - ys_all[0] > 3.5 * sm) else ys_all[len(ys_all) // 2]
                keep = [hq for hq in hits if hq[2].dot(up) >= ref_y - 2.0 * sm]
                void += len(hits) - len(keep)
                hits = keep
            two_level = False
            step_h = 0.0
            if len(hits) >= 2:
                ys_h = [hq[2].dot(up) for hq in hits]
                y_lo, y_hi = min(ys_h), max(ys_h)
                step_h = y_hi - y_lo
                if (all(hq[3].dot(up) >= 0.98 for hq in hits)
                        and all((y - y_lo <= 0.05) or (y_hi - y <= 0.05) for y in ys_h)):
                    two_level = True
            cl = [V3(hq[2].x - m.pos.x, hq[2].y, hq[2].z - m.pos.z) for hq in hits]
            if len(cl) < 4:
                if steep_hits >= 2:
                    cl_rough.append(0.24 * sm)
                elif steep_hits == 1:
                    cl_rough.append(0.15 * sm)
                elif len(hits) >= 2:
                    if two_level:
                        cl_rough.append(0.0 if step_h <= sm + 0.05 else min(0.155 * sm, 0.5 * step_h))
                    elif step_h > 0.16 * sm:
                        cl_rough.append(0.5 * step_h)
                prev_h, prev_d = None, dist
                prev_flat = False
                continue
            by_pt = {(hq[0], hq[1]): hq[2].dot(up) for hq in hits}
            by_n = {(hq[0], hq[1]): hq[3] for hq in hits}
            med_ny = sorted(hq[3].dot(up) for hq in hits)[len(hits) // 2]
            min_ny = min(hq[3].dot(up) for hq in hits)
            nr_vals = [hq[3].dot(right) for hq in hits]
            is_smooth_ramp = (
                (0.0, 0.0) in by_pt and (1.0, 0.0) in by_pt and (-1.0, 0.0) in by_pt
                and abs(by_pt[(1.0, 0.0)] - by_pt[(0.0, 0.0)]) < 0.04
                and abs(by_pt[(-1.0, 0.0)] - by_pt[(0.0, 0.0)]) < 0.04
                and by_n[(1.0, 0.0)].dot(by_n[(0.0, 0.0)]) > 0.995
                and by_n[(-1.0, 0.0)].dot(by_n[(0.0, 0.0)]) > 0.995
                and min_ny < 0.985
                and (max(nr_vals) - min(nr_vals)) < 0.08
            )
            # unevenness about the cluster's own plane (a smooth slope isn't rough)
            c, nrm = fit_plane(cl)
            if nrm is None:
                continue
            res = math.sqrt(sum(((q - c).dot(nrm)) ** 2 for q in cl) / len(cl))
            if two_level and steep_hits == 0:
                res = 0.0 if step_h <= sm + 0.05 else min(res, 0.155 * sm)
            if steep_hits >= 2:
                res = max(res, 0.24 * sm)
            elif steep_hits == 1:
                res = max(res, 0.15 * sm)
            cl_rough.append(0.0 if (is_smooth_ramp and steep_hits == 0) else res)
            if is_smooth_ramp and med_ny < 0.985:
                intra_slopes.append(math.degrees(math.acos(clamp(med_ny, -1.0, 1.0))))
            # local slope: steady monotonic ramp/stairs across back/mid/fwd, ignoring isolated step lips
            if (0.0, -1.0) in by_pt and (0.0, 0.0) in by_pt and (0.0, 1.0) in by_pt:
                h_back, h_mid, h_fwd = by_pt[(0.0, -1.0)], by_pt[(0.0, 0.0)], by_pt[(0.0, 1.0)]
                dh = h_fwd - h_back
                if 0.45 <= abs(dh) <= max(1.2 * sm, 1.6 * spread):
                    frac = (h_mid - h_back) / dh
                    if 0.20 <= frac <= 0.80:
                        intra_slopes.append(math.degrees(math.atan2(abs(dh), 2.0 * spread)))
            hm = sorted(q.y for q in cl)[len(cl) // 2]
            cur_flat = med_ny >= 0.985 and min_ny >= 0.98 and res < 0.05
            if prev_h is not None:
                if prev_flat and cur_flat and (prev_h - hm) > 2.5 * sm:
                    # sheer cliff / platform edge onto another flat level: void drop, not a slope
                    void += 2
                else:
                    dx = max(2.5, dist - prev_d)
                    dh_inter = abs(hm - prev_h)
                    if dh_inter <= 0.76 * dx:
                        inter_slopes.append(math.degrees(math.atan2(dh_inter, dx)))
            prev_h, prev_d = hm, dist
            prev_flat = cur_flat
        slope = max(inter_slopes) if inter_slopes else 0.0
        if intra_slopes:
            slope = max(slope, max(intra_slopes))
        if cl_rough:
            cl_rough.sort()
            mid = cl_rough[len(cl_rough) // 2] if len(cl_rough) >= 3 else (cl_rough[0] if len(cl_rough) == 2 else 0.0)
            peek_cap = min(0.055, 0.075 * sm) + 0.36 * 0.32 * sm
            rough = max(mid, min(0.45 * cl_rough[-1], peek_cap))
        else:
            rough = 0.0
        return rough, slope, void / float(n), hz + L

    def update(self, m, dt):
        self._n += 1
        p = m.probe_period
        self._probe_t += dt
        if self._probe_t >= p - 1e-6:
            self._probe_t = 0.0 if self._probe_t >= 2.0 * p else (self._probe_t % p)
            self._probe_prev = self._probe_raw
            self._probe_raw = self._probe_ahead(m)
        raw, prev = self._probe_raw, self._probe_prev
        # rise quickly, clear promptly once the terrain opens back up
        def ema(cur, x, up_tau=0.3, down_tau=0.45):
            tau = up_tau if x > cur else down_tau
            return cur + (x - cur) * (1.0 - math.exp(-dt / tau))
        hold = min(0.65, (raw[3] + m.d["body_half"][2]) / max(m.speed, 3.0))
        vals = []
        for i, cur in enumerate((self.ahead_rough, self.ahead_slope, self.ahead_void)):
            seen = min(raw[i], prev[i])
            if seen >= cur:
                cur = ema(cur, seen, up_tau=0.06, down_tau=0.45) if i == 0 else seen
                self._hold[i] = hold
            else:
                if self._hold[i] > 0.0:
                    decay = 3.0 * dt if (abs(m.yaw_rate) > 0.45 or raw[i] < 0.5 * cur) else dt
                    self._hold[i] -= decay
                if self._hold[i] <= 0.0 or (i != 0 and raw[i] > cur):
                    cur = ema(cur, raw[i])
            vals.append(cur)
        self.ahead_rough, self.ahead_slope, self.ahead_void = vals
        self.under_rough = ema(self.under_rough, m.foot_rms, 0.45, 0.45)
        tried = m.fh_count
        bad = m.fh_bad
        m.fh_count = m.fh_bad = 0
        if tried > 0:
            self.clutter = ema(self.clutter, bad / tried, 0.25, 0.45)
        else:
            self.clutter = ema(self.clutter, 0.0, 0.25, 0.70)
        edge = 1.0 if any(l.searching or l.void_mark > 0.0 for l in m.legs) else 0.0
        self.edge = ema(self.edge, edge, 0.1, 0.45)
        stance_slope = 0.0
        if m.plane_n_local is not None:
            stance_slope = math.degrees(math.acos(clamp(m.plane_n_local.y, -1.0, 1.0)))
        self.stance_slope = ema(self.stance_slope, stance_slope)
        self.slope = max(self.ahead_slope, self.stance_slope)
        # roughness relative to what the machine steps over comfortably
        sm = m.d["step_max"]
        base_r = min(0.055, 0.075 * sm)
        ahead_n = clamp((self.ahead_rough - base_r) / (0.32 * sm), 0.0, 1.0)
        under_n = clamp((0.85 * self.under_rough - base_r) / (0.32 * sm), 0.0, 0.38)
        rough_n = max(ahead_n, under_n)
        # clutter (fallback footholds) can force a walk, not a static gait:
        # that takes rough ground the probes can measure
        clutter_n = clamp((self.clutter - 0.30) / 0.65, 0.0, 0.38)
        parts = [(rough_n, "rough ground"), (clutter_n, "cluttered footing")]
        self.harsh, self.reason = max(parts, key=lambda p: p[0])


# ------------------------------------------------------------------ selector
class GaitSelector:
    """Picks the target gait each frame (see module docstring)."""

    def __init__(self, m):
        d = m.d
        g = d["gait_by_mode"]
        fast = d.get("walk_fast") or d.get("drift_gait") or g["walk"]
        self.ladder = [g["slow"], g["walk"], fast, g["run"]]
        self.rung = 1
        self.dwell = 10.0
        self.reason = ""
        self.cap = 3            # highest rung the situation allows (hysteresis)
        self.low_t = 0.0        # time the speed has been below the current rung's
        self.cap_why = ""
        self.cap_t = 10.0
        self.hold_v = None      # on an upshift: what the old gait carried (held while blending)
        self.relax_wait = RELAX_T    # s the ground must be better before a cap lifts a rung
        self.since_relax = 10.0
        self.relaxed = False
        self.better_t = 0.0     # s the ground has allowed a looser cap without a break
        self._prev_v = 0.0
        self._accel_v = 0.0

    def caps(self, m):
        """Speed each rung carries (m/s)."""
        sp = m.d["speeds"]
        return [m.v_slow, m.v_walk, sp["walk"], sp["run"]]

    @staticmethod
    def _cap(m, sit, relax):
        """Highest rung the situation allows, and why.  relax = 1 uses the
        thresholds for lifting a cap (hysteresis)."""
        h = sit.harsh + 0.12 * relax
        cap_r, why = 3, ""
        if sit.harsh + 0.28 * relax > H_WALK:
            cap_r, why = 0, sit.reason
        elif h > H_FAST:
            cap_r, why = 1, sit.reason
        elif h > H_GALLOP:
            cap_r, why = 2, sit.reason
        if sit.slope + 4.0 * relax > SLOPE_WALK and cap_r > 1:
            cap_r, why = 1, "steep slope"
        elif sit.slope + 4.0 * relax > SLOPE_FAST and cap_r > 2:
            cap_r, why = 2, "steep slope"
        if (max(sit.edge, sit.ahead_void) + 0.15 * relax > 0.3) and cap_r > 2:
            # a drop beside the path ends the gallop (no leaps next to it) but
            # doesn't force a walk: these wide machines only keep their feet
            # on a 2.5 m bridge by crossing it at speed (forced to walk they
            # tipped off it, 10 falls in 28 crossings instead of 1).  A lean
            # off balance doesn't change the gait either: the balance model's
            # catch steps handle it, and a static gait's stability veto would
            # block them.
            cap_r, why = 2, "drop ahead" if sit.ahead_void >= sit.edge else "footing lost"
        return cap_r, why

    def select(self, m, dt):
        """Returns (gait name, reason, speed cap or None)."""
        d = m.d
        self.dwell += dt
        sit = m.situation
        v = m.speed
        inst_a = clamp((v - self._prev_v) / max(dt, 1e-4), -30.0, 30.0)
        self._prev_v = v
        self._accel_v += (inst_a - self._accel_v) * (1.0 - math.exp(-dt / 0.15))
        if m.gait_override or m.drifting or m.climbing_mode:
            self.hold_v = None
        if m.gait_override:
            self.reason = "forced"
            return m.gait_override, self.reason, None
        if m.drifting:
            self.reason = "slide"
            return d.get("drift_gait", self.ladder[1]), self.reason, None
        if m.climbing_mode:
            self.reason = "climbing"
            return self.ladder[1], self.reason, None
        caps = self.caps(m)
        sp = d["speeds"]
        lo = 0 if m.mode == "slow" else 1
        hi = 3 if m.mode == "run" else 2
        # --- speed picks the rung (hysteresis)
        up = [0.9 * caps[0], 0.9 * caps[1], 0.55 * sp["run"]]
        down = [0.75 * caps[0], 0.75 * caps[1], 0.42 * sp["run"]]
        r = int(clamp(self.rung, lo, hi))
        v = m.speed
        # change down only when the speed asked for is low too: a brief dip
        # while the throttle is held (a transition, a bump) isn't a reason,
        # but being held back for half a second is (stairs, a slope)
        if r > lo and v < down[r - 1]:
            if r < 3 and m.v_desired >= down[r - 1] and self._accel_v > 1.0:
                self.low_t = max(0.0, self.low_t - dt)
            else:
                self.low_t += dt
        else:
            self.low_t = 0.0
        low_lim = 0.15 if (r == 3 and v < 0.40 * sp["run"]) else 0.85
        v_dn = v if self.low_t > low_lim else max(v, 0.9 * m.v_desired)
        while r < hi and v > up[r]:
            r += 1
        while r > lo and v_dn < down[r - 1]:
            r -= 1
        reason = ("galloping" if r == 3 else "fast" if r == 2 else "walking" if r == 1 else "careful")
        inp = getattr(m, "inp", None)
        turning = abs(inp.turn) > 0.35 if inp is not None else False
        if getattr(m, "lateral_ratio", 0.0) > 1.6 and not turning and r > 2:
            r = 2
            reason = "strafing"
        # --- the situation caps it.  A tighter cap applies at once (safety);
        # a cap relaxes one rung at a time, only once the ground has been
        # clearly better (relaxed thresholds) for relax_wait without a break.
        enter, why_in = self._cap(m, sit, 0.0)
        exit_, _ = self._cap(m, sit, 1.0)
        self.cap_t += dt
        self.since_relax += dt
        self.better_t = self.better_t + dt if exit_ > self.cap else 0.0
        if self.since_relax > 2.0 * RELAX_T_MAX:
            self.relax_wait = RELAX_T
        wait = 0.60 * self.relax_wait if (self.cap == 0 and exit_ >= 2 and sit.harsh < 0.20) else self.relax_wait
        if enter < self.cap:
            if self.relaxed and self.since_relax < 3.0:
                # back in soon after lifting: ground that alternates (a
                # boulder field) waits longer each time (no flapping)
                self.relax_wait = min(RELAX_T_MAX, 2.0 * self.relax_wait)
            self.cap, self.cap_why, self.cap_t, self.relaxed = enter, why_in, 0.0, False
            self.better_t = 0.0
        elif self.better_t > wait:
            # the ground has been clearly better for the whole wait
            self.cap, self.cap_t, self.since_relax, self.relaxed = self.cap + 1, 0.0, 0.0, True
            self.better_t = 0.0
            if self.cap >= 3:
                self.cap_why = ""
        cap_r, why = self.cap, self.cap_why
        speed_cap = None
        if r > self.rung and r > exit_:
            # changing up only with the same margin a cap needs to lift: a
            # 16 deg ramp reads 16-21 deg around its crest, and a gallop picked
            # up at 16 was dropped again 6 m on
            r = max(self.rung, exit_)
        if r > cap_r:
            if self.rung > cap_r and enter > cap_r:
                # still above the cap (the speed held the gait while braking)
                # and the ground is already better again: changing down now
                # would only last until the cap lifts
                r = min(r, self.rung)
            else:
                r = cap_r
            reason = why
        # ... but never down into a gait that can't carry the speed the mech
        # still has: the speed cap brakes it, the gait follows the speed down
        # one rung at a time (walking legs at gallop speed over-stretch at
        # once and brake the hull from 30 to 6 m/s in a tenth of a second)
        floor = 0
        while floor < 3 and v > 1.1 * caps[floor]:
            floor += 1
        if r < self.rung and floor > r:
            r = min(self.rung, floor)
        m.calm_gait = (cap_r < 3 or sit.edge > 0.3 or getattr(m, "crouch", 0.0) > 0.5) and r < 3     # the fast gait without hops
        if getattr(m, "crouch", 0.0) > 0.5:
            m.calm_gait = True
        if cap_r < 3:
            # a gait never runs past what its legs carry, and on the ground
            # that capped it the walking gaits keep a margin (at 90 % of
            # their reach on rubble, feet hit their reach right after landing
            # and step again: quick re-steps 6.4 -> 8.8 per 100 in the ripple)
            if cap_r < 2:
                speed_cap = 0.75 * caps[cap_r]
            else:
                chord = getattr(m, "chord_avg", None)
                if chord is not None:
                    grounded_cap = 0.85 * chord / max(d["swing_min"], 0.10)
                    speed_cap = min(caps[2], max(1.35 * caps[1], grounded_cap))
                else:
                    speed_cap = caps[2]
                if sit.slope > SLOPE_FAST:
                    slope_k = 15.5 if sit.harsh < 0.08 else 10.5
                    slope_cap = (slope_k * m.d["step_max"]) / max(0.25, math.tan(math.radians(sit.slope)))
                    speed_cap = min(speed_cap, max(1.35 * caps[1], slope_cap))
        # --- no flapping: hold a gait about a cycle (a terrain downshift may come sooner)
        if r != self.rung:
            min_dwell = clamp(0.9 * m.cycle, 0.55, 0.85)
            safety = ((r < self.rung and r == cap_r and why != "")
                      or (self.rung == 3 and r < 3 and v < 0.40 * sp["run"]))
            if self.dwell < min_dwell and not (safety and self.dwell > 0.15):
                r = self.rung
            else:
                # changing up: the speed waits near what the old gait carries
                # until the new one has blended in (it can't run past its
                # legs half way through the change)
                self.hold_v = caps[self.rung] if r > self.rung and self.rung < 2 else None
                self.rung = r
                self.dwell = 0.0
        self.reason = reason
        return self.ladder[r], reason, speed_cap


# ------------------------------------------------------------------ blending
class GaitBlend:
    """The gait the scheduler runs on, blended between two table gaits."""

    LERP = ("duty", "stride", "cycle_min", "cycle_max", "lift", "reach", "leap", "stride_pitch")

    def __init__(self, g):
        self.src = _copy_gait(g)
        self.dst = g
        self.t = 1.0
        self.s = 1.0
        self.T = 0.5
        self.eff = _copy_gait(g)

    @property
    def blending(self):
        return self.t < 1.0

    def start(self, dst, T):
        """Blend from wherever the gait is now (also mid-blend) to dst over T s."""
        self.src = _copy_gait(self.eff)
        self.dst = dst
        self.t = 0.0
        self.T = max(T, 0.05)

    def update(self, dt):
        self.t = min(1.0, self.t + dt / self.T)
        s = self.s = smoothstep(self.t)
        e, a, b = self.eff, self.src, self.dst
        for f in self.LERP:
            setattr(e, f, _lerp(getattr(a, f), getattr(b, f), s))
        e.offsets = [phase_lerp(x, y, s) for x, y in zip(a.offsets, b.offsets)]
        done = s >= 0.85
        e.name = b.name if done else a.name
        # the rules that can't be halfway stay on the safe side until then
        # (min_support: see fit_support)
        e.min_support = b.min_support if done else max(a.min_support, b.min_support)
        e.static_stable = b.static_stable if done else (a.static_stable or b.static_stable)
        e.adjacent_lock = b.adjacent_lock if done else (a.adjacent_lock and b.adjacent_lock)
        e.groups = b.groups if done else None
        e.pair_sync = b.pair_sync if done else False
        e.phase_lock = b.phase_lock if done else False
        e.duty_fast = b.duty_fast
        return e

    def fit_support(self, duty):
        """While blending, the feet that must stay down follow the duty the
        legs actually run (the tables have min_support ~ floor(duty * legs)):
        the safe side of both gaits with the duty already halfway to the fast
        gait's vetoes every lift, the feet over-stretch and brake the hull."""
        if self.t >= 1.0 or smoothstep(self.t) >= 0.85:
            return
        a, b, e = self.src, self.dst, self.eff
        lo, hi = min(a.min_support, b.min_support), max(a.min_support, b.min_support)
        e.min_support = int(clamp(int(duty * len(e.offsets) + 0.05), lo, hi))

    def label(self):
        if self.t >= 1.0:
            return self.dst.name
        return "%s > %s" % (self.src.name, self.dst.name)
