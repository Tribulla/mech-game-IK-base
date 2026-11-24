"""Leg inverse kinematics.

Mech legs are solved as [optional coxa yaw] + a planar hinge chain.
Hinges only bend inside the leg plane, matching physical linkages and
avoiding the uncontrolled ball-joint twist of unconstrained 3D FABRIK.

Two plane modes:
  * "radial"   - insect / spider style.  A coxa joint yaws the leg plane around
                 the body up axis towards the foot, then femur/tibia/tarsus hinge
                 in the vertical plane through the foot.
  * "sagittal" - biped / quadruped style.  The plane always contains the body
                 forward axis; a hip-roll rotates it sideways to reach the foot.

Inside the plane:
  * 2 segments -> exact analytic law-of-cosines solve.
  * 3+ segments -> 2D FABRIK, warm-started from last frame, with per-joint
                 "bend side" enforcement so knees never flip (digitigrade legs
                 keep the knee forward / ankle back etc.).
  * joint limits -> if that solution breaks a limit (coxa yaw, hip roll, hip
                 pitch, bend range per knee), an active-set damped least-squares
                 (`_dls`) solve takes over from the last legal pose (see LegIK).
"""
import math
from .vmath import V3, LOCAL_FRAME, clamp

TAU = math.pi * 2.0


def solve_two_bone(l1, l2, tx, ty, side):
    """Analytic 2-bone planar IK.  Root at origin.

    side=+1 puts the middle joint counter-clockwise of the root->target line.
    Returns (kx, ky, fx, fy, reached).
    """
    d = math.hypot(tx, ty)
    reached = True
    dmax = l1 + l2 - 1e-5
    dmin = abs(l1 - l2) + 1e-5
    if d > dmax:
        reached = d <= dmax + 1e-3
        d = dmax
    elif d < dmin:
        d = dmin
    base = math.atan2(ty, tx)
    cos_a = clamp((l1 * l1 + d * d - l2 * l2) / (2.0 * l1 * d), -1.0, 1.0)
    a = base + side * math.acos(cos_a)
    kx = l1 * math.cos(a)
    ky = l1 * math.sin(a)
    # foot = knee + l2 * dir(knee -> target), so the chain stays rigid even if clamped
    dx, dy = tx - kx, ty - ky
    dl = math.hypot(dx, dy) or 1e-9
    fx = kx + dx / dl * l2
    fy = ky + dy / dl * l2
    return kx, ky, fx, fy, reached


def _reflect_across_line(px, py, lx, ly):
    """Reflect point p across the line through origin with unit direction (lx, ly)."""
    d = px * lx + py * ly
    return 2.0 * d * lx - px, 2.0 * d * ly - py


def solve_fabrik_2d(lengths, pts, tx, ty, sides, iters=10, tol=1e-3):
    """Planar FABRIK.  pts: list of [x, y] (len(lengths)+1), pts[0] is the fixed root (0,0).

    `sides[i]` (+1/-1) is the required side of interior joint i+1 relative to the
    root->target line.  Mutates pts in place, returns True if target reached.
    """
    n = len(lengths)
    total = sum(lengths)
    d = math.hypot(tx, ty)
    if d < 1e-6:
        tx, ty, d = 1e-6, 0.0, 1e-6
    lx, ly = tx / d, ty / d

    if d >= total * 0.999:
        # Out of reach: keep the configured bend just barely so it reads as a
        # straining leg rather than snapping to a perfectly straight rod.
        acc = 0.0
        for i in range(n):
            acc += lengths[i]
            pts[i + 1][0] = lx * acc
            pts[i + 1][1] = ly * acc
        return d <= total * 1.001

    for _ in range(iters):
        # bend side enforcement (before the passes so lengths get restored)
        for j in range(1, n):
            px, py = pts[j]
            s = lx * py - ly * px
            if s * sides[j - 1] < 0.0:
                pts[j][0], pts[j][1] = _reflect_across_line(px, py, lx, ly)
        # backward pass
        pts[n][0], pts[n][1] = tx, ty
        for i in range(n - 1, -1, -1):
            dx = pts[i][0] - pts[i + 1][0]
            dy = pts[i][1] - pts[i + 1][1]
            l = math.hypot(dx, dy) or 1e-9
            k = lengths[i] / l
            pts[i][0] = pts[i + 1][0] + dx * k
            pts[i][1] = pts[i + 1][1] + dy * k
        # forward pass
        pts[0][0], pts[0][1] = 0.0, 0.0
        for i in range(n):
            dx = pts[i + 1][0] - pts[i][0]
            dy = pts[i + 1][1] - pts[i][1]
            l = math.hypot(dx, dy) or 1e-9
            k = lengths[i] / l
            pts[i + 1][0] = pts[i][0] + dx * k
            pts[i + 1][1] = pts[i][1] + dy * k
        ex = pts[n][0] - tx
        ey = pts[n][1] - ty
        if ex * ex + ey * ey < tol * tol:
            break
    return True  # geometrically reachable (warm start converges over frames)


def initial_pose_2d(lengths, sides, tx, ty):
    """A zig-zag starting pose that already respects the bend sides."""
    d = math.hypot(tx, ty) or 1e-6
    lx, ly = tx / d, ty / d
    nx, ny = -ly, lx  # CCW normal
    pts = [[0.0, 0.0]]
    x = y = 0.0
    n = len(lengths)
    for i, L in enumerate(lengths):
        # aim each segment off the line towards the side of its far joint
        s = sides[i] if i < n - 1 else (-sides[i - 1] if i > 0 else 0)
        ang = 0.6 * s
        dx = lx * math.cos(ang) + nx * math.sin(ang)
        dy = ly * math.cos(ang) + ny * math.sin(ang)
        x += dx * L
        y += dy * L
        pts.append([x, y])
    return pts


def _wrap(a):
    return (a + math.pi) % TAU - math.pi


class LegIK:
    """Stateful solver for one leg (keeps a warm-start pose).

    Joint limits (`limits`, degrees, all optional; see mechdefs.py):
      coxa  max coxa yaw either way from the leg's rest direction (radial)
      roll  max hip roll of the leg plane away from vertical (sagittal)
      hip   (min, max) pitch of the first in-plane segment.  radial: 0 =
            horizontal outward, +90 = straight up.  sagittal: 0 = straight
            down, + = forward.
      bend  (min, max) per interior joint: bend away from straight, + = towards
            the joint's configured side.  min > 0 keeps a knee from locking
            straight or flipping, max < 180 keeps it from folding flat.

    The unconstrained solve (analytic / FABRIK) runs first; if it breaks a
    limit, an active-set damped least-squares (`_dls`) solve takes over,
    warm-started from the last legal pose.  `err` is how far the foot ended up
    from the target because of the limits (0 when reachable).
    """

    def __init__(self, plane_mode, coxa, lengths, sides, rest_dir_local, limits=None):
        self.mode = plane_mode
        self.coxa = coxa
        self.lengths = list(lengths)
        self.sides = list(sides)
        self.reach = coxa + sum(lengths)
        self.chain_reach = sum(lengths)
        self.rest_dir_local = rest_dir_local  # horizontal outward dir (radial legs)
        self.pts = None
        self.reached = True
        self.plane_normal = V3(1, 0, 0)
        self.err = 0.0
        self.limited = False      # last solve had to be constrained
        self.angles = None        # last legal pose: [hip, rel1, rel2, ...] (radians)
        self.yaw_deg = 0.0
        self.hip_deg = 0.0
        self.bends_deg = []
        self.coxa_ratio = 0.0
        self.limit_margin_deg = 90.0

        lim = limits or {}
        self.coxa_max = math.radians(lim.get("coxa", 180.0))
        self.roll_max = math.radians(lim.get("roll", 60.0))
        hip = lim.get("hip")
        if hip is None:
            self.hip_lim = (-math.pi, math.pi)
        else:
            h0, h1 = hip
            if plane_mode == "sagittal":       # authored from straight down
                h0, h1 = h0 - 90.0, h1 - 90.0
            self.hip_lim = (math.radians(h0), math.radians(h1))
        n_int = len(lengths) - 1
        bends = lim.get("bend")
        if bends is None:
            bends = [(-180.0, 180.0)] * n_int
        elif isinstance(bends[0], (int, float)):
            bends = [tuple(bends)] * n_int
        bends = list(bends) + [bends[-1]] * max(0, n_int - len(bends))
        self.bend_lim = [(math.radians(b0), math.radians(b1)) for b0, b1 in bends[:n_int]]
        # relative-angle limits: bend b = -side * rel  ->  rel in [-s*b1, -s*b0] (sorted)
        self.rel_lim = []
        for i, (b0, b1) in enumerate(self.bend_lim):
            s = self.sides[i]
            lo, hi = sorted((-s * b0, -s * b1))
            self.rel_lim.append((lo, hi))

    # ------------------------------------------------------------------ plane
    def _plane_scalars(self, mount, frame, target, clamp_yaw=True):
        """Scalar leg-plane basis: returns (rx, ry, rz, ux, uy, uz, vx, vy, vz, yaw_or_roll)."""
        up = frame.up
        upx, upy, upz = up.x, up.y, up.z
        mx, my, mz = mount.x, mount.y, mount.z
        dx, dy, dz = target.x - mx, target.y - my, target.z - mz
        if self.mode == "radial":
            if frame is LOCAL_FRAME:
                rd = self.rest_dir_local
                rdx, rdy, rdz = rd.x, rd.y, rd.z
            else:
                rd = self.rest_dir_local
                r_ax, u_ax, f_ax = frame.right, up, frame.fwd
                rdx = r_ax.x * rd.x + u_ax.x * rd.y + f_ax.x * rd.z
                rdy = r_ax.y * rd.x + u_ax.y * rd.y + f_ax.y * rd.z
                rdz = r_ax.z * rd.x + u_ax.z * rd.y + f_ax.z * rd.z
            du = dx * upx + dy * upy + dz * upz
            tx, ty, tz = dx - upx * du, dy - upy * du, dz - upz * du
            tl2 = tx * tx + ty * ty + tz * tz
            if tl2 < 1e-18:
                ux, uy, uz = rdx, rdy, rdz
            else:
                inv = 1.0 / math.sqrt(tl2)
                ux, uy, uz = tx * inv, ty * inv, tz * inv
            cx = rdy * uz - rdz * uy
            cy = rdz * ux - rdx * uz
            cz = rdx * uy - rdy * ux
            yaw = math.atan2(cx * upx + cy * upy + cz * upz, rdx * ux + rdy * uy + rdz * uz)
            if clamp_yaw and abs(yaw) > self.coxa_max:
                if self.angles is not None:
                    prev_yaw = math.radians(self.yaw_deg)
                    if abs(prev_yaw) > 0.5 * self.coxa_max and yaw * prev_yaw < 0.0 and abs(yaw) > math.pi - 0.85:
                        yaw = math.copysign(self.coxa_max, prev_yaw)
                    else:
                        yaw = clamp(yaw, -self.coxa_max, self.coxa_max)
                else:
                    yaw = clamp(yaw, -self.coxa_max, self.coxa_max)
                cy_a, sy_a = math.cos(yaw), math.sin(yaw)
                crx = upy * rdz - upz * rdy
                cry = upz * rdx - upx * rdz
                crz = upx * rdy - upy * rdx
                ux = rdx * cy_a + crx * sy_a
                uy = rdy * cy_a + cry * sy_a
                uz = rdz * cy_a + crz * sy_a
                ul2 = ux * ux + uy * uy + uz * uz
                if ul2 < 1e-18:
                    ux, uy, uz = rdx, rdy, rdz
                else:
                    inv = 1.0 / math.sqrt(ul2)
                    ux, uy, uz = ux * inv, uy * inv, uz * inv
            coxa = self.coxa
            return mx + ux * coxa, my + uy * coxa, mz + uz * coxa, ux, uy, uz, upx, upy, upz, yaw

        f = frame.fwd
        fx, fy, fz = f.x, f.y, f.z
        df = dx * fx + dy * fy + dz * fz
        px, py, pz = dx - fx * df, dy - fy * df, dz - fz * df
        pl2 = px * px + py * py + pz * pz
        roll = 0.0
        if pl2 < 1e-8:
            vx, vy, vz = upx, upy, upz
        else:
            inv = -1.0 / math.sqrt(pl2)
            vx, vy, vz = px * inv, py * inv, pz * inv
            c = math.cos(self.roll_max)
            vdu = vx * upx + vy * upy + vz * upz
            if vdu < c:
                sdx, sdy, sdz = vx - upx * vdu, vy - upy * vdu, vz - upz * vdu
                sdl2 = sdx * sdx + sdy * sdy + sdz * sdz
                if sdl2 < 1e-18:
                    fr = frame.right
                    sdx, sdy, sdz = fr.x, fr.y, fr.z
                else:
                    invs = 1.0 / math.sqrt(sdl2)
                    sdx, sdy, sdz = sdx * invs, sdy * invs, sdz * invs
                sr = math.sin(self.roll_max)
                vx = upx * c + sdx * sr
                vy = upy * c + sdy * sr
                vz = upz * c + sdz * sr
                vl2 = vx * vx + vy * vy + vz * vz
                if vl2 < 1e-18:
                    vx, vy, vz = upx, upy, upz
                else:
                    invv = 1.0 / math.sqrt(vl2)
                    vx, vy, vz = vx * invv, vy * invv, vz * invv
            roll = math.acos(clamp(vx * upx + vy * upy + vz * upz, -1.0, 1.0))
        return mx, my, mz, fx, fy, fz, vx, vy, vz, roll

    def _plane(self, mount, frame, target, clamp_yaw=True):
        """Leg plane for a target: returns (root, u, v, joints_prefix, yaw)."""
        rx, ry, rz, ux, uy, uz, vx, vy, vz, yaw = self._plane_scalars(mount, frame, target, clamp_yaw)
        root = V3(rx, ry, rz)
        joints = [mount, root] if (self.mode == "radial" and self.coxa > 1e-6) else [mount]
        return root, V3(ux, uy, uz), V3(vx, vy, vz), joints, yaw

    # ------------------------------------------------------------------ angles
    def _to_angles(self, pts2):
        ang = []
        px = py = 0.0
        prev = None
        for x, y in pts2:
            a = math.atan2(y - py, x - px)
            ang.append(a if prev is None else _wrap(a - prev))
            prev = a
            px, py = x, y
        return ang

    def _fk(self, ang):
        pts = []
        a = 0.0
        x = y = 0.0
        for i, L in enumerate(self.lengths):
            a = ang[0] if i == 0 else a + ang[i]
            x += L * math.cos(a)
            y += L * math.sin(a)
            pts.append((x, y))
        return pts

    def _legal(self, ang, tol=1e-3):
        h0, h1 = self.hip_lim
        if not (h0 - tol <= ang[0] <= h1 + tol):
            return False
        for i, (lo, hi) in enumerate(self.rel_lim):
            if not (lo - tol <= ang[i + 1] <= hi + tol):
                return False
        return True

    def _clamp(self, ang):
        out = [clamp(ang[0], *self.hip_lim)]
        for i, (lo, hi) in enumerate(self.rel_lim):
            out.append(clamp(ang[i + 1], lo, hi))
        return out

    def _dls(self, ang, tx, ty, iters=12):
        """Active-set damped least squares solve in the leg plane."""
        lengths = self.lengths
        n = len(lengths)
        lims = [self.hip_lim] + self.rel_lim
        _cos, _sin = math.cos, math.sin
        ang = self._clamp(ang)
        pts = self._fk(ang)
        ex, ey = pts[-1]
        cur_err2 = (tx - ex) ** 2 + (ty - ey) ** 2
        best_ang = list(ang)
        best_err2 = cur_err2
        lam = 0.08
        for _ in range(iters):
            if cur_err2 < 1e-7:
                break
            rx, ry = tx - ex, ty - ey
            Jx = [-(ey - (0.0 if k == 0 else pts[k - 1][1])) for k in range(n)]
            Jy = [ex - (0.0 if k == 0 else pts[k - 1][0]) for k in range(n)]
            active = [True] * n
            step = None
            for _ in range(n):
                lam2 = lam * lam
                a = c = lam2
                b = 0.0
                for k in range(n):
                    if active[k]:
                        jkx, jky = Jx[k], Jy[k]
                        a += jkx * jkx
                        b += jkx * jky
                        c += jky * jky
                det = a * c - b * b
                if det <= 1e-12:
                    step = None
                    break
                vx = (c * rx - b * ry) / det
                vy = (a * ry - b * rx) / det
                step = [(Jx[k] * vx + Jy[k] * vy) if active[k] else 0.0 for k in range(n)]
                pinned = False
                for k in range(n):
                    if not active[k]:
                        continue
                    lo, hi = lims[k]
                    sk = step[k]
                    ak = ang[k]
                    nk = ak + sk
                    applied = (lo if nk < lo else (hi if nk > hi else nk)) - ak
                    if abs(sk) > 1e-6 and abs(applied) <= 1e-6:
                        active[k] = False
                        pinned = True
                if not pinned:
                    break
                if not any(active):
                    step = None
                    break
            if step is None:
                lam = min(4.0, lam * 2.5)
                continue
            m = max(abs(s) for s in step)
            if m > 0.55:
                sc = 0.55 / m
                step = [s * sc for s in step]
            accepted = False
            for scale in (1.0, 0.5, 0.25, 0.125):
                trial = []
                aa = tex = tey = 0.0
                for k in range(n):
                    lo, hi = lims[k]
                    v = ang[k] + step[k] * scale
                    tk = lo if v < lo else (hi if v > hi else v)
                    trial.append(tk)
                    aa = tk if k == 0 else aa + tk
                    lk = lengths[k]
                    tex += lk * _cos(aa)
                    tey += lk * _sin(aa)
                terr2 = (tx - tex) ** 2 + (ty - tey) ** 2
                if terr2 < cur_err2 - 1e-10:
                    ang = trial
                    pts = self._fk(trial)
                    cur_err2 = terr2
                    ex, ey = pts[-1]
                    if terr2 < best_err2:
                        best_ang = list(ang)
                        best_err2 = terr2
                    lam = max(0.01, lam / 2.5)
                    accepted = True
                    break
            if not accepted:
                lam = min(4.0, lam * 2.5)
        return best_ang

    # ------------------------------------------------------------------ solve
    def _three_link_pose(self, tx, ty):
        """Share the bend between two hinges that fold in the same direction."""
        thigh, shin, foot = self.lengths
        # For equal bends, the squared reach is quadratic in the bend cosine.
        a = 4.0 * thigh * foot
        b = 2.0 * shin * (thigh + foot)
        c = (thigh - foot)**2 + shin**2 - tx * tx - ty * ty
        cosine = (-b + math.sqrt(max(0.0, b * b - 4.0 * a * c))) / (2.0 * a)
        ankle = -self.sides[1] * math.acos(clamp(cosine, -1.0, 1.0))
        ankle = clamp(ankle, *self.rel_lim[1])

        # With the ankle angle known, the shin and foot form one virtual link.
        dx = shin + foot * math.cos(ankle)
        dy = foot * math.sin(ankle)
        kx, ky, ex, ey, _ = solve_two_bone(thigh, math.hypot(dx, dy), tx, ty, self.sides[0])
        hip = math.atan2(ky, kx)
        knee = _wrap(math.atan2(ey - ky, ex - kx) - math.atan2(dy, dx) - hip)
        return self._fk([hip, knee, ankle])

    def _three_link_clamped_candidates(self, tx, ty, ang):
        """Exact single-limit 2-bone solves when hip or knee hits a stop."""
        thigh, shin, foot = self.lengths
        out = []
        # 1) Hip clamped -> 2-bone (shin, foot) from knee to target
        if not (self.hip_lim[0] - 1e-4 <= ang[0] <= self.hip_lim[1] + 1e-4):
            hip = clamp(ang[0], *self.hip_lim)
            kx = thigh * math.cos(hip)
            ky = thigh * math.sin(hip)
            kx2, ky2, fx2, fy2, _ = solve_two_bone(shin, foot, tx - kx, ty - ky, self.sides[1])
            a1 = math.atan2(ky2, kx2)
            a2 = math.atan2(fy2 - ky2, fx2 - kx2)
            cand = [hip, _wrap(a1 - hip), _wrap(a2 - a1)]
            out.append((cand, math.hypot(kx + fx2 - tx, ky + fy2 - ty)))
        # 2) Knee clamped -> lump (thigh, shin), 2-bone (l_up, foot) to target
        if not (self.rel_lim[0][0] - 1e-4 <= ang[1] <= self.rel_lim[0][1] + 1e-4):
            knee = clamp(ang[1], *self.rel_lim[0])
            dx = thigh + shin * math.cos(knee)
            dy = shin * math.sin(knee)
            l_up = math.hypot(dx, dy)
            if l_up > 1e-6:
                off = math.atan2(dy, dx)
                ax, ay, fx, fy, _ = solve_two_bone(l_up, foot, tx, ty, self.sides[1])
                hip = _wrap(math.atan2(ay, ax) - off)
                ankle = _wrap(math.atan2(fy - ay, fx - ax) - (hip + knee))
                cand = [hip, knee, ankle]
                out.append((cand, math.hypot(fx - tx, fy - ty)))
        return out

    def solve(self, mount, frame, target, plane_roll=0.0):
        """Returns list of world joint positions: [mount, (coxa end), ..., foot]."""
        rx, ry, rz, ux, uy, uz, vx, vy, vz, yaw = self._plane_scalars(mount, frame, target)
        dx, dy, dz = target.x - rx, target.y - ry, target.z - rz
        if self.mode == "radial" and abs(plane_roll) > 1e-5:
            dl2 = dx * dx + dy * dy + dz * dz
            if dl2 > 1e-6:
                inva = 1.0 / math.sqrt(dl2)
                ax, ay, az = dx * inva, dy * inva, dz * inva
                cp, sp = math.cos(plane_roll), math.sin(plane_roll)
                mc = 1.0 - cp
                du = (ax * ux + ay * uy + az * uz) * mc
                cux, cuy, cuz = ay * uz - az * uy, az * ux - ax * uz, ax * uy - ay * ux
                ux, uy, uz = ux * cp + cux * sp + ax * du, uy * cp + cuy * sp + ay * du, uz * cp + cuz * sp + az * du
                dv = (ax * vx + ay * vy + az * vz) * mc
                cvx, cvy, cvz = ay * vz - az * vy, az * vx - ax * vz, ax * vy - ay * vx
                vx, vy, vz = vx * cp + cvx * sp + ax * dv, vy * cp + cvy * sp + ay * dv, vz * cp + cvz * sp + az * dv
        tx = dx * ux + dy * uy + dz * uz
        ty = dx * vx + dy * vy + dz * vz
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        nl2 = nx * nx + ny * ny + nz * nz
        if nl2 < 1e-18:
            pn = frame.right.copy()
            nx, ny, nz = pn.x, pn.y, pn.z
        else:
            invn = 1.0 / math.sqrt(nl2)
            nx, ny, nz = nx * invn, ny * invn, nz * invn
            pn = V3(nx, ny, nz)
        self.plane_normal = pn
        joints = [mount, V3(rx, ry, rz)] if (self.mode == "radial" and self.coxa > 1e-6) else [mount]

        three_same = (len(self.lengths) == 3 and self.sides[0] == self.sides[1]
                      and self.lengths[0] > 0.0 and self.lengths[2] > 0.0)
        if len(self.lengths) == 2:
            kx, ky, fx, fy, self.reached = solve_two_bone(
                self.lengths[0], self.lengths[1], tx, ty, self.sides[0])
            pts2 = [(kx, ky), (fx, fy)]
        elif three_same:
            pts2 = self._three_link_pose(tx, ty)
            self.reached = True
        else:
            if self.pts is None or not self.reached:
                # cold start (or recovering from a fully-stretched straight
                # chain, which has no bend left to warm-start from)
                self.pts = initial_pose_2d(self.lengths, self.sides, tx, ty)
            self.reached = solve_fabrik_2d(self.lengths, self.pts, tx, ty, self.sides)
            pts2 = [tuple(p) for p in self.pts[1:]]

        ang = self._to_angles(pts2)
        self.limited = not self._legal(ang)
        if self.limited:
            solved_exact = False
            if three_same:
                for cand, cerr in self._three_link_clamped_candidates(tx, ty, ang):
                    if cerr < 2e-3 and self._legal(cand):
                        ang = cand
                        pts2 = self._fk(ang)
                        solved_exact = True
                        break
            if not solved_exact:
                was_reachable = self.angles is not None and self.err <= 0.02
                start = list(self.angles) if was_reachable else self._clamp(ang)
                ang_dls = self._dls(start, tx, ty)
                pts_dls = self._fk(ang_dls)
                if was_reachable:
                    ex, ey = pts_dls[-1]
                    if (ex - tx) ** 2 + (ey - ty) ** 2 > 1e-4:
                        ang_alt = self._dls(self._clamp(ang), tx, ty)
                        pts_alt = self._fk(ang_alt)
                        ax, ay = pts_alt[-1]
                        if (ax - tx) ** 2 + (ay - ty) ** 2 < (ex - tx) ** 2 + (ey - ty) ** 2:
                            ang_dls, pts_dls = ang_alt, pts_alt
                ang, pts2 = ang_dls, pts_dls
            if len(self.lengths) > 2:   # warm-start FABRIK from the legal pose
                self.pts = [[0.0, 0.0]] + [[x, y] for x, y in pts2]
        self.angles = ang

        fx, fy = pts2[-1]
        off = dx * nx + dy * ny + dz * nz          # what a clamped coxa / roll can't cover
        self.err = math.sqrt((fx - tx) ** 2 + (fy - ty) ** 2 + off * off)
        self.reached = self.reached and self.err < 0.02

        for px, py in pts2:
            joints.append(V3(rx + ux * px + vx * py, ry + uy * px + vy * py, rz + uz * px + vz * py))
        self.yaw_deg = math.degrees(yaw)
        self.hip_deg = math.degrees(ang[0])
        self.bends_deg = [math.degrees(-self.sides[i] * ang[i + 1]) for i in range(len(ang) - 1)]
        if self.mode == "radial" and self.coxa_max < 3.0:
            self.coxa_ratio = abs(yaw) / max(self.coxa_max, 1e-6)
        elif self.mode == "sagittal" and self.roll_max < 1.5:
            self.coxa_ratio = abs(yaw) / max(self.roll_max, 1e-6)
        else:
            self.coxa_ratio = 0.0
        m_ang = min(ang[0] - self.hip_lim[0], self.hip_lim[1] - ang[0])
        for i, (lo, hi) in enumerate(self.rel_lim):
            m_ang = min(m_ang, ang[i + 1] - lo, hi - ang[i + 1])
        self.limit_margin_deg = math.degrees(m_ang)
        return joints

    # ------------------------------------------------------------------ feasibility
    def feasible(self, mount, frame, target, slack_deg=6.0):
        """Cheap test: could this leg stand on `target` within its limits?"""
        slack = math.radians(slack_deg)
        if self.mode == "radial":
            up = frame.up
            upx, upy, upz = up.x, up.y, up.z
            dx, dy, dz = target.x - mount.x, target.y - mount.y, target.z - mount.z
            du = dx * upx + dy * upy + dz * upz
            dtx, dty, dtz = dx - upx * du, dy - upy * du, dz - upz * du
            dtl2 = dtx * dtx + dty * dty + dtz * dtz
            if dtl2 > 1e-6:
                if frame is LOCAL_FRAME:
                    rd = self.rest_dir_local
                    rdx, rdy, rdz = rd.x, rd.y, rd.z
                else:
                    rd = self.rest_dir_local
                    r_ax, f_ax = frame.right, frame.fwd
                    rdx = r_ax.x * rd.x + upx * rd.y + f_ax.x * rd.z
                    rdy = r_ax.y * rd.x + upy * rd.y + f_ax.y * rd.z
                    rdz = r_ax.z * rd.x + upz * rd.y + f_ax.z * rd.z
                inv = 1.0 / math.sqrt(dtl2)
                ux0, uy0, uz0 = dtx * inv, dty * inv, dtz * inv
                cx = rdy * uz0 - rdz * uy0
                cy = rdz * ux0 - rdx * uz0
                cz = rdx * uy0 - rdy * ux0
                yaw = math.atan2(cx * upx + cy * upy + cz * upz, rdx * ux0 + rdy * uy0 + rdz * uz0)
                if abs(yaw) > self.coxa_max + slack:
                    return False
        rx, ry, rz, ux, uy, uz, vx, vy, vz, _ = self._plane_scalars(mount, frame, target)
        dx, dy, dz = target.x - rx, target.y - ry, target.z - rz
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        off = dx * nx + dy * ny + dz * nz
        max_off = 0.018 if self.mode == "sagittal" else (0.025 + 0.04 * self.reach * (slack_deg / 6.0))
        if abs(off) > max_off:
            return False                          # beyond what the roll / coxa limit allows
        tx = dx * ux + dy * uy + dz * uz
        ty = dx * vx + dy * vy + dz * vz
        d = math.hypot(tx, ty)
        h0, h1 = self.hip_lim
        if (len(self.lengths) == 3 and self.sides[0] == self.sides[1]
                and self.lengths[0] > 0.0 and self.lengths[2] > 0.0):
            if d < 1e-4 or d > self.chain_reach + 1e-3:
                return False
            pts2 = self._three_link_pose(tx, ty)
            ang = self._to_angles(pts2)
            tip_err = math.hypot(pts2[-1][0] - tx, pts2[-1][1] - ty)
            if tip_err <= 0.03:
                if (h0 - slack <= _wrap(ang[0]) <= h1 + slack
                        and all(lo - slack <= ang[i + 1] <= hi + slack
                                for i, (lo, hi) in enumerate(self.rel_lim))):
                    return True
            for cand, cerr in self._three_link_clamped_candidates(tx, ty, ang):
                if (cerr <= 0.03
                        and h0 - slack <= _wrap(cand[0]) <= h1 + slack
                        and all(lo - slack <= cand[i + 1] <= hi + slack
                                for i, (lo, hi) in enumerate(self.rel_lim))):
                    return True
            return False

        l1 = self.lengths[0]
        b_lo, b_hi = self.bend_lim[0]
        if len(self.lengths) == 2:
            l2 = self.lengths[1]
        else:
            # lump the rest of the chain at the middle of its bend range
            l2, l3 = self.lengths[1], self.lengths[2]
            bm = 0.5 * (self.bend_lim[1][0] + self.bend_lim[1][1])
            l2 = math.sqrt(max(1e-6, l2 * l2 + l3 * l3 + 2.0 * l2 * l3 * math.cos(bm)))
            slack += math.radians(15.0)
        if d < 1e-4 or d > l1 + l2:
            return False
        cb = (d * d - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        if abs(cb) > 1.0:
            return False
        bend = math.acos(cb)                     # deviation from straight
        if bend < max(0.0, b_lo) - slack or bend > b_hi + slack:
            return False
        alpha = math.acos(clamp((l1 * l1 + d * d - l2 * l2) / (2.0 * l1 * d), -1.0, 1.0))
        hip = math.atan2(ty, tx) + self.sides[0] * alpha
        return h0 - slack <= _wrap(hip) <= h1 + slack
