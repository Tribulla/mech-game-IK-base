"""Constants, event ids, input container, and planar geometry helpers for Mech.

Physical values match ``IK/c/src/gc_internal.h`` so Python tests and the C11
runtime stay aligned within numerical solver tolerances.
"""
import math

from .vmath import V3, WORLD_UP, clamp

G = 20.0  # gravity (slightly heavier than earth so heavy rigs settle crisply)
A_LAT = 135.0  # sideways grip cap (m/s^2); caps yaw rate at speed to bound lateral acceleration
AIM_MAX = 1.53  # look pitch limit in the air (rad, ~88 deg: avoid gimbal flip at +/-90 deg)

# Event ids mirrored in gaitcore.h (GC_EV_*).
EV_STEP, EV_SPARK, EV_LAND, EV_BOOST, EV_IMPACT, EV_SKID, EV_MANTLE = 1, 2, 3, 4, 5, 6, 7
# Debug-line roles mapped to theme colours by the host renderer.
(DBG_LINE, DBG_LINE_DIM, DBG_INFO, DBG_HIGHLIGHT, DBG_GOOD, DBG_WARNING, DBG_ERROR, DBG_FORCE,
 DBG_NORMAL, DBG_VELOCITY, DBG_AXIS_X, DBG_AXIS_Y, DBG_AXIS_Z) = range(13)

GROUND, AIR, MANTLE, CROUCHJUMP = 0, 1, 2, 3
STATE_NAMES = {GROUND: "ground", AIR: "air", MANTLE: "mantle", CROUCHJUMP: "jump-crouch"}

INPUT_FIELDS = ("fwd", "strafe", "turn", "run", "slow", "crouch", "jump", "drift",
                "climb", "impact", "gait", "aim")   # aim: look pitch (radians, + = up)


class Input:
    __slots__ = INPUT_FIELDS

    def __init__(self):
        for f in INPUT_FIELDS:
            setattr(self, f, 0.0)

    def set(self, vals):
        for f, v in zip(INPUT_FIELDS, vals):
            x = float(v)
            if not math.isfinite(x):
                x = 0.0
            elif f in ("fwd", "strafe", "turn"):
                x = clamp(x, -1.0, 1.0)
            elif f == "aim":
                x = clamp(x, -AIM_MAX, AIM_MAX)
            setattr(self, f, x)

    def copy_from(self, o):
        for f in INPUT_FIELDS:
            setattr(self, f, getattr(o, f))


class Mantle:
    __slots__ = ("p0", "p2", "t", "dur", "height", "top", "fwd", "wire", "landed")


def _nearest_on_hull(hull, p):
    """Nearest point to 2D p on the boundary of a convex hull (point / segment / polygon)."""
    if len(hull) == 1:
        return hull[0]
    best, bd = hull[0], 1e18
    n = len(hull)
    for i in range(n if n > 2 else 1):
        a, b = hull[i], hull[(i + 1) % n]
        ex, ez = b[0] - a[0], b[1] - a[1]
        L2 = ex * ex + ez * ez
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p[0] - a[0]) * ex + (p[1] - a[1]) * ez) / L2))
        q = (a[0] + ex * t, a[1] + ez * t)
        dd = (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2
        if dd < bd:
            best, bd = q, dd
    return best


def _dist_to_segment(p, a, b, up):
    """Distance from p to the segment a-b, measured in the plane normal to up."""
    if up is WORLD_UP:
        abx, abz = b.x - a.x, b.z - a.z
        apx, apz = p.x - a.x, p.z - a.z
        L2 = abx * abx + abz * abz
        if L2 <= 1e-9:
            return math.hypot(apx, apz)
        t = (apx * abx + apz * abz) / L2
        t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
        return math.hypot(apx - abx * t, apz - abz * t)
    ux, uy, uz = up.x, up.y, up.z
    dx, dy, dz = b.x - a.x, b.y - a.y, b.z - a.z
    d = dx * ux + dy * uy + dz * uz
    abx, aby, abz = dx - ux * d, dy - uy * d, dz - uz * d
    px, py, pz = p.x - a.x, p.y - a.y, p.z - a.z
    dp = px * ux + py * uy + pz * uz
    apx, apy, apz = px - ux * dp, py - uy * dp, pz - uz * dp
    L2 = abx * abx + aby * aby + abz * abz
    if L2 <= 1e-9:
        return math.sqrt(apx * apx + apy * apy + apz * apz)
    t = (apx * abx + apy * aby + apz * abz) / L2
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    rx, ry, rz = apx - abx * t, apy - aby * t, apz - abz * t
    return math.sqrt(rx * rx + ry * ry + rz * rz)
