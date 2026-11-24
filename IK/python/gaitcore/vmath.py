"""Small, dependency-free 3D math used by the gait / IK core.

Kept deliberately tiny and allocation-light: this runs inside the game loop
every frame, so the hot paths (planar IK) work on plain floats instead.
"""
import math

EPS = 1e-9


class V3:
    __slots__ = ("x", "y", "z")

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    # --- arithmetic -------------------------------------------------------
    def __add__(self, o):
        return V3(self.x + o.x, self.y + o.y, self.z + o.z)

    def __sub__(self, o):
        return V3(self.x - o.x, self.y - o.y, self.z - o.z)

    def __mul__(self, k):
        return V3(self.x * k, self.y * k, self.z * k)

    __rmul__ = __mul__

    def __truediv__(self, k):
        return V3(self.x / k, self.y / k, self.z / k)

    def __neg__(self):
        return V3(-self.x, -self.y, -self.z)

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.z

    def __repr__(self):
        return "V3(%.3f, %.3f, %.3f)" % (self.x, self.y, self.z)

    # --- products / norms --------------------------------------------------
    def dot(self, o):
        return self.x * o.x + self.y * o.y + self.z * o.z

    def cross(self, o):
        return V3(self.y * o.z - self.z * o.y,
                  self.z * o.x - self.x * o.z,
                  self.x * o.y - self.y * o.x)

    def length(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def length_sq(self):
        return self.x * self.x + self.y * self.y + self.z * self.z

    def normalized(self, fallback=None):
        l2 = self.x * self.x + self.y * self.y + self.z * self.z
        if l2 < 1e-18:
            return fallback.copy() if fallback is not None else V3()
        inv = 1.0 / math.sqrt(l2)
        return V3(self.x * inv, self.y * inv, self.z * inv)

    def copy(self):
        return V3(self.x, self.y, self.z)

    def set(self, o):
        self.x, self.y, self.z = o.x, o.y, o.z
        return self

    def lerp(self, o, t):
        return V3(self.x + (o.x - self.x) * t,
                  self.y + (o.y - self.y) * t,
                  self.z + (o.z - self.z) * t)

    def dist(self, o):
        dx, dy, dz = self.x - o.x, self.y - o.y, self.z - o.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)


WORLD_UP = V3(0.0, 1.0, 0.0)


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def smoothstep(t):
    t = clamp(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def damp_factor(rate, dt):
    """Frame-rate independent exponential smoothing factor."""
    return 1.0 - math.exp(-rate * dt)


def project_on_plane(v, n):
    d = v.x * n.x + v.y * n.y + v.z * n.z
    return V3(v.x - n.x * d, v.y - n.y * d, v.z - n.z * d)


def rotate_axis(v, axis, angle):
    """Rodrigues rotation of v around unit axis."""
    c = math.cos(angle)
    s = math.sin(angle)
    d = (axis.x * v.x + axis.y * v.y + axis.z * v.z) * (1.0 - c)
    cx = axis.y * v.z - axis.z * v.y
    cy = axis.z * v.x - axis.x * v.z
    cz = axis.x * v.y - axis.y * v.x
    return V3(v.x * c + cx * s + axis.x * d,
              v.y * c + cy * s + axis.y * d,
              v.z * c + cz * s + axis.z * d)


def turn_displacement(travel, up, angle):
    """Integrate horizontal travel along a circular yaw arc instead of a straight
    chord, scaling by sin(angle/2)/(angle/2) rotated by half the turn.
    """
    if abs(angle) < 1e-5:
        return travel
    half = 0.5 * angle
    return rotate_axis(travel, up, half) * (math.sin(half) / half)


def angle_between(a, b):
    return math.acos(clamp(a.normalized().dot(b.normalized()), -1.0, 1.0))


def rotate_towards(v, target, max_angle):
    """Rotate unit vector v towards unit vector target by at most max_angle."""
    ang = math.acos(clamp(v.dot(target), -1.0, 1.0))
    if ang < 1e-6:
        return target.copy()
    axis = v.cross(target)
    if axis.length_sq() < 1e-12:
        # opposite vectors: pick any perpendicular axis
        axis = v.cross(V3(1, 0, 0))
        if axis.length_sq() < 1e-12:
            axis = v.cross(V3(0, 0, 1))
    axis = axis.normalized()
    return rotate_axis(v, axis, min(ang, max_angle))


def bezier3(p0, p1, p2, p3, t):
    u = 1.0 - t
    a = u * u * u
    b = 3.0 * u * u * t
    c = 3.0 * u * t * t
    d = t * t * t
    return V3(p0.x * a + p1.x * b + p2.x * c + p3.x * d,
              p0.y * a + p1.y * b + p2.y * c + p3.y * d,
              p0.z * a + p1.z * b + p2.z * c + p3.z * d)


class Frame:
    """Orthonormal body frame.  Local coordinates: x = right, y = up, z = forward.

    right is defined as fwd x up so that, in the game's right-handed / y-up
    world, +x local really is the mech's right-hand side.
    """
    __slots__ = ("fwd", "up", "right")

    def __init__(self, fwd=None, up=None):
        self.fwd = V3(0, 0, 1)
        self.up = V3(0, 1, 0)
        self.right = V3(-1, 0, 0)
        self.set(fwd or V3(0, 0, 1), up or V3(0, 1, 0))

    def set(self, fwd, up):
        up = up.normalized(WORLD_UP)
        f = project_on_plane(fwd, up)
        if f.length_sq() < 1e-10:
            # forward collapsed onto up: keep the old forward's projection
            f = project_on_plane(self.fwd, up)
            if f.length_sq() < 1e-10:
                f = project_on_plane(self.up * -1.0, up)
        self.up = up
        self.fwd = f.normalized(V3(0, 0, 1))
        self.right = self.fwd.cross(self.up).normalized()
        return self

    def copy(self):
        fr = Frame.__new__(Frame)
        fr.fwd = self.fwd.copy()
        fr.up = self.up.copy()
        fr.right = self.right.copy()
        return fr

    def to_world(self, l):
        r, u, f = self.right, self.up, self.fwd
        return V3(r.x * l.x + u.x * l.y + f.x * l.z,
                  r.y * l.x + u.y * l.y + f.y * l.z,
                  r.z * l.x + u.z * l.y + f.z * l.z)

    def to_local(self, w):
        r, u, f = self.right, self.up, self.fwd
        return V3(w.x * r.x + w.y * r.y + w.z * r.z,
                  w.x * u.x + w.y * u.y + w.z * u.z,
                  w.x * f.x + w.y * f.y + w.z * f.z)

    def rotate(self, axis, angle):
        if abs(angle) < 1e-9:
            return self
        self.set(rotate_axis(self.fwd, axis, angle), rotate_axis(self.up, axis, angle))
        return self

    def yaw(self, angle):
        """Positive angle turns to the right around the frame's own up."""
        c, s = math.cos(angle), math.sin(angle)
        self.set(self.fwd * c + self.right * s, self.up)
        return self

    def tilt_up_towards(self, target_up, max_angle):
        """Rotate the whole frame so up moves towards target_up (parallel transport of fwd)."""
        t = target_up.normalized(self.up)
        ang = math.acos(clamp(self.up.dot(t), -1.0, 1.0))
        if ang < 1e-6:
            return self
        axis = self.up.cross(t)
        if axis.length_sq() < 1e-12:
            axis = self.right
        axis = axis.normalized()
        return self.rotate(axis, min(ang, max_angle))


def convex_hull_2d(pts):
    """Monotone-chain convex hull of (x, z) tuples, counter-clockwise."""
    pts = sorted(set(pts))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def support_margin(hull, p):
    """Signed distance of 2D point p inside convex CCW polygon `hull`.

    Positive = inside (distance to nearest edge), negative = outside.
    Degenerate hulls (point / segment) return -distance to the shape.
    """
    n = len(hull)
    if n == 0:
        return -1e9
    if n == 1:
        return -math.hypot(p[0] - hull[0][0], p[1] - hull[0][1])
    if n == 2:
        return -_dist_point_segment(p, hull[0], hull[1])
    inside = True
    best = 1e9
    for i in range(n):
        a = hull[i]
        b = hull[(i + 1) % n]
        ex, ez = b[0] - a[0], b[1] - a[1]
        el = math.hypot(ex, ez) or 1e-9
        # left of edge (CCW) is inside
        d = (ex * (p[1] - a[1]) - ez * (p[0] - a[0])) / el
        if d < 0:
            inside = False
        best = min(best, d)
    if inside:
        return best
    return -min(_dist_point_segment(p, hull[i], hull[(i + 1) % n]) for i in range(n))


def _dist_point_segment(p, a, b):
    ax, az = a
    bx, bz = b
    ex, ez = bx - ax, bz - az
    l2 = ex * ex + ez * ez
    t = 0.0 if l2 < 1e-12 else clamp(((p[0] - ax) * ex + (p[1] - az) * ez) / l2, 0.0, 1.0)
    return math.hypot(p[0] - (ax + ex * t), p[1] - (az + ez * t))


def fit_plane(points, weights=None):
    """Least-squares plane through points.  Returns (centroid, unit normal or None).

    Solves for y = a*x + b*z + c in centroid-relative coordinates, which is
    well conditioned for the mostly-horizontal foot planes we care about.
    """
    n = len(points)
    if n == 0:
        return None, None
    if weights is None:
        weights = [1.0] * n
    wsum = sum(weights) or 1.0
    cx = sum(p.x * w for p, w in zip(points, weights)) / wsum
    cy = sum(p.y * w for p, w in zip(points, weights)) / wsum
    cz = sum(p.z * w for p, w in zip(points, weights)) / wsum
    c = V3(cx, cy, cz)
    if n < 3:
        return c, None
    sxx = sxz = szz = sxy = szy = 0.0
    for p, w in zip(points, weights):
        dx, dy, dz = p.x - cx, p.y - cy, p.z - cz
        sxx += w * dx * dx
        sxz += w * dx * dz
        szz += w * dz * dz
        sxy += w * dx * dy
        szy += w * dz * dy
    det = sxx * szz - sxz * sxz
    if abs(det) < 1e-6:
        return c, None
    a = (sxy * szz - szy * sxz) / det
    b = (szy * sxx - sxy * sxz) / det
    return c, V3(-a, 1.0, -b).normalized()


class _LocalFrame:
    """Identity frame for evaluating leg IK purely in body-local space.

    Note: chassis-local mounts already express right as +x (matching authored
    definitions), whereas world Frame has right = fwd x up (-x at yaw 0).
    """
    __slots__ = ("fwd", "up", "right")

    def __init__(self):
        self.fwd = V3(0, 0, 1)
        self.up = V3(0, 1, 0)
        self.right = V3(1, 0, 0)

    def to_world(self, l):
        return l.copy()

    def to_local(self, w):
        return w.copy()


LOCAL_FRAME = _LocalFrame()

