"""World query interface used by the gait core.

When running inside the C game, the native `engine` module (registered by the
host via PyImport_AppendInittab) answers raycasts and collision queries against
the real level geometry.  When run standalone (unit tests, headless tools) a
pure-Python StubWorld with ground + boxes is used instead, so the IK / gait code
can be developed and tested without the game.
"""
import math
from .vmath import V3


class Hit:
    __slots__ = ("t", "point", "normal", "surface")

    def __init__(self, t, point, normal, surface=0):
        self.t = t
        self.point = point
        self.normal = normal
        self.surface = surface


class World:
    """Interface + per-frame debug ray recording."""

    def __init__(self):
        self.record = False
        self.rays = []          # (ox,oy,oz, ex,ey,ez, hit) for debug drawing
        self.ray_count = 0

    def raycast(self, origin, direction, max_dist):
        """direction must be unit length.  Returns Hit or None."""
        self.ray_count += 1
        h = self._raycast(origin, direction, max_dist)
        if self.record:
            end = h.point if h else origin + direction * max_dist
            self.rays.append((origin.x, origin.y, origin.z, end.x, end.y, end.z, 1.0 if h else 0.0))
        return h

    def sphere_push(self, center, radius):
        """Resolve sphere penetration.  Returns (new_center, normal or None)."""
        raise NotImplementedError

    def _raycast(self, origin, direction, max_dist):
        raise NotImplementedError


class EngineWorld(World):
    def __init__(self, engine):
        super().__init__()
        self._e = engine

    def _raycast(self, o, d, max_dist):
        r = self._e.raycast(o.x, o.y, o.z, d.x, d.y, d.z, max_dist)
        if r is None:
            return None
        t, px, py, pz, nx, ny, nz, sid = r
        return Hit(t, V3(px, py, pz), V3(nx, ny, nz), int(sid))

    def sphere_push(self, c, radius):
        x, y, z, hit, nx, ny, nz = self._e.sphere_push(c.x, c.y, c.z, radius)
        return V3(x, y, z), (V3(nx, ny, nz) if hit else None)


class StubWorld(World):
    """Flat ground at y=ground_y plus axis-aligned boxes (min, max)."""

    def __init__(self, boxes=(), ground_y=0.0, has_ground=True):
        super().__init__()
        self.boxes = [(V3(*a), V3(*b)) for a, b in boxes]
        self.ground_y = ground_y
        self.has_ground = has_ground
        self.oboxes = []
        self.grid = {}

    def add_box(self, mn, mx):
        self.boxes.append((V3(*mn), V3(*mx)))

    def add_obox(self, c, h, yaw=0.0, pitch=0.0):
        """Oriented box: centre, half extents, yaw about +y then pitch about x."""
        cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
        X, Y, Z = V3(cy, 0.0, -sy), V3(0.0, 1.0, 0.0), V3(sy, 0.0, cy)
        ax = (X, Y * cp + Z * sp, Z * cp - Y * sp)
        c = V3(*c)
        self.oboxes.append((c, tuple(h), ax))
        # bucket by its xz bounds so queries only test nearby boxes
        ex = [sum(abs(ax[i].x) * h[i] for i in range(3)), 0.0, sum(abs(ax[i].z) * h[i] for i in range(3))]
        for gx in range(int(math.floor((c.x - ex[0]) / self.CELL)), int(math.floor((c.x + ex[0]) / self.CELL)) + 1):
            for gz in range(int(math.floor((c.z - ex[2]) / self.CELL)), int(math.floor((c.z + ex[2]) / self.CELL)) + 1):
                self.grid.setdefault((gx, gz), []).append(len(self.oboxes) - 1)

    CELL = 3.0

    def _near(self, x0, z0, x1, z1):
        if x0 > x1:
            x0, x1 = x1, x0
        if z0 > z1:
            z0, z1 = z1, z0
        out = set()
        C = self.CELL
        for gx in range(int(math.floor(x0 / C)), int(math.floor(x1 / C)) + 1):
            for gz in range(int(math.floor(z0 / C)), int(math.floor(z1 / C)) + 1):
                out.update(self.grid.get((gx, gz), ()))
        return [self.oboxes[i] for i in out]

    def _raycast(self, o, d, max_dist):
        best = None
        ox, oy, oz = o.x, o.y, o.z
        dx, dy, dz = d.x, d.y, d.z
        if self.has_ground and dy < -1e-9 and oy > self.ground_y:
            t = (self.ground_y - oy) / dy
            if t <= max_dist:
                best = Hit(t, V3(ox + dx * t, self.ground_y, oz + dz * t), V3(0.0, 1.0, 0.0))
        for mn, mx in self.boxes:
            tmin, tmax, axis, sgn = -1e30, 1e30, -1, 0.0
            if abs(dx) < 1e-12:
                if ox < mn.x or ox > mx.x:
                    continue
            else:
                inv = 1.0 / dx
                t1 = (mn.x - ox) * inv
                t2 = (mx.x - ox) * inv
                s = -1.0
                if t1 > t2:
                    t1, t2 = t2, t1
                    s = 1.0
                if t1 > tmin:
                    tmin, axis, sgn = t1, 0, s
                if t2 < tmax:
                    tmax = t2
                if tmin > tmax:
                    continue
            if abs(dy) < 1e-12:
                if oy < mn.y or oy > mx.y:
                    continue
            else:
                inv = 1.0 / dy
                t1 = (mn.y - oy) * inv
                t2 = (mx.y - oy) * inv
                s = -1.0
                if t1 > t2:
                    t1, t2 = t2, t1
                    s = 1.0
                if t1 > tmin:
                    tmin, axis, sgn = t1, 1, s
                if t2 < tmax:
                    tmax = t2
                if tmin > tmax:
                    continue
            if abs(dz) < 1e-12:
                if oz < mn.z or oz > mx.z:
                    continue
            else:
                inv = 1.0 / dz
                t1 = (mn.z - oz) * inv
                t2 = (mx.z - oz) * inv
                s = -1.0
                if t1 > t2:
                    t1, t2 = t2, t1
                    s = 1.0
                if t1 > tmin:
                    tmin, axis, sgn = t1, 2, s
                if t2 < tmax:
                    tmax = t2
                if tmin > tmax:
                    continue
            if axis < 0 or tmin < 0.0 or tmin > max_dist:
                continue
            if best is None or tmin < best.t:
                nx = sgn if axis == 0 else 0.0
                ny = sgn if axis == 1 else 0.0
                nz = sgn if axis == 2 else 0.0
                best = Hit(tmin, V3(ox + dx * tmin, oy + dy * tmin, oz + dz * tmin), V3(nx, ny, nz))
        if self.oboxes:
            md = min(max_dist, 1e4)
            for c, h, ax in self._near(ox, oz, ox + dx * md, oz + dz * md):
                rx, ry, rz = ox - c.x, oy - c.y, oz - c.z
                tmin, tmax, axis, sgn = -1e30, 1e30, -1, 0.0
                ok = True
                for i in range(3):
                    axi = ax[i]
                    axx, axy, axz = axi.x, axi.y, axi.z
                    oi = rx * axx + ry * axy + rz * axz
                    di = dx * axx + dy * axy + dz * axz
                    hi = h[i]
                    if abs(di) < 1e-12:
                        if oi < -hi or oi > hi:
                            ok = False
                            break
                        continue
                    inv = 1.0 / di
                    t1 = (-hi - oi) * inv
                    t2 = (hi - oi) * inv
                    s = -1.0
                    if t1 > t2:
                        t1, t2 = t2, t1
                        s = 1.0
                    if t1 > tmin:
                        tmin, axis, sgn = t1, i, s
                    if t2 < tmax:
                        tmax = t2
                    if tmin > tmax:
                        ok = False
                        break
                if not ok or axis < 0 or tmin < 0.0 or tmin > max_dist:
                    continue
                if best is None or tmin < best.t:
                    best = Hit(tmin, V3(ox + dx * tmin, oy + dy * tmin, oz + dz * tmin), ax[axis] * sgn)
        return best

    def sphere_push(self, c, r):
        cx, cy, cz = c.x, c.y, c.z
        normal = None
        if self.has_ground and cy - r < self.ground_y:
            cy = self.ground_y + r
            normal = V3(0.0, 1.0, 0.0)
        r2 = r * r
        for mn, mx in self.boxes:
            px = mn.x if cx < mn.x else (mx.x if cx > mx.x else cx)
            py = mn.y if cy < mn.y else (mx.y if cy > mx.y else cy)
            pz = mn.z if cz < mn.z else (mx.z if cz > mx.z else cz)
            dx, dy, dz = cx - px, cy - py, cz - pz
            l2 = dx * dx + dy * dy + dz * dz
            if l2 < 1e-18:
                # Sphere centre lies inside the AABB: push out across the nearest face.
                pen = ((cx - mn.x, -1.0, 0.0, 0.0), (mx.x - cx, 1.0, 0.0, 0.0),
                       (cy - mn.y, 0.0, -1.0, 0.0), (mx.y - cy, 0.0, 1.0, 0.0),
                       (cz - mn.z, 0.0, 0.0, -1.0), (mx.z - cz, 0.0, 0.0, 1.0))
                depth, nx, ny, nz = min(pen, key=lambda q: q[0])
                k = depth + r
                cx += nx * k
                cy += ny * k
                cz += nz * k
                normal = V3(nx, ny, nz)
            elif l2 < r2:
                l = math.sqrt(l2)
                inv = 1.0 / l
                nx, ny, nz = dx * inv, dy * inv, dz * inv
                k = r - l
                cx += nx * k
                cy += ny * k
                cz += nz * k
                normal = V3(nx, ny, nz)
        if self.oboxes:
            for bc, h, ax in self._near(cx - r, cz - r, cx + r, cz + r):
                rx, ry, rz = cx - bc.x, cy - bc.y, cz - bc.z
                ax0, ax1, ax2 = ax[0], ax[1], ax[2]
                l0 = rx * ax0.x + ry * ax0.y + rz * ax0.z
                l1 = rx * ax1.x + ry * ax1.y + rz * ax1.z
                l2 = rx * ax2.x + ry * ax2.y + rz * ax2.z
                h0, h1, h2 = h[0], h[1], h[2]
                q0 = -h0 if l0 < -h0 else (h0 if l0 > h0 else l0)
                q1 = -h1 if l1 < -h1 else (h1 if l1 > h1 else l1)
                q2 = -h2 if l2 < -h2 else (h2 if l2 > h2 else l2)
                d0, d1, d2 = l0 - q0, l1 - q1, l2 - q2
                dl2 = d0 * d0 + d1 * d1 + d2 * d2
                if dl2 < 1e-18:
                    pen = ((l0 + h0, ax0, -1.0), (h0 - l0, ax0, 1.0),
                           (l1 + h1, ax1, -1.0), (h1 - l1, ax1, 1.0),
                           (l2 + h2, ax2, -1.0), (h2 - l2, ax2, 1.0))
                    depth, nax, ns = min(pen, key=lambda t: t[0])
                    normal = nax * ns
                    k = depth + r
                    cx += normal.x * k
                    cy += normal.y * k
                    cz += normal.z * k
                elif dl2 < r2:
                    dl = math.sqrt(dl2)
                    inv = 1.0 / dl
                    nx = (ax0.x * d0 + ax1.x * d1 + ax2.x * d2) * inv
                    ny = (ax0.y * d0 + ax1.y * d1 + ax2.y * d2) * inv
                    nz = (ax0.z * d0 + ax1.z * d1 + ax2.z * d2) * inv
                    normal = V3(nx, ny, nz)
                    k = r - dl
                    cx += nx * k
                    cy += ny * k
                    cz += nz * k
        return V3(cx, cy, cz), normal


_world = None


def get_world():
    global _world
    if _world is None:
        try:
            import engine  # provided by the C host
            _world = EngineWorld(engine)
        except ImportError:
            _world = StubWorld()
    return _world


def set_world(w):
    global _world
    _world = w
    return w


def is_native():
    return isinstance(get_world(), EngineWorld)
