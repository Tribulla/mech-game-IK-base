"""Flat API called by the C++ host (see samples/mechs/mech_bridge.h).

All bulk data crosses the boundary as packed float32 `bytes` so the host can
memcpy it - no per-value Python object traffic in the hot path.

Per-mech input record (INPUT_STRIDE floats):
    fwd, strafe, turn, run, slow, crouch, jump, drift, climb, impact, gait
"""
import array
import importlib
import math
import time
import traceback

from . import (vmath, world, ik, gaits, gaitsel, mechdefs, leg,
               mech_const, foothold, clearance, scheduler, mech as mech_mod)

INPUT_STRIDE = len(mech_mod.INPUT_FIELDS)
HEADER_FLOATS = 28
LEG_EXTRA_FLOATS = 6
EVENT_FLOATS = 11
DEBUG_LINE_FLOATS = 9
MAX_SUBSTEP = 1.0 / 60.0
MAX_FRAME_DT = 0.1
PROFILE_HEADER = 16
PROFILE_KINDS = 9
PROFILE_KIND_STRIDE = 5
PROFILE_FLOATS = PROFILE_HEADER + PROFILE_KINDS * PROFILE_KIND_STRIDE

_mechs = []
_events = []
_debug = False
_stats = {"ms": 0.0, "rays": 0}
_max_substep = MAX_SUBSTEP
_leg_ik_hz = 0.0
_probe_period = 0.05
_speed_scale = 1.0
_plane_roll = True
_profile = False
_prof_buf = [0.0] * PROFILE_FLOATS


def _apply_bench_cfg(m, idx=0):
    m.ik_solve_hz = _leg_ik_hz
    m.probe_period = _probe_period
    m.speed_scale = _speed_scale
    m.plane_roll_enabled = _plane_roll
    m._ik_acc = (idx % 8) * (0.125 / _leg_ik_hz) if _leg_ik_hz > 0.0 else 0.0


def init():
    world.get_world()
    return "gaitcore ready (%s world, %d mech types)" % (
        "native engine" if world.is_native() else "python stub", len(mechdefs.ALL))


def reset(reload_modules=False):
    """Drop every mech (a sample was (re)started). Optionally reload the gait
    code first, so editing Python + pressing R in the app picks up the change."""
    global _debug, _max_substep, _leg_ik_hz, _probe_period, _speed_scale, _plane_roll, _profile, _prof_buf
    _mechs.clear()
    _events.clear()
    _debug = False
    _max_substep = MAX_SUBSTEP
    _leg_ik_hz = 0.0
    _probe_period = 0.05
    _speed_scale = 1.0
    _plane_roll = True
    _profile = False
    _prof_buf = [0.0] * PROFILE_FLOATS
    if reload_modules:
        try:
            _reload_modules()
        except Exception:
            traceback.print_exc()
            raise
    world.set_world(None)
    world.get_world()
    return True


def _reload_modules():
    global INPUT_STRIDE
    for mod in (vmath, world, ik, gaits, gaitsel, mechdefs, leg,
                mech_const, foothold, clearance, scheduler, mech_mod):
        importlib.reload(mod)
    INPUT_STRIDE = len(mech_mod.INPUT_FIELDS)


def kinds():
    return [d["name"] for d in mechdefs.ALL]


def _def(kind):
    if isinstance(kind, int):
        return mechdefs.ALL[kind % len(mechdefs.ALL)]
    return mechdefs.BY_NAME[kind]


def spawn(kind, x, y, z, yaw):
    m = mech_mod.Mech(_def(kind), vmath.V3(x, y, z), yaw, _events)
    _apply_bench_cfg(m, len(_mechs))
    _mechs.append(m)
    return len(_mechs) - 1


def respawn(h, kind, x, y, z, yaw):
    m = mech_mod.Mech(_def(kind), vmath.V3(x, y, z), yaw, _events)
    _apply_bench_cfg(m, h)
    _mechs[h] = m
    return h


def despawn_to(n):
    n = max(0, int(n))
    if n < len(_mechs):
        del _mechs[n:]
    return len(_mechs)


def set_bench_config(max_substep=MAX_SUBSTEP, leg_ik_hz=0.0, probe_hz=20.0,
                     speed_scale=1.0, plane_roll=True, profile=False):
    global _max_substep, _leg_ik_hz, _probe_period, _speed_scale, _plane_roll, _profile
    _max_substep = max(1.0 / 240.0, min(MAX_FRAME_DT, float(max_substep)))
    _leg_ik_hz = max(0.0, min(240.0, float(leg_ik_hz)))
    _probe_period = 1.0 / max(1.0, min(120.0, float(probe_hz)))
    _speed_scale = max(0.0, min(3.0, float(speed_scale)))
    _plane_roll = bool(plane_roll)
    _profile = bool(profile)
    for idx, m in enumerate(_mechs):
        _apply_bench_cfg(m, idx)
        if not _profile:
            m._prof = None


def count():
    return len(_mechs)


def info(h):
    m = _mechs[h]
    d = m.d
    return (d["name"], d["kind_id"], m.n, m.joints_per_leg,
            float(d["body_half"][0]), float(d["body_half"][1]), float(d["body_half"][2]),
            tuple(float(w) for w in d["widths"]), float(d["mass"]), d["blurb"],
            1 if d["climber"] else 0, int(d.get("faction", 0)), float(d.get("wire_max", 0.0)),
            float(d["speeds"]["walk"]), float(d["speeds"]["run"]))


def floats_per_mech(h):
    m = _mechs[h]
    return HEADER_FLOATS + m.n * (m.joints_per_leg * 3 + LEG_EXTRA_FLOATS)


def step(dt, inputs):
    if not isinstance(dt, (int, float)) or not math.isfinite(dt) or dt <= 0.0:
        return
    dt = min(float(dt), MAX_FRAME_DT)
    vals = array.array("f")
    vals.frombytes(inputs)
    for i, m in enumerate(_mechs):
        rec = vals[i * INPUT_STRIDE:(i + 1) * INPUT_STRIDE]
        if len(rec) == INPUT_STRIDE:
            m.inp.set(rec)
    w = world.get_world()
    w.record = _debug
    w.rays.clear()
    w.ray_count = 0
    t0 = time.perf_counter()
    n = max(1, int(math.ceil(dt / _max_substep - 1e-6)))
    sub = dt / n
    if not _profile:
        for _ in range(n):
            for m in _mechs:
                m.update(sub)
        _stats["ms"] = (time.perf_counter() - t0) * 1000.0
        _stats["rays"] = w.ray_count
        return

    prof = [0.0] * 10
    kind_ms = [0.0] * PROFILE_KINDS
    kind_rays = [0] * PROFILE_KINDS
    kind_ik = [0] * PROFILE_KINDS
    kind_cnt = [0] * PROFILE_KINDS
    kind_legs = [0] * PROFILE_KINDS
    for m in _mechs:
        m._prof = prof
        kid = m.d["kind_id"]
        if 0 <= kid < PROFILE_KINDS:
            kind_cnt[kid] += 1
            kind_legs[kid] += m.n
    perf = time.perf_counter
    for _ in range(n):
        for m in _mechs:
            kid = m.d["kind_id"]
            r0 = w.ray_count
            ik0 = prof[8]
            mt0 = perf()
            m.update(sub)
            elapsed = (perf() - mt0) * 1000.0
            if 0 <= kid < PROFILE_KINDS:
                kind_ms[kid] += elapsed
                kind_rays[kid] += w.ray_count - r0
                kind_ik[kid] += int(prof[8] - ik0)
    total_ms = (perf() - t0) * 1000.0
    _stats["ms"] = total_ms
    _stats["rays"] = w.ray_count
    prev_poses_ms = _prof_buf[13]
    _prof_buf[0] = total_ms
    for i in range(8):
        _prof_buf[1 + i] = prof[i] * 1000.0
    _prof_buf[9] = float(n)
    _prof_buf[10] = prof[8]
    _prof_buf[11] = prof[9]
    _prof_buf[12] = float(w.ray_count)
    _prof_buf[13] = prev_poses_ms
    _prof_buf[14] = float(sum(kind_legs))
    _prof_buf[15] = float(len(_mechs))
    for k in range(PROFILE_KINDS):
        base = PROFILE_HEADER + k * PROFILE_KIND_STRIDE
        _prof_buf[base] = float(kind_cnt[k])
        _prof_buf[base + 1] = float(kind_legs[k])
        _prof_buf[base + 2] = kind_ms[k]
        _prof_buf[base + 3] = float(kind_rays[k])
        _prof_buf[base + 4] = float(kind_ik[k])


def poses():
    t0 = time.perf_counter() if _profile else 0.0
    out = array.array("f")
    for m in _mechs:
        m.pack(out)
    res = out.tobytes()
    if _profile:
        _prof_buf[13] = (time.perf_counter() - t0) * 1000.0
    return res


def profile_data():
    return array.array("f", _prof_buf).tobytes()


def events():
    out = array.array("f")
    for e in _events:
        out.extend(e)
    _events.clear()
    return out.tobytes()


def set_debug(flag):
    global _debug
    _debug = bool(flag)


def debug_lines(h):
    out = array.array("f")
    if 0 <= h < len(_mechs):
        _mechs[h].debug_lines(out)
    if _debug:
        for r in world.get_world().rays:
            role = mech_mod.DBG_WARNING if r[6] > 0.5 else mech_mod.DBG_LINE_DIM
            out.extend((r[0], r[1], r[2], r[3], r[4], r[5], float(role), 0.0, 0.0))
    return out.tobytes()


def status(h):
    m = _mechs[h]
    return m.status() + "\nik/gait sim %.2f ms   rays %d" % (_stats["ms"], _stats["rays"])


def stats(h):
    """Numbers and labels for the host's info panel."""
    m = _mechs[h]
    d = m.d
    return (m.mode_label(), m.gait_label(), 1 if m.gait_override else 0, mech_mod.STATE_NAMES[m.state],
            float(m.duty), float(m.cycle), float(max(l.stretch for l in m.legs)),
            float(_stats["ms"]), int(_stats["rays"]), 1 if m.climb_enabled else 0,
            1 if m.crouch_toggle else 0)


def gait_names(h):
    return list(_mechs[h].d["gaits"].keys())


def hot_reload():
    """Reload every gaitcore module and rebuild mechs in place (F5 in game)."""
    saved = []
    for m in _mechs:
        g = m.pos - m.frame.up * m.ride
        saved.append((m.d["name"], (g.x, g.y, g.z), math.atan2(m.frame.fwd.x, m.frame.fwd.z)))
    prev_w = world._world
    saved_world = None
    if prev_w is not None and hasattr(prev_w, "boxes") and hasattr(prev_w, "oboxes"):
        saved_world = (
            prev_w.ground_y,
            prev_w.has_ground,
            [((a.x, a.y, a.z), (b.x, b.y, b.z)) for a, b in prev_w.boxes],
            [((c.x, c.y, c.z), h, ((ax[0].x, ax[0].y, ax[0].z),
                                    (ax[1].x, ax[1].y, ax[1].z),
                                    (ax[2].x, ax[2].y, ax[2].z))) for c, h, ax in prev_w.oboxes],
            {k: list(v) for k, v in prev_w.grid.items()},
        )
    try:
        _reload_modules()
    except Exception:
        traceback.print_exc()
        return False
    if saved_world is not None:
        gy, hg, boxes, oboxes, grid = saved_world
        nw = world.StubWorld(boxes=boxes, ground_y=gy, has_ground=hg)
        nw.oboxes = [(vmath.V3(*c), h, (vmath.V3(*ax[0]), vmath.V3(*ax[1]), vmath.V3(*ax[2]))) for c, h, ax in oboxes]
        nw.grid = grid
        world.set_world(nw)
    else:
        world.get_world()
    try:
        for i, (name, gp, yaw) in enumerate(saved):
            d = mechdefs.BY_NAME.get(name, mechdefs.ALL[0])
            _mechs[i] = mech_mod.Mech(d, vmath.V3(*gp), yaw, _events)
            _apply_bench_cfg(_mechs[i], i)
    except Exception:
        traceback.print_exc()
        return False
    print("[gaitcore] hot reload ok")
    return True
