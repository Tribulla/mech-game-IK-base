"""Foothold check: on a rock field, does a foot step *onto* a rock when the
spot the gait wants lies on top of one, or does it dodge around / between
rocks to reach lower ground?

    python ik/tools/foothold_check.py [MECH ...] [--seeds N] [--secs S] [--world rubble|boulders|rocks]

Worlds: `rubble` is as dense as course A's rubble field (tilted slabs,
0.1-0.6 m tall), `boulders` like course D's boulders (0.4-1.9 m, tilted up
to 23 deg) on flat ground, `rocks` is fuzz_check's sparse rock field.

Every foothold search on the ground is compared with what lies under the
spot the gait asked for (the "ideal"):

* rock-top  - the ideal is on a rock top the foot fits on and the leg can
              stand on (reachable, flat, clear).
* kept      - of those, the foot landed on that rock top.
* dodged    - of those, the foot went to the ground beside / between rocks.
* off m     - how far (horizontally) chosen footholds land from the ideal,
              averaged over all searches; big numbers mean side-steps.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

from gaitcore import api, world, mechdefs, mech as mech_mod  # noqa: E402
from gaitcore.vmath import V3, WORLD_UP  # noqa: E402
import fuzz_check  # noqa: E402  (same rock world and random driver)

DT = 1.0 / 60.0


def rock_top_under(m, l, ideal, pose=None):
    """The rock top under `ideal` if the foot could stand on it, else None."""
    w = m.world
    h = w.raycast(V3(ideal.x, ideal.y + 6.0, ideal.z), V3(0, -1, 0), 12.0)
    if h is None or h.point.y < 0.06 or h.normal.y < 0.85:
        return None  # ground (y = 0) or a rock side
    r = l.foot_r
    for dx, dz in ((r, 0), (-r, 0), (0, r), (0, -r)):
        hh = w.raycast(V3(h.point.x + dx, h.point.y + 0.5, h.point.z + dz), V3(0, -1, 0), 1.0)
        if hh is None or abs(hh.point.y - h.point.y) > 0.6 * r + 0.03:
            return None  # the foot doesn't fit
    if h.point.y - (m.pos.y - m.ride) > m.d["step_max"] + 0.05:
        return None  # taller than the mech steps up
    mount = pose[0] if pose is not None else l.mount_world
    if not (m._reachable(l, h.point, pose) and m._clear_spot(l, h.point, WORLD_UP, mount)):
        return None
    return h.point


def run(name, seed, secs, stats, kind="rubble"):
    import random
    rng = random.Random(seed * 7919 + sum(ord(c) for c in name))
    w, _ = fuzz_check.make_world(kind, seed)
    world.set_world(w)
    api._mechs.clear()
    api._events.clear()
    m = api._mechs[api.spawn(name, 0, 0, 0, rng.uniform(0, math.tau))]
    orig = m._find_foothold

    def wrapped(l, ideal, delay=0.0):
        fh = orig(l, ideal, delay)
        if m.state != mech_mod.GROUND or m.climbing_mode or fh is None:
            return fh
        stats["searches"] += 1
        p = fh[0]
        off = math.hypot(p.x - ideal.x, p.z - ideal.z)
        stats["off"] += off
        stats["side"] += off > 0.35
        if off > 0.35:
            g0 = m.world.raycast(V3(ideal.x, ideal.y + 6.0, ideal.z), V3(0, -1, 0), 12.0)
            where = "rock" if g0 is not None and g0.point.y > 0.05 else "ground"
            k = "%s:%s->%s" % (getattr(m, "fh_why", "?"), where, "rock" if p.y > 0.05 else "ground")
            stats["sidewhy"][k] = stats["sidewhy"].get(k, 0) + 1
        g = m.world.raycast(V3(ideal.x, ideal.y + 6.0, ideal.z), V3(0, -1, 0), 12.0)
        stats["ideal_rock"] += g is not None and g.point.y > 0.05
        stats["landed_rock"] += p.y > 0.05
        pose = m._landing_pose(l, delay) if delay > 0.0 else None
        top = rock_top_under(m, l, ideal, pose)
        if top is not None:
            stats["top"] += 1
            if p.y > top.y - 0.08:
                stats["kept"] += 1
            else:
                stats["dodged"] += 1
                why = stats["why"]
                h = m.world.raycast(V3(top.x, top.y + 0.3, top.z), V3(0, -1, 0), 1.0)
                if h is not None and m._near_edge(h, WORLD_UP):
                    why["near_edge"] = why.get("near_edge", 0) + 1
                elif not m._joints_clear(l, top, pose):
                    why["joints"] = why.get("joints", 0) + 1
                else:
                    why["other"] = why.get("other", 0) + 1
        return fh

    m._find_foothold = wrapped
    state = {}
    for i in range(int(secs / DT)):
        inp = fuzz_check.drive(rng, i * DT, state)
        # a walking pace (~4 m/s): full throttle is 18-36 m/s, which leaves the field in a second
        inp = dict(inp, jump=0.0, impact=0.0, run=0.0, fwd=inp.get("fwd", 0.0) * min(1.0, 4.0 / m.d["speeds"]["walk"]))
        m.inp.set([float(inp.get(f, 0.0)) for f in mech_mod.INPUT_FIELDS])
        m.update(DT)
        api._events.clear()


def main():
    args = sys.argv[1:]
    seeds, secs = 3, 25.0
    if "--seeds" in args:
        i = args.index("--seeds")
        seeds = int(args[i + 1])
        del args[i:i + 2]
    if "--secs" in args:
        i = args.index("--secs")
        secs = float(args[i + 1])
        del args[i:i + 2]
    kind = "rubble"
    if "--world" in args:
        i = args.index("--world")
        kind = args[i + 1]
        del args[i:i + 2]
    names = args or [d["name"] for d in mechdefs.ALL]
    print("%-10s %8s %9s %9s %7s %8s %7s %7s" % ("mech", "searches", "ideal-on", "landed-on", "usable", "dodged%",
                                                  "side%", "off m"))
    print("%-10s %8s %9s %9s %7s %8s" % ("", "", "rock %", "rock %", "tops", ""))
    keys = ("searches", "top", "kept", "dodged", "off", "side", "ideal_rock", "landed_rock")
    tot = {k: 0 for k in keys}
    tot["why"] = {}
    tot["sidewhy"] = {}
    for name in names:
        st = {k: 0 for k in keys}
        st["why"] = tot["why"]
        st["sidewhy"] = tot["sidewhy"]
        for s in range(seeds):
            run(name, s, secs, st, kind)
        for k in keys:
            tot[k] += st[k]
        n = max(1, st["searches"])
        print("%-10s %8d %9.1f %9.1f %7d %8.1f %7.1f %7.2f" % (
            name, st["searches"], 100.0 * st["ideal_rock"] / n, 100.0 * st["landed_rock"] / n, st["top"],
            100.0 * st["dodged"] / max(1, st["top"]), 100.0 * st["side"] / n, st["off"] / n))
    n = max(1, tot["searches"])
    print("\ntotal: ideal on rock %.1f%% of searches, foot landed on rock %.1f%%; usable rock top under the ideal %d times, "
          "dodged to lower ground %.1f%% (%s); side-steps > 0.35 m %.1f%%, mean offset %.2f m" % (
              100.0 * tot["ideal_rock"] / n, 100.0 * tot["landed_rock"] / n, tot["top"],
              100.0 * tot["dodged"] / max(1, tot["top"]), ", ".join("%s %d" % kv for kv in sorted(tot["why"].items())),
              100.0 * tot["side"] / n, tot["off"] / n))
    print("side-steps by cause (why the spot under the ideal was refused: where the ideal was -> where the foot went):")
    for k, v in sorted(tot["sidewhy"].items(), key=lambda kv: -kv[1]):
        print("  %-28s %5d" % (k, v))


if __name__ == "__main__":
    main()
