"""Headless regression harness for the gait core (no game needed).

    python ik/tools/headless_sim.py            # all mechs, all scenarios
    python ik/tools/headless_sim.py STRIKER    # one mech

Runs each mech through scripted scenarios on a pure-Python stub world and
prints metrics that catch the classic procedural-gait failures: feet sliding
while planted, legs over-stretching, body sinking / launching, mechs that
don't make progress, and per-tick cost.
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from gaitcore import world, mechdefs, api  # noqa: E402
from gaitcore.vmath import V3  # noqa: E402
from gaitcore import mech as mech_mod  # noqa: E402

DT = 1.0 / 60.0


def build_world():
    w = world.StubWorld()
    # stairs going +z starting at z=20: 8 steps of 0.3 x 0.5
    for i in range(8):
        w.add_box((-4, 0, 20 + i * 0.5), (4, 0.3 * (i + 1), 20 + (i + 1) * 0.5))
    w.add_box((-4, 0, 24), (4, 2.4, 30))
    # a 1.5 m ledge at x=40
    w.add_box((36, 0, 10), (50, 1.5, 30))
    # a tall wall at z=-20 (climbing)
    w.add_box((-10, 0, -26), (10, 8, -20))
    return w


SCENARIOS = {
    # walking scenarios walk at ~4 m/s (WALK_PACE): full throttle is 18-36 m/s now
    "walk_flat": dict(start=(0, 0, -5), yaw=math.pi / 2, secs=5, inp=dict(fwd=1)),
    "walk_fast": dict(start=(0, 0, -5), yaw=math.pi / 2, secs=5, inp=dict(fwd=1), full=True),
    "run_flat": dict(start=(0, 0, -5), yaw=math.pi / 2, secs=4, inp=dict(fwd=1, run=1)),
    "sprint": dict(start=(0, 0, -5), yaw=math.pi / 2, secs=6, inp=dict(fwd=1, run=1)),
    "slow_turn": dict(start=(0, 0, -5), yaw=0, secs=4, inp=dict(fwd=0.5, slow=1, turn=1)),
    "stairs": dict(start=(0, 0, 14), yaw=0, secs=9, inp=dict(fwd=1)),
    "ledge": dict(start=(30, 0, 20), yaw=math.pi / 2, secs=7, inp=dict(fwd=1)),
    "wall": dict(start=(0, 0, -12), yaw=math.pi, secs=10, inp=dict(fwd=1)),
    "drift": dict(start=(0, 0, -5), yaw=math.pi / 2, secs=5, inp=dict(fwd=1, run=1),
                  late=dict(t=2.0, inp=dict(fwd=1, run=1, drift=1, turn=1))),
    "jump": dict(start=(0, 0, -5), yaw=math.pi / 2, secs=3, inp=dict(fwd=0.5),
                 late=dict(t=1.0, inp=dict(fwd=0.5, jump=1))),
    "idle_settle": dict(start=(0, 0, -5), yaw=0, secs=4, inp=dict(fwd=1),
                        late=dict(t=1.3, inp=dict())),
}


WALK_PACE = 4.0


def _throttle(d, sc, inp):
    """Walking inputs at WALK_PACE m/s (unless the scenario wants full throttle)."""
    if sc.get("full") or inp.get("run") or not inp.get("fwd"):
        return inp
    inp = dict(inp)
    if inp.get("slow"):
        inp["fwd"] = inp["fwd"] * min(1.0, 0.5 * WALK_PACE / d["speeds"]["slow"])   # a creep
    else:
        inp["fwd"] = inp["fwd"] * min(1.0, WALK_PACE / d["speeds"]["walk"])
    return inp


def run(mname, sname, sc, verbose=False):
    w = world.set_world(build_world())
    api._mechs.clear()
    api._events.clear()
    h = api.spawn(mname, *sc["start"], sc["yaw"])
    m = api._mechs[h]
    start = m.pos.copy()
    max_stretch = 0.0
    slip = 0.0
    min_clear = 1e9
    steps = 0
    last_feet = [(l.planted, l.foot.copy()) for l in m.legs]
    t0 = time.perf_counter()
    ticks = int(sc["secs"] / DT)
    for i in range(ticks):
        t = i * DT
        inp = sc["inp"]
        if "late" in sc and t >= sc["late"]["t"]:
            inp = sc["late"]["inp"]
        inp = _throttle(m.d, sc, inp)
        vals = [float(inp.get(f, 0.0)) for f in mech_mod.INPUT_FIELDS]
        m.inp.set(vals)
        m.update(DT)
        for l, (was, pos) in zip(m.legs, last_feet):
            if was and l.planted and not m.drifting:
                slip = max(slip, l.foot.dist(pos))
            max_stretch = max(max_stretch, l.stretch if l.planted else 0.0)
        steps += sum(1 for e in api._events if e[0] == mech_mod.EV_STEP)
        api._events.clear()
        last_feet = [(l.planted, l.foot.copy()) for l in m.legs]
        g = w.raycast(m.pos, V3(0, -1, 0), 50)
        if g and m.state == mech_mod.GROUND and not m.on_wall:
            min_clear = min(min_clear, g.t)
    ms = (time.perf_counter() - t0) * 1000.0 / ticks
    moved = m.pos - start
    res = dict(mech=mname, scen=sname, moved=moved.length(), dy=m.pos.y - start.y,
               slip=slip, stretch=max_stretch, steps=steps, min_clear=min_clear,
               state=mech_mod.STATE_NAMES[m.state], wall=m.on_wall, ms=ms,
               up_y=m.frame.up.y, leaps=m.flight_count)
    return res


def main():
    names = sys.argv[1:] or [d["name"] for d in mechdefs.ALL]
    print("%-10s %-12s %7s %6s %6s %7s %5s %6s %-7s %5s %6s" % (
        "mech", "scenario", "moved", "dy", "slip", "stretch", "steps", "clear", "state", "upY", "ms/tk") + " leaps")
    bad = 0
    for n in names:
        d = mechdefs.BY_NAME[n]
        for sname, sc in SCENARIOS.items():
            if sname == "wall" and not (d["climber"] or d.get("wire_max", 0.0) >= 8.0):
                continue
            if sname == "ledge" and d["mantle_max"] <= 0:
                continue
            if sname == "jump" and d["jump_speed"] <= 0:
                continue
            r = run(n, sname, sc)
            flag = ""
            if r["slip"] > 0.02:
                flag += " SLIP"
            if r["stretch"] > 1.05:
                flag += " STRETCH"
            if r["min_clear"] < d["ride"] * 0.4:
                flag += " LOW"
            bad += bool(flag)
            print("%-10s %-12s %7.2f %6.2f %6.3f %7.2f %5d %6.2f %-7s %5.2f %6.2f%s" % (
                r["mech"], r["scen"], r["moved"], r["dy"], r["slip"], r["stretch"], r["steps"],
                r["min_clear"] if r["min_clear"] < 1e8 else -1, r["state"], r["up_y"], r["ms"], flag)
                  + ("  %d" % r["leaps"] if r["leaps"] else ""))
    print("\n%d flagged runs" % bad)


if __name__ == "__main__":
    main()
