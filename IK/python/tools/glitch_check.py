"""Glitch check: the leg glitches that are easy to see in the game and hard to
catch in scripted runs.

    python ik/tools/glitch_check.py [MECH ...] [--seeds N]

* stop steps/s  - after stopping on rough ground (rubble, rocks, boulders, a pit
                  edge), steps per second 6-10 s after the stop: a settled mech
                  takes none (the "stepping up and down non-stop" bug)
* quick re-step - a foot lifted again less than 0.15 s after it landed, in a
                  walking gait (not the flying trot / gallop), per 100 steps
* in-place      - a walking step that lands within 0.1 m of where it lifted, per
                  100 steps ("steps straight up again")
* floating %    - leg-frames with a planted foot that has nothing under it
* hanging %     - leg-frames with a foot hanging in the air while it searches
                  for ground ("stands on nothing")
* odd pose %    - all tests: leg-frames where a lifted foot is
                  more than half a leg reach to the side of its rest spot, or
                  more than 1.6 reaches ahead / behind it
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

from gaitcore import api, world, mechdefs, mech as mech_mod  # noqa: E402
from gaitcore.vmath import V3  # noqa: E402
import fuzz_check  # noqa: E402

DT = 1.0 / 60.0


def pit_world(seed):
    """A floor with an 8 m deep, 6 m wide trench along x (course A's pit)."""
    w = world.StubWorld(ground_y=-8.0)
    w.add_box((-60, -8.5, -60), (60, 0.0, -2.0))
    w.add_box((-60, -8.5, 4.0), (60, 0.0, 60))
    return w


def make(kind, seed):
    if kind == "pit":
        return pit_world(seed)
    return fuzz_check.make_world(kind, seed)[0]


class Probe:
    def __init__(self, m):
        self.m = m
        self.t = 0.0
        self.plant_t = [None] * m.n
        self.was_planted = [l.planted for l in m.legs]
        self.lift_pos = [None] * m.n
        self.steps = 0
        self.quick = 0
        self.inplace = 0
        self.leg_frames = 0
        self.floating = 0
        self.hanging = 0
        self.odd = 0
        self.odd_frames = 0
        self.odd_why = {}

    def frame(self, measure_pose=False):
        m = self.m
        w = m.world
        # a walking gait (the flying trot / gallop have brief contacts by design)
        walking = m.state == mech_mod.GROUND and m.gait.min_support > 0
        for l in m.legs:
            i = l.idx
            self.leg_frames += 1
            if l.planted and not self.was_planted[i]:
                self.plant_t[i] = self.t
                if m.state == mech_mod.GROUND:
                    self.steps += 1
                    if walking and self.lift_pos[i] is not None and l.foot.dist(self.lift_pos[i]) < 0.1:
                        self.inplace += 1
            if not l.planted and self.was_planted[i] and m.state == mech_mod.GROUND:
                self.lift_pos[i] = l.foot.copy() if l.swing is None else l.swing.start_w.copy()
                if walking and self.plant_t[i] is not None and self.t - self.plant_t[i] < 0.15:
                    self.quick += 1
            self.was_planted[i] = l.planted
            if l.planted and m.state == mech_mod.GROUND and not m.climbing_mode:
                h = w.raycast(l.foot + V3(0, 0.3, 0), V3(0, -1, 0), 0.45)
                c2, n = w.sphere_push(l.foot - V3(0, 0.05, 0), 0.08)
                if h is None and n is None:
                    self.floating += 1
            if l.searching and m.state == mech_mod.GROUND:
                self.hanging += 1
            if measure_pose and not l.planted and l.joints:
                self.odd_frames += 1
                fl = m.frame.to_local(l.foot - m.pos)
                side = abs(fl.x - l.rest_xz[0])
                ahead = fl.z - l.rest_xz[1]
                if side > 0.5 * l.reach or abs(ahead) > 1.6 * l.reach:
                    self.odd += 1
                    st = ("flight" if m.flight_t > 0.0 else "search" if l.searching else
                          "swing<.5" if l.swing is not None and l.swing.t < 0.5 else
                          "swing>.5" if l.swing is not None else "other")
                    k = "%s %s" % ("side" if side > 0.5 * l.reach else "ahead" if ahead > 0 else "behind", st)
                    self.odd_why[k] = self.odd_why.get(k, 0) + 1
        self.t += DT


def spawn(name, w, x, z, yaw):
    world.set_world(w)
    api._mechs.clear()
    api._events.clear()
    h = api.spawn(name, x, 0, z, yaw)
    return api._mechs[h]


def step(m, inp):
    m.inp.set([float(inp.get(f, 0.0)) for f in mech_mod.INPUT_FIELDS])
    m.update(DT)
    api._events.clear()


def stop_test(name, kind, seed):
    """Walk onto rough ground, stop, count steps after settling."""
    rng = random.Random(seed * 31 + sum(ord(c) for c in name) + len(kind))
    w = make(kind, seed)
    if kind == "pit":
        m = spawn(name, w, rng.uniform(-20, 20), -6.0, rng.uniform(-0.4, 0.4))   # walks towards the edge
        walk_t = rng.uniform(0.8, 1.6)
    else:
        m = spawn(name, w, 0, 0, rng.uniform(0, math.tau))
        walk_t = rng.uniform(1.5, 3.5)
    pr = Probe(m)
    fwd = rng.choice((0.15, 0.3, 0.5))
    for i in range(int(walk_t / DT)):
        step(m, dict(fwd=fwd))
        pr.frame()
    for i in range(int(6.0 / DT)):     # settling: one leg at a time, a few seconds at most
        step(m, {})
        pr.frame()
    s0 = pr.steps
    for i in range(int(4.0 / DT)):     # settled: no more steps
        step(m, {})
        pr.frame()
    return (pr.steps - s0) / 4.0, pr


def walk_test(name, kind, seed, run=False):
    rng = random.Random(seed * 17 + sum(ord(c) for c in name) + len(kind) + (7 if run else 0))
    w = make(kind, seed)
    m = spawn(name, w, 0, 0 if kind != "pit" else -12.0, rng.uniform(0, math.tau) if kind != "pit" else 0.0)
    pr = Probe(m)
    state = {}
    for i in range(int(14.0 / DT)):
        t = i * DT
        if t >= state.get("next", 0.0):
            state["next"] = t + rng.uniform(0.6, 2.0)
            state["inp"] = dict(fwd=rng.choice((1, 1, 0.6, 0.3, 0)), turn=rng.choice((0, 0, 0.4, -0.4, 1, -1)),
                                strafe=rng.choice((0, 0, 0, 1, -1)), run=1.0 if run else 0.0)
        step(m, state["inp"])
        pr.frame(measure_pose=True)
    return pr


def main():
    args = sys.argv[1:]
    seeds = 3
    if "--seeds" in args:
        i = args.index("--seeds")
        seeds = int(args[i + 1])
        del args[i:i + 2]
    names = args or [d["name"] for d in mechdefs.ALL]
    print("%-10s %9s %9s %8s %9s %9s %10s %10s" % ("mech", "stop st/s", "worst", "quick/100", "inplace", "float %",
                                                   "hanging %", "odd pose %"))
    tot = dict(stop=0.0, n=0, quick=0, inplace=0, steps=0, lf=0, fl=0, hang=0, odd=0, oddf=0)
    why = {}
    for name in names:
        rates = []
        agg = dict(quick=0, inplace=0, steps=0, lf=0, fl=0, hang=0, odd=0, oddf=0)

        def add(pr):
            agg["quick"] += pr.quick
            agg["inplace"] += pr.inplace
            agg["steps"] += pr.steps
            agg["lf"] += pr.leg_frames
            agg["fl"] += pr.floating
            agg["hang"] += pr.hanging
            agg["odd"] += pr.odd
            agg["oddf"] += pr.odd_frames
            for k, v in pr.odd_why.items():
                why[k] = why.get(k, 0) + v
        for s in range(seeds):
            for kind in ("rubble", "rocks", "boulders", "pit"):
                r, pr = stop_test(name, kind, s)
                rates.append(r)
                add(pr)
            for kind in ("rubble", "boulders", "pit"):
                add(walk_test(name, kind, s))
            for kind in ("rocks", "rubble"):
                add(walk_test(name, kind, s, run=True))
        for k in agg:
            tot[k] += agg[k]
        tot["stop"] += sum(rates)
        tot["n"] += len(rates)
        print("%-10s %9.2f %9.2f %8.1f %9.1f %9.2f %10.2f %10.2f" % (
            name, sum(rates) / len(rates), max(rates), 100.0 * agg["quick"] / max(1, agg["steps"]),
            100.0 * agg["inplace"] / max(1, agg["steps"]), 100.0 * agg["fl"] / max(1, agg["lf"]),
            100.0 * agg["hang"] / max(1, agg["lf"]), 100.0 * agg["odd"] / max(1, agg["oddf"])))
    print("\ntotal: stop steps/s %.2f, quick re-steps %.1f / 100 steps, in-place %.1f / 100, floating %.2f %%, "
          "hanging %.2f %%, odd pose %.2f %%" % (
              tot["stop"] / max(1, tot["n"]), 100.0 * tot["quick"] / max(1, tot["steps"]),
              100.0 * tot["inplace"] / max(1, tot["steps"]), 100.0 * tot["fl"] / max(1, tot["lf"]),
              100.0 * tot["hang"] / max(1, tot["lf"]), 100.0 * tot["odd"] / max(1, tot["oddf"])))
    print("odd poses by kind:", ", ".join("%s %d" % kv for kv in sorted(why.items(), key=lambda kv: -kv[1])))


if __name__ == "__main__":
    main()
