"""Fuzz check: random driving over a rock field, counting the two leg failures
that are hard to see in scripted scenarios.

    python ik/tools/fuzz_check.py [MECH ...] [--seeds N] [--secs S] [--world rocks|rubble|boulders]

* stuck   - a planted foot sits at its reach / joint limit (stretch > 0.97)
            for more than 0.15 s without being stepped
* in rock - a foot or knee is inside a rock (swinging through it, or planted
            under it)
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from gaitcore import api, world, mechdefs, mech as mech_mod  # noqa: E402
from gaitcore.vmath import V3  # noqa: E402

DT = 1.0 / 60.0


def rock_world(seed):
    """Flat ground with a field of rocks (0.2 - 1.8 m tall) around the origin."""
    rng = random.Random(seed)
    w = world.StubWorld()
    rocks = []
    for _ in range(140):
        x, z = rng.uniform(-45, 45), rng.uniform(-45, 45)
        if abs(x) < 4 and abs(z) < 4:
            continue  # clear spawn
        sx, sz = rng.uniform(0.25, 1.4), rng.uniform(0.25, 1.4)
        sy = rng.uniform(0.2, 1.8) if rng.random() < 0.3 else rng.uniform(0.15, 0.6)
        mn, mx = (x - sx, 0.0, z - sz), (x + sx, sy, z + sz)
        w.add_box(mn, mx)
        rocks.append((V3(*mn), V3(*mx)))
    return w, rocks


def rubble_world(seed):
    """Course A's rubble density (70 slabs per 16 x 12 m) over 36 x 36 m."""
    rng = random.Random(seed)
    w = world.StubWorld()
    for _ in range(470):
        x, z = rng.uniform(-18, 18), rng.uniform(-18, 18)
        if abs(x) < 2.5 and abs(z) < 2.5:
            continue
        sx, sz, sy = rng.uniform(0.25, 0.9), rng.uniform(0.25, 0.9), rng.uniform(0.08, 0.38)
        w.add_obox((x, sy * 0.5, z), (sx, sy, sz), rng.uniform(0.0, 3.14), rng.uniform(-0.25, 0.25))
    return w, None


def boulder_world(seed):
    """Course D's boulders, packed denser, on flat ground."""
    rng = random.Random(seed)
    w = world.StubWorld()
    for _ in range(110):
        x, z = rng.uniform(-22, 22), rng.uniform(-22, 22)
        if abs(x) < 4 and abs(z) < 4:
            continue
        sz = rng.uniform(0.5, 2.2)
        w.add_obox((x, sz * 0.25, z), (sz, sz * 0.6, sz * 0.8), rng.uniform(0.0, 3.14), rng.uniform(-0.4, 0.4))
    return w, None


def make_world(kind, seed):
    """(world, rocks): rocks is the AABB list for `inside`, None when the
    world's oriented boxes are used instead."""
    if kind == "rubble":
        return rubble_world(seed)
    if kind == "boulders":
        return boulder_world(seed)
    return rock_world(seed)


def inside(p, rocks, margin=0.03, w=None):
    if rocks is None:
        for c, h, ax in w._near(p.x, p.z, p.x, p.z):
            r = p - c
            if all(abs(r.dot(ax[i])) < h[i] - margin for i in range(3)):
                return True
        return False
    for mn, mx in rocks:
        if (mn.x + margin < p.x < mx.x - margin and mn.y + margin < p.y < mx.y - margin
                and mn.z + margin < p.z < mx.z - margin):
            return True
    return False


def drive(rng, t, state):
    """Random driver: new intent every 0.6 - 2 s."""
    if t >= state.get("next", 0.0):
        state["next"] = t + rng.uniform(0.6, 2.0)
        state["inp"] = dict(
            fwd=rng.choice((1, 1, 1, 0.5, 0, -1)),
            strafe=rng.choice((0, 0, 0, 1, -1)),
            turn=rng.choice((0, 0, 0.5, -0.5, 1, -1)),
            run=1.0 if rng.random() < 0.35 else 0.0,
            slow=1.0 if rng.random() < 0.1 else 0.0,
            jump=1.0 if rng.random() < 0.08 else 0.0,
            impact=1.0 if rng.random() < 0.05 else 0.0,
        )
    return state["inp"]


def run(name, seed, secs, kind="rocks"):
    rng = random.Random(seed * 7919 + sum(ord(c) for c in name))  # deterministic across runs
    w, rocks = make_world(kind, seed)
    world.set_world(w)
    api._mechs.clear()
    api._events.clear()
    m = api._mechs[api.spawn(name, 0, 0, 0, rng.uniform(0, math.tau))]
    state = {}
    stuck_t = [0.0] * m.n
    episodes = 0
    stuck_total = 0.0
    worst = 0.0
    foot_in = knee_in = planted_in = 0
    frames = 0
    for i in range(int(secs / DT)):
        t = i * DT
        inp = drive(rng, t, state)
        m.inp.set([float(inp.get(f, 0.0)) for f in mech_mod.INPUT_FIELDS])
        m.update(DT)
        api._events.clear()
        frames += 1
        for l in m.legs:
            if l.planted and l.stretch > 0.97 and m.state == mech_mod.GROUND:
                stuck_t[l.idx] += DT
                if abs(stuck_t[l.idx] - 0.15) < DT * 0.5:
                    episodes += 1
                if stuck_t[l.idx] > 0.15:
                    stuck_total += DT
                worst = max(worst, stuck_t[l.idx])
            else:
                stuck_t[l.idx] = 0.0
            js = l.joints
            if js and inside(js[-1], rocks, w=w):
                foot_in += 1
                if l.planted:
                    planted_in += 1
            if len(js) >= 3 and any(inside(j, rocks, w=w) for j in js[1:-1]):
                knee_in += 1
    return dict(episodes=episodes, stuck=stuck_total, worst=worst, foot_in=foot_in, planted_in=planted_in,
                knee_in=knee_in, frames=frames * m.n, dist=m.pos.dist(V3(0, m.pos.y, 0)))


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
    kind = "rocks"
    if "--world" in args:
        i = args.index("--world")
        kind = args[i + 1]
        del args[i:i + 2]
    names = args or [d["name"] for d in mechdefs.ALL]
    print("%-10s %8s %9s %9s %10s %10s %10s" % ("mech", "stuck#", "stuck s", "worst s", "foot-in-%", "(planted)", "knee-in-%"))
    tot = dict(episodes=0, stuck=0.0, foot_in=0, planted_in=0, knee_in=0, frames=0)
    for name in names:
        agg = dict(episodes=0, stuck=0.0, worst=0.0, foot_in=0, planted_in=0, knee_in=0, frames=0)
        for s in range(seeds):
            r = run(name, s, secs, kind)
            for k in ("episodes", "stuck", "foot_in", "planted_in", "knee_in", "frames"):
                agg[k] += r[k]
            agg["worst"] = max(agg["worst"], r["worst"])
        for k in tot:
            tot[k] += agg[k]
        print("%-10s %8d %9.2f %9.2f %10.2f %10.2f %10.2f" % (
            name, agg["episodes"], agg["stuck"], agg["worst"],
            100.0 * agg["foot_in"] / agg["frames"], 100.0 * agg["planted_in"] / agg["frames"],
            100.0 * agg["knee_in"] / agg["frames"]))
    print("\ntotal: %d stuck episodes, %.1f s stuck, feet in rock %.2f%%, knees in rock %.2f%% of leg-frames" % (
        tot["episodes"], tot["stuck"], 100.0 * tot["foot_in"] / tot["frames"], 100.0 * tot["knee_in"] / tot["frames"]))


if __name__ == "__main__":
    main()
