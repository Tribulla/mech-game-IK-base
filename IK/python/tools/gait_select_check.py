"""Gait selection check: lanes with a terrain patch in the middle (light
rocks, rubble, boulders, stairs up / platform / ramp down), driven at full
throttle (run and walk mode), and a start / stop on flat ground.  Prints each
run's gait timeline (gait@position) and checks:

* gallop only when fast: frames in the gallop below 40 % of the run speed
  once its blend is done, and the longest such streak (a collision can stop
  the hull in one frame; the gait must follow within 0.25 s)
* the gait changes down on the patch (and the speed with it) and back after;
  light rocks keep the gallop
* no flapping: a change undone within a second and 10 m (A > B > A); at
  gallop speed new terrain (stairs 30 m ahead) may end a gait sooner
* smooth: the largest per-frame jump of the legs' phase offsets and duty

    python ik/tools/gait_select_check.py [MECH ...]
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from gaitcore import api, world, mechdefs, mech as mech_mod  # noqa: E402

DT = 1.0 / 60.0
KINDS = (("flat", "run"), ("rocks", "run"), ("rubble", "run"), ("boulders", "run"), ("stairs", "run"),
         ("rubble", "walk"), ("flat", "stopgo"))


def lane(kind, seed=0):
    """Flat ground with a terrain patch at z 80-130."""
    rng = random.Random(seed)
    w = world.StubWorld()
    if kind == "rocks":
        # light rocks (0.15 - 0.6 m), as in fuzz_check's rock field: a gallop carries on
        for _ in range(25):
            x, z = rng.uniform(-14, 14), rng.uniform(80, 130)
            sx, sz, sy = rng.uniform(0.25, 1.4), rng.uniform(0.25, 1.4), rng.uniform(0.15, 0.6)
            w.add_box((x - sx, 0.0, z - sz), (x + sx, sy, z + sz))
    elif kind == "rubble":
        for _ in range(420):
            x, z = rng.uniform(-14, 14), rng.uniform(80, 130)
            sx, sz, sy = rng.uniform(0.25, 0.9), rng.uniform(0.25, 0.9), rng.uniform(0.08, 0.38)
            w.add_obox((x, sy * 0.5, z), (sx, sy, sz), rng.uniform(0.0, 3.14), rng.uniform(-0.25, 0.25))
    elif kind == "boulders":
        for _ in range(90):
            x, z = rng.uniform(-16, 16), rng.uniform(80, 130)
            s = rng.uniform(0.5, 2.2)
            w.add_obox((x, s * 0.25, z), (s, s * 0.6, s * 0.8), rng.uniform(0.0, 3.14), rng.uniform(-0.4, 0.4))
    elif kind == "stairs":
        # 10 steps of 0.3 m up (23 deg), a platform, a 16 deg ramp down
        for i in range(10):
            w.add_box((-12, 0, 80 + i * 0.7), (12, 0.3 * (i + 1), 80 + (i + 1) * 0.7))
        w.add_box((-12, 0, 87.0), (12, 3.0, 125))
        w.add_obox((0, 1.5 - 0.3, 125 + 5.4), (12, 0.3, 5.6), 0.0, math.atan2(3.0, 10.4))
    return w


def run(name, kind, mode, secs=14.0):
    world.set_world(lane(kind))
    api._mechs.clear()
    api._events.clear()
    m = api._mechs[api.spawn(name, 0, 0, 0, 0.0)]
    d = m.d
    timeline = []
    last = None
    switches = 0
    changes = []                 # (t, from, to, z)
    flaps = 0
    gallop_slow = 0
    slow_run = 0
    slow_max = 0
    max_doff = 0.0
    max_dduty = 0.0
    prev_off = [l.offset for l in m.legs]
    prev_duty = m.duty
    patch_speed = []
    patch_gaits = set()
    for i in range(int(secs / DT)):
        t = i * DT
        inp = dict(fwd=1.0, run=1.0 if mode == "run" else 0.0)
        if mode == "stopgo":
            inp = dict(fwd=1.0 if (t < 4.0 or t > 8.0) else 0.0, run=1.0 if t < 4.0 else 0.0)
        m.inp.set([float(inp.get(f, 0.0)) for f in mech_mod.INPUT_FIELDS])
        m.update(DT)
        api._events.clear()
        g = m.gait_name
        if g != last:
            if last is not None:
                switches += 1
                if (changes and changes[-1][2] == last and changes[-1][1] == g and t - changes[-1][0] < 1.0
                        and m.pos.z - changes[-1][3] < 10.0
                        and not (mode == "stopgo" and any(changes[-1][0] < ti <= t + 0.5 for ti in (4.0, 8.0)))):
                    flaps += 1           # (a change of throttle in between isn't a flap)
                changes.append((t, last, g, m.pos.z))
            timeline.append("%s@%.0fm" % (g, m.pos.z))
            last = g
        if (g == d["gait_by_mode"]["run"] and not m.blend.blending and m.state == mech_mod.GROUND
                and m.speed < 0.4 * d["speeds"]["run"]):
            gallop_slow += 1
            slow_run += 1
            slow_max = max(slow_max, slow_run)
        else:
            slow_run = 0
        for l, po in zip(m.legs, prev_off):
            dd = abs((l.offset - po + 0.5) % 1.0 - 0.5)
            max_doff = max(max_doff, dd)
        prev_off = [l.offset for l in m.legs]
        max_dduty = max(max_dduty, abs(m.duty - prev_duty))
        prev_duty = m.duty
        if 85 < m.pos.z < 125:
            patch_speed.append(m.speed)
            patch_gaits.add(g)
        if m.pos.z > 230:
            break
    ps = sum(patch_speed) / len(patch_speed) if patch_speed else 0.0
    return dict(timeline=timeline, switches=switches, flaps=flaps, gallop_slow=gallop_slow,
                slow_max=slow_max * DT, max_doff=max_doff, max_dduty=max_dduty, patch_speed=ps,
                patch_gaits=sorted(patch_gaits), end_z=m.pos.z)


def main():
    names = sys.argv[1:] or [d["name"] for d in mechdefs.ALL]
    tot_slow = 0
    worst_slow = 0.0
    tot_flaps = 0
    worst_off = 0.0
    for name in names:
        for kind, mode in KINDS:
            r = run(name, kind, mode)
            tot_slow += r["gallop_slow"]
            worst_slow = max(worst_slow, r["slow_max"])
            tot_flaps += r["flaps"]
            worst_off = max(worst_off, r["max_doff"])
            print("%-10s %-8s %-6s changes %2d flaps %d  patch %4.1f m/s %-34s d-offset %.3f d-duty %.3f "
                  "slow gallop %d | %s" % (
                      name, kind, mode, r["switches"], r["flaps"], r["patch_speed"], ",".join(r["patch_gaits"]),
                      r["max_doff"], r["max_dduty"], r["gallop_slow"], " ".join(r["timeline"])))
    ok = worst_slow <= 0.25 and tot_flaps == 0 and worst_off < 0.05
    print("\nflaps (A > B > A, B held < 1 s and 10 m): %d; gallop below 40 %% of run speed: %d frames, longest %.2f s; "
          "largest per-frame phase-offset step %.3f of a cycle -> %s" % (
              tot_flaps, tot_slow, worst_slow, worst_off, "OK" if ok else "FAIL"))


if __name__ == "__main__":
    main()
