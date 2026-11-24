"""Joint-limit check: runs every mech through the headless scenarios and
reports how often the IK limits engage, the worst joint angles seen (they must
stay inside mechdefs `limits`), and how far feet end up from their targets.

    python ik/tools/limit_check.py [MECH ...]
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

import headless_sim as hs  # noqa: E402
from gaitcore import api, world, mechdefs, mech as mech_mod  # noqa: E402


def main():
    names = sys.argv[1:] or [d["name"] for d in mechdefs.ALL]
    bad_total = 0
    for name in names:
        d = mechdefs.BY_NAME[name]
        frames = limited = 0
        worst_err = 0.0
        out_of_range = 0
        yaw = [1e9, -1e9]
        hip = [1e9, -1e9]
        bends = {}
        for sname, sc in hs.SCENARIOS.items():
            if sname == "wall" and not (d["climber"] or d.get("wire_max", 0.0) >= 8.0):
                continue
            if sname == "ledge" and d["mantle_max"] <= 0:
                continue
            if sname == "jump" and d["jump_speed"] <= 0:
                continue
            world.set_world(hs.build_world())
            api._mechs.clear()
            m = api._mechs[api.spawn(name, *sc["start"], sc["yaw"])]
            for i in range(int(sc["secs"] / hs.DT)):
                t = i * hs.DT
                inp = sc["inp"]
                if "late" in sc and t >= sc["late"]["t"]:
                    inp = sc["late"]["inp"]
                m.inp.set([float(inp.get(f, 0.0)) for f in mech_mod.INPUT_FIELDS])
                m.update(hs.DT)
                for l in m.legs:
                    ik = l.ik
                    frames += 1
                    limited += ik.limited
                    if l.planted:
                        worst_err = max(worst_err, l.joints[-1].dist(l.foot))
                    yaw = [min(yaw[0], ik.yaw_deg), max(yaw[1], ik.yaw_deg)]
                    hip = [min(hip[0], ik.hip_deg), max(hip[1], ik.hip_deg)]
                    h0, h1 = (math.degrees(a) for a in ik.hip_lim)
                    ok = h0 - 0.5 <= ik.hip_deg <= h1 + 0.5
                    for j, b in enumerate(ik.bends_deg):
                        r = bends.setdefault(j, [1e9, -1e9])
                        bends[j] = [min(r[0], b), max(r[1], b)]
                        b0, b1 = (math.degrees(a) for a in ik.bend_lim[j])
                        ok = ok and (b0 - 0.5 <= b <= b1 + 0.5)
                    if d["plane"] == "radial" and abs(ik.yaw_deg) > math.degrees(ik.coxa_max) + 0.5:
                        ok = False
                    out_of_range += not ok
        bad_total += out_of_range
        hs_off = -90.0 if d["plane"] == "sagittal" else 0.0
        s = "%-10s limited %4.1f%% of leg-frames  worst planted-foot error %.3f m  out of range %d | yaw %+4.0f..%+4.0f hip %+4.0f..%+4.0f" % (
            name, 100.0 * limited / max(1, frames), worst_err, out_of_range, yaw[0], yaw[1], hip[0] - hs_off, hip[1] - hs_off)
        for j, r in bends.items():
            s += " bend%d %+4.0f..%+4.0f" % (j + 1, r[0], r[1])
        print(s)
    print("\n%d joint angles outside their limits" % bad_total)


if __name__ == "__main__":
    main()
