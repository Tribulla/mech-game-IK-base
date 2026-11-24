"""Gait film: record a mech in the Python sim and render what its legs do.

    python ik/tools/gait_film.py STRIKER --mode walk --throttle 1 --start 4 --secs 0.8 --out film.png
        [--every 2] [--gif film.gif] [--gait NAME] [--aim RAD] [--jump-at T] [--ledge H]
        [--lane rocks|rubble|boulders|stairs]

--ledge H puts the mech on a platform H m high whose edge is 1.5 m behind it
(throttle < 0 backs it off); --aim / --jump-at look up / down and jump.
--lane runs on gait_select_check's lane (flat, terrain patch at z 80-130) to
film the situation's gait changes; panel titles show the blend ("bound >
trot_run") and the footfall strip marks where each change starts.

The PNG is a contact sheet: one panel per sampled frame with a side view (the
hull stays put, the ground scrolls past) above a top view; planted feet are
filled, lifted feet hollow; a footfall diagram (stance bars per leg) runs along
the bottom.  --gif writes the same frames as an animation.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from gaitcore import api, world, mechdefs, mech as mech_mod  # noqa: E402
from gaitcore.vmath import V3  # noqa: E402

DT = 1.0 / 60.0


def record(name, mode, throttle, start, secs, gait=None, turn=0.0, aim=0.0, jump_at=None, ledge=0.0, lane=None):
    w = world.StubWorld()
    if lane:
        sys.path.insert(0, os.path.dirname(__file__))
        import gait_select_check
        w = gait_select_check.lane(lane)
    elif ledge > 0.0:
        w.add_box((-30, 0, 0), (30, ledge, 60))
    world.set_world(w)
    api._mechs.clear()
    api._events.clear()
    m = api._mechs[api.spawn(name, 0, ledge, 1.5 if ledge > 0.0 else 0.0, 0.0)]
    if gait:
        m.gait_override = gait
    inp = dict(fwd=throttle, run=1.0 if mode == "run" else 0.0, slow=1.0 if mode == "slow" else 0.0)
    frames = []
    for i in range(int((start + secs) / DT)):
        if i * DT >= start:
            inp["turn"] = turn
        inp["aim"] = aim
        inp["jump"] = 1.0 if (jump_at is not None and jump_at <= i * DT < jump_at + 0.05) else 0.0
        m.inp.set([float(inp.get(f, 0.0)) for f in mech_mod.INPUT_FIELDS])
        m.update(DT)
        api._events.clear()
        if i * DT >= start:
            fr = m.frame
            legs = []
            for l in m.legs:
                legs.append(dict(j=[(p.x, p.y, p.z) for p in l.joints], planted=l.planted,
                                 swing=l.swing.t if l.swing else None, search=l.searching))
            frames.append(dict(t=i * DT, pos=(m.pos.x, m.pos.y, m.pos.z), ledge=ledge, fwd=(fr.fwd.x, fr.fwd.y, fr.fwd.z),
                               up=(fr.up.x, fr.up.y, fr.up.z), right=(fr.right.x, fr.right.y, fr.right.z),
                               speed=m.speed, gait=m.gait_label(), flight=m.flight_t > 0.0, legs=legs,
                               half=m.d["body_half"], ride=m.ride))
    return m, frames


def _local(f, p):
    """World point -> (forward, up, right) relative to the hull, heading only."""
    hx, hz = f["fwd"][0], f["fwd"][2]
    n = math.hypot(hx, hz) or 1.0
    hx, hz = hx / n, hz / n
    dx, dy, dz = p[0] - f["pos"][0], p[1], p[2] - f["pos"][2]
    return dx * hx + dz * hz, dy, dx * hz - dz * hx


def draw(frames, every, title, out_png, out_gif=None):
    sel = frames[::every]
    cols = min(8, len(sel))
    rows = (len(sel) + cols - 1) // cols
    reach = 1.0
    for f in sel:
        for lg in f["legs"]:
            a = _local(f, lg["j"][0])
            b = _local(f, lg["j"][-1])
            reach = max(reach, abs(b[0] - a[0]), abs(b[2] - a[2]))
    W = max(4.0, reach * 1.4 + max(f["half"][2] for f in sel))
    fig = plt.figure(figsize=(cols * 2.4, rows * 3.6 + 1.4), dpi=90)
    gs = fig.add_gridspec(rows * 2 + 1, cols, height_ratios=[1.0, 0.8] * rows + [0.5])
    imgs = []
    for k, f in enumerate(sel):
        r, c = divmod(k, cols)
        for view in (0, 1):
            ax = fig.add_subplot(gs[r * 2 + view, c])
            ax.set_xlim(-W, W)
            ax.set_aspect("equal")
            ax.set_xticks([])
            ax.set_yticks([])
            hy = f["pos"][1]
            if view == 0:
                ax.set_ylim(-0.5, max(hy, f["ledge"]) + 3.0)
                ax.axhline(0.0, color="0.6", lw=0.8)
                if f["ledge"] > 0.0:
                    ex = -f["pos"][2]    # the platform's edge (heading +z)
                    ax.fill([ex, W, W, ex], [0, 0, f["ledge"], f["ledge"]], color="0.85")
                # scrolling ground ticks
                s0 = f["pos"][0] * 0 + f["pos"][2]
                for g in range(-20, 21):
                    gx = g * 1.0 - (s0 % 1.0)
                    if -W < gx < W:
                        ax.plot([gx, gx], [0, -0.15], color="0.7", lw=0.6)
                hz = f["half"][2]
                hh = f["half"][1]
                # hull as a pitched box (side)
                fy = f["fwd"][1]
                pitch = math.asin(max(-1, min(1, fy)))
                cx, cy = 0.0, hy
                pts = []
                for sx, sy in ((-hz, -hh), (hz, -hh), (hz, hh), (-hz, hh), (-hz, -hh)):
                    pts.append((cx + sx * math.cos(pitch) - sy * math.sin(pitch), cy + sx * math.sin(pitch) + sy * math.cos(pitch)))
                ax.plot([p[0] for p in pts], [p[1] for p in pts], color="0.25", lw=1.2)
                for li, lg in enumerate(f["legs"]):
                    L = [_local(f, p) for p in lg["j"]]
                    far = L[0][2] < 0  # right side legs drawn lighter
                    col = ("tab:blue" if li % 2 == 0 else "tab:orange")
                    ax.plot([p[0] for p in L], [p[1] for p in L], color=col, lw=1.4 if not far else 0.9,
                            alpha=1.0 if not far else 0.55)
                    fx, fy2 = L[-1][0], L[-1][1]
                    ax.plot([fx], [fy2], "o", ms=4, color=col, mfc=col if lg["planted"] else "white")
                ax.set_title("%.2fs %s%s" % (f["t"], f["gait"], " AIR" if f["flight"] else ""), fontsize=7)
            else:
                ax.set_ylim(-W * 0.7, W * 0.7)
                hz = f["half"][2]
                hx = f["half"][0]
                ax.plot([-hz, hz, hz, -hz, -hz], [-hx, -hx, hx, hx, -hx], color="0.25", lw=1.0)
                for li, lg in enumerate(f["legs"]):
                    L = [_local(f, p) for p in lg["j"]]
                    col = ("tab:blue" if li % 2 == 0 else "tab:orange")
                    ax.plot([p[0] for p in L], [-p[2] for p in L], color=col, lw=1.1)
                    ax.plot([L[-1][0]], [-L[-1][2]], "o", ms=4, color=col, mfc=col if lg["planted"] else "white")
    # footfall strip
    ax = fig.add_subplot(gs[rows * 2, :])
    n = len(frames[0]["legs"])
    for li in range(n):
        for k, f in enumerate(frames):
            if f["legs"][li]["planted"]:
                ax.add_patch(plt.Rectangle((f["t"], n - 1 - li), DT, 0.8, color="0.2"))
        ax.text(frames[0]["t"] - 0.01, n - 1 - li + 0.3, ("L" if li % 2 == 0 else "R") + str(li // 2), fontsize=7, ha="right")
    for f in frames:
        if f["flight"]:
            ax.add_patch(plt.Rectangle((f["t"], -0.6), DT, 0.4, color="tab:red", alpha=0.6))
    prev = None
    for f in frames:
        if f["gait"] != prev and prev is not None:
            ax.axvline(f["t"], color="tab:green", lw=1.0)
            ax.text(f["t"], n - 0.2, " " + f["gait"], fontsize=7, color="tab:green", va="bottom")
        prev = f["gait"]
    ax.set_xlim(frames[0]["t"], frames[-1]["t"] + DT)
    ax.set_ylim(-0.8, n)
    ax.set_yticks([])
    ax.set_xlabel("time (s): bars = stance, red = hull airborne, green = gait label changes", fontsize=8)
    fig.suptitle(title, fontsize=10)
    fig.savefig(out_png)
    plt.close(fig)
    if out_gif:
        from PIL import Image
        imgs = []
        for f in frames:
            fig = plt.figure(figsize=(6, 4.5), dpi=80)
            ax = fig.add_subplot(2, 1, 1)
            ax2 = fig.add_subplot(2, 1, 2)
            for a in (ax, ax2):
                a.set_xlim(-W, W)
                a.set_aspect("equal")
                a.set_xticks([])
                a.set_yticks([])
            hy = f["pos"][1]
            ax.set_ylim(-0.5, hy + 2.0)
            ax.axhline(0.0, color="0.6", lw=0.8)
            s0 = f["pos"][2]
            for g in range(-30, 31):
                gx = g * 1.0 - (s0 % 1.0)
                if -W < gx < W:
                    ax.plot([gx, gx], [0, -0.2], color="0.7", lw=0.6)
            hz, hh, hx = f["half"][2], f["half"][1], f["half"][0]
            pitch = math.asin(max(-1, min(1, f["fwd"][1])))
            pts = [(sx * math.cos(pitch) - sy * math.sin(pitch), hy + sx * math.sin(pitch) + sy * math.cos(pitch))
                   for sx, sy in ((-hz, -hh), (hz, -hh), (hz, hh), (-hz, hh), (-hz, -hh))]
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color="0.25", lw=1.5)
            ax2.set_ylim(-W * 0.6, W * 0.6)
            ax2.plot([-hz, hz, hz, -hz, -hz], [-hx, -hx, hx, hx, -hx], color="0.25", lw=1.2)
            for li, lg in enumerate(f["legs"]):
                L = [_local(f, p) for p in lg["j"]]
                col = ("tab:blue" if li % 2 == 0 else "tab:orange")
                far = L[0][2] < 0
                ax.plot([p[0] for p in L], [p[1] for p in L], color=col, lw=2 if not far else 1.2, alpha=1 if not far else 0.5)
                ax.plot([L[-1][0]], [L[-1][1]], "o", ms=5, color=col, mfc=col if lg["planted"] else "white")
                ax2.plot([p[0] for p in L], [-p[2] for p in L], color=col, lw=1.5)
                ax2.plot([L[-1][0]], [-L[-1][2]], "o", ms=5, color=col, mfc=col if lg["planted"] else "white")
            ax.set_title("%s  %.1f m/s  %.2fs%s" % (f["gait"], f["speed"], f["t"], "  AIR" if f["flight"] else ""), fontsize=9)
            fig.canvas.draw()
            w, h = fig.canvas.get_width_height()
            imgs.append(Image.frombuffer("RGBA", (w, h), fig.canvas.buffer_rgba(), "raw", "RGBA", 0, 1).convert("P"))
            plt.close(fig)
        imgs[0].save(out_gif, save_all=True, append_images=imgs[1:], duration=int(DT * 1000 * 2), loop=0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mech")
    ap.add_argument("--mode", default="walk")
    ap.add_argument("--throttle", type=float, default=1.0)
    ap.add_argument("--start", type=float, default=4.0)
    ap.add_argument("--secs", type=float, default=0.8)
    ap.add_argument("--every", type=int, default=2)
    ap.add_argument("--gait", default=None)
    ap.add_argument("--turn", type=float, default=0.0)
    ap.add_argument("--out", default="film.png")
    ap.add_argument("--aim", type=float, default=0.0)
    ap.add_argument("--jump-at", type=float, default=None)
    ap.add_argument("--ledge", type=float, default=0.0)
    ap.add_argument("--gif", default=None)
    ap.add_argument("--lane", default=None)
    a = ap.parse_args()
    m, frames = record(a.mech, a.mode, a.throttle, a.start, a.secs, a.gait, a.turn, a.aim, a.jump_at, a.ledge, a.lane)
    title = "%s %s throttle %.2f: %.1f m/s, gait %s" % (a.mech, a.mode, a.throttle, frames[-1]["speed"], frames[-1]["gait"])
    draw(frames, a.every, title, a.out, a.gif)
    print("wrote", a.out, a.gif or "")


if __name__ == "__main__":
    main()
