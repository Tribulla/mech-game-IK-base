"""Unit tests for gait selection and blending (gaitcore/gaitsel.py).
    python -m unittest discover -s ik/tests"""
import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from gaitcore import gaits, mechdefs  # noqa: E402
from gaitcore.gaitsel import GaitBlend, GaitSelector, Situation, H_FAST  # noqa: E402

DT = 1.0 / 60.0


def fake_mech(name="STRIKER", mode="run"):
    """Just what GaitSelector reads from a Mech."""
    d = next(x for x in mechdefs.ALL if x["name"] == name)
    sit = Situation()
    return SimpleNamespace(d=d, situation=sit, gait_override=None, drifting=False, climbing_mode=False,
                           mode=mode, speed=0.0, v_desired=d["speeds"][mode], v_slow=4.0, v_walk=7.0,
                           cycle=0.5, balance_out=None, calm_gait=False)


class Blend(unittest.TestCase):
    def test_offsets_slide_and_rules_hand_over_late(self):
        q = gaits.quadruped()
        b = GaitBlend(q["walk"])
        b.start(q["trot_run"], 0.5)
        self.assertEqual(b.label(), "walk > trot_run")
        prev = list(b.eff.offsets)
        steps = 0
        while b.blending:
            e = b.update(DT)
            steps += 1
            for x, y in zip(prev, e.offsets):
                self.assertLess(abs((x - y + 0.5) % 1.0 - 0.5), 0.05)   # no jump
            if b.s < 0.85:
                self.assertIsNone(e.groups)          # groups / phase lock only at the end
                self.assertFalse(e.phase_lock)
            prev = list(e.offsets)
        self.assertIn(steps, (30, 31))            # 0.5 s at 60 fps (float rounding)
        for x, y in zip(b.eff.offsets, q["trot_run"].offsets):
            self.assertAlmostEqual(x % 1.0, y % 1.0, places=6)
        self.assertEqual(b.eff.groups, q["trot_run"].groups)
        self.assertEqual(b.label(), "trot_run")

    def test_support_follows_the_duty(self):
        o = gaits.octopod()
        b = GaitBlend(o["metachronal"])        # min_support 5
        b.start(o["tetrapod_run"], 0.6)       # min_support 0
        b.update(DT)
        b.fit_support(0.55)                   # 0.55 x 8 legs: 4 down on average
        self.assertEqual(b.eff.min_support, 4)
        b.fit_support(0.70)
        self.assertEqual(b.eff.min_support, 5)


class Selector(unittest.TestCase):
    def run_speeds(self, m, sel, speeds):
        out = []
        for v in speeds:
            m.speed = v
            name, reason, cap = sel.select(m, DT)
            out.append(name)
        return out

    def test_gallop_only_when_fast(self):
        m = fake_mech()
        sel = GaitSelector(m)
        run = m.d["speeds"]["run"]
        ramp = [run * i / 300.0 for i in range(301)]
        names = self.run_speeds(m, sel, ramp)
        first = names.index("bound")
        self.assertGreater(ramp[first], 0.55 * run - 1e-6)
        self.assertIn(m.d["walk_fast"], names[:first])          # walk -> fast -> gallop
        # walk mode never gallops
        m2 = fake_mech(mode="walk")
        sel2 = GaitSelector(m2)
        self.assertNotIn("bound", self.run_speeds(m2, sel2, ramp))

    def test_rough_ground_caps_rung_and_speed(self):
        m = fake_mech()
        sel = GaitSelector(m)
        self.run_speeds(m, sel, [30.0] * 120)
        self.assertEqual(sel.ladder[sel.rung], "bound")
        m.situation.harsh, m.situation.reason = H_FAST + 0.1, "rough ground"
        # at 30 m/s the walk can't carry the speed: the gait waits for the
        # speed to come down (speed cap), one rung at a time
        name, reason, cap = sel.select(m, DT)
        self.assertNotEqual(name, m.d["gait_by_mode"]["walk"])
        self.assertAlmostEqual(cap, 0.75 * m.v_walk)
        names = self.run_speeds(m, sel, [30.0 - i * 0.25 for i in range(117)] + [m.v_walk * 0.75] * 120)
        self.assertEqual(names[-1], m.d["gait_by_mode"]["walk"])
        # bound -> fast -> walk, never straight from the gallop to the walk
        seq = [n for i, n in enumerate(names) if i == 0 or n != names[i - 1]]
        self.assertEqual(seq[-2:], [m.d["walk_fast"], m.d["gait_by_mode"]["walk"]])

    def test_cap_lifts_only_after_steady_better_ground(self):
        m = fake_mech()
        sel = GaitSelector(m)
        m.situation.harsh = 0.5
        sel.select(m, DT)
        self.assertEqual(sel.cap, 1)
        # better for 1 s, a relapse, better again: still capped
        for i in range(60):
            m.situation.harsh = 0.0
            sel.select(m, DT)
        m.situation.harsh = 0.3                 # between: not good enough to lift the next rung
        sel.select(m, DT)
        m.situation.harsh = 0.0
        for i in range(60):
            sel.select(m, DT)
        self.assertEqual(sel.cap, 1)
        for i in range(60):
            sel.select(m, DT)
        self.assertEqual(sel.cap, 2)            # 1.5 s without a break: one rung


if __name__ == "__main__":
    unittest.main()
