"""Unit tests for the IK solvers + helpers.   python -m unittest discover -s ik/tests"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from gaitcore.ik import solve_two_bone, solve_fabrik_2d, initial_pose_2d, LegIK  # noqa: E402
from gaitcore.vmath import LOCAL_FRAME, V3, Frame, convex_hull_2d, support_margin, fit_plane  # noqa: E402


class TwoBone(unittest.TestCase):
    def test_reaches_target(self):
        for tx, ty in ((1.0, -1.0), (0.3, -1.6), (1.5, 0.2), (-0.5, -1.2)):
            kx, ky, fx, fy, ok = solve_two_bone(1.0, 1.0, tx, ty, 1)
            self.assertTrue(ok)
            self.assertAlmostEqual(fx, tx, places=4)
            self.assertAlmostEqual(fy, ty, places=4)
            self.assertAlmostEqual(math.hypot(kx, ky), 1.0, places=5)

    def test_bend_side(self):
        kx, ky, *_ = solve_two_bone(1.0, 1.0, 0.0, -1.5, 1)
        self.assertGreater(kx, 0)  # +1: knee CCW of a downward line = forward
        kx, ky, *_ = solve_two_bone(1.0, 1.0, 0.0, -1.5, -1)
        self.assertLess(kx, 0)

    def test_unreachable_is_straight_and_rigid(self):
        kx, ky, fx, fy, ok = solve_two_bone(1.0, 1.0, 5.0, 0.0, 1)
        self.assertFalse(ok)
        self.assertAlmostEqual(math.hypot(fx - kx, fy - ky), 1.0, places=5)


class Fabrik(unittest.TestCase):
    def test_three_segment_digitigrade(self):
        L = [1.25, 1.35, 0.72]
        sides = [1, -1]
        tx, ty = 0.2, -2.3
        pts = initial_pose_2d(L, sides, tx, ty)
        self.assertTrue(solve_fabrik_2d(L, pts, tx, ty, sides, iters=40))
        self.assertLess(math.hypot(pts[3][0] - tx, pts[3][1] - ty), 1e-2)
        for i in range(3):
            seg = math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            self.assertAlmostEqual(seg, L[i], places=3)
        d = math.hypot(tx, ty)
        lx, ly = tx / d, ty / d
        s1 = lx * pts[1][1] - ly * pts[1][0]
        s2 = lx * pts[2][1] - ly * pts[2][0]
        self.assertGreater(s1, 0)  # knee forward
        self.assertLess(s2, 0)     # ankle back

    def test_warm_start_tracks_moving_target(self):
        L = [1.0, 1.0, 0.5]
        sides = [1, 1]
        pts = initial_pose_2d(L, sides, 1.2, -1.0)
        for k in range(60):
            tx = 1.2 + 0.6 * math.sin(k * 0.2)
            solve_fabrik_2d(L, pts, tx, -1.0, sides)
        self.assertLess(math.hypot(pts[3][0] - tx, pts[3][1] + 1.0), 2e-2)


class LegSolve(unittest.TestCase):
    def test_radial_leg_foot_on_target(self):
        fr = Frame(V3(0, 0, 1), V3(0, 1, 0))
        ik = LegIK("radial", 0.3, [0.95, 1.25], [1], V3(-1, 0, 0))
        mount = V3(0, 1, 0)
        target = V3(1.5, 0.0, 0.4)
        js = ik.solve(mount, fr, target)
        self.assertLess(js[-1].dist(target), 1e-3)
        self.assertGreater(js[2].y, mount.y - 0.3)  # knee up-ish

    def test_sagittal_leg(self):
        fr = Frame(V3(0, 0, 1), V3(0, 1, 0))
        ik = LegIK("sagittal", 0.0, [1.25, 1.35, 0.72], [1, -1], V3(0, 0, 1))
        mount = V3(0.5, 2.3, 0)
        target = V3(0.55, 0.0, 0.4)
        for _ in range(5):
            js = ik.solve(mount, fr, target)
        self.assertLess(js[-1].dist(target), 2e-2)


class JointLimits(unittest.TestCase):
    """LegIK never leaves its joint limits, still hits legal targets exactly,
    and its cheap feasibility test agrees with the full solve."""

    LIMITS3 = dict(coxa=60.0, hip=(-30.0, 70.0), bend=[(10.0, 140.0), (15.0, 140.0)])

    def _leg(self, limits=LIMITS3, lengths=(1.6, 1.5, 0.55), sides=(1, 1)):
        return LegIK("radial", 0.3, lengths, sides, V3(1, 0, 0), limits)

    def _within(self, ik):
        h0, h1 = (math.degrees(a) for a in ik.hip_lim)
        ok = h0 - 0.5 <= ik.hip_deg <= h1 + 0.5
        for (b0, b1), b in zip(ik.bend_lim, ik.bends_deg):
            ok = ok and math.degrees(b0) - 0.5 <= b <= math.degrees(b1) + 0.5
        return ok and abs(ik.yaw_deg) <= math.degrees(ik.coxa_max) + 0.5

    def test_sweep_never_leaves_limits(self):
        ik = self._leg()
        fr = Frame(V3(0, 0, 1), V3(0, 1, 0))
        mount = V3(0, 0, 0)
        for i in range(400):
            a = i * 0.07
            # a target that wanders all around (and through) the hip, incl. behind and above it
            target = V3(2.4 * math.cos(a), 1.5 * math.sin(a * 1.7) - 0.6, 2.4 * math.sin(a * 0.9))
            ik.solve(mount, fr, target)
            self.assertTrue(self._within(ik), "leg left its limits at step %d" % i)

    def test_legal_target_reached_exactly(self):
        ik = self._leg()
        fr = Frame(V3(0, 0, 1), V3(0, 1, 0))
        mount = V3(0, 0, 0)
        target = V3(-2.0, -1.4, 0.3)  # outward (x is outward for the rest dir), below the hip
        # rest direction is +x in body space = -x world (right-handed frame, see Geometry)
        for _ in range(5):
            js = ik.solve(mount, fr, target)
        self.assertLess(js[-1].dist(target), 2e-2)
        self.assertLess(ik.err, 2e-2)

    def test_coxa_stop_reports_error(self):
        ik = self._leg()
        fr = Frame(V3(0, 0, 1), V3(0, 1, 0))
        rest = fr.to_world(V3(1, 0, 0))
        behind = V3(0, -1.2, 0) + rest.cross(V3(0, 1, 0)) * 2.2   # 90 deg from rest, past the 60 deg stop
        ik.solve(V3(0, 0, 0), fr, behind)
        self.assertLessEqual(abs(ik.yaw_deg), 60.5)
        self.assertGreater(ik.err, 0.3)

    def test_feasible_matches_solve(self):
        ik = self._leg(lengths=(1.4, 1.6), sides=(1,),
                       limits=dict(coxa=60.0, hip=(-30.0, 75.0), bend=(8.0, 150.0)))
        fr = Frame(V3(0, 0, 1), V3(0, 1, 0))
        mount = V3(0, 0, 0)
        agree = total = 0
        for ix in range(-6, 7):
            for iz in range(-6, 7):
                target = V3(ix * 0.45, -1.15, iz * 0.45)
                f = ik.feasible(mount, fr, target, slack_deg=0.0)
                probe = self._leg(lengths=(1.4, 1.6), sides=(1,),
                                  limits=dict(coxa=60.0, hip=(-30.0, 75.0), bend=(8.0, 150.0)))
                for _ in range(4):
                    probe.solve(mount, fr, target)
                agree += f == (probe.err < 0.03)
                total += 1
        self.assertGreater(agree / total, 0.9)

    def test_three_link_limit_continuity_and_feasibility(self):
        lim = dict(coxa=75.0, hip=(-35.0, 80.0), bend=(8.0, 150.0))
        ik = LegIK("radial", 0.25, (1.4, 1.5, 1.1), (1, 1), V3(1, 0, 0), lim)
        prev = None
        hit_limit = False
        for k in range(80):
            u = k / 79.0
            target = V3(1.1 + 1.8 * u, 0.2 - 3.6 * u, 0.15)
            feas = ik.feasible(V3(), LOCAL_FRAME, target, slack_deg=0.0)
            ik.solve(V3(), LOCAL_FRAME, target)
            self.assertTrue(self._within(ik))
            if ik.limited:
                hit_limit = True
            if feas:
                self.assertLess(ik.err, 0.01)
            cur = [ik.hip_deg] + list(ik.bends_deg)
            if prev is not None and feas:
                self.assertLess(max(abs(a - b) for a, b in zip(cur, prev)), 12.0)
            prev = cur
        self.assertTrue(hit_limit)

        agree = total = 0
        for ix in range(-4, 9):
            for iz in range(-4, 5):
                target = V3(ix * 0.45, -1.35, iz * 0.45)
                f = ik.feasible(V3(), LOCAL_FRAME, target, slack_deg=0.0)
                probe = LegIK("radial", 0.25, (1.4, 1.5, 1.1), (1, 1), V3(1, 0, 0), lim)
                for _ in range(4):
                    probe.solve(V3(), LOCAL_FRAME, target)
                agree += f == (probe.err < 0.03)
                total += 1
        self.assertGreater(agree / total, 0.95)

    def test_radial_plane_roll_preserves_foot_and_lengths(self):
        lengths = (1.4, 1.5, 1.1)
        lim = dict(coxa=75.0, hip=(-35.0, 80.0), bend=(8.0, 150.0))
        target = V3(2.2, -1.4, 0.2)
        ik0 = LegIK("radial", 0.25, lengths, (1, 1), V3(1, 0, 0), lim)
        ikp = LegIK("radial", 0.25, lengths, (1, 1), V3(1, 0, 0), lim)
        ikm = LegIK("radial", 0.25, lengths, (1, 1), V3(1, 0, 0), lim)
        j0 = ik0.solve(V3(), LOCAL_FRAME, target, plane_roll=0.0)
        n0 = ik0.plane_normal.copy()
        jp = ikp.solve(V3(), LOCAL_FRAME, target, plane_roll=0.3)
        jm = ikm.solve(V3(), LOCAL_FRAME, target, plane_roll=-0.3)
        self.assertLess(ikp.err, 1e-4)
        self.assertLess(ikm.err, 1e-4)
        self.assertLess(jp[-1].dist(target), 1e-4)
        self.assertLess(jm[-1].dist(target), 1e-4)
        self.assertLess((jp[2] - j0[2]).dot(n0) * (jm[2] - j0[2]).dot(n0), -1e-4)
        for seg, expected in zip(zip(jp, jp[1:]), (0.25,) + lengths):
            self.assertAlmostEqual(seg[0].dist(seg[1]), expected, places=5)
        self.assertAlmostEqual(ikp.hip_deg, ik0.hip_deg, places=3)
        for b1, b0 in zip(ikp.bends_deg, ik0.bends_deg):
            self.assertAlmostEqual(b1, b0, places=3)

    def test_radial_coxa_rear_branch_cut_no_flip(self):
        ik = LegIK("radial", 0.25, (1.2, 1.4), (1,), V3(1, 0, 0),
                   dict(coxa=75.0, hip=(-40.0, 80.0), bend=(10.0, 150.0)))
        ik.solve(V3(), LOCAL_FRAME, V3(0.2, -1.2, -1.4))
        prev_yaw = ik.yaw_deg
        for k in range(30):
            z = -0.3 + 0.6 * k / 29.0
            ik.solve(V3(), LOCAL_FRAME, V3(-0.8, -1.2, z))
            self.assertLess(abs(ik.yaw_deg - prev_yaw), 60.0)
            prev_yaw = ik.yaw_deg

    def test_dls_constrained_multilimit_and_mixed_side(self):
        # 2-link hitting bend/hip limit -> constrained DLS fallback
        lim2 = dict(coxa=60.0, hip=(-25.0, 65.0), bend=(25.0, 140.0))
        ik2 = LegIK("radial", 0.25, (1.2, 1.3), (1,), V3(1, 0, 0), lim2)
        prev = None
        hit_dls = False
        for k in range(40):
            u = k / 39.0
            target = V3(0.7 + 1.9 * u, 0.15 - 2.3 * u, 0.25 * math.sin(u * math.pi))
            feas = ik2.feasible(V3(), LOCAL_FRAME, target, slack_deg=0.0)
            ik2.solve(V3(), LOCAL_FRAME, target)
            self.assertTrue(self._within(ik2))
            if ik2.limited:
                hit_dls = True
            if feas:
                self.assertLess(ik2.err, 0.02)
                cur = [ik2.hip_deg] + list(ik2.bends_deg)
                if prev is not None:
                    self.assertLess(max(abs(a - b) for a, b in zip(cur, prev)), 12.0)
                prev = cur
            else:
                prev = None
        self.assertTrue(hit_dls)

        # 3-link mixed-side (1, -1) digitigrade with tight knee & ankle stops (uses FABRIK + DLS)
        lim3m = dict(coxa=60.0, hip=(-35.0, 75.0), bend=[(15.0, 95.0), (15.0, 95.0)])
        ik3 = LegIK("radial", 0.25, (1.25, 1.35, 0.72), (1, -1), V3(1, 0, 0), lim3m)
        prev = None
        hit_dls = False
        for k in range(50):
            u = k / 49.0
            target = V3(0.95 + 1.8 * u, -2.0 + 0.5 * u, 0.1 * u)
            ik3.solve(V3(), LOCAL_FRAME, target)
            self.assertTrue(self._within(ik3))
            if ik3.limited:
                hit_dls = True
            if ik3.err < 0.02:
                cur = [ik3.hip_deg] + list(ik3.bends_deg)
                if prev is not None:
                    self.assertLess(max(abs(a - b) for a, b in zip(cur, prev)), 12.0)
                prev = cur
            else:
                prev = None
        self.assertTrue(hit_dls)

        # 3-link same-side (1, 1) multi-limit sweep where both hip and knee constrain simultaneously
        lim3s = dict(coxa=60.0, hip=(-20.0, 60.0), bend=[(20.0, 125.0), (20.0, 125.0)])
        ik3s = LegIK("radial", 0.25, (1.4, 1.5, 1.1), (1, 1), V3(1, 0, 0), lim3s)
        hit_dls = False
        for k in range(45):
            u = k / 44.0
            target = V3(1.2 + 2.0 * u, 0.35 - 3.3 * u, -0.2 + 0.4 * u)
            ik3s.solve(V3(), LOCAL_FRAME, target)
            self.assertTrue(self._within(ik3s))
            if ik3s.limited:
                hit_dls = True
        self.assertTrue(hit_dls)


class Geometry(unittest.TestCase):
    def test_support_margin(self):
        hull = convex_hull_2d([(0, 0), (2, 0), (2, 2), (0, 2)])
        self.assertAlmostEqual(support_margin(hull, (1, 1)), 1.0)
        self.assertLess(support_margin(hull, (3, 1)), 0)

    def test_fit_plane(self):
        pts = [V3(x, 0.5 * x, z) for x in (-1, 0, 1) for z in (-1, 1)]
        c, n = fit_plane(pts)
        self.assertAlmostEqual(n.dot(V3(-0.5, 1, 0).normalized()), 1.0, places=5)

    def test_frame_right_handed_world(self):
        fr = Frame(V3(0, 0, 1), V3(0, 1, 0))
        # facing +z with y up in a right-handed world, the right hand is -x
        self.assertAlmostEqual(fr.right.x, -1.0)
        fr.yaw(math.pi / 2)  # turn right
        self.assertAlmostEqual(fr.fwd.x, -1.0, places=6)


class OrientedBoxes(unittest.TestCase):
    """The stub world's tilted rocks (match the app's World::AddRot)."""

    def setUp(self):
        from gaitcore import world
        self.w = world.StubWorld()
        self.w.add_obox((0, 0.5, 0), (1.0, 0.5, 1.0), 0.3, 0.2)

    def test_top_hit_follows_tilt(self):
        h = self.w.raycast(V3(0, 5, 0), V3(0, -1, 0), 10)
        self.assertAlmostEqual(h.point.y, 0.5 + 0.5 / math.cos(0.2), places=3)
        self.assertAlmostEqual(h.normal.y, math.cos(0.2), places=3)

    def test_miss_beside(self):
        h = self.w.raycast(V3(3, 5, 0), V3(0, -1, 0), 10)
        self.assertAlmostEqual(h.point.y, 0.0, places=6)  # the ground

    def test_sphere_push_out_of_top(self):
        c, n = self.w.sphere_push(V3(0, 0.9, 0), 0.2)
        self.assertIsNotNone(n)
        self.assertGreater(n.y, 0.9)
        h = self.w.raycast(V3(c.x, 5, c.z), V3(0, -1, 0), 10)
        self.assertGreaterEqual(c.y - h.point.y, 0.2 * 0.97)


if __name__ == "__main__":
    unittest.main()
