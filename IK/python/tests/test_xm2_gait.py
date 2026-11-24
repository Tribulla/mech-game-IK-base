"""Check mech locomotion and gait behavior through the input and pose records used by hosts."""
import array
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from gaitcore import api
from gaitcore.mech import A_LAT, INPUT_FIELDS
from gaitcore.vmath import V3, WORLD_UP


def controls(**values):
    return array.array("f", (values.get(name, 0.0) for name in INPUT_FIELDS)).tobytes()


def drive(record, seconds, hz=60):
    for _ in range(round(seconds * hz)):
        api.step(1.0 / hz, record)


class MechGait(unittest.TestCase):
    def setUp(self):
        api.reset()
        self.addCleanup(api.reset)
        self.handle = api.spawn("STRIKER", 0, 0, 0, 0)
        self.mech = api._mechs[self.handle]

    def reset_pose(self):
        api.respawn(self.handle, "STRIKER", 0, 0, 0, 0)
        self.mech = api._mechs[self.handle]

    def contacts(self):
        pose = memoryview(api.poses()).cast("f")
        stride = self.mech.joints_per_leg * 3 + api.LEG_EXTRA_FLOATS
        for start in range(api.HEADER_FLOATS, len(pose), stride):
            end = start + stride - api.LEG_EXTRA_FLOATS
            yield bool(pose[end + 3]), V3(*pose[end - 3:end])

    def test_three_game_speeds_and_foot_contacts(self):
        for mode, studs_per_second in (("slow", 50), ("walk", 130), ("run", 170)):
            with self.subTest(mode=mode):
                self.reset_pose()
                record = controls(fwd=1, slow=mode == "slow", run=mode == "run")
                expected = studs_per_second * 0.28
                drive(record, 4)
                start_z = self.mech.pos.z
                previous = list(self.contacts())
                bends = [leg.ik.bends_deg for leg in self.mech.legs]
                landings = [0] * self.mech.n
                for _ in range(180):
                    api.step(1.0 / 60.0, record)
                    current = list(self.contacts())
                    self.assertAlmostEqual(self.mech.speed, expected, places=3)
                    for i, (planted, tip) in enumerate(current):
                        leg = self.mech.legs[i]
                        self.assertFalse(leg.searching)
                        for new, old in zip(leg.ik.bends_deg, bends[i]):
                            self.assertLess(abs(new - old), 35.0)
                        if planted:
                            self.assertLess(tip.dist(leg.foot), 0.001)
                            was_planted, old_tip = previous[i]
                            if was_planted:
                                self.assertLess(tip.dist(old_tip), 0.001)
                            else:
                                self.assertLess(tip.dist(old_tip) * 60.0, 3.0)
                                landings[i] += 1
                    previous = current
                    bends = [leg.ik.bends_deg for leg in self.mech.legs]
                self.assertGreaterEqual(min(landings), 3)
                self.assertAlmostEqual(self.mech.pos.z - start_z, 3 * expected, places=3)

    def test_gallop_distance_and_support_at_game_frame_rates(self):
        record = controls(fwd=1, run=1)
        for hz in (30, 60, 120):
            with self.subTest(hz=hz):
                self.reset_pose()
                drive(record, 4, hz)
                z = self.mech.pos.z
                flight_frames = 0
                pairs = set()
                for _ in range(2 * hz):
                    api.step(1.0 / hz, record)
                    support = tuple(i for i, (planted, _) in enumerate(self.contacts()) if planted)
                    if support:
                        pairs.add(support)
                    else:
                        flight_frames += 1
                self.assertIn((0, 1), pairs)
                self.assertIn((2, 3), pairs)
                self.assertGreater(flight_frames, 0)
                self.assertAlmostEqual(self.mech.pos.z - z, 2 * 47.6, places=3)

    def test_sprint_turn_strafe_reverse_and_stop(self):
        for direction in (-1, 1):
            with self.subTest(turn=direction):
                self.reset_pose()
                drive(controls(fwd=1, run=1), 4)
                drive(controls(fwd=1, turn=direction, run=1), 1)
                self.assertGreater(-self.mech.vel.x * direction, 8.0)
                self.assertGreater(abs(self.mech.yaw_rate), 1.2)
                self.assertLessEqual(abs(self.mech.yaw_rate) * self.mech.speed, A_LAT + 0.05)
                self.assertAlmostEqual(self.mech.speed, 47.6, delta=0.2)
        self.reset_pose()
        drive(controls(fwd=1, run=1), 4)
        drive(controls(strafe=1, run=1), 1)
        self.assertLess(self.mech.vel.x, -10.0)
        drive(controls(fwd=-1, run=1), 2.5)
        self.assertLess(self.mech.vel.z, -10.0)
        drive(controls(), 4)
        planted = list(self.contacts())
        self.assertTrue(all(down for down, _ in planted))
        api.events()
        drive(controls(), 1)
        self.assertLess(self.mech.speed, 0.001)
        self.assertFalse(api.events())
        for (_, old_tip), (down, tip) in zip(planted, self.contacts()):
            self.assertTrue(down)
            self.assertLess(tip.dist(old_tip), 0.001)

    def test_swing_retarget_preserves_motion_and_landing_time(self):
        leg = self.mech.legs[0]
        start = leg.foot.copy()
        ground = start + V3(0, 0, 8)
        leg.begin_swing(ground, WORLD_UP, 0.4, 0.5, WORLD_UP, True, self.mech.world, 0, self.mech)
        leg.update_swing(0.14, self.mech)
        sw = leg.swing
        phase, duration = sw.t, sw.dur
        position = leg.foot.copy()
        epsilon = 1e-5
        before = (position - sw.sample(phase - epsilon)[0]) / (epsilon * duration)
        new_ground = ground + V3(0.4, 0.1, -0.7)
        leg.retarget(new_ground, WORLD_UP, True, self.mech.world, 0, self.mech)
        self.assertEqual((sw.t, sw.dur), (phase, duration))
        self.assertLess(sw.sample(phase)[0].dist(position), 1e-7)
        after = (sw.sample(phase + epsilon)[0] - position) / (epsilon * duration)
        self.assertLess(after.dist(before), 0.01)
        near_end, final_speed = sw.sample(1.0 - epsilon)
        self.assertLess(near_end.dist(new_ground), 1e-6)
        self.assertLess(final_speed.length(), 0.001)
        self.assertTrue(leg.update_swing((1.0 - phase) * duration + 1e-8, self.mech))
        self.assertLess(leg.foot.dist(new_ground), 1e-7)

    def test_reach_uses_the_hip_at_touchdown(self):
        drive(controls(fwd=1, run=1), 4)
        leg = self.mech.legs[0]
        old_spot = leg.ideal_now
        pose = self.mech._landing_pose(leg, 0.3)
        self.assertTrue(self.mech._reachable(leg, old_spot))
        self.assertFalse(self.mech._reachable(leg, old_spot, pose))
        new_spot = old_spot + self.mech.planar_travel(0.3)
        self.assertGreater(new_spot.dist(leg.mount_world), leg.reach)
        self.assertTrue(self.mech._reachable(leg, new_spot, pose))

    def test_striker_smoothness_and_separation(self):
        self.assertEqual(self.mech.d["speeds"], {"slow": 14.0, "walk": 36.4, "run": 47.6})
        cases = (
            ("run_47", lambda t: controls(fwd=1, run=1), 3000.0, 0.50),
            ("run_turn", lambda t: controls(fwd=1, run=1, turn=0.7), 3000.0, None),
            ("walk_drift", lambda t: controls(fwd=1, drift=(1 if t > 1.0 else 0), turn=(1.0 if t > 1.0 else 0.0)), 3000.0, None),
            ("stand_spin", lambda t: controls(turn=1.0), 1200.0, None),
            ("strafe_flip", lambda t: controls(strafe=(1.0 if int(t / 1.2) % 2 == 0 else -1.0), run=1), 3000.0, None),
            ("walk_36", lambda t: controls(fwd=1), 3000.0, 0.50),
        )
        for name, inp_fn, max_coxa_dps, min_sep_req in cases:
            with self.subTest(case=name):
                self.reset_pose()
                prev_yaw = [l.ik.yaw_deg for l in self.mech.legs]
                max_coxa = 0.0
                min_sep = 99.0
                for i in range(300):
                    t = i / 60.0
                    api.step(1.0 / 60.0, inp_fn(t))
                    if i >= 60:
                        for li, l in enumerate(self.mech.legs):
                            self.assertFalse(l.searching)
                            max_coxa = max(max_coxa, abs(l.ik.yaw_deg - prev_yaw[li]) * 60.0)
                        for a in range(self.mech.n):
                            for b in range(a + 1, self.mech.n):
                                if self.mech.legs[a].mount_local.x * self.mech.legs[b].mount_local.x > 0:
                                    min_sep = min(min_sep, self.mech.legs[a].foot.dist(self.mech.legs[b].foot))
                    prev_yaw = [l.ik.yaw_deg for l in self.mech.legs]
                self.assertLess(max_coxa, max_coxa_dps)
                if min_sep_req is not None:
                    self.assertGreaterEqual(min_sep, min_sep_req)

    def test_cam_turn_agility_and_ramp_recovery(self):
        import math
        from gaitcore import world

        def cam_flick(deg, frames, run=1):
            aim_yaw = -math.radians(deg)
            aim_f = (math.sin(aim_yaw), math.cos(aim_yaw))
            max_slip = 0.0
            min_fwd_dot = 1e9
            for _ in range(frames):
                mf = (self.mech.heading_fwd().x, self.mech.heading_fwd().z)
                ml = math.hypot(*mf) or 1.0
                mf = (mf[0] / ml, mf[1] / ml)
                mr = (-mf[1], mf[0])
                fwd = aim_f[0] * mf[0] + aim_f[1] * mf[1]
                strafe = aim_f[0] * mr[0] + aim_f[1] * mr[1]
                heading = math.atan2(mf[0], mf[1])
                diff = (aim_yaw - heading + math.pi) % (2.0 * math.pi) - math.pi
                turn = max(-1.0, min(1.0, -3.0 * diff))
                u = max(0.0, min(1.0, (abs(turn) - 0.25) / 0.50))
                turn_mag = u * u * (3.0 - 2.0 * u)
                if turn_mag > 0.0:
                    turn_aligned = strafe * turn > 0.0
                    sharp_about = fwd < -0.1 and (self.mech.vt.x * mf[0] + self.mech.vt.z * mf[1]) > 4.0 and abs(turn) > 0.75
                    if turn_aligned or sharp_about:
                        in_len = min(1.0, math.hypot(fwd, strafe))
                        align_fwd = max(0.0, min(1.0, 0.55 + 0.90 * fwd))
                        want_fwd = in_len * align_fwd
                        fwd = fwd + (want_fwd - fwd) * turn_mag
                        strafe = strafe * (1.0 - turn_mag * (1.0 - 0.12 * align_fwd))
                api.step(1.0 / 60.0, controls(fwd=fwd, strafe=strafe, turn=turn, run=run))
                if self.mech.speed > 1.0:
                    vdir = math.atan2(self.mech.vt.x, self.mech.vt.z)
                    hdg = math.atan2(self.mech.frame.fwd.x, self.mech.frame.fwd.z)
                    slip = math.degrees(abs((vdir - hdg + math.pi) % (2.0 * math.pi) - math.pi))
                    max_slip = max(max_slip, slip)
                min_fwd_dot = min(min_fwd_dot, self.mech.vt.dot(self.mech.heading_fwd()))
            mf = (self.mech.frame.fwd.x, self.mech.frame.fwd.z)
            heading_err = abs((aim_yaw - math.atan2(mf[0], mf[1]) + math.pi) % (2.0 * math.pi) - math.pi)
            return heading_err, max_slip, min_fwd_dot

        # 1a) CamAim 90 deg flick at full run speed aligns fast without sideways crab-walk or strafe cap
        self.reset_pose()
        drive(controls(fwd=1, run=1), 2.0)
        err, max_slip, _ = cam_flick(90.0, 39, run=1)
        self.assertLess(err, math.radians(15.0))
        self.assertLess(max_slip, 20.0)
        self.assertNotIn("strafing", self.mech.gait_label())
        cam_flick(90.0, 21, run=1)
        self.assertGreater(self.mech.speed, 34.0)

        # 1b) CamAim 90 deg flick from standstill pivots hull rather than walking sideways
        self.reset_pose()
        err, max_slip, _ = cam_flick(90.0, 33, run=0)
        self.assertLess(err, math.radians(15.0))
        self.assertLess(max_slip, 20.0)

        # 1c) CamAim 150 deg flick at full run speed aligns quickly without reversing
        self.reset_pose()
        drive(controls(fwd=1, run=1), 2.0)
        err, _, min_fwd_dot = cam_flick(150.0, 57, run=1)
        self.assertLess(err, math.radians(15.0))
        self.assertGreater(min_fwd_dot, -2.0)

        # 1d) Intentional circle-strafe (strafe opposing turn) retains strong lateral velocity,
        # and direct control (strafe=1, turn=1) remains faithful without overwriting strafe
        self.reset_pose()
        drive(controls(fwd=1, run=1), 2.0)
        drive(controls(strafe=1, turn=-0.8, run=1), 0.6)
        self.assertGreater(abs(self.mech.vt.dot(self.mech.support_frame().right)), 10.0)
        self.reset_pose()
        drive(controls(strafe=1, turn=1, run=1), 1.5)
        self.assertGreater(abs(self.mech.vt.dot(self.mech.support_frame().right)), 10.0)

        # 2) Smooth ramp creases must not trigger crawl (rough ground), and flat run recovers quickly
        w = world.StubWorld()
        for k in range(3):
            z = 30.0 + k * 28.0
            rad = math.radians(16.0)
            rise = 8.0 * math.tan(rad)
            w.add_obox((0, rise * 0.5 - 0.15, z + 4.0), (6.0, 0.15, math.hypot(4.0, rise * 0.5)), 0.0, -rad)
            w.add_box((-6.0, 0.0, z + 8.0), (6.0, rise, z + 12.0))
            w.add_obox((0, rise * 0.5 - 0.15, z + 16.0), (6.0, 0.15, math.hypot(4.0, rise * 0.5)), 0.0, rad)
        world.set_world(w)
        self.addCleanup(lambda: world.set_world(world.StubWorld()))
        self.reset_pose()
        min_rung = 9
        for i in range(420):
            api.step(1.0 / 60.0, controls(fwd=1, run=1))
            if i >= 120:
                min_rung = min(min_rung, self.mech.selector.rung)
        self.assertGreaterEqual(min_rung, 2)
        self.assertEqual(self.mech.gait_name, "bound")
        self.assertAlmostEqual(self.mech.speed, 47.6, delta=0.5)

    def test_ground_relative_hull_pitch(self):
        import math
        from gaitcore import world
        self.addCleanup(lambda: world.set_world(world.StubWorld()))

        def check_pitch(slope_deg, aim_deg, expected_deg, delta):
            api.reset()
            w = world.StubWorld()
            y0 = 0.0
            if slope_deg > 0:
                rad = math.radians(slope_deg)
                w.add_obox((0, 6.0, 0), (30.0, 0.5, 30.0), 0.0, -rad)
                y0 = 6.0 + 0.5 / math.cos(rad)
            world.set_world(w)
            self.handle = api.spawn("STRIKER", 0, y0, 0, 0)
            self.mech = api._mechs[self.handle]
            drive(controls(aim=math.radians(aim_deg)), 3.0)
            hull_pitch = math.degrees(math.asin(max(-1.0, min(1.0, self.mech.frame.fwd.y))))
            self.assertAlmostEqual(hull_pitch, expected_deg, delta=delta)
            self.assertFalse(self.mech.on_wall)
            self.assertTrue(all(l.planted and l.ik.err < 0.01 for l in self.mech.legs))

        check_pitch(0, 0, 0.0, 1.0)
        check_pitch(0, 60, 25.0, 1.5)
        check_pitch(0, -60, -25.0, 1.5)
        check_pitch(20, 0, 20.0, 2.0)
        check_pitch(20, 60, 45.0, 2.0)
        check_pitch(20, -60, -5.0, 2.0)
        check_pitch(30, 60, 55.0, 2.0)

    def test_video_physics_and_terrain_improvements(self):
        import math
        from gaitcore import mech as mech_mod, world
        from gaitcore.leg import Swing
        self.addCleanup(lambda: world.set_world(world.StubWorld()))

        # 1) External push injects rotational recoil torque around WORLD_UP x dv and damped spring recovers
        self.reset_pose()
        self.mech.push(V3(6.0, 0.0, 0.0))
        self.assertLess(self.mech.ang_vel.z, -0.5)
        drive(controls(), 2.5)
        self.assertGreater(self.mech.frame.up.y, 0.99)
        self.assertLess(self.mech.ang_vel.length(), 0.1)

        # 2) Running jump landing injects pitch-dip angular velocity and recovers cleanly
        self.reset_pose()
        drive(controls(fwd=1, run=1), 1.5)
        for _ in range(40):
            if any(l.planted for l in self.mech.legs):
                break
            api.step(1.0 / 60.0, controls(fwd=1, run=1))
        api.step(1.0 / 60.0, controls(fwd=1, run=1, jump=1))
        saw_air = False
        landed_ang = None
        for _ in range(300):
            api.step(1.0 / 60.0, controls(fwd=1, run=1))
            if self.mech.state == mech_mod.AIR:
                saw_air = True
            elif saw_air and self.mech.state == mech_mod.GROUND:
                landed_ang = self.mech.ang_vel.copy()
                break
        self.assertTrue(saw_air)
        self.assertIsNotNone(landed_ang)
        self.assertGreater(landed_ang.x, 0.15)
        drive(controls(), 2.5)
        self.assertTrue(all(l.planted and l.ik.err < 0.01 for l in self.mech.legs))

        # 3) ground_n holds 20 deg slope smoothly even if plane_n_local is momentarily None
        api.reset()
        w = world.StubWorld()
        rad = math.radians(20.0)
        w.add_obox((0, 6.0, 0), (30.0, 0.5, 30.0), 0.0, -rad)
        world.set_world(w)
        self.handle = api.spawn("STRIKER", 0, 6.0 + 0.5 / math.cos(rad), 0, 0)
        self.mech = api._mechs[self.handle]
        drive(controls(), 2.0)
        slope_before = math.degrees(math.acos(max(-1.0, min(1.0, self.mech.ground_n.y))))
        self.assertAlmostEqual(slope_before, 20.0, delta=2.0)
        self.mech.plane_n_local = None
        self.mech._update_orientation(1.0 / 60.0)
        slope_after = math.degrees(math.acos(max(-1.0, min(1.0, self.mech.ground_n.y))))
        self.assertGreater(slope_after, 18.0)

        # 4) keep_clear pushes swinging foot laterally out of a vertical obstacle face
        api.reset()
        w2 = world.StubWorld()
        w2.add_box((0.5, 0.0, -2.0), (2.0, 2.0, 2.0))
        world.set_world(w2)
        self.handle = api.spawn("STRIKER", 0, 0, 0, 0)
        self.mech = api._mechs[self.handle]
        leg = self.mech.legs[0]
        sw = Swing()
        sw.t = 0.5
        leg.swing = sw
        leg.planted = False
        leg.foot = V3(0.55, 0.8, 0.0)
        leg.keep_clear(w2, WORLD_UP)
        self.assertLess(leg.foot.x, 0.5)

    def test_crawl_and_wave_footfall_sequence(self):
        # Verify quadruped crawl (LH->LF->RH->RF) and hexapod/octopod back-to-front wave sequences
        def collect_steady(name, throttle, warmup_s, measure_s):
            self.reset_pose()
            api.respawn(self.handle, name, 0, 0, 0, 0)
            self.mech = api._mechs[self.handle]
            rec = controls(fwd=throttle, slow=1)
            drive(rec, warmup_s)
            lifts = []
            min_pl = self.mech.n
            was = [l.planted for l in self.mech.legs]
            for _ in range(round(measure_s * 60)):
                api.step(1.0 / 60.0, rec)
                min_pl = min(min_pl, sum(1 for l in self.mech.legs if l.planted))
                for l in self.mech.legs:
                    if was[l.idx] and not l.planted:
                        lifts.append(l.idx)
                    was[l.idx] = l.planted
            return self.mech.gait_name, min_pl, lifts

        # Quadruped crawl: strictly >= 3/4 legs planted, cycle (2 -> 0 -> 3 -> 1)
        gait, min_pl, lifts = collect_steady("STRIKER", 0.15, 2.5, 4.0)
        self.assertEqual(gait, "crawl")
        self.assertGreaterEqual(min_pl, 3)
        pairs = set(zip(lifts, lifts[1:]))
        self.assertGreater(len(lifts), 6)
        self.assertTrue(pairs.issubset({(2, 0), (0, 3), (3, 1), (1, 2)}), f"unexpected crawl pairs: {pairs} from {lifts}")

        # Hexapod wave: >= 5/6 legs planted, back-to-front (4 -> 2 -> 0 -> 5 -> 3 -> 1)
        gait, min_pl, lifts = collect_steady("MANTIS", 0.12, 5.0, 3.0)
        self.assertEqual(gait, "wave")
        self.assertGreaterEqual(min_pl, 5)
        pairs = set(zip(lifts, lifts[1:]))
        self.assertGreater(len(lifts), 8)
        self.assertTrue(pairs.issubset({(4, 2), (2, 0), (0, 5), (5, 3), (3, 1), (1, 4)}), f"unexpected hex wave pairs: {pairs} from {lifts}")

        # Octopod wave: >= 6/8 legs planted, back-to-front (6 -> 4 -> 2 -> 0 -> 7 -> 5 -> 3 -> 1)
        gait, min_pl, lifts = collect_steady("WARDEN", 0.12, 3.5, 3.5)
        self.assertEqual(gait, "wave")
        self.assertGreaterEqual(min_pl, 6)
        pairs = set(zip(lifts, lifts[1:]))
        self.assertGreater(len(lifts), 10)
        self.assertTrue(pairs.issubset({(6, 4), (4, 2), (2, 0), (0, 7), (7, 5), (5, 3), (3, 1), (1, 6)}), f"unexpected oct wave pairs: {pairs} from {lifts}")

    def test_no_false_slowdowns_on_ramps_and_bridges(self):
        import math
        from gaitcore import world
        self.addCleanup(lambda: (api.reset(), world.set_world(world.StubWorld())))

        # 1) 25 deg smooth ramp (run=1): stays at cap >= 2 and > 15 m/s after accel
        api.reset()
        w = world.StubWorld()
        rad = math.radians(25.0)
        rise = 15.0 * math.tan(rad)
        w.add_obox((0, rise * 0.5 - 0.15, 50.0 + 7.5), (10.0, 0.15, math.hypot(7.5, rise * 0.5)), 0.0, -rad)
        w.add_box((-10.0, 0.0, 65.0), (10.0, rise, 140.0))
        world.set_world(w)
        h = api.spawn("STRIKER", 0, 0, 0, 0)
        m = api._mechs[h]
        min_cap = 3
        min_spd = 999.0
        for i in range(480):
            api.step(1.0 / 60.0, controls(fwd=1, run=1))
            min_cap = min(min_cap, m.selector.cap)
            if i >= 120:
                min_spd = min(min_spd, m.speed)
        self.assertGreaterEqual(min_cap, 2)
        self.assertGreater(min_spd, 15.0)

        # 2) LabSlopes (10, 18, 26, 34 deg ramps, walk): never drops to cap 1 (5 m/s)
        api.reset()
        w2 = world.StubWorld()
        for k in range(4):
            z = 20.0 + k * 28.0
            deg = 10.0 + 8.0 * k
            r = math.radians(deg)
            ri = 8.0 * math.tan(r)
            w2.add_obox((0, ri * 0.5 - 0.15, z + 4.0), (6.0, 0.15, math.hypot(4.0, ri * 0.5)), 0.0, -r)
            w2.add_box((-6.0, 0.0, z + 8.0), (6.0, ri, z + 12.0))
            w2.add_obox((0, ri * 0.5 - 0.15, z + 16.0), (6.0, 0.15, math.hypot(4.0, ri * 0.5)), 0.0, r)
        world.set_world(w2)
        h = api.spawn("STRIKER", 0, 0, 0, 0)
        m = api._mechs[h]
        min_cap = 3
        for _ in range(600):
            api.step(1.0 / 60.0, controls(fwd=1))
            min_cap = min(min_cap, m.selector.cap)
        self.assertGreaterEqual(min_cap, 2)

        # 3) Flat 6 m bridge over -7 m pit (run=1): stays at cap >= 2 and > 20 m/s after accel
        api.reset()
        w3 = world.StubWorld(ground_y=-7.0)
        w3.add_box((-30.0, -7.5, -30.0), (30.0, 0.0, 30.0))
        w3.add_box((-3.0, -7.5, 30.0), (3.0, 0.0, 48.0))
        w3.add_box((-30.0, -7.5, 48.0), (30.0, 0.0, 150.0))
        world.set_world(w3)
        h = api.spawn("STRIKER", 0, 0, 0, 0)
        m = api._mechs[h]
        min_cap = 3
        min_spd = 999.0
        for i in range(300):
            api.step(1.0 / 60.0, controls(fwd=1, run=1))
            min_cap = min(min_cap, m.selector.cap)
            if i >= 90:
                min_spd = min(min_spd, m.speed)
        self.assertGreaterEqual(min_cap, 2)
        self.assertGreater(min_spd, 20.0)

    def test_no_walking_or_running_on_thin_walls(self):
        import math
        from gaitcore import world
        self.addCleanup(lambda: (api.reset(), world.set_world(world.StubWorld())))
        dt = 1.0 / 60.0

        def run_wall(kind, width, rec, yaw_deg=0.0, wall_h=8.0, secs=5.0):
            api.reset()
            w = world.StubWorld(has_ground=True, ground_y=0.0)
            rad = math.radians(yaw_deg)
            w.add_obox((0.0, wall_h * 0.5, 0.0), (8.0, wall_h * 0.5, 10.0), rad, 0.0)
            cx, cz = 60.0 * math.sin(rad), 60.0 * math.cos(rad)
            w.add_obox((cx, wall_h * 0.5, cz), (width * 0.5, wall_h * 0.5, 50.0), rad, 0.0)
            world.set_world(w)
            idx = api.spawn(kind, 0.0, wall_h, 0.0, rad)
            m = api._mechs[idx]
            max_along_top = 0.0
            for _ in range(int(secs * 60)):
                api.step(dt, rec)
                along = m.pos.x * math.sin(rad) + m.pos.z * math.cos(rad)
                if m.pos.y > wall_h + 0.35:
                    max_along_top = max(max_along_top, along)
            return max_along_top, m.pos.y

        # 1) Thin walls (0.40, 0.70, 0.90 m) at walk, cruise, and run speeds (+ 45 deg yaw)
        slow_rec = controls(fwd=1, slow=1)
        cruise_rec = controls(fwd=1)
        run_rec = controls(fwd=1, run=1)
        for kind in ("STRIKER", "WARDEN", "COLOSSUS"):
            for width in (0.40, 0.70, 0.90):
                with self.subTest(kind=kind, width=width, mode="walk"):
                    mt, ey = run_wall(kind, width, slow_rec)
                    self.assertLess(mt, 20.0)
                    self.assertTrue(ey < 6.5 or mt < 15.0)
                for mode, rec in (("cruise", cruise_rec), ("run", run_rec)):
                    with self.subTest(kind=kind, width=width, mode=mode):
                        mt, ey = run_wall(kind, width, rec)
                        self.assertLess(mt, 35.0)
                        self.assertTrue(ey < 6.5 or mt < 15.0)

        for mode, rec in (("walk", slow_rec), ("run", run_rec)):
            with self.subTest(kind="STRIKER", yaw=45, width=0.90, mode=mode):
                mt, ey = run_wall("STRIKER", 0.90, rec, yaw_deg=45.0)
                limit = 20.0 if mode == "walk" else 35.0
                self.assertLess(mt, limit)
                self.assertTrue(ey < 6.5 or mt < 15.0)

        # 2) Gap/wall straddle (wide left ledge, trench, right 0.70 m wall)
        for mode, rec in (("walk", slow_rec), ("cruise", cruise_rec), ("run", run_rec)):
            with self.subTest(straddle=mode):
                api.reset()
                w = world.StubWorld(has_ground=True, ground_y=-8.0)
                w.add_box((-10.0, -8.0, -10.0), (10.0, 0.0, 10.0))
                w.add_box((-10.0, -8.0, 10.0), (-0.2, 0.0, 100.0))
                w.add_box((1.6, -8.0, 10.0), (2.3, 0.0, 100.0))
                world.set_world(w)
                idx = api.spawn("STRIKER", 0.7, 0.0, 0.0, 0.0)
                m = api._mechs[idx]
                min_y = 999.0
                for _ in range(int(5.0 * 60)):
                    api.step(dt, rec)
                    min_y = min(min_y, m.pos.y)
                self.assertLess(min_y, -2.0)

        # 3) Course A pit: 0.90 m beam (-4.45..-3.55) and gap (-1.5) fail; 6.0 m bridge (+3.5) crosses
        def build_pit():
            w = world.StubWorld(has_ground=False)
            w.add_box((-20.0, -8.0, 50.0), (20.0, 0.0, 84.0))
            w.add_box((-20.0, -8.0, 102.0), (20.0, 0.0, 140.0))
            w.add_box((-20.0, -9.0, 84.0), (20.0, -8.0, 102.0))
            w.add_box((-4.45, -8.0, 84.0), (-3.55, 0.0, 102.0))
            w.add_box((0.5, -8.0, 84.0), (6.5, 0.0, 102.0))
            return w

        for x0, expect_cross in ((-4.0, False), (-1.5, False), (3.5, True)):
            with self.subTest(pit_x0=x0):
                api.reset()
                world.set_world(build_pit())
                idx = api.spawn("STRIKER", x0, 0.0, 74.0, 0.0)
                m = api._mechs[idx]
                min_y = 999.0
                for _ in range(int(7.0 * 60)):
                    api.step(dt, slow_rec)
                    min_y = min(min_y, m.pos.y)
                    if expect_cross and m.pos.z > 106.0:
                        break
                if expect_cross:
                    self.assertGreater(m.pos.z, 105.0)
                    self.assertGreater(min_y, 0.8)
                else:
                    self.assertTrue(m.pos.z < 86.0 or min_y < -1.5)

        # 4) Course A 1.50 m x 1.50 m stepping stones remain cleanly traversable
        for kind in ("STRIKER", "WARDEN"):
            with self.subTest(stepping_stones=kind):
                api.reset()
                w = world.StubWorld(has_ground=True, ground_y=0.0)
                w.add_box((-5.0, 0.0, 36.0), (5.0, 3.0, 44.0))
                for i in range(5):
                    hz = 45.2 + i * 2.0
                    w.add_box((-2.15, 0.0, hz - 0.75), (-0.65, 3.0, hz + 0.75))
                    hz += 1.0
                    w.add_box((0.65, 0.0, hz - 0.75), (2.15, 3.0, hz + 0.75))
                w.add_box((-5.0, 0.0, 55.8), (5.0, 3.0, 62.0))
                world.set_world(w)
                idx = api.spawn(kind, 0.0, 3.0, 40.0, 0.0)
                m = api._mechs[idx]
                min_y = 999.0
                max_z = 0.0
                for _ in range(int(6.0 * 60)):
                    api.step(dt, cruise_rec)
                    min_y = min(min_y, m.pos.y)
                    max_z = max(max_z, m.pos.z)
                    if m.pos.z > 59.0:
                        break
                self.assertGreater(max_z, 58.0)
                self.assertGreater(min_y, 2.5)

    def test_no_wall_pin_stuck_after_pit_fall(self):
        from gaitcore import world
        self.addCleanup(lambda: (api.reset(), world.set_world(world.StubWorld())))
        dt = 1.0 / 60.0

        def make_trench():
            w = world.StubWorld(has_ground=True, ground_y=-7.0)
            w.add_box((0.5, -8.0, 50.0), (6.5, 0.0, 120.0))
            w.add_box((-4.45, -8.0, 50.0), (-3.55, 0.0, 120.0))
            return w

        def spawn_settled(x0):
            api.reset()
            world.set_world(make_trench())
            h = api.spawn("STRIKER", x0, -3.0, 88.0, 0.0)
            m = api._mechs[h]
            m._enter_air()
            for _ in range(int(1.5 * 60)):
                api.step(dt, controls())
            return m

        for x0 in (-0.12, -0.20, -0.40):
            with self.subTest(drop_x0=x0):
                m = spawn_settled(x0)
                self.assertEqual(m.state, 0)
                self.assertGreater(m.pos.y, -7.0 + 0.75 * m.ride)
                self.assertLessEqual(m.pos.x, 0.5 - m.d["body_half"][0] + 0.05)

                # Strafe away from wall (A)
                q0 = m.pos.copy()
                for _ in range(int(1.5 * 60)):
                    api.step(dt, controls(strafe=1))
                self.assertLess(m.pos.x - q0.x, -1.5)

                # Walk forward along the wall (W)
                m = spawn_settled(x0)
                q0 = m.pos.copy()
                for _ in range(int(2.0 * 60)):
                    api.step(dt, controls(fwd=1))
                self.assertGreater(m.pos.z - q0.z, 15.0)

                # Jump escape launches cleanly airborne
                m = spawn_settled(x0)
                max_y = m.pos.y
                for _ in range(6):
                    api.step(dt, controls(jump=1))
                    max_y = max(max_y, m.pos.y)
                for _ in range(36):
                    api.step(dt, controls())
                    max_y = max(max_y, m.pos.y)
                self.assertGreater(max_y, -4.0)

        # Strafing into the wall then away stays upright and escapes cleanly
        m = spawn_settled(-0.40)
        q0 = m.pos.copy()
        min_y = m.pos.y
        for _ in range(int(1.5 * 60)):
            api.step(dt, controls(strafe=-1))
            min_y = min(min_y, m.pos.y)
        for _ in range(int(1.5 * 60)):
            api.step(dt, controls(strafe=1))
            min_y = min(min_y, m.pos.y)
        self.assertGreater(min_y, -7.0 + 0.65 * m.ride)
        self.assertLess(m.pos.x - q0.x, -1.5)

    def test_quick_slide_is_finite_and_responsive(self):
        import array, math
        from gaitcore import world, mech_const as mc
        self.addCleanup(lambda: (api.reset(), world.set_world(world.StubWorld())))
        dt = 1.0 / 60.0

        def drain_ev_kinds():
            arr = array.array("f")
            arr.frombytes(api.events())
            return [int(arr[i]) for i in range(0, len(arr), api.EVENT_FLOATS)]

        # 1) Quick tap slide: short tap enters SLIDE cleanly, crouches, skids/sparks, zero boost
        api.reset()
        world.set_world(world.StubWorld())
        h = api.spawn("STRIKER", 0, 0, 0, 0)
        m = api._mechs[h]
        for _ in range(int(1.5 * 60)):
            api.step(dt, controls(fwd=1, run=1))
        base_y = m.pos.y
        api.events()
        slide_frames = 0
        min_y = 999.0
        labels = set()
        ev_kinds = []
        for i in range(int(1.2 * 60)):
            api.step(dt, controls(fwd=1, run=1, drift=(1 if i < 3 else 0)))
            if m.drifting:
                slide_frames += 1
                labels.add(m.mode_label())
                min_y = min(min_y, m.pos.y)
            ev_kinds.extend(drain_ev_kinds())
        self.assertGreaterEqual(slide_frames, int(0.18 * 60))
        self.assertLessEqual(slide_frames, int(0.50 * 60))
        self.assertIn("SLIDE", labels)
        self.assertLess(min_y, base_y - 0.15)
        self.assertIn(mc.EV_SKID, ev_kinds)
        self.assertIn(mc.EV_SPARK, ev_kinds)
        self.assertNotIn(mc.EV_BOOST, ev_kinds)
        self.assertFalse(m.drifting)
        self.assertEqual(m.state, mc.GROUND)

        # 2) Held X + full turn: slide ends in finite time, slows down, does not re-enter while held, mild carve
        api.reset()
        world.set_world(world.StubWorld())
        h = api.spawn("STRIKER", 0, 0, 0, 0)
        m = api._mechs[h]
        for _ in range(int(2.0 * 60)):
            api.step(dt, controls(fwd=1, run=1))
        start_spd = end_spd = None
        v0 = v1 = None
        slide_frames = 0
        entries = 0
        was_sliding = False
        for _ in range(int(2.5 * 60)):
            api.step(dt, controls(fwd=1, run=1, drift=1, turn=1))
            if m.drifting:
                slide_frames += 1
                if not was_sliding:
                    entries += 1
                    if start_spd is None:
                        start_spd = m.vt.length()
                        v0 = m.vt.normalized()
                end_spd = m.vt.length()
                v1 = m.vt.normalized()
            was_sliding = m.drifting
        self.assertEqual(entries, 1)
        self.assertGreaterEqual(slide_frames / 60.0, 0.35)
        self.assertLessEqual(slide_frames / 60.0, 1.10)
        self.assertLess(end_spd, start_spd - 2.5)
        self.assertGreater(end_spd, start_spd * 0.78)
        carve_deg = math.degrees(math.acos(max(-1.0, min(1.0, v0.dot(v1)))))
        self.assertLess(carve_deg, 45.0)

        # 3) Re-press works quickly after brief release
        for _ in range(int(0.45 * 60)):
            api.step(dt, controls(fwd=1, run=1))
        reentered = False
        for _ in range(int(0.40 * 60)):
            api.step(dt, controls(fwd=1, run=1, drift=1))
            if m.drifting:
                reentered = True
                break
        self.assertTrue(reentered)

        # 4) Slope-aware sliding: 20 deg downhill slides longer and exits faster than 20 deg uphill
        def run_slope(down):
            api.reset()
            w = world.StubWorld(has_ground=False)
            rad = math.radians(20.0)
            w.add_obox((0.0, 20.0, 0.0), (20.0, 0.5, 100.0), 0.0, rad if down else -rad)
            world.set_world(w)
            hit = w.raycast(V3(0.0, 80.0, -30.0), V3(0.0, -1.0, 0.0), 120.0)
            hh = api.spawn("STRIKER", 0.0, hit.point.y, -30.0, 0.0)
            mm = api._mechs[hh]
            for _ in range(int(1.2 * 60)):
                api.step(dt, controls(fwd=1, run=1))
            sf = 0
            esp = 0.0
            for _ in range(int(2.0 * 60)):
                api.step(dt, controls(fwd=1, run=1, drift=1))
                if mm.drifting:
                    sf += 1
                    esp = mm.vt.length()
            return sf / 60.0, esp

        down_dur, down_spd = run_slope(True)
        up_dur, up_spd = run_slope(False)
        self.assertGreater(down_dur, up_dur + 0.35)
        self.assertGreater(down_spd, up_spd)

        # 5) Slide-jump cancel: jumping mid-slide clears drifting and launches airborne
        api.reset()
        world.set_world(world.StubWorld())
        h = api.spawn("STRIKER", 0, 0, 0, 0)
        m = api._mechs[h]
        for _ in range(int(1.5 * 60)):
            api.step(dt, controls(fwd=1, run=1))
        for f in range(30):
            api.step(dt, controls(fwd=1, run=1, drift=1))
            if m.drifting and f >= 12 and any(l.planted for l in m.legs):
                break
        self.assertTrue(m.drifting)
        launched = False
        for f in range(30):
            api.step(dt, controls(fwd=1, run=1, jump=(1 if f < 3 else 0)))
            if m.state == mc.AIR:
                launched = True
                break
        self.assertTrue(launched)
        self.assertFalse(m.drifting)

    def test_api_input_and_dt_safety(self):
        import math, time
        from gaitcore import world
        self.addCleanup(lambda: (api.reset(), world.set_world(world.StubWorld())))
        w = world.StubWorld()
        w.add_box((-2.0, 0.0, 10.0), (2.0, 1.0, 12.0))
        world.set_world(w)
        h = api.spawn("STRIKER", 0, 0, 0, 0)
        m = api._mechs[h]
        # Non-finite dt and huge dt spikes neither crash nor stall
        api.step(float("nan"), controls(fwd=1))
        api.step(float("inf"), controls(fwd=1))
        t0 = time.perf_counter()
        api.step(2.0, controls(fwd=1))
        self.assertLess((time.perf_counter() - t0) * 1000.0, 100.0)
        # Non-finite inputs do not poison mech state
        for _ in range(10):
            api.step(1.0 / 60.0, controls(fwd=float("nan"), strafe=float("inf"), turn=float("-inf"), aim=float("nan")))
        self.assertTrue(all(math.isfinite(x) for x in (m.pos.x, m.pos.y, m.pos.z, m.speed, m.frame.fwd.y)))
        # hot_reload preserves custom StubWorld geometry
        self.assertTrue(api.hot_reload())
        self.assertEqual(len(world.get_world().boxes), 1)

    def test_low_tunnel_no_roof_teleport(self):
        from gaitcore import world
        self.addCleanup(lambda: (api.reset(), world.set_world(world.StubWorld())))
        api.reset()
        w = world.StubWorld()
        w.add_box((-8.0, 2.2, 6.0), (8.0, 2.7, 40.0))
        world.set_world(w)
        h = api.spawn("STRIKER", 0, 0, 0, 0)
        m = api._mechs[h]
        # Toggle crouch on
        api.step(1.0 / 60.0, controls(crouch=1))
        max_foot_y = 0.0
        for _ in range(360):
            api.step(1.0 / 60.0, controls(fwd=1))
            for l in m.legs:
                if m.pos.z > 6.5:
                    max_foot_y = max(max_foot_y, l.foot.y)
        self.assertGreater(m.pos.z, 25.0)
        self.assertLess(max_foot_y, 1.9)

    def test_slope_moving_aim_stability_and_steep_ramps(self):
        import math
        from gaitcore import mech as mech_mod, world
        self.addCleanup(lambda: (api.reset(), world.set_world(world.StubWorld())))
        # Moving on a 30 deg slope with +-60 deg aim does not pogo or eject to AIR
        rad = math.radians(30.0)
        for aim_deg in (60.0, -60.0):
            api.reset()
            w = world.StubWorld()
            w.add_obox((0, 15.0, 0), (40.0, 0.5, 80.0), 0.0, -rad)
            world.set_world(w)
            y0 = 15.0 + 0.5 / math.cos(rad) - 20.0 * math.tan(rad)
            h = api.spawn("STRIKER", 0, y0, -20.0, 0.0)
            m = api._mechs[h]
            pitches = []
            for i in range(180):
                api.step(1.0 / 60.0, controls(fwd=0.3, aim=math.radians(aim_deg)))
                self.assertEqual(m.state, mech_mod.GROUND)
                if i >= 60:
                    pitches.append(math.degrees(math.asin(max(-1.0, min(1.0, m.frame.fwd.y)))))
            self.assertLess(max(pitches) - min(pitches), 8.0)

        # Smooth 35 deg ramp climbs to the top platform cleanly at run
        api.reset()
        w2 = world.StubWorld()
        r35 = math.radians(35.0)
        rise = 16.0 * math.tan(r35)
        w2.add_obox((0, rise * 0.5 - 0.25, 28.0), (10.0, 0.25, math.hypot(8.0, rise * 0.5)), 0.0, -r35)
        w2.add_box((-10.0, 0.0, 36.0), (10.0, rise, 52.0))
        world.set_world(w2)
        h = api.spawn("STRIKER", 0, 0, 0, 0)
        m = api._mechs[h]
        max_y = 0.0
        for _ in range(360):
            api.step(1.0 / 60.0, controls(fwd=1, run=1))
            max_y = max(max_y, m.pos.y)
        self.assertGreater(max_y, rise + 0.5)

    def test_bastion_sagittal_feasibility_and_frame_rate_invariance(self):
        from gaitcore import ik as ik_mod, mechdefs, world
        from gaitcore.vmath import LOCAL_FRAME
        self.addCleanup(lambda: (api.reset(), world.set_world(world.StubWorld())))
        vd = mechdefs.BY_NAME["BASTION"]
        vik = ik_mod.LegIK(vd["plane"], vd["coxa"], vd["lengths"], vd["sides"], V3(-1, 0, 0), vd["limits"])
        false_pos = 0
        for ix in range(-5, 6):
            for iy in (-1.5, -0.8, 0.0):
                for iz in range(-5, 6):
                    tgt = V3(ix * vik.reach * 0.22, iy, iz * vik.reach * 0.22)
                    if vik.feasible(V3(), LOCAL_FRAME, tgt, slack_deg=0.0):
                        p = ik_mod.LegIK(vd["plane"], vd["coxa"], vd["lengths"], vd["sides"], V3(-1, 0, 0), vd["limits"])
                        for _ in range(3):
                            p.solve(V3(), LOCAL_FRAME, tgt)
                        if p.err >= 0.03:
                            false_pos += 1
        self.assertEqual(false_pos, 0)

        # Raycast rate across 60 Hz vs 144 Hz stays bounded
        rates = {}
        for hz in (60, 144):
            api.reset()
            w = world.set_world(world.StubWorld())
            api.spawn("STRIKER", 0, 0, 0, 0)
            rays = 0
            secs = 3.0
            n = int(secs * hz)
            dt = 1.0 / hz
            for _ in range(n):
                api.step(dt, controls(fwd=1, run=1, turn=0.8))
                rays += api._stats["rays"]
            rates[hz] = rays / secs
        self.assertLess(abs(rates[144] - rates[60]) / max(1.0, rates[60]), 0.20)

    def test_bound_flight_all_quads_and_octos_no_inverted_hover(self):
        from gaitcore import world
        self.addCleanup(lambda: (api.reset(), world.set_world(world.StubWorld())))
        rec = controls(fwd=1, run=1)
        dt = 1.0 / 60.0
        for name in ("LYNX", "BASTION", "STRIKER"):
            with self.subTest(mech=name):
                api.reset()
                world.set_world(world.StubWorld())
                h = api.spawn(name, 0, 0, 0, 0)
                m = api._mechs[h]
                for _ in range(240):
                    api.step(dt, rec)
                fl_cnt = 0
                hover_frames = 0
                for _ in range(180):
                    api.step(dt, rec)
                    npl = sum(1 for l in m.legs if l.planted)
                    if npl == 0:
                        if m.flight_t > 0.0:
                            fl_cnt += 1
                        elif m.state == 0:
                            hover_frames += 1
                self.assertGreater(fl_cnt, 0)
                self.assertEqual(hover_frames, 0)

    def test_slow_walk_symmetry_and_no_grounded_hover(self):
        import os, sys
        from gaitcore import world
        tools_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "tools"))
        if tools_dir not in sys.path:
            sys.path.insert(0, tools_dir)
        import gait_select_check
        self.addCleanup(lambda: (api.reset(), world.set_world(world.StubWorld())))
        dt = 1.0 / 60.0
        rec = controls(fwd=1, slow=1)
        for name, max_spread in (("STRIKER", 0.025), ("MANTIS", 0.030)):
            with self.subTest(slow_mech=name):
                api.reset()
                world.set_world(world.StubWorld())
                h = api.spawn(name, 0, 0, 0, 0)
                m = api._mechs[h]
                for _ in range(180):
                    api.step(dt, rec)
                prev_pl = [l.planted for l in m.legs]
                cur = [0.0 for _ in m.legs]
                started = [False for _ in m.legs]
                durs = [[] for _ in m.legs]
                fl_cnt = 0
                for _ in range(360):
                    api.step(dt, rec)
                    npl = sum(1 for l in m.legs if l.planted)
                    if npl == 0:
                        fl_cnt += 1
                    for i, l in enumerate(m.legs):
                        if l.planted:
                            if not prev_pl[i]:
                                started[i] = True
                                cur[i] = dt
                            elif started[i]:
                                cur[i] += dt
                        elif prev_pl[i] and started[i]:
                            durs[i].append(cur[i])
                            cur[i] = 0.0
                            started[i] = False
                        prev_pl[i] = l.planted
                if name == "STRIKER":
                    self.assertEqual(fl_cnt, 0)
                means = [sum(ds) / len(ds) for ds in durs if ds]
                self.assertEqual(len(means), len(m.legs))
                self.assertLess(max(means) - min(means), max_spread)

        run_rec = controls(fwd=1, run=1)
        for kind in ("rubble", "stairs"):
            with self.subTest(hover_lane=kind):
                api.reset()
                world.set_world(gait_select_check.lane(kind))
                h = api.spawn("STRIKER", 0, 0, 0, 0)
                m = api._mechs[h]
                hover_frames = 0
                consec = 0
                max_consec = 0
                for _ in range(720):
                    api.step(dt, run_rec)
                    npl = sum(1 for l in m.legs if l.planted)
                    if npl == 0 and m.flight_t <= 0.0 and m.state == 0:
                        hover_frames += 1
                        consec += 1
                        if consec > max_consec:
                            max_consec = consec
                    else:
                        consec = 0
                self.assertLessEqual(max_consec, 1)
                self.assertLessEqual(hover_frames, 5)

    def test_narrow_bridge_warden_5ms_and_stairs_144hz_parity(self):
        import os, sys
        from gaitcore import world, mechdefs
        tools_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "tools"))
        if tools_dir not in sys.path:
            sys.path.insert(0, tools_dir)
        import gait_select_check
        self.addCleanup(lambda: (api.reset(), world.set_world(world.StubWorld())))
        dt = 1.0 / 60.0
        api.reset()
        w = world.StubWorld(ground_y=-8.0)
        w.add_box((-20.0, -8.5, -20.0), (20.0, 0.0, 10.0))
        w.add_box((-1.25, -8.5, 10.0), (1.25, 0.0, 34.0))
        w.add_box((-20.0, -8.5, 34.0), (20.0, 0.0, 80.0))
        world.set_world(w)
        h = api.spawn("WARDEN", 0, 0, 0, 0)
        m = api._mechs[h]
        rec = controls(fwd=5.0 / mechdefs.BY_NAME["WARDEN"]["speeds"]["walk"])
        fell = False
        for _ in range(600):
            api.step(dt, rec)
            if m.pos.y < -2.0:
                fell = True
                break
        self.assertFalse(fell)
        self.assertGreater(m.pos.z, 34.0)

        zs = {}
        run_rec = controls(fwd=1, run=1)
        for hz in (60, 144):
            api.reset()
            world.set_world(gait_select_check.lane("stairs"))
            h = api.spawn("STRIKER", 0, 0, 55.0, 0)
            m = api._mechs[h]
            step_dt = 1.0 / hz
            for _ in range(int(4.0 * hz)):
                api.step(step_dt, run_rec)
            zs[hz] = m.pos.z
        self.assertLess(abs(zs[144] - zs[60]), 12.0)

    def test_round4_turn_arc_bound_symmetry_and_rubble_parity(self):
        import os, sys
        from gaitcore import world
        tools_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "tools"))
        if tools_dir not in sys.path:
            sys.path.insert(0, tools_dir)
        import gait_select_check
        self.addCleanup(lambda: (api.reset(), world.set_world(world.StubWorld())))
        dt = 1.0 / 60.0

        # 1) Turn arc_w actively modulates mid-swing feet without extreme stretch or coxa snaps
        api.reset()
        world.set_world(world.StubWorld())
        h = api.spawn("STRIKER", 0, 0, 0, 0)
        m = api._mechs[h]
        drive(controls(fwd=1, run=1), 2.0)
        prev_yaw = [l.ik.yaw_deg for l in m.legs]
        max_arc_w = 0.0
        max_sw_str = 0.0
        max_coxa_dps = 0.0
        rec_turn = controls(fwd=1, run=1, turn=1)
        for _ in range(150):
            api.step(dt, rec_turn)
            for l in m.legs:
                if l.swing is not None:
                    max_arc_w = max(max_arc_w, l.swing.arc_w)
                    max_sw_str = max(max_sw_str, l.stretch)
                max_coxa_dps = max(max_coxa_dps, abs(l.ik.yaw_deg - prev_yaw[l.idx]) * 60.0)
            prev_yaw = [l.ik.yaw_deg for l in m.legs]
        self.assertGreater(max_arc_w, 0.5)
        self.assertLess(max_sw_str, 1.25)
        self.assertLess(max_coxa_dps, 2800.0)

        # 2) bound front-pair L0/R0 stance symmetry
        rec_run = controls(fwd=1, run=1)
        for name in ("STRIKER", "LYNX"):
            with self.subTest(bound_pair=name):
                api.reset()
                world.set_world(world.StubWorld())
                h = api.spawn(name, 0, 0, 0, 0)
                m = api._mechs[h]
                drive(rec_run, 2.5)
                st = [[], []]
                cur = [0.0, 0.0]
                was = [m.legs[0].planted, m.legs[1].planted]
                for _ in range(180):
                    api.step(dt, rec_run)
                    for i in (0, 1):
                        pl = m.legs[i].planted
                        if pl:
                            cur[i] += dt
                        elif was[i] and cur[i] > 0.0:
                            st[i].append(cur[i])
                            cur[i] = 0.0
                        was[i] = pl
                m0 = sum(st[0]) / len(st[0])
                m1 = sum(st[1]) / len(st[1])
                self.assertLessEqual(abs(m0 - m1), 0.017)

        # 3) rubble 30/60/120 Hz parity and low joint pops on rubble/stairs
        zs = {}
        for hz in (30, 60, 120):
            api.reset()
            world.set_world(gait_select_check.lane("rubble"))
            h = api.spawn("STRIKER", 0, 0, 0, 0)
            m = api._mechs[h]
            step_dt = 1.0 / hz
            for _ in range(int(6.0 * hz)):
                api.step(step_dt, rec_run)
            zs[hz] = m.pos.z
        self.assertLess(max(zs.values()) - min(zs.values()), 25.0)

        for kind in ("rubble", "stairs"):
            with self.subTest(pop_lane=kind):
                api.reset()
                world.set_world(gait_select_check.lane(kind))
                h = api.spawn("STRIKER", 0, 0, 0, 0)
                m = api._mechs[h]
                prev_a = [tuple(l.ik.bends_deg) + (l.ik.yaw_deg,) for l in m.legs]
                pops = 0
                for _ in range(360):
                    api.step(dt, rec_run)
                    for l in m.legs:
                        cur_a = tuple(l.ik.bends_deg) + (l.ik.yaw_deg,)
                        if max(abs(a - b) for a, b in zip(cur_a, prev_a[l.idx])) >= 35.0:
                            pops += 1
                        prev_a[l.idx] = cur_a
                self.assertLessEqual(pops, 10 if kind == "rubble" else 6)

    def test_round4_colossus_narrow_bridge_safe(self):
        from gaitcore import mech as mech_mod, mechdefs, world
        self.addCleanup(lambda: (api.reset(), world.set_world(world.StubWorld())))
        api.reset()
        w = world.StubWorld(ground_y=-20.0)
        w.add_box((-30.0, -20.5, -30.0), (30.0, 0.0, 5.0))
        w.add_box((-1.25, -20.5, 5.0), (1.25, 0.0, 25.0))
        w.add_box((-30.0, -20.5, 25.0), (30.0, 0.0, 80.0))
        world.set_world(w)
        h = api.spawn("COLOSSUS", 0, 0, -5, 0)
        m = api._mechs[h]
        rec = controls(fwd=5.0 / mechdefs.BY_NAME["COLOSSUS"]["speeds"]["walk"])
        min_y = 999.0
        saw_air = False
        for _ in range(360):
            api.step(1.0 / 60.0, rec)
            min_y = min(min_y, m.pos.y)
            if m.state == mech_mod.AIR:
                saw_air = True
        self.assertGreater(min_y, 0.0)
        self.assertFalse(saw_air)

    def test_easy_obstacles_do_not_force_crawl(self):
        from gaitcore import mech as mech_mod, mechdefs, world
        self.addCleanup(lambda: (api.reset(), world.set_world(world.StubWorld())))
        dt = 1.0 / 60.0

        def lab_ledges():
            w = world.StubWorld()
            for i, lh in enumerate((0.5, 0.9, 1.3, 1.7, 2.1)):
                z0 = 15.0 + i * 20.0
                w.add_box((-10.0, 0.0, z0), (10.0, lh, z0 + 6.0))
            return w

        def single_box(bh):
            w = world.StubWorld()
            w.add_box((-4.0, 0.0, 12.0), (4.0, bh, 16.0))
            return w

        def drive_track(name, w_fn, secs, run_mode):
            api.reset()
            world.set_world(w_fn())
            h = api.spawn(name, 0, 0, 0, 0)
            m = api._mechs[h]
            rec = controls(fwd=1, run=int(run_mode))
            min_cap = 3
            for _ in range(round(secs * 60)):
                api.step(dt, rec)
                min_cap = min(min_cap, m.selector.cap)
            return m, min_cap

        for name in ("STRIKER", "LYNX", "MANTIS", "WARDEN", "BASTION"):
            d = mechdefs.BY_NAME[name]
            with self.subTest(case=("step", name)):
                _, min_cap = drive_track(name, lambda bh=0.85 * d["step_max"]: single_box(bh), 6.0, True)
                self.assertGreaterEqual(min_cap, 2)
            with self.subTest(case=("rock", name)):
                _, min_cap = drive_track(name, lambda: single_box(0.5), 6.0, True)
                self.assertGreaterEqual(min_cap, 2)
            for is_run in (False, True):
                with self.subTest(case=("lab_ledges", name, is_run)):
                    m, min_cap = drive_track(name, lab_ledges, 18.0, is_run)
                    self.assertGreaterEqual(min_cap, 2)
                    if name in ("STRIKER", "LYNX"):
                        self.assertGreater(m.pos.z, 105.0)

    def test_air_brace_preserves_tilt_and_rights_instantly_on_landing(self):
        import math
        from gaitcore import mech as mech_mod, world
        from gaitcore.vmath import Frame, angle_between, rotate_axis
        self.addCleanup(lambda: (api.reset(), world.set_world(world.StubWorld())))
        dt = 1.0 / 60.0

        def check_air_to_land(m, max_frames=360, drive_fn=None):
            saw_air = False
            tilt_pre = 0.0
            brace_foot_dy = 999.0
            land_f = None
            righted_250ms = None
            settled_err = 0.0
            for f in range(max_frames):
                rec = drive_fn(f, m) if drive_fn is not None else controls()
                prev_s = m.state
                prev_tilt = math.degrees(angle_between(m.frame.up, WORLD_UP))
                min_fy = min(l.foot.y for l in m.legs)
                api.step(dt, rec)
                if m.state == mech_mod.AIR:
                    saw_air = True
                    if m._brace_weight() > 0.5:
                        brace_foot_dy = min(brace_foot_dy, min_fy - m.pos.y)
                elif saw_air and prev_s == mech_mod.AIR and m.state == mech_mod.GROUND and land_f is None:
                    land_f = f
                    tilt_pre = prev_tilt
                elif land_f is not None:
                    since = (f - land_f) * dt
                    tilt_now = math.degrees(angle_between(m.frame.up, WORLD_UP))
                    if since >= 0.25 and righted_250ms is None:
                        righted_250ms = tilt_now
                    if since >= 0.40:
                        for l in m.legs:
                            if l.planted:
                                settled_err = max(settled_err, l.ik.err)
            self.assertTrue(saw_air)
            self.assertIsNotNone(land_f)
            self.assertIsNotNone(righted_250ms)
            self.assertLess(brace_foot_dy, -0.55 * m.ride)
            self.assertLess(righted_250ms, 5.0)
            self.assertLess(settled_err, 0.015)
            return tilt_pre

        # 1) Jump launched from a 25 deg ramp onto flat floor (controls released once airborne)
        api.reset()
        w = world.StubWorld()
        rad = math.radians(25.0)
        w.add_obox((0.0, 4.0, 18.0), (10.0, 0.4, 12.0), 0.0, -rad)
        world.set_world(w)
        h = api.spawn("STRIKER", 0.0, 0.0, 0.0, 0.0)
        m = api._mechs[h]
        jumped = False
        for _ in range(360):
            up_tilt = math.degrees(angle_between(m.frame.up, WORLD_UP))
            want_j = (not jumped) and m.pos.z > 21.0 and up_tilt >= 22.0
            if want_j:
                jumped = True
            api.step(dt, controls(fwd=0.35, jump=float(want_j)))
            if m.state == mech_mod.AIR:
                break
        self.assertEqual(m.state, mech_mod.AIR)
        tilt_pre = check_air_to_land(m)
        self.assertGreaterEqual(tilt_pre, 18.0)

        # 2) Jump launched while pitched +25 deg with aim, then aim released in air
        api.reset()
        world.set_world(world.StubWorld())
        h = api.spawn("STRIKER", 0.0, 0.0, 0.0, 0.0)
        m = api._mechs[h]
        aim_rad = math.radians(25.0)
        drive(controls(aim=aim_rad), 0.8)
        api.step(dt, controls(aim=aim_rad, jump=1.0))
        for _ in range(40):
            if m.state == mech_mod.AIR:
                break
            api.step(dt, controls(aim=aim_rad))
        self.assertEqual(m.state, mech_mod.AIR)
        tilt_pre = check_air_to_land(m)
        self.assertGreaterEqual(tilt_pre, 18.0)

        # 3) 45 deg rolled drops (radial + sagittal) and 75 deg tilted drop preserve air tilt & snap upright on impact
        for name, roll_deg in (("STRIKER", 45.0), ("BASTION", 45.0), ("STRIKER", 75.0)):
            with self.subTest(drop=(name, roll_deg)):
                api.reset()
                world.set_world(world.StubWorld())
                h = api.spawn(name, 0.0, 0.0, 0.0, 0.0)
                m = api._mechs[h]
                m.pos = V3(0.0, 9.0, 0.0)
                m.vel = V3(4.0, 1.0, 0.0)
                fr = Frame(V3(0.0, 0.0, 1.0), WORLD_UP.copy())
                fr.rotate(fr.fwd, math.radians(roll_deg))
                m.frame = fr
                m.state, m.state_t = mech_mod.AIR, 0.0
                m.ang_vel = V3()
                init_tilt = math.degrees(angle_between(m.frame.up, WORLD_UP))
                tilt_pre = check_air_to_land(m)
                self.assertGreaterEqual(tilt_pre, 0.85 * init_tilt)

    def test_benchmark_config_and_profiler_api(self):
        import array
        from gaitcore import world
        self.addCleanup(lambda: (api.reset(), world.set_world(world.StubWorld())))

        api.reset()
        world.set_world(world.StubWorld())
        h0 = api.spawn("STRIKER", 0.0, 0.0, 0.0, 0.0)
        h1 = api.spawn("MANTIS", 8.0, 0.0, 0.0, 0.0)
        h2 = api.spawn("WARDEN", -8.0, 0.0, 0.0, 0.0)
        self.assertEqual(api.count(), 3)

        m0, m1, m2 = api._mechs[h0], api._mechs[h1], api._mechs[h2]
        base_walk_spd = m0.target_speed()

        api.set_bench_config(
            max_substep=1.0 / 60.0,
            leg_ik_hz=15.0,
            probe_hz=10.0,
            speed_scale=0.5,
            plane_roll=False,
            profile=True,
        )
        self.assertAlmostEqual(m0.target_speed(), 0.5 * base_walk_spd, places=4)
        self.assertEqual(m1.probe_period, 0.1)
        self.assertFalse(m2.plane_roll_enabled)

        rec = controls(fwd=1)
        inp = b"".join(bytes(array.array("f", rec)) for _ in range(3))
        for _ in range(12):
            api.step(1.0 / 60.0, inp)
            for mm in (m0, m1, m2):
                for l in mm.legs:
                    self.assertLess(l.joints[0].dist(l.mount_world), 1e-4)
            api.poses()

        prof = array.array("f")
        prof.frombytes(api.profile_data())
        self.assertEqual(len(prof), api.PROFILE_FLOATS)
        self.assertGreater(prof[0], 0.0)   # total step ms
        self.assertGreater(prof[10], 0.0)  # ik solves
        self.assertGreater(prof[11], 0.0)  # ik skipped via reproject_cached
        self.assertEqual(int(round(prof[15])), 3)   # active mechs
        self.assertEqual(int(round(prof[14])), 18)  # 4 + 6 + 8 legs
        for m in (m0, m1, m2):
            base = api.PROFILE_HEADER + m.d["kind_id"] * api.PROFILE_KIND_STRIDE
            self.assertEqual(int(round(prof[base])), 1)
            self.assertEqual(int(round(prof[base + 1])), m.n)
            self.assertGreater(prof[base + 2], 0.0)

        self.assertEqual(api.despawn_to(1), 1)
        self.assertEqual(api.count(), 1)

        api.reset()
        self.assertFalse(api._profile)
        self.assertEqual(api._leg_ik_hz, 0.0)
        self.assertEqual(api._speed_scale, 1.0)
        self.assertAlmostEqual(api._probe_period, 0.05, places=6)
        self.assertTrue(api._plane_roll)
        self.assertEqual(api._max_substep, api.MAX_SUBSTEP)

    def test_strider_biped_and_gecko_climber(self):
        from gaitcore import mech as mech_mod, world
        self.addCleanup(lambda: (api.reset(), world.set_world(world.StubWorld())))
        dt = 1.0 / 60.0

        # 1) STRIDER biped walks and runs cleanly with both legs alternating
        api.reset()
        world.set_world(world.StubWorld())
        hb = api.spawn("STRIDER", 0.0, 0.0, 0.0, 0.0)
        mb = api._mechs[hb]
        self.assertEqual(mb.n, 2)
        self.assertTrue(mb.biped)
        steps_b = [0, 0]
        was_b = [l.planted for l in mb.legs]
        for _ in range(180):
            api.step(dt, controls(fwd=1, run=1))
            for idx_l, l in enumerate(mb.legs):
                if was_b[idx_l] and not l.planted:
                    steps_b[idx_l] += 1
                was_b[idx_l] = l.planted
                self.assertLess(l.ik.err, 0.05)
        self.assertGreater(mb.pos.z, 20.0)
        self.assertGreaterEqual(min(steps_b), 3)

        # 2) GECKO climbs up a vertical 8 m wall on its feet
        api.reset()
        w = world.StubWorld()
        w.add_box((-8.0, 0.0, 6.0), (8.0, 8.0, 14.0))
        world.set_world(w)
        hg = api.spawn("GECKO", 0.0, 0.0, 0.0, 0.0)
        mg = api._mechs[hg]
        self.assertTrue(mg.climbing_mode)
        saw_wall = False
        max_y = 0.0
        for _ in range(420):
            api.step(dt, controls(fwd=1))
            if mg.on_wall:
                saw_wall = True
            max_y = max(max_y, mg.pos.y)
        self.assertTrue(saw_wall)
        self.assertGreater(max_y, 6.0)
        self.assertEqual(mg.state, mech_mod.GROUND)
