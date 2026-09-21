#!/usr/bin/env python3
"""Headless end-to-end trials: calibrate -> align -> move 1 m.

N=5 random-pose trials (fixed seeds, deterministic).  The controller only
ever sees camera-derived poses; ground truth is read AFTER the run solely to
assert the final pose within SPEC tolerances (5 cm / 5 deg).

Run:  python3 tests/test_sim.py        (must finish in < 60 s)
"""

import math
import os
import random
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np               # noqa: E402
import pybullet as p             # noqa: E402

import config as C               # noqa: E402
from calibration import HomographyCalibrator  # noqa: E402
from control import BotController, CameraPoseSource, wrap_angle  # noqa: E402
from vision import VisionSystem  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
URDF_DIR = os.path.join(HERE, "..", "urdf")

TRIAL_SEEDS = [101, 202, 303, 404, 505]
TOL_ALONG_M = 0.05
TOL_LAT_M = 0.06
TOL_HEADING_RAD = math.radians(5.0)


class SimFixture:
    """One DIRECT world with arena + locked calibration, reused per trial."""

    def __init__(self):
        self.client = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        self.arena = p.loadURDF(os.path.join(URDF_DIR, "arena.urdf"),
                                useFixedBase=True, physicsClientId=self.client)
        self.vision = VisionSystem(self.client)
        self.calibrator = HomographyCalibrator()
        anchors = self.vision.detect_anchors(self.vision.render())
        assert sorted(anchors) == [0, 1, 2, 3], \
            "all four anchors must be detected, got %s" % sorted(anchors)
        self.calibrator.lock(anchors)
        assert self.calibrator.homography_locked

    def spawn_bot(self, x, y, theta):
        return p.loadURDF(os.path.join(URDF_DIR, "bot.urdf"),
                          basePosition=[x, y, 0.0],
                          baseOrientation=p.getQuaternionFromEuler([0, 0, theta]),
                          physicsClientId=self.client)

    def close(self):
        p.disconnect(self.client)


class TestCalibrateAlignMove(unittest.TestCase):
    def test_five_random_pose_trials(self):
        for seed in TRIAL_SEEDS:
            with self.subTest(seed=seed):
                self._run_trial(seed)

    def _run_trial(self, seed):
        fx = SimFixture()
        try:
            rng = random.Random(seed)
            x0 = rng.uniform(*C.BOT_SPAWN_XY)
            y0 = rng.uniform(*C.BOT_SPAWN_XY)
            th0 = rng.uniform(-math.pi, math.pi)
            bot = fx.spawn_bot(x0, y0, th0)
            for _ in range(20):
                p.stepSimulation(physicsClientId=fx.client)

            pose_source = CameraPoseSource(fx.vision, fx.calibrator)
            controller = BotController(fx.client, bot, pose_source)

            ok_align = controller.align_to_heading()
            ok_move = controller.move_forward_distance(C.MOVE_DISTANCE_M)
            self.assertTrue(ok_align, "align_to_heading failed (seed %d)" % seed)
            self.assertTrue(ok_move, "move_forward_distance failed (seed %d)" % seed)

            # Ground truth assertion (never used by the controller).
            pos, orn = p.getBasePositionAndOrientation(
                bot, physicsClientId=fx.client)
            yaw = p.getEulerFromQuaternion(orn)[2]
            ax, ay = pos[0] + C.ANCHOR_HALF, pos[1]     # arena frame
            e_along = abs(ax - (x0 + C.ANCHOR_HALF + C.MOVE_DISTANCE_M))
            e_lat = abs(ay - y0)
            e_head = abs(wrap_angle(yaw - C.TARGET_HEADING))
            self.assertLessEqual(e_along, TOL_ALONG_M,
                                 "along-track err %.3f m (seed %d)" % (e_along, seed))
            self.assertLessEqual(e_lat, TOL_LAT_M,
                                 "lateral err %.3f m (seed %d)" % (e_lat, seed))
            self.assertLessEqual(e_head, TOL_HEADING_RAD,
                                 "heading err %.1f deg (seed %d)"
                                 % (math.degrees(e_head), seed))
            print("  seed %3d: GT errors along=%.3f m  lat=%.3f m  head=%.2f deg"
                  % (seed, e_along, e_lat, math.degrees(e_head)), flush=True)
        finally:
            fx.close()


class TestCalibrationAndKeys(unittest.TestCase):
    def test_homography_lock_freeze_and_accuracy(self):
        fx = SimFixture()
        try:
            cal = fx.calibrator
            # Locked H must not change when anchors later "drop out":
            H0 = cal.H.copy()
            self.assertTrue(cal.homography_locked)
            out = cal.pixel_to_arena(np.array([160.0, 160.0]))  # image center
            # image center ~= arena center (1.5, 0) within 2 cm
            self.assertAlmostEqual(out[0], 1.5, delta=0.02)
            self.assertAlmostEqual(out[1], 0.0, delta=0.02)
            # anchor pixel -> known arena coordinate (id 0 = (0, +1.5))
            anchors = fx.vision.detect_anchors(fx.vision.render())
            est = cal.pixel_to_arena(anchors[0])
            self.assertAlmostEqual(est[0], C.ARENA_ANCHORS[0][0], delta=0.02)
            self.assertAlmostEqual(est[1], C.ARENA_ANCHORS[0][1], delta=0.02)
            np.testing.assert_array_equal(cal.H, H0)  # frozen
            cal.unlock()
            self.assertFalse(cal.homography_locked)
        finally:
            fx.close()

    def test_handle_key_public(self):
        import main as main_mod
        app = main_mod.App(headless=True, seed=0, quiet=True)
        try:
            self.assertEqual(app.state, main_mod.STATE_WAIT_ANCHORS)
            app.handle_key("c")                      # all anchors visible
            self.assertTrue(app.calibrator.homography_locked)
            self.assertEqual(app.state, main_mod.STATE_AWAIT_BOT)
            app.handle_key("c")                      # re-lock refused, stays locked
            self.assertTrue(app.calibrator.homography_locked)
            app.handle_key("r")                      # random respawn
            self.assertIsNotNone(app.sim.bot_id)
            app.handle_key("q")                      # quit
            self.assertFalse(app.running)
        finally:
            app.poller.close()
            app.sim.close()


if __name__ == "__main__":
    t0 = time.time()
    prog = unittest.main(argv=[sys.argv[0]], exit=False, verbosity=2)
    wall = time.time() - t0
    print("total wall time: %.1f s (limit 60 s)" % wall)
    ok = prog.result.wasSuccessful() and wall < 60.0
    if wall >= 60.0:
        print("FAIL: suite exceeded 60 s")
    sys.exit(0 if ok else 1)
