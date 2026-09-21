#!/usr/bin/env python3
"""Arena Calibrate, Align, and Move 1 m -- PyBullet simulation entry point.

Operator workflow (terminal driven, per SPEC):
  1. Program starts and opens the (simulated) overhead camera.
  2. It continuously reports anchor markers 0-3.  When all four are visible,
     press `c` to lock the homography calibration (pixel -> arena meters).
  3. Press `r` to (re)spawn the bot at a random pose, then confirm with
     Enter (or `s`).  The program waits for marker 4 (the bot) and reports
     the detected pose.
  4. It closed-loop pivots the bot to theta = 0 rad (+X), then drives
     forward 1 m along +X using ONLY camera feedback.
  5. The bot coasts at the end and on every abort path (`q`, Ctrl-C,
     timeout, camera loss).

Keys: c = lock calibration, r = respawn random pose, q = coast + quit.

`--headless` runs DIRECT physics (no GUI).  With an interactive terminal it
behaves as above; when stdin is closed/EOF (CI, pipes) it auto-drives the
whole sequence once and exits, so `python3 main.py --headless` is a complete
smoke test.  Piping `printf 'c\\n'` locks calibration via the real key path
and then auto-completes the rest.

`App.handle_key(ch)` is public for tests.
"""

import argparse
import math
import os
import random
import sys

import pybullet as p

import config as C
from calibration import CalibrationError, HomographyCalibrator
from control import BotController, CameraPoseError, CameraPoseSource
from vision import VisionSystem

BANNER = """\
======================================================================
  ARENA CALIBRATE, ALIGN & MOVE 1 m  (PyBullet sim)
----------------------------------------------------------------------
  STEP 1  overhead camera opens; anchors 0-3 are reported below
  STEP 2  press 'c' when ALL FOUR anchors are visible -> locks homography
  STEP 3  press 'r' to spawn/respawn the bot at a random pose
  STEP 4  press ENTER (or 's') to start: waits for marker 4, reports pose,
          pivots to theta=0 (+X), then drives 1 m along +X (camera-only)
  STEP 5  bot coasts at the end / on any abort
  keys: c = lock calibration   r = respawn random   q = coast + quit
======================================================================"""

STATE_WAIT_ANCHORS = "WAIT_ANCHORS"
STATE_AWAIT_BOT = "AWAIT_BOT"
STATE_RUNNING = "RUNNING"      # align+move sequence in progress
STATE_DONE = "DONE"


# ---------------------------------------------------------------------------
# Non-blocking terminal key polling (POSIX termios/select, Windows msvcrt)
# ---------------------------------------------------------------------------
class KeyPoller:
    def __init__(self):
        self.is_tty = sys.stdin.isatty()
        self.eof = False
        self._old = None
        if os.name == "posix" and self.is_tty:
            import termios
            import tty
            self._old = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

    def poll(self):
        """Return one pressed key char, or None.  Sets self.eof at EOF."""
        if self.eof:
            return None
        if os.name == "nt":
            import msvcrt
            return msvcrt.getwch() if msvcrt.kbhit() else None
        import select
        try:
            r, _, _ = select.select([sys.stdin], [], [], 0)
        except (OSError, ValueError):
            self.eof = True
            return None
        if not r:
            return None
        ch = sys.stdin.read(1)
        if ch == "":
            self.eof = True
            return None
        return ch

    def close(self):
        if self._old is not None:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old)
            self._old = None


# ---------------------------------------------------------------------------
# Sim world wrapper
# ---------------------------------------------------------------------------
class ArenaSim:
    def __init__(self, gui=False):
        if gui:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        here = os.path.dirname(os.path.abspath(__file__))
        self.arena_id = p.loadURDF(os.path.join(here, "urdf", "arena.urdf"),
                                   useFixedBase=True, physicsClientId=self.client)
        self.bot_urdf = os.path.join(here, "urdf", "bot.urdf")
        self.bot_id = None

    def spawn_bot_random(self, rng):
        self.remove_bot()
        x = rng.uniform(*C.BOT_SPAWN_XY)
        y = rng.uniform(*C.BOT_SPAWN_XY)
        th = rng.uniform(-math.pi, math.pi)
        self.bot_id = p.loadURDF(
            self.bot_urdf, basePosition=[x, y, 0.0],
            baseOrientation=p.getQuaternionFromEuler([0, 0, th]),
            physicsClientId=self.client)
        for _ in range(10):
            p.stepSimulation(physicsClientId=self.client)
        return x, y, th

    def remove_bot(self):
        if self.bot_id is not None:
            p.removeBody(self.bot_id, physicsClientId=self.client)
            self.bot_id = None

    def ground_truth_pose(self):
        """Sim verification ONLY -- never fed to the controller."""
        if self.bot_id is None:
            return None
        pos, orn = p.getBasePositionAndOrientation(
            self.bot_id, physicsClientId=self.client)
        yaw = p.getEulerFromQuaternion(orn)[2]
        return (pos[0] + C.ANCHOR_HALF, pos[1]), yaw

    def close(self):
        p.disconnect(self.client)


# ---------------------------------------------------------------------------
# Application / state machine
# ---------------------------------------------------------------------------
class App:
    def __init__(self, headless=False, seed=None, quiet=False):
        self.headless = headless
        self.quiet = quiet
        self.rng = random.Random(seed)
        self.sim = ArenaSim(gui=not headless)
        self.vision = VisionSystem(self.sim.client)
        self.calibrator = HomographyCalibrator()
        self.state = STATE_WAIT_ANCHORS
        self.running = True
        self.poller = KeyPoller()
        self._status_every = 10           # cycles between anchor status prints
        self._cycle = 0

    # ------------------------------------------------------------------
    def _say(self, msg):
        if not self.quiet:
            print(msg, flush=True)

    # ------------------------------------------------------------------
    def _try_lock(self):
        frame = self.vision.render()
        anchors = self.vision.detect_anchors(frame)
        try:
            self.calibrator.lock(anchors)
        except CalibrationError as exc:
            self._say("[cal] LOCK REFUSED: %s" % exc)
            return False
        self.state = STATE_AWAIT_BOT
        self._say("[cal] homography LOCKED (max reprojection err %.2e m)"
                  % self.calibrator.reprojection_err_px)
        self._say("[cal] anchor dropouts from now on do NOT invalidate the lock")
        self._say("[bot] press 'r' to spawn the bot at a random pose")
        return True

    def _respawn(self):
        x, y, th = self.sim.spawn_bot_random(self.rng)
        self._say("[bot] respawned at random pose "
                  "(ground truth, sim only: x=%.2f y=%.2f th=%.1f deg)"
                  % (x, y, math.degrees(th)))

    def _run_sequence(self):
        """SPEC steps 5-7: wait for marker 4, report pose, align, move, coast."""
        if self.sim.bot_id is None:
            self._say("[bot] no bot placed -- press 'r' first")
            return
        if not self.calibrator.homography_locked:
            self._say("[bot] calibrate first (press 'c' with 4 anchors visible)")
            return
        self.state = STATE_RUNNING
        pose_source = CameraPoseSource(self.vision, self.calibrator)
        controller = BotController(self.sim.client, self.sim.bot_id,
                                   pose_source, on_status=self._say)
        aborted = {"flag": False}

        def abort_check():
            ch = self.poller.poll()
            if ch == "q":
                aborted["flag"] = True
                return True
            return False

        try:
            # Wait for marker 4 and report the detected pose.
            self._say("[bot] waiting for marker 4 ...")
            pose = None
            while pose is None:
                if abort_check():
                    break
                pose = pose_source.update()
            if pose is not None:
                (x, y), th = pose
                self._say("[bot] detected pose: x=%.3f m  y=%.3f m  "
                          "theta=%.1f deg (camera-derived)"
                          % (x, y, math.degrees(th)))
                self._say("[align] pivoting to theta=0 rad (+X) ...")
                ok_a = controller.align_to_heading(abort_check=abort_check)
                ok_m = False
                if ok_a:
                    self._say("[move] driving 1.0 m along +X ...")
                    ok_m = controller.move_forward_distance(
                        abort_check=abort_check)
                self.state = STATE_DONE
                if ok_a and ok_m:
                    est = pose_source.read()
                    self._say("[done] SUCCESS. camera final pose: "
                              "x=%.3f y=%.3f th=%.1f deg"
                              % (est[0][0], est[0][1], math.degrees(est[1])))
                else:
                    self._say("[done] sequence ended early "
                              "(align=%s move=%s aborted=%s)"
                              % (ok_a, ok_m, aborted["flag"]))
                gt = self.sim.ground_truth_pose()
                if gt is not None:
                    self._say("[done] ground truth (sim verification only): "
                              "x=%.3f y=%.3f th=%.1f deg"
                              % (gt[0][0], gt[0][1], math.degrees(gt[1])))
        except CameraPoseError as exc:
            controller.coast()
            self._say("[abort] camera pose lost: %s -- coasted" % exc)
            self.state = STATE_DONE
        except KeyboardInterrupt:
            controller.coast()
            self._say("[abort] Ctrl-C -- coasted")
            self.running = False
        self._say("[bot] press 'r' to respawn, ENTER to run again, 'q' to quit")

    # ------------------------------------------------------------------
    # Public key handler (used by the loop AND by tests)
    # ------------------------------------------------------------------
    def handle_key(self, ch):
        if ch is None:
            return
        if ch == "q":
            self._say("[key] q -> coasting and quitting")
            self.running = False
        elif ch == "c":
            if self.state == STATE_WAIT_ANCHORS:
                self._try_lock()
            else:
                self._say("[key] calibration already locked "
                          "(homography is frozen while locked)")
        elif ch == "r":
            self._respawn()
            if self.state == STATE_DONE:
                self.state = STATE_AWAIT_BOT
        elif ch in ("\n", "\r", "s"):
            if self.state == STATE_AWAIT_BOT or self.state == STATE_DONE:
                self._run_sequence()
            else:
                self._say("[key] lock calibration first (press 'c')")
        # all other keys ignored

    # ------------------------------------------------------------------
    def _anchor_status_line(self, anchors):
        cells = []
        for aid in C.ANCHOR_IDS:
            cells.append("%d:%s" % (aid, "OK" if aid in anchors else "--"))
        return ("[anchors] " + "  ".join(cells)
                + ("   <-- all visible: press 'c' to lock calibration"
                   if len(anchors) == 4 else ""))

    def run(self):
        self._say(BANNER)
        while self.running:
            ch = self.poller.poll()
            if ch is not None:
                self.handle_key(ch)
                continue
            # Headless + stdin closed -> auto-drive (CI smoke test).
            auto = self.headless and self.poller.eof
            if self.state == STATE_WAIT_ANCHORS:
                frame = self.vision.render()
                anchors = self.vision.detect_anchors(frame)
                self._cycle += 1
                if self._cycle % self._status_every == 1 or len(anchors) == 4:
                    self._say(self._anchor_status_line(anchors))
                if auto and len(anchors) == 4:
                    self._say("[auto] stdin closed: auto-locking calibration")
                    self._try_lock()
            elif self.state == STATE_AWAIT_BOT and auto:
                self._say("[auto] spawning bot at random pose")
                self._respawn()
                self._run_sequence()
            elif self.state == STATE_DONE and auto:
                self._say("[auto] sequence complete, exiting")
                self.running = False
            else:
                # idle: keep the camera warm and the loop responsive
                self.vision.render()
        self.poller.close()
        self.sim.close()
        self._say("[exit] clean shutdown")
        return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--headless", action="store_true",
                    help="DIRECT physics (no GUI); auto-drives the sequence "
                         "when stdin is closed")
    ap.add_argument("--seed", type=int, default=None,
                    help="seed for random bot spawn")
    args = ap.parse_args(argv)
    app = App(headless=args.headless, seed=args.seed)
    try:
        return app.run()
    except KeyboardInterrupt:
        print("\n[abort] Ctrl-C -- coasted")
        return 130


if __name__ == "__main__":
    sys.exit(main())
