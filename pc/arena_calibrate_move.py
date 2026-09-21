#!/usr/bin/env python3
"""CMD-guided arena workflow: lock homography, align bot to +X, move 1 m.

This is the focused entry point for the workflow described by the operator:

  1. start program -> detect arena anchors 0-3;
  2. press `c` in the terminal -> lock the pixel->arena homography;
  3. place/start the bot anywhere in the arena, any heading;
  4. detect marker 4 pose;
  5. closed-loop align heading to theta = 0 rad (+X) within tolerance;
  6. move forward until the camera says the bot is inside the 1 m target band;
  7. stop correcting there (no exact-point chasing), coast, and report the result.

Run from the pc/ folder so sibling imports resolve:

    python3 arena_calibrate_move.py --ip 192.168.x.x --camera-index 0 --debug-view

Use --sim for a hardware-free logic check:

    python3 arena_calibrate_move.py --sim --yes
"""

import argparse
import math
import os
import sys
import time

from test_suite import (
    CMD_HZ,
    CMD_PERIOD,
    EmaPoseFilter,
    K_ALPHA,
    MAX_WHEEL_SPEED_MS,
    MIN_WHEEL_SPEED_MS,
    TRACK_WIDTH_M,
    BotLink,
    SimulatedPoseSource,
    ms_to_rpm,
    shape_wheel_speeds,
    wrap_pi,
)

DEFAULT_DISTANCE_M = 1.0
DEFAULT_ALIGN_TOL_RAD = 0.10       # about +/-5.7 deg; confirmed over several stable reads
DEFAULT_ALIGN_TIMEOUT_S = 30.0     # pulsed alignment is slower but much less jerky
DEFAULT_MOVE_TIMEOUT_S = 30.0
DEFAULT_MOVE_TOL_M = 0.05          # target is distance +/- this band, not an exact point
DEFAULT_LATERAL_TOL_M = 0.06       # acceptable lateral band around the +X line
DEFAULT_MOVE_SPEED_MS = 0.30       # user-validated smooth translation speed
ALIGN_WHEEL_SPEED_MS = MIN_WHEEL_SPEED_MS  # slowest executable pivot speed
ALIGN_STABLE_SAMPLES = 4           # require this many in-tolerance reads while coasting
MAX_COAST_TRIGGER_M = 0.05         # never drive past the near edge of the target band
MOVE_V_GAIN = 1.5
POSE_STABLE_SAMPLES = 5


class AbortRun(RuntimeError):
    """Operator abort (q/Ctrl-C) or safety stop."""


class NullLink(object):
    """Drop-in BotLink replacement for --sim: records commands, sends nothing."""

    def __init__(self):
        self.commands = []

    def send_command(self, left_rpm, right_rpm):
        self.commands.append((left_rpm, right_rpm))

    def recv_telemetry(self, timeout_s=0.0):
        return None

    def close(self):
        pass


class TerminalKeys(object):
    """Non-blocking single-key polling for CMD/PowerShell and POSIX terminals."""

    def __init__(self):
        self._fd = None
        self._old_term = None
        self._msvcrt = None
        if os.name == "nt":
            try:
                import msvcrt
                self._msvcrt = msvcrt
            except ImportError:
                self._msvcrt = None
        else:
            try:
                import termios
                import tty
                self._termios = termios
                self._tty = tty
                if sys.stdin.isatty():
                    self._fd = sys.stdin.fileno()
                    self._old_term = termios.tcgetattr(self._fd)
                    tty.setcbreak(self._fd)
            except Exception:
                self._fd = None
                self._old_term = None

    def poll(self):
        """Return one lowercase character if a key is waiting, else None."""
        if self._msvcrt is not None:
            if not self._msvcrt.kbhit():
                return None
            ch = self._msvcrt.getwch()
            if ch in ("\x00", "\xe0"):       # arrow/function key prefix
                if self._msvcrt.kbhit():
                    self._msvcrt.getwch()
                return None
            return ch.lower()
        if self._fd is None:
            return None
        try:
            import select
            ready, _w, _x = select.select([sys.stdin], [], [], 0.0)
            if not ready:
                return None
            ch = sys.stdin.read(1)
            return ch.lower() if ch else None
        except Exception:
            return None

    def close(self):
        if self._fd is not None and self._old_term is not None:
            try:
                self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN,
                                        self._old_term)
            except Exception:
                pass
            self._fd = None
            self._old_term = None


def coast(link, seconds=0.5, pose_source=None):
    """Explicitly coast while keeping the link alive.

    If pose_source is provided, tell it the commanded speed is now zero so its
    latency dead-reckoning does not keep predicting motion from stale commands.
    """
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        link.send_command(0.0, 0.0)
        if pose_source is not None:
            pose_source.update(0.0, 0.0, CMD_PERIOD)
        time.sleep(CMD_PERIOD)


def _poll_abort(keys):
    """Raise AbortRun when the operator presses q in a motion-phase loop."""
    if keys is not None and keys.poll() == "q":
        raise AbortRun("operator pressed q")


def in_target_band(displacement_m, lateral_m, distance_m,
                   move_tol_m, lateral_tol_m):
    """Robust noisy-arena success test: inside a range, not on an exact point."""
    return (abs(displacement_m - distance_m) <= move_tol_m
            and abs(lateral_m) <= lateral_tol_m)


def anchor_quad_status(detections, min_area_px2=10000.0, min_side_px=40.0):
    """Validate the four anchor detections before allowing a calibration lock.

    Returns (ok, message, metrics_dict). IDs are ordered 0->1->2->3, which is
    bottom-left -> bottom-right -> top-right -> top-left (a convex quad when the
    camera sees the arena correctly).
    """
    import cv2
    import numpy as np

    missing = [mid for mid in (0, 1, 2, 3) if mid not in detections]
    if missing:
        return False, "missing anchor IDs %s" % missing, {}
    pts = np.array([detections[mid].mean(axis=0) for mid in (0, 1, 2, 3)],
                   dtype=np.float32)
    contour = pts.reshape((-1, 1, 2))
    area = abs(float(cv2.contourArea(contour)))
    convex = bool(cv2.isContourConvex(contour))
    sides = [float(np.linalg.norm(pts[(i + 1) % 4] - pts[i])) for i in range(4)]
    metrics = {"area_px2": area, "min_side_px": min(sides), "convex": convex}
    if not convex:
        return False, "anchor quad is not convex (check marker IDs/placement)", metrics
    if area < min_area_px2:
        return False, "anchor quad too small (%.0f px^2); move camera closer" % area, metrics
    if min(sides) < min_side_px:
        return False, "anchor markers too close together in image", metrics
    return True, "anchors OK", metrics


def calibrate_camera(args, keys):
    """Open the camera and run the press-`c`-to-lock calibration phase."""
    import camera_pose

    src = camera_pose.CameraPoseSource(
        camera_index=args.camera_index,
        debug_view=args.debug_view,
        width=args.width,
        height=args.height,
        fps=args.fps,
        mjpeg=not args.no_mjpeg,
        latency_s=args.latency_s,
        focus=args.focus,
        exposure=args.exposure,
        max_hold_s=args.max_hold,
    )
    print("\nSTEP 1/5 - ARENA CALIBRATION")
    print("  Camera is open. Aim it so ALL FOUR arena anchors (IDs 0, 1, 2, 3)")
    print("  are visible. The robot marker (ID 4) is NOT needed yet.")
    print("  When the status line says READY, press c in THIS terminal to lock")
    print("  the homography. Press q to quit.")
    last_print = 0.0
    try:
        while True:
            frame, detections, _t = src.grab_detections()
            H = camera_pose.compute_homography(detections)
            quad_ok, quad_msg, metrics = anchor_quad_status(
                detections, args.min_anchor_area, args.min_anchor_side)
            ready = H is not None and quad_ok
            if args.debug_view:
                src.show_debug_frame(frame, detections, H, None, None)
            now = time.monotonic()
            if now - last_print >= 0.5:
                last_print = now
                seen = sorted(detections.keys())
                state = "READY - press c to lock" if ready else "WAITING"
                extra = ""
                if metrics:
                    extra = " | quad area %.0f px^2, min side %.0f px" % (
                        metrics["area_px2"], metrics["min_side_px"])
                print("  [calibrate] seen IDs=%s | %s | %s%s" % (
                    seen, state, quad_msg, extra))
            key = keys.poll()
            if key == "q":
                raise AbortRun("operator pressed q during calibration")
            if key == "c":
                if ready:
                    src.lock_homography(H)
                    print("\n  CALIBRATION LOCKED. Homography is frozen; anchor dropouts")
                    print("  will not change the meter coordinate mapping.")
                    return src
                print("  Cannot lock yet: %s" % quad_msg)
            time.sleep(0.03)
    except Exception:
        src.close()
        raise


def wait_for_robot_pose(pose_source, timeout_s=25.0, camera_mode=True, keys=None):
    """Wait until marker 4 produces a stable pose; return (pose, EMA filter)."""
    print("\nSTEP 3/5 - DETECT BOT POSE")
    print("  Waiting for robot marker ID 4 (top edge must point to the bot FRONT)...")
    filt = EmaPoseFilter()
    stable = 0
    last_print = 0.0
    t0 = time.monotonic()
    while True:
        _poll_abort(keys)
        now = time.monotonic()
        if now - t0 > timeout_s:
            raise AbortRun("timed out waiting for robot marker ID 4")
        try:
            raw = pose_source.read()
        except Exception as exc:
            stable = 0
            if now - last_print >= 1.0:
                last_print = now
                print("  [pose] not yet: %s" % exc)
            time.sleep(CMD_PERIOD)
            continue
        pose = filt.update(*raw)
        stable += 1
        if now - last_print >= 0.5:
            last_print = now
            print("  [pose] x=%+.3f m y=%+.3f m theta=%+.2f rad (stable %d/%d)" % (
                pose[0], pose[1], pose[2], stable, POSE_STABLE_SAMPLES))
        if stable >= POSE_STABLE_SAMPLES:
            print("  BOT POSE LOCKED: x=%+.3f m y=%+.3f m theta=%+.2f rad" % pose)
            return pose, filt
        time.sleep(CMD_PERIOD)


def _align_pulse_times(abs_herr):
    """Short rotate/settle pulses for low-FPS, high-stiction alignment."""
    if abs_herr > 1.0:
        return 0.10, 0.25
    if abs_herr > 0.35:
        return 0.07, 0.25
    return 0.05, 0.30


def align_to_heading(link, pose_source, filt, target_theta=0.0,
                     tol_rad=DEFAULT_ALIGN_TOL_RAD,
                     timeout_s=DEFAULT_ALIGN_TIMEOUT_S, keys=None,
                     align_wheel_speed_ms=ALIGN_WHEEL_SPEED_MS,
                     stable_samples=ALIGN_STABLE_SAMPLES):
    """Smooth pulsed pivot until heading is stably inside tolerance.

    Continuous pivoting was too aggressive for a low-FPS camera: the marker
    heading could swing during the blind settle window and produce a false
    "aligned" read. Here we rotate in short pulses, coast while re-measuring,
    and only declare alignment after several consecutive in-tolerance reads.
    """
    print("\nSTEP 4/5 - ALIGN TO +X")
    print("  Smooth pulsed pivot to theta=%.2f rad (tolerance %.2f rad)." % (
        target_theta, tol_rad))
    print("  Using short %.2f m/s wheel pulses with coast-and-remeasure settling." % (
        align_wheel_speed_ms))
    t0 = time.monotonic()
    last_print = 0.0
    t_prev = t0
    stable = 0
    phase = "sense"          # "rotate" or "settle"
    phase_until = t0
    direction = 0            # +1 = CCW (theta increases), -1 = CW
    v_left = v_right = 0.0
    while True:
        _poll_abort(keys)
        now = time.monotonic()
        if now - t0 > timeout_s:
            coast(link, pose_source=pose_source)
            raise AbortRun("alignment timed out after %.1f s" % timeout_s)
        dt = max(now - t_prev, 1e-3)
        t_prev = now
        try:
            raw = pose_source.read()
        except Exception:
            coast(link, pose_source=pose_source)
            raise
        px, py, ptheta = filt.update(*raw)
        herr = wrap_pi(target_theta - ptheta)

        if abs(herr) <= tol_rad:
            stable += 1
            phase = "sense"
            link.send_command(0.0, 0.0)
            pose_source.update(0.0, 0.0, dt)
            if stable >= stable_samples:
                break
        else:
            stable = 0
            pulse_s, settle_s = _align_pulse_times(abs(herr))
            if phase == "rotate" and now >= phase_until:
                phase = "settle"
                phase_until = now + settle_s
            elif phase != "rotate" and now >= phase_until:
                direction = 1.0 if herr > 0.0 else -1.0
                phase = "rotate"
                phase_until = now + pulse_s

            if phase == "rotate":
                # herr > 0 -> rotate CCW: left wheel backward, right wheel forward.
                v_left = -direction * align_wheel_speed_ms
                v_right = +direction * align_wheel_speed_ms
            else:
                v_left = v_right = 0.0
            link.send_command(ms_to_rpm(v_left), ms_to_rpm(v_right))
            pose_source.update(v_left, v_right, dt)

        if now - last_print >= 0.5:
            last_print = now
            print("  [align] pose=(%+.3f,%+.3f,%+.2f) herr=%+.2f rad phase=%s stable=%d/%d" % (
                px, py, ptheta, herr, phase, stable, stable_samples))
        sleep_s = CMD_PERIOD - (time.monotonic() - now)
        if sleep_s > 0.0:
            time.sleep(sleep_s)

    coast(link, 0.50, pose_source)
    try:
        raw = pose_source.read()
        px, py, ptheta = filt.update(*raw)
    except Exception:
        # Keep the last filtered pose if the camera hiccups during the settle.
        pass
    final_err = wrap_pi(target_theta - ptheta)
    print("  ALIGN COMPLETE: theta=%+.2f rad (error %+.2f rad)" % (
        ptheta, final_err))
    if abs(final_err) > tol_rad:
        print("  WARNING: heading drifted after the settle coast; consider a")
        print("  larger --align-tol or slower camera latency tuning.")
    return (px, py, ptheta), filt


def _straight_move_command(px, py, ptheta, target_x, target_y, coast_trigger_m,
                           move_speed_ms):
    """Wheel speeds for straight +X travel with small heading/lateral correction."""
    remaining_x = target_x - px
    lateral = target_y - py
    if remaining_x > 0.25:
        desired_heading = math.atan2(lateral, remaining_x)
    else:
        desired_heading = 0.0        # near the band, hold +X instead of turning
    herr = wrap_pi(desired_heading - ptheta)
    v = min(move_speed_ms, max(0.0, MOVE_V_GAIN * remaining_x))
    if 0.0 < v < MIN_WHEEL_SPEED_MS and remaining_x > coast_trigger_m:
        v = MIN_WHEEL_SPEED_MS
    omega = K_ALPHA * herr
    v_left = v - omega * TRACK_WIDTH_M / 2.0
    v_right = v + omega * TRACK_WIDTH_M / 2.0
    v_left, v_right = shape_wheel_speeds(
        v_left, v_right, MIN_WHEEL_SPEED_MS, move_speed_ms)
    return v_left, v_right, remaining_x, lateral, herr


def move_forward_distance(link, pose_source, filt, start_pose,
                          distance_m=DEFAULT_DISTANCE_M,
                          move_tol_m=DEFAULT_MOVE_TOL_M,
                          lateral_tol_m=DEFAULT_LATERAL_TOL_M,
                          timeout_s=DEFAULT_MOVE_TIMEOUT_S, keys=None,
                          move_speed_ms=DEFAULT_MOVE_SPEED_MS):
    """Move forward until the bot enters the target band around distance_m.

    The environment is noisy (dust, uneven mat, low-FPS camera, wheel slip), so
    this does NOT chase an exact Cartesian point. Success is declared when the
    filtered pose enters the acceptable range; correction stops there.
    """
    start_x, start_y, start_theta = start_pose
    target_x = start_x + distance_m
    target_y = start_y
    coast_trigger_m = min(MAX_COAST_TRIGGER_M, move_tol_m)
    print("\nSTEP 5/5 - MOVE FORWARD %.2f m" % distance_m)
    print("  Target band: displacement %.2f..%.2f m, lateral +/-%.2f m." % (
        distance_m - move_tol_m, distance_m + move_tol_m, lateral_tol_m))
    print("  Translation speed limit: %.2f m/s." % move_speed_ms)
    print("  The bot is successful when it ENTERS this band; it will not keep")
    print("  nudging toward an exact coordinate.")
    t0 = time.monotonic()
    t_prev = t0
    last_print = 0.0
    driving = True
    entered_band = False
    overshot = False
    px, py, ptheta = start_pose
    while True:
        _poll_abort(keys)
        now = time.monotonic()
        if now - t0 > timeout_s:
            coast(link, pose_source=pose_source)
            raise AbortRun("1 m move timed out after %.1f s" % timeout_s)
        dt = max(now - t_prev, 1e-3)
        t_prev = now
        try:
            raw = pose_source.read()
        except Exception:
            coast(link, pose_source=pose_source)
            raise
        px, py, ptheta = filt.update(*raw)
        displacement = px - start_x
        lateral_now = py - start_y
        remaining_x = target_x - px

        if in_target_band(displacement, lateral_now, distance_m,
                          move_tol_m, lateral_tol_m):
            entered_band = True
            print("  [move] target band entered: displacement=%.3f m lateral=%+.3f m" % (
                displacement, lateral_now))
            break
        if displacement > distance_m + move_tol_m:
            overshot = True
            print("  [move] passed beyond the target band: displacement=%.3f m" % displacement)
            break

        v_left, v_right, remaining_x, lateral, herr = _straight_move_command(
            px, py, ptheta, target_x, target_y, coast_trigger_m, move_speed_ms)
        if driving and remaining_x <= coast_trigger_m:
            # Near the near edge of the band: stop driving and let the bot coast
            # into the acceptable range instead of correcting to the exact point.
            driving = False
            coast(link, 0.30, pose_source)
            continue
        if not driving:
            # Coast fell short or lateral noise kept us outside the band: creep
            # at the executable minimum speed. Still range-based; no exact-point
            # hunting.
            v_left = MIN_WHEEL_SPEED_MS
            v_right = MIN_WHEEL_SPEED_MS
        link.send_command(ms_to_rpm(v_left), ms_to_rpm(v_right))
        pose_source.update(v_left, v_right, dt)
        if now - last_print >= 0.5:
            last_print = now
            print("  [move] pose=(%+.3f,%+.3f,%+.2f) disp=%.3f m lateral=%+.3f m" % (
                px, py, ptheta, displacement, lateral_now))
        sleep_s = CMD_PERIOD - (time.monotonic() - now)
        if sleep_s > 0.0:
            time.sleep(sleep_s)

    coast(link, 0.50, pose_source)
    try:
        raw = pose_source.read()
        fx, fy, ftheta = filt.update(*raw)
    except Exception:
        # Camera hiccup after the final coast: report the last filtered pose.
        fx, fy, ftheta = px, py, ptheta
    final_displacement = fx - start_x
    final_lateral = fy - start_y
    heading_err = wrap_pi(ftheta - start_theta)
    final_in_band = in_target_band(final_displacement, final_lateral,
                                   distance_m, move_tol_m, lateral_tol_m)
    ok = entered_band or final_in_band
    print("  MOVE STOPPED: entered_band=%s overshot=%s" % (
        "yes" if entered_band else "no", "yes" if overshot else "no"))
    print("  Final measured: displacement=%.3f m, lateral=%+.3f m, heading change=%+.2f rad" % (
        final_displacement, final_lateral, heading_err))
    print("  PASS CRITERIA: enter band %.2f..%.2f m with lateral +/-%.2f m -> %s" % (
        distance_m - move_tol_m, distance_m + move_tol_m, lateral_tol_m,
        "PASS" if ok else "FAIL"))
    return ok, (fx, fy, ftheta)


def run_real_workflow(args, keys):
    pose_source = calibrate_camera(args, keys)
    keys.close()  # restore canonical terminal mode before blocking input() prompts
    try:
        print("\nSTEP 2/5 - START THE BOT")
        print("  Place the bot ANYWHERE inside the arena, with ANY heading.")
        print("  Power it on / reset it so robot_firmware.py is running.")
        if args.ip:
            robot_ip = args.ip
            print("  Using robot IP from --ip: %s" % robot_ip)
        else:
            robot_ip = input("  Type the robot IP from its boot log, then press Enter: ").strip()
        if not robot_ip:
            raise AbortRun("no robot IP provided")
        if not args.yes:
            input("  Press Enter when the bot is on the arena floor and ready...")
        print("  Motion phase started: press q in THIS terminal at any time to abort + coast.")
        link = BotLink(robot_ip)
        motion_keys = TerminalKeys()
        try:
            _pose, filt = wait_for_robot_pose(
                pose_source, timeout_s=args.pose_timeout, camera_mode=True,
                keys=motion_keys)
            aligned_pose, filt = align_to_heading(
                link, pose_source, filt, target_theta=0.0,
                tol_rad=args.align_tol, timeout_s=args.align_timeout,
                keys=motion_keys,
                align_wheel_speed_ms=args.align_wheel_speed,
                stable_samples=args.align_stable)
            ok, _final = move_forward_distance(
                link, pose_source, filt, aligned_pose,
                distance_m=args.distance, move_tol_m=args.move_tol,
                lateral_tol_m=args.lateral_tol, timeout_s=args.move_timeout,
                keys=motion_keys, move_speed_ms=args.speed)
            return ok
        finally:
            motion_keys.close()
            coast(link, 0.5, pose_source)
            link.close()
    finally:
        pose_source.close()


def run_sim_workflow(args):
    print("\nSIM MODE - no camera, no UDP; validates the state machine and control logic.")
    start_x = 0.45
    start_y = -0.31
    start_theta = 2.35
    pose_source = SimulatedPoseSource(x=start_x, y=start_y, theta=start_theta,
                                      noise_std=args.noise)
    link = NullLink()
    print("  Simulated calibration: anchors locked automatically.")
    print("  Simulated random drop: x=%.3f y=%.3f theta=%.2f rad" % (
        start_x, start_y, start_theta))
    _pose, filt = wait_for_robot_pose(pose_source, timeout_s=5.0, camera_mode=False)
    aligned_pose, filt = align_to_heading(
        link, pose_source, filt, target_theta=0.0,
        tol_rad=args.align_tol, timeout_s=args.align_timeout,
        align_wheel_speed_ms=args.align_wheel_speed,
        stable_samples=args.align_stable)
    ok, final_pose = move_forward_distance(
        link, pose_source, filt, aligned_pose, distance_m=args.distance,
        move_tol_m=args.move_tol, lateral_tol_m=args.lateral_tol,
        timeout_s=args.move_timeout, move_speed_ms=args.speed)
    print("  Simulated final pose: x=%.3f y=%.3f theta=%.2f rad" % final_pose)
    return ok


def build_parser():
    parser = argparse.ArgumentParser(
        description="Lock arena homography with 'c', align bot to +X, move 1 m.")
    parser.add_argument("--ip", help="robot IP address (prompted after calibration if omitted)")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--width", type=int, default=2560,
                        help="camera width (try 1280 if the driver rejects 2K)")
    parser.add_argument("--height", type=int, default=1440,
                        help="camera height (try 720 if the driver rejects 2K)")
    parser.add_argument("--fps", type=int, default=0,
                        help="explicit camera FPS request (default 0 = do not set)")
    parser.add_argument("--no-mjpeg", action="store_true",
                        help="do not force MJPEG (use if the driver rejects MJPG)")
    parser.add_argument("--debug-view", action="store_true",
                        help="show the annotated camera window (terminal keys still work)")
    parser.add_argument("--focus", type=float, default=None,
                        help="manual focus value passed to the camera")
    parser.add_argument("--exposure", type=float, default=None,
                        help="manual exposure value passed to the camera")
    parser.add_argument("--latency-s", type=float, default=0.0,
                        help="explicit camera latency compensation in seconds (0=auto)")
    parser.add_argument("--max-hold", type=float, default=0.5,
                        help="robot-marker occlusion hold time in seconds")
    parser.add_argument("--min-anchor-area", type=float, default=10000.0,
                        help="minimum anchor quadrilateral area in px^2 to allow lock")
    parser.add_argument("--min-anchor-side", type=float, default=40.0,
                        help="minimum anchor quadrilateral side length in px")
    parser.add_argument("--pose-timeout", type=float, default=25.0)
    parser.add_argument("--align-tol", type=float, default=DEFAULT_ALIGN_TOL_RAD,
                        help="heading alignment tolerance in rad")
    parser.add_argument("--align-timeout", type=float, default=DEFAULT_ALIGN_TIMEOUT_S)
    parser.add_argument("--align-wheel-speed", type=float, default=ALIGN_WHEEL_SPEED_MS,
                        help="wheel speed used for short alignment pulses in m/s")
    parser.add_argument("--align-stable", type=int, default=ALIGN_STABLE_SAMPLES,
                        help="consecutive in-tolerance reads required before alignment is accepted")
    parser.add_argument("--speed", type=float, default=DEFAULT_MOVE_SPEED_MS,
                        help="translation speed limit for the 1 m move in m/s")
    parser.add_argument("--distance", type=float, default=DEFAULT_DISTANCE_M,
                        help="forward distance in meters (default 1.0)")
    parser.add_argument("--move-tol", type=float, default=DEFAULT_MOVE_TOL_M,
                        help="pass tolerance on final displacement in meters")
    parser.add_argument("--lateral-tol", type=float, default=DEFAULT_LATERAL_TOL_M,
                        help="pass tolerance on lateral drift in meters")
    parser.add_argument("--move-timeout", type=float, default=DEFAULT_MOVE_TIMEOUT_S)
    parser.add_argument("--yes", action="store_true",
                        help="skip the post-calibration placement confirmation prompt")
    parser.add_argument("--sim", action="store_true",
                        help="run without camera/robot using the simulated pose source")
    parser.add_argument("--noise", type=float, default=0.0,
                        help="simulated pose noise std-dev in meters (--sim only)")
    return parser


def main():
    args = build_parser().parse_args()
    if args.distance <= 0.0:
        print("--distance must be positive")
        return 2
    keys = TerminalKeys()
    try:
        if args.sim:
            ok = run_sim_workflow(args)
        else:
            ok = run_real_workflow(args, keys)
        print("\nWORKFLOW RESULT: %s" % ("PASS" if ok else "FAIL"))
        return 0 if ok else 1
    except AbortRun as exc:
        print("\nABORTED: %s" % exc)
        return 2
    except KeyboardInterrupt:
        print("\nABORTED: Ctrl-C")
        return 2
    except Exception as exc:
        print("\nERROR: %s" % exc)
        return 2
    finally:
        keys.close()


if __name__ == "__main__":
    sys.exit(main())
