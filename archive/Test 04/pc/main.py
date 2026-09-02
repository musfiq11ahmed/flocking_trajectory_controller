"""
main.py — Top-level application for the trajectory-following robot.

Usage
-----
    cd d:\\NIRO\\flocking test\\pc
    python main.py --port COM3 --trajectory ../trajectories/sample_trajectory.csv

Keyboard controls (OpenCV window must be focused):
    SPACE   Start / resume tracking
    E       Emergency stop (motors off)
    R       Reset (re-calibrate arena, restart trajectory)
    Q       Quit
"""

import argparse
import math
import sys
import time

import cv2

from config import (
    CONTROL_RATE_HZ, LOST_MARKER_TIMEOUT, SERIAL_PORT,
    ARENA_WIDTH_M, ARENA_HEIGHT_M,
)
from vision import ArenaLocalizer
from trajectory_controller import TrajectoryController
from serial_comm import RobotSerial
from utils import normalize_angle


# ─────────────────────────────────────────────────────────────
#  State machine
# ─────────────────────────────────────────────────────────────
STATE_CALIBRATING = "CALIBRATING"
STATE_READY       = "READY"
STATE_TRACKING    = "TRACKING"
STATE_FINISHED    = "FINISHED"
STATE_ESTOP       = "E-STOP"


def main():
    # ── CLI arguments ─────────────────────────────────────────
    ap = argparse.ArgumentParser(description="Trajectory-following robot controller")
    ap.add_argument("--port", default=SERIAL_PORT,
                    help="Serial port for ESP32 (e.g. COM3)")
    ap.add_argument("--trajectory", default="../trajectories/sample_trajectory.csv",
                    help="Path to trajectory CSV file")
    ap.add_argument("--no-serial", action="store_true",
                    help="Run without ESP32 (vision-only debug mode)")
    args = ap.parse_args()

    # ── Initialise subsystems ─────────────────────────────────
    localizer  = ArenaLocalizer()
    controller = TrajectoryController()
    serial_link: RobotSerial | None = None

    # Load trajectory
    try:
        controller.load_csv(args.trajectory)
    except Exception as e:
        print(f"[MAIN] Failed to load trajectory: {e}")
        sys.exit(1)

    # Open camera
    cap = localizer.open_camera()
    print(f"[MAIN] Camera opened  "
          f"({int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}×"
          f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))})")

    # Connect to ESP32 (optional)
    if not args.no_serial:
        serial_link = RobotSerial(port=args.port)
        if not serial_link.connect(timeout=8):
            print("[MAIN] Continuing without serial (vision-only mode).")
            serial_link = None

    # ── State ─────────────────────────────────────────────────
    state = STATE_CALIBRATING
    robot_pose = None
    target_idx = None
    lookahead_wp = None
    cmd_vl = cmd_vr = 0.0
    loop_dt = 1.0 / CONTROL_RATE_HZ

    print(f"\n[MAIN] Arena: {ARENA_WIDTH_M:.2f}m × {ARENA_HEIGHT_M:.2f}m")
    print("[MAIN] Place all 4 arena markers in view to calibrate …\n")

    try:
        while True:
            t_start = time.perf_counter()

            # ── 1. Capture frame ──────────────────────────────
            ok, frame = cap.read()
            if not ok:
                print("[MAIN] Camera read failed.")
                break

            # ── 2. State machine ──────────────────────────────
            if state == STATE_CALIBRATING:
                if localizer.calibrate(frame):
                    print("[MAIN] Arena calibrated ✓   Press SPACE to start.")
                    state = STATE_READY

            elif state == STATE_READY:
                robot_pose = localizer.detect_robot(frame)

            elif state == STATE_TRACKING:
                robot_pose = localizer.detect_robot(frame)

                if robot_pose is None or \
                   localizer.time_since_detection() > LOST_MARKER_TIMEOUT:
                    # Lost the robot — send stop
                    cmd_vl = cmd_vr = 0.0
                    if serial_link:
                        serial_link.send_command(0, 0)
                else:
                    rx, ry, rt = robot_pose
                    cmd_vl, cmd_vr, target_idx, lookahead_wp = \
                        controller.compute(rx, ry, rt)

                    if serial_link:
                        serial_link.send_command(cmd_vl, cmd_vr)

                    if controller.is_finished:
                        state = STATE_FINISHED
                        if serial_link:
                            serial_link.send_command(0, 0)
                        print("[MAIN] Trajectory finished ✓")

            elif state == STATE_FINISHED:
                robot_pose = localizer.detect_robot(frame)

            elif state == STATE_ESTOP:
                pass   # motors already stopped

            # ── 3. Read ESP32 feedback (non-blocking) ─────────
            if serial_link:
                fb = serial_link.read_feedback()
                # (currently logged only — could be used for diagnostics)

            # ── 4. Build status text ──────────────────────────
            status_lines = [f"State: {state}"]
            if robot_pose:
                rx, ry, rt = robot_pose
                status_lines.append(
                    f"Robot: ({rx:+.3f}, {ry:+.3f})  "
                    f"theta={math.degrees(rt):.1f} deg")
            if state == STATE_TRACKING:
                status_lines.append(
                    f"Cmd: L={cmd_vl:+.0f}  R={cmd_vr:+.0f} ticks/s")
                status_lines.append(
                    f"Waypoint: {controller.current_idx}/{len(controller.path)-1}")
            status_text = "\n".join(status_lines)

            # ── 5. Draw overlay + show ────────────────────────
            vis = localizer.draw_overlay(
                frame,
                robot_pose=robot_pose,
                trajectory=controller.path if controller.path else None,
                target_idx=target_idx,
                lookahead_pt=lookahead_wp,
                state_text=status_text,
            )
            cv2.imshow("Trajectory Tracker", vis)

            # ── 6. Keyboard input ─────────────────────────────
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord(" "):          # SPACE — start / resume
                if state in (STATE_READY, STATE_ESTOP, STATE_FINISHED):
                    if state == STATE_FINISHED:
                        controller.reset()
                    state = STATE_TRACKING
                    print("[MAIN] TRACKING started.")
            elif key == ord("e"):          # E-STOP
                state = STATE_ESTOP
                cmd_vl = cmd_vr = 0.0
                if serial_link:
                    serial_link.send_command(0, 0)
                print("[MAIN] *** E-STOP ***")
            elif key == ord("r"):          # RESET
                state = STATE_CALIBRATING
                localizer.homography = None
                localizer.arena_calibrated = False
                localizer._robot_x = None
                controller.reset()
                cmd_vl = cmd_vr = 0.0
                if serial_link:
                    serial_link.send_command(0, 0)
                print("[MAIN] RESET — re-calibrating …")

            # ── 7. Rate-limit ─────────────────────────────────
            elapsed = time.perf_counter() - t_start
            sleep_s = loop_dt - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)

    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted.")
    finally:
        if serial_link:
            serial_link.send_command(0, 0)
            serial_link.disconnect()
        cap.release()
        cv2.destroyAllWindows()
        print("[MAIN] Shutdown complete.")


if __name__ == "__main__":
    main()
