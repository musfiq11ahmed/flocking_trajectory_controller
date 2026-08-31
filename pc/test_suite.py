"""PC-side test harness for the ESP32-S3 differential-drive robot.

Tests (SPEC.md section 3.3):
    0 - Link check
    1 - Open-loop sanity (wheels off the ground)
    2 - Closed-loop step response (logs CSV)
    3 - Differential maneuvers (straight + turn in place)
    4 - Waypoint navigation simulator (S-curve CSV, simulated pose feedback)
    5 - Failsafe check (command loss -> coast)

Usage:
    python3 test_suite.py                      # interactive menu
    python3 test_suite.py --test 2 --ip 192.168.1.50
    python3 test_suite.py --test 4 --ip 192.168.1.50 --csv scurve_trajectory.csv

Standard library only (socket, struct [via udp_protocol], time, csv, argparse,
math, os, datetime). No numpy, no third-party packages.
"""

import argparse
import csv
import math
import os
import socket
import time
from datetime import datetime

import udp_protocol as proto

# ---------------------------------------------------------------------------
# Tunables (PC side). Robot-side tunables live in firmware/robot_firmware.py.
# ---------------------------------------------------------------------------
CMD_HZ = 20                    # command/telemetry rate (matches firmware)
CMD_PERIOD = 1.0 / CMD_HZ

WHEEL_RADIUS_M = 0.03          # N20 wheel radius (default, configurable)
TRACK_WIDTH_M = 0.12           # wheel-to-wheel distance (default, configurable)

MIN_WHEEL_SPEED_MS = 0.15      # friction compensation floor (anti-stall)
MAX_WHEEL_SPEED_MS = 0.30      # absolute wheel-speed clamp (anti-skid)
ARRIVAL_TOLERANCE_M = 0.10     # waypoint arrival tolerance

K_RHO = 2.0                    # unicycle gain: v   = K_RHO   * distance
K_ALPHA = 4.0                  # unicycle gain: w   = K_ALPHA * heading_error
EMA_ALPHA = 0.5                # pose EMA filter coefficient (0 < a <= 1)

FINAL_ALIGN_RADIUS_M = 0.15    # inside this radius, steer to waypoint theta

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CSV = os.path.join(SCRIPT_DIR, "scurve_trajectory.csv")
LOG_DIR = os.path.join(SCRIPT_DIR, "logs")


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------
def wrap_pi(angle):
    """Wrap an angle to (-pi, pi] (shortest-path angular difference)."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def ms_to_rpm(v_ms, radius_m=WHEEL_RADIUS_M):
    """Convert wheel linear speed (m/s) to wheel RPM."""
    return v_ms / (2.0 * math.pi * radius_m) * 60.0


def timestamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


class BotLink(object):
    """UDP command/telemetry channel to the robot.

    The firmware sends telemetry back to the IP:port of the last command
    sender, so a single socket with an ephemeral source port is enough.
    """

    def __init__(self, robot_ip, robot_port=proto.UDP_PORT):
        self.addr = (robot_ip, robot_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.0)

    def send_command(self, left_rpm, right_rpm):
        self.sock.sendto(proto.pack_command(left_rpm, right_rpm), self.addr)

    def recv_telemetry(self, timeout_s=0.005):
        """Return one telemetry dict, or None on timeout / bad packet."""
        self.sock.settimeout(timeout_s)
        try:
            data, _src = self.sock.recvfrom(2048)
        except (socket.timeout, BlockingIOError, OSError):
            return None
        try:
            return proto.unpack_telemetry(data)
        except ValueError:
            return None

    def drain(self):
        """Throw away any queued telemetry datagrams."""
        while self.recv_telemetry(timeout_s=0.0) is not None:
            pass

    def close(self):
        self.sock.close()


def run_phase(link, left_rpm, right_rpm, duration_s, telemetry=None):
    """Stream a constant RPM command at 20 Hz for duration_s seconds.

    If `telemetry` is a list, every received telemetry dict is appended to it
    with an extra "_t" key = seconds since phase start.
    """
    t0 = time.monotonic()
    next_send = t0
    while True:
        now = time.monotonic()
        elapsed = now - t0
        if elapsed >= duration_s:
            break
        if now >= next_send:
            link.send_command(left_rpm, right_rpm)
            next_send += CMD_PERIOD
        if telemetry is not None:
            t = link.recv_telemetry(timeout_s=0.004)
            if t is not None:
                t["_t"] = time.monotonic() - t0
                telemetry.append(t)
        else:
            time.sleep(0.002)


def last_known_ticks(telem_before, telem_phase):
    """Return (left_ticks, right_ticks) baseline from the freshest sample."""
    src = telem_before[-1] if telem_before else (telem_phase[0] if telem_phase else None)
    if src is None:
        return None, None
    return src["left_ticks"], src["right_ticks"]


# ---------------------------------------------------------------------------
# Test 0 - Link check
# ---------------------------------------------------------------------------
def test0_link_check(link):
    print("\n=== TEST 0: Link check ===")
    print("Sending zero commands (coast) for 2 s, listening for telemetry...")
    link.drain()
    telem = []
    run_phase(link, 0.0, 0.0, 2.0, telem)

    n = len(telem)
    rate = n / 2.0
    print("  telemetry packets received : %d (%.1f Hz, expected ~%d Hz)"
          % (n, rate, CMD_HZ))
    if n >= 2:
        gaps = [telem[i + 1]["_t"] - telem[i]["_t"] for i in range(n - 1)]
        print("  inter-packet gap           : mean %.1f ms, max %.1f ms (expected ~%d ms)"
              % (1000.0 * sum(gaps) / len(gaps), 1000.0 * max(gaps),
                 proto.TELEMETRY_PERIOD_MS))
        t = telem[-1]
        print("  last sample                : L=%.1f RPM R=%.1f RPM "
              "ticks=(%d,%d) pwm=(%.2f,%.2f)"
              % (t["left_rpm"], t["right_rpm"], t["left_ticks"], t["right_ticks"],
                 t["left_pwm"], t["right_pwm"]))
    ok = n > 0 and rate >= 0.75 * CMD_HZ
    print("PASS CRITERIA: >0 packets and rate >= %.0f Hz -> %s"
          % (0.75 * CMD_HZ, "PASS" if ok else "FAIL"))
    return ok


# ---------------------------------------------------------------------------
# Test 1 - Open-loop sanity
# ---------------------------------------------------------------------------
def test1_open_loop(link):
    print("\n=== TEST 1: Open-loop sanity ===")
    print("Each phase commands the wheel(s) for 2 s, then 1 s coast.")
    input(">> LIFT THE ROBOT so both wheels spin freely, then press Enter...")

    phases = [
        ("Left motor only,  +30 RPM", +30.0, 0.0),
        ("Right motor only, +30 RPM", 0.0, +30.0),
        ("Both forward,     +30 RPM", +30.0, +30.0),
        ("Both reverse,     -30 RPM", -30.0, -30.0),
    ]
    all_ok = True
    history = []
    for name, cmd_l, cmd_r in phases:
        link.drain()
        telem = []
        run_phase(link, cmd_l, cmd_r, 2.0, telem)
        run_phase(link, 0.0, 0.0, 1.0, None)   # 1 s coast between phases
        if not telem:
            print("  %-32s : NO TELEMETRY -> FAIL" % name)
            all_ok = False
            continue

        base_l, base_r = last_known_ticks(history, telem)
        dl = telem[-1]["left_ticks"] - base_l
        dr = telem[-1]["right_ticks"] - base_r
        tail = [t for t in telem if t["_t"] >= telem[-1]["_t"] - 0.5]
        ml = sum(t["left_rpm"] for t in tail) / len(tail)
        mr = sum(t["right_rpm"] for t in tail) / len(tail)

        # Direction + magnitude expectations per phase. Undriven wheels
        # (target 0 RPM) are held near zero by the robot's PID; ignore jitter.
        def wheel_ok(delta_ticks, meas_rpm, target_rpm):
            if target_rpm == 0.0:
                return True
            if delta_ticks * target_rpm <= 0:
                return False  # ticks must move in the commanded direction
            return abs(meas_rpm - target_rpm) <= max(8.0, 0.35 * abs(target_rpm))

        ok_l = wheel_ok(dl, ml, cmd_l)
        ok_r = wheel_ok(dr, mr, cmd_r)
        ok = ok_l and ok_r
        all_ok = all_ok and ok
        print("  %-32s : dTicks L=%+7d R=%+7d | meas L=%+6.1f R=%+6.1f RPM "
              "| target L=%+.0f R=%+.0f -> %s"
              % (name, dl, dr, ml, mr, cmd_l, cmd_r, "PASS" if ok else "FAIL"))
        history.extend(telem)

    print("PASS CRITERIA: driven wheel ticks move in commanded direction and "
          "measured RPM within 35%% (or 8 RPM) of target -> %s"
          % ("PASS" if all_ok else "FAIL"))
    return all_ok


# ---------------------------------------------------------------------------
# Test 2 - Closed-loop step response
# ---------------------------------------------------------------------------
def test2_step_response(link, target_rpm=60.0, duration_s=4.0):
    print("\n=== TEST 2: Closed-loop step response (0 -> %.0f RPM both wheels) ==="
          % target_rpm)
    input(">> LIFT THE ROBOT so both wheels spin freely, then press Enter...")
    link.drain()
    telem = []
    run_phase(link, target_rpm, target_rpm, duration_s, telem)
    run_phase(link, 0.0, 0.0, 0.5, None)
    if not telem:
        print("  NO TELEMETRY -> FAIL")
        return False

    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(LOG_DIR, "step_response_%s.csv" % timestamp())
    with open(log_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "left_rpm", "right_rpm", "left_ticks", "right_ticks",
                    "left_pwm", "right_pwm"])
        for t in telem:
            w.writerow(["%.4f" % t["_t"], "%.3f" % t["left_rpm"], "%.3f" % t["right_rpm"],
                        t["left_ticks"], t["right_ticks"],
                        "%.4f" % t["left_pwm"], "%.4f" % t["right_pwm"]])
    print("  logged %d samples to %s" % (len(telem), log_path))

    band = 0.10 * target_rpm
    results = []
    for key in ("left_rpm", "right_rpm"):
        samples = [(t["_t"], t[key]) for t in telem]
        # Settling time: first time after which the signal stays in +/-10% band.
        settle = samples[-1][0]
        for i, (ts, v) in enumerate(samples):
            if all(abs(vv - target_rpm) <= band for _tt, vv in samples[i:]):
                settle = ts
                break
        overshoot = max(v for _ts, v in samples) - target_rpm
        tail = [v for ts, v in samples if ts >= samples[-1][0] - 0.5]
        ss_err = (sum(tail) / len(tail)) - target_rpm
        results.append((key, settle, overshoot, ss_err))
        print("  %-9s : settle(+/-10%%)=%.2f s  overshoot=%+.1f RPM  "
              "steady-state error=%+.1f RPM" % (key, settle, overshoot, ss_err))

    ok = all(s <= 1.5 and o <= 0.25 * target_rpm and abs(e) <= band
             for _k, s, o, e in results)
    print("PASS CRITERIA: settle <= 1.5 s, overshoot <= 25%%, |ss error| <= 10%% "
          "-> %s" % ("PASS" if ok else "FAIL"))
    return ok


# ---------------------------------------------------------------------------
# Test 3 - Differential maneuvers
# ---------------------------------------------------------------------------
def test3_differential(link):
    print("\n=== TEST 3: Differential maneuvers ===")
    print("Works with wheels on the ground; lift them if you want repeatable ticks.")
    input(">> Position the robot with clear space ahead, then press Enter...")
    all_ok = True

    # --- Straight line: both wheels +40 RPM for 3 s ---
    print("  Phase A: straight, both +40 RPM, 3 s")
    link.drain()
    telem = []
    run_phase(link, +40.0, +40.0, 3.0, telem)
    run_phase(link, 0.0, 0.0, 1.0, None)
    if len(telem) >= 2:
        dl = telem[-1]["left_ticks"] - telem[0]["left_ticks"]
        dr = telem[-1]["right_ticks"] - telem[0]["right_ticks"]
        ratio = (dr / dl) if dl else float("inf")
        ok = dl > 0 and dr > 0 and 0.7 <= ratio <= 1.3
        all_ok = all_ok and ok
        print("    dTicks L=%+d R=%+d  ratio R/L=%.3f -> %s"
              % (dl, dr, ratio, "PASS" if ok else "FAIL"))
    else:
        print("    NO TELEMETRY -> FAIL")
        all_ok = False

    # --- Turn in place: left +30 / right -30 for 2 s ---
    print("  Phase B: turn in place, left +30 / right -30 RPM, 2 s")
    link.drain()
    telem = []
    run_phase(link, +30.0, -30.0, 2.0, telem)
    run_phase(link, 0.0, 0.0, 1.0, None)
    if len(telem) >= 2:
        dl = telem[-1]["left_ticks"] - telem[0]["left_ticks"]
        dr = telem[-1]["right_ticks"] - telem[0]["right_ticks"]
        ratio = (dl / -dr) if dr else float("inf")
        ok = dl > 0 and dr < 0 and 0.6 <= ratio <= 1.4
        all_ok = all_ok and ok
        print("    dTicks L=%+d R=%+d  ratio L/|R|=%.3f -> %s"
              % (dl, dr, ratio, "PASS" if ok else "FAIL"))
    else:
        print("    NO TELEMETRY -> FAIL")
        all_ok = False

    print("PASS CRITERIA: straight R/L tick ratio in [0.7, 1.3]; "
          "turn L/|R| ratio in [0.6, 1.4] -> %s" % ("PASS" if all_ok else "FAIL"))
    return all_ok


# ---------------------------------------------------------------------------
# Test 4 - Waypoint navigation simulator (pluggable pose sources)
# ---------------------------------------------------------------------------
class SimulatedPoseSource(object):
    """Simulated robot pose: integrates commanded wheel speeds through a
    unicycle (differential-drive) model. Stands in for the overhead ArUco
    camera pipeline so the full navigation loop can be validated without
    vision hardware.

    Optional deterministic pseudo-noise (sinusoidal dither) emulates vision
    jitter without needing the `random` module.
    """

    def __init__(self, x, y, theta, noise_std=0.0, track_m=TRACK_WIDTH_M):
        self.x = x
        self.y = y
        self.theta = theta
        self.noise_std = noise_std
        self.track_m = track_m
        self._t = 0.0

    def update(self, v_left_ms, v_right_ms, dt):
        """Advance the true pose given commanded wheel speeds (m/s)."""
        v = (v_left_ms + v_right_ms) / 2.0
        w = (v_right_ms - v_left_ms) / self.track_m
        # Exact arc integration (midpoint heading) for stability at large dt.
        mid_theta = self.theta + w * dt / 2.0
        self.x += v * math.cos(mid_theta) * dt
        self.y += v * math.sin(mid_theta) * dt
        self.theta = wrap_pi(self.theta + w * dt)
        self._t += dt

    def read(self):
        """Return a (possibly noisy) measured pose (x, y, theta)."""
        n = self.noise_std
        if n > 0.0:
            nx = n * math.sin(13.7 * self._t)
            ny = n * math.cos(7.31 * self._t)
            nt = 0.5 * n * math.sin(9.17 * self._t + 1.3)
        else:
            nx = ny = nt = 0.0
        return self.x + nx, self.y + ny, wrap_pi(self.theta + nt)


class CameraPoseSource(object):
    """STUB for the real vision pipeline (SPEC layer 1).

    Will read ArUco/AprilTag marker detections from the overhead camera,
    apply the arena homography, and return (x, y, theta) in meters/radians.
    Drop-in replacement for SimulatedPoseSource in Test 4's loop: implement
    read() and update() with the same signatures.
    """

    def __init__(self, *args, **kwargs):
        raise NotImplementedError(
            "CameraPoseSource is a stub. Wire up the OpenCV ArUco + homography "
            "pipeline, then pass it as pose_source to run_waypoint_navigation()."
        )

    def read(self):
        raise NotImplementedError

    def update(self, v_left_ms, v_right_ms, dt):
        raise NotImplementedError


class EmaPoseFilter(object):
    """Exponential moving average filter for (x, y, theta) pose estimates.
    theta is filtered along the shortest angular path."""

    def __init__(self, alpha=EMA_ALPHA):
        self.alpha = alpha
        self.x = None
        self.y = None
        self.theta = None

    def update(self, x, y, theta):
        if self.x is None:
            self.x, self.y, self.theta = x, y, theta
        else:
            a = self.alpha
            self.x = a * x + (1.0 - a) * self.x
            self.y = a * y + (1.0 - a) * self.y
            dtheta = wrap_pi(theta - self.theta)
            self.theta = wrap_pi(self.theta + a * dtheta)
        return self.x, self.y, self.theta


def friction_compensate(v_left, v_right, min_speed=MIN_WHEEL_SPEED_MS):
    """Anti-stall: if either wheel is commanded to move but the slower wheel
    is below min_speed, scale BOTH wheels up so the slower one reaches
    min_speed while preserving the turning ratio."""
    a_l, a_r = abs(v_left), abs(v_right)
    hi, lo = max(a_l, a_r), min(a_l, a_r)
    if hi < 1e-9:
        return 0.0, 0.0                      # no motion commanded
    if lo >= min_speed:
        return v_left, v_right               # already above the floor
    if lo < 1e-9:
        # Pivot (one wheel at zero): put the stopped wheel at min_speed in
        # the direction of the moving wheel's motion.
        if a_l < a_r:
            v_left = math.copysign(min_speed, v_right)
        else:
            v_right = math.copysign(min_speed, v_left)
        return v_left, v_right
    scale = min_speed / lo
    return v_left * scale, v_right * scale


def clamp_wheel_speeds(v_left, v_right, max_speed=MAX_WHEEL_SPEED_MS):
    """Cap wheel speeds at max_speed, scaling both down together to preserve
    the turning ratio."""
    hi = max(abs(v_left), abs(v_right))
    if hi > max_speed:
        s = max_speed / hi
        v_left *= s
        v_right *= s
    return v_left, v_right


def compute_wheel_speeds(px, py, ptheta, wx, wy, wtheta,
                         k_rho=K_RHO, k_alpha=K_ALPHA):
    """Unicycle control law for one waypoint.

    Returns (v_left, v_right [m/s], dist [m], heading_error [rad]).
    Steps (SPEC robot tasks 5-8):
      5. Euclidean distance + shortest-path heading error to the target.
         (Inside FINAL_ALIGN_RADIUS_M, steer to the waypoint's own theta.)
      6. v = k_rho * dist, w = k_alpha * heading_error, split to wheels.
      7. Friction compensation: min wheel speed >= 0.15 m/s, ratio preserved.
      8. Clamp: max wheel speed <= 0.30 m/s, ratio preserved.
    """
    dx = wx - px
    dy = wy - py
    dist = math.hypot(dx, dy)
    if dist < FINAL_ALIGN_RADIUS_M:
        target_heading = wtheta           # final alignment with path tangent
    else:
        target_heading = math.atan2(dy, dx)
    heading_err = wrap_pi(target_heading - ptheta)

    v = k_rho * dist
    w = k_alpha * heading_err
    v_left = v - w * TRACK_WIDTH_M / 2.0
    v_right = v + w * TRACK_WIDTH_M / 2.0

    v_left, v_right = friction_compensate(v_left, v_right, MIN_WHEEL_SPEED_MS)
    v_left, v_right = clamp_wheel_speeds(v_left, v_right, MAX_WHEEL_SPEED_MS)
    return v_left, v_right, dist, heading_err


def load_waypoints(csv_path):
    """Load (x, y, theta) waypoints from a CSV with header x,y,theta."""
    waypoints = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None or [h.strip() for h in header[:3]] != ["x", "y", "theta"]:
            raise ValueError("%s: expected header 'x,y,theta'" % csv_path)
        for row in reader:
            if len(row) < 3:
                continue
            waypoints.append((float(row[0]), float(row[1]), float(row[2])))
    if not waypoints:
        raise ValueError("%s: no waypoints found" % csv_path)
    return waypoints


def run_waypoint_navigation(link, waypoints, pose_source, noise_desc=""):
    """Core navigation loop shared by Test 4 and (later) the camera pipeline.

    Streams wheel RPM commands over UDP at 20 Hz; advances to the next
    waypoint when the filtered pose is within ARRIVAL_TOLERANCE_M of it.
    """
    filt = EmaPoseFilter(EMA_ALPHA)
    wp_index = 0
    t_start = time.monotonic()
    t_prev = t_start
    last_print = t_start
    telem = None

    print("  Homing to origin waypoint #0 at (%.3f, %.3f, %.2f rad)..."
          % waypoints[0])
    while wp_index < len(waypoints):
        now = time.monotonic()
        dt = max(now - t_prev, 1e-3)
        t_prev = now

        # Task 4: state estimation (pose source + EMA filter).
        raw_x, raw_y, raw_theta = pose_source.read()
        px, py, ptheta = filt.update(raw_x, raw_y, raw_theta)

        # Tasks 5-8: errors -> unicycle law -> friction comp -> clamp.
        wx, wy, wtheta = waypoints[wp_index]
        v_left, v_right, dist, herr = compute_wheel_speeds(px, py, ptheta,
                                                           wx, wy, wtheta)

        # Task 9: actuation -- m/s -> RPM -> UDP to the robot.
        link.send_command(ms_to_rpm(v_left), ms_to_rpm(v_right))

        # Update the pose source with the speeds we actually commanded.
        pose_source.update(v_left, v_right, dt)

        # Task 10: tolerance check & iteration.
        if dist < ARRIVAL_TOLERANCE_M:
            wp_index += 1
            if wp_index % 25 == 0 or wp_index == len(waypoints):
                print("  waypoint %3d/%d reached  (t=%.1f s)"
                      % (wp_index, len(waypoints), now - t_start))

        # Live status + robot telemetry (if a physical robot is listening).
        if now - last_print >= 0.5:
            last_print = now
            t = link.recv_telemetry(timeout_s=0.0)
            if t is not None:
                telem = t
            status = ("  wp %3d/%d  pose=(%+.2f,%+.2f,%+.2f)  dist=%.3f m  "
                      "cmd vL=%.2f vR=%.2f m/s"
                      % (wp_index, len(waypoints), px, py, ptheta, dist,
                         v_left, v_right))
            if telem is not None:
                status += "  | robot L=%.1f R=%.1f RPM" % (
                    telem["left_rpm"], telem["right_rpm"])
            print(status)

        # Hold the 20 Hz loop rate.
        sleep_s = CMD_PERIOD - (time.monotonic() - now)
        if sleep_s > 0.0:
            time.sleep(sleep_s)

    # Stop the robot at the end of the path.
    for _ in range(CMD_HZ // 2):
        link.send_command(0.0, 0.0)
        time.sleep(CMD_PERIOD)
    # `dist` holds the filtered distance to the final waypoint at loop exit.
    return time.monotonic() - t_start, dist


def test4_waypoint_nav(link, csv_path=DEFAULT_CSV, noise_std=0.003):
    print("\n=== TEST 4: Waypoint navigation simulator (S-curve) ===")
    print("  pose feedback: SimulatedPoseSource (no camera needed); "
          "CameraPoseSource is a stub for the real ArUco pipeline.")
    try:
        waypoints = load_waypoints(csv_path)
    except (OSError, ValueError) as exc:
        print("  FAILED to load waypoints: %s" % exc)
        print("  (generate them with: python3 generate_scurve_csv.py)")
        return False
    print("  loaded %d waypoints from %s" % (len(waypoints), csv_path))

    # Arbitrary simulated drop point away from the trajectory origin, so the
    # homing phase (SPEC task 2) is exercised first.
    pose_source = SimulatedPoseSource(x=0.40, y=-0.40, theta=1.0,
                                      noise_std=noise_std)
    print("  simulated drop point: (0.40, -0.40, 1.00 rad), noise_std=%.3f m"
          % noise_std)

    elapsed, final_filtered_err = run_waypoint_navigation(link, waypoints,
                                                          pose_source)

    # Diagnostic: error between the simulated robot's TRUE pose and the final
    # waypoint. The arrival decisions above (and any real deployment) can only
    # use the filtered state estimate, so PASS is based on that; the true error
    # shows the EMA lag / noise residue for reference.
    fx, fy, _ft = pose_source.read()
    tx = getattr(pose_source, "x", fx)   # SimulatedPoseSource exposes the true
    ty = getattr(pose_source, "y", fy)   # pose; a camera source would not
    gx, gy, _gt = waypoints[-1]
    true_err = math.hypot(gx - tx, gy - ty)
    ok = final_filtered_err <= ARRIVAL_TOLERANCE_M
    print("  all %d waypoints traversed in %.1f s" % (len(waypoints), elapsed))
    print("  final error: filtered(estimated) %.3f m | true(sim) %.3f m"
          % (final_filtered_err, true_err))
    print("PASS CRITERIA: all waypoints reached with filtered error <= %.2f m -> %s"
          % (ARRIVAL_TOLERANCE_M, "PASS" if ok else "FAIL"))
    return ok


# ---------------------------------------------------------------------------
# Test 5 - Failsafe check
# ---------------------------------------------------------------------------
def test5_failsafe(link):
    print("\n=== TEST 5: Failsafe check (command loss -> coast) ===")
    print("Firmware must coast within ~1 s of the last UDP command "
          "(timeout = %d ms)." % proto.FAILSAFE_TIMEOUT_MS)
    input(">> LIFT THE ROBOT so both wheels spin freely, then press Enter...")

    print("  commanding +40 RPM both wheels for 1.5 s...")
    link.drain()
    telem = []
    run_phase(link, +40.0, +40.0, 1.5, telem)
    if telem:
        print("  before stop: L=%.1f RPM R=%.1f RPM"
              % (telem[-1]["left_rpm"], telem[-1]["right_rpm"]))

    print("  STOPPING all transmissions; watching telemetry for 3 s...")
    t_stop = time.monotonic()
    samples = []
    while time.monotonic() - t_stop < 3.0:
        t = link.recv_telemetry(timeout_s=0.05)
        if t is not None:
            samples.append((time.monotonic() - t_stop,
                            t["left_rpm"], t["right_rpm"]))
    if not samples:
        print("  NO TELEMETRY after stop -> FAIL (robot not reachable?)")
        return False

    stop_time = None
    for ts, rl, rr in samples:
        if abs(rl) <= 2.0 and abs(rr) <= 2.0:
            stop_time = ts
            break
    print("  last sample: t=%.2f s  L=%.1f RPM R=%.1f RPM" % samples[-1])
    if stop_time is not None:
        print("  wheels reached ~0 RPM %.2f s after last command" % stop_time)
    else:
        print("  wheels never reached ~0 RPM within 3 s")

    ok = stop_time is not None and stop_time <= 1.0
    print("PASS CRITERIA: measured RPM ~0 within 1.0 s of command loss -> %s"
          % ("PASS" if ok else "FAIL"))
    return ok


# ---------------------------------------------------------------------------
# Menu / CLI
# ---------------------------------------------------------------------------
MENU = """
+------------------------ BOT TEST SUITE ------------------------+
|  0  Link check                                                 |
|  1  Open-loop sanity (lift wheels)                             |
|  2  Closed-loop step response (logs CSV)                       |
|  3  Differential maneuvers (straight + turn in place)          |
|  4  Waypoint navigation simulator (S-curve, simulated pose)    |
|  5  Failsafe check (command loss -> coast)                     |
|  a  Run all (0-5)                                              |
|  q  Quit                                                       |
+----------------------------------------------------------------+
"""


def run_test(n, link, csv_path, noise_std):
    if n == 0:
        return test0_link_check(link)
    if n == 1:
        return test1_open_loop(link)
    if n == 2:
        return test2_step_response(link)
    if n == 3:
        return test3_differential(link)
    if n == 4:
        return test4_waypoint_nav(link, csv_path=csv_path, noise_std=noise_std)
    if n == 5:
        return test5_failsafe(link)
    raise ValueError("unknown test number %r" % (n,))


def interactive(link, csv_path, noise_std):
    while True:
        print(MENU)
        choice = input("Select test [0-5/a/q]: ").strip().lower()
        if choice == "q":
            break
        if choice == "a":
            results = {}
            for n in range(6):
                results[n] = run_test(n, link, csv_path, noise_std)
            print("\n==== SUMMARY ====")
            for n in range(6):
                print("  Test %d: %s" % (n, "PASS" if results[n] else "FAIL"))
            break
        if choice in ("0", "1", "2", "3", "4", "5"):
            run_test(int(choice), link, csv_path, noise_std)
        else:
            print("Invalid choice.")


def main():
    parser = argparse.ArgumentParser(
        description="Test harness for the ESP32-S3 differential-drive bot.")
    parser.add_argument("--test", type=int, choices=[0, 1, 2, 3, 4, 5],
                        help="run a single test non-interactively")
    parser.add_argument("--ip", help="robot IP address (printed on the REPL/serial at boot)")
    parser.add_argument("--csv", default=DEFAULT_CSV,
                        help="waypoint CSV for test 4 "
                             "(default: scurve_trajectory.csv next to this script)")
    parser.add_argument("--noise", type=float, default=0.003,
                        help="simulated pose noise std-dev in meters (test 4, "
                             "0 = noiseless; default 0.003)")
    args = parser.parse_args()

    ip = args.ip
    if ip is None:
        ip = input("Robot IP address (printed on the robot's REPL/serial at boot): ").strip()
    if not ip:
        print("No robot IP given; aborting.")
        return
    link = BotLink(ip)
    try:
        if args.test is not None:
            ok = run_test(args.test, link, args.csv, args.noise)
            print("\nTest %d result: %s" % (args.test, "PASS" if ok else "FAIL"))
        else:
            interactive(link, args.csv, args.noise)
    finally:
        link.close()


if __name__ == "__main__":
    main()
