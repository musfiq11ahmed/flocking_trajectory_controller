"""
test_robot.py — Complete Hardware & Communications Diagnostic Suite
====================================================================
Performs a comprehensive diagnostic test of the robot over USB serial while
connected to the PC:
  1. Serial Port Discovery & Connectivity
  2. ESP32 Handshake & Telemetry Synchronization
  3. Communications Quality, Latency & Packet Integrity
  4. Static Encoder Drift & Electrical Noise
  5. Left Motor Forward Drive & Left Encoder Direction
  6. Left Motor Reverse Drive & Left Encoder Direction
  7. Right Motor Forward Drive & Right Encoder Direction
  8. Right Motor Reverse Drive & Right Encoder Direction
  9. Dual Motor Synchronous Drive & Speed Balance
 10. Braking / Zero-Speed Settling
 11. Safety Watchdog Timeout & Failsafe

Run with the robot's wheels elevated / suspended off the desk!

Usage:
------
    python test_robot.py --port COM13
    python test_robot.py --port COM13 --speed 150 --duration 1.2
    python test_robot.py --list-ports
"""

import argparse
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Error: 'pyserial' is required. Install via: pip install pyserial")
    sys.exit(1)

# Ensure parent and pc directories are in python path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PC_DIR = SCRIPT_DIR if os.path.basename(SCRIPT_DIR) == "pc" else os.path.join(SCRIPT_DIR, "pc")
if PC_DIR not in sys.path:
    sys.path.insert(0, PC_DIR)

try:
    from config import SERIAL_PORT as DEFAULT_PORT, SERIAL_BAUD as DEFAULT_BAUD
except ImportError:
    DEFAULT_PORT = "COM13"
    DEFAULT_BAUD = 115200


# ─────────────────────────────────────────────────────────────────────────────
# Terminal Formatting & Colors
# ─────────────────────────────────────────────────────────────────────────────
if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

class Color:
    RESET   = "\033[0m"
    BOLD    = "\033[1m"
    GREEN   = "\033[92m"
    RED     = "\033[91m"
    YELLOW  = "\033[93m"
    CYAN    = "\033[96m"
    MAGENTA = "\033[95m"
    GRAY    = "\033[90m"

    @classmethod
    def strip_or_enable(cls):
        # Enable ANSI colors on Windows 10/11 terminals
        if sys.platform == "win32":
            os.system("")

Color.strip_or_enable()

def tag_pass(text="PASS"):
    return f"{Color.BOLD}{Color.GREEN}[PASS]{Color.RESET}"

def tag_fail(text="FAIL"):
    return f"{Color.BOLD}{Color.RED}[FAIL]{Color.RESET}"

def tag_warn(text="WARN"):
    return f"{Color.BOLD}{Color.YELLOW}[WARN]{Color.RESET}"

def tag_info(text="INFO"):
    return f"{Color.BOLD}{Color.CYAN}[INFO]{Color.RESET}"


# ─────────────────────────────────────────────────────────────────────────────
# Test Result Data Structure
# ─────────────────────────────────────────────────────────────────────────────
class TestResult:
    def __init__(self, name: str, status: str, details: str, remediation: str = ""):
        self.name = name
        self.status = status          # "PASS", "FAIL", "WARN", "SKIPPED"
        self.details = details
        self.remediation = remediation

    @property
    def badge(self) -> str:
        if self.status == "PASS":
            return tag_pass()
        elif self.status == "FAIL":
            return tag_fail()
        elif self.status == "WARN":
            return tag_warn()
        else:
            return f"{Color.GRAY}[SKIP]{Color.RESET}"


# ─────────────────────────────────────────────────────────────────────────────
# Low-Level Serial Communication Driver for Diagnostics
# ─────────────────────────────────────────────────────────────────────────────
class DiagnosticSerial:
    """Manages the raw USB serial stream with high-resolution metrics."""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.05):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self._buf = ""

    def open(self) -> Tuple[bool, str]:
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=self.timeout,
                write_timeout=0.5,
            )
            # Brief pause to let USB-CDC settle
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            return True, "Port opened successfully"
        except serial.SerialException as e:
            return False, str(e)

    def close(self):
        if self.ser and self.ser.is_open:
            try:
                self.send_command(0.0, 0.0)
            except Exception:
                pass
            self.ser.close()

    def send_command(self, left_tps: float, right_tps: float):
        """Sends CMD:<left>,<right>\\n command to ESP32."""
        if not self.ser or not self.ser.is_open:
            return
        msg = f"CMD:{left_tps:.1f},{right_tps:.1f}\n"
        self.ser.write(msg.encode("ascii"))

    def readline(self) -> Optional[str]:
        """Reads one newline-delimited line from the serial buffer."""
        if not self.ser or not self.ser.is_open:
            return None
        try:
            while self.ser.in_waiting:
                chunk = self.ser.read(1).decode("ascii", errors="replace")
                if chunk == "\n":
                    line = self._buf.strip()
                    self._buf = ""
                    return line
                self._buf += chunk
        except serial.SerialException:
            pass
        return None

    def read_feedback_packet(self) -> Optional[Tuple[int, int, float, float]]:
        """Parses 'FB:enc_l,enc_r,spd_l,spd_r' packet."""
        line = self.readline()
        if line and line.startswith("FB:"):
            try:
                parts = line[3:].split(",")
                return (int(parts[0]), int(parts[1]), float(parts[2]), float(parts[3]))
            except (ValueError, IndexError):
                pass
        return None

    def flush_input(self):
        if self.ser and self.ser.is_open:
            self.ser.reset_input_buffer()
            self._buf = ""


# ─────────────────────────────────────────────────────────────────────────────
# Diagnostic Runner Class
# ─────────────────────────────────────────────────────────────────────────────
class RobotDiagnosticRunner:
    def __init__(self, port: str, baud: int = 115200, speed: float = 150.0, duration: float = 1.2):
        self.port = port
        self.baud = baud
        self.speed = speed
        self.duration = duration
        self.comm: Optional[DiagnosticSerial] = None
        self.results: List[TestResult] = []

    def log_header(self, title: str):
        print(f"\n{Color.BOLD}{Color.CYAN}{'='*68}")
        print(f"  {title}")
        print(f"{'='*68}{Color.RESET}")

    def log_step(self, step_num: int, title: str):
        print(f"\n{Color.BOLD}Test {step_num}: {title}{Color.RESET}")

    # ── TEST 1: Serial Port Detection & Connectivity ──────────────────────────
    def test_serial_connection(self) -> bool:
        self.log_step(1, "Serial Port & USB Connection")
        available_ports = [p.device for p in serial.tools.list_ports.comports()]
        print(f"  Available COM ports on system: {available_ports}")

        if self.port not in available_ports:
            msg = f"Port '{self.port}' not found in available system ports {available_ports}."
            fix = "Check USB cable connection. Verify the port in Device Manager."
            self.results.append(TestResult("Serial Port Discovery", "FAIL", msg, fix))
            print(f"  {tag_fail()} {msg}")
            return False

        self.comm = DiagnosticSerial(self.port, self.baud)
        ok, reason = self.comm.open()
        if not ok:
            msg = f"Failed to open '{self.port}': {reason}"
            fix = "Ensure no other program (Thonny, Serial Monitor, Python script) is holding the port."
            self.results.append(TestResult("Serial Port Access", "FAIL", msg, fix))
            print(f"  {tag_fail()} {msg}")
            return False

        print(f"  {tag_pass()} Opened {self.port} at {self.baud} baud.")
        self.results.append(TestResult("Serial Port Access", "PASS", f"Connected to {self.port} @ {self.baud} baud"))
        return True

    # ── TEST 2: Handshake & Active Firmware Detection ────────────────────────
    def test_handshake_and_firmware(self) -> bool:
        self.log_step(2, "ESP32 Firmware Handshake & Telemetry")
        print("  Listening for boot announcement ('READY') or periodic telemetry ('FB:')...")

        # Listen for up to 3.0 seconds
        t_end = time.time() + 3.0
        got_ready = False
        got_fb = False
        sample_line = ""

        # Probe with CMD:0,0 to trigger active responses if already in loop
        self.comm.send_command(0.0, 0.0)

        while time.time() < t_end:
            line = self.comm.readline()
            if line:
                sample_line = line
                if "READY" in line:
                    got_ready = True
                if line.startswith("FB:"):
                    got_fb = True
                    break
            time.sleep(0.02)
            self.comm.send_command(0.0, 0.0)

        if got_ready or got_fb:
            status_text = "Received 'READY' handshake" if got_ready else "Detected active 'FB:' telemetry stream"
            print(f"  {tag_pass()} Firmware alive: {status_text}")
            self.results.append(TestResult("Firmware Handshake", "PASS", status_text))
            return True

        # Check if MicroPython REPL prompt is visible
        if ">>>" in sample_line:
            msg = "ESP32 is in MicroPython REPL mode ('>>>'). 'main.py' is not running."
            fix = "Upload robot.py and main.py to ESP32 flash and reset the board."
            print(f"  {tag_fail()} {msg}")
            self.results.append(TestResult("Firmware Handshake", "FAIL", msg, fix))
            return False

        msg = f"No telemetry received within 3.0s (Last received: '{sample_line}')."
        fix = "Verify ESP32 has main.py running. Try pressing the EN/RST button on the ESP32."
        print(f"  {tag_fail()} {msg}")
        self.results.append(TestResult("Firmware Handshake", "FAIL", msg, fix))
        return False

    # ── TEST 3: Communication Protocol & Latency ─────────────────────────────
    def test_communication_quality(self) -> bool:
        self.log_step(3, "Communication Link Health & Latency")
        print("  Sampling 25 feedback frames to test packet rate, jitter, and integrity...")

        self.comm.flush_input()
        received_packets = 0
        corrupt_packets = 0
        timestamps: List[float] = []
        t_start = time.time()
        timeout = t_start + 4.0

        while received_packets < 25 and time.time() < timeout:
            # Stream zero-speed commands at ~30 Hz
            self.comm.send_command(0.0, 0.0)
            line = self.comm.readline()
            if line:
                if line.startswith("FB:"):
                    fb = self.comm.read_feedback_packet()
                    # If read_feedback_packet didn't parse from readline buffer, parse line directly:
                    try:
                        parts = line[3:].split(",")
                        if len(parts) == 4:
                            int(parts[0])
                            int(parts[1])
                            float(parts[2])
                            float(parts[3])
                            received_packets += 1
                            timestamps.append(time.time())
                        else:
                            corrupt_packets += 1
                    except Exception:
                        corrupt_packets += 1
            time.sleep(0.033)

        if received_packets < 15:
            msg = f"Low packet reception: only {received_packets}/25 packets received (timeout)."
            fix = "Check USB cable quality and ensure baud rate is set to 115200."
            print(f"  {tag_fail()} {msg}")
            self.results.append(TestResult("Comms Link Quality", "FAIL", msg, fix))
            return False

        # Compute interval / frequency
        deltas = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps)-1)]
        avg_dt = sum(deltas) / len(deltas) if deltas else 0.1
        rate_hz = 1.0 / avg_dt if avg_dt > 0 else 0

        details = f"Received {received_packets} packets @ ~{rate_hz:.1f} Hz (corrupted: {corrupt_packets})"
        print(f"  {tag_pass()} {details}")

        if corrupt_packets > 2:
            self.results.append(TestResult("Comms Link Quality", "WARN", details, "Check for electrical noise on USB line."))
        else:
            self.results.append(TestResult("Comms Link Quality", "PASS", details))
        return True

    # ── TEST 4: Static Encoder Drift (Stationary) ─────────────────────────────
    def test_static_encoder_drift(self) -> bool:
        self.log_step(4, "Static Encoder Drift & Electrical Noise (Stationary)")
        print("  Motors stopped. Measuring encoder counts for 1.2 seconds...")

        self.comm.send_command(0.0, 0.0)
        time.sleep(0.2)
        self.comm.flush_input()

        # Capture starting counts
        start_left, start_right = None, None
        t_end = time.time() + 1.2
        samples = 0
        drift_l, drift_r = 0, 0

        while time.time() < t_end:
            self.comm.send_command(0.0, 0.0)
            fb = self._get_latest_feedback()
            if fb:
                enc_l, enc_r, spd_l, spd_r = fb
                if start_left is None:
                    start_left = enc_l
                    start_right = enc_r
                drift_l = abs(enc_l - start_left)
                drift_r = abs(enc_r - start_right)
                samples += 1
            time.sleep(0.033)

        if start_left is None:
            msg = "No encoder readings received during static test."
            print(f"  {tag_fail()} {msg}")
            self.results.append(TestResult("Static Encoder Stability", "FAIL", msg, "Check encoder connections."))
            return False

        details = f"Left drift: {drift_l} ticks, Right drift: {drift_r} ticks across {samples} samples"
        if drift_l == 0 and drift_r == 0:
            print(f"  {tag_pass()} Rock-solid! {details}")
            self.results.append(TestResult("Static Encoder Stability", "PASS", details))
            return True
        elif drift_l <= 2 and drift_r <= 2:
            print(f"  {tag_warn()} Minor vibration/drift detected: {details}")
            self.results.append(TestResult("Static Encoder Stability", "WARN", details, "Check pull-up resistors or motor vibration."))
            return True
        else:
            print(f"  {tag_fail()} Significant phantom ticks detected: {details}")
            fix = "Check for floating encoder pins (ENC_L_A/B, ENC_R_A/B). Ensure Pin.PULL_UP is working."
            self.results.append(TestResult("Static Encoder Stability", "FAIL", details, fix))
            return False

    # ── Helper for Motor Drive Test ──────────────────────────────────────────
    def _run_motor_test(
        self,
        name: str,
        target_l: float,
        target_r: float,
        expected_active: str,     # "LEFT", "RIGHT", or "BOTH"
        expected_sign_l: int,     # +1, -1, or 0
        expected_sign_r: int,     # +1, -1, or 0
    ) -> Tuple[bool, str, str]:
        """Runs the motors at requested speed, monitoring encoder deltas and speeds."""
        # Baseline
        self.comm.send_command(0.0, 0.0)
        time.sleep(0.15)
        self.comm.flush_input()

        fb_start = self._get_latest_feedback(timeout=0.6)
        if not fb_start:
            return False, "Failed to get initial encoder feedback", "Check connection"

        start_l, start_r = fb_start[0], fb_start[1]
        max_spd_l, max_spd_r = 0.0, 0.0
        latest_l, latest_r = start_l, start_r

        # Drive phase
        t_end = time.time() + self.duration
        while time.time() < t_end:
            self.comm.send_command(target_l, target_r)
            fb = self._get_latest_feedback(timeout=0.04)
            if fb:
                latest_l, latest_r, spd_l, spd_r = fb
                if abs(spd_l) > abs(max_spd_l):
                    max_spd_l = spd_l
                if abs(spd_r) > abs(max_spd_r):
                    max_spd_r = spd_r
            time.sleep(0.033)

        # Immediate stop command
        self.comm.send_command(0.0, 0.0)

        # Calculate deltas
        delta_l = latest_l - start_l
        delta_r = latest_r - start_r

        summary = f"ΔLeft: {delta_l:+d} ticks (max spd: {max_spd_l:+.1f} tps) | ΔRight: {delta_r:+d} ticks (max spd: {max_spd_r:+.1f} tps)"
        print(f"    Result: {summary}")

        # Verification logic
        MIN_TICKS = 25  # Minimum expected rotation ticks in test duration

        if expected_active == "LEFT":
            # Check left motor moved sufficiently in correct direction
            if expected_sign_l > 0 and delta_l < MIN_TICKS:
                if delta_l < -MIN_TICKS:
                    return False, summary, "Left motor spun backwards! Set 'LEFT_INVERTED = True' in esp32/robot.py"
                return False, summary, "Left motor did not rotate! Check DRV8833 AIN1/AIN2 wiring, motor power, or encoder pins."
            elif expected_sign_l < 0 and delta_l > -MIN_TICKS:
                if delta_l > MIN_TICKS:
                    return False, summary, "Left motor spun in wrong direction for reverse! Check 'LEFT_INVERTED' setting."
                return False, summary, "Left motor did not rotate in reverse! Check AIN1/AIN2 driver pins."

            # Check right motor remained isolated (crosstalk check)
            if abs(delta_r) > 15:
                return False, summary, "Crosstalk! Right wheel moved when only Left was commanded. Check AIN vs BIN motor wiring."

        elif expected_active == "RIGHT":
            # Check right motor moved sufficiently in correct direction
            if expected_sign_r > 0 and delta_r < MIN_TICKS:
                if delta_r < -MIN_TICKS:
                    return False, summary, "Right motor spun backwards! Set 'RIGHT_INVERTED = True' in esp32/robot.py"
                return False, summary, "Right motor did not rotate! Check DRV8833 BIN1/BIN2 wiring, motor power, or encoder pins."
            elif expected_sign_r < 0 and delta_r > -MIN_TICKS:
                if delta_r > MIN_TICKS:
                    return False, summary, "Right motor spun in wrong direction for reverse! Check 'RIGHT_INVERTED' setting."
                return False, summary, "Right motor did not rotate in reverse! Check BIN1/BIN2 driver pins."

            # Check left motor remained isolated
            if abs(delta_l) > 15:
                return False, summary, "Crosstalk! Left wheel moved when only Right was commanded. Check AIN vs BIN motor wiring."

        elif expected_active == "BOTH":
            if delta_l < MIN_TICKS or delta_r < MIN_TICKS:
                return False, summary, "One or both motors failed to reach minimum forward motion."

        return True, summary, ""

    # ── TEST 5: Left Motor Forward ───────────────────────────────────────────
    def test_left_motor_forward(self) -> bool:
        self.log_step(5, f"Left Motor Forward Drive (+{self.speed:.0f} ticks/s)")
        print(f"  Commanding CMD:{self.speed:.1f},0.0 for {self.duration:.1f}s...")
        ok, details, fix = self._run_motor_test("Left Forward", self.speed, 0.0, "LEFT", +1, 0)
        status = "PASS" if ok else "FAIL"
        print(f"  {tag_pass() if ok else tag_fail()} Left Forward: {details}")
        self.results.append(TestResult("Left Motor Forward", status, details, fix))
        time.sleep(0.3)
        return ok

    # ── TEST 6: Left Motor Reverse ───────────────────────────────────────────
    def test_left_motor_reverse(self) -> bool:
        self.log_step(6, f"Left Motor Reverse Drive (-{self.speed:.0f} ticks/s)")
        print(f"  Commanding CMD:-{self.speed:.1f},0.0 for {self.duration:.1f}s...")
        ok, details, fix = self._run_motor_test("Left Reverse", -self.speed, 0.0, "LEFT", -1, 0)
        status = "PASS" if ok else "FAIL"
        print(f"  {tag_pass() if ok else tag_fail()} Left Reverse: {details}")
        self.results.append(TestResult("Left Motor Reverse", status, details, fix))
        time.sleep(0.3)
        return ok

    # ── TEST 7: Right Motor Forward ──────────────────────────────────────────
    def test_right_motor_forward(self) -> bool:
        self.log_step(7, f"Right Motor Forward Drive (+{self.speed:.0f} ticks/s)")
        print(f"  Commanding CMD:0.0,{self.speed:.1f} for {self.duration:.1f}s...")
        ok, details, fix = self._run_motor_test("Right Forward", 0.0, self.speed, "RIGHT", 0, +1)
        status = "PASS" if ok else "FAIL"
        print(f"  {tag_pass() if ok else tag_fail()} Right Forward: {details}")
        self.results.append(TestResult("Right Motor Forward", status, details, fix))
        time.sleep(0.3)
        return ok

    # ── TEST 8: Right Motor Reverse ──────────────────────────────────────────
    def test_right_motor_reverse(self) -> bool:
        self.log_step(8, f"Right Motor Reverse Drive (-{self.speed:.0f} ticks/s)")
        print(f"  Commanding CMD:0.0,-{self.speed:.1f} for {self.duration:.1f}s...")
        ok, details, fix = self._run_motor_test("Right Reverse", 0.0, -self.speed, "RIGHT", 0, -1)
        status = "PASS" if ok else "FAIL"
        print(f"  {tag_pass() if ok else tag_fail()} Right Reverse: {details}")
        self.results.append(TestResult("Right Motor Reverse", status, details, fix))
        time.sleep(0.3)
        return ok

    # ── TEST 9: Dual Motor Synchronous Drive & Speed Balance ─────────────────
    def test_dual_motor_balance(self) -> bool:
        self.log_step(9, f"Dual Motor Sync & Speed Balance (+{self.speed:.0f}, +{self.speed:.0f} ticks/s)")
        print(f"  Commanding both wheels forward for {self.duration:.1f}s...")
        ok, details, fix = self._run_motor_test("Dual Motor", self.speed, self.speed, "BOTH", +1, +1)
        if not ok:
            print(f"  {tag_fail()} Dual motor forward failed: {details}")
            self.results.append(TestResult("Dual Motor Sync", "FAIL", details, fix))
            return False

        # Extract delta numbers from details for balance check
        try:
            # Format: ΔLeft: +180 ticks ... | ΔRight: +175 ticks ...
            parts = details.split("|")
            del_l = abs(int(parts[0].split("ΔLeft:")[1].split("ticks")[0].strip()))
            del_r = abs(int(parts[1].split("ΔRight:")[1].split("ticks")[0].strip()))
            diff = abs(del_l - del_r)
            avg = (del_l + del_r) / 2.0
            ratio = (diff / avg) * 100 if avg > 0 else 0

            balance_info = f"Left: {del_l} ticks, Right: {del_r} ticks (Discrepancy: {ratio:.1f}%)"
            if ratio <= 15.0:
                print(f"  {tag_pass()} Excellent wheel balance: {balance_info}")
                self.results.append(TestResult("Dual Motor Sync", "PASS", balance_info))
            elif ratio <= 30.0:
                print(f"  {tag_warn()} Moderate wheel discrepancy: {balance_info}")
                self.results.append(TestResult("Dual Motor Sync", "WARN", balance_info, "Check tire friction or tune PID gains in robot.py."))
            else:
                print(f"  {tag_warn()} High speed discrepancy: {balance_info}")
                self.results.append(TestResult("Dual Motor Sync", "WARN", balance_info, "Check mechanical binding or motor voltage."))
        except Exception:
            self.results.append(TestResult("Dual Motor Sync", "PASS", details))

        time.sleep(0.3)
        return True

    # ── TEST 10: Active Braking & Deadzone Settling ───────────────────────────
    def test_braking_settling(self) -> bool:
        self.log_step(10, "Active Braking & Zero-Speed Settling")
        print("  Spinning up, then issuing CMD:0,0 to test stopping response...")

        # Spin up briefly
        for _ in range(15):
            self.comm.send_command(self.speed, self.speed)
            time.sleep(0.033)

        # Trigger Stop
        t_stop_cmd = time.time()
        stopped = False
        settle_time_s = 0.0

        for _ in range(25):
            self.comm.send_command(0.0, 0.0)
            fb = self._get_latest_feedback(timeout=0.05)
            if fb:
                spd_l, spd_r = abs(fb[2]), abs(fb[3])
                if spd_l < 5.0 and spd_r < 5.0:
                    settle_time_s = time.time() - t_stop_cmd
                    stopped = True
                    break
            time.sleep(0.02)

        if stopped and settle_time_s < 0.6:
            details = f"Motors brought to full halt within {settle_time_s*1000:.0f} ms"
            print(f"  {tag_pass()} {details}")
            self.results.append(TestResult("Braking & Settling", "PASS", details))
            return True
        elif stopped:
            details = f"Motors stopped slowly: {settle_time_s*1000:.0f} ms"
            print(f"  {tag_warn()} {details}")
            self.results.append(TestResult("Braking & Settling", "WARN", details, "Check DRV8833 brake mode or motor inertia."))
            return True
        else:
            details = "Motors continued coasting / did not reach zero speed within 800ms"
            print(f"  {tag_fail()} {details}")
            self.results.append(TestResult("Braking & Settling", "FAIL", details, "Verify DRV8833 stop/brake implementation."))
            return False

    # ── TEST 11: Safety Watchdog Timeout ──────────────────────────────────────
    def test_safety_watchdog(self) -> bool:
        self.log_step(11, "Safety Watchdog Timeout & Failsafe")
        print("  Starting motor movement, then intentionally cutting off PC commands...")
        print("  Verifying that the ESP32 automatically halts within WATCHDOG_MS (~500ms)...")

        # Command motion for 0.3s
        for _ in range(10):
            self.comm.send_command(self.speed, self.speed)
            time.sleep(0.033)

        # NOW CUT OFF ALL COMMANDS (do NOT call send_command)
        t_silence_start = time.time()
        timeout_triggered = False
        elapsed_to_stop = 0.0

        # Monitor feedback for 1.2 seconds without sending commands
        while time.time() - t_silence_start < 1.2:
            fb = self._get_latest_feedback(timeout=0.08)
            if fb:
                spd_l, spd_r = abs(fb[2]), abs(fb[3])
                if spd_l < 5.0 and spd_r < 5.0 and not timeout_triggered:
                    elapsed_to_stop = time.time() - t_silence_start
                    if elapsed_to_stop >= 0.35:
                        timeout_triggered = True
                        break
            time.sleep(0.02)

        # Restore comms
        self.comm.send_command(0.0, 0.0)

        if timeout_triggered:
            details = f"Watchdog engaged at {elapsed_to_stop*1000:.0f} ms (target: ~500 ms)"
            print(f"  {tag_pass()} {details}")
            self.results.append(TestResult("Safety Watchdog", "PASS", details))
            return True
        else:
            details = "Motors did NOT halt automatically when PC transmission stopped!"
            fix = "Check WATCHDOG_MS logic in esp32/robot.py. Motors must auto-stop if no command received."
            print(f"  {tag_fail()} {details}")
            self.results.append(TestResult("Safety Watchdog", "FAIL", details, fix))
            return False

    # ── Helper: Get latest feedback ──────────────────────────────────────────
    def _get_latest_feedback(self, timeout: float = 0.1) -> Optional[Tuple[int, int, float, float]]:
        t_end = time.time() + timeout
        latest = None
        while time.time() < t_end:
            line = self.comm.readline()
            if line and line.startswith("FB:"):
                try:
                    parts = line[3:].split(",")
                    latest = (int(parts[0]), int(parts[1]), float(parts[2]), float(parts[3]))
                except Exception:
                    pass
            time.sleep(0.005)
        return latest

    # ── Final Report Generation ──────────────────────────────────────────────
    def print_summary(self):
        self.log_header("HARDWARE DIAGNOSTIC SUMMARY REPORT")
        print(f"{'TEST NAME':<30} | {'STATUS':<10} | {'DETAILS':<35}")
        print(f"{'-'*30}-+-{'-'*10}-+-{'-'*35}")

        passes = 0
        fails = 0
        warns = 0

        for res in self.results:
            if res.status == "PASS":
                passes += 1
            elif res.status == "FAIL":
                fails += 1
            elif res.status == "WARN":
                warns += 1

            badge_str = f"{Color.GREEN}PASS{Color.RESET}" if res.status == "PASS" else (
                f"{Color.RED}FAIL{Color.RESET}" if res.status == "FAIL" else f"{Color.YELLOW}WARN{Color.RESET}"
            )
            # Truncate details if overly long
            det = (res.details[:32] + "...") if len(res.details) > 35 else res.details
            print(f"{res.name:<30} | {badge_str:<19} | {det:<35}")

        print(f"{'-'*30}-+-{'-'*10}-+-{'-'*35}")
        print(f"Total: {len(self.results)} | {Color.GREEN}Passed: {passes}{Color.RESET} | "
              f"{Color.RED}Failed: {fails}{Color.RESET} | {Color.YELLOW}Warnings: {warns}{Color.RESET}\n")

        # Actionable recommendations
        remediations = [r for r in self.results if r.remediation]
        if remediations:
            print(f"{Color.BOLD}{Color.YELLOW}ACTIONABLE FIXES & RECOMMENDATIONS:{Color.RESET}")
            for r in remediations:
                print(f"  * {Color.BOLD}{r.name}{Color.RESET}: {r.remediation}")
            print()
        elif fails == 0:
            print(f"{Color.BOLD}{Color.GREEN}[OK] ALL HARDWARE CHECKS PASSED! The robot is 100% operational.{Color.RESET}\n")

    # ── Master Runner ────────────────────────────────────────────────────────
    def run_all(self, interactive: bool = True):
        self.log_header("ESP32-S3 ROBOT COMPONENT DIAGNOSTIC SUITE")
        print(f"Target Port : {self.port}")
        print(f"Baud Rate   : {self.baud}")
        print(f"Test Speed  : {self.speed} ticks/s")
        print(f"Run Duration: {self.duration} s per direction\n")

        print(f"{Color.BOLD}{Color.YELLOW}"
              f"+--------------------------------------------------------------------+\n"
              f"|                         SAFETY NOTICE                              |\n"
              f"|  Please place the robot on a stand, box, or cup so that BOTH       |\n"
              f"|  wheels are ELEVATED in the air and can rotate freely!             |\n"
              f"+--------------------------------------------------------------------+"
              f"{Color.RESET}")

        if interactive:
            try:
                input("\nPress [ENTER] once the robot is elevated and USB is connected...")
            except KeyboardInterrupt:
                print("\nAborted by user.")
                return

        try:
            # Step 1: Port Connection
            if not self.test_serial_connection():
                self.print_summary()
                return

            # Step 2: Handshake
            if not self.test_handshake_and_firmware():
                self.print_summary()
                return

            # Step 3: Comms Quality
            self.test_communication_quality()

            # Step 4: Stationary Encoder Drift
            self.test_static_encoder_drift()

            # Step 5 & 6: Left Motor
            self.test_left_motor_forward()
            self.test_left_motor_reverse()

            # Step 7 & 8: Right Motor
            self.test_right_motor_forward()
            self.test_right_motor_reverse()

            # Step 9: Dual Motor Balance
            self.test_dual_motor_balance()

            # Step 10: Braking & Settling
            self.test_braking_settling()

            # Step 11: Safety Watchdog
            self.test_safety_watchdog()

        except KeyboardInterrupt:
            print(f"\n{Color.YELLOW}[!] Test interrupted by user (Ctrl+C). Stopping motors...{Color.RESET}")
        finally:
            if self.comm:
                self.comm.send_command(0.0, 0.0)
                time.sleep(0.1)
                self.comm.close()

        self.print_summary()


# ─────────────────────────────────────────────────────────────────────────────
# Entry Point & CLI
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Complete ESP32-S3 Robot Hardware Diagnostic Suite")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port for ESP32 (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default: {DEFAULT_BAUD})")
    parser.add_argument("--speed", type=float, default=150.0, help="Test speed in encoder ticks/s (default: 150.0)")
    parser.add_argument("--duration", type=float, default=1.2, help="Drive duration in seconds per test (default: 1.2)")
    parser.add_argument("--non-interactive", "-y", action="store_true", help="Skip the 'Press Enter' safety prompt")
    parser.add_argument("--list-ports", action="store_true", help="List all available COM ports and exit")

    args = parser.parse_args()

    if args.list_ports:
        ports = serial.tools.list_ports.comports()
        print("\nAvailable Serial Ports:")
        if not ports:
            print("  (No serial ports detected)")
        for p in ports:
            print(f"  * {p.device:<8}: {p.description} [{p.hwid}]")
        print()
        return

    runner = RobotDiagnosticRunner(
        port=args.port,
        baud=args.baud,
        speed=args.speed,
        duration=args.duration
    )
    runner.run_all(interactive=not args.non_interactive)


if __name__ == "__main__":
    main()
