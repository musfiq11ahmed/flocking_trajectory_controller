"""
serial_comm.py — USB-serial communication with the ESP32-S3.

Protocol (text-based, newline-terminated)
─────────────────────────────────────────
  PC  → ESP32:   CMD:<v_left>,<v_right>\n     target wheel speeds (ticks/s)
  ESP32 → PC:    FB:<enc_l>,<enc_r>,<spd_l>,<spd_r>\n    feedback
  ESP32 → PC:    READY\n                       handshake after boot
"""

import time
import serial
from typing import Optional, Tuple

from config import SERIAL_PORT, SERIAL_BAUD, SERIAL_TIMEOUT


class RobotSerial:
    """Thread-safe serial link to the ESP32-S3 motor controller."""

    def __init__(self, port: str = SERIAL_PORT, baud: int = SERIAL_BAUD):
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self._connected = False
        self._buf = ""

    # ── connection ────────────────────────────────────────────

    def connect(self, timeout: float = 10.0) -> bool:
        """Open the serial port and wait for the ESP32 'READY' handshake.

        Returns True on success, False on timeout.
        """
        try:
            self.ser = serial.Serial(
                self.port, self.baud,
                timeout=SERIAL_TIMEOUT,
                write_timeout=1.0,
            )
            time.sleep(0.5)          # let the ESP32 finish booting
            self.ser.reset_input_buffer()
        except serial.SerialException as e:
            print(f"[SERIAL] Cannot open {self.port}: {e}")
            return False

        print(f"[SERIAL] Port {self.port} opened — waiting for READY …")
        t0 = time.time()
        while time.time() - t0 < timeout:
            line = self._readline()
            if line and "READY" in line:
                print("[SERIAL] ESP32 is READY.")
                self._connected = True
                return True
            time.sleep(0.05)

        print("[SERIAL] Timeout waiting for READY.")
        return False

    def disconnect(self):
        """Stop motors and close the port."""
        if self.ser and self.ser.is_open:
            try:
                self.send_command(0, 0)
            except Exception:
                pass
            self.ser.close()
        self._connected = False
        print("[SERIAL] Disconnected.")

    @property
    def is_connected(self) -> bool:
        return self._connected and self.ser is not None and self.ser.is_open

    # ── TX ────────────────────────────────────────────────────

    def send_command(self, v_left_tps: float, v_right_tps: float):
        """Send a wheel-speed command (ticks/s, signed)."""
        if not self.is_connected:
            return
        msg = f"CMD:{v_left_tps:.1f},{v_right_tps:.1f}\n"
        try:
            self.ser.write(msg.encode("ascii"))
        except serial.SerialException as e:
            print(f"[SERIAL] Write error: {e}")
            self._connected = False

    # ── RX ────────────────────────────────────────────────────

    def read_feedback(self) -> Optional[Tuple[int, int, float, float]]:
        """Non-blocking read of the latest feedback line.

        Returns (enc_left, enc_right, speed_left, speed_right) or None.
        """
        line = self._readline()
        if line and line.startswith("FB:"):
            try:
                parts = line[3:].split(",")
                return (int(parts[0]), int(parts[1]),
                        float(parts[2]), float(parts[3]))
            except (ValueError, IndexError):
                pass
        return None

    # ── internal ──────────────────────────────────────────────

    def _readline(self) -> Optional[str]:
        """Read one newline-terminated line (non-blocking)."""
        if not self.ser or not self.ser.is_open:
            return None
        try:
            while self.ser.in_waiting:
                ch = self.ser.read(1).decode("ascii", errors="replace")
                if ch == "\n":
                    line = self._buf.strip()
                    self._buf = ""
                    return line
                self._buf += ch
        except serial.SerialException:
            pass
        return None
