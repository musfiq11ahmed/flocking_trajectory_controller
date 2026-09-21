"""UDP protocol helpers shared by every PC-side test.

These functions are byte-identical to the packed structs in
firmware/robot_firmware.py (MicroPython). Both sides use little-endian,
standard-size, no-padding layouts:

  Command   PC -> ESP32, 8 bytes:
      struct '<ff'  ->  float target_left_rpm, float target_right_rpm

  Telemetry ESP32 -> PC, 24 bytes @ 20 Hz:
      struct '<ffiiff' ->
          float left_rpm_measured
          float right_rpm_measured
          int32  left_ticks          (total quadrature ticks since boot)
          int32  right_ticks
          float  left_pwm_duty       (signed, -1.0 .. +1.0)
          float  right_pwm_duty      (signed, -1.0 .. +1.0)

IMPORTANT: 'i' (int32), NOT 'l' (long, platform-dependent size), is used for
the tick counters so the packet stays exactly 24 bytes on every platform.
"""

import struct

# --- Protocol constants (must match #defines in the firmware) ---------------
UDP_PORT = 4210                 # command listener port on the ESP32
COMMAND_FORMAT = "<ff"
COMMAND_SIZE = struct.calcsize(COMMAND_FORMAT)      # 8 bytes
TELEMETRY_FORMAT = "<ffiiff"
TELEMETRY_SIZE = struct.calcsize(TELEMETRY_FORMAT)  # 24 bytes
TELEMETRY_PERIOD_MS = 50        # firmware streams telemetry at 20 Hz
COMMAND_PERIOD_MS = 50          # recommended PC->robot command rate (20 Hz)
FAILSAFE_TIMEOUT_MS = 500       # robot coasts if no command within this window

# Telemetry dict keys, in wire order.
TELEMETRY_KEYS = (
    "left_rpm",
    "right_rpm",
    "left_ticks",
    "right_ticks",
    "left_pwm",
    "right_pwm",
)


def pack_command(left_rpm, right_rpm):
    """Pack an RPM command into the 8-byte wire format (<ff).

    A command of exactly (0.0, 0.0) means COAST on the robot; simply not
    sending packets for > FAILSAFE_TIMEOUT_MS has the same effect (failsafe).
    """
    return struct.pack(COMMAND_FORMAT, float(left_rpm), float(right_rpm))


def unpack_telemetry(data):
    """Unpack a 24-byte telemetry datagram (<ffiiff) into a dict.

    Raises ValueError if the datagram size is wrong (e.g. truncated packet or
    a protocol/version mismatch).
    """
    if len(data) != TELEMETRY_SIZE:
        raise ValueError(
            "telemetry datagram must be %d bytes, got %d" % (TELEMETRY_SIZE, len(data))
        )
    values = struct.unpack(TELEMETRY_FORMAT, data)
    return dict(zip(TELEMETRY_KEYS, values))
