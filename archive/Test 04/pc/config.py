"""
config.py — Central configuration for the trajectory-following robot system.

All tunable parameters in one place.  Edit values here, not scattered across files.
"""

import math

# ─────────────────────────────────────────────────────────────
#  ARENA
# ─────────────────────────────────────────────────────────────
ARENA_WIDTH_M  = 9 * 0.3048          # 9 ft  → 2.7432 m
ARENA_HEIGHT_M = 77 * 0.0254         # 77 in → 1.9558 m

# World-coordinate positions of each arena-corner ArUco marker.
# Origin is at the **center** of the arena so that negative trajectory
# coordinates (like x = −0.6) make sense.
ARENA_MARKERS = {
    0: (-ARENA_WIDTH_M / 2, -ARENA_HEIGHT_M / 2),   # bottom-left
    1: ( ARENA_WIDTH_M / 2, -ARENA_HEIGHT_M / 2),   # bottom-right
    2: ( ARENA_WIDTH_M / 2,  ARENA_HEIGHT_M / 2),   # top-right
    3: (-ARENA_WIDTH_M / 2,  ARENA_HEIGHT_M / 2),   # top-left
}

# ─────────────────────────────────────────────────────────────
#  ARUCO / VISION
# ─────────────────────────────────────────────────────────────
ARUCO_DICT_ID        = "DICT_4X4_50"
ARENA_MARKER_IDS     = [0, 1, 2, 3]        # IDs of the four corner markers
ROBOT_MARKER_ID      = 4                    # ID of the marker mounted on the robot

# Camera index (0 = default USB webcam)
CAMERA_INDEX         = 0
CAMERA_WIDTH         = 1280                 # Capture resolution
CAMERA_HEIGHT        = 720

# Smoothing: exponential moving average weight for position / heading.
# 1.0 = no smoothing (raw), 0.3 = heavy smoothing.
EMA_ALPHA_POS        = 0.6
EMA_ALPHA_THETA      = 0.5

# ─────────────────────────────────────────────────────────────
#  ROBOT PHYSICAL PARAMETERS
# ─────────────────────────────────────────────────────────────
WHEEL_DIAMETER_M     = 0.042               # 42 mm Pololu wheel
WHEEL_RADIUS_M       = WHEEL_DIAMETER_M / 2
WHEEL_CIRCUMFERENCE  = math.pi * WHEEL_DIAMETER_M   # ≈ 0.13195 m
WHEEL_BASE_M         = 0.075               # 7.5 cm between wheel centres

# ─────────────────────────────────────────────────────────────
#  MOTOR / ENCODER
# ─────────────────────────────────────────────────────────────
MOTOR_RATED_RPM      = 500                  # No-load RPM @ 6 V
ENCODER_PPR          = 7                    # Pulses per motor-shaft revolution
GEAR_RATIO           = 30                   # Approximate for 500 RPM N20

# Ticks per output-shaft revolution (2× quadrature decoding on channel A).
# >>> CALIBRATE THIS by rotating the wheel exactly one revolution
#     and reading the encoder count.  See README. <<<
TICKS_PER_REV        = ENCODER_PPR * 2 * GEAR_RATIO   # 420

# Derived conversion factor
METERS_PER_TICK      = WHEEL_CIRCUMFERENCE / TICKS_PER_REV
TICKS_PER_METER      = TICKS_PER_REV / WHEEL_CIRCUMFERENCE

# Maximum wheel speed the PID is allowed to demand (ticks / s)
MAX_MOTOR_RPM        = 400                  # leave headroom below 500
MAX_TICKS_PER_SEC    = MAX_MOTOR_RPM / 60 * TICKS_PER_REV

# ─────────────────────────────────────────────────────────────
#  TRAJECTORY CONTROLLER  (Pure Pursuit + heading correction)
# ─────────────────────────────────────────────────────────────
LOOKAHEAD_DISTANCE   = 0.08                # metres  (tune: 0.05–0.15)
CRUISE_SPEED         = 0.12                # m/s nominal linear speed
MIN_SPEED            = 0.03                # m/s when approaching goal
GOAL_TOLERANCE       = 0.02                # m — waypoint reached
FINAL_HEADING_TOL    = math.radians(5)     # rad — final heading tolerance
K_HEADING            = 1.5                 # heading-error gain (rad/s per rad)

# Slow-down zone near trajectory end
SLOWDOWN_DISTANCE    = 0.10                # start decelerating this far from end (m)

# ─────────────────────────────────────────────────────────────
#  SERIAL COMMUNICATION
# ─────────────────────────────────────────────────────────────
SERIAL_PORT          = "COM3"              # change to match your port
SERIAL_BAUD          = 115200
SERIAL_TIMEOUT       = 0.01               # read timeout (seconds)
COMMAND_INTERVAL_S   = 0.033              # send commands at ~30 Hz
WATCHDOG_TIMEOUT_S   = 0.5               # ESP32 stops motors if silent this long

# ─────────────────────────────────────────────────────────────
#  CONTROL LOOP
# ─────────────────────────────────────────────────────────────
CONTROL_RATE_HZ      = 30                  # vision + controller loop rate
LOST_MARKER_TIMEOUT  = 0.5                # seconds without robot detection → stop
