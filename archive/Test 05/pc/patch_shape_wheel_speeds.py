#!/usr/bin/env python3
"""One-shot patch: test_suite.py anti-stall fix (shape_wheel_speeds).

Fixes the bug that made the physical bot spin randomly in place during
Test 4: friction_compensate()+clamp_wheel_speeds() cancelled each other,
so sub-stall (1-15 RPM) wheel targets were sent to motors that need
>= 27 RPM. Replaced by shape_wheel_speeds(): clamp FIRST, then floor.

Usage:  cd <project root>   (the folder that contains pc/)
        python3 patch_shape_wheel_speeds.py
Creates pc/test_suite.py.bak before modifying anything.
"""
import pathlib, sys

p = pathlib.Path("pc/test_suite.py")
if not p.exists():
    sys.exit("ERROR: pc/test_suite.py not found -- run from the project root")
src = p.read_text(encoding="utf-8")
if "def shape_wheel_speeds" in src:
    sys.exit("ALREADY PATCHED: nothing to do")
if "def friction_compensate" not in src:
    sys.exit("ERROR: file version unknown, patch aborted (no changes made)")

PAIRS = [
    ('def friction_compensate(v_left, v_right, min_speed=MIN_WHEEL_SPEED_MS):\n    """Anti-stall: if either wheel is commanded to move but the slower wheel\n    is below min_speed, scale BOTH wheels up so the slower one reaches\n    min_speed while preserving the turning ratio."""\n    a_l, a_r = abs(v_left), abs(v_right)\n    hi, lo = max(a_l, a_r), min(a_l, a_r)\n    if hi < 1e-9:\n        return 0.0, 0.0                      # no motion commanded\n    if lo >= min_speed:\n        return v_left, v_right               # already above the floor\n    if lo < 1e-9:\n        # Pivot (one wheel at zero): put the stopped wheel at min_speed in\n        # the direction of the moving wheel\'s motion.\n        if a_l < a_r:\n            v_left = math.copysign(min_speed, v_right)\n        else:\n            v_right = math.copysign(min_speed, v_left)\n        return v_left, v_right\n    scale = min_speed / lo\n    return v_left * scale, v_right * scale\n\n\ndef clamp_wheel_speeds(v_left, v_right, max_speed=MAX_WHEEL_SPEED_MS):\n    """Cap wheel speeds at max_speed, scaling both down together to preserve\n    the turning ratio."""\n    hi = max(abs(v_left), abs(v_right))\n    if hi > max_speed:\n        s = max_speed / hi\n        v_left *= s\n        v_right *= s\n    return v_left, v_right\n',
     'def shape_wheel_speeds(v_left, v_right,\n                       min_speed=MIN_WHEEL_SPEED_MS,\n                       max_speed=MAX_WHEEL_SPEED_MS):\n    """Clamp + anti-stall floor, in the ONLY order that works.\n\n    ORDER MATTERS -- the old friction_compensate()-then-clamp sequence\n    cancelled itself: scaling the slow wheel up by min/lo and then\n    clamping the fast wheel back down to max restores the ORIGINAL ratio\n    exactly, so the anti-stall floor vanished precisely when it was\n    needed (any turn sharper than lo/hi = min/max = 0.6). The robot then\n    received 1-15 RPM targets this drivetrain physically cannot execute\n    (minimum sustained wheel speed = 27 RPM): the slow wheel stalls, the\n    firmware stall-guard winds its integrator up, the wheel breaks free\n    at 27+ RPM and lurches -- on the ground this looks like the bot\n    "spinning randomly in place" (confirmed by telemetry in the Test 4\n    log: commanded vR=0.00-0.03 m/s while the physical wheel reported\n    34-48 RPM).\n\n    Here the clamp runs FIRST (turning ratio preserved), then any wheel\n    that is moving-but-below-the-floor is bumped up to min_speed (sign\n    preserved). Nothing clamps afterwards, so the floor always survives.\n    Bumping only the slow wheel distorts the ratio slightly (turns run a\n    little wider than requested); the 20 Hz pose feedback loop corrects\n    the heading on the very next cycle, and every command sent is one\n    the motors can actually track.\n    """\n    a_l, a_r = abs(v_left), abs(v_right)\n    if a_l < 1e-9 and a_r < 1e-9:\n        return 0.0, 0.0                       # no motion commanded\n    # 1) clamp to max, ratio preserved\n    hi = max(a_l, a_r)\n    if hi > max_speed:\n        s = max_speed / hi\n        v_left *= s\n        v_right *= s\n    # 2) anti-stall floor (survives because nothing clamps after this).\n    #    A wheel at exact zero stays stopped: a true pivot about a\n    #    dragged wheel is executable and tighter than a wide arc.\n    if 0.0 < abs(v_left) < min_speed:\n        v_left = math.copysign(min_speed, v_left)\n    if 0.0 < abs(v_right) < min_speed:\n        v_right = math.copysign(min_speed, v_right)\n    return v_left, v_right\n'),
    ('      7. Friction compensation: min wheel speed >= 0.15 m/s, ratio preserved.\n      8. Clamp: max wheel speed <= 0.30 m/s, ratio preserved.',
     '      7. Shape for the drivetrain: clamp to MAX_WHEEL_SPEED_MS, then lift\n         any moving wheel to the MIN_WHEEL_SPEED_MS anti-stall floor.'),
    ('    v_left, v_right = friction_compensate(v_left, v_right, MIN_WHEEL_SPEED_MS)\n    v_left, v_right = clamp_wheel_speeds(v_left, v_right, MAX_WHEEL_SPEED_MS)\n    return v_left, v_right, dist, heading_err',
     '    v_left, v_right = shape_wheel_speeds(v_left, v_right,\n                                         MIN_WHEEL_SPEED_MS,\n                                         MAX_WHEEL_SPEED_MS)\n    return v_left, v_right, dist, heading_err'),
    ('        # law. The anti-stall floor (0.15 m/s min wheel speed) means the\n        # robot cannot park dead-on, so "arrived" is HOME_POS_TOL_M; the\n        # heading is aligned afterwards in stage 1b.',
     '        # law. The anti-stall floor (MIN_WHEEL_SPEED_MS per wheel) means\n        # the robot cannot park dead-on, so "arrived" is HOME_POS_TOL_M;\n        # the heading is aligned afterwards in stage 1b.'),
    ('            v_left, v_right = friction_compensate(v_left, v_right,\n                                                  MIN_WHEEL_SPEED_MS)\n            v_left, v_right = clamp_wheel_speeds(v_left, v_right,\n                                                 MAX_WHEEL_SPEED_MS)',
     '            v_left, v_right = shape_wheel_speeds(v_left, v_right,\n                                                 MIN_WHEEL_SPEED_MS,\n                                                 MAX_WHEEL_SPEED_MS)'),
    ('# Tasks 5-8: errors -> unicycle law -> friction comp -> clamp.',
     '# Tasks 5-8: errors -> unicycle law -> drivetrain shaping.'),
]

for old_s, new_s in PAIRS:
    if old_s not in src:
        sys.exit("ERROR: expected text not found: " + old_s[:70]
                 + " ... patch aborted (no changes made)")
    src = src.replace(old_s, new_s)

p.with_suffix(".py.bak").write_text(p.read_text(encoding="utf-8"), encoding="utf-8")
p.write_text(src, encoding="utf-8")
compile(src, str(p), "exec")
leftover = src.replace("old friction_compensate()-then-clamp", "")
assert "friction_compensate" not in leftover
print("PATCH OK: %d shape_wheel_speeds refs; backup at pc/test_suite.py.bak"
      % src.count("shape_wheel_speeds"))
