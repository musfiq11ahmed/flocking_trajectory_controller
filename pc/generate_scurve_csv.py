"""Generate the S-curve waypoint file used by Test 4 (waypoint navigation).

Produces a smooth parametric S-curve in the plane with EXACTLY 250 waypoints.
The curve is a cubic "smoothstep" S centered on the arena origin:

    x(t) = WIDTH  * (t - 0.5)
    y(t) = HEIGHT * (3*s^2 - 2*s^3) - HEIGHT/2,   s = t in [0, 1]

which has zero slope at both ends and a single smooth inflection in the
middle (an S shape). theta at each waypoint is the path tangent angle:
theta = atan2(dy/dt, dx/dt).

Default arena scale fits inside ~1.5 m x 1.5 m (WIDTH = 1.2 m, HEIGHT = 0.9 m).

Usage:
    python3 generate_scurve_csv.py                  # writes scurve_trajectory.csv
                                                    # next to this script
    python3 generate_scurve_csv.py --out /tmp/s.csv
"""

import argparse
import csv
import math
import os

NUM_WAYPOINTS = 250          # spec: exactly 250 waypoints
ARENA_WIDTH_M = 1.2          # x extent of the curve (~1.5 m arena)
ARENA_HEIGHT_M = 0.9         # y extent of the curve


def scurve_point(t):
    """Return (x, y, theta) for parameter t in [0, 1] on the cubic S-curve."""
    s = t
    x = ARENA_WIDTH_M * (s - 0.5)
    y = ARENA_HEIGHT_M * (3.0 * s * s - 2.0 * s * s * s) - ARENA_HEIGHT_M / 2.0
    # Path tangent from parametric derivatives.
    dx_dt = ARENA_WIDTH_M
    dy_dt = ARENA_HEIGHT_M * (6.0 * s - 6.0 * s * s)
    theta = math.atan2(dy_dt, dx_dt)
    return x, y, theta


def main():
    default_out = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "scurve_trajectory.csv"
    )
    parser = argparse.ArgumentParser(
        description="Generate scurve_trajectory.csv (250 waypoints: x,y,theta)."
    )
    parser.add_argument(
        "--out",
        default=default_out,
        help="output CSV path (default: scurve_trajectory.csv next to this script)",
    )
    args = parser.parse_args()

    with open(args.out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "theta"])   # header required by the spec
        for i in range(NUM_WAYPOINTS):
            t = i / (NUM_WAYPOINTS - 1.0)      # inclusive endpoints 0..1
            x, y, theta = scurve_point(t)
            writer.writerow(["%.6f" % x, "%.6f" % y, "%.6f" % theta])

    print("Wrote %d waypoints to %s" % (NUM_WAYPOINTS, args.out))


if __name__ == "__main__":
    main()
