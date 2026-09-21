#!/usr/bin/env python3
"""Generate the URDF files for the arena calibrate-align-move simulation.

Writes:
    urdf/arena.urdf  -- light-gray floor + 4 saturated colored anchor plates
                        at world (+-1.5, +-1.5) (ids 0..3 per config.ANCHORS)
    urdf/bot.urdf    -- gray chassis + asymmetric white top plate + black nose
                        stripe marker (the bot is "marker 4")

Run from the repo root:

    python3 scripts/generate_urdf.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import config as C  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
URDF_DIR = os.path.normpath(os.path.join(HERE, "..", "urdf"))


def _rgba(rgb, a=1.0):
    return " ".join(f"{v:.4f}" for v in (*rgb, a))


def _box_geometry(size):
    return (
        "    <geometry><box size=\"{}\"/></geometry>".format(
            " ".join(f"{v:.4f}" for v in size)
        )
    )


def _visual(name, xyz, size, rgb):
    return (
        "  <visual name=\"{name}\">\n"
        "    <origin xyz=\"{xyz}\" rpy=\"0 0 0\"/>\n"
        "{geom}\n"
        "    <material name=\"{name}_mat\"><color rgba=\"{rgba}\"/></material>\n"
        "  </visual>".format(
            name=name, xyz=" ".join(f"{v:.4f}" for v in xyz),
            geom=_box_geometry(size), rgba=_rgba(rgb),
        )
    )


def generate_arena_urdf():
    """Floor base link with 4 fixed-joint colored anchor plate links."""
    parts = ['<?xml version="1.0"?>', '<robot name="arena">']
    # Floor base link.
    parts.append('<link name="floor">')
    parts.append(_visual(
        "floor", (0.0, 0.0, -0.5 * C.FLOOR_THICK),
        (2 * C.FLOOR_HALF, 2 * C.FLOOR_HALF, C.FLOOR_THICK), C.FLOOR_RGB))
    parts.append(
        "  <collision>\n"
        "    <origin xyz=\"0 0 {z:.4f}\" rpy=\"0 0 0\"/>\n"
        "{geom}\n"
        "  </collision>".format(z=-0.5 * C.FLOOR_THICK, geom=_box_geometry(
            (2 * C.FLOOR_HALF, 2 * C.FLOOR_HALF, C.FLOOR_THICK)))
    )
    parts.append(
        "  <inertial><mass value=\"1.0\"/>"
        "<inertia ixx=\"1\" ixy=\"0\" ixz=\"0\" iyy=\"1\" iyz=\"0\" izz=\"1\"/></inertial>"
    )
    parts.append('</link>')

    for aid in C.ANCHOR_IDS:
        wx, wy, rgb = C.ANCHORS[aid]
        link = "anchor_%d" % aid
        parts.append('<link name="%s">' % link)
        z = 0.5 * C.ANCHOR_PLATE_THICK
        parts.append(_visual(
            link, (0.0, 0.0, z),
            (C.ANCHOR_PLATE_SIZE, C.ANCHOR_PLATE_SIZE, C.ANCHOR_PLATE_THICK), rgb))
        # Small inertia so the fixed joint is well formed; the arena is welded
        # to the world anyway (fixed base).
        parts.append(
            "  <inertial><mass value=\"0.05\"/>"
            "<inertia ixx=\"1e-4\" ixy=\"0\" ixz=\"0\" iyy=\"1e-4\" iyz=\"0\""
            " izz=\"1e-4\"/></inertial>"
        )
        parts.append('</link>')
        parts.append(
            '<joint name="%s_joint" type="fixed">\n'
            '  <parent link="floor"/>\n'
            '  <child link="%s"/>\n'
            '  <origin xyz="%.4f %.4f 0" rpy="0 0 0"/>\n'
            '</joint>' % (link, link, wx, wy)
        )
    parts.append('</robot>')
    return "\n".join(parts) + "\n"


def generate_bot_urdf():
    """Chassis base link + fixed links for the white plate and black stripe."""
    cx, cy, cz = C.BOT_CHASSIS_SIZE
    parts = ['<?xml version="1.0"?>', '<robot name="bot">']

    # Chassis base link (collision + inertia here).
    parts.append('<link name="chassis">')
    parts.append(_visual("chassis", (0.0, 0.0, 0.5 * cz),
                         C.BOT_CHASSIS_SIZE, C.BOT_CHASSIS_RGB))
    parts.append(
        "  <collision>\n"
        "    <origin xyz=\"0 0 {z:.4f}\" rpy=\"0 0 0\"/>\n"
        "{geom}\n"
        "  </collision>".format(z=0.5 * cz, geom=_box_geometry(C.BOT_CHASSIS_SIZE))
    )
    ixx = C.BOT_MASS * (cy ** 2 + cz ** 2) / 12.0
    iyy = C.BOT_MASS * (cx ** 2 + cz ** 2) / 12.0
    izz = C.BOT_MASS * (cx ** 2 + cy ** 2) / 12.0
    parts.append(
        "  <inertial><mass value=\"{m:.4f}\"/>"
        "<inertia ixx=\"{ixx:.6f}\" ixy=\"0\" ixz=\"0\" iyy=\"{iyy:.6f}\""
        " iyz=\"0\" izz=\"{izz:.6f}\"/></inertial>".format(
            m=C.BOT_MASS, ixx=ixx, iyy=iyy, izz=izz)
    )
    parts.append('</link>')

    # White top plate (visual only, sits on top of the chassis).
    px = 0.5 * (C.PLATE_X_MIN + C.PLATE_X_MAX)
    py = 0.5 * (C.PLATE_Y_MIN + C.PLATE_Y_MAX)
    psx = C.PLATE_X_MAX - C.PLATE_X_MIN
    psy = C.PLATE_Y_MAX - C.PLATE_Y_MIN
    parts.append('<link name="top_plate">')
    parts.append(_visual(
        "top_plate", (0.0, 0.0, 0.5 * C.PLATE_THICK),
        (psx, psy, C.PLATE_THICK), C.PLATE_RGB))
    parts.append(
        "  <inertial><mass value=\"0.02\"/>"
        "<inertia ixx=\"1e-5\" ixy=\"0\" ixz=\"0\" iyy=\"1e-5\" iyz=\"0\""
        " izz=\"1e-5\"/></inertial>"
    )
    parts.append('</link>')
    parts.append(
        '<joint name="top_plate_joint" type="fixed">\n'
        '  <parent link="chassis"/>\n'
        '  <child link="top_plate"/>\n'
        '  <origin xyz="{px:.4f} {py:.4f} {pz:.4f}" rpy="0 0 0"/>\n'
        '</joint>'.format(px=px, py=py, pz=cz)
    )

    # Black nose stripe (visual only, on top of the plate at +X front).
    sx = 0.5 * (C.STRIPE_X_MIN + C.STRIPE_X_MAX)
    sy = 0.5 * (C.STRIPE_Y_MIN + C.STRIPE_Y_MAX)
    ssx = C.STRIPE_X_MAX - C.STRIPE_X_MIN
    ssy = C.STRIPE_Y_MAX - C.STRIPE_Y_MIN
    parts.append('<link name="nose_stripe">')
    parts.append(_visual(
        "nose_stripe", (0.0, 0.0, 0.5 * C.STRIPE_THICK),
        (ssx, ssy, C.STRIPE_THICK), C.STRIPE_RGB))
    parts.append(
        "  <inertial><mass value=\"0.005\"/>"
        "<inertia ixx=\"1e-6\" ixy=\"0\" ixz=\"0\" iyy=\"1e-6\" iyz=\"0\""
        " izz=\"1e-6\"/></inertial>"
    )
    parts.append('</link>')
    parts.append(
        '<joint name="nose_stripe_joint" type="fixed">\n'
        '  <parent link="chassis"/>\n'
        '  <child link="nose_stripe"/>\n'
        '  <origin xyz="{sx:.4f} {sy:.4f} {sz:.4f}" rpy="0 0 0"/>\n'
        '</joint>'.format(sx=sx, sy=sy, sz=cz + C.PLATE_THICK)
    )

    parts.append('</robot>')
    return "\n".join(parts) + "\n"


def main():
    os.makedirs(URDF_DIR, exist_ok=True)
    arena_path = os.path.join(URDF_DIR, "arena.urdf")
    bot_path = os.path.join(URDF_DIR, "bot.urdf")
    with open(arena_path, "w") as f:
        f.write(generate_arena_urdf())
    with open(bot_path, "w") as f:
        f.write(generate_bot_urdf())
    print("wrote %s" % arena_path)
    print("wrote %s" % bot_path)


if __name__ == "__main__":
    main()
