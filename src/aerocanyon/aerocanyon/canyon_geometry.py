"""Urban canyon building geometry -- the single source of truth.

The Gazebo world SDF is GENERATED from BUILDINGS (run this module as a
script), and the CBF obstacle barrier imports the same list. Never edit
worlds/urban_canyon.sdf by hand: regenerate it. If the SDF and this list
disagree, the safety filter is protecting against buildings that are not
in the simulation and every trial result is void.

All coordinates are Gazebo ENU, metres. Boxes rest on the ground, so a
box spans z in [0, sz].
"""
from collections import namedtuple

import numpy as np

Box = namedtuple('Box', 'name cx cy sx sy sz')

# Two facing rows of three towers, corridor running along +x at y = 0.
# 24 m gap between the rows: wide enough for a tilt-rotor transition,
# narrow enough for corner vortices to matter.
_ROW_OFFSET = 20.0        # centre of each row, +/- y
_DEPTH = 16.0             # building extent across the canyon
_WIDTH = 20.0             # building extent along the canyon
_SPACING = 45.0           # spacing along +x between tower centres

BUILDINGS = [
    Box(f'tower_{i}_{"n" if side > 0 else "s"}',
        cx=(i - 1) * _SPACING,
        cy=side * _ROW_OFFSET,
        sx=_WIDTH,
        sy=_DEPTH,
        sz=height)
    for i, height in enumerate((45.0, 60.0, 38.0))
    for side, in ((1,), (-1,))
]

# Half-width of the clear corridor between the two rows.
CANYON_HALF_WIDTH = _ROW_OFFSET - _DEPTH / 2.0

# Transit waypoints, clear of the towers at both ends and symmetric about
# the tower group's own centre (the towers themselves already span
# x in [-55, 55], centred at x=0 -- see BUILDINGS above -- and the ground
# plane is centred at the origin too, so +-100 keeps everything, including
# the vehicle's own spawn point (run_trial.SPAWN_XYZ, derived from
# CANYON_ENTRY), centred on the ground plane rather than offset toward one
# end). z = 25 m puts the vehicle in the shear layer rather than above the
# roofline.
CANYON_ENTRY = np.array([-100.0, 0.0, 25.0])
CANYON_EXIT = np.array([100.0, 0.0, 25.0])


def _box_distance_and_normal(p, b):
    """Distance from ENU point p to box b, and the outward unit normal.

    Uses the standard exterior box distance: clamp the point into the box
    to find the closest surface point, then the offset vector gives both
    the distance and the direction that increases it. Exact and
    piecewise-constant in gradient, which is what makes the CBF's
    second-order term vanish.
    """
    half = np.array([b.sx / 2.0, b.sy / 2.0, b.sz / 2.0])
    centre = np.array([b.cx, b.cy, b.sz / 2.0])
    d = np.abs(p - centre) - half
    outside = np.maximum(d, 0.0)
    dist = float(np.linalg.norm(outside))
    if dist > 1e-9:
        normal = outside * np.sign(p - centre) / dist
        return dist, normal
    # Inside or exactly on the surface: push out along the least-penetrated
    # axis. Negative distance signals a violation to the caller.
    axis = int(np.argmax(d))
    normal = np.zeros(3)
    normal[axis] = np.sign(p[axis] - centre[axis]) or 1.0
    return float(d[axis]), normal


def distance_and_normal(p):
    """Distance to the nearest building, and the unit outward normal.

    Negative distance means the point is inside a building.
    """
    p = np.asarray(p, dtype=float)
    best = min((_box_distance_and_normal(p, b) for b in BUILDINGS),
               key=lambda dn: dn[0])
    return best


def to_sdf():
    """Render BUILDINGS as SDF <model> blocks for inclusion in a world."""
    blocks = []
    for b in BUILDINGS:
        blocks.append(f"""
    <model name="{b.name}">
      <static>true</static>
      <pose>{b.cx} {b.cy} {b.sz / 2.0} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{b.sx} {b.sy} {b.sz}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{b.sx} {b.sy} {b.sz}</size></box></geometry>
          <material>
            <ambient>0.76 0.69 0.55 1</ambient>
            <diffuse>0.83 0.76 0.61 1</diffuse>
          </material>
        </visual>
      </link>
    </model>""")
    return ''.join(blocks)


if __name__ == '__main__':
    import pathlib
    import sys

    src = pathlib.Path(__file__).resolve().parents[1] / 'worlds' / '_template.sdf'
    dst = pathlib.Path(__file__).resolve().parents[1] / 'worlds' / 'urban_canyon.sdf'
    if not src.exists():
        sys.exit(f'missing template: {src}')
    dst.write_text(src.read_text().replace('<!--BUILDINGS-->', to_sdf()))
    print(f'wrote {dst} with {len(BUILDINGS)} buildings')
