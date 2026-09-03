"""Real-building geometry for the map_zone world, parsed from the raw
OpenStreetMap source data map_zone_ap.sdf's terrain mesh was generated
from -- unlike urban_canyon's synthetic BUILDINGS list, there's no
hand-authored per-building geometry for the real Bucharest terrain, just
the baked .dae mesh itself. map_zone.osm (the mesh's own OSM source,
checked in alongside it) still has each building's footprint and
height/level tags, so this module recovers a Box list from that instead.

Produces the same Box shape canyon_geometry.BUILDINGS does, so
canyon_field's per-building recirculation math (_recirculation) can
consume either list unchanged -- see canyon_field.generate_map_zone.

The map_zone world's vehicle spawn and terrain <include> both sit at
local ENU (0, 0) (see worlds/map_zone_ap.sdf), matching this project's
--home coordinates (HOME_LAT/HOME_LON below) -- i.e. local ENU (0, 0) IS
the home lat/lon point, same anchor controller_node's mission lat/lon
math uses. Sanity-check this alignment once live (compare a known
building's Gazebo world position against its parsed Box here) before
trusting the generated wind field's building positions.
"""
import pathlib
import xml.etree.ElementTree as ET

import numpy as np

from . import frames
from .canyon_geometry import Box

OSM_PATH = (pathlib.Path(__file__).resolve().parents[1]
            / 'map_zone' / 'meshes' / 'map_zone.osm')

# Matches controller_node.HOME_LAT/HOME_LON and run_trial.HOME_LAT/HOME_LON
# -- kept as its own copy rather than a shared import, the same
# duplication already established between those two modules.
HOME_LAT, HOME_LON = 44.434424990487216, 26.04781615647584

DEFAULT_HEIGHT_M = 9.0  # ~3-storey default for buildings with neither tag
LEVEL_HEIGHT_M = 3.0


def _building_height(tags):
    if 'height' in tags:
        try:
            return float(tags['height'])
        except ValueError:
            pass
    if 'building:levels' in tags:
        try:
            return float(tags['building:levels']) * LEVEL_HEIGHT_M
        except ValueError:
            pass
    return DEFAULT_HEIGHT_M


def _parse_buildings(osm_path):
    """Every OSM way tagged building=* -> a Box, its footprint's
    axis-aligned ENU bounding box (ENU x=east, y=north, both metres from
    HOME_LAT/HOME_LON -- see the module docstring for why that's the
    right anchor)."""
    root = ET.parse(osm_path).getroot()
    node_latlon = {n.get('id'): (float(n.get('lat')), float(n.get('lon')))
                   for n in root.iter('node')}

    boxes = []
    for i, way in enumerate(root.iter('way')):
        tags = {tag.get('k'): tag.get('v') for tag in way.iter('tag')}
        if 'building' not in tags:
            continue
        points = []
        for nd in way.iter('nd'):
            latlon = node_latlon.get(nd.get('ref'))
            if latlon is None:
                continue
            north, east = frames.latlon_to_ned(latlon[0], latlon[1],
                                               HOME_LAT, HOME_LON)
            points.append((east, north))
        if len(points) < 3:
            continue  # degenerate way, not a real footprint
        pts = np.array(points)
        x_lo, y_lo = pts[:, 0].min(), pts[:, 1].min()
        x_hi, y_hi = pts[:, 0].max(), pts[:, 1].max()
        boxes.append(Box(
            name=f'map_zone_building_{i}',
            cx=(x_lo + x_hi) / 2.0, cy=(y_lo + y_hi) / 2.0,
            sx=max(x_hi - x_lo, 1.0), sy=max(y_hi - y_lo, 1.0),
            sz=_building_height(tags)))
    return boxes


BUILDINGS = _parse_buildings(OSM_PATH) if OSM_PATH.exists() else []
