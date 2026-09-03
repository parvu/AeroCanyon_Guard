# map_zone Baseline/Treatment Trials Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `run_trial.py --world map_zone --mission-file <file>` runs baseline/treatment trials against the real Bucharest terrain world, flying a mission authored once in Mission Planner and captured to a file, under a wind field generated from the real OSM building geometry.

**Architecture:** Every new code path is additive and gated on `world == 'map_zone'` — the existing `urban_canyon` path (default) is untouched behaviorally, verified by the existing test suite staying green throughout. A new `map_zone_geometry.py` parses real building footprints from the checked-in `map_zone.osm` into the same `Box` shape `canyon_geometry.BUILDINGS` already uses, so the existing per-building wind recirculation math needs no changes to consume it. A mission is authored live in Mission Planner (against a manually-flown map_zone SITL session) and captured once via a new `dump_mission.py` to a JSON file; `controller_node` replays that file's items verbatim on every trial leg instead of building its own fixed entry/exit mission, and generalizes its wind-correction target from a hardcoded waypoint index to whichever waypoint MAVROS reports as currently active (a strict generalization — for `urban_canyon`'s single-waypoint mission this remains the same index it always was).

**Tech Stack:** Python 3, ROS2 Jazzy/rclpy, MAVROS (`mavros_msgs`), Gazebo (`gz-transport13`), stdlib `xml.etree.ElementTree` for OSM parsing, numpy. pytest for tests (existing `src/aerocanyon/test/` suite, no new test framework).

**Spec:** `docs/superpowers/specs/2026-09-03-map-zone-trials-design.md`

## Global Constraints

- `world == 'urban_canyon'` behavior must not change — every existing test in `src/aerocanyon/test/` must stay green after every task.
- No new third-party dependencies — OSM parsing uses stdlib `xml.etree.ElementTree` only (per the spec's ponytail note and this codebase's existing no-new-deps posture).
- `HOME_LAT`/`HOME_LON` = `44.434424990487216, 26.04781615647584` throughout (matches the existing duplicated constants in `run_trial.py` and `controller_node.py` — map_zone's own copy follows the same duplication convention already established between those two, not a new shared module).
- Follow this codebase's existing test style: plain `assert`-based pytest functions, explicit `rclpy.init()`/`rclpy.shutdown()` pairs in `try/finally` for any node-touching test, no fixtures/mocking frameworks beyond `monkeypatch` (already used in `test_run_trial.py`).

---

### Task 1: `frames.latlon_to_ned` (inverse of the existing `ned_to_latlon`)

**Files:**
- Modify: `src/aerocanyon/aerocanyon/frames.py`
- Test: `src/aerocanyon/test/test_frames.py`

**Interfaces:**
- Produces: `frames.latlon_to_ned(lat_deg, lon_deg, home_lat_deg, home_lon_deg) -> (north_m, east_m)` — used by Task 2 (`map_zone_geometry.py`) and Task 6 (`controller_node.py`'s generalized correction target).

- [ ] **Step 1: Write the failing test**

Add to `src/aerocanyon/test/test_frames.py` (add `latlon_to_ned` to the existing `from aerocanyon.frames import (...)` line):

```python
def test_latlon_to_ned_is_the_exact_inverse_of_ned_to_latlon():
    home_lat, home_lon = 44.434424990487216, 26.04781615647584
    north_in, east_in = 123.4, -67.8
    lat, lon = ned_to_latlon([north_in, east_in, 0.0], home_lat, home_lon)
    north_out, east_out = latlon_to_ned(lat, lon, home_lat, home_lon)
    assert north_out == pytest.approx(north_in, abs=1e-6)
    assert east_out == pytest.approx(east_in, abs=1e-6)


def test_latlon_to_ned_is_zero_at_home():
    home_lat, home_lon = 44.434424990487216, 26.04781615647584
    north, east = latlon_to_ned(home_lat, home_lon, home_lat, home_lon)
    assert north == pytest.approx(0.0, abs=1e-9)
    assert east == pytest.approx(0.0, abs=1e-9)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/aerocanyon && python3 -m pytest test/test_frames.py -v -k latlon_to_ned`
Expected: FAIL with `ImportError: cannot import name 'latlon_to_ned'`

- [ ] **Step 3: Write the implementation**

Add to `src/aerocanyon/aerocanyon/frames.py`, directly after `ned_to_latlon`:

```python
def latlon_to_ned(lat_deg, lon_deg, home_lat_deg, home_lon_deg):
    """(lat, lon) degrees -> NED (north, east) metre offset from a home
    point -- the exact inverse of ned_to_latlon above (same flat-earth
    approximation, solved backward)."""
    home_lat_rad = math.radians(home_lat_deg)
    north = math.radians(lat_deg - home_lat_deg) * _EARTH_RADIUS_M
    east = (math.radians(lon_deg - home_lon_deg) * _EARTH_RADIUS_M
            * math.cos(home_lat_rad))
    return north, east
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/aerocanyon && python3 -m pytest test/test_frames.py -v`
Expected: PASS (all tests in the file, not just the new ones)

- [ ] **Step 5: Commit**

```bash
git add src/aerocanyon/aerocanyon/frames.py src/aerocanyon/test/test_frames.py
git commit -m "frames: add latlon_to_ned, the inverse of ned_to_latlon

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

### Task 2: `map_zone_geometry.py` — real buildings from OSM data

**Files:**
- Create: `src/aerocanyon/aerocanyon/map_zone_geometry.py`
- Test: `src/aerocanyon/test/test_map_zone_geometry.py`

**Interfaces:**
- Consumes: `frames.latlon_to_ned` (Task 1); `canyon_geometry.Box` (existing namedtuple, fields `name cx cy sx sy sz`).
- Produces: `map_zone_geometry.BUILDINGS` (module-level `list[Box]`, parsed at import time from the checked-in `map_zone/meshes/map_zone.osm`) and `map_zone_geometry._parse_buildings(osm_path) -> list[Box]` (the testable pure function) — used by Task 3 (`canyon_field.generate_map_zone`).

- [ ] **Step 1: Write the failing test**

Create `src/aerocanyon/test/test_map_zone_geometry.py`:

```python
"""map_zone_geometry parses real building footprints out of OpenStreetMap
XML -- these tests use small synthetic OSM fixtures, not the full
map_zone.osm (144 ways), except for one sanity check against the real
file.
"""
import pathlib
import tempfile

from aerocanyon.map_zone_geometry import BUILDINGS, OSM_PATH, _parse_buildings

_SQUARE_WITH_HEIGHT = """<?xml version="1.0"?>
<osm version="0.6">
 <node id="1" lat="44.4340000" lon="26.0480000"/>
 <node id="2" lat="44.4340000" lon="26.0481000"/>
 <node id="3" lat="44.4341000" lon="26.0481000"/>
 <node id="4" lat="44.4341000" lon="26.0480000"/>
 <way id="10">
  <nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/><nd ref="1"/>
  <tag k="building" v="yes"/>
  <tag k="height" v="15"/>
 </way>
</osm>
"""

_SQUARE_WITH_LEVELS = """<?xml version="1.0"?>
<osm version="0.6">
 <node id="1" lat="44.4340000" lon="26.0480000"/>
 <node id="2" lat="44.4340000" lon="26.0481000"/>
 <node id="3" lat="44.4341000" lon="26.0481000"/>
 <node id="4" lat="44.4341000" lon="26.0480000"/>
 <way id="10">
  <nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/><nd ref="1"/>
  <tag k="building" v="yes"/>
  <tag k="building:levels" v="4"/>
 </way>
</osm>
"""

_NON_BUILDING_WAY = """<?xml version="1.0"?>
<osm version="0.6">
 <node id="1" lat="44.4340000" lon="26.0480000"/>
 <node id="2" lat="44.4340000" lon="26.0481000"/>
 <way id="10">
  <nd ref="1"/><nd ref="2"/>
  <tag k="highway" v="residential"/>
 </way>
</osm>
"""


def _write(tmp_path, xml):
    p = tmp_path / 'test.osm'
    p.write_text(xml)
    return p


def test_parses_a_building_way_with_an_explicit_height(tmp_path):
    boxes = _parse_buildings(_write(tmp_path, _SQUARE_WITH_HEIGHT))
    assert len(boxes) == 1
    assert boxes[0].sz == 15.0
    assert boxes[0].sx > 0.0 and boxes[0].sy > 0.0


def test_falls_back_to_building_levels_times_3m(tmp_path):
    boxes = _parse_buildings(_write(tmp_path, _SQUARE_WITH_LEVELS))
    assert len(boxes) == 1
    assert boxes[0].sz == 12.0  # 4 levels * 3.0m


def test_ignores_ways_not_tagged_building(tmp_path):
    boxes = _parse_buildings(_write(tmp_path, _NON_BUILDING_WAY))
    assert boxes == []


def test_real_osm_file_parses_to_at_least_one_building():
    assert OSM_PATH.exists(), 'map_zone.osm should be checked into the repo'
    boxes = _parse_buildings(OSM_PATH)
    assert len(boxes) == 26  # map_zone.osm has 26 ways tagged building=*
    for b in boxes:
        assert b.sx > 0.0 and b.sy > 0.0 and b.sz > 0.0


def test_module_level_buildings_matches_parsing_the_real_file():
    assert len(BUILDINGS) == len(_parse_buildings(OSM_PATH))
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/aerocanyon && python3 -m pytest test/test_map_zone_geometry.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'aerocanyon.map_zone_geometry'`

- [ ] **Step 3: Write the implementation**

Create `src/aerocanyon/aerocanyon/map_zone_geometry.py`:

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/aerocanyon && python3 -m pytest test/test_map_zone_geometry.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/aerocanyon/aerocanyon/map_zone_geometry.py src/aerocanyon/test/test_map_zone_geometry.py
git commit -m "aerocanyon: parse real map_zone building geometry from OSM data

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

### Task 3: `canyon_field.generate_map_zone` + world-aware `WindGrid.load`/`save`

**Files:**
- Modify: `src/aerocanyon/aerocanyon/canyon_field.py`
- Test: `src/aerocanyon/test/test_canyon_field.py`

**Interfaces:**
- Consumes: `map_zone_geometry.BUILDINGS` (Task 2).
- Produces: `canyon_field.generate_map_zone(nx, ny, nz) -> (field, meta)`; `canyon_field.WindGrid.load(data_dir, world='urban_canyon')` / `.save(data_dir, world='urban_canyon')` — used by Task 4 (`wind_field_node`).

- [ ] **Step 1: Write the failing test**

Add to `src/aerocanyon/test/test_canyon_field.py`:

```python
def test_generate_map_zone_has_the_declared_shape():
    field, meta = cf.generate_map_zone(nx=10, ny=8, nz=6)
    assert field.shape == (10, 8, 6, 3)
    assert meta['shape'] == [10, 8, 6]


def test_generate_map_zone_wind_increases_with_height_away_from_buildings():
    """log_law is still the vertical-profile backbone for map_zone --
    away from any building's recirculation zone, wind should still
    increase with height the same way it does over urban_canyon."""
    field, meta = cf.generate_map_zone(nx=10, ny=10, nz=16)
    grid = cf.WindGrid(field, meta)
    p_far = np.array([200.0, 200.0])  # corner of the grid, away from buildings
    low = np.linalg.norm(grid.at(np.array([p_far[0], p_far[1], cg.GROUND_Z + 5.0])))
    high = np.linalg.norm(grid.at(np.array([p_far[0], p_far[1], cg.GROUND_Z + 50.0])))
    assert high > low


def test_wind_grid_save_and_load_round_trip_per_world(tmp_path):
    field, meta = cf.generate(nx=6, ny=5, nz=4)
    grid = cf.WindGrid(field, meta)
    grid.save(tmp_path, world='urban_canyon')
    grid.save(tmp_path, world='map_zone')
    assert (tmp_path / 'wind_grid.npy').exists(), (
        'urban_canyon keeps the original, un-suffixed filename')
    assert (tmp_path / 'wind_grid_map_zone.npy').exists()

    loaded_uc = cf.WindGrid.load(tmp_path, world='urban_canyon')
    loaded_mz = cf.WindGrid.load(tmp_path, world='map_zone')
    np.testing.assert_array_equal(loaded_uc.field, field)
    np.testing.assert_array_equal(loaded_mz.field, field)


def test_wind_grid_load_defaults_to_urban_canyon(tmp_path):
    field, meta = cf.generate(nx=6, ny=5, nz=4)
    cf.WindGrid(field, meta).save(tmp_path)
    loaded = cf.WindGrid.load(tmp_path)
    np.testing.assert_array_equal(loaded.field, field)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/aerocanyon && python3 -m pytest test/test_canyon_field.py -v`
Expected: FAIL — `generate_map_zone` doesn't exist; `save`/`load` don't accept `world=`.

- [ ] **Step 3: Write the implementation**

In `src/aerocanyon/aerocanyon/canyon_field.py`, generalize `_recirculation` to accept an explicit buildings list (default preserves today's behavior exactly):

```python
def _recirculation(p, buildings=None):
    """Corner separation: a lateral+vertical eddy in each tower's lee.

    Modelled as a rotational contribution centred just downstream of each
    tower, decaying over one building depth. This is what makes the corners
    hazardous and is the disturbance the PINN has to learn.
    """
    v = np.zeros(3)
    for b in (buildings if buildings is not None else cg.BUILDINGS):
        centre = np.array([b.cx + b.sx * 0.75, b.cy, cg.GROUND_Z + b.sz * 0.5])
        r = p - centre
        scale = np.array([b.sx, b.sy, b.sz])
        d2 = float(np.sum((r / scale) ** 2))
        if d2 > 4.0:
            continue
        strength = 3.0 * np.exp(-d2)
        sign = np.sign(b.cy) or 1.0
        v += strength * np.array([-0.4, -sign * 0.5, -0.3])
    return v
```

(This only changes the signature — the body is identical, `buildings=None` falls back to `cg.BUILDINGS` exactly as the hardcoded reference did before, so `generate()`'s existing call site and every existing test are unaffected.)

Add `generate_map_zone` after `generate`:

```python
def generate_map_zone(nx=60, ny=40, nz=24):
    """Wind grid over the real map_zone terrain: log-law vertical
    profile + per-building recirculation from real OSM geometry
    (map_zone_geometry.BUILDINGS) -- no channeling term, unlike
    generate() above, since _channeling assumes urban_canyon's symmetric
    two-row corridor (CANYON_HALF_WIDTH, flow along +x), which doesn't
    exist in a real street layout. See the design spec, part 2.

    +-300m in x/y comfortably covers map_zone.osm's own bounds (~500m x
    180m around the home point). Returns (field, meta), same shape as
    generate()'s own return.
    """
    from . import map_zone_geometry as mz

    xs_lo, xs_hi = -300.0, 300.0
    ys_lo, ys_hi = -300.0, 300.0
    zs_lo, zs_hi = cg.GROUND_Z, cg.GROUND_Z + 100.0

    xs = np.linspace(xs_lo, xs_hi, nx)
    ys = np.linspace(ys_lo, ys_hi, ny)
    zs = np.linspace(zs_lo, zs_hi, nz)

    field = np.zeros((nx, ny, nz, 3))
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            for k, z in enumerate(zs):
                p = np.array([x, y, z])
                agl = z - cg.GROUND_Z
                base = log_law(agl)
                v = np.array([base, 0.0, 0.0]) + _recirculation(p, mz.BUILDINGS)
                field[i, j, k] = v

    meta = {
        'origin': [xs_lo, ys_lo, zs_lo],
        'spacing': [(xs_hi - xs_lo) / (nx - 1),
                    (ys_hi - ys_lo) / (ny - 1),
                    (zs_hi - zs_lo) / (nz - 1)],
        'shape': [nx, ny, nz],
        'u_ref': 10.0,
        'z0': 1.0,
    }
    return field, meta
```

Change `WindGrid.load`/`save` to accept a `world` param, keeping `urban_canyon`'s filenames unchanged for backward compatibility:

```python
def _grid_filenames(world):
    if world == 'urban_canyon':
        return 'wind_grid.npy', 'wind_grid.json'
    return f'wind_grid_{world}.npy', f'wind_grid_{world}.json'
```

```python
    @classmethod
    def load(cls, data_dir, world='urban_canyon'):
        d = pathlib.Path(data_dir)
        npy_name, json_name = _grid_filenames(world)
        field = np.load(d / npy_name)
        meta = json.loads((d / json_name).read_text())
        return cls(field, meta)

    def save(self, data_dir, world='urban_canyon'):
        d = pathlib.Path(data_dir)
        d.mkdir(parents=True, exist_ok=True)
        npy_name, json_name = _grid_filenames(world)
        np.save(d / npy_name, self.field)
        (d / json_name).write_text(json.dumps(self.meta, indent=2))
```

Add a CLI world switch to the module's own `if __name__ == '__main__':` block (check the existing block first and extend it rather than replacing it — it currently just calls `generate()` and `.save()` for the default world). Make it:

```python
if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', choices=('urban_canyon', 'map_zone'), default='urban_canyon')
    args = ap.parse_args()
    field, meta = generate_map_zone() if args.world == 'map_zone' else generate()
    WindGrid(field, meta).save(pathlib.Path(__file__).resolve().parents[3] / 'data', world=args.world)
```

(If the file already ends with a different `__main__` block, replace it with the one above rather than adding a second one.)

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/aerocanyon && python3 -m pytest test/test_canyon_field.py -v`
Expected: PASS (all tests in the file)

- [ ] **Step 5: Generate the actual map_zone wind grid data file**

Run: `cd $HOME/AeroCanyon_Guard && source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 -m aerocanyon.canyon_field --world map_zone`
Expected: writes `data/wind_grid_map_zone.npy` and `data/wind_grid_map_zone.json` — confirm both exist (`ls data/wind_grid_map_zone.*`). This can take a few minutes (60x40x24 grid, each cell looping every map_zone building).

- [ ] **Step 6: Commit**

```bash
git add src/aerocanyon/aerocanyon/canyon_field.py src/aerocanyon/test/test_canyon_field.py src/aerocanyon/data/wind_grid_map_zone.npy src/aerocanyon/data/wind_grid_map_zone.json
git commit -m "canyon_field: generate a map_zone wind field from real OSM buildings

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

### Task 4: world-aware `wind_field_node`

**Files:**
- Modify: `src/aerocanyon/aerocanyon/constants.py`
- Modify: `src/aerocanyon/aerocanyon/wind_field_node.py`
- Test: `src/aerocanyon/test/test_wind_field_node.py`

**Interfaces:**
- Consumes: `canyon_field.WindGrid.load(data_dir, world=...)` (Task 3).
- Produces: `constants.gz_wind_topic(world) -> str`; `WindFieldNode` gains a `world` ROS parameter (default `constants.WORLD_NAME`, i.e. `'urban_canyon'`) — used by Task 8's launch wiring.

- [ ] **Step 1: Write the failing test**

Add to `src/aerocanyon/test/test_wind_field_node.py`:

```python
from aerocanyon import constants as C


def test_gz_wind_topic_is_world_specific():
    assert C.gz_wind_topic('urban_canyon') == '/world/urban_canyon/wind'
    assert C.gz_wind_topic('map_zone') == '/world/map_zone/wind'
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/aerocanyon && python3 -m pytest test/test_wind_field_node.py -v -k gz_wind_topic`
Expected: FAIL with `AttributeError: module 'aerocanyon.constants' has no attribute 'gz_wind_topic'`

- [ ] **Step 3: Write the implementation**

In `src/aerocanyon/aerocanyon/constants.py`, replace the `GZ_WIND_TOPIC` constant (it has exactly one consumer, `wind_field_node.py`, which is about to become world-aware — see the grep confirming this in the design spec's exploration) with a function:

```python
def gz_wind_topic(world):
    """Gazebo wind-plugin topic for a given world name."""
    return f'/world/{world}/wind'
```

(Delete the line `GZ_WIND_TOPIC = f'/world/{WORLD_NAME}/wind'`. `WORLD_NAME` itself stays — it's still the default/`urban_canyon` value used elsewhere.)

In `src/aerocanyon/aerocanyon/wind_field_node.py`, add a `world` parameter and thread it through grid loading and the Gazebo publisher topic. In `__init__`, alongside the other `declare_parameter` calls near the top:

```python
        self.declare_parameter('world', C.WORLD_NAME)
```

Move the `world` read up before `data_dir`/grid loading, and use it:

```python
        self.world = self.get_parameter('world').value

        data_dir = self.get_parameter('data_dir').value
        if not data_dir:
            from ament_index_python.packages import get_package_share_directory
            data_dir = get_package_share_directory('aerocanyon') + '/data'
        self.grid = WindGrid.load(data_dir, world=self.world)
        self.get_logger().info(f'loaded {self.world} wind grid from {data_dir}')
```

And where `self.gz_pub` is created:

```python
        self.gz_pub = self.gz.advertise(C.gz_wind_topic(self.world), Wind)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/aerocanyon && python3 -m pytest test/test_wind_field_node.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/aerocanyon/aerocanyon/constants.py src/aerocanyon/aerocanyon/wind_field_node.py src/aerocanyon/test/test_wind_field_node.py
git commit -m "wind_field_node: make the Gazebo wind topic and grid file world-aware

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

### Task 5: mission-file schema + `controller_node`'s map_zone mission build

**Files:**
- Modify: `src/aerocanyon/aerocanyon/controller_node.py`
- Test: `src/aerocanyon/test/test_controller_node.py`

**Interfaces:**
- Consumes: nothing new from earlier tasks.
- Produces: mission-file JSON schema — a list of `{"command": int, "frame": int, "x_lat": float, "y_long": float, "z_alt": float, "autocontinue": bool}` objects, one per real mission item (no home placeholder) — this is what Task 6 (`dump_mission.py`) writes and Task 8 (`run_trial.py`'s spawn point) reads. `ControllerNode` gains `world`/`mission_file` ROS parameters.

- [ ] **Step 1: Write the failing test**

Add to `src/aerocanyon/test/test_controller_node.py` (near the top, alongside the existing imports — `import json` and `pathlib` will be needed):

```python
import json
import pathlib


def _write_mission_file(tmp_path, items):
    p = tmp_path / 'mission.json'
    p.write_text(json.dumps(items))
    return str(p)


def test_build_mission_for_map_zone_loads_and_replays_the_mission_file(tmp_path):
    items = [
        {'command': 84, 'frame': 3, 'x_lat': 44.4345, 'y_long': 26.0480,
         'z_alt': 25.0, 'autocontinue': True},
        {'command': 16, 'frame': 3, 'x_lat': 44.4348, 'y_long': 26.0490,
         'z_alt': 30.0, 'autocontinue': True},
        {'command': 85, 'frame': 3, 'x_lat': 44.4350, 'y_long': 26.0495,
         'z_alt': 0.0, 'autocontinue': True},
    ]
    mission_file = _write_mission_file(tmp_path, items)

    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.world = 'map_zone'
        node.mission_file = mission_file
        mission = node._build_mission()
        node.destroy_node()
    finally:
        rclpy.shutdown()

    assert len(mission) == 4, (
        'home placeholder (seq 0, always overwritten by ArduPilot -- see '
        'the urban_canyon mission\'s own docstring) + the 3 real items')
    assert mission[1].command == 84 and mission[1].is_current
    assert mission[2].command == 16 and not mission[2].is_current
    assert mission[3].command == 85
    assert mission[3].x_lat == pytest.approx(44.4350)
    assert mission[2].z_alt == pytest.approx(30.0), (
        'map_zone replays each item\'s own captured altitude, not the '
        'fixed CRUISE_ALT_M urban_canyon uses')


def test_build_mission_for_urban_canyon_is_unaffected():
    """world defaults to urban_canyon -- _build_mission's existing
    4-item fixed mission must be byte-for-byte the same as before this
    task's changes."""
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        mission = node._build_mission()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    assert len(mission) == 4
    assert mission[1].command == 84  # NAV_VTOL_TAKEOFF
    assert mission[3].command == 85  # NAV_VTOL_LAND
```

Add `pytest` to the test file's imports if not already present (it is not currently imported in `test_controller_node.py` — check the top of the file and add `import pytest` alongside the existing `import numpy as np` / `import rclpy` lines).

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/aerocanyon && python3 -m pytest test/test_controller_node.py -v -k build_mission`
Expected: FAIL — `node.world`/`node.mission_file` don't exist yet, and `_build_mission` doesn't branch on them.

- [ ] **Step 3: Write the implementation**

In `src/aerocanyon/aerocanyon/controller_node.py`, add imports at the top:

```python
import json
import pathlib
```

In `ControllerNode.__init__`, alongside the existing `declare_parameter('mode', ...)`/`declare_parameter('feedforward_gain', ...)`:

```python
        self.declare_parameter('world', 'urban_canyon')
        self.world = self.get_parameter('world').value
        self.declare_parameter('mission_file', '')
        self.mission_file = self.get_parameter('mission_file').value
```

Replace `_build_mission` with a dispatcher plus the two branches (keep the existing urban_canyon body verbatim, just renamed):

```python
    def _build_mission(self):
        if self.world == 'map_zone':
            return self._build_map_zone_mission()
        return self._build_urban_canyon_mission()

    def _build_urban_canyon_mission(self):
        """[home placeholder, NAV_VTOL_TAKEOFF @ entry, NAV_WAYPOINT @
        landing-trigger point, NAV_VTOL_LAND @ landing-trigger point] --
        all at CRUISE_ALT_M, all in Q-mode/VTOL navigation the whole way
        (QuadPlane::in_vtol_auto() latches true from the takeoff item and
        never auto-clears without an explicit transition command, which
        this project never issues). Landing targets the REAL landing-
        trigger point (last tower row's edge + LAND_CLEARANCE_M), not
        CANYON_EXIT -- see the design spec for why the old 45m margin
        doesn't apply to ArduPilot's own navigation controller.

        Item 0 is a placeholder: ArduPilot always treats mission seq 0
        as the home position and overwrites/ignores whatever is uploaded
        there (confirmed live -- pushing [TAKEOFF, WAYPOINT, LAND]
        starting at seq 0 came back from /mavros/mission/waypoints with
        item 0 silently replaced by an all-zero home entry and the real
        TAKEOFF item dropped). The real mission starts at seq 1."""
        entry_ned = frames.enu_to_ned(cg.CANYON_ENTRY)
        land_ned = np.array([entry_ned[0], LAND_TRIGGER_LOCAL_M, entry_ned[2]])

        def wp(command, ned, is_current=False):
            lat, lon = frames.ned_to_latlon(ned, HOME_LAT, HOME_LON)
            w = Waypoint()
            w.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            w.command = command
            w.is_current = is_current
            w.autocontinue = True
            w.x_lat = lat
            w.y_long = lon
            w.z_alt = CRUISE_ALT_M
            return w

        return [
            wp(16, entry_ned),                    # seq 0: home placeholder, overwritten by ArduPilot
            wp(84, entry_ned, is_current=True),   # MAV_CMD_NAV_VTOL_TAKEOFF
            wp(16, land_ned),                     # MAV_CMD_NAV_WAYPOINT
            wp(85, land_ned),                     # MAV_CMD_NAV_VTOL_LAND
        ]

    def _build_map_zone_mission(self):
        """Replay a mission captured by dump_mission.py from a live
        Mission Planner session (see README/dump_mission.py) verbatim:
        [home placeholder, item 0 (is_current), item 1, ..., item N-1] --
        same seq-0-placeholder shape as _build_urban_canyon_mission, for
        the same reason (ArduPilot always overwrites it). Each item's
        own command/frame/altitude is replayed as captured, unlike
        urban_canyon's fixed CRUISE_ALT_M -- the mission was authored at
        whatever altitudes made sense for the real terrain."""
        items = json.loads(pathlib.Path(self.mission_file).read_text())
        if not items:
            raise ValueError(f'mission file {self.mission_file} has no waypoints')

        def wp(item, is_current=False):
            w = Waypoint()
            w.frame = item['frame']
            w.command = item['command']
            w.is_current = is_current
            w.autocontinue = item['autocontinue']
            w.x_lat = item['x_lat']
            w.y_long = item['y_long']
            w.z_alt = item['z_alt']
            return w

        home_placeholder = wp(items[0])  # content is irrelevant -- ArduPilot overwrites seq 0
        return [home_placeholder] + [
            wp(item, is_current=(i == 0)) for i, item in enumerate(items)
        ]
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/aerocanyon && python3 -m pytest test/test_controller_node.py -v`
Expected: PASS (all tests in the file, including the pre-existing ones)

- [ ] **Step 5: Commit**

```bash
git add src/aerocanyon/aerocanyon/controller_node.py src/aerocanyon/test/test_controller_node.py
git commit -m "controller_node: build the AUTO mission from a captured map_zone file

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

### Task 6: generalize treatment's wind correction to the active mission waypoint

**Files:**
- Modify: `src/aerocanyon/aerocanyon/controller_node.py`
- Test: `src/aerocanyon/test/test_controller_node.py`

**Interfaces:**
- Consumes: `frames.latlon_to_ned` (Task 1).
- Produces: `ControllerNode._active_target() -> (np.array([north, east]), alt_m)` — internal, used by `_treatment_tick` and `_yaw_to_target` (this task only).

- [ ] **Step 1: Write the failing test**

Extend the existing `from aerocanyon.controller_node import (...)` line at
the top of `src/aerocanyon/test/test_controller_node.py` to also pull in
`CRUISE_ALT_M`, `HOME_LAT`, `HOME_LON` (used by the new tests below,
alongside the names it already imports).

Add to `src/aerocanyon/test/test_controller_node.py`:

```python
from mavros_msgs.msg import Waypoint as MavWaypoint
from mavros_msgs.msg import WaypointList


def _waypoint_list(current_seq, items):
    """items: list of (lat, lon, alt) -- builds a WaypointList the way
    MAVROS would report it, home placeholder at seq 0 included (that's
    how the real topic looks -- see WaypointList.msg)."""
    msg = WaypointList()
    msg.current_seq = current_seq
    msg.waypoints = []
    for lat, lon, alt in items:
        w = MavWaypoint()
        w.x_lat = lat
        w.y_long = lon
        w.z_alt = alt
        msg.waypoints.append(w)
    return msg


def test_active_target_defaults_to_the_fixed_urban_canyon_land_point():
    """Before any WaypointList has arrived, _active_target must match
    what _treatment_tick has always corrected -- CRUISE_WP_SEQ's own
    land-trigger point at CRUISE_ALT_M -- so every existing treatment
    test (which never publishes a WaypointList) keeps passing."""
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        target_ned, alt = node._active_target()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    entry_ned = frames.enu_to_ned(cg.CANYON_ENTRY)
    assert target_ned[0] == pytest.approx(entry_ned[0])
    assert target_ned[1] == pytest.approx(LAND_TRIGGER_LOCAL_M)
    assert alt == pytest.approx(CRUISE_ALT_M)


def test_active_target_tracks_the_fcu_reported_current_waypoint():
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node._on_mission_waypoints(_waypoint_list(
            current_seq=2,
            items=[(44.4344, 26.0478, 25.0),   # seq 0: home placeholder
                   (44.4345, 26.0480, 25.0),   # seq 1: takeoff
                   (44.4350, 26.0490, 42.0)]))  # seq 2: active cruise wp
        target_ned, alt = node._active_target()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    expected_north, expected_east = frames.latlon_to_ned(
        44.4350, 26.0490, HOME_LAT, HOME_LON)
    assert target_ned[0] == pytest.approx(expected_north)
    assert target_ned[1] == pytest.approx(expected_east)
    assert alt == pytest.approx(42.0)


def test_treatment_corrects_the_fcu_reported_current_seq_not_a_hardcoded_one():
    """Generalization for map_zone's N-waypoint missions: the correction
    push/restart must target whichever seq MAVROS reports as current,
    not the urban_canyon-only CRUISE_WP_SEQ constant."""
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.mode = 'treatment'
        node.mavros_armed = True
        node._mission_confirmed = True
        node.wind_est = np.array([2.0, 0.0, 0.0])
        node._on_mission_waypoints(_waypoint_list(
            current_seq=5,
            items=[(44.4344, 26.0478, 25.0)] * 6))
        pushes = []
        restarts = []
        node.mission_client.call_async = _confirmed_push(pushes)
        node.set_current_client.call_async = lambda req: restarts.append(req)

        for _ in range(60):
            node._tick()
        node.destroy_node()
    finally:
        rclpy.shutdown()
    assert len(pushes) == 1
    assert pushes[0].start_index == 5
    assert len(restarts) == 1
    assert restarts[0].wp_seq == 5
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/aerocanyon && python3 -m pytest test/test_controller_node.py -v -k "active_target or corrects_the_fcu"`
Expected: FAIL — `_active_target`/`_on_mission_waypoints` don't exist yet.

- [ ] **Step 3: Write the implementation**

In `src/aerocanyon/aerocanyon/controller_node.py`, add `WaypointList` to the `mavros_msgs.msg` import:

```python
from mavros_msgs.msg import OverrideRCIn, State, Waypoint, WaypointList
```

In `__init__`, initialize the new state (near `self._waypoint_offset = np.zeros(2)`):

```python
        self._mission_current_seq = CRUISE_WP_SEQ
        self._mission_waypoints = []
```

and subscribe (near the other `create_subscription` calls, plain QoS depth 10 like `/mavros/state`, not `qos_profile_sensor_data` — this is a latched mission topic, not high-rate sensor data):

```python
        self.create_subscription(
            WaypointList, '/mavros/mission/waypoints', self._on_mission_waypoints, 10)
```

Add the handler and the shared target helper (near `_build_mission`):

```python
    def _on_mission_waypoints(self, msg):
        self._mission_current_seq = msg.current_seq
        self._mission_waypoints = msg.waypoints

    def _active_target(self):
        """(north, east) NED metres and altitude (relative-alt, metres)
        of the mission's currently active nav waypoint -- read from the
        FCU's own reported mission (/mavros/mission/waypoints) once
        available. Before the first WaypointList arrives, falls back to
        the fixed urban_canyon land-trigger point at CRUISE_ALT_M -- the
        same point/seq (CRUISE_WP_SEQ, this class's own default
        _mission_current_seq) _treatment_tick has always corrected, so
        every world behaves exactly as before until MAVROS actually
        reports a mission."""
        if (self._mission_waypoints
                and self._mission_current_seq < len(self._mission_waypoints)):
            wp = self._mission_waypoints[self._mission_current_seq]
            north, east = frames.latlon_to_ned(wp.x_lat, wp.y_long, HOME_LAT, HOME_LON)
            return np.array([north, east]), float(wp.z_alt)
        entry_ned = frames.enu_to_ned(cg.CANYON_ENTRY)
        return np.array([entry_ned[0], LAND_TRIGGER_LOCAL_M]), CRUISE_ALT_M
```

Update `_treatment_tick` to use `_active_target()`/`self._mission_current_seq` instead of the hardcoded `cg.CANYON_ENTRY`/`LAND_TRIGGER_LOCAL_M`/`CRUISE_WP_SEQ`. Replace this block:

```python
            entry_ned = frames.enu_to_ned(cg.CANYON_ENTRY)
            land_ned = np.array([entry_ned[0], LAND_TRIGGER_LOCAL_M, entry_ned[2]])
            corrected_ned = land_ned + np.array(
                [self._waypoint_offset[0], self._waypoint_offset[1], 0.0])
            lat, lon = frames.ned_to_latlon(corrected_ned, HOME_LAT, HOME_LON)

            wp = Waypoint()
            wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            wp.command = 16  # MAV_CMD_NAV_WAYPOINT
            wp.is_current = False
            wp.autocontinue = True
            wp.x_lat = lat
            wp.y_long = lon
            wp.z_alt = CRUISE_ALT_M

            req = WaypointPush.Request()
            req.start_index = CRUISE_WP_SEQ
            req.waypoints = [wp]
```

with:

```python
            target_ned, target_alt = self._active_target()
            corrected_ned = target_ned + self._waypoint_offset
            lat, lon = frames.ned_to_latlon(
                np.array([corrected_ned[0], corrected_ned[1], 0.0]), HOME_LAT, HOME_LON)

            wp = Waypoint()
            wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            wp.command = 16  # MAV_CMD_NAV_WAYPOINT
            wp.is_current = False
            wp.autocontinue = True
            wp.x_lat = lat
            wp.y_long = lon
            wp.z_alt = target_alt

            req = WaypointPush.Request()
            req.start_index = self._mission_current_seq
            req.waypoints = [wp]
```

And the confirmed-push restart just above it (still inside `_treatment_tick`), replace:

```python
                set_current_req.wp_seq = CRUISE_WP_SEQ
```

with:

```python
                set_current_req.wp_seq = self._mission_current_seq
```

Finally, `_yaw_to_target` currently recomputes `entry_ned`/`land_ned` itself. Replace:

```python
        entry_ned = frames.enu_to_ned(cg.CANYON_ENTRY)
        land_ned = np.array([entry_ned[0], LAND_TRIGGER_LOCAL_M, entry_ned[2]])
        target_ned = land_ned + np.array(
            [self._waypoint_offset[0], self._waypoint_offset[1], 0.0])

        d_north = target_ned[0] - self.pos[0]
        d_east = target_ned[1] - self.pos[1]
```

with:

```python
        target_ned, _ = self._active_target()
        target_ned = target_ned + self._waypoint_offset

        d_north = target_ned[0] - self.pos[0]
        d_east = target_ned[1] - self.pos[1]
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/aerocanyon && python3 -m pytest test/test_controller_node.py -v`
Expected: PASS (all tests, including every pre-existing treatment test — `CRUISE_WP_SEQ` stays the default `_mission_current_seq`, so their assertions of `wp_seq == CRUISE_WP_SEQ`/`start_index == CRUISE_WP_SEQ` still hold)

- [ ] **Step 5: Commit**

```bash
git add src/aerocanyon/aerocanyon/controller_node.py src/aerocanyon/test/test_controller_node.py
git commit -m "controller_node: correct whichever waypoint MAVROS reports active

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

### Task 7: `dump_mission.py` — capture a live Mission Planner mission to a file

**Files:**
- Create: `src/aerocanyon/aerocanyon/dump_mission.py`
- Test: `src/aerocanyon/test/test_dump_mission.py`

**Interfaces:**
- Produces: `dump_mission._dump(waypoints) -> list[dict]` (the pure, testable transform — matches the schema Task 5 and Task 8 already consume) and a `python3 -m aerocanyon.dump_mission <name>` CLI that writes `data/missions/<name>.json`.

- [ ] **Step 1: Write the failing test**

Create `src/aerocanyon/test/test_dump_mission.py`:

```python
"""Pure-function coverage for dump_mission's WaypointList -> JSON
transform. No rclpy/MAVROS needed -- see dump_mission._dump's own
docstring for why it's factored out."""
from types import SimpleNamespace

from aerocanyon.dump_mission import _dump


def _wp(command, frame, lat, lon, alt, autocontinue=True):
    return SimpleNamespace(command=command, frame=frame, x_lat=lat,
                           y_long=lon, z_alt=alt, autocontinue=autocontinue)


def test_dump_drops_the_seq_zero_home_placeholder():
    waypoints = [
        _wp(16, 0, 0.0, 0.0, 0.0),  # seq 0: home placeholder
        _wp(84, 3, 44.4345, 26.0480, 25.0),
        _wp(85, 3, 44.4350, 26.0495, 0.0),
    ]
    items = _dump(waypoints)
    assert len(items) == 2
    assert items[0]['command'] == 84
    assert items[1]['command'] == 85


def test_dump_produces_plain_json_serializable_types():
    waypoints = [_wp(16, 0, 0.0, 0.0, 0.0), _wp(84, 3, 44.4345, 26.0480, 25.0)]
    items = _dump(waypoints)
    assert items == [{
        'command': 84, 'frame': 3, 'x_lat': 44.4345, 'y_long': 26.0480,
        'z_alt': 25.0, 'autocontinue': True,
    }]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/aerocanyon && python3 -m pytest test/test_dump_mission.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'aerocanyon.dump_mission'`

- [ ] **Step 3: Write the implementation**

Create `src/aerocanyon/aerocanyon/dump_mission.py`:

```python
"""One-shot capture of the FCU's currently uploaded mission (e.g. drawn
and uploaded live in Mission Planner against a manually-flown map_zone
SITL session -- see README) to a JSON file run_trial/controller_node
replay on every trial leg. Each leg boots a completely fresh, wiped
ArduPilot SITL (run_trial.run_leg), so a live-drawn mission would
otherwise be lost the moment that manual session ends.

Usage (with the manual-flight stack from the README already running,
Mission Planner connected to its gcs_url port and a mission uploaded):
    python3 -m aerocanyon.dump_mission <name>
writes data/missions/<name>.json.
"""
import argparse
import json
import pathlib
import sys
import time

import rclpy
from mavros_msgs.msg import WaypointList
from rclpy.node import Node

MISSIONS_DIR = pathlib.Path(__file__).resolve().parents[3] / 'data' / 'missions'


def _dump(waypoints):
    """WaypointList.waypoints -> the plain JSON-serializable list
    controller_node._build_map_zone_mission replays from. Drops index 0
    -- ArduPilot always owns/overwrites the seq-0 home placeholder (see
    controller_node._build_urban_canyon_mission's own docstring), so
    capturing its content would be meaningless."""
    return [
        {'command': int(w.command), 'frame': int(w.frame),
         'x_lat': float(w.x_lat), 'y_long': float(w.y_long),
         'z_alt': float(w.z_alt), 'autocontinue': bool(w.autocontinue)}
        for w in waypoints[1:]
    ]


class _MissionDumper(Node):
    def __init__(self):
        super().__init__('mission_dumper')
        self.waypoints = None
        self.create_subscription(
            WaypointList, '/mavros/mission/waypoints', self._on_waypoints, 10)

    def _on_waypoints(self, msg):
        self.waypoints = msg.waypoints


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('name', help='mission name -- written to '
                                 'data/missions/<name>.json')
    ap.add_argument('--timeout', type=float, default=10.0,
                    help='seconds to wait for MAVROS to report a mission')
    args = ap.parse_args(argv)

    rclpy.init(args=[])
    try:
        node = _MissionDumper()
        deadline = time.monotonic() + args.timeout
        while node.waypoints is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
        waypoints = node.waypoints
        node.destroy_node()
    finally:
        rclpy.shutdown()

    if not waypoints or len(waypoints) < 2:
        raise SystemExit(
            f'no mission (or an empty one) reported by MAVROS within '
            f'{args.timeout}s -- upload one in Mission Planner first')

    MISSIONS_DIR.mkdir(parents=True, exist_ok=True)
    out_path = MISSIONS_DIR / f'{args.name}.json'
    out_path.write_text(json.dumps(_dump(waypoints), indent=2))
    print(f'wrote {out_path} ({len(waypoints) - 1} waypoints)')


if __name__ == '__main__':
    main(sys.argv[1:])
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/aerocanyon && python3 -m pytest test/test_dump_mission.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/aerocanyon/aerocanyon/dump_mission.py src/aerocanyon/test/test_dump_mission.py
git commit -m "aerocanyon: add dump_mission.py to capture a live Mission Planner mission

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

### Task 8: `run_trial.py` — `--world`/`--mission-file`, spawn point, launch wiring

**Files:**
- Modify: `src/aerocanyon/aerocanyon/run_trial.py`
- Modify: `src/aerocanyon/launch/canyon_sim.launch.py`
- Test: `src/aerocanyon/test/test_run_trial.py`

**Interfaces:**
- Consumes: mission-file schema (Task 5/7); `WaypointList`/mission-file `x_lat`/`y_long` fields; `frames.latlon_to_ned` (Task 1).
- Produces: `run_trial.py --world {urban_canyon,map_zone} --mission-file <path>` CLI; `run_trial._spawn_xyz(world, mission_file) -> (x, y, z)`.

- [ ] **Step 1: Write the failing test**

Add to `src/aerocanyon/test/test_run_trial.py`:

```python
import json

from aerocanyon.run_trial import MAP_ZONE_SPAWN_XYZ, _spawn_xyz


def test_spawn_xyz_is_unchanged_for_urban_canyon():
    assert _spawn_xyz('urban_canyon', '') == SPAWN_XYZ


def test_spawn_xyz_uses_the_documented_default_for_map_zone_with_no_mission():
    assert _spawn_xyz('map_zone', '') == MAP_ZONE_SPAWN_XYZ


def test_spawn_xyz_uses_the_mission_files_first_waypoint(tmp_path):
    mission_file = tmp_path / 'mission.json'
    mission_file.write_text(json.dumps([
        {'command': 84, 'frame': 3, 'x_lat': 44.4345, 'y_long': 26.0480,
         'z_alt': 25.0, 'autocontinue': True},
        {'command': 85, 'frame': 3, 'x_lat': 44.4350, 'y_long': 26.0495,
         'z_alt': 0.0, 'autocontinue': True},
    ]))
    x, y, z = _spawn_xyz('map_zone', str(mission_file))
    expected_north, expected_east = rt.frames.latlon_to_ned(
        44.4345, 26.0480, rt.HOME_LAT, rt.HOME_LON)
    assert x == pytest.approx(expected_east)
    assert y == pytest.approx(expected_north)
    assert z == pytest.approx(cg.GROUND_Z + 0.2)


def test_reset_gazebo_model_uses_the_given_world_and_spawn_point(monkeypatch):
    calls = []

    def fake_request(self, service, request, request_type, response_type, timeout):
        calls.append((service, request))
        return True, Boolean(data=True)

    monkeypatch.setattr('gz.transport13.Node.request', fake_request)
    monkeypatch.setattr('time.sleep', lambda _: None)

    _reset_gazebo_model(world='map_zone', spawn_xyz=(1.0, 2.0, 3.0))

    assert len(calls) == 1
    service, request = calls[0]
    assert service == '/world/map_zone/set_pose'
    assert request.position['x'] == 1.0 and request.position['z'] == 3.0
```

Add the needed imports at the top of `test_run_trial.py` — `import pytest` and `from aerocanyon import canyon_geometry as cg` if not already present (check the existing import block first), plus extend the existing `from aerocanyon.run_trial import (...)` line with `_spawn_xyz` and `MAP_ZONE_SPAWN_XYZ`.

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/aerocanyon && python3 -m pytest test/test_run_trial.py -v -k "spawn_xyz or uses_the_given_world"`
Expected: FAIL — `_spawn_xyz`/`MAP_ZONE_SPAWN_XYZ` don't exist; `_reset_gazebo_model` doesn't accept `world=`/`spawn_xyz=`.

- [ ] **Step 3: Write the implementation**

In `src/aerocanyon/aerocanyon/run_trial.py`, add near the top-level imports:

```python
import json
```

and add `from . import frames` alongside the existing `from . import canyon_geometry as cg` / `from . import constants as C`.

Add, near `SPAWN_XYZ`'s own definition:

```python
# map_zone's documented default spawn (README's manual-flight setup):
# local ENU (0, 0), 0.2m above the terrain's z=74 ground level, same
# convention SPAWN_XYZ above uses for urban_canyon.
MAP_ZONE_SPAWN_XYZ = (0.0, 0.0, cg.GROUND_Z + 0.2)


def _spawn_xyz(world, mission_file):
    """Spawn point for a trial leg. urban_canyon is unaffected (always
    SPAWN_XYZ). map_zone spawns at the mission file's first waypoint if
    one is given (so the vehicle starts where the captured mission
    actually begins), else the documented default spawn above."""
    if world != 'map_zone':
        return SPAWN_XYZ
    if not mission_file:
        return MAP_ZONE_SPAWN_XYZ
    items = json.loads(pathlib.Path(mission_file).read_text())
    north, east = frames.latlon_to_ned(items[0]['x_lat'], items[0]['y_long'],
                                       HOME_LAT, HOME_LON)
    return (east, north, cg.GROUND_Z + 0.2)
```

Change `WORLD_SDF` from a module-level constant to a small helper (it's only consumed inside `_spawn_gazebo`, which becomes world-aware):

```python
def _world_sdf(world):
    return WS / 'src' / 'aerocanyon' / 'worlds' / f'{world}.sdf'
```

(Delete the old `WORLD_SDF = WS / 'src' / 'aerocanyon' / 'worlds' / f'{C.WORLD_NAME}.sdf'` line.)

Update `_spawn_gazebo` to take `world`:

```python
def _spawn_gazebo(world):
    ...  # docstring unchanged
    proc = _spawn(f'gz sim -v 2 {_world_sdf(world)} -r -s',
                  cwd=WS, env=_gazebo_env())
    ...
```

Update `_reset_gazebo_model`/`_recreate_gazebo_model` to take `world`/`spawn_xyz`, defaulting to today's behavior:

```python
def _reset_gazebo_model(world=C.WORLD_NAME, spawn_xyz=SPAWN_XYZ):
    ...  # docstring unchanged
    node = GzNode()
    x, y, z = spawn_xyz
    req = Pose(name=C.MODEL_NAME, position={'x': x, 'y': y, 'z': z},
               orientation={'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0})
    node.request(f'/world/{world}/set_pose', req, Pose, Boolean, 2000)
    time.sleep(1)


def _recreate_gazebo_model(world=C.WORLD_NAME):
    ...  # docstring unchanged
    node = GzNode()
    req = Entity(name=C.MODEL_NAME, type=Entity.MODEL)
    node.request(f'/world/{world}/remove', req, Entity, Boolean, 2000)
    time.sleep(1)
```

Update `run_one` to accept and use `world`/`mission_file`:

```python
def run_one(mode, trial, duration, clean_respawn=False, seed=0, turbulence=2.5,
           ff_gain=0.2, world=C.WORLD_NAME, mission_file=''):
    spawn_xyz = _spawn_xyz(world, mission_file)
    if clean_respawn:
        _recreate_gazebo_model(world)
    else:
        _reset_gazebo_model(world, spawn_xyz)
```

(Leave the rest of `run_one`'s body alone except the launch command, below.)

Update the `ros2 launch` command inside `run_one` to pass `world`/`mission_file`:

```python
    launch_args = (f'mode:={mode} trial:={trial} seed:={seed} '
                   f'turbulence_sigma:={turbulence} feedforward_gain:={ff_gain} '
                   f'world:={world}')
    if mission_file:
        launch_args += f' mission_file:={mission_file}'
    launch = (f'bash -lc "source /opt/ros/jazzy/setup.bash && '
              f'source {WS}/install/setup.bash && '
              f'ros2 launch aerocanyon canyon_sim.launch.py {launch_args}"')
    nodes = _spawn(launch, cwd=WS)
```

Update `run_leg` to accept and forward `world`/`mission_file`:

```python
def run_leg(mode, trial, duration, seed=0, turbulence=2.5, ff_gain=0.2,
           world=C.WORLD_NAME, mission_file=''):
    ...  # docstring unchanged
    for attempt in range(MAX_STALL_RETRIES + 1):
        gz = _spawn_gazebo(world)
        bridge = _spawn_web_bridge()
        try:
            return run_one(mode, trial, duration, seed=seed, turbulence=turbulence,
                           ff_gain=ff_gain, world=world, mission_file=mission_file)
        except _LegStalled as e:
            ...  # unchanged
        finally:
            ...  # unchanged
```

Update `main()`: add the two new flags and validate/forward them:

```python
    ap.add_argument('--world', choices=('urban_canyon', 'map_zone'),
                    default='urban_canyon',
                    help='which Gazebo world to fly the trial in')
    ap.add_argument('--mission-file', default='', dest='mission_file',
                    help='JSON mission (see dump_mission.py) to fly for '
                         '--world map_zone -- required in that case')
    args = ap.parse_args()

    if args.world == 'map_zone' and not args.mission_file:
        raise SystemExit(
            '--world map_zone requires --mission-file <path> -- capture '
            'one first with `python3 -m aerocanyon.dump_mission <name>` '
            '(see README)')

    if args.mode:
        run_leg(args.mode, args.trial, args.duration, seed=args.seed,
                turbulence=args.turbulence, ff_gain=args.ff_gain,
                world=args.world, mission_file=args.mission_file)
        return
```

And the subprocess re-invocation list a bit further down:

```python
        cmd = [sys.executable, '-m', 'aerocanyon.run_trial',
               '--mode', mode, '--trial', args.trial,
               '--duration', str(args.duration), '--seed', str(args.seed),
               '--turbulence', str(args.turbulence),
               '--ff-gain', str(args.ff_gain),
               '--world', args.world]
        if args.mission_file:
            cmd += ['--mission-file', args.mission_file]
        result = subprocess.run(cmd, cwd=WS)
```

Finally, in `src/aerocanyon/launch/canyon_sim.launch.py`, add the two new launch args and pass them through to `wind_field_node`/`controller_node`:

```python
    world = LaunchConfiguration('world')
    mission_file = LaunchConfiguration('mission_file')
```

(add alongside the other `LaunchConfiguration(...)` lines), and:

```python
        DeclareLaunchArgument('world', default_value='urban_canyon',
                              description='urban_canyon or map_zone'),
        DeclareLaunchArgument('mission_file', default_value='',
                              description='JSON mission path (map_zone '
                                          'only -- see dump_mission.py)'),
```

(add alongside the other `DeclareLaunchArgument(...)` entries), and add `'world': world` to the `wind_field_node` Node's `parameters=[{...}]` dict, and `'world': world, 'mission_file': mission_file` to the `controller_node` Node's `parameters=[{...}]` dict.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/aerocanyon && python3 -m pytest test/test_run_trial.py -v`
Expected: PASS (all tests, including the pre-existing `_reset_gazebo_model`/`_verify_sitl_started` ones — the new params default to today's exact values)

- [ ] **Step 5: Commit**

```bash
git add src/aerocanyon/aerocanyon/run_trial.py src/aerocanyon/launch/canyon_sim.launch.py src/aerocanyon/test/test_run_trial.py
git commit -m "run_trial: add --world/--mission-file, wire world through spawn and launch

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

### Task 9: README — document the map_zone trial flow

**Files:**
- Modify: `README.md`

**Interfaces:** None (documentation only).

- [ ] **Step 1: Update the Phase 2 gap note**

In the `## What's not here yet (Phase 2)` section, remove this bullet (it's now done):

```
- A wind field regenerated for the `map_zone` terrain (the existing
  `canyon_field.py`/wind-grid pipeline targets the synthetic
  `urban_canyon.sdf` box-canyon geometry).
```

- [ ] **Step 2: Add a "Run a trial against map_zone" subsection**

Add a new subsection right after `### Run the paired baseline/treatment trial` (before `## View the figures`):

```markdown
### Run a trial against map_zone instead

`run_trial.py` can fly baseline/treatment against the real Bucharest
terrain (`map_zone_ap.sdf`) instead of the synthetic canyon, using a
mission you draw yourself:

1. Fly manually against `map_zone` (the ["Fly the tricopter
   manually"](#fly-the-tricopter-manually) setup above, unchanged).
2. Connect Mission Planner to the same GCS MAVLink port MAVROS already
   opens (`tcp://127.0.0.1:5761`), draw a mission, and upload it.
3. Capture it to a file (from a new terminal, same workspace sourced):
   ```bash
   python3 -m aerocanyon.dump_mission my_mission
   ```
   writes `data/missions/my_mission.json`.
4. Stop the manual-flight terminals (Gazebo/SITL/MAVROS) -- `run_trial.py`
   owns its own instances of all three, same as it always has for
   `urban_canyon`.
5. Run the trial:
   ```bash
   python3 -m aerocanyon.run_trial --trial map_zone_run \
       --world map_zone --mission-file data/missions/my_mission.json
   ```

The wind field for `map_zone` (`data/wind_grid_map_zone.npy`) is
generated from real building footprints in `map_zone/meshes/map_zone.osm`
-- regenerate it with `python3 -m aerocanyon.canyon_field --world
map_zone` if that source file ever changes. Unlike `urban_canyon`, the
CBF obstacle barrier does not see these real buildings yet (only wind
does) -- see the design spec's "Out of scope" section.
```

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "README: document running baseline/treatment trials against map_zone

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>"
```

---

## Final check

- [ ] Run the full test suite once more from a clean tree: `cd src/aerocanyon && python3 -m pytest test/ -v` — every test (old and new) must pass.
- [ ] Confirm `data/wind_grid_map_zone.npy`/`.json` are committed (Task 3, step 5-6).
- [ ] Spot check `map_zone_geometry.BUILDINGS` positions against the real Gazebo world once, live (per that module's docstring note) before trusting the generated wind field for an actual flight.
