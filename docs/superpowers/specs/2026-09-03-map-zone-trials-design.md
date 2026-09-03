# map_zone baseline/treatment trials — design

2026-09-03

## Problem

`run_trial.py` only ever flies the synthetic `urban_canyon.sdf` world: a
fixed two-tower-row corridor (`canyon_geometry.BUILDINGS`), a fixed
`CANYON_ENTRY`/`CANYON_EXIT` mission, and a wind field
(`canyon_field.py`) generated from that same tower geometry.

The README already flags the real Bucharest terrain world
(`worlds/map_zone_ap.sdf`, currently only used for manual hand-flying)
as a known Phase 2 gap: no wind field, no automated mission, no
baseline/treatment support. This spec closes that gap: `run_trial` gains
a `--world {urban_canyon,map_zone}` flag, map_zone gets its own wind
field generated from real OSM building data, and a mission for map_zone
is authored once in Mission Planner, captured to a file, and replayed
automatically by both trial legs.

`urban_canyon` behavior is unchanged throughout — every new code path is
additive, gated on `world == 'map_zone'`.

## 1. World selection threading

`constants.WORLD_NAME` and `constants.GZ_WIND_TOPIC` are currently
module-level constants baked at import time. Since both worlds now
coexist in one process tree, `GZ_WIND_TOPIC` becomes a function of the
world name instead:

```python
def gz_wind_topic(world):
    return f'/world/{world}/wind'
```

`run_trial.py` gains `--world {urban_canyon,map_zone}` (default
`urban_canyon`). It:
- picks `WORLD_SDF` from the flag instead of `C.WORLD_NAME`
- passes `world` as a ROS param (`-p world:=...`) to both
  `controller_node` and `wind_field_node`
- uses the flag (not `C.WORLD_NAME`) in `_reset_gazebo_model`/
  `_recreate_gazebo_model`'s `/world/{...}/...` gz-transport service
  calls

`HOME_LAT`/`HOME_LON`/`GROUND_Z` are already shared between both worlds
(confirmed live: both worlds' ground raised to z=74 to match
`--home`) — no per-world frame math needed.

## 2. map_zone wind field

`canyon_field.py`'s `_channeling()` assumes the synthetic world's
symmetric two-row corridor (`CANYON_HALF_WIDTH`, flow along +x) and
cannot generalize to a real street layout — it stays urban_canyon-only.
`log_law()` (vertical profile) and `_recirculation()` (per-building
corner eddies, already just a loop over whatever `Box` list it's given)
both generalize directly.

New module `map_zone_geometry.py`:
- parses `map_zone/meshes/map_zone.osm` (stdlib `xml.etree`) for `way`s
  tagged `building=*`
- resolves each way's node refs to lat/lon, converts to ENU via a new
  `frames.latlon_to_ned` (inverse of the existing `ned_to_latlon`) +
  `frames.ned_to_enu`, using `controller_node.HOME_LAT/HOME_LON` as the
  origin (already shared with map_zone's world file)
- takes the axis-aligned bounding box of each way's footprint as
  `(cx, cy, sx, sy)`
- height: `height` tag if present, else `building:levels` tag × 3.0m,
  else a fixed default (9.0m, ~3-storey)
- emits a `BUILDINGS` list of `canyon_geometry.Box`, same shape as the
  synthetic world's, so `_recirculation()` needs no changes to accept it

`canyon_field.py` gains `generate_map_zone()`: same grid-construction
loop as `generate()`, but `field = log_law(...) + _recirculation_for(p,
map_zone_geometry.BUILDINGS)` (channeling term omitted). Written to a
new `data/wind_grid_map_zone.npy` (separate file — `generate()`'s
existing `data/wind_grid.npy` output is untouched).

`WindGrid.load(data_dir, world)` picks the file by world name.
`wind_field_node` reads its `world` param (from part 1) and passes it
through to `WindGrid.load` and `gz_wind_topic`.

## 3. Mission authoring (Mission Planner) + capture

Each `run_trial` leg boots a completely fresh, wiped ArduPilot SITL
(`run_leg`/`run_one`), so a mission drawn live in a GCS does not survive
into the next leg on its own — it has to be captured to a file and
replayed on every leg's boot.

Flow:
1. Fly manually against `map_zone` (existing README manual-flight
   setup, `worlds/map_zone_ap.sdf` + SITL + MAVROS with `gcs_url`) and
   connect Mission Planner to the existing GCS MAVLink TCP port (5761)
   — same port already opened for QGroundControl. Draw and upload a
   mission there (Mission Planner talks the MAVLink mission protocol
   directly to ArduPilot; MAVROS is not involved in the upload).
2. New script `dump_mission.py`: subscribes to MAVROS's
   `/mavros/mission/waypoints` (latched `WaypointList`, which MAVROS
   populates by pulling from the FCU on connect/change) and writes the
   current mission to `data/missions/<name>.json` — a plain list of
   `{lat, lon, alt, command}` per waypoint (mission items only, item 0's
   home placeholder excluded — see part 4).

## 4. controller_node generalization

`_build_mission()` currently hardcodes 4 items built from
`cg.CANYON_ENTRY` and a computed land-trigger point. It becomes:
- `world == 'urban_canyon'`: unchanged, exactly today's 4-item mission.
- `world == 'map_zone'`: load `--mission-file`'s JSON, build
  `[home placeholder, VTOL_TAKEOFF@wp0, WAYPOINT@wp1, ...,
  WAYPOINT@wpN-2, VTOL_LAND@wpN-1]` — first waypoint is the takeoff
  point, last is the landing point, same structural shape as today's
  fixed mission just with N points instead of 2.

Treatment's correction (`_treatment_tick`) currently nudges a hardcoded
`CRUISE_WP_SEQ=2` — the single cruise waypoint in the fixed mission.
That doesn't generalize to N waypoints: it must correct whichever
waypoint is *currently active*. `ControllerNode` subscribes to
`/mavros/mission/waypoints` (`WaypointList`, has `current_seq`) and uses
that instead of the `CRUISE_WP_SEQ` constant in both worlds (for
`urban_canyon` this is always seq 2 anyway, so behavior there is
unchanged — this is a strict generalization, not a behavior change).

Module-level constants computed at import time from `cg.CANYON_ENTRY`/
`cg.BUILDINGS` (`_LAST_TOWER_EDGE_ENU_X`, `LAND_TRIGGER_LOCAL_M`) stay
as-is for `urban_canyon`; `_yaw_to_target()`'s target point becomes "the
active waypoint's position" (read from the same `WaypointList`
subscription) instead of the hardcoded `land_ned` expression, in both
worlds.

`run_trial.py`'s `SPAWN_XYZ`/`_reset_gazebo_model` spawn point: for
`map_zone`, use the mission file's first waypoint if `--mission-file` is
given, else the README's documented default map_zone spawn
(z = GROUND_Z + 0.2, same as today's manual-flight instructions).

## Out of scope

- Editing an existing map_zone mission file (re-run steps 3.1–3.2 to
  replace it).
- CBF obstacle avoidance against the real OSM buildings in map_zone —
  `cbf_filter.py`'s barrier still only sees `canyon_geometry.BUILDINGS`
  (urban_canyon). Wiring `map_zone_geometry.BUILDINGS` into the CBF is a
  separate follow-up, not required for baseline/treatment wind-response
  trials to run.
- VTOL forward-flight transition — unaffected by this change, stays
  disabled (`ENABLE_VTOL_TRANSITION = False`) in both worlds per
  existing Phase 2 scope.
