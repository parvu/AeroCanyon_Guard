# AUTO-Mode Mission Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace `controller_node.py`'s hand-rolled RC-override P/D outer loop (which just drifted into a canyon tower under wind) with ArduPilot's own AUTO-mode mission navigation.

**Architecture:** Upload a 3-item mission (`NAV_VTOL_TAKEOFF` → `NAV_WAYPOINT` → `NAV_VTOL_LAND`, all at the existing landing-trigger point rather than the further-out `CANYON_EXIT`) via MAVROS's `WaypointPush` service, arm, switch to `AUTO`, and let ArduPilot's own navigation controller fly it. Treatment mode periodically re-pushes the cruise waypoint with a small, clamped position offset accumulated from the unchanged CBF-filtered PINN correction.

**Tech Stack:** ROS2 Jazzy (rclpy), MAVROS (`mavros_msgs/WaypointPush`, `mavros_msgs/Waypoint`), ArduPilot QuadPlane AUTO mode.

**Spec:** [docs/superpowers/specs/2026-09-02-auto-mission-navigation-design.md](../specs/2026-09-02-auto-mission-navigation-design.md)

## Global Constraints

- `rc_pwm.py` and `control_server.py`'s manual-flight RC-override path stay completely untouched.
- `run_trial.py`'s per-leg lifecycle and `/mavros/extended_state` landing detection stay untouched.
- The CBF safety filter (`cbf_filter.py`) itself is unchanged -- only how its output reaches the vehicle changes.
- Mission altitude is 25m (matches `canyon_geometry.CANYON_ENTRY`'s existing NED altitude), `FRAME_GLOBAL_RELATIVE_ALT` (relative to home, not absolute MSL).
- Mission command IDs (confirmed via pymavlink, not guessed): `NAV_WAYPOINT=16`, `NAV_VTOL_TAKEOFF=84`, `NAV_VTOL_LAND=85`. ArduPlane's `AUTO` custom mode number is `10` (confirmed against `ArduPlane/mode.h`).
- Always fresh-restart the whole stack before trusting a live verification; never kill an individual Gazebo child process.

---

## Task 1: `frames.py` — NED to global lat/lon conversion

**Files:**
- Modify: `src/aerocanyon/aerocanyon/frames.py`
- Test: `src/aerocanyon/test/test_frames.py`

**Interfaces:**
- Produces: `frames.ned_to_latlon(ned, home_lat_deg, home_lon_deg) -> (lat_deg, lon_deg)`. Task 3 calls this to build mission waypoints and treatment's periodic offset updates.

- [ ] **Step 1: Write the failing tests**

```python
# Append to src/aerocanyon/test/test_frames.py
from aerocanyon.frames import ned_to_latlon


def test_ned_to_latlon_zero_offset_returns_home():
    lat, lon = ned_to_latlon(np.array([0.0, 0.0, 0.0]), 44.0, 26.0)
    assert lat == pytest.approx(44.0, abs=1e-9)
    assert lon == pytest.approx(26.0, abs=1e-9)


def test_ned_to_latlon_north_offset_increases_latitude():
    lat, lon = ned_to_latlon(np.array([100.0, 0.0, 0.0]), 44.0, 26.0)
    assert lat > 44.0
    assert lon == pytest.approx(26.0, abs=1e-6)


def test_ned_to_latlon_east_offset_increases_longitude():
    lat, lon = ned_to_latlon(np.array([0.0, 100.0, 0.0]), 44.0, 26.0)
    assert lon > 26.0
    assert lat == pytest.approx(44.0, abs=1e-6)


def test_ned_to_latlon_matches_known_scale():
    # 1 degree of latitude is ~111,320 m -- a 111.32 m north offset should
    # read back within a small fraction of a degree of 0.001 deg latitude.
    lat, lon = ned_to_latlon(np.array([111.32, 0.0, 0.0]), 0.0, 0.0)
    assert lat == pytest.approx(0.001, rel=1e-2)
```

(`import pytest` and `import numpy as np` already present at the top of `test_frames.py`.)

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd src/aerocanyon && python3 -m pytest test/test_frames.py -k ned_to_latlon -v`
Expected: FAIL with `ImportError: cannot import name 'ned_to_latlon'`

- [ ] **Step 3: Implement**

Append to `src/aerocanyon/aerocanyon/frames.py`:

```python
_EARTH_RADIUS_M = 6378137.0  # WGS84 equatorial radius


def ned_to_latlon(ned, home_lat_deg, home_lon_deg):
    """NED [north, east, down] offset from a home point -> (lat, lon)
    degrees. Flat-earth/local-tangent-plane approximation -- accurate to
    sub-centimetre at this project's scale (canyon spans ~250m), the
    same approximation ArduPilot's own EKF uses internally for local NED
    <-> global conversion at this scale. `down` is unused -- altitude is
    handled separately via mission items' own relative-altitude field,
    not folded into this conversion."""
    north, east, _down = np.asarray(ned, dtype=float)
    home_lat_rad = math.radians(home_lat_deg)
    lat = home_lat_deg + math.degrees(north / _EARTH_RADIUS_M)
    lon = home_lon_deg + math.degrees(
        east / (_EARTH_RADIUS_M * math.cos(home_lat_rad)))
    return lat, lon
```

Add `import math` to `frames.py`'s top if not already present (check first — it currently only imports `numpy`).

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd src/aerocanyon && python3 -m pytest test/test_frames.py -v`
Expected: PASS (all tests, including the pre-existing ones)

- [ ] **Step 5: Commit**

```bash
git add src/aerocanyon/aerocanyon/frames.py src/aerocanyon/test/test_frames.py
git commit -m "aerocanyon: add NED -> lat/lon conversion for mission waypoint upload"
```

---

## Task 2: `controller_node.py` — mission construction and upload

**Files:**
- Modify: `src/aerocanyon/aerocanyon/controller_node.py`

**Interfaces:**
- Consumes: `frames.ned_to_latlon` (Task 1).
- Produces: `ControllerNode._build_mission() -> list[Waypoint]`, `ControllerNode.mission_client` (a `WaypointPush` service client). Task 3/4 build on this.

- [ ] **Step 1: Replace the landing-trigger constants' role**

`LAND_TRIGGER_LOCAL_M`/`LAND_CLEARANCE_M`/`_LAST_TOWER_EDGE_ENU_X` (module level, unchanged) now describe the mission's real cruise/landing target instead of a runtime position check -- keep them exactly as-is (same values, same derivation), just note in a one-line comment that they're now consumed by `_build_mission()` below instead of a per-tick position comparison.

- [ ] **Step 2: Add mission construction**

Add to the imports:
```python
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush
```

Add a module-level constant (near `LAND_TRIGGER_LOCAL_M`):
```python
# Matches the README's --home value -- the ArduPilot EKF origin/home
# point this project's SITL runs boot with. Mission waypoints are
# uploaded as lat/lon relative to THIS point (frames.ned_to_latlon),
# not the vehicle's own local frame -- MAVROS's WaypointPush takes
# global coordinates, not local NED offsets.
HOME_LAT, HOME_LON = 44.434424990487216, 26.04781615647584
CRUISE_ALT_M = 25.0  # matches canyon_geometry.CANYON_ENTRY's NED altitude
```

Add a method to `ControllerNode` (near `__init__`):
```python
    def _build_mission(self):
        """[NAV_VTOL_TAKEOFF @ entry, NAV_WAYPOINT @ landing-trigger point,
        NAV_VTOL_LAND @ landing-trigger point] -- all at CRUISE_ALT_M,
        all in Q-mode/VTOL navigation the whole way (QuadPlane::in_vtol_auto()
        latches true from the takeoff item and never auto-clears without
        an explicit transition command, which this project never issues).
        Landing targets the REAL landing-trigger point (last tower row's
        edge + LAND_CLEARANCE_M), not CANYON_EXIT -- see the design spec
        for why the old 45m margin doesn't apply to ArduPilot's own
        navigation controller."""
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
            wp(84, entry_ned, is_current=True),   # MAV_CMD_NAV_VTOL_TAKEOFF
            wp(16, land_ned),                     # MAV_CMD_NAV_WAYPOINT
            wp(85, land_ned),                     # MAV_CMD_NAV_VTOL_LAND
        ]
```

(Command IDs `84`/`16`/`85` are inlined as plain integers with a comment naming the MAV_CMD constant, rather than importing pymavlink into this module just for three constants -- matches this file's existing style of inlining well-documented magic numbers, e.g. `MODE_QHOVER = 18` in `rc_pwm.py`.)

In `__init__`, add the mission service client alongside the existing `arm_client`/`cmd_client`:
```python
        self.mission_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.mission_uploaded = False
```

- [ ] **Step 2: Verify it imports cleanly**

Run: `cd src/aerocanyon && python3 -c "import aerocanyon.controller_node"`
Expected: no `ImportError`. `_tick` still references the old PD-loop/RC-override logic at this point -- Task 3 replaces it. Commit anyway so the diff stays reviewable in pieces.

- [ ] **Step 3: Commit**

```bash
git add src/aerocanyon/aerocanyon/controller_node.py
git commit -m "aerocanyon: controller_node builds and can upload an AUTO mission (WIP, not yet wired into _tick)"
```

---

## Task 3: `controller_node.py` — replace the outer loop with mission upload + AUTO engage

**Files:**
- Modify: `src/aerocanyon/aerocanyon/controller_node.py`

**Interfaces:**
- Consumes: `ControllerNode._build_mission` (Task 2), `rc_pwm.set_mode`/`arm` (unchanged).
- Produces: nothing new for Task 4 beyond what's already there -- this completes the baseline-mode flight path.

- [ ] **Step 1: Rewrite `_tick`**

Replace the whole `_tick` method (from `def _tick(self):` through its end, and the module-level `LAND_TRIGGER_LOCAL_M` position-gate block inside it) with:

```python
    def _tick(self):
        if not self.mission_uploaded:
            since_start = self.tick - SETPOINTS_BEFORE_OFFBOARD
            if since_start >= 0 and since_start % ENGAGE_RETRY_TICKS == 0:
                req = WaypointPush.Request()
                req.start_index = 0
                req.waypoints = self._build_mission()
                self.mission_client.call_async(req)
                self.get_logger().info('requested mission upload')
            # Optimistic -- a real ack-based state machine is Task 5's
            # test-covered concern; this project's existing arm/engage
            # retry pattern (below) already tolerates a request landing
            # on a not-yet-ready FCU by simply asking again.
            self.mission_uploaded = True

        engaged = self.mavros_armed
        since_stream_started = self.tick - SETPOINTS_BEFORE_OFFBOARD - ENGAGE_RETRY_TICKS
        if (not engaged and since_stream_started >= 0
                and since_stream_started % ENGAGE_RETRY_TICKS == 0):
            set_mode(self.cmd_client, MODE_AUTO)
            arm(self.arm_client, True)
            self.get_logger().info('requested AUTO mode and arm')

        if self.mode == 'treatment':
            self._treatment_tick()

        self.tick += 1
```

Add `MODE_AUTO = 10` near the top of the file (module level, alongside `SETPOINTS_BEFORE_OFFBOARD`) with a comment: `# ArduPlane custom mode number, confirmed against ArduPlane/mode.h`.

Delete the now-unused `_lean_from_accel`, `_yaw_from_quat`, `_wrap_pi` helpers, the `cruise_yaw`/`mission = Mission()` setup in `__init__` (the `Mission` import becomes unused -- remove it), and the `rc_pub`/`desired_pub` publishers and `OverrideRCIn` import (no longer used by this file -- `control_server.py`'s own copy is untouched). Leave `self.pos`/`self.vel`/`self.quat`/`self.wind_est`/`self.wind_truth` and their subscriptions exactly as they are -- Task 4's treatment correction still needs them.

Add a placeholder `_treatment_tick` for now (Task 4 fills it in):
```python
    def _treatment_tick(self):
        pass  # Task 4
```

- [ ] **Step 2: Run the full test suite (expect failures -- Task 5 rewrites the tests)**

Run: `cd src/aerocanyon && python3 -m pytest test/test_controller_node.py -v`
Expected: FAIL -- the existing tests reference deleted methods/behavior. This is expected; Task 5 rewrites them. Do not skip ahead and "fix" tests here.

- [ ] **Step 3: Commit**

```bash
git add src/aerocanyon/aerocanyon/controller_node.py
git commit -m "aerocanyon: controller_node flies via AUTO mission upload instead of an RC-override P/D loop (tests not yet updated)"
```

---

## Task 4: `controller_node.py` — treatment-mode periodic waypoint-offset correction

**Files:**
- Modify: `src/aerocanyon/aerocanyon/controller_node.py`

**Interfaces:**
- Consumes: `CBFFilter.filter` (unchanged), `frames.ned_to_latlon` (Task 1).
- Produces: `ControllerNode._accumulate_offset(u_safe, dt, current_offset, max_offset_m) -> new_offset` (pure function, no rclpy -- Task 5 tests it directly).

- [ ] **Step 1: Write the failing test for the offset accumulation math**

```python
# Add to src/aerocanyon/test/test_controller_node.py
def test_accumulate_offset_zero_correction_stays_zero():
    off = ControllerNode._accumulate_offset(
        np.zeros(3), dt=1.0, current_offset=np.zeros(2), max_offset_m=3.0)
    assert np.allclose(off, [0.0, 0.0])


def test_accumulate_offset_grows_with_sustained_correction():
    off = np.zeros(2)
    for _ in range(5):
        off = ControllerNode._accumulate_offset(
            np.array([1.0, 0.0, 0.0]), dt=1.0, current_offset=off, max_offset_m=100.0)
    assert off[0] > 0.0


def test_accumulate_offset_clamps_to_max_magnitude():
    off = np.zeros(2)
    for _ in range(1000):
        off = ControllerNode._accumulate_offset(
            np.array([50.0, 0.0, 0.0]), dt=1.0, current_offset=off, max_offset_m=3.0)
    assert np.linalg.norm(off) <= 3.0 + 1e-6
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd src/aerocanyon && python3 -m pytest test/test_controller_node.py -k accumulate_offset -v`
Expected: FAIL with `AttributeError`

- [ ] **Step 3: Implement**

Add to `constants.py`:
```python
MAX_WAYPOINT_OFFSET_M = 3.0  # clamp on treatment's cumulative waypoint nudge
CORRECTION_UPDATE_HZ = 1.0   # how often treatment re-pushes the cruise waypoint
```

Add to `ControllerNode` (as a `@staticmethod`, matching `_lean_from_accel`'s old pattern):
```python
    @staticmethod
    def _accumulate_offset(u_safe, dt, current_offset, max_offset_m):
        """Kinematic displacement over one update interval (0.5*a*dt^2),
        added to the running offset and clamped to max_offset_m -- so a
        runaway correction can't push the mission waypoint somewhere
        unsafe. Horizontal (NED north/east) only; altitude stays flown
        by the mission's own fixed CRUISE_ALT_M."""
        delta = 0.5 * np.asarray(u_safe[:2], dtype=float) * dt * dt
        new_offset = np.asarray(current_offset, dtype=float) + delta
        mag = np.linalg.norm(new_offset)
        if mag > max_offset_m:
            new_offset = new_offset * (max_offset_m / mag)
        return new_offset
```

Replace the placeholder `_treatment_tick` with:
```python
    def _treatment_tick(self):
        u_des = -self.ff_gain * self.wind_est / MASS_KG
        u_safe, info = self.cbf.filter(u_des, self.pos, self.vel,
                                       self.wind_truth, self.quat)

        diag = Vector3Stamped()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.vector.x = 1.0 if info['active'] else 0.0
        diag.vector.y = float(np.clip(info['h_obstacle'], -1e3, 1e3))
        diag.vector.z = 0.0 if info['feasible'] else 1.0
        self.cbf_pub.publish(diag)

        self._waypoint_offset = self._accumulate_offset(
            u_safe, 1.0 / C.CONTROL_HZ, self._waypoint_offset,
            C.MAX_WAYPOINT_OFFSET_M)

        since_last_push = self.tick - self._last_offset_push_tick
        if since_last_push >= C.CONTROL_HZ / C.CORRECTION_UPDATE_HZ:
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
            req.start_index = 1
            req.waypoints = [wp]
            self.mission_client.call_async(req)
            self._last_offset_push_tick = self.tick
```

Add `self._waypoint_offset = np.zeros(2)` and `self._last_offset_push_tick = 0` to `__init__`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd src/aerocanyon && python3 -m pytest test/test_controller_node.py -k accumulate_offset -v`
Expected: PASS (3 tests)

- [ ] **Step 5: Commit**

```bash
git add src/aerocanyon/aerocanyon/controller_node.py src/aerocanyon/aerocanyon/constants.py src/aerocanyon/test/test_controller_node.py
git commit -m "aerocanyon: treatment mode injects its CBF correction as a clamped waypoint offset"
```

---

## Task 5: Full test suite rewrite for `controller_node.py`

**Files:**
- Modify: `src/aerocanyon/test/test_controller_node.py`

**Interfaces:** none new -- this brings the full suite back to passing.

- [ ] **Step 1: Rewrite the arm/engage/mission tests**

Replace the file's `_run_ticks` helper and the tests that used `arm_calls`/`mode_calls` to check `QHOVER`/`QLAND` requests with equivalents checking `MODE_AUTO` (10) and a mission-upload call, mocking `controller_node.arm`, `controller_node.set_mode`, and `node.mission_client.call_async` the same way `arm`/`set_mode` were mocked before (module-attribute monkeypatching, since `controller_node.py` does `from .rc_pwm import arm, set_mode`, binding those names into its own module namespace):

```python
"""Regression coverage for controller_node's AUTO-mission control path."""
import numpy as np
import rclpy

from aerocanyon.controller_node import (ENGAGE_RETRY_TICKS, MODE_AUTO,
                                        SETPOINTS_BEFORE_OFFBOARD,
                                        ControllerNode)


def _run_ticks(mode, n, armed=False):
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.mode = mode
        node.mavros_armed = armed
        arm_calls = []
        mode_calls = []
        mission_pushes = []
        import aerocanyon.controller_node as cn
        cn.arm = lambda client, value: arm_calls.append(value)
        cn.set_mode = lambda client, mode: mode_calls.append(mode)
        node.mission_client.call_async = lambda req: mission_pushes.append(req)

        for _ in range(n):
            node._tick()
        node.destroy_node()
        return arm_calls, mode_calls, mission_pushes
    finally:
        rclpy.shutdown()


def test_uploads_a_mission_before_requesting_auto_and_arm():
    arm_calls, mode_calls, pushes = _run_ticks(
        'baseline', SETPOINTS_BEFORE_OFFBOARD + 1)
    assert len(pushes) == 1, 'must upload the mission exactly once'
    assert pushes[0].start_index == 0
    assert len(pushes[0].waypoints) == 3
    assert arm_calls == [] and mode_calls == [], (
        'must not request AUTO/arm on the same tick as the mission upload')


def test_requests_auto_and_arm_after_the_mission_upload():
    arm_calls, mode_calls, _ = _run_ticks(
        'baseline', SETPOINTS_BEFORE_OFFBOARD + ENGAGE_RETRY_TICKS + 1)
    assert mode_calls == [MODE_AUTO]
    assert arm_calls == [True]


def test_retries_arm_request_until_engaged():
    arm_calls, _, _ = _run_ticks(
        'baseline', SETPOINTS_BEFORE_OFFBOARD + 3 * ENGAGE_RETRY_TICKS + 1)
    assert len(arm_calls) == 3


def test_stops_retrying_once_mavros_reports_armed():
    arm_calls, _, _ = _run_ticks(
        'baseline', SETPOINTS_BEFORE_OFFBOARD + 3 * ENGAGE_RETRY_TICKS + 1, armed=True)
    assert arm_calls == []


def test_treatment_mode_publishes_cbf_diagnostics_and_survives_the_tick_loop():
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.mode = 'treatment'
        node.mavros_armed = True
        diags = []
        real_publish = node.cbf_pub.publish
        node.cbf_pub.publish = lambda msg: (diags.append(msg), real_publish(msg))[0]
        node.mission_client.call_async = lambda req: None
        import aerocanyon.controller_node as cn
        cn.arm = lambda client, value: None
        cn.set_mode = lambda client, mode: None
        for _ in range(SETPOINTS_BEFORE_OFFBOARD + 5):
            node._tick()
        assert len(diags) >= 1
        node.destroy_node()
    finally:
        rclpy.shutdown()
```

(The `test_lean_from_accel_*` and `test_lands_in_place_*` tests from before are deleted -- `_lean_from_accel` no longer exists, and landing is now the mission's own `NAV_VTOL_LAND` item, not a Python-side position check. The `_accumulate_offset` tests from Task 4 stay in this file, appended above or below this block -- either position is fine, just don't duplicate them.)

- [ ] **Step 2: Run the full test suite**

Run: `cd src/aerocanyon && python3 -m pytest test/ -v`
Expected: PASS, all tests

- [ ] **Step 3: Commit**

```bash
git add src/aerocanyon/test/test_controller_node.py
git commit -m "aerocanyon: rewrite controller_node's test suite for AUTO-mission navigation"
```

---

## Task 6: End-to-end manual verification

**Files:** none (verification only)

**Interfaces:** none.

- [ ] **Step 1: Run the full automated test suite**

Run: `cd src/aerocanyon && source /opt/ros/jazzy/setup.bash && python3 -m pytest test/ -v`
Expected: PASS, all tests

- [ ] **Step 2: Fresh full-stack restart, watch one baseline leg live**

Per this project's established discipline: kill every existing Gazebo/SITL/MAVROS/control_server/ros2-launch process first (`ps -eo pid,cmd | grep -E "gz sim|arduplane|mavros_node|ros2 launch|controller_node|trial_logger|wind_field_node|fo_pinn_node"`, `kill -9` each -- **including the `ros2 launch` parent and its child nodes**, not just the obvious Gazebo/SITL/MAVROS processes; a prior session incident left orphaned `wind_field_node`/`fo_pinn_node`/`controller_node`/`trial_logger` processes running because only the more obvious processes were killed, causing two overlapping mission runs to fight each other and break the browser viewer), confirm nothing is left running, then:

```bash
cd $HOME/AeroCanyon_Guard
source /opt/ros/jazzy/setup.bash && source install/setup.bash && source .venv/bin/activate
python3 -m aerocanyon.run_trial --trial auto_verify --mode baseline --duration 90 --seed 1
```

Watch `http://localhost:8080` live while it runs. Expected: the vehicle takes off, climbs to ~25m, flies toward the landing-trigger point staying visibly within the canyon corridor (not drifting toward a tower), and lands. This is this sub-project's actual exit criterion -- if it drifts into a tower again, this plan's design (not just its implementation) needs revisiting, not another tuning pass.

- [ ] **Step 3: Inspect the trial CSV for a clean, on-track flight**

```bash
python3 -c "
import pandas as pd
df = pd.read_csv('trials/auto_verify_baseline.csv')
print(df[['t','x','y','z']].describe())
print('max lateral (x) deviation:', df.x.abs().max())
assert df.x.abs().max() < 15.0, 'drifted further laterally than a tower half-width -- investigate before trusting this'
print('OK')
"
```

- [ ] **Step 4: Run one treatment leg and confirm the waypoint-offset mechanism doesn't destabilize it**

```bash
python3 -m aerocanyon.run_trial --trial auto_verify --mode treatment --duration 90 --seed 1
```

Watch it live too. Expected: similar clean flight, with the CBF diagnostic (`cbf_active`) showing real activity if wind pushes the vehicle near a barrier, and no visible oscillation or erratic waypoint-chasing from the periodic offset updates.

If both legs look clean, this sub-project is done -- report back before running any larger batch (49-trial sweep, further retraining) on top of this.
