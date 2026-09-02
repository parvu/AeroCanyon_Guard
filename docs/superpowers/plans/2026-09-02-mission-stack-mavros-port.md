# Mission-Stack MAVROS Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the CBF/PINN mission stack (`controller_node.py`, `trial_logger.py`, `fo_pinn_node.py`, `run_trial.py`) off the now-deleted `px4_msgs`/PX4 onto MAVROS/ArduPilot, so `run_trial.py` can fly a baseline leg end to end again.

**Architecture:** `controller_node.py` gains a new outer position/altitude/heading control loop (PD toward the mission target, or the existing CBF-filtered PINN feedforward in treatment mode) that outputs RC-override PWM on `/mavros/rc/override`, since ArduPilot exposes no MAVLink XY position/velocity injection path for this airframe. A new `frames.py` quaternion/rate conversion (MAVROS's ENU-world/FLU-body -> this project's existing NED-world/FRD-body convention) and a new shared `rc_pwm.py` module (PWM mapping + arm/mode helpers, extracted from `control_server.py`) are the two pieces of new shared infrastructure everything else builds on.

**Tech Stack:** ROS2 Jazzy (rclpy), MAVROS (`mavros_msgs`), ArduPilot SITL (`arduplane`), numpy, existing `cbf_filter.py`/`mission.py`/`canyon_geometry.py` (untouched).

**Spec:** [docs/superpowers/specs/2026-09-02-mission-stack-mavros-port-design.md](../specs/2026-09-02-mission-stack-mavros-port-design.md)

## Global Constraints

- `trial_logger.py`'s `COLUMNS` list and CSV schema must not change — `train_pinn.py`/`plot_results.py` read it by name.
- `canyon_geometry.py`, `constants.py`, `mission.py`, `cbf_filter.py` are untouched — pure math/geometry with no PX4 dependency.
- `control_server.py`'s own runtime behavior must not change from the `rc_pwm.py` extraction — pure refactor, verified by its existing `test_control_server.py` still passing unmodified.
- `ENABLE_VTOL_TRANSITION` stays `False` — VTOL transition is sub-project 4, out of scope here.
- Every task's new pure-function logic (frame conversion, PWM mapping, lean-angle math) gets a unit test with no rclpy/live-SITL dependency, mirroring the existing `test_control_server.py`/`test_frames.py` (if any) pattern.
- Always fresh-restart the *whole* stack (never kill a single Gazebo child process) before trusting a live verification.
- `system_id:=255` is required on every MAVROS launch in this project — a wrong/default system id silently drops RC overrides with no visible error.

---

## Task 1: Frame conversion — MAVROS's ENU/FLU convention to this project's NED/FRD convention

**Files:**
- Modify: `src/aerocanyon/aerocanyon/frames.py`
- Test: `src/aerocanyon/test/test_frames.py` (new file)

**Interfaces:**
- Consumes: nothing new (pure numpy).
- Produces: `frames.quat_mul(q1, q2) -> np.ndarray[4]`, `frames.enu_flu_quat_to_ned_frd(q) -> np.ndarray[4]` (quaternion `[w,x,y,z]`), `frames.enu_flu_rate_to_ned_frd(v) -> np.ndarray[3]` (angular velocity or body-frame linear acceleration, `[x,y,z]`). Later tasks (3, 6) call these on MAVROS's `/mavros/imu/data` orientation/angular_velocity/linear_acceleration fields before handing them to `cbf_filter.py`/`fo_pinn.py`, both of which assume NED-world/FRD-body (PX4's convention, unchanged from before this port).

- [ ] **Step 1: Write the failing tests**

MAVROS publishes orientation as a world-ENU-to-body-FLU quaternion and rates in body-FLU. This project's existing math (`cbf_filter.py`, the trained PINN's state vector) assumes PX4's NED-world/FRD-body convention. The standard conversion (used internally by MAVROS itself, see `mavros/mavros/src/lib/ftf_frame_conversions.cpp`) sandwiches the orientation between two fixed 180° quaternions: `NED_ENU_Q = (w=0, x=0.70710678, y=0.70710678, z=0)` (world) and `AIRCRAFT_BASELINK_Q = (w=0, x=1, y=0, z=0)` (body). Rates only need the body-frame axis flip (FLU `[x,y,z]` -> FRD `[x,-y,-z]`), no rotation matrix.

```python
# src/aerocanyon/test/test_frames.py
"""Pure-function tests for the ENU/FLU (MAVROS) <-> NED/FRD (this
project's, and PX4's) convention conversion. No rclpy/ROS needed.
"""
import numpy as np

from aerocanyon.frames import (enu_flu_quat_to_ned_frd,
                               enu_flu_rate_to_ned_frd, quat_mul)


def test_quat_mul_identity():
    q = np.array([0.7071, 0.0, 0.0, 0.7071])
    identity = np.array([1.0, 0.0, 0.0, 0.0])
    result = quat_mul(q, identity)
    np.testing.assert_allclose(result, q, atol=1e-6)


def test_quat_mul_two_90_degree_yaws_make_a_180():
    # 90 deg yaw (about ENU +z / NED -z, doesn't matter which -- pure
    # quaternion algebra) composed with itself twice is a 180 deg yaw.
    q90 = np.array([np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)])
    q180 = quat_mul(q90, q90)
    expected = np.array([np.cos(np.pi / 2), 0.0, 0.0, np.sin(np.pi / 2)])
    np.testing.assert_allclose(np.abs(q180), np.abs(expected), atol=1e-6)


def test_enu_flu_level_nose_east_matches_ned_frd_level_nose_east():
    # Physical orientation must not change, only its numeric
    # representation. "Level, nose pointing east" in ENU/FLU is a 90 deg
    # yaw about ENU +z (body FLU +x from ENU +x/east to ENU... actually
    # nose-east in ENU means body +x aligned with world +x, i.e. the
    # IDENTITY quaternion, since ENU's own +x axis IS east). In NED, east
    # is world +y, so "nose east" there is a +90 deg yaw about NED +z (a
    # quaternion with real part cos(45deg), z part sin(45deg)).
    q_enu_flu_identity = np.array([1.0, 0.0, 0.0, 0.0])
    q_ned_frd = enu_flu_quat_to_ned_frd(q_enu_flu_identity)
    expected = np.array([np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)])
    # Quaternions double-cover rotations (q and -q are the same
    # orientation) -- compare up to sign.
    assert (np.allclose(q_ned_frd, expected, atol=1e-6)
            or np.allclose(q_ned_frd, -expected, atol=1e-6))


def test_enu_flu_rate_to_ned_frd_flips_y_and_z_only():
    v = np.array([1.0, 2.0, 3.0])
    result = enu_flu_rate_to_ned_frd(v)
    np.testing.assert_allclose(result, [1.0, -2.0, -3.0])
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd src/aerocanyon && python3 -m pytest test/test_frames.py -v`
Expected: FAIL with `ImportError: cannot import name 'enu_flu_quat_to_ned_frd'`

- [ ] **Step 3: Implement**

Add to `src/aerocanyon/aerocanyon/frames.py` (append after the existing `body_z_in_ned` function):

```python
# ENU/FLU (MAVROS/ROS convention: world East-North-Up, body
# Forward-Left-Up) <-> NED/FRD (this project's convention throughout --
# PX4's, unchanged since this port only touches where telemetry comes
# from, not how mission.py/cbf_filter.py/fo_pinn.py interpret it).
#
# Sourced from MAVROS's own internal conversion (mavros/src/lib/
# ftf_frame_conversions.cpp: NED_ENU_Q, AIRCRAFT_BASELINK_Q) rather than
# derived from scratch -- this exact sandwich transform is already
# proven correct in MAVROS's own NED-convention topics, and a
# from-scratch derivation is exactly the kind of stray-sign-flip risk
# this file's own docstring warns about.
_NED_ENU_Q = np.array([0.0, 0.70710678, 0.70710678, 0.0])       # world: 180 deg about (1,1,0)/sqrt(2)
_AIRCRAFT_BASELINK_Q = np.array([0.0, 1.0, 0.0, 0.0])            # body: 180 deg about x


def quat_mul(q1, q2):
    """Hamilton product of two [w, x, y, z] quaternions."""
    w1, x1, y1, z1 = np.asarray(q1, dtype=float)
    w2, x2, y2, z2 = np.asarray(q2, dtype=float)
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def enu_flu_quat_to_ned_frd(q):
    """MAVROS's world-ENU/body-FLU orientation quaternion [w,x,y,z] ->
    this project's world-NED/body-FRD convention."""
    return quat_mul(quat_mul(_NED_ENU_Q, np.asarray(q, dtype=float)),
                    _AIRCRAFT_BASELINK_Q)


def enu_flu_rate_to_ned_frd(v):
    """Body-frame FLU rate (angular velocity or linear acceleration,
    [x,y,z]) -> body-frame FRD. No rotation matrix needed -- this is a
    body-axis relabelling (forward stays forward; left/up flip to
    right/down), not a world-frame rotation."""
    x, y, z = np.asarray(v, dtype=float)
    return np.array([x, -y, -z])
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd src/aerocanyon && python3 -m pytest test/test_frames.py -v`
Expected: PASS (4 tests)

- [ ] **Step 5: Commit**

```bash
git add src/aerocanyon/aerocanyon/frames.py src/aerocanyon/test/test_frames.py
git commit -m "aerocanyon: add MAVROS ENU/FLU -> NED/FRD frame conversion"
```

---

## Task 2: `rc_pwm.py` — extract PWM mapping and arm/mode helpers from `control_server.py`

**Files:**
- Create: `src/aerocanyon/aerocanyon/rc_pwm.py`
- Modify: `web_viewer/control_server.py`
- Test: `src/aerocanyon/test/test_rc_pwm.py` (new file)

**Interfaces:**
- Consumes: nothing new.
- Produces: `rc_pwm.RC_CENTER=1500`, `rc_pwm.RC_SPAN=500`, `rc_pwm.THROTTLE_MID=1450`, `rc_pwm.THROTTLE_SPAN=450`, `rc_pwm.MODE_QHOVER=18`, `rc_pwm.MODE_QLAND=20`, `rc_pwm.MAV_CMD_DO_SET_MODE=176`, `rc_pwm.pwm(value, scale, invert=False) -> int`, `rc_pwm.pwm_throttle(value, scale) -> int`, `rc_pwm.resolve_stick(stick, stick_time, now, timeout) -> dict`, `rc_pwm.arm(client, value)` (takes a `mavros_msgs.srv.CommandBool` rclpy client), `rc_pwm.set_mode(client, mode)` (takes a `mavros_msgs.srv.CommandLong` rclpy client). Task 5 (`controller_node.py`) imports all of these.

- [ ] **Step 1: Write the failing tests**

```python
# src/aerocanyon/test/test_rc_pwm.py
"""Pure-function tests for rc_pwm's PWM mapping -- mirrors
web_viewer/test_control_server.py's existing coverage, since this is
that same logic after extraction. No rclpy needed for these three
functions (arm/set_mode need a live client and are exercised via
controller_node's own tests instead, same as control_server's manual
verification)."""
from aerocanyon.rc_pwm import (RC_CENTER, RC_SPAN, THROTTLE_MID,
                               THROTTLE_SPAN, pwm, pwm_throttle,
                               resolve_stick)


def test_pwm_centres_at_zero():
    assert pwm(0.0, 1.0) == RC_CENTER


def test_pwm_full_deflection_hits_the_span_edge():
    assert pwm(1.0, 1.0) == RC_CENTER + RC_SPAN
    assert pwm(-1.0, 1.0) == RC_CENTER - RC_SPAN


def test_pwm_invert_flips_sign():
    assert pwm(1.0, 1.0, invert=True) == RC_CENTER - RC_SPAN


def test_pwm_throttle_uses_its_own_range_not_rc_span():
    assert pwm_throttle(0.0, 1.0) == THROTTLE_MID
    assert pwm_throttle(1.0, 1.0) == THROTTLE_MID + THROTTLE_SPAN
    assert pwm_throttle(-1.0, 1.0) == THROTTLE_MID - THROTTLE_SPAN


def test_pwm_never_exceeds_its_band_however_large_the_scale():
    for scale in (1.0, 3.0, 100.0):
        assert RC_CENTER - RC_SPAN <= pwm(1.0, scale) <= RC_CENTER + RC_SPAN
        assert (THROTTLE_MID - THROTTLE_SPAN <= pwm_throttle(1.0, scale)
                <= THROTTLE_MID + THROTTLE_SPAN)


def test_resolve_stick_zeros_a_stale_axis_only():
    live = {'yaw': 0.5, 'throttle': 0.5, 'roll': 0.5, 'pitch': 0.5}
    fresh = {k: 10.0 for k in live}
    stale = dict(fresh, pitch=5.0)
    resolved = resolve_stick(live, stale, now=10.1, timeout=0.3)
    assert resolved['pitch'] == 0.0
    assert resolved['yaw'] == 0.5 and resolved['roll'] == 0.5
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd src/aerocanyon && python3 -m pytest test/test_rc_pwm.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'aerocanyon.rc_pwm'`

- [ ] **Step 3: Implement**

Create `src/aerocanyon/aerocanyon/rc_pwm.py`:

```python
"""RC-override PWM mapping and MAVROS arm/mode helpers, shared between
web_viewer/control_server.py (manual flight) and controller_node.py
(autonomous mission control) -- both drive the vehicle through the same
/mavros/rc/override channel, since ArduPilot exposes no MAVLink XY
position/velocity injection path for this airframe in any flight mode
(see docs/superpowers/specs/2026-09-02-mission-stack-mavros-port-design.md).

Extracted from control_server.py, verified live there during Phase 1 --
this is not new/unverified logic, just given a second caller.
"""
# RC override PWM: 1500 centre, +/-500 at full stick deflection, the
# standard 1000-2000 us band ArduPilot's RCn_MIN/MAX default to.
RC_CENTER = 1500
RC_SPAN = 500
# Throttle's own PWM range: caps at 1900, not 2000 (RC_CENTER + RC_SPAN),
# matching a real Mode 2 transmitter's ratcheted throttle gimbal.
THROTTLE_MID = 1450
THROTTLE_SPAN = 450
# ArduPlane custom mode numbers (ArduPlane/mode.h).
MODE_QHOVER = 18
MODE_QLAND = 20
MAV_CMD_DO_SET_MODE = 176
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1.0


def resolve_stick(stick, stick_time, now, timeout):
    """Per-axis dead-man's-switch: an axis whose last update is older
    than `timeout` reads as 0.0 regardless of its last commanded value."""
    return {
        name: (0.0 if now - stick_time[name] > timeout else stick[name])
        for name in stick
    }


def pwm(value, scale, invert=False):
    """A [-1, 1] command axis -> RC_CENTER +/- RC_SPAN PWM."""
    v = max(-1.0, min(1.0, value * scale))
    return int(round(RC_CENTER + (-v if invert else v) * RC_SPAN))


def pwm_throttle(value, scale):
    """A [-1, 1] throttle command -> THROTTLE_MID +/- THROTTLE_SPAN PWM."""
    v = max(-1.0, min(1.0, value * scale))
    return int(round(THROTTLE_MID + v * THROTTLE_SPAN))


def arm(client, value):
    """Request arm (value=True) or disarm (value=False) over an already-
    constructed mavros_msgs.srv.CommandBool rclpy client
    (create_client(CommandBool, '/mavros/cmd/arming'))."""
    from mavros_msgs.srv import CommandBool
    req = CommandBool.Request()
    req.value = value
    client.call_async(req)


def set_mode(client, mode):
    """Request a custom-mode switch (e.g. MODE_QHOVER) over an already-
    constructed mavros_msgs.srv.CommandLong rclpy client
    (create_client(CommandLong, '/mavros/cmd/command')). Goes through
    COMMAND_LONG/DO_SET_MODE rather than /mavros/set_mode -- this
    airframe heartbeats as MAV_TYPE_VTOL_TILTROTOR (21), which is absent
    from MAVROS's ArduPilot mode tables, so /mavros/set_mode always
    returns mode_sent=False for it (verified live in Phase 1)."""
    from mavros_msgs.srv import CommandLong
    req = CommandLong.Request()
    req.command = MAV_CMD_DO_SET_MODE
    req.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    req.param2 = float(mode)
    client.call_async(req)
```

Then update `web_viewer/control_server.py` to import from it instead of defining its own copies. Replace lines 64-105 (the `RC_CENTER` through `COMMAND_COMMANDS` constant block) and the `resolve_stick`/`stick_to_rc` function definitions (lines 108-143) with:

```python
import sys
sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parents[1]
                       / 'src' / 'aerocanyon'))
from aerocanyon.rc_pwm import (MAV_CMD_DO_SET_MODE,
                               MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODE_QHOVER,
                               MODE_QLAND, RC_CENTER, RC_SPAN, THROTTLE_MID,
                               THROTTLE_SPAN, arm, pwm, pwm_throttle,
                               resolve_stick, set_mode)

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
CONTROL_HZ = 50
STICK_TIMEOUT_S = 0.3
RC_PRESENT_TIMEOUT_S = 1.0
COMMAND_COMMANDS = {'arm', 'disarm', 'land'}


def stick_to_rc(stick, scale):
    """Mode 2 stick state -> (roll, pitch, throttle, yaw) PWM for RC
    channels 1-4. Pitch is the one inverted axis -- see rc_pwm.pwm's
    caller here for why."""
    return (pwm(stick['roll'], scale), pwm(stick['pitch'], scale, invert=True),
            pwm_throttle(stick['throttle'], scale), pwm(stick['yaw'], scale))
```

And replace `WebControlNode._arm`/`_set_mode` (lines 208-218) with calls to the shared helpers -- in `apply_command` (around line 201), change:
```python
            self._set_mode(MODE_QHOVER)
            self._arm(True)
        elif cmd == 'disarm':
            self._arm(False)
        elif cmd == 'land':
            self._set_mode(MODE_QLAND)
```
to:
```python
            set_mode(self.cmd_client, MODE_QHOVER)
            arm(self.arm_client, True)
        elif cmd == 'disarm':
            arm(self.arm_client, False)
        elif cmd == 'land':
            set_mode(self.cmd_client, MODE_QLAND)
```
and delete the now-unused `_arm`/`_set_mode` methods entirely.

(This is an import-path workaround, not a proper package dependency, since `web_viewer/` is not itself a ROS2 package and doesn't have `aerocanyon` as an installed dependency -- acceptable for now since Phase 1 already established `web_viewer/` as a standalone script directory; flag this in the task's self-review, don't over-engineer a proper packaging fix here.)

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd src/aerocanyon && python3 -m pytest test/test_rc_pwm.py -v`
Expected: PASS (6 tests)

Then confirm the extraction didn't change `control_server.py`'s behavior:
Run: `cd web_viewer && source /opt/ros/jazzy/setup.bash && python3 test_control_server.py`
Expected: `ok` (same as before this task — this test file's own assertions still hold since the underlying PWM math didn't change, only where it lives)

- [ ] **Step 5: Commit**

```bash
git add src/aerocanyon/aerocanyon/rc_pwm.py src/aerocanyon/test/test_rc_pwm.py web_viewer/control_server.py
git commit -m "aerocanyon: extract rc_pwm.py from control_server.py for controller_node to share"
```

---

## Task 3: `controller_node.py` — MAVROS telemetry subscriptions

**Files:**
- Modify: `src/aerocanyon/aerocanyon/controller_node.py`

**Interfaces:**
- Consumes: `frames.enu_to_ned`, `frames.enu_flu_quat_to_ned_frd` (Task 1).
- Produces: `ControllerNode.pos`/`.vel` (NED, `np.ndarray[3]`, unchanged shape/meaning from before), `ControllerNode.quat` (NED/FRD `[w,x,y,z]`, unchanged shape/meaning), `ControllerNode.armed`/`.mavros_state` (new: tracks MAVROS's own connection/mode state). Task 4 and Task 5 build the rest of `_tick` on top of these.

- [ ] **Step 1: Replace the PX4 subscriptions**

In `src/aerocanyon/aerocanyon/controller_node.py`, replace the import block (lines 9-23):

```python
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, CommandLong
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from . import canyon_geometry as cg
from . import constants as C
from . import frames
from .cbf_filter import CBFFilter
from .constants import MASS_KG
from .mission import Mission
from .rc_pwm import (MODE_QHOVER, MODE_QLAND, arm, pwm, pwm_throttle,
                     resolve_stick, set_mode)
```

Replace the publisher/subscriber block in `__init__` (lines 195-216) with:

```python
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')

        self.mavros_connected = False
        self.mavros_armed = False

        self.create_subscription(
            State, '/mavros/state', self._on_state, 10)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._on_pose, qos_profile_sensor_data)
        self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local',
            self._on_velocity, qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/mavros/imu/data', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Vector3Stamped, C.TOPIC_WIND_EST, self._on_wind_est, 10)
        self.create_subscription(
            Vector3Stamped, C.TOPIC_WIND_TRUTH, self._on_wind_truth, 10)
```

Replace the telemetry callback methods (`_on_position`, `_on_status`, `_on_attitude`, lines 220-232) with:

```python
    def _on_state(self, msg):
        self.mavros_connected = msg.connected
        self.mavros_armed = msg.armed

    def _on_pose(self, msg):
        p = msg.pose.position
        self.pos = frames.enu_to_ned([p.x, p.y, p.z])

    def _on_velocity(self, msg):
        v = msg.twist.linear
        self.vel = frames.enu_to_ned([v.x, v.y, v.z])

    def _on_imu(self, msg):
        q = msg.orientation
        self.quat = frames.enu_flu_quat_to_ned_frd([q.w, q.x, q.y, q.z])
```

(`_on_wind_est`/`_on_wind_truth` stay exactly as they are — unrelated to PX4/MAVROS.)

Delete `_send_command` and `_publish_offboard_mode` (lines 237-258) — Task 4/5 replace their callers.

- [ ] **Step 2: Verify it imports cleanly**

Run: `cd src/aerocanyon && python3 -c "import aerocanyon.controller_node"`
Expected: no `ImportError` (the file is not yet functionally complete — `_tick` still references the now-deleted methods/PX4 messages at this point in the plan; Task 4/5 finish it. This step only confirms the new import block and telemetry callbacks are syntactically and reference-correct in isolation.)

Since `_tick` isn't finished yet, this task has no independent test run beyond the import check — Task 5 is where `controller_node.py` becomes test-passing again. Commit anyway so the diff stays reviewable in small pieces; the branch is expected to not fully pass tests again until Task 5 lands.

- [ ] **Step 3: Commit**

```bash
git add src/aerocanyon/aerocanyon/controller_node.py
git commit -m "aerocanyon: controller_node reads telemetry from MAVROS instead of px4_msgs (WIP, not yet functional)"
```

---

## Task 4: `controller_node.py` — new outer position/altitude/heading control loop

**Files:**
- Modify: `src/aerocanyon/aerocanyon/controller_node.py`

**Interfaces:**
- Consumes: `ControllerNode.pos`/`.vel`/`.quat` (Task 3, NED/FRD), `CBFFilter.filter` (unchanged, existing), `frames.quat_to_rotmat` (existing).
- Produces: `ControllerNode._lean_from_accel(accel_ned, quat) -> (roll, pitch)` (pure method, radians), `ControllerNode._tick`'s new RC-override publish. Task 5 wires arm/mode/land around this.

- [ ] **Step 1: Write the failing test for the lean-angle math (pure function, no rclpy)**

```python
# Add to src/aerocanyon/test/test_controller_node.py, near the top,
# after the existing imports:
import math

from aerocanyon.controller_node import ControllerNode


def test_lean_from_accel_zero_is_level():
    roll, pitch = ControllerNode._lean_from_accel(
        np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]))
    assert abs(roll) < 1e-9 and abs(pitch) < 1e-9


def test_lean_from_accel_forward_accel_pitches_forward():
    # NED +x = north = "forward" at zero yaw (identity quat, nose north).
    # Positive forward acceleration must produce a positive (nose-down,
    # ArduPilot's own sign) pitch command.
    roll, pitch = ControllerNode._lean_from_accel(
        np.array([2.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]))
    assert pitch > 0.0
    assert abs(roll) < 1e-9


def test_lean_from_accel_rightward_accel_rolls_right():
    # NED +y = east = "right" at zero yaw.
    roll, pitch = ControllerNode._lean_from_accel(
        np.array([0.0, 2.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]))
    assert roll > 0.0
    assert abs(pitch) < 1e-9
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd src/aerocanyon && python3 -m pytest test/test_controller_node.py -k lean_from_accel -v`
Expected: FAIL with `AttributeError: type object 'ControllerNode' has no attribute '_lean_from_accel'`

- [ ] **Step 3: Implement the control loop**

Add to `ControllerNode` in `controller_node.py` (as a `@staticmethod`, callable without a live node instance — hence the tests above pass `None` as `self`):

```python
    @staticmethod
    def _lean_from_accel(accel_ned, yaw_quat):
        """Desired horizontal NED acceleration -> (roll, pitch) body-frame
        lean angles, small-angle. Rotates the horizontal acceleration into
        the body frame using current yaw only (not full 3D attitude --
        QHOVER's own attitude controller handles roll/pitch response to
        this lean command; this loop only needs to know which way is
        'forward' right now)."""
        g = 9.81
        # Yaw from the NED/FRD quaternion: standard yaw-from-quaternion
        # for [w,x,y,z], body-to-world.
        w, x, y, z = yaw_quat
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        ax_body = accel_ned[0] * math.cos(yaw) + accel_ned[1] * math.sin(yaw)
        ay_body = -accel_ned[0] * math.sin(yaw) + accel_ned[1] * math.cos(yaw)
        pitch = math.atan2(ax_body, g)
        roll = math.atan2(ay_body, g)
        return roll, pitch
```

Replace the whole `_tick` method body from `sp = TrajectorySetpoint()` onward (former lines 339-378) with:

```python
        if self.mode == 'baseline':
            # PD toward the mission target -- PX4's own position controller
            # used to compute this; ArduPilot exposes no equivalent
            # injection path for this airframe (see the spec), so this
            # project now provides it.
            pos_err = target - self.pos
            u_des = C.POSITION_KP * pos_err - C.POSITION_KD * self.vel
        else:
            u_des = -self.ff_gain * self.wind_est / MASS_KG

        u_safe, info = self.cbf.filter(u_des, self.pos, self.vel,
                                       self.wind_truth, self.quat)

        if self.mode == 'treatment':
            diag = Vector3Stamped()
            diag.header.stamp = self.get_clock().now().to_msg()
            diag.vector.x = 1.0 if info['active'] else 0.0
            diag.vector.y = float(np.clip(info['h_obstacle'], -1e3, 1e3))
            diag.vector.z = 0.0 if info['feasible'] else 1.0
            self.cbf_pub.publish(diag)

        roll, pitch = self._lean_from_accel(u_safe[:2], self.quat)
        alt_err = target[2] - self.pos[2]  # NED: more negative target = higher
        climb_cmd = float(np.clip(-C.ALTITUDE_KP * alt_err, -1.0, 1.0))
        heading_err = _wrap_pi(self.cruise_yaw - _yaw_from_quat(self.quat))
        yaw_cmd = float(np.clip(C.HEADING_KP * heading_err, -1.0, 1.0))

        msg = OverrideRCIn()
        channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        channels[0:4] = [
            pwm(float(np.clip(roll / C.MAX_LEAN_RAD, -1.0, 1.0)), 1.0),
            pwm(float(np.clip(pitch / C.MAX_LEAN_RAD, -1.0, 1.0)), 1.0, invert=True),
            pwm_throttle(climb_cmd, 1.0),
            pwm(yaw_cmd, 1.0),
        ]
        channels[4:8] = [1500] * 4
        msg.channels = channels
        self.rc_pub.publish(msg)

        desired = Vector3Stamped()
        desired.header.stamp = self.get_clock().now().to_msg()
        desired.vector.x, desired.vector.y, desired.vector.z = (
            float(target[0]), float(target[1]), float(target[2]))
        self.desired_pub.publish(desired)

        self.tick += 1
```

Add the two small helpers used above (module level, above `class ControllerNode`):

```python
def _yaw_from_quat(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
```

And add `import math` to the top of the file, plus these new tuning constants to `src/aerocanyon/aerocanyon/constants.py` (append):

```python
# controller_node's new outer-loop gains (Phase 2 MAVROS port -- PX4's
# own position controller used to make these unnecessary). Starting
# points, not yet live-tuned; see docs/superpowers/plans/
# 2026-09-02-mission-stack-mavros-port.md Task 8 for the verification
# this needs before being trusted.
POSITION_KP = 0.5   # m/s^2 per metre of position error
POSITION_KD = 0.8   # m/s^2 per m/s of velocity (damping)
ALTITUDE_KP = 0.6   # climb-rate command [-1,1] per metre of altitude error
HEADING_KP = 1.0    # yaw-rate command [-1,1] per radian of heading error
MAX_LEAN_RAD = math.radians(20.0)  # lean angle that saturates the RC stick
```

(`constants.py` will need `import math` added too if not already present — check before adding.)

- [ ] **Step 4: Run test to verify it passes**

Run: `cd src/aerocanyon && python3 -m pytest test/test_controller_node.py -k lean_from_accel -v`
Expected: PASS (3 tests)

- [ ] **Step 5: Commit**

```bash
git add src/aerocanyon/aerocanyon/controller_node.py src/aerocanyon/aerocanyon/constants.py
git commit -m "aerocanyon: controller_node's new outer position/altitude/heading control loop"
```

---

## Task 5: `controller_node.py` — arm/mode/land via MAVROS, and full test rewrite

**Files:**
- Modify: `src/aerocanyon/aerocanyon/controller_node.py`
- Modify: `src/aerocanyon/test/test_controller_node.py`

**Interfaces:**
- Consumes: `rc_pwm.arm`, `rc_pwm.set_mode`, `rc_pwm.MODE_QHOVER`, `rc_pwm.MODE_QLAND` (Task 2).
- Produces: nothing new for later tasks — this completes `controller_node.py`.

- [ ] **Step 1: Replace the engage/land logic**

In `_tick`, replace the arm/offboard-engage block (former lines 268-283, the `_publish_offboard_mode`/`VEHICLE_CMD_DO_SET_MODE`/`ARM_DISARM` retry logic) with:

```python
        engaged = self.mavros_armed
        since_stream_started = self.tick - SETPOINTS_BEFORE_OFFBOARD
        if (not engaged and since_stream_started >= 0
                and since_stream_started % ENGAGE_RETRY_TICKS == 0):
            set_mode(self.cmd_client, MODE_QHOVER)
            arm(self.arm_client, True)
            self.get_logger().info('requested QHOVER mode and arm')
```

Replace the landing block (former lines 331-337, `VEHICLE_CMD_NAV_LAND`) with:

```python
        if engaged and self.pos[1] >= LAND_TRIGGER_LOCAL_M:
            set_mode(self.cmd_client, MODE_QLAND)
            self.land_requested = True
            self.get_logger().info(
                f'cleared the tower row by {LAND_CLEARANCE_M}m -- requested landing')
            self.tick += 1
            return
```

Delete the whole `ENABLE_VTOL_TRANSITION`-gated block (former lines 292-314) — VTOL transition is sub-project 4; keep the `ENABLE_VTOL_TRANSITION = False` module constant and its docstring comment (historical record of why it's off), but the dead code that referenced `VtolVehicleStatus`/`VEHICLE_CMD_DO_VTOL_TRANSITION` goes.

Update the `land_requested` early-return at the top of `_tick` (former lines 261-266) — unchanged, still correct, no edit needed there.

- [ ] **Step 2: Rewrite `test_controller_node.py`**

Replace the whole file (the PX4-specific `VehicleCommand`/`VtolVehicleStatus` assertions and the now-deleted VTOL transition tests don't apply) with:

```python
"""Regression coverage for controller_node's MAVROS control loop."""
import math

import numpy as np
import rclpy
from rclpy.duration import Duration

from aerocanyon.controller_node import (ENGAGE_RETRY_TICKS,
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
        import aerocanyon.rc_pwm as rc_pwm
        real_arm, real_set_mode = rc_pwm.arm, rc_pwm.set_mode
        import aerocanyon.controller_node as cn
        cn.arm = lambda client, value: arm_calls.append(value)
        cn.set_mode = lambda client, mode: mode_calls.append(mode)

        rc_msgs = []
        real_publish = node.rc_pub.publish
        node.rc_pub.publish = lambda msg: (rc_msgs.append(msg), real_publish(msg))[0]

        for _ in range(n):
            node._tick()
        node.destroy_node()
        return arm_calls, mode_calls, rc_msgs
    finally:
        rclpy.shutdown()


def test_tick_loop_survives_past_first_tick():
    _, _, rc_msgs = _run_ticks('baseline', SETPOINTS_BEFORE_OFFBOARD + 10)
    assert len(rc_msgs) >= 1


def test_requests_qhover_and_arm_after_setpoint_stream():
    arm_calls, mode_calls, _ = _run_ticks('baseline', SETPOINTS_BEFORE_OFFBOARD + 1)
    assert mode_calls == [18]  # rc_pwm.MODE_QHOVER
    assert arm_calls == [True]


def test_does_not_request_before_the_setpoint_stream_is_established():
    arm_calls, mode_calls, _ = _run_ticks('baseline', SETPOINTS_BEFORE_OFFBOARD)
    assert arm_calls == [] and mode_calls == []


def test_retries_arm_request_until_engaged():
    arm_calls, _, _ = _run_ticks(
        'baseline', SETPOINTS_BEFORE_OFFBOARD + 2 * ENGAGE_RETRY_TICKS + 1)
    assert len(arm_calls) == 3


def test_stops_retrying_once_mavros_reports_armed():
    arm_calls, _, _ = _run_ticks(
        'baseline', SETPOINTS_BEFORE_OFFBOARD + 2 * ENGAGE_RETRY_TICKS + 1, armed=True)
    assert arm_calls == []


def test_treatment_mode_also_survives_the_tick_loop():
    _, _, rc_msgs = _run_ticks('treatment', SETPOINTS_BEFORE_OFFBOARD + 10)
    assert len(rc_msgs) >= 1


def test_lean_from_accel_zero_is_level():
    roll, pitch = ControllerNode._lean_from_accel(
        np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]))
    assert abs(roll) < 1e-9 and abs(pitch) < 1e-9


def test_lean_from_accel_forward_accel_pitches_forward():
    roll, pitch = ControllerNode._lean_from_accel(
        np.array([2.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]))
    assert pitch > 0.0
    assert abs(roll) < 1e-9


def test_lean_from_accel_rightward_accel_rolls_right():
    roll, pitch = ControllerNode._lean_from_accel(
        np.array([0.0, 2.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]))
    assert roll > 0.0
    assert abs(pitch) < 1e-9


def _run_land_ticks(n, pos, mode='baseline'):
    rclpy.init(args=[])
    try:
        node = ControllerNode()
        node.mode = mode
        node.mavros_armed = True
        node.start_time = node.get_clock().now() - Duration(seconds=30.0)
        node.pos = np.array(pos, dtype=float)
        import aerocanyon.controller_node as cn
        mode_calls = []
        cn.set_mode = lambda client, mode: mode_calls.append(mode)
        rc_msgs = []
        real_publish = node.rc_pub.publish
        node.rc_pub.publish = lambda msg: (rc_msgs.append(msg), real_publish(msg))[0]
        for _ in range(n):
            node._tick()
        result = (mode_calls, rc_msgs, node.land_requested)
        node.destroy_node()
        return result
    finally:
        rclpy.shutdown()


def test_lands_in_place_once_position_clears_the_last_tower_row():
    import aerocanyon.controller_node as controller_node
    for mode in ('baseline', 'treatment'):
        mode_calls, rc_msgs, land_requested = _run_land_ticks(
            2, pos=(0.0, controller_node.LAND_TRIGGER_LOCAL_M, 0.0), mode=mode)
        assert land_requested, f'{mode}: must request landing on clearing the tower row'
        assert mode_calls == [20], f'{mode}: must switch to QLAND exactly once'
        assert rc_msgs == [], (
            f'{mode}: must stop publishing RC overrides once handed off to QLAND')


def test_does_not_land_before_clearing_the_last_tower_row():
    import aerocanyon.controller_node as controller_node
    _, _, land_requested = _run_land_ticks(
        5, pos=(0.0, controller_node.LAND_TRIGGER_LOCAL_M - 1.0, 0.0))
    assert not land_requested
```

(The `import aerocanyon.controller_node as cn; cn.arm = ...` monkeypatch pattern works because `controller_node.py` does `from .rc_pwm import arm, set_mode` — that binds the names into `controller_node`'s own module namespace, which is exactly what needs patching to intercept the calls without a live MAVROS peer.)

- [ ] **Step 3: Run tests to verify they pass**

Run: `cd src/aerocanyon && python3 -m pytest test/test_controller_node.py -v`
Expected: PASS (11 tests)

- [ ] **Step 4: Commit**

```bash
git add src/aerocanyon/aerocanyon/controller_node.py src/aerocanyon/test/test_controller_node.py
git commit -m "aerocanyon: controller_node arm/mode/land via MAVROS; rewrite its test suite"
```

---

## Task 6: `trial_logger.py` and `fo_pinn_node.py` — MAVROS telemetry

**Files:**
- Modify: `src/aerocanyon/aerocanyon/trial_logger.py`
- Modify: `src/aerocanyon/aerocanyon/fo_pinn_node.py`

**Interfaces:**
- Consumes: `frames.enu_to_ned`, `frames.enu_flu_quat_to_ned_frd`, `frames.enu_flu_rate_to_ned_frd` (Task 1).
- Produces: nothing new for later tasks — both files' published/logged data keeps its existing shape (NED position/velocity, NED/FRD quaternion, FRD body rates/accel), so `train_pinn.py`/`plot_results.py` (untouched) keep working unmodified.

- [ ] **Step 1: Port `trial_logger.py`**

Replace the import (line 14) and the telemetry subscriptions in `trial_logger.py`. Read the file first (`cat src/aerocanyon/aerocanyon/trial_logger.py`) to find its exact current subscription block (the module docstring notes it reads `SensorCombined`/`VehicleAttitude`/`VehicleLocalPosition`); replace with:

```python
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import State
from sensor_msgs.msg import Imu

from . import frames
```

and the three PX4 subscriptions with MAVROS ones (same topics as Task 3's `controller_node.py`: `/mavros/local_position/pose`, `/mavros/local_position/velocity_local`, `/mavros/imu/data`), converting each with `frames.enu_to_ned`/`frames.enu_flu_quat_to_ned_frd`/`frames.enu_flu_rate_to_ned_frd` before storing, so the CSV columns (`x,y,z,vx,vy,vz,qw,qx,qy,qz,ax,ay,az,p,q,r`) keep exactly the same NED/FRD meaning they had under PX4 — `COLUMNS` itself does not change.

- [ ] **Step 2: Port `fo_pinn_node.py`**

Same telemetry swap in `fo_pinn_node.py` (replace lines 12, 52-60, 65-73): MAVROS's `/mavros/local_position/velocity_local` for `self.vel`, `/mavros/imu/data` for `self.quat`/`self.gyro`/`self.accel`, each converted via the Task 1 `frames` helpers before use — the state vector fed to the trained network (`np.concatenate([self.vel, self.quat, self.gyro, self.accel])`) must keep the exact same NED/FRD convention the checkpoint was trained on (or will be retrained on, in sub-project 3), or the network's input distribution silently shifts with no error.

- [ ] **Step 3: Verify both import cleanly**

Run: `cd src/aerocanyon && python3 -c "import aerocanyon.trial_logger; import aerocanyon.fo_pinn_node"`
Expected: no `ImportError`

- [ ] **Step 4: Commit**

```bash
git add src/aerocanyon/aerocanyon/trial_logger.py src/aerocanyon/aerocanyon/fo_pinn_node.py
git commit -m "aerocanyon: trial_logger and fo_pinn_node read telemetry from MAVROS"
```

---

## Task 7: `run_trial.py` — ArduPilot SITL lifecycle and `ExtendedState` landing detection

**Files:**
- Modify: `src/aerocanyon/aerocanyon/run_trial.py`
- Modify: `src/aerocanyon/test/test_run_trial.py`

**Interfaces:**
- Consumes: Phase 1's proven per-leg launch sequence (this session's manual verification: `gz sim -v 2 -s -r <world>`, `arduplane --model JSON --home <lat,lon,alt,0> --wipe --defaults tricopter.parm`, `mavros_node --ros-args -p fcu_url:=tcp://127.0.0.1:5760 -p system_id:=255` with `GEOGRAPHICLIB_DATA` set).
- Produces: nothing new for later tasks — this completes sub-project 1's runnable end state.

- [ ] **Step 1: Replace the PX4/agent process lifecycle**

Read `run_trial.py` in full first (`cat src/aerocanyon/aerocanyon/run_trial.py`) to find every reference to `PX4_DIR`, `AGENT`, `PX4_SIM_MODEL`, `PX4_GZ_WORLD`, `PX4_GZ_MODEL_POSE`, and the subprocess launch calls for the PX4 binary and `MicroXRCEAgent`. Replace:

- `PX4_DIR = pathlib.Path.home() / 'PX4-Autopilot'` and `AGENT = ...MicroXRCEAgent` with:
  ```python
  ARDUPILOT_DIR = pathlib.Path.home() / 'ardupilot'
  ARDUPLANE = ARDUPILOT_DIR / 'build' / 'sitl' / 'bin' / 'arduplane'
  TRICOPTER_PARM = (pathlib.Path(__file__).resolve().parents[3]
                    / 'src' / 'aerocanyon' / 'ardupilot' / 'tricopter.parm')
  GEOGRAPHICLIB_DATA = pathlib.Path.home() / '.local' / 'share' / 'GeographicLib'
  ```
- The PX4 subprocess launch (env vars `PX4_SIM_MODEL`/`PX4_GZ_WORLD`/`PX4_GZ_MODEL_POSE`, `./build/px4_sitl_default/bin/px4`) with:
  ```python
  apstate_dir = pathlib.Path(tempfile.mkdtemp(prefix='aerocanyon_apstate_'))
  home_str = f'{HOME_LAT},{HOME_LON},{HOME_ALT},0'  # define HOME_LAT/LON/ALT
                                                     # near SPAWN_XYZ, matching
                                                     # README's --home value
  proc = subprocess.Popen(
      [str(ARDUPLANE), '--model', 'JSON', '--home', home_str,
       '--wipe', '--defaults', str(TRICOPTER_PARM)],
      cwd=str(apstate_dir))
  ```
  (`import tempfile` at the top of the file if not already imported.)
- The `MicroXRCEAgent` subprocess launch with a `mavros_node` launch:
  ```python
  env = dict(os.environ, GEOGRAPHICLIB_DATA=str(GEOGRAPHICLIB_DATA))
  mavros_proc = subprocess.Popen(
      ['ros2', 'run', 'mavros', 'mavros_node', '--ros-args',
       '-p', 'fcu_url:=tcp://127.0.0.1:5760', '-p', 'system_id:=255'],
      env=env)
  ```

- [ ] **Step 2: Replace `VehicleLandDetected` with `ExtendedState`**

Find `_LandWatcher` and `_wait_for_landing`. Replace the `px4_msgs.msg.VehicleLandDetected` import and subscription with:

```python
from mavros_msgs.msg import ExtendedState
```

and, in `_LandWatcher`, subscribe to `/mavros/extended_state` instead of `/fmu/out/vehicle_land_detected`, storing `msg.landed_state` and updating `self.landed` using the same "must have seen airborne first" guard the existing code already has for `VehicleLandDetected.landed` (`ExtendedState.LANDED_STATE_ON_GROUND == 1`, `LANDED_STATE_IN_AIR == 2` — treat `ON_GROUND` after having previously seen `IN_AIR` as landed, exactly mirroring the existing regression test's intent).

- [ ] **Step 3: Update `test_run_trial.py`**

Replace `from px4_msgs.msg import VehicleLandDetected` with `from mavros_msgs.msg import ExtendedState`, and every `VehicleLandDetected(landed=True/False)` construction with `ExtendedState(landed_state=ExtendedState.LANDED_STATE_ON_GROUND)` / `ExtendedState(landed_state=ExtendedState.LANDED_STATE_IN_AIR)` respectively (matching the two states used in the existing `test_land_watcher_*` tests). The Gazebo teleport test (`test_reset_gazebo_model_teleports_the_right_entity_to_the_spawn_pose`) and the `_verify_px4_started`-equivalent test need their function name/reference updated if Step 1 renamed `_verify_px4_started` (rename it to `_verify_sitl_started` for accuracy, update the test import and `pytest.raises(SystemExit, match=...)` message string to match whatever exit message the renamed function now raises — keep the same fail-fast behavior, just against `arduplane` instead of `px4`).

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd src/aerocanyon && python3 -m pytest test/test_run_trial.py -v`
Expected: PASS (all tests, same count as before this task)

- [ ] **Step 5: Commit**

```bash
git add src/aerocanyon/aerocanyon/run_trial.py src/aerocanyon/test/test_run_trial.py
git commit -m "aerocanyon: run_trial.py launches ArduPilot SITL + MAVROS instead of PX4 + Micro-XRCE-DDS-Agent"
```

---

## Task 8: End-to-end manual verification and final review

**Files:** none (verification only)

**Interfaces:** none — this task verifies Tasks 1-7 together.

- [ ] **Step 1: Run the full test suite**

Run: `cd src/aerocanyon && source /opt/ros/jazzy/setup.bash && python3 -m pytest test/ -v`
Expected: PASS, all tests (Tasks 1-7's new/updated tests plus any untouched existing ones, e.g. `test_plot_results.py`)

- [ ] **Step 2: Fresh full-stack restart, one live baseline leg**

Per this project's established discipline: kill every existing Gazebo/SITL/MAVROS/control_server process first (`ps -eo pid,cmd | grep -E "gz sim|arduplane|mavros_node"`, `kill -9` each, never a partial/selective kill), confirm nothing is left running, then:

```bash
cd $HOME/AeroCanyon_Guard
source /opt/ros/jazzy/setup.bash && source install/setup.bash
source .venv/bin/activate
python3 -m aerocanyon.run_trial --trial verify_port --mode baseline --duration 60
```

Expected: the leg's own Gazebo instance boots, arduplane SITL arms and reaches QHOVER, `controller_node` drives it toward the mission's first waypoint (visible in `web_viewer` at `http://localhost:8080` while the leg runs, per `run_trial.py`'s own per-leg browser bridge), and the leg completes and writes `trials/verify_port_baseline.csv` without crashing. The vehicle does not need to fly the mission *well* yet (the new `POSITION_KP`/`POSITION_KD`/`ALTITUDE_KP`/`HEADING_KP` gains in `constants.py` are untuned starting points, not verified-good ones) — this step only confirms the whole pipeline runs end to end without exceptions, silent telemetry freezes, or a crash, which is this sub-project's actual exit criterion. Gain tuning, if the flight looks unstable rather than merely imprecise, is follow-up work, not blocking for this plan.

- [ ] **Step 3: Inspect the trial CSV**

```bash
python3 -c "
import pandas as pd
df = pd.read_csv('trials/verify_port_baseline.csv')
print(df[['t','x','y','z']].describe())
assert len(df) > 10, 'trial_logger must have actually logged rows'
assert df['x'].std() > 0 or df['y'].std() > 0, 'vehicle must have actually moved'
print('OK')
"
```

Expected: `OK` — confirms `trial_logger.py`'s MAVROS-sourced telemetry is real, moving data, not frozen/zero (this project's own established "silent telemetry freeze" failure mode from the PX4 era, per `run_trial.py`'s own module docstring).

- [ ] **Step 4: Final commit (docs only, if anything needs updating)**

If Steps 2-3 revealed anything a future reader needs to know (e.g. the untuned gains producing a visibly rough flight — expected, not a blocker), add one short note to `constants.py`'s `POSITION_KP` etc. comment block documenting what was actually observed, and commit:

```bash
git add -A
git commit -m "aerocanyon: verify mission-stack MAVROS port flies one leg end to end"
```

If nothing needs updating, no commit is needed for this task.
