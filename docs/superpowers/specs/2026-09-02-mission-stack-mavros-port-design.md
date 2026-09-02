# Mission-stack MAVROS port (Phase 2, sub-project 1) — Design

## Context

Phase 1 (this session, earlier) ported manual hover flight from PX4 to
ArduPilot SITL + MAVROS, on what is now the sole `tricopter` branch (PX4
and all its infrastructure — `px4_msgs`, the PX4 Convergence airframe,
the old PX4 Gazebo model — have been removed). The autonomous CBF/PINN
mission stack was explicitly out of scope for Phase 1 and is still built
entirely on `px4_msgs` types and PX4's offboard-position control path:

- `controller_node.py` — streams a `TrajectorySetpoint` (position, with
  velocity/acceleration set to NaN so PX4's own position controller has
  full authority) toward the mission's target, with a CBF-filtered PINN
  wind-feedforward layered in for `treatment` mode.
- `trial_logger.py` / `fo_pinn_node.py` — subscribe to `SensorCombined`,
  `VehicleAttitude`, `VehicleLocalPosition` for telemetry.
- `run_trial.py` — owns the whole PX4 SITL + Micro-XRCE-DDS-Agent + Gazebo
  process lifecycle, one fresh set of processes per trial leg.

None of `px4_msgs`, PX4's offboard-position path, or the PX4 SITL/agent
lifecycle exist anymore. This is sub-project 1 of Phase 2 (of 4: this
port, then a new wind field for the `map_zone` terrain, then FO-PINN
retraining + a 49-point hover wind-sweep with mean RMS, then VTOL
transition flight modes) and blocks the other three, since nothing in
the mission stack currently runs.

## Key finding: ArduPilot exposes no XY position/velocity injection here

Read directly from `ArduPlane/GCS_MAVLink_Plane.cpp` in this project's
pinned ArduPilot build (commit `b9439efde1`):
`handle_set_position_target_local_ned` (the MAVLink handler behind
MAVROS's `/mavros/setpoint_position/local` and
`/mavros/setpoint_velocity/*` topics) only runs when
`plane.control_mode == &plane.mode_guided`, and even there is a stub —
"just do altitude for now" — that ignores the X/Y fields entirely.
`handle_set_position_target_global_int` has the same GUIDED-only,
altitude-only limitation. Phase 1 already found GUIDED's velocity path
is a no-op for this MAV_TYPE; this closes the loop — there is **no**
companion-computer XY position or velocity injection path for this
airframe in any flight mode in this ArduPilot build, not just GUIDED.

Consequence: `controller_node` cannot hand ArduPilot a position and walk
away, the way it handed PX4 one. It must compute its own outer
position-hold loop and drive the vehicle the same way a human pilot
does — RC-override PWM under `QHOVER` — exactly like Phase 1's manual
flight control already does. Autonomous flight becomes "the mission
logic drives the same RC-override channels a human would," not "the
mission logic hands ArduPilot a waypoint."

## Architecture

Every control tick (`CONTROL_HZ`, unchanged):

1. Read current position/velocity (MAVROS `/mavros/local_position/pose`
   + `/mavros/local_position/velocity_local`, both ENU) and attitude
   (quaternion, same topic). Convert to NED via the existing
   `frames.enu_to_ned`/`ned_to_enu` — `mission.py` and `cbf_filter.py`
   both already work in NED and stay that way; only the MAVROS boundary
   needs the conversion, not the mission math.
2. Compute a desired world-frame (NED) acceleration:
   - `baseline` mode: a plain PD controller toward `mission.target(t)` —
     new code, since PX4's position controller used to do this for free.
   - `treatment` mode: unchanged — `CBFFilter.filter(u_des, pos_ned,
     vel_ned, wind_ned, q)` still produces the safety-filtered
     acceleration; `u_des` still comes from the PINN feedforward. The
     only change is that baseline now needs an analogous "desired
     acceleration toward target" the CBF filter can also wrap, rather
     than skipping straight to a position setpoint.
3. Rotate the desired horizontal acceleration into body-frame lean
   angles using current yaw: `roll ≈ atan2(-a_y_body, g)`, `pitch ≈
   atan2(a_x_body, g)` (small-angle, consistent with QHOVER's own
   attitude response to RC input). A separate altitude-error P-loop
   produces a climb-rate command for throttle; a heading-error P-loop
   (target: `cruise_yaw`, unchanged) produces a yaw-rate command.
4. Map lean/climb-rate/yaw-rate to RC-override PWM using the **same**
   `RC_CENTER=1500`/`RC_SPAN=500` and `THROTTLE_MID=1450`/
   `THROTTLE_SPAN=450` convention `control_server.py` already proved
   live in Phase 1 manual flight, via a new shared module (below) so the
   two files don't duplicate the mapping.
5. Publish via `/mavros/rc/override` (`mavros_msgs/OverrideRCIn`) —
   exactly the message manual flight already uses, just from a node
   instead of a browser stick.

Arming/mode-engage: replace the PX4 `VEHICLE_CMD_DO_SET_MODE`(1,6) +
`ARM_DISARM` retry loop with `/mavros/cmd/arming`
(`mavros_msgs/CommandBool`) + `/mavros/cmd/command`
(`mavros_msgs/CommandLong`, `MAV_CMD_DO_SET_MODE` → QHOVER=18) — the same
calls `control_server.py`'s `MavrosBridge._arm`/`_set_mode` already make
and already proved live (including the `system_id:=255` requirement).
`controller_node` calls the same helpers rather than reimplementing them
— extract them from `control_server.py` into the new shared module too.

Landing: same position-gate logic as today (`self.pos[1] >=
LAND_TRIGGER_LOCAL_M`, unchanged), but the action becomes a mode-switch
to QLAND (`MAV_CMD_DO_SET_MODE` → 20) instead of
`VEHICLE_CMD_NAV_LAND` — QLAND auto-disarms on touchdown the same way
PX4's `AUTO_LAND` did, matching the existing design rationale in the
code (hand landing off to the autopilot's own field-tested logic, never
a from-scratch descent).

VTOL transition stays disabled (`ENABLE_VTOL_TRANSITION = False`) here —
sub-project 4 owns that.

## Components

- **New: `rc_pwm.py`** (`src/aerocanyon/aerocanyon/`) — the PWM-mapping
  helpers (`pwm()`, `pwm_throttle()`, `RC_CENTER`/`RC_SPAN`/
  `THROTTLE_MID`/`THROTTLE_SPAN`) and the arm/set-mode MAVROS service
  calls (`arm()`, `set_mode()`), extracted from `web_viewer/
  control_server.py` and imported by both it and `controller_node.py`.
  `control_server.py`'s own behavior does not change — pure extraction.
- **`controller_node.py`** — outer position/altitude/heading control
  loop as above (new code); MAVROS topic subscriptions replace the
  `px4_msgs` ones; arm/mode/land via `rc_pwm.py`'s helpers.
- **`trial_logger.py`** — `SensorCombined`/`VehicleAttitude`/
  `VehicleLocalPosition` subscriptions replaced with MAVROS's
  `/mavros/imu/data` (`sensor_msgs/Imu`, gives orientation + angular
  velocity + linear acceleration all in one message — replaces all
  three PX4 subscriptions), plus `/mavros/local_position/velocity_local`
  for vx/vy/vz (IMU's linear acceleration is body-frame, not velocity).
  `COLUMNS` and CSV format unchanged — this only changes where the
  numbers come from, not the schema `train_pinn.py`/`plot_results.py`
  read.
- **`fo_pinn_node.py`** — same telemetry swap as `trial_logger.py`.
- **`run_trial.py`** — SITL lifecycle swaps the PX4 binary + Micro-XRCE-
  DDS-Agent for `arduplane` + `mavros_node`, reusing Phase 1's proven
  per-leg headless launch sequence (`gz sim -s`, `arduplane --model JSON
  --home ... --wipe --defaults tricopter.parm`, `mavros_node
  --ros-args -p fcu_url:=... -p system_id:=255` with
  `GEOGRAPHICLIB_DATA` set). `VehicleLandDetected` (PX4's "landed"
  signal, used to confirm a leg actually finished) has no direct MAVROS
  message — replaced by watching `/mavros/extended_state`
  (`mavros_msgs/ExtendedState.landed_state`). The Gazebo-entity
  reset/teleport logic (`_reset_gazebo_model` etc., used only by the
  manual "watch a trial fly" flow) is untouched — it's gz-transport
  code, autopilot-agnostic.
- **`canyon_geometry.py`, `constants.py`, `mission.py`, `cbf_filter.py`**
  — untouched. All pure math/geometry with no PX4 dependency;
  `mission.py`'s NED convention is exactly what the new outer loop needs
  after the MAVROS ENU→NED conversion. (`constants.py` gains new outer-
  loop gain constants during implementation — additive, not a change to
  anything existing.)
- **`frames.py`** — gains new quaternion/rate conversion functions
  (additive, existing `ned_to_enu`/`enu_to_ned`/`quat_to_rotmat`/
  `body_z_in_ned` untouched). Position/velocity vectors convert via the
  existing ENU↔NED functions, but MAVROS's orientation
  (`/mavros/imu/data`) and body rates are in **ENU-world/FLU-body**
  (ROS's convention), not PX4's NED-world/FRD-body that `cbf_filter.py`
  and the trained PINN's state vector both assume — a second, distinct
  conversion this design initially missed while focused on position.
  `frames.py` needs `enu_flu_quat_to_ned_frd`/`enu_flu_rate_to_ned_frd`
  before `controller_node.py`, `trial_logger.py`, and `fo_pinn_node.py`
  can hand MAVROS's attitude/rate data to any of the unchanged NED/FRD
  math.

## Data flow (baseline mode, per tick)

```
MAVROS (/mavros/local_position/pose, /velocity_local)
  -> ENU->NED (frames.py)
  -> position error vs mission.target(t)
  -> PD -> desired accel (NED)
  -> CBFFilter.filter(...)  [always runs now, not just treatment;
                              treatment additionally feeds in PINN u_des]
  -> body-frame lean angles (needs current yaw)
  -> rc_pwm.py PWM mapping
  -> /mavros/rc/override (OverrideRCIn)
```

Treatment mode differs only in what feeds `u_des` into the same
`CBFFilter.filter()` call — the PINN wind-force estimate, exactly as
today, scaled by `feedforward_gain`.

**Deliberate behavior change, called out explicitly:** the original PX4
design ran the CBF filter in `treatment` mode only — `baseline` handed
PX4 a raw position and PX4's own controller (not this project's code)
was the only thing keeping it away from buildings. With no PX4 position
controller to lean on, `baseline` now needs *some* obstacle-safety layer
of its own, and running the (already-built, already-tested) CBF filter
under both modes — with `u_des` from a plain PD controller in `baseline`
and from the PINN feedforward in `treatment` — is the natural way to get
one without writing a second safety mechanism. This does not change the
scientific comparison: `baseline` still receives zero PINN wind
cancellation, which is the only thing the trial is measuring. It does
mean `baseline`'s `cbf_active`/`h_obstacle` diagnostic becomes
meaningful for the first time (previously only published in
`treatment`) — worth plotting for both modes once this lands.

## Safety / error handling

- Stale/missing MAVROS telemetry (no message within a timeout) zeros the
  corresponding RC channel rather than commanding a stale lean angle —
  the same dead-man's-switch pattern Phase 1's `control_server.py`
  already uses for manual control (`resolve_stick`).
  `controller_node` reuses that function from `rc_pwm.py` rather than
  reimplementing it.
- Landing is handed off to QLAND unconditionally, never a
  self-controlled descent — same rationale already documented in
  `controller_node.py`'s existing comments (a prior self-controlled
  descent design was verified live to be capable of an uncontrolled
  tumble; QLAND/AUTO_LAND's field-tested logic doesn't have that
  failure mode).
- The CBF safety filter's behavior and API (`CBFFilter.filter`) do not
  change at all — this port only changes what feeds `u_des` in and what
  consumes `u_safe` out.

## Testing

- New unit tests (pure functions, no ROS/rclpy needed) for: the
  position-error→lean-angle math, and `rc_pwm.py`'s PWM mapping
  (mirrors the existing `test_control_server.py` pattern from Phase 1).
- `test_controller_node.py` — update the `VehicleCommand`/
  `VtolVehicleStatus` imports/assertions to the MAVROS
  `CommandBool`/`CommandLong` equivalents.
- `test_run_trial.py` — update the `VehicleLandDetected` import/usage to
  `mavros_msgs/ExtendedState`.
- Manual verification (same discipline as Phase 1): fresh full-stack
  restart, one full baseline leg watched live via the browser viewer,
  confirming the vehicle actually tracks the mission's NED waypoints
  under RC-override control before trusting any automated trial output.

## Out of scope for this sub-project

- The wind field itself (sub-project 2) — `canyon_field.py` and its
  synthetic box-canyon geometry are untouched here; the mission still
  flies the existing `urban_canyon` scenario in this sub-project.
  Retargeting the mission/wind field at `map_zone` terrain is
  sub-project 2's work.
- FO-PINN retraining and the 49-point hover wind-sweep + mean RMS
  (sub-project 3).
- VTOL transition flight modes and the ArduPilot `tiltrotor.cpp` patch
  (sub-project 4).
