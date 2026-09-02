# Tricopter on ArduPilot, Phase 1: flight only

Branch: `tricopter-ap` (off `tricopter`)

## Purpose

The `tricopter` branch's PX4-based VTOL tricopter (three tilting rotors,
front pair for hover yaw, all three now tiltable after this session's
yaw-right-at-takeoff fix) has had its control loop tuned entirely
empirically against PX4's control allocator this session -- including a
build-staging gotcha that invalidated a whole session's worth of "fixes"
before it was found. This project explores whether ArduPilot, which has
native, first-class support for tilt-tricopter VTOL frames
(`Q_FRAME_CLASS=7` + `Q_TILT_MASK`, see
https://ardupilot.org/plane/docs/guide-tilt-rotor.html) rather than the
hand-derived allocator tuning PX4 required here, is a better fit for this
vehicle -- without disturbing the working PX4 branch.

**Concrete, not just "tuning was hard":** the vehicle's actual intended
design (see Geometry below) has the front rotors fully stop and fold
their propeller blades in forward flight -- and PX4's control allocator
has no way to express "tiltable (for hover yaw) AND auto-stopped in FW"
on the same rotor (see `4022_gz_tricopter`'s own header comment, point
1). The current PX4 config's front rotors keep spinning in cruise as a
direct consequence of that limitation, not by design choice. This is a
real capability gap, not just an empirical-tuning inconvenience.

This is Phase 1 of a two-phase migration: **flight only**. Getting the
tricopter armed, hovering stably, and manually flyable through the
existing browser viewer under ArduPilot SITL. The autonomous
CBF/PINN mission stack (`controller_node.py`, `run_trial.py`,
`trial_logger`, `plot_results`) is explicitly Phase 2, a separate spec,
started only after Phase 1 lands and is verified.

## Non-goals (this phase)

- Porting `controller_node.py`, the CBF safety filter, or the PINN wind
  estimator to ArduPilot. They stay PX4-only (`tricopter` branch) for now.
- Touching anything on the `tricopter` (PX4) branch. All work happens on
  `tricopter-ap`; the PX4 model/airframe/`run_trial.py` are left
  untouched as a reference and a working fallback.
- Matching or beating PX4's tuning numerically. The goal is a stable,
  flyable vehicle under ArduPilot's own control loop, not a numeric
  comparison between autopilots (that's a natural Phase 2+ question,
  once both stacks fly the actual mission).
- VTOL fixed-wing transition **flight-testing**. Exactly as with the PX4
  branch (see `README.md`'s honest caveat there), actually flying to
  cruise and tuning that regime is out of scope -- hover/multicopter
  flight only. The tilt ranges/roles themselves (see Geometry below)
  still need to be configured correctly in Phase 1's param file, since
  the front pair's yaw-trim sub-range is genuinely a hover-flight
  mechanism -- only flying the transition itself is deferred.

## Geometry (corrected -- authoritative, from the user, supersedes any
## earlier guess in this doc or in PX4-side comments about "all three tilt
## the same way")

- **Front motors** (the pair): tilt **-20 deg to +90 deg**, where 90 deg
  is the horizontal axis (thrust pointing forward) and 0 deg is vertical
  (hover). The **-20..0 deg range is dedicated to yaw trim** (differential
  tilt past vertical, same purpose as the PX4 branch's negative-range
  extension) -- it is not used in the hover-to-cruise transition itself.
  **In forward/cruise flight the front motors fully stop and fold their
  propeller blades** -- they contribute zero thrust in cruise.
- **Rear motor**: a pusher with a **high-pitch propeller** (sized for
  cruise efficiency, not hover). Tilts in the vertical plane from
  **-90 deg (vertical/hover) to 0 deg (horizontal/cruise)** -- note this
  is the opposite sign convention from the front pair's range, and covers
  a different span (90 deg total, no yaw-trim-only sub-range). **It is
  the only thruster active in forward/cruise flight.**

This means the tricopter has **two structurally different tilt roles**,
not one uniform "all rotors tilt together" mechanism:
- Front pair: classic tiltrotor-with-fold -- vertical for hover (plus a
  small negative range purely for yaw trim, never reached during an
  actual hover-to-cruise transition), full stop+fold at the forward end.
- Rear: an always-active tilting pusher, tilts through its own separate
  range, contributes hover thrust throughout and is the sole cruise
  thruster -- never stops.

## Open risk, resolved first

Whether ArduPilot's `Q_TILT_MASK`/tiltrotor system can represent **two
motors with different tilt ranges and different forward-flight roles**
within one frame is **unconfirmed** and must not be assumed. ArduPilot's
documented tilt-tricopter examples (`Q_TILT_MASK=3`, front two tilt) all
appear to assume every masked motor shares the same tilt schedule and
role. This project's actual geometry doesn't fit that shape cleanly: the
front pair needs stop+fold behavior the rear must NOT have, and the rear
needs its own independent tilt range/schedule.

**First implementation step, before any other Phase 1 work:** a bounded
spike investigating how ArduPilot actually models this, in order of
preference:
1. Whether `Q_TILT_MASK` supports per-motor tilt angle limits/roles (so
   front and rear can have different ranges/behavior within one masked
   group), via SITL param inspection and ArduPilot's own tiltrotor
   source (`AP_MotorsTiltrotor`), not just the docs page.
2. Whether the rear pusher should instead be modeled OUTSIDE
   `Q_TILT_MASK` entirely -- e.g. a plain always-on motor with its own
   tilt servo driven independently (`SERVOn_FUNCTION`), decoupled from
   ArduPilot's tiltrotor transition state machine, while only the front
   pair is a `Q_TILT_MASK` tiltrotor group.
3. If neither cleanly represents the real geometry, document the closest
   achievable approximation here (update this spec) before continuing,
   and flag exactly what behavior is being approximated away.

## Architecture

- **SITL**: `sim_vehicle.py` (ArduCopter/QuadPlane, whichever
  `Q_FRAME_CLASS=7` resolves to) replaces the PX4 binary +
  Micro-XRCE-DDS-Agent pair.
- **Gazebo integration**: `ardupilot_gazebo` (ArduPilot's own Gazebo
  plugin) replaces PX4's `gz_bridge`-specific plugin wiring
  (`gz-sim-multicopter-motor-model-system` topic/servo conventions in
  `model.sdf`). Same `urban_canyon.sdf` world, unmodified -- reused as-is
  so Phase 2's canyon/wind-field work isn't redone.
- **ROS2 bridge**: MAVROS, replacing `px4_msgs`/uXRCE-DDS. Chosen over
  ArduPilot's newer AP_DDS for maturity; `mavros_msgs` types (not
  `px4_msgs`) throughout the manual-flight path.
- **Headless + browser viewer**: reuses this session's PX4-side pattern
  (`gz sim -s`, `web_viewer/`'s websocket bridge, a plain static server)
  unchanged in shape -- Gazebo's headless/viewer setup doesn't care which
  autopilot is driving it.

## Components / files touched

- **New model directory**: `src/aerocanyon/models/tricopter_ap/`,
  parallel to (not replacing) `src/aerocanyon/models/tricopter/`. Same
  visual meshes/geometry; motor/servo plugin blocks swapped for
  `ardupilot_gazebo`'s conventions.
- **New ArduPilot parameter file** (`.parm`), replacing the role of
  `4022_gz_tricopter`: `Q_FRAME_CLASS=7`, `Q_TILT_MASK` (per the spike
  above), thrust/hover-fraction and rate-gain tuning. Starts from this
  session's *empirically-found* PX4 numbers as a reference point only --
  ArduPilot's allocator is different, needs its own tuning pass, not a
  blind port of PX4 param values.
- **New SITL launch script**, replacing `run_trial.py`'s
  `_spawn_gazebo`/PX4-spawn logic for this branch's manual-flight path:
  `sim_vehicle.py` + MAVROS launch, keeping the headless-Gazebo +
  web-viewer pattern from this session's most recent PX4-side change.
- **`web_viewer/control_server.py`**: rewritten against MAVROS
  (arm/mode-set services, velocity setpoints) instead of `px4_msgs`.
  `web_viewer/index.html`'s UI stays as-is (Mode 2 sticks, arm/disarm/land
  buttons) -- only the backend it talks to changes.
- **`README.md`**: new section documenting the ArduPilot path
  (prerequisites, setup, manual flying). PX4's existing README content
  is untouched (different branch).

## Testing / verification approach

This session's PX4 tuning surfaced three concrete process failures worth
carrying forward as explicit discipline here, not just lessons learned:

1. **Config staging**: PX4 had a build-staged copy of airframe scripts
   that silently went stale, invalidating a whole session of "fixes"
   before being found. Before assuming an ArduPilot param change took
   effect, verify it landed in whatever SITL actually reads at boot
   (check ArduPilot's own param-persistence/EEPROM behavior for the
   equivalent trap before relying on param file edits alone).
2. **Fresh restart discipline**: repeated arm/disarm/test cycles on the
   same long-running SITL process corrupted controller/land-detector
   state and produced misleading results this session. Every
   verification test in Phase 1's plan restarts the whole stack
   (Gazebo + SITL + bridge) fresh first, never trusts a result from a
   process that's already been armed/tested before.
3. **Don't touch Gazebo subprocesses individually**: killing just the
   GUI subprocess of a combined server+GUI `gz sim` process took the
   whole instance down this session. Headless (`-s`) sidesteps this by
   not spawning a GUI process at all; if that ever needs revisiting,
   treat the whole `gz sim` process as the unit to stop/restart, never a
   child of it.

## Phase 1 exit criteria

The tricopter arms, hovers stably (level attitude, no yaw spin, matching
what got verified for PX4 this session) under ArduPilot SITL, and
responds correctly to manual stick input through the browser viewer.
Autonomous mission flight is explicitly out of scope for this phase's
"done."
