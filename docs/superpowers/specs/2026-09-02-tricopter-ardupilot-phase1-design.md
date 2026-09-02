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

Both motors' tilt sits on **one shared, continuous angle axis** (not two
independent conventions as an earlier version of this doc described) --
90 deg is the single common reference point, horizontal, where the two
ranges meet:

- **Front motors** (the pair): **-20 deg to 90 deg**. 0 deg is vertical
  (hover); 90 deg is horizontal, pointing forward. The **-20..0 deg
  range is dedicated to yaw trim** (differential tilt past vertical) --
  it is not used in the hover-to-cruise transition itself, only in
  hover. **In forward/cruise flight the front motors fully stop and fold
  their propeller blades** -- they contribute zero thrust in cruise, so
  they never actually operate AT the 90 deg end of their own range.
- **Rear motor**: a pusher with a **high-pitch propeller** (sized for
  cruise efficiency, not hover). **90 deg to 180 deg**, continuing the
  SAME axis past the shared horizontal reference: 90 deg is horizontal
  (pointing aft, pusher/cruise orientation -- opposite direction from the
  front pair's 90 deg, since the rear nacelle faces backward), 180 deg is
  vertical/hover -- the mirror of the front pair's own 0 deg, 90 deg away
  from the shared horizontal reference on the other side. **It is the
  only thruster active in forward/cruise flight, and never stops.**

So the full physical range is a single -20..180 deg sweep: front owns
the low end (-20..90), rear owns the high end (90..180), and both
motors' "vertical/hover" position sits exactly 90 deg from the shared
"horizontal" reference point they touch at -- just on opposite sides,
matching each nacelle's opposite mounted-facing direction (front faces
forward, rear faces backward).

This is still **two structurally different tilt roles sharing one axis**,
not one uniform "all rotors move identically" mechanism -- front stops
+ folds at its end of the range and has an extra yaw-trim sub-range the
rear doesn't; rear never stops and is the sole forward-flight thruster.
But a single shared axis/reference point is a much more promising fit
for ArduPilot's tiltrotor mixer than two unrelated conventions would be
-- see the spike below.

## Open risk -- RESOLVED (geometry spike, 2026-09-02)

**Hypothesis 1 is confirmed**, with one material refinement: the rear
tilt servo does not need to be a `Q_TILT_MASK` member at all. The
statement of the risk and the three candidate answers are kept below for
the record; the confirmed answer follows.

### Confirmed mechanism

ArduPilot computes **one** normalized transition value, `current_tilt`
(0..1), and hands it to every tilt servo. Two facts make the asymmetric
geometry fall out for free:

1. **`Q_TILT_TYPE=2` (VectoredYaw) drives three distinct servo
   functions** off that single value:
   `k_tiltMotorLeft` (75) and `k_tiltMotorRight` (76) get
   `base_output ± yaw_offset`, while `k_tiltMotorRear` (45) gets
   `base_output` alone, with **no** yaw term
   (`ArduPlane/tiltrotor.cpp:655-659`). That is exactly this vehicle:
   front pair differential-tilts for hover yaw, rear does not.
2. **Each servo's own `SERVOn_MIN`/`SERVOn_MAX`/`SERVOn_REVERSED`
   independently maps that shared 0..1000 value to PWM**
   (`SRV_Channel::pwm_from_range`, `SRV_Channel.cpp:186-196`). Setting
   `SERVOn_REVERSED=1` on the rear makes it sweep the opposite way
   along the shared axis from the same transition value.

`Q_TILT_YAW_ANGLE` is the ArduPilot equivalent of PX4's
`CA_SV_TL0/1_CT=Yaw`. It extends the front pair's travel *past vertical*
by that many degrees and reserves the extra range for yaw trim:
`total_angle = 90 + Q_TILT_YAW_ANGLE + Q_TILT_FIX_ANGLE`, and hover sits
at `zero_out = Q_TILT_YAW_ANGLE / total_angle`
(`tiltrotor.cpp:553-560`). With `Q_TILT_YAW_ANGLE=20` this *is* the
front pair's -20..0 deg hover-only yaw-trim sub-range.

Verified live on a `quadplane-tilttrivec` SITL frame with
`Q_TILT_YAW_ANGLE=20`, front tilts on SERVO12/13 (1000-2000, not
reversed) and a rear tilt added on SERVO14 (function 45, 1000-2000,
`REVERSED=1`):

| state | front (ch12/13) | rear (ch14) |
|---|---|---|
| hover (`current_tilt=0`) | 1181 = **0 deg** | 1818 = **180 deg** |
| forward (`current_tilt=1`) | 2000 = **90 deg** | 1000 = **90 deg** |
| hover + half right rudder | 1289 / 1073 (differential) | 1818 (**unmoved**) |

Both servos are the same linear scale, 0.11 deg/us; only the offset
differs. The rear tracks the front in exact lockstep and in the opposite
direction, and the yaw trim moves the front pair only.

`Q_TILT_MASK` stays at **3** (motors 1+2, the front pair). Motor order
for `Q_FRAME_CLASS=7` is motor 1 = front right, motor 2 = front left,
motor 4 = rear (`AP_MotorsTri.cpp:109-111`); the rear tilt servo is
driven by the vectoring code regardless of mask membership, so the rear
motor does not join the mask.

### Flagged for Phase 2, not approximated away now

In pure fixed-wing flight `continuous_update()` runs the **masked**
motors as forward thrusters (`tiltrotor.cpp:248-249`) -- i.e. ArduPilot
wants the front pair to push in cruise, the exact opposite of this
vehicle's stop-and-fold front rotors, and it shuts down the unmasked
rear, which is meant to be the sole cruise thruster. Phase 1 is
hover-only (see Non-goals), so this is recorded, not solved. It will
need either a fixed forward motor (`SERVO3_FUNCTION=70`) on the rear or
Lua scripting when cruise is actually flown.

### Original statement of the risk (kept for context)

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
1. **Concrete hypothesis worth testing first**, suggested by the shared
   axis above: ArduPilot's tiltrotor mixer likely outputs one normalized
   hover-to-cruise transition value to every `Q_TILT_MASK` motor, and
   each physical tilt servo's own `SERVOn_MIN`/`SERVOn_MAX` (and
   `SERVOn_REVERSED` if needed) calibration maps that shared value to
   real degrees independently per channel. If so, front and rear CAN
   both be in `Q_TILT_MASK` together: front's servo calibrated so the
   transition sweeps its 0->90 deg, rear's calibrated (reversed) so the
   same transition value sweeps its 180->90 deg. Verify this against
   ArduPilot's own tiltrotor source (`AP_MotorsTiltrotor`), not just the
   docs page, before relying on it -- this is a plausible read of how
   the params fit together, not a confirmed fact.
2. If (1) doesn't hold, whether the rear pusher should instead be
   modeled OUTSIDE `Q_TILT_MASK` entirely -- e.g. a plain always-on motor
   with its own tilt servo driven independently (`SERVOn_FUNCTION`) or
   custom scripting, decoupled from
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
