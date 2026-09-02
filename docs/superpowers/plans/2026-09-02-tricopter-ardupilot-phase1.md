# Tricopter on ArduPilot, Phase 1 (flight only) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Get this project's tricopter VTOL armed, hovering stably, and
manually flyable through the existing browser viewer, running under
ArduPilot SITL instead of PX4, on the `tricopter-ap` branch.

**Architecture:** ArduPilot's `arduplane` SITL binary (QuadPlane mode,
`Q_FRAME_CLASS=7`) replaces PX4+Micro-XRCE-DDS-Agent; `ardupilot_gazebo`'s
single `ArduPilotPlugin` (one plugin block driving all motors/servos over
a local FDM socket) replaces PX4's per-motor `gz-sim-multicopter-motor-
model-system`/`gz-sim-joint-position-controller-system` plugin blocks in
a new, parallel Gazebo model; MAVROS bridges ArduPilot to ROS2, replacing
`px4_msgs`, for the manual-flight web control panel.

**Tech Stack:** ArduPilot (ArduPlane/QuadPlane), `ardupilot_gazebo`
(Gazebo Harmonic plugin), `ros-jazzy-mavros`, Gazebo Harmonic (existing
`urban_canyon.sdf` world, unmodified), Python/rclpy (`control_server.py`).

**Spec:** `docs/superpowers/specs/2026-09-02-tricopter-ardupilot-phase1-design.md`

## Global Constraints

- Branch `tricopter-ap` only. Never modify anything on `tricopter` (PX4)
  as part of this plan -- `src/aerocanyon/models/tricopter/`,
  `src/aerocanyon/airframes/4022_gz_tricopter`, and `run_trial.py` stay
  untouched.
- Tilt geometry is a single shared -20..180 deg axis (spec's Geometry
  section): front pair -20..90 deg (0=hover, 90=horizontal-forward,
  -20..0=hover-only yaw trim, stops+folds in cruise), rear 90..180 deg
  (90=horizontal-aft/pusher, 180=hover, never stops, sole forward-flight
  thruster).
- Every verification step restarts the WHOLE stack (Gazebo + SITL +
  bridge) from a clean process state first -- this session's PX4 work
  hit real, reproducible corruption from testing against an
  already-armed/already-tested process. Never trust a test result from a
  process that has been armed/tested before in the same run.
- After any ArduPilot param file change, verify the change actually
  reached whatever SITL reads at boot before trusting a flight test --
  the PX4 branch had a build-staging gotcha (edits to a source file
  silently not reaching the running binary) that invalidated a whole
  session's testing before it was caught. Confirm the equivalent doesn't
  exist here (or does, and is worked around) before relying on param
  edits.
- Gazebo runs headless (`-s` equivalent / no GUI process) for all
  verification in this plan. Never kill an individual Gazebo child
  process (e.g. a GUI subprocess) -- this session found that killing just
  the GUI child of a combined server+GUI `gz sim` process took the whole
  instance down. If a process needs stopping, stop the whole `gz sim`
  process group.

---

### Task 1: Prerequisites -- ArduPilot + ardupilot_gazebo, own clones for this project

**Files:**
- Create (outside repo, documented in README later): `~/ardupilot`,
  `~/ardupilot_gazebo`

**Interfaces:**
- Consumes: nothing (first task)
- Produces: `~/ardupilot/build/sitl/bin/arduplane` (SITL binary),
  `~/ardupilot_gazebo/build/libArduPilotPlugin.so` (Gazebo plugin) --
  every later task's SITL/Gazebo steps depend on both existing and
  running.

This project keeps its own dedicated ArduPilot checkout, the same way
the PX4 branch has its own `~/PX4-Autopilot` rather than sharing one
with any other project on this machine. (A sibling project,
`~/CaveX-Explorer-Pro`, already has both ArduPilot and `ardupilot_gazebo`
cloned and built on this same machine -- confirms the toolchain and
system dependencies already work here, and `ccache` is installed and
populated, so this build should be materially faster than a true cold
build. Do not point this project's config at CaveX's clone; build your
own.)

- [ ] **Step 1: Clone ArduPilot**

```bash
git clone --recursive https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
git log --oneline -1   # record this commit -- pin it the same way px4_msgs is pinned to a PX4 commit
```

- [ ] **Step 2: Install build prerequisites and build ArduPlane for SITL**

```bash
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf plane
```

- [ ] **Step 3: Verify the SITL binary runs**

```bash
ls -la ~/ardupilot/build/sitl/bin/arduplane
timeout 5 ~/ardupilot/build/sitl/bin/arduplane --help 2>&1 | head -5
```

Expected: the binary exists and `--help` prints ArduPilot's usage text
(not a missing-library error). A `timeout`-induced exit is fine here --
this is just confirming the binary loads and runs, not a full boot test.

- [ ] **Step 4: Clone and build ardupilot_gazebo**

```bash
source /opt/ros/jazzy/setup.bash
git clone https://github.com/ArduPilot/ardupilot_gazebo.git ~/ardupilot_gazebo
cd ~/ardupilot_gazebo
git log --oneline -1   # record this commit too
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
```

- [ ] **Step 5: Verify the plugin built**

```bash
ls -la ~/ardupilot_gazebo/build/libArduPilotPlugin.so
```

Expected: the `.so` file exists (matches what's already built at
`~/CaveX-Explorer-Pro/ardupilot_gazebo/build/libArduPilotPlugin.so`,
confirming this is the right build target for this Gazebo version).

- [ ] **Step 6: Install MAVROS**

```bash
sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-extras
source /opt/ros/jazzy/setup.bash
ros2 pkg list | grep mavros
```

Expected: `mavros` and related packages listed.

- [ ] **Step 7: Commit -- record the pinned commits in a tracking note**

No repo files changed yet in this task (clones live outside the repo),
but record the two commit hashes from steps 1 and 4 -- they go into
README.md's prerequisites section in Task 7. Keep them in your working
notes for now; nothing to `git commit` yet.

---

### Task 2: Geometry spike -- resolve how ArduPilot models the asymmetric tilt

**Files:**
- Modify (if the hypothesis needs correcting): `docs/superpowers/specs/2026-09-02-tricopter-ardupilot-phase1-design.md`

**Interfaces:**
- Consumes: `~/ardupilot/build/sitl/bin/arduplane` (Task 1)
- Produces: a confirmed answer -- either "front+rear share one
  `Q_TILT_MASK` group with per-servo SERVO_MIN/MAX/REVERSED calibration"
  (spec's hypothesis 1) or "rear is a separate, non-`Q_TILT_MASK` tilt
  servo" (spec's hypothesis 2) -- that Task 4's param file and Task 3's
  model plugin config both depend on.

This is the spec's "open risk, resolved first" -- do not proceed to
Task 3/4 with an assumed answer.

- [ ] **Step 1: Read `AP_MotorsTiltrotor` source directly**

```bash
grep -rn "tilt_mask\|_tilt_angle\|SERVO_MIN\|SERVO_MAX\|is_motor_tilting" \
  ~/ardupilot/libraries/AP_Motors/AP_MotorsTiltrotor.cpp \
  ~/ardupilot/libraries/AP_Motors/AP_MotorsTiltrotor.h
```

Look specifically for: does the tilt angle computed for the transition
get sent as one shared normalized value to every `Q_TILT_MASK` motor's
servo output (which would then rely on each `SERVOn_MIN/MAX/REVERSED`
to produce different real degrees per motor), or does the code compute
per-motor angles directly in degrees (which would make a shared
calibration-based trick impossible, since the code itself would need to
know each motor's real range)?

- [ ] **Step 2: Boot a stock SITL frame and inspect available params**

```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane-tilttri --console --map -w -M --no-mavproxy &
sleep 20
# in another terminal, once SITL is up:
mavproxy.py --master=tcp:127.0.0.1:5760 --cmd="param show Q_TILT_MASK; param show SERVO*_MIN; param show SERVO*_MAX; param show SERVO*_REVERSED; param show Q_TILT_MAX; param show Q_TILT_YAW_ANGLE"
```

(`quadplane-tilttri` is ArduPilot's own built-in tilt-tricopter SITL
frame -- use it as a known-working starting point rather than a blank
frame, so you're inspecting real defaults, not guessing at param names.)

Expected: confirms whether `Q_TILT_YAW_ANGLE` (or similarly-named param)
exists -- this would be the ArduPilot equivalent of PX4's
`CA_SV_TL0/1_CT=Yaw` hover-yaw-trim mechanism the front pair's -20..0 deg
sub-range needs.

- [ ] **Step 3: Test the hypothesis -- set asymmetric SERVO calibration**

With SITL still running from Step 2, set:

```
param set SERVO_TILT_FRONT_MIN <value mapping to 0 deg on the front's own scale>
param set SERVO_TILT_FRONT_MAX <value mapping to 90 deg>
param set SERVO_TILT_REAR_MIN <value mapping to 180 deg>
param set SERVO_TILT_REAR_MAX <value mapping to 90 deg>
param set SERVO_TILT_REAR_REVERSED 1
```

(Exact `SERVOn_*` channel numbers depend on which channels
`quadplane-tilttri` assigns to which motor -- read them from Step 2's
`param show` output rather than guessing; substitute the real `SERVOn_`
names here.) Command the vehicle through `Q_TILT_MASK`'s transition (or
directly command a tilt via MAVProxy's `tilt` equivalent / RC override on
the tilt channel) and watch via `param show` + the SITL console whether
front and rear servo outputs move in the expected opposite directions
for the same underlying transition value.

- [ ] **Step 4: Record the result**

If Step 3 confirms the shared-value/per-servo-calibration hypothesis:
note the confirmed channel/param mapping in a comment at the top of the
Task 4 param file (write it now, as a plain-text note, even before Task
4 exists as a file -- e.g. append it to this plan's own notes or a
scratch file, whichever is easier to carry into Task 4).

If Step 3 does NOT confirm it: update
`docs/superpowers/specs/2026-09-02-tricopter-ardupilot-phase1-design.md`'s
"Open risk" section with what was actually found, and follow the spec's
fallback (rear modeled outside `Q_TILT_MASK`, likely via
`SERVOn_FUNCTION` = a generic/scripted output rather than a `Q_TILT`
member). Commit the spec update before continuing.

- [ ] **Step 5: Stop SITL, commit if the spec changed**

```bash
pkill -f arduplane
pkill -f mavproxy
```

```bash
cd /home/parvu/AeroCanyon_Guard
git add docs/superpowers/specs/2026-09-02-tricopter-ardupilot-phase1-design.md
git commit -m "spec: record geometry spike result" # only if the file actually changed
```

---

### Task 3: New Gazebo model -- `src/aerocanyon/models/tricopter_ap/model.sdf`

**Files:**
- Create: `src/aerocanyon/models/tricopter_ap/model.sdf`
- Create: `src/aerocanyon/models/tricopter_ap/model.config`
- Reference (read-only): `src/aerocanyon/models/tricopter/model.sdf`
  (existing PX4 model -- copy visual/collision/link/sensor blocks
  verbatim from here, only the motor/servo PLUGIN blocks change)

**Interfaces:**
- Consumes: Task 2's confirmed tilt-mechanism answer; the existing
  `tricopter/model.sdf`'s link names (`base_link`, `motor_0`/`rotor_0`,
  `motor_1`/`rotor_1`, `motor_2`/`rotor_2`) and joint names
  (`motor_0_joint`, `motor_1_joint`, `motor_2_joint`,
  `rotor_0_joint`/`rotor_1_joint`/`rotor_2_joint`) -- keep these
  identical so the visual mesh/inertial work carries over unchanged.
- Produces: `model://tricopter_ap` resolvable once `GZ_SIM_RESOURCE_PATH`
  includes this directory (Task 5) -- the model Gazebo spawns for every
  later flight test.

- [ ] **Step 1: Copy the base model, strip PX4-specific plugins**

```bash
cp -r /home/parvu/AeroCanyon_Guard/src/aerocanyon/models/tricopter \
      /home/parvu/AeroCanyon_Guard/src/aerocanyon/models/tricopter_ap
cd /home/parvu/AeroCanyon_Guard/src/aerocanyon/models/tricopter_ap
```

Edit `model.config`: change `<name>` from `tricopter` to `tricopter_ap`
so it doesn't collide with the PX4 model in `gz model --list`.

In `model.sdf`, delete every `<plugin filename="gz-sim-multicopter-
motor-model-system" ...>...</plugin>` block (3 of them, motor_0/1/2) and
every `<plugin filename="gz-sim-joint-position-controller-system"
...>...</plugin>` block tied to `motor_0_joint`/`motor_1_joint`/
`motor_2_joint` (the tilt servos -- leave the elevon/elevator/rudder
`JointPositionController` blocks alone for now, out of scope for Phase 1
hover flight, though harmless to leave in place).

- [ ] **Step 2: Add the single `ArduPilotPlugin` block**

Add this block once, as a direct child of `<model>` (not per-motor) --
based on the verified working structure in
`~/CaveX-Explorer-Pro/ros2_ws/src/cavex_tracked_vehicle/models/blueboat/model.sdf`:

```xml
<plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
  <fdm_addr>127.0.0.1</fdm_addr>
  <fdm_port_in>9002</fdm_port_in>
  <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
  <lock_step>1</lock_step>
  <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
  <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
  <imuName>base_link::imu_sensor</imuName>

  <!-- Motor 0 (front, per Task 2's confirmed SERVO_FUNCTION channel) -->
  <control channel="0">
    <jointName>rotor_0_joint</jointName>
    <useForces>1</useForces>
    <multiplier>800</multiplier>
    <offset>0</offset>
    <servo_min>1000</servo_min>
    <servo_max>2000</servo_max>
    <type>VELOCITY</type>
    <p_gain>0.20</p_gain>
    <i_gain>0</i_gain>
    <d_gain>0</d_gain>
  </control>
  <!-- Repeat <control> blocks for motor_1_joint's rotor and motor_2's
       rotor, using the channel numbers Task 2 Step 2's `param show`
       output assigned to those motors), plus one PER TILT SERVO once
       Task 2 confirms whether front/rear share one Q_TILT_MASK group
       (one shared control block referenced by two joints is NOT valid
       SDF -- if they share a transition value but need different real
       angles, that's TWO <control> blocks, one per joint, each with its
       OWN multiplier/offset per Task 2's Step 3 findings) or the rear
       needs a separate SERVOn_FUNCTION control block outside the
       tiltrotor group entirely. -->
</plugin>
```

The `<control channel="N">` numbers, and whether motor thrust uses
`type=VELOCITY` (rotor speed, matching this vehicle's existing
`MulticopterMotorModel` velocity-based PX4 setup) vs `type=COMMAND`
(matches the blueboat reference exactly but that's thrust-based, not
RPM-based) is an empirical question -- boot Gazebo+SITL together (Task
5) and confirm the motors actually spin the right direction and produce
lift before treating any of these numbers as final. Do not treat the
placeholder `multiplier`/`p_gain` values above as tuned -- they are a
structurally-valid starting point only, the same way the PX4 branch's
`CA_ROTORi_CT` needed live empirical tuning rather than a value derived
up front.

- [ ] **Step 3: Sanity-check the SDF is well-formed**

```bash
source /opt/ros/jazzy/setup.bash
gz sdf --check /home/parvu/AeroCanyon_Guard/src/aerocanyon/models/tricopter_ap/model.sdf
```

Expected: no errors.

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/AeroCanyon_Guard
git add src/aerocanyon/models/tricopter_ap/
git commit -m "Add tricopter_ap Gazebo model with ArduPilotPlugin (motor/tilt mapping WIP, see Task 5)"
```

---

### Task 4: ArduPilot parameter file -- `src/aerocanyon/ardupilot/tricopter.parm`

**Files:**
- Create: `src/aerocanyon/ardupilot/tricopter.parm`

**Interfaces:**
- Consumes: Task 2's confirmed tilt mechanism; this session's PX4
  empirical tuning values as a reference point (`MC_ROLL_P=5.0`,
  `MC_ROLLRATE_P/I/D=0.2/0.2/0.004`, `MC_YAWRATE_P/I/D=0.16/0.027/0.004`,
  hover thrust fraction ~0.7 -- **reference only, not a direct port**,
  ArduPilot's rate controller uses different gain scaling than PX4's).
- Produces: the param set Task 5 loads at SITL boot -- everything from
  here on assumes these params are present.

- [ ] **Step 1: Write the base frame/tilt params**

```
# src/aerocanyon/ardupilot/tricopter.parm
# QuadPlane tilt-tricopter -- see docs/superpowers/specs/2026-09-02-tricopter-ardupilot-phase1-design.md
# for the geometry this encodes (front -20..90deg incl. hover-yaw-trim
# sub-range + stop/fold in cruise, rear 90..180deg always-on pusher).
Q_ENABLE        1
Q_FRAME_CLASS   7
Q_FRAME_TYPE    1
Q_TILT_MASK     # copy the exact integer Task 2 Step 4 recorded, from that step's SITL param inspection
Q_TILT_TYPE     0
Q_TILT_MAX      # copy the exact degree value Task 2 Step 4 recorded (total sweep the servos are calibrated to cover)
Q_TILT_RATE_UP  40
Q_TILT_RATE_DN  40
```

Fill in `Q_TILT_MASK`/`Q_TILT_MAX` from Task 2's Step 4 recorded result,
not a guess.

- [ ] **Step 2: Write the rate-gain starting point, referencing PX4's tuning**

```
# Rate gains: starting point ONLY, reasoned from this session's PX4
# empirical tuning (MC_ROLL_P=5.0, MC_ROLLRATE_P/I/D=0.2/0.2/0.004,
# MC_YAWRATE_P/I/D=0.16/0.027/0.004) -- ArduPilot's rate controller
# scales gains differently than PX4's allocator-mediated ones, so these
# are a REASONED GUESS to start iterating from in Task 5, not a port.
Q_A_RAT_RLL_P   0.20
Q_A_RAT_RLL_I   0.20
Q_A_RAT_RLL_D   0.004
Q_A_RAT_PIT_P   0.20
Q_A_RAT_PIT_I   0.20
Q_A_RAT_PIT_D   0.004
Q_A_RAT_YAW_P   0.16
Q_A_RAT_YAW_I   0.027
Q_A_RAT_YAW_D   0.004
```

- [ ] **Step 3: Write arming/simulation params**

```
ARMING_CHECK    0
SIM_SPEEDUP     1
FRAME_CLASS     1
SERIAL0_PROTOCOL 2
```

(`ARMING_CHECK 0` mirrors the PX4 branch's own `param set NAV_DLL_ACT 0`
-- disables the GCS-required-for-arming type checks so headless SITL
testing/manual flying doesn't get blocked by checks meant for real
hardware. Revisit if this masks a real problem later.)

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/AeroCanyon_Guard
git add src/aerocanyon/ardupilot/tricopter.parm
git commit -m "Add ArduPilot tricopter param file (frame/tilt from spike, gains as a PX4-informed starting point)"
```

---

### Task 5: First boot -- verify spawn, arm, and level stable hover

**Files:**
- Modify (if boot reveals a config bug): `src/aerocanyon/models/tricopter_ap/model.sdf`,
  `src/aerocanyon/ardupilot/tricopter.parm`

**Interfaces:**
- Consumes: Tasks 1-4's outputs (binaries, model, param file)
- Produces: a running, armable, stably-hovering vehicle in Gazebo --
  the prerequisite every later manual-control task builds on.

- [ ] **Step 1: Confirm nothing is already running**

```bash
ps -eo pid,cmd | grep -E "gz sim|arduplane|mavproxy" | grep -v grep
```

Expected: no output. If anything is running from Task 2's spike, kill it
first (whole process, not a child of it -- see Global Constraints).

- [ ] **Step 2: Start Gazebo headless with the new model/world**

```bash
export GZ_SIM_RESOURCE_PATH=/home/parvu/AeroCanyon_Guard/src/aerocanyon/models:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/parvu/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
source /opt/ros/jazzy/setup.bash
gz sim -v 2 -s -r /home/parvu/PX4-Autopilot/Tools/simulation/gz/worlds/urban_canyon.sdf &
sleep 5
gz model --list
```

(Reuses the existing `urban_canyon.sdf` world file as-is, per the spec
-- it's not PX4-specific, just a world definition.) You'll need to spawn
`model://tricopter_ap` into this world -- either add a `<include>` block
temporarily to a copy of the world file for this test, or use `gz
service` to spawn it directly; either is fine for this verification step,
just don't commit a modified `urban_canyon.sdf` (Global Constraints: this
project's Gazebo world stays shared/unmodified between the PX4 and
ArduPilot paths).

- [ ] **Step 3: Start ArduPilot SITL with the param file**

```bash
cd ~/ardupilot
./build/sitl/bin/arduplane --model JSON --home 47.397742,8.545594,488,0 \
  --add-param-file=/home/parvu/AeroCanyon_Guard/src/aerocanyon/ardupilot/tricopter.parm &
sleep 10
```

- [ ] **Step 4: Verify the param file actually applied**

```bash
mavproxy.py --master=tcp:127.0.0.1:5760 --cmd="param show Q_FRAME_CLASS; param show Q_TILT_MASK"
```

Expected: values match what Task 4 wrote. If they don't (defaults
showing instead), STOP -- find out why before doing anything else. This
is the equivalent check for the PX4 branch's build-staging gotcha; do
not assume `--add-param-file` worked just because SITL booted without
error.

- [ ] **Step 5: Arm and command a hover, watch attitude**

```bash
mavproxy.py --master=tcp:127.0.0.1:5760 --cmd="mode QHOVER; arm throttle; rc 3 1600"
sleep 8
mavproxy.py --master=tcp:127.0.0.1:5760 --cmd="status"
```

Watch roll/pitch in the status output (or via `gz topic -e -t
/world/urban_canyon/pose/info` for the model's actual orientation).
Expected, matching the Phase 1 exit criteria: level attitude (roll/pitch
within a couple degrees), no sustained yaw rotation, and the model
actually gaining altitude in Gazebo (not stuck at idle, not tumbling).

If it doesn't hover cleanly: this is expected to take iteration, exactly
like the PX4 branch's `CA_ROTORi_CT`/`MPC_THR_HOVER`/rate-gain tuning did
this session. Adjust Task 3's `<control>` multipliers or Task 4's rate
gains, **restart the WHOLE stack fresh** (Global Constraints) before each
retest, and don't move to Task 6 until a clean hover is confirmed.

- [ ] **Step 6: Stop everything cleanly**

```bash
pkill -f arduplane
pkill -f mavproxy
pkill -f "gz sim"
```

- [ ] **Step 7: Commit whatever tuning changes made it hover**

```bash
cd /home/parvu/AeroCanyon_Guard
git add src/aerocanyon/models/tricopter_ap/model.sdf src/aerocanyon/ardupilot/tricopter.parm
git commit -m "Tune tricopter_ap motor mapping and rate gains for a stable ArduPilot hover"
```

---

### Task 6: MAVROS bridge, browser 3D viewer, and manual web control

**Files:**
- Modify: `web_viewer/control_server.py`
- Test: manual verification via the browser (this is a live-hardware-style
  integration surface, matching how `control_server.py` was verified all
  session on the PX4 branch -- no unit tests for the flight loop itself)

**Interfaces:**
- Consumes: Task 5's confirmed-hovering vehicle; MAVROS's
  `mavros_msgs` (installed in Task 1) -- specifically
  `mavros_msgs.srv.CommandBool` (arm/disarm),
  `mavros_msgs.srv.SetMode` (mode changes, e.g. GUIDED/QHOVER),
  `mavros_msgs.msg.PositionTarget` or `TwistStamped` published to
  `/mavros/setpoint_velocity/cmd_vel` (velocity setpoints, replacing
  PX4's `OffboardControlMode`+`TrajectorySetpoint` pair).
- Produces: the same HTTP API surface `index.html` already expects
  (`/api/stick?throttle=&roll=&pitch=&yaw=`, `/api/manual?cmd=arm|
  disarm|land`, `/api/rc_status`) -- **`index.html` itself is NOT
  modified**, only what's behind these endpoints changes.

- [ ] **Step 1: Start MAVROS against the SITL instance from Task 5**

```bash
source /opt/ros/jazzy/setup.bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=tcp://127.0.0.1:5760 &
sleep 5
ros2 topic list | grep mavros
```

Expected: topics like `/mavros/state`, `/mavros/local_position/pose`
listed.

- [ ] **Step 2: Rewrite `control_server.py`'s imports and node setup**

Replace the `px4_msgs` import block (currently `from px4_msgs.msg import
(OffboardControlMode, TrajectorySetpoint, VehicleAttitude,
VehicleCommand)`) with:

```python
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import TwistStamped, PoseStamped
```

- [ ] **Step 3: Replace the attitude subscription**

Where the node currently subscribes to `/fmu/out/vehicle_attitude`
(`VehicleAttitude`), subscribe instead to
`/mavros/local_position/pose` (`PoseStamped`) -- its `.pose.orientation`
gives the same quaternion `stick_to_velocity` needs, just via
`PoseStamped.pose.orientation` instead of `VehicleAttitude.q`.

- [ ] **Step 4: Replace the velocity setpoint publisher**

Where the node currently publishes `OffboardControlMode` +
`TrajectorySetpoint` to `/fmu/in/offboard_control_mode`/
`/fmu/in/trajectory_setpoint` every tick, publish a single
`TwistStamped` to `/mavros/setpoint_velocity/cmd_vel` instead --
`twist.linear.{x,y,z}` for the velocity vector `stick_to_velocity`
already computes, `twist.angular.z` for yaw rate. No mode-heartbeat
message is needed the way PX4's `OffboardControlMode` was -- MAVROS/
ArduPilot's GUIDED mode doesn't require a continuous "I intend to stay
in this mode" stream the same way.

- [ ] **Step 5: Replace arm/disarm/mode-change commands**

Where `apply_command()` currently calls `_send_command(VehicleCommand.
VEHICLE_CMD_COMPONENT_ARM_DISARM, ...)` and `VEHICLE_CMD_DO_SET_MODE`,
call MAVROS services instead:

```python
arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
mode_client = self.create_client(SetMode, '/mavros/set_mode')

# arm:
req = CommandBool.Request()
req.value = True
arm_client.call_async(req)

# switch to GUIDED (offboard-equivalent) before arming, same ordering PX4 used:
req = SetMode.Request()
req.custom_mode = 'GUIDED'
mode_client.call_async(req)
```

For `land`, call `SetMode` with `custom_mode = 'QLAND'` (ArduPilot's
QuadPlane vertical-landing mode) instead of PX4's
`VEHICLE_CMD_NAV_LAND`.

Remove the `transition_fw`/`transition_mc` commands entirely for
Phase 1 -- forward-flight transition is explicitly out of scope (Global
Constraints / spec Non-goals), and ArduPilot's transition trigger
(`Q_TRANSITION_MS` / auto-transition on airspeed, or a manual mode
change to `FBWA`) is different enough from PX4's
`VEHICLE_CMD_DO_VTOL_TRANSITION` that porting it now would be
speculative. `index.html`'s transition buttons can stay in place calling
`/api/manual?cmd=transition_fw` -- just don't wire that command to
anything in Phase 1's `control_server.py` (falls through as an unhandled
command, same as any other invalid `cmd` value already does).

- [ ] **Step 6: Fresh-restart the whole stack and verify manual control end-to-end**

```bash
pkill -f arduplane; pkill -f mavros_node; pkill -f "gz sim"
sleep 2
```

Then repeat Task 5's Steps 2-5 (Gazebo, SITL, param verification, arm),
plus MAVROS, the browser 3D-viewer bridge, and the control server:

```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=tcp://127.0.0.1:5760 &
sleep 5

cd /home/parvu/AeroCanyon_Guard/web_viewer
# Websocket bridge for the browser 3D view -- unchanged from the PX4
# branch's own setup (Gazebo-side only, doesn't care which autopilot is
# driving it, see spec's Architecture section). Use the direct plugin
# binary, not the `gz launch` subcommand -- this session found the
# subcommand can silently stop resolving depending on which
# gz_tools_vendor is first on PATH, while the plugin binary always works.
/usr/lib/x86_64-linux-gnu/gz/launch7/gz-launch websocket.gzlaunch &
sleep 2

python3 control_server.py 8080 &
```

Open `http://localhost:8080`: confirm the 3D view actually renders the
vehicle (not just a blank/loading page -- the websocket bridge working
is a separate concern from MAVROS/control_server.py working, verify both
independently), arm via the UI, push the throttle stick up, and confirm
the vehicle climbs and holds level attitude -- matching exactly the
verification this session did for the PX4 branch (single clean
arm/throttle test on a freshly-restarted stack, watching for level
attitude and no yaw spin).

- [ ] **Step 7: Commit**

```bash
cd /home/parvu/AeroCanyon_Guard
git add web_viewer/control_server.py
git commit -m "Port control_server.py from px4_msgs to MAVROS for the ArduPilot branch"
```

---

### Task 7: README documentation and final verification pass

**Files:**
- Modify: `README.md`

**Interfaces:**
- Consumes: everything from Tasks 1-6 -- this task documents the exact
  working sequence, so re-derive each command from what actually worked
  in those tasks rather than writing new ones from scratch.
- Produces: nothing further downstream -- this is Phase 1's last task.

- [ ] **Step 1: Add an ArduPilot prerequisites section to README.md**

Mirror the structure of the existing PX4 "Prerequisites" section:
ArduPilot commit pinned (from Task 1), `ardupilot_gazebo` commit pinned,
`ros-jazzy-mavros`/`ros-jazzy-mavros-extras` apt packages. State plainly
that this section applies to the `tricopter-ap` branch only -- the PX4
section elsewhere in this same file is for the `tricopter` branch and is
unaffected.

- [ ] **Step 2: Add an ArduPilot "fly the tricopter manually" section**

Document the exact working sequence from Task 6's Step 6 (env vars,
Gazebo launch, SITL launch with the param file, MAVROS launch,
`control_server.py` launch, browser URL) as copy-pasteable `$HOME`-
anchored commands, matching this session's established README style
(see the PX4 branch's own "Fly the tricopter manually" section for the
format to match).

- [ ] **Step 3: Note what's explicitly NOT here yet**

Add a short paragraph: Phase 1 covers manual hover flight only: the
autonomous CBF/PINN mission stack (`controller_node.py`, `run_trial.py`
equivalent) and VTOL forward-flight transition are both out of scope,
tracked as Phase 2 in a future spec. Link to
`docs/superpowers/specs/2026-09-02-tricopter-ardupilot-phase1-design.md`.

- [ ] **Step 4: Full fresh-stack verification, one more time**

Repeat Task 6 Step 6's full sequence exactly as now documented in
README.md, copy-pasting the commands FROM the README rather than from
memory -- this catches any transcription mistakes made while writing the
docs. Confirm: clean arm, stable level hover, responsive manual stick
control, all on a stack started fresh from nothing running.

- [ ] **Step 5: Commit**

```bash
cd /home/parvu/AeroCanyon_Guard
git add README.md
git commit -m "Document the ArduPilot manual-flight path (tricopter-ap branch)"
git push -u origin tricopter-ap
```

Phase 1 is complete once this push succeeds and Step 4's verification
passed on the freshly-documented sequence.
