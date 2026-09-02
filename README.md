# AeroCanyon_Guard

ROS2 / Gazebo Harmonic simulation environment for **AeroCanyon-Guard**: a
tricopter VTOL transiting an urban canyon under spatially-varying wind
disturbance, controlled by a fractional-order physics-informed neural network
(FO-PINN) with a control-barrier-function (CBF) safety filter.

Built on ArduPilot SITL (`ArduPlane`, `Q_FRAME_CLASS=7` tricopter-VTOL) +
MAVROS for ROS2 ↔ ArduPilot bridging.

## Packages

- **aerocanyon** — the primary scenario: canyon geometry, wind field, PINN
  wind estimator, CBF safety filter, trial orchestration, and figure
  generation. Autonomous missions (no manual control). **Currently being
  ported from px4_msgs to mavros_msgs** — see
  [What's not here yet](#whats-not-here-yet-phase-2) below; until that
  lands, `controller_node.py`, `run_trial.py`, `trial_logger.py`,
  `fo_pinn_node.py`, and `wind_field_node.py` still import the
  now-removed `px4_msgs` package and will not run.

## Prerequisites

Cloned/built separately, outside this workspace (not part of this repo):

- **ROS2 Jazzy + Gazebo Harmonic:** `sudo apt install ros-jazzy-ros-gz* ros-jazzy-rosidl*`
- **[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)** cloned at
  `$HOME/PX4-Autopilot` -- **asset source only**, not built or run: PX4
  itself, `px4_msgs`, and this project's own PX4 model/airframe were all
  removed once the ArduPilot port landed, but `tricopter_ap`'s model.sdf
  still reuses several stock meshes and the airspeed sensor model
  verbatim from `Tools/simulation/gz/models/` (standard_vtol's wing/
  prop/elevon `.dae` files, `model://airspeed`) rather than vendoring
  copies into this repo. Confirmed live: without this on disk, `gz sim`
  fails the whole world load outright (`Unable to find uri[model://
  airspeed]` and several unresolved mesh URIs) -- not a silent no-op.
- **[ArduPilot](https://github.com/ArduPilot/ardupilot)** cloned
  `--recursive` at `$HOME/ardupilot`, pinned to commit `b9439efde1`, built
  for SITL: `./waf configure --board sitl && ./waf plane` (produces
  `build/sitl/bin/arduplane`)
- **[ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo)**
  cloned at `$HOME/ardupilot_gazebo`, pinned to commit `082a0fe`, built
  against Gazebo Harmonic: `cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j$(nproc)`
  (produces `build/libArduPilotPlugin.so`)
- **MAVROS:** `sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-extras`.
  `ros-jazzy-mavros` is usually already present; `ros-jazzy-mavros-extras`
  has needed a human to install it by hand in this environment (no
  passwordless sudo) -- check with `ros2 pkg list | grep mavros` and run
  the `apt install` above yourself if `mavros_extras` doesn't show up.
- **GeographicLib geoid dataset.** `mavros_node` hard-aborts on startup
  without it (`GeographicLib exception: File not readable
  .../egm96-5.pgm`), and `geographiclib-get-geoids`'s default install
  location needs root. Install it to a user-writable path instead and
  point MAVROS at it with `GEOGRAPHICLIB_DATA`:
  ```bash
  mkdir -p $HOME/.local/share/GeographicLib
  geographiclib-get-geoids -p $HOME/.local/share/GeographicLib egm96-5
  export GEOGRAPHICLIB_DATA=$HOME/.local/share/GeographicLib  # before launching mavros_node
  ```

Note `mavproxy.py` is **not** assumed to be installed anywhere below --
all SITL parameter/telemetry checks in this project were done with
`pymavlink` directly, or standard `ros2`/MAVROS CLI tools.

### The tricopter model

The vehicle is a **tricopter VTOL** in the style of the E-flite
Convergence: two front rotors that tilt `-20..90°` (vertical for hover,
where the sub-range below 0° trims yaw; horizontal and stopped/folded in
cruise) and one rear rotor that tilts `90..180°` on the same shared axis
(vertical for hover, horizontal as the sole forward-flight pusher thruster
-- it never stops). `src/aerocanyon/models/tricopter_ap/` holds the
Gazebo model, driven by ArduPilot's own `ArduPilotPlugin`. Two things are
worth knowing if you read the model file:

- `ArduPilotPlugin`'s motor `VELOCITY` control alone spins the rotors but
  generates **zero aerodynamic lift** -- Gazebo has no idea a spinning
  joint should push air. A `gz-sim-lift-drag-system` "thrust rig" (two
  `LiftDrag` blocks per rotor, opposite `cp` so their off-axis moments
  cancel) is layered on top.
- The vehicle's centre of gravity sits at the **rotor centroid**
  (`x=0.11666...`), not at the model origin. `AP_MotorsTri`'s mixer gives
  all three motors equal thrust at zero roll/pitch demand with no
  per-rotor moment-arm compensation -- so with the front pair and rear
  rotor at different distances from an origin-CG, equal thrust alone
  would leave a standing pitch moment. Moving the CG to the centroid is a
  deliberate, required part of the design, not a bug.

## Build this workspace

```bash
cd $HOME/AeroCanyon_Guard
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## Fly the tricopter manually

Start Gazebo and ArduPilot SITL by hand and keep them running -- useful
to sanity-check a fresh checkout, watch it fly without the trial
harness's own process orchestration, or fly it yourself from a browser or
a physical RC transmitter. **`system_id:=255` on the MAVROS launch line
below is not optional** -- ArduPilot only accepts RC override (and other
GCS commands) from the system id in its `MAV_GCS_SYSID` parameter, which
defaults to **255**. MAVROS itself defaults to system id 1. Get this
wrong and there is **no error anywhere** -- MAVROS connects fine, arming
works fine, `/mavros/rc/override` publishes fine, and the sticks in the
browser just silently do nothing. This is the single easiest thing to get
wrong here.

The default manual-flight world is `src/aerocanyon/worlds/map_zone_ap.sdf`:
a real-world OSM-derived 3D terrain model of the Politehnica/AFI area of
Bucharest (`src/aerocanyon/map_zone/`), matching this project's `--home`
coordinates below, with the tricopter spawned at z=74.2m, 0.2m above the
terrain's z=74 ground level.

```bash
# Terminal 1: Gazebo (headless -- no native GUI needed for the browser viewer below)
source /opt/ros/jazzy/setup.bash
# The last path is PX4-Autopilot's own model library, needed because
# tricopter_ap's model.sdf still reuses several of its stock meshes and
# the airspeed sensor model -- see the Prerequisites note above. Without
# it gz sim fails the whole world load, not a silent/partial failure.
export GZ_SIM_RESOURCE_PATH=$HOME/AeroCanyon_Guard/src/aerocanyon/models:$HOME/AeroCanyon_Guard/src/aerocanyon:$HOME/PX4-Autopilot/Tools/simulation/gz/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
gz sim -v 2 -s -r $HOME/AeroCanyon_Guard/src/aerocanyon/worlds/map_zone_ap.sdf &
sleep 5

# Terminal 2: ArduPilot SITL. NOTE: --defaults, NOT --add-param-file
# (that flag doesn't exist in this ArduPilot build -- arduplane --help
# lists --defaults). mavproxy.py is not installed/required.
mkdir -p /tmp/apstate && cd /tmp/apstate
$HOME/ardupilot/build/sitl/bin/arduplane --model JSON \
  --home 44.434424990487216,26.04781615647584,74,0 \
  --wipe --defaults $HOME/AeroCanyon_Guard/src/aerocanyon/ardupilot/tricopter.parm &
sleep 10

# Terminal 3: MAVROS. system_id:=255 is REQUIRED -- see above.
# gcs_url opens a second TCP port MAVROS listens on for a ground
# control station (QGroundControl: "Comm Link" -> TCP, localhost:5761)
# to connect independently and see the same MAVLink stream, without
# contending with MAVROS for ArduPilot's single SERIAL0 connection on
# 5760.
export GEOGRAPHICLIB_DATA=$HOME/.local/share/GeographicLib
source /opt/ros/jazzy/setup.bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=tcp://127.0.0.1:5760 -p system_id:=255 \
  -p gcs_url:=tcp-l://0.0.0.0:5761@ &
sleep 12

# Terminal 4: browser viewer's Gazebo websocket bridge (scene stream)
cd $HOME/AeroCanyon_Guard/web_viewer
/usr/lib/x86_64-linux-gnu/gz/launch7/gz-launch websocket.gzlaunch &
sleep 4

# Terminal 5: control server (static files + RC-override manual control)
cd $HOME/AeroCanyon_Guard/web_viewer
source /opt/ros/jazzy/setup.bash && source ../install/setup.bash
python3 control_server.py 8080
```

Wait ~30 s total after SITL starts for EKF alignment and a 3D GPS fix
before arming. Then open `http://localhost:8080` and hit `arm`; the
vehicle goes straight to `QHOVER`. (`tricopter.parm` sets `FLTMODE_CH=0`
to disable ArduPilot's RC mode-switch, so channel 8 being pinned to 1500
by `control_server.py` is never misread as a mode-switch position --
without that, the vehicle would briefly pull toward a fixed-wing mode
right after arming before self-settling.)

Two Mode 2 RC-style proportional sticks (left: yaw + throttle, right:
roll + pitch) drive `/mavros/rc/override` (an `OverrideRCIn` message,
i.e. the same four RC channels a real transmitter would send) --
roll/pitch are **body-frame lean commands**, exactly like a real
transmitter's sticks. The throttle stick is ratcheted (no spring-back,
idle-at-bottom by default, ceiling 1900 not 2000); roll/pitch/yaw
self-center. A centred/stale stick resolves to PWM 1500, which in
`QHOVER` means *hold attitude, hold altitude* -- a real RC failsafe
behaviour. `land now` switches to `QLAND` and disarms automatically on
touchdown. The `transition FW`/`transition MC` buttons are inert for
now: `control_server.py` deliberately has no handler for those commands
(forward-flight transition is out of scope until Phase 2 -- see below),
so clicking them fires a `fetch()` that gets an unread HTTP error and
silently does nothing, same as any other unrecognized
`/api/manual?cmd=...` value.

### Fly with a physical RC transmitter instead

`rc_bridge.py` reads a real RC transmitter's USB "simulator" dongle as a
standard Linux joystick (`/dev/input/js0`) and forwards all four Mode 2
axes to `control_server.py`'s `/api/stick` -- no MAVLink/autopilot RC
parameter config involved, this is a second poster to the same
`/api/stick` HTTP surface the browser sticks use. When it's running, the
page's own virtual sticks **hide themselves automatically** (polling
`/api/rc_status`) and show again if the transmitter is unplugged
mid-session -- nothing to toggle by hand.

An earlier dongle tried here (a Tactic transmitter's Nordic
Semiconductor RF adapter, VID:PID `1781:0e58`) turned out not to be
viable: its HID report is an undocumented vendor-specific format with no
public docs, and after extensive live reverse-engineering (holding each
stick at known positions, diffing raw `/dev/hidraw0` reports, checking
trim-button behaviour) pitch's byte never carried a signed direction --
only "how far from center", not which way. The dongle that IS wired up
now (Novatek "ART TECH GAME", VID:PID `0603:1a13`) declares a real HID
joystick usage page, so the kernel's own `usbhid`/`joydev` drivers
create a normal, already-calibrated `/dev/input/js0` -- no protocol
guessing needed. `rc_bridge.py`'s axis mapping (`AXIS_ROLL` etc.) was
still confirmed live, one stick at a time, since axis-number-to-function
isn't standardized across devices.

One-time setup, since this is WSL2 and the dongle needs USB passthrough:
```powershell
# Windows PowerShell (elevated), once usbipd-win is installed (winget install usbipd):
usbipd list                       # find the dongle's BUSID
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```
```bash
# In WSL2, once per attach. modprobe only takes ONE module name -- passing
# more (`modprobe hid hid-generic usbhid joydev`) silently treats the rest
# as parameters to `hid`, not separate modules (dmesg would show "unknown
# parameter 'hid-generic' ignored" etc.), so usbhid never loads and no
# /dev/input node ever appears. Load each one separately:
sudo modprobe hid-generic   # if not already loaded
sudo modprobe usbhid        # the one that actually creates /dev/input/js0
sudo modprobe joydev
sudo chmod 666 /dev/input/js0
```

Then, alongside `control_server.py` from above:
```bash
cd $HOME/AeroCanyon_Guard/web_viewer
python3 rc_bridge.py 8080
```

A different transmitter (even a different unit of the same dongle model)
may use different axis numbers for roll/pitch/throttle/yaw -- confirm
with `python3 -c "import struct; ..."` reading `/dev/input/js0`'s
`struct js_event` stream while moving one stick at a time, the same way
`AXIS_ROLL`/`AXIS_PITCH`/`AXIS_THROTTLE`/`AXIS_YAW` were confirmed here.

## Run the automated baseline/treatment trial

**PENDING PORT — the section below still describes the pre-ArduPilot
(PX4) design.** `controller_node.py`, `run_trial.py`, `trial_logger.py`,
`fo_pinn_node.py`, and `wind_field_node.py` all still import the
now-removed `px4_msgs` package and do not currently run. Porting these
to MAVROS/mavros_msgs, regenerating the wind field for the `map_zone`
terrain, retraining the FO-PINN estimator, and re-running the trial
suite is the active Phase 2 work -- see
[What's not here yet](#whats-not-here-yet-phase-2) below.

### AeroCanyon-Guard: Tricopter Canyon Transit

Autonomous mission: tricopter VTOL transits an urban canyon under
spatially-varying wind disturbance. FO-PINN estimates wind forces, CBF
safety filter prevents collisions and stalls.

### Regenerate the world and the wind grid

```bash
cd $HOME/AeroCanyon_Guard
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 -m aerocanyon.canyon_geometry        # writes worlds/urban_canyon.sdf
python3 -m aerocanyon.canyon_field           # writes data/wind_grid.npy
```

### Train the FO-PINN wind estimator

Collect baseline trials first (see below), then:

```bash
cd $HOME/AeroCanyon_Guard
source .venv/bin/activate
PYTHONPATH=src/aerocanyon python3 -m aerocanyon.train_pinn trials/train*_baseline.csv --alpha 1.0
```

`docs/alpha_sweep.txt` records the alpha-vs-skill sweep used to pick the
fractional order; re-run it against real flight data before trusting the
checkpoint (see `task-8-report.md` — the shipped checkpoint was trained on
synthetic stand-in data because no live autopilot process was available in
that task's environment).

### Run the paired baseline/treatment trial

`run_trial.py` owns the whole stack itself, for both legs: Gazebo and the
autopilot SITL process. There's nothing to start by hand and nothing
external to keep running in another terminal.

**Each leg runs as its own separate OS process** (`run_trial --mode
baseline` / `--mode treatment`, spawned internally by `main()`), and
each of those spawns a completely fresh `gz sim` world just for that
leg, boots the autopilot against it, runs the leg, then tears both
processes back down before the next leg's process even starts. No
entity, physics state, or Python/rclpy state is shared between legs at
all.

Do not start the autopilot SITL by hand alongside `run_trial.py`: its own
spawn will silently lose the port/instance-0-lock race against a
manually-started one and die immediately (it raises with a clear message
if that happens, rather than failing silently).

**Landing behaviour:** both modes land in place once `controller_node`
measures having actually cleared the canyon exit -- not just at a
wall-clock timeout. The VTOL fixed-wing transition stays disabled
(`controller_node.ENABLE_VTOL_TRANSITION = False`) for both legs until
Phase 2 lands -- the whole flight flies in stable multicopter mode; see
History.md for why.

```bash
cd $HOME/AeroCanyon_Guard
source /opt/ros/jazzy/setup.bash && source install/setup.bash
source .venv/bin/activate
python3 -m aerocanyon.run_trial --trial live_full  # --duration defaults to 220s, see --help
python3 -m aerocanyon.plot_results --trial live_full
```

Each leg runs Gazebo headless (`-s`, no native GUI, no X11 needed) and
starts its own `web_viewer/` browser bridge instead -- open
`http://localhost:8080` while a leg is running to watch it fly (same 3D
viewer as [Fly the tricopter manually](#fly-the-tricopter-manually)
above, read-only here since nothing is driving `/api/stick`). Once both
legs finish, `run_trial.py` runs `plot_results` itself, copies the two
figures into `web_viewer/results/`, and prints a `results:` URL --
reload `http://localhost:8080/results.html` to see them without leaving
the browser. `python3 -m aerocanyon.plot_results --trial live_full` (the
explicit form below) is still there for regenerating figures from
existing CSVs without re-flying anything.

`plot_results` reads `trials/live_full_baseline.csv` and
`trials/live_full_treatment.csv` (the same `--trial` name used for the run)
and writes `figures/comparison.png` and `figures/cbf_intervention.png` —
see [View the figures](#view-the-figures) below for what's in them.

**SITL-specific configuration:** see
[History.md](History.md#known-good-arming-and-telemetry-configuration) —
it's the single source of truth for the airframe parameters; don't
duplicate them here, they've drifted out of sync with reality before.

## View the figures

`plot_results.py` writes two figures to `figures/`:

- `comparison.png` — overlaid baseline/treatment trajectories through the
  canyon plus a bar chart of RMS lateral (east) deviation, with the
  percentage reduction in the chart title.
- `cbf_intervention.png` — the barrier value `h(t)` and the filter's
  active/inactive flag over the treatment flight, showing when and how
  often the CBF had to intervene.

Both print their headline numbers to stdout as well, including the RMS
lateral deviation reduction percentage — treat that number honestly: if
it's small or negative, confirm `cbf_active` is non-zero and that the PINN
estimate correlates with wind truth before concluding the pipeline needs
retuning.

See [History.md](History.md) for known pitfalls and their fixes (arming, telemetry, spawning) from this project's debugging history.

## What's not here yet (Phase 2)

Manual hover flight (documented above) works end to end. Explicitly out
of scope, not yet implemented:

- The autonomous CBF/PINN mission stack (`controller_node.py`,
  `run_trial.py`'s automated baseline/treatment trial, `wind_field_node.py`,
  `train_pinn.py`) -- these still import the removed `px4_msgs` package
  and need porting to `mavros_msgs`/MAVROS topics before anything flies a
  mission by itself.
- A wind field regenerated for the `map_zone` terrain (the existing
  `canyon_field.py`/wind-grid pipeline targets the synthetic
  `urban_canyon.sdf` box-canyon geometry).
- VTOL forward-flight transition -- `ArduPlane`'s `GUIDED` mode has no
  velocity-setpoint path for this airframe (see `control_server.py`'s
  module docstring), and ArduPilot's own `tiltrotor.cpp` cruise logic
  (masked/front motors as cruise thrusters, rear shut down) is the
  opposite of this vehicle's real design (front motors fold and stop,
  rear is the sole cruise thruster) -- this needs resolving before
  transition tuning can start, not just gain-tuning.

Tracked in
[`docs/superpowers/specs/2026-09-02-tricopter-ardupilot-phase1-design.md`](docs/superpowers/specs/2026-09-02-tricopter-ardupilot-phase1-design.md).
