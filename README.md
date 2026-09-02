# AeroCanyon_Guard

ROS2 / Gazebo Harmonic simulation environment for **AeroCanyon-Guard**: a
tricopter VTOL transiting an urban canyon under spatially-varying wind
disturbance, controlled by a fractional-order physics-informed neural network
(FO-PINN) with a control-barrier-function (CBF) safety filter.

Built on PX4-Autopilot SITL + Micro-XRCE-DDS for ROS2 ↔ PX4 bridging.

## Packages

- **aerocanyon** — the primary scenario: canyon geometry, wind field, PINN
  wind estimator, CBF safety filter, trial orchestration, and figure
  generation. Autonomous missions (no manual control).
- **px4_msgs** — PX4's ROS2 message definitions (git submodule, pinned to
  the commit this workspace was built against).

## Prerequisites

Cloned/built separately, outside this workspace (not part of this repo):

- **ROS2 Jazzy + Gazebo Harmonic:** `sudo apt install ros-jazzy-ros-gz* ros-jazzy-rosidl*`
- **[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)** cloned at `$HOME/PX4-Autopilot`,
  built for SITL: `make px4_sitl gz_tricopter` (after installing the
  airframe below — it does not exist in stock PX4)
- **[Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)**
  built at `$HOME/Micro-XRCE-DDS-Agent` (bridges ROS2 ↔ PX4)

### PX4 setup for the tricopter + wind

The vehicle is a **tricopter VTOL** in the style of the E-flite
Convergence: three tilting rotors -- two front (which also give yaw in
hover, via differential tilt) and one rear, tiltable too (vertical for
hover, an active pusher in cruise). All three share equal thrust; see
[History.md](History.md) for why the rear used to be undersized to half
the front motors' thrust and what that broke. PX4 has a real-hardware
airframe for the Convergence (ID 13012) but has **never shipped a
simulation model** for it, so this project carries its own — both files
live in this repo and get copied into `PX4-Autopilot`:

1. **Install the tricopter model and airframe:**
   ```bash
   cp -r $HOME/AeroCanyon_Guard/src/aerocanyon/models/tricopter \
         $HOME/PX4-Autopilot/Tools/simulation/gz/models/
   cp $HOME/AeroCanyon_Guard/src/aerocanyon/airframes/4022_gz_tricopter \
      $HOME/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
   # then register it and rebuild
   #   add `4022_gz_tricopter` to that directory's CMakeLists.txt
   cd $HOME/PX4-Autopilot && make px4_sitl gz_tricopter
   ```
   The model is derived from PX4's stock quad `tiltrotor`: one rear rotor
   removed, the other moved to the centreline, and the tilt joints
   widened to `-15..90°` so the allocator can trim the (now inherently
   unbalanced) single rear rotor's torque in both yaw directions.

2. **Place the canyon world:**
   ```bash
   cp $HOME/AeroCanyon_Guard/src/aerocanyon/worlds/urban_canyon.sdf \
      $HOME/PX4-Autopilot/Tools/simulation/gz/worlds/urban_canyon.sdf
   ```
   This world loads `gz-sim-air-speed-system`; without it the model's
   airspeed sensor declares itself but never publishes, and PX4 refuses
   to arm with "Preflight Fail: Airspeed invalid".

3. **Disable GCS requirement for offboard arming:**
   ```bash
   # In PX4 shell:
   param set NAV_DLL_ACT 0
   param save
   ```

Wind effects are already enabled on the model (`<enable_wind>true</enable_wind>`
on `base_link`), and its `base_link_visual` is white rather than the
stock dark grey — purely for visibility against the canyon buildings in
the demo video/GUI, no effect on simulation behaviour.

The airframe file `4022_gz_tricopter` has SITL sensor checks disabled,
and the model's motor topics already match what PX4's `gz_bridge`
publishes -- neither needs editing. See
[History.md](History.md#known-good-arming-and-telemetry-configuration)
for the pitfalls that look like they need a patch here but don't (and the
ones that actually do, elsewhere in the stack).

## Build this workspace

```bash
cd $HOME/AeroCanyon_Guard
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## Fly the tricopter manually

Start Gazebo and PX4 by hand and keep them running -- useful to
sanity-check a fresh PX4/model checkout, watch it fly in the GUI without
the trial harness's own PX4/agent lifecycle management, or fly it
yourself from QGroundControl, a browser, or a physical RC transmitter.
This is the opposite of the [automated trial](#run-the-automated-baselinetreatment-trial)
below, which spawns a completely fresh, split-process Gazebo/PX4 pair
per leg and tears them down itself -- **do not run `run_trial.py` on top
of a manually-started stack**: its own spawn of PX4 or the DDS agent will
silently lose the port/instance-0-lock race against the manually-started
one and die immediately (it raises with a clear message if that happens,
rather than failing silently).

```bash
cd $HOME/PX4-Autopilot
source /opt/ros/jazzy/setup.bash
export GZ_IP=127.0.0.1

# Terminal 1: Gazebo
# MUST source gz_env.sh first (sets GZ_SIM_RESOURCE_PATH) -- without it
# gz-sim can't resolve model://tricopter and the vehicle silently never
# spawns (nothing shows up in `gz model --list`, no error visible unless
# you check the server's own stderr for "Unable to find uri[...]"). If
# another project's shell startup files also export GZ_SIM_RESOURCE_PATH
# (check `env | grep GZ_SIM_RESOURCE_PATH` before sourcing), gz_env.sh
# appends to whatever is already set rather than replacing it, so this
# is safe to source alongside that -- just source it in THIS shell,
# every time, before starting gz sim from it.
source $HOME/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh
gz sim -v 2 Tools/simulation/gz/worlds/urban_canyon.sdf -r -s -g &  # drop -g for headless
sleep 5

# Terminal 2: XRCE-DDS bridge
$HOME/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888 &
sleep 2

# Terminal 3: PX4 SITL
export R="build/px4_sitl_default/rootfs/"
export PX4_SIM_MODEL=gz_tricopter
export PX4_GZ_WORLD=urban_canyon
export PX4_GZ_MODEL_POSE="-100,0,0.246,0,0,0"  # ground, at the canyon entry -- see run_trial.py's SPAWN_POSE
./build/px4_sitl_default/bin/px4 &
sleep 15
```

Then either fly manually from QGroundControl (it auto-connects to PX4
over MAVLink UDP), from the browser (see below), or launch just the
mission nodes without `run_trial.py`'s process orchestration:
`ros2 launch aerocanyon canyon_sim.launch.py mode:=baseline trial:=manual`.

### Fly from a browser instead

`web_viewer/` is a small browser-based Gazebo 3D viewer (ported from a
sibling project) plus a manual control panel, useful when the native `gz
sim` GUI window won't render (a known issue under WSLg -- window shows in
the taskbar but never paints, `[WARN:COPY MODE]` in its log). Like
QGroundControl above, it targets this fly-by-hand path only, not a real
`run_trial.py` leg -- `controller_node` is the sole source of setpoints
during a trial, and this server isn't meant to run alongside one.

One-time setup -- the vehicle's mesh files aren't vendored into this repo,
the browser fetches them straight from PX4-Autopilot's own copy:
```bash
cd $HOME/AeroCanyon_Guard
mkdir -p web_viewer/assets
ln -s $HOME/PX4-Autopilot/Tools/simulation/gz/models/standard_vtol web_viewer/assets/standard_vtol
```

With Gazebo and PX4 already running (previous section):
```bash
# tab 1: browser viewer (scene stream)
cd $HOME/AeroCanyon_Guard
gz launch web_viewer/websocket.gzlaunch

# tab 2: control server (static files + manual offboard velocity setpoints)
cd $HOME/AeroCanyon_Guard/web_viewer
source /opt/ros/jazzy/setup.bash && source ../install/setup.bash
python3 control_server.py 8080
```

Then open `http://localhost:8080` (or the host's LAN IP, from another
device). With no physical RC transmitter connected (see below), two
Mode 2 RC-style proportional sticks (left: yaw + throttle, right: roll +
pitch) stream continuously while touched and self-center on release;
`arm` requests offboard mode + arm, `land now` hands off to PX4's
`AUTO_LAND`. `transition FW` / `transition MC` send PX4
`VEHICLE_CMD_DO_VTOL_TRANSITION` directly -- this bypasses
`controller_node`'s own `ENABLE_VTOL_TRANSITION=False` (that only gates
the autonomous trial's logic, not PX4 itself), but fixed-wing flight has
had no tuning at all here: the front transition currently just times out
and PX4's own quad-chute safety reverts it back to MC, since there's no
airspeed sensor for the open-loop transition timer to use and no
forward-flight/cruise gains have ever been verified. Treat these two
buttons as a hook for future FW tuning work, not a working transition yet.

### Fly with a physical RC transmitter instead

`rc_bridge.py` reads a real RC transmitter's USB "simulator" dongle as a
standard Linux joystick (`/dev/input/js0`) and forwards all four Mode 2
axes to `control_server.py`'s `/api/stick` -- no MAVLink/PX4 RC
parameter config involved, this is a second poster to the same
offboard-velocity path the browser sticks use. When it's running, the
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

### AeroCanyon-Guard: Tricopter Canyon Transit

Autonomous mission: tricopter VTOL transits an urban canyon (6 box buildings)
under spatially-varying wind disturbance. FO-PINN estimates wind forces,
CBF safety filter prevents collisions and stalls.

### Regenerate the world and the wind grid

```bash
cd $HOME/AeroCanyon_Guard
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 -m aerocanyon.canyon_geometry        # writes worlds/urban_canyon.sdf
python3 -m aerocanyon.canyon_field           # writes data/wind_grid.npy
cp src/aerocanyon/worlds/urban_canyon.sdf \
   $HOME/PX4-Autopilot/Tools/simulation/gz/worlds/urban_canyon.sdf
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
synthetic stand-in data because no live PX4 process was available in that
task's environment).

### Run the paired baseline/treatment trial

`run_trial.py` owns the whole stack itself now, for both legs: Gazebo,
PX4 SITL, and the Micro-XRCE-DDS-Agent. There's nothing to start by
hand and nothing external to keep running in another terminal.

**Each leg runs as its own separate OS process** (`run_trial --mode
baseline` / `--mode treatment`, spawned internally by `main()`), and
each of those spawns a completely fresh `gz sim` world just for that
leg, boots PX4 against it, runs the leg, then tears both processes back
down before the next leg's process even starts. No entity, physics
state, or Python/rclpy state is shared between legs at all -- this
replaced an earlier design where one long-lived Gazebo instance was
reused across legs and the vehicle was reset in place between them
(`run_trial._reset_gazebo_model`, kept in the code for the manual-flying
flow above, which still uses an external Gazebo).

Do not start PX4 or the DDS agent by hand alongside `run_trial.py`: see
the warning at the top of [Fly the tricopter manually](#fly-the-tricopter-manually)
above.

**Landing behaviour:** both modes land in place (`VEHICLE_CMD_NAV_LAND`)
once `controller_node` measures having actually cleared the canyon exit
-- not just at a wall-clock timeout. An earlier design instead flew the
vehicle all the way back to the spawn point before landing, to make sure
the next leg's PX4 process wouldn't inherit a drifting or crashed
vehicle's state; that's no longer necessary now that each leg gets its
own fresh Gazebo/PX4 process with nothing shared between legs at all
(see above), so landing in place is simpler and just as safe. Landing
itself is handed off entirely to PX4's own `AUTO_LAND` -- it may turn
the vehicle's heading during the descent, but always lands flat/level
and disarms reliably; see History.md for why a heading-locked
self-controlled descent was tried and rejected (verified live to be
capable of losing control entirely). The VTOL fixed-wing transition
stays disabled (`controller_node.ENABLE_VTOL_TRANSITION = False`) for
both legs — the whole flight flies in stable multicopter mode; see
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

**SITL-specific PX4 configuration:** see
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

## ArduPilot port (`tricopter-ap` branch only)

Everything above this section is the **PX4** stack, and lives on the
`tricopter` branch -- none of it is affected by what follows. This section
is for the **`tricopter-ap`** branch: the same tricopter airframe, ported
to fly under **ArduPilot SITL** (`ArduPlane`, `Q_FRAME_CLASS=7`
tricopter-VTOL) instead of PX4, bridged to ROS2 via **MAVROS** instead of
Micro-XRCE-DDS. Phase 1 covers manual hover flight only -- see
[What's not here yet](#whats-not-here-yet-phase-2) below.

### ArduPilot prerequisites

Cloned/built separately, outside this workspace (not part of this repo),
in addition to the ROS2 Jazzy + Gazebo Harmonic install from the PX4
Prerequisites section above (still required -- only the PX4-Autopilot and
Micro-XRCE-DDS-Agent entries there are `tricopter`-branch-only):

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
  .../egm96-5.pgm`), and `geographiclib-get-geoids` needs root. Fetch it
  into your home directory instead, once, machine-wide (not a repo file):
  ```bash
  curl -sSL -o /tmp/egm96-5.tar.bz2 \
    https://sourceforge.net/projects/geographiclib/files/geoids-distrib/egm96-5.tar.bz2/download
  mkdir -p $HOME/GeographicLib && cd $HOME && tar xjf /tmp/egm96-5.tar.bz2 && mv $HOME/geoids $HOME/GeographicLib/geoids
  ```

Note `mavproxy.py` is **not** assumed to be installed anywhere below --
all SITL parameter/telemetry checks in this project were done with
`pymavlink` directly, or standard `ros2`/MAVROS CLI tools.

### The ArduPilot model (`tricopter_ap`)

`src/aerocanyon/models/tricopter_ap/` shares its geometry/visual/sensor
blocks with the PX4 model (`src/aerocanyon/models/tricopter/`, untouched
by this branch) but is driven by ArduPilot's own `ArduPilotPlugin`
instead of PX4's `gz_bridge`. Two things are worth knowing if you read the
model file:

- `ArduPilotPlugin`'s motor `VELOCITY` control alone spins the rotors but
  generates **zero aerodynamic lift** -- Gazebo has no idea a spinning
  joint should push air. A `gz-sim-lift-drag-system` "thrust rig" (two
  `LiftDrag` blocks per rotor, opposite `cp` so their off-axis moments
  cancel) is layered on top, sized to match the PX4 model's own live-tuned
  per-rotor thrust.
- The vehicle's centre of gravity sits at the **rotor centroid**
  (`x=0.11666...`), not at the origin like the PX4 model. `AP_MotorsTri`'s
  mixer gives all three motors equal thrust at zero roll/pitch demand with
  no per-rotor moment-arm compensation, unlike PX4's allocator -- so with
  the front pair and rear rotor at different distances from an
  origin-CG, equal thrust alone leaves a standing pitch moment. This is a
  deliberate, required divergence from the PX4 model, not a bug.

### Fly the tricopter manually (ArduPilot)

Same idea as [Fly the tricopter manually](#fly-the-tricopter-manually)
above (start Gazebo + the autopilot by hand, fly it yourself), but for
ArduPilot. **`system_id:=255` on the MAVROS launch line below is not
optional** -- ArduPilot only accepts RC override (and other GCS commands)
from the system id in its `MAV_GCS_SYSID` parameter, which defaults to
**255**. MAVROS itself defaults to system id 1. Get this wrong and there
is **no error anywhere** -- MAVROS connects fine, arming works fine,
`/mavros/rc/override` publishes fine, and the sticks in the browser just
silently do nothing. This is the single easiest thing to get wrong here.

The world file used for hover testing is a **scratchpad copy** of
`urban_canyon.sdf` with the ArduPilot model included -- the repo's own
world file (used by the PX4 branch above) is never modified:

```bash
mkdir -p /tmp/aerocanyon_ap
cp $HOME/AeroCanyon_Guard/src/aerocanyon/worlds/urban_canyon.sdf \
   /tmp/aerocanyon_ap/urban_canyon_ap.sdf
sed -i 's#</world>#  <include>\n      <uri>model://tricopter_ap</uri>\n      <pose>-100 0 0.5 0 0 0</pose>\n    </include>\n  </world>#' \
   /tmp/aerocanyon_ap/urban_canyon_ap.sdf
```

```bash
# Terminal 1: Gazebo (headless -- no native GUI needed for the browser viewer below)
export GZ_SIM_RESOURCE_PATH=$HOME/AeroCanyon_Guard/src/aerocanyon/models:$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
source /opt/ros/jazzy/setup.bash
gz sim -v 2 -s -r /tmp/aerocanyon_ap/urban_canyon_ap.sdf &
sleep 12

# Terminal 2: ArduPilot SITL. NOTE: --defaults, NOT --add-param-file
# (that flag doesn't exist in this ArduPilot build -- arduplane --help
# lists --defaults). mavproxy.py is not installed/required.
mkdir -p /tmp/apstate && cd /tmp/apstate
$HOME/ardupilot/build/sitl/bin/arduplane --model JSON \
  --home 44.434424990487216,26.04781615647584,76,0 \
  --wipe --defaults $HOME/AeroCanyon_Guard/src/aerocanyon/ardupilot/tricopter.parm &
sleep 10

# Terminal 3: MAVROS. system_id:=255 is REQUIRED -- see above.
export GEOGRAPHICLIB_GEOID_PATH=$HOME/GeographicLib/geoids
source /opt/ros/jazzy/setup.bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=tcp://127.0.0.1:5760 -p system_id:=255 &
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

**Manual flight here is not the PX4 branch's world-frame velocity
offboard control.** Arming engages `QHOVER` and the sticks drive
`/mavros/rc/override` (an `OverrideRCIn` message, i.e. the same four RC
channels a real transmitter would send) -- roll/pitch are **body-frame
lean commands**, exactly like a real transmitter's sticks, not
yaw-rotated world-frame velocity setpoints. A centred/stale stick resolves
to PWM 1500, which in `QHOVER` means *hold attitude, hold altitude* -- a
real RC failsafe behaviour, and a difference worth knowing if you're
used to the PX4 branch's zero-velocity failsafe. `land now` switches to
`QLAND` and disarms automatically on touchdown. The `rc_bridge.py`
physical-transmitter path is unaffected by any of this (same
`/api/stick` HTTP surface). The `transition FW`/`transition MC` buttons,
however, are inert on this branch: `control_server.py` deliberately has
no handler for those commands (forward-flight is out of scope here --
see below), so clicking them fires a `fetch()` that gets an unread HTTP
error and silently does nothing, same as any other unrecognized
`/api/manual?cmd=...` value.

### What's not here yet (Phase 2)

Phase 1 (this branch, as documented above) covers **manual hover flight
only**. Two things are explicitly out of scope and not implemented:

- The autonomous CBF/PINN mission stack (`controller_node.py`,
  `run_trial.py`'s automated baseline/treatment trial) -- nothing on this
  branch flies a mission by itself.
- VTOL forward-flight transition -- `ArduPlane`'s `GUIDED` mode has no
  velocity-setpoint path for this airframe (see `control_server.py`'s
  module docstring), and the transition itself has not been tuned or
  verified at all here.

Both are tracked as Phase 2 in
[`docs/superpowers/specs/2026-09-02-tricopter-ardupilot-phase1-design.md`](docs/superpowers/specs/2026-09-02-tricopter-ardupilot-phase1-design.md).
