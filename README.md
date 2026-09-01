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
- **[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)** cloned at `~/PX4-Autopilot`,
  built for SITL: `make px4_sitl gz_tricopter` (after installing the
  airframe below — it does not exist in stock PX4)
- **[Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)**
  built at `~/Micro-XRCE-DDS-Agent` (bridges ROS2 ↔ PX4)

### PX4 setup for the tricopter + wind

The vehicle is a **tricopter VTOL** in the style of the E-flite
Convergence: three tilting rotors -- two front (which also give yaw in
hover, via differential tilt) and one rear, an active pusher in cruise
sized to half the vehicle's weight rather than a fixed rotor that stops
in forward flight. PX4 has a real-hardware airframe for the Convergence
(ID 13012) but has **never shipped a simulation model** for it, so this
project carries its own — both files live in this repo and get copied
into `PX4-Autopilot`:

1. **Install the tricopter model and airframe:**
   ```bash
   cp -r ~/AeroCanyon_Guard/src/aerocanyon/models/tricopter \
         ~/PX4-Autopilot/Tools/simulation/gz/models/
   cp ~/AeroCanyon_Guard/src/aerocanyon/airframes/4022_gz_tricopter \
      ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
   # then register it and rebuild
   #   add `4022_gz_tricopter` to that directory's CMakeLists.txt
   #   make px4_sitl gz_tricopter
   ```
   The model is derived from PX4's stock quad `tiltrotor`: one rear rotor
   removed, the other moved to the centreline, and the tilt joints
   widened to `-15..90°` so the allocator can trim the (now inherently
   unbalanced) single rear rotor's torque in both yaw directions.

2. **Place the canyon world:**
   ```bash
   cp ~/AeroCanyon_Guard/src/aerocanyon/worlds/urban_canyon.sdf \
      ~/PX4-Autopilot/Tools/simulation/gz/worlds/urban_canyon.sdf
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

## Docker

`Dockerfile` builds the entire stack above in one image: ROS2 Jazzy,
Gazebo Harmonic, PX4-Autopilot SITL (pinned to the commit the vendored
`px4_msgs` submodule was generated against, with this project's tricopter
airframe registered into it), and Micro-XRCE-DDS-Agent. See the comment
block at the top of the Dockerfile for build/run invocations, including
the X11 flags GUI trials need. It's an alternative to the manual
Prerequisites setup above, not a replacement for anything below this
section — everything from "Build this workspace" onward still applies,
just run inside the container.

## Build this workspace

```bash
cd ~/AeroCanyon_Guard
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## AeroCanyon-Guard: Tricopter Canyon Transit

Autonomous mission: tricopter VTOL transits an urban canyon (6 box buildings)
under spatially-varying wind disturbance. FO-PINN estimates wind forces,
CBF safety filter prevents collisions and stalls.


### Regenerate the world and the wind grid

```bash
cd ~/AeroCanyon_Guard
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 -m aerocanyon.canyon_geometry        # writes worlds/urban_canyon.sdf
python3 -m aerocanyon.canyon_field           # writes data/wind_grid.npy
cp src/aerocanyon/worlds/urban_canyon.sdf \
   ~/PX4-Autopilot/Tools/simulation/gz/worlds/urban_canyon.sdf
```

### Train the FO-PINN wind estimator

Collect baseline trials first (see below), then:

```bash
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
flow below, which still uses an external Gazebo).

Do not start PX4 or the DDS agent by hand alongside `run_trial.py`: its
own spawn of either will silently lose the port/instance-0-lock race
against a manually-started one and its own PX4 process dies immediately
(it raises with a clear message if that happens, rather than failing
silently).

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
cd ~/AeroCanyon_Guard
source /opt/ros/jazzy/setup.bash && source install/setup.bash
source .venv/bin/activate
python3 -m aerocanyon.run_trial --trial live_full  # --duration defaults to 220s, see --help
python3 -m aerocanyon.plot_results --trial live_full
```

Each leg's Gazebo GUI is visible by default while it runs (needs a
working X11 display server -- on WSL2, [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
or X410, and `export DISPLAY=:0` or your WSL2 host IP first) — useful
for actually watching a trial fly, or catching the intermittent
spawn-time attitude flip in History.md happening live.

`plot_results` reads `trials/live_full_baseline.csv` and
`trials/live_full_treatment.csv` (the same `--trial` name used for the run)
and writes `figures/comparison.png` and `figures/cbf_intervention.png` —
see [View the figures](#view-the-figures) below for what's in them.

**SITL-specific PX4 configuration:** see
[History.md](History.md#known-good-arming-and-telemetry-configuration) —
it's the single source of truth for the airframe parameters; don't
duplicate them here, they've drifted out of sync with reality before.

**Watching a trial fly, or flying manually (no `run_trial.py`):** if you
want to fly the tricopter by hand instead — e.g. to sanity-check a fresh
PX4/model checkout, watch it in the GUI without the trial harness's own
PX4/agent lifecycle management, or fly it yourself from QGroundControl —
Gazebo and PX4 need to be started externally and kept running, exactly
the opposite of the split-process, fresh-per-leg default above:

```bash
cd ~/PX4-Autopilot
source /opt/ros/jazzy/setup.bash
export GZ_IP=127.0.0.1

# Terminal 1: Gazebo
# MUST source gz_env.sh first (sets GZ_SIM_RESOURCE_PATH) -- without it
# gz-sim can't resolve model://tricopter and the vehicle silently never
# spawns (nothing shows up in `gz model --list`, no error visible unless
# you check the server's own stderr for "Unable to find uri[...]").
source build/px4_sitl_default/rootfs/gz_env.sh
gz sim -v 2 Tools/simulation/gz/worlds/urban_canyon.sdf -r -s -g &  # drop -g for headless
sleep 5

# Terminal 2: XRCE-DDS bridge
~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888 &
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

Do not run `run_trial.py` on top of this -- see the warning above.

#### Fly from a browser instead

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
mkdir -p web_viewer/assets
ln -s ~/PX4-Autopilot/Tools/simulation/gz/models/standard_vtol web_viewer/assets/standard_vtol
```

```bash
# tab 1: browser viewer (scene stream) -- needs the gz sim from above running
gz launch web_viewer/websocket.gzlaunch

# tab 2: control server (static files + manual offboard velocity setpoints)
cd web_viewer && source /opt/ros/jazzy/setup.bash && source ../install/setup.bash
python3 control_server.py 8080
```

Then open `http://localhost:8080` (or the host's LAN IP, from another
device). With no physical RC transmitter connected (see below), two
Mode 2 RC-style proportional sticks (left: yaw + throttle, right: roll +
pitch) stream continuously while touched and self-center on release;
`arm` requests offboard mode + arm, `land now` hands off to PX4's
`AUTO_LAND`.

#### Fly with a physical RC transmitter instead

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
# In WSL2, once per attach -- the input device node is root-only by default:
sudo modprobe hid hid-generic usbhid joydev   # if not already loaded
sudo chmod 666 /dev/input/js0
```

Then, alongside `control_server.py` from above:
```bash
python3 rc_bridge.py 8080
```

A different transmitter (even a different unit of the same dongle model)
may use different axis numbers for roll/pitch/throttle/yaw -- confirm
with `python3 -c "import struct; ..."` reading `/dev/input/js0`'s
`struct js_event` stream while moving one stick at a time, the same way
`AXIS_ROLL`/`AXIS_PITCH`/`AXIS_THROTTLE`/`AXIS_YAW` were confirmed here.

### View the figures

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
