# ros2_pinn_sim

ROS2 / Gazebo Harmonic simulation environment for **AeroCanyon-Guard**: a
tilt-rotor VTOL transiting an urban canyon under spatially-varying wind
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
  built for SITL: `make px4_sitl gz_tiltrotor`
- **[Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)**
  built at `~/Micro-XRCE-DDS-Agent` (bridges ROS2 ↔ PX4)

### PX4 patches for tilt-rotor + wind

Apply these edits to `PX4-Autopilot`:

1. **Enable wind effects on the vehicle:**
   ```bash
   # In Tools/simulation/gz/models/tiltrotor/model.sdf, add to base_link:
   <enable_wind>true</enable_wind>
   ```

2. **Place the canyon world:**
   ```bash
   cp ~/ros2_pinn_sim/src/aerocanyon/worlds/urban_canyon.sdf \
      ~/PX4-Autopilot/Tools/simulation/gz/worlds/urban_canyon.sdf
   ```

3. **Disable GCS requirement for offboard arming:**
   ```bash
   # In PX4 shell:
   param set NAV_DLL_ACT 0
   param save
   ```

4. **White `base_link` visual (cosmetic, optional):** in
   `Tools/simulation/gz/models/tiltrotor/model.sdf`, `base_link_visual`'s
   `<material>` is changed from the stock dark grey to white
   (`<ambient>`/`<diffuse>` both `1.0 1.0 1.0 1.0`) — purely for
   visibility against the canyon buildings in the demo video/GUI, no
   effect on simulation behaviour.

The airframe file `4020_gz_tiltrotor` already has SITL sensor checks
disabled, and the stock tiltrotor model's motor topic already matches
what PX4's `gz_bridge` publishes -- neither needs editing. See
[History.md](History.md#known-good-arming-and-telemetry-configuration)
for the pitfalls that look like they need a patch here but don't (and the
ones that actually do, elsewhere in the stack).

## Build this workspace

```bash
cd ~/ros2_pinn_sim
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## AeroCanyon-Guard: Tilt-rotor Canyon Transit

Autonomous mission: tilt-rotor VTOL transits an urban canyon (6 box buildings)
under spatially-varying wind disturbance. FO-PINN estimates wind forces,
CBF safety filter prevents collisions and stalls.


### Regenerate the world and the wind grid

```bash
cd ~/ros2_pinn_sim
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

**Return-to-start behaviour:** each leg must genuinely fly back to the
spawn point and land — not just clear the canyon or hit a wall-clock
timeout — because the next leg's PX4 process reuses that same spawn
pose, and a vehicle left drifting or crashed there is a likely cause of
the *next* leg failing to arm/fly. Baseline hands off to PX4's own
`VEHICLE_CMD_NAV_RETURN_TO_LAUNCH` on clearing the canyon (it has no
wind feedforward to fly itself home against the canyon's own crosswind);
treatment flies itself back and lands under its own offboard control,
using the wind estimate the whole way home. **Known issue:** PX4's
native RTL has been verified live to engage `AUTO_RTL` but not actually
navigate back toward home in this SITL configuration when triggered
from `controller_node`'s own offboard flow (it has, however, been seen
to work correctly when triggered manually from QGroundControl on a
vehicle that took off normally -- see History.md for that lead), so
baseline currently still times out on `--duration` most runs. The VTOL
fixed-wing transition stays disabled (`controller_node.
ENABLE_VTOL_TRANSITION = False`) for both legs — the whole flight,
including the return, flies in stable multicopter mode; see History.md
for why.

```bash
cd ~/ros2_pinn_sim
source /opt/ros/jazzy/setup.bash && source install/setup.bash
source .venv/bin/activate
python3 -m aerocanyon.run_trial --trial live_full  # --duration defaults to 120s, see --help
python3 -m aerocanyon.plot_results --trial live_full
```

Add `--gui` to watch each leg's Gazebo window as it runs (requires an
X11 display server -- on WSL2, [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
or X410, and `export DISPLAY=:0` or your WSL2 host IP first); omit it
for headless (recommended day-to-day, saves ~100MB RAM per leg and
doesn't need a display).

`plot_results` reads `trials/live_full_baseline.csv` and
`trials/live_full_treatment.csv` (the same `--trial` name used for the run)
and writes `figures/comparison.png` and `figures/cbf_intervention.png` —
see [View the figures](#view-the-figures) below for what's in them.

**SITL-specific PX4 configuration:** see
[History.md](History.md#known-good-arming-and-telemetry-configuration) —
it's the single source of truth for the airframe parameters; don't
duplicate them here, they've drifted out of sync with reality before.

**Watching a trial fly, or flying manually (no `run_trial.py`):** if you
want to fly the tiltrotor by hand instead — e.g. to sanity-check a fresh
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
# gz-sim can't resolve model://tiltrotor and the vehicle silently never
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
export PX4_SIM_MODEL=gz_tiltrotor
export PX4_GZ_WORLD=urban_canyon
export PX4_GZ_MODEL_POSE="-90,0,0.246,0,0,0"  # ground, at the canyon entry -- see run_trial.py's SPAWN_POSE
./build/px4_sitl_default/bin/px4 &
sleep 15
```

Then either fly manually from QGroundControl (it auto-connects to PX4
over MAVLink UDP), or launch just the mission nodes without
`run_trial.py`'s process orchestration:
`ros2 launch aerocanyon canyon_sim.launch.py mode:=baseline trial:=manual`.

Do not run `run_trial.py` on top of this -- see the warning above.

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
