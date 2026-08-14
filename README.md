# ros2_pinn_sim

PX4 SITL (x500 quadrotor) in Gazebo Harmonic, with a wind disturbance world
and a PINN telemetry bridge, controllable either by a keyboard teleop node
over ROS2 or by QGroundControl over MAVLink.

## Packages

- **phy_ai_simulation** — the `dve_wind_arena` Gazebo world (WindEffects
  plugin, same config as the original custom-drone world) and
  `physics_bridge`, which reads the vehicle's IMU and publishes
  `/pinn/input_state`. Also contains the original custom-drone plugins/world,
  kept for reference but superseded by PX4 for actual flight control.
- **px4_teleop** — keyboard teleop for PX4 offboard velocity control
  (arms, engages offboard mode, streams `TrajectorySetpoint` over the
  uXRCE-DDS bridge).
- **px4_msgs** — PX4's ROS2 message definitions (git submodule, pinned to
  the commit this workspace was built against).

## Prerequisites

Cloned/built separately, outside this workspace (not part of this repo):

- ROS2 Jazzy + Gazebo Harmonic (`ros-jazzy-ros-gz-*`, `ros-jazzy-rosidl-*`)
- [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) cloned at `~/PX4-Autopilot`,
  built for SITL: `make px4_sitl gz_x500`
- [Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)
  built at `~/Micro-XRCE-DDS-Agent` (only needed for the ROS2/teleop path,
  not for QGroundControl)
- The wind world requires two local edits inside `PX4-Autopilot`:
  - `Tools/simulation/gz/worlds/dve_wind_arena.sdf` — copy of `default.sdf`
    with `Physics`/`UserCommands`/`SceneBroadcaster`/`Sensors`/`Imu`/
    `AirPressure`/`Magnetometer`/`NavSat`/`WindEffects` plugins declared
    explicitly (adding any plugin to a PX4 world disables its implicit
    default plugin set, so all of them need to be listed).
  - `Tools/simulation/gz/models/x500_base/model.sdf` — add
    `<enable_wind>true</enable_wind>` to `base_link` (WindEffects only
    applies force to links that opt in).
- PX4 must have `NAV_DLL_ACT` set to `0` (via `param set NAV_DLL_ACT 0` +
  `param save` in the PX4 shell) if you plan to arm via the ROS2 teleop
  without a GCS connected — otherwise arming is rejected with "No connection
  to the GCS". Not needed for the QGroundControl path, since QGC itself
  counts as the GCS connection.

Build this workspace:

```bash
cd ~/ros2_pinn_sim
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## Launch: simulation + keyboard teleop

```bash
# Terminal 1 -- PX4 + Gazebo (wind-enabled world)
cd ~/PX4-Autopilot
source /opt/ros/jazzy/setup.bash
PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=dve_wind_arena GZ_IP=127.0.0.1 ./build/px4_sitl_default/bin/px4

# Terminal 2 -- ROS2 <-> PX4 bridge
~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888

# Terminal 3 (optional) -- PINN telemetry (/pinn/input_state)
cd ~/ros2_pinn_sim
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch phy_ai_simulation pinn_telemetry_launch.py

# Terminal 4 -- fly it
cd ~/ros2_pinn_sim
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run px4_teleop teleop_keyboard
```

Controls: `w/s` north/south, `a/d` west/east, `i/k` up/down, `j/l` yaw,
space to hover, `+/-` speed, `q` to quit (disarms). The node arms and
switches to offboard mode automatically on startup.

## Launch: simulation + QGroundControl

```bash
# Terminal 1 -- PX4 + Gazebo (wind-enabled world)
cd ~/PX4-Autopilot
source /opt/ros/jazzy/setup.bash
PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=dve_wind_arena GZ_IP=127.0.0.1 ./build/px4_sitl_default/bin/px4
```

Then launch QGroundControl (download the AppImage from the official
[QGroundControl](https://qgroundcontrol.com/) site if not already
installed):

```bash
chmod +x QGroundControl.AppImage   # first time only
./QGroundControl.AppImage
```

PX4 SITL broadcasts MAVLink on UDP port 14550 by default, so QGC
auto-detects and connects to the vehicle within a few seconds — no manual
connection setup needed. Fly it from QGC's own controls (arm, takeoff,
manual/position/offboard modes) as you would with any PX4 vehicle.

The PINN telemetry launch (Terminal 3 above) can be run alongside QGC too,
if you want `/pinn/input_state` while flying from QGC — the DDS agent is
only needed for that, not for QGC itself.

## AeroCanyon-Guard

`src/aerocanyon` is a separate scenario from the wind-arena package above: a
tilt-rotor VTOL transiting a canyon of six box buildings under a spatially
varying urban wind field, with a fractional-order PINN estimating the
disturbance and a CBF safety filter keeping the vehicle off the buildings
and away from stall.

### PX4 patches required

Same pattern as the wind-arena setup, applied to the tilt-rotor airframe
and the generated canyon world instead:

- `Tools/simulation/gz/models/tiltrotor/model.sdf` — add
  `<enable_wind>true</enable_wind>` as the first child of `<link
  name='base_link'>` (WindEffects only applies force to links that opt in).
- `Tools/simulation/gz/worlds/urban_canyon.sdf` — a copy of this repo's
  generated world (see below), placed inside `PX4-Autopilot` so PX4 can
  find it by name.
- `NAV_DLL_ACT` set to `0` (`param set NAV_DLL_ACT 0` + `param save` in the
  PX4 shell), same reason as the wind-arena setup: offboard control needs
  to arm without a GCS connected.
- **Airframe env var correction:** the airframe file is
  `4020_gz_tiltrotor`, and PX4 matches `PX4_SIM_MODEL` against that
  filename suffix — so use `PX4_SIM_MODEL=gz_tiltrotor`, not `tiltrotor`.

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
PYTHONPATH=src/aerocanyon python -m aerocanyon.train_pinn trials/train*_baseline.csv --alpha 1.0
```

`docs/alpha_sweep.txt` records the alpha-vs-skill sweep used to pick the
fractional order; re-run it against real flight data before trusting the
checkpoint (see `task-8-report.md` — the shipped checkpoint was trained on
synthetic stand-in data because no live PX4 process was available in that
task's environment).

### Run the paired baseline/treatment trial

```bash
cd ~/ros2_pinn_sim
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon build --symlink-install --packages-select aerocanyon
python3 -m aerocanyon.run_trial --trial compare --duration 60
source .venv/bin/activate
PYTHONPATH=src/aerocanyon python -m aerocanyon.plot_results --trial compare
```

`run_trial.py` spawns PX4 SITL and the Micro-XRCE-DDS-Agent, runs the
baseline mission, tears everything down, then repeats for the treatment
(FO-PINN + CBF) mission against the same wind seed — PX4's EKF and mission
state don't reset cleanly in place, so each trial gets a fresh PX4 process.
Add `--gui` to watch it fly (used for the demo recording).

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
