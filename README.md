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
