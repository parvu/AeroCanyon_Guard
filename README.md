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

2. **Fix the motor command topic.** The stock tiltrotor model's four
   `MulticopterMotorModel` plugins subscribe to `command/motor_speed`, but
   PX4's `gz_bridge` publishes on the model-namespaced
   `gazebo/command/motor_speed`. With the mismatch the vehicle spawns and
   even arms, but the motors never spin and it never leaves the ground:
   ```bash
   sed -i 's#<commandSubTopic>command/motor_speed</commandSubTopic>#<commandSubTopic>gazebo/command/motor_speed</commandSubTopic>#' \
     ~/PX4-Autopilot/Tools/simulation/gz/models/tiltrotor/model.sdf
   ```

3. **Place the canyon world:**
   ```bash
   cp ~/ros2_pinn_sim/src/aerocanyon/worlds/urban_canyon.sdf \
      ~/PX4-Autopilot/Tools/simulation/gz/worlds/urban_canyon.sdf
   ```

4. **Disable GCS requirement for offboard arming:**
   ```bash
   # In PX4 shell:
   param set NAV_DLL_ACT 0
   param save
   ```

(The airframe file `4020_gz_tiltrotor` already has SITL sensor checks disabled.)

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
PYTHONPATH=src/aerocanyon python -m aerocanyon.train_pinn trials/train*_baseline.csv --alpha 1.0
```

`docs/alpha_sweep.txt` records the alpha-vs-skill sweep used to pick the
fractional order; re-run it against real flight data before trusting the
checkpoint (see `task-8-report.md` — the shipped checkpoint was trained on
synthetic stand-in data because no live PX4 process was available in that
task's environment).

### Run the paired baseline/treatment trial

**Headless mode (recommended, works on WSL2):**

```bash
cd ~/PX4-Autopilot
source /opt/ros/jazzy/setup.bash
export R="build/px4_sitl_default/rootfs/"
export GZ_IP=127.0.0.1

# Terminal 1: Gazebo headless
gz sim -v 2 Tools/simulation/gz/worlds/urban_canyon.sdf -r -s &
sleep 5

# Terminal 2: XRCE-DDS bridge
~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888 &
sleep 2

# Terminal 3: PX4 SITL
export PX4_SIM_MODEL=gz_tiltrotor
export PX4_GZ_WORLD=urban_canyon
./build/px4_sitl_default/bin/px4 &
sleep 15

# Terminal 4: run the trial, then plot it
cd ~/ros2_pinn_sim
source /opt/ros/jazzy/setup.bash && source install/setup.bash
source .venv/bin/activate
python3 -m aerocanyon.run_trial --trial live_full --duration 180
python3 -m aerocanyon.plot_results --trial live_full
```

`plot_results` reads `trials/live_full_baseline.csv` and
`trials/live_full_treatment.csv` (the same `--trial` name used for the run)
and writes `figures/comparison.png` and `figures/cbf_intervention.png` —
see [View the figures](#view-the-figures) below for what's in them.

**GUI mode (requires X11 display server):**

On native Linux or with WSL2 X11 forwarding ([VcXsrv](https://sourceforge.net/projects/vcxsrv/), X410):

```bash
# Same as above, but replace Terminal 1 with:
export DISPLAY=:0  # or your WSL2 host IP
gz sim -v 2 Tools/simulation/gz/worlds/urban_canyon.sdf -r -s -g &
```

The `-g` flag opens a Gazebo GUI window; omit it for headless (saves ~100MB RAM).

**SITL-specific PX4 configuration:** see
[Known-good arming configuration](#known-good-arming-configuration) below —
it's the single source of truth for the airframe parameters; don't
duplicate them here, they've drifted out of sync with reality before.

**Output:**

`run_trial.py` spawns PX4 SITL and the Micro-XRCE-DDS-Agent, runs the
baseline mission, tears everything down, then repeats for the treatment
(FO-PINN + CBF) mission against the same wind seed — PX4's EKF and mission
state don't reset cleanly in place, so each trial gets a fresh PX4 process.

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

### Known-good arming configuration

Four mistakes will silently prevent the vehicle from arming or moving
(motors spin but the vehicle never lifts, arming is outright denied, or
the vehicle just sits there armed and idle):

1. **Missing `<spherical_coordinates>` in the world.** Without it, Gazebo's
   simulated magnetometer/GPS have no reference location, and PX4's EKF
   reports "Preflight Fail: no heading reference" / "Strong magnetic
   interference" forever. `worlds/_template.sdf` sets this (Zurich, same
   as PX4's stock `default.sdf`) — keep it if you edit the template.
2. **Disabling `SYS_HAS_MAG` / `SYS_HAS_BARO` in the airframe file.**
   Gazebo *does* simulate both sensors via the world's `Magnetometer` and
   `AirPressure` plugins, so telling PX4 the vehicle has neither starves
   the EKF of its yaw reference. The airframe
   (`ROMFS/px4fmu_common/init.d-posix/airframes/4020_gz_tiltrotor`) only
   needs:
   ```
   param set-default COM_ARM_WO_GPS 1
   param set-default SYS_HAS_NUM_ASPD 0
   param set-default CBRK_SUPPLY_CHK 894281
   param set-default COM_DISARM_LAND 0
   param set-default NAV_DLL_ACT 0
   ```
   (`SYS_HAS_NUM_ASPD=0` skips the airspeed-sensor check — there's no
   airspeed sensor on this airframe; `CBRK_SUPPLY_CHK` is PX4's documented
   circuit-breaker magic value for the SITL-only "system power unavailable"
   check.)

3. **Wrong motor command topic on the tiltrotor model.** Its four
   `MulticopterMotorModel` plugins must use
   `<commandSubTopic>command/motor_speed</commandSubTopic>` — the same
   value the stock `x500` model ships with. `gz-sim` scopes that relative
   topic under the model's own namespace, matching PX4's `gz_bridge`
   publisher (`/<model_name>/command/motor_speed`). Verify with
   `gz topic -i -t /tiltrotor_0/command/motor_speed` while the sim is
   running — it should show one publisher and four subscribers. Get this
   wrong and the vehicle spawns and even arms, but the motors never spin.
4. **Not an SDF/parameter issue at all:** the vehicle can arm cleanly via
   the PX4 shell yet the actual mission never moves anything. That was
   `controller_node` crashing on its very first control tick — see the
   commit fixing it (`2455ea8`) — because `TrajectorySetpoint.position`
   (a `float32[3]` PX4 field) round-trips through rosidl as a numpy array,
   and unpacking it into a `geometry_msgs/Vector3` (float64) field handed
   `numpy.float32` scalars to rclpy's serializer, which aborts the whole
   process with a C-level assertion. No Python traceback, no ROS log, and
   critically nothing in PX4's log either — the publisher was already
   dead, so PX4 never even received the arm/offboard request.
   `test_controller_node.py` now drives the real tick loop specifically to
   catch this class of bug.

If you edit airframe parameters (item 2) and arming still fails the same
way, the parameter store may have a stale saved value from a previous run
— `param set-default` only takes effect when nothing has been saved yet:
```bash
rm -f build/px4_sitl_default/rootfs/parameters.bson \
      build/px4_sitl_default/rootfs/parameters_backup.bson
rm -rf build/px4_sitl_default/rootfs/eeprom
```
