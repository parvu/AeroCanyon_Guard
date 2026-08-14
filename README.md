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

# Terminal 4: Trials (generates baseline + treatment CSVs and figures)
cd ~/ros2_pinn_sim
source /opt/ros/jazzy/setup.bash && source install/setup.bash
source .venv/bin/activate
python3 -m aerocanyon.run_trial --trial live_full --duration 180
```

**GUI mode (requires X11 display server):**

On native Linux or with WSL2 X11 forwarding ([VcXsrv](https://sourceforge.net/projects/vcxsrv/), X410):

```bash
# Same as above, but replace Terminal 1 with:
export DISPLAY=:0  # or your WSL2 host IP
gz sim -v 2 Tools/simulation/gz/worlds/urban_canyon.sdf -r -s -g &
```

The `-g` flag opens a Gazebo GUI window; omit it for headless (saves ~100MB RAM).

**SITL-specific PX4 configuration:**

The airframe file (`build/px4_sitl_default/rootfs/etc/init.d-posix/airframes/4020_gz_tiltrotor`)
has been configured to disable sensor checks inappropriate for simulation:

- `COM_ARM_WO_GPS=1` — arm without GPS lock
- `SYS_HAS_MAG=0`, `SYS_HAS_BARO=0` — disable mag/baro requirements
- `COM_DISARM_LAND=0`, `SYS_FAILURE_EN=0` — disable failsafe systems for clean SITL

These allow the vehicle to arm and fly automatically during trials without
preflight check delays.

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
