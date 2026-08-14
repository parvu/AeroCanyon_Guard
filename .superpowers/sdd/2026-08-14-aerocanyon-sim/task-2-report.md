# Task 2 Report: Canyon geometry and the generated world

## Status: DONE_WITH_CONCERNS

## What was done

1. `src/aerocanyon/test/test_canyon_geometry.py` — the 7 given tests, written first, confirmed failing (`ModuleNotFoundError`).
2. `src/aerocanyon/aerocanyon/canyon_geometry.py` — implemented exactly per the plan: `Box` namedtuple, `BUILDINGS` (6 boxes, two rows of 3 at y=+/-20, spaced 45m along x), `distance_and_normal()` (exact exterior box distance + outward unit normal, negative distance = inside), `to_sdf()`, `CANYON_ENTRY`/`CANYON_EXIT`, and a `__main__` block that reads `worlds/_template.sdf` and writes `worlds/urban_canyon.sdf`. All 7 tests pass.
3. `src/aerocanyon/worlds/_template.sdf` — copied from `src/phy_ai_simulation/worlds/dve_wind_arena.sdf`, then edited:
   - world name `dve_wind_arena` -> `urban_canyon`
   - added the four missing plugins (`gz-sim-sensors-system`, `gz-sim-air-pressure-system`, `gz-sim-magnetometer-system`, `gz-sim-navsat-system`) alongside the existing physics/user-commands/scene-broadcaster/imu/wind-effects plugins
   - `WindEffects` `default_wind` zeroed to `0.0 0.0 0.0`
   - `<!--BUILDINGS-->` marker placed immediately before `</world>`
4. Generated `src/aerocanyon/worlds/urban_canyon.sdf` via `python -m aerocanyon.canyon_geometry` — "wrote ... with 6 buildings". Verified headless load: `gz sim -s -r --iterations 200 src/aerocanyon/worlds/urban_canyon.sdf` exited 0 with no output (no SDF parse errors).
5. Patched `~/PX4-Autopilot/Tools/simulation/gz/models/tiltrotor/model.sdf`: confirmed 0 occurrences of `enable_wind` before editing, added `<enable_wind>true</enable_wind>` as the first child inside `<link name='base_link'>`. `grep -c enable_wind` now reports `1`.
6. Copied the generated world to `~/PX4-Autopilot/Tools/simulation/gz/worlds/urban_canyon.sdf` (byte-identical, confirmed with `diff`).
7. `src/aerocanyon/launch/canyon_sim.launch.py` written verbatim per the plan.
8. `colcon build --symlink-install --packages-select aerocanyon` succeeds (only a pre-existing, unrelated `pytest-repeat` setuptools warning on stderr).
9. Committed: `git add src/aerocanyon && git commit -m "feat(aerocanyon): canyon geometry, generated world and launch file"` (commit `f6a2c63`).

## Concern 1: PX4_SIM_MODEL value in the plan is wrong for this PX4 checkout

The plan's Step 8 command uses `PX4_SIM_MODEL=tiltrotor`. On this machine's `~/PX4-Autopilot` build, that fails immediately:

```
ERROR [init] Unknown model tiltrotor (not found by name on .../airframes)
```

PX4's `rcS` resolves the model by matching the full airframe filename suffix (`etc/init.d-posix/airframes/4020_gz_tiltrotor`), so the correct value is `PX4_SIM_MODEL=gz_tiltrotor`, not `tiltrotor`. With that correction, PX4 SITL boots cleanly against `urban_canyon.sdf`:

```
INFO  [init] found model autostart file as SYS_AUTOSTART=4020
INFO  [init] Gazebo simulator 8.11.0
INFO  [init] Starting gazebo with world: .../worlds/urban_canyon.sdf
INFO  [init] Gazebo world is ready
INFO  [init] Spawning Gazebo model
INFO  [tone_alarm] home set
```

No SDF parse errors, no fatal errors — only pre-existing, unrelated `gz_frame_id` warnings (present in the tiltrotor/airspeed model files already, not something this task touched) and the usual SITL preflight warnings (airspeed selector, system power) that are normal for a headless SITL run and don't block arming. The process was left running its full 45s test window and exited via `timeout`, not a crash.

## Concern 2: Step 8 (visual takeoff confirmation) could not be completed

I have no GUI/display access and no interactive stdin into the PX4 `pxh>` shell from this environment (background shell commands run with `< /dev/null`), so I could not type `param set NAV_DLL_ACT 0`, `param save`, or `commander takeoff` into the running instance, nor visually confirm the Gazebo GUI shows six grey towers with the tiltrotor lifting off between them. This is exactly the gate the plan calls out as "YOUR eyes, not automated."

What I *did* verify as a proxy: PX4 SITL boots against `urban_canyon.sdf` with `PX4_SIM_MODEL=gz_tiltrotor`, spawns the tiltrotor model without SDF/plugin errors, initializes IMU/GPS/mag/air-pressure sensors, and reaches "home set" — i.e., everything up to the point a human would type `commander takeoff` works. The actual takeoff/flight-through-the-canyon needs a human (or a follow-up task with MAVSDK/pymavlink scripting) to complete.

## Concern 3: the template retains the unrelated `pinn_drone` model

Per the plan's literal Step 5 instructions, the template is `dve_wind_arena.sdf` edited only for world name / plugin list / wind default / marker — nothing in the plan says to strip the embedded `pinn_drone` custom quadrotor model (with its own LiftDrag/MulticopterMotorModel plugins) that's baked into that source file for the unrelated `phy_ai_simulation` package. I left it in place since the plan didn't ask for its removal and it didn't break the load test or PX4 boot (it's a separately-named `<model>` that PX4 doesn't reference). It does mean `urban_canyon.sdf` spins up a second, unused rotorcraft's physics/plugins alongside the six buildings and the tiltrotor PX4 spawns. Flagging in case a later task wants it stripped for a cleaner/faster world.

## Files changed

- `/home/parvu/ros2_pinn_sim/src/aerocanyon/aerocanyon/canyon_geometry.py` (new)
- `/home/parvu/ros2_pinn_sim/src/aerocanyon/test/test_canyon_geometry.py` (new)
- `/home/parvu/ros2_pinn_sim/src/aerocanyon/worlds/_template.sdf` (new)
- `/home/parvu/ros2_pinn_sim/src/aerocanyon/worlds/urban_canyon.sdf` (new, generated)
- `/home/parvu/ros2_pinn_sim/src/aerocanyon/launch/canyon_sim.launch.py` (new)
- `/home/parvu/PX4-Autopilot/Tools/simulation/gz/models/tiltrotor/model.sdf` (modified, +1 line `enable_wind`)
- `/home/parvu/PX4-Autopilot/Tools/simulation/gz/worlds/urban_canyon.sdf` (new copy)

Commit: `f6a2c63` "feat(aerocanyon): canyon geometry, generated world and launch file"
