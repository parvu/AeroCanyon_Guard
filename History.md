# History

Known pitfalls hit and fixed while building this project, kept here rather
than in the README since they're debugging history, not day-to-day usage
instructions.

### Known-good arming and telemetry configuration

Six mistakes will silently prevent the vehicle from arming, moving, or
producing usable telemetry (motors spin but the vehicle never lifts,
arming is outright denied, the vehicle sits there armed and idle, it
flies just fine while every logged position stays at zero, or nothing
ever spawns at all):

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
5. **Unversioned `/fmu/out/vehicle_local_position` and `/fmu/out/vehicle_status`
   topic names don't exist on this PX4 build** — only the versioned
   `vehicle_local_position_v1` and `vehicle_status_v4` are actually
   published (`ros2 topic list | grep /fmu/out` while the sim is running
   to check on any other PX4 checkout). Subscribing to the unversioned
   name doesn't error — `ros2 topic echo` just warns "does not appear to
   be published yet" and the callback is never invoked, so the field stays
   at its zero-initialized default forever. This silently zeroed out
   position in `controller_node`, `trial_logger`, `wind_field_node`, and
   `fo_pinn_node` at once: the mission still flew (it doesn't feed position
   back into anything), but every logged trajectory was flat, `plot_results`
   computed RMS deviation from all-zero position, and the PINN's physics
   residual saw no real acceleration signal. Same fix everywhere: point
   the subscription at the versioned topic name; `px4_msgs` uses the same
   message class for both, so nothing else changes.
6. **Gazebo launched without `GZ_SIM_RESOURCE_PATH` set** (i.e. without
   sourcing `build/px4_sitl_default/rootfs/gz_env.sh` first). PX4's own spawn
   request asks for a fixed entity name (`allow_renaming: false`); if
   gz-sim can't resolve `model://tiltrotor` it logs `[Err]
   [UserCommands.cc:928] ... Unable to find uri[file:///tiltrotor/model.sdf]`
   to its own stderr and the create call just fails — no vehicle ever
   appears, `gz model --list` shows only the world's static geometry, and
   nothing else in the stack (PX4, the trial scripts) surfaces an error,
   because from their side the request was sent and nothing crashed. This
   is easy to miss because it's silent from every angle except the Gazebo
   server's own log. `run_trial.py` never launches Gazebo itself (see the
   top of that file) — sourcing `gz_env.sh` before your own `gz sim`
   command is on you.

If you edit airframe parameters (item 2) and arming still fails the same
way, the parameter store may have a stale saved value from a previous run
— `param set-default` only takes effect when nothing has been saved yet:
```bash
rm -f build/px4_sitl_default/rootfs/parameters.bson \
      build/px4_sitl_default/rootfs/parameters_backup.bson
rm -rf build/px4_sitl_default/rootfs/eeprom
```
