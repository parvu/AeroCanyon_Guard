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

### Return-to-start: why baseline and treatment fly home differently

The original design had `controller_node` request PX4's native
`VEHICLE_CMD_NAV_RETURN_TO_LAUNCH` once it measured clearing the canyon
exit. Verified live, twice: `nav_state` correctly engages `AUTO_RTL`
(`vtol_vehicle_status` stays MC throughout, ruling out the transition
instability below), but the vehicle never actually turns back — local
east kept increasing in the same direction the whole flight, out to
~1900 m over a 360 s window, right through PX4's own 60 m
`RTL_RETURN_ALT` climb. Root cause not confirmed (`COM_ARM_WO_GPS=1` may
be skipping normal home-position initialisation), and still unresolved.

The replacement — flying itself home under the same proven offboard
position control used for the outbound transit — surfaced two more real
bugs before it worked at all:

- **`TrajectorySetpoint.velocity`/`.acceleration` must be `NaN`, not
  left at their zero default.** The message's own doc comment says
  "setting a value to NaN means the state should not be controlled" —
  `sp.velocity`/`sp.acceleration` were never set, which ROS2 defaults to
  `[0.0, 0.0, 0.0]`, not `NaN`. PX4 read that as an explicit
  hold-zero-velocity/zero-acceleration command layered on top of the
  position setpoint, fighting the position controller's own authority
  the entire time — verified live: cruise velocity never got anywhere
  near the mission's 12 m/s (capped around 2-6 m/s), and a large reverse
  position setpoint produced no turnaround at all. Explicitly marking
  both `NaN` (`controller_node.py`, in `_tick`'s `TrajectorySetpoint`
  construction) is what hands full authority back to the position
  controller.
- **The canyon crosswind (~12-14 m/s) is close to the mission's own
  12 m/s cruise speed.** Baseline intentionally carries no wind
  feedforward — that's the comparison being measured — and even after
  the NaN fix, baseline's un-compensated position control still could
  not make headway flying home against a headwind that strong; it kept
  drifting further away instead of turning back. That's what splits the
  two modes: baseline hands off to native RTL (no better, but no worse,
  and doesn't need wind compensation to attempt it), while treatment —
  which already estimates wind for the CBF/PINN feedforward — reuses
  that estimate during the return leg too, since getting home safely
  isn't part of the baseline/treatment comparison being scienced.

**New lead, not yet chased down:** flying manually from QGroundControl
(arm, climb, cruise, `ORBIT`, then trigger `AUTO_RTL` from QGC itself) --
RTL worked correctly. `nav_state` went `AUTO_RTL` -> `AUTO_LAND`, the
vehicle flew back toward the spawn point and landed cleanly and slowly
(~105 s descent, roll pinned near 0° the whole way, a real
airborne-to-landed transition confirmed by `vehicle_land_detected`).
That's the opposite of the `controller_node`-triggered failure above.
The concrete difference between the two: `controller_node` skips PX4's
own takeoff/mission sequencing and jumps straight into OFFBOARD mode,
where QGC's flight went through PX4's normal arm/manual-climb/mission
flow first. If PX4's home-position latching depends on passing through
one of those normal modes at least once, that would explain both
results at once -- worth testing directly (e.g. briefly holding
`AUTO_TAKEOFF` before switching to OFFBOARD) before writing off native
RTL as fundamentally broken in this SITL configuration.

### Land-in-place: why the return-to-start design above was replaced

Everything in the section above -- native RTL, the custom fly-home state
machine, the NaN fix, the wind-authority split between modes -- existed
to solve one problem: a vehicle left drifting or crashed when the next
leg's PX4 process booted, back when Gazebo and the vehicle entity stayed
alive across both legs. Once each leg got its own fresh Gazebo/PX4
process with nothing shared between legs at all (see the split-process
section below), that problem stopped existing: wherever a leg's vehicle
ends up when its own Gazebo process gets killed is irrelevant to the
next leg's boot. So `controller_node` now just requests
`VEHICLE_CMD_NAV_LAND` in place once it clears the canyon, for both
modes uniformly -- no flying anywhere first. Verified live: this lands
in ~90s total (hold + transit + descent) versus 150-220s+ for the old
fly-home designs, and does so reliably rather than depending on native
RTL actually working. `LAND_CLEARANCE_M`/`VEHICLE_CMD_NAV_LAND` replaced
`RETURN_CLEARANCE_M`/`VEHICLE_CMD_NAV_RETURN_TO_LAUNCH` and the whole
`returning`/`disarmed`/`rtl_handoff` state machine in `controller_node.py`.

### Split-process trial runner: why each leg gets its own Gazebo

`main()` no longer runs both legs' `run_one()` calls in the same Python
process against one long-lived, externally-started Gazebo. Each leg
(`run_leg`, invoked as `run_trial --mode <mode>`) now spawns a
brand-new `gz sim` server, runs its leg against it, and tears both
Gazebo and PX4 back down before the next leg's *separate OS process*
even starts. Motivated by the spawn-time flip above: a genuinely fresh
Gazebo process shares nothing with a previous leg -- no entity, no
physics engine state, no Python/rclpy state either, unlike the
in-place entity reset this replaced as the default (`_reset_gazebo_model`,
still used by the manual/external-Gazebo flow in the README). Whether
this actually reduces the flip rate hasn't been verified live over
enough runs yet to say -- it's a reasonable additional isolation
guarantee regardless, since a leg's Gazebo/PX4 can no longer affect the
next leg's boot at the OS-process level, not just at the entity level.

One thing this surfaced immediately: `gz sim ... -s` (server-only,
headless) occasionally left behind an already-orphaned companion process
(`gz sim -g`, no world argument, reparented to init) that escaped the
normal process-group kill on the main server -- observed live, twice.
`run_leg`'s teardown sweeps for that exact command line (`pkill -xf 'gz
sim -g'`) so a stray Gazebo process can never survive into the next
leg's supposedly-fresh one. `_spawn_gazebo` no longer passes `-s` at all
(every trial's GUI is visible by default now, see below) or `-g` (tried
using it for an explicit headless/GUI toggle; verified live that it
makes gz-sim detach a *different* orphaned child that escapes the
process-group kill just the same, so it was dropped rather than papered
over with another exact-match pkill) -- the plain combined server+gui
process `gz sim -v 2 <world> -r` launches has, so far, torn down cleanly
every time.

### Intermittent spawn-time attitude flip (unresolved)

Live testing occasionally (roughly 1 run in 3-4) shows `vehicle_attitude`
reporting the tiltrotor flipped ~180° in roll immediately on spawn or
teleport-reset — as early as t≈0.1s, with the vehicle still essentially
at rest (near-zero altitude change, near-zero velocity at the instant of
the flip). Once flipped, the vehicle stays flipped for the rest of that
recording; it never self-corrects.

What this **isn't**, ruled out by inspecting the actual telemetry rather
than guessing:
- **Not this project's axis-conversion code.** `frames.py`'s NED/ENU
  swap and `cbf_filter.py`'s `quat_to_rotmat`/`body_z_in_ned` were
  audited and are self-consistent (verified: a global quaternion sign
  flip q → -q cancels out of the roll/pitch/yaw formula used to inspect
  this, so it isn't a sign-convention artifact in the inspection either).
  `vehicle_attitude` comes straight from PX4's own EKF; this project
  only reads it.
- **Not arm-time motor torque.** The flip has been observed at t≈0.1s,
  before `controller_node` has sent any arm command (PX4 has already
  been running for 10+ seconds by the time it does) and before PX4's own
  `COM_SPOOLUP_TIME` window would even apply. Doubling `COM_SPOOLUP_TIME`
  (1.0s → 3.0s) live, as a targeted test, made no difference.
- **Not specific to teleporting the same long-lived entity.** Tried
  recreating the entity outright (`run_trial._recreate_gazebo_model()`,
  wired in as `run_one(..., clean_respawn=True)`) on the theory that
  physics state carried over from repeated `set_pose` teleports was the
  cause. Confirmed live that this instead reproduces, deterministically,
  the *other* known failure mode: every `/fmu/out` telemetry topic frozen
  at exactly zero for the whole flight (see the module docstring in
  `run_trial.py` for why recreating the entity is unreliable in the
  first place). `_recreate_gazebo_model()` is kept in the code, unused,
  specifically so this dead end doesn't get re-tried blind.
- `_reset_gazebo_model()` now resets orientation (identity quaternion),
  not just position, on every teleport — this can't prevent the initial
  flip, but it stops one flipped leg from starting the *next* leg still
  upside-down at the spawn point.

Current best guess: a Gazebo contact-physics settling issue specific to
this vehicle's collision geometry at spawn, not a PX4/EKF or project-code
bug — but this is unconfirmed. Next things worth trying, not yet
attempted: adjusting the `base_link_collision` contact `<ode>` stiffness/
damping (`kp`/`kd`/`max_vel` in `model.sdf`), or checking whether the
flip correlates with spawn `min_depth`/initial penetration rather than
with any PX4-side event at all.

### Canyon geometry: symmetric entry/exit, ground/tower colours

`CANYON_ENTRY`/`CANYON_EXIT` were `[-90, 0, 25]`/`[110, 0, 25]` --
asymmetric around the tower group's own centre (the towers themselves
already span x in [-55, 55], centred at x=0, and the ground plane is
centred at the origin too). Changed to `±100` so the whole layout,
including the vehicle's spawn point (`run_trial.SPAWN_XYZ`, derived from
`CANYON_ENTRY`), is centred on the ground plane rather than offset
toward the exit end. Mission distance is unchanged (still 200m). Ground
plane recoloured green, towers beige (`canyon_geometry.to_sdf`,
`worlds/_template.sdf`) -- purely cosmetic.

### AUTO_LAND turns the vehicle's heading during descent (accepted)

Landing hands off to PX4's own `VEHICLE_CMD_NAV_LAND`/`AUTO_LAND`
(`controller_node.LAND_CLEARANCE_M`), which can visibly turn the vehicle
off whatever heading it had at the moment landing was requested -- worth
knowing if you're watching a trial fly and the heading change looks
surprising. A self-controlled descent was tried instead specifically to
avoid this (freeze the position at the clearance point, keep publishing
`cruise_yaw` every tick under this node's own offboard control, same
NaN-authority setpoint pattern as the whole transit) and it DID hold
heading correctly. But its own disarm logic proved genuinely unsafe:
verified live, repeatedly, that the vehicle could destabilise into a
violent, uncontrolled tumble (roll/pitch/yaw all swinging wildly,
climbing back up to 60m instead of landing) after sitting near the
ground for 20-30+ seconds. The most likely mechanism:
`VEHICLE_CMD_COMPONENT_ARM_DISARM` without PX4's force parameter can be
silently REJECTED while PX4 doesn't consider the vehicle landed, and
this node's own logic stopped publishing setpoints the moment it
(wrongly) assumed the disarm had succeeded -- leaving the vehicle
airborne under thrust with zero control input. Neither an instantaneous
`|z|`/speed threshold, a sustained multi-second settle check, nor an
unconditional timeout backstop closed this gap reliably enough to trust
live. AUTO_LAND -- PX4's own, field-tested landing logic, including its
own correct judgement of when disarming is actually safe -- doesn't have
this failure mode, and was verified live (repeatedly, across baseline
and treatment) to always land flat and level and disarm cleanly, even
though the heading can turn on the way down. A turn during descent is
cosmetic; loss of control is not. If heading-locked landing is revisited
later, the disarm-safety problem (not the heading problem) is what
actually needs solving first -- e.g. investigating PX4's force-disarm
parameter (mavlink `PARAM2=21196` convention) rather than another
settle-detection heuristic.
