# AUTO-mode mission navigation for controller_node — Design

## Context

Earlier this session, `controller_node.py` was ported to a hand-rolled
outer position/altitude/heading control loop that converts a desired
acceleration into RC-override lean-angle commands, published at 50Hz
via `/mavros/rc/override` (see
`docs/superpowers/specs/2026-09-02-mission-stack-mavros-port-design.md`).
That design was forced by an earlier finding: ArduPilot exposes no live
GUIDED-mode XY position/velocity injection for this airframe
(`MAV_TYPE_VTOL_TILTROTOR`) in this ArduPilot build.

That RC-override loop just failed a live demo, watched directly in the
browser: the baseline leg drifted laterally under wind and struck a
canyon tower. The user, watching live, proposed switching to
ArduPilot's own AUTO-mode mission navigation instead of continuing to
hand-tune a from-scratch P/D loop.

## Feasibility (confirmed against ArduPilot's own source, not assumed)

`QuadPlane::in_vtol_auto()` (`ArduPlane/quadplane.cpp` ~line 2025)
explicitly recognizes `MAV_TYPE_VTOL_TILTROTOR` and keeps
`plane.auto_state.vtol_mode` latched true through a mission's
`MAV_CMD_NAV_WAYPOINT` items once set by an initial
`MAV_CMD_NAV_VTOL_TAKEOFF` — it only clears on an explicit transition
command, which this project's controller never issues
(`ENABLE_VTOL_TRANSITION` stays `False`, matching Phase 1/2's
hover-only scope). A mission of `[NAV_VTOL_TAKEOFF, NAV_WAYPOINT,
NAV_VTOL_LAND]` therefore flies entirely in Q-mode/VTOL navigation,
using ArduPilot's own real position controller — never touching fixed-
wing flight.

This is mechanically different from the earlier GUIDED-mode finding:
AUTO reads a **pre-uploaded mission** (MAVLink's
MISSION_COUNT/MISSION_ITEM_INT/MISSION_ACK protocol, exposed via
MAVROS's mission plugin — `mavros_msgs/WaypointPush`,
`/mavros/mission/push`), not a live per-tick position-target
injection. It is a completely different ArduPilot code path from
`handle_set_position_target_local_ned`, which is GUIDED-only and was
already confirmed broken for this airframe. Nothing about that earlier
finding applies here.

## Architecture

`controller_node.py`'s outer P/D loop and its RC-override publishing
are removed entirely. In their place:

1. **On startup** (both modes): convert the mission's two real-world
   points — `CANYON_ENTRY` (takeoff/cruise point) and the existing
   landing-trigger point (last tower row's edge + `LAND_CLEARANCE_M`,
   **not** `CANYON_EXIT` — see below) — from this project's local NED
   canyon frame to global lat/lon via a new `frames.ned_to_latlon(ned,
   home_lat, home_lon)` helper (a standard flat-earth/local-tangent-
   plane approximation, accurate to sub-centimetre at this project's
   ~250m scale — the same approximation ArduPilot's own EKF uses
   internally for local↔global conversion at this scale). Build a
   3-item mission:
   - `NAV_VTOL_TAKEOFF` at `CANYON_ENTRY`, `FRAME_GLOBAL_RELATIVE_ALT`,
     alt = 25m (matching `CANYON_ENTRY`'s existing NED altitude,
     unchanged)
   - `NAV_WAYPOINT` at the landing-trigger point, same altitude
   - `NAV_VTOL_LAND` at the landing-trigger point
   Push it via `/mavros/mission/push` (a `CommandBool`-style async
   service call, matching the existing `rc_pwm.py` pattern for
   arm/mode calls — never block the tick loop on it).
2. **Arm + switch to AUTO** (replacing today's arm + QHOVER retry
   loop): same retry-until-`mavros_armed` pattern already proven in
   the current code, just requesting the `AUTO` custom mode instead of
   `QHOVER`.
3. **Baseline mode**: nothing further. ArduPilot's own navigation
   controller flies the mission — takeoff, cruise, arrival, transition
   to `NAV_VTOL_LAND`, and disarm-on-touchdown — entirely on its own.
   Real wind (from `wind_field_node.py`, unchanged) still applies and
   ArduPilot's own inner-loop control still rejects some of it, same
   as any real autopilot; there is no PINN/CBF correction in this
   mode, matching its role as the uncorrected comparison arm.
4. **Treatment mode**: every ~1s, compute the CBF-filtered correction
   as today (`cbf.filter(u_des, self.pos, self.vel, self.wind_truth,
   self.quat)`, where `u_des` is still the PINN feedforward,
   unchanged) and integrate it into a **cumulative position offset**:
   `Δoffset += 0.5 * u_safe[:2] * dt²` (kinematic displacement over the
   update interval; only horizontal components — altitude stays flown
   by the mission's own fixed alt), clamped to a small max magnitude
   (a few metres) so a runaway correction can't push the target
   waypoint somewhere unsafe, mirroring the existing
   `feedforward_gain` comment's own "must fit inside the controller's
   authority" reasoning. Re-push waypoint index 1 (the cruise
   waypoint) via `WaypointPush` with `start_index=1`, position =
   landing-trigger point's lat/lon + `Δoffset` converted back through
   the same NED→lat/lon helper.

`controller_node.py` no longer tracks mission phase, elapsed time, or
a position-triggered landing check at all — ArduPilot's own mission
sequencer owns that entirely. `mission.py`'s time-parameterized
`Mission.target(t)` becomes unused by `controller_node.py` (kept for
now — `plot_results.py` and others may still reference
`canyon_geometry`/mission constants directly; only `controller_node`'s
own consumption of `Mission.target()` goes away).

## Landing point, not `CANYON_EXIT`

The current code already documents `CANYON_EXIT` as deliberately
placed 45m past the real landing point, "for stable transit dynamics,
not as a landing cue" — a concession to the old hand-rolled
trajectory-follower's own stability needs. That reasoning does not
apply to ArduPilot's own navigation controller, so the mission targets
the real landing-trigger point directly (last tower row's edge +
`LAND_CLEARANCE_M`, exactly the existing `LAND_TRIGGER_LOCAL_M`/
`LAND_CLEARANCE_M` geometry) rather than routing through the
now-unnecessary extra margin.

## Data flow (treatment mode, per correction-update tick)

```
CBFFilter.filter(u_des, self.pos, self.vel, wind_truth, quat)
  -> u_safe (NED acceleration)
  -> Δoffset += 0.5 * u_safe[:2] * dt^2   (clamped)
  -> landing_trigger_ned[:2] + Δoffset
  -> frames.ned_to_latlon(...)
  -> WaypointPush(start_index=1, waypoints=[updated cruise waypoint])
```

Baseline mode has no equivalent — it uploads the mission once and does
nothing further.

## Components

- **`frames.py`** — new `ned_to_latlon(ned, home_lat, home_lon)` and
  its inverse (or the inverse folded into the same call site) — flat-
  earth approximation, a new small pure-function unit, tested the same
  way as this file's other conversions.
- **`controller_node.py`** — outer P/D loop, RC-override publishing,
  and the position-triggered landing check are deleted. New: mission
  construction/upload on startup, arm+AUTO engage retry (replacing
  arm+QHOVER), and (treatment only) the periodic waypoint-offset
  update timer.
- **`rc_pwm.py`** — untouched. `control_server.py`'s manual-flight
  path (which still uses RC-override for hand-flying) is completely
  unaffected by this change; only `controller_node`'s autonomous-
  mission path changes.
- **`run_trial.py`** — unaffected. Its per-leg Gazebo/SITL/MAVROS
  lifecycle and `/mavros/extended_state`-based landing detection
  already don't care how the vehicle is being commanded.
- **`mission.py`** — `Mission.target(t)` becomes dead code for
  `controller_node.py`'s purposes; left in place rather than deleted
  in this pass (other code may still use `canyon_geometry`/mission
  constants directly) — a follow-up cleanup, not blocking.

## Testing

- Unit tests for `frames.ned_to_latlon` (pure function, no rclpy) —
  round-trip accuracy against a known reference point, and a sanity
  check that a small NED offset produces the expected sign/magnitude
  of lat/lon change.
- Unit tests for the waypoint-offset accumulation math (pure function,
  extracted from the timer callback) — zero correction produces zero
  offset, a sustained correction accumulates and clamps correctly.
- `controller_node.py`'s existing tests
  (`test_requests_qhover_and_arm_after_setpoint_stream`, the landing
  tests, etc.) need rewriting for the new mission-upload/AUTO-engage
  behavior — mission upload and mode-engage calls get mocked the same
  way `arm`/`set_mode` are mocked today.
- Manual verification (same discipline as every other change this
  session): fresh full-stack restart, watch one full baseline leg fly
  live in the browser, confirming it takes off, cruises, and lands
  without drifting into a tower, before trusting any automated
  baseline/treatment comparison.

## Out of scope for this change

- VTOL forward-flight transition (`ENABLE_VTOL_TRANSITION`) — still a
  separate, later sub-project.
- The 49-point wind sweep and any further FO-PINN retraining — blocked
  on this fix landing and being verified live first.
- Rewriting `mission.py`'s time-parameterized design or removing its
  now-dead `controller_node` consumption path — noted above as a
  follow-up, not part of this change.
