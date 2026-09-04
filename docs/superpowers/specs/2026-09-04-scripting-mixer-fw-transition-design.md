# FW transition via a dynamic scripting motor mixer

2026-09-04. Architectural design for resolving the tiltrotor.cpp
motor-role mismatch identified in `ardupilot_phase2_notes.md` item 3.

## Problem

This vehicle's real design: front motor pair fold/stop in cruise, rear
motor is the sole forward-flight thruster. ArduPilot's stock QuadPlane
(`Q_FRAME_CLASS=7`, Tri) assumes the opposite -- `Q_TILT_MASK` motors
(the front pair) become cruise thrusters as they tilt forward
(`tiltrotor.cpp`'s `tilt_compensate()` boosts tilting-motor thrust in
forward flight), and there is no built-in QuadPlane frame class for "2
lift motors + control-surface hover stabilization" (`MOTOR_FRAME_SINGLE`/
`COAX` aren't handled in QuadPlane's frame-class switch at all --
confirmed via `quadplane.cpp`'s actual switch statement, not just its
stale param doc comment). `Q_FWD_THR_USE` (the stock "separate forward
thrust motor" feature) needs a genuinely separate physical motor
outside the Q_ mixer -- this airframe only has 3 motors, all currently
inside the Tri mixer.

## Approach

Replace the Tri mixer with `Q_FRAME_CLASS=17`
(`AP_MotorsMatrix_Scripting_Dynamic`, "Dynamic Scripting Matrix") and a
Lua script that owns motor-factor allocation directly, swapping between
a hover table and a cruise table at runtime. Confirmed available in
this ArduPilot build (Lua scripting already compiled into the SITL
binary; relevant bindings present in
`AP_Scripting/generator/description/bindings.desc`):
`quadplane:in_vtol_mode()` (mode-detection trigger), `Motors_dynamic`/
`motor_factor_table` (the dynamic mixer itself), `SRV_Channels` (to
drive the rear tilt servo directly).

The rear motor's tilt joint (`motor_1_joint` in
`tricopter_ap/model.sdf`, revolute about Y, range -20..+90 deg, 0=down/
hover, +90=horizontal/cruise) already exists physically -- no new
geometry needed. It currently gets driven automatically by
`tiltrotor.cpp` via `SERVO14_FUNCTION=45` (`k_tiltMotorRear`), which
follows the same tilt-progress as the front pair. That has to be taken
away from ArduPilot's own Tiltrotor class so the script can own it
independently: reassign `SERVO14_FUNCTION` to a scripting function
(`k_scripting1`, ArduPilot's Lua-controlled servo output) instead.

## Components

**Motor factor tables** (`motor_factor_table()`, one hover + one
cruise):
- Hover: front pair (motors 0, 2) = roll + pitch + yaw + throttle,
  matching today's Tri mixer allocation as closely as possible. Rear
  (motor 1) = pitch + throttle only -- no roll, no yaw (yaw stays the
  front pair's `Q_TILT_TYPE=2` VectoredYaw job, unchanged).
- Cruise: front pair throttle factor zeroed (fold/stop; roll/pitch/yaw
  factors irrelevant once throttle is zero). Rear = throttle only, no
  roll/pitch/yaw (those come from elevons/rudder in forward flight, the
  same as any fixed-wing).

**Mode-detection + tilt coordination** (a single script, runs on a
timer callback): reads `quadplane:in_vtol_mode()`. On a transition
edge, slews the rear tilt servo (via `SRV_Channels:set_output_scaled`
on `k_scripting1`) toward the target angle (0 deg for hover, 90 deg for
cruise) over a fixed duration -- not instant -- and only calls
`Motors_dynamic:load_factors()` to swap tables once the tilt has
reached (or is very close to) its target. This ordering matters: a
mid-tilt rear thrust vector combined with the wrong factor table would
put thrust into the wrong axis (e.g. rear thrust partially horizontal
while the hover table still expects it to contribute pure vertical
pitch/throttle).

**Config changes** (`tricopter.parm`): `Q_FRAME_CLASS` 7->17,
`SERVO14_FUNCTION` 45->`k_scripting1`'s function number, `SCR_ENABLE=1`,
script placed in the SITL scripts directory. `Q_TILT_MASK`/
`Q_TILT_TYPE=2`/`Q_TILT_MAX` etc. (front pair's own VectoredYaw tilt)
are UNCHANGED -- this only touches the rear motor's role and the
overall motor mixer, not the front pair's existing tilt-yaw mechanism.

## Testing / re-tuning plan

Existing `Q_A_RAT_*` gains, `Q_M_THST_HOVER`, etc. are tuned against
the Tri mixer's specific thrust allocation and will very likely need
retuning under the new one, even though the hover table's front-pair
factors are designed to match as closely as possible. Order of
verification, each gated on the previous succeeding (no skipping ahead
even if short on patience -- every step this whole project has taken
without live verification has cost more time backtracking):

1. Load the new config, confirm SITL boots without a `config_error`
   and the script loads without a Lua error (check `SCR_ENABLE`
   logs/`MAVLink STATUSTEXT` for script load failures before ever
   arming).
2. QSTABILIZE hover, disarmed-adjacent altitude only (a few meters),
   re-tune rate/attitude gains from scratch if needed -- same process
   as the original Phase 1 hover tuning.
3. Re-verify the existing AUTO mission (`map_zone_demo.json`) still
   flies correctly under the new mixer, hover-only (no transition
   attempted yet).
4. Only then: a real transition attempt -- commanding `in_vtol_mode()`
   to go false and watching the rear tilt sweep + factor-table swap
   happen live, at low altitude, calm air (no `wind_field_node`) first.

## Out of scope for this spec

- Real fixed-wing cruise gain tuning (TECS, `PTCH_LIM`, aileron/rudder
  authority) -- once the mixer/tilt mechanism itself is verified
  working, that's its own separate tuning pass.
- Reverting if this doesn't work out -- if the Dynamic Scripting Matrix
  approach hits an unforeseen blocker (e.g. `Tiltrotor`'s
  `VectoredYaw` logic turns out to depend on the base motor class in a
  way not caught by this design's review of its generic
  `is_motor_enabled()`/`get_roll_factor()` calls), falling back to
  `Q_FRAME_CLASS=7` is a config revert, not a data-loss risk -- not
  spec'd further here since it's the same as any other rollback.
