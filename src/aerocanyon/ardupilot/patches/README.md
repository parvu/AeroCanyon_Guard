# ArduPilot source patches

`~/ardupilot` is a separate, external checkout of upstream ArduPilot
(pinned to commit `b9439efde1`, see the top-level README's
Prerequisites section) -- not part of this repo's own git history.
These patches are kept here so the deviations from stock ArduPilot are
documented and reproducible; they are NOT applied automatically by any
build step. Apply with:

```
cd ~/ardupilot
git apply ~/AeroCanyon_Guard/src/aerocanyon/ardupilot/patches/tricopter-fw-transition.patch
./waf plane
```

## `tricopter-fw-transition.patch`

Two fixes to `ArduPlane/quadplane.cpp` and `ArduPlane/tiltrotor.cpp`,
found live-debugging this project's FW-transition crash (2026-09-05):

1. **`quadplane.cpp`, `assist_climb_rate_cms()`**: skips the 2-second
   ramp-from-zero on the assisted-flight climb-rate demand for
   tiltrotors. That ramp assumes an airframe with a dedicated
   forward-thrust motor, where easing VTOL climb-rate authority in over
   2s while the forward motor takes over is fine -- this tiltrotor has
   no such motor and depends entirely on VTOL lift until real airspeed
   is reached, so ramping authority from zero right as assisted_flight
   engages guaranteed a ~2s window with near-zero climb-rate correction
   at exactly the moment a sink was most likely to start.

2. **`tiltrotor.cpp`, `continuous_update()`**: caps the front tilt
   pair at `Q_TILT_MAX` (this project's own deliberate ceiling)
   instead of slewing to full forward tilt (~90deg), in BOTH places
   `continuous_update()` can reach that slew (the assisted_flight/TIMER
   branch, and the earlier `!in_vtol_mode() && !assisted_flight`
   branch that fires when `assisted_flight` flickers false mid-flight).
   Unconditional full-forward tilt assumes the STANDARD ArduPilot
   tiltrotor design, where the front (tilting) pair becomes the cruise
   thruster once transitioned -- this airframe is the deliberate
   opposite (only the rear motor tilts/cruises, see
   `scripts/tricopter_mixer.lua`), so slamming the front pair to 90deg
   removes vertical lift this design never intended to give up, and
   feeds a rapidly-changing tilt fraction into `vectoring()`'s live
   sin/cos yaw-roll mixing, producing a torque transient. Confirmed via
   live SITL RCOU: before this patch, both front tilt servos pinned at
   PWM 2000 (max) for the rest of the flight; after, they hold exactly
   the predicted Q_TILT_MAX-derived PWM (~1363).

**Not fixed by this patch**, still open as of 2026-09-05: even with
both of the above applied, the front-left motor still pins at minimum
PWM while front-right and rear pin at max, for the remainder of a
flight -- a real, unresolved asymmetry in AP_MotorsMatrix's
RPY/collective throttle allocation once this airframe's assisted-flight
transition triggers, independent of the tilt-servo behavior these
patches fix. See `ardupilot_phase2_notes` project memory and
`tricopter_mixer.lua`'s own comments for the mixer-side attempts (and
their failures) at addressing this.
