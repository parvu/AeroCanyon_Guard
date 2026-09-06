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

One fix remains applied to `ArduPlane/quadplane.cpp`, found
live-debugging this project's FW-transition crash (2026-09-05):

1. **`quadplane.cpp`, `assist_climb_rate_cms()`**: skips the 2-second
   ramp-from-zero on the assisted-flight climb-rate demand for
   tiltrotors. That ramp assumes an airframe with a dedicated
   forward-thrust motor, where easing VTOL climb-rate authority in over
   2s while the forward motor takes over is fine -- this tiltrotor has
   no such motor and depends entirely on VTOL lift until real airspeed
   is reached, so ramping authority from zero right as assisted_flight
   engages guaranteed a ~2s window with near-zero climb-rate correction
   at exactly the moment a sink was most likely to start. Still valid
   for any tiltrotor regardless of which motors do the tilting, so kept
   through the 2026-09-06 redesign below.

**REVERTED 2026-09-06** (pilot's call: "disable script and let ardupilot
do the transition"): `tiltrotor.cpp`, `continuous_update()` no longer
caps the front tilt pair at `Q_TILT_MAX` -- it slews to full forward
tilt (~90deg) again, ArduPilot's own unmodified behaviour. That cap
existed because this project's earlier design kept the front pair
near-vertical always and used only the rear motor as the cruise pusher
(entirely managed by `scripts/tricopter_mixer.lua`'s own transition
state machine) -- the project now uses ArduPilot's standard tiltrotor
assumption instead (front pair tilts fully forward and becomes the
cruise thruster, native `Tiltrotor`/`QuadPlane` transition logic used
as-is), so the cap's reasoning no longer applies. The file now carries
only comment-only diffs marking where the reverted hunks were: no
functional difference from stock `tiltrotor.cpp`. The patch file itself
was regenerated from `git diff` after this revert, so applying it fresh
reproduces exactly this state (fix #1 + comment-only tiltrotor.cpp
markers), not the old front-tilt cap.

**Previously "not fixed by this patch" (2026-09-05), root-caused and
fixed 2026-09-06**: the front-left motor pinning at minimum PWM while
front-right/rear pinned at max during assisted flight turned out to be
`Q_TILT_MASK` itself, not `AP_MotorsMatrix`'s RPY/collective allocation
-- see `tricopter.parm`'s own comment on that param. `Q_TILT_MASK=3`
(binary `011`, bits 0+1) told `Tiltrotor::tiltrotor_is_motor_tilting()`
that motor index 1 (the REAR motor, in this project's `Motors_dynamic`
numbering) was one of the tilting pair, while excluding index 2
(front-left) from ever being recognised as tilting -- exactly backwards
from the intended front-right+front-left pair. Corrected to `5` (binary
`101`, bits 0+2).
