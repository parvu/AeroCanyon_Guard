# 2026-09-06: Native FW transition — status report

Session goal: replace the custom Lua-driven "rear becomes cruise pusher"
transition (from the 2026-09-04/05 scripting-mixer design) with
ArduPilot's own native tiltrotor/QuadPlane transition logic ("disable
script and let ardupilot do the transition"), then debug the resulting
flight behavior live in SITL.

## What changed

**Architecture reversal** (`scripts/tricopter_mixer.lua`, `tricopter.parm`):
- Front pair now tilts fully forward and becomes the cruise thruster —
  ArduPilot's standard tiltrotor assumption — instead of staying
  near-vertical while the rear tilted to become the pusher.
- `tricopter_mixer.lua` reduced to a one-shot script: registers the 3
  motors, loads a single static (not hover/cruise-blended) motor factor
  table, pins the rear tilt vertical once, and returns. No more
  transition state machine, tilt slewing, or airspeed-trigger logic in
  Lua — all of that is now native ArduPilot behavior.
- `SERVO12/13_FUNCTION` reverted to native `k_tiltMotorRight/Left`
  (were `k_scripting2/3`). Restores native `Q_TILT_TYPE=2` VectoredYaw
  hover-yaw authority as a side effect (degraded since 2026-09-05).
- Kept `Q_FRAME_CLASS=17` (Dynamic Scripting Matrix) rather than
  reverting to `7` (`AP_MotorsTri`) — `AP_MotorsTri` hardcodes pitch
  factors for a 2:1 rear-to-front arm ratio, and this airframe's arms
  are equal length, which caused a confirmed hover pitch-runaway crash
  earlier in the project. The static factor table in the simplified
  script is what avoids reintroducing that bug.

**C++ patch reverted** (`patches/tricopter-fw-transition.patch`,
`~/ardupilot` checkout, rebuilt via `./waf plane`): the 2026-09-05
patch capping front tilt at `Q_TILT_MAX` in `tiltrotor.cpp` assumed the
old design (front stays near-vertical). Reverted to stock full-forward
slew, needed for the front pair to actually reach cruise-thrust angle.
The `quadplane.cpp` fix (skip the 2s climb-rate-assist ramp for
tiltrotors) stayed — its reasoning holds regardless of which motors tilt.

**Bugs found and fixed, in the order they surfaced:**

1. **`Q_TILT_MASK=3` was backwards.** It's a bitmask over `AP_Motors`'
   0-based index (confirmed against `Tiltrotor::tiltrotor_is_motor_tilting()`),
   not a 1-indexed label. `3` = bits 0+1 = front-right + **rear** (this
   project's own `Motors_dynamic` numbering: 0=front-right, 1=rear,
   2=front-left) — exactly backwards. Fixed to `5` (bits 0+2 =
   front-right + front-left). This also explains a previously-mysterious
   "front-left pins at minimum PWM" note from 2026-09-05.
2. **Disarmed front tilt defaulted to full-forward.** Without
   `Q_OPTIONS` bit 21 (`DISARMED_TILT_UP`), `Tiltrotor::continuous_update()`
   tilts the front pair to ~90° whenever disarmed — correct for a
   normal plane resting on wheels, wrong for a vehicle that needs to
   hover-launch vertically. Set `Q_OPTIONS=2097152` (bit 21 alone).
3. **Mode must be set before arming, not after.** With `FLTMODE_CH=0`
   disabling the RC mode switch, the vehicle sits in compiled-default
   `MANUAL` at boot, and `DISARMED_TILT_UP` is unconditionally bypassed
   while `control_mode==MANUAL` regardless of `Q_OPTIONS`. Arming
   before switching to `AUTO` left a ~2s window armed-in-MANUAL with
   front tilt pinned forward, causing an immediate flip once
   `VTOL_TAKEOFF` demanded climb throttle. Test scripts fixed to set
   mode first.
4. **`Q_TILT_MAX=20` (from the old design) was far too conservative**
   for the front pair to ever build real airspeed. Live-tested in
   steps holding the vehicle at 50m: 20° → ~1.3 m/s plateau, 45° →
   ~6 m/s plateau, 70° → cleared `ARSPD_FBW_MIN=9`, and the real
   ArduPilot transition engaged (`Transition airspeed reached`,
   `Transition FW done`, front to 90°, rear correctly stopped). Raised
   to `Q_TILT_MAX=70`.
5. **`TECS_PITCH_MAX=15`/`TECS_PITCH_MIN=0` clamped TECS's pitch
   *target*** to a narrow band regardless of how large the actual
   attitude error is (`AP_TECS.cpp`: `_pitch_dem = constrain_float(_pitch_dem_unc, _PITCHminf, _PITCHmaxf)`).
   This, not gain magnitude or servo direction, was the actual cause of
   an uncorrected nose-down dive right after `Transition FW done`
   (confirmed by two ruled-out hypotheses first: `SERVO1/2_REVERSED`
   direction made no difference, and a 5x `PTCH_RATE_P` raise made no
   difference — both symptoms of a target that was never large enough
   to chase). Proven with a decisive test: bypassing the controller
   entirely (`SERVO1/2_FUNCTION` → raw RC5/6 passthrough) and slamming
   the elevons to near-full deflection **did** recover the aircraft
   from a -89° dive to level flight — proof the airframe has enough
   authority and this clamp was the bottleneck. Raised to `±45`.

**Gain changes** (`PTCH_RATE_P/D`, `PTCH2SRV_TCONST`, `RLL_RATE_P/D`,
`RLL2SRV_TCONST`, `SERVO1/2_REVERSED`): this project tuned the Q-mode
(`Q_A_RAT_*`) gains extensively but never touched ArduPilot's
fixed-wing side, which was still 100% stock defaults. Raised as a
starting point once the TECS clamp was identified; see inline comments
in `tricopter.parm` for exact before/after values and reasoning.

## Verified working

The native transition **mechanics** are fully verified live and correct:
mode-before-arm sequencing, front pair tilting fully forward on its own
schedule, airspeed-gated cutover firing (`Transition airspeed wait` →
`Transition airspeed reached` → `Transition FW done`), rear motor
correctly stopping once transitioned. This is the actual "let ArduPilot
do the transition" request — done and confirmed.

## Not yet resolved

Once genuinely in fixed-wing flight, the aircraft cannot yet complete a
clean recovery through the pitch/roll coupling that occurs as pitch
passes through the near-90° region during the post-transition
correction. Behavior is inconsistent run to run as gains are adjusted:
one configuration crashed via an uncontrolled roll-to-inverted while
pitch itself recovered cleanly; the next (after fixing that with
`RLL_RATE_P/D`) had roll stay well-controlled but pitch fell into an
oscillation and crashed instead. This is consistent with the system
sitting right at the edge of controllability for this specific
maneuver — not a single remaining discrete bug, but a genuine
multi-axis tuning problem.

**Recommended next step**: rather than continuing single-flight manual
gain guesses, use ArduPilot's own `AUTOTUNE` flight mode (or a more
systematic sweep) for `PTCH_RATE_*`/`RLL_RATE_*`, ideally isolated from
the full transition maneuver first (e.g. tune in level FBWA flight,
then re-test the transition) rather than tuning directly against the
hardest case.

## Environment note (not committed)

This WSL2 local SITL setup has two independent flakiness sources,
unrelated to the code above, worked around during this session but not
fixed in the repo:
- ArduPilot's own internal "reboot" (triggered by cascading `Q_ENABLE`/
  `SCR_ENABLE`-style dependent param groups) calls a real process exit
  expecting an external supervisor to relaunch it, like a real flight
  controller reset — there is none in the manual/README launch flow.
  A simple respawn-loop wrapper around `arduplane` resolved this
  reliably; worth folding into the README's manual-flight instructions
  if this keeps recurring.
- `web_viewer/control_server.py` publishes `OverrideRCIn` with channels
  5-8 forced to `RC_CENTER` continuously — it must be stopped before
  using those channels for anything else (e.g. a raw RC-passthrough
  diagnostic test, as used to prove elevon authority above).
