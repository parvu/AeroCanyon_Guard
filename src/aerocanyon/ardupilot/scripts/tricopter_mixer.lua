-- Dynamic scripting motor mixer for the tricopter-ap VTOL.
-- Hover: front pair (motors 0,2) do roll+pitch+yaw+throttle (matches the
-- old Q_FRAME_CLASS=7 Tri mixer's allocation as closely as possible);
-- rear (motor 1) does pitch+throttle only -- no roll, no yaw (yaw stays
-- the front pair's own Q_TILT_TYPE=2 VectoredYaw job, unchanged).
-- Cruise: front pair throttle zeroed (fold/stop); rear is throttle-only
-- (roll/pitch/yaw come from elevons/rudder in forward flight).
--
-- The rear motor's own tilt joint (SERVO14, reassigned from
-- k_tiltMotorRear to k_scripting1 by this same config change so this
-- script can own it independently of ArduPilot's own Tiltrotor class)
-- is slewed 0deg(down/hover)->90deg(horizontal/cruise) in lockstep with
-- the factor-table swap. The table swap only happens once the tilt has
-- actually reached its target -- a mid-tilt rear thrust vector combined
-- with the wrong table would put thrust into the wrong axis. See the
-- design spec's "Mode-detection + tilt coordination" section.

local UPDATE_HZ = 10
local TILT_SLEW_DURATION_S = 3.0
local REAR_TILT_FUNCTION = 94  -- k_scripting1, see SERVO14_FUNCTION in tricopter.parm

local MOTOR_FRONT_RIGHT = 0
local MOTOR_REAR = 1
local MOTOR_FRONT_LEFT = 2

-- Pitch factors are NOT copied from AP_MotorsTri's own hardcoded values
-- (neither get_pitch_factor_json()'s dead-code 0.5/-1.0 nor
-- output_armed_stabilizing()'s real flown 0.5/-0.5) -- both of those are
-- calibrated for AP_MotorsTri's assumed frame geometry, where the rear
-- tail boom is roughly 2x the length of the front arms. Checked against
-- our own model.sdf 2026-09-04: motor_0/motor_2 (front pair) sit at
-- x=+0.35m from CG, motor_1 (rear) sits at x=-0.35m -- EQUAL arm
-- lengths, not the 2:1 ratio Tri assumes. Pitch torque is
-- factor*arm_length per motor; with two front motors combining and one
-- rear motor alone, balance requires 2*front_factor = |rear_factor| for
-- equal arms, i.e. front=0.5 each pairs with rear=-1.0, not -0.5.
-- Live-caught 2026-09-04 (three times): (1) front=+-1.0/rear=-1.0 --
-- front combined torque (2*1.0=2.0) 2x rear (1.0), front-dominant
-- runaway; (2) front=+-0.5/rear=-1.0 -- correctly balanced (1:1), stable
-- 10s hover, this is the right answer; (3) "fixed" down to rear=-0.5 by
-- copying AP_MotorsTri's real source verbatim without checking OUR
-- geometry -- silently reintroduced the same class of bug in the other
-- direction (front combined 0.35 vs rear 0.175, front now 2x rear
-- again), causing the same kind of positive-feedback pitch runaway.
-- Roll only involves the front pair (rear has zero roll authority by
-- design), and both front motors sit at equal |y| offsets, so roll's
-- +-0.5/+-0.5 split is inherently balanced regardless of arm length --
-- no analogous bug possible there.
-- Kept as plain per-axis/per-motor numbers (not built straight into a
-- motor_factor_table) so update() can linearly blend hover -> cruise
-- over BLEND_DURATION_S instead of swapping tables in one tick -- see
-- the 2026-09-05 comment in update() for why the instant swap crashed.
local HOVER_ROLL  = {[MOTOR_FRONT_RIGHT]=-0.5, [MOTOR_FRONT_LEFT]=0.5, [MOTOR_REAR]=0.0}
local HOVER_PITCH = {[MOTOR_FRONT_RIGHT]=0.5,  [MOTOR_FRONT_LEFT]=0.5, [MOTOR_REAR]=-1.0}
local HOVER_YAW   = {[MOTOR_FRONT_RIGHT]=0.0,  [MOTOR_FRONT_LEFT]=0.0, [MOTOR_REAR]=0.0}
local HOVER_THR   = {[MOTOR_FRONT_RIGHT]=1.0,  [MOTOR_FRONT_LEFT]=1.0, [MOTOR_REAR]=1.0}

local CRUISE_ROLL  = {[MOTOR_FRONT_RIGHT]=0.0, [MOTOR_FRONT_LEFT]=0.0, [MOTOR_REAR]=0.0}
local CRUISE_PITCH = {[MOTOR_FRONT_RIGHT]=0.0, [MOTOR_FRONT_LEFT]=0.0, [MOTOR_REAR]=0.0}
local CRUISE_YAW   = {[MOTOR_FRONT_RIGHT]=0.0, [MOTOR_FRONT_LEFT]=0.0, [MOTOR_REAR]=0.0}
local CRUISE_THR   = {[MOTOR_FRONT_RIGHT]=0.0, [MOTOR_FRONT_LEFT]=0.0, [MOTOR_REAR]=1.0}

local MOTOR_IDS = {MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT, MOTOR_REAR}

-- Builds a motor_factor_table blended between hover (blend=0) and
-- cruise (blend=1). Called every tick while blend is moving so the
-- mixer output is continuous, not a step.
local function build_blended_factors(blend)
    local t = motor_factor_table()
    for _, m in ipairs(MOTOR_IDS) do
        t:roll(m, HOVER_ROLL[m] + (CRUISE_ROLL[m] - HOVER_ROLL[m]) * blend)
        t:pitch(m, HOVER_PITCH[m] + (CRUISE_PITCH[m] - HOVER_PITCH[m]) * blend)
        t:yaw(m, HOVER_YAW[m] + (CRUISE_YAW[m] - HOVER_YAW[m]) * blend)
        t:throttle(m, HOVER_THR[m] + (CRUISE_THR[m] - HOVER_THR[m]) * blend)
    end
    return t
end

Motors_dynamic:add_motor(MOTOR_FRONT_RIGHT, 1)
Motors_dynamic:add_motor(MOTOR_REAR, 2)
Motors_dynamic:add_motor(MOTOR_FRONT_LEFT, 3)
Motors_dynamic:load_factors(build_blended_factors(0.0))
assert(Motors_dynamic:init(3), "tricopter_mixer: failed to init Motors_dynamic")
motors:set_frame_string("tricopter scripting mixer")

local current_tilt_deg = 0.0
local cruise_request_ticks = 0
local CRUISE_DEBOUNCE_TICKS = 10  -- 1s at UPDATE_HZ=10
local MIN_CRUISE_AIRSPEED_MS = param:get('ARSPD_FBW_MIN') or 6.0
local cruise_blend = 0.0
local BLEND_DURATION_S = 2.5
local BLEND_STEP = 1.0 / (BLEND_DURATION_S * UPDATE_HZ)

local function update()
    -- Live-caught 2026-09-04: gating purely on in_vtol_mode() let the
    -- vehicle sit in the cruise table/tilt=90 (horizontal) whenever it
    -- was armed from a non-VTOL mode (e.g. MANUAL) -- ArduPilot's own
    -- baseline arm-idle throttle then pushed the whole idle-spin thrust
    -- straight out the rear motor, horizontally, jolting the vehicle
    -- off its spawn point the instant it armed, before any real climb
    -- attempt. Disarmed always means hover-safe (tilt=0, hover table),
    -- regardless of whatever mode happens to be selected -- a grounded,
    -- about-to-arm vehicle must never be caught horizontal.
    --
    -- Live-caught 2026-09-04 (again): in_vtol_mode() itself
    -- (QuadPlane::in_vtol_mode() in quadplane.cpp) legitimately reads
    -- false for a brief window right after arming into AUTO -- it
    -- depends on poscontrol's internal state machine advancing past
    -- QPOS_AIRBRAKE, which hasn't happened yet in the same tick as
    -- arming. That one-tick false reading was enough to make
    -- want_hover false immediately on arm, tilting the rear motor
    -- horizontal before the mission's VTOL takeoff item had even
    -- started -- "didn't wait for a transition command". Debounce: only
    -- commit to cruise after in_vtol_mode() has read false for
    -- CRUISE_DEBOUNCE_TICKS in a row, so a single-tick startup race
    -- can't trigger a real tilt.
    local raw_want_hover = (not arming:is_armed()) or quadplane:in_vtol_mode()
    if raw_want_hover then
        cruise_request_ticks = 0
    else
        cruise_request_ticks = cruise_request_ticks + 1
    end
    local want_hover = raw_want_hover or (cruise_request_ticks < CRUISE_DEBOUNCE_TICKS)
    local target_tilt_deg = want_hover and 0.0 or 90.0
    local step_deg = 90.0 / (TILT_SLEW_DURATION_S * UPDATE_HZ)

    if current_tilt_deg < target_tilt_deg then
        current_tilt_deg = math.min(current_tilt_deg + step_deg, target_tilt_deg)
    elseif current_tilt_deg > target_tilt_deg then
        current_tilt_deg = math.max(current_tilt_deg - step_deg, target_tilt_deg)
    end

    -- Live-verified 2026-09-04: k_scripting1 is an ANGLE-type function
    -- (SRV_Channel_aux.cpp's function_is_pwm_or_angle-style switch calls
    -- set_angle(4500) for it, same as k_tiltMotorRear/k_tiltMotorLeft/
    -- k_tiltMotorRight) -- scaled input range is -4500..+4500
    -- (centidegrees), mapped to the full SERVO14_MIN..MAX PWM span, NOT
    -- 0..1000 as first assumed. Confirmed live: sending 0 produced
    -- PWM 1500 (TRIM) instead of the expected "down" endpoint, and 1000
    -- produced ~1611 -- both exactly consistent with the angle-type
    -- +-4500 formula, not the 0..1000 one. -4500 = fully down (matches
    -- SERVO14_MIN, tilt=0deg), +4500 = fully horizontal (matches
    -- SERVO14_MAX, tilt=90deg).
    SRV_Channels:set_output_scaled(REAR_TILT_FUNCTION, (current_tilt_deg / 90.0) * 9000.0 - 4500.0)

    -- Live-caught 2026-09-04: the front pair never tilts (only the rear
    -- does), so tilting the rear forward is the ONLY way this airframe
    -- gains airspeed -- the front motors must keep carrying full hover
    -- lift throughout that tilt. The table swap used to fire the
    -- instant the tilt animation finished (a fixed 3s timer), with no
    -- check that real airspeed existed yet: front lift got cut to zero
    -- while the aircraft was still essentially hovering forward at near
    -- 0 m/s, and it fell -- Q_ASSIST's boost spiked trying to catch a
    -- collapse the table swap itself caused. Gate the swap to cruise on
    -- airspeed in addition to tilt completion, using ARSPD_FBW_MIN as
    -- the flying-speed threshold (falls back to 6.0 m/s if unset). The
    -- tilt itself is unaffected -- it still runs on want_hover alone, so
    -- the rear keeps leaning forward and accelerating the aircraft while
    -- the front pair holds full lift, until the wings can take over.
    --
    -- Live-caught 2026-09-05: airspeed correctly built up (1 m/s ->
    -- 7 m/s over the tilt) and the gate correctly fired at threshold,
    -- but the crash still happened, right at that instant. cruise_factors
    -- zeros roll/pitch/yaw on ALL THREE motors in one tick -- correct
    -- in principle, since forward flight should be attitude-controlled
    -- by the elevons/rudder, not motors, but ArduPilot's own QuadPlane
    -- transition state machine hands that authority to the control
    -- surfaces on its OWN internal schedule, which this script has no
    -- visibility into. Cutting motor RPY authority in one tick can land
    -- before ArduPilot's side actually trusts the surfaces yet, leaving
    -- a gap with no working attitude control at all. Fix: blend
    -- linearly over BLEND_DURATION_S instead of swapping instantly, so
    -- there's no single tick where authority disappears outright --
    -- motor authority fades out as (nominally) the surfaces fade in.
    local airspeed_ms = ahrs:airspeed_EAS() or 0.0
    local cruise_ready = (not want_hover) and current_tilt_deg >= 89.99 and airspeed_ms >= MIN_CRUISE_AIRSPEED_MS

    if cruise_ready then
        cruise_blend = math.min(cruise_blend + BLEND_STEP, 1.0)
    else
        cruise_blend = math.max(cruise_blend - BLEND_STEP, 0.0)
    end
    Motors_dynamic:load_factors(build_blended_factors(cruise_blend))

    return update, 1000 / UPDATE_HZ
end

return update, 1000
