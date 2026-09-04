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

-- Roll/pitch factors here match AP_MotorsTri's own real, tested values
-- (get_roll_factor()/get_pitch_factor_json() in AP_MotorsTri.cpp), NOT
-- arbitrary +-1.0 guesses. Live-caught 2026-09-04: an earlier version of
-- this table used pitch=+-1.0 for the front pair (matching rear 1:1
-- each), doubling the real front pitch authority vs AP_MotorsTri's own
-- 0.5 each -- caused an immediate pitching moment on throttle-up before
-- any real attitude error existed to correct, compounding into a roll
-- tip-over within ~3s of arming.
local hover_factors = motor_factor_table()
hover_factors:roll(MOTOR_FRONT_RIGHT, -1.0)
hover_factors:pitch(MOTOR_FRONT_RIGHT, 0.5)
hover_factors:yaw(MOTOR_FRONT_RIGHT, 0.0)
hover_factors:throttle(MOTOR_FRONT_RIGHT, 1.0)

hover_factors:roll(MOTOR_FRONT_LEFT, 1.0)
hover_factors:pitch(MOTOR_FRONT_LEFT, 0.5)
hover_factors:yaw(MOTOR_FRONT_LEFT, 0.0)
hover_factors:throttle(MOTOR_FRONT_LEFT, 1.0)

hover_factors:roll(MOTOR_REAR, 0.0)
hover_factors:pitch(MOTOR_REAR, -1.0)
hover_factors:yaw(MOTOR_REAR, 0.0)
hover_factors:throttle(MOTOR_REAR, 1.0)

local cruise_factors = motor_factor_table()
cruise_factors:roll(MOTOR_FRONT_RIGHT, 0.0)
cruise_factors:pitch(MOTOR_FRONT_RIGHT, 0.0)
cruise_factors:yaw(MOTOR_FRONT_RIGHT, 0.0)
cruise_factors:throttle(MOTOR_FRONT_RIGHT, 0.0)

cruise_factors:roll(MOTOR_FRONT_LEFT, 0.0)
cruise_factors:pitch(MOTOR_FRONT_LEFT, 0.0)
cruise_factors:yaw(MOTOR_FRONT_LEFT, 0.0)
cruise_factors:throttle(MOTOR_FRONT_LEFT, 0.0)

cruise_factors:roll(MOTOR_REAR, 0.0)
cruise_factors:pitch(MOTOR_REAR, 0.0)
cruise_factors:yaw(MOTOR_REAR, 0.0)
cruise_factors:throttle(MOTOR_REAR, 1.0)

Motors_dynamic:add_motor(MOTOR_FRONT_RIGHT, 1)
Motors_dynamic:add_motor(MOTOR_REAR, 2)
Motors_dynamic:add_motor(MOTOR_FRONT_LEFT, 3)
Motors_dynamic:load_factors(hover_factors)
assert(Motors_dynamic:init(3), "tricopter_mixer: failed to init Motors_dynamic")
motors:set_frame_string("tricopter scripting mixer")

local current_tilt_deg = 0.0
local in_hover_table = true

local function update()
    local want_hover = quadplane:in_vtol_mode()
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

    if want_hover and current_tilt_deg <= 0.01 and not in_hover_table then
        Motors_dynamic:load_factors(hover_factors)
        in_hover_table = true
    elseif not want_hover and current_tilt_deg >= 89.99 and in_hover_table then
        Motors_dynamic:load_factors(cruise_factors)
        in_hover_table = false
    end

    return update, 1000 / UPDATE_HZ
end

return update, 1000
