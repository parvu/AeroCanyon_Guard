-- Dynamic scripting motor mixer for the tricopter-ap VTOL.
--
-- REDESIGNED 2026-09-06 (pilot's call: "disable script and let ardupilot
-- do the transition"). This script no longer manages the FW transition
-- at all -- ArduPilot's own native Tiltrotor/QuadPlane logic now owns
-- tilt scheduling and the hover->cruise handoff entirely (SERVO12/13
-- reverted to k_tiltMotorRight/Left in tricopter.parm; the front pair
-- becomes the cruise thruster, ArduPilot's standard tiltrotor
-- assumption). The rear motor's own tilt mechanism is no longer used
-- for anything -- it stays a plain, non-tilting VTOL lift motor, pinned
-- vertical below.
--
-- The ONLY reason this script still exists at all: Q_FRAME_CLASS stays
-- 17 (Dynamic Scripting Matrix), not reverted to 7 (Tri), because
-- AP_MotorsTri hardcodes pitch factors for a 2:1 rear-to-front arm
-- length ratio and this airframe's arms are equal length (confirmed
-- against model.sdf: motor_0/motor_2 at x=+0.35m, motor_1 at x=-0.35m)
-- -- using AP_MotorsTri's real factors caused a confirmed hover pitch-
-- runaway crash earlier this project (see HOVER_PITCH's comment below).
-- So this script's whole job is now: register the 3 motors, load ONE
-- static, correctly-balanced factor table, pin the rear tilt vertical,
-- and quit -- no per-tick update() callback, no transition state
-- machine, no tilt slewing, no airspeed/blend logic. ArduPilot's own
-- still-active motors_output() keeps using this table for as long as
-- the vehicle is in a VTOL-authority mode; once genuinely out of VTOL
-- mode it freezes whatever PWM each channel last held (State::DONE),
-- same as any other tiltrotor.

local MOTOR_FRONT_RIGHT = 0
local MOTOR_REAR = 1
local MOTOR_FRONT_LEFT = 2
local MOTOR_IDS = {MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT, MOTOR_REAR}

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
-- no analogous bug possible there. Yaw is left to ArduPilot's native
-- Q_TILT_TYPE=2 VectoredYaw mixing on the front tilt servos now that
-- they're native-owned again (2026-09-06), so all yaw factors here stay
-- zero, same as before.
local ROLL  = {[MOTOR_FRONT_RIGHT]=-0.5, [MOTOR_FRONT_LEFT]=0.5, [MOTOR_REAR]=0.0}
local PITCH = {[MOTOR_FRONT_RIGHT]=0.5,  [MOTOR_FRONT_LEFT]=0.5, [MOTOR_REAR]=-1.0}
local YAW   = {[MOTOR_FRONT_RIGHT]=0.0,  [MOTOR_FRONT_LEFT]=0.0, [MOTOR_REAR]=0.0}
local THR   = {[MOTOR_FRONT_RIGHT]=1.0,  [MOTOR_FRONT_LEFT]=1.0, [MOTOR_REAR]=1.0}

local function build_factors()
    local t = motor_factor_table()
    for _, m in ipairs(MOTOR_IDS) do
        t:roll(m, ROLL[m])
        t:pitch(m, PITCH[m])
        t:yaw(m, YAW[m])
        t:throttle(m, THR[m])
    end
    return t
end

Motors_dynamic:add_motor(MOTOR_FRONT_RIGHT, 1)
Motors_dynamic:add_motor(MOTOR_REAR, 2)
Motors_dynamic:add_motor(MOTOR_FRONT_LEFT, 3)
Motors_dynamic:load_factors(build_factors())
assert(Motors_dynamic:init(3), "tricopter_mixer: failed to init Motors_dynamic")
motors:set_frame_string("tricopter scripting mixer (native transition)")

-- Rear tilt (SERVO14, k_scripting1): pinned vertical once and never
-- touched again -- the rear no longer tilts for anything, see this
-- file's header comment. -4500 = fully down/vertical, matches
-- SERVO14_MIN (tricopter.parm) -- same ANGLE-type k_scripting
-- convention and PWM mapping verified live 2026-09-04.
local REAR_TILT_FUNCTION = 94  -- k_scripting1, see SERVO14_FUNCTION in tricopter.parm
SRV_Channels:set_output_scaled(REAR_TILT_FUNCTION, -4500.0)
