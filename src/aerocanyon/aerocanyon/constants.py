"""Single source of truth for physical and interface constants."""
import math

# Sum of every link mass in PX4-Autopilot's tricopter model.sdf: base_link
# (5 kg) + 2x tilting motor (0.05 kg each) + 3x rotor (0.005 kg each); the
# four control-surface links are ~0 kg. Was 5.12 on the tiltrotor, which
# carried a fourth rotor. (It was 2.0 before that, matching nothing at all --
# every physics calculation using this constant, the CBF feedforward and the
# PINN's thrust reconstruction and physics residual, was off by more than
# half.)
MASS_KG = 5.115
G = 9.81

WORLD_NAME = 'urban_canyon'
# The Gazebo entity name for an <include> with no explicit <name> defaults
# to the model directory's own name -- src/aerocanyon/models/tricopter_ap.
# (Was 'tricopter_0', matching PX4's gz_bridge naming convention --
# <PX4_SIM_MODEL with the gz_ prefix stripped>_<instance> -- back when
# PX4 spawned the vehicle itself via a gz service call rather than a
# static <include> in the world file.)
MODEL_NAME = 'tricopter_ap'
CONTROL_HZ = 50

TOPIC_WIND_TRUTH = '/aerocanyon/wind_truth'
TOPIC_WIND_EST = '/aerocanyon/wind_estimate'
TOPIC_CBF_DIAG = '/aerocanyon/cbf_diagnostics'
TOPIC_SETPOINT_DESIRED = '/aerocanyon/setpoint_desired'

GZ_WIND_TOPIC = f'/world/{WORLD_NAME}/wind'

# controller_node's new outer-loop gains (Phase 2 MAVROS port -- PX4's
# own position controller used to make these unnecessary, since it
# accepted a position setpoint directly; ArduPilot exposes no equivalent
# injection path for this airframe in any flight mode). Starting points,
# not yet live-tuned -- see docs/superpowers/plans/
# 2026-09-02-mission-stack-mavros-port.md Task 8 for the verification
# this needs before being trusted.
#
# Task 8 live-verified (2026-09-02): a 60s baseline leg armed, flew, and
# logged real continuously-varying telemetry throughout (no NaNs, no
# frozen/zero readings except a normal ~0.3s tail at teardown) -- the
# pipeline itself works end to end.
#
# The initial "just untuned" read turned out to be wrong -- checked by
# plotting the resulting trajectory and reading the logged data
# directly, not by assumption. What actually needed fixing:
#
# HEADING_KP mapped angle error directly, at gain 1.0, into QHOVER's
# yaw-RATE channel with no damping -- a proportional-only controller
# driving a rate (not position) actuator, which saturates and
# overshoots on any real error. Confirmed live: logged yaw swung
# 90 -> 98 -> 110 -> -171 -> 122 -> 25 -> -169 degrees within a few
# seconds, an actual spin, not gentle imprecision -- and since the
# position loop's lean commands are rotated into the body frame using
# that same (garbage) yaw reading every tick, the whole trajectory went
# nowhere useful regardless of how correct the position math itself was.
# Dropped by more than 3x below as a first mitigation, not a proper
# tuning pass (a real fix likely needs a rate-damping term, not just a
# lower gain on the same P-only law).
#
# A SEPARATE hypothesis (that self.pos needed a spawn-relative-to-
# absolute offset added before comparing against mission.target()) was
# tried, live-tested, and DISPROVEN: querying LOCAL_POSITION_NED
# directly at spawn (unarmed, no control input) showed self.pos is
# already in the same absolute canyon-frame as mission.target() --
# reads CANYON_ENTRY's own NED value there, not (0,0,0). The offset
# code was reverted (controller_node.py); see that commit for the full
# account of how a placeholder-zero CSV row before real telemetry
# arrived was mistaken for a genuine spawn-time reading.
#
# With the heading fix alone: a fresh 70s baseline leg holds a stable
# heading (~85-100 degrees throughout, no oscillation) and tracks the
# mission target smoothly. Still not a real tuning pass -- re-verify
# live before trusting a longer flight, and before sub-project 3's
# 49-trial sweep runs on top of this.
POSITION_KP = 0.5   # m/s^2 per metre of position error
POSITION_KD = 0.8   # m/s^2 per m/s of velocity (damping)
ALTITUDE_KP = 0.6   # climb-rate command [-1,1] per metre of altitude error
HEADING_KP = 0.3    # yaw-rate command [-1,1] per radian of heading error -- was 1.0, oscillated
MAX_LEAN_RAD = math.radians(20.0)  # lean angle that saturates the RC stick
