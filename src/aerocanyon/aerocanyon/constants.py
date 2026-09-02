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
# Matches PX4's own naming: <PX4_SIM_MODEL with the gz_ prefix stripped>_<instance>.
# Instance is always 0 here -- nothing in this project runs multiple vehicles.
MODEL_NAME = 'tricopter_0'
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
POSITION_KP = 0.5   # m/s^2 per metre of position error
POSITION_KD = 0.8   # m/s^2 per m/s of velocity (damping)
ALTITUDE_KP = 0.6   # climb-rate command [-1,1] per metre of altitude error
HEADING_KP = 1.0    # yaw-rate command [-1,1] per radian of heading error
MAX_LEAN_RAD = math.radians(20.0)  # lean angle that saturates the RC stick
