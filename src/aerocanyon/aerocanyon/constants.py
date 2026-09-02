"""Single source of truth for physical and interface constants."""

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
TOPIC_WIND_SPEED_SCALE = '/aerocanyon/wind_speed_scale'

GZ_WIND_TOPIC = f'/world/{WORLD_NAME}/wind'

# controller_node's earlier outer-loop gains (POSITION_KP/KD,
# ALTITUDE_KP, HEADING_KP, MAX_LEAN_RAD -- an RC-override-based P/D
# position/altitude/heading controller, Phase 2's first MAVROS-port
# attempt) are GONE, not just untuned: that whole approach was replaced
# after a live demo watched it drift laterally under wind and strike a
# canyon tower. See docs/superpowers/specs/
# 2026-09-02-auto-mission-navigation-design.md -- controller_node now
# uploads an ArduPilot AUTO mission and lets ArduPilot's own navigation
# controller fly it, instead of a hand-rolled loop computing RC-override
# lean angles every tick.
MAX_WAYPOINT_OFFSET_M = 3.0  # clamp on treatment's cumulative waypoint nudge
CORRECTION_UPDATE_HZ = 1.0   # how often treatment re-pushes the cruise waypoint
