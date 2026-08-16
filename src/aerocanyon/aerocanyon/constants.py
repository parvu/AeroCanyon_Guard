"""Single source of truth for physical and interface constants."""

# Sum of every link mass in PX4-Autopilot's wingonly model.sdf: base_link
# (5 kg) + the single rear pusher's rotor link (0.005 kg); the four
# control-surface links are ~0 kg. Was 5.115 on the tricopter, which carried
# two tilting-motor housings (0.05 kg each) and one extra rotor (0.005 kg)
# that this airframe doesn't have.
MASS_KG = 5.005
G = 9.81

WORLD_NAME = 'urban_canyon'
# Matches PX4's own naming: <PX4_SIM_MODEL with the gz_ prefix stripped>_<instance>.
# Instance is always 0 here -- nothing in this project runs multiple vehicles.
MODEL_NAME = 'wingonly_0'
CONTROL_HZ = 50

TOPIC_WIND_TRUTH = '/aerocanyon/wind_truth'
TOPIC_WIND_EST = '/aerocanyon/wind_estimate'
TOPIC_CBF_DIAG = '/aerocanyon/cbf_diagnostics'
TOPIC_SETPOINT_DESIRED = '/aerocanyon/setpoint_desired'

GZ_WIND_TOPIC = f'/world/{WORLD_NAME}/wind'
