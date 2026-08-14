"""Single source of truth for physical and interface constants."""

MASS_KG = 2.0
G = 9.81

WORLD_NAME = 'urban_canyon'
# Matches PX4's own naming: <PX4_SIM_MODEL with the gz_ prefix stripped>_<instance>.
# Instance is always 0 here -- nothing in this project runs multiple vehicles.
MODEL_NAME = 'tiltrotor_0'
CONTROL_HZ = 50

TOPIC_WIND_TRUTH = '/aerocanyon/wind_truth'
TOPIC_WIND_EST = '/aerocanyon/wind_estimate'
TOPIC_CBF_DIAG = '/aerocanyon/cbf_diagnostics'
TOPIC_SETPOINT_DESIRED = '/aerocanyon/setpoint_desired'

GZ_WIND_TOPIC = f'/world/{WORLD_NAME}/wind'
