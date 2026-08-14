"""Single source of truth for physical and interface constants."""

MASS_KG = 2.0
G = 9.81

WORLD_NAME = 'urban_canyon'
CONTROL_HZ = 50

TOPIC_WIND_TRUTH = '/aerocanyon/wind_truth'
TOPIC_WIND_EST = '/aerocanyon/wind_estimate'
TOPIC_CBF_DIAG = '/aerocanyon/cbf_diagnostics'
TOPIC_SETPOINT_DESIRED = '/aerocanyon/setpoint_desired'

GZ_WIND_TOPIC = f'/world/{WORLD_NAME}/wind'
