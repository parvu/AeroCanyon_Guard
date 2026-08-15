"""Single source of truth for physical and interface constants."""

# Sum of every link mass in PX4-Autopilot's tiltrotor model.sdf: base_link
# (5 kg) + 2x motor (0.05 kg each) + 4x rotor (0.005 kg each); the four
# control-surface links are ~0 kg. Previously hardcoded to 2.0, which
# didn't match the actual simulated vehicle at all -- every physics
# calculation that uses this constant (CBF feedforward, the PINN's thrust
# reconstruction and physics residual) was off by more than half.
MASS_KG = 5.12
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
