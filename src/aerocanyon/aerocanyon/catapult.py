"""Simulates the catapult toss the wing-only vehicle needs to get flying.

PX4's own launch-detection module (FW_LAUN_DETCN_ON, LaunchDetector) only
runs inside the AUTO_MISSION takeoff flight task -- traced directly in
FixedWingModeManager.cpp: when flag_control_offboard_enabled is set (this
project's architecture for every vehicle, all branches), the incoming
trajectory setpoint is converted straight to SETPOINT_TYPE_POSITION, which
never reaches the FW_POSCTRL_MODE_AUTO_TAKEOFF switch case where the launch
detector lives. Building a parallel AUTO_MISSION mission-upload path just to
reach that one module would be a large, disconnected piece of
infrastructure for what this project actually needs. So "catapult launch"
here means what it physically is: give the vehicle a real velocity kick,
then fly it in OFFBOARD from the first tick, same as every other vehicle.

There is no Gazebo catapult plugin either. The toss is a real physical
force applied to the airframe via gz-sim-apply-link-wrench-system (see
worlds/_template.sdf), over a short duration rather than a single-timestep
impulse -- a single-step force big enough to move a ~5kg airframe by
~10 m/s in one 1ms physics step would be a numerically extreme impulse
(tens of thousands of newtons) with a real risk of blowing up the physics
engine or clipping through the ground; sustaining a much smaller, sane
force over a fraction of a second integrates to the same delta-v without
either problem.

ApplyLinkWrench is TOPIC-based (publish/subscribe), not a request/response
service like the Pose set_pose calls elsewhere in this project (run_trial's
_reset_gazebo_model) -- gz's own usage example publishes onto
/world/<world>/wrench/persistent and /wrench/clear with plain `gz topic -t`,
not `gz service -s`.
"""
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.msgs10.wrench_pb2 import Wrench

LAUNCH_DURATION_S = 0.3  # catapult stroke time -- see the force-vs-impulse note above
LAUNCH_DELTA_V_MS = 10.0  # target forward speed gained during the stroke


def force_newtons(mass_kg, duration_s=LAUNCH_DURATION_S, delta_v_ms=LAUNCH_DELTA_V_MS):
    """F = m*dv/dt for a constant force over the stroke duration."""
    return mass_kg * delta_v_ms / duration_s


def start(gz_node, world_name, model_name, force_n):
    """Begin a persistent forward (world/ENU +x) force on the model -- this
    project's spawn convention has yaw=0 already facing +x (see
    run_trial.SPAWN_POSE), which is also the canyon's own travel direction
    (Mission.direction), so a pure +x force needs no body-frame rotation.
    Returns the Publisher -- keep it alive (garbage collection tears down
    the advertisement) until stop() has been called."""
    pub = gz_node.advertise(f'/world/{world_name}/wrench/persistent', EntityWrench)
    msg = EntityWrench(
        entity=Entity(name=model_name, type=Entity.MODEL),
        wrench=Wrench(force=Vector3d(x=force_n, y=0.0, z=0.0)))
    pub.publish(msg)
    return pub


def stop(gz_node, world_name, model_name):
    """Clear the persistent force -- called once LAUNCH_DURATION_S has
    elapsed. Leaving it applied forever would just be a constant thruster,
    not a toss. Returns the Publisher; same lifetime note as start()."""
    pub = gz_node.advertise(f'/world/{world_name}/wrench/clear', Entity)
    pub.publish(Entity(name=model_name, type=Entity.MODEL))
    return pub
