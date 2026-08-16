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

Launcher.start()/stop() publish on Publishers created once, at __init__ --
NOT freshly advertised right before the time-critical first publish. Live-
verified this matters: gz-transport's peer discovery needs a moment after
advertise() to connect to Gazebo's subscriber, and the very first message
on a freshly-advertised topic can be silently dropped before that happens.
A first cut that called advertise()+publish() together at launch time hit
this directly -- one run's measured launch acceleration was ~100x smaller
than intended, consistent with the toss message being lost and only idle-
throttle creep (FW_THR_IDLE) showing up in the telemetry. Advertising in
__init__ instead gives discovery the whole multi-second arm/engage wait to
complete before the toss is ever published.
"""
import numpy as np
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.msgs10.wrench_pb2 import Wrench

LAUNCH_DURATION_S = 0.3  # catapult stroke time -- see the force-vs-impulse note above
LAUNCH_DELTA_V_MS = 10.0  # target forward speed gained during the stroke

# The vehicle spawns sitting on a ramp (see run_trial.SPAWN_XYZ/SPAWN_POSE
# and the static ramp prop in worlds/_template.sdf) rather than flat on the
# ground, nose pitched up by this same angle -- these three numbers are the
# single source of truth all three places derive from; the ramp's SDF pose
# and the spawn pose are hand-computed FROM these values (see the comments
# there) rather than computed at runtime, since the world file is static
# text, not Python.
RAMP_ANGLE_DEG = 12.0
RAMP_LENGTH_M = 2.5
RAMP_RISE_M = RAMP_LENGTH_M * np.sin(np.radians(RAMP_ANGLE_DEG))  # ~0.520


def force_newtons(mass_kg, duration_s=LAUNCH_DURATION_S, delta_v_ms=LAUNCH_DELTA_V_MS):
    """F = m*dv/dt for a constant force over the stroke duration."""
    return mass_kg * delta_v_ms / duration_s


class Launcher:
    """Owns the two ApplyLinkWrench publishers for one model, advertised
    once and reused -- see the module docstring for why that matters."""

    def __init__(self, gz_node, world_name, model_name):
        self.world_name = world_name
        self.model_name = model_name
        self._start_pub = gz_node.advertise(
            f'/world/{world_name}/wrench/persistent', EntityWrench)
        self._stop_pub = gz_node.advertise(f'/world/{world_name}/wrench/clear', Entity)

    def start(self, force_n):
        """Begin a persistent force on the model, angled up the ramp
        (world/ENU +x and +z) rather than pure horizontal -- this
        project's spawn convention has yaw=0 already facing +x (see
        run_trial.SPAWN_POSE), which is also the canyon's own travel
        direction (Mission.direction), so the force needs no further
        body-frame rotation, only the same RAMP_ANGLE_DEG tilt the vehicle
        is spawned pitched up by."""
        angle = np.radians(RAMP_ANGLE_DEG)
        msg = EntityWrench(
            entity=Entity(name=self.model_name, type=Entity.MODEL),
            wrench=Wrench(force=Vector3d(
                x=force_n * float(np.cos(angle)), y=0.0,
                z=force_n * float(np.sin(angle)))))
        self._start_pub.publish(msg)

    def stop(self):
        """Clear the persistent force -- called once LAUNCH_DURATION_S has
        elapsed. Leaving it applied forever would just be a constant
        thruster, not a toss."""
        self._stop_pub.publish(Entity(name=self.model_name, type=Entity.MODEL))
