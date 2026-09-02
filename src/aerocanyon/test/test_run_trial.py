"""run_trial.py orchestrates real subprocesses and a real Gazebo instance,
so it isn't meaningfully unit-testable end-to-end. These tests check the
two pieces of non-trivial logic that can be verified without either:

- the Gazebo teleport request is built with the right service name and
  message fields (a typo there -- wrong field, wrong model name -- would
  silently no-op, which is exactly the failure mode this exists to
  prevent); an earlier version used the entity-remove service instead,
  which was verified live (twice, on independently fresh Gazebo
  instances) to leave every telemetry topic frozen at zero after the
  second SITL boot in a session -- see the module docstring;
- the "did the SITL actually start" check reports failure the moment the
  subprocess has already exited, without needing a real arduplane/Gazebo
  pair.
"""
import subprocess

import pytest
import rclpy

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from mavros_msgs.msg import ExtendedState

from aerocanyon import constants as C
from aerocanyon.run_trial import (SPAWN_XYZ, _LandWatcher, _reset_gazebo_model,
                                  _verify_sitl_started, _wait_for_landing)


def test_reset_gazebo_model_teleports_the_right_entity_to_the_spawn_pose(monkeypatch):
    calls = []

    def fake_request(self, service, request, request_type, response_type, timeout):
        calls.append((service, request, request_type, response_type, timeout))
        return True, Boolean(data=True)

    monkeypatch.setattr('gz.transport13.Node.request', fake_request)
    monkeypatch.setattr('time.sleep', lambda _: None)  # skip the real 1s wait

    _reset_gazebo_model()

    assert len(calls) == 1
    service, request, request_type, response_type, _ = calls[0]
    assert service == f'/world/{C.WORLD_NAME}/set_pose'
    assert request_type is Pose
    assert response_type is Boolean
    assert request.name == C.MODEL_NAME
    x, y, z = SPAWN_XYZ
    assert (request.position.x, request.position.y, request.position.z) == (x, y, z)


def test_verify_sitl_started_passes_when_the_process_is_still_alive(monkeypatch):
    monkeypatch.setattr('time.sleep', lambda _: None)
    proc = subprocess.Popen(['sleep', '30'])
    try:
        _verify_sitl_started(proc, timeout_s=0)  # must not raise
    finally:
        proc.kill()
        proc.wait()


def test_verify_sitl_started_fails_loudly_when_it_exited_immediately(monkeypatch):
    # The actual regression this exists to catch: a manually-started
    # arduplane or mavros_node from another terminal wins the port race,
    # this process's own arduplane dies within moments, and -- before
    # this check existed -- nothing else in the stack ever noticed.
    monkeypatch.setattr('time.sleep', lambda _: None)
    proc = subprocess.Popen(['false'])
    proc.wait()
    with pytest.raises(SystemExit, match='arduplane exited immediately'):
        _verify_sitl_started(proc, timeout_s=0)


def test_land_watcher_ignores_the_landed_at_boot_default():
    # Regression: ExtendedState.landed_state is ON_GROUND by DEFAULT at
    # boot (resting on the ground, motors off, before the vehicle has
    # ever flown). Treating that as "landed" made _wait_for_landing
    # return almost instantly every time -- verified live under the
    # PX4-era equivalent (VehicleLandDetected.landed): nodes (including
    # trial_logger) got killed ~1.4s after starting, before the vehicle
    # had even armed, let alone flown the mission and produced any data.
    rclpy.init(args=[])
    try:
        node = _LandWatcher()
        node._on_land(ExtendedState(
            landed_state=ExtendedState.LANDED_STATE_ON_GROUND))  # the boot-default reading
        assert node.landed is False, (
            'ON_GROUND with no prior airborne reading must not count as '
            'actually landed -- that is just the vehicle sitting on the ground')
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_land_watcher_confirms_landed_after_a_real_airborne_to_landed_transition():
    rclpy.init(args=[])
    try:
        node = _LandWatcher()
        node._on_land(ExtendedState(
            landed_state=ExtendedState.LANDED_STATE_IN_AIR))  # took off
        assert node.landed is False
        node._on_land(ExtendedState(
            landed_state=ExtendedState.LANDED_STATE_ON_GROUND))  # touched back down
        assert node.landed is True
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_wait_for_landing_times_out_without_a_real_sitl_peer():
    # No SITL is running in this test, so extended_state never arrives --
    # exercises the real production code path (no mocking) end to end
    # and confirms it returns False rather than hanging.
    landed = _wait_for_landing(timeout_s=1)
    assert landed is False


def test_wait_for_landing_returns_true_once_a_real_landing_is_observed(monkeypatch):
    # controller_node -- not this function -- is what actually requests
    # QLAND (once it measures having cleared the canyon exit), so this
    # only needs to prove it detects a genuine airborne-then-landed
    # transition and returns promptly rather than waiting out the full
    # timeout.
    class _FakeNode(_LandWatcher):
        def __init__(self):
            super().__init__()
            self._on_land(ExtendedState(
                landed_state=ExtendedState.LANDED_STATE_IN_AIR))
            self._on_land(ExtendedState(
                landed_state=ExtendedState.LANDED_STATE_ON_GROUND))

    monkeypatch.setattr('aerocanyon.run_trial._LandWatcher', _FakeNode)
    landed = _wait_for_landing(timeout_s=30)
    assert landed is True
