"""run_trial.py orchestrates real subprocesses and a real Gazebo instance,
so it isn't meaningfully unit-testable end-to-end. These tests check the
two pieces of non-trivial logic that can be verified without either:

- the Gazebo teleport request is built with the right service name and
  message fields (a typo there -- wrong field, wrong model name -- would
  silently no-op, which is exactly the failure mode this exists to
  prevent); an earlier version used the entity-remove service instead,
  which was verified live (twice, on independently fresh Gazebo
  instances) to leave every /fmu/out telemetry topic frozen at zero after
  the second PX4 boot in a session -- see the module docstring;
- the "did PX4 actually start" check reports failure the moment the
  subprocess has already exited, without needing a real PX4/Gazebo pair.
"""
import subprocess

import pytest

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose

from aerocanyon import constants as C
from aerocanyon.run_trial import SPAWN_XYZ, _reset_gazebo_model, _verify_px4_started


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


def test_verify_px4_started_passes_when_the_process_is_still_alive(monkeypatch):
    monkeypatch.setattr('time.sleep', lambda _: None)
    proc = subprocess.Popen(['sleep', '30'])
    try:
        _verify_px4_started(proc, timeout_s=0)  # must not raise
    finally:
        proc.kill()
        proc.wait()


def test_verify_px4_started_fails_loudly_when_px4_exited_immediately(monkeypatch):
    # The actual regression this exists to catch: a manually-started PX4
    # or agent from another terminal wins the instance-0/port-8888 race,
    # this process's own PX4 dies within moments, and -- before this
    # check existed -- nothing else in the stack ever noticed.
    monkeypatch.setattr('time.sleep', lambda _: None)
    proc = subprocess.Popen(['false'])
    proc.wait()
    with pytest.raises(SystemExit, match='PX4 exited immediately'):
        _verify_px4_started(proc, timeout_s=0)
