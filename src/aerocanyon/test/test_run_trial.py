"""run_trial.py orchestrates real subprocesses and a real Gazebo instance,
so it isn't meaningfully unit-testable end-to-end. This checks the one
piece of non-trivial logic that can be verified without either: the
Gazebo entity-removal request is built with the right service name and
the right message fields, since a typo there (wrong entity type, wrong
model name) would silently no-op -- the failure mode this whole function
exists to prevent in the first place.
"""
import pytest

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.model_pb2 import Model
from gz.msgs10.scene_pb2 import Scene

from aerocanyon import constants as C
from aerocanyon.run_trial import _reset_gazebo_model, _wait_for_spawn


def test_reset_gazebo_model_requests_removal_of_the_right_entity(monkeypatch):
    calls = []

    def fake_request(self, service, request, request_type, response_type, timeout):
        calls.append((service, request, request_type, response_type, timeout))
        return True, Boolean(data=True)

    monkeypatch.setattr('gz.transport13.Node.request', fake_request)
    monkeypatch.setattr('time.sleep', lambda _: None)  # skip the real 1s wait

    _reset_gazebo_model()

    assert len(calls) == 1
    service, request, request_type, response_type, _ = calls[0]
    assert service == f'/world/{C.WORLD_NAME}/remove'
    assert request_type is Entity
    assert response_type is Boolean
    assert request.name == C.MODEL_NAME
    assert request.type == Entity.MODEL


def _fake_scene(*names):
    def fake_request(self, service, request, request_type, response_type, timeout):
        return True, Scene(model=[Model(name=n) for n in names])
    return fake_request


def test_wait_for_spawn_returns_once_the_model_appears(monkeypatch):
    # _reset_gazebo_model() removes the entity before every PX4 restart; if
    # nothing then respawns it (e.g. a manually-started PX4/agent from
    # another terminal is blocking this run's own, see run_trial's
    # docstring), the vehicle used to just be silently gone for the whole
    # trial. This is the "it did come back" path.
    monkeypatch.setattr('gz.transport13.Node.request',
                        _fake_scene('ground_plane', C.MODEL_NAME))
    _wait_for_spawn(timeout_s=5)  # must return, not raise


def test_wait_for_spawn_fails_loudly_when_the_model_never_reappears(monkeypatch):
    # The actual regression this exists to catch: nothing in the rest of
    # the stack (PX4, the ROS nodes, run_trial's own CSV-size check) would
    # ever have noticed a missing vehicle before this.
    monkeypatch.setattr('gz.transport13.Node.request', _fake_scene('ground_plane'))
    monkeypatch.setattr('time.sleep', lambda _: None)
    with pytest.raises(SystemExit, match=C.MODEL_NAME):
        _wait_for_spawn(timeout_s=0.2)
