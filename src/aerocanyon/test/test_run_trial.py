"""run_trial.py orchestrates real subprocesses and a real Gazebo instance,
so it isn't meaningfully unit-testable end-to-end. This checks the one
piece of non-trivial logic that can be verified without either: the
Gazebo entity-removal request is built with the right service name and
the right message fields, since a typo there (wrong entity type, wrong
model name) would silently no-op -- the failure mode this whole function
exists to prevent in the first place.
"""
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_pb2 import Entity

from aerocanyon import constants as C
from aerocanyon.run_trial import _reset_gazebo_model


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
