import numpy as np
import pytest

from aerocanyon import catapult


def test_force_newtons_is_mass_times_delta_v_over_duration():
    f = catapult.force_newtons(5.0, duration_s=0.5, delta_v_ms=10.0)
    assert f == pytest.approx(5.0 * 10.0 / 0.5)


class _FakePublisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _FakeGzNode:
    def __init__(self):
        self.advertised = []

    def advertise(self, topic, msg_type):
        self.advertised.append((topic, msg_type))
        return _FakePublisher()


def test_launcher_advertises_both_publishers_at_construction():
    # Live-verified bug this guards against: advertising right before the
    # first publish hits a gz-transport discovery race that can silently
    # drop the toss (measured ~100x smaller acceleration than intended in
    # one run). Advertising must happen at __init__, not lazily inside
    # start()/stop(), so discovery has the whole arm/engage wait to settle.
    gz = _FakeGzNode()
    catapult.Launcher(gz, 'test_world', 'test_model')
    topics = [t for t, _ in gz.advertised]
    assert '/world/test_world/wrench/persistent' in topics
    assert '/world/test_world/wrench/clear' in topics


def test_start_angles_the_force_up_the_ramp_not_purely_horizontal():
    # The vehicle spawns pitched up by RAMP_ANGLE_DEG (see run_trial.py /
    # worlds/_template.sdf's catapult_ramp) -- the toss must follow the same
    # angle, not push it flat along the ground out from under itself.
    launcher = catapult.Launcher(_FakeGzNode(), 'test_world', 'test_model')
    launcher.start(force_n=100.0)
    msg = launcher._start_pub.published[0]
    angle = np.radians(catapult.RAMP_ANGLE_DEG)
    assert msg.wrench.force.x == pytest.approx(100.0 * np.cos(angle))
    assert msg.wrench.force.z == pytest.approx(100.0 * np.sin(angle))
    assert msg.wrench.force.y == pytest.approx(0.0)
    # x^2 + z^2 must still equal the requested magnitude -- angling the
    # force must not silently change how hard the toss actually is.
    mag = np.hypot(msg.wrench.force.x, msg.wrench.force.z)
    assert mag == pytest.approx(100.0)
    assert msg.entity.name == 'test_model'


def test_stop_targets_the_named_entity():
    launcher = catapult.Launcher(_FakeGzNode(), 'test_world', 'test_model')
    launcher.stop()
    assert launcher._stop_pub.published[0].name == 'test_model'


def test_ramp_rise_matches_length_and_angle():
    # RAMP_RISE_M is what run_trial.py and worlds/_template.sdf's
    # catapult_ramp pose are hand-computed from -- if this drifts out of
    # sync with RAMP_LENGTH_M/RAMP_ANGLE_DEG, the vehicle spawns floating
    # above or clipped into the ramp.
    expected = catapult.RAMP_LENGTH_M * np.sin(np.radians(catapult.RAMP_ANGLE_DEG))
    assert catapult.RAMP_RISE_M == pytest.approx(expected)


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-q']))
