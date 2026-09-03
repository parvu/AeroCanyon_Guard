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
import json
import subprocess
import time

import pytest
import rclpy

from geometry_msgs.msg import PoseStamped
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from mavros_msgs.msg import ExtendedState

from aerocanyon import canyon_geometry as cg
from aerocanyon import constants as C
from aerocanyon import run_trial as rt
from aerocanyon.run_trial import (MAP_ZONE_SPAWN_XYZ, MAX_STALL_RETRIES,
                                  SPAWN_XYZ, _LandWatcher, _LegStalled,
                                  _reset_gazebo_model, _spawn_xyz, _world_sdf,
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


def test_world_sdf_matches_the_file_on_disk_for_urban_canyon():
    assert _world_sdf('urban_canyon').name == 'urban_canyon.sdf'


def test_world_sdf_maps_map_zone_to_its_actual_filename():
    """Live-caught: gz sim exited immediately (255, "Unable to find or
    download file") because _world_sdf('map_zone') built
    worlds/map_zone.sdf -- the real file is map_zone_ap.sdf, even though
    the world's own internal name (<world name="map_zone">, what the gz
    services key off) is just "map_zone"."""
    assert _world_sdf('map_zone').name == 'map_zone_ap.sdf'


def test_spawn_xyz_is_unchanged_for_urban_canyon():
    assert _spawn_xyz('urban_canyon', '') == SPAWN_XYZ


def test_spawn_xyz_uses_the_documented_default_for_map_zone_with_no_mission():
    assert _spawn_xyz('map_zone', '') == MAP_ZONE_SPAWN_XYZ


def test_spawn_xyz_uses_the_mission_files_first_waypoint(tmp_path):
    mission_file = tmp_path / 'mission.json'
    mission_file.write_text(json.dumps([
        {'command': 84, 'frame': 3, 'x_lat': 44.4345, 'y_long': 26.0480,
         'z_alt': 25.0, 'autocontinue': True},
        {'command': 85, 'frame': 3, 'x_lat': 44.4350, 'y_long': 26.0495,
         'z_alt': 0.0, 'autocontinue': True},
    ]))
    x, y, z = _spawn_xyz('map_zone', str(mission_file))
    expected_north, expected_east = rt.frames.latlon_to_ned(
        44.4345, 26.0480, rt.HOME_LAT, rt.HOME_LON)
    assert x == pytest.approx(expected_east)
    assert y == pytest.approx(expected_north)
    assert z == pytest.approx(cg.GROUND_Z + 1.2)


def test_spawn_xyz_skips_a_takeoff_items_zeroed_coordinates(tmp_path):
    """Live-caught: Mission Planner's own NAV_VTOL_TAKEOFF item (command
    84) carries x_lat=y_long=0.0 -- that command takes off in place and
    never reads its own lat/lon. Spawning at literal (0, 0) would put the
    vehicle off the coast of Africa instead of near the mission."""
    mission_file = tmp_path / 'mission.json'
    mission_file.write_text(json.dumps([
        {'command': 84, 'frame': 3, 'x_lat': 0.0, 'y_long': 0.0,
         'z_alt': 20.0, 'autocontinue': True},
        {'command': 16, 'frame': 3, 'x_lat': 44.434464, 'y_long': 26.0505104,
         'z_alt': 20.0, 'autocontinue': True},
    ]))
    x, y, z = _spawn_xyz('map_zone', str(mission_file))
    expected_north, expected_east = rt.frames.latlon_to_ned(
        44.434464, 26.0505104, rt.HOME_LAT, rt.HOME_LON)
    assert x == pytest.approx(expected_east)
    assert y == pytest.approx(expected_north)


def test_reset_gazebo_model_uses_the_given_world_and_spawn_point(monkeypatch):
    calls = []

    def fake_request(self, service, request, request_type, response_type, timeout):
        calls.append((service, request))
        return True, Boolean(data=True)

    monkeypatch.setattr('gz.transport13.Node.request', fake_request)
    monkeypatch.setattr('time.sleep', lambda _: None)

    _reset_gazebo_model(world='map_zone', spawn_xyz=(1.0, 2.0, 3.0))

    assert len(calls) == 1
    service, request = calls[0]
    assert service == '/world/map_zone/set_pose'
    assert request.position.x == 1.0 and request.position.z == 3.0


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
    # and confirms it returns 'timeout' rather than hanging. (Not
    # 'stalled': pose_seen never goes True either, since nothing is
    # publishing pose -- the stall check only fires once a real pose
    # stream has been seen and then goes quiet, see the next tests.)
    status = _wait_for_landing(timeout_s=1)
    assert status == 'timeout'


def test_wait_for_landing_returns_landed_once_a_real_landing_is_observed(monkeypatch):
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
    status = _wait_for_landing(timeout_s=30)
    assert status == 'landed'


def _pose_at(x, y, z):
    msg = PoseStamped()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
    return msg


def test_on_pose_does_not_refresh_the_change_clock_for_a_repeated_position():
    # The actual bug this whole mechanism was built to catch, on the
    # first attempt: MAVROS keeps publishing /mavros/local_position/pose
    # at its normal ~20ms cadence straight through a Gazebo<->ArduPilot
    # JSON-FDM stall -- just with the last-known, UNCHANGING position
    # (ArduPilot's own MAVLink telemetry doesn't stop just because its
    # FDM input did). A message-arrival-timing check never saw a gap and
    # never fired; only comparing the VALUE catches this.
    rclpy.init(args=[])
    try:
        node = _LandWatcher()
        node._on_pose(_pose_at(1.0, 2.0, 3.0))
        first_change = node.last_pose_change_monotonic
        time.sleep(0.05)
        node._on_pose(_pose_at(1.0, 2.0, 3.0))  # same position, "new" message
        assert node.last_pose_change_monotonic == first_change, (
            'a repeated position must not look like fresh movement')
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_on_pose_refreshes_the_change_clock_for_real_movement():
    rclpy.init(args=[])
    try:
        node = _LandWatcher()
        node._on_pose(_pose_at(1.0, 2.0, 3.0))
        first_change = node.last_pose_change_monotonic
        time.sleep(0.05)
        node._on_pose(_pose_at(1.5, 2.0, 3.0))  # actually moved
        assert node.last_pose_change_monotonic > first_change
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_wait_for_landing_detects_a_stalled_pose_topic(monkeypatch):
    # End-to-end version of the two tests above: a pose topic that keeps
    # arriving (see _wait_for_landing's own loop, which spins the node
    # normally) but whose value never changes must be caught well before
    # the full timeout, not waited out.
    class _FakeNode(_LandWatcher):
        def __init__(self):
            super().__init__()
            self._on_pose(_pose_at(5.0, 5.0, 5.0))
            self.last_pose_change_monotonic -= 999  # simulate a long-frozen value

    monkeypatch.setattr('aerocanyon.run_trial._LandWatcher', _FakeNode)
    start = time.monotonic()
    status = _wait_for_landing(timeout_s=30, stall_timeout_s=1.0)
    elapsed = time.monotonic() - start
    assert status == 'stalled'
    assert elapsed < 5.0, 'must catch the stall promptly, not wait out the full timeout'


def test_wait_for_landing_does_not_false_trigger_on_a_live_pose_stream(monkeypatch):
    # A pose stream that's actually moving must not be mistaken for a
    # stall, even with a short stall_timeout_s and a landing that never
    # arrives.
    class _FakeNode(_LandWatcher):
        def __init__(self):
            super().__init__()
            self._on_pose(_pose_at(5.0, 5.0, 5.0))  # change clock is fresh

    monkeypatch.setattr('aerocanyon.run_trial._LandWatcher', _FakeNode)
    status = _wait_for_landing(timeout_s=1, stall_timeout_s=30.0)
    assert status == 'timeout'


def test_run_leg_retries_a_stalled_leg_from_a_fresh_gz_sim(monkeypatch):
    # A stall is on Gazebo's side of the JSON-FDM link and never
    # self-heals in place (live-confirmed) -- run_leg must respawn `gz
    # sim` and try again rather than accepting the stalled result.
    monkeypatch.setattr(rt, '_spawn_gazebo', lambda world: 'gz-proc')
    monkeypatch.setattr(rt, '_spawn_web_bridge', lambda: 'bridge-proc')
    monkeypatch.setattr(rt, '_kill', lambda proc: None)
    monkeypatch.setattr(rt.subprocess, 'run', lambda *a, **k: None)
    monkeypatch.setattr(rt.time, 'sleep', lambda _: None)

    calls = []

    def fake_run_one(mode, trial, duration, seed=0, turbulence=2.5, ff_gain=0.2,
                     world=C.WORLD_NAME, mission_file=''):
        calls.append(1)
        if len(calls) < MAX_STALL_RETRIES + 1:
            raise _LegStalled('fake stall')
        return 'a-real-csv-path'

    monkeypatch.setattr(rt, 'run_one', fake_run_one)
    result = rt.run_leg('baseline', 'trial', 30)
    assert result == 'a-real-csv-path'
    assert len(calls) == MAX_STALL_RETRIES + 1, (
        'must have retried on every stall up to the last attempt, which succeeded')


def test_run_leg_gives_up_after_max_stall_retries(monkeypatch):
    monkeypatch.setattr(rt, '_spawn_gazebo', lambda world: 'gz-proc')
    monkeypatch.setattr(rt, '_spawn_web_bridge', lambda: 'bridge-proc')
    monkeypatch.setattr(rt, '_kill', lambda proc: None)
    monkeypatch.setattr(rt.subprocess, 'run', lambda *a, **k: None)
    monkeypatch.setattr(rt.time, 'sleep', lambda _: None)
    monkeypatch.setattr(rt, 'run_one',
                        lambda *a, **k: (_ for _ in ()).throw(_LegStalled('fake stall')))

    with pytest.raises(SystemExit, match='giving up'):
        rt.run_leg('baseline', 'trial', 30)
