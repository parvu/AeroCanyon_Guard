"""Pure-function coverage for dump_mission's WaypointList -> JSON
transform. No rclpy/MAVROS needed -- see dump_mission._dump's own
docstring for why it's factored out."""
from types import SimpleNamespace

from aerocanyon.dump_mission import _dump


def _wp(command, frame, lat, lon, alt, autocontinue=True):
    return SimpleNamespace(command=command, frame=frame, x_lat=lat,
                           y_long=lon, z_alt=alt, autocontinue=autocontinue)


def test_dump_drops_the_seq_zero_home_placeholder():
    waypoints = [
        _wp(16, 0, 0.0, 0.0, 0.0),  # seq 0: home placeholder
        _wp(84, 3, 44.4345, 26.0480, 25.0),
        _wp(85, 3, 44.4350, 26.0495, 0.0),
    ]
    items = _dump(waypoints)
    assert len(items) == 2
    assert items[0]['command'] == 84
    assert items[1]['command'] == 85


def test_dump_produces_plain_json_serializable_types():
    waypoints = [_wp(16, 0, 0.0, 0.0, 0.0), _wp(84, 3, 44.4345, 26.0480, 25.0)]
    items = _dump(waypoints)
    assert items == [{
        'command': 84, 'frame': 3, 'x_lat': 44.4345, 'y_long': 26.0480,
        'z_alt': 25.0, 'autocontinue': True,
    }]
