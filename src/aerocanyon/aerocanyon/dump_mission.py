"""One-shot capture of the FCU's currently uploaded mission (e.g. drawn
and uploaded live in Mission Planner against a manually-flown map_zone
SITL session -- see README) to a JSON file run_trial/controller_node
replay on every trial leg. Each leg boots a completely fresh, wiped
ArduPilot SITL (run_trial.run_leg), so a live-drawn mission would
otherwise be lost the moment that manual session ends.

Usage (with the manual-flight stack from the README already running,
Mission Planner connected to its gcs_url port and a mission uploaded):
    python3 -m aerocanyon.dump_mission <name>
writes data/missions/<name>.json.
"""
import argparse
import json
import pathlib
import sys
import time

import rclpy
from mavros_msgs.msg import WaypointList
from mavros_msgs.srv import WaypointPull
from rclpy.node import Node

MISSIONS_DIR = pathlib.Path(__file__).resolve().parents[3] / 'data' / 'missions'


def _dump(waypoints):
    """WaypointList.waypoints -> the plain JSON-serializable list
    controller_node._build_map_zone_mission replays from. Drops index 0
    -- ArduPilot always owns/overwrites the seq-0 home placeholder (see
    controller_node._build_urban_canyon_mission's own docstring), so
    capturing its content would be meaningless."""
    return [
        {'command': int(w.command), 'frame': int(w.frame),
         'x_lat': float(w.x_lat), 'y_long': float(w.y_long),
         'z_alt': float(w.z_alt), 'autocontinue': bool(w.autocontinue)}
        for w in waypoints[1:]
    ]


class _MissionDumper(Node):
    def __init__(self):
        super().__init__('mission_dumper')
        self.waypoints = None
        self.create_subscription(
            WaypointList, '/mavros/mission/waypoints', self._on_waypoints, 10)
        self.pull_client = self.create_client(WaypointPull, '/mavros/mission/pull')

    def _on_waypoints(self, msg):
        self.waypoints = msg.waypoints

    def request_pull(self):
        """MAVROS only republishes /mavros/mission/waypoints when
        something asks it to pull -- live-verified it does NOT do this
        on its own just because a mission was uploaded while this node
        wasn't running yet. Fire-and-forget: _on_waypoints picks up
        whatever comes back."""
        if self.pull_client.service_is_ready():
            self.pull_client.call_async(WaypointPull.Request())


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('name', help='mission name -- written to '
                                 'data/missions/<name>.json')
    ap.add_argument('--timeout', type=float, default=10.0,
                    help='seconds to wait for MAVROS to report a mission')
    args = ap.parse_args(argv)

    rclpy.init(args=[])
    try:
        node = _MissionDumper()
        deadline = time.monotonic() + args.timeout
        node.request_pull()
        while node.waypoints is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            node.request_pull()  # retry -- the first call may fire before the service is ready
        waypoints = node.waypoints
        node.destroy_node()
    finally:
        rclpy.shutdown()

    if not waypoints or len(waypoints) < 2:
        raise SystemExit(
            f'no mission (or an empty one) reported by MAVROS within '
            f'{args.timeout}s -- upload one in Mission Planner first')

    MISSIONS_DIR.mkdir(parents=True, exist_ok=True)
    out_path = MISSIONS_DIR / f'{args.name}.json'
    out_path.write_text(json.dumps(_dump(waypoints), indent=2))
    print(f'wrote {out_path} ({len(waypoints) - 1} waypoints)')


if __name__ == '__main__':
    main(sys.argv[1:])
