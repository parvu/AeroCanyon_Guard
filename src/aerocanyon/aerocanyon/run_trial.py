"""Run one baseline and one treatment trial against the same wind seed.

PX4 SITL is restarted between trials because its EKF and mission state do
not reset cleanly in place, and a warm-started EKF would make the two runs
incomparable. Gazebo is now restarted between trials too, by default:
main() runs each leg (baseline, treatment) as its own separate OS process
(see run_leg/`--mode`), each of which spawns a completely fresh `gz sim`
world, boots PX4 against it, runs the leg, and tears both down again --
no entity, physics state, or Python/rclpy state is shared between legs at
all. This replaced an earlier design where Gazebo stayed up for the whole
session and the vehicle entity was reset in place between legs (see
_reset_gazebo_model/_recreate_gazebo_model below, kept for that use case
-- e.g. the manual "watch a trial fly" flow in the README still starts
Gazebo externally and reuses it).

PX4 tries to (re-)spawn its model with allow_renaming:false on every
start; if the previous trial's vehicle entity is still there, the create
call fails and gz_bridge just attaches to the existing entity instead
(px4-rc.gzsim doesn't check create's result, it starts gz_bridge either
way) -- so the next trial would otherwise start wherever the last one
ended, not from the canyon entry. An earlier version of this function
fixed that by REMOVING the entity first so PX4's create would actually
succeed -- but destroying and recreating the entity between legs turned
out to be unreliable: verified live, twice, on two independently fresh
Gazebo instances, that the first PX4 boot in a session (nothing to
remove, plain create) always works, while the second (remove, then a
real create) reliably left every /fmu/out telemetry topic silently
frozen at zero for the rest of that trial, despite the entity itself
existing in the scene and arming succeeding. Teleporting the SAME entity
back to the spawn pose via set_pose, instead of destroying and recreating
it, sidesteps that fragility entirely: PX4's own create call is expected
to fail every leg after the first (harmlessly, same as before the
remove-based fix existed) and gz_bridge just reattaches to the one
long-lived entity, exactly as it did on the very first, always-reliable
boot.

set_pose only resets position, not velocity or attitude -- so a vehicle
that is still moving (or worse, has crashed) when this process kills PX4
between legs carries that state into the next leg's boot. controller_node
now flies a genuine return-to-start-and-land before this function ever
kills PX4 for that reason: the intermittent failure of the treatment leg
to arm/fly was traced to exactly this -- the baseline leg's vehicle either
crashing mid-flight or simply not being at rest at the spawn point when
the next PX4 process starts. Landing for real, not just clearing the
canyon or hitting a wall-clock timeout, is what makes the next leg's spawn
position and initial state actually match what PX4/the EKF expect.
"""
import argparse
import os
import pathlib
import signal
import subprocess
import sys
import time

import rclpy
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from gz.transport13 import Node as GzNode
from px4_msgs.msg import VehicleLandDetected
from rclpy.node import Node as RosNode
from rclpy.qos import qos_profile_sensor_data

from . import canyon_geometry as cg
from . import constants as C

PX4_DIR = pathlib.Path.home() / 'PX4-Autopilot'
AGENT = pathlib.Path.home() / 'Micro-XRCE-DDS-Agent' / 'build' / 'MicroXRCEAgent'
WS = pathlib.Path(__file__).resolve().parents[3]

# Spawn/reset position: the canyon entry's horizontal position (ENU east,
# north from CANYON_ENTRY), not the world origin -- which sits almost
# exactly under the middle tower row. 0.246 m is the tiltrotor model's own
# stock ground clearance (its unpatched <pose> z value); reusing it here
# keeps the landing gear resting on the ground instead of floating or
# clipping through it. Facing yaw=0 in Gazebo's ENU-frame pose already
# points the nose along +x (east), which is the mission's actual direction
# of travel -- see the yaw fix in controller_node.py for why 0 is not the
# answer over in NED-land.
SPAWN_XYZ = (float(cg.CANYON_ENTRY[0]), float(cg.CANYON_ENTRY[1]), 0.246)
SPAWN_POSE = f'{SPAWN_XYZ[0]},{SPAWN_XYZ[1]},{SPAWN_XYZ[2]},0,0,0'

WORLD_SDF = PX4_DIR / 'Tools/simulation/gz/worlds' / f'{C.WORLD_NAME}.sdf'


def _gazebo_env():
    """Equivalent of sourcing build/px4_sitl_default/rootfs/gz_env.sh --
    needed because this process launches `gz sim` itself now (see
    _spawn_gazebo) instead of relying on it already being started,
    sourced, externally. Without GZ_SIM_RESOURCE_PATH, gz-sim can't
    resolve model://tiltrotor and the vehicle silently never spawns."""
    models = str(PX4_DIR / 'Tools/simulation/gz/models')
    worlds = str(PX4_DIR / 'Tools/simulation/gz/worlds')
    plugins = str(PX4_DIR / 'build/px4_sitl_default/src/modules/simulation/gz_plugins')
    env = dict(os.environ)
    env['GZ_SIM_RESOURCE_PATH'] = ':'.join(
        p for p in (env.get('GZ_SIM_RESOURCE_PATH', ''), models, worlds) if p)
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = ':'.join(
        p for p in (env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''), plugins) if p)
    return env


def _spawn_gazebo(headless=True):
    """Start a brand-new `gz sim` server for this leg -- see run_leg for
    why: a genuinely fresh Gazebo process, not just an in-world entity
    reset, gives PX4 nothing carried over from any previous leg at all
    (no shared entity, no shared physics engine state)."""
    flag = '' if headless else '-g'
    proc = _spawn(f'gz sim -v 2 {WORLD_SDF} -r -s {flag}'.strip(),
                  cwd=PX4_DIR, env=_gazebo_env())
    time.sleep(5)  # matches the README's manual startup margin
    if proc.poll() is not None:
        raise SystemExit(
            f'gz sim exited immediately (code {proc.returncode}) instead of '
            'staying up -- check that ros-jazzy-ros-gz* is installed and '
            '`gz` is on PATH (source /opt/ros/jazzy/setup.bash).')
    return proc


def _reset_gazebo_model():
    """Teleport the tiltrotor entity back to the spawn pose, if it exists
    from a previous trial (a no-op the first time nothing has spawned
    yet), instead of destroying and recreating it -- see the module
    docstring for why recreating it is unreliable. This doesn't reset
    velocity (set_pose has no twist field), so the vehicle may carry a
    small residual velocity into the first tick or two of the next leg;
    that's a minor, bounded imperfection next to the alternative of
    telemetry not working at all.

    ORIENTATION is reset explicitly (identity quaternion -- level,
    yaw=0), not left alone. Verified live: vehicle_attitude occasionally
    reports the vehicle flipped ~180 degrees in roll immediately on
    spawn/settle -- as early as t=0.1s, before this project's own code has
    sent an arm command and before PX4 itself considers the vehicle
    armable, so it isn't an arm-time motor-torque event (COM_SPOOLUP_TIME
    was tried live and made no difference) and it isn't this project's
    own axis-conversion code (frames.py's NED/ENU swap and quat_to_rotmat
    were checked and are self-consistent). The timing instead points at
    physics-state carried over from repeatedly teleporting the SAME
    long-lived entity via set_pose rather than a genuinely fresh spawn --
    see _recreate_gazebo_model() below for the (riskier) alternative that
    tests that theory directly. Previously, set_pose only touched
    position, so a leg that ended flipped over stayed flipped over
    through the teleport and started the NEXT leg still upside-down at
    the spawn point -- turning one bad moment into a permanently-broken
    trial. Forcing orientation level on every reset can't fix the initial
    flip, but it stops it from persisting across legs."""
    node = GzNode()
    x, y, z = SPAWN_XYZ
    req = Pose(name=C.MODEL_NAME, position={'x': x, 'y': y, 'z': z},
               orientation={'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0})
    node.request(f'/world/{C.WORLD_NAME}/set_pose', req, Pose, Boolean, 2000)
    time.sleep(1)


def _recreate_gazebo_model():
    """Remove the tiltrotor entity outright instead of teleporting it, so
    PX4's own create-on-boot call produces a genuinely fresh entity with
    no carried-over physics state, on the theory that the live-observed
    spawn-time flip (see _reset_gazebo_model) comes from repeatedly
    teleporting the same long-lived body rather than from arming/motor
    torque. TRIED LIVE as run_one's clean_respawn and CONFIRMED to
    reproduce the exact failure the module docstring already warned
    about: every /fmu/out telemetry topic (position AND attitude) came
    back frozen at exactly zero for the entire flight, on the very next
    PX4 boot after the remove. That trades an intermittent flip for a
    guaranteed dead trial, so run_one/main do NOT call this -- it is kept
    here, unused, as a documented dead end so nobody re-tries it without
    reading this first."""
    node = GzNode()
    req = Entity(name=C.MODEL_NAME, type=Entity.MODEL)
    node.request(f'/world/{C.WORLD_NAME}/remove', req, Entity, Boolean, 2000)
    time.sleep(1)


def _verify_px4_started(px4_proc, timeout_s=5):
    """The most common way the vehicle silently goes missing: something
    else (a manually-started PX4/agent from an earlier terminal, per the
    README's warning against combining that with run_trial.py) is already
    holding the SITL instance-0 lock or UDP 8888, so THIS process's own
    PX4/agent spawn dies within moments of starting -- previously with no
    visible error anywhere in the stack. Check the subprocess directly
    rather than inferring it from Gazebo state, which no longer changes
    in a way that would reveal this now that _reset_gazebo_model() doesn't
    destroy/recreate the entity every leg."""
    time.sleep(timeout_s)
    if px4_proc.poll() is not None:
        raise SystemExit(
            f'PX4 exited immediately (code {px4_proc.returncode}) instead of '
            'staying up. Most likely cause: a PX4 and/or Micro-XRCE-DDS-Agent '
            'instance is already running from another terminal and is '
            'blocking this one\'s own (run_trial.py spawns and owns both '
            'itself -- do not start them manually alongside it).')


class _LandWatcher(RosNode):
    """Watches vehicle_land_detected -- nothing else. controller_node flies
    itself back to the spawn point and lands under its own offboard control
    once it measures having cleared the canyon exit by RETURN_CLEARANCE_M
    (see controller_node.py) -- it keeps streaming setpoints the whole way,
    only disarming once it confirms landed at home. That means the mission
    nodes (including trial_logger) can -- and should -- stay alive and
    keep recording for the whole hold/transit/return/landing sequence; this
    node only needs to watch for the landing to actually finish.

    landed requires having been AIRBORNE first: vehicle_land_detected.landed
    is true by default at boot (resting on the ground, motors off), so
    checking it alone reports "landed" before the vehicle has flown at
    all. Verified live: the vehicle never actually cleared the canyon or
    got anywhere near RTL before this falsely signalled landed at ~1s in,
    and the mission nodes (trial_logger included) got killed before they
    had produced any real data."""

    def __init__(self):
        super().__init__('land_watcher')
        self.was_airborne = False
        self.landed = False
        self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._on_land, qos_profile_sensor_data)

    def _on_land(self, msg):
        if not msg.landed:
            self.was_airborne = True
        self.landed = self.was_airborne and msg.landed


def _wait_for_landing(timeout_s):
    """Block until vehicle_land_detected reports landed after having been
    airborne, or timeout_s elapses. Returns whether it actually confirmed
    landed -- callers should keep going either way (killing PX4 mid-air is
    still better than hanging the whole pipeline on a stuck telemetry
    reading), but should log the difference."""
    rclpy.init(args=[])
    try:
        node = _LandWatcher()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not node.landed:
            rclpy.spin_once(node, timeout_sec=0.5)
        landed = node.landed
        node.destroy_node()
        return landed
    finally:
        rclpy.shutdown()


def _spawn(cmd, cwd=None, env=None):
    return subprocess.Popen(cmd, shell=True, cwd=cwd, env=env,
                            preexec_fn=os.setsid,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _kill(proc):
    if proc and proc.poll() is None:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)


def run_one(mode, trial, duration, headless=True, clean_respawn=False):
    if clean_respawn:
        _recreate_gazebo_model()
    else:
        _reset_gazebo_model()

    env = dict(os.environ,
               # NOTE: the airframe file is 4020_gz_tiltrotor; PX4 matches
               # PX4_SIM_MODEL against that suffix, so the bare 'tiltrotor'
               # named in the original spec fails to boot (see task 2).
               PX4_SIM_MODEL='gz_tiltrotor',
               PX4_GZ_WORLD='urban_canyon',
               PX4_GZ_MODEL_POSE=SPAWN_POSE,
               GZ_IP='127.0.0.1')
    if headless:
        env['HEADLESS'] = '1'

    px4 = _spawn('./build/px4_sitl_default/bin/px4 -d', cwd=PX4_DIR, env=env)
    agent = _spawn(f'{AGENT} udp4 -p 8888')
    try:
        _verify_px4_started(px4)  # fails loudly rather than flying a trial with no vehicle
    except SystemExit:
        _kill(agent)
        _kill(px4)
        raise
    time.sleep(10)  # remaining EKF convergence margin

    launch = (f'bash -lc "source /opt/ros/jazzy/setup.bash && '
              f'source {WS}/install/setup.bash && '
              f'ros2 launch aerocanyon canyon_sim.launch.py '
              f'mode:={mode} trial:={trial}"')
    nodes = _spawn(launch, cwd=WS)

    # Baseline hands off to native PX4 RTL on clearing the canyon (no wind
    # feedforward to fly itself home against the crosswind); treatment
    # flies itself back to the spawn point and lands under its own
    # offboard control, using the wind estimate the whole way. See
    # controller_node.RETURN_CLEARANCE_M.
    landed = _wait_for_landing(timeout_s=duration)
    if not landed:
        print(f'warning: {mode} did not confirm landed within {duration}s '
              '(vehicle_land_detected.landed never went true) -- proceeding anyway')

    for p in (nodes, agent, px4):
        _kill(p)
    time.sleep(3)

    csv = WS / 'trials' / f'{trial}_{mode}.csv'
    if not csv.exists() or csv.stat().st_size < 1000:
        raise SystemExit(
            f'trial {trial}/{mode} produced no usable log at {csv} -- '
            'aborting rather than reporting a figure built on nothing')
    print(f'{mode}: wrote {csv} ({csv.stat().st_size} bytes)')
    return csv


def run_leg(mode, trial, duration, headless=True):
    """Own one leg's entire Gazebo+PX4 lifecycle: spawn a fresh `gz sim`,
    run the leg, tear the world back down -- always, even if the leg
    raises. This is what main() runs as a separate OS process per leg
    (see `--mode`), so a leg's Gazebo/PX4/rclpy state can never leak into
    the next one."""
    gz = _spawn_gazebo(headless=headless)
    try:
        return run_one(mode, trial, duration, headless=headless)
    finally:
        _kill(gz)
        # Verified live, twice: gz-sim can leave behind a companion
        # process (`gz sim -g`, no world argument) that's already
        # reparented to init by the time _kill's killpg runs, so it
        # survives the process-group kill above. -x matches the pattern
        # against the WHOLE command line only, so this can't collide with
        # our own real `gz sim -v 2 <world> -r -s -g` GUI invocation.
        subprocess.run(['pkill', '-xf', 'gz sim -g'],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trial', default='compare')
    ap.add_argument('--duration', type=float, default=120.0,
                    help='max seconds to wait for a real landing before '
                         'giving up and moving on anyway. Covers the hold '
                         '(3s), the ~17s canyon transit, the ~17s flyback, '
                         'and the descent+disarm at the spawn point, with '
                         'margin. Baseline hands off to native PX4 RTL on '
                         'clearing the canyon, which was verified live not '
                         'to actually navigate home in this SITL '
                         'configuration, so baseline will likely still time '
                         'out here; treatment flies itself home under its '
                         'own wind-compensated offboard control instead')
    ap.add_argument('--gui', action='store_true',
                    help='show the Gazebo GUI (use this for the demo video)')
    ap.add_argument('--mode', choices=('baseline', 'treatment'), default=None,
                    help=argparse.SUPPRESS)  # internal: run_leg's own subprocess re-invokes with this set
    args = ap.parse_args()

    if args.mode:
        run_leg(args.mode, args.trial, args.duration, headless=not args.gui)
        return

    # Each leg gets its own OS process, and inside that its own fresh
    # Gazebo+PX4 (see run_leg) -- no state at all carries from one leg
    # into the next.
    for mode in ('baseline', 'treatment'):
        cmd = [sys.executable, '-m', 'aerocanyon.run_trial',
               '--mode', mode, '--trial', args.trial,
               '--duration', str(args.duration)]
        if args.gui:
            cmd.append('--gui')
        result = subprocess.run(cmd, cwd=WS)
        if result.returncode != 0:
            raise SystemExit(f'{mode} leg failed (exit code {result.returncode})')
    print('both trials complete; now run plot_results')


if __name__ == '__main__':
    main()
