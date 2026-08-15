"""Run one baseline and one treatment trial against the same wind seed.

PX4 SITL is restarted between trials because its EKF and mission state do
not reset cleanly in place, and a warm-started EKF would make the two runs
incomparable. Gazebo itself, however, is NOT restarted between trials --
it's started once, externally (see the repo README), and every PX4
restart just reconnects gz_bridge to whatever's already in the world.

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
"""
import argparse
import os
import pathlib
import signal
import subprocess
import time

from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from gz.transport13 import Node as GzNode

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


def _reset_gazebo_model():
    """Teleport the tiltrotor entity back to the spawn pose, if it exists
    from a previous trial (a no-op the first time nothing has spawned
    yet), instead of destroying and recreating it -- see the module
    docstring for why recreating it is unreliable. This doesn't reset
    velocity (set_pose has no twist field), so the vehicle may carry a
    small residual velocity into the first tick or two of the next leg;
    that's a minor, bounded imperfection next to the alternative of
    telemetry not working at all."""
    node = GzNode()
    x, y, z = SPAWN_XYZ
    req = Pose(name=C.MODEL_NAME, position={'x': x, 'y': y, 'z': z})
    node.request(f'/world/{C.WORLD_NAME}/set_pose', req, Pose, Boolean, 2000)
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


def run_one(mode, trial, duration, headless=True):
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

    time.sleep(duration)

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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trial', default='compare')
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--gui', action='store_true',
                    help='show the Gazebo GUI (use this for the demo video)')
    args = ap.parse_args()
    for mode in ('baseline', 'treatment'):
        run_one(mode, args.trial, args.duration, headless=not args.gui)
    print('both trials complete; now run plot_results')


if __name__ == '__main__':
    main()
