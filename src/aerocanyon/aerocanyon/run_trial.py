"""Run one baseline and one treatment trial against the same wind seed.

PX4 SITL is restarted between trials because its EKF and mission state do
not reset cleanly in place, and a warm-started EKF would make the two runs
incomparable. Gazebo itself, however, is NOT restarted between trials --
it's started once, externally (see the repo README), and every PX4
restart just reconnects gz_bridge to whatever's already in the world. PX4
tries to (re-)spawn its model with allow_renaming:false on every start; if
the previous trial's vehicle entity is still sitting wherever that flight
ended, the spawn silently fails (its stdout is discarded) and gz_bridge
just attaches to the stale, displaced entity instead -- so the next trial
starts mid-canyon at whatever attitude/velocity the last one ended with,
not from the canyon entry. Removing the stale entity first lets PX4's own
spawn create a genuinely fresh one at the model's default pose with zero
velocity.
"""
import argparse
import os
import pathlib
import signal
import subprocess
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.scene_pb2 import Scene
from gz.transport13 import Node as GzNode

from . import constants as C

PX4_DIR = pathlib.Path.home() / 'PX4-Autopilot'
AGENT = pathlib.Path.home() / 'Micro-XRCE-DDS-Agent' / 'build' / 'MicroXRCEAgent'
WS = pathlib.Path(__file__).resolve().parents[3]


def _reset_gazebo_model():
    """Remove the tiltrotor entity left over from a previous trial, if any,
    so the upcoming PX4 restart spawns a fresh one at the default pose
    instead of silently reattaching to wherever the last flight ended.
    A no-op (result is False) the first time nothing has spawned yet."""
    node = GzNode()
    req = Entity(name=C.MODEL_NAME, type=Entity.MODEL)
    node.request(f'/world/{C.WORLD_NAME}/remove', req, Entity, Boolean, 2000)
    time.sleep(1)  # let Gazebo actually process the removal before PX4's own create races it


def _model_is_spawned():
    node = GzNode()
    ok, scene = node.request(f'/world/{C.WORLD_NAME}/scene/info', Empty(),
                             Empty, Scene, 2000)
    return ok and any(m.name == C.MODEL_NAME for m in scene.model)


def _wait_for_spawn(timeout_s=20):
    """_reset_gazebo_model() deletes the old entity before every PX4
    (re)start; if the new PX4 process then fails to actually spawn a
    replacement -- most commonly because something else (a manually
    started PX4/agent from an earlier terminal, per the README's OLDER
    4-terminal instructions) is already holding the SITL instance-0 lock
    or the UDP 8888 port this function's caller is about to try to bind,
    so this run_one()'s own PX4/agent spawn dies immediately -- the
    vehicle is simply gone for the rest of the trial with nothing else in
    the stack ever raising an error. Fail loudly here instead."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _model_is_spawned():
            return
        time.sleep(1)
    raise SystemExit(
        f'{C.MODEL_NAME} never appeared in the Gazebo scene {timeout_s}s '
        'after starting PX4 -- _reset_gazebo_model() already removed the '
        'previous one, so the vehicle is gone from the sim. Most likely '
        'cause: a PX4 and/or Micro-XRCE-DDS-Agent instance is already '
        'running from another terminal and is blocking this one\'s own '
        '(run_trial.py spawns and owns both itself -- do not start them '
        'manually alongside it). Other cause: Gazebo was started without '
        'sourcing build/px4_sitl_default/rootfs/gz_env.sh, see the README.')


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
               GZ_IP='127.0.0.1')
    if headless:
        env['HEADLESS'] = '1'

    px4 = _spawn('./build/px4_sitl_default/bin/px4 -d', cwd=PX4_DIR, env=env)
    agent = _spawn(f'{AGENT} udp4 -p 8888')
    try:
        _wait_for_spawn()  # fails loudly rather than flying a trial with no vehicle
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
