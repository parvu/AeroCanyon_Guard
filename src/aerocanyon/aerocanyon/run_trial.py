"""Run one baseline and one treatment trial against the same wind seed.

PX4 SITL is restarted between trials because its EKF and mission state do
not reset cleanly in place, and a warm-started EKF would make the two runs
incomparable.
"""
import argparse
import os
import pathlib
import signal
import subprocess
import time

PX4_DIR = pathlib.Path.home() / 'PX4-Autopilot'
AGENT = pathlib.Path.home() / 'Micro-XRCE-DDS-Agent' / 'build' / 'MicroXRCEAgent'
WS = pathlib.Path(__file__).resolve().parents[3]


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
    time.sleep(15)  # PX4 boot + EKF convergence

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
