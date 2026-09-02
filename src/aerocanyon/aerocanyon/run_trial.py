"""Run one baseline and one treatment trial against the same wind seed.

ArduPilot SITL is restarted between trials because its EKF and mission
state do not reset cleanly in place, and a warm-started EKF would make
the two runs incomparable. Gazebo is now restarted between trials too, by
default: main() runs each leg (baseline, treatment) as its own separate
OS process (see run_leg/`--mode`), each of which spawns a completely
fresh `gz sim` world, boots ArduPilot against it, runs the leg, and tears
both down again -- no entity, physics state, or Python/rclpy state is
shared between legs at all. This replaced an earlier design where Gazebo
stayed up for the whole session and the vehicle entity was reset in
place between legs (see _reset_gazebo_model/_recreate_gazebo_model
below, kept for that use case -- e.g. the manual "watch a trial fly"
flow in the README still starts Gazebo externally and reuses it).

Under PX4, the model was spawned dynamically by PX4's own SITL startup
script (a gz service call, PX4_SIM_MODEL/PX4_GZ_MODEL_POSE env vars) --
whether that create call succeeded or just re-attached to an existing
entity determined the reset strategy below. Under ArduPilot, the vehicle
is baked into the world file itself (worlds/urban_canyon.sdf's own
<include>, since ArduPilotPlugin expects its model already present and
connects over its own FDM socket) -- there is no create-vs-attach
distinction any more, since nothing ever creates a new entity after the
world's own initial load. _reset_gazebo_model's teleport-in-place
approach is unaffected by this and is still exactly what's needed
between legs that reuse a Gazebo instance (the manual flow); it was
never actually PX4-specific logic, just called from PX4-era code.
_recreate_gazebo_model (removing and letting the next boot recreate the
entity) is kept as the same documented dead end it always was -- ArduPilot
has no create-on-boot behavior to exploit for it any more than PX4's
own version reliably worked.

set_pose only resets position, not velocity or attitude -- so a vehicle
that is still moving (or worse, has crashed) when this process kills the
SITL between legs carries that state into the next leg's boot, if that
boot reuses the same Gazebo world/entity. It no longer does by default
(see run_leg above): each leg gets its own fresh Gazebo process, so
whatever state the previous leg's vehicle ended up in is simply gone
when that leg's Gazebo process is killed. controller_node accordingly
just lands where it is once it clears the canyon (QLAND, see
LAND_CLEARANCE_M in controller_node.py) rather than flying anywhere
first -- an earlier design flew a full return-to-spawn-and-land, back
when that mattered for the next leg's boot state; it no longer does.
_reset_gazebo_model/_recreate_gazebo_model below are what still matter
for the manual/external-Gazebo flow (README), where a single Gazebo
process IS reused across runs.
"""
import argparse
import os
import pathlib
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from gz.transport13 import Node as GzNode
from mavros_msgs.msg import ExtendedState
from rclpy.node import Node as RosNode
from rclpy.qos import qos_profile_sensor_data

from . import canyon_geometry as cg
from . import constants as C

WS = pathlib.Path(__file__).resolve().parents[3]

ARDUPILOT_DIR = pathlib.Path.home() / 'ardupilot'
ARDUPLANE = ARDUPILOT_DIR / 'build' / 'sitl' / 'bin' / 'arduplane'
TRICOPTER_PARM = WS / 'src' / 'aerocanyon' / 'ardupilot' / 'tricopter.parm'
# geographiclib-get-geoids' default install location needs root; MAVROS
# hard-aborts on startup without the dataset (see README) -- installed
# once, machine-wide, to a user-writable path instead.
GEOGRAPHICLIB_DATA = pathlib.Path.home() / '.local' / 'share' / 'GeographicLib'
# An arbitrary real-world point (Bucharest), not tied to the canyon's own
# local-frame geometry, which is unrelated to ArduPilot's EKF origin.
# Altitude matches canyon_geometry.GROUND_Z (74) -- the manual-flight
# demo's README instructions use a separate, unrelated home altitude (76)
# for map_zone_ap.sdf's own terrain height, not this world.
HOME_LAT, HOME_LON, HOME_ALT = 44.434424990487216, 26.04781615647584, 74

# Spawn/reset position: the canyon entry's horizontal position (ENU east,
# north from CANYON_ENTRY), not the world origin -- which sits almost
# exactly under the middle tower row. 0.246 m is the tricopter model's own
# stock ground clearance (its unpatched <pose> z value) added on top of
# canyon_geometry.GROUND_Z; reusing it here keeps the landing gear resting
# on the ground instead of floating or clipping through it. Facing yaw=0
# in Gazebo's ENU-frame pose already points the nose along +x (east),
# which is the mission's actual direction of travel -- see the yaw fix in
# controller_node.py for why 0 is not the answer over in NED-land.
SPAWN_XYZ = (float(cg.CANYON_ENTRY[0]), float(cg.CANYON_ENTRY[1]),
             cg.GROUND_Z + 0.246)
# Matches worlds/urban_canyon.sdf's own <include><pose> for tricopter_ap
# -- that static XML can't reference this value directly, so keep them
# in sync by hand if either changes.

WORLD_SDF = WS / 'src' / 'aerocanyon' / 'worlds' / f'{C.WORLD_NAME}.sdf'

WEB_VIEWER = WS / 'web_viewer'
# The launch7 plugin's own binary, not the `gz launch` subcommand -- verified
# live this session that the subcommand can silently stop resolving (falls
# back to `gz`'s generic top-level help instead of actually launching)
# depending on which gz_tools_vendor happens to be first on PATH, while the
# plugin's own binary always works regardless.
GZ_LAUNCH_BIN = pathlib.Path('/usr/lib/x86_64-linux-gnu/gz/launch7/gz-launch')
WEBVIEW_PORT = 8080


def _gazebo_env():
    """Equivalent of the README's manual GZ_SIM_RESOURCE_PATH/
    GZ_SIM_SYSTEM_PLUGIN_PATH exports -- needed because this process
    launches `gz sim` itself now (see _spawn_gazebo) instead of relying
    on it already being started, sourced, externally. Without
    GZ_SIM_RESOURCE_PATH, gz-sim can't resolve model://tricopter_ap and
    the vehicle silently never spawns; without GZ_SIM_SYSTEM_PLUGIN_PATH,
    ArduPilotPlugin itself can't be found and the model.sdf's <plugin>
    tag silently fails to load.

    tricopter_ap's model.sdf reuses several stock meshes (standard_vtol's
    wing/prop/elevon .dae files) and the airspeed sensor model verbatim
    from PX4-Autopilot's own Gazebo asset library, rather than vendoring
    copies into this repo -- a Phase 1 decision, unrelated to PX4 itself
    being gone from this project (px4_msgs, the old PX4 model, and PX4
    SITL are all removed; this is asset reuse only, not a PX4 runtime
    dependency). $HOME/PX4-Autopilot must still exist on disk for these
    `model://` URIs to resolve -- confirmed live: omitting it fails the
    whole world load with "Unable to find uri[model://airspeed]" and
    several unresolved standard_vtol mesh URIs, not a silent no-op."""
    models = str(WS / 'src' / 'aerocanyon' / 'models')
    aerocanyon_dir = str(WS / 'src' / 'aerocanyon')
    px4_models = str(pathlib.Path.home() / 'PX4-Autopilot' / 'Tools' / 'simulation' / 'gz' / 'models')
    plugins = str(pathlib.Path.home() / 'ardupilot_gazebo' / 'build')
    env = dict(os.environ)
    env['GZ_SIM_RESOURCE_PATH'] = ':'.join(
        p for p in (env.get('GZ_SIM_RESOURCE_PATH', ''), models, aerocanyon_dir,
                    px4_models) if p)
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = ':'.join(
        p for p in (env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''), plugins) if p)
    return env


def _spawn_gazebo():
    """Start a brand-new `gz sim` server for this leg -- see run_leg for
    why: a genuinely fresh Gazebo process, not just an in-world entity
    reset, gives the SITL nothing carried over from any previous leg at
    all (no shared entity, no shared physics engine state).

    `-s` (server-only/headless): a leg no longer needs a working X11
    DISPLAY to run at all -- watching it fly is now the browser viewer's
    job (see _spawn_web_bridge), not the native GUI's. Headless also
    sidesteps the native-GUI-window-under-WSLg rendering issue documented
    in the README/History.md entirely, rather than working around it.

    No separate `-g` flag (that's the OTHER, standalone way to get a
    GUI): tried it in an earlier version of this function and verified
    live that `-g` makes gz-sim detach a child that escapes the
    process-group kill in run_leg's teardown. Moot now that this
    function doesn't want a GUI process at all, but worth remembering if
    a native GUI is ever wanted back."""
    proc = _spawn(f'gz sim -v 2 {WORLD_SDF} -r -s',
                  cwd=WS, env=_gazebo_env())
    time.sleep(5)  # matches the README's manual startup margin
    if proc.poll() is not None:
        raise SystemExit(
            f'gz sim exited immediately (code {proc.returncode}) instead of '
            'staying up -- check that ros-jazzy-ros-gz* is installed and '
            '`gz` is on PATH (source /opt/ros/jazzy/setup.bash).')
    return proc


def _spawn_web_bridge():
    """Bridge this leg's already-running gz-transport session to a
    browser via websocket (web_viewer/, see README's "Fly from a
    browser instead") so a headless leg (see _spawn_gazebo) is still
    watchable live, just through the browser instead of a native window.
    Best-effort: a missing gz-launch binary shouldn't fail the whole
    trial over a visualization nicety."""
    if not GZ_LAUNCH_BIN.exists():
        print(f'warning: {GZ_LAUNCH_BIN} not found -- web viewer bridge '
              'skipped, trial continues without live viewing')
        return None
    return _spawn(f'{GZ_LAUNCH_BIN} websocket.gzlaunch', cwd=WEB_VIEWER)


def _spawn_static_server():
    """Static file server for web_viewer/, plus the wind-speed-only
    control endpoint (control_server.py --no-rc, see that module's own
    docstring) so the browser's spd+/spd- buttons ("wind medium speed")
    work live while watching an autonomous leg. Not full control_server.py
    -- that publishes RC overrides continuously, which would fight
    controller_node's own /mavros/rc/override calls for control authority
    during an actual trial (see README's warning against running it
    alongside one); --no-rc skips exactly that part while keeping the
    static files and the (now RC-free) speed_up/speed_down handling.
    Safe to run for the whole trial's lifetime -- started once here, not
    per leg like the web bridge above, and deliberately never killed by
    this script: it needs to keep serving results.html after main()
    itself has returned, and os.setsid (see _spawn) already detaches it
    from this process's own lifetime. Kills any previous instance on the
    same port first so re-running this script doesn't pile up orphaned
    servers fighting over WEBVIEW_PORT."""
    subprocess.run(['pkill', '-f', f'http.server {WEBVIEW_PORT}'],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-f', f'control_server.py {WEBVIEW_PORT} --no-rc'],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)
    return _spawn(f'{sys.executable} control_server.py {WEBVIEW_PORT} --no-rc',
                  cwd=WEB_VIEWER)


def _reset_gazebo_model():
    """Teleport the tricopter entity back to the spawn pose, if it exists
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
    """Remove the tricopter entity outright instead of teleporting it, so
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


def _verify_sitl_started(sitl_proc, timeout_s=5):
    """The most common way the vehicle silently goes missing: something
    else (a manually-started arduplane/MAVROS from an earlier terminal,
    per the README's warning against combining that with run_trial.py)
    is already holding the SITL's TCP port, so THIS process's own
    arduplane spawn dies within moments of starting -- previously with no
    visible error anywhere in the stack. Check the subprocess directly
    rather than inferring it from Gazebo state, which no longer changes
    in a way that would reveal this now that _reset_gazebo_model() doesn't
    destroy/recreate the entity every leg."""
    time.sleep(timeout_s)
    if sitl_proc.poll() is not None:
        raise SystemExit(
            f'arduplane exited immediately (code {sitl_proc.returncode}) '
            'instead of staying up. Most likely cause: an arduplane and/or '
            'mavros_node instance is already running from another terminal '
            'and is blocking this one\'s own (run_trial.py spawns and owns '
            'both itself -- do not start them manually alongside it).')


class _LandWatcher(RosNode):
    """Watches /mavros/extended_state -- nothing else. controller_node
    requests ArduPilot's own QLAND, in place, once it measures having
    cleared the canyon exit by LAND_CLEARANCE_M (see controller_node.py),
    and stops publishing RC overrides at that same moment so it isn't
    fighting QLAND's landing logic for control authority. That means the
    mission nodes (including trial_logger) can -- and should -- stay
    alive and keep recording for the whole hold/transit/landing sequence;
    this node only needs to watch for the landing to actually finish.

    landed requires having been AIRBORNE first: ExtendedState.landed_state
    reads ON_GROUND by default at boot (resting on the ground, motors
    off), so checking it alone reports "landed" before the vehicle has
    flown at all -- the same failure mode the PX4-era VehicleLandDetected
    check was built to avoid, and the same guard here."""

    def __init__(self):
        super().__init__('land_watcher')
        self.was_airborne = False
        self.landed = False
        self.create_subscription(
            ExtendedState, '/mavros/extended_state',
            self._on_land, qos_profile_sensor_data)

    def _on_land(self, msg):
        if msg.landed_state == ExtendedState.LANDED_STATE_IN_AIR:
            self.was_airborne = True
        self.landed = (self.was_airborne
                       and msg.landed_state == ExtendedState.LANDED_STATE_ON_GROUND)


def _wait_for_landing(timeout_s):
    """Block until /mavros/extended_state reports landed after having
    been airborne, or timeout_s elapses. Returns whether it actually
    confirmed landed -- callers should keep going either way (killing the
    SITL mid-air is still better than hanging the whole pipeline on a
    stuck telemetry reading), but should log the difference."""
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


def run_one(mode, trial, duration, clean_respawn=False, seed=0, turbulence=2.5, ff_gain=0.2):
    if clean_respawn:
        _recreate_gazebo_model()
    else:
        _reset_gazebo_model()

    apstate_dir = pathlib.Path(tempfile.mkdtemp(prefix='aerocanyon_apstate_'))
    home_str = f'{HOME_LAT},{HOME_LON},{HOME_ALT},0'
    arduplane = _spawn(
        f'{ARDUPLANE} --model JSON --home {home_str} '
        f'--wipe --defaults {TRICOPTER_PARM}',
        cwd=apstate_dir)
    try:
        _verify_sitl_started(arduplane)  # fails loudly rather than flying a trial with no vehicle
    except SystemExit:
        _kill(arduplane)
        raise
    time.sleep(20)  # remaining EKF/GPS-fix convergence margin (~30s total, see README)

    mavros_env = dict(os.environ, GEOGRAPHICLIB_DATA=str(GEOGRAPHICLIB_DATA))
    mavros = _spawn(
        'ros2 run mavros mavros_node --ros-args '
        '-p fcu_url:=tcp://127.0.0.1:5760 -p system_id:=255',
        env=mavros_env)
    time.sleep(5)  # MAVROS connect margin

    launch = (f'bash -lc "source /opt/ros/jazzy/setup.bash && '
              f'source {WS}/install/setup.bash && '
              f'ros2 launch aerocanyon canyon_sim.launch.py '
              f'mode:={mode} trial:={trial} seed:={seed} '
              f'turbulence_sigma:={turbulence} feedforward_gain:={ff_gain}"')
    nodes = _spawn(launch, cwd=WS)

    # Both modes hand off to ArduPilot's own QLAND, in place, once
    # controller_node measures clearing the canyon exit -- see
    # controller_node.LAND_CLEARANCE_M.
    landed = _wait_for_landing(timeout_s=duration)
    if not landed:
        print(f'warning: {mode} did not confirm landed within {duration}s '
              '(extended_state never reported landed) -- proceeding anyway')

    for p in (nodes, mavros, arduplane):
        _kill(p)
    shutil.rmtree(apstate_dir, ignore_errors=True)
    time.sleep(3)

    csv = WS / 'trials' / f'{trial}_{mode}.csv'
    if not csv.exists() or csv.stat().st_size < 1000:
        raise SystemExit(
            f'trial {trial}/{mode} produced no usable log at {csv} -- '
            'aborting rather than reporting a figure built on nothing')
    print(f'{mode}: wrote {csv} ({csv.stat().st_size} bytes)')
    return csv


def run_leg(mode, trial, duration, seed=0, turbulence=2.5, ff_gain=0.2):
    """Own one leg's entire Gazebo+ArduPilot lifecycle: spawn a fresh
    `gz sim`, run the leg, tear the world back down -- always, even if
    the leg raises. This is what main() runs as a separate OS process
    per leg (see `--mode`), so a leg's Gazebo/ArduPilot/rclpy state can
    never leak into the next one. Also owns this leg's web viewer bridge (see
    _spawn_web_bridge) -- paired 1:1 with its own gz sim instance, unlike
    the static file server in main(), which spans the whole trial."""
    gz = _spawn_gazebo()
    bridge = _spawn_web_bridge()
    try:
        return run_one(mode, trial, duration, seed=seed, turbulence=turbulence,
                       ff_gain=ff_gain)
    finally:
        _kill(bridge)
        _kill(gz)
        # Belt-and-suspenders: verified live that `gz sim ... -g` (the
        # standalone gui-client flag, no longer used by _spawn_gazebo --
        # see its docstring for why) can leave behind an orphaned
        # companion process that escapes the process-group kill above.
        # -x matches the pattern against the WHOLE command line only, so
        # this can never collide with the combined server+gui process
        # _spawn_gazebo actually launches.
        subprocess.run(['pkill', '-xf', 'gz sim -g'],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trial', default='compare')
    ap.add_argument('--duration', type=float, default=220.0,
                    help='max seconds to wait for a real landing before '
                         'giving up and moving on anyway. Both modes hand '
                         'off to ArduPilot\'s own QLAND, in place, once '
                         'controller_node measures clearing the canyon '
                         'exit -- covers the hold, the canyon transit '
                         '(nominally ~17s of mission time, but real flight '
                         'dynamics have taken 60-90s live), and the '
                         'AUTO_LAND descent itself (~100s observed live, '
                         'manually, from ~75m), with margin')
    ap.add_argument('--seed', type=int, default=0,
                    help='Dryden gust RNG seed -- vary this across trials '
                         'for wind diversity (e.g. when building a training set)')
    ap.add_argument('--turbulence', type=float, default=2.5,
                    help='Dryden gust intensity (m/s). The scenario default is '
                         '4.0; the original 1.5 left the disturbance dominated '
                         'by the steady mean flow, which the position/altitude '
                         'control loop absorbs on its own')
    ap.add_argument('--ff-gain', type=float, default=0.2, dest='ff_gain',
                    help='scales the PINN feedforward (treatment mode only)')
    ap.add_argument('--mode', choices=('baseline', 'treatment'), default=None,
                    help=argparse.SUPPRESS)  # internal: run_leg's own subprocess re-invokes with this set
    args = ap.parse_args()

    if args.mode:
        run_leg(args.mode, args.trial, args.duration, seed=args.seed,
                turbulence=args.turbulence, ff_gain=args.ff_gain)
        return

    # Started once here, not per leg -- see _spawn_static_server's own
    # docstring for why (no ROS2 involvement, safe for the whole trial,
    # needs to keep serving results.html after this function returns).
    _spawn_static_server()
    print(f'web viewer: http://localhost:{WEBVIEW_PORT} '
          '(live during each leg, results page once both finish)')

    # Each leg gets its own OS process, and inside that its own fresh
    # Gazebo+ArduPilot (see run_leg) -- no state at all carries from one
    # leg into the next.
    for mode in ('baseline', 'treatment'):
        cmd = [sys.executable, '-m', 'aerocanyon.run_trial',
               '--mode', mode, '--trial', args.trial,
               '--duration', str(args.duration), '--seed', str(args.seed),
               '--turbulence', str(args.turbulence),
               '--ff-gain', str(args.ff_gain)]
        result = subprocess.run(cmd, cwd=WS)
        if result.returncode != 0:
            raise SystemExit(f'{mode} leg failed (exit code {result.returncode})')
    print('both trials complete; generating figures')

    plot_cmd = [sys.executable, '-m', 'aerocanyon.plot_results', '--trial', args.trial]
    result = subprocess.run(plot_cmd, cwd=WS)
    if result.returncode != 0:
        raise SystemExit(f'plot_results failed (exit code {result.returncode})')

    # Copies rather than points results.html at ../figures/ directly:
    # the static server is rooted at web_viewer/ (see _spawn_static_server)
    # and Python's http.server refuses to serve outside its own root.
    results_dir = WEB_VIEWER / 'results'
    results_dir.mkdir(exist_ok=True)
    for name in ('comparison.png', 'cbf_intervention.png'):
        shutil.copy(WS / 'figures' / name, results_dir / name)

    print(f'results: http://localhost:{WEBVIEW_PORT}/results.html')


if __name__ == '__main__':
    main()
