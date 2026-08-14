# Task 9 report: Close the loop and produce the figures

**Status: DONE_WITH_CONCERNS**

Steps 1, 2, 3, 6, 7 are complete and verified. Steps 4 and 5 (the live
paired trial run and the demo video) could not be completed — this
sandbox's PX4 subprocess cannot locate the `gz` binary even though `gz` is
on `$PATH` and resolves fine from an interactive shell in the same
session, so PX4 SITL never reaches the Gazebo handshake and times out
waiting for the world. This is a step further than Task 2's earlier
"no display/no interactive shell" limitation — here PX4 itself can't find
Gazebo at all in this sandbox. The plotting code was verified end-to-end
against synthetic CSVs instead, and the exact commands to run for real are
documented below for you to run outside this sandbox.

## What was built

### Step 1 — controller treatment branch (DONE)

`src/aerocanyon/aerocanyon/controller_node.py`:
- Imports `CBFFilter` and `MASS_KG`, plus `VehicleAttitude` from `px4_msgs.msg`.
- `__init__` now constructs `self.cbf = CBFFilter()`, tracks `self.quat` and
  `self.wind_truth`, and adds the `cbf_pub` publisher on `C.TOPIC_CBF_DIAG`.
- New subscriptions: `/fmu/out/vehicle_attitude` → `_on_attitude`, and
  `C.TOPIC_WIND_TRUTH` → `_on_wind_truth`.
- `_tick`'s setpoint block now branches on `self.mode == 'treatment'`:
  computes `u_des = -self.wind_est / MASS_KG`, runs it through
  `self.cbf.filter(u_des, self.pos, self.vel, self.wind_truth, self.quat)`,
  publishes the resulting acceleration on the `TrajectorySetpoint`, and
  publishes a `Vector3Stamped` diagnostic (`active`, `h_min` clamped to
  ±1e3, `infeasible` flag) on `cbf_pub`.
- `_publish_offboard_mode` now sets `msg.acceleration = True` so PX4 accepts
  the acceleration channel alongside position.
- Matches the plan's brief exactly; no deviations.

### Step 2 — `run_trial.py` (DONE)

`src/aerocanyon/aerocanyon/run_trial.py`, written verbatim from the plan
with one correction carried over from Task 2's finding: `PX4_SIM_MODEL` is
set to `'gz_tiltrotor'`, not the plan's literal `'tiltrotor'` (PX4 matches
the airframe file suffix `4020_gz_tiltrotor`; `tiltrotor` alone fails to
boot). Spawns PX4 and the Micro-XRCE-DDS-Agent via `subprocess.Popen` with
`preexec_fn=os.setsid`, kills the whole process group with `SIGTERM` (then
`SIGKILL` on timeout) so no zombie PX4/Gazebo processes are left behind,
and aborts with a clear error if the resulting CSV is missing or under
1000 bytes rather than silently reporting an empty trial.

### Step 3 — `plot_results.py` (DONE, verified end-to-end)

`src/aerocanyon/aerocanyon/plot_results.py`, written verbatim from the
plan: `comparison_figure` (overlaid trajectories over the six canyon
buildings + RMS lateral-deviation bar chart with reduction % in the
title) and `intervention_figure` (barrier `h(t)` time series + CBF
active/inactive step plot), both saved at `dpi=150` into `figures/`.

Since the live trial (Step 4) could not run in this sandbox, I generated
synthetic CSVs matching `trial_logger.py`'s exact column schema (a
lower-lateral-deviation, higher-`cbf_active` "treatment" run vs. a
higher-deviation, `cbf_active`-free "baseline" run) and ran
`python -m aerocanyon.plot_results` against them directly. Both PNGs were
produced correctly and the script printed a sane, non-degenerate summary:

```
filter active for 16.6% of the flight; closest approach to the barrier: h=4.50 m
RMS lateral deviation: baseline 2.83 m, treatment 1.09 m, reduction 61.5%
```

This confirms the plotting/statistics code path is correct; it is not a
substitute for the real headline numbers, which require Step 4.

## Step 4 — run the paired trial and generate the figures — NOT DONE, needs live PX4+Gazebo

Attempted directly. `colcon build --symlink-install --packages-select
aerocanyon` succeeds and `~/PX4-Autopilot/build/px4_sitl_default/bin/px4`
and `~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent` both exist and are
executable. Running PX4 SITL headless against `urban_canyon.sdf` with
`PX4_SIM_MODEL=gz_tiltrotor` produces:

```
INFO  [init] Gazebo simulator Icannotfindanyavailable'gz'command:
	* Did you install any Gazebo library?
	* Did you set the GZ_CONFIG_PATH environment variable?
INFO  [init] Waiting for Gazebo world...
[... repeats until timeout, PX4 exits with code 15]
```

but `command -v gz` inside the identical `env ... bash -c '...'` invocation
resolves fine to `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz`, and `gz-sim8`,
`gz-tools2`, and the Python `gz.transport13`/`gz.msgs10` bindings are all
installed (confirmed via `dpkg -l`). So `gz` is genuinely on `$PATH` in
this shell but PX4's own subprocess (or whatever helper script it shells
out to internally — grepping the PX4-Autopilot tree for the literal
message text found nothing, so it's compiled in or in a dependency I
didn't chase further) doesn't see it, most likely because this agent's
sandboxed Bash tool changes what environment/capabilities a
further-nested subprocess inherits. I did not force this open by disabling
the sandbox — that was explicitly declined when I asked.

**To run this for real, outside the sandbox:**

```bash
cd /home/parvu/ros2_pinn_sim
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon build --symlink-install --packages-select aerocanyon
python3 -m aerocanyon.run_trial --trial compare --duration 60
source .venv/bin/activate
PYTHONPATH=src/aerocanyon python -m aerocanyon.plot_results --trial compare
```

Read the reduction number honestly per the plan's own instruction: confirm
`cbf_active` is non-zero in the treatment log, confirm
`corr(wind_est_e, wind_true_e)` is meaningfully positive, then only tune
the feedforward if the number is still weak — and if it stays weak, report
the CBF-only comparison as the headline rather than tuning until a number
appears.

## Step 5 — demo video — NOT DONE, blocked by the same Step 4 issue

`python3 -m aerocanyon.run_trial --trial demo --duration 60 --gui` needs
the same live PX4/Gazebo path as Step 4, so it inherits the same block.
Command is otherwise ready to run and screen-record once Step 4 is
unblocked.

## Step 6 — full test suite (DONE)

```
source .venv/bin/activate
PYTHONPATH=src/aerocanyon python -m pytest src/aerocanyon/test/ -v
```

Result: **49 passed, 1 pre-existing failure** —
`test_dryden_is_zero_mean_and_reproducible` in `test_canyon_field.py`,
same failure already documented in Task 8's report as pre-existing and
unrelated to that task; still unrelated here (Task 9 touched
`controller_node.py`, `run_trial.py`, `plot_results.py`, `README.md` only —
nothing in `canyon_field.py`). Not touched or investigated further, per
the pre-existing-failure note already on record.

`colcon build --symlink-install --packages-select aerocanyon` passes
cleanly (one harmless `pytest-repeat` setuptools warning, present before
this task too).

## Step 7 — README and commit (DONE)

Added a `## AeroCanyon-Guard` section to `README.md` covering: the PX4
patches required (`enable_wind` on the tiltrotor model, the
`urban_canyon.sdf` copy into `PX4-Autopilot`, `NAV_DLL_ACT 0`, and the
`PX4_SIM_MODEL=gz_tiltrotor` correction), how to regenerate the world
(`canyon_geometry`) and wind grid (`canyon_field`), how to train the
FO-PINN, how to run the paired baseline/treatment trial, and how to read
the two output figures.

Committed as `9c9bc4f`:
`feat(aerocanyon): close the FO-PINN/CBF loop and generate trial figures`.
`figures/` was not added — no real figures exist yet since Step 4 is
blocked; adding placeholder or synthetic-data figures to the repo would
misrepresent the project's actual result, so I left that for a run outside
this sandbox.

## Verification run

- `colcon build --symlink-install --packages-select aerocanyon` — passes.
- `pytest src/aerocanyon/test/` — 49 passed, 1 pre-existing failure
  (unrelated to this task, documented above).
- `python -m aerocanyon.plot_results` — verified end-to-end against
  synthetic CSVs; both figures generated correctly, printed summary sane.
- Syntax/import checks on `run_trial.py`, `plot_results.py`,
  `controller_node.py` — all import cleanly against the built workspace.

## Files touched

- `/home/parvu/ros2_pinn_sim/src/aerocanyon/aerocanyon/controller_node.py` (modified — treatment branch)
- `/home/parvu/ros2_pinn_sim/src/aerocanyon/aerocanyon/run_trial.py` (new)
- `/home/parvu/ros2_pinn_sim/src/aerocanyon/aerocanyon/plot_results.py` (new)
- `/home/parvu/ros2_pinn_sim/README.md` (modified — AeroCanyon-Guard section)

## What's left before this is a finished result

1. Run Step 4 (`run_trial.py` + `plot_results.py`) on a machine/session
   where PX4's subprocess can actually see `gz` — outside this sandbox,
   interactively, or with elevated subprocess permissions — to get the
   real `figures/comparison.png`, `figures/cbf_intervention.png`, and the
   real reduction percentage.
2. If the checkpoint deployed in Task 8 was trained on synthetic data
   (it was — see that task's report), retrain on real trial CSVs collected
   from a live PX4 run before trusting the treatment numbers.
3. Run Step 5 (`--gui` demo run) and screen-record it once Step 4 works,
   saving to `figures/canyon_transit.mp4`.
4. `git add figures/ trials/train*.csv` (trials/ is gitignored by design;
   only figures/ needs adding) and commit once real figures exist.
