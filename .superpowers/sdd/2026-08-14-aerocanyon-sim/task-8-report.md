# Task 8 report: PINN training and inference node

**Status: DONE_WITH_CONCERNS**

Steps 2–5 and 7 are complete and verified. Step 1 (real PX4 data collection)
and the live half of Step 6 (`ros2 topic hz` against a running PX4) could not
be executed — no PX4 process or binary is available in this environment.
Both are noted below with what stands in for them and what to re-run before
trusting the deployed checkpoint operationally.

## What was built

- `src/aerocanyon/aerocanyon/train_pinn.py` — `wind_force`, `thrust_from_state`,
  `load_dataset(csv_paths, alpha, n)`, `train(csv_paths, out_path, ...)`,
  CLI `__main__` with the deploy-safety assert (`skill > 0`). Matches the
  plan's spec exactly, with one addition: `train()` now picks CUDA via
  `torch.cuda.is_available()` and falls back to CPU (no GPU present in this
  sandbox, so this path is untested on real hardware but the fallback path
  is exercised).
- `src/aerocanyon/aerocanyon/fo_pinn_node.py` — ROS node, loads the checkpoint,
  runs `WindEstimator` + `FractionalMemory` at `CONTROL_HZ` (50 Hz), publishes
  `Vector3Stamped` on `constants.TOPIC_WIND_EST`. Same CUDA/CPU device
  selection as the trainer.
- `src/aerocanyon/data/wind_estimator.pt` — trained checkpoint
  (`state_dict`, `alpha=1.0`, `n=16`, `hidden=96`).
- `docs/alpha_sweep.txt` — alpha vs. skill table.

## Step 1 (data collection) — NOT DONE, needs live PX4

No PX4 binary or process exists in this environment (`which px4` and
`pgrep px4` both empty). Real `trials/train{1,2,3}_baseline.csv` were not
collected. **This must be run before deploying the current checkpoint**:

```bash
cd /home/parvu/ros2_pinn_sim
source /opt/ros/jazzy/setup.bash && source install/setup.bash
for seed in 1 2 3; do
  ros2 launch aerocanyon canyon_sim.launch.py mode:=baseline trial:=train$seed
done
```

Then retrain: `PYTHONPATH=src/aerocanyon .venv/bin/python -m aerocanyon.train_pinn trials/train*_baseline.csv --alpha 1.0`
(or re-sweep alpha — see below) and re-check skill before trusting the
checkpoint. `trials/` is gitignored, so nothing here touched the repo's
data-collection story.

To exercise the trainer end-to-end without PX4, I generated synthetic
stand-in CSVs matching `trial_logger.py`'s exact column schema (OU-process
wind gusts per seed, velocity responding to the same quadratic-drag law
`train_pinn.wind_force` targets, near-level attitude). Generator script:
`/tmp/claude-1000/-home-parvu-ros2-pinn-sim/6737c97f-6ce9-4b7d-b628-109fb21bd7d5/scratchpad/gen_synth_trials.py`
(not committed — scratchpad only). This validates the pipeline mechanically
but is not a substitute for real turbulence data or for the alpha-sweep
conclusion below, which reflects synthetic dynamics, not real gust memory.

## Step 3 (train, verify skill > 0) — DONE (on synthetic data)

```
val_mse=0.0483  zero-predictor=1.0692  skill=0.955   (alpha=0.7, first run)
```

Skill well above 0; assert did not fire.

## Step 4 (alpha sweep) — DONE (on synthetic data)

`docs/alpha_sweep.txt`:

```
alpha=0.0  skill=0.949
alpha=0.3  skill=0.956
alpha=0.5  skill=0.960
alpha=0.7  skill=0.955
alpha=0.9  skill=0.959
alpha=1.0  skill=0.963
```

Retrained the deployed checkpoint at alpha=1.0 (best in this sweep):
`val_mse=0.0395  zero-predictor=1.0692  skill=0.963`.

**Caveat that matters for the project's central claim:** the synthetic wind
is a first-order Ornstein-Uhlenbeck process, which has no genuine
fractional/long-memory structure, so it's unsurprising alpha=1.0 (plain
first difference) edges out fractional values here — the sweep is only
evidence about the pipeline, not about real turbulence. **Re-run this sweep
against real trial data from Step 1**; the fractional term earning its place
is an empirical claim about real gusts, and this table doesn't establish it.

## Step 5 (inference node) — DONE

`fo_pinn_node.py` written per spec: loads checkpoint via
`get_package_share_directory` fallback or `model_path` param, subscribes to
`vehicle_local_position` / `vehicle_attitude` / `sensor_combined`, ticks at
`CONTROL_HZ`, publishes on `constants.TOPIC_WIND_EST`. `enabled` param gates
publishing so baseline and treatment trials share one launch graph.

## Step 6 (verify live rate) — PARTIAL

- `colcon build --symlink-install --packages-select aerocanyon` succeeds.
- Live `ros2 topic hz /aerocanyon/wind_estimate` against running PX4 was
  **not** run — no PX4 in this environment.
- Substitute check: benchmarked the trained network's forward pass in
  isolation — mean 0.20 ms/call (~5000 Hz ceiling) on CPU, vs. the 20 ms
  budget for 50 Hz. Plenty of headroom; if the live check comes back slow
  it will point at a blocked callback, not the network, consistent with the
  plan's own troubleshooting note.

## Step 7 (commit) — DONE

Commit `c7b7cca`: `feat(aerocanyon): FO-PINN training pipeline and inference
node`, containing `train_pinn.py`, `fo_pinn_node.py`, `wind_estimator.pt`,
`docs/alpha_sweep.txt`.

## Verification run

- `colcon build --symlink-install --packages-select aerocanyon` — passes.
- `pytest src/aerocanyon/test/` — 49 passed, 1 pre-existing failure
  (`test_dryden_is_zero_mean_and_reproducible` in `test_canyon_field.py`,
  from an earlier task's `canyon_field.py`, unrelated to Task 8 — not
  touched by this work).

## Files touched

- `/home/parvu/ros2_pinn_sim/src/aerocanyon/aerocanyon/train_pinn.py` (new)
- `/home/parvu/ros2_pinn_sim/src/aerocanyon/aerocanyon/fo_pinn_node.py` (new)
- `/home/parvu/ros2_pinn_sim/src/aerocanyon/data/wind_estimator.pt` (new, trained on synthetic data — retrain before flight)
- `/home/parvu/ros2_pinn_sim/docs/alpha_sweep.txt` (new, synthetic-data sweep)
