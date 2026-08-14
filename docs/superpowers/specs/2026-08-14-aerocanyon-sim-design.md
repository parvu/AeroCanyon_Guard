# AeroCanyon-Guard — Simulation & Development Environment Design

**Date:** 2026-08-14
**Status:** Approved, pending implementation plan
**Deadline:** ~2026-10-01 (UEFISCDI PED 2026 submission)

## Purpose

Produce simulation evidence for the AeroCanyon-Guard proposal: a tilt-rotor
VTOL traversing an urban canyon under wind disturbance, controlled by a
fractional-order physics-informed network with a control-barrier-function
safety filter, measurably outperforming the stock PX4 controller.

Three artifacts must exist by the deadline:

1. Baseline vs FO-PINN+CBF lateral deviation comparison (overlaid trajectories
   plus a bar chart).
2. CBF intervention time series — requested command, filtered command, barrier
   boundary.
3. A screen recording of a canyon transit.

This spec covers only the simulation substrate serving WP1 and WP2. The HIL
bench (WP3), the physical airframe (WP4), and the funded CFD sweeps (WP1
proper) each get their own spec later and plug into the interfaces defined
here.

## Scope decisions

| Decision | Choice | Rationale |
|---|---|---|
| Airframe | PX4 stock tiltrotor SITL | Transition logic already tuned; a custom TRFW SDF is weeks of work before anything flies |
| Wind source | Published urban-canyon profiles fitted to a 3D grid, replayed | No solver toolchain fits in 8 weeks; the funded project replaces this with real CFD |
| Control boundary | Outer loop only — PX4 keeps rate/attitude/mixer | Makes the baseline comparison apples-to-apples and avoids retuning PX4 |
| New code location | One new `ament_python` package, `aerocanyon` | Existing `phy_ai_simulation` is a CMake/C++ package; mixing Python nodes there is awkward |

Explicitly out of scope: replacing PX4's controller, custom airframe SDF, BEM
rotor modelling, transient CFD, multi-vehicle, hardware.

## Existing assets reused

- PX4 SITL + Gazebo Harmonic toolchain (documented in the repo README)
- `dve_wind_arena.sdf` and its explicit plugin set — the new canyon world is a
  copy with buildings added
- `WindEffects` plugin already wired, and `enable_wind` already patched into
  the PX4 vehicle model
- `phy_ai_simulation/physics_bridge` — IMU to `/pinn/input_state`
- `px4_teleop` offboard plumbing — arm, offboard-mode engage, `TrajectorySetpoint`
  streaming. The controller node reuses this pattern.
- `px4_msgs` submodule

## Architecture

```
canyon_field.py  (offline, run once)
        │  writes wind_grid.npy
        ▼
   wind_field_node ──gz topic──> WindEffects ──> force on airframe
        │
        │ ground-truth wind at drone position
        │ (free training labels — we generated the field)
        ▼
PX4 SITL ──/fmu/out/vehicle_local_position, vehicle_attitude, sensor_combined──>
        │
        ▼
   fo_pinn_node ──> estimated wind force F̂_wind
        │
        ▼ feedforward acceleration
   cbf_filter ──> /fmu/in/trajectory_setpoint ──> PX4 inner loop
```

The stack is an outer-loop disturbance-rejecting setpoint generator. PX4's
inner loop is untouched, so a baseline trial is the same binary, same vehicle,
same wind — only the outer loop differs.

## Components

### `canyon_field.py` — offline wind field generator

Emits a 3D velocity grid `(nx, ny, nz, 3)` as `.npy` plus a small JSON of grid
origin and spacing. Composed from published urban-canyon aerodynamics:

- Log-law vertical profile using a roughness length `z0` from standard urban
  roughness tables
- Channeling speedup along the canyon axis
- Recirculation and separation zones behind building corners

Turbulence is *not* baked into the grid. Dryden turbulence is added at
inference time in `wind_field_node`, which keeps the stored field 3D rather
than 4D and lets turbulence intensity be varied per trial without regenerating.

Output is committed to the repo so trials are reproducible without rerunning
generation.

### `wind_field_node` — spatial wind replay

Gazebo's `WindEffects` plugin models a **globally uniform** wind. Rather than
write a new plugin, this node subscribes to the drone's position, looks up the
grid at that position, adds the Dryden turbulence increment, and republishes
the result to Gazebo's global wind topic at 50 Hz. The vehicle therefore
experiences the correct spatially-varying field.

```
# ponytail: single-vehicle only — the global wind topic is driven from one
# drone's position. Multi-vehicle needs a real per-link wind plugin.
```

It also publishes the ground-truth wind vector on a ROS topic, which is what
supervises PINN training.

Positions outside the grid clamp to the nearest cell rather than erroring.

### `urban_canyon.sdf` — world

Copy of `dve_wind_arena.sdf` with roughly six box buildings forming a corridor.
Boxes, not meshes: they give analytic distance queries for the CBF obstacle
barrier and cost nothing to render. Retains the full explicit plugin list —
adding any plugin to a PX4 world disables the implicit default set, so all of
`Physics`, `UserCommands`, `SceneBroadcaster`, `Sensors`, `Imu`,
`AirPressure`, `Magnetometer`, `NavSat`, `WindEffects` stay declared.

### `fo_pinn.py` / `fo_pinn_node` — fractional-order PINN

Extends the existing `PhysicsInformedDronePilot`.

**Output** is the wind disturbance force `F_wind ∈ ℝ³`, not motor PWM.
Predicting PWM directly is both unlearnable from this signal and redundant with
PX4's mixer.

**Fractional order** is implemented as Grünwald-Letnikov coefficients over a
ring buffer of the last N states:

```
w_k = (-1)^k · binomial(α, k),   k = 0 … N-1
```

The weighted sum of state history is concatenated to the network input. This is
the Caputo memory term from Objective 1, and it is the mechanism that damps
control jitter from corner gusts: the estimate cannot swing instantaneously
because it is anchored to weighted history. `α` and `N` are tunable per trial.

**Loss** has two terms:

- Supervised MSE against the ground-truth wind published by `wind_field_node`
- Physics residual `‖m·a_imu − T_body − m·g − F̂_wind‖²`, from rigid-body
  Newton-Euler

The residual is what makes this a PINN rather than a regressor, and it is the
term that survives to hardware, where ground truth is unavailable.

Training data comes from logged canyon transits. Inference runs at 50 Hz.

### `cbf_filter.py` — safety filter

A quadratic program over the 3-DOF acceleration setpoint, solved with `cvxpy`
and OSQP. Barriers:

- `h₁ = α_stall − α` — angle of attack, computed from the airspeed vector
  against the body x-axis
- `h₂` — tilt-rate limit
- `h₃` — distance to the nearest building face; boxes make this analytic, so no
  collision query is needed

Runs after the PINN and before publishing, at 50 Hz. If the QP is infeasible,
fall back to the last known-safe command and log the event — never publish an
unfiltered command.

This node alone produces the intervention figure.

### `controller_node` — the loop

Reuses the `px4_teleop` offboard pattern: arm, engage offboard, stream
setpoints. Per cycle: read PX4 state, get `F̂_wind` from the PINN, convert to a
feedforward acceleration, pass the combined setpoint through the CBF filter,
publish `TrajectorySetpoint`.

A `--baseline` flag bypasses both the PINN and the filter, publishing the raw
mission setpoint. This is how the two trials stay identical apart from the
outer loop.

### `run_trial.py` / `plot_results.py` — evaluation harness

`run_trial.py` flies one fixed mission — hover, transition, canyon traverse,
land — twice against the same wind seed, once baseline and once treatment,
logging to CSV. Gazebo runs headless so trials are unattended.

`plot_results.py` reads the CSVs and emits the comparison and intervention
figures.

## Error handling

- Grid lookups outside bounds clamp rather than raise
- QP infeasibility falls back to the last safe command and logs
- Loss of PX4 telemetry for more than a configurable timeout drops to hover
  setpoints
- Trial runs that fail to arm or fail to transition abort with a non-zero exit
  and a named reason, so a slipped trial is never silently treated as data

## Testing

Per the repo's existing convention, non-trivial logic leaves one runnable check
behind — an `assert`-based `__main__` self-check or a small `test_*.py`. No
frameworks or fixtures.

- `canyon_field.py`: assert the generated field satisfies the log-law at a
  sample column and that speedup appears in the canyon throat
- `wind_field_node`: assert grid interpolation returns known values at cell
  centres and clamps outside bounds
- `fo_pinn.py`: assert the GL coefficients match known values for a couple of
  `α`, and that the physics residual is near zero for a hand-constructed
  consistent state
- `cbf_filter.py`: assert a command violating a barrier is modified, a safe
  command passes through unchanged, and infeasibility returns the fallback

The end-to-end check is a trial run producing a non-empty CSV with the drone
reaching the mission end waypoint.

## Schedule

| Week | Deliverable |
|---|---|
| 1 | Canyon world with box buildings; stock tiltrotor flies through it |
| 2 | `canyon_field.py` grid and `wind_field_node`; wind visibly displaces the drone |
| 3 | Mission script and baseline trial logging cleanly |
| 4–5 | FO-PINN: data collection, training, offline validation of `F̂_wind` |
| 6 | `cbf_filter` and controller node closing the loop |
| 7 | Trials, tuning, the two figures |
| 8 | Video, README, buffer |

**Principal risk:** weeks 4–5. If the PINN underperforms by week 6, the
fallback headline figure is CBF-only versus baseline. That remains a real,
publishable preliminary result and preserves the submission.

## Interfaces for later work packages

These boundaries are chosen so downstream work packages replace parts without
restructuring:

- **WP1 (real CFD):** replaces `canyon_field.py`'s output only. The `.npy` grid
  plus origin/spacing JSON is the contract; `wind_field_node` is unchanged.
- **WP3 (HIL / Jetson):** `fo_pinn_node` and `cbf_filter` are ROS nodes with
  topic interfaces, so they move to edge hardware without touching the
  simulation side.
- **WP4 (custom TRFW):** swapping the airframe changes the SDF, the PX4
  airframe config, and the mass/inertia constants in the physics residual.
  Nothing else.
