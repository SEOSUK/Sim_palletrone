# v6 CRN CoM-bias Monte Carlo validation

## Purpose and frozen baseline

This pipeline evaluates MOCE robustness for payload transportation using the
flight-data-calibrated v5 plant. CoM bias is treated as the primary experimental
axis, while stochastic flight uncertainty is treated as nuisance variability.
Monte Carlo is not used to further fit the simulator to real-flight tracking
metrics.

The runner reads the existing v5 `model.yaml` and changes only the payload arm
and the three RNG seeds. Controller/DOB/MOCE parameters, inertias, actuator
dynamics, effectiveness drift, measurement uncertainty, and calibrated torque
and force OU parameters remain unchanged. Parameter randomization was explicitly
disabled throughout the completed validation; controller parameters are
intentionally absent from the uncertainty schema.

## Layout

```text
monte/
  config/monte_carlo_config.yaml  experiment definition
  results/monte_carlo_results.csv one summary row per trial
  results/monte_carlo_parameters.csv exact trial inputs
  logs/failures/                  retained failed trajectories
  logs/representatives/           best/median/worst successful trajectories
  monte_carlo_analyzer.m          population statistics and figures
```

`palletrone_logger.m` remains the detailed single-run diagnostic tool.

## Build and run

Build once from the repository root, then run any number of trials without
rebuilding:

```bash
colcon build --symlink-install --packages-select palletrone_interfaces palletrone_controller plant
python3 src/plant/test/monte_carlo_runner.py
```

Use `--config PATH` for another experiment file. `--plan` prints the complete
trial/seed mapping without running MuJoCo. The native `drift_validation` program
runs headless (there is no GLFW viewer in this executable). Execution is serial,
and `process_timeout_s` applies independently to every subprocess.

Modes are `pipeline_debug` (scale 1.0, default 10 trials),
`stochastic_repeatability` (scale 1.0, default 50 trials), and `com_sweep`.
For the latter, `com_scales` and `trials_per_level` define the population.
The completed final experiment used `monte_carlo_fine_config.yaml`: 50 CRN
replicates at each of scales 1.00, 1.05, 1.10, 1.15, 1.20, and 1.25, for 300
trials in total.

## CoM and seed policy

For scale `s`, the physical payload arm in simulator FLU is

```text
r_payload_trial = s * [0.31, 0.31, 0.56] m
c_true = payload_mass / (vehicle_mass + payload_mass) * r_payload_trial
```

Scaling the payload arm is a controlled severity sweep rather than a statement
that one physical payload slides along this line. It preserves direction and
gives a controlled axis for evaluating increasing CoM-bias severity.

With `common_random_numbers: false`, trial `i` uses
`base_seed + 3*i + {0,1,2}` for measurement, torque OU, and force OU respectively.
With CRN enabled, replicate `j` uses `base_seed + 3*j + {0,1,2}` at every CoM
scale. Both CSVs contain `replicate_id`, allowing paired scale differences.
Different replicates remain independent. These are separate C++ RNG engines.
The same config, baseline,
base seed, and trial IDs reproduce identical parameters and numerical metrics
(wall-clock `runtime_s` naturally varies). Changing the base seed changes all
three stochastic histories.

## Metrics, status, and resume

The first MOCE rising edge defines evaluation zero as `edge - 5 s`. Metrics in
the summary use Complete `[60,80]`; the simulator also executes the historical
Overall `[0,80]`, Transient `[5,60]`, and Trajectory transient `[20,60]` windows.
The CSV contains tracking RMS values, attitude peak, angular-acceleration
command RMS, DOB norm, final CoM error, utilization/saturation observations,
status, failure reason, and runtime. `numerical_success` records process/data
completion independently of `flight_success`; the backward-compatible `success`
field is the overall flight result. Normal flights report `normal_completion`,
while numerically completed terminal losses report `loss_of_flight`. The
parameters CSV records the exact fixed
v5 plant values and seeds used for each trial.

Physical status uses the existing controller airborne/contact boundary,
`controller.adaptation_min_height` (currently 0.4 m above the Z=0 floor). The
experiment reference stays at or above 0.625 m, so a Complete-window terminal
altitude below that gate is a loss of flight. Terminal altitude and body-Z tilt
are logged for audit; tilt is diagnostic, not another tuned threshold.

NaN/Inf, crashes, timeouts, and incomplete experiments fail immediately. Other
limits are applied only when their config value is non-null. In particular,
`moce_convergence_time` is `NaN` unless `max_com_error_m` supplies an explicit
definition; when supplied, it is the first post-MOCE time after which the CoM
error remains within that bound. Physical-limit contact is always measured as
`saturation_time`, but is not automatically failure without a configured limit.

With `resume: true`, numerically completed trial IDs are skipped, including
physical-loss cases that should not be silently rerun. Numerical failures are
retried, and each completed trial atomically updates the summary. Historical
pre-classifier summaries can be upgraded with `--reclassify-results PATH`.
Because their full trajectories were not retained, migration uses exact zero
Complete-window DOB activity as a legacy proxy for the airborne gate being
inactive; new runs always classify direct terminal altitude.

## Trajectory retention and MATLAB analysis

All full trajectories are temporary by default. Physical-loss and numerical-
failure logs are retained when enabled. For each CoM level, overall-successful
trials created in the current run are
ranked by Complete-window position RMS; minimum, nearest sample median, and
maximum are copied as best, median, and worst-success representatives. Retained
logs may be gzip-compressed as `.csv.gz`; trajectory files are git-ignored and
`.gitkeep` preserves the directories.

## Final CRN fine-sweep result

All 300 trials completed numerically. Physical loss counts were 0, 0, 0, 1, 4,
and 30 out of 50 at scales 1.00 through 1.25 respectively, giving empirical
loss probabilities of 0.00, 0.00, 0.00, 0.02, 0.08, and 0.60. The probabilistic
robustness transition under the calibrated stochastic v5 model is concentrated
between scales 1.20 and 1.25; an exploratory logistic fit estimated the 50%
loss scale as 1.2429.

Airborne-only tracking remained comparatively stable through scale 1.25. The
primary observed phenomenon was a stochastic normal-flight / catastrophic-loss
bifurcation rather than smooth tracking degradation. These simulation results
are not a guaranteed physical-system robustness boundary.

In MATLAB, run `monte_carlo_analyzer` from this directory (or add it to the
path). It reads `results/monte_carlo_results.csv`, prints numerical completion,
physical success and loss rates plus Q90/Q95 statistics, and adds physical loss
probability as the main robustness plot.

For reproducibility checks, compare result files after excluding `runtime_s`;
use `--plan` to compare exact parameter mappings. Existing plant tests protect
the fixed v5 parameters and independent RNG behavior. A failed subprocess is
handled per trial, so subsequent trials continue. The headless runner uses the
same `PlantModel`, controller, command generator, and event-based evaluation as
the v5 validation executable.
