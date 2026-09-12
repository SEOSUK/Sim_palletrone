# 500 g modeled CoM-bias mission-phase pilot

## Window and classification

The automated sequence starts with 5 s minimum-jerk ascent and 30 s hover.
MOCE starts 15 s into hover (raw simulation time 20 s), and Lissajous starts at
raw time 35 s. Because the historical evaluation origin is five seconds before
MOCE, Lissajous occupies exactly `[20,80]` and the final 20 s is `[60,80]`.

The analyzer does not assume the start value: it detects the logged MOCE-Z
rising edge, then uses `automated_experiment.lissajous_duration_s` for the end.
At the start event, altitude above the existing 0.4 m airborne gate makes a
trial mission-eligible. An eligible trial crossing downward through the gate is
a mission loss. Tracking errors do not affect classification.

The checked-in source currently defines its normal automated sequence as
MOCE-off, whereas historical v6 logs were made by an older MOCE-on binary. The
new `mission_moce` executable argument makes the diagnostic behavior explicit
without changing default runner or command behavior.

## Existing pilot reanalysis

| scale | eligible | pre-mission | mission loss | median position RMS (m) | median attitude RMS (rad) |
|---:|---:|---:|---:|---:|---:|
| 1.20 | 5 | 0 | 0 | 0.0760 | 0.0244 |
| 1.22 | 5 | 0 | 0 | 0.0761 | 0.0250 |
| 1.24 | 3 | 2 | 0 | 0.0778 | 0.0258 |
| 1.25 | 2 | 3 | 0 | 0.0766 | 0.0254 |
| 1.26 | 2 | 3 | 0 | 0.0766 | 0.0257 |

The 17 eligible trajectories show no material position-RMS trend and no
mission loss. Samples above 1.22 are too sparse for population conclusions.

## New mission sweep

All values below are medians over eligible trials in `[20,80]`.

| scale | eligible | pre | mission loss | pos RMS m | att RMS rad | peak att rad | motor util. | servo angle rad | servo rate rad/s | realized torque residual RMS Nm | CoM error RMS m | norm. sigma min |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.00 | 5 | 0 | 0 | 0.0760 | 0.0197 | 0.0578 | 0.415 | 0.447 | 3.06 | 0.337 | 0.0230 | 0.845 |
| 1.10 | 5 | 0 | 0 | 0.0760 | 0.0217 | 0.0639 | 0.429 | 0.494 | 3.17 | 0.355 | 0.0244 | 0.830 |
| 1.20 | 5 | 0 | 0 | 0.0760 | 0.0244 | 0.0780 | 0.446 | 0.556 | 3.33 | 0.375 | 0.0256 | 0.814 |
| 1.25 | 2 | 3 | 0 | 0.0766 | 0.0254 | 0.0916 | 0.457 | 0.556 | 3.53 | 0.389 | 0.0271 | 0.782 |
| 1.30 | 1 | 4 | 0 | 0.0782 | 0.0285 | 0.1189 | 0.470 | 0.586 | 3.99 | 0.415 | 0.0256 | 0.745 |
| 1.35 | 0 | 5 | 0 | — | — | — | — | — | — | — | — | — |

There are 18 eligible, 12 pre-mission failures, and zero mission losses. Scale
1.35 cannot be evaluated for mission robustness with this sequence because no
trial reaches Lissajous airborne. Scale 1.30 has only one eligible sample.

Position RMS remains essentially flat through 1.25. Before position tracking
changes, attitude RMS/peak, motor utilization, servo demand, realized torque
residual, and normalized allocation margin change gradually. From 1.00 to 1.25,
median peak attitude error rises about 59%, servo angle 24%, motor utilization
10%, and torque-residual RMS 15%, while normalized minimum singular value falls
about 7%. These are modeled control-margin trends, not physical safety limits.
CoM error changes modestly and is not presently the dominant trend.

No catastrophic loss occurs after Lissajous begins in any eligible pilot trial.
The current data therefore identify gradual attitude/control-effort degradation,
not a mission-phase loss boundary.

## Next full experiment and real-flight consistency

Concentrate a full CRN study on `[1.00,1.10,1.15,1.20,1.22,1.24,1.25]` with at
least 50 replicates per scale. Values above 1.25 yield too few eligible samples
under this unchanged takeoff sequence; report eligibility separately rather
than treating it as mission loss. Raw scatter, median, Q10/Q90, and bootstrap
confidence intervals should accompany mission metrics.

For a later physical x-y placement envelope at fixed measured Z, output raw
mission eligibility, mission-loss probability conditional on eligibility,
position/attitude RMS and peak, per-rotor utilization/saturation, servo
angle/rate/contact, realized torque residual, CoM error, and normalized
allocation singular value. Do not impose a safe/unsafe threshold.

Setup-2 real flight is calibration data, so comparisons are in-sample
consistency checks only. Common metrics are position RMS, attitude RMS/peak,
CoM estimate trajectory, actuator command/response, and auxiliary DOB norm.
Missing true realized thrust in hardware must be acknowledged when comparing
wrench residuals.
