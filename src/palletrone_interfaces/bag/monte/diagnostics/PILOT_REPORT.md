# v6 failure-mechanism pilot report

Commit audited: `b3e5405` on branch `palletrone`. The tracked v1-v6 files and
the frozen `monte/results` directory were not modified.

## Repository audit

- Flight loss is a numerically completed run whose final Complete-window
  altitude is at or below `controller.adaptation_min_height = 0.4 m`. NaN/Inf,
  divergence, timeout, crash, missing MOCE edge, and incomplete 80 s evaluation
  are numerical failures. Tilt is diagnostic only.
- Existing logs contain desired force/torque, attitude/state/reference, DOB,
  raw and filtered MOCE estimates/rate, motor speed command, servo command,
  actual motor force, actual servo angle, residual force/torque, and truth
  acceleration. Existing summaries contain only a fleet-wide maximum motor
  utilization, maximum servo angle, and a combined limit-contact time.
- Missing diagnostics were per-rotor utilization/contact, per-servo angle/rate
  and contact, separate contact durations, realized and allocatable wrench,
  wrench residuals, allocation rank/SVD/conditioning, and pre-loss CRN pairs.
- Motor thrust is clipped to 50 N both before delay and by the MuJoCo actuator.
  It has 10 ms delay and 17 ms first-order lag. Servo command and joint are
  limited to +/-0.7 rad; command delay is 50 ms and the joint has PD, damping,
  armature, and a +/-2 Nm torque limit. There is no explicit servo rate limiter.
- A1 is rebuilt each control update from measured servo angles and estimated
  CoM. A2 is rebuilt from allocated thrust and estimated CoM. Thus both are
  time-varying. Raw condition numbers mix N and Nm and A2 also scales with
  thrust; diagnostics therefore report row/column-equilibrated SVD plus
  physical residuals in N and Nm.
- The priority variables are loss time, attitude error, servo angle/rate,
  desired-versus-realized torque, per-rotor thrust, normalized A1 minimum
  singular value, and DOB/CoM state, in that causal order around takeoff.

## Baseline pilot

Five CRN replicates used seeds `7000 + 3*j + {0,1,2}` at each scale.

| CoM scale | Loss / 5 | P_loss | median max motor util. | median max servo angle (rad) |
|---:|---:|---:|---:|---:|
| 1.20 | 0 | 0.00 | 0.465 | 0.681 |
| 1.22 | 0 | 0.00 | 0.471 | 0.680 |
| 1.24 | 2 | 0.40 | 0.474 | 0.687 |
| 1.25 | 3 | 0.60 | 0.480 | 0.703 |
| 1.26 | 3 | 0.60 | 0.482 | 0.703 |

This small pilot reproduces a sharp transition, not the population rates. In
particular, the first five fine-sweep seeds happen to have 0/5 losses at 1.20.
The retained repository summary is header-only, so the published README counts
could not be re-analysed trial-by-trial.

All eight pilot losses occurred during initial ascent, 5.26 to 11.66 s before
MOCE activation. Consequently the transition is not caused by MOCE adaptation
state divergence in these trials. No motor saturation occurred. Servo contact
was zero or very brief (0 to 0.31% of the full run), but angle demand approached
the physical bound during the decisive ascent.

For CRN replicate 2, scale 1.22 succeeds and 1.24 fails. During the common
pre-loss interval, the failed case reaches 0.60 rad attitude error and descends
through 0.4 m; motor utilization stays below 0.418. Servo angle reaches 0.689
rad, realized torque residual reaches 4.95 Nm, and normalized A1 conditioning
remains modest (condition <=1.31). The successful case also reaches 0.686 rad
servo angle, showing proximity to the limit is a marker, not by itself proof of
the mechanism.

## Paired ablations

| Variant | losses at 1.20 / 1.22 / 1.24 / 1.25 / 1.26 | total |
|---|---|---:|
| baseline | 0 / 0 / 2 / 3 / 3 | 8/25 |
| residual torque OU OFF | 0 / 0 / 0 / 2 / 1 | 3/25 |
| idealized servo delay/dynamics | 0 / 0 / 2 / 2 / 3 | 7/25 |
| servo angle limit relaxed to 1.0 rad | 0 / 0 / 2 / 3 / 3 | 8/25 |

The strongest pilot evidence is for an interaction between CoM-induced ascent
torque demand and calibrated stochastic torque residual. Removing torque OU
eliminates five of eight paired losses. Simple motor authority is contradicted
by <50.2% median maximum utilization and zero saturation. A raw servo angle
shortage is not supported: relaxing the angle bound does not change the loss
counts, while idealizing servo dynamics changes only one outcome. Allocation
rank/conditioning also does not collapse. Residual force, measurement noise,
motor lag, nonlinear feasible-set geometry, takeoff/contact dynamics, and
controller/DOB transient interaction remain unexcluded.

## Real-flight data audit

The model explicitly identifies a 0.5 kg Setup-2 payload at FRD
`[0.31,-0.31,-0.56] m`, converted to FLU `[0.31,0.31,0.56] m`. README states
that v1-v5 rigid body, actuator, noise, and residual parameters were derived or
calibrated against real flight data for this model. Therefore the Setup-2 data
used for those values is calibration data and may only be called an in-sample
or calibration-consistency check.

Tracked real-flight-looking logs exist under `bag/paper_plot`: static payload,
varying payload, and PID/DOB/L1/MOCE comparison logs. However, no tracked
manifest links a CSV to payload mass, placement, flight ID, or calibration
split. The 3rd-experiment MOCE log contains an estimated offset but its script
does not declare mass/placement. The repository therefore does **not** establish
that it is a 500 g condition or a held-out flight. No defensible independent
500 g cross-validation set can currently be identified. A manifest from the
experimental records is required.

## Recommended next experiments

1. Run 50 CRN replicates at scales `[1.20,1.22,1.23,1.24,1.25,1.26]` for
   baseline and torque-OU-off. Add torque-OU amplitude factors 0.5 and 1.5 to
   test monotonicity rather than relying only on OFF.
2. Run 20 diagnostic replicates at `[1.22,1.24,1.26]` for force-OU-off,
   measurement-noise-off, motor-dynamics-ideal, and servo-dynamics-ideal.
3. Add a takeoff-phase-only analysis and a counterfactual using true CoM in the
   allocator (diagnostic only) to separate initial estimator bias from physical
   geometry. Do not retune the frozen controller.

For the later 500 g placement study, keep mass and the measured mounting Z
fixed, sweep an explicitly supplied physically mountable x-y grid, use CRN at
every grid point, and output raw loss probability, attitude RMS/peak, motor and
servo margins, torque residual, and normalized allocation SVD maps. Until the
mount geometry is documented, label it a **modeled payload-position envelope**.
Do not apply a safe/unsafe probability threshold; publish raw probabilities and
confidence intervals for user-selected criteria.

Figures: `pilot/baseline_scale_metrics.png` and
`pilot/representative_pair_preloss.png`. Detailed reproducible tables are under
each variant's `results` directory; full logs are locally retained but ignored
by Git.
