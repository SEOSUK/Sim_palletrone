# Palletrone Flight-Data-Driven MuJoCo Validation

The Palletrone MuJoCo model is progressively calibrated against real flight data.
Each version adds one major source of model discrepancy so that its contribution
to the real-vs-simulation gap can be evaluated independently.

The overall workflow is:

```text
Real Flight Data
      ↓
Model Identification
      ↓
Deterministic Calibration
      ↓
Measurement Uncertainty
      ↓
Residual Dynamics
      ↓
Monte Carlo Robustness Validation
```

---

# Version History

## v1 — Rigid-Body and Actuator Identification

The first calibrated model replaces idealized plant parameters with
flight-data-derived deterministic parameters.

Main updates:

- separated physical plant inertia from controller/DOB/MOCE nominal inertia
- calibrated unloaded vehicle inertia
- modeled the 0.5 kg Setup-2 payload as a point mass
- preserved the fixed body-frame origin while generating the physical CoM offset
- identified static thrust effectiveness
- identified motor delay and first-order dynamics
- identified servo command-to-angle dynamics

The payload-induced CoM offset is computed as

```math
\mathbf c
=
\frac{m_p}{m_v+m_p}\mathbf r_p.
```

Rigid-body inertia was identified from

```math
\boldsymbol{\tau}
=
\mathbf J\dot{\boldsymbol{\omega}}
+
\boldsymbol{\omega}\times
\mathbf J\boldsymbol{\omega},
```

using filtered flight angular-rate data and least-squares fitting.

Representative parameters:

```yaml
nominal_inertia: [0.0768, 0.0871, 0.113]

motor:
  delay_s: 0.010
  time_constant_s: 0.017

servo:
  dc_gain: 0.994
  delay_s: 0.050
```

The actuator dynamics were approximated using a first-order-plus-dead-time model,

```math
G(s)
=
\frac{K}{\tau s+1}e^{-T_ds}.
```

---

## v2 — Slow Common-Mode Thrust Effectiveness Drift

Real flight logs showed a repeatable decrease in common thrust effectiveness
during flight.

The instantaneous effectiveness proxy was estimated from

```math
\eta_T(t)
\approx
\frac{m\|\mathbf a_{\mathrm{specific}}(t)\|}
{\|\mathbf F_{\mathrm{cmd}}(t)\|}.
```

A linear trend across repeated flight logs was fitted as

```math
\eta_T(t)
\approx
0.6966 - 8.44\times10^{-4}t.
```

The implemented rotor thrust is

```math
T_{i,\mathrm{actual}}
=
\eta_{\mathrm{common}}(t)
\eta_i
T_{i,\mathrm{nominal}}.
```

This effect increased late-flight position tracking error toward the real-flight
level, but did not explain the persistent rotational activity observed in hardware.

---

## v3 — Measurement Uncertainty

Flight-data-derived high-frequency uncertainty was added to the simulated
state-estimator outputs.

For each logged state signal,

```math
\mathbf r_y(t)
=
\mathbf y(t)
-
\mathrm{LPF}_{2\,\mathrm{Hz}}\{\mathbf y(t)\},
```

and the effective uncertainty level was estimated as

```math
\boldsymbol{\sigma}_y
=
\mathrm{std}\big(\mathbf r_y(t)\big).
```

Representative standard deviations:

```yaml
position_std: [0.00019, 0.00022, 0.00017]
velocity_std: [0.00473, 0.00500, 0.00368]
attitude_std: [0.00071, 0.00067, 0.00044]
gyro_std:     [0.0236,  0.0215,  0.0174]
```

These values represent the effective high-frequency uncertainty of the logged
state-estimator outputs rather than intrinsic sensor noise densities.

```text
Angular-acceleration command RMS
v2:   0.545
v3:   2.809
Real: 5.295
```

---

## v4 — Colored Physical Torque Residual

The remaining rotational discrepancy was modeled as a body-frame
Ornstein-Uhlenbeck torque process.

The torque residual was characterized from the converged flight interval using

```math
\mathbf r_\tau
\approx
\hat{\mathbf d}_{\tau,\mathrm{real}}
-
\hat{\mathbf d}_{\tau,\mathrm{sim}},
```

with its standard deviation and autocorrelation used as calibration targets.

The OU process is

```math
d_{k+1}
=
a d_k
+
\sigma\sqrt{1-a^2}\,\xi_k,
\qquad
a=e^{-\Delta t/\tau}.
```

Final gray-box calibrated parameters:

```yaml
residual_torque:
  enabled: true
  frame: body
  std_nm: [0.0891, 0.0807, 0.0323]
  correlation_time_s: [0.296, 0.586, 0.616]
```

The physical-input parameters were calibrated against DOB-output statistics,
not against position or attitude tracking error.

```text
                    v3       v4       Real
Attitude RMS        0.0027   0.0143   0.0203
Peak attitude       0.0065   0.0311   0.0428
DOB norm            0.407    1.674    1.698
```

---

## v5 — Colored Physical Force Residual

After v4, rotational fidelity was close to real flight while position tracking
remained substantially cleaner than the hardware result.

The physical force proxy was reconstructed as

```math
\mathbf F_{\mathrm{actual}}
=
m\,\mathbf a_{\mathrm{specific}},
```

and the residual force was defined as

```math
\mathbf r_F
=
\mathbf F_{\mathrm{actual}}
-
\mathbf F_{\mathrm{predicted}},
```

where $\mathbf F_{\mathrm{predicted}}$ includes the identified thrust
effectiveness, motor delay, motor first-order lag, and measured servo geometry.

The unexplained residual magnitude was initialized using variance subtraction,

```math
\sigma_{\mathrm{missing}}
=
\sqrt{
\sigma_{\mathrm{real}}^2
-
\sigma_{\mathrm{v4}}^2
}.
```

A zero-mean body-frame OU force model was then gray-box calibrated using the
residual-force STD and ACF statistics.

Final parameters:

```yaml
residual_force:
  enabled: true
  frame: body
  std_n: [0.514, 1.266, 0.981]
  correlation_time_s: [0.25, 0.62, 0.27]
```

Real and simulated residual-force statistics were closely matched:

```text
Residual-force STD [N]

Real : [0.449, 1.128, 0.878]
v5   : [0.443, 1.088, 0.883]
```

The Complete-window position RMS changed as follows:

```text
v4   : 0.05269 m
v5   : 0.07057 m
Real : 0.11792 m
```

Thus, the colored force residual explained approximately 27% of the remaining
v4-to-real position gap while leaving the calibrated rotational response almost
unchanged.

The current v5 model is treated as a **flight-data-calibrated validation
candidate**, rather than a uniquely identified physical truth.

---

## v6 — CRN Monte Carlo Robustness Validation

The flight-data-calibrated v5 model was frozen as the baseline for the final
Monte Carlo robustness validation. No parameter-uncertainty randomization was
performed: payload mass, actuator dynamics, thrust-effectiveness drift,
controller/DOB/MOCE settings, inertias, and calibrated residual-process
parameters remained fixed. Stochastic variation came only from independent
measurement-noise, torque-OU, and force-OU realizations. Payload-offset/CoM-bias
severity was the primary experimental axis.

The validation proceeded in four stages:

- Stage A: 10-trial pipeline validation
- Stage B: 50-trial stochastic baseline
- Stage C: 120-trial coarse CoM sweep
- Final: 300-trial Common Random Numbers (CRN) fine sweep

The final sweep used 50 paired replicates at each scale:

```text
com_scale: [1.00, 1.05, 1.10, 1.15, 1.20, 1.25]
trials per scale: 50
total trials: 300
common random numbers: enabled
```

Numerical completion was recorded separately from physical flight success. A
numerically completed trajectory was considered airborne when
`terminal_altitude > adaptation_min_height`, where
`adaptation_min_height = 0.4 m`; otherwise it was classified as
`loss_of_flight`.

| CoM scale | Loss / 50 | P_loss |
|---:|---:|---:|
| 1.00 | 0 | 0.00 |
| 1.05 | 0 | 0.00 |
| 1.10 | 0 | 0.00 |
| 1.15 | 1 | 0.02 |
| 1.20 | 4 | 0.08 |
| 1.25 | 30 | 0.60 |

No physical loss was observed through scale 1.10; the first loss appeared at
1.15, followed by 8% loss at 1.20 and 60% at 1.25. The empirical probabilistic
robustness transition is therefore concentrated between scales 1.20 and 1.25.
An exploratory logistic fit placed the 50% loss scale at 1.2429; the empirical
proportions remain the primary result.

Conditioned on trajectories that remained airborne, tracking performance stayed
comparatively stable through scale 1.25. The dominant robustness phenomenon was
therefore not gradual tracking degradation, but a stochastic normal-flight /
catastrophic-loss bifurcation under the calibrated stochastic v5 model.

These results describe a probabilistic robustness boundary in a
flight-data-calibrated simulation model. They are not a guaranteed boundary or
an experimentally measured physical-system failure probability.

Parameter-uncertainty randomization and alternative sampling designs such as
Latin hypercube sampling remain outside the scope of this completed validation.

---

# Final Roadmap

```text
v1  Rigid-body + actuator identification
 ↓
v2  Thrust-effectiveness drift
 ↓
v3  Measurement uncertainty
 ↓
v4  Colored torque residual
 ↓
v5  Colored force residual
 ↓
v6  CRN Monte Carlo robustness validation
```

The result is a reproducible, flight-data-calibrated simulation workflow for
Monte Carlo robustness validation under the modeled stochastic conditions.
