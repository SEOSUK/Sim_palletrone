# v6 failure-mechanism diagnostics

This directory is intentionally separate from the frozen `monte/results` and
does not modify v1-v6 data. `diagnostic_runner.py` reuses the production v6
runner and post-processes its existing append-only trajectory fields. It never
changes controller behavior.

Run the baseline pilot first:

```bash
python3 src/plant/test/diagnostic_runner.py \
  --config src/palletrone_interfaces/bag/monte/diagnostics/config/pilot.yaml \
  --variants baseline
```

After inspecting `pilot/baseline/results/diagnostic_metrics.csv`, run only the
mechanism-specific variants supported by the baseline evidence. For example:

```bash
python3 src/plant/test/diagnostic_runner.py \
  --config src/palletrone_interfaces/bag/monte/diagnostics/config/pilot.yaml \
  --variants motor_limit_relaxed servo_dynamics_ideal residual_torque_off
```

The five scales and five replicates are deliberately a pilot, not a probability
estimate suitable for a safety claim. All variants retain the same three seeds
per replicate. The measurement, torque-OU, and force-OU generators are separate
C++ engines, so disabling one process does not shift either of the other random
sequences. A disabled process naturally has no meaningful self-pairing sequence.

`allocation_a1_*` and `allocation_a2_*` use iterative row/column equilibration
before SVD. Raw conditioning would mix force and moment units and, for A2,
depend strongly on thrust magnitude. The normalized singular values are useful
as relative geometry/numerical diagnostics; they are not physical actuator
margins. Physical allocation and realized-wrench residuals are reported in N
and Nm alongside them.

Loss time is the last downward crossing of the existing 0.4 m airborne gate
after MOCE begins when the terminal sample remains below the gate. The exported
representative pair contains the configured five seconds before that event.
There is no explicit servo rate limiter in the v5 plant, so the corresponding
contact fraction is reported as NaN; delay, joint PD, torque limit, damping, and
armature create servo dynamics instead.

## Mission-phase sweep

Build the current source, run the explicit MOCE-on diagnostic sequence, then
classify and analyze only Lissajous-eligible trials:

```bash
colcon build --symlink-install --packages-select palletrone_interfaces palletrone_controller plant
python3 src/plant/test/monte_carlo_runner.py \
  --config src/palletrone_interfaces/bag/monte/diagnostics/config/mission_sweep.yaml \
  --output-dir src/palletrone_interfaces/bag/monte/diagnostics/mission_sweep
python3 src/plant/test/mission_analysis.py \
  --logs src/palletrone_interfaces/bag/monte/diagnostics/mission_sweep/logs/all \
  --results src/palletrone_interfaces/bag/monte/diagnostics/mission_sweep/results/monte_carlo_results.csv \
  --output src/palletrone_interfaces/bag/monte/diagnostics/mission_sweep/results/mission_metrics.csv
```

The Lissajous start is detected from the logged MOCE-Z event; its end is the
configured 60 s duration. In the current sequence these are evaluation times
20 and 80 s. A trial below the existing 0.4 m airborne gate at the start is
`pre_mission_failure` and is excluded from mission denominators.
