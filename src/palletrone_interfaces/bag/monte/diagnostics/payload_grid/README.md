# Modeled 500 g payload-position pilot

This is a diagnostic position map, not a physical mounting or safety envelope.
No CAD-derived mount boundary exists in the repository. Payload mass is 0.5 kg,
z is fixed at 0.56 m, and x/y use `0.31 * [-1.25,-0.625,0,0.625,1.25]`.

Mission-only initializes the plant at the configured 0.7 m hover with zero
velocity/rate, enables MOCE after a 0.5 s initialization interval, holds hover
for 15 s, and then executes the unchanged 60 s Lissajous reference. Controller,
DOB, MOCE, and plant parameters are unchanged.

```bash
python3 src/plant/test/monte_carlo_runner.py \
  --config src/palletrone_interfaces/bag/monte/diagnostics/payload_grid/config/grid_pilot.yaml \
  --output-dir src/palletrone_interfaces/bag/monte/diagnostics/payload_grid
python3 src/plant/test/mission_analysis.py \
  --logs src/palletrone_interfaces/bag/monte/diagnostics/payload_grid/logs/all \
  --results src/palletrone_interfaces/bag/monte/diagnostics/payload_grid/results/monte_carlo_results.csv \
  --output src/palletrone_interfaces/bag/monte/diagnostics/payload_grid/results/mission_metrics.csv
```

Full logs are ignored by Git. Summary tables and figures are retained.
