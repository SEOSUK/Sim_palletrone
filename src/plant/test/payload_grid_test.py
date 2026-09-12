#!/usr/bin/env python3
"""Regression test for payload-grid mapping and CRN reuse."""
from monte_carlo_runner import load_yaml, parameter_row, repository_root, trial_plan

ROOT=repository_root()
config=load_yaml(ROOT/"src/palletrone_interfaces/bag/monte/diagnostics/payload_grid/config/grid_pilot.yaml")
model=load_yaml(ROOT/"src/palletrone_interfaces/config/model.yaml")
plan=trial_plan(config); assert len(plan)==125
rows=[parameter_row(t,model) for t in plan]
assert sorted({r["payload_pos_x"] for r in rows})==[-.3875,-.19375,0,.19375,.3875]
assert sorted({r["payload_pos_y"] for r in rows})==[-.3875,-.19375,0,.19375,.3875]
assert {r["payload_pos_z"] for r in rows}=={.56}
for replicate in range(5):
 q=[r for r in rows if r["replicate_id"]==replicate]
 assert len(q)==25 and len({(r["measurement_seed"],r["torque_seed"],r["force_seed"]) for r in q})==1
print("PASS: 25x5 grid, fixed z=0.56 m, and per-replicate CRN mapping")
