#!/usr/bin/env python3
"""Non-invasive failure-mechanism diagnostics for v6 Monte Carlo logs.

The simulator/controller are not changed.  This driver creates temporary model
and experiment files, delegates execution to monte_carlo_runner.py, and derives
actuator, allocation, and wrench metrics from its append-only 124-column log.
"""

from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import json
import math
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[3]
MONTE_RUNNER = Path(__file__).with_name("monte_carlo_runner.py")


def load_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def deep_set(mapping: dict, dotted: str, value) -> None:
    node = mapping
    parts = dotted.split(".")
    for part in parts[:-1]:
        node = node[part]
    node[parts[-1]] = value


def normalized_svd(matrix: np.ndarray) -> tuple[float, float, float, int]:
    """Equilibrate rows/columns to avoid mixing N and Nm in a raw condition number."""
    scaled = matrix.copy()
    for _ in range(3):
        row = np.linalg.norm(scaled, axis=1)
        scaled = scaled / np.maximum(row[:, None], 1e-12)
        col = np.linalg.norm(scaled, axis=0)
        scaled = scaled / np.maximum(col[None, :], 1e-12)
    singular = np.linalg.svd(scaled, compute_uv=False)
    rank = int(np.count_nonzero(singular > singular[0] * 1e-10)) if singular[0] else 0
    condition = float(singular[0] / singular[-1]) if singular[-1] > 1e-12 else math.inf
    return condition, float(singular[-1]), float(singular[0]), rank


def geometry(model: dict, servo: np.ndarray, com: np.ndarray, thrust: np.ndarray):
    m = model["model"]
    arm, rotor_z, reaction = float(m["arm_xy"]), float(m["rotor_z"]), float(m["motor"]["reaction_ratio"])
    sx = np.array([1, -1, -1, 1]); sy = np.array([1, 1, -1, -1])
    tx = np.array([1, 1, -1, -1]) / math.sqrt(2)
    ty = np.array([-1, 1, 1, -1]) / math.sqrt(2)
    spin = np.array([1, -1, 1, -1])
    a1, a2, directions = np.zeros((4, 4)), np.zeros((4, 4)), []
    for i in range(4):
        direction = np.array([tx[i] * math.sin(servo[i]), ty[i] * math.sin(servo[i]), math.cos(servo[i])])
        position = np.array([sx[i] * arm, sy[i] * arm, rotor_z])
        moment = np.cross(position - com, direction) + spin[i] * reaction * direction
        a1[:, i] = [moment[0], moment[1], spin[i] * reaction * direction[2], direction[2]]
        tangent = np.array([tx[i], ty[i], 0.0]) * thrust[i]
        a2[:, i] = [tangent[0], tangent[1], np.cross(position - com, tangent)[2], spin[i] * thrust[i]]
        directions.append(direction)
    return a1, a2, np.asarray(directions), spin


def read_log(path: Path) -> tuple[np.ndarray, np.ndarray]:
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    times = np.asarray([float(row["time"]) * 1e-9 for row in rows])
    values = np.asarray([[float(row[f"data[{i}]"]) for i in range(124)] for row in rows])
    edge = np.flatnonzero((values[:-1, 64] <= .5) & (values[1:, 64] > .5))
    if edge.size == 0:
        raise ValueError(f"MOCE edge missing: {path}")
    return times - (times[edge[0] + 1] - 5.0), values


def analyze_log(path: Path, model: dict, control: dict, preloss_s: float) -> tuple[dict, dict[str, np.ndarray]]:
    t, v = read_log(path)
    # Include the complete pre-MOCE segment: transition cases can lose flight
    # during takeoff and would otherwise appear already down at evaluation zero.
    mask = t <= 80
    t, v = t[mask], v[mask]
    max_thrust = float(model["model"]["motor"]["max_thrust"])
    servo_limit = float(model["model"]["servo"]["limit_rad"])
    airborne_gate = float(control["controller"]["adaptation_min_height"])
    payload = model["model"]["payload"]
    true_com = float(payload["mass"]) / (float(model["model"]["vehicle_mass"]) + float(payload["mass"])) * np.asarray(payload["position"], float)
    motor = v[:, 116:120]
    servo = v[:, 120:124]
    rate = np.vstack((np.zeros(4), np.diff(servo, axis=0) / np.maximum(np.diff(t)[:, None], 1e-12)))
    motor_sat = motor >= max_thrust - 1e-7
    servo_contact = np.abs(servo) >= servo_limit - 1e-5
    # Actuator contacts use every 400 Hz sample. Allocation/wrench geometry is
    # evaluated at 40 Hz: far above its 0.5 Hz split filter and controller
    # dynamics, while keeping a multi-trial diagnostic pass tractable.
    analysis_stride = 10
    va, ta = v[::analysis_stride], t[::analysis_stride]
    a1_cond, a1_min, a1_max, a2_cond, a2_min, a2_max, ranks = [], [], [], [], [], [], []
    alloc_force, alloc_torque, real_force, real_torque = [], [], [], []
    for row in va:
        estimated_com = row[54:57]
        cmd_thrust = .02 * np.maximum(row[34:38], 0) ** 2
        cmd_angle = row[38:42]
        a1, a2, cmd_direction, spin = geometry(model, row[120:124], estimated_com, cmd_thrust)
        c1 = normalized_svd(a1); c2 = normalized_svd(a2)
        a1_cond.append(c1[0]); a1_min.append(c1[1]); a1_max.append(c1[2]); ranks.append(min(c1[3], c2[3]))
        a2_cond.append(c2[0]); a2_min.append(c2[1]); a2_max.append(c2[2])
        _, _, command_direction, spin = geometry(model, cmd_angle, estimated_com, cmd_thrust)
        _, _, actual_direction, spin = geometry(model, row[120:124], true_com, row[116:120])
        def wrench(thrust, direction, center):
            force = np.sum(thrust[:, None] * direction, axis=0)
            torque = np.zeros(3)
            sx = np.array([1, -1, -1, 1]); sy = np.array([1, 1, -1, -1])
            for i in range(4):
                position = np.array([sx[i]*float(model['model']['arm_xy']), sy[i]*float(model['model']['arm_xy']), float(model['model']['rotor_z'])])
                fi = thrust[i] * direction[i]
                torque += np.cross(position-center, fi) + spin[i]*float(model['model']['motor']['reaction_ratio'])*fi
            return force, torque
        af, at = wrench(cmd_thrust, command_direction, estimated_com)
        rf, rt = wrench(row[116:120], actual_direction, true_com)
        alloc_force.append(af); alloc_torque.append(at); real_force.append(rf); real_torque.append(rt)
    alloc_force, alloc_torque = np.asarray(alloc_force), np.asarray(alloc_torque)
    real_force, real_torque = np.asarray(real_force), np.asarray(real_torque)
    desired_force, desired_torque = va[:, 24:27], va[:, 27:30]
    downward = np.flatnonzero((v[:-1, 2] > airborne_gate) & (v[1:, 2] <= airborne_gate))
    loss_time = float(t[downward[-1] + 1]) if downward.size and v[-1, 2] <= airborne_gate else math.nan
    dt = np.diff(t, prepend=t[0]); dt[0] = np.median(np.diff(t))
    record = {
        **{f"motor_{i+1}_max_utilization": float(np.max(motor[:, i])/max_thrust) for i in range(4)},
        **{f"motor_{i+1}_saturation_fraction": float(np.sum(dt[motor_sat[:, i]])/np.sum(dt)) for i in range(4)},
        **{f"servo_{i+1}_max_abs_angle_rad": float(np.max(np.abs(servo[:, i]))) for i in range(4)},
        **{f"servo_{i+1}_max_abs_rate_rad_s": float(np.max(np.abs(rate[:, i]))) for i in range(4)},
        **{f"servo_{i+1}_angle_contact_fraction": float(np.sum(dt[servo_contact[:, i]])/np.sum(dt)) for i in range(4)},
        "motor_saturation_fraction": float(np.sum(dt[np.any(motor_sat, axis=1)])/np.sum(dt)),
        "servo_angle_contact_fraction": float(np.sum(dt[np.any(servo_contact, axis=1)])/np.sum(dt)),
        "servo_rate_limit_contact_fraction": math.nan,
        "servo_rate_limit_implemented": False,
        "max_servo_rate_rad_s": float(np.max(np.abs(rate))),
        "allocation_a1_condition_max": float(np.max(a1_cond)),
        "allocation_a1_min_singular_min": float(np.min(a1_min)),
        "allocation_a1_max_singular_max": float(np.max(a1_max)),
        "allocation_a2_condition_max": float(np.max(a2_cond)),
        "allocation_a2_min_singular_min": float(np.min(a2_min)),
        "allocation_a2_max_singular_max": float(np.max(a2_max)),
        "allocation_rank_min": int(np.min(ranks)),
        "allocation_force_residual_max_n": float(np.max(np.linalg.norm(desired_force-alloc_force, axis=1))),
        "allocation_torque_residual_max_nm": float(np.max(np.linalg.norm(desired_torque-alloc_torque, axis=1))),
        "realized_force_residual_max_n": float(np.max(np.linalg.norm(desired_force-real_force, axis=1))),
        "realized_torque_residual_max_nm": float(np.max(np.linalg.norm(desired_torque-real_torque, axis=1))),
        "loss_event_time_s": loss_time,
    }
    series = {"time": ta, "altitude": va[:, 2], "attitude_error": np.linalg.norm(va[:, 12:15]-va[:, 15:18], axis=1),
              "motor_utilization": np.max(motor[::analysis_stride], axis=1)/max_thrust, "servo_angle": np.max(np.abs(servo[::analysis_stride]), axis=1),
              "servo_rate": np.max(np.abs(rate[::analysis_stride]), axis=1), "a1_min_singular": np.asarray(a1_min),
              "a1_condition": np.asarray(a1_cond), "force_residual": np.linalg.norm(desired_force-real_force, axis=1),
              "torque_residual": np.linalg.norm(desired_torque-real_torque, axis=1),
              "com_error": np.linalg.norm(va[:, 54:57]-true_com, axis=1), "dob_norm": np.linalg.norm(va[:, 31:34], axis=1)}
    if math.isfinite(loss_time):
        keep = (ta >= loss_time-preloss_s) & (ta <= loss_time)
        series = {key: value[keep] for key, value in series.items()}
    return record, series


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fields, lineterminator="\n"); writer.writeheader(); writer.writerows(rows)


def run_variant(config_path: Path, config: dict, variant: dict, output: Path, executable: Path) -> None:
    baseline_path = (ROOT / config["paths"]["model"]).resolve()
    model = load_yaml(baseline_path)
    for key, value in variant.get("overrides", {}).items(): deep_set(model, key, value)
    run_config = json.loads(json.dumps(config))
    run_config["experiment"] = dict(config["experiment"])
    run_config["experiment"]["mode"] = "com_sweep"
    run_config["experiment"]["resume"] = True
    run_config["logging"] = {"save_all_full_csv": True, "save_failures": True, "save_representatives": False}
    with tempfile.TemporaryDirectory(prefix="palletrone_diag_") as temp_name:
        temp = Path(temp_name); model_path = temp/"model.yaml"; cfg_path = temp/"config.yaml"
        model_path.write_text(yaml.safe_dump(model, sort_keys=False), encoding="utf-8")
        run_config["paths"]["model"] = str(model_path)
        cfg_path.write_text(yaml.safe_dump(run_config, sort_keys=False), encoding="utf-8")
        subprocess.run(["python3", str(MONTE_RUNNER), "--config", str(cfg_path), "--executable", str(executable), "--output-dir", str(output)], check=True)
    provenance = {"variant": variant["name"], "overrides": variant.get("overrides", {}), "config": str(config_path),
                  "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                  "command": f"python3 {Path(__file__).relative_to(ROOT)} --config {config_path}",
                  "model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest() if model_path.exists() else "recorded-in-run-config"}
    (output/"provenance.json").write_text(json.dumps(provenance, indent=2)+"\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--executable", type=Path, default=ROOT/"build/plant/drift_validation")
    parser.add_argument("--analyze-only", action="store_true")
    parser.add_argument("--variants", nargs="*", help="Subset of configured variant names")
    args = parser.parse_args(); config_path = args.config.resolve(); config = load_yaml(config_path)
    output_root = (ROOT/config["diagnostics"]["output_dir"]).resolve()
    variants = [v for v in config["diagnostics"]["variants"] if not args.variants or v["name"] in args.variants]
    all_rows, series_by_key = [], {}
    for variant in variants:
        output = output_root/variant["name"]
        if not args.analyze_only: run_variant(config_path, config, variant, output, args.executable.resolve())
        model = load_yaml((ROOT/config["paths"]["model"]).resolve())
        for key, value in variant.get("overrides", {}).items(): deep_set(model, key, value)
        control = load_yaml((ROOT/config["paths"]["control"]).resolve())
        with (output/"results/monte_carlo_results.csv").open(newline="", encoding="utf-8") as stream: summaries = {int(r["trial_id"]):r for r in csv.DictReader(stream)}
        rows=[]
        for log in sorted((output/"logs/all").glob("trial_*.csv.gz")):
            trial_id=int(log.stem.split("_")[1].split(".")[0]); summary=summaries[trial_id]
            record, series=analyze_log(log, model, control, float(config["diagnostics"].get("preloss_window_s", 5)))
            row={"variant":variant["name"], **summary, **record}; rows.append(row); all_rows.append(row)
            series_by_key[(variant["name"], int(summary["replicate_id"]), float(summary["com_scale"]))]=series
        write_csv(output/"results/diagnostic_metrics.csv", rows)
    write_csv(output_root/"diagnostic_metrics_all.csv", all_rows)
    # Pick the closest CRN success/loss scale pair and export aligned pre-loss series.
    pair=None
    for failed in all_rows:
        if failed["variant"] != "baseline" or str(failed["flight_success"]).lower()=="true": continue
        candidates=[r for r in all_rows if r["variant"]=="baseline" and r["replicate_id"]==failed["replicate_id"] and str(r["flight_success"]).lower()=="true" and float(r["com_scale"])<float(failed["com_scale"])]
        if candidates:
            success=max(candidates,key=lambda r:float(r["com_scale"])); pair=(success,failed); break
    if pair:
        pair_rows=[]
        failed_loss = float(pair[1]["loss_event_time_s"])
        for label,row in zip(("success","failure"),pair):
            series=series_by_key[("baseline",int(row["replicate_id"]),float(row["com_scale"]))]
            if math.isfinite(failed_loss):
                keep=(series["time"] >= failed_loss-float(config["diagnostics"].get("preloss_window_s",5))) & (series["time"] <= failed_loss)
                series={key:value[keep] for key,value in series.items()}
            for i,time in enumerate(series["time"]): pair_rows.append({"case":label,"replicate_id":row["replicate_id"],"com_scale":row["com_scale"],**{k:float(x[i]) for k,x in series.items()}})
        write_csv(output_root/"representative_pair.csv", pair_rows)
    return 0


if __name__ == "__main__": raise SystemExit(main())
