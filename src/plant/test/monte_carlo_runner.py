#!/usr/bin/env python3
"""Reproducible serial Monte Carlo runner for the frozen v5 plant."""

from __future__ import annotations

import argparse
import csv
import gzip
import json
import math
import shutil
import subprocess
import tempfile
import time
from pathlib import Path

import yaml


RESULT_FIELDS = [
    "trial_id", "replicate_id", "com_scale", "payload_mass", "payload_pos_x", "payload_pos_y",
    "payload_pos_z", "true_com_x", "true_com_y", "true_com_z", "measurement_seed",
    "torque_seed", "force_seed", "position_rms", "position_rms_x", "position_rms_y",
    "position_rms_z", "velocity_rms", "attitude_rms", "attitude_rms_x",
    "attitude_rms_y", "attitude_rms_z", "peak_attitude", "angular_acc_command_rms",
    "mean_dob_norm", "final_com_error", "final_com_error_x", "final_com_error_y",
    "final_com_error_z", "moce_convergence_time", "max_motor_utilization",
    "max_servo_angle", "saturation_time", "terminal_altitude_m", "terminal_tilt_rad",
    "terminal_airborne", "numerical_success", "flight_success", "success", "failure_reason",
    "runtime_s",
]

PARAMETER_FIELDS = [
    "trial_id", "replicate_id", "com_scale", "measurement_seed", "torque_seed", "force_seed",
    "payload_mass", "payload_pos_x", "payload_pos_y", "payload_pos_z", "motor_delay",
    "motor_tau", "effectiveness_initial", "effectiveness_slope", "torque_sigma_x",
    "torque_sigma_y", "torque_sigma_z", "torque_tau_x", "torque_tau_y", "torque_tau_z",
    "force_sigma_x", "force_sigma_y", "force_sigma_z", "force_tau_x", "force_tau_y",
    "force_tau_z",
]


def copy_log_compressed(source: Path, destination: Path) -> None:
    """Preserve a full CSV log without exhausting storage during large sweeps."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    with source.open("rb") as source_stream, gzip.open(destination, "wb") as destination_stream:
        shutil.copyfileobj(source_stream, destination_stream)


def repository_root() -> Path:
    return Path(__file__).resolve().parents[3]


def load_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"Expected a YAML mapping: {path}")
    return value


def trial_plan(config: dict) -> list[dict]:
    experiment = config["experiment"]
    mode = experiment.get("mode", "com_sweep")
    if mode == "pipeline_debug":
        scales, count = [1.0], int(experiment.get("trials", 10))
    elif mode == "stochastic_repeatability":
        scales, count = [1.0], int(experiment.get("trials", 50))
    elif mode == "com_sweep":
        scales = [float(x) for x in experiment["com_scales"]]
        count = int(experiment["trials_per_level"])
    else:
        raise ValueError(f"Unsupported experiment.mode: {mode}")
    base = int(experiment["base_seed"])
    common_random_numbers = bool(experiment.get("common_random_numbers", False))
    plan = []
    for scale in scales:
        for replicate_id in range(count):
            trial_id = len(plan)
            seed_index = replicate_id if common_random_numbers else trial_id
            plan.append({
                "trial_id": trial_id,
                "replicate_id": replicate_id,
                "com_scale": scale,
                "measurement_seed": base + 3 * seed_index,
                "torque_seed": base + 3 * seed_index + 1,
                "force_seed": base + 3 * seed_index + 2,
            })
    return plan


def parameter_row(trial: dict, model: dict) -> dict:
    m = model["model"]
    payload = m["payload"]
    pos = [float(x) * trial["com_scale"] for x in payload["position"]]
    motor, torque, force = m["motor"], m["residual_torque"], m["residual_force"]
    row = dict(trial)
    row.update({
        "payload_mass": payload["mass"], "payload_pos_x": pos[0], "payload_pos_y": pos[1],
        "payload_pos_z": pos[2], "motor_delay": motor["delay_s"],
        "motor_tau": motor["time_constant_s"],
        "effectiveness_initial": motor["effectiveness"]["initial"],
        "effectiveness_slope": motor["effectiveness"]["slope_per_s"],
    })
    for prefix, values in (("torque_sigma", torque["std_nm"]),
                           ("torque_tau", torque["correlation_time_s"]),
                           ("force_sigma", force["std_n"]),
                           ("force_tau", force["correlation_time_s"])):
        for axis, value in zip("xyz", values):
            row[f"{prefix}_{axis}"] = value
    return row


def write_table(path: Path, fields: list[str], rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def read_results(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def floats(row: dict, start: int, length: int) -> list[float]:
    return [float(row[f"data[{i}]"]) for i in range(start, start + length)]


def norm(vector: list[float]) -> float:
    return math.sqrt(sum(x * x for x in vector))


def rms(values: list[float]) -> float:
    return math.sqrt(sum(x * x for x in values) / len(values))


def analyze_log(path: Path, params: dict, criteria: dict, model: dict, control: dict) -> dict:
    samples = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            raw_values = [row.get(f"data[{i}]") for i in range(124)]
            if any(value is None or value == "" for value in raw_values):
                raise ValueError("experiment_incomplete")
            values = [float(value) for value in raw_values]
            timestamp = float(row["time"]) * 1e-9
            if not math.isfinite(timestamp) or not all(math.isfinite(x) for x in values):
                raise FloatingPointError("NaN or Inf in trajectory")
            samples.append((timestamp, values))
    if len(samples) < 2:
        raise ValueError("experiment_incomplete")
    rising = next((i for i in range(1, len(samples))
                   if samples[i - 1][1][64] <= 0.5 < samples[i][1][64]), None)
    if rising is None:
        raise ValueError("experiment_incomplete")
    zero = samples[rising][0] - 5.0
    complete = [(t - zero, v) for t, v in samples if 60.0 <= t - zero <= 80.0]
    if not complete or samples[-1][0] - zero < 80.0:
        raise ValueError("experiment_incomplete")

    position_axes = [[], [], []]
    velocity_errors, attitude_axes, attitude_norms, angacc, dob = [], [[], [], []], [], [], []
    motor_util, servo_angle = [], []
    inertia = [float(x) for x in model["model"]["nominal_inertia"]]
    max_thrust = float(model["model"]["motor"]["max_thrust"])
    servo_limit = float(model["model"]["servo"]["limit_rad"])
    saturated_times = []
    previous_t = None
    for teval, values in complete:
        pe = [values[i] - values[i + 3] for i in range(3)]
        ve = [values[i + 6] - values[i + 9] for i in range(3)]
        ae = [values[i + 12] - values[i + 15] for i in range(3)]
        ae[2] = math.atan2(math.sin(ae[2]), math.cos(ae[2]))
        for axis in range(3):
            position_axes[axis].append(pe[axis])
            attitude_axes[axis].append(ae[axis])
        velocity_errors.append(norm(ve))
        attitude_norms.append(norm(ae))
        angacc.append(norm([values[i + 27] / inertia[i] for i in range(3)]))
        dob.append(norm([values[i + 31] / inertia[i] for i in range(3)]))
        mu = max(values[116:120]) / max_thrust
        sa = max(abs(x) for x in values[120:124])
        motor_util.append(mu)
        servo_angle.append(sa)
        if previous_t is not None and (mu >= 1.0 - 1e-9 or sa >= servo_limit - 1e-9):
            saturated_times.append(teval - previous_t)
        previous_t = teval

    payload_mass = float(params["payload_mass"])
    total_mass = float(model["model"]["vehicle_mass"]) + payload_mass
    true_com = [payload_mass / total_mass * float(params[f"payload_pos_{a}"]) for a in "xyz"]
    final_values = complete[-1][1]
    final_com = final_values[54:57]
    com_error = [final_com[i] - true_com[i] for i in range(3)]
    terminal_altitude = final_values[2]
    # This is the controller's existing contact/airborne boundary, not a fitted
    # Monte Carlo threshold. The reference trajectory never commands below 0.625 m.
    airborne_height = float(control["controller"]["adaptation_min_height"])
    terminal_airborne = terminal_altitude > airborne_height
    terminal_tilt = math.acos(max(-1.0, min(1.0,
        math.cos(final_values[12]) * math.cos(final_values[13]))))
    result = dict(params)
    result.update({
        "true_com_x": true_com[0], "true_com_y": true_com[1], "true_com_z": true_com[2],
        "position_rms_x": rms(position_axes[0]), "position_rms_y": rms(position_axes[1]),
        "position_rms_z": rms(position_axes[2]),
        "position_rms": math.sqrt(sum(sum(x*x for x in axis) for axis in position_axes) /
                                  len(complete)),
        "velocity_rms": rms(velocity_errors), "attitude_rms": rms(attitude_norms),
        "attitude_rms_x": rms(attitude_axes[0]), "attitude_rms_y": rms(attitude_axes[1]),
        "attitude_rms_z": rms(attitude_axes[2]), "peak_attitude": max(attitude_norms),
        "angular_acc_command_rms": rms(angacc), "mean_dob_norm": sum(dob) / len(dob),
        "final_com_error": norm(com_error), "final_com_error_x": com_error[0],
        "final_com_error_y": com_error[1], "final_com_error_z": com_error[2],
        "moce_convergence_time": math.nan, "max_motor_utilization": max(motor_util),
        "max_servo_angle": max(servo_angle), "saturation_time": sum(saturated_times),
        "terminal_altitude_m": terminal_altitude, "terminal_tilt_rad": terminal_tilt,
        "terminal_airborne": terminal_airborne, "numerical_success": True,
        "flight_success": terminal_airborne, "success": terminal_airborne,
        "failure_reason": "normal_completion" if terminal_airborne else "loss_of_flight",
    })

    com_limit = criteria.get("max_com_error_m")
    if com_limit is not None:
        after_moce = [(t - zero, v) for t, v in samples if t >= samples[rising][0]]
        errors = [norm([v[54 + i] - true_com[i] for i in range(3)]) for _, v in after_moce]
        suffix_ok = True
        convergence = math.nan
        for index in range(len(errors) - 1, -1, -1):
            suffix_ok = suffix_ok and errors[index] <= float(com_limit)
            if suffix_ok:
                convergence = after_moce[index][0] - 5.0
        result["moce_convergence_time"] = convergence

    checks = [
        ("max_position_error_m", max(norm([position_axes[a][i] for a in range(3)])
                                     for i in range(len(complete))), "excessive_position_error"),
        ("max_attitude_error_rad", result["peak_attitude"], "excessive_attitude_error"),
        ("max_com_error_m", result["final_com_error"], "moce_nonconvergence"),
        ("max_motor_utilization", result["max_motor_utilization"], "actuator_saturation"),
        ("max_servo_angle_rad", result["max_servo_angle"], "actuator_saturation"),
    ]
    for key, value, reason in checks:
        if (result["flight_success"] and criteria.get(key) is not None and
                value > float(criteria[key])):
            result["success"], result["failure_reason"] = False, reason
            break
    return result


def failed_result(params: dict, reason: str, runtime: float) -> dict:
    row = {field: math.nan for field in RESULT_FIELDS}
    row.update(params)
    row.update({"terminal_airborne": False, "numerical_success": False,
                "flight_success": False, "success": False,
                "failure_reason": reason, "runtime_s": runtime})
    return row


def reclassify_legacy_results(path: Path) -> None:
    """Upgrade summaries made before terminal state fields were introduced.

    Full Stage C logs were intentionally not retained. Exact zero Complete-window
    DOB activity is used only for this legacy migration: in this stochastic v5
    model it records that the controller's existing airborne gate stayed false.
    New runs classify directly from terminal altitude.
    """
    rows = read_results(path)
    if not rows:
        raise ValueError(f"No result rows to reclassify: {path}")
    for row in rows:
        numerical = str(row.get("success", "")).lower() in ("true", "1")
        loss = numerical and float(row.get("mean_dob_norm", "nan")) == 0.0
        row.setdefault("replicate_id", row["trial_id"])
        row.update({
            "terminal_altitude_m": math.nan,
            "terminal_tilt_rad": math.nan,
            "terminal_airborne": not loss if numerical else False,
            "numerical_success": numerical,
            "flight_success": numerical and not loss,
            "success": numerical and not loss,
            "failure_reason": ("loss_of_flight" if loss else
                               "normal_completion" if numerical else row["failure_reason"]),
        })
    write_table(path, RESULT_FIELDS, rows)


def retain_representatives(rows: list[dict], logs: dict[int, Path], destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    scales = sorted({float(r["com_scale"]) for r in rows})
    for scale in scales:
        eligible = sorted((r for r in rows if float(r["com_scale"]) == scale and
                           str(r["success"]).lower() == "true" and
                           int(r["trial_id"]) in logs),
                          key=lambda r: float(r["position_rms"]))
        if not eligible:
            continue
        chosen = [(eligible[0], "best"),
                  (eligible[(len(eligible) - 1) // 2], "median"),
                  (eligible[-1], "worst_success")]
        for row, label in chosen:
            trial_id = int(row["trial_id"])
            name = f"trial_{trial_id:04d}_com_{scale:.2f}_{label}.csv.gz"
            copy_log_compressed(logs[trial_id], destination / name)


def main() -> int:
    root = repository_root()
    default_config = root / "src/palletrone_interfaces/bag/monte/config/monte_carlo_config.yaml"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=default_config)
    parser.add_argument("--executable", type=Path, default=root / "build/plant/drift_validation")
    parser.add_argument("--output-dir", type=Path,
                        help="Override monte directory (useful for smoke tests)")
    parser.add_argument("--plan", action="store_true", help="Print deterministic plan and exit")
    parser.add_argument("--reclassify-results", type=Path,
                        help="Upgrade an existing pre-classifier summary in place and exit")
    args = parser.parse_args()
    if args.reclassify_results:
        reclassify_legacy_results(args.reclassify_results.resolve())
        return 0
    config = load_yaml(args.config.resolve())
    plan = trial_plan(config)
    if args.plan:
        print(json.dumps(plan, indent=2))
        return 0

    if config.get("uncertainties", {}).get("parameter_randomization_enabled", False):
        raise ValueError("Parameter randomization is schema-only in this implementation")
    if not args.executable.is_file():
        raise FileNotFoundError(f"Build once before running: {args.executable}")

    paths = config.get("paths", {})
    model_path = (root / paths.get("model", "src/palletrone_interfaces/config/model.yaml")).resolve()
    control_path = (root / paths.get("control", "src/palletrone_interfaces/config/control.yaml")).resolve()
    command_path = (root / paths.get("command", "src/palletrone_interfaces/config/command.yaml")).resolve()
    scene_path = (root / paths.get("scene", "src/plant/xml/scene.xml")).resolve()
    monte_dir = args.output_dir.resolve() if args.output_dir else args.config.resolve().parents[1]
    results_path = monte_dir / "results/monte_carlo_results.csv"
    parameters_path = monte_dir / "results/monte_carlo_parameters.csv"
    baseline = load_yaml(model_path)
    control = load_yaml(control_path)
    parameter_rows = [parameter_row(trial, baseline) for trial in plan]
    write_table(parameters_path, PARAMETER_FIELDS, parameter_rows)

    old_rows = read_results(results_path)
    resume = bool(config["experiment"].get("resume", True))
    planned_ids = {trial["trial_id"] for trial in plan}
    expected = {trial["trial_id"]: trial for trial in plan}
    completed = {}
    for row in old_rows:
        trial_id = int(row["trial_id"])
        trial = expected.get(trial_id)
        same_definition = trial is not None and all([
            float(row["com_scale"]) == trial["com_scale"],
            int(row["measurement_seed"]) == trial["measurement_seed"],
            int(row["torque_seed"]) == trial["torque_seed"],
            int(row["force_seed"]) == trial["force_seed"],
        ])
        numerical_success = row.get("numerical_success", row.get("success", ""))
        if resume and same_definition and str(numerical_success).lower() in ("true", "1"):
            completed[trial_id] = row
    completed = {trial_id: row for trial_id, row in completed.items() if trial_id in planned_ids}
    rows = dict(completed)
    timeout = float(config["experiment"].get("process_timeout_s", 120.0))
    logging = config.get("logging", {})
    failure_dir = monte_dir / "logs/failures"
    representative_dir = monte_dir / "logs/representatives"
    fresh_logs: dict[int, Path] = {}

    with tempfile.TemporaryDirectory(prefix="palletrone_mc_") as temp_name:
        temp = Path(temp_name)
        for trial, params in zip(plan, parameter_rows):
            trial_id = trial["trial_id"]
            if trial_id in completed:
                print(f"SKIP trial {trial_id}: completed")
                continue
            trial_model = yaml.safe_load(yaml.safe_dump(baseline))
            trial_model["model"]["payload"]["position"] = [params[f"payload_pos_{a}"] for a in "xyz"]
            trial_model["model"]["sensors"]["seed"] = trial["measurement_seed"]
            trial_model["model"]["residual_torque"]["seed"] = trial["torque_seed"]
            trial_model["model"]["residual_force"]["seed"] = trial["force_seed"]
            model_file, log_file = temp / f"model_{trial_id}.yaml", temp / f"trial_{trial_id}.csv"
            with model_file.open("w", encoding="utf-8") as stream:
                yaml.safe_dump(trial_model, stream, sort_keys=False)
            command = [str(args.executable.resolve()), str(control_path), str(model_file),
                       str(command_path), str(scene_path), f"mc_{trial_id}", str(log_file),
                       str(trial["measurement_seed"]), str(trial["torque_seed"]),
                       str(trial["force_seed"])]
            started = time.monotonic()
            try:
                process = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
                runtime = time.monotonic() - started
                if process.returncode != 0:
                    row = failed_result(params, "process_crash", runtime)
                else:
                    row = analyze_log(log_file, params, config.get("success_criteria", {}),
                                      baseline, control)
                    row["runtime_s"] = runtime
            except subprocess.TimeoutExpired:
                runtime = time.monotonic() - started
                row = failed_result(params, "timeout", runtime)
            except FloatingPointError:
                runtime = time.monotonic() - started
                row = failed_result(params, "numerical_failure", runtime)
            except (OSError, ValueError, KeyError, csv.Error) as error:
                runtime = time.monotonic() - started
                reason = "experiment_incomplete" if str(error) == "experiment_incomplete" else "unknown"
                row = failed_result(params, reason, runtime)
            rows[trial_id] = row
            if log_file.exists():
                fresh_logs[trial_id] = log_file
                if not row["success"] and logging.get("save_failures", True):
                    failure_dir.mkdir(parents=True, exist_ok=True)
                    copy_log_compressed(
                        log_file,
                        failure_dir /
                        f"trial_{trial_id:04d}_com_{trial['com_scale']:.2f}_failure.csv.gz")
                if logging.get("save_all_full_csv", False):
                    all_dir = monte_dir / "logs/all"
                    all_dir.mkdir(parents=True, exist_ok=True)
                    copy_log_compressed(log_file, all_dir / f"trial_{trial_id:04d}.csv.gz")
            ordered = [rows[i] for i in sorted(rows)]
            write_table(results_path, RESULT_FIELDS, ordered)
            print(f"DONE trial {trial_id}: {row['failure_reason']}")
        ordered = [rows[i] for i in sorted(rows)]
        if logging.get("save_representatives", True):
            retain_representatives(ordered, fresh_logs, representative_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
