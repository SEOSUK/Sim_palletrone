#!/usr/bin/env python3
"""Reconstruct body-FLU force residuals from hardware or simulator CSV logs."""

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import signal
from scipy.spatial.transform import Rotation

MASS = 4.7
GRAVITY = 9.81
INERTIA = np.array([0.0768, 0.0871, 0.1130])
TANGENT = np.array([[1, -1, 0], [1, 1, 0], [-1, 1, 0], [-1, -1, 0]]) / np.sqrt(2)


def lowpass(values, fs):
    sos = signal.butter(2, 2.0, btype="low", fs=fs, output="sos")
    return signal.sosfiltfilt(sos, values, axis=0)


def stats(values, fs):
    centered = values - values.mean(axis=0)
    std = values.std(axis=0, ddof=1)
    tau, peak = [], []
    for axis in range(3):
        n = len(values)
        corr = signal.correlate(centered[:, axis], centered[:, axis], mode="full", method="fft")[n - 1:]
        corr /= np.arange(n, 0, -1)
        if corr[0] <= np.finfo(float).eps:
            tau.append(float("nan"))
            peak.append(float("nan"))
            continue
        corr /= corr[0]
        crossing = np.flatnonzero(corr <= np.exp(-1))
        tau.append(float(crossing[0] / fs) if len(crossing) else float("nan"))
        f, pxx = signal.periodogram(centered[:, axis], fs=fs)
        peak.append(float(f[1 + np.argmax(pxx[1:])]))
    return {"mean": values.mean(axis=0).tolist(), "std": std.tolist(),
            "acf_1e_s": tau, "dominant_psd_hz": peak}


def directions(servo):
    return TANGENT[None, :, :] * np.sin(servo)[:, :, None] + \
        np.array([0.0, 0.0, 1.0])[None, None, :] * np.cos(servo)[:, :, None]


def load_columns(path, indices):
    names = ["time"] + [f"data[{i}]" for i in indices]
    frame = pd.read_csv(path, usecols=names)
    t = frame["time"].to_numpy(dtype=float) * 1e-9
    data = {i: frame[f"data[{i}]"].to_numpy(dtype=float) for i in indices}
    return t, data


def real_residual(path):
    indices = list(range(34, 42)) + list(range(48, 51))
    t, d = load_columns(path, indices)
    fs = 1.0 / np.median(np.diff(t))
    thrust_command = np.column_stack([d[i] for i in range(34, 38)])
    servo = np.column_stack([d[i] for i in range(38, 42)])
    specific_frd = np.column_stack([d[i] for i in range(48, 51)])
    actual_body_flu = MASS * specific_frd * np.array([1.0, -1.0, -1.0])
    delay_samples = int(round(0.010 * fs))
    delayed = np.vstack([np.zeros((delay_samples, 4)), thrust_command[:-delay_samples]])
    eta = np.clip(0.6966 - 0.000844 * t, 0.63, 0.70)
    target = delayed * eta[:, None]
    motor = np.zeros_like(target)
    a = np.exp(-1.0 / (fs * 0.017))
    for k in range(1, len(t)):
        motor[k] = a * motor[k - 1] + (1 - a) * target[k]
    predicted_body = np.sum(motor[:, :, None] * directions(servo), axis=1)
    residual = lowpass(actual_body_flu - predicted_body, fs)
    mask = (t >= 60) & (t <= 80)
    return stats(residual[mask], fs)


def sim_residual(path):
    indices = list(range(0, 65)) + list(range(104, 124))
    t, d = load_columns(path, indices)
    fs = 1.0 / np.median(np.diff(t))
    rpy = np.column_stack([d[i] for i in range(12, 15)])
    acceleration_world = np.column_stack([d[i] for i in range(42, 45)])
    rotation = Rotation.from_euler("xyz", rpy).as_matrix()
    specific_world = acceleration_world - np.array([0.0, 0.0, -GRAVITY])
    actual_body = MASS * np.einsum("nji,nj->ni", rotation, specific_world)
    speed_command = np.column_stack([d[i] for i in range(34, 38)])
    thrust_command = 0.02 * np.maximum(speed_command, 0.0) ** 2
    delay_samples = int(round(0.010 * fs))
    delayed = np.vstack([np.zeros((delay_samples, 4)), thrust_command[:-delay_samples]])
    active = np.flatnonzero(np.max(thrust_command, axis=1) > 1e-9)
    elapsed = np.maximum(t - (t[active[0]] if len(active) else 0.0), 0.0)
    eta = np.clip(0.6966 - 0.000844 * elapsed, 0.63, 0.70)
    target = delayed * eta[:, None]
    motor = np.zeros_like(target)
    a = np.exp(-1.0 / (fs * 0.017))
    for k in range(1, len(t)):
        motor[k] = a * motor[k - 1] + (1 - a) * target[k]
    servo = np.column_stack([d[i] for i in range(120, 124)])
    predicted_body = np.sum(motor[:, :, None] * directions(servo), axis=1)
    residual = lowpass(actual_body - predicted_body, fs)
    rises = np.flatnonzero(np.diff(d[64]) > 0) + 1
    if not len(rises):
        raise ValueError(f"No MOCE rising edge in {path}")
    evaluation_time = t - (t[rises[0]] - 5.0)
    mask = (evaluation_time >= 60) & (evaluation_time <= 80)
    result = {"reconstructed": stats(residual[mask], fs)}
    ou = np.column_stack([d[i] for i in range(110, 113)])
    result["ou_input"] = stats(ou[mask], fs)

    pos_error = np.column_stack([d[i] - d[i + 3] for i in range(3)])
    vel_error = np.column_stack([d[i] - d[i + 3] for i in range(6, 9)])
    attitude_error = np.column_stack([d[i] - d[i + 3] for i in range(12, 15)])
    attitude_error[:, 2] = np.arctan2(np.sin(attitude_error[:, 2]), np.cos(attitude_error[:, 2]))
    torque = np.column_stack([d[i] for i in range(27, 30)])
    dob = np.column_stack([d[i] for i in range(31, 34)]) / INERTIA
    m = mask
    rms_axis = lambda x: np.sqrt(np.mean(x[m] ** 2, axis=0)).tolist()
    norm_rms = lambda x: float(np.sqrt(np.mean(np.sum(x[m] ** 2, axis=1))))
    true_com = np.array([0.0329787, 0.0329787, 0.0595745])
    final_com = np.array([d[i][-1] for i in range(54, 57)])
    result["flight_complete"] = {
        "position_rms": norm_rms(pos_error), "position_axis_rms": rms_axis(pos_error),
        "velocity_rms": norm_rms(vel_error), "velocity_axis_rms": rms_axis(vel_error),
        "attitude_rms": norm_rms(attitude_error),
        "peak_attitude": float(np.linalg.norm(attitude_error[m], axis=1).max()),
        "angular_acc_command_rms": norm_rms(torque / INERTIA),
        "mean_dob_norm": float(np.linalg.norm(dob[m], axis=1).mean()),
        "dob_axis_rms": rms_axis(dob), "final_com": final_com.tolist(),
        "final_com_axis_error": (final_com - true_com).tolist(),
        "final_com_error": float(np.linalg.norm(final_com - true_com)),
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("kind", choices=["real", "sim"])
    parser.add_argument("paths", nargs="+", type=Path)
    args = parser.parse_args()
    analyze = real_residual if args.kind == "real" else sim_residual
    print(json.dumps({str(path): analyze(path) for path in args.paths}, indent=2))


if __name__ == "__main__":
    main()
