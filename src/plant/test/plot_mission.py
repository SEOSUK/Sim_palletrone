#!/usr/bin/env python3
"""Raw-scatter and median/IQR figures for the mission-phase pilot."""
import argparse,csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

p=argparse.ArgumentParser(); p.add_argument("table",type=Path); p.add_argument("output",type=Path); a=p.parse_args()
with a.table.open(newline="",encoding="utf-8") as f: rows=list(csv.DictReader(f))
eligible=[r for r in rows if r["mission_eligible"]=="True"]
metrics=[("mission_position_rms","Position RMS [m]"),("mission_attitude_rms","Attitude RMS [rad]"),
         ("mission_peak_attitude_error","Peak attitude error [rad]"),("mission_max_motor_utilization","Max motor utilization"),
         ("mission_max_servo_angle","Max servo angle [rad]"),("mission_max_servo_rate","Max servo rate [rad/s]"),
         ("mission_realized_torque_residual_rms","Realized torque residual RMS [Nm]"),
         ("mission_com_error_rms","CoM estimation error RMS [m]"),("mission_allocation_min_singular","Normalized allocation sigma min")]
scales=sorted({float(r["com_scale"]) for r in rows}); fig,axs=plt.subplots(3,3,figsize=(13,10))
for ax,(field,label) in zip(axs.ravel(),metrics):
    meds=[]; valid=[]
    for scale in scales:
        vals=[float(r[field]) for r in eligible if float(r["com_scale"])==scale]
        if not vals: continue
        jitter=np.linspace(-.006,.006,len(vals)); ax.scatter(scale+jitter,vals,s=20,alpha=.65); meds.append(np.median(vals)); valid.append(scale)
        if len(vals)>1: ax.vlines(scale,*np.percentile(vals,[25,75]),color="k",lw=2)
    ax.plot(valid,meds,"k-o",lw=1,label="median"); ax.set(xlabel="CoM scale",ylabel=label); ax.grid(True,alpha=.25)
fig.suptitle("500 g modeled CoM-bias mission pilot (eligible trials only)"); fig.tight_layout(); a.output.parent.mkdir(parents=True,exist_ok=True); fig.savefig(a.output,dpi=180)
