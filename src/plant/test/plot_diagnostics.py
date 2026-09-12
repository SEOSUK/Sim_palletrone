#!/usr/bin/env python3
"""Merge diagnostic pilot tables and create compact PNG evidence plots."""
import argparse, csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

def read(path):
    with path.open(newline="", encoding="utf-8") as f: return list(csv.DictReader(f))

def main():
    p=argparse.ArgumentParser(); p.add_argument("directory",type=Path); a=p.parse_args()
    root=a.directory.resolve(); rows=[]
    for table in sorted(root.glob("*/results/diagnostic_metrics.csv")): rows += read(table)
    fields=list(dict.fromkeys(k for r in rows for k in r))
    with (root/"diagnostic_metrics_all.csv").open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fields); w.writeheader(); w.writerows(rows)
    baseline=[r for r in rows if r["variant"]=="baseline"]
    scales=sorted({float(r["com_scale"]) for r in baseline})
    groups=[[r for r in baseline if float(r["com_scale"])==s] for s in scales]
    maxmotor=lambda r:max(float(r[f"motor_{i}_max_utilization"]) for i in range(1,5))
    maxangle=lambda r:max(float(r[f"servo_{i}_max_abs_angle_rad"]) for i in range(1,5))
    metrics=[("P_loss",lambda q:sum(str(r["flight_success"]).lower()=="false" for r in q)/len(q)),
             ("Max motor utilization",lambda q:np.median([maxmotor(r) for r in q])),
             ("Max servo angle [rad]",lambda q:np.median([maxangle(r) for r in q])),
             ("Max servo rate [rad/s]",lambda q:np.median([float(r["max_servo_rate_rad_s"]) for r in q])),
             ("Min normalized A1 singular value",lambda q:np.median([float(r["allocation_a1_min_singular_min"]) for r in q]))]
    fig,axs=plt.subplots(2,3,figsize=(12,7)); axs=axs.ravel()
    for ax,(label,fn) in zip(axs,metrics): ax.plot(scales,[fn(q) for q in groups],"o-"); ax.set(xlabel="CoM scale",ylabel=label); ax.grid(True,alpha=.3)
    axs[-1].axis("off"); fig.tight_layout(); fig.savefig(root/"baseline_scale_metrics.png",dpi=180); plt.close(fig)
    pair=root/"representative_pair.csv"
    if pair.exists():
        data=read(pair); fig,axs=plt.subplots(3,2,figsize=(11,9)); columns=[("altitude","Altitude [m]"),("attitude_error","Attitude error [rad]"),("motor_utilization","Motor utilization"),("servo_angle","Servo angle [rad]"),("a1_min_singular","Normalized A1 sigma min"),("torque_residual","Realized torque residual [Nm]")]
        for ax,(column,label) in zip(axs.ravel(),columns):
            for case in ("success","failure"):
                q=[r for r in data if r["case"]==case]; ax.plot([float(r["time"]) for r in q],[float(r[column]) for r in q],label=f"{case} s={q[0]['com_scale']}")
            ax.set(xlabel="Evaluation time [s]",ylabel=label); ax.grid(True,alpha=.3); ax.legend()
        fig.tight_layout(); fig.savefig(root/"representative_pair_preloss.png",dpi=180); plt.close(fig)

if __name__=="__main__": main()
