#!/usr/bin/env python3
"""Aggregate five-replicate payload-grid results and render raw metric maps."""
import argparse,csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

p=argparse.ArgumentParser(); p.add_argument("table",type=Path); p.add_argument("output_dir",type=Path); a=p.parse_args()
with a.table.open(newline="",encoding="utf-8") as f: rows=list(csv.DictReader(f))
xs=sorted({float(r["payload_pos_x"]) for r in rows}); ys=sorted({float(r["payload_pos_y"]) for r in rows})
metrics=[("mission_position_rms","Position RMS [m]"),("mission_attitude_rms","Attitude RMS [rad]"),
 ("mission_peak_attitude_error","Peak attitude [rad]"),("mission_max_motor_utilization","Motor utilization"),
 ("mission_max_servo_angle","Servo angle [rad]"),("mission_max_servo_rate","Servo rate [rad/s]"),
 ("mission_realized_torque_residual_rms","Torque residual RMS [Nm]"),("mission_allocation_min_singular","Normalized sigma min"),
 ("mission_com_error_rms","CoM error RMS [m]")]
summary=[]
for y in ys:
 for x in xs:
  q=[r for r in rows if float(r["payload_pos_x"])==x and float(r["payload_pos_y"])==y]
  out={"payload_pos_x":x,"payload_pos_y":y,"replicates":len(q),"lissajous_eligible":sum(r["mission_eligible"]=="True" for r in q),
       "stabilization_failure":sum(r["mission_status"]=="stabilization_failure" for r in q),"mission_loss":sum(r["mission_loss"]=="True" for r in q)}
  for field,_ in metrics: out[field+"_median"]=float(np.median([float(r[field]) for r in q if r["mission_eligible"]=="True"]))
  summary.append(out)
a.output_dir.mkdir(parents=True,exist_ok=True)
with (a.output_dir.parent/"results/payload_grid_summary.csv").open("w",newline="",encoding="utf-8") as f:
 w=csv.DictWriter(f,summary[0],lineterminator="\n"); w.writeheader(); w.writerows(summary)
fig,axs=plt.subplots(3,3,figsize=(13,11),constrained_layout=True)
extent=[min(xs),max(xs),min(ys),max(ys)]
for ax,(field,label) in zip(axs.ravel(),metrics):
 z=np.array([[next(r[field+"_median"] for r in summary if r["payload_pos_x"]==x and r["payload_pos_y"]==y) for x in xs] for y in ys])
 im=ax.imshow(z,origin="lower",extent=extent,aspect="equal",cmap="viridis"); ax.scatter([.31],[.31],marker="*",s=100,c="red",edgecolor="white",label="Setup-2")
 ax.set(xlabel="payload x [m]",ylabel="payload y [m]",title=label); ax.legend(loc="upper left",fontsize=7); fig.colorbar(im,ax=ax,shrink=.78)
fig.suptitle("Modeled 500 g payload-position map: eligible-trial medians")
fig.savefig(a.output_dir/"payload_grid_metrics.png",dpi=180); plt.close(fig)
completion=np.array([[next(r["lissajous_eligible"] for r in summary if r["payload_pos_x"]==x and r["payload_pos_y"]==y) for x in xs] for y in ys])
fig,ax=plt.subplots(figsize=(6,5)); im=ax.imshow(completion,origin="lower",extent=extent,vmin=0,vmax=5,aspect="equal",cmap="viridis")
ax.scatter([.31],[.31],marker="*",s=120,c="red",edgecolor="white",label="Setup-2"); ax.set(xlabel="payload x [m]",ylabel="payload y [m]",title="Lissajous-eligible / 5 (mission loss = 0 everywhere)"); ax.legend(); fig.colorbar(im,ax=ax); fig.tight_layout(); fig.savefig(a.output_dir/"payload_grid_completion.png",dpi=180)
