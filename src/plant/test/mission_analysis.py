#!/usr/bin/env python3
"""Classify mission eligibility and measure Lissajous-window performance."""
from __future__ import annotations
import argparse, copy, csv, gzip, math
from pathlib import Path
import numpy as np
import yaml
from diagnostic_runner import ROOT, geometry, normalized_svd, read_log, write_csv

def mission_windows(t,v,duration=60.0,steady_duration=20.0):
    edges=np.flatnonzero((v[:-1,65]<=.5)&(v[1:,65]>.5))
    if edges.size==0: raise ValueError("Lissajous/MOCE-Z start event missing")
    start=float(t[edges[0]+1]); end=start+duration
    return (start,end),(end-steady_duration,end)

def classify_mission(t,v,gate,window):
    start=np.flatnonzero(t>=window[0])[0]; end=np.flatnonzero(t<=window[1])[-1]
    eligible=bool(v[start,2]>gate)
    crossing=bool(np.any((v[start:end,2]>gate)&(v[start+1:end+1,2]<=gate))) if eligible else False
    loss=bool(eligible and (crossing or v[end,2]<=gate))
    return eligible,loss

def rms(x,axis=None): return np.sqrt(np.mean(np.square(x),axis=axis))

def window_metrics(t,v,model,window):
    keep=(t>=window[0])&(t<=window[1]); t=t[keep]; v=v[keep]
    if len(t)<2: raise ValueError(f"empty mission window {window}")
    m=model["model"]; max_thrust=float(m["motor"]["max_thrust"]); servo_limit=float(m["servo"]["limit_rad"])
    mass=float(m["vehicle_mass"])+float(m["payload"]["mass"])
    true_com=float(m["payload"]["mass"])/mass*np.asarray(m["payload"]["position"],float)
    pe=v[:,0:3]-v[:,3:6]; ve=v[:,6:9]-v[:,9:12]; ae=v[:,12:15]-v[:,15:18]
    ae[:,2]=np.arctan2(np.sin(ae[:,2]),np.cos(ae[:,2]))
    motor=v[:,116:120]; servo=v[:,120:124]
    rate=np.vstack((np.zeros(4),np.diff(servo,axis=0)/np.diff(t)[:,None]))
    result={"position_rms":float(rms(np.linalg.norm(pe,axis=1))),"velocity_rms":float(rms(np.linalg.norm(ve,axis=1))),
            "attitude_rms":float(rms(np.linalg.norm(ae,axis=1))),"peak_attitude_error":float(np.max(np.linalg.norm(ae,axis=1))),
            "com_error_rms":float(rms(np.linalg.norm(v[:,54:57]-true_com,axis=1))),"final_com_error":float(np.linalg.norm(v[-1,54:57]-true_com)),
            "max_motor_utilization":float(np.max(motor)/max_thrust),"motor_saturation_fraction":float(np.mean(np.any(motor>=max_thrust-1e-7,axis=1))),
            "max_servo_angle":float(np.max(np.abs(servo))),"max_servo_rate":float(np.max(np.abs(rate))),
            "servo_angle_contact_fraction":float(np.mean(np.any(np.abs(servo)>=servo_limit-1e-5,axis=1))),
            "mean_dob_norm":float(np.mean(np.linalg.norm(v[:,31:34],axis=1))),"dob_rms":float(rms(np.linalg.norm(v[:,31:34],axis=1)))}
    for i,a in enumerate("xyz"): result[f"position_rms_{a}"]=float(rms(pe[:,i])); result[f"attitude_rms_{a}"]=float(rms(ae[:,i]))
    for i in range(4):
        result[f"motor_{i+1}_max_utilization"]=float(np.max(motor[:,i])/max_thrust)
        result[f"servo_{i+1}_max_angle"]=float(np.max(np.abs(servo[:,i])))
    # Allocation diagnostics at 40 Hz. Actuator/contact metrics above retain 400 Hz.
    v=v[::10]; desired_f=v[:,24:27]; desired_t=v[:,27:30]
    alloc_f=[]; alloc_t=[]; real_f=[]; real_t=[]; sigma=[]; condition=[]; ranks=[]
    sx=np.array([1,-1,-1,1]); sy=np.array([1,1,-1,-1]); reaction=float(m["motor"]["reaction_ratio"])
    def wrench(thrust,direction,center,spin):
        force=np.sum(thrust[:,None]*direction,axis=0); torque=np.zeros(3)
        for i in range(4):
            pos=np.array([sx[i]*float(m["arm_xy"]),sy[i]*float(m["arm_xy"]),float(m["rotor_z"])])
            fi=thrust[i]*direction[i]; torque+=np.cross(pos-center,fi)+spin[i]*reaction*fi
        return force,torque
    for row in v:
        est=row[54:57]; cmd_thrust=float(m["motor"]["thrust_coefficient"])*np.maximum(row[34:38],0)**2
        a1,_,cmd_dir,spin=geometry(model,row[38:42],est,cmd_thrust); c=normalized_svd(a1)
        af,at=wrench(cmd_thrust,cmd_dir,est,spin)
        _,_,actual_dir,spin=geometry(model,row[120:124],true_com,row[116:120]); rf,rt=wrench(row[116:120],actual_dir,true_com,spin)
        alloc_f.append(af); alloc_t.append(at); real_f.append(rf); real_t.append(rt); condition.append(c[0]); sigma.append(c[1]); ranks.append(c[3])
    alloc_f=np.asarray(alloc_f); alloc_t=np.asarray(alloc_t); real_f=np.asarray(real_f); real_t=np.asarray(real_t)
    for name,x in [("allocation_force_residual",desired_f-alloc_f),("allocation_torque_residual",desired_t-alloc_t),
                   ("realized_force_residual",desired_f-real_f),("realized_torque_residual",desired_t-real_t)]:
        n=np.linalg.norm(x,axis=1); result[name+"_rms"]=float(rms(n)); result[name+"_max"]=float(np.max(n))
    result.update({"allocation_min_singular":float(np.min(sigma)),"allocation_condition_max":float(np.max(condition)),"allocation_rank_min":int(np.min(ranks))})
    return result

def analyze(log,summary,base_model,control,duration):
    t,v=read_log(log); model=copy.deepcopy(base_model)
    model["model"]["payload"]["position"]=[float(summary[f"payload_pos_{axis}"]) for axis in "xyz"]
    primary,steady=mission_windows(t,v,duration); gate=float(control["controller"]["adaptation_min_height"])
    eligible,mission_loss=classify_mission(t,v,gate,primary)
    grid_trial=summary.get("payload_x_factor","") not in ("",None)
    status=("stabilization_failure" if grid_trial else "pre_mission_failure") if not eligible else "mission_loss" if mission_loss else "mission_completion"
    row={"mission_status":status,"mission_eligible":eligible,"mission_completion":eligible and not mission_loss,"mission_loss":mission_loss,
         "lissajous_start_s":primary[0],"lissajous_end_s":primary[1],**summary}
    if eligible:
        for prefix,window in (("mission",primary),("steady",steady)):
            row.update({prefix+"_"+k:x for k,x in window_metrics(t,v,model,window).items()})
    return row

def main():
    p=argparse.ArgumentParser(); p.add_argument("--logs",type=Path,required=True); p.add_argument("--results",type=Path,required=True); p.add_argument("--output",type=Path,required=True); a=p.parse_args()
    base=yaml.safe_load((ROOT/"src/palletrone_interfaces/config/model.yaml").read_text()); control=yaml.safe_load((ROOT/"src/palletrone_interfaces/config/control.yaml").read_text()); command=yaml.safe_load((ROOT/"src/palletrone_interfaces/config/command.yaml").read_text())
    duration=float(command["command"]["automated_experiment"]["lissajous_duration_s"])
    with a.results.open(newline="",encoding="utf-8") as f: summaries={int(r["trial_id"]):r for r in csv.DictReader(f)}
    rows=[]
    for log in sorted(a.logs.glob("trial_*.csv.gz")):
        trial=int(log.name.split("_")[1].split(".")[0]); rows.append(analyze(log,summaries[trial],base,control,duration))
    write_csv(a.output,rows)

if __name__=="__main__": main()
