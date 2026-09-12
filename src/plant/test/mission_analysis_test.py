#!/usr/bin/env python3
"""Synthetic event/window and mission-classification regression test."""
import numpy as np
from mission_analysis import mission_windows, classify_mission

t=np.arange(0,100.0025,.0025); v=np.zeros((len(t),124)); v[:,2]=.7; v[t>=20,65]=1
primary,steady=mission_windows(t,v,60,20)
assert primary==(20.0,80.0),primary
assert steady==(60.0,80.0),steady
assert classify_mission(t,v,.4,primary)==(True,False)
failed=v.copy(); failed[t>=50,2]=.2
assert classify_mission(t,failed,.4,primary)==(True,True)
pre=v.copy(); pre[t>=19,2]=.2
assert classify_mission(t,pre,.4,primary)==(False,False)
print("PASS: event-derived [20,80], steady [60,80], and phase classification")
