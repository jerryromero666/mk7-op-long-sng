import numpy as np, mpcsim as M
from launch_gap import launch
V = [('parent', 't2_lead2_phantom_fix', {}, {}), ('gap match', 't2_launch_gap_match', {}, {}),
     ('A_CHG 30', 't2_lead2_phantom_fix', {'A_CHANGE_COST': 30.}, {}), ('J_EGO 2', 't2_lead2_phantom_fix', {'J_EGO_COST': 2.}, {}),
     ('floor 2.0', 't2_lead2_phantom_fix', {}, {'UPHILL_ACCEL_FLOOR': 2.0}),
     ('gap+A_CHG30', 't2_launch_gap_match', {'A_CHANGE_COST': 30.}, {})]
for name, ref, mo, co in V:
  m, cfg = M.load_ref(ref, mpc_overrides=mo, planner_overrides=co)
  for gap0, al in ((4.5, 1.6), (5.0, 2.2)):
    d, v2, v4, gg, am, creep, g0, jk = launch(m, cfg, gap0, al)
    print(f'{name:12s} gap {gap0} aL {al}: delay {d:4.2f} v2 {v2:4.2f} v4 {v4:4.2f} growth {gg:+5.1f} peak {am:4.2f}', flush=True)
