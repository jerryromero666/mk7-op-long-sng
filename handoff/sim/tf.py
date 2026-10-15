import numpy as np, mpcsim as M
from launch_gap import launch
for tf in (1.45, 1.25, 1.0, 0.8):
  m, cfg = M.load_ref('t2_launch_gap_match')
  m.get_T_FOLLOW = lambda p=None, tf=tf: tf
  for gap0, al in ((4.5, 1.6), (5.0, 2.2)):
    d, v2, v4, gg, am, creep, g0, jk = launch(m, cfg, gap0, al)
    print(f'tf {tf}: gap {gap0} aL {al}: delay {d:4.2f} v2 {v2:4.2f} v4 {v4:4.2f} growth {gg:+5.1f} peak {am:4.2f}', flush=True)
