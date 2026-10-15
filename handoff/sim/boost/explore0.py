import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np, mpcsim as M
m, cfg = M.load_ref('t2_lead2_phantom_fix')
print(cfg)
T0 = 8.0
for gap0 in (3.5, 4.5, 5.5):
  for al in (1.0, 1.6, 2.3):
    r = M.run(m, cfg, T0 + 6.0, M.lead_launch(t0=T0, a=al), v0=0.0, gap0=gap0, record=True)
    t = np.array(r.t); v = np.array(r.v); at = np.array(r.a_target); ss = np.array(r.should_stop); st = np.array(r.state); g = np.array(r.gap)
    pre = t < T0
    i0 = np.searchsorted(t, T0)
    pos = np.where((t >= T0) & (at > 0.1))[0]
    nss = np.where((t >= T0) & (~ss))[0]
    mv = np.where((t >= T0) & (v > 0.1))[0]
    print(f"gap0 {gap0} aL {al}: pre a_target {at[pre][-1]:+.2f} ss={ss[pre][-1]} | a>0.1 @ {t[pos[0]]-T0:.2f} | ss False @ {t[nss[0]]-T0:.2f} | moving @ {t[mv[0]]-T0:.2f} | v2 {v[np.searchsorted(t,T0+2)]:.2f} v4 {v[np.searchsorted(t,T0+4)]:.2f} | gap4 {g[np.searchsorted(t,T0+4)]-g[i0-1]:+.1f}")
    if gap0 == 4.5 and al == 1.6:
      for k in range(i0-2, i0+40, 2):
        print(f"   t {t[k]-T0:+.2f} at {at[k]:+.2f} ss {ss[k]} st {st[k]} acmd {r.a_cmd[k]:+.2f} v {v[k]:.2f} gap {g[k]:.2f}")
