import sys, numpy as np
import boostsim as B
M = B.M
B.check_wiring('WT')
mb, cfgb = M.load_ref('t2_lead2_phantom_fix')
bm = B.load_boost('WT')
T0 = 8.0
mode = sys.argv[1] if len(sys.argv) > 1 else 'vision'
for gap0, al in ((4.5, 1.6), (4.5, 1.0), (4.5, 2.3)):
  for name, bmod in (('base', None), ('boost', bm)):
    r = B.run(mb, cfgb, bmod, T0 + 8, B.launch(T0, al), gap0=gap0, mode=mode, seed=1)
    m = B.launch_metrics(r, T0)
    print(f"{name:5s} gap0 {gap0} aL {al}: " + " ".join(f"{k}={v:.2f}" if isinstance(v, float) else f"{k}={v}" for k, v in m.items()), r.events)
    if gap0 == 4.5 and al == 1.6:
      i0 = np.searchsorted(r.t, T0)
      for k in range(i0 - 2, i0 + 100, 4):
        print(f"   t {r.t[k]-T0:+.2f} amp {r.a_mpc[k]:+.2f} boost {r.boost[k]:.2f} at {r.a_target[k]:+.2f} ss {int(r.ss[k])} acmd {r.a_cmd[k]:+.2f} a {r.a[k]:+.2f} v {r.v[k]:.2f} lv {r.lv[k]:.2f} gap {r.gap[k]:.2f}")
