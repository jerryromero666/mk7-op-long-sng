import numpy as np, mpcsim as M
T0 = 8.0
refs = [('#2 ghost fix (parent)', 't2_lead2_phantom_fix'), ('#4 accel cap', 't2_accel_cap_launch'), ('NEW gap match', 't2_launch_gap_match')]
def launch(m, cfg, gap0, al):
  r = M.run(m, cfg, T0 + 10.0, M.lead_launch(t0=T0, a=al, v_max=13.4), v0=0.0, gap0=gap0, record=True)
  t = np.array(r.t); v = np.array(r.v); g = np.array(r.gap); a = np.array(r.a_cmd)
  pre = (t < T0)
  mv = np.where((t >= T0) & (v > 0.1))[0]
  delay = t[mv[0]] - T0 if len(mv) else float('nan')
  i = lambda tt: np.searchsorted(t, T0 + tt)
  return delay, v[i(2)], v[i(4)], g[i(4)] - g[i(0)-1], a[t >= T0].max(), v[pre].max(), g[i(0)-1], np.max(np.abs(np.diff(a)))/M.DT_CTRL
if __name__ == "__main__":
  print('lead pulls away at aL; cells: start delay s | v@2s | v@4s m/s | gap growth 4s m | peak cmd | creep v before launch | jerk')
  for gap0 in (4.0, 5.0, 6.5):
    for al in (1.4, 2.2):
      print(f'-- stopped {gap0} m behind, lead aL={al}')
      for name, ref in refs:
        m, cfg = M.load_ref(ref)
        d, v2, v4, gg, am, creep, g0, jk = launch(m, cfg, gap0, al)
        print(f'   {name:22s} {d:5.2f} | {v2:4.2f} | {v4:4.2f} | {gg:+5.1f} | {am:4.2f} | {creep:4.2f} | {jk:5.1f}')
  print('standstill 60 s behind stopped lead with radar noise (0.3 m, 0.2 m/s): max speed, stop->go flips')
  for name, ref in refs:
    m, cfg = M.load_ref(ref)
    for gap0 in (3.5, 4.5, 6.0):
      worst = 0; flips = 0
      for s in (1, 2, 3):
        r = M.run(m, cfg, 60.0, M.lead_const(0.0), v0=0.0, gap0=gap0, noise=(0.3, 0.2), seed=s, record=True)
        worst = max(worst, max(r.v)); flips += r.stop_to_pid
      print(f'   {name:22s} gap {gap0}: max v {worst:.2f}  flips {flips}')
