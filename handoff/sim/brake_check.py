import numpy as np, mpcsim as M
P = M.LongitudinalPersonality
BASE = 'claude/determined-hypatia-pnidyz'
def brake(ref, v, pers, decel, v_end, files):
  m, cfg = M.load_ref(ref)
  for k, path in files.items(): setattr(m, k, path)
  # settle at the variant's own steady-state gap first, then brake
  r = M.run(m, cfg, 70.0, M.lead_brake(v, 50.0, decel, v_end), v0=v, gap0=1.45 * v + 6.0, personality=pers, v_cruise=v + 5, record=True)
  t = np.array(r.t); g = np.array(r.gap); vv = np.array(r.v); lv = np.array(r.lv); a = np.array(r.a_cmd); cc = np.array(r.crash_cnt)
  pre = g[np.searchsorted(t, 49.9)]
  post = t >= 50.0
  closing = np.maximum(vv - lv, 1e-3)
  ttc = (g / closing)[post & (vv - lv > 0.1)]
  return pre, g[post].min(), ttc.min() if len(ttc) else float('inf'), a[post].min(), int((cc[post] > 2).sum())
F = {'T_FOLLOW_SCALE_FILE': '/nonexistent', 'STOP_DIST_FILE': '/nonexistent'}
cases = [("65mph lead brakes 3.0 to stop", 29.1, 3.0, 0.0), ("65mph lead brakes 3.0 to 30mph", 29.1, 3.0, 13.4),
         ("75mph lead brakes 2.5 to 40mph", 33.5, 2.5, 17.9)]
for title, v, d, ve in cases:
  print(title)
  print(f"  {'variant':26s} {'pers':>5s} {'gap0':>6s} {'min gap':>8s} {'min TTC':>8s} {'min a':>6s} {'FCW':>4s}")
  for name, ref in [("baseline", BASE), ("t3_tfollow_taper", 't3_tfollow_taper'), ("t3_stop_distance_taper", 't3_stop_distance_taper')]:
    for pers, pn in [(P.standard, 'std'), (P.aggressive, 'aggr')]:
      files = {k: p for k, p in F.items() if (k.startswith('T_') and 'tfollow' in ref) or (k.startswith('STOP') and 'stop_dist' in ref)}
      g0, mg, ttc, ma, fcw = brake(ref, v, pers, d, ve, files)
      print(f"  {name:26s} {pn:>5s} {g0:6.1f} {mg:8.2f} {ttc:8.2f} {ma:6.2f} {fcw:4d}")
