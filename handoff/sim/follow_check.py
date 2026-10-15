import numpy as np, mpcsim as M
P = M.LongitudinalPersonality
BASE = 'claude/determined-hypatia-pnidyz'
def steady(ref, v, pers, mo=None, files=None):
  m, cfg = M.load_ref(ref, mpc_overrides=mo)
  for k, path in (files or {}).items(): setattr(m, k, path)
  r = M.run(m, cfg, 60.0, M.lead_const(v), v0=v, gap0=1.45 * v + 6.0, personality=pers, v_cruise=v + 5)
  return r.final_gap
def brake(ref, v, pers, decel=4.0):
  m, cfg = M.load_ref(ref)
  r = M.run(m, cfg, 45.0, M.lead_brake(v, 30.0, decel), v0=v, gap0=1.45 * v + 6.0, personality=pers, v_cruise=v + 5, record=True)
  g = np.array(r.gap); a = np.array(r.a_cmd); cc = np.array(r.crash_cnt)
  return g.min(), a.min(), int((cc > 2).sum()), r.final_gap
missing = {'T_FOLLOW_SCALE_FILE': '/nonexistent', 'STOP_DIST_FILE': '/nonexistent'}
print("STEADY-STATE GAP (m), lead at constant speed, files absent (defaults)")
print(f"{'variant':28s} " + " ".join(f"{s:>12s}" for s in ['45mph std', '60mph std', '65mph std', '75mph std', '65mph aggr']))
rows = [("baseline", BASE, None, {}),
        ("t3_tfollow_taper", 't3_tfollow_taper', None, {'T_FOLLOW_SCALE_FILE': '/nonexistent'}),
        ("t3_stop_distance_taper", 't3_stop_distance_taper', None, {'STOP_DIST_FILE': '/nonexistent'}),
        ("base COMFORT_BRAKE=3.0", BASE, {"COMFORT_BRAKE": 3.0}, {}),
        ("base COMFORT_BRAKE=2.0", BASE, {"COMFORT_BRAKE": 2.0}, {})]
for name, ref, mo, files in rows:
  vals = [steady(ref, 20.1, P.standard, mo, files), steady(ref, 26.8, P.standard, mo, files), steady(ref, 29.1, P.standard, mo, files),
          steady(ref, 33.5, P.standard, mo, files), steady(ref, 29.1, P.aggressive, mo, files)]
  print(f"{name:28s} " + " ".join(f"{x:12.2f}" for x in vals))
print()
print("HARD BRAKE: lead at 65 mph brakes 4 m/s2 to stop from steady following")
print(f"{'variant':28s} {'pers':>5s} {'min gap':>8s} {'min a_cmd':>9s} {'FCW frames':>10s} {'stop gap':>8s}")
for name, ref in [("baseline", BASE), ("t3_tfollow_taper", 't3_tfollow_taper'), ("t3_stop_distance_taper", 't3_stop_distance_taper')]:
  for pers, pn in [(P.standard, 'std'), (P.aggressive, 'aggr')]:
    mg, ma, fcw, fg = brake(ref, 29.1, pers)
    print(f"{name:28s} {pn:>5s} {mg:8.2f} {ma:9.2f} {fcw:10d} {fg:8.2f}")
