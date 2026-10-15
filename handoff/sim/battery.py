import sys, numpy as np, mpcsim as M
P = M.LongitudinalPersonality
BRANCHES = ['claude/determined-hypatia-pnidyz', 't2_launch_floor_2p0', 't2_accel_cap_2p0', 't2_achange_cost_40', 't2_jerk_cost_4',
            't2_lead2_phantom_fix', 't3_tfollow_taper', 't3_stop_distance_taper']
def load(ref):
  m, cfg = M.load_ref(ref)
  for k in ('T_FOLLOW_SCALE_FILE', 'STOP_DIST_FILE'):
    if hasattr(m, k): setattr(m, k, '/nonexistent')   # defaults
  return m, cfg
def launch(ref, lead2, hold):
  m, cfg = load(ref)
  r = M.run(m, cfg, 20.0, M.lead_launch(t0=8.0, a=2.0, v_max=13.4), v0=0.0, gap0=6.5, record=True, lead2=lead2, hold_release_s=hold)
  t = np.array(r.t); v = np.array(r.v)
  mv = np.where((t >= 8.0) & (v > 0.1))[0]
  return t[mv[0]] - 8.0, v[np.searchsorted(t, 12.0)], v[np.searchsorted(t, 14.0)]
def flips(ref):
  m, cfg = load(ref)
  return sum(M.run(m, cfg, 60.0, M.lead_const(0.0), v0=0.0, gap0=6.5, noise=(0.3, 0.2), seed=s).stop_to_pid for s in (1, 2, 3))
def steady(ref, v):
  m, cfg = load(ref)
  return M.run(m, cfg, 60.0, M.lead_const(v), v0=v, gap0=1.45 * v + 6.0, v_cruise=v + 5).final_gap
def hard_stop(ref):
  m, cfg = load(ref)
  r = M.run(m, cfg, 70.0, M.lead_brake(29.1, 50.0, 3.0, 0.0), v0=29.1, gap0=48.0, v_cruise=34, record=True)
  t = np.array(r.t); g = np.array(r.gap)
  return g[t >= 50].min()
def city_stop(ref):
  m, cfg = load(ref)
  r = M.run(m, cfg, 40.0, M.lead_const(0.0), v0=13.4, gap0=80.0, v_cruise=20)
  return r.final_gap
print(f"{'branch':34s} {'delay':>5s} {'v@4':>5s} {'v@6':>5s} | {'ph v@6':>6s} | {'hold.3':>6s} | {'flips':>5s} | {'45mph':>6s} {'65mph':>6s} | {'stop65':>6s} | {'city':>5s}")
for b in BRANCHES:
  d, v4, v6 = launch(b, 'mirror', 0.0)
  _, _, pv6 = launch(b, 'absent', 0.0)
  hd, _, _ = launch(b, 'mirror', 0.3)
  print(f"{b:34s} {d:5.2f} {v4:5.2f} {v6:5.2f} | {pv6:6.2f} | {hd:6.2f} | {flips(b):5d} | {steady(b, 20.1):6.2f} {steady(b, 29.1):6.2f} | {hard_stop(b):6.2f} | {city_stop(b):5.2f}", flush=True)
