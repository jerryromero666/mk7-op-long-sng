import sys, numpy as np, mpcsim as M
REF = sys.argv[1] if len(sys.argv) > 1 else 'claude/determined-hypatia-pnidyz'
T0 = 8.0
LEAD2 = sys.argv[2] if len(sys.argv) > 2 else 'mirror'
def launch_metrics(m, cfg, pers=M.LongitudinalPersonality.standard, hold=0.0):
  r = M.run(m, cfg, T0 + 12.0, M.lead_launch(t0=T0, a=2.0, v_max=13.4), v0=0.0, gap0=6.5,
            personality=pers, hold_release_s=hold, record=True, lead2=LEAD2)
  t = np.array(r.t); v = np.array(r.v); a = np.array(r.a_cmd); g = np.array(r.gap)
  mv = np.where((t >= T0) & (v > 0.1))[0]
  delay = t[mv[0]] - T0 if len(mv) else float('nan')
  at = lambda tt: v[np.searchsorted(t, T0 + tt)]
  gt = lambda tt: g[np.searchsorted(t, T0 + tt)]
  jerk = np.max(np.abs(np.diff(a))) / M.DT_MDL
  return dict(delay=delay, v2=at(2), v4=at(4), v6=at(6), gap4=gt(4), gap0=g[np.searchsorted(t, T0)-1], amax=a.max(), jmax=jerk)
def chatter(m, cfg, pers=M.LongitudinalPersonality.standard, seeds=(1, 2, 3)):
  tot = 0
  for s in seeds:
    r = M.run(m, cfg, 60.0, M.lead_const(0.0), v0=0.0, gap0=6.5, personality=pers, noise=(0.3, 0.2), seed=s, lead2=LEAD2)
    tot += r.stop_to_pid
  return tot
variants = [
  ("baseline", {}, {}, M.LongitudinalPersonality.standard, 0.0),
  ("J_EGO 4", {"J_EGO_COST": 4.}, {}, M.LongitudinalPersonality.standard, 0.0),
  ("J_EGO 3", {"J_EGO_COST": 3.}, {}, M.LongitudinalPersonality.standard, 0.0),
  ("J_EGO 2", {"J_EGO_COST": 2.}, {}, M.LongitudinalPersonality.standard, 0.0),
  ("A_CHG 50", {"A_CHANGE_COST": 50.}, {}, M.LongitudinalPersonality.standard, 0.0),
  ("A_CHG 30", {"A_CHANGE_COST": 30.}, {}, M.LongitudinalPersonality.standard, 0.0),
  ("cap 2.0/1.5", {}, {"A_CRUISE_MAX_VALS": [2.0, 1.5, 0.8, 0.6]}, M.LongitudinalPersonality.standard, 0.0),
  ("CRUISE_MAX 2.0", {"CRUISE_MAX_ACCEL": 2.0}, {}, M.LongitudinalPersonality.standard, 0.0),
  ("aggressive pers", {}, {}, M.LongitudinalPersonality.aggressive, 0.0),
  ("base hold0.3", {}, {}, M.LongitudinalPersonality.standard, 0.3),
  ("floor2.0 hold0.3", {}, {"UPHILL_ACCEL_FLOOR": 2.0}, M.LongitudinalPersonality.standard, 0.3),
]
print(f"{'variant':18s} {'delay':>6s} {'v@2s':>6s} {'v@4s':>6s} {'v@6s':>6s} {'gap0':>6s} {'gap@4':>6s} {'amax':>5s} {'jmax':>6s} {'flips':>5s}")
for name, mo, co, pers, hold in variants:
  m, cfg = M.load_ref(REF, mpc_overrides=mo, planner_overrides=co)
  r = launch_metrics(m, cfg, pers, hold)
  c = chatter(m, cfg, pers)
  print(f"{name:18s} {r['delay']:6.2f} {r['v2']:6.2f} {r['v4']:6.2f} {r['v6']:6.2f} {r['gap0']:6.2f} {r['gap4']:6.2f} {r['amax']:5.2f} {r['jmax']:6.2f} {c:5d}")
