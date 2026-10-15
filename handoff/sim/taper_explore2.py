import os, numpy as np, mpcsim as M
P = M.LongitudinalPersonality
BASE = 'claude/determined-hypatia-pnidyz'
SCR = os.path.dirname(os.path.abspath(__file__))
TAPER_SRC = M.git_show('t3_tfollow_taper', M.MPC_PATH)
def combo_src(jerk_bp, jerk_scale=0.5):
  s = TAPER_SRC.replace(
    "  def set_weights(self, prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard):\n    jerk_factor = get_jerk_factor(personality)\n",
    f"  def set_weights(self, prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard):\n    jerk_factor = get_jerk_factor(personality) * float(np.interp(self.x0[1], {jerk_bp}, [1.0, {jerk_scale}]))\n")
  assert s != TAPER_SRC
  return s
def scale_file(val):
  path = os.path.join(SCR, f"tf_{val}"); open(path, "w").write(str(val)); return path
def mk(kind, scale, jerk_bp=None):
  if kind == 'base':
    return M.load_ref(BASE)
  m = M.load_mpc_module(src=TAPER_SRC if kind == 'taper' else combo_src(jerk_bp)); _, cfg = M.load_ref(BASE)
  m.T_FOLLOW_SCALE_FILE = scale_file(scale)
  return m, cfg
def hard_stop(m, cfg, pers, delay, tau, v=29.1, decel=3.0):
  r = M.run(m, cfg, 70.0, M.lead_brake(v, 50.0, decel, 0.0), v0=v, gap0=1.45 * v + 6.0, personality=pers, v_cruise=v + 5,
            record=True, plant_delay=delay, plant_tau=tau)
  t = np.array(r.t); g = np.array(r.gap); vv = np.array(r.v); lv = np.array(r.lv)
  post = t >= 50
  ttc = (g / np.maximum(vv - lv, 1e-3))[post & (vv - lv > 0.5) & (lv > 0.5)]
  return g[np.searchsorted(t, 49.9)], g[post].min(), ttc.min()
def launch(m, cfg, pers):
  r = M.run(m, cfg, 20.0, M.lead_launch(t0=8.0, a=2.0, v_max=13.4), v0=0.0, gap0=6.5, personality=pers, record=True)
  t = np.array(r.t); v = np.array(r.v)
  return v[np.searchsorted(t, 12.0)], v[np.searchsorted(t, 14.0)]
def chatter(m, cfg, pers):
  return sum(M.run(m, cfg, 60.0, M.lead_const(0.0), v0=0.0, gap0=6.5, personality=pers, noise=(0.3, 0.2), seed=s).stop_to_pid for s in (1, 2, 3))
plants = [("fast", 0.10, 0.20), ("mid", 0.15, 0.35), ("slow", 0.20, 0.50)]
print("Hard stop (3.0 m/s2 from 65 mph): steady gap | min gap fast/mid/slow | min TTC mid while lead moving ; launch v@4s v@6s ; standstill flips")
variants = [("baseline standard", 'base', None, None, P.standard), ("baseline aggressive", 'base', None, None, P.aggressive)]
for sc in (0.90, 0.85):
  variants.append((f"tf {sc} + jerk 0.5 @>=30mph", 'combo', sc, [5.0, 13.4], P.standard))
variants.append(("tf 0.85 + jerk 0.5 @>=30mph AGGR", 'combo', 0.85, [5.0, 13.4], P.aggressive))
for name, kind, sc, bp, pers in variants:
  mg = []
  for _, d, tau in plants:
    m, cfg = mk(kind, sc, bp); g0, mn, ttc = hard_stop(m, cfg, pers, d, tau)
    mg.append(mn)
    if _ == 'mid': ttc_mid = ttc
  m, cfg = mk(kind, sc, bp); v4, v6 = launch(m, cfg, pers)
  m, cfg = mk(kind, sc, bp); fl = chatter(m, cfg, pers)
  print(f"{name:34s} gap {g0:5.1f} | min {mg[0]:5.2f} {mg[1]:5.2f} {mg[2]:5.2f} | TTC {ttc_mid:4.2f} ; launch {v4:5.2f} {v6:5.2f} ; flips {fl}")
