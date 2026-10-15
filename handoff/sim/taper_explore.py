import os, numpy as np, mpcsim as M
P = M.LongitudinalPersonality
BASE = 'claude/determined-hypatia-pnidyz'
SCR = os.path.dirname(os.path.abspath(__file__))
TAPER_SRC = M.git_show('t3_tfollow_taper', M.MPC_PATH)
# combined variant: also taper the jerk factor (aggressive-like brake onset at speed, unchanged at low speed)
COMBO_SRC = TAPER_SRC.replace(
  "  def set_weights(self, prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard):\n    jerk_factor = get_jerk_factor(personality)\n",
  "  def set_weights(self, prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard):\n    jerk_factor = get_jerk_factor(personality) * float(np.interp(self.x0[1], T_FOLLOW_TAPER_BP, [1.0, JERK_HIGHWAY_SCALE]))\n")
assert COMBO_SRC != TAPER_SRC
COMBO_SRC = COMBO_SRC.replace("T_FOLLOW_MIN = 1.0", "T_FOLLOW_MIN = 1.0\nJERK_HIGHWAY_SCALE = 0.5")
def scale_file(val):
  path = os.path.join(SCR, f"tf_{val}")
  open(path, "w").write(str(val)); return path
def mk(kind, scale):
  if kind == 'base':
    return M.load_ref(BASE)
  src = TAPER_SRC if kind == 'taper' else COMBO_SRC
  m = M.load_mpc_module(src=src); _, cfg = M.load_ref(BASE)
  m.T_FOLLOW_SCALE_FILE = scale_file(scale)
  return m, cfg
def hard_stop(m, cfg, pers, delay, tau, v=29.1, decel=3.0):
  r = M.run(m, cfg, 70.0, M.lead_brake(v, 50.0, decel, 0.0), v0=v, gap0=1.45 * v + 6.0, personality=pers, v_cruise=v + 5,
            record=True, plant_delay=delay, plant_tau=tau)
  t = np.array(r.t); g = np.array(r.gap)
  return g[np.searchsorted(t, 49.9)], g[t >= 50].min()
print("Lead brakes 3.0 m/s2 to a stop from 65 mph. Cells: steady gap / min gap during the stop (m)")
plants = [("fast 0.10+0.20", 0.10, 0.20), ("mid 0.15+0.35", 0.15, 0.35), ("slow 0.20+0.50", 0.20, 0.50)]
print(f"{'variant':30s} " + " ".join(f"{p[0]:>16s}" for p in plants))
variants = [("baseline standard", 'base', None, P.standard), ("baseline aggressive", 'base', None, P.aggressive)]
for sc in (0.95, 0.90, 0.85):
  variants.append((f"tfollow {sc} standard", 'taper', sc, P.standard))
for sc in (0.95, 0.90, 0.85):
  variants.append((f"tfollow+jerk {sc} standard", 'combo', sc, P.standard))
variants.append(("tfollow+jerk 0.85 aggressive", 'combo', 0.85, P.aggressive))
for name, kind, sc, pers in variants:
  cells = []
  for _, d, tau in plants:
    m, cfg = mk(kind, sc)
    g0, mg = hard_stop(m, cfg, pers, d, tau)
    cells.append(f"{g0:6.1f} / {mg:6.2f}")
  print(f"{name:30s} " + " ".join(f"{c:>16s}" for c in cells))
