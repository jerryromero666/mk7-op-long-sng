import numpy as np, mpcsim as M
from types import SimpleNamespace as NS
BASE = 'claude/determined-hypatia-pnidyz'
base_src = M.git_show(BASE, M.MPC_PATH)
# minimal alternative: shared state kept, only leadOne may use the dropout hold
mini_src = base_src.replace("  def process_lead(self, lead):", "  def process_lead(self, lead, idx=0):") \
  .replace("      self._lead_dropout_frames += 1\n      if self._last_dRel < 25.0 and v_ego > 1.0 and self._lead_dropout_frames <= 10:",
           "      if idx == 0:\n        self._lead_dropout_frames += 1\n      if idx == 0 and self._last_dRel < 25.0 and v_ego > 1.0 and self._lead_dropout_frames <= 10:") \
  .replace("    lead_xv_1 = self.process_lead(radarstate.leadTwo)", "    lead_xv_1 = self.process_lead(radarstate.leadTwo, 1)")
assert mini_src.count("idx == 0") == 2
srcs = {"base (shared state)": base_src, "minimal (hold leadOne only)": mini_src,
        "per-lead fix (branch)": M.git_show('t2_lead2_phantom_fix', M.MPC_PATH)}
def cutin(src):
  """Ego 8 m/s behind leadOne at 18 m (8 m/s). A car cuts in at 9 m doing 5 m/s:
  model shows it as leadTwo (leadsV3[1]) for 1.0 s before it becomes leadOne."""
  m = M.load_mpc_module(src=src); _, cfg = M.load_ref(BASE)
  plant = M.Plant(8.0); pl = M.Planner(m, cfg, init_v=8.0); lc = M.LongControl(); lc.state = M.PID
  l1x, l1v, cx, cv = 18.0, 8.0, 9.0, 5.0
  ming = 1e9; a_hist = []
  for k in range(int(12 / M.DT_CTRL)):
    t = k * M.DT_CTRL
    l1x += l1v * M.DT_CTRL
    if t < 3.0:
      cx = plant.x + 9.0          # cut-in car appears 9 m ahead at t = 3 s
    else:
      cx += cv * M.DT_CTRL
    g_cut = cx - plant.x
    if k % 5 == 0:
      if t < 3.0:
        rs = NS(leadOne=M.lead_msg(l1x - plant.x, l1v, 0), leadTwo=M.lead_msg(l1x - plant.x, l1v, 0))
      elif t < 4.0:
        rs = NS(leadOne=M.lead_msg(l1x - plant.x, l1v, 0), leadTwo=M.lead_msg(g_cut, cv, 0))
      else:
        rs = NS(leadOne=M.lead_msg(g_cut, cv, 0), leadTwo=M.lead_msg(g_cut, cv, 0))
      at, ss = pl.update(plant.v, plant.v == 0, rs, 20.0)
    a = lc.update(plant.v, at, ss); a_hist.append(a)
    plant.step(float(np.clip(a, -3.5, 2.0)))
    if t >= 3.0: ming = min(ming, g_cut)
  return ming, min(a_hist[300:420])
def launch_ph(src, lead2):
  m = M.load_mpc_module(src=src); _, cfg = M.load_ref(BASE)
  r = M.run(m, cfg, 20.0, M.lead_launch(t0=8.0, a=2.0, v_max=13.4), v0=0.0, gap0=6.5, record=True, lead2=lead2)
  return r.v[int(14.0 / 0.05)]
def flips(src):
  m = M.load_mpc_module(src=src); _, cfg = M.load_ref(BASE)
  return sum(M.run(m, cfg, 60.0, M.lead_const(0.0), v0=0.0, gap0=6.5, noise=(0.3, 0.2), seed=s).stop_to_pid for s in (1, 2, 3, 4, 5, 6))
print(f"{'variant':30s} {'launch v@6 mirror':>17s} {'absent':>7s} {'flips(6 seeds)':>14s} {'cut-in min gap':>14s} {'min a during preview':>20s}")
for name, src in srcs.items():
  mg, ma = cutin(src)
  print(f"{name:30s} {launch_ph(src, 'mirror'):17.2f} {launch_ph(src, 'absent'):7.2f} {flips(src):14d} {mg:14.2f} {ma:20.2f}")

print()
stock_src = M.git_show('eabb2aa', M.MPC_PATH)
# stock process_lead dropped into the current file (keeps A_CHANGE 75 / STOP_DISTANCE 7.5 so only the lead handling differs)
import re
cur = base_src
s_start = stock_src.index("  def process_lead(self, lead):"); s_end = stock_src.index("  def update(self, radarstate")
c_start = cur.index("  def process_lead(self, lead):"); c_end = cur.index("  def update(self, radarstate")
stock_lead_src = cur[:c_start] + stock_src[s_start:s_end] + cur[c_end:]
for name, src in {"stock process_lead (no clamp)": stock_lead_src}.items():
  mg, ma = cutin(src)
  print(f"{name:30s} {launch_ph(src, 'mirror'):17.2f} {launch_ph(src, 'absent'):7.2f} {flips(src):14d} {mg:14.2f} {ma:20.2f}")
