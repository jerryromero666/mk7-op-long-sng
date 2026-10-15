import os, sys, importlib.util, numpy as np, mpcsim as M
from types import SimpleNamespace as NS
R = '/tmp/claude-0/-home-user-mk7-op-long-sng/48e1abac-3f14-57e3-a7c0-d6a536957af8/scratchpad/review'
sys.path.insert(0, os.path.join(R, 't3_tfollow_taper')); import common as TC
_s = importlib.util.spec_from_file_location('all_common', os.path.join(R, 'ALL', 'common.py')); AC = importlib.util.module_from_spec(_s); _s.loader.exec_module(AC)
P = M.LongitudinalPersonality
SCR = os.path.dirname(os.path.abspath(__file__))
# ---- brake ramp: 1.0 (current) vs 4.0 (branch), same MPC code; gradient honoured by the ESP (assumption)
m0, cfg = M.load_ref('fix_brake_ramp_4p0')
def stop_case(neg, v, d, model, pers=P.standard):
  AC.set_grad(neg, 4.0)
  m = M.load_mpc_module('fix_brake_ramp_4p0')
  kw = dict(lead_tau=1.5, alead_mode='true') if model == 'radarlike' else dict(lead_tau=0.3, alead_mode='lag')
  r = TC.run(m, cfg, 30 + v / d + 8, M.lead_brake(v, 30.0, d, 0.0), v0=v, gap0=m.get_T_FOLLOW(pers) * v + 6.0, v_cruise=v + 5, personality=pers, **kw)
  return float(np.array(r.gap)[np.array(r.t) >= 30].min())
def city_comfort(neg):
  AC.set_grad(neg, 4.0)
  m = M.load_mpc_module('fix_brake_ramp_4p0')
  r = M.run(m, cfg, 40.0, M.lead_const(0.0), v0=13.4, gap0=80.0, v_cruise=20, record=True)
  a = np.array(r.a_cmd); v = np.array(r.v)
  return r.final_gap, a.min(), np.max(np.abs(np.diff(a))) / 0.05
def follow_slowdown(neg):
  """lead at 45 mph eases off at 1.0 m/s2 to 25 mph (ordinary traffic slowdown): min gap and peak decel"""
  AC.set_grad(neg, 4.0)
  m = M.load_mpc_module('fix_brake_ramp_4p0')
  r = M.run(m, cfg, 50.0, M.lead_brake(20.1, 30.0, 1.0, 11.2), v0=20.1, gap0=1.45 * 20.1 + 6, v_cruise=25, record=True)
  t = np.array(r.t); g = np.array(r.gap); a = np.array(r.a_cmd)
  return g[t >= 30].min(), a[t >= 30].min()
print("BRAKE RAMP (ESP assumed to honour ACC_neg_Sollbeschl_Grad_02); min gap in m, negative = contact")
print(f"{'case':34s} {'ramp 1.0 (now)':>14s} {'ramp 4.0':>9s}")
for model in ('vision', 'radarlike'):
  for v in (29.1, 33.5):
    for d in (2.5, 3.0, 3.4):
      print(f"{model:9s} {v*2.237:3.0f}mph lead stops at {d:3.1f}   {stop_case(1.0, v, d, model):14.2f} {stop_case(4.0, v, d, model):9.2f}", flush=True)
c1, c4 = city_comfort(1.0), city_comfort(4.0)
print(f"city stop from 30 mph: final gap {c1[0]:.2f} vs {c4[0]:.2f} m; peak decel cmd {c1[1]:.2f} vs {c4[1]:.2f}; peak cmd jerk {c1[2]:.1f} vs {c4[2]:.1f}")
f1, f4 = follow_slowdown(1.0), follow_slowdown(4.0)
print(f"ordinary slowdown 45->25 mph at 1 m/s2: min gap {f1[0]:.2f} vs {f4[0]:.2f}; peak decel {f1[1]:.2f} vs {f4[1]:.2f}")
AC.set_grad(None, None)
# ---- stop distance: edit the live file while stopped behind a stopped lead
def edit_while_stopped(before, after, hold):
  p = os.path.join(SCR, 'sd_live')
  open(p, 'w').write(before)
  m, cfg2 = M.load_ref('t3_stop_distance_taper'); m.STOP_DIST_FILE = p
  orig = m.LongitudinalMpc.update; calls = [0]
  def upd(self, *a, **k):
    calls[0] += 1
    if calls[0] == 300:                           # t = 15 s, car long since stopped
      if after is None: os.remove(p)
      else: open(p, 'w').write(after)
    return orig(self, *a, **k)
  m.LongitudinalMpc.update = upd
  r = M.run(m, cfg2, 35.0, M.lead_const(0.0), v0=0.0, gap0=float(before.split()[0]) + 0.3, hold_release_s=hold, record=True)
  g = np.array(r.gap); t = np.array(r.t)
  return r.stop_to_pid, g[t < 14.9][-1] - g[-1], max(r.a_cmd)
print("\nSTOP DISTANCE file edited while stopped (flips, metres crept, max accel cmd)")
for before, after in (("8.5 4.5", "6.0 4.5"), ("8.5 4.5", None), ("6.0 4.5", "4.5 4.5"), ("6.0 4.5", "8.5 4.5")):
  for hold in (0.0, 0.3):
    print(f"  '{before}' -> {repr(after):10s} hold {hold}: flips/creep/amax = %d / %.2f / %.2f" % edit_while_stopped(before, after, hold), flush=True)
