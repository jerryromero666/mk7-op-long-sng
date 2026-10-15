import os, sys, importlib.util, numpy as np, mpcsim as M
from types import SimpleNamespace as NS
R = '/tmp/claude-0/-home-user-mk7-op-long-sng/48e1abac-3f14-57e3-a7c0-d6a536957af8/scratchpad/review'
_s = importlib.util.spec_from_file_location('all_common', os.path.join(R, 'ALL', 'common.py')); AC = importlib.util.module_from_spec(_s); _s.loader.exec_module(AC)
P = M.LongitudinalPersonality
T1 = 'claude/determined-hypatia-pnidyz'
def L(ref, files=None):
  m, cfg = M.load_ref(ref)
  if hasattr(m, 'STOP_DIST_FILE'): m.STOP_DIST_FILE = files or '/nonexistent'
  return m, cfg
def launch(ref, l2='mirror', hold=0.0):
  m, cfg = L(ref); r = M.run(m, cfg, 20.0, M.lead_launch(8.0, 2.0, 13.4), v0=0.0, gap0=6.5, record=True, lead2=l2, hold_release_s=hold)
  t = np.array(r.t); v = np.array(r.v); i = np.where((t >= 8) & (v > 0.1))[0]
  return t[i[0]] - 8.0, v[int(12 / .05)], v[int(14 / .05)]
def nolead(ref):
  m, cfg = L(ref); f = lambda t, x, v: 0.0; f.v_init = 40.0
  r = M.run(m, cfg, 8.0, f, v0=0.0, gap0=1000.0, v_cruise=20.0, record=True)
  return r.v[int(2 / .05)], r.v[int(6 / .05)]
def flips(ref, l2='mirror', hold=0.0):
  m, cfg = L(ref)
  return sum(M.run(L(ref)[0], cfg, 60.0, M.lead_const(0.0), v0=0.0, gap0=6.5, noise=(0.3, 0.2), seed=s, lead2=l2, hold_release_s=hold).stop_to_pid for s in range(1, 7))
def steady(ref, v, pers):
  m, cfg = L(ref); return M.run(m, cfg, 60.0, M.lead_const(v), v0=v, gap0=1.45 * v + 6.0, v_cruise=v + 5, personality=pers).final_gap
def city(ref, files=None):
  m, cfg = L(ref, files); return M.run(m, cfg, 40.0, M.lead_const(0.0), v0=13.4, gap0=80.0, v_cruise=20).final_gap
print("LAUNCH / STANDSTILL (lead launches at 2.0 m/s2; flips = stopping->pid at standstill, noise 0.3/0.2, 6 seeds)")
print(f"{'branch':26s} {'delay':>5s} {'v@4':>5s} {'v@6':>5s} {'v@6 l2abs':>9s} {'delay hold.3':>12s} {'noLead v@2/v@6':>15s} {'flips m/a':>9s} {'flips hold.3':>12s} {'city':>5s}")
for b in [T1, 't2_lead2_phantom_fix', 't2_launch_floor_2p0', 't2_accel_cap_launch', 't2_achange_cost_launch', 't2_cruise_max_accel_2p0',
          'fix_brake_ramp_4p0', 't3_highway_aggressive', 't3_stop_distance_taper']:
  d, v4, v6 = launch(b); _, _, v6a = launch(b, 'absent'); dh, _, _ = launch(b, hold=0.3); n2, n6 = nolead(b)
  print(f"{b:26s} {d:5.2f} {v4:5.2f} {v6:5.2f} {v6a:9.2f} {dh:12.2f} {n2:7.2f}/{n6:5.2f}   {flips(b):4d}/{flips(b, 'absent'):4d} {flips(b, hold=0.3):12d} {city(b):5.2f}", flush=True)
print("\nSTEADY GAP at 65 mph (m)")
for b in [T1, 't3_highway_aggressive', 't3_stop_distance_taper']:
  print(f"  {b:26s} relaxed {steady(b, 29.1, P.relaxed):6.2f}  standard {steady(b, 29.1, P.standard):6.2f}  aggressive {steady(b, 29.1, P.aggressive):6.2f}   (standard at 25 mph {steady(b, 11.2, P.standard):5.2f})", flush=True)
