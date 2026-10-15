"""Driver coasts toward stopped traffic with cruise off, presses SET mid-slowdown.
Replays the planner's disengaged path (longitudinal_planner.py:102-121): while long control is off,
v_desired_filter = vEgo and a_desired = clip(aEgo); prev_accel_constraint False. On SET, v_cruise = vEgo."""
import os, sys, importlib.util, numpy as np, mpcsim as M
from types import SimpleNamespace as NS
R = '/tmp/claude-0/-home-user-mk7-op-long-sng/48e1abac-3f14-57e3-a7c0-d6a536957af8/scratchpad/review'
_s = importlib.util.spec_from_file_location('all_common', os.path.join(R, 'ALL', 'common.py')); AC = importlib.util.module_from_spec(_s); _s.loader.exec_module(AC)

def scenario(mpc_src=None, ref='claude/determined-hypatia-pnidyz', v_engage=13.4, gap_engage=50.0, detect=60.0,
             a_drv=-0.5, neg=1.0, record=False):
  AC.set_grad(neg, 4.0)
  m, cfg = M.load_ref(ref)
  if mpc_src is not None: m = M.load_mpc_module(src=mpc_src)
  # start 3 s of manual coasting before SET
  v0 = v_engage - a_drv * 3.0
  plant = M.Plant(v0); pl = M.Planner(m, cfg, init_v=v0); lc = M.LongControl(); lc.state = 'off'
  x_engage = v0 * 3.0 + 0.5 * a_drv * 9.0
  lead_x = x_engage + gap_engage
  engaged = False; v_cruise = 40.0; at, ss = 0.0, False; a_send = a_drv
  out = NS(t=[], v=[], gap=[], a=[], areq=[])
  peak = 0.0; t_hard = 0.0; v_at_20 = None; seen_at = None
  for k in range(int(30 / M.DT_CTRL)):
    t = k * M.DT_CTRL
    gap = lead_x - plant.x
    if not engaged and t >= 3.0:
      engaged = True; v_cruise = plant.v; lc.state = M.PID   # off -> pid (moving, should_stop False)
    if k % 5 == 0:
      vis = gap <= detect
      if vis and seen_at is None: seen_at = (gap, plant.v, engaged)
      l = NS(status=True, dRel=gap, vLead=0.0, aLeadK=0.0, aLeadTau=0.3, modelProb=1.0) if vis else M.NO_LEAD
      rs = NS(leadOne=l, leadTwo=l)
      if not engaged:
        pl.v_f_x = plant.v                                   # reset_state path
        pl.a_desired = float(np.clip(plant.a, M.ACCEL_MIN, pl.get_max_accel(plant.v)))
        pl.mpc.set_weights(False); pl.mpc.set_cur_state(pl.v_f_x, pl.a_desired); pl.mpc.update(rs, v_cruise)
        pl.prev_accel_clip = [M.ACCEL_MIN, pl.get_max_accel(plant.v)]
      else:
        at, ss = pl.update(plant.v, plant.v == 0, rs, v_cruise)
    if engaged:
      a = lc.update(plant.v, at, ss)
      if k % 2 == 0: a_send = float(np.clip(a, -3.5, 2.0))
    else:
      a_send = a_drv
    plant.step(a_send)
    if engaged:
      peak = min(peak, plant.a)
      if plant.a < -2.5: t_hard += M.DT_CTRL
      if v_at_20 is None and gap <= 20.0: v_at_20 = plant.v
    if record and k % 5 == 0:
      out.t.append(t); out.v.append(plant.v); out.gap.append(gap); out.a.append(plant.a); out.areq.append(a_send)
  AC.set_grad(None, None)
  return NS(final_gap=lead_x - plant.x, peak=peak, t_hard=t_hard, v_at_20=v_at_20 if v_at_20 is not None else 0.0, out=out)

if __name__ == "__main__":
  print("Coasting at -0.5 m/s2, SET pressed at the given speed and distance from a stopped car.")
  print("Cells: final gap m (negative = contact) / peak real decel m/s2 / seconds harder than 2.5 m/s2")
  for detect in (60.0, 100.0):
    print(f"\n=== camera sees the stopped car from {detect:.0f} m ===")
    print(f"{'SET at':22s} {'ramp 1.0 (current)':>24s} {'ramp 4.0 (fix branch)':>24s}")
    for v, mph in ((11.2, 25), (13.4, 30), (15.6, 35), (17.9, 40)):
      for g in (35.0, 50.0, 70.0, 90.0):
        c = []
        for neg in (1.0, 4.0):
          r = scenario(v_engage=v, gap_engage=g, detect=detect, neg=neg)
          c.append(f"{r.final_gap:6.1f} / {r.peak:5.2f} / {r.t_hard:3.1f}s")
        print(f"{mph} mph, {g:3.0f} m ahead   {c[0]:>24s} {c[1]:>24s}", flush=True)
