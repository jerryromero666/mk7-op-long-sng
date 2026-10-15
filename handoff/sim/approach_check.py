import os, importlib.util, numpy as np, mpcsim as M
from types import SimpleNamespace as NS
R = '/tmp/claude-0/-home-user-mk7-op-long-sng/48e1abac-3f14-57e3-a7c0-d6a536957af8/scratchpad/review'
_s = importlib.util.spec_from_file_location('all_common', os.path.join(R, 'ALL', 'common.py')); AC = importlib.util.module_from_spec(_s); _s.loader.exec_module(AC)
P = M.LongitudinalPersonality
def approach(ref, v0, detect, neg, lead2='mirror', tau=0.3):
  """Ego at v0 toward a STOPPED car that vision only reports once it is within `detect` m."""
  AC.set_grad(neg, 4.0)
  m, cfg = M.load_ref(ref)
  plant = M.Plant(v0); pl = M.Planner(m, cfg, init_v=v0); lc = M.LongControl(); lc.state = M.PID
  lead_x = detect + 120.0          # starts well out of view
  at, ss = 0.0, False; a_send = 0.0
  brake_start_d = None; peak_dec = 0.0; peak_req = 0.0; first_seen_v = None
  for k in range(int(40 / M.DT_CTRL)):
    gap = lead_x - plant.x
    if k % 5 == 0:
      vis = gap <= detect
      if vis and first_seen_v is None: first_seen_v = plant.v
      l = NS(status=True, dRel=gap, vLead=0.0, aLeadK=0.0, aLeadTau=tau, modelProb=1.0) if vis else M.NO_LEAD
      rs = NS(leadOne=l, leadTwo=l if lead2 == 'mirror' else M.NO_LEAD)
      at, ss = pl.update(plant.v, plant.v == 0, rs, v0 + 3)
    a = lc.update(plant.v, at, ss)
    if k % 2 == 0: a_send = float(np.clip(a, -3.5, 2.0))
    plant.step(a_send)
    if brake_start_d is None and plant.a < -0.5: brake_start_d = gap
    peak_dec = min(peak_dec, plant.a); peak_req = min(peak_req, a_send)
  AC.set_grad(None, None)
  return brake_start_d, peak_dec, peak_req, lead_x - plant.x
rows = [("current build (ramp 1.0)", 'claude/determined-hypatia-pnidyz', 1.0), ("ghost fix only (ramp 1.0)", 't2_lead2_phantom_fix', 1.0),
        ("brake-ramp fix (ramp 4.0)", 'fix_brake_ramp_4p0', 4.0)]
print("Stopped car ahead, seen by vision only within D metres. Cells: real decel peak m/s2 / stop gap m  (contact if gap < 0)")
print("(assumes the ESP obeys the ramp value; 'brakes at' = distance when real decel passes 0.5)")
for v0, mph in ((13.4, 30), (17.9, 40), (22.4, 50)):
  for D in (80, 60, 45):
    cells = []
    for name, ref, neg in rows:
      bs, pk, rq, fg = approach(ref, v0, D, neg)
      cells.append(f"{pk:5.2f} / {fg:5.2f} (brakes at {bs:4.0f} m)")
    print(f"{mph} mph, seen at {D:2d} m: " + " | ".join(cells), flush=True)
print("\ncolumns:", " | ".join(r[0] for r in rows))
