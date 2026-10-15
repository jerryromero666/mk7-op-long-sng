import numpy as np, mpcsim as M
from types import SimpleNamespace as NS
BASE = 'claude/determined-hypatia-pnidyz'
base_src = M.git_show(BASE, M.MPC_PATH)
def launch(m, cfg):
  r = M.run(m, cfg, 20.0, M.lead_launch(t0=8.0, a=2.0, v_max=13.4), v0=0.0, gap0=6.5, record=True)
  v = np.array(r.v); return v[int(12/0.05)], v[int(14/0.05)], max(r.a_cmd)
def merge(m, cfg):
  r = M.run(m, cfg, 25.0, M.lead_const(0.0) if False else (lambda t, x, v: 0.0), v0=10.0, gap0=500.0, v_cruise=20.0, record=True) if False else None
  # no lead: cruise from 10 to 20 m/s
  f = lambda t, x, v: 0.0; f.v_init = 40.0
  r = M.run(m, cfg, 25.0, f, v0=10.0, gap0=1000.0, v_cruise=20.0, record=True)
  v = np.array(r.v); t = np.array(r.t); i = np.where(v >= 19.5)[0]
  return t[i[0]] if len(i) else float('nan')
def cutin_accel(m, cfg):
  """ego accelerating from 8 m/s toward 29 m/s cruise, no lead; at t=4 a car appears 12 m ahead at 10 m/s braking 2 m/s2 to 0."""
  plant = M.Plant(8.0); pl = M.Planner(m, cfg, init_v=8.0); lc = M.LongControl(); lc.state = M.PID
  cx = cv = None; ming = 1e9
  for k in range(int(14 / M.DT_CTRL)):
    t = k * M.DT_CTRL
    if t >= 4.0:
      if cx is None: cx, cv = plant.x + 12.0, 10.0
      cv = max(0.0, cv - 2.0 * M.DT_CTRL); cx += cv * M.DT_CTRL
      ming = min(ming, cx - plant.x)
    if k % 5 == 0:
      l = M.lead_msg(cx - plant.x, cv, -2.0) if cx is not None else M.NO_LEAD
      at, ss = pl.update(plant.v, plant.v == 0, NS(leadOne=l, leadTwo=l), 29.0)
    a = lc.update(plant.v, at, ss); plant.step(float(np.clip(a, -3.5, 2.0)))
  return ming
def noisy_follow(m, cfg, v=20.1, seeds=(1, 2, 3)):
  out = []
  for s in seeds:
    r = M.run(m, cfg, 35.0, M.lead_const(v), v0=v, gap0=1.45 * v + 6, v_cruise=v + 5, noise=(0.05 * (1.45 * v + 6), 0.5), seed=s, record=True)
    a = np.array(r.a_cmd)[100:]; out.append(np.sqrt(np.mean((np.diff(a) / 0.05) ** 2)))
  return float(np.mean(out))
_, cfg0 = M.load_ref(BASE)
print("ACCEL CAP options (sim now includes the 1.7 straight-wheel turn limit)")
print(f"{'cap':38s} {'v@4':>5s} {'v@6':>5s} {'amax':>5s} {'merge10->20 s':>13s} {'cut-in min gap':>14s}")
for name, bp, vals in [("base [1.6,1.2,0.8,0.6]", [0., 10., 25., 40.], [1.6, 1.2, 0.8, 0.6]),
                       ("branch [2.0,1.5,0.8,0.6]", [0., 10., 25., 40.], [2.0, 1.5, 0.8, 0.6]),
                       ("[2.0,1.2,0.8,0.6]", [0., 10., 25., 40.], [2.0, 1.2, 0.8, 0.6]),
                       ("[1.7,1.7,1.2,..] @ [0,6,10,25,40]", [0., 6., 10., 25., 40.], [1.7, 1.7, 1.2, 0.8, 0.6])]:
  cfg = dict(cfg0, A_CRUISE_MAX_BP=bp, A_CRUISE_MAX_VALS=vals)
  m = M.load_mpc_module(src=base_src); v4, v6, am = launch(m, cfg)
  m = M.load_mpc_module(src=base_src); mt = merge(m, cfg)
  m = M.load_mpc_module(src=base_src); cg = cutin_accel(m, cfg)
  print(f"{name:38s} {v4:5.2f} {v6:5.2f} {am:5.2f} {mt:13.2f} {cg:14.2f}")
print()
print("A_CHANGE_COST options")
sched_src = base_src.replace("    a_change_cost = A_CHANGE_COST if prev_accel_constraint else 0\n",
  "    a_change_cost = float(np.interp(self.x0[1], [3., 8.], [40., A_CHANGE_COST])) if prev_accel_constraint else 0\n")
assert sched_src != base_src
print(f"{'variant':30s} {'v@4':>5s} {'v@6':>5s} {'jerk rms 20 m/s':>15s} {'jerk rms 29 m/s':>15s}")
for name, src in [("base 75", base_src), ("flat 40 (branch)", base_src.replace("A_CHANGE_COST = 75.", "A_CHANGE_COST = 40.")),
                  ("40 below 3 m/s -> 75 at 8 m/s", sched_src)]:
  m = M.load_mpc_module(src=src); v4, v6, _ = launch(m, cfg0)
  j20 = noisy_follow(M.load_mpc_module(src=src), cfg0, 20.1); j29 = noisy_follow(M.load_mpc_module(src=src), cfg0, 29.1)
  print(f"{name:30s} {v4:5.2f} {v6:5.2f} {j20:15.3f} {j29:15.3f}")
