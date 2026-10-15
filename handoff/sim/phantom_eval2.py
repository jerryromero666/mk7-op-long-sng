import sys, os, numpy as np, mpcsim as M
from types import SimpleNamespace as NS
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "review", "t2_lead2_phantom_fix"))
from variants import VARIANTS
BASE = 'claude/determined-hypatia-pnidyz'
srcs = {"base": M.git_show(BASE, M.MPC_PATH), "minimal (branch)": M.git_show('t2_lead2_phantom_fix', M.MPC_PATH),
        "perlead_nodec": VARIANTS['perlead_nodec']}
_, cfg = M.load_ref(BASE)
def mk(src): return M.load_mpc_module(src=src)
def launch(src, l2):
  r = M.run(mk(src), cfg, 20.0, M.lead_launch(t0=8.0, a=2.0, v_max=13.4), v0=0.0, gap0=6.5, record=True, lead2=l2)
  return r.v[int(14.0 / 0.05)]
def flips(src, l2, gap0=6.5):
  return sum(M.run(mk(src), cfg, 60.0, M.lead_const(0.0), v0=0.0, gap0=gap0, noise=(0.3, 0.2), seed=s, lead2=l2).stop_to_pid for s in range(1, 7))
def city(src, l2):
  return M.run(mk(src), cfg, 40.0, M.lead_const(0.0), v0=13.4, gap0=80.0, v_cruise=20, lead2=l2).final_gap
def custom(src, T, v0, rs_fn, v_cruise):
  m = mk(src); plant = M.Plant(v0); pl = M.Planner(m, cfg, init_v=v0); lc = M.LongControl(); lc.state = M.PID if v0 > 0.1 else M.STOPPING
  mina = 9; at, ss = 0, False; ming = 1e9; k0 = None
  for k in range(int(T / M.DT_CTRL)):
    t = k * M.DT_CTRL
    if k % 5 == 0:
      rs, g = rs_fn(k // 5, plant)
      at, ss = pl.update(plant.v, plant.v == 0, rs, v_cruise)
    a = lc.update(plant.v, at, ss); plant.step(float(np.clip(a, -3.5, 2.0)))
    if t > 3: mina = min(mina, a)
  return mina, plant.v
def dropout_case(v, n_drop, l2):
  """steady following at v; leadOne (and leadTwo if mirror) drop out for n_drop planner frames at frame 100"""
  st = {'lx': 1.45 * v + 6.0}
  def fn(f, plant):
    st['lx'] += v * 0.05
    g = st['lx'] - plant.x
    l = M.lead_msg(g, v, 0.0)
    drop = 100 <= f < 100 + n_drop
    l1 = M.NO_LEAD if drop else l
    l2v = (M.NO_LEAD if drop else l) if l2 == 'mirror' else M.NO_LEAD
    return NS(leadOne=l1, leadTwo=l2v), g
  return custom(src_cur, 10.0, v, fn, v + 3)
def outlier_case(v, l2, frac=0.5):
  """steady following at v; every 2 s one frame reads dRel*frac (false-close outlier)"""
  st = {'lx': 1.45 * v + 6.0}
  def fn(f, plant):
    st['lx'] += v * 0.05
    g = st['lx'] - plant.x
    d = g * frac if (f % 40 == 20 and f > 60) else g
    l = M.lead_msg(d, v, 0.0)
    return NS(leadOne=l, leadTwo=l if l2 == 'mirror' else M.NO_LEAD), g
  return custom(src_cur, 12.0, v, fn, v + 3)
print(f"{'variant':18s} {'launch m/a':>11s} {'flips m/a':>10s} {'flips gap8 m/a':>14s} {'city m/a':>11s} | {'drop1@10 m/a':>13s} {'drop3@6 m/a':>13s} | {'outlier@5 m/a':>13s} {'outlier@10 m/a':>14s}")
for name, src_cur in srcs.items():
  lm, la = launch(src_cur, 'mirror'), launch(src_cur, 'absent')
  fm, fa = flips(src_cur, 'mirror'), flips(src_cur, 'absent')
  f8m, f8a = flips(src_cur, 'mirror', 8.0), flips(src_cur, 'absent', 8.0)
  cm, ca = city(src_cur, 'mirror'), city(src_cur, 'absent')
  d1m, d1a = dropout_case(10.0, 1, 'mirror')[0], dropout_case(10.0, 1, 'absent')[0]
  d3m, d3a = dropout_case(6.0, 3, 'mirror')[0], dropout_case(6.0, 3, 'absent')[0]
  o5m, o5a = outlier_case(5.0, 'mirror')[0], outlier_case(5.0, 'absent')[0]
  o10m, o10a = outlier_case(10.0, 'mirror')[0], outlier_case(10.0, 'absent')[0]
  print(f"{name:18s} {lm:5.2f}/{la:5.2f} {fm:4d}/{fa:4d} {f8m:6d}/{f8a:6d} {cm:5.2f}/{ca:5.2f} | {d1m:6.2f}/{d1a:6.2f} {d3m:6.2f}/{d3a:6.2f} | {o5m:6.2f}/{o5a:6.2f} {o10m:6.2f}/{o10a:6.2f}", flush=True)
