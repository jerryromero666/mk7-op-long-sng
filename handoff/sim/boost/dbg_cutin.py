import sys, numpy as np, boostsim as B
M = B.M
T0 = 8.0
SYNC = len(sys.argv) > 1 and sys.argv[1] == 'sync'
mb, cfgb = M.load_ref('t2_lead2_phantom_fix')
for name in ('base', 'boost'):
  bm = B.load_boost('WT') if name == 'boost' else None
  log = []
  def probe(t, pl, plant, rs):
    if SYNC and (pl.boost > 0 or (pl.lb is not None and pl.lb.event.startswith("abort"))):
      pl.v_f_x = max(pl.v_f_x, plant.v)
    if T0 + 2.3 <= t <= T0 + 5.0:
      log.append((t - T0, rs.leadOne.dRel, pl.a_mpc, pl.boost, pl.output_a_target, plant.a, plant.v, pl.v_f_x, plant.x))
  r = B.run(mb, cfgb, bm, T0 + 8, B.launch(T0, 1.6), gap0=4.5, mode='vision', seed=1, noise_kw=dict(sd=0.3, sv=0.4, sa=0.3),
            lead2=dict(t_on=T0 + 2.5, d=4.0, v=0.0, a=0.0, role='cutin'), probe=probe)
  print(name, r.events, 'min gap', r.gap_true[r.t >= T0].min())
  print("   t   dRel  a_mpc boost a_out a_act v_ego v_mpc")
  for l in log[::2]:
    print(" ".join(f"{x:5.2f}" for x in l[:8]))
