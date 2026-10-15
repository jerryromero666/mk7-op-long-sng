import sys, numpy as np, boostsim as B
M = B.M
T0 = 8.0
d = float(sys.argv[1]); von = float(sys.argv[2]); vcut = float(sys.argv[3]); seed = int(sys.argv[4]) if len(sys.argv) > 4 else 1
OV = eval(sys.argv[5]) if len(sys.argv) > 5 else {}
mb, cfgb = M.load_ref('t2_lead2_phantom_fix')
for name in ('base', 'boost'):
  bm = B.load_boost('WT', OV) if name == 'boost' else None
  log = []; tc = {}
  def probe(t, pl, plant, rs):
    if plant.v >= von and 't' not in tc and t > T0: tc['t'] = t
    if 't' in tc and t <= tc['t'] + 3.0 and t >= tc['t'] - 0.2:
      log.append((t - tc['t'], rs.leadOne.dRel, pl.a_mpc, pl.boost, pl.output_a_target, plant.a, plant.v, pl.v_f_x, pl.a_desired, pl.mpc.crash_cnt))
  r = B.run(mb, cfgb, bm, T0 + 10, B.launch(T0, 1.6), gap0=4.5, mode='vision', seed=seed, noise_kw=dict(sd=0.3, sv=0.4, sa=0.3),
            lead2=dict(t_on=T0, v_on=von, d=d, v=vcut, a=0.0, role='cutin'), probe=probe)
  print(name, r.events, 'min gap', round(r.gap_true[r.t >= T0].min(), 2))
  print("   t   dRel  a_mpc boost a_out a_act v_ego v_mpc a_mpcst crash")
  for l in log[::2]:
    print(" ".join(f"{x:5.2f}" for x in l))
