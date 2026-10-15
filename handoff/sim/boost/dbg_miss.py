import numpy as np, boostsim as B
M = B.M
T0 = 8.0
mb, cfgb = M.load_ref('t2_lead2_phantom_fix')
for al in (1.0, 1.6, 2.3):
  for seed in range(1, 6):
    bm = B.load_boost('WT', dict(V_TRIGGER=1.2))
    log = []
    def probe(t, pl, plant, rs):
      lb = pl.lb
      if t >= T0 - 0.2 and t <= T0 + 3:
        log.append((round(t - T0, 2), lb.armed, lb.active, round(lb.v_f, 2), round((lb.d_f - lb.d_stop) if lb.d_stop is not None else -9, 2), round(pl.a_mpc, 2), round(plant.v, 2), lb.trigger_cnt, lb.spent))
    r = B.run(mb, cfgb, bm, T0 + 3, B.launch(T0, al), gap0=4.5, mode='vision', seed=seed, noise_kw=dict(sv=0.4), probe=probe)
    f = [e for e in r.events]
    print(al, seed, f)
    if not any(e[1] == 'fired' for e in f):
      for l in log[::2]:
        print('   ', l)
