# Consequence of a forced false fire at standstill behind a stationary lead (thresholds set so noise fires it).
import numpy as np, boostsim as B
M = B.M
mb, cfgb = M.load_ref('t2_lead2_phantom_fix')
OV = dict(V_TRIGGER=0.25, D_TRIGGER=-1.0, A_MPC_GO=-1.0, TRIGGER_FRAMES=1)
worst = dict(travel=0, vmax=0, bmax=0, gmin=99)
nf = 0
for nk in (dict(sv=0.4), dict(sd=0.4, sv=0.6, sa=0.5, white_frac=0.2)):
  for gap0 in (3.5, 4.5, 5.5):
    for seed in range(1, 11):
      bm = B.load_boost('WT', OV)
      x0 = {}
      def probe(t, pl, plant, rs):
        if pl.lb.event == 'fired':
          x0.setdefault('x', plant.x)
      r = B.run(mb, cfgb, bm, 30.0, B.stationary(), gap0=gap0, mode='vision', seed=seed, noise_kw=nk, probe=probe)
      fires = [e for e in r.events if e[1].startswith('fired')]
      nf += len(fires)
      if fires:
        worst['vmax'] = max(worst['vmax'], r.v.max()); worst['bmax'] = max(worst['bmax'], r.boost.max())
        worst['gmin'] = min(worst['gmin'], r.gap.min() - gap0)
      ends = [e[1] for e in r.events if e[1] != 'fired']
      if seed <= 2:
        print(gap0, nk, 'fires', len(fires), 'ends', sorted(set(ends)), f"vmax {r.v.max():.2f} boost max {r.boost.max():.2f} gap change min {r.gap.min()-gap0:+.2f} a max {r.a.max():.2f}")
print('total fires', nf, 'worst', worst)
