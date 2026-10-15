# Trigger tuning: false fires (stationary, creeping leads) vs fire latency on real departures.
import sys, numpy as np, boostsim as B, itertools, collections
from multiprocessing import Pool
M = B.M
T0 = 8.0
NOISE = {'std': dict(sv=0.4), 'sv.5': dict(sv=0.5), 'corr': dict(sv=0.5, white_frac=0.2), 'stress': dict(sd=0.4, sv=0.6, sa=0.5, white_frac=0.2)}
def job(args):
  ov, kind, p, nkname, seed = args
  mb, cfgb = M.load_ref('t2_lead2_phantom_fix')
  bm = B.load_boost('WT', ov)
  nk = NOISE[nkname]
  if kind == 'stat':
    r = B.run(mb, cfgb, bm, 60.0, B.stationary(), gap0=p, mode='vision', seed=seed, noise_kw=nk)
    return kind, sum(e[1].startswith('fired') for e in r.events)
  if kind == 'creep':
    vc, tc = p
    r = B.run(mb, cfgb, bm, T0 + 12, B.creep(T0, vc, tc), gap0=4.5, mode='vision', seed=seed, noise_kw=nk)
    return kind, sum(e[1].startswith('fired') for e in r.events)
  r = B.run(mb, cfgb, bm, T0 + 3, B.launch(T0, p), gap0=4.5, mode='vision', seed=seed, noise_kw=nk)
  f = [e[0] - T0 for e in r.events if e[1].startswith('fired')]
  return (kind, p), (f[0] if f else np.nan)
def evaluate(ov, pool, seeds_stat=range(1, 9)):
  jobs = [(ov, 'stat', g, nk, s) for nk in ('sv.5', 'corr', 'stress') for g in (3.5, 4.5, 5.5) for s in seeds_stat]
  jobs += [(ov, 'creep', (vc, tc), nk, s) for nk in ('sv.5', 'corr', 'stress') for vc in (0.3, 0.4, 0.5, 0.6) for tc in (2.0, 5.0) for s in range(1, 6)]
  jobs += [(ov, 'launch', al, 'std', s) for al in (1.0, 1.6, 2.3) for s in range(1, 6)]
  res = pool.map(job, jobs)
  stat = sum(v for k, v in res if k == 'stat'); creep = sum(v for k, v in res if k == 'creep')
  lat = collections.defaultdict(list)
  for k, v in res:
    if isinstance(k, tuple):
      lat[k[1]].append(v)
  return stat, creep, {al: np.nanmean(v) for al, v in lat.items()}, {al: int(np.sum(np.isnan(v))) for al, v in lat.items()}
if __name__ == '__main__':
  cands = eval(sys.argv[1])
  with Pool(4) as pool:
    for ov in cands:
      st, cr, lat, miss = evaluate(ov, pool)
      print(f"{str(ov):75s} stat fires {st:3d}  creep fires {cr:3d}  fire latency " + " ".join(f"aL{al}:{lat[al]:.2f}" for al in sorted(lat)) + f"  missed {sum(miss.values())}", flush=True)
