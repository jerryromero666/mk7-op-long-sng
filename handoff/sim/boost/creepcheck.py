import sys, numpy as np, boostsim as B, collections
from multiprocessing import Pool
M = B.M
T0 = 8.0
OV = eval(sys.argv[1]) if len(sys.argv) > 1 else {}
NOISE = {'sv.4': dict(sv=0.4), 'sv.5': dict(sv=0.5), 'corr': dict(sv=0.5, white_frac=0.2), 'stress': dict(sd=0.4, sv=0.6, sa=0.5, white_frac=0.2)}
def job(a):
  name, vc, tc, nk, seed = a
  mb, cfgb = M.load_ref('t2_lead2_phantom_fix')
  bm = B.load_boost('WT', OV) if name == 'boost' else None
  r = B.run(mb, cfgb, bm, T0 + 14, B.creep(T0, vc, tc), gap0=4.5, mode='vision', seed=seed, noise_kw=NOISE[nk])
  post = r.t >= T0
  return (name, vc, nk), (sum(e[1] == 'fired' for e in r.events), r.v[post].max(), r.gap[post].min(), r.a[post].max(), r.boost.max())
if __name__ == '__main__':
  jobs = [(n, vc, tc, nk, s) for n in ('base', 'boost') for vc in (0.3, 0.4, 0.5, 0.6) for tc in (2.0, 5.0) for nk in NOISE for s in range(1, 6)]
  with Pool(4) as p:
    res = p.map(job, jobs)
  agg = collections.defaultdict(list)
  for k, v in res:
    agg[k].append(v)
  print(OV)
  print('creep v  noise   | base: vmax  gmin  amax | boost: fires/runs vmax gmin amax bmax')
  for vc in (0.3, 0.4, 0.5, 0.6):
    for nk in NOISE:
      b = np.array(agg[('base', vc, nk)]); x = np.array(agg[('boost', vc, nk)])
      print(f"{vc:.1f}  {nk:7s} | {b[:,1].max():5.2f} {b[:,2].min():5.2f} {b[:,3].max():5.2f} | {int(x[:,0].sum()):2d}/{len(x)} {x[:,1].max():5.2f} {x[:,2].min():5.2f} {x[:,3].max():5.2f} {x[:,4].max():4.2f}")
