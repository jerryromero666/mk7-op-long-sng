# 60 s at a stop behind a stationary lead with vision-like noise: boost fires (must be 0), stop->go flips, max speed.
import sys, numpy as np, boostsim as B, collections
from multiprocessing import Pool
M = B.M
BOOST_REF = sys.argv[1] if len(sys.argv) > 1 else 'WT'
SEEDS = range(1, int(sys.argv[2]) + 1 if len(sys.argv) > 2 else 21)
NOISE = {'sd.3 sv.3 sa.3': dict(sd=0.3, sv=0.3, sa=0.3), 'sd.3 sv.5 sa.3': dict(sd=0.3, sv=0.5, sa=0.3),
         'sd.3 sv.5 corr': dict(sd=0.3, sv=0.5, sa=0.3, white_frac=0.2), 'stress sd.4 sv.6 sa.5 corr': dict(sd=0.4, sv=0.6, sa=0.5, white_frac=0.2)}
def job(a):
  name, nk, gap0, seed = a
  if name == 'base':
    m, cfg = M.load_ref('t2_lead2_phantom_fix'); bm = None
  else:
    m, cfg = M.load_ref('t2_lead2_phantom_fix' if BOOST_REF == 'WT' else BOOST_REF); bm = B.load_boost(BOOST_REF)
  r = B.run(m, cfg, bm, 60.0, B.stationary(), gap0=gap0, mode='vision', seed=seed, noise_kw=NOISE[nk])
  return (name, nk), (sum(e[1].startswith('fired') for e in r.events), r.stop_to_pid, float(r.v.max()), float(r.gap.min() - gap0))
if __name__ == '__main__':
  jobs = [(n, nk, g, s) for n in ('base', 'boost') for nk in NOISE for g in (3.5, 4.5, 5.5) for s in SEEDS]
  with Pool(4) as p:
    res = p.map(job, jobs)
  agg = collections.defaultdict(list)
  for k, v in res: agg[k].append(v)
  print(f"stationary lead 60 s, gaps 3.5/4.5/5.5 x seeds {SEEDS.start}..{SEEDS.stop - 1}  (boost={BOOST_REF})")
  print(f"{'noise':28s} {'runs':>4s} | base flips vmax | boost fires flips vmax")
  for nk in NOISE:
    b = np.array(agg[('base', nk)]); x = np.array(agg[('boost', nk)])
    print(f"{nk:28s} {len(x):4d} | {int(b[:,1].sum()):10d} {b[:,2].max():4.2f} | {int(x[:,0].sum()):11d} {int(x[:,1].sum()):5d} {x[:,2].max():4.2f}")
