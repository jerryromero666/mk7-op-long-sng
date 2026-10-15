import sys, numpy as np, boostsim as B, collections
from multiprocessing import Pool
M = B.M
T0 = 8.0
NK = dict(sd=0.3, sv=0.4, sa=0.3)
VARS = {'V1 v while boosting': None, 'V2 v until caught up': {'v_until'}, 'V3 V2 + a while boosting': {'v_until', 'a'}}
def scen():
  S = []
  for d in (4.0, 6.0):
    for von in (1.0, 2.0):
      for vcut in (0.0, 1.0):
        S.append((f"cutin {d} v{vcut} @ego{von}", dict(lead_a=B.launch(T0, 1.6), lead2=dict(t_on=T0, v_on=von, d=d, v=vcut, a=0.0, role='cutin'))))
  for tb in (1.0, 2.0, 3.0):
    S.append((f"brake 4.0 after {tb}", dict(lead_a=B.launch_then_brake(T0, 1.6, tb, 4.0))))
  S.append(("launch 1.6", dict(lead_a=B.launch(T0, 1.6))))
  S.append(("launch 1.0", dict(lead_a=B.launch(T0, 1.0))))
  return S
def job(a):
  vname, i, seed = a
  B.EXPLORE_SYNC = VARS[vname] if vname != 'base' else None
  mb, cfgb = M.load_ref('t2_lead2_phantom_fix')
  bm = B.load_boost('WT') if vname != 'base' else None
  title, kw = scen()[i]; kw = dict(kw)
  if 'lead2' in kw: kw['lead2'] = dict(kw['lead2'])
  la = kw.pop('lead_a')
  r = B.run(mb, cfgb, bm, T0 + 10, la, gap0=4.5, mode='vision', seed=seed, noise_kw=NK, **kw)
  m = B.launch_metrics(r, T0)
  return (vname, i), (m['gmin'], m['v2'], m['v4'], m['a1'], m['jrun'])
if __name__ == '__main__':
  S = scen()
  names = ['base'] + list(VARS)
  jobs = [(v, i, s) for v in names for i in range(len(S)) for s in range(1, 6)]
  with Pool(4) as p:
    res = p.map(job, jobs)
  agg = collections.defaultdict(list)
  for k, v in res: agg[k].append(v)
  print(f"{'scenario':24s} " + " | ".join(f"{n[:22]:>22s}" for n in names))
  for i, (t, _) in enumerate(S):
    cells = []
    for n in names:
      a = np.array(agg[(n, i)])
      if t.startswith('launch'):
        cells.append(f"v2 {a[:,1].mean():.2f} v4 {a[:,2].mean():.2f} j{a[:,4].max():.1f}")
      else:
        cells.append(f"gmin {a[:,0].min():5.2f}")
    print(f"{t:24s} " + " | ".join(f"{c:>22s}" for c in cells))
