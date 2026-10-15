import sys, numpy as np, boostsim as B, collections
from multiprocessing import Pool
M = B.M
T0 = 8.0
NK = dict(sd=0.3, sv=0.4, sa=0.3)
VARS = [None, {}, dict(BOOST_MAX=1.3), dict(BOOST_MAX=1.1), dict(J_UP=1.5), dict(BOOST_MAX=1.3, J_UP=2.0)]
SC = [('launch1.0', dict(lead_a=B.launch(T0, 1.0))), ('launch1.6', dict(lead_a=B.launch(T0, 1.6))), ('launch2.3', dict(lead_a=B.launch(T0, 2.3))),
      ('cut4 v1 t2.5', dict(lead_a=B.launch(T0, 1.6), lead2=dict(t_on=T0 + 2.5, d=4.0, v=1.0, a=0.0, role='cutin'))),
      ('cut6 v0 @1', dict(lead_a=B.launch(T0, 1.6), lead2=dict(t_on=T0, v_on=1.0, d=6.0, v=0.0, a=0.0, role='cutin'))),
      ('cut4 v1 @1', dict(lead_a=B.launch(T0, 1.6), lead2=dict(t_on=T0, v_on=1.0, d=4.0, v=1.0, a=0.0, role='cutin'))),
      ('brk4 after2', dict(lead_a=B.launch_then_brake(T0, 1.6, 2.0, 4.0)))]
def job(a):
  vi, si, seed, ecu = a
  mb, cfgb = M.load_ref('t2_lead2_phantom_fix')
  bm = None if VARS[vi] is None else B.load_boost('WT', VARS[vi])
  kw = dict(SC[si][1]); la = kw.pop('lead_a')
  if 'lead2' in kw: kw['lead2'] = dict(kw['lead2'])
  r = B.run(mb, cfgb, bm, T0 + 10, la, gap0=4.5, mode='vision', seed=seed, noise_kw=NK, ecu_grad=ecu, **kw)
  m = B.launch_metrics(r, T0)
  return (vi, si, ecu), (m['v2'], m['v4'], m['t5'], m['gmin'])
if __name__ == '__main__':
  jobs = [(vi, si, s, e) for vi in range(len(VARS)) for si in range(len(SC)) for s in range(1, 6) for e in (None, (4.0, 4.0))]
  with Pool(4) as p: res = p.map(job, jobs)
  agg = collections.defaultdict(list)
  for k, v in res: agg[k].append(v)
  for e in (None, (4.0, 4.0)):
    print('ecu', e)
    print(f"{'variant':32s} " + " ".join(f"{n:>13s}" for n, _ in SC))
    for vi, ov in enumerate(VARS):
      cells = []
      for si, (n, _) in enumerate(SC):
        a = np.array(agg[(vi, si, e)])
        cells.append(f"v4 {a[:,1].mean():.2f} t5 {a[:,2].mean():.1f}" if n.startswith('launch') else f"gmin {a[:,3].min():.2f}")
      print(f"{'base' if ov is None else str(ov):32s} " + " ".join(f"{c:>13s}" for c in cells))
