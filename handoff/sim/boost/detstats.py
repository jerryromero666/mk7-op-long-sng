# Detector statistics at standstill behind a stationary lead (no trigger thresholds involved):
# max filtered vLead and max (d_f - d_stop) while armed, per run.
import sys, numpy as np, boostsim as B
from multiprocessing import Pool
M = B.M
OV = eval(sys.argv[1]) if len(sys.argv) > 1 else {}
def job(args):
  gap0, seed, nk = args
  mb, cfgb = M.load_ref('t2_lead2_phantom_fix')
  bm = B.load_boost('WT', dict(OV, V_TRIGGER=99.0))
  st = dict(v=[], d=[], vd=[])
  def probe(t, pl, plant, rs):
    lb = pl.lb
    if lb.armed:
      st['v'].append(lb.v_f); st['d'].append(lb.d_f - lb.d_stop)
  B.run(mb, cfgb, bm, 60.0, B.stationary(), gap0=gap0, mode='vision', seed=seed, noise_kw=nk, probe=probe)
  v = np.array(st['v']); d = np.array(st['d'])
  return gap0, seed, str(nk), len(v), v.max() if len(v) else np.nan, d.max() if len(d) else np.nan, np.percentile(v, 99.9) if len(v) else np.nan
if __name__ == '__main__':
  jobs = [(g, s, nk) for nk in (dict(sv=0.3), dict(sv=0.5), dict(sv=0.5, white_frac=0.2)) for g in (3.5, 4.5, 5.5) for s in range(1, 11)]
  with Pool(4) as p:
    res = p.map(job, jobs)
  import collections
  agg = collections.defaultdict(list)
  for g, s, nk, n, vm, dm, v999 in res:
    agg[nk].append((n, vm, dm, v999))
  for nk, l in agg.items():
    a = np.array(l)
    print(f"{nk:35s} armed frames/run {a[:,0].mean():.0f}  max v_f {np.nanmax(a[:,1]):.2f} (median of run-max {np.nanmedian(a[:,1]):.2f})  max d_f-d_stop {np.nanmax(a[:,2]):.2f} (median {np.nanmedian(a[:,2]):.2f})")
