# Launch comparison: baseline (t2_lead2_phantom_fix) vs boost, mean over seeds.
import sys, numpy as np, boostsim as B, collections
from multiprocessing import Pool
M = B.M
T0 = 8.0
BOOST_REF = sys.argv[1] if len(sys.argv) > 1 else 'WT'
MODE = sys.argv[2] if len(sys.argv) > 2 else 'vision'
OV = eval(sys.argv[3]) if len(sys.argv) > 3 else {}
NK = dict(sd=0.3, sv=0.4, sa=0.3)
SEEDS = range(1, 7) if MODE == 'vision' else [0]
KEYS = ['delay', 'a1', 'v2', 'v4', 't5', 'gg4', 'gmin', 'apk', 'jcmd', 'jact', 'jrun', 'fire', 'bmax']
def job(args):
  name, gap0, al, seed = args
  if name == 'base':
    m, cfg = M.load_ref('t2_lead2_phantom_fix'); bm = None
  else:
    m, cfg = M.load_ref('t2_lead2_phantom_fix' if BOOST_REF == 'WT' else BOOST_REF); bm = B.load_boost(BOOST_REF, OV)
  r = B.run(m, cfg, bm, T0 + 10, B.launch(T0, al), gap0=gap0, mode=MODE, seed=seed, noise_kw=NK)
  mt = B.launch_metrics(r, T0)
  # MPC braking after the boost ended (feel): min a_target in the 8 s after lead start
  post = (r.t >= T0) & (r.t <= T0 + 8)
  mt['amin'] = float(np.min(r.a_target[post]))
  return (name, gap0, al), mt
if __name__ == '__main__':
  if BOOST_REF != 'WT':
    B.check_wiring(BOOST_REF)
  jobs = [(n, g, a, s) for n in ('base', 'boost') for g in (3.5, 4.5, 5.5) for a in (1.0, 1.6, 2.3, 3.0) for s in SEEDS]
  with Pool(4) as p:
    res = p.map(job, jobs)
  agg = collections.defaultdict(list)
  for k, v in res:
    agg[k].append(v)
  hdr = f"{'gap0':>4s} {'aL':>3s} {'ref':5s} | {'delay':>5s} {'a1s':>5s} {'v@2':>5s} {'v@4':>5s} {'t5':>5s} {'gap+4s':>6s} {'gmin':>5s} {'apk':>5s} {'jcmd':>5s} {'jrun':>5s} {'jact':>5s} {'fire':>5s} {'bmax':>4s} {'amin':>5s}"
  print(f"mode={MODE} boost={BOOST_REF} {OV} seeds={list(SEEDS)}")
  print(hdr)
  tot = collections.defaultdict(list)
  for g in (3.5, 4.5, 5.5):
    for a in (1.0, 1.6, 2.3, 3.0):
      for n in ('base', 'boost'):
        L = agg[(n, g, a)]
        mean = {k: np.nanmean([x[k] for x in L]) for k in KEYS + ['amin']}
        mn = {k: np.nanmin([x[k] for x in L]) for k in ('gmin', 'amin')}
        mx = {k: np.nanmax([x[k] for x in L]) for k in ('apk', 'jcmd', 'jrun', 'jact')}
        tot[n].append(mean)
        print(f"{g:4.1f} {a:3.1f} {n:5s} | {mean['delay']:5.2f} {mean['a1']:5.2f} {mean['v2']:5.2f} {mean['v4']:5.2f} {mean['t5']:5.2f} {mean['gg4']:6.2f} {mn['gmin']:5.2f} {mx['apk']:5.2f} {mx['jcmd']:5.1f} {mx['jrun']:5.2f} {mx['jact']:5.2f} {mean['fire']:5.2f} {mean['bmax']:4.2f} {mn['amin']:5.2f}")
  ends = collections.Counter(x['end'].split(', ')[-1] if x['end'] else ('no fire' if np.isnan(x['fire']) else 'still on') for (n, g, a), L in agg.items() if n == 'boost' for x in L)
  print('-- how the boost ended (all boost launches):', dict(ends))
  print('-- mean over all cells')
  for n in ('base', 'boost'):
    T = tot[n]
    print(f"{'all':>8s} {n:5s} | " + " ".join(f"{k}={np.nanmean([x[k] for x in T]):.2f}" for k in ('delay', 'a1', 'v2', 'v4', 't5', 'gg4')))
