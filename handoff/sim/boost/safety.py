# Safety scenarios: baseline (t2_lead2_phantom_fix) vs boost. Worst case over noise seeds.
# usage: safety.py <boost ref|WT> [ecu]   ('ecu' adds the ACC_06 accel-gradient limit +4.0/-1.0 m/s3)
import sys, numpy as np, boostsim as B, collections
from multiprocessing import Pool
M = B.M
T0 = 8.0
BOOST_REF = sys.argv[1] if len(sys.argv) > 1 else 'WT'
ECU = {'ecu': (4.0, 1.0), 'ecu4': (4.0, 4.0)}.get(sys.argv[2]) if len(sys.argv) > 2 else None
NK = dict(sd=0.3, sv=0.4, sa=0.3)
SEEDS = range(1, 6)
INF = 1e9


def scenarios():
  S = []
  for al in (1.6, 2.3):
    for tb in (1.0, 2.0, 3.0):
      for dec in (2.5, 4.0):
        S.append((f"lead departs aL{al}, brakes {dec} to stop after {tb:.0f}s", dict(lead_a=B.launch_then_brake(T0, al, tb, dec), T=T0 + 12)))
  for vc in (0.3, 0.4, 0.5):
    for tc in (2.0, 5.0):
      S.append((f"lead creeps {vc} m/s for {tc:.0f}s then stops", dict(lead_a=B.creep(T0, vc, tc), T=T0 + 14)))
  for d in (4.0, 5.0, 6.0):
    for tc in (1.5, 2.5):
      for vcut in (0.0, 1.0):
        S.append((f"cut-in at {d:.0f} m, v={vcut:.0f}, t+{tc}s", dict(lead_a=B.launch(T0, 1.6), T=T0 + 10,
                  lead2=dict(t_on=T0 + tc, d=d, v=vcut, a=0.0, role='cutin'))))
  for d in (4.0, 6.0):
    for von in (1.0, 2.0):
      for vcut in (0.0, 1.0):
        S.append((f"cut-in at {d:.0f} m, v={vcut:.0f}, when ego at {von:.0f} m/s", dict(lead_a=B.launch(T0, 1.6), T=T0 + 12,
                  lead2=dict(t_on=T0, v_on=von, d=d, v=vcut, a=0.0, role='cutin'))))
  S.append(("leadOne lost 1 s mid-boost (t+1.5..2.5)", dict(lead_a=B.launch(T0, 1.6), T=T0 + 10, lead1_lost=(T0 + 1.5, T0 + 2.5))))
  S.append(("leadOne lost for good mid-boost (t+1.5)", dict(lead_a=B.launch(T0, 1.6), T=T0 + 10, lead1_lost=(T0 + 1.5, INF))))
  S.append(("leadTwo 2.5 m closer appears mid-boost", dict(lead_a=B.launch(T0, 1.6), T=T0 + 10, lead2_mode='none',
            lead2=dict(t_on=T0 + 1.4, d_rel_lead1=-2.5, v=None, a=0.0, role='lead2'))))
  S.append(("stopped leadTwo 2 m closer whole time", dict(lead_a=B.launch(T0, 1.6), T=T0 + 10, lead2_mode='none',
            lead2=dict(t_on=0.0, d=2.5, v=0.0, a=0.0, role='lead2'))))
  S.append(("driver brake mid-boost (t+1.5)", dict(lead_a=B.launch(T0, 1.6), T=T0 + 8, driver_brake_t=T0 + 1.5)))
  S.append(("driver gas mid-boost (t+1.5)", dict(lead_a=B.launch(T0, 1.6), T=T0 + 8, gas_t=T0 + 1.5)))
  return S


def job(args):
  name, idx, gap0, seed, mode = args
  title, kw = scenarios()[idx]
  kw = dict(kw)
  if name == 'base':
    m, cfg = M.load_ref('t2_lead2_phantom_fix'); bm = None
  else:
    m, cfg = M.load_ref('t2_lead2_phantom_fix' if BOOST_REF == 'WT' else BOOST_REF); bm = B.load_boost(BOOST_REF)
  T = kw.pop('T'); lead_a = kw.pop('lead_a')
  if 'lead2' in kw:
    kw['lead2'] = dict(kw['lead2'])
  r = B.run(m, cfg, bm, T, lead_a, gap0=gap0, mode=mode, seed=seed, noise_kw=NK, ecu_grad=ECU, **kw)
  post = r.t >= T0
  fired = [e for e in r.events if e[1].startswith('fired')]
  ends = [e for e in r.events if e[1] != 'fired']
  # boost after an abort trigger time (lead lost / brake / gas): must be 0 from that frame on
  t_ev = kw.get('driver_brake_t') or kw.get('gas_t') or (kw['lead1_lost'][0] if 'lead1_lost' in kw else None)
  b_after = float(np.max(r.boost[r.t >= t_ev])) if t_ev is not None else 0.0
  acc_after = float(np.max(r.a_cmd[r.t >= t_ev + 0.05])) if (t_ev is not None and 'driver_brake_t' in kw) else np.nan
  return (name, idx, gap0), dict(gmin=float(np.min(r.gap_true[post])), vmax=float(np.max(r.v)), amax=float(np.max(r.a[post])),
                                 over_clip=float(np.max(r.a_target - r.clip_hi)), acmd=float(np.max(r.a_cmd)),
                                 fired=len(fired), ends=tuple(sorted(set(e[1] for e in ends))), bmax=float(np.max(r.boost)),
                                 b_after=b_after, acc_after=acc_after)


if __name__ == '__main__':
  if BOOST_REF != 'WT':
    B.check_wiring(BOOST_REF)
  S = scenarios()
  jobs = [(n, i, g, s, 'vision') for n in ('base', 'boost') for i in range(len(S)) for g in (3.5, 4.5) for s in SEEDS]
  jobs += [(n, i, g, 0, 'ideal') for n in ('base', 'boost') for i in range(len(S)) for g in (3.5, 4.5)]
  with Pool(4) as p:
    res = p.map(job, jobs)
  agg = collections.defaultdict(list)
  for k, v in res:
    agg[k].append(v)
  print(f"boost={BOOST_REF} ecu_grad={ECU} noise={NK} seeds={list(SEEDS)} + ideal; worst case over seeds")
  print(f"{'scenario':48s} gap0 | base gmin vmax | boost gmin vmax  amax fired  bmax overclip acmd | boost ends")
  worst_ok = True
  for i, (title, _) in enumerate(S):
    for g in (3.5, 4.5):
      b = agg[('base', i, g)]; x = agg[('boost', i, g)]
      bg = min(v['gmin'] for v in b); xg = min(v['gmin'] for v in x)
      bv = max(v['vmax'] for v in b); xv = max(v['vmax'] for v in x)
      fired = sum(v['fired'] for v in x)
      oc = max(v['over_clip'] for v in x); ac = max(v['acmd'] for v in x)
      ends = sorted(set(e for v in x for e in v['ends']))
      extra = ''
      if any(v['b_after'] > 0 for v in x):
        extra += ' BOOST AFTER EVENT!'
      aa = [v['acc_after'] for v in x if not np.isnan(v['acc_after'])]
      if aa:
        extra += f' max accel cmd after brake {max(aa):+.2f}'
      flag = '' if (xg >= min(2.5, bg) - 1e-6 and oc <= 1e-6 and ac <= 2.0 + 1e-6) else '  <-- CHECK'
      print(f"{title:48s} {g:4.1f} | {bg:9.2f} {bv:4.2f} | {xg:10.2f} {xv:4.2f} {max(v['amax'] for v in x):5.2f} {fired:2d}/{len(x)} {max(v['bmax'] for v in x):5.2f} {oc:+8.2f} {ac:4.2f} | {', '.join(ends)}{extra}{flag}")
