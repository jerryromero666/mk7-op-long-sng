import os, re, sys, statistics as st
sys.path.insert(0, '/data/openpilot')
from openpilot.tools.lib.logreader import LogReader
root = '/data/media/0/realdata'
segs = {}
for d in os.listdir(root):
  m = re.match(r'^([0-9a-f]{8}--[0-9a-f]+)--(\d+)$', d)
  if m: segs.setdefault(m.group(1), []).append(int(m.group(2)))
routes = sorted(segs, reverse=True)
idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0
route = routes[idx]
branch = '?'; commit = '?'
v = a = 0.0; plan_a = 0.0; lead = None; stand_t = None; cur = None
t0 = None; launches = []; faults = []; prev_events = set(); prev_accf = False
for n in sorted(segs[route]):
  p = f'{root}/{route}--{n}/rlog.zst'
  if not os.path.exists(p): continue
  for m in LogReader(p):
    w = m.which(); t = m.logMonoTime * 1e-9
    if t0 is None: t0 = t
    if w == 'initData':
      branch = m.initData.gitBranch; commit = m.initData.gitCommit[:7]
    elif w == 'longitudinalPlan':
      plan_a = m.longitudinalPlan.aTarget
    elif w == 'radarState':
      l1 = m.radarState.leadOne
      lead = (l1.dRel, l1.vLead, l1.aLeadK) if l1.status else None
    elif w == 'onroadEvents':
      names = {str(e.name) for e in m.onroadEvents}
      for nm in names - prev_events:
        if nm not in ('preEnableStandstill', 'pcmEnable', 'buttonEnable', 'silentButtonEnable', 'gasPressedOverride', 'resumeRequired'):
          faults.append((t, nm, v))
      prev_events = names
    elif w == 'carState':
      cs = m.carState; v = cs.vEgo; a = cs.aEgo
      if cs.accFaulted and not prev_accf: faults.append((t, 'accFaulted', v))
      prev_accf = cs.accFaulted
      if v < 0.1:
        if stand_t is None: stand_t = t
      elif v > 0.3 and stand_t is not None:
        if t - stand_t > 1.0:
          cur = {'t0': t, 'lead': lead, 'pa': [], 'ae': [], 'la': [], 't5': None}
          launches.append(cur)
        stand_t = None
      if cur is not None:
        dt = t - cur['t0']
        if dt <= 4.0:
          cur['pa'].append(plan_a); cur['ae'].append(a)
          if lead: cur['la'].append(lead[2])
          if cur['t5'] is None and v >= 5.0: cur['t5'] = dt
        else:
          cur['v4'] = v; cur = None
print(f'route {route}  branch {branch}@{commit}  segs {len(segs[route])}')
print('FAULT/ALERT EVENTS (min:sec into drive, event, speed m/s):')
base = t0 or 0
for t, nm, vv in sorted(faults):
  print(f'  {int((t-base)//60)}:{int((t-base)%60):02d} {nm} v={vv:.1f}')
if not faults: print('  none')
real = [L for L in launches if L.get('v4', 0) > 1.0 and L['pa']]
print(f'LAUNCHES from a stop: {len(launches)}  real (>1 m/s at 4 s): {len(real)}')
for i, L in enumerate(real, 1):
  ld = 'nolead' if not L['lead'] else f"lead d={L['lead'][0]:.0f} aL={max(L['la'] or [0]):.1f}"
  t5 = f"{L['t5']:.1f}s" if L['t5'] else '>4s'
  print(f"  {i:2d} planMax={max(L['pa']):.2f} actMax={max(L['ae']):.2f} act1s={st.mean(L['ae'][:50]):.2f} v4={L['v4']:.1f} to5m/s={t5} {ld}")
if real:
  print('MEDIAN planMax', round(st.median(max(L['pa']) for L in real), 2),
        'actMax', round(st.median(max(L['ae']) for L in real), 2),
        'v4', round(st.median(L['v4'] for L in real), 2),
        'capHit(plan>=1.65)', sum(max(L['pa']) >= 1.65 for L in real))
