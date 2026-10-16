#!/usr/bin/env python3
"""
mk7_boost_check.py - read-only launch-boost drive report (car OFF).

  python3 /data/mk7_boost_check.py              newest drive vs the newest t2_accel_cap_launch drive
  python3 /data/mk7_boost_check.py 0 1          drive 0 (newest) vs drive 1
  python3 /data/mk7_boost_check.py 0 00000339   a drive can also be a route-name prefix
  python3 /data/mk7_boost_check.py 0 none       no comparison drive

For each drive it prints:
  - branch, Experimental mode share while engaged, personality, faults
  - every launch from a stop behind a lead: when the lead moved, when the plan asked to go,
    when the car moved, when the boost fired / ended and why, accel and speed numbers,
    and the plan / actual accel every 0.5 s after the lead moved
  - brake build-up check: how fast the car's decel followed sharp drops in the accel command
Only reads logs. Writes nothing.
"""
import json
import os
import re
import sys
import statistics as st

import numpy as np

sys.path.insert(0, '/data/openpilot')
from openpilot.tools.lib.logreader import LogReader  # noqa: E402

ROOT = '/data/media/0/realdata'
BASE_BRANCH = 't2_accel_cap_launch'
LIST_N = 8
IGNORE_EVENTS = {'preEnableStandstill', 'pcmEnable', 'buttonEnable', 'silentButtonEnable',
                 'gasPressedOverride', 'resumeRequired', 'steerOverride', 'preLaneChangeLeft',
                 'preLaneChangeRight', 'laneChange', 'laneChangeBlocked', 'steerTempUnavailableSilent'}


def routes_on_disk():
  segs = {}
  for d in os.listdir(ROOT):
    m = re.match(r'^([0-9a-f]{8}--[0-9a-f]+)--(\d+)$', d)
    if m:
      segs.setdefault(m.group(1), []).append(int(m.group(2)))
  return sorted(segs, reverse=True), segs


def seg_log(route, n, kinds=('rlog.zst', 'rlog.bz2', 'rlog')):
  for k in kinds:
    p = f'{ROOT}/{route}--{n}/{k}'
    if os.path.exists(p):
      return p
  return None


def route_branch(route, segs):
  for n in sorted(segs[route])[:3]:
    p = seg_log(route, n, ('qlog.zst', 'qlog.bz2', 'qlog', 'rlog.zst', 'rlog.bz2', 'rlog'))
    if p is None:
      continue
    try:
      for m in LogReader(p):
        if m.which() == 'initData':
          return m.initData.gitBranch, m.initData.gitCommit[:7]
    except Exception:
      continue
  return '?', '?'


def pick_route(arg, routes):
  if re.fullmatch(r'\d{1,3}', arg) and int(arg) < len(routes):
    return routes[int(arg)]
  hits = [r for r in routes if r.startswith(arg)]
  return hits[0] if hits else None


def read_route(route, segs):
  D = {k: [] for k in ('cs', 'rs', 'lp', 'cc', 'ss', 'boost', 'faults')}
  info = {'branch': '?', 'commit': '?', 'segs': len(segs[route]), 'missing': 0}
  prev_events = set()
  prev_accf = False
  v_now = 0.0
  for n in sorted(segs[route]):
    p = seg_log(route, n)
    if p is None:
      info['missing'] += 1
      continue
    try:
      lr = list(LogReader(p))
    except Exception as e:
      print(f'  (segment {n} unreadable: {e})')
      info['missing'] += 1
      continue
    for m in lr:
      try:
        w = m.which()
      except Exception:
        continue
      t = m.logMonoTime * 1e-9
      if w == 'carState':
        cs = m.carState
        v_now = cs.vEgo
        D['cs'].append((t, cs.vEgo, cs.aEgo, cs.gasPressed, cs.brakePressed))
        if cs.accFaulted and not prev_accf:
          D['faults'].append((t, 'accFaulted', v_now))
        prev_accf = cs.accFaulted
      elif w == 'radarState':
        l1, l2 = m.radarState.leadOne, m.radarState.leadTwo
        D['rs'].append((t, l1.status, l1.dRel, l1.vLead, l1.aLeadK, l2.status, l2.dRel))
      elif w == 'longitudinalPlan':
        lp = m.longitudinalPlan
        D['lp'].append((t, lp.aTarget, lp.shouldStop))
      elif w == 'carControl':
        cc = m.carControl
        D['cc'].append((t, cc.longActive, cc.actuators.accel, str(cc.actuators.longControlState)))
      elif w == 'selfdriveState':
        s = m.selfdriveState
        D['ss'].append((t, s.enabled, s.experimentalMode, str(s.personality)))
      elif w == 'logMessage':
        txt = m.logMessage
        if 'launch boost' in txt:
          try:
            txt = json.loads(txt).get('msg', txt)
          except (ValueError, AttributeError):
            pass
          D['boost'].append((t, str(txt).replace('launch boost: ', '')))
      elif w == 'onroadEvents':
        names = {str(e.name) for e in m.onroadEvents}
        for nm in names - prev_events:
          if nm not in IGNORE_EVENTS:
            D['faults'].append((t, nm, v_now))
        prev_events = names
      elif w == 'initData':
        info['branch'] = m.initData.gitBranch
        info['commit'] = m.initData.gitCommit[:7]
    del lr
  return D, info


def at(ts, vals, t):
  """Value of a sampled series at time t (last sample at or before t), nan if none."""
  i = np.searchsorted(ts, t, side='right') - 1
  return vals[i] if 0 <= i < len(vals) else np.nan


def fmt(x, nd=2, w=5):
  return f'{x:{w}.{nd}f}' if x is not None and np.isfinite(x) else ' ' * (w - 1) + '-'


def analyze(route, segs, brief=False):
  D, info = read_route(route, segs)
  if not D['cs']:
    print(f'route {route}: no carState found')
    return None
  cs = np.array([r[:3] for r in D['cs']], dtype=float)
  gas = np.array([r[3] for r in D['cs']], dtype=bool)
  brk = np.array([r[4] for r in D['cs']], dtype=bool)
  o = np.argsort(cs[:, 0], kind='stable')
  cs, gas, brk = cs[o], gas[o], brk[o]
  ct, cv, ca = cs[:, 0], cs[:, 1], cs[:, 2]
  rs = np.array(D['rs'], dtype=float) if D['rs'] else np.zeros((0, 7))
  lp = np.array([r[:2] for r in D['lp']], dtype=float) if D['lp'] else np.zeros((0, 2))
  cc_t = np.array([r[0] for r in D['cc']]); cc_act = np.array([r[1] for r in D['cc']], dtype=bool)
  cc_a = np.array([r[2] for r in D['cc']], dtype=float)
  ss_t = np.array([r[0] for r in D['ss']]); ss_en = np.array([r[1] for r in D['ss']], dtype=bool)
  ss_exp = np.array([r[2] for r in D['ss']], dtype=bool)
  if len(rs):
    rs = rs[np.argsort(rs[:, 0], kind='stable')]
  if len(lp):
    lp = lp[np.argsort(lp[:, 0], kind='stable')]
  o = np.argsort(cc_t, kind='stable')
  cc_t, cc_a, cc_act = cc_t[o], cc_a[o], cc_act[o]
  o = np.argsort(ss_t, kind='stable')
  ss_t, ss_en, ss_exp = ss_t[o], ss_en[o], ss_exp[o]
  t0 = ct[0]

  def clock(t):
    return f'{int((t - t0) // 60)}:{int((t - t0) % 60):02d}'

  print(f'=== route {route}  branch {info["branch"]}@{info["commit"]}  segs {info["segs"]}'
        + (f' ({info["missing"]} without rlog)' if info['missing'] else ''))
  if len(ss_t):
    en = ss_en
    exp_share = float(ss_exp[en].mean()) if en.any() else float('nan')
    pers = sorted({r[3] for r in D['ss'] if r[1]})
    print(f'engaged {en.mean() * 100:.0f}% of drive | Experimental mode while engaged: {exp_share * 100:.0f}%'
          f' | personality {",".join(pers) or "?"}')
  if not brief:
    try:
      live = open('/data/launch_boost').read().strip()
    except OSError:
      live = 'no file (default 1.3)'
    print(f'/data/launch_boost now: {live}')
  ev = {}
  for _, e in D['boost']:
    ev[e] = ev.get(e, 0) + 1
  print('boost log events: ' + (', '.join(f'{k} x{v}' for k, v in sorted(ev.items())) or 'none'))
  print('FAULT/ALERT EVENTS (min:sec, event, speed m/s):')
  for t, nm, vv in sorted(D['faults']):
    print(f'  {clock(t)} {nm} v={vv:.1f}')
  if not D['faults']:
    print('  none')

  # ---- launches: stopped >= 1 s, then v > 0.3
  launches = []
  stopped = cv < 0.1
  stand_t = s1 = None
  for q in range(len(cv)):
    if stopped[q]:
      if stand_t is None:
        stand_t = ct[q]
      s1 = ct[q]
    elif cv[q] > 0.3 and stand_t is not None:
      if ct[q] - stand_t > 1.0:
        launches.append((stand_t, s1, ct[q]))
      stand_t = None

  rows = []
  for s0, s1, tm in launches:
    if rs.shape[0] == 0:
      break
    sel = (rs[:, 0] >= s0) & (rs[:, 0] <= tm + 6.0)
    R = rs[sel]
    stop_sel = (R[:, 0] <= s1) & (R[:, 1] > 0.5)
    has_lead = stop_sel.sum() >= 5 and np.median(R[stop_sel, 2]) < 25.0
    row = {'tm': tm, 'lead': has_lead, 'stop_s': s1 - s0}
    if has_lead:
      Rs = R[stop_sel]
      slow = Rs[Rs[:, 3] < 0.5]
      d_stop = float(np.median((slow if len(slow) >= 5 else Rs)[-40:, 2]))
      # lead departure: the sample after the last one where the lead still looked stopped
      pre = R[(R[:, 0] <= tm) & (R[:, 1] > 0.5)]
      still = np.where((pre[:, 3] < 0.5) & (pre[:, 2] < d_stop + 0.5))[0]
      if len(still) and still[-1] + 1 < len(pre):
        t_lead = pre[still[-1] + 1, 0]
      elif len(still):
        t_lead = tm
      else:
        t_lead = np.nan
      # refine: fit a line to the lead's speed while it pulls away and take where it crosses 0
      # (the 'last stopped sample' alone reads late when vision vLead is noisy). The fit window is
      # chosen by time, not by speed value, so noise does not bias the slope.
      if np.isfinite(t_lead):
        F = R[(R[:, 0] >= t_lead - 0.5) & (R[:, 0] <= t_lead + 3.0) & (R[:, 1] > 0.5)]
        fast = np.convolve(F[:, 3] > 1.0, np.ones(3), mode='valid') == 3 if len(F) >= 3 else np.zeros(0, bool)
        if fast.any():
          t_a = F[np.argmax(fast), 0]
          W = F[(F[:, 0] >= t_a - 0.3) & (F[:, 0] <= t_a + 1.0)]
          if len(W) >= 8:
            slope, icpt = np.polyfit(W[:, 0] - t_a, W[:, 3], 1)
            if slope > 0.3:
              t_lead = float(np.clip(t_a - icpt / slope, t_lead - 1.0, t_lead + 0.5))
      row.update(d_stop=d_stop, t_lead=t_lead)
      if np.isfinite(t_lead):
        post = R[(R[:, 0] >= t_lead) & (R[:, 0] <= t_lead + 6.0) & (R[:, 1] > 0.5)]
        row['delay'] = tm - t_lead
        a3 = post[post[:, 0] <= t_lead + 3.0, 4]
        row['aLmax'] = float(a3.max()) if len(a3) else np.nan
        row['gap4'] = float(at(post[:, 0], post[:, 2], t_lead + 4.0) - d_stop) if len(post) else np.nan
        row['gmin'] = float(post[:, 2].min()) if len(post) else np.nan
        go = lp[(lp[:, 0] >= t_lead - 1.0) & (lp[:, 1] > 0.1)] if len(lp) else lp
        row['plan_go'] = go[0, 0] - t_lead if len(go) else np.nan
        row['vL4'] = float(at(ct, cv, t_lead + 4.0))
        row['prof_p'] = [float(at(lp[:, 0], lp[:, 1], t_lead + x)) if len(lp) else np.nan for x in np.arange(0.5, 4.01, 0.5)]
        row['prof_a'] = [float(at(ct, ca, t_lead + x)) for x in np.arange(0.5, 4.01, 0.5)]
        i_m = np.searchsorted(ct, tm)
        nx = np.where(stopped[i_m:])[0]
        t_end = min(tm + 8.0, ct[i_m + nx[0]]) if len(nx) else tm + 8.0
        bev = [(t - t_lead, e) for t, e in D['boost'] if s0 <= t < t_end]
        row['boost'] = bev
    w4 = (ct >= tm) & (ct <= tm + 4.0)
    w1 = (ct >= tm) & (ct <= tm + 1.0)
    w05 = (ct >= tm) & (ct <= tm + 0.5)
    lw = (lp[:, 0] >= tm) & (lp[:, 0] <= tm + 4.0) if len(lp) else np.zeros(0, dtype=bool)
    row['planMax'] = float(lp[lw, 1].max()) if lw.any() else np.nan
    row['actMax'] = float(ca[w4].max())
    row['act05'] = float(ca[w05].mean())
    row['act1'] = float(ca[w1].mean())
    row['v4'] = float(at(ct, cv, tm + 4.0))
    reach = np.where(w4 & (cv >= 5.0))[0]
    row['t5'] = ct[reach[0]] - tm if len(reach) else np.nan
    # 5 -> 25 mph pick-up: time from 2.2 to 11.2 m/s, only if the car got there within 20 s without slowing
    w20 = (ct >= tm) & (ct <= tm + 20.0)
    k5, k25 = np.where(w20 & (cv >= 2.2))[0], np.where(w20 & (cv >= 11.2))[0]
    row['t525'] = np.nan
    if len(k5) and len(k25):
      seg = cv[k5[0]:k25[0] + 1]
      if np.all(np.diff(seg) > -0.05) or np.min(seg[np.argmax(seg):]) >= seg.max() - 0.3:
        row['t525'] = ct[k25[0]] - ct[k5[0]]
    row['gas'] = bool(gas[(ct >= tm - 2.0) & (ct <= tm + 4.0)].any())
    row['brake'] = bool(brk[(ct >= tm - 2.0) & (ct <= tm + 4.0)].any())
    row['exp'] = bool(at(ss_t, ss_exp, tm)) if len(ss_t) else False
    row['long'] = bool(at(cc_t, cc_act, tm)) if len(cc_t) else False
    rows.append(row)

  real = [r for r in rows if r['v4'] > 1.0]
  lead_rows = [r for r in real if r['lead'] and np.isfinite(r.get('t_lead', np.nan))]
  print(f'LAUNCHES from a stop >= 1 s: {len(rows)}  real (>1 m/s at 4 s): {len(real)}  behind a lead: {len(lead_rows)}')
  if not brief:
    print('  times in s from the lead moving: delay = car moved, plan = plan asked > 0.1, boost = events')
  for i, r in enumerate([] if brief else real, 1):
    flags = ''.join(f' {x}' for x, on in (('GAS', r['gas']), ('BRAKE', r['brake']), ('EXP', r['exp']),
                                           ('notLongActive', not r['long'])) if on)
    head = (f"  {i:2d} {clock(r['tm'])} stop {r['stop_s']:.0f}s planMax={fmt(r['planMax'])} actMax={fmt(r['actMax'])}"
            f" act0.5={fmt(r['act05'])} act1={fmt(r['act1'])} v4={fmt(r['v4'], 1, 4)} to5={fmt(r['t5'], 1, 4)}"
            f" 5-25mph={fmt(r['t525'], 1, 4)}s")
    if not r['lead'] or not np.isfinite(r.get('t_lead', np.nan)):
      print(head + ' nolead' + flags)
      continue
    print(head + flags)
    b = ' '.join(f'{e}@{t:+.2f}' for t, e in r['boost']) or 'none'
    print(f"     lead d={r['d_stop']:.1f} aL={fmt(r['aLmax'], 1, 3)} delay={fmt(r['delay'])} plan={fmt(r['plan_go'])}"
          f" vL4={fmt(r['vL4'], 1, 4)} gap+4={fmt(r['gap4'], 1, 4)} gmin={fmt(r['gmin'], 1, 4)} boost: {b}")
    print('     plan ' + ' '.join(fmt(x, 2, 5) for x in r['prof_p']) + '  | aEgo ' + ' '.join(fmt(x, 2, 5) for x in r['prof_a'])
          + '  (lead +0.5..+4.0 s)')
  clean = [r for r in lead_rows if r['long'] and not (r['gas'] or r['brake'])]
  if clean:
    def med(k):
      vals = [r[k] for r in clean if np.isfinite(r.get(k, np.nan))]
      return f'{st.median(vals):.2f}' if vals else '-'
    fired = sum(any(e.startswith('fired') for _, e in r['boost']) for r in clean)
    n_exp = sum(r['exp'] for r in clean)
    print(f'MEDIAN of {len(clean)} openpilot launches behind a lead ({len(lead_rows) - len(clean)} with gas/brake or long off left out'
          f'{f", {n_exp} in Experimental mode" if n_exp else ""}):')
    print(f'  delay {med("delay")} plan {med("plan_go")} planMax {med("planMax")} actMax {med("actMax")}'
          f' act0.5 {med("act05")} act1 {med("act1")} v4 {med("v4")} vL4 {med("vL4")} gap+4 {med("gap4")} 5-25mph {med("t525")}s'
          f' | boost fired on {fired}/{len(clean)}')

  # ---- brake build-up: sharp drops in the accel command while moving and long active
  print('BRAKE BUILD-UP (cmd drops >= 1.0 in 0.5 s and reaches <= -0.5, moving >= 2 m/s, long active):')
  if len(cc_t) > 100:
    ka = np.convolve(ca, np.ones(5) / 5, mode='same')   # light smoothing of aEgo
    later = np.interp(cc_t + 0.5, cc_t, cc_a)
    cand = np.where(cc_act & (cc_a - later >= 1.0))[0]
    evs, last = [], -1e9
    for idx in cand:
      t = cc_t[idx]
      if t - last < 3.0 or at(ct, cv, t) < 2.0:
        continue
      wc = (cc_t >= t) & (cc_t <= t + 1.5)
      cmin = float(cc_a[wc].min())
      if cmin > -0.5:
        continue   # throttle cut (e.g. a boost abort), not brake build-up
      last = t
      cslope = float(np.max((cc_a[wc][:-25] - cc_a[wc][25:]) / 0.25)) if wc.sum() > 30 else np.nan
      we = (ct >= t) & (ct <= t + 2.0)
      a0 = float(at(ct, ka, t))
      ae = ka[we]
      eslope = float(np.max((ae[:-25] - ae[25:]) / 0.25)) if len(ae) > 30 else np.nan
      hit = np.where(ae <= a0 - 1.0)[0]
      hit_c = np.where(cc_a[wc] <= cc_a[idx] - 1.0)[0]
      t_cmd = cc_t[wc][hit_c[0]] if len(hit_c) else np.nan
      t_follow = (ct[we][hit[0]] - t_cmd) if len(hit) and np.isfinite(t_cmd) else np.nan
      evs.append((cc_a[idx] - cmin, t, at(ct, cv, t), cc_a[idx], cmin, cslope, a0, float(ae.min()), eslope, t_follow))
    evs.sort(reverse=True)
    for _, t, vv, c0, cmin, cs_, a0, amin, es, tf in evs[:4 if brief else 8]:
      print(f'  {clock(t)} v={vv:4.1f} cmd {c0:+.2f}->{cmin:+.2f} (max {fmt(cs_, 1, 4)} m/s3)'
            f'  aEgo {a0:+.2f}->{amin:+.2f} (max {fmt(es, 1, 4)} m/s3, lags cmd by {fmt(tf, 2, 4)} s at -1.0)')
    if not evs:
      print('  none')
    else:
      es_all = [e[8] for e in evs if np.isfinite(e[8])]
      print(f'  events {len(evs)}  steepest aEgo build-up {max(es_all):.1f} m/s3  median {st.median(es_all):.1f} m/s3'
            if es_all else f'  events {len(evs)}')
  else:
    print('  no carControl data')
  return info


def main():
  args = sys.argv[1:]
  routes, segs = routes_on_disk()
  if not routes:
    print('no drives found in', ROOT)
    return
  print(f'mk7_boost_check  newest {LIST_N} drives (0 = newest):')
  branches = {}
  for i, r in enumerate(routes[:LIST_N]):
    branches[r] = route_branch(r, segs)
    print(f'  {i} {r} segs {len(segs[r]):3d} {branches[r][0]}@{branches[r][1]}')

  target = pick_route(args[0], routes) if args else routes[0]
  if target is None:
    print(f'no drive matches {args[0]!r}')
    return
  if len(args) > 1:
    base = None if args[1].lower() == 'none' else pick_route(args[1], routes)
    if base is None and args[1].lower() != 'none':
      print(f'no drive matches {args[1]!r}; skipping the comparison')
  else:
    base = None
    for r in routes[routes.index(target) + 1:][:40]:
      b = branches.get(r) or route_branch(r, segs)
      if b[0] == BASE_BRANCH:
        base = r
        break
    if base is None:
      print(f'no {BASE_BRANCH} drive in the 40 before it; no comparison')

  analyze(target, segs)
  if base is not None:
    print()
    print(f'--- comparison drive ---')
    analyze(base, segs, brief=True)


if __name__ == '__main__':
  main()
