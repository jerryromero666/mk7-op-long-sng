#!/usr/bin/env python3
"""
mk7_kick_replay.py - read-only (car OFF). Replays the launch kick/boost trigger over logged stops.

  python3 /data/mk7_kick_replay.py            newest 6 drives
  python3 /data/mk7_kick_replay.py 0 1 2      drives by index (0 = newest) or route-name prefix

It loads selfdrive/controls/lib/launch_boost.py from the git ref in KICK_REF (default
sngrepo/t2_launch_kick, fetch it first), feeds it the logged inputs open-loop (the same values the
planner passes it), and reports for several trigger settings:
  - real departures (ego stopped >= 1 s behind a stopped lead, then the lead left): how many fired
    and how long after the lead started to move
  - false fires: fired while the lead had not left (it did not open the gap by 1.5 m within 2 s)
  - noise of the lead measurements while both cars stood still (to calibrate the simulator)
Only reads logs. Writes nothing.
"""
import os
import re
import statistics as st
import subprocess
import sys
import types

import numpy as np

sys.path.insert(0, '/data/openpilot')
from openpilot.tools.lib.logreader import LogReader  # noqa: E402

ROOT = os.environ.get('MK7_ROOT', '/data/media/0/realdata')
OP_DIR = os.environ.get('MK7_OP_DIR', '/data/openpilot')
KICK_REF = os.environ.get('KICK_REF', 'sngrepo/t2_launch_kick')
MODULE = 'selfdrive/controls/lib/launch_boost.py'
VARIANTS = [
  ('boost only', {'KICK_MAX': 0.0}),
  ('kick (branch)', {}),
  ('kick, 3 frames', {'KICK_FRAMES': 3}),
  ('kick, 4 frames', {'KICK_FRAMES': 4}),
  ('kick, v.5 gap.2', {'V_KICK': 0.5, 'D_KICK': 0.2}),
  ('kick, v.5 gap.2 3f', {'V_KICK': 0.5, 'D_KICK': 0.2, 'KICK_FRAMES': 3}),
]


def load_module(src, overrides):
  mod = types.ModuleType('launch_boost_replay')
  exec(compile(src, f'{KICK_REF}:{MODULE}', 'exec'), mod.__dict__)
  mod.LIVE_FILE = '/nonexistent/launch_boost'
  if hasattr(mod, 'KICK_FILE'):
    mod.KICK_FILE = '/nonexistent/launch_kick'
  for k, v in overrides.items():
    if not hasattr(mod, k):
      return None
    setattr(mod, k, v)
  return mod


def routes_on_disk():
  segs = {}
  for d in os.listdir(ROOT):
    m = re.match(r'^([0-9a-f]{8}--[0-9a-f]+)--(\d+)$', d)
    if m:
      segs.setdefault(m.group(1), []).append(int(m.group(2)))
  return sorted(segs, reverse=True), segs


def pick(arg, routes):
  if re.fullmatch(r'\d{1,3}', arg) and int(arg) < len(routes):
    return routes[int(arg)]
  hits = [r for r in routes if r.startswith(arg)]
  return hits[0] if hits else None


class Lead:
  __slots__ = ('status', 'dRel', 'vLead', 'aLeadK')

  def __init__(self, l=None):
    self.status = bool(l.status) if l is not None else False
    self.dRel = float(l.dRel) if l is not None else 0.0
    self.vLead = float(l.vLead) if l is not None else 0.0
    self.aLeadK = float(l.aLeadK) if l is not None else 0.0


def replay_route(route, segs, mods):
  """Feeds every longitudinalPlan frame to one LaunchBoost per variant. Returns frames and fires."""
  lbs = [m.LaunchBoost() if m is not None else None for m in mods]
  fires = [[] for _ in mods]
  frames = []      # (t, vEgo, lead status, dRel, vLead, aLeadK, plan aTarget)
  v = gas = brk = 0.0
  long_active = exp = False
  l1, l2 = Lead(), Lead()
  branch = '?'
  for n in sorted(segs[route]):
    p = None
    for k in ('rlog.zst', 'rlog.bz2', 'rlog'):
      if os.path.exists(f'{ROOT}/{route}--{n}/{k}'):
        p = f'{ROOT}/{route}--{n}/{k}'
        break
    if p is None:
      continue
    try:
      msgs = list(LogReader(p))
    except Exception as e:
      print(f'  (segment {n} unreadable: {e})')
      continue
    msgs.sort(key=lambda m: m.logMonoTime)
    for m in msgs:
      try:
        w = m.which()
      except Exception:
        continue
      if w == 'carState':
        v, gas, brk = m.carState.vEgo, m.carState.gasPressed, m.carState.brakePressed
      elif w == 'carControl':
        long_active = m.carControl.longActive
      elif w == 'selfdriveState':
        exp = m.selfdriveState.experimentalMode
      elif w == 'radarState':
        l1, l2 = Lead(m.radarState.leadOne), Lead(m.radarState.leadTwo)
      elif w == 'initData':
        branch = m.initData.gitBranch
      elif w == 'longitudinalPlan':
        t = m.logMonoTime * 1e-9
        a = m.longitudinalPlan.aTarget
        frames.append((t, v, l1.status, l1.dRel, l1.vLead, l1.aLeadK, a))
        for i, lb in enumerate(lbs):
          if lb is None:
            continue
          lb.update(long_active and not exp, v, gas, brk, l1, l2, a, 0)
          if lb.event.startswith('fired'):
            fires[i].append((t, lb.event))
    del msgs
  return branch, np.array(frames, dtype=float) if frames else np.zeros((0, 7)), fires


def departures(F):
  """Stops >= 1 s behind a stopped lead that then left. Returns list of (t_stop0, t_lead, d_stop)."""
  out = []
  if len(F) == 0:
    return out
  t, v, st_, d, vl = F[:, 0], F[:, 1], F[:, 2] > 0.5, F[:, 3], F[:, 4]
  i, n = 0, len(t)
  while i < n:
    if v[i] >= 0.1:
      i += 1
      continue
    j = i
    while j < n and v[j] < 0.1:
      j += 1
    s0, s1 = t[i], t[j - 1]
    if s1 - s0 >= 1.0:
      W = np.where((t >= s0) & (t <= s1 + 6.0) & st_)[0]
      stop_w = W[t[W] <= s1]
      if len(stop_w) >= 10 and np.median(d[stop_w]) < 25.0:
        d_stop = float(np.median(d[stop_w][-20:]))
        # the lead left: gap opened >= 1.5 m within 6 s of ego moving
        grew = W[(d[W] > d_stop + 1.5)]
        if len(grew):
          still = W[(t[W] < t[grew[0]]) & (vl[W] < 0.5) & (d[W] < d_stop + 0.5)]
          t_lead = t[still[-1]] if len(still) else np.nan
          # refine with a line fit to vLead while it pulls away (time window, not value window)
          fast = W[(t[W] > t_lead) & (vl[W] > 1.0)] if np.isfinite(t_lead) else []
          if len(fast):
            ta = t[fast[0]]
            Wf = W[(t[W] >= ta - 0.3) & (t[W] <= ta + 1.0)]
            if len(Wf) >= 8:
              sl, ic = np.polyfit(t[Wf] - ta, vl[Wf], 1)
              if sl > 0.3:
                t_lead = float(np.clip(ta - ic / sl, t_lead - 1.0, t_lead + 0.5))
          out.append((s0, t_lead, d_stop))
    i = j
  return out


def noise_stats(F):
  """Std of dRel / vLead / aLeadK in 1 s windows while ego and lead both stood still."""
  if len(F) == 0:
    return None
  t, v, st_, d, vl, al = F[:, 0], F[:, 1], F[:, 2] > 0.5, F[:, 3], F[:, 4], F[:, 5]
  sd, sv, sa = [], [], []
  k = 0
  while k + 20 <= len(t):
    w = slice(k, k + 20)
    if (v[w] < 0.05).all() and st_[w].all() and (d[w] < 25).all() and np.median(vl[w]) < 0.3 and t[k + 19] - t[k] < 1.2:
      sd.append(np.std(d[w])); sv.append(np.std(vl[w])); sa.append(np.std(al[w]))
    k += 20
  if not sd:
    return None
  return len(sd), float(np.median(sd)), float(np.median(sv)), float(np.median(sa)), float(np.percentile(sv, 90))


def main():
  routes, segs = routes_on_disk()
  args = sys.argv[1:] or [str(i) for i in range(min(6, len(routes)))]
  chosen = [r for r in (pick(a, routes) for a in args) if r]
  try:
    src = subprocess.check_output(['git', '-C', OP_DIR, 'show', f'{KICK_REF}:{MODULE}'], text=True)
  except subprocess.CalledProcessError:
    print(f'cannot read {KICK_REF}:{MODULE} - run: cd /data/openpilot && git fetch sngrepo t2_launch_kick')
    return
  mods = [load_module(src, ov) for _, ov in VARIANTS]
  tot = {name: dict(dep=0, fired=0, lag=[], false=0) for name, _ in VARIANTS}
  stand_s = 0.0
  noise = []
  print(f'mk7_kick_replay  module {KICK_REF}  drives: {", ".join(chosen)}')
  for r in chosen:
    branch, F, fires = replay_route(r, segs, mods)
    deps = departures(F)
    ns = noise_stats(F)
    if ns:
      noise.append(ns)
    if len(F):
      lead_still = (F[:, 1] < 0.1) & (F[:, 2] > 0.5) & (F[:, 3] < 25) & (F[:, 4] < 0.5)
      stand_s += float(lead_still.sum()) * 0.05
    print(f'  {r} {branch}: {len(F) * 0.05 / 60:.1f} min, {len(deps)} departures behind a lead')
    for (name, _), fl in zip(VARIANTS, fires):
      T = tot[name]
      used = set()
      for s0, t_lead, d_stop in deps:
        T['dep'] += 1
        hit = [(t, e) for t, e in fl if s0 <= t <= (t_lead + 4.0 if np.isfinite(t_lead) else s0 + 60)]
        if hit:
          T['fired'] += 1
          used.add(hit[0][0])
          if np.isfinite(t_lead):
            T['lag'].append(hit[0][0] - t_lead)
      for t, e in fl:
        if t in used:
          continue
        # a fire that is not on a counted departure: false if the lead did not open the gap 1.5 m in 2 s
        w = (F[:, 0] >= t) & (F[:, 0] <= t + 2.0) & (F[:, 2] > 0.5)
        k0 = np.searchsorted(F[:, 0], t)
        d0 = F[min(k0, len(F) - 1), 3]
        if not (w.any() and F[w, 3].max() > d0 + 1.5):
          T['false'] += 1
          print(f'     {name}: FALSE fire at {(t - F[0, 0]) / 60:.0f}:{(t - F[0, 0]) % 60:04.1f} ({e})')
  print(f'RESULT over {stand_s / 60:.1f} min standing behind a stopped lead:')
  print(f'  {"setting":20s} departures fired  median/max s after lead moved   false fires')
  for name, _ in VARIANTS:
    T = tot[name]
    lag = f'{st.median(T["lag"]):5.2f} / {max(T["lag"]):5.2f}' if T['lag'] else '    - /     -'
    print(f'  {name:20s} {T["dep"]:5d} {T["fired"]:6d}      {lag}                  {T["false"]:3d}')
  if noise:
    n = sum(x[0] for x in noise)
    print(f'LEAD NOISE while both stood still ({n} one-second windows): median std dRel {st.median(x[1] for x in noise):.2f} m,'
          f' vLead {st.median(x[2] for x in noise):.2f} m/s (90th pct {st.median(x[4] for x in noise):.2f}),'
          f' aLeadK {st.median(x[3] for x in noise):.2f} m/s2')


if __name__ == '__main__':
  main()
