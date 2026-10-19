#!/usr/bin/env python3
"""
mk7_follow_check.py - read-only: how far behind the car ahead this car drove on the highway (45 mph and faster), by who
was driving (openpilot / the driver on the gas over openpilot / the driver alone), against what the code wants (car OFF).

INSTALL OR UPDATE (on the comma, over SSH; nothing is replaced if the download fails):
  cd /data/openpilot && git fetch sngrepo t2_set_min && git show FETCH_HEAD:mk7_follow_check.py > /data/mk7_follow_check.py.new && mv /data/mk7_follow_check.py.new /data/mk7_follow_check.py

RUN IT WITH THE CAR OFF:

  python3 /data/mk7_follow_check.py 0-4             the 5 newest drives together (0 = newest)
  python3 /data/mk7_follow_check.py                 newest drive
  python3 /data/mk7_follow_check.py 0 1 3           drives 0, 1 and 3
  python3 /data/mk7_follow_check.py 00000343        a drive can also be a route-name prefix
  python3 /data/mk7_follow_check.py 0-4 --detail 7  episode 7 as a timeline (S = steady following, O = gas override, A = approach)
  python3 /data/mk7_follow_check.py 0-4 --all       every episode as a timeline

The last block of the output, between "PASTE THIS TO CLAUDE (start)" and "(end)", is meant to be copied back as it is.

WHAT IT LOOKS AT (every 0.1 s of driving at 45 mph or faster with a car ahead: radarState.leadOne, which on this car comes
from the camera model; its modelProb is the model's lead probability, so modelV2 is not read)
 A. Who was driving: OP = openpilot's long control on (carControl.longActive) and no gas; OVERRIDE = openpilot engaged but
    the gas pressed (the driver's own distance); MANUAL = openpilot not engaged (the one or two frames of 'engaged, long
    off, no gas' at a brake press count as MANUAL too). The brake pedal is noted. The share of highway time in each mode,
    with and without a car ahead.
 B. Steady following episodes: a car ahead for 5 s or more at nearly the same speed (closing speed under 1.5 m/s and the
    gap not drifting faster than 1.5 m/s over 2 s), all in one mode (a gas tap splits an episode). Per episode: duration,
    speed, median gap in metres, time gap (gap / speed) with its 10-90 % spread, set speed, and in OP mode whether the set
    speed or the car ahead held the speed (longitudinalPlan source cruise / lead; by the speeds if it is not logged). Per
    speed band (45-55, 55-65, 65-75, 75-90 mph; 90 mph = 145 km/h is the highest set speed, faster goes in the top band)
    and mode: episodes, time, median and quartiles of the time gap and the gap, time-weighted (every 0.1 s counts, so a
    60 s episode weighs twelve 5 s ones). The code's steady gap: 6 m + T x speed, T = 1.25 s for the aggressive
    personality (1.45 standard, 1.75 relaxed; the logged personality is said per drive). The 6 m is compiled into the
    solver; T is a live parameter (selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py get_T_FOLLOW). The 'close
    follow' feature of that file never acts at these speeds (its stopping-distance floor is above the wanted gap).
 C. Gas overrides at 45+ with a car ahead: the gap and time gap when the gas went down (and what openpilot was asking right
    then), how long the gas stayed down, the gap the driver settled at (median over the steady part, else the last 5 s),
    the gap when the gas came up, and what openpilot did in the 5 s after: its strongest braking request and when, how
    much the gap grew and the speed dropped (the 'fall-back'), and whether the driver pressed the gas again inside those
    5 s. Also every gas press shorter than 1 s (a tap) is counted.
 D. Approaches at 45+: closing on a slower car (2 m/s or more for 1 s or more). In OP mode: the gap, time gap and time to
    collision when openpilot first asked for braking under -0.3 m/s2, and its strongest request; if the driver pressed
    the gas during an openpilot approach (openpilot braking too early for him): where. Under the driver: the gap, time
    gap and time to collision where the gas was lifted or the brake pressed. This separates 'the steady gap is too long'
    from 'it brakes too early when closing'.
 E. The recommendation in plain words: the T that reproduces the driver's steady override gap with the compiled 6 m
    (T = (gap - 6) / speed, time-weighted median over the settled OVERRIDE points, per band and over all; with under 30 s
    of steady override the OVERRIDE and MANUAL points are pooled), the gaps that gives at 50 / 65 / 80 / 90 mph, and the
    safety margin two ways at 65 and 90 mph, for the recommended T and today's 1.25 s: the physics limit (this car at the
    panda's -3.5 m/s2 from 0.5 s after the car ahead starts braking) and what openpilot alone manages in the simulator of
    this branch (it takes 3-4 s to reach full braking, so it absorbs about half of the physics figure). The OP steady gap
    is shown as a check that the formula matches the logs.
Only reads logs. Writes nothing. Exit codes: 0 = report printed, 1 = no driving at 45 mph or faster with a car ahead,
2 = problem (car on, no logs, missing libraries, a bug).
"""
import sys
sys.dont_write_bytecode = True   # never leave .pyc files behind on the device

import argparse  # noqa: E402
import bz2       # noqa: E402
import math      # noqa: E402
import os        # noqa: E402
import re        # noqa: E402
import time      # noqa: E402

TOOL_VERSION = '1.1 (2026-10-19)'
OP_DIR = '/data/openpilot'
ROOT = '/data/media/0/realdata'
VENV_PYTHON = '/usr/local/venv/bin/python3'
REEXEC_FLAG = 'MK7_FOLLOW_CHECK_REEXEC'
ONROAD_PARAM = '/data/params/d/IsOnroad'
ZSTD_MAGIC = b'\x28\xb5\x2f\xfd'
LIST_N = 8
NAN = float('nan')
INF = float('inf')
MPH = 2.23694
PASTE_MAX = 30         # lines between the paste markers
PASTE_OVR_MAX = 6      # at most this many override episodes in the paste block (the worst fall-back, the typical one, the rest by strength)

DT = 0.1               # s: analysis grid
HWY_MPH = 45.0         # mph: 'highway' = this fast or faster ...
HWY_V = HWY_MPH / MPH  # (20.1 m/s)
BANDS = [(45, 55), (55, 65), (65, 75), (75, 90)]   # mph; the top band takes everything from 75 mph up
V_CRUISE_MAX_MPH = 90.0   # 145 km/h, the highest set speed (and the top of the model's trained range)
# ---- the car ahead
LEAD_MAX_D = 150.0     # m: a car ahead farther than this is not followed
LEAD_BRIDGE_S = 0.5    # s: a dropout of the camera lead shorter than this is bridged (gap and speed interpolated)
# ---- B. steady following
STEADY_VREL = 1.5      # m/s: closing speed (|vRel|) under this ...
STEADY_SLOPE = 1.5     # m/s: ... and the gap's trend over +-1 s under this = steady
STEADY_MIN_S = 5.0     # s: at least this much steady time (in one mode) = an episode
STEADY_BRIDGE_S = 1.0  # s: shorter unsteady breaks inside an episode are bridged (same mode throughout)
SETTLED_SLOPE = 0.5    # m/s: the gap's trend under this = settled; the gap / time gap / T of an episode (and so the recommendation)
SETTLED_MIN_S = 3.0    # are taken over the settled points when there are at least 3 s of them, else over all its steady points
                       # (closing slowly at 1-1.5 m/s under the gas counted as 'steady' and biased the override gap by +2 m / +0.07 s)
SET_NEAR = 1.0 / MPH   # m/s (1 mph): OP at or above the set speed minus this = the set speed held the speed (fallback rule)
SETTLE_S = 10.0        # s: an OP episode starting within this after openpilot took over (engage / gas release), with the gap
SETTLE_MOVED = 3.0     # moved by more than 3 m or 10 % since the handover, is 'settling' (openpilot still opening or closing
SETTLE_MOVED_PC = 0.10 # the gap): shown with a '*', left out of the OP headline
DRIFT_PC = 0.20        # an episode whose gap drifted by more than 20 % (or 5 m) of its median end to end is 'drift': shown with a
DRIFT_M = 5.0          # 'd', left out of the OP headline too (a transient, not openpilot's steady choice)
# ---- C. gas overrides
OVR_MIN_S = 1.0        # s: a gas press shorter than this is a tap (counted, not listed)
DRIVER_OVR_MIN_S = 30.0  # s: with less steady OVERRIDE time than this, the recommendation pools OVERRIDE and MANUAL (the driver's gap)
OVR_AFTER_S = 5.0      # s after the release that are looked at
OVR_SETTLE_S = 5.0     # s: without a steady part, the settled gap = median over the last 5 s of the override
BRAKE_ON = -0.3        # m/s2: openpilot's request under this = braking
# ---- D. approaches
APP_VREL = -2.0        # m/s: closing at least this fast ...
APP_MIN_S = 1.0        # ... for at least this long = an approach
APP_END_VREL = -1.0    # m/s: the approach runs on until the closing speed is weaker than this (or the lead is lost)
APP_BRIDGE_S = 0.5     # s: shorter interruptions are bridged
APP_AFTER_S = 2.0      # s past the end that are looked at for openpilot's braking / the closest point
# ---- E. the code and the physics
STOP_DISTANCE = 6.0    # m: compiled into the acados solver (c_generated_code)
COMFORT_BRAKE = 2.5    # m/s2: the v^2 / 5 terms of the wanted gap (compiled too)
T_FOLLOW = {'aggressive': 1.25, 'standard': 1.45, 'relaxed': 1.75}
REACT_S = 0.5          # s: reaction (camera + planner + actuator delay) in the safety margin
A_EGO_MAX = 3.5        # m/s2: the panda's braking limit (ACCEL_MIN)
A_LEAD_CAP = 12.0      # m/s2: lead braking beyond this is reported as 'a wall' (instant stop)
REC_SPEEDS_MPH = (50, 65, 80, 90)
SAFETY_SPEEDS_MPH = (65, 90)
PERS = {'aggressive': 0, 'standard': 1, 'relaxed': 2}
PERS_NAME = {v: k for k, v in PERS.items()}
SRC = {'cruise': 0, 'lead0': 1, 'lead1': 2, 'e2e': 3}
MODES = ['OP', 'OVERRIDE', 'MANUAL']
MODE_SHORT = ['OP', 'OVR', 'MAN']
M_OP, M_OVR, M_MAN = 0, 1, 2
KIND_LETTER = {'steady': 'S', 'override': 'O', 'approach': 'A'}


class UsageError(Exception):
  pass


QUIET = False


def say(s=''):
  if not QUIET:
    print(s, flush=True)


# ============================================================================================ setup
def maybe_reexec():
  """If capnp / numpy can't be imported, retry once with openpilot's own Python."""
  if os.environ.get(REEXEC_FLAG) == '1' or not os.access(VENV_PYTHON, os.X_OK):
    return
  if os.path.abspath(sys.prefix) == os.path.dirname(os.path.dirname(os.path.abspath(VENV_PYTHON))):
    return
  env = dict(os.environ, **{REEXEC_FLAG: '1', 'PYTHONDONTWRITEBYTECODE': '1'})
  sys.stdout.flush()
  os.execve(VENV_PYTHON, [VENV_PYTHON, os.path.abspath(__file__)] + sys.argv[1:], env)


class RT:
  np = None
  log = None
  zstd = None
  LogReader = None


def setup(op_dir):
  op_dir = os.path.abspath(op_dir)
  for p in (os.path.dirname(op_dir), op_dir):
    if os.path.isdir(p) and p not in sys.path:
      sys.path.insert(0, p)
  try:
    import numpy as np
    import zstandard
    from cereal import log
  except Exception as e:
    maybe_reexec()
    raise UsageError(f"This needs openpilot's Python libraries (numpy, pycapnp, zstandard, cereal): {e}\n"
                     f"  Run it on the comma (openpilot in {OP_DIR}).") from None
  rt = RT()
  rt.np, rt.zstd, rt.log = np, zstandard, log
  try:
    from openpilot.tools.lib.logreader import LogReader   # only for files the streaming reader can't read
    rt.LogReader = LogReader
  except Exception:
    pass
  return rt


# ============================================================================================ drives
SEG_RE = re.compile(r'^([0-9a-f]{8}--[0-9a-f]+)--(\d+)$')
RLOGS = ('rlog.zst', 'rlog.bz2', 'rlog')
QLOGS = ('qlog.zst', 'qlog.bz2', 'qlog')


def routes_on_disk(root):
  if not os.path.isdir(root):
    raise UsageError(f'The log folder {root} does not exist.')
  segs = {}
  for d in os.listdir(root):
    m = SEG_RE.match(d)
    if m:
      segs.setdefault(m.group(1), []).append(int(m.group(2)))
  return sorted(segs, reverse=True), segs


def seg_file(root, route, n, kinds):
  for k in kinds:
    p = f'{root}/{route}--{n}/{k}'
    if os.path.isfile(p) and os.path.getsize(p) > 0:
      return p
  return None


def pick_routes(args, routes):
  """Drive arguments: N (0 = newest), N-M, or a route-name prefix. Returned oldest first."""
  out = []
  for a in args:
    m = re.fullmatch(r'(\d{1,3})-(\d{1,3})', a)
    if m:
      lo, hi = sorted((int(m.group(1)), int(m.group(2))))
      picked = [routes[i] for i in range(lo, hi + 1) if i < len(routes)]
    elif re.fullmatch(r'\d{1,3}', a) and int(a) < len(routes):
      picked = [routes[int(a)]]
    else:
      picked = [r for r in routes if r.startswith(a)][:1]
    if not picked:
      raise UsageError(f'No drive matches {a!r}.')
    out += [r for r in picked if r not in out]
  return sorted(out)


def rid(route):
  """Short drive name: '00000342--20011e5df3' -> '342'."""
  return route.split('--')[0].lstrip('0') or '0'


def decompress(rt, path):
  with open(path, 'rb') as f:
    raw = f.read()
  if raw[:3] == b'BZh':
    try:
      return bz2.BZ2Decompressor().decompress(raw)
    except (OSError, EOFError, ValueError):
      return b''
  if raw[:4] != ZSTD_MAGIC:
    return raw
  parts, data = [], raw
  while data:   # concatenated frames; a damaged tail keeps the readable part
    d = rt.zstd.ZstdDecompressor().decompressobj()
    try:
      parts.append(d.decompress(data))
    except rt.zstd.ZstdError:
      break
    if not getattr(d, 'eof', True):
      break
    data = d.unused_data
  return b''.join(parts)


def events_of(rt, path):
  dat = decompress(rt, path)
  n = 0
  try:
    for e in rt.log.Event.read_multiple_bytes(dat):
      n += 1
      yield e
  except Exception:
    pass
  del dat
  if n == 0 and rt.LogReader is not None:
    try:
      yield from rt.LogReader(path)
    except Exception:
      return


def route_branch(rt, root, route, segs):
  for n in sorted(segs[route])[:3]:
    p = seg_file(root, route, n, QLOGS + RLOGS)
    if p is None:
      continue
    for e in events_of(rt, p):
      try:
        if e.which() == 'initData':
          return e.initData.gitBranch, e.initData.gitCommit[:7]
      except Exception:
        break
  return '?', '?'


class Table:
  """Rows of floats; turned into a numpy block at the end of every segment, so memory stays small."""
  __slots__ = ('rows', 'chunks', 'ncol')

  def __init__(self, ncol):
    self.rows, self.chunks, self.ncol = [], [], ncol

  def flush(self, np):
    if self.rows:
      self.chunks.append(np.array(self.rows, dtype=np.float64))
      self.rows.clear()

  def array(self, np):
    self.flush(np)
    if not self.chunks:
      return np.zeros((0, self.ncol))
    a = np.concatenate(self.chunks) if len(self.chunks) > 1 else self.chunks[0]
    self.chunks = []
    if len(a) > 1 and not np.all(a[1:, 0] >= a[:-1, 0]):   # logs are in time order: usually no copy needed
      a = a[np.argsort(a[:, 0], kind='stable')]
    return a


def read_route(rt, root, route, segs):
  """Only the fields needed, all segments of one drive (rlog; qlog where a segment has no rlog). Times in s. No CAN."""
  np = rt.np
  T = {'cs': Table(6), 'cc': Table(4), 'lp': Table(3), 'rs': Table(7), 'ss': Table(4)}
  cs_add, cc_add, lp_add, rs_add, ss_add = (T[k].rows.append for k in ('cs', 'cc', 'lp', 'rs', 'ss'))
  info = {'route': route, 'branch': '?', 'commit': '?', 'delay': NAN, 'op_long': -1, 'segs': 0, 'q_segs': 0, 'none': 0,
          'bad': 0, 'nseg': len(segs[route])}
  cs_n = cc_n = ss_n = 0
  last_ped = last_cc = last_ss = None
  src_get, pers_get = SRC.get, PERS.get
  for n in sorted(segs[route]):
    p = seg_file(root, route, n, RLOGS)
    q = False
    if p is None:
      p = seg_file(root, route, n, QLOGS)
      q = True
      if p is None:
        info['none'] += 1
        continue
      info['q_segs'] += 1
    info['segs'] += 1
    got = 0
    for e in events_of(rt, p):
      got += 1
      try:
        w = e.which()
        if w == 'can' or w == 'sendcan':
          continue
        if w == 'carState':
          c = e.carState
          cs_n += 1
          gas, brk = c.gasPressed, c.brakePressed
          ped = (gas, brk)
          if q or cs_n % 2 == 0 or gas or brk or ped != last_ped:   # 50 Hz, every pedal frame and every pedal edge
            cs_add((e.logMonoTime * 1e-9, c.vEgo, c.aEgo, c.vCruise, gas, brk))
          last_ped = ped
        elif w == 'carControl':
          c = e.carControl
          cc_n += 1
          st = (c.longActive, c.enabled)
          if q or cc_n % 2 == 0 or st != last_cc:
            cc_add((e.logMonoTime * 1e-9, c.longActive, c.actuators.accel, c.enabled))
          last_cc = st
        elif w == 'radarState':
          l1 = e.radarState.leadOne
          rs_add((e.logMonoTime * 1e-9, l1.status, l1.dRel, l1.vRel, l1.vLead, l1.aLeadK, l1.modelProb))
        elif w == 'longitudinalPlan':
          lp = e.longitudinalPlan
          lp_add((e.logMonoTime * 1e-9, lp.aTarget, src_get(str(lp.longitudinalPlanSource), -1)))
        elif w == 'selfdriveState':
          ss_n += 1
          s = e.selfdriveState
          st = (s.enabled, str(s.personality))
          if q or ss_n % 10 == 0 or st != last_ss:
            ss_add((e.logMonoTime * 1e-9, s.enabled, s.experimentalMode, pers_get(st[1], -1)))
          last_ss = st
        elif w == 'carParams' and info['delay'] != info['delay']:
          cp = e.carParams
          info['delay'], info['op_long'] = float(cp.longitudinalActuatorDelay), int(bool(cp.openpilotLongitudinalControl))
        elif w == 'initData' and info['branch'] == '?':
          info['branch'], info['commit'] = e.initData.gitBranch, e.initData.gitCommit[:7]
      except Exception:
        info['bad'] += 1
    if got == 0:
      info['bad'] += 1
    for tb in T.values():
      tb.flush(np)
  return {k: tb.array(np) for k, tb in T.items()}, info


# ============================================================================================ signals
class Drive:
  pass


def hold(np, t_src, v_src, t, stale=0.5, cap=1.2):
  """Last value at or before each t; nan before the first sample, or once it is older than `stale` s (or than 1.5x the
  gap to the next sample, up to `cap` s, so 4-10 per second qlog data still holds between its samples)."""
  t = np.asarray(t, dtype=np.float64)
  out = np.full(len(t), NAN)
  if len(t_src) == 0 or len(t) == 0:
    return out
  i = np.searchsorted(t_src, t, side='right') - 1
  ok = i >= 0
  ii = np.where(ok, i, 0)
  nxt = np.minimum(ii + 1, len(t_src) - 1)
  gap = np.where(ii + 1 < len(t_src), t_src[nxt] - t_src[ii], 0.0)
  lim = np.maximum(stale, np.minimum(1.5 * gap, cap))
  ok &= (t - t_src[ii]) <= lim
  out[ok] = v_src[ii[ok]]
  return out


def grid_interp(np, t_src, v_src, g, gap=0.15):
  """Linear interpolation onto grid g, nan where the nearest sample is more than `gap` s away."""
  g = np.asarray(g, dtype=np.float64)
  if len(t_src) < 2:
    return np.full(len(g), NAN)
  out = np.interp(g, t_src, v_src)
  j = np.clip(np.searchsorted(t_src, g), 1, len(t_src) - 1)
  near = np.minimum(np.abs(g - t_src[j - 1]), np.abs(t_src[j] - g))
  out[near > gap] = NAN
  return out


def runs_of(np, mask):
  """[(start, end)) index pairs of the True runs of a boolean array."""
  if not len(mask):
    return []
  d = np.diff(np.concatenate([[0], mask.astype(np.int8), [0]]))
  s, e = np.flatnonzero(d == 1), np.flatnonzero(d == -1)
  return list(zip(s.tolist(), e.tolist(), strict=True))


def bridge(np, mask, n):
  """True runs separated by a False gap of at most n samples are joined."""
  out = mask.copy()
  runs = runs_of(np, mask)
  for (a0, a1), (b0, b1) in zip(runs, runs[1:], strict=False):
    if b0 - a1 <= n:
      out[a1:b0] = True
  return out


def slope(np, x, half=10):
  """Least-squares slope of x per second over +-half grid samples (nan where the window touches a nan)."""
  if len(x) < 2 * half + 1:
    return np.full(len(x), NAN)
  k = np.arange(-half, half + 1, dtype=np.float64)
  s = np.correlate(x, k / (np.sum(k * k) * DT), mode='same')
  s[:half] = NAN
  s[-half:] = NAN
  return s


def fin(x):
  x = np.asarray(x, dtype=np.float64)
  return x[np.isfinite(x)]


def nmed(x):
  f = fin(x)
  return float(np.median(f)) if len(f) else NAN


def nq(x, q):
  f = fin(x)
  return float(np.percentile(f, q)) if len(f) else NAN


def nmin(x):
  f = fin(x)
  return float(f.min()) if len(f) else NAN


def nmax(x):
  f = fin(x)
  return float(f.max()) if len(f) else NAN


def isnum(x):
  return x is not None and x == x and not (isinstance(x, float) and math.isinf(x))


def wanted_gap(v, vl, t_follow):
  """The MPC's wanted distance to the car ahead at the first horizon node (get_safe_obstacle_distance minus the lead's
  stopped-equivalence; the 6 m and the /5 are compiled into the solver)."""
  return v ** 2 / (2 * COMFORT_BRAKE) - max(vl, 0.0) ** 2 / (2 * COMFORT_BRAKE) + t_follow * v + STOP_DISTANCE


def band_of(v_mph):
  if not isnum(v_mph) or v_mph < BANDS[0][0]:
    return None
  for i, (lo, hi) in enumerate(BANDS):
    if v_mph < hi or i == len(BANDS) - 1:
      return i
  return None


def band_name(i):
  lo, hi = BANDS[i]
  return f'{lo}-{hi}'


def build(rt, svc, info):
  """All signals on the 0.1 s grid of one drive."""
  np = rt.np
  D = Drive()
  D.np, D.info, D.notes = np, info, []
  dr = rid(info['route'])
  cs = svc['cs']
  if len(cs) < 50:
    return None
  t0, t1 = cs[0, 0], cs[-1, 0]
  G = np.arange(t0, t1 + 1e-9, DT)
  D.G, D.t0 = G, t0
  D.v = grid_interp(np, cs[:, 0], cs[:, 1], G, gap=0.3)
  D.aego = grid_interp(np, cs[:, 0], cs[:, 2], G, gap=0.3)
  sk = hold(np, cs[:, 0], cs[:, 3], G, stale=0.5)
  with np.errstate(invalid='ignore'):
    D.set = np.where((sk > 0) & (sk < 254), sk / 3.6 * MPH, NAN)
  D.gas = hold(np, cs[:, 0], cs[:, 4], G, stale=0.1) > 0.5
  D.brk = hold(np, cs[:, 0], cs[:, 5], G, stale=0.1) > 0.5
  cc = svc['cc']
  D.long = hold(np, cc[:, 0], cc[:, 1], G, stale=0.1) > 0.5
  D.en = hold(np, cc[:, 0], cc[:, 3], G, stale=0.1) > 0.5
  D.ask = np.where(D.long, hold(np, cc[:, 0], cc[:, 2], G, stale=0.1), NAN)
  lp = svc['lp']
  D.aT = hold(np, lp[:, 0], lp[:, 1], G, stale=0.2)
  D.src = hold(np, lp[:, 0], lp[:, 2], G, stale=0.2)
  rs = svc['rs']
  status = hold(np, rs[:, 0], rs[:, 1], G, stale=0.3) > 0.5
  d = hold(np, rs[:, 0], rs[:, 2], G, stale=0.3)
  vrel = hold(np, rs[:, 0], rs[:, 3], G, stale=0.3)
  vl = hold(np, rs[:, 0], rs[:, 4], G, stale=0.3)
  al = hold(np, rs[:, 0], rs[:, 5], G, stale=0.3)
  prob = hold(np, rs[:, 0], rs[:, 6], G, stale=0.3)
  with np.errstate(invalid='ignore'):
    lead_raw = status & np.isfinite(d) & (d > 0) & (d < LEAD_MAX_D) & np.isfinite(vrel)
  lead = bridge(np, lead_raw, int(round(LEAD_BRIDGE_S / DT)))
  filled = lead & ~lead_raw
  if filled.any():
    for arr in (d, vrel, vl, al, prob):
      good = lead_raw & np.isfinite(arr)
      if good.sum() >= 2:
        arr[filled] = np.interp(G[filled], G[good], arr[good])
  D.lead = lead
  D.d = np.where(lead, d, NAN)
  D.vrel = np.where(lead, vrel, NAN)
  D.vl = np.where(lead, vl, NAN)
  D.al = np.where(lead, al, NAN)
  D.prob = np.where(lead, prob, NAN)
  ss = svc['ss']
  D.ss_en = hold(np, ss[:, 0], ss[:, 1], G, stale=1.5) > 0.5 if len(ss) else np.zeros(len(G), bool)
  D.exp = hold(np, ss[:, 0], ss[:, 2], G, stale=1.5) > 0.5 if len(ss) else np.zeros(len(G), bool)
  pers_g = hold(np, ss[:, 0], ss[:, 3], G, stale=1.5) if len(ss) else np.full(len(G), NAN)
  if len(cc) < 50:
    D.notes.append((0, f'drive {dr}: carControl is not in the logs: cannot tell when openpilot drove'))
    if len(ss):
      D.en = D.ss_en
  # modes
  op = D.long & ~D.gas
  ovr = D.en & D.gas
  D.mode = np.where(op, M_OP, np.where(ovr, M_OVR, M_MAN)).astype(np.int8)
  with np.errstate(invalid='ignore'):
    D.hwy = np.isfinite(D.v) & (D.v >= HWY_V)
  D.hwy_lead = D.hwy & D.lead
  n_g = max(len(G), 1)
  if len(rs) < 0.02 * n_g:
    D.notes.append((0, f'drive {dr}: radarState is (mostly) not in the logs: the car ahead is unknown'))
  if len(lp) < 0.02 * n_g:
    D.notes.append((1, f'drive {dr}: longitudinalPlan is (mostly) not in the logs: no plan source (set / lead told by the speeds)'))
  if info['q_segs']:
    D.notes.append((0, f"drive {dr}: {info['q_segs']} of {info['segs']} segments only have a qlog: the car ahead 4 times a"
                       " second there, pedals and openpilot 10 times: coarse"))
  if info['op_long'] == 0:
    D.notes.append((0, f"drive {dr}: carParams says openpilot long control is OFF: the car's own ACC followed"))
  # personality (the MPC's follow time): the most common while engaged
  pe = pers_g[D.en & np.isfinite(pers_g)] if len(ss) else pers_g[:0]
  if len(pe):
    vals, cnt = np.unique(pe.astype(int), return_counts=True)
    D.pers = PERS_NAME.get(int(vals[np.argmax(cnt)]), '?')
  else:
    D.pers = '?'
  D.t_follow = T_FOLLOW.get(D.pers, T_FOLLOW['aggressive'])
  if D.pers not in ('aggressive', '?'):
    D.notes.append((0, f'drive {dr}: personality {D.pers} (T = {D.t_follow} s), not aggressive'))
  if (D.exp & D.en).sum() > 50:
    D.notes.append((0, f'drive {dr}: experimental mode was on for {(D.exp & D.en).sum() * DT:.0f} s while engaged'))
  with np.errstate(invalid='ignore'):
    D.want = (D.v ** 2 - np.maximum(D.vl, 0.0) ** 2) / (2 * COMFORT_BRAKE) + D.t_follow * D.v + STOP_DISTANCE
    D.tg = D.d / D.v
    D.ttc = np.where(D.vrel < -0.1, D.d / np.maximum(-D.vrel, 0.1), INF)
  D.dslope = slope(np, D.d)
  return D


def clock(D, k):
  t = D.G[k] - D.t0
  return f'{int(t // 60)}:{int(t % 60):02d}'


# ============================================================================================ A. modes
def mode_shares(D):
  """Highway time per mode, with and without a car ahead; brake share."""
  out = {}
  for name, m in (('all', D.hwy), ('lead', D.hwy_lead)):
    n = int(m.sum())
    out[name] = {'s': n * DT, 'mode': [int((m & (D.mode == i)).sum()) * DT for i in range(3)],
                 'brk': int((m & D.brk).sum()) * DT}
  with np.errstate(invalid='ignore'):
    moving = D.v > 0.5
  out['move_s'] = float(moving.sum() * DT)
  out['op_s'] = float((D.long & moving).sum() * DT)
  return out


# ============================================================================================ B. steady following
def steady_points(D):
  np = D.np
  with np.errstate(invalid='ignore'):
    return D.hwy_lead & (np.abs(D.vrel) <= STEADY_VREL) & (np.abs(D.dslope) <= STEADY_SLOPE)


def settled_subset(D, use, sl):
  """The settled points (gap trend under SETTLED_SLOPE) of a steady selection, if there are SETTLED_MIN_S of them; else all."""
  np = D.np
  with np.errstate(invalid='ignore'):
    st = use & (np.abs(D.dslope[sl]) <= SETTLED_SLOPE)
  return (st, True) if st.sum() >= int(round(SETTLED_MIN_S / DT)) else (use, False)


def episodes_of(np, pts, mode, min_n, bridge_n):
  """[(k0, k1, mode)] runs of pts in one mode; breaks up to bridge_n samples inside the same mode are bridged; a run needs
  min_n points of pts."""
  key = np.where(pts, mode, -1)
  out = []
  k = 0
  n = len(key)
  while k < n:
    if key[k] < 0:
      k += 1
      continue
    m = key[k]
    j = k
    while j < n and key[j] == m:
      j += 1
    out.append([k, j, int(m)])
    k = j
  merged = []
  for k0, k1, m in out:
    if merged and m == merged[-1][2] and k0 - merged[-1][1] <= bridge_n and np.all(mode[merged[-1][1]:k0] == m):
      merged[-1][1] = k1
    else:
      merged.append([k0, k1, m])
  return [(k0, k1, m) for k0, k1, m in merged if int(pts[k0:k1].sum()) >= min_n]


def find_steady(D, pts):
  np = D.np
  eps = []
  for k0, k1, m in episodes_of(np, pts, D.mode, int(round(STEADY_MIN_S / DT)), int(round(STEADY_BRIDGE_S / DT))):
    sl = slice(k0, k1)
    use_all = pts[sl]
    use, settled = settled_subset(D, use_all, sl)
    v, d, tg = D.v[sl][use], D.d[sl][use], D.tg[sl][use]
    v_med = nmed(v)
    set_mph = nmed(D.set[sl])
    src = D.src[sl][use]
    lead_src = np.isin(src, [SRC['lead0'], SRC['lead1']])
    src_known = np.isfinite(src) & (src >= 0)
    if m == M_OP:
      if src_known.sum() >= 0.5 * len(src):
        limited = 'lead' if lead_src.sum() >= 0.5 * src_known.sum() else 'set'
      else:
        limited = 'set' if isnum(set_mph) and v_med >= set_mph / MPH - SET_NEAR else 'lead'
    elif m == M_OVR:
      limited = 'gas'
    else:
      limited = '-'
    d_med = nmed(d)
    post = False
    if m == M_OP:
      before = np.flatnonzero(D.mode[max(k0 - int(round(SETTLE_S / DT)), 0):k0] != M_OP)
      if len(before):
        k_h = max(k0 - int(round(SETTLE_S / DT)), 0) + int(before[-1])       # the last frame before openpilot took over
        d_h = D.d[k_h] if D.lead[k_h] else NAN
        post = not isnum(d_h) or abs(d_med - d_h) > max(SETTLE_MOVED, SETTLE_MOVED_PC * d_med)
    kk = np.flatnonzero(use_all).astype(np.float64) * DT
    d_all = D.d[sl][use_all]
    drift_m = float(np.polyfit(kk, d_all, 1)[0] * (kk[-1] - kk[0])) if len(kk) >= 2 else 0.0
    drift = abs(drift_m) > max(DRIFT_M, DRIFT_PC * d_med)
    e = {'kind': 'steady', 'rid': D.rid, 'route': D.route, 'k0': k0, 'k1': k1, 't': float(D.G[k0]), 'clock': clock(D, k0),
         'mode': m, 'dur': (k1 - k0) * DT, 'post': post, 'drift': drift, 'drift_m': drift_m, 'steady_s': float(use.sum()) * DT,
         'settled': settled, 'steady_all_s': float(use_all.sum()) * DT,
         'v': v_med, 'v_mph': v_med * MPH,
         'band': band_of(v_med * MPH), 'd': nmed(d), 'd_q1': nq(d, 25), 'd_q3': nq(d, 75), 'tg': nmed(tg),
         'tg_p10': nq(tg, 10), 'tg_p90': nq(tg, 90), 'set_mph': set_mph, 'limited': limited,
         'want': nmed(D.want[sl][use]), 'T_eq': nmed((d - STOP_DISTANCE) / v), 'vl_mph': nmed(D.vl[sl][use]) * MPH,
         'vrel': nmed(D.vrel[sl][use]), 'prob': nmed(D.prob[sl][use]), 'ask': nmed(D.ask[sl][use]) if m == M_OP else NAN,
         'brk': bool(D.brk[sl].any()), 'pts_d': d.astype(np.float32), 'pts_v': v.astype(np.float32), 't_follow': D.t_follow}
    eps.append(e)
  return eps


# ============================================================================================ C. gas overrides
def find_overrides(D, pts):
  np = D.np
  n = len(D.G)
  after_n = int(round(OVR_AFTER_S / DT))
  out, taps = [], 0
  for k_dn, k_up in runs_of(np, D.en & D.gas):
    kl = k_up - 1
    if not (D.hwy[k_dn] or D.hwy[kl]):
      continue
    if not (D.lead[k_dn] or D.lead[kl]):
      continue
    dur = (k_up - k_dn) * DT
    if dur < OVR_MIN_S:
      taps += 1
      continue
    sl = slice(k_dn, k_up)
    use = pts[sl]
    settled_steady = bool(use.sum() >= 10)
    if settled_steady:
      use, _ = settled_subset(D, use, sl)
    else:
      use = np.zeros(k_up - k_dn, bool)
      use[max(0, k_up - k_dn - int(round(OVR_SETTLE_S / DT))):] = True
      use &= D.lead[sl]
    kb = max(k_dn - 3, 0)
    e = {'kind': 'override', 'rid': D.rid, 'route': D.route, 'k0': k_dn, 'k1': k_up, 't': float(D.G[k_dn]),
         'clock': clock(D, k_dn), 'mode': M_OVR, 'dur': dur, 'v_dn': float(D.v[k_dn]), 'd_dn': float(D.d[k_dn]),
         'tg_dn': float(D.tg[k_dn]), 'vrel_dn': float(D.vrel[k_dn]), 'ask_dn': nmed(D.ask[kb:k_dn]) if k_dn > kb else NAN,
         'want_dn': float(D.want[k_dn]), 'en_before': bool(D.long[kb:k_dn].any()) if k_dn > kb else bool(D.long[k_dn]),
         'd_st': nmed(D.d[sl][use]), 'tg_st': nmed(D.tg[sl][use]), 'v_st': nmed(D.v[sl][use]), 'st_steady': settled_steady,
         'v_up': float(D.v[kl]), 'd_up': float(D.d[kl]), 'tg_up': float(D.tg[kl]), 'vrel_up': float(D.vrel[kl]),
         'want_up': float(D.want[kl]), 'set_mph': nmed(D.set[sl]), 'brk_in': bool(D.brk[sl].any()),
         'v_max': nmax(D.v[sl]), 'band': band_of(nmed(D.v[sl][use]) * MPH) if use.any() else band_of(float(D.v[kl]) * MPH)}
    e['room_up'] = e['d_up'] - e['want_up'] if isnum(e['d_up']) and isnum(e['want_up']) else NAN
    # the 5 s after the release
    a0, a1 = k_up, min(k_up + after_n + 1, n)
    aft = slice(a0, a1)
    op_after = D.mode[aft] == M_OP
    e['after_op_s'] = float(op_after.sum()) * DT
    ask = np.where(op_after, D.ask[aft], NAN)
    e['a_min'] = nmin(ask)
    e['a_min_t'] = (int(np.nanargmin(ask)) * DT) if np.isfinite(ask).any() else NAN
    kb2 = np.flatnonzero(np.isfinite(ask) & (ask < BRAKE_ON))
    e['brk_t'] = float(kb2[0] * DT) if len(kb2) else NAN
    e['brk_pedal_after'] = bool(D.brk[aft].any())
    g = np.flatnonzero(D.gas[a0 + 1:a1])
    e['regas_t'] = float((g[0] + 1) * DT) if len(g) else NAN
    d_a = D.d[aft]
    last = np.flatnonzero(np.isfinite(d_a))
    e['d_end'] = float(d_a[last[-1]]) if len(last) else NAN
    e['d_grow'] = e['d_end'] - e['d_up'] if isnum(e['d_end']) and isnum(e['d_up']) else NAN
    e['d_grow_max'] = nmax(d_a) - e['d_up'] if isnum(e['d_up']) and np.isfinite(d_a).any() else NAN
    e['v_drop'] = (e['v_up'] - nmin(D.v[aft])) if isnum(e['v_up']) else NAN
    e['lead_after'] = float(D.lead[aft].sum()) * DT
    out.append(e)
  return out, taps


# ============================================================================================ D. approaches
def find_approaches(D):
  np = D.np
  n = len(D.G)
  with np.errstate(invalid='ignore'):
    core = D.lead & (D.vrel <= APP_END_VREL)
    fast = D.lead & (D.vrel <= APP_VREL) & D.hwy
  core = bridge(np, core, int(round(APP_BRIDGE_S / DT)))
  out = []
  min_n = int(round(APP_MIN_S / DT))
  after_n = int(round(APP_AFTER_S / DT))
  for r0, r1 in runs_of(np, core):
    idx = np.flatnonzero(fast[r0:r1])
    if len(idx) < min_n:
      continue
    k0 = r0 + int(idx[0])
    k1 = r1
    kz = min(k1 + after_n, n)
    sl = slice(k0, kz)
    m0 = int(D.mode[k0])
    e = {'kind': 'approach', 'rid': D.rid, 'route': D.route, 'k0': k0, 'k1': k1, 't': float(D.G[k0]), 'clock': clock(D, k0),
         'mode': m0, 'dur': (k1 - k0) * DT, 'v0': float(D.v[k0]), 'vl0': float(D.vl[k0]), 'd0': float(D.d[k0]),
         'vrel0': float(D.vrel[k0]), 'ttc0': float(D.ttc[k0]), 'tg0': float(D.tg[k0]), 'al_min': nmin(D.al[k0:k1]),
         'd_min': nmin(D.d[sl]), 'v_end': float(D.v[k1 - 1]), 'vrel_end': float(D.vrel[k1 - 1]),
         'band': band_of(float(D.v[k0]) * MPH), 'set_mph': nmed(D.set[k0:k1]), 'brk_pedal': bool(D.brk[k0:k1].any())}
    e['d_min_t'] = (int(np.nanargmin(D.d[sl])) * DT) if np.isfinite(D.d[sl]).any() else NAN
    if k1 >= n or not D.lead[min(k1, n - 1)]:
      e['end'] = 'lost' if k1 < n else 'log end'
    else:
      e['end'] = 'matched'
    # openpilot's braking (while in OP mode anywhere in the approach + 2 s)
    op = D.mode[sl] == M_OP
    ask = np.where(op, D.ask[sl], NAN)
    e['op_s'] = float(op.sum()) * DT
    kb = np.flatnonzero(np.isfinite(ask) & (ask < BRAKE_ON))
    if len(kb):
      k = k0 + int(kb[0])
      e.update({'brk_k': k, 'brk_t': float(kb[0] * DT), 'brk_d': float(D.d[k]), 'brk_tg': float(D.tg[k]), 'brk_ttc': float(D.ttc[k]),
                'brk_v': float(D.v[k]), 'brk_vrel': float(D.vrel[k]), 'brk_want': float(D.want[k])})
    else:
      e['brk_k'] = None
    e['a_min'] = nmin(ask)
    e['a_min_t'] = (int(np.nanargmin(ask)) * DT) if np.isfinite(ask).any() else NAN
    e['aT_min'] = nmin(np.where(op, D.aT[sl], NAN))
    # the driver: the gas lifted (the end of a gas press) or the brake pressed inside the approach
    gas, brk = D.gas[k0:kz], D.brk[k0:kz]
    lift = np.flatnonzero(gas[:-1] & ~gas[1:]) + 1
    press = np.flatnonzero(~brk[:-1] & brk[1:]) + 1
    cands = [(int(j), 'lift') for j in lift] + [(int(j), 'brake') for j in press]
    if cands:
      j, kind = min(cands)
      k = k0 + j
      e.update({'drv_k': k, 'drv_kind': kind, 'drv_t': j * DT, 'drv_d': float(D.d[k]), 'drv_tg': float(D.tg[k]),
                'drv_ttc': float(D.ttc[k]), 'drv_v': float(D.v[k]), 'drv_mode': int(D.mode[max(k - 1, 0)])})
    else:
      e['drv_k'] = None
    # the gas pressed while openpilot was braking in the approach (openpilot too early for the driver)
    gp = np.flatnonzero(~gas[:-1] & gas[1:] & D.en[k0:kz][1:]) + 1
    gp = [int(j) for j in gp if D.mode[k0 + j - 1] == M_OP]
    if gp:
      j = gp[0]
      k = k0 + j
      e.update({'gas_k': k, 'gas_t': j * DT, 'gas_d': float(D.d[k]), 'gas_tg': float(D.tg[k]), 'gas_ttc': float(D.ttc[k]),
                'gas_ask': nmed(D.ask[max(k - 3, k0):k]) if k > k0 else NAN, 'gas_v': float(D.v[k])})
    else:
      e['gas_k'] = None
    out.append(e)
  return out


# ============================================================================================ E. the physics
def lead_brake_absorbed(d, v, t_r=REACT_S, a_e=A_EGO_MAX):
  """Hardest steady braking of the car ahead (m/s2) that this car, reacting after t_r s and then braking at a_e, can absorb
  from gap d without touching it; both start at speed v, the car ahead brakes to a stop. inf = even an instant stop."""
  if not (isnum(d) and isnum(v)) or v <= 0.1:
    return NAN

  def min_gap(a_l):
    t_l, t_e = v / a_l, t_r + v / a_e
    t = np.arange(0.0, max(t_l, t_e) + 0.05, 0.01)
    x_l = np.where(t < t_l, v * t - 0.5 * a_l * t * t, v * v / (2 * a_l))
    te = np.clip(t - t_r, 0.0, v / a_e)
    x_e = v * np.minimum(t, t_r) + v * te - 0.5 * a_e * te * te
    return float(np.min(d + x_l - x_e))

  if min_gap(A_LEAD_CAP) >= 0:
    return INF
  lo, hi = 0.05, A_LEAD_CAP
  if min_gap(lo) < 0:
    return 0.0
  for _ in range(40):
    mid = 0.5 * (lo + hi)
    if min_gap(mid) >= 0:
      lo = mid
    else:
      hi = mid
  return lo


# What openpilot ALONE absorbs, from a closed-loop simulation of this branch (the real planner + MPC + compiled solver, the VW
# long control and a plant with the car's 0.15 s delay / 0.35 s lag, a camera lead with 0.2 s latency and noise, worst of two
# noise seeds; hwyfollow_verify/vsim.py V11): the hardest steady braking of the car ahead, to a stop, from the settled gap
# 6 m + T x speed, without touching it. Far below the physics bound above because the MPC's lead-acceleration extrapolation decays
# within ~3 s, its comfort costs and the delays make the stack take 3-4 s to reach full braking (the physics line assumes 0.5 s).
SIM_ABSORB_T = (0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.25)        # s
SIM_ABSORB_MPH = (45, 55, 65, 75, 85, 90)
SIM_ABSORB = {                                              # m/s2, rows = T, columns = mph
  0.50: (2.25, 1.75, 1.25, 1.25, 1.00, 1.00),
  0.60: (2.25, 2.25, 1.75, 1.50, 1.25, 1.25),
  0.70: (2.25, 2.25, 2.25, 1.75, 1.75, 1.75),
  0.80: (2.50, 2.50, 2.50, 2.25, 2.00, 2.25),
  0.90: (2.75, 2.50, 2.50, 2.50, 2.25, 2.25),
  1.00: (2.75, 2.75, 2.50, 2.50, 2.50, 2.25),
  1.25: (3.00, 3.00, 2.75, 2.75, 3.00, 3.00),
}


def sim_absorbed(T, v_mph):
  """SIM_ABSORB interpolated in T (clamped to the table) and speed."""
  if not isnum(T):
    return NAN
  T = min(max(T, SIM_ABSORB_T[0]), SIM_ABSORB_T[-1])
  rows = [np.interp(v_mph, SIM_ABSORB_MPH, SIM_ABSORB[t]) for t in SIM_ABSORB_T]
  return float(np.interp(T, SIM_ABSORB_T, rows))


def wmedian(np, vals, w):
  """Median of vals weighted by w (seconds)."""
  vals, w = np.asarray(vals, dtype=np.float64), np.asarray(w, dtype=np.float64)
  ok = np.isfinite(vals) & (w > 0)
  if not ok.any():
    return NAN
  o = np.argsort(vals[ok])
  v, ww = vals[ok][o], w[ok][o]
  c = np.cumsum(ww)
  return float(v[np.searchsorted(c, 0.5 * c[-1])])


def pooled(np, eps, mode, band=None, limited=None, settled=False):
  """Time-weighted quantiles of the time gap, the gap and T = (d - 6) / v over the steady points of episodes."""
  modes = mode if isinstance(mode, tuple) else (mode,)
  sel = [e for e in eps if e['mode'] in modes and (band is None or e['band'] == band) and (limited is None or e['limited'] == limited)
         and not (settled and (e['post'] or e['drift']))]
  if not sel:
    return None
  d = np.concatenate([e['pts_d'] for e in sel]).astype(np.float64)
  v = np.concatenate([e['pts_v'] for e in sel]).astype(np.float64)
  tg = d / v
  T = (d - STOP_DISTANCE) / v
  return {'n': len(sel), 's': sum(e['steady_s'] for e in sel), 'tg': [nq(tg, q) for q in (25, 50, 75)],
          'd': [nq(d, q) for q in (25, 50, 75)], 'T': [nq(T, q) for q in (25, 50, 75)], 'v': nmed(v),
          'n_set': sum(1 for e in sel if e['limited'] == 'set'), 'n_post': sum(1 for e in sel if e['post'] or e['drift']),
          'tg_p10': nq(tg, 10), 'tg_p90': nq(tg, 90)}


def recommend(np, steady, t_code):
  """T that copies the driver's steady override gap; the resulting gaps and the safety margins."""
  rec = {'t_code': t_code}
  for key, mode, lim, settled in (('ovr', M_OVR, None, False), ('man', M_MAN, None, False), ('op', M_OP, 'lead', True),
                                  ('op_all', M_OP, None, False)):
    rec[key] = pooled(np, steady, mode, None, lim, settled)
    rec[key + '_bands'] = [pooled(np, steady, mode, b, lim, settled) for b in range(len(BANDS))]
  rec['drv'] = pooled(np, steady, (M_OVR, M_MAN))
  rec['drv_bands'] = [pooled(np, steady, (M_OVR, M_MAN), b) for b in range(len(BANDS))]
  if rec['ovr'] and (rec['ovr']['s'] >= DRIVER_OVR_MIN_S or not rec['man']):
    rec['T'], rec['T_src'], rec['use'] = rec['ovr']['T'][1], 'OVERRIDE', 'ovr'
  elif rec['ovr'] and rec['man']:
    rec['T'], rec['T_src'], rec['use'] = rec['drv']['T'][1], 'OVERRIDE+MANUAL', 'drv'
  elif rec['man']:
    rec['T'], rec['T_src'], rec['use'] = rec['man']['T'][1], 'MANUAL', 'man'
  else:
    rec['T'], rec['T_src'], rec['use'] = NAN, None, None
  rec['T_man'] = rec['man']['T'][1] if rec['man'] else NAN
  rec['T_op'] = rec['op']['T'][1] if rec['op'] else NAN
  rec['T_use'] = max(rec['T'], 0.0) if isnum(rec['T']) else NAN
  rec['gaps'] = {}
  rec['safety'] = {}
  rec['sim'] = {}
  for name, T in (('rec', rec['T_use']), ('today', t_code), ('man', max(rec['T_man'], 0.0) if isnum(rec['T_man']) else NAN)):
    if not isnum(T):
      continue
    rec['gaps'][name] = {m: STOP_DISTANCE + T * m / MPH for m in REC_SPEEDS_MPH}
    rec['safety'][name] = {m: lead_brake_absorbed(STOP_DISTANCE + T * m / MPH, m / MPH) for m in SAFETY_SPEEDS_MPH}
    rec['sim'][name] = {m: sim_absorbed(T, m) for m in SAFETY_SPEEDS_MPH}
  return rec


# ============================================================================================ text
def fmt(x, nd=2, w=5, plus=False):
  if x is None or x != x or (isinstance(x, float) and math.isinf(x)):
    return ' ' * (w - 1) + ('-' if not (isinstance(x, float) and x == INF) else 'inf')
  x = round(x, nd) + 0.0     # no '-0.00'
  return f'{x:+{w}.{nd}f}' if plus else f'{x:{w}.{nd}f}'


def f1(x, nd=1):
  """Compact number for key=value text."""
  if x is None or x != x:
    return '-'
  if isinstance(x, float) and math.isinf(x):
    return 'wall' if x > 0 else '-'
  return f'{round(x, nd) + 0.0:.{nd}f}'


def f1p(x, nd=1):
  if not isnum(x):
    return '-'
  return f'{round(x, nd) + 0.0:+.{nd}f}'


def mph(v):
  return v * MPH if isnum(v) else NAN


def ttc_txt(x, w=5):
  if not isnum(x):
    return ' ' * (w - 1) + '-'
  return f'{x:{w}.1f}'[:w] if x < 99 else ' ' * (w - 3) + '>99'


def brk_abs_txt(x):
  if not isnum(x):
    return 'even an instant stop (a wall)' if isinstance(x, float) and x == INF else '-'
  return f'{x:.1f} m/s2'


def letter(e):
  return KIND_LETTER[e['kind']] + str(e['n'])


def print_modes(R, tot):
  say('A. WHO DROVE AT 45 MPH AND FASTER (minutes; the modes as shares of the highway time; brk = brake pedal down)')
  say('   drive   moving  with OP   45+ mph  OP   OVR  MAN  brk  | 45+ with a car ahead  OP   OVR  MAN  brk   pers')
  rows = [(r['rid'], r['modes']) for r in R] + ([('all', tot)] if len(R) > 1 else [])
  def pc(x, s):
    return f'{100 * x / s:3.0f}%' if s > 0 else '   -'
  for name, m in rows:
    a, ld = m['all'], m['lead']
    say(f"   {name:<7} {m['move_s'] / 60:5.1f}  {m['op_s'] / 60:5.1f}    {a['s'] / 60:5.1f}  "
        + ' '.join(pc(x, a['s']) for x in a['mode']) + f" {pc(a['brk'], a['s'])}  |   {ld['s'] / 60:5.1f} min          "
        + ' '.join(pc(x, ld['s']) for x in ld['mode']) + f" {pc(ld['brk'], ld['s'])}   "
        + (next((r['pers'] for r in R if r['rid'] == name), '') if name != 'all' else ''))


def print_steady(eps, rec):
  say('B. STEADY FOLLOWING EPISODES AT 45+ (a car ahead at nearly the same speed for 5 s or more, one mode)')
  say('   # = episode number (--detail #); gap = median metres; tgap = gap / speed with its 10-90 % spread; T = (gap - 6) / speed,'
      ' the follow time that would make the code hold this gap;')
  say("   code wants = 6 m + T_code x speed (+ the speed-difference term); held by: in OP, what held the speed: 'lead' = the car ahead"
      " (the gap is openpilot's choice), 'set' = the set speed (the gap is not),")
  say(f"   '*' = openpilot took over less than {SETTLE_S:.0f} s before (engaged, or the gas let go) and the gap has moved since: still"
      " settling; drift = how far the gap moved end to end ('d' = more than 20 %): both left out of the OP headline;")
  say("   ask = openpilot's median request; prob = the model's lead probability; brk = the brake pedal was pressed in the episode")
  say('     #  drive     at  mode  band    s    mph  set  held  lead mph  gap m [q1-q3]  tgap s [10-90%]   T s   code wants  drift  ask   prob brk')
  for e in eps:
    say(f"  {letter(e):>4}  {e['rid']:>5} {e['clock']:>6}  {MODE_SHORT[e['mode']]:<4} {band_name(e['band']) if e['band'] is not None else '-':<6}"
        f" {e['dur']:4.0f} {e['v_mph']:6.1f} {fmt(e['set_mph'], 0, 4)}  {e['limited'] + ('*' if e['post'] else ''):<5} {fmt(e['vl_mph'], 0, 6)}"
        f"   {e['d']:5.1f} [{e['d_q1']:4.1f}-{e['d_q3']:4.1f}]  {e['tg']:5.2f} [{e['tg_p10']:4.2f}-{e['tg_p90']:4.2f}]  {e['T_eq']:5.2f}"
        f"  {fmt(e['want'], 1, 7)}m {fmt(e['drift_m'], 0, 4, True)}{'d' if e['drift'] else ' '} {fmt(e['ask'], 2, 5)}  {fmt(e['prob'], 2, 4)}"
        f" {'yes' if e['brk'] else '  -'}")
  say()
  say('   PER SPEED BAND AND MODE (time-weighted over the steady points; median [quartiles]; set-ltd = OP episodes where the set'
      ' speed, not the car ahead, held the speed; settling = OP episodes still settling or drifting)')
  say('   band    mode  episodes  steady s   tgap s median [q1-q3]   gap m median [q1-q3]   T s median [q1-q3]  mph   code wants m  set-ltd settling')
  for b in range(len(BANDS)):
    for mode in (M_OVR, M_MAN, M_OP):
      key = {M_OVR: 'ovr', M_MAN: 'man', M_OP: 'op_all'}[mode]
      p = rec[key + '_bands'][b]
      if p is None:
        continue
      want = STOP_DISTANCE + rec['t_code'] * p['v']
      say(f"   {band_name(b):<7} {MODE_SHORT[mode]:<4}  {p['n']:8d}  {p['s']:8.0f}   {p['tg'][1]:5.2f} [{p['tg'][0]:4.2f}-{p['tg'][2]:4.2f}]"
          f"          {p['d'][1]:5.1f} [{p['d'][0]:4.1f}-{p['d'][2]:4.1f}]        {p['T'][1]:5.2f} [{p['T'][0]:4.2f}-{p['T'][2]:4.2f}]"
          f"     {p['v'] * MPH:4.0f}   {want:7.1f}      {p['n_set'] if mode == M_OP else '-':>4} {p['n_post'] if mode == M_OP else '-':>6}")
  say()
  parts = []
  for key, nm in (('ovr', 'OVERRIDE (the gas held down, the driver chose the gap)'), ('man', 'MANUAL (openpilot off)'),
                  ('op', 'OP settled, the car ahead holding the speed'), ('op_all', 'OP, all episodes (settling and set-speed ones too)')):
    p = rec[key]
    if p:
      parts.append(f"   {nm}: {p['n']} episodes, {p['s']:.0f} s: time gap {p['tg'][1]:.2f} s [{p['tg'][0]:.2f}-{p['tg'][2]:.2f}]"
                   f" (10-90 % {p['tg_p10']:.2f}-{p['tg_p90']:.2f}), gap {p['d'][1]:.0f} m at {p['v'] * MPH:.0f} mph, T {p['T'][1]:.2f} s")
    else:
      parts.append(f'   {nm}: none')
  say('   HEADLINE (all bands):')
  for ln in parts:
    say(ln)
  say(f"   the code wants 6 m + {rec['t_code']:.2f} s x speed: {STOP_DISTANCE + rec['t_code'] * 50 / MPH:.0f} m at 50 mph,"
      f" {STOP_DISTANCE + rec['t_code'] * 65 / MPH:.0f} m at 65, {STOP_DISTANCE + rec['t_code'] * 75 / MPH:.0f} m at 75,"
      f" {STOP_DISTANCE + rec['t_code'] * 90 / MPH:.0f} m at 90 (time gap {rec['t_code'] + STOP_DISTANCE / (50 / MPH):.2f} s at 50,"
      f" {rec['t_code'] + STOP_DISTANCE / (90 / MPH):.2f} s at 90)")


def print_overrides(ovr, taps):
  say(f'C. GAS OVERRIDES AT 45+ WITH A CAR AHEAD (openpilot engaged, the gas held down for 1 s or more; {taps} shorter taps not listed)')
  say('   gas down: speed, gap / time gap, closing speed, what openpilot asked just before (ask); settled: the gap the driver kept'
      " (~ = no steady part: last 5 s); gas up: gap / time gap, the code's wanted gap and room = gap - wanted (minus = inside it);")
  say("   after (5 s): openpilot's strongest request and when, when it first braked under -0.3, how much the gap grew and the speed"
      ' dropped, gas again = the driver pressed the gas again within 5 s')
  say('     #  drive     at   mph  set  | down: gap    tgap  vRel   ask  | held s  settled gap  tgap | up: mph  gap    tgap  wanted  room'
      ' | after: min   @s  brk@s  gap+  mph-  gas again')
  for e in ovr:
    say(f"  {letter(e):>4}  {e['rid']:>5} {e['clock']:>6} {mph(e['v_dn']):5.0f} {fmt(e['set_mph'], 0, 4)}  | {fmt(e['d_dn'], 0, 7)}m"
        f" {fmt(e['tg_dn'], 2, 5)}s {fmt(e['vrel_dn'], 1, 5)} {fmt(e['ask_dn'], 2, 5)}  | {e['dur']:5.0f}  {fmt(e['d_st'], 0, 7)}m"
        f"{'~' if not e['st_steady'] else ' '}  {fmt(e['tg_st'], 2, 4)} | {mph(e['v_up']):7.0f} {fmt(e['d_up'], 0, 4)}m {fmt(e['tg_up'], 2, 5)}s"
        f" {fmt(e['want_up'], 0, 5)}m {fmt(e['room_up'], 0, 5, True)} | {fmt(e['a_min'], 2, 10)} {fmt(e['a_min_t'], 1, 4)} {fmt(e['brk_t'], 1, 5)}"
        f" {fmt(e['d_grow'], 0, 5, True)}m {fmt(mph(e['v_drop']), 0, 4)}  {fmt(e['regas_t'], 1, 5) + 's' if isnum(e['regas_t']) else '    -'}"
        + ('  brake' if e['brk_pedal_after'] or e['brk_in'] else '') + ('  (OP not back)' if e['after_op_s'] < 1.0 else ''))
  say()


def ovr_summary(ovr):
  """Medians over the overrides and the worst / typical fall-back."""
  if not ovr:
    return None
  valid = [e for e in ovr if e['after_op_s'] >= 1.0]
  s = {'n': len(ovr), 'dur': nmed([e['dur'] for e in ovr]), 'tg_dn': nmed([e['tg_dn'] for e in ovr]),
       'ask_dn': nmed([e['ask_dn'] for e in ovr]), 'tg_st': nmed([e['tg_st'] for e in ovr]), 'tg_up': nmed([e['tg_up'] for e in ovr]),
       'room_up': nmed([e['room_up'] for e in ovr]), 'a_min': nmed([e['a_min'] for e in valid]), 'd_grow': nmed([e['d_grow'] for e in valid]),
       'v_drop': nmed([e['v_drop'] for e in valid]), 'n_regas': sum(1 for e in ovr if isnum(e['regas_t'])),
       'n_brk_dn': sum(1 for e in ovr if isnum(e['ask_dn']) and e['ask_dn'] < BRAKE_ON), 'n_valid': len(valid)}
  s['worst'] = min(valid, key=lambda e: e['a_min'] if isnum(e['a_min']) else 0.0) if valid else None
  if valid:
    med = s['a_min']
    s['typical'] = min(valid, key=lambda e: abs((e['a_min'] if isnum(e['a_min']) else 0.0) - med))
  else:
    s['typical'] = None
  return s


def print_approaches(app):
  say('D. APPROACHES AT 45+ (closing on a slower car by 2 m/s or more for 1 s or more)')
  say("   start: mode, speed, the car ahead's speed, gap, time to collision (TTC); OP braked: when openpilot first asked under -0.3 (s after"
      ' the start; gap, tgap, TTC there) and its strongest request; driver: the gas lifted / brake pressed (gap, tgap, TTC);')
  say('   gas in OP: the driver pressed the gas while openpilot braked in the approach (gap, TTC, what openpilot asked); closest = the'
      " smallest gap; end = matched the car ahead's speed / lost it")
  say('     #  drive     at  mode   mph  lead  gap  TTC  | OP braked @s  gap  tgap   TTC   min   | driver     @s  gap  tgap   TTC  |'
      ' gas in OP @s  gap   TTC   ask | closest  end')
  for e in app:
    op = (f"{fmt(e['brk_t'], 1, 5)} {fmt(e['brk_d'], 0, 4)}m {fmt(e['brk_tg'], 2, 5)} {ttc_txt(e['brk_ttc'])} {fmt(e['a_min'], 2, 6)}"
          if e['brk_k'] is not None else (f"  none           {fmt(e['a_min'], 2, 6)}" if e['op_s'] > 0 else '  (not in OP)        '))
    drv = (f"{e['drv_kind']:<6} {fmt(e['drv_t'], 1, 5)} {fmt(e['drv_d'], 0, 4)}m {fmt(e['drv_tg'], 2, 5)} {ttc_txt(e['drv_ttc'])}"
           if e['drv_k'] is not None else '  -                        ')
    gas = (f"{fmt(e['gas_t'], 1, 5)} {fmt(e['gas_d'], 0, 4)}m {ttc_txt(e['gas_ttc'])} {fmt(e['gas_ask'], 2, 5)}"
           if e['gas_k'] is not None else '  -                   ')
    say(f"  {letter(e):>4}  {e['rid']:>5} {e['clock']:>6}  {MODE_SHORT[e['mode']]:<4} {mph(e['v0']):5.0f} {mph(e['vl0']):5.0f}"
        f" {fmt(e['d0'], 0, 4)}m {ttc_txt(e['ttc0'])} | {op} | {drv} | {gas} | {fmt(e['d_min'], 0, 4)}m  {e['end']}")
  say()


def app_summary(app):
  out = {'op': [e for e in app if e['mode'] == M_OP], 'drv': [e for e in app if e['mode'] != M_OP]}
  op = out['op']
  brk = [e for e in op if e['brk_k'] is not None]
  s = {'n_op': len(op), 'n_brk': len(brk), 'brk_d': nmed([e['brk_d'] for e in brk]), 'brk_tg': nmed([e['brk_tg'] for e in brk]),
       'brk_ttc': nmed([e['brk_ttc'] for e in brk if e['brk_ttc'] < 99]), 'a_min': nmed([e['a_min'] for e in op]),
       'brk_t': nmed([e['brk_t'] for e in brk]), 'n_gas': sum(1 for e in op if e['gas_k'] is not None),
       'gas_ttc': nmed([e['gas_ttc'] for e in op if e['gas_k'] is not None and e['gas_ttc'] < 99]),
       'gas_d': nmed([e['gas_d'] for e in op if e['gas_k'] is not None]),
       'gas_ask': nmed([e['gas_ask'] for e in op if e['gas_k'] is not None])}
  drv = out['drv']
  dl = [e for e in drv if e['drv_k'] is not None]
  s.update({'n_drv': len(drv), 'n_ovr': sum(1 for e in drv if e['mode'] == M_OVR), 'n_man': sum(1 for e in drv if e['mode'] == M_MAN),
            'n_drv_act': len(dl), 'n_lift': sum(1 for e in dl if e['drv_kind'] == 'lift'),
            'n_brake': sum(1 for e in dl if e['drv_kind'] == 'brake'), 'drv_d': nmed([e['drv_d'] for e in dl]),
            'drv_tg': nmed([e['drv_tg'] for e in dl]), 'drv_ttc': nmed([e['drv_ttc'] for e in dl if e['drv_ttc'] < 99]),
            'drv_dmin': nmed([e['d_min'] for e in drv]), 'op_dmin': nmed([e['d_min'] for e in op])})
  return s


def print_recommendation(rec, osum, asum, R):
  say('E. RECOMMENDATION')
  t_code = rec['t_code']
  pers = sorted({r['pers'] for r in R})
  if rec['T_src'] is None:
    say('   No steady following under a gas override (nor with openpilot off) at 45 mph or faster with a car ahead in these drives,'
        ' so there is no driver gap to copy yet. Drive a highway stretch holding the gas at the distance you want for 10 s or')
    say('   more behind a car going your speed, several times, then run this again.')
    return
  p = rec[rec['use']]
  T = rec['T_use']
  who = {'ovr': "The driver's steady gap under the gas", 'man': "The driver's steady gap with openpilot off",
         'drv': "The driver's steady gap (gas override and openpilot off pooled: less than %.0f s of steady override)" % DRIVER_OVR_MIN_S}[rec['use']]
  say(f"   {who} ({rec['T_src']}, {p['n']} episodes, {p['s']:.0f} s at 45+): time gap"
      f" {p['tg'][1]:.2f} s [{p['tg'][0]:.2f}-{p['tg'][2]:.2f}], {p['d'][1]:.0f} m at {p['v'] * MPH:.0f} mph.")
  if rec['use'] == 'drv':
    say(f"     (gas override alone: T {rec['ovr']['T'][1]:.2f} s over {rec['ovr']['s']:.0f} s;"
        f" openpilot off alone: T {rec['man']['T'][1]:.2f} s over {rec['man']['s']:.0f} s)")
  say(f"   With the compiled 6 m, the follow time that reproduces it is T = (gap - 6) / speed = {T:.2f} s (median; quartiles"
      f" {p['T'][0]:.2f}-{p['T'][2]:.2f}), against {t_code:.2f} s in the code today ({'/'.join(pers)}).")
  bands = [(band_name(b), q) for b, q in enumerate(rec[rec['use'] + '_bands']) if q]
  if bands:
    say('   Per band: ' + '; '.join(f"{nm} mph T {q['T'][1]:.2f} s (tgap {q['tg'][1]:.2f}, {q['d'][1]:.0f} m, {q['s']:.0f} s)" for nm, q in bands))
  if rec['man'] and rec['ovr']:
    say(f"   With openpilot off the driver kept T {rec['T_man']:.2f} s (tgap {rec['man']['tg'][1]:.2f} s, {rec['man']['s']:.0f} s).")
  if rec['op']:
    say(f"   Check: openpilot's own settled gap with the car ahead holding the speed came out at T {rec['T_op']:.2f} s"
        f" (tgap {rec['op']['tg'][1]:.2f} s, {rec['op']['s']:.0f} s)" + (': the 6 m + T x speed formula matches the logs.'
        if abs(rec['T_op'] - t_code) < 0.12 else f": not the {t_code:.2f} s of the code (set speed, hills, or the lead not steady?)."))
  g, gt = rec['gaps']['rec'], rec['gaps']['today']
  say(f"   T = {T:.2f} s gives a gap of " + ', '.join(f'{g[m]:.0f} m at {m} mph' for m in REC_SPEEDS_MPH)
      + f" (today {', '.join(f'{gt[m]:.0f}' for m in REC_SPEEDS_MPH)} m): time gap {T + STOP_DISTANCE / (65 / MPH):.2f} s at 65,"
      f" {T + STOP_DISTANCE / (90 / MPH):.2f} s at 90 mph.")
  s, st_, si, sit = rec['safety']['rec'], rec['safety']['today'], rec['sim']['rec'], rec['sim']['today']
  say('   Safety margin: the hardest steady braking of the car ahead (to a stop, both from the same speed) that is absorbed without'
      ' touching it, from the gap 6 m + T x speed:')
  say(f"     the physics limit (this car braking at the panda's -{A_EGO_MAX} m/s2 from {REACT_S} s after the car ahead starts):"
      + ', '.join(f' at {m} mph {brk_abs_txt(s[m])} from {g[m]:.0f} m (today {brk_abs_txt(st_[m])} from {gt[m]:.0f} m)' for m in SAFETY_SPEEDS_MPH) + ';')
  say("     what OPENPILOT ALONE manages in the simulator of this branch (camera lead, no driver; it takes 3-4 s to reach full braking):"
      + ', '.join(f' at {m} mph {brk_abs_txt(si[m])} (today {brk_abs_txt(sit[m])})' for m in SAFETY_SPEEDS_MPH) + '.')
  say('     The second line is the one to go by: a normal hard stop is 3-4 m/s2, an emergency stop 6-8. Below about 2 m/s2 the driver'
      ' has to cover a car ahead that brakes firmly for several seconds; today the driver does that under the gas with no help at all.')
  if 'man' in rec['safety'] and rec['ovr'] and rec['use'] != 'man':
    sm, smi = rec['safety']['man'], rec['sim']['man']
    say(f"     (the openpilot-off gap, T = {max(rec['T_man'], 0.0):.2f} s: physics " + ', '.join(f'{brk_abs_txt(sm[m])} at {m} mph' for m in SAFETY_SPEEDS_MPH)
        + '; openpilot alone ' + ', '.join(f'{brk_abs_txt(smi[m])} at {m}' for m in SAFETY_SPEEDS_MPH) + ')')
  if osum and osum['n_valid']:
    w = osum['worst']
    say(f"   The fall-back after the gas comes up ({osum['n_valid']} overrides at 45+ with a car ahead): openpilot asked"
        f" {osum['a_min']:.2f} m/s2 (median; worst {w['a_min']:.2f} in drive {w['rid']} at {w['clock']}), opened the gap by"
        f" {osum['d_grow']:.0f} m and lost {mph(osum['v_drop']):.0f} mph in 5 s, from a release at {osum['tg_up']:.2f} s,"
        f" {-osum['room_up']:.0f} m inside the wanted gap.")
    if osum['n_brk_dn']:
      say(f"   On {osum['n_brk_dn']} of {osum['n']} the gas went down while openpilot was already braking (under -0.3 m/s2).")
    if osum['n_regas']:
      say(f"   On {osum['n_regas']} of {osum['n']} the driver pressed the gas again within 5 s of letting go.")
  if asum and (asum['n_op'] or asum['n_drv']):
    ln = f"   Approaches: {asum['n_op']} in OP"
    if asum['n_brk']:
      ln += (f", openpilot first braked {asum['brk_t']:.1f} s in at {asum['brk_d']:.0f} m / {asum['brk_tg']:.2f} s / TTC {asum['brk_ttc']:.0f} s"
             f" (median), strongest {asum['a_min']:.2f} m/s2")
    if asum['n_gas']:
      ln += (f"; the driver pressed the gas during {asum['n_gas']} of them at {asum['gas_d']:.0f} m / TTC {asum['gas_ttc']:.0f} s"
             f" (openpilot asking {asum['gas_ask']:.2f})")
    ln += f"; {asum['n_drv']} under the driver ({asum['n_ovr']} on the gas over openpilot, {asum['n_man']} with it off)"
    if asum['n_drv_act']:
      ln += (f", who lifted off / braked ({asum['n_lift']}/{asum['n_brake']}) at {asum['drv_d']:.0f} m / {asum['drv_tg']:.2f} s / TTC"
             f" {asum['drv_ttc']:.0f} s (median)")
    say(ln + '.')
    if asum['n_brk'] and asum['n_drv_act'] and isnum(asum['brk_ttc']) and isnum(asum['drv_ttc']):
      if asum['brk_ttc'] > 1.5 * asum['drv_ttc']:
        say('   Openpilot starts braking much earlier in an approach than the driver does: the closing behaviour, not only the'
            ' steady gap, is part of the feel (the v^2/5 terms and the jerk costs, not T).')
      else:
        say('   Openpilot and the driver start braking at similar TTCs when closing: the steady gap is the main difference.')


# ============================================================================================ detail
def print_detail(D_by_route, e):
  """Timeline of one episode."""
  D = D_by_route.get(e['route'])
  if D is None:
    return
  kind = e['kind']
  t_ref = D.G[e['k0']]
  if kind == 'steady':
    head = (f"STEADY FOLLOWING {MODE_SHORT[e['mode']]} for {e['dur']:.0f} s at {e['v_mph']:.0f} mph: gap {e['d']:.1f} m, time gap"
            f" {e['tg']:.2f} s (T {e['T_eq']:.2f} s), the code wants {e['want']:.0f} m; held by {e['limited']}")
    step = 0.5 if e['dur'] > 15 else 0.2
    rows = [(k, step) for k in range(max(e['k0'] - 20, 0), min(e['k1'] + 20, len(D.G)))]
  elif kind == 'override':
    head = (f"GAS OVERRIDE {e['dur']:.0f} s at {mph(e['v_dn']):.0f}-{mph(e['v_up']):.0f} mph: down at {f1(e['d_dn'], 0)} m / {f1(e['tg_dn'], 2)} s"
            f" (openpilot asking {f1(e['ask_dn'], 2)}), settled at {f1(e['d_st'], 0)} m / {f1(e['tg_st'], 2)} s, up at {f1(e['d_up'], 0)} m /"
            f" {f1(e['tg_up'], 2)} s = {f1(e['room_up'], 0)} m from the wanted {f1(e['want_up'], 0)} m; after: {f1(e['a_min'], 2)} m/s2"
            f" at +{f1(e['a_min_t'], 1)} s, gap {f1p(e['d_grow'], 0)} m, {f1(mph(e['v_drop']), 0)} mph lost")
    rows = []
    k_up = e['k1']
    for k in range(max(e['k0'] - 20, 0), min(k_up + 51, len(D.G))):
      rows.append((k, 0.1 if k >= k_up - 20 else 0.5))
  else:
    head = (f"APPROACH in {MODE_SHORT[e['mode']]} from {mph(e['v0']):.0f} mph on a car at {mph(e['vl0']):.0f} mph, {f1(e['d0'], 0)} m, TTC"
            f" {f1(e['ttc0'], 1)} s" + (f"; openpilot braked at +{e['brk_t']:.1f} s ({f1(e['brk_d'], 0)} m, {f1(e['brk_tg'], 2)} s, TTC"
                                        f" {f1(e['brk_ttc'], 1)} s), strongest {f1(e['a_min'], 2)}" if e['brk_k'] is not None else '')
            + (f"; the driver {e['drv_kind']} at +{e['drv_t']:.1f} s ({f1(e['drv_d'], 0)} m, TTC {f1(e['drv_ttc'], 1)} s)" if e['drv_k'] is not None else '')
            + (f"; gas pressed at +{e['gas_t']:.1f} s ({f1(e['gas_d'], 0)} m, TTC {f1(e['gas_ttc'], 1)} s, openpilot asking {f1(e['gas_ask'], 2)})"
               if e['gas_k'] is not None else '') + f"; closest {f1(e['d_min'], 0)} m; {e['end']}")
    rows = [(k, 0.2) for k in range(max(e['k0'] - 20, 0), min(e['k1'] + 30, len(D.G)))]
  say()
  say(f"==== EPISODE {letter(e)}: drive {e['rid']} at {e['clock']}: {head}")
  say('   t = seconds from the start; mode; mph; set; gas/brk = pedals; gap m; tgap s; vRel m/s (minus = closing); lead mph; aLead;'
      ' prob = model lead probability;')
  say("   ask = openpilot's request (carControl, blank when its long control is off); plan = longitudinalPlan.aTarget; want = the"
      " code's wanted gap; room = gap - want; TTC s")
  say('       t  mode  mph  set gas brk    gap  tgap   vRel  lead  aLead  prob     ask   plan   want   room    TTC')
  last_t = -INF
  for k, step in rows:
    t = D.G[k] - t_ref
    if t - last_t < step - 1e-6:
      continue
    last_t = t
    say(f"  {t:6.1f}  {MODE_SHORT[int(D.mode[k])]:<4} {fmt(mph(D.v[k]), 0, 4)} {fmt(D.set[k], 0, 4)}  {'G' if D.gas[k] else '.'}   {'B' if D.brk[k] else '.'} "
        f" {fmt(D.d[k], 1, 6)} {fmt(D.tg[k], 2, 5)} {fmt(D.vrel[k], 1, 6, True)} {fmt(mph(D.vl[k]), 0, 5)} {fmt(D.al[k], 1, 6, True)}"
        f" {fmt(D.prob[k], 2, 5)}  {fmt(D.ask[k], 2, 6)} {fmt(D.aT[k], 2, 6)} {fmt(D.want[k], 0, 6)} {fmt(D.d[k] - D.want[k], 0, 6, True)}"
        f" {ttc_txt(D.ttc[k], 6)}")


# ============================================================================================ paste block
def paste_block(ctx):
  R, steady, ovr, app, rec = ctx['R'], ctx['steady'], ctx['ovr'], ctx['app'], ctx['rec']
  tot, osum, asum = ctx['tot'], ctx['osum'], ctx['asum']
  say()
  say('===== PASTE THIS TO CLAUDE (start) =====')
  a, ld = tot['all'], tot['lead']

  def pct(parts, s):
    return '/'.join(f'{100 * x / s:.0f}' for x in parts) + '%' if s > 0 else '-'
  lines = [f"mk7_follow_check v{TOOL_VERSION.split()[0]} drives={','.join(r['rid'] for r in R)} hwy45+={a['s'] / 60:.1f}min"
           f" lead={ld['s'] / 60:.1f}min OP/OVR/MAN={pct(a['mode'], a['s'])} withlead={pct(ld['mode'], ld['s'])}"
           f" brk={pct([ld['brk']], ld['s'])} steady=S{len(steady)} ovr=O{len(ovr)}(+{ctx['taps']}taps) app=A{len(app)}"
           f" code=6m+{rec['t_code']:.2f}s*v; tg=time-gap s d=gap m [q1-q3] T=(d-6)/v s; m/s2, mph, s"]
  shown_r = R if len(R) <= 5 else R[:3] + R[-2:]
  for r in shown_r:
    m = r['modes']
    lines.append(f"D {r['rid']} {r['branch']}@{r['commit']} segs={r['segs']}/{r['nseg']}" + (f" q={r['q_segs']}" if r['q_segs'] else '')
                 + f" move={m['move_s'] / 60:.0f}min op={m['op_s'] / 60:.0f} hwy={m['all']['s'] / 60:.1f} lead={m['lead']['s'] / 60:.1f}"
                 f" OP/OVR/MAN={pct(m['lead']['mode'], m['lead']['s'])} brk={pct([m['lead']['brk']], m['lead']['s'])} pers={r['pers']}"
                 + (f" delay={r['delay']:.2f}" if isnum(r['delay']) and abs(r['delay'] - 0.5) > 0.01 else '')
                 + (' OPLONG=OFF' if r['op_long'] == 0 else ''))
  if len(R) > len(shown_r):
    lines.append(f'D ... {len(R) - len(shown_r)} more drives')
  bl = []
  for b in range(len(BANDS)):
    for mode, key in ((M_OVR, 'ovr'), (M_MAN, 'man'), (M_OP, 'op_all')):
      p = rec[key + '_bands'][b]
      if p is None:
        continue
      bl.append((-p['s'], f"B {band_name(b)} {MODE_SHORT[mode]} n={p['n']} t={p['s']:.0f}s tg={p['tg'][1]:.2f}[{p['tg'][0]:.2f}-{p['tg'][2]:.2f}]"
                 f" d={p['d'][1]:.0f}[{p['d'][0]:.0f}-{p['d'][2]:.0f}] v={p['v'] * MPH:.0f} T={p['T'][1]:.2f}[{p['T'][0]:.2f}-{p['T'][2]:.2f}]"
                 + (f" setltd={p['n_set']}" if mode == M_OP and p['n_set'] else '') + (f" settling={p['n_post']}" if mode == M_OP and p['n_post'] else '')))
  if asum and (asum['n_op'] or asum['n_drv']):
    al = f"A OP n={asum['n_op']} brk={asum['n_brk']}"
    if asum['n_brk']:
      al += f"@{f1(asum['brk_t'], 1)}s d={f1(asum['brk_d'], 0)} tg={f1(asum['brk_tg'], 2)} ttc={f1(asum['brk_ttc'], 0)} min={f1(asum['a_min'], 2)}"
    if asum['n_gas']:
      al += f" gasin={asum['n_gas']}@d{f1(asum['gas_d'], 0)}/ttc{f1(asum['gas_ttc'], 0)}/ask{f1(asum['gas_ask'], 2)}"
    al += f" dmin={f1(asum['op_dmin'], 0)} | DRV n={asum['n_drv']}(ovr{asum['n_ovr']},man{asum['n_man']}) lift/brk={asum['n_lift']}/{asum['n_brake']}"
    if asum['n_drv_act']:
      al += f"@d={f1(asum['drv_d'], 0)} tg={f1(asum['drv_tg'], 2)} ttc={f1(asum['drv_ttc'], 0)}"
    al += f" dmin={f1(asum['drv_dmin'], 0)}"
  else:
    al = 'A none'
  if rec['T_src'] is None:
    rl = ['REC no steady gas override (or manual) following at 45+ with a car ahead: nothing to copy yet']
  else:
    p = rec[rec['use']]
    g, gt, s, st_ = rec['gaps']['rec'], rec['gaps']['today'], rec['safety']['rec'], rec['safety']['today']
    si, sit = rec['sim']['rec'], rec['sim']['today']
    rl = [f"REC T={rec['T_use']:.2f}s ({rec['T_src']} n={p['n']} {p['s']:.0f}s tg={p['tg'][1]:.2f} d={p['d'][1]:.0f}@{p['v'] * MPH:.0f}mph"
          f" q={p['T'][0]:.2f}-{p['T'][2]:.2f}; MAN T={f1(rec['T_man'], 2)}; OP-settled T={f1(rec['T_op'], 2)} vs code {rec['t_code']:.2f})"
          f" gap@50/65/80/90={'/'.join(f'{g[m]:.0f}' for m in REC_SPEEDS_MPH)}m (today {'/'.join(f'{gt[m]:.0f}' for m in REC_SPEEDS_MPH)})"
          f" physics@65/90={'/'.join(f1(s[m], 1) for m in SAFETY_SPEEDS_MPH)} (today {'/'.join(f1(st_[m], 1) for m in SAFETY_SPEEDS_MPH)})"
          f" sim@65/90={'/'.join(f1(si[m], 2) for m in SAFETY_SPEEDS_MPH)} (today {'/'.join(f1(sit[m], 2) for m in SAFETY_SPEEDS_MPH)})m/s2"]
    if osum and osum['n_valid']:
      rl.append(f"OVR n={osum['n']} dur={osum['dur']:.0f}s dn:tg={f1(osum['tg_dn'], 2)} ask={f1(osum['ask_dn'], 2)} opbrk={osum['n_brk_dn']}"
                f" st:tg={f1(osum['tg_st'], 2)} up:tg={f1(osum['tg_up'], 2)} room={f1(osum['room_up'], 0)} after:min={f1(osum['a_min'], 2)}"
                f" d+{f1(osum['d_grow'], 0)} v-{f1(mph(osum['v_drop']), 0)} regas={osum['n_regas']}")
  notes = [n for _, n in ctx['notes']][:3]
  if len(ctx['notes']) > 3:
    notes.append(f"(+{len(ctx['notes']) - 3} more notes in the full output above)")
  fixed = len(lines) + 1 + len(rl) + 1          # header, D lines, A line, REC lines, the 'more' line
  room = PASTE_MAX - fixed - len(notes)
  if room < 6:
    notes = notes[:max(0, len(notes) - (6 - room))]
    room = PASTE_MAX - fixed - len(notes)
  bl.sort()
  n_b = min(len(bl), max(room - 3, room * 2 // 3))
  lines += [s for _, s in bl[:n_b]]
  room -= n_b
  # override episodes: the worst fall-back first, then the typical one, then the rest by the strongest request
  ol = []
  if ovr:
    order = []
    if osum and osum['worst'] is not None:
      order.append(osum['worst'])
    if osum and osum['typical'] is not None and osum['typical'] is not (osum['worst']):
      order.append(osum['typical'])
    order += sorted([e for e in ovr if e not in order], key=lambda e: (e['a_min'] if isnum(e['a_min']) else 0.0, e['t']))
    for e in order:
      st_mark = '' if e['st_steady'] else '~'
      ol.append(f"{letter(e)} {e['rid']} {e['clock']} v={f1(mph(e['v_dn']), 0)} set={f1(e['set_mph'], 0)}"
                f" dn:d={f1(e['d_dn'], 0)}/{f1(e['tg_dn'], 2)}s vr={f1(e['vrel_dn'], 1)} ask={f1(e['ask_dn'], 2)} held={e['dur']:.0f}s"
                f" st:d={f1(e['d_st'], 0)}{st_mark}/{f1(e['tg_st'], 2)}s up:v={f1(mph(e['v_up']), 0)} d={f1(e['d_up'], 0)}/{f1(e['tg_up'], 2)}s"
                f" want={f1(e['want_up'], 0)} room={f1p(e['room_up'], 0)} after:min={f1(e['a_min'], 2)}@{f1(e['a_min_t'], 1)}"
                f" brk@{f1(e['brk_t'], 1)} d{f1p(e['d_grow'], 0)} v-{f1(mph(e['v_drop']), 0)}"
                + (f" regas@{f1(e['regas_t'], 1)}" if isnum(e['regas_t']) else '') + (' pedal' if e['brk_pedal_after'] or e['brk_in'] else '')
                + (' noOP' if e['after_op_s'] < 1.0 else ''))
  n_o = min(len(ol), max(room, 0), PASTE_OVR_MAX)
  lines += ol[:n_o]
  more = (len(bl) - n_b) + (len(ol) - n_o)
  lines.append(al)
  lines += rl
  if more > 0:
    lines.append(f'... {more} more band / override lines not shown (bands by time, overrides by the strongest fall-back)')
  lines += ['note: ' + n for n in notes]
  for ln in lines[:PASTE_MAX]:
    say(ln)
  say('===== PASTE THIS TO CLAUDE (end) =====')


# ============================================================================================ main
def parse(argv):
  p = argparse.ArgumentParser(description='Read-only: how far behind the car ahead this car drove at 45 mph and faster, by who drove.')
  p.add_argument('drives', nargs='*', default=['0'], help='0 = newest (default), 1 = the one before, 0-4 = a range,'
                 ' or a route-name prefix; several are pooled')
  p.add_argument('--detail', type=int, help='show episode N as a timeline (the number in the tables)')
  p.add_argument('--all', action='store_true', help='show every episode as a timeline')
  p.add_argument('--car-on', action='store_true', help='run even though the car is on (not recommended)')
  p.add_argument('--log-root', default=ROOT, help=argparse.SUPPRESS)
  p.add_argument('--op-dir', default=OP_DIR, help=argparse.SUPPRESS)
  p.add_argument('--onroad-param', default=ONROAD_PARAM, help=argparse.SUPPRESS)
  return p.parse_args(argv)


def collect(argv=None):
  """Read and analyse the chosen drives. Returns a dict with everything the report prints."""
  global np
  args = parse(argv)
  try:
    with open(args.onroad_param, 'rb') as f:
      onroad = f.read().strip() in (b'1', b'\x01')
  except OSError:
    onroad = False
  if onroad and not args.car_on:
    raise UsageError('The car is on. Switch the ignition off first (the comma stays on and you can still SSH in).')
  if onroad:
    try:
      os.nice(10)
    except OSError:
      pass
  rt = setup(args.op_dir)
  np = rt.np
  routes, segs = routes_on_disk(args.log_root)
  if not routes:
    raise UsageError(f'No drives found in {args.log_root}.')
  say(f'mk7_follow_check {TOOL_VERSION} (read-only)  newest {LIST_N} drives (0 = newest):')
  for i, r in enumerate(routes[:LIST_N]):
    b = route_branch(rt, args.log_root, r, segs)
    say(f'  {i} {r} segs {len(segs[r]):3d} {b[0]}@{b[1]}')
  chosen = pick_routes(args.drives, routes)
  R, steady, ovr, app, D_by_route = [], [], [], [], {}
  taps = 0
  keep_drives = bool(args.detail or args.all)
  for route in chosen:
    t_start = time.time()
    svc, info = read_route(rt, args.log_root, route, segs)
    D = build(rt, svc, info)
    del svc
    head = (f"=== drive {route}  {info['branch']}@{info['commit']}  {info['segs']}/{info['nseg']} segments read"
            + (f" ({info['q_segs']} qlog only)" if info['q_segs'] else '')
            + (f", {info['none']} without a log" if info['none'] else '')
            + (f", {info['bad']} damaged messages" if info['bad'] else ''))
    if D is None:
      say(head + ': no carState in the logs, skipped')
      continue
    D.route, D.rid = route, rid(route)
    pts = steady_points(D)
    st_route = find_steady(D, pts)
    ov_route, tp = find_overrides(D, pts)
    ap_route = find_approaches(D)
    taps += tp
    modes = mode_shares(D)
    r = {'rid': D.rid, 'route': route, 'branch': info['branch'], 'commit': info['commit'], 'segs': info['segs'],
         'nseg': info['nseg'], 'q_segs': info['q_segs'], 'modes': modes, 'pers': D.pers, 't_follow': D.t_follow,
         'notes': D.notes, 'delay': info['delay'], 'op_long': info['op_long'], 'n_steady': len(st_route),
         'n_ovr': len(ov_route), 'n_app': len(ap_route)}
    R.append(r)
    steady += st_route
    ovr += ov_route
    app += ap_route
    a, ld = modes['all'], modes['lead']
    say(head + f": {modes['move_s'] / 60:.0f} min moving ({modes['op_s'] / 60:.0f} with openpilot), {a['s'] / 60:.1f} min at 45+ mph"
        f" ({ld['s'] / 60:.1f} with a car ahead), {len(st_route)} steady episodes, {len(ov_route)} gas overrides, {len(ap_route)}"
        f" approaches, personality {D.pers}, read in {time.time() - t_start:.0f} s")
    if keep_drives:
      D_by_route[route] = D
    else:
      del D
  if not R:
    raise UsageError('None of the chosen drives has carState in its logs.')
  events = sorted(steady + ovr + app, key=lambda e: (chosen.index(e['route']), e['t']))
  for i, e in enumerate(events, 1):
    e['n'] = i
  tot = {'move_s': sum(r['modes']['move_s'] for r in R), 'op_s': sum(r['modes']['op_s'] for r in R)}
  for key in ('all', 'lead'):
    tot[key] = {'s': sum(r['modes'][key]['s'] for r in R), 'brk': sum(r['modes'][key]['brk'] for r in R),
                'mode': [sum(r['modes'][key]['mode'][i] for r in R) for i in range(3)]}
  t_codes = sorted({r['t_follow'] for r in R})
  rec = recommend(np, steady, t_codes[0] if len(t_codes) == 1 else T_FOLLOW['aggressive'])
  notes = sorted({n for r in R for n in r['notes']})
  if len(t_codes) > 1:
    notes.insert(0, (0, f"the drives have different personalities ({'/'.join(sorted({r['pers'] for r in R}))}): the code's T is"
                        f" taken as {T_FOLLOW['aggressive']} s (aggressive) in the comparison"))
  return {'args': args, 'R': R, 'steady': steady, 'ovr': ovr, 'app': app, 'events': events, 'rec': rec, 'tot': tot, 'taps': taps,
          'osum': ovr_summary(ovr), 'asum': app_summary(app), 'notes': notes, 'drives': D_by_route}


def report(ctx):
  R, steady, ovr, app, rec, events = ctx['R'], ctx['steady'], ctx['ovr'], ctx['app'], ctx['rec'], ctx['events']
  args = ctx['args']
  for _, n in ctx['notes']:
    say('note: ' + n)
  say()
  print_modes(R, ctx['tot'])
  say()
  if ctx['tot']['lead']['s'] < 1.0:
    say(f"No driving at {HWY_MPH:.0f} mph or faster with a car ahead in these drives"
        + (f" ({ctx['tot']['all']['s'] / 60:.1f} min at 45+ without one)." if ctx['tot']['all']['s'] >= 1.0 else '.'))
    paste_block(ctx)
    return 1
  if steady:
    print_steady(steady, rec)
  else:
    say('B. No steady following episode (5 s or more behind a car at nearly the same speed) at 45+ in these drives.')
  say()
  if ovr:
    print_overrides(ovr, ctx['taps'])
  else:
    say(f"C. No gas override of 1 s or more at 45+ with a car ahead in these drives ({ctx['taps']} shorter taps).")
    say()
  if app:
    print_approaches(app)
  else:
    say('D. No approach (closing by 2 m/s or more on a slower car) at 45+ in these drives.')
    say()
  print_recommendation(rec, ctx['osum'], ctx['asum'], R)
  if args.detail:
    ev = [e for e in events if e['n'] == args.detail]
    if ev:
      print_detail(ctx['drives'], ev[0])
    else:
      say(f'There is no episode {args.detail}; there are {len(events)}.')
  elif args.all:
    for e in events:
      print_detail(ctx['drives'], e)
  paste_block(ctx)
  return 0


def main(argv=None):
  ctx = collect(argv)
  return report(ctx)


np = None

if __name__ == '__main__':
  try:
    sys.exit(main())
  except UsageError as e:
    say(f'ERROR: {e}')
    sys.exit(2)
  except KeyboardInterrupt:
    sys.exit(130)
  except Exception:   # a bug: exit 2 (error), never 1 (= "nothing found")
    import traceback
    sys.stdout.flush()
    traceback.print_exc()
    say(f'ERROR: mk7_follow_check {TOOL_VERSION} failed (traceback above). Paste the last 20 lines to Claude.')
    sys.exit(2)
