#!/usr/bin/env python3
"""
mk7_follow_check.py - read-only: how far behind the car ahead the car drove at highway speed (45-90 mph), and who was
driving at the time: openpilot, the driver on the gas over openpilot ("my override"), or the driver with openpilot off.

INSTALL OR UPDATE (on the comma, over SSH; nothing is replaced if the download fails):
  cd /data/openpilot && git fetch sngrepo t2_set_min && git show FETCH_HEAD:mk7_follow_check.py > /data/mk7_follow_check.py.new && mv /data/mk7_follow_check.py.new /data/mk7_follow_check.py

RUN IT WITH THE CAR OFF:
  python3 /data/mk7_follow_check.py            newest drive
  python3 /data/mk7_follow_check.py 0-4        newest five drives, pooled
  python3 /data/mk7_follow_check.py 0 --detail 3   the timeline of event 3

WHAT IT SHOWS
 A. Every 0.1 s at 45 mph or faster with a car ahead is sorted by who drove: OP = openpilot's long control on and no gas;
    OVR = openpilot engaged but the driver on the gas (the override); MAN = openpilot off.
 B. STEADY FOLLOWING: stretches of 5 s or more behind a car going about the same speed (closing speed under 1.5 m/s): the gap in m
    and in seconds (gap / speed), per speed band (45-55, 55-65, 65-75, 75-90 mph) and per driver, against what the code wants
    today (6 m + 1.25 s x speed with the aggressive personality).
 C. OVERRIDES: every time the gas went down at 45+ mph with a car ahead: the gap then, the closest the driver got, the gap the
    driver settled at, the gap at release, and what openpilot did next (its strongest braking request within 5 s, the gap 10 s
    later, whether the gas went down again).
 D. APPROACHES: closing on a slower car (2 m/s or more): where openpilot started braking (gap, time gap, time to collision) and
    its strongest request; where the driver lifted off or braked when the driver was driving.
 E. The time-gap setting that would make openpilot drive like the driver: T = (gap - 6 m) / speed (6 m is the stop distance built
    into the planner), the gaps that gives at 50 / 65 / 80 / 90 mph, and the braking of the car ahead that can still be absorbed
    with 0.5 s of reaction and openpilot's -3.5 m/s2 limit, for that T and for today's 1.25 s.
Only reads logs. Writes nothing. Exit codes: 0 = report printed, 1 = no highway following found, 2 = problem.
"""
import sys
sys.dont_write_bytecode = True   # never leave .pyc files behind on the device

import argparse  # noqa: E402
import bz2       # noqa: E402
import math      # noqa: E402
import os        # noqa: E402
import re        # noqa: E402
import time      # noqa: E402

TOOL_VERSION = '0.9 (2026-10-19)'
OP_DIR = '/data/openpilot'
ROOT = '/data/media/0/realdata'
VENV_PYTHON = '/usr/local/venv/bin/python3'
REEXEC_FLAG = 'MK7_FOLLOW_CHECK_REEXEC'
ONROAD_PARAM = '/data/params/d/IsOnroad'
ZSTD_MAGIC = b'\x28\xb5\x2f\xfd'
LIST_N = 8
NAN = float('nan')
MPH = 2.23694
PASTE_MAX = 30         # lines between the paste markers

DT = 0.1               # s: analysis grid
HWY_MPH = 45.0         # highway = at or above this
BANDS = [(45, 55), (55, 65), (65, 75), (75, 95)]   # mph (the last one is '75-90')
BAND_TXT = ['45-55', '55-65', '65-75', '75-90']
LEAD_MAX_D = 150.0     # m: a car ahead counts within this
STEADY_VREL = 1.5      # m/s: closing speed under this = going about the same speed ...
STEADY_MIN_S = 5.0     # ... for at least this long = steady following
BRIDGE_S = 0.5         # s: shorter breaks (the camera losing the car for a moment) are bridged
OVR_MIN_S = 1.0        # s: a gas press shorter than this is not an override
TAIL_S = 3.0           # s: the gap the driver 'settled at' = the median over the last 3 s of the override
AFTER_S = 5.0          # s after the release: openpilot's strongest request
AFTER_LONG_S = 10.0    # s after the release: the gap then, and a new gas press within this = 're-pressed'
APPROACH_VREL = -2.0   # m/s: closing faster than this on the car ahead = an approach
APPROACH_MIN_S = 1.0
BRAKE_ON = -0.3        # m/s2: openpilot asking under this = braking
SOLVER_STOP_D = 6.0    # m: stop distance compiled into the planner
T_FOLLOW = {'aggressive': 1.25, 'standard': 1.45, 'relaxed': 1.75}
REACT_S = 0.5          # s: camera + brake delay assumed for the margin numbers
MAX_BRAKE = 3.5        # m/s2: the most braking the panda lets openpilot ask
SHOW_MPH = (50, 65, 80, 90)
MODES = ['OP', 'OVR', 'MAN']
PERS = {'aggressive': 0, 'standard': 1, 'relaxed': 2}
PERS_NAME = {v: k for k, v in PERS.items()}


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


# ============================================================================================ drives and logs
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
    if len(a) > 1 and not np.all(a[1:, 0] >= a[:-1, 0]):
      a = a[np.argsort(a[:, 0], kind='stable')]
    return a


def read_route(rt, root, route, segs):
  """Only the fields needed, all segments of one drive (rlog; qlog where a segment has no rlog). Times in s."""
  np = rt.np
  T = {'cs': Table(6), 'cc': Table(4), 'rs': Table(6), 'lp': Table(2), 'ss': Table(3)}
  cs_add, cc_add, rs_add, lp_add, ss_add = (T[k].rows.append for k in ('cs', 'cc', 'rs', 'lp', 'ss'))
  info = {'route': route, 'branch': '?', 'commit': '?', 'segs': 0, 'q_segs': 0, 'none': 0, 'bad': 0, 'nseg': len(segs[route])}
  cs_n = cc_n = ss_n = 0
  last_gas = last_brk = last_la = last_en = None
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
        if w == 'carState':
          c = e.carState
          cs_n += 1
          gas, brk = c.gasPressed, c.brakePressed
          if q or cs_n % 5 == 0 or gas != last_gas or brk != last_brk:   # 20 per second plus every pedal edge
            cs_add((e.logMonoTime * 1e-9, c.vEgo, c.aEgo, c.vCruise, gas, brk))
          last_gas, last_brk = gas, brk
        elif w == 'carControl':
          c = e.carControl
          cc_n += 1
          la, en = c.longActive, c.enabled
          if q or cc_n % 5 == 0 or la != last_la or en != last_en:
            cc_add((e.logMonoTime * 1e-9, la, c.actuators.accel, en))
          last_la, last_en = la, en
        elif w == 'radarState':
          l1 = e.radarState.leadOne
          rs_add((e.logMonoTime * 1e-9, l1.status, l1.dRel, l1.vRel, l1.vLead, l1.aLeadK))
        elif w == 'longitudinalPlan':
          lp_add((e.logMonoTime * 1e-9, e.longitudinalPlan.aTarget))
        elif w == 'selfdriveState':
          ss_n += 1
          if q or ss_n % 10 == 0:
            s = e.selfdriveState
            ss_add((e.logMonoTime * 1e-9, s.enabled, PERS.get(str(s.personality), -1)))
        elif w == 'initData' and info['branch'] == '?':
          info['branch'], info['commit'] = e.initData.gitBranch, e.initData.gitCommit[:7]
      except Exception:
        info['bad'] += 1
    if got == 0:
      info['bad'] += 1
    for tb in T.values():
      tb.flush(np)
  return {k: tb.array(np) for k, tb in T.items()}, info


# ============================================================================================ signals on a 0.1 s grid
class Drive:
  pass


def hold(np, t_src, v_src, t, stale=0.5, cap=1.2):
  """Last value at or before each t; nan before the first sample, or once it is older than `stale` s (or than 1.5x the
  gap to the next sample, up to `cap` s, so 10-per-second qlog data still holds between its samples)."""
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


def grid_interp(np, t_src, v_src, g, gap=0.3):
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
  """[a, b) index ranges where mask is True."""
  m = np.asarray(mask, dtype=bool)
  if not len(m):
    return []
  d = np.diff(np.concatenate([[0], m.astype(np.int8), [0]]))
  return list(zip(np.flatnonzero(d == 1).tolist(), np.flatnonzero(d == -1).tolist(), strict=True))


def fill_gaps(np, mask, n):
  """mask with False gaps of n samples or fewer (between Trues) filled."""
  m = np.array(mask, dtype=bool)
  for a, b in runs_of(np, ~m):
    if a > 0 and b < len(m) and b - a <= n:
      m[a:b] = True
  return m


def fin(np, x):
  x = np.asarray(x, dtype=np.float64)
  return x[np.isfinite(x)]


def nmed(np, x):
  f = fin(np, x)
  return float(np.median(f)) if len(f) else NAN


def npct(np, x, q):
  f = fin(np, x)
  return float(np.percentile(f, q)) if len(f) else NAN


def nmin(np, x):
  f = fin(np, x)
  return float(f.min()) if len(f) else NAN


def nmax(np, x):
  f = fin(np, x)
  return float(f.max()) if len(f) else NAN


def isnum(x):
  return x is not None and x == x and not (isinstance(x, float) and math.isinf(x))


def wpct(vals, weights, q):
  """Weighted percentile (q in 0..1) of vals, weights = seconds."""
  pairs = sorted((v, w) for v, w in zip(vals, weights, strict=True) if isnum(v) and w > 0)
  if not pairs:
    return NAN
  tot = sum(w for _, w in pairs)
  acc = 0.0
  for v, w in pairs:
    acc += w
    if acc >= q * tot:
      return v
  return pairs[-1][0]


def mph(v):
  return v * MPH if isnum(v) else NAN


def f1(x, nd=1):
  if not isnum(x):
    return '-'
  return f'{round(x, nd) + 0.0:.{nd}f}'


def clock(t):
  return f'{int(t // 60)}:{int(t % 60):02d}'


def band_of(v_ms):
  m = mph(v_ms)
  for i, (lo, hi) in enumerate(BANDS):
    if lo <= m < hi:
      return i
  return None


def build(rt, svc, info):
  np = rt.np
  cs = svc['cs']
  if len(cs) < 50:
    return None
  D = Drive()
  D.np, D.info, D.rid, D.notes = np, info, rid(info['route']), []
  g = np.arange(cs[0, 0], cs[-1, 0], DT)
  D.g, D.n, D.t0 = g, len(g), cs[0, 0]
  D.v = grid_interp(np, cs[:, 0], cs[:, 1], g)
  D.gas = hold(np, cs[:, 0], cs[:, 4], g, stale=0.3) > 0.5
  D.brk = hold(np, cs[:, 0], cs[:, 5], g, stale=0.3) > 0.5
  vc = hold(np, cs[:, 0], cs[:, 3], g, stale=0.3)
  with np.errstate(invalid='ignore'):
    vc[(vc <= 0) | (vc >= 250)] = NAN
  D.vc = vc / 3.6       # carState.vCruise is km/h
  cc = svc['cc']
  if len(cc) >= 20:
    D.long = hold(np, cc[:, 0], cc[:, 1], g, stale=0.5) > 0.5
    D.en = hold(np, cc[:, 0], cc[:, 3], g, stale=0.5) > 0.5
    D.acc = hold(np, cc[:, 0], cc[:, 2], g, stale=0.5)
  else:
    D.long = np.zeros(D.n, dtype=bool)
    D.en = np.zeros(D.n, dtype=bool)
    D.acc = np.full(D.n, NAN)
    D.notes.append(f'drive {D.rid}: carControl is not in the logs: openpilot never counted as driving')
  ss = svc['ss']
  if len(cc) < 20 and len(ss):
    D.en = hold(np, ss[:, 0], ss[:, 1], g, stale=2.0) > 0.5
  rs = svc['rs']
  if len(rs) >= 20:
    st = hold(np, rs[:, 0], rs[:, 1], g, stale=0.3) > 0.5
    D.dRel = np.where(st, hold(np, rs[:, 0], rs[:, 2], g, stale=0.3), NAN)
    D.vRel = np.where(st, hold(np, rs[:, 0], rs[:, 3], g, stale=0.3), NAN)
    D.vLead = np.where(st, hold(np, rs[:, 0], rs[:, 4], g, stale=0.3), NAN)
    D.aLead = np.where(st, hold(np, rs[:, 0], rs[:, 5], g, stale=0.3), NAN)
  else:
    D.dRel = D.vRel = D.vLead = D.aLead = np.full(D.n, NAN)
    D.notes.append(f'drive {D.rid}: radarState is not in the logs: no car ahead can be seen')
  lp = svc['lp']
  D.aT = hold(np, lp[:, 0], lp[:, 1], g, stale=0.5) if len(lp) else np.full(D.n, NAN)
  pers = [int(x) for x in ss[:, 2] if x >= 0] if len(ss) else []
  D.pers = PERS_NAME.get(max(set(pers), key=pers.count), '?') if pers else '?'
  D.t_follow = T_FOLLOW.get(D.pers, 1.25)
  mode = np.full(D.n, -1, dtype=np.int8)
  mode[D.long & ~D.gas] = 0
  mode[D.en & D.gas] = 1
  mode[~D.en] = 2
  D.mode = mode
  with np.errstate(invalid='ignore'):
    D.hw = D.v >= HWY_MPH / MPH
    D.lead = np.isfinite(D.dRel) & (D.dRel > 0) & (D.dRel < LEAD_MAX_D)
    D.tg = np.where(D.v > 1.0, D.dRel / np.where(D.v > 1.0, D.v, 1.0), NAN)
    D.ttc = np.where(D.vRel < -0.1, D.dRel / np.where(D.vRel < -0.1, -D.vRel, 1.0), NAN)
  D.want = SOLVER_STOP_D + D.t_follow * D.v      # what the code wants behind a car at the same speed
  return D


# ============================================================================================ events
def steady_episodes(D):
  np = D.np
  n_bridge = int(round(BRIDGE_S / DT))
  with np.errstate(invalid='ignore'):
    core = D.hw & D.lead & (np.abs(D.vRel) < STEADY_VREL) & (D.mode >= 0)
  core = fill_gaps(np, core, n_bridge) & D.hw & (D.mode >= 0)
  out = []
  for a, b in runs_of(np, core):
    k = a
    while k < b:
      m0 = D.mode[k]
      j = k
      while j < b and D.mode[j] == m0:
        j += 1
      if (j - k) * DT >= STEADY_MIN_S:
        sl = slice(k, j)
        v_med = nmed(np, D.v[sl])
        d_med = nmed(np, D.dRel[sl])
        vc_med = nmed(np, D.vc[sl])
        lim = '?' if not isnum(vc_med) else ('lead' if v_med < vc_med - 1.0 else 'set')
        out.append({'kind': 'S', 'rid': D.rid, 't': float(D.g[k] - D.t0), 'dur': (j - k) * DT, 'mode': int(m0), 'k': (k, j),
                    'v': v_med, 'd': d_med, 'tg': nmed(np, D.tg[sl]), 'tg10': npct(np, D.tg[sl], 10), 'tg90': npct(np, D.tg[sl], 90),
                    'd10': npct(np, D.dRel[sl], 10), 'vc': vc_med, 'lim': lim, 'band': band_of(v_med),
                    'want': SOLVER_STOP_D + D.t_follow * v_med, 'T': (d_med - SOLVER_STOP_D) / v_med if v_med > 1 else NAN,
                    'gas_share': float(D.gas[sl].mean())})
      k = j
  return out


def overrides(D):
  np = D.np
  out = []
  n1, n5, n10, ntail = int(round(1.0 / DT)), int(round(AFTER_S / DT)), int(round(AFTER_LONG_S / DT)), int(round(TAIL_S / DT))
  for a, b in runs_of(np, D.gas & D.en):
    if (b - a) * DT < OVR_MIN_S or not D.hw[a]:
      continue
    head = slice(a, min(a + n1, b))
    if not D.lead[head].any():
      continue
    kd = a + int(np.flatnonzero(D.lead[head])[0])
    tail = slice(max(a, b - ntail), b)
    ku = np.flatnonzero(D.lead[max(a, b - n1):b])
    ku = max(a, b - n1) + int(ku[-1]) if len(ku) else None
    e5, e10 = min(b + n5, D.n), min(b + n10, D.n)
    rep = np.flatnonzero(D.gas[b:e10])
    rep_t = float(rep[0] * DT) if len(rep) else NAN
    op_after = bool(len(D.mode[b:e5])) and float((D.mode[b:e5] == 0).mean()) > 0.5
    out.append({'kind': 'O', 'rid': D.rid, 't': float(D.g[a] - D.t0), 'dur': (b - a) * DT, 'mode': 1, 'k': (a, b),
                'v': nmed(np, D.v[a:b]), 'v_down': float(D.v[a]), 'd_down': float(D.dRel[kd]), 'tg_down': float(D.tg[kd]),
                'tg_min': nmin(np, D.tg[a:b]), 'd_min': nmin(np, D.dRel[a:b]), 'tg_steady': nmed(np, D.tg[tail]),
                'd_steady': nmed(np, D.dRel[tail]), 'v_steady': nmed(np, D.v[tail]),
                'd_up': float(D.dRel[ku]) if ku is not None else NAN, 'tg_up': float(D.tg[ku]) if ku is not None else NAN,
                'min_acc': nmin(np, D.acc[b:e5]), 'min_aT': nmin(np, D.aT[b:e5]),
                'tg5': float(D.tg[min(b + n5, D.n - 1)]), 'tg10': float(D.tg[min(b + n10, D.n - 1)]),
                'd10': float(D.dRel[min(b + n10, D.n - 1)]), 'repress': rep_t, 'op_after': op_after,
                'band': band_of(nmed(np, D.v[tail])), 'brk': bool(D.brk[a:b].any())})
  return out


def approaches(D):
  np = D.np
  out = []
  with np.errstate(invalid='ignore'):
    m = D.hw & D.lead & (D.vRel < APPROACH_VREL) & (D.mode >= 0)
  m = fill_gaps(np, m, int(round(BRIDGE_S / DT))) & D.hw
  for a, b in runs_of(np, m):
    if (b - a) * DT < APPROACH_MIN_S:
      continue
    modes = D.mode[a:b]
    m0 = int(np.bincount(modes[modes >= 0], minlength=3).argmax()) if (modes >= 0).any() else -1
    ev = {'kind': 'A', 'rid': D.rid, 't': float(D.g[a] - D.t0), 'dur': (b - a) * DT, 'mode': m0, 'k': (a, b),
          'v': float(D.v[a]), 'vrel': nmin(np, D.vRel[a:b]), 'd0': float(D.dRel[a]), 'tg0': float(D.tg[a]), 'ttc0': float(D.ttc[a]),
          'd_min': nmin(np, D.dRel[a:b]), 'tg_min': nmin(np, D.tg[a:b]), 'ttc_min': nmin(np, D.ttc[a:b]),
          'min_acc': nmin(np, D.acc[a:b]), 'kb': NAN, 'd_b': NAN, 'tg_b': NAN, 'ttc_b': NAN, 'took': False,
          'kl': NAN, 'd_l': NAN, 'tg_l': NAN, 'ttc_l': NAN, 'band': band_of(float(D.v[a]))}
    if m0 == 0:
      with np.errstate(invalid='ignore'):
        kb = np.flatnonzero((D.acc[a:b] < BRAKE_ON) & (D.mode[a:b] == 0))
      if len(kb):
        k = a + int(kb[0])
        ev.update({'kb': float(D.g[k] - D.g[a]), 'd_b': float(D.dRel[k]), 'tg_b': float(D.tg[k]), 'ttc_b': float(D.ttc[k])})
      ev['took'] = bool((D.gas[a:b] | D.brk[a:b]).any())
    else:
      lift = (~D.gas[a:b]) if m0 == 1 else D.brk[a:b]
      kl = np.flatnonzero(lift)
      if len(kl):
        k = a + int(kl[0])
        ev.update({'kl': float(D.g[k] - D.g[a]), 'd_l': float(D.dRel[k]), 'tg_l': float(D.tg[k]), 'ttc_l': float(D.ttc[k])})
    out.append(ev)
  return out


def drive_shares(D):
  """Seconds at highway speed, with a car ahead, per mode."""
  np = D.np
  hw = float(D.hw.sum()) * DT
  hl = D.hw & D.lead
  per = [float((hl & (D.mode == i)).sum()) * DT for i in range(3)]
  return {'hw': hw, 'hl': float(hl.sum()) * DT, 'per': per, 'move': float((D.v > 0.5).sum()) * DT,
          'vmax': mph(nmax(np, D.v))}


# ============================================================================================ numbers
def band_mode_stats(steady):
  """{(band, mode): n, time, tg median / quartiles, d median, T median} time-weighted over the steady episodes."""
  out = {}
  for bi in range(len(BANDS)):
    for mi in range(3):
      eps = [e for e in steady if e['band'] == bi and e['mode'] == mi]
      w = [e['dur'] for e in eps]
      out[(bi, mi)] = {'n': len(eps), 'time': sum(w),
                       'tg': wpct([e['tg'] for e in eps], w, 0.5), 'tg25': wpct([e['tg'] for e in eps], w, 0.25),
                       'tg75': wpct([e['tg'] for e in eps], w, 0.75), 'd': wpct([e['d'] for e in eps], w, 0.5),
                       'T': wpct([e['T'] for e in eps], w, 0.5), 'v': wpct([e['v'] for e in eps], w, 0.5)}
  return out


def absorb(T, v):
  """The hardest braking of the car ahead (m/s2) after which this car, reacting REACT_S later and braking at MAX_BRAKE, still
  stops where the car ahead stopped, from a gap of 6 m + T x v. inf = any."""
  d = SOLVER_STOP_D + T * v
  s_ego = REACT_S * v + v * v / (2 * MAX_BRAKE)
  if s_ego <= d:
    return float('inf')
  return v * v / (2 * (s_ego - d))


def recommend(steady, R):
  """T from the driver's steady override episodes (and manual ones), time-weighted."""
  rec = {}
  for mi, name in ((1, 'ovr'), (2, 'man'), (0, 'op')):
    eps = [e for e in steady if e['mode'] == mi]
    w = [e['dur'] for e in eps]
    rec[name] = {'n': len(eps), 'time': sum(w), 'T': wpct([e['T'] for e in eps], w, 0.5),
                 'T25': wpct([e['T'] for e in eps], w, 0.25), 'T75': wpct([e['T'] for e in eps], w, 0.75),
                 'tg': wpct([e['tg'] for e in eps], w, 0.5),
                 'bands': {bi: wpct([e['T'] for e in eps if e['band'] == bi], [e['dur'] for e in eps if e['band'] == bi], 0.5)
                           for bi in range(len(BANDS))}}
  t_today = T_FOLLOW.get(R[-1]['pers'], 1.25) if R else 1.25
  rec['today'] = t_today
  return rec


# ============================================================================================ output
def print_drive_head(route, info, D, sh, n_s, n_o, n_a, dt):
  say(f"=== drive {route}  {info['branch']}@{info['commit']}  {info['segs']}/{info['nseg']} segments read"
      + (f" ({info['q_segs']} qlog only)" if info['q_segs'] else '') + (f", {info['none']} without a log" if info['none'] else '')
      + f": {sh['move'] / 60:.0f} min moving, {sh['hw'] / 60:.1f} min at 45+ mph (fastest {sh['vmax']:.0f}),"
      f" {sh['hl'] / 60:.1f} min of it behind a car: OP {sh['per'][0] / 60:.1f} / OVR {sh['per'][1] / 60:.1f} / MAN {sh['per'][2] / 60:.1f} min;"
      f" {n_s} steady stretches, {n_o} overrides, {n_a} approaches; personality {D.pers}; read in {dt:.0f} s")


def print_steady(ev):
  say('STEADY FOLLOWING AT 45+ MPH (5 s or more behind a car going about the same speed; gap = radar/camera distance m, time gap =')
  say('  gap / speed s, 10-90% = its spread in the stretch; want = what the code wants at that speed today (6 m + t_follow x v);')
  say('  limit = what held the speed: the car ahead (lead) or the set speed (set); who = OP openpilot, OVR driver on the gas over')
  say('  openpilot, MAN openpilot off)')
  say('   #  drive     at   who    s   mph  gap m   time gap s  10-90%        want m  T=(gap-6)/v  limit  set')
  for e in ev:
    say(f"  {e['n']:2d} {e['rid']:>5} {clock(e['t']):>6}  {MODES[e['mode']]:<4} {e['dur']:4.0f}  {mph(e['v']):4.0f}  {f1(e['d']):>5}"
        f"     {f1(e['tg'], 2):>5}     {f1(e['tg10'], 2):>5}-{f1(e['tg90'], 2):<5}   {f1(e['want']):>5}    {f1(e['T'], 2):>5}     {e['lim']:<5}"
        f"  {f1(mph(e['vc']), 0)}")


def print_overrides(ev):
  say('OVERRIDES AT 45+ MPH (gas pressed with openpilot engaged, a car ahead; down = gap and time gap when the gas went down;')
  say('  closest = the smallest time gap during it; settled = the median over its last 3 s; up = at release; then = what openpilot')
  say('  asked within 5 s after (strongest, m/s2), the time gap 5 and 10 s later, and a new gas press within 10 s)')
  say('   #  drive     at    s   mph  down m/s    closest s  settled m/s     up m/s      then: min accel  tg+5  tg+10  re-press')
  for e in ev:
    say(f"  {e['n']:2d} {e['rid']:>5} {clock(e['t']):>6} {e['dur']:4.0f}  {mph(e['v']):4.0f}  {f1(e['d_down'], 0):>3}/{f1(e['tg_down'], 2):<5}"
        f"  {f1(e['tg_min'], 2):>6}     {f1(e['d_steady'], 0):>3}/{f1(e['tg_steady'], 2):<5}   {f1(e['d_up'], 0):>3}/{f1(e['tg_up'], 2):<5}"
        f"       {f1(e['min_acc'], 2):>6}      {f1(e['tg5'], 2):>5} {f1(e['tg10'], 2):>5}"
        + (f"  {e['repress']:.1f} s" if isnum(e['repress']) else '      -') + ('  (brake too)' if e['brk'] else '')
        + ('' if e['op_after'] else '  (openpilot not driving after)'))


def print_approaches(ev):
  say('APPROACHES AT 45+ MPH (closing on a slower car by 2 m/s or more; start = gap / time gap / time to collision at the start;')
  say('  OP: when openpilot first asked under -0.3 m/s2 (s after the start, gap, time gap, TTC) and its strongest request; OVR / MAN:')
  say('  when the driver lifted off the gas / pressed the brake; closest = the smallest gap and TTC in the approach)')
  say('   #  drive     at   who    s   mph  closing  start m/s/ttc        brake or lift: +s   gap   tg   ttc   min acc  closest m/ttc')
  for e in ev:
    k, d, tg, ttc = (e['kb'], e['d_b'], e['tg_b'], e['ttc_b']) if e['mode'] == 0 else (e['kl'], e['d_l'], e['tg_l'], e['ttc_l'])
    say(f"  {e['n']:2d} {e['rid']:>5} {clock(e['t']):>6}  {MODES[e['mode']] if e['mode'] >= 0 else '?':<4} {e['dur']:4.0f}  {mph(e['v']):4.0f}"
        f"  {f1(-e['vrel'] * MPH, 0):>4}mph  {f1(e['d0'], 0):>3}/{f1(e['tg0'], 2)}/{f1(e['ttc0'], 0):<4}"
        f"      {f1(k):>5}  {f1(d, 0):>4}  {f1(tg, 2):>4}  {f1(ttc, 0):>4}   {f1(e['min_acc'], 2):>6}    {f1(e['d_min'], 0)}/{f1(e['ttc_min'], 0)}"
        + ('  (driver took over)' if e['took'] else ''))


def print_bands(bm, rec):
  say('PER SPEED BAND AND DRIVER (steady stretches, weighted by their length: n = stretches, time s, time gap median [quartiles],')
  say('  gap median m, T = the time-gap setting that reproduces it; code today = 6 m + 1.25 s x v)')
  say('   band     who   n   time   time gap s           gap m    T s      | code today: gap m  time gap s')
  for bi, txt in enumerate(BAND_TXT):
    vmid = (BANDS[bi][0] + min(BANDS[bi][1], 90)) / 2 / MPH
    want = SOLVER_STOP_D + rec['today'] * vmid
    for mi in range(3):
      s = bm[(bi, mi)]
      if not s['n']:
        continue
      say(f"   {txt:<7}  {MODES[mi]:<4} {s['n']:2d}  {s['time']:5.0f}   {f1(s['tg'], 2):>5} [{f1(s['tg25'], 2)}-{f1(s['tg75'], 2)}]"
          f"   {f1(s['d']):>5}   {f1(s['T'], 2):>5}      | {want:5.1f}  {want / vmid:5.2f}")


def bottom_line(rec, shares, steady, ovr, apps):
  out = []
  hl = sum(s['hl'] for s in shares)
  per = [sum(s['per'][i] for s in shares) for i in range(3)]
  if hl > 0:
    out.append(f"At 45+ mph behind a car ({hl / 60:.1f} min in all): openpilot drove {per[0] / hl * 100:.0f}% of it, the driver on the gas"
               f" over openpilot {per[1] / hl * 100:.0f}%, openpilot off {per[2] / hl * 100:.0f}%.")
  t_today = rec['today']
  for name, label in (('op', 'openpilot'), ('ovr', 'the driver on the gas'), ('man', 'the driver with openpilot off')):
    r = rec[name]
    if r['n']:
      out.append(f"Steady following by {label}: {r['n']} stretches, {r['time'] / 60:.1f} min, time gap {f1(r['tg'], 2)} s median ->"
                 f" T = {f1(r['T'], 2)} s (quartiles {f1(r['T25'], 2)}-{f1(r['T75'], 2)})"
                 + (f"; by band {', '.join(f'{BAND_TXT[b]} {f1(t, 2)}' for b, t in r['bands'].items() if isnum(t))}" if name != 'op' else ''))
  src = 'ovr' if rec['ovr']['n'] else ('man' if rec['man']['n'] else None)
  if src is None:
    out.append('No steady stretch driven by the driver at 45+ mph behind a car: nothing to copy yet (drive some with the gas down).')
    return out
  T = rec[src]['T']
  out.append(f"SETTING THAT COPIES THE DRIVER ({'gas override' if src == 'ovr' else 'openpilot off'}): T = {T:.2f} s (today {t_today:.2f}).")
  gaps = ', '.join(f"{m} mph {SOLVER_STOP_D + T * m / MPH:.0f} m (today {SOLVER_STOP_D + t_today * m / MPH:.0f})" for m in SHOW_MPH)
  out.append(f"  Gap behind a car at the same speed: {gaps}.")
  margins = []
  for m in (65, 90):
    v = m / MPH
    a_new, a_old = absorb(T, v), absorb(t_today, v)
    margins.append(f"{m} mph: {f1(a_new)} m/s2 with T, {f1(a_old)} today")
  out.append(f"  Hardest braking of the car ahead still absorbed (it brakes to a stop; this car reacts after {REACT_S} s and brakes at"
             f" {MAX_BRAKE} m/s2 at most): {'; '.join(margins)}. (A normal hard stop is 4-5 m/s2, a panic stop 7-9.)")
  if ovr:
    acc = [e['min_acc'] for e in ovr if isnum(e['min_acc'])]
    rep = [e for e in ovr if isnum(e['repress'])]
    if acc:
      out.append(f"  After the {len(ovr)} releases openpilot asked {f1(sorted(acc)[len(acc) // 2], 2)} m/s2 median (strongest {f1(min(acc), 2)})"
                 f" within 5 s; the gas went down again within 10 s after {len(rep)} of them.")
  a_op = [e for e in apps if e['mode'] == 0 and isnum(e['tg_b'])]
  if a_op:
    tgs = sorted(e['tg_b'] for e in a_op)
    ttcs = sorted(e['ttc_b'] for e in a_op if isnum(e['ttc_b']))
    out.append(f"  Closing on a slower car, openpilot started braking at a time gap of {f1(tgs[len(tgs) // 2], 2)} s median"
               + (f" (TTC {f1(ttcs[len(ttcs) // 2], 0)} s)" if ttcs else '') + f" on {len(a_op)} approaches.")
  return out


def print_detail(D_cache, e):
  D = D_cache.get(e['rid'])
  if D is None:
    say('(detail needs the drive to be read in this run)')
    return
  a, b = e['k']
  a, b = max(a - int(round(5.0 / DT)), 0), min(b + int(round(10.0 / DT)), D.n)
  say()
  say(f"==== EVENT {e['n']}: {dict(S='STEADY FOLLOWING', O='OVERRIDE', A='APPROACH')[e['kind']]} (drive {e['rid']} at {clock(e['t'])}),"
      f" {MODES[e['mode']] if e['mode'] >= 0 else '?'}, {e['dur']:.0f} s")
  say('      t   mph   who  gap m  tgap s  vRel  vLead  aLead   plan   accel  gas brk  set')
  for k in range(a, b, int(round(0.5 / DT))):
    say(f"  {D.g[k] - D.t0 - e['t']:+6.1f}  {mph(D.v[k]):4.0f}  {MODES[D.mode[k]] if D.mode[k] >= 0 else '?':<4}  {f1(D.dRel[k], 0):>4}"
        f"  {f1(D.tg[k], 2):>5}  {f1(D.vRel[k]):>5}  {f1(mph(D.vLead[k]), 0):>4}  {f1(D.aLead[k]):>5}  {f1(D.aT[k], 2):>5}  {f1(D.acc[k], 2):>5}"
        f"   {'G' if D.gas[k] else '.'}   {'B' if D.brk[k] else '.'}  {f1(mph(D.vc[k]), 0)}")


def paste_block(R, shares, bm, rec, steady, ovr, apps, notes):
  lines = [f"mk7_follow_check v{TOOL_VERSION.split()[0]} drives={','.join(r['rid'] for r in R)} steady={len(steady)} ovr={len(ovr)}"
           f" app={len(apps)}; OP=openpilot OVR=driver gas over openpilot MAN=openpilot off; tg=time gap s d=gap m T=(d-6)/v;"
           f" S/O/A lines: see tables; m, s, mph"]
  for r, s in zip(R, shares, strict=True):
    lines.append(f"D {r['rid']} {r['branch']}@{r['commit']} segs={r['segs']}/{r['nseg']} move={s['move'] / 60:.0f}min hwy={s['hw'] / 60:.1f}min"
                 f" lead={s['hl'] / 60:.1f}min OP/OVR/MAN={s['per'][0] / 60:.1f}/{s['per'][1] / 60:.1f}/{s['per'][2] / 60:.1f}min"
                 f" vmax={s['vmax']:.0f} pers={r['pers']}")
  for bi, txt in enumerate(BAND_TXT):
    parts = []
    for mi in range(3):
      s = bm[(bi, mi)]
      if s['n']:
        parts.append(f"{MODES[mi]} n={s['n']} t={s['time']:.0f}s tg={f1(s['tg'], 2)}[{f1(s['tg25'], 2)}-{f1(s['tg75'], 2)}] d={f1(s['d'], 0)} T={f1(s['T'], 2)}")
    if parts:
      lines.append(f"B {txt} " + ' | '.join(parts))
  room = PASTE_MAX - len(lines) - 4
  o_sel = sorted(ovr, key=lambda e: -e['dur'])[:max(min(room // 2, 8), 0)]
  for e in sorted(o_sel, key=lambda e: e['n']):
    lines.append(f"O{e['n']} {e['rid']} {clock(e['t'])} {e['dur']:.0f}s v={mph(e['v']):.0f} down={f1(e['d_down'], 0)}/{f1(e['tg_down'], 2)}"
                 f" min={f1(e['tg_min'], 2)} settled={f1(e['d_steady'], 0)}/{f1(e['tg_steady'], 2)} up={f1(e['d_up'], 0)}/{f1(e['tg_up'], 2)}"
                 f" then={f1(e['min_acc'], 2)} tg5={f1(e['tg5'], 2)} tg10={f1(e['tg10'], 2)}"
                 + (f" re={e['repress']:.1f}s" if isnum(e['repress']) else '') + ('' if e['op_after'] else ' noOP'))
  room = PASTE_MAX - len(lines) - 3
  s_sel = sorted([e for e in steady if e['mode'] != 0], key=lambda e: -e['dur'])[:max(room, 0)]
  for e in sorted(s_sel, key=lambda e: e['n']):
    lines.append(f"S{e['n']} {e['rid']} {clock(e['t'])} {MODES[e['mode']]} {e['dur']:.0f}s v={mph(e['v']):.0f} d={f1(e['d'], 0)} tg={f1(e['tg'], 2)}"
                 f"[{f1(e['tg10'], 2)}-{f1(e['tg90'], 2)}] T={f1(e['T'], 2)} want={f1(e['want'], 0)} lim={e['lim']}")
  a_op = [e for e in apps if e['mode'] == 0 and isnum(e['tg_b'])]
  a_dr = [e for e in apps if e['mode'] != 0 and isnum(e['tg_l'])]
  lines.append("A OP n=" + str(len(a_op)) + (f" brake@tg={f1(sorted(e['tg_b'] for e in a_op)[len(a_op) // 2], 2)}"
                                             f" ttc={f1(sorted(e['ttc_b'] for e in a_op)[len(a_op) // 2], 0)}"
                                             f" minacc={f1(min(e['min_acc'] for e in a_op), 2)}" if a_op else '')
               + f"; driver n={len(a_dr)}" + (f" lift@tg={f1(sorted(e['tg_l'] for e in a_dr)[len(a_dr) // 2], 2)}"
                                               f" ttc={f1(sorted(e['ttc_l'] for e in a_dr)[len(a_dr) // 2], 0)}" if a_dr else '')
               + f"; all approaches {len(apps)}")
  src = 'ovr' if rec['ovr']['n'] else ('man' if rec['man']['n'] else None)
  if src:
    T = rec[src]['T']
    lines.append(f"R T_{src}={T:.2f}s[{f1(rec[src]['T25'], 2)}-{f1(rec[src]['T75'], 2)}] n={rec[src]['n']} t={rec[src]['time']:.0f}s"
                 f" T_op={f1(rec['op']['T'], 2)} today={rec['today']:.2f} gaps@50/65/80/90={'/'.join(f'{SOLVER_STOP_D + T * m / MPH:.0f}' for m in SHOW_MPH)}m"
                 f" absorb@65={f1(absorb(T, 65 / MPH))}/{f1(absorb(rec['today'], 65 / MPH))} @90={f1(absorb(T, 90 / MPH))}/{f1(absorb(rec['today'], 90 / MPH))}")
  else:
    lines.append('R no driver steady following at 45+ with a car ahead')
  for n in notes[:max(PASTE_MAX - len(lines), 0)]:
    lines.append('N ' + n)
  say()
  say('===== PASTE THIS TO CLAUDE (start) =====')
  for ln in lines[:PASTE_MAX]:
    say(ln)
  say('===== PASTE THIS TO CLAUDE (end) =====')


# ============================================================================================ main
def parse(argv):
  p = argparse.ArgumentParser(description='Read-only: how far behind the car ahead the car drove at 45-90 mph, and who drove.')
  p.add_argument('drives', nargs='*', default=['0'], help='0 = newest (default), 1 = the one before, 0-4 = a range,'
                 ' or a route-name prefix; several are pooled')
  p.add_argument('--detail', type=int, help='show event N in detail (numbers in the tables)')
  p.add_argument('--all', action='store_true', help='show every event in detail')
  p.add_argument('--car-on', action='store_true', help='run even though the car is on (not recommended)')
  p.add_argument('--log-root', default=ROOT, help=argparse.SUPPRESS)
  p.add_argument('--op-dir', default=OP_DIR, help=argparse.SUPPRESS)
  p.add_argument('--onroad-param', default=ONROAD_PARAM, help=argparse.SUPPRESS)
  return p.parse_args(argv)


def main(argv=None):
  args = parse(argv)
  try:
    with open(args.onroad_param, 'rb') as f:
      onroad = f.read().strip() in (b'1', b'\x01')
  except OSError:
    onroad = False
  if onroad and not args.car_on:
    raise UsageError('The car is on. Switch the ignition off first (the comma stays on and you can still SSH in).')
  rt = setup(args.op_dir)
  routes, segs = routes_on_disk(args.log_root)
  if not routes:
    raise UsageError(f'No drives found in {args.log_root}.')
  say(f'mk7_follow_check {TOOL_VERSION} (read-only)  newest {LIST_N} drives (0 = newest):')
  for i, r in enumerate(routes[:LIST_N]):
    b = route_branch(rt, args.log_root, r, segs)
    say(f'  {i} {r} segs {len(segs[r]):3d} {b[0]}@{b[1]}')
  chosen = pick_routes(args.drives, routes)
  R, shares, steady, ovr, apps, notes, D_cache = [], [], [], [], [], [], {}
  for route in chosen:
    t_start = time.time()
    svc, info = read_route(rt, args.log_root, route, segs)
    D = build(rt, svc, info)
    del svc
    if D is None:
      say(f"=== drive {route}  {info['branch']}@{info['commit']}: no carState in the logs, skipped")
      continue
    s_r, o_r, a_r = steady_episodes(D), overrides(D), approaches(D)
    sh = drive_shares(D)
    print_drive_head(route, info, D, sh, len(s_r), len(o_r), len(a_r), time.time() - t_start)
    R.append({'rid': D.rid, 'route': route, 'branch': info['branch'], 'commit': info['commit'], 'segs': info['segs'],
              'nseg': info['nseg'], 'pers': D.pers})
    shares.append(sh)
    steady += s_r
    ovr += o_r
    apps += a_r
    notes += D.notes
    if args.detail or args.all:
      D_cache[D.rid] = D
    else:
      del D
  if not R:
    raise UsageError('None of the chosen drives has carState in its logs.')
  events = sorted(steady + ovr + apps, key=lambda e: (chosen.index(next(r['route'] for r in R if r['rid'] == e['rid'])), e['t']))
  for i, e in enumerate(events, 1):
    e['n'] = i
  for n in notes:
    say('note: ' + n)
  bm = band_mode_stats(steady)
  rec = recommend(steady, R)
  say()
  if not events:
    say('No driving at 45+ mph behind a car in these drives.')
    paste_block(R, shares, bm, rec, steady, ovr, apps, notes)
    return 1
  if steady:
    print_steady(steady)
  else:
    say('No steady following at 45+ mph (5 s or more behind a car at about the same speed).')
  say()
  if ovr:
    print_overrides(ovr)
  else:
    say('No gas override at 45+ mph with a car ahead.')
  say()
  if apps:
    print_approaches(apps)
  else:
    say('No approach to a slower car at 45+ mph.')
  say()
  print_bands(bm, rec)
  say()
  say('BOTTOM LINE')
  for ln in bottom_line(rec, shares, steady, ovr, apps):
    say(ln)
  if args.detail:
    ev = [e for e in events if e['n'] == args.detail]
    if ev:
      print_detail(D_cache, ev[0])
    else:
      say(f'There is no event {args.detail}; there are {len(events)}.')
  elif args.all:
    for e in events:
      print_detail(D_cache, e)
  paste_block(R, shares, bm, rec, steady, ovr, apps, notes)
  return 0


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
