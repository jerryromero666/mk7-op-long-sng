#!/usr/bin/env python3
"""
mk7_brake_timing.py - read-only: how long the car takes to brake after openpilot asks (car OFF).

INSTALL OR UPDATE (on the comma, over SSH):
  cd /data/openpilot && git fetch sngrepo t2_close_follow && git show FETCH_HEAD:mk7_brake_timing.py > /data/mk7_brake_timing.py

  python3 /data/mk7_brake_timing.py               newest drive
  python3 /data/mk7_brake_timing.py 1             drive 1 (0 = newest; the list at the top shows the numbers)
  python3 /data/mk7_brake_timing.py 00000340      a drive can also be a route-name prefix
  python3 /data/mk7_brake_timing.py 0 --event 3   braking event 3 in detail, 10 rows per second
  python3 /data/mk7_brake_timing.py 0 --all       every braking event in detail

For every time openpilot's request drops sharply into braking while driving, it times each step from
openpilot's request to the car actually slowing:
  1. openpilot's request as it went out on the car's CAN bus (ACC_06)
  2. the cruise computer's (TSK) own ramped target (Motor_20 TSK_a_Soll_gradientenbegrenzt)
  3. the TSK asking the ESP for brake torque (TSK_06 TSK_Radbremsmom) and the ESP taking it (ESP_05 bit 27)
  4. the ESP building brake pressure (ESP_05 ESP_Bremsdruck)
  5. the car slowing: the wheel speeds (ESP_19) and the ESP's accelerometer (ESP_02)
  6. openpilot's own filtered acceleration (carState.aEgo), which is what its controller sees
and fits the car model the simulator uses (ramp limit + delay + lag), so the result can go straight
into the simulator's hard-brake tests. Only reads logs. Writes nothing.
"""
import sys
sys.dont_write_bytecode = True   # never leave .pyc files behind on the device

import argparse  # noqa: E402
import bz2       # noqa: E402
import os        # noqa: E402
import re        # noqa: E402
import statistics as st  # noqa: E402

TOOL_VERSION = '1.0 (2026-10-17)'
OP_DIR = '/data/openpilot'
ROOT = '/data/media/0/realdata'
VENV_PYTHON = '/usr/local/venv/bin/python3'
REEXEC_FLAG = 'MK7_BRAKE_TIMING_REEXEC'
ONROAD_PARAM = '/data/params/d/IsOnroad'
DBC_REL = ('opendbc_repo/opendbc/dbc/vw_mqb.dbc', 'opendbc/dbc/vw_mqb.dbc')
ZSTD_MAGIC = b'\x28\xb5\x2f\xfd'
LIST_N = 8
NAN = float('nan')

DT = 0.02              # s: analysis grid (the car's messages come at 50 Hz)
# a braking event: openpilot's request (as sent) falls at least DROP_MIN within DROP_WIN, reaches CMD_MAX
# or lower within CMD_WIN, at V_MIN or faster, with openpilot driving and no pedal in the PRE_S before
DROP_MIN = 0.5         # m/s2
DROP_WIN = 1.0         # s
CMD_MAX = -0.5         # m/s2
CMD_WIN = 2.0          # s
V_MIN = 2.0            # m/s
EVENT_GAP = 2.5        # s: the next event starts at least this long after the last one
PRE_S = 0.4            # s before the start: the "before" level of every signal
WIN_S = 3.0            # s after the start that are analysed (less if the car stops or the driver steps in)
STOP_V = 0.5           # m/s: the analysis ends when the car is this slow
CLEAN_S = 2.0          # an event is clean if no pedal and OP stayed on for this long after the start
SUSTAIN_S = 0.10       # a level counts as reached once it has been held this long
START_CMD = 0.10       # m/s2 below the before level = openpilot's request started to fall (time 0)
ONSET_ACC = 0.20       # m/s2: measured acceleration change that counts as "started to react"
ONSET_TSK = 0.15       # m/s2 (the TSK target comes in 0.1 steps)
ONSET_BRM = 40.0       # Nm brake torque request
ONSET_BAR = 1.0        # bar brake pressure
ONSET_ENG = 15.0       # Nm engine torque
MIN_RESP = 0.25        # m/s2: a smaller change in the car's acceleration is no clear response (no fit)
FIT_MAX_DELAY = 1.5    # s
FIT_TAUS = [0.05 * i for i in range(1, 25)]   # s: first-order lag 0.05 .. 1.2
FIT_GAIN = (0.2, 2.0)
FIT_R2_MIN = 0.5       # fits worse than this are shown but left out of the medians
SIM_PLANT = (0.15, 0.35)   # the simulator's car: delay + lag (s), behind a 4 m/s3 ramp limit
DEFAULT_JERK = 4.0     # m/s3 if ACC_06 carries no ramp limit
MASS_KG, WHEEL_R = 1566.0, 0.328   # only to turn brake torque into an equivalent deceleration

# key, message, signal (car -> comma, read from 'can')
CAR = [
  ('ws_fl', 'ESP_19', 'ESP_VL_Radgeschw_02'),
  ('ws_fr', 'ESP_19', 'ESP_VR_Radgeschw_02'),
  ('ws_rl', 'ESP_19', 'ESP_HL_Radgeschw_02'),
  ('ws_rr', 'ESP_19', 'ESP_HR_Radgeschw_02'),
  ('bar', 'ESP_05', 'ESP_Bremsdruck'),
  ('verz', 'ESP_05', 'ESP_Verz_TSK_aktiv'),
  ('esp_drv_brake', 'ESP_05', 'ESP_Fahrer_bremst'),
  ('brm', 'TSK_06', 'TSK_Radbremsmom'),
  ('tsk_status', 'TSK_06', 'TSK_Status'),
  ('tsk_dev', 'TSK_06', 'TSK_zul_Regelabw'),
  ('tsk_target', 'Motor_20', 'TSK_a_Soll_gradientenbegrenzt'),
  ('eng', 'Motor_11', 'MO_Mom_Ist_Summe'),
  ('a_esp', 'ESP_02', 'ESP_Laengsbeschl'),
  ('grade', 'Motor_16', 'TSK_Steigung'),
]
# key, message, signal (comma -> car: the ACC_06 echo the panda saw go out, or sendcan)
OP = [
  ('cmd', 'ACC_06', 'ACC_Sollbeschleunigung_02'),
  ('jneg', 'ACC_06', 'ACC_neg_Sollbeschl_Grad_02'),
  ('jpos', 'ACC_06', 'ACC_pos_Sollbeschl_Grad_02'),
  ('dev_lo', 'ACC_06', 'ACC_zul_Regelabw_unten'),
]
LCS = {'off': 0, 'pid': 1, 'stopping': 2, 'starting': 3}

# what each timing means: when each part of the chain starts to react, and when it is half-way to the
# asked change. A step marked side=True is shown but the next step's increment is not counted from it.
ONSET_STEPS = [
  ('tsk0', 'TSK target starts to fall', False),
  ('eng0', 'engine torque starts to drop', False),
  ('brm0', 'TSK asks the ESP for brake torque', False),
  ('verz0', 'ESP reports TSK braking active', True),
  ('bar0', 'brake pressure starts to rise', False),
  ('car0', 'car starts to slow (wheels)', False),
  ('aego0', "openpilot's aEgo starts to show it", False),
]
HALF_STEPS = [
  ('req50', "openpilot's request", False),
  ('tsk50', 'TSK target', False),
  ('car50', 'car (wheels)', False),
  ('esp50', 'car (ESP accelerometer)', True),
  ('aego50', "openpilot's aEgo", False),
]
STEPS = [(k, t) for k, t, _ in ONSET_STEPS + HALF_STEPS]


class UsageError(Exception):
  pass


def say(s=''):
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
  DBC = None


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
  try:
    from opendbc.can.dbc import DBC
    rt.DBC = DBC
  except Exception:
    pass
  return rt


# ============================================================================================ DBC
class SigDef:
  def __init__(self, lsb, msb, size, le, signed, factor, offset):
    self.lsb, self.msb, self.size, self.le, self.signed = lsb, msb, size, le, signed
    self.factor, self.offset = factor, offset


BO_RE = re.compile(r'^BO_ (\w+) (\w+) *: (\w+) (\w+)')
SG_RE = re.compile(r'^SG_ (\w+)(?: \w+)? *: (\d+)\|(\d+)@(\d)([+-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\)')
BE_BITS = [j + i * 8 for i in range(64) for j in range(7, -1, -1)]


def load_dbc(rt, op_dir):
  """{message name: (address, {signal name: SigDef})} from vw_mqb.dbc (openpilot's parser if importable)."""
  path = next((os.path.join(op_dir, r) for r in DBC_REL if os.path.isfile(os.path.join(op_dir, r))), None)
  if path is None:
    raise UsageError(f'Could not find vw_mqb.dbc under {op_dir}.')
  msgs = {}
  if rt.DBC is not None:
    try:
      for addr, m in rt.DBC(path).msgs.items():
        msgs[m.name] = (addr, {sn: SigDef(s.lsb, s.msb, s.size, s.is_little_endian, s.is_signed, float(s.factor),
                                          float(s.offset)) for sn, s in m.sigs.items()})
      return msgs, path
    except Exception:
      msgs = {}
  cur = None
  with open(path, encoding='utf-8', errors='replace') as f:
    for line in f:
      line = line.strip()
      if line.startswith('BO_ '):
        m = BO_RE.match(line)
        if m:
          cur = msgs[m.group(2)] = (int(m.group(1), 0), {})
      elif line.startswith('SG_ ') and cur is not None:
        m = SG_RE.match(line)
        if m:
          start, size, le = int(m.group(2)), int(m.group(3)), m.group(4) == '1'
          lsb, msb = (start, start + size - 1) if le else (BE_BITS[BE_BITS.index(start) + size - 1], start)
          cur[1][m.group(1)] = SigDef(lsb, msb, size, le, m.group(5) == '-', float(m.group(6)), float(m.group(7)))
  return msgs, path


def decode(np, words, s):
  """Physical values of one signal from 8-byte little-endian words (vectorised for little-endian signals)."""
  if s.le:
    raw = ((words >> np.uint64(s.lsb)) & np.uint64((1 << s.size) - 1)).astype(np.int64)
  else:
    out = []
    for w in words:
      dat, ret, i, bits = int(w).to_bytes(8, 'little'), 0, s.msb // 8, s.size
      while 0 <= i < 8 and bits > 0:
        lo = s.lsb if (s.lsb // 8) == i else i * 8
        hi = s.msb if (s.msb // 8) == i else (i + 1) * 8 - 1
        n = hi - lo + 1
        ret |= ((dat[i] >> (lo - i * 8)) & ((1 << n) - 1)) << (bits - n)
        bits -= n
        i += 1
      out.append(ret)
    raw = np.array(out, dtype=np.int64)
  if s.signed:
    raw = np.where(raw >= (1 << (s.size - 1)), raw - (1 << s.size), raw)
  return raw, raw.astype(np.float64) * s.factor + s.offset


# ============================================================================================ drives
SEG_RE = re.compile(r'^([0-9a-f]{8}--[0-9a-f]+)--(\d+)$')


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


RLOGS = ('rlog.zst', 'rlog.bz2', 'rlog')
QLOGS = ('qlog.zst', 'qlog.bz2', 'qlog')


def pick_route(arg, routes):
  if re.fullmatch(r'\d{1,3}', arg) and int(arg) < len(routes):
    return routes[int(arg)]
  hits = [r for r in routes if r.startswith(arg)]
  return hits[0] if hits else None


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


def read_route(rt, root, route, segs, want):
  """Everything needed, all segments together. CAN keyed by (stream, address, src)."""
  np = rt.np
  can, cs, cc, lp = {}, [], [], []
  info = {'branch': '?', 'commit': '?', 'delay': NAN, 'segs': 0, 'no_rlog': 0, 'bad': 0}
  from_bytes = int.from_bytes
  for n in sorted(segs[route]):
    p = seg_file(root, route, n, RLOGS)
    if p is None:
      info['no_rlog'] += 1
      continue
    info['segs'] += 1
    got = 0
    for e in events_of(rt, p):
      got += 1
      try:
        w = e.which()
        t = e.logMonoTime
        if w == 'can' or w == 'sendcan':
          for f in (e.can if w == 'can' else e.sendcan):
            a = f.address
            if a in want and len(f.dat) <= 8:
              b = can.get((w, a, f.src))
              if b is None:
                b = can[(w, a, f.src)] = ([], [])
              b[0].append(t)
              b[1].append(from_bytes(f.dat, 'little'))
        elif w == 'carState':
          c = e.carState
          cs.append((t, c.vEgo, c.aEgo, c.brakePressed, c.gasPressed))
        elif w == 'carControl':
          c = e.carControl
          cc.append((t, c.longActive, c.actuators.accel, LCS.get(str(c.actuators.longControlState), 0)))
        elif w == 'longitudinalPlan':
          lp.append((t, e.longitudinalPlan.aTarget))
        elif w == 'carParams' and info['delay'] != info['delay']:
          info['delay'] = float(e.carParams.longitudinalActuatorDelay)
        elif w == 'initData' and info['branch'] == '?':
          info['branch'], info['commit'] = e.initData.gitBranch, e.initData.gitCommit[:7]
      except Exception:
        info['bad'] += 1
    if got == 0:
      info['bad'] += 1
  out = {}
  for k, (ts, vs) in can.items():
    t = np.array(ts, dtype=np.int64)
    v = np.array(vs, dtype=np.uint64)
    o = np.argsort(t, kind='stable')
    out[k] = (t[o], v[o])
  svc = {}
  for name, rows in (('cs', cs), ('cc', cc), ('lp', lp)):
    if rows:
      t = np.array([r[0] for r in rows], dtype=np.int64)
      v = np.array([r[1:] for r in rows], dtype=np.float64)
      o = np.argsort(t, kind='stable')
      svc[name] = (t[o], v[o])
  return out, svc, info


# ============================================================================================ signals
class Drive:
  pass


def build(rt, dbc, can, svc, info):
  np = rt.np
  D = Drive()
  D.np, D.info, D.notes = np, info, []
  if info['segs'] == 0:
    raise UsageError('None of this drive\'s segments has an rlog (only qlogs, which keep about 1 in 2000 CAN messages),'
                     ' so there is nothing to time. Pick another drive.')
  if 'cs' not in svc:
    raise UsageError('This drive has no carState in its rlogs.')
  t0 = svc['cs'][0][0]
  D.t0 = t0
  sec = lambda t: (t - t0).astype(np.float64) * 1e-9   # noqa: E731
  ts, v = svc['cs']
  D.cs_t, D.v, D.aego, D.brk, D.gas = sec(ts), v[:, 0], v[:, 1], v[:, 2] > 0.5, v[:, 3] > 0.5
  if 'cc' in svc:
    ts, v = svc['cc']
    D.cc_t, D.long, D.cc_a = sec(ts), v[:, 0] > 0.5, v[:, 1]
  else:
    D.cc_t, D.long, D.cc_a = np.zeros(0), np.zeros(0, bool), np.zeros(0)
  addr = {name: a for name, (a, _) in dbc.items()}

  # car bus: the lowest bus that carries ESP_19 at a real rate; other car messages from there if they are on it
  e19 = {src: len(x[0]) for (s, a, src), x in can.items() if s == 'can' and a == addr.get('ESP_19') and src < 128}
  pref = min(b for b, c in e19.items() if c >= 0.25 * max(e19.values())) if e19 else 0

  def pick(msg, sources):
    a = addr.get(msg)
    cands = {k: x for k, x in can.items() if k[1] == a and k in sources}
    if not cands:
      return None, None
    mx = max(len(x[0]) for x in cands.values())
    for k in sorted(cands, key=lambda k: (k[2] != pref, k[2])):
      if len(cands[k][0]) >= 0.25 * mx:
        return k, cands[k]
    return None, None

  car_src = {k for k in can if k[0] == 'can' and k[2] < 128}
  D.c, D.src = {}, {}
  for key, msg, sname in CAR:
    k, x = pick(msg, car_src)
    s = dbc.get(msg, (None, {}))[1].get(sname)
    if x is None or s is None:
      D.c[key] = (np.zeros(0), np.zeros(0))
      continue
    D.c[key] = (sec(x[0]), decode(np, x[1], s)[1])
    D.src[msg] = f'bus {k[2]}'
  # what OP sent: the panda's echo of the frames that went out (same clock as the car's frames), else sendcan
  a06 = addr.get('ACC_06')
  snd = {k: x for k, x in can.items() if k[0] == 'sendcan' and k[1] == a06}
  op_bus = max(snd, key=lambda k: len(snd[k][0]))[2] if snd else 0
  echo = can.get(('can', a06, 128 + op_bus))
  n_snd = len(snd[('sendcan', a06, op_bus)][0]) if snd else 0
  if echo is not None and len(echo[0]) >= 0.5 * max(n_snd, 1):
    x, D.src['ACC_06'] = echo, f'panda echo bus {op_bus}'
  elif snd:
    x, D.src['ACC_06'] = snd[('sendcan', a06, op_bus)], f'sendcan bus {op_bus}'
  else:
    x = None
  for key, msg, sname in OP:
    s = dbc.get(msg, (None, {}))[1].get(sname)
    if x is None or s is None:
      D.c[key] = (np.zeros(0), np.zeros(0))
      continue
    raw, val = decode(np, x[1], s)
    if key == 'cmd':
      val[raw >= 2046] = NAN      # neutral / error codes
    D.c[key] = (sec(x[0]), val)

  # openpilot's request while it drives (ACC_06 says 3.01 when it doesn't); carControl if ACC_06 is missing
  t, c = D.c['cmd']
  if len(t):
    act = hold(np, D.cc_t, D.long.astype(float), t) > 0.5 if len(D.cc_t) else np.ones(len(t), bool)
    D.cmd = (t, np.where(act & (c < 3.0), c, NAN))
  else:
    D.cmd = (D.cc_t, np.where(D.long, D.cc_a, NAN))
    D.src['ACC_06'] = 'not found: carControl accel used instead'
  # the TSK target is "not on this car" if it never changes while OP drives
  tt, tv = D.c['tsk_target']
  if len(tt):
    tv = np.where(tv > 5.4, NAN, tv)          # raw 127 = no value
    D.c['tsk_target'] = (tt, tv)
    on = hold(np, D.cmd[0], np.isfinite(D.cmd[1]).astype(float), tt) > 0.5
    if on.sum() < 50 or np.nanstd(tv[on]) < 0.05:
      D.notes.append('Motor_20 TSK target never moved while openpilot drove: left out.')
      D.c['tsk_target'] = (np.zeros(0), np.zeros(0))
  # one wheel speed in m/s on the 50 Hz grid, and its slope = the car's real acceleration along the road
  ws = [D.c[k] for k in ('ws_fl', 'ws_fr', 'ws_rl', 'ws_rr')]
  if all(len(w[0]) for w in ws):
    tw = ws[0][0]
    vw = np.mean([np.interp(tw, w[0], w[1]) for w in ws], axis=0) / 3.6
    D.g_t, D.a_whl, D.v_whl = slope_grid(np, tw, vw)
  else:
    D.notes.append('ESP_19 wheel speeds missing: carState.vEgo used for the car\'s acceleration.')
    D.g_t, D.a_whl, D.v_whl = slope_grid(np, D.cs_t, D.v)
  # the ESP accelerometer on the same grid, lightly smoothed (centered, so no time shift)
  te, ae = D.c['a_esp']
  if len(te) > 100:
    g = grid_interp(np, te, ae, D.g_t)
    D.a_esp = np.convolve(g, np.ones(5) / 5, mode='same')
    ok = np.isfinite(D.a_esp) & np.isfinite(D.a_whl)
    if ok.sum() > 500:
      r = np.corrcoef(D.a_esp[ok], D.a_whl[ok])[0, 1]
      if r < -0.3:
        D.a_esp = -D.a_esp
        D.notes.append('ESP_02 accelerometer has the opposite sign: flipped.')
      elif r < 0.3:
        D.notes.append(f'ESP_02 accelerometer does not follow the wheels (r={r:.2f}): left out.')
        D.a_esp = np.full(len(D.g_t), NAN)
  else:
    D.a_esp = np.full(len(D.g_t), NAN)
  return D


def hold(np, t_src, v_src, t, stale=0.5):
  """Last value at or before each t (nan before the first sample or more than `stale` s after the last)."""
  t = np.asarray(t, dtype=np.float64)
  out = np.full(len(t), NAN)
  if len(t_src) == 0:
    return out
  i = np.searchsorted(t_src, t, side='right') - 1
  ok = i >= 0
  ok[ok] &= (t[ok] - t_src[i[ok]]) <= stale
  out[ok] = v_src[i[ok]]
  return out


def grid_interp(np, t_src, v_src, g, gap=0.1):
  """Linear interpolation onto grid g, nan where the nearest sample is more than `gap` s away."""
  if len(t_src) < 2:
    return np.full(len(g), NAN)
  out = np.interp(g, t_src, v_src)
  j = np.clip(np.searchsorted(t_src, g), 1, len(t_src) - 1)
  near = np.minimum(np.abs(g - t_src[j - 1]), np.abs(t_src[j] - g))
  out[near > gap] = NAN
  return out


def slope_grid(np, t, v, half=5):
  """Speed on a uniform DT grid and its least-squares slope over +-half samples (+-0.1 s, centered)."""
  g = np.arange(t[0], t[-1], DT)
  vg = grid_interp(np, t, v, g)
  k = np.arange(-half, half + 1, dtype=np.float64)
  a = np.correlate(vg, k / (np.sum(k * k) * DT), mode='same')
  a[:half] = NAN
  a[-half:] = NAN
  return g, a, vg


# ============================================================================================ events
def find_events(D):
  np = D.np
  t_c, c = D.cmd
  D.skipped = {'driver on a pedal': 0, 'TSK not regulating': 0, 'slower than 2 m/s': 0, 'OP just engaged': 0}
  if len(t_c) < 100:
    return []
  G = np.arange(t_c[0], t_c[-1], DT)
  g = hold(np, t_c, c, G, stale=0.1)
  W = int(round(DROP_WIN / DT))
  pad = np.concatenate([g, np.full(W, NAN)])
  win = np.lib.stride_tricks.sliding_window_view(pad, W + 1)[:len(g)]
  fmin = np.where(np.isnan(win), np.inf, win).min(axis=1)
  cand = np.isfinite(g) & (g - fmin >= DROP_MIN)
  pedal_t = D.cs_t[D.brk | D.gas]
  st_t, st_v = D.c['tsk_status']
  events, i = [], 0
  while i < len(G):
    if not cand[i]:
      i += 1
      continue
    below = np.where(g[i:] < g[i] - START_CMD)[0]
    if not len(below):
      break
    k = i + below[0]
    t_on = G[k]
    i = k + int(round(EVENT_GAP / DT))
    pre = (G >= t_on - PRE_S) & (G <= t_on)
    nxt = (G >= t_on) & (G <= t_on + CMD_WIN)
    if not np.isfinite(g[(G >= t_on - 0.5) & (G <= t_on)]).all():
      D.skipped['OP just engaged'] += 1
      continue
    c0 = float(np.median(g[pre]))
    cmin = float(np.nanmin(g[nxt]))
    if cmin > CMD_MAX or c0 - cmin < DROP_MIN:
      continue
    v_on = float(np.interp(t_on, D.cs_t, D.v))
    if v_on < V_MIN:
      D.skipped['slower than 2 m/s'] += 1
      continue
    if np.any((pedal_t >= t_on - 0.5) & (pedal_t <= t_on)):
      D.skipped['driver on a pedal'] += 1
      continue
    if len(st_t) and hold(np, st_t, st_v, [t_on])[0] != 3:
      D.skipped['TSK not regulating'] += 1
      continue
    # where the analysis ends and why
    ends = [(t_on + WIN_S, '')]
    slow = D.cs_t[(D.cs_t > t_on) & (D.v < STOP_V)]
    if len(slow):
      ends.append((slow[0], 'stopped'))
    ped = D.cs_t[(D.cs_t > t_on) & D.brk]
    if len(ped):
      ends.append((ped[0], 'BRAKE'))
    ped = D.cs_t[(D.cs_t > t_on) & D.gas]
    if len(ped):
      ends.append((ped[0], 'GAS'))
    off = G[(G > t_on) & ~np.isfinite(g)]
    if len(off):
      ends.append((off[0], 'OP off'))
    t_end, why = min(ends)
    clean = why in ('', 'stopped') or t_end >= t_on + CLEAN_S
    events.append(dict(t_on=t_on, c0=c0, cmin=cmin, v=v_on, t_end=t_end, why=why, clean=clean))
  return events


def first_cross(np, E, x, level, a, b, below=True):
  """First time in [a, b] from which x stays at/below (or above) level for SUSTAIN_S; nan if never."""
  ns = int(round(SUSTAIN_S / DT))
  ok = np.isfinite(x) & ((x <= level) if below else (x >= level))
  for j in np.where((E >= a) & (E <= b))[0]:
    if j + ns < len(x) and ok[j:j + ns + 1].all():
      return float(E[j])
  return NAN


def steepest_fall(np, E, x, a, b, span=0.2):
  """Steepest fall of x over `span` s inside [a, b], in units/s (positive = falling)."""
  n = int(round(span / DT))
  m = np.where((E >= a) & (E <= b))[0]
  if len(m) <= n:
    return NAN
  seg = x[m]
  d = (seg[:-n] - seg[n:]) / span
  return float(np.nanmax(d)) if np.isfinite(d).any() else NAN


def rate_limit(np, u, jneg, jpos):
  r = np.empty_like(u)
  r[0] = u[0]
  dn, up = jneg * DT, jpos * DT
  for i in range(1, len(u)):
    r[i] = r[i - 1] + min(max(u[i] - r[i - 1], -dn), up)
  return r


def fit_plant(np, E, u, y, a, b, jneg=None, jpos=None):
  """Fit y ~ k * lag(delay(ramp_limit(u))). Returns (delay, lag, gain, r2) or None. Same model as the simulator."""
  u = u.copy()
  for i in range(1, len(u)):      # carry the last request over short gaps
    if not np.isfinite(u[i]):
      u[i] = u[i - 1]
  if not np.isfinite(u).all():
    return None
  if jneg is not None:
    u = rate_limit(np, u, jneg, jpos)
  m = (E >= a) & (E <= b) & np.isfinite(y)
  if m.sum() < 25:
    return None
  ym_ = y[m]
  sst = float(np.sum((ym_ - ym_.mean()) ** 2))
  nd = int(round(FIT_MAX_DELAY / DT)) + 1
  n = len(u)
  best = None
  for tau in FIT_TAUS:
    al = 1.0 - np.exp(-DT / tau)
    L = np.empty(n)
    L[0] = u[0]
    for i in range(1, n):
      L[i] = L[i - 1] + al * (u[i] - L[i - 1])
    S = np.empty((nd, n))
    for d in range(nd):
      S[d, :d] = L[0]
      S[d, d:] = L[:n - d]
    Sm = S[:, m]
    k = np.clip((Sm @ ym_) / np.maximum(np.sum(Sm * Sm, axis=1), 1e-9), *FIT_GAIN)
    err = np.sum((ym_ - k[:, None] * Sm) ** 2, axis=1)
    d = int(np.argmin(err))
    if best is None or err[d] < best[0]:
      best = (float(err[d]), d * DT, tau, float(k[d]))
  sse, d, tau, k = best
  return d, tau, k, (1.0 - sse / sst) if sst > 1e-9 else NAN


def analyse(D, ev):
  """All the timings of one event, in s from the moment openpilot's request started to fall."""
  np = D.np
  t_on, t_end = ev['t_on'], ev['t_end']
  E = t_on + np.arange(-0.6, WIN_S + 0.6 + 1e-9, DT)
  ev['E'] = E
  S = {
    'cmd': hold(np, *D.cmd, E, stale=0.1),
    'tsk': hold(np, *D.c['tsk_target'], E),
    'brm': hold(np, *D.c['brm'], E),
    'bar': hold(np, *D.c['bar'], E),
    'verz': hold(np, *D.c['verz'], E),
    'eng': hold(np, *D.c['eng'], E),
    'whl': grid_interp(np, D.g_t, D.a_whl, E) if len(D.g_t) else np.full(len(E), NAN),
    'esp': grid_interp(np, D.g_t, D.a_esp, E) if len(D.g_t) else np.full(len(E), NAN),
    'aego': np.interp(E, D.cs_t, D.aego),
    'v': np.interp(E, D.cs_t, D.v),
    'brk': hold(np, D.cs_t, D.brk.astype(float), E),
    'gas': hold(np, D.cs_t, D.gas.astype(float), E),
    'jneg': hold(np, *D.c['jneg'], E),
    'jpos': hold(np, *D.c['jpos'], E),
    'grade': hold(np, *D.c['grade'], E, stale=2.0),
  }
  ev['S'] = S
  pre = (E >= t_on - PRE_S) & (E <= t_on)
  x0 = {k: (float(np.nanmedian(x[pre])) if np.isfinite(x[pre]).any() else NAN) for k, x in S.items()}
  ev['x0'] = x0
  half = ev['c0'] - (ev['c0'] - ev['cmin']) / 2.0
  dlt = ev['c0'] - ev['cmin']
  a, b = t_on - 0.2, t_end
  T = {}
  T['req0'] = first_cross(np, E, S['cmd'], ev['c0'] - START_CMD, t_on - 0.1, b)
  T['req50'] = first_cross(np, E, S['cmd'], half, a, b)
  T['tsk0'] = first_cross(np, E, S['tsk'], x0['tsk'] - ONSET_TSK, a, b)
  T['tsk50'] = first_cross(np, E, S['tsk'], x0['tsk'] - dlt / 2.0, a, b)
  T['eng0'] = first_cross(np, E, S['eng'], x0['eng'] - ONSET_ENG, a, b)
  T['brm0'] = first_cross(np, E, S['brm'], x0['brm'] + ONSET_BRM, a, b, below=False)
  T['verz0'] = first_cross(np, E, S['verz'], 0.5, a, b, below=False) if x0['verz'] < 0.5 else NAN
  T['bar0'] = first_cross(np, E, S['bar'], x0['bar'] + ONSET_BAR, a, b, below=False)
  T['car0'] = first_cross(np, E, S['whl'], x0['whl'] - ONSET_ACC, a, b)
  T['car50'] = first_cross(np, E, S['whl'], x0['whl'] - dlt / 2.0, a, b)
  T['esp50'] = first_cross(np, E, S['esp'], x0['esp'] - dlt / 2.0, a, b)
  T['aego0'] = first_cross(np, E, S['aego'], x0['aego'] - ONSET_ACC, a, b)
  T['aego50'] = first_cross(np, E, S['aego'], x0['aego'] - dlt / 2.0, a, b)
  ev['T'] = {k: v - t_on for k, v in T.items()}
  w = (E >= t_on) & (E <= t_end)
  got = x0['whl'] - float(np.nanmin(S['whl'][w])) if np.isfinite(S['whl'][w]).any() else NAN
  ev['got'] = max(got, 0.0) / dlt if dlt > 0 else NAN
  ev['brm_max'] = float(np.nanmax(S['brm'][w])) if np.isfinite(S['brm'][w]).any() else NAN
  ev['bar_max'] = float(np.nanmax(S['bar'][w])) if np.isfinite(S['bar'][w]).any() else NAN
  ev['j_cmd'] = steepest_fall(np, E, S['cmd'], t_on - 0.1, t_end)
  ev['j_tsk'] = steepest_fall(np, E, S['tsk'], t_on - 0.1, t_end)
  ev['j_car'] = steepest_fall(np, E, S['whl'], t_on - 0.1, t_end)
  jneg = float(np.nanmedian(S['jneg'][w])) if np.isfinite(S['jneg'][w]).any() else NAN
  jpos = float(np.nanmedian(S['jpos'][w])) if np.isfinite(S['jpos'][w]).any() else NAN
  ev['jneg'] = jneg
  jneg = jneg if np.isfinite(jneg) and jneg > 0 else DEFAULT_JERK
  jpos = jpos if np.isfinite(jpos) and jpos > 0 else DEFAULT_JERK
  ev['fit'] = ev['fit_tsk'] = None
  if np.isfinite(got) and got >= MIN_RESP:
    y = S['whl'] - x0['whl']
    ev['fit'] = fit_plant(np, E, S['cmd'] - ev['c0'], y, t_on, t_end, jneg, jpos)
    if np.isfinite(S['tsk'][w]).sum() > 25:
      ev['fit_tsk'] = fit_plant(np, E, S['tsk'] - x0['tsk'], y, t_on, t_end)
  ev['grade'] = x0['grade']
  ev['kind'] = 'from gas' if ev['c0'] > 0.05 else 'slowing'
  return ev


# ============================================================================================ output
def fmt(x, nd=2, w=5, plus=False):
  if x is None or x != x:
    return ' ' * (w - 1) + '-'
  return f'{x:+{w}.{nd}f}' if plus else f'{x:{w}.{nd}f}'


def fit_txt(f, w=True):
  if f is None:
    return '      -       ' if w else '-'
  d, tau, k, r2 = f
  return f'{d:.2f}+{tau:.2f}={d + tau:.2f} x{k:.2f}' + ('' if r2 >= FIT_R2_MIN else '?')


def clock(t):
  return f'{int(t // 60)}:{int(t % 60):02d}'


def med(vals):
  vals = [v for v in vals if v is not None and v == v]
  return (st.median(vals), len(vals)) if vals else (NAN, 0)


def medians(evs):
  out = {k: med([e['T'][k] for e in evs]) for k, _ in STEPS}
  out['lag50'] = med([e['T']['car50'] - e['T']['req50'] for e in evs])
  out['got'] = med([e['got'] for e in evs])
  for key in ('fit', 'fit_tsk'):
    good = [e[key] for e in evs if e[key] is not None and e[key][3] >= FIT_R2_MIN]
    out[key] = (med([f[0] for f in good])[0], med([f[1] for f in good])[0], med([f[2] for f in good])[0],
                len(good)) if good else None
  out['j_cmd'] = med([e['j_cmd'] for e in evs])
  out['jneg'] = med([e['jneg'] for e in evs])
  out['j_tsk'] = med([e['j_tsk'] for e in evs])
  return out


def print_events(D, evs):
  say(f'BRAKING EVENTS: openpilot\'s request fell >= {DROP_MIN} m/s2 within {DROP_WIN:g} s into braking (<= {CMD_MAX}),'
      f' at >= {V_MIN * 2.237:.1f} mph, with openpilot')
  say('  driving. Times in s from the moment the request started to fall ("-" = not in the window).')
  say('  req50 = request half-way down   TSK0/TSK50 = cruise computer target starts / half-way   brk = TSK asks for brakes')
  say('  bar = brake pressure starts   car0/car50 = car starts to slow / slows half as much as asked   aEgo50 = openpilot sees it')
  say('  got = share of the asked change the car delivered   fit = delay + lag = total, x gain ("?" = poor fit)')
  say('   #    at  mph  request        req50  TSK0 TSK50   brk   bar  car0 car50 aEgo50  got  fit (ramp+delay+lag)  notes')
  for i, e in enumerate(evs, 1):
    T = e['T']
    notes = []
    if e['why'] and e['why'] != 'stopped':
      notes.append(f"{e['why']} at {e['t_end'] - e['t_on']:+.2f}")
    elif e['why'] == 'stopped':
      notes.append(f"stopped at {e['t_end'] - e['t_on']:+.2f}")
    if e['grade'] == e['grade'] and abs(e['grade']) >= 3:
      notes.append(f"grade {e['grade']:+.0f}%")
    if not e['clean']:
      notes.append('not in medians')
    say(f"  {i:2d} {clock(e['t_on']):>5} {e['v'] * 2.237:4.0f}  {e['c0']:+.2f}>{e['cmin']:+.2f}  "
        f"{fmt(T['req50'])} {fmt(T['tsk0'])} {fmt(T['tsk50'])} {fmt(T['brm0'])} {fmt(T['bar0'])} {fmt(T['car0'])}"
        f" {fmt(T['car50'])} {fmt(T['aego50'], w=6)} {fmt(e['got'] * 100, 0, 4)}%  {fit_txt(e['fit']):<20} "
        + ', '.join(notes))


def print_medians(D, groups):
  say('MEDIANS (clean events only; n = events where that step happened):')
  say('  group            n   req50  TSK0 TSK50   eng   brk   bar  car0 car50 aEgo50 | car50-req50  got'
      '  fit cmd->car               fit TSK->car')
  for name, evs in groups:
    if not evs:
      continue
    M = medians(evs)
    f, ft = M['fit'], M['fit_tsk']
    ftxt = f'{f[0]:.2f}+{f[1]:.2f}={f[0] + f[1]:.2f} x{f[2]:.2f} (n{f[3]})' if f else '-'
    fttxt = f'{ft[0]:.2f}+{ft[1]:.2f}={ft[0] + ft[1]:.2f} x{ft[2]:.2f} (n{ft[3]})' if ft else '-'
    g = lambda k: fmt(M[k][0])   # noqa: E731
    say(f'  {name:<15} {len(evs):2d}   {g("req50")} {g("tsk0")} {g("tsk50")} {g("eng0")} {g("brm0")}'
        f' {g("bar0")} {g("car0")} {g("car50")} {fmt(M["aego50"][0], w=6)} |'
        f'       {fmt(M["lag50"][0])} {fmt(M["got"][0] * 100, 0, 4)}%  {ftxt:<26} {fttxt}')


def print_story(D, evs):
  M = medians(evs)
  say(f'WHERE THE TIME GOES (medians of {len(evs)} clean events; 0.00 = openpilot\'s request starts to fall;')
  say('  (+x) = time added since the line above, n = events where it happened)')
  big = []
  for title, steps in (('When each part STARTS to react:', ONSET_STEPS),
                       ('When each part is HALF-WAY to the change openpilot asked for:', HALF_STEPS)):
    say('  ' + title)
    prev = (0.0, "openpilot's request starts to fall") if steps is ONSET_STEPS else None
    for key, text, side in steps:
      v, n = M[key]
      if n == 0:
        say(f'    {text + " ":.<40} never seen')
        continue
      inc = ''
      if prev is not None and not side:
        inc = f'(+{max(v - prev[0], 0):.2f})'
        big.append((v - prev[0], prev[1], text))
      say(f'    {text + " ":.<40} {v:+.2f} s  {inc:<8} n={n}')
      if not side:
        prev = (v, text)
  if big:
    d, a, b = max(big)
    say(f'  -> biggest single step: "{a}" to "{b}", {d:.2f} s.')
  lag, _ = M['lag50']
  if lag == lag:
    say(f'  -> at the half-way mark the car follows openpilot\'s request {lag:.2f} s late.')
  f = M['fit']
  if f:
    d, tau, k, n = f
    say(f'  -> fitted like the simulator (request ramp-limited, then delay + lag): delay {d:.2f} s + lag {tau:.2f} s ='
        f' {d + tau:.2f} s, and the car delivers {k * 100:.0f}% of the asked change (n={n}).')
    say(f'     The simulator assumed {SIM_PLANT[0]:.2f} + {SIM_PLANT[1]:.2f} = {sum(SIM_PLANT):.2f} s.')
  ft = M['fit_tsk']
  if ft and f:
    say(f'  -> from the TSK\'s own ramped target to the car: {ft[0]:.2f} + {ft[1]:.2f} = {ft[0] + ft[1]:.2f} s (brakes and car'
        f' only), so about {max(f[0] + f[1] - ft[0] - ft[1], 0):.2f} s is the TSK smoothing the request.')
  jc, jt = M['j_cmd'][0], M['j_tsk'][0]
  if jc == jc:
    say(f'  -> openpilot\'s request fell at up to {jc:.1f} m/s3 (median of the steepest 0.2 s);'
        + (f' the TSK target at up to {jt:.1f} m/s3.' if jt == jt else ' (no TSK target signal).'))
  jn = M['jneg'][0]
  if jn == jn:
    say(f'  -> openpilot told the TSK to ramp braking at {jn:.1f} m/s3 (ACC_06 ACC_neg_Sollbeschl_Grad_02).')
  dl = D.info['delay']
  if dl == dl:
    say(f'  -> openpilot plans as if the car responds {dl:.2f} s after it asks (CarParams.longitudinalActuatorDelay).')


def print_detail(D, i, e):
  np = D.np
  S, E, t_on = e['S'], e['E'], e['t_on']
  say()
  say(f"==== EVENT {i} at {clock(t_on)}: {e['v'] * 2.237:.0f} mph, request {e['c0']:+.2f} -> {e['cmin']:+.2f} m/s2"
      f" ({e['kind']}), analysed to {e['t_end'] - t_on:+.2f} s {e['why']}")
  T = e['T']
  for title, steps in (('starts to react:', ONSET_STEPS), ('half-way to the asked change:', HALF_STEPS)):
    say('  ' + title)
    for key, text, _ in steps:
      say(f'    {text + " ":.<40} {fmt(T[key], plus=True)} s')
  say(f"  car delivered {fmt(e['got'] * 100, 0, 3)}% of the asked change; brake torque up to {fmt(e['brm_max'], 0, 4)} Nm"
      f" (about {fmt(e['brm_max'] / (MASS_KG * WHEEL_R), 1, 3)} m/s2), pressure up to {fmt(e['bar_max'], 1, 4)} bar")
  say(f"  steepest 0.2 s fall: request {fmt(e['j_cmd'], 1, 4)} m/s3, TSK target {fmt(e['j_tsk'], 1, 4)} m/s3,"
      f" car {fmt(e['j_car'], 1, 4)} m/s3; ramp limit sent (ACC_06) {fmt(e['jneg'], 1, 3)} m/s3")
  say(f"  fit request->car: {fit_txt(e['fit'], False)}   fit TSK target->car: {fit_txt(e['fit_tsk'], False)}"
      + (f"   (r2 {e['fit'][3]:.2f})" if e['fit'] else ''))
  say('      t    mph   req   TSK   brkNm  bar verz  engNm  a_car a_ESP  aEgo  B G')
  for t in np.arange(-0.5, e['t_end'] - t_on + 0.31, 0.1):
    j = int(round((t + 0.6) / DT))
    if j >= len(E):
      break
    mark = ''
    for key in ('req50', 'tsk50', 'brm0', 'bar0', 'car0', 'car50', 'aego50'):
      if T[key] == T[key] and abs(T[key] - t) < 0.05 + 1e-9:
        mark += f' <{key}'
    say(f"  {round(t, 1) + 0.0:+5.1f} {S['v'][j] * 2.237:5.1f} {fmt(S['cmd'][j], plus=True)} {fmt(S['tsk'][j], 1, plus=True)}"
        f" {fmt(S['brm'][j], 0, 7)} {fmt(S['bar'][j], 1)} {fmt(S['verz'][j], 0, 4)} {fmt(S['eng'][j], 0, 6)}"
        f"  {fmt(S['whl'][j], plus=True)} {fmt(S['esp'][j], plus=True)} {fmt(S['aego'][j], plus=True)}"
        f"  {'B' if S['brk'][j] > 0.5 else '.'} {'G' if S['gas'][j] > 0.5 else '.'}{mark}")
  say('  (a_car = slope of the wheel speeds, centered over +-0.1 s so it is not delayed; a_ESP = ESP accelerometer;'
      ' aEgo = openpilot\'s own filtered value)')


def paste_block(D, route, evs, clean):
  info = D.info
  say()
  say('===== PASTE THIS TO CLAUDE (start) =====')
  say(f"mk7_brake_timing v{TOOL_VERSION.split()[0]} route={route} branch={info['branch']} commit={info['commit']}"
      f" segs={info['segs']} op_delay={fmt(info['delay'], 2, 1)}")
  say('src ' + ' '.join(f'{m}={s.replace(" ", "")}' for m, s in sorted(D.src.items())))
  late = sum(1 for e in evs if e['why'] in ('BRAKE', 'GAS', 'OP off') and not e['T']['car50'] == e['T']['car50'])
  say(f"events={len(evs)} clean={len(clean)} taken_over_before_car50={late} skipped=" + ','.join(f"{k.replace(' ', '_')}:{v}" for k, v in D.skipped.items() if v))
  for name, grp in (('all', clean), ('from_gas', [e for e in clean if e['kind'] == 'from gas']),
                    ('slowing', [e for e in clean if e['kind'] == 'slowing'])):
    if not grp:
      continue
    M = medians(grp)
    f, ft = M['fit'], M['fit_tsk']
    say(f"MED {name} n={len(grp)} " + ' '.join(f'{k}={fmt(M[k][0], 2, 1)}' for k, _ in STEPS)
        + f" lag50={fmt(M['lag50'][0], 2, 1)} got={fmt(M['got'][0], 2, 1)}"
        + (f" fit={f[0]:.2f}+{f[1]:.2f}x{f[2]:.2f}/n{f[3]}" if f else ' fit=-')
        + (f" fitTSK={ft[0]:.2f}+{ft[1]:.2f}x{ft[2]:.2f}/n{ft[3]}" if ft else ' fitTSK=-')
        + f" jcmd={fmt(M['j_cmd'][0], 1, 1)} jtsk={fmt(M['j_tsk'][0], 1, 1)}")
  for i, e in enumerate(evs[:15], 1):
    T = e['T']
    f = e['fit']
    say(f"E{i} {clock(e['t_on'])} v={e['v']:.1f} c={e['c0']:+.2f}>{e['cmin']:+.2f} "
        + ' '.join(f'{k}={fmt(T[k], 2, 1)}' for k in ('req50', 'tsk50', 'brm0', 'bar0', 'car0', 'car50', 'aego50'))
        + f" got={fmt(e['got'], 2, 1)} brm={fmt(e['brm_max'], 0, 1)} bar={fmt(e['bar_max'], 1, 1)}"
        + (f" fit={f[0]:.2f}+{f[1]:.2f}x{f[2]:.2f}r{f[3]:.2f}" if f else ' fit=-')
        + f" jn={fmt(e['jneg'], 1, 1)} g={fmt(e['grade'], 0, 1)}"
        + (f" end={e['why']}{e['t_end'] - e['t_on']:+.2f}" if e['why'] else '') + ('' if e['clean'] else ' X'))
  if len(evs) > 15:
    say(f'... {len(evs) - 15} more events not shown')
  for n in D.notes:
    say('note: ' + n)
  say('===== PASTE THIS TO CLAUDE (end) =====')


# ============================================================================================ main
def main():
  p = argparse.ArgumentParser(description='Read-only: times every step from openpilot\'s brake request to the car slowing.')
  p.add_argument('drive', nargs='?', default='0', help='0 = newest drive (default), 1 = the one before, or a name prefix')
  p.add_argument('--event', type=int, help='show braking event N in detail')
  p.add_argument('--all', action='store_true', help='show every braking event in detail')
  p.add_argument('--car-on', action='store_true', help='run even though the car is on (not recommended)')
  p.add_argument('--log-root', default=ROOT, help=argparse.SUPPRESS)
  p.add_argument('--op-dir', default=OP_DIR, help=argparse.SUPPRESS)
  args = p.parse_args()
  try:
    with open(ONROAD_PARAM, 'rb') as f:
      onroad = f.read().strip() in (b'1', b'\x01')
  except OSError:
    onroad = False
  if onroad and not args.car_on:
    raise UsageError('The car is on. Switch the ignition off first (the comma stays on and you can still SSH in).')
  rt = setup(args.op_dir)
  np = rt.np
  dbc, _ = load_dbc(rt, args.op_dir)
  want = {dbc[m][0] for m in {m for _, m, _ in CAR + OP} if m in dbc}
  routes, segs = routes_on_disk(args.log_root)
  if not routes:
    raise UsageError(f'No drives found in {args.log_root}.')
  say(f'mk7_brake_timing {TOOL_VERSION} (read-only)  newest {LIST_N} drives (0 = newest):')
  for i, r in enumerate(routes[:LIST_N]):
    b = route_branch(rt, args.log_root, r, segs)
    say(f'  {i} {r} segs {len(segs[r]):3d} {b[0]}@{b[1]}')
  route = pick_route(args.drive, routes)
  if route is None:
    raise UsageError(f'No drive matches {args.drive!r}.')
  can, svc, info = read_route(rt, args.log_root, route, segs, want)
  D = build(rt, dbc, can, svc, info)
  del can
  say(f"=== drive {route}  branch {info['branch']}@{info['commit']}  {info['segs']} segments read"
      + (f", {info['no_rlog']} without rlog (skipped)" if info['no_rlog'] else '')
      + (f", {info['bad']} damaged messages" if info['bad'] else ''))
  say('signals: ' + ', '.join(f'{m} {s}' for m, s in sorted(D.src.items())))
  missing = sorted({m for k, m, _ in CAR if not len(D.c[k][0])})
  if missing:
    say('not found on this car / in this drive: ' + ', '.join(missing))
  for n in D.notes:
    say('note: ' + n)
  evs = [analyse(D, e) for e in find_events(D)]
  clean = [e for e in evs if e['clean']]
  sk = ', '.join(f'{v} {k}' for k, v in D.skipped.items() if v)
  say(f'{len(evs)} braking events, {len(clean)} clean' + (f' (left out before the start: {sk})' if sk else ''))
  late = [e for e in evs if e['why'] in ('BRAKE', 'GAS', 'OP off') and not e['T']['car50'] == e['T']['car50']]
  if late:
    say(f'{len(late)} of them ended with the driver braking or OP switching off before the car had slowed half as much as'
        ' asked (listed, but not in the medians).')
  if not evs:
    say('Nothing to time: openpilot never dropped its request sharply into braking while driving in this drive.')
    paste_block(D, route, evs, clean)
    return 1
  say()
  print_events(D, evs)
  say()
  print_medians(D, [('all clean', clean), ('from gas', [e for e in clean if e['kind'] == 'from gas']),
                    ('already slowing', [e for e in clean if e['kind'] == 'slowing']),
                    ('under 15 mph', [e for e in clean if e['v'] < 6.7]),
                    ('15 mph and up', [e for e in clean if e['v'] >= 6.7])])
  say()
  if clean:
    print_story(D, clean)
  else:
    say('No clean event (the driver stepped in or OP switched off within 2 s every time); see the table above.')
  if args.event:
    if 1 <= args.event <= len(evs):
      print_detail(D, args.event, evs[args.event - 1])
    else:
      say(f'There is no event {args.event}; there are {len(evs)}.')
  elif args.all:
    for i, e in enumerate(evs, 1):
      print_detail(D, i, e)
  else:
    def slowness(e):   # the car never got half-way = slowest; then the longest lag behind the request
      lag = e['T']['car50'] - e['T']['req50']
      return lag if lag == lag else 9.0
    worst = max(clean or evs, key=slowness)
    say()
    say('The slowest event in detail (another with --event N, all with --all):')
    print_detail(D, evs.index(worst) + 1, worst)
  paste_block(D, route, evs, clean)
  return 0


if __name__ == '__main__':
  try:
    sys.exit(main())
  except UsageError as e:
    say(f'ERROR: {e}')
    sys.exit(2)
  except KeyboardInterrupt:
    sys.exit(130)
