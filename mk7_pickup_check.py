#!/usr/bin/env python3
"""
mk7_pickup_check.py - read-only: what held the acceleration back on every pickup from 5 mph (car OFF).

INSTALL OR UPDATE (on the comma, over SSH):
  cd /data/openpilot && git fetch sngrepo t2_close_follow && git show FETCH_HEAD:mk7_pickup_check.py > /data/mk7_pickup_check.py

RUN IT WITH THE CAR OFF:

  python3 /data/mk7_pickup_check.py                 newest drive
  python3 /data/mk7_pickup_check.py 0 1 2 3 4       the 5 newest drives together (0 = newest)
  python3 /data/mk7_pickup_check.py 0-4             the same
  python3 /data/mk7_pickup_check.py 00000340        a drive can also be a route-name prefix
  python3 /data/mk7_pickup_check.py 0-4 --detail 3  pickup 3 in detail, 4 rows per second
  python3 /data/mk7_pickup_check.py 0-4 --all       every pickup in detail

A pickup starts when the car passes 5 mph (2.2 m/s) going up, with openpilot driving and no pedal. It
ends at 25 mph (11.2 m/s), at a set speed (or Speed Limit Assist limit) under 25 mph, when the driver
presses gas or brake, when openpilot switches off, when the car slows under 4 mph (1.8 m/s) again, or
after 25 s. It counts as 'lead' when radarState.leadOne was there (closer than 60 m) 20% of the time.
For every planner frame (20 per second) it works out what held openpilot's planned accel back:
  CAP  the planner's accel cap (pickup cap from /data/pickup_accel, default 1.8; older branches: stock)
  THR  the driving model's gas prediction said "no throttle" (longitudinalPlan.allowThrottle off)
  TRN  the turn cap (steering wheel angle)
  SLW  the cap coming back up after THR / TRN, at 1 m/s2 per second
  SET  the set speed is close, so the MPC eases off
  LEAD a car ahead is close or slower, so the MPC follows it
  SP   a sunnypilot speed target is lower (Smart Cruise Control vision / map; Speed Limit Assist when close)
  E2E  Experimental mode's model accel is lower
  RMP  the MPC is still ramping its accel up (its jerk costs)
  MPC  the MPC planned less than the cap for another reason
  BST  launch boost asked for more than the MPC (only behind a car)
and compares the car with the plan: what went out in ACC_06, the cruise computer's (TSK) own target
(Motor_20), TSK_06 TSK_Status (5 = brake only: no engine torque), the engine torque asked and delivered
(Motor_11), the gear lever and the DSG's target gear (Getriebe_11), the wheel-speed slope and openpilot's
aEgo. It also shows the set speed at the start: 40 km/h right after switching on = SET pressed under
25 mph, and the MPC then eases off from about 14 mph. Only reads logs. Writes nothing.
"""
import sys
sys.dont_write_bytecode = True   # never leave .pyc files behind on the device

import argparse  # noqa: E402
import bz2       # noqa: E402
import math      # noqa: E402
import os        # noqa: E402
import re        # noqa: E402
import statistics as st  # noqa: E402
import time      # noqa: E402

TOOL_VERSION = '1.1 (2026-10-17)'
OP_DIR = '/data/openpilot'
ROOT = '/data/media/0/realdata'
VENV_PYTHON = '/usr/local/venv/bin/python3'
REEXEC_FLAG = 'MK7_PICKUP_CHECK_REEXEC'
ONROAD_PARAM = '/data/params/d/IsOnroad'
PICKUP_FILE = '/data/pickup_accel'
DBC_REL = ('opendbc_repo/opendbc/dbc/vw_mqb.dbc', 'opendbc/dbc/vw_mqb.dbc')
ZSTD_MAGIC = b'\x28\xb5\x2f\xfd'
LIST_N = 8
NAN = float('nan')
MPH = 2.23694

DT = 0.02              # s: grid for the car's 50 Hz messages
DT_PLAN = 0.05         # s: planner rate
# a pickup
V_START = 2.2          # m/s (5 mph): starts when the car passes this going up
V_END = 11.2           # m/s (25 mph): done
V_SLOW = 1.8           # m/s: slowed down again
V_20 = 8.94            # m/s (20 mph): 5->20 mph is timed too (a 25 mph set speed never gets to 25)
V_CRUISE_INITIAL = 40.0   # km/h: what SET gives under 25 mph (selfdrive/car/cruise.py)
MAX_S = 25.0           # s
MIN_S = 1.0            # s: shorter pickups are left out
SET_MARGIN = 0.3       # m/s: "reached the set speed" when this close to a set speed under 25 mph
LEAD_D = 60.0          # m: a lead closer than this counts
LEAD_SHARE = 0.2       # a pickup is "lead" if a lead was there this share of the time
# the classification
CAP_TOL = 0.05         # m/s2: plan within this of the cap = held by the cap
CLIP_TOL = 0.02        # m/s2: plan this far under the MPC's own wish = clipped by the planner
COMP_TOL = 0.02        # m/s2: which part of the cap was lowest
SET_NEAR = 6.5         # m/s: the MPC starts to ease off about 5 m/s under the set speed (sim)
RAMP_RATE = 0.25       # m/s3: plan still rising this fast = the MPC is ramping up
CLOSE_GAP = 2.0        # m: lead within this of the MPC's wanted gap = close
CAR_LAG = 0.3          # s: the car's response compared with the request this much later
GATE = (0.2, 14.5)     # m/s: CAN is only kept while the car is in this speed range (memory, time)
# the planner (selfdrive/controls/lib/longitudinal_planner.py, pickup_accel.py)
T_IDXS = [10.0 * (i / 32.0) ** 2 for i in range(17)]   # ModelConstants.T_IDXS[:CONTROL_N]
STOCK_BP, STOCK_V = [0., 10., 25., 40.], [1.6, 1.2, 0.8, 0.6]
CAP17_BP, CAP17_V = [0., 6., 10., 25., 40.], [1.7, 1.7, 1.2, 0.8, 0.6]   # t2_accel_cap_launch
PICKUP_BP = [0., 11.2, 20.]
PICKUP_DEFAULT, PICKUP_LIMIT = 1.8, 1.95
TURN_BP, TURN_V = [20., 40.], [1.7, 3.2]
ACCEL_MIN, ACCEL_MAX = -3.5, 2.5
SLEW = 0.05            # per planner update on the cap
PANDA_MAX = 2.0
STEER_RATIO, WHEELBASE = 15.6, 2.62
T_FOLLOW = {'aggressive': 1.25, 'standard': 1.45, 'relaxed': 1.75}
BRANCH_CAP = {'t2_close_follow': 'pickup', 't2_hill_announce': 'pickup', 't2_accel_cap_launch': 'cap17',
              't2_launch_kick': 'stock', 't2_launch_boost': 'stock'}
CAP_NAMES = {'pickup': 'pickup cap', 'stock': 'stock cap', 'cap17': '1.7 cap (t2_accel_cap_launch)'}
SRC = {'cruise': 0, 'lead0': 1, 'lead1': 2, 'lead2': 3, 'e2e': 4}
SPSRC = {'cruise': 0, 'sccVision': 1, 'sccMap': 2, 'speedLimitAssist': 3}
PERS = {'aggressive': 0, 'standard': 1, 'relaxed': 2}
PERS_NAME = {v: k for k, v in PERS.items()}
GEAR = {5: 'P', 6: 'R', 7: 'N', 8: 'D', 9: 'S', 10: 'E'}

CODES = ['CAP', 'THR', 'TRN', 'SLW', 'SET', 'LEAD', 'SP', 'E2E', 'RMP', 'MPC', 'BST', '?']
C = {k: i for i, k in enumerate(CODES)}
CODE_TEXT = {
  'CAP': 'the accel cap', 'THR': 'the throttle gate (model said no gas)', 'TRN': 'the turn cap',
  'SLW': 'the cap coming back up', 'SET': 'the set speed being close', 'LEAD': 'the car ahead',
  'SP': 'a sunnypilot speed target', 'E2E': 'Experimental mode', 'RMP': 'the MPC still ramping up',
  'MPC': 'the MPC planning less on its own', 'BST': 'launch boost (more, not less)', '?': 'no plan data',
}

# key, message, signal (car -> comma, read from 'can')
CAR = [
  ('ws_fl', 'ESP_19', 'ESP_VL_Radgeschw_02'),
  ('ws_fr', 'ESP_19', 'ESP_VR_Radgeschw_02'),
  ('ws_rl', 'ESP_19', 'ESP_HL_Radgeschw_02'),
  ('ws_rr', 'ESP_19', 'ESP_HR_Radgeschw_02'),
  ('tsk_target', 'Motor_20', 'TSK_a_Soll_gradientenbegrenzt'),
  ('tsk_status', 'TSK_06', 'TSK_Status'),
  ('eng_req', 'Motor_11', 'MO_Mom_Soll_Roh'),
  ('eng', 'Motor_11', 'MO_Mom_Ist_Summe'),
  ('rpm', 'Motor_12', 'MO_Drehzahl_01'),
  ('lever', 'Gateway_73', 'GE_Fahrstufe'),
  ('lever_tx', 'Getriebe_11', 'GE_Fahrstufe'),
  ('gear', 'Getriebe_11', 'GE_Zielgang'),
  ('grade', 'Motor_16', 'TSK_Steigung'),
]
RAW_KEYS = {'lever', 'lever_tx', 'gear', 'tsk_status'}
# physical ranges outside which a value is an error / "no value" code (raw all-ones etc.), not a reading
VALID = {'ws_fl': (0.0, 400.0), 'ws_fr': (0.0, 400.0), 'ws_rl': (0.0, 400.0), 'ws_rr': (0.0, 400.0),
         'eng_req': (-400.0, 500.0), 'eng': (-400.0, 500.0), 'rpm': (0.0, 9000.0), 'grade': (-40.0, 40.0)}
# key, message, signal (comma -> car: the ACC_06 echo the panda saw go out, or sendcan)
OP = [
  ('cmd', 'ACC_06', 'ACC_Sollbeschleunigung_02'),
  ('dyn', 'ACC_06', 'ACC_DynErhoehung'),
]


# car messages that may not be on this car's bus: what is lost without them
MISSING_TXT = {'Motor_20': 'no TSK target', 'TSK_06': 'no TSK_Status', 'Motor_11': 'no engine torque asked / delivered',
               'Getriebe_11': 'no DSG target gear', 'Gateway_73': 'no gear lever from Gateway_73'}


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


class CanBuf:
  __slots__ = ('t', 'd', 'ct', 'cd')

  def __init__(self):
    self.t, self.d, self.ct, self.cd = [], [], [], []

  def flush(self, np):
    if self.t:
      self.ct.append(np.array(self.t, dtype=np.float64))
      self.cd.append(np.array(self.d, dtype=np.uint64))
      self.t.clear()
      self.d.clear()


def interp_index(action_t):
  k = max(i for i in range(len(T_IDXS) - 1) if T_IDXS[i] <= action_t)
  return k, (action_t - T_IDXS[k]) / (T_IDXS[k + 1] - T_IDXS[k])


def read_route(rt, root, route, segs, want):
  """Only the fields needed, all segments of one drive. Times in s (logMonoTime). CAN keyed by (stream, address, src)."""
  np = rt.np
  T = {'cs': Table(8), 'cc': Table(4), 'lp': Table(8), 'rs': Table(7), 'sp': Table(4), 'lpar': Table(2), 'ss': Table(4)}
  cs_add, cc_add, lp_add = T['cs'].rows.append, T['cc'].rows.append, T['lp'].rows.append
  rs_add, sp_add, lpar_add, ss_add = T['rs'].rows.append, T['sp'].rows.append, T['lpar'].rows.append, T['ss'].rows.append
  can = {}
  info = {'route': route, 'branch': '?', 'commit': '?', 'delay': NAN, 'steer_ratio': NAN, 'wheelbase': NAN, 'segs': 0,
          'no_rlog': 0, 'bad': 0, 'nseg': len(segs[route])}
  from_bytes = int.from_bytes
  action_t = 0.55
  ki, kf = interp_index(action_t)
  gate_on = False
  ss_n = cs_n = cc_n = 0
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
        if w == 'can' or w == 'sendcan':
          if not gate_on:
            continue
          t = e.logMonoTime * 1e-9
          for f in (e.can if w == 'can' else e.sendcan):
            a = f.address
            if a in want:
              dat = f.dat
              if len(dat) <= 8:
                k = (w, a, f.src)
                b = can.get(k)
                if b is None:
                  b = can[k] = CanBuf()
                b.t.append(t)
                b.d.append(from_bytes(dat, 'little'))
        elif w == 'carState':
          c = e.carState
          v = c.vEgo
          gate_on = GATE[0] <= v <= GATE[1]
          cs_n += 1
          if cs_n % 2 and not (c.gasPressed or c.brakePressed):   # 50 Hz is plenty; every pedal frame is kept
            continue
          cs_add((e.logMonoTime * 1e-9, v, c.aEgo, c.vCruise, c.steeringAngleDeg, c.gasPressed, c.brakePressed,
                  c.standstill))
        elif w == 'carControl':
          cc_n += 1
          if cc_n % 2:      # 50 Hz
            continue
          c = e.carControl
          o = c.orientationNED
          cc_add((e.logMonoTime * 1e-9, c.longActive, c.actuators.accel, o[1] if len(o) == 3 else NAN))
        elif w == 'longitudinalPlan':
          lp = e.longitudinalPlan
          sv, av = lp.speeds, lp.accels
          if len(sv) == len(T_IDXS) and len(av) == len(T_IDXS):
            v0 = sv[0]
            vt = sv[ki] + kf * (sv[ki + 1] - sv[ki])
            a_unc = 2.0 * (vt - v0) / action_t - av[0]      # drive_helpers.get_accel_from_plan, before the clip
            a_pk = max(list(av)[:12])                       # the MPC's plan peak over the next ~1.4 s
          else:
            a_unc = a_pk = NAN
          lp_add((e.logMonoTime * 1e-9, lp.aTarget, a_unc, a_pk, lp.allowThrottle, lp.hasLead,
                  SRC.get(str(lp.longitudinalPlanSource), -1), lp.shouldStop))
        elif w == 'radarState':
          r = e.radarState
          l1, l2 = r.leadOne, r.leadTwo
          rs_add((e.logMonoTime * 1e-9, l1.status, l1.dRel, l1.vLead, l1.aLeadK, l2.status, l2.dRel))
        elif w == 'longitudinalPlanSP':
          s = e.longitudinalPlanSP
          sp_add((e.logMonoTime * 1e-9, SPSRC.get(str(s.longitudinalPlanSource), -1), s.vTarget, s.aTarget))
        elif w == 'liveParameters':
          lpar_add((e.logMonoTime * 1e-9, e.liveParameters.angleOffsetDeg))
        elif w == 'selfdriveState':
          ss_n += 1
          if ss_n % 10 == 0:
            s = e.selfdriveState
            ss_add((e.logMonoTime * 1e-9, s.enabled, s.experimentalMode, PERS.get(str(s.personality), -1)))
        elif w == 'carParams' and info['delay'] != info['delay']:
          cp = e.carParams
          info['delay'] = float(cp.longitudinalActuatorDelay)
          info['steer_ratio'], info['wheelbase'] = float(cp.steerRatio), float(cp.wheelbase)
          if 0.0 < info['delay'] < 2.0:
            action_t = info['delay'] + DT_PLAN
            ki, kf = interp_index(action_t)
        elif w == 'initData' and info['branch'] == '?':
          info['branch'], info['commit'] = e.initData.gitBranch, e.initData.gitCommit[:7]
      except Exception:
        info['bad'] += 1
    if got == 0:
      info['bad'] += 1
    for tb in T.values():
      tb.flush(np)
    for b in can.values():
      b.flush(np)
  out = {}
  for k in list(can):
    b = can.pop(k)
    if b.ct:
      t = np.concatenate(b.ct)
      d = np.concatenate(b.cd)
      b.ct, b.cd = [], []
      if len(t) > 1 and not np.all(t[1:] >= t[:-1]):
        o = np.argsort(t, kind='stable')
        t, d = t[o], d[o]
      out[k] = (t, d)
  svc = {k: tb.array(np) for k, tb in T.items()}
  return out, svc, info


# ============================================================================================ signals
class Drive:
  pass


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
  if len(t) < 2 * half + 2:
    return np.zeros(0), np.zeros(0), np.zeros(0)
  g = np.arange(t[0], t[-1], DT)
  vg = grid_interp(np, t, v, g)
  k = np.arange(-half, half + 1, dtype=np.float64)
  a = np.correlate(vg, k / (np.sum(k * k) * DT), mode='same')
  a[:half] = NAN
  a[-half:] = NAN
  return g, a, vg


def build(rt, dbc, can, svc, info):
  np = rt.np
  D = Drive()
  D.np, D.info, D.notes = np, info, []      # notes: (priority 0 = must reach the paste block .. 1, text)
  dr = rid(info['route'])
  cs = svc['cs']
  if len(cs) < 100:
    return None
  D.t0 = cs[0, 0]
  D.cs_t, D.v, D.aego, D.vcruise, D.steer = cs[:, 0], cs[:, 1], cs[:, 2], cs[:, 3], cs[:, 4]
  D.gas, D.brk = cs[:, 5] > 0.5, cs[:, 6] > 0.5
  cc = svc['cc']
  D.cc_t, D.long, D.cc_a, D.pitch = cc[:, 0], cc[:, 1] > 0.5, cc[:, 2], cc[:, 3]
  lp = svc['lp']
  D.lp_t, D.aT, D.a_unc, D.a_pk = lp[:, 0], lp[:, 1], lp[:, 2], lp[:, 3]
  D.allow, D.src = lp[:, 4] > 0.5, lp[:, 6]
  rs = svc['rs']
  D.rs_t, D.l1, D.dRel, D.vLead = rs[:, 0], rs[:, 1] > 0.5, rs[:, 2], rs[:, 3]
  sp = svc['sp']
  D.sp_t, D.sp_src, D.sp_v = sp[:, 0], sp[:, 1], sp[:, 2]
  D.off_t, D.off = svc['lpar'][:, 0], svc['lpar'][:, 1]
  ss = svc['ss']
  D.ss_t, D.ss_en, D.ss_exp, D.ss_pers = ss[:, 0], ss[:, 1] > 0.5, ss[:, 2] > 0.5, ss[:, 3]
  addr = {name: a for name, (a, _) in dbc.items()}
  # services the limits depend on: say so when they are not in the logs instead of quietly reporting less
  n_lp = max(len(lp), 1)
  if len(cc) < 100:
    D.notes.append((0, f'drive {dr}: carControl is not in the logs: cannot tell when openpilot drove (no pickups)'))
  if len(rs) < 0.5 * n_lp:
    D.notes.append((0, f'drive {dr}: radarState is (mostly) not in the logs: leads unknown, pickups count as no lead'))
  if len(sp) < 0.5 * n_lp:
    D.notes.append((1, f'drive {dr}: longitudinalPlanSP is not in the logs: sunnypilot speed targets (SP) not seen'))
  if len(ss) == 0:
    D.notes.append((1, f'drive {dr}: selfdriveState is not in the logs: personality / Experimental mode unknown'))
  if len(svc['lpar']) == 0:
    D.notes.append((1, f'drive {dr}: liveParameters is not in the logs: steering angle offset 0 assumed (turn cap)'))
  if info['delay'] != info['delay']:
    D.notes.append((1, f'drive {dr}: carParams is not in the logs: delay 0.5 s, steer ratio / wheelbase assumed'))

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
  D.c, D.srcs = {}, {}
  for key, msg, sname in CAR:
    k, x = pick(msg, car_src)
    s = dbc.get(msg, (None, {}))[1].get(sname)
    if x is None or s is None:
      D.c[key] = (np.zeros(0), np.zeros(0))
      continue
    raw, val = decode(np, x[1], s)
    if key in VALID:
      lo, hi = VALID[key]
      val[(val < lo) | (val > hi)] = NAN
    D.c[key] = (x[0], raw.astype(np.float64) if key in RAW_KEYS else val)
    D.srcs[msg] = f'bus {k[2]}'
  # what OP sent: the panda's echo of the frames that went out (same clock as the car's frames), else sendcan
  a06 = addr.get('ACC_06')
  snd = {k: x for k, x in can.items() if k[0] == 'sendcan' and k[1] == a06}
  op_bus = max(snd, key=lambda k: len(snd[k][0]))[2] if snd else 0
  echo = can.get(('can', a06, 128 + op_bus))
  n_snd = len(snd[('sendcan', a06, op_bus)][0]) if snd else 0
  rej = can.get(('can', a06, 192 + op_bus))      # the panda's safety blocked these (pandad: src + 0xC0)
  if rej is not None and len(rej[0]):
    D.notes.append((0, f'drive {dr}: the panda BLOCKED {len(rej[0])} ACC_06 frames (safety limits) while under 32 mph'))
  if echo is not None and len(echo[0]) >= 0.5 * max(n_snd, 1):
    x, D.srcs['ACC_06'] = echo, f'panda echo bus {op_bus}'
  elif snd:
    x, D.srcs['ACC_06'] = snd[('sendcan', a06, op_bus)], f'sendcan bus {op_bus}'
  else:
    x = None
  for key, msg, sname in OP:
    s = dbc.get(msg, (None, {}))[1].get(sname)
    if x is None or s is None:
      D.c[key] = (np.zeros(0), np.zeros(0))
      continue
    raw, val = decode(np, x[1], s)
    if key == 'cmd':
      val[raw >= 2046] = NAN      # 3.01 = openpilot not asking / error codes
    D.c[key] = (x[0], val)
  t, c = D.c['cmd']
  if len(t):
    act = hold(np, D.cc_t, D.long.astype(float), t) > 0.5 if len(D.cc_t) else np.ones(len(t), bool)
    D.cmd = (t, np.where(act & (c < 3.0), c, NAN))
  else:
    D.cmd = (D.cc_t, np.where(D.long, D.cc_a, NAN))
    D.srcs['ACC_06'] = 'not found: carControl accel used instead'
  # the TSK target is "not on this car" if it never changes while OP drives
  tt, tv = D.c['tsk_target']
  if len(tt):
    tv = np.where(tv > 5.4, NAN, tv)          # raw 127 = no value
    D.c['tsk_target'] = (tt, tv)
    on = hold(np, D.cmd[0], np.isfinite(D.cmd[1]).astype(float), tt) > 0.5
    if on.sum() < 50 or not np.isfinite(tv[on]).any() or np.nanstd(tv[on]) < 0.05:
      if on.sum() >= 50:
        D.notes.append((1, f'drive {dr}: Motor_20 TSK target never moved while openpilot drove: left out'))
      D.c['tsk_target'] = (np.zeros(0), np.zeros(0))
  D.missing = [m for m in MISSING_TXT if m not in D.srcs]
  # one wheel speed in m/s on the 50 Hz grid, and its slope = the car's real acceleration along the road
  ws = [D.c[k] for k in ('ws_fl', 'ws_fr', 'ws_rl', 'ws_rr')]
  if all(len(w[0]) > 20 for w in ws):
    tw = ws[0][0]
    vw = np.mean([np.interp(tw, w[0], w[1]) for w in ws], axis=0) / 3.6
    D.g_t, D.a_whl, _ = slope_grid(np, tw, vw)
  else:
    D.notes.append((1, f"drive {dr}: ESP_19 wheel speeds missing: carState.vEgo slope used for the car's acceleration"))
    D.g_t, D.a_whl, _ = slope_grid(np, D.cs_t, D.v)
  return D


# ============================================================================================ planner rebuild
def stock_cap(np, prof, v):
  if prof == 'cap17':
    return np.interp(v, CAP17_BP, CAP17_V)
  return np.interp(v, STOCK_BP, STOCK_V)


def straight_cap(np, prof, pmax, v):
  pick = np.interp(v, PICKUP_BP, [pmax, pmax, 0.0]) if prof == 'pickup' else np.zeros_like(v)
  return np.maximum(stock_cap(np, prof, v), pick)


def ideal_time(np, prof, pmax, v_to=V_END):
  """5 mph -> v_to at the straight-road cap the whole way (a perfect car, no MPC ramp)."""
  v = np.arange(V_START, v_to, 0.01) + 0.005
  return float(np.sum(0.01 / np.maximum(straight_cap(np, prof, pmax, v), 0.05)))


def plan_inputs(D):
  """What the planner saw at each longitudinalPlan frame."""
  np = D.np
  t = D.lp_t
  D.p_v = hold(np, D.cs_t, D.v, t, stale=1.0)
  off = hold(np, D.off_t, D.off, t, stale=10.0)
  D.p_ang = hold(np, D.cs_t, D.steer, t, stale=1.0) - np.where(np.isfinite(off), off, 0.0)
  D.p_pitch = hold(np, D.cc_t, D.pitch, t, stale=1.0)
  D.p_long = hold(np, D.cc_t, D.long.astype(float), t) > 0.5
  D.p_exp = hold(np, D.ss_t, D.ss_exp.astype(float), t, stale=2.0) > 0.5
  sr = D.info['steer_ratio'] if D.info['steer_ratio'] > 1.0 else STEER_RATIO
  wb = D.info['wheelbase'] if D.info['wheelbase'] > 1.0 else WHEELBASE
  D.p_ay = D.p_v ** 2 * D.p_ang * (math.pi / 180.0) / (sr * wb)


def infer_cap(D, prior, pmax_prior):
  """Which cap the drive's planner had, from frames where the plan was clearly clipped on a straight road."""
  np = D.np
  with np.errstate(invalid='ignore'):
    ok = (D.p_long & D.allow & ~D.p_exp & (D.src != SRC['e2e']) & np.isfinite(D.a_unc) & np.isfinite(D.aT)
          & (np.abs(D.p_ay) < 0.3) & (D.p_v >= 3.0) & (D.p_v <= 10.5) & (D.a_unc - D.aT > 0.05) & (D.aT > 0.5))
  n = int(ok.sum())
  if n < 20:
    return prior, pmax_prior, 'assumed', n
  a, v = D.aT[ok], D.p_v[ok]

  def err(cap):
    return float(np.median(np.abs(a - cap)))
  # every candidate cap is fitted to the clipped frames; the best one wins if it fits within 0.03 m/s2
  cands = [('stock', pmax_prior, err(stock_cap(np, 'stock', v))), ('cap17', pmax_prior, err(stock_cap(np, 'cap17', v)))]
  hi = a > stock_cap(np, 'stock', v) + 0.05      # frames above the stock cap: the pickup cap's level
  if hi.sum() >= 10:
    lvl = round(min(float(np.median(a[hi])), PICKUP_LIMIT), 3)
    cands.append(('pickup', lvl, err(straight_cap(np, 'pickup', lvl, v))))
  prof, pmax, e = min(cands, key=lambda c: c[2])
  if e <= 0.03:
    return prof, pmax, 'seen', n
  return prior, pmax_prior, 'unclear', n


def rebuild_caps(D, prof, pmax):
  """The planner's upper accel clip, part by part (longitudinal_planner.py update())."""
  np = D.np
  v = np.where(np.isfinite(D.p_v), D.p_v, 0.0)
  pick = np.interp(v, PICKUP_BP, [pmax, pmax, 0.0]) if prof == 'pickup' else np.zeros_like(v)
  D.c_raw = np.maximum(stock_cap(np, prof, v), pick)
  a_tot = np.maximum(np.interp(v, TURN_BP, TURN_V), pick)
  ay = np.where(np.isfinite(D.p_ay), D.p_ay, 0.0)
  D.c_turn = np.minimum(D.c_raw, np.sqrt(np.maximum(a_tot ** 2 - ay ** 2, 0.0)))
  coast = np.where(np.isfinite(D.p_pitch), np.sin(np.where(np.isfinite(D.p_pitch), D.p_pitch, 0.0)) * -5.65 - 0.3,
                   ACCEL_MAX)
  coast = np.maximum(coast, ACCEL_MIN)
  frac = np.clip((v - 2.5) / 2.5, 0.0, 1.0)
  gate = D.c_turn + frac * (coast - D.c_turn)
  D.c_thr = np.where(D.allow, D.c_turn, np.minimum(D.c_turn, gate))
  cs_ = np.empty(len(v))
  prev = ACCEL_MAX
  for i, x in enumerate(D.c_thr.tolist()):     # the 0.05 per update slew on the clip
    prev = min(max(x, prev - SLEW), prev + SLEW)
    cs_[i] = prev
  D.c_s = cs_


def lead_inputs(D, t_follow):
  np = D.np
  t = D.lp_t
  l1 = hold(np, D.rs_t, D.l1.astype(float), t) > 0.5
  d = hold(np, D.rs_t, D.dRel, t)
  vl = hold(np, D.rs_t, D.vLead, t)
  v = D.p_v
  with np.errstate(invalid='ignore'):
    D.p_lead = l1 & (d < LEAD_D)
    wanted = v ** 2 / 5.0 + t_follow * v + 6.0          # the MPC's wanted gap (6 m is compiled into the solver)
    have = d + np.maximum(vl, 0.0) ** 2 / 5.0
    D.p_room = np.where(D.p_lead, have - wanted, NAN)   # m: <= 0.5 = already at / under the wanted gap
    D.p_close = D.p_lead & ((D.src == 1) | (D.src == 2) | (D.p_room <= CLOSE_GAP) | ((vl < v - 0.5) & (d < 40.0)))
  D.p_d = np.where(D.p_lead, d, NAN)
  D.p_vl = np.where(D.p_lead, vl, NAN)


def set_inputs(D):
  np = D.np
  t = D.lp_t
  vc = hold(np, D.cs_t, D.vcruise, t, stale=1.0)
  with np.errstate(invalid='ignore'):
    vc = np.where((vc > 0) & (vc < 254), np.minimum(vc, 145.0) / 3.6, NAN)
  D.p_set = vc
  D.p_spsrc = hold(np, D.sp_t, D.sp_src, t, stale=1.0)
  spv = hold(np, D.sp_t, D.sp_v, t, stale=1.0)
  D.p_target = np.where(np.isfinite(spv) & (spv > 0.1), np.fmin(spv, np.where(np.isfinite(vc), vc, spv)), vc)
  rate = np.full(len(t), NAN)
  if len(t) > 6:
    rate[6:] = (D.aT[6:] - D.aT[:-6]) / np.maximum(t[6:] - t[:-6], 0.05)
  D.p_rate = rate


def classify(D):
  """The binding limiter of every plan frame (an index into CODES)."""
  np = D.np
  n = len(D.lp_t)
  code = np.full(n, C['?'], dtype=np.int64)
  mism = np.zeros(n, bool)
  aT, au, cs_, cthr, cturn, craw = D.aT, D.a_unc, D.c_s, D.c_thr, D.c_turn, D.c_raw
  for i in range(n):
    a = aT[i]
    if not a == a:
      continue
    u = au[i]
    hu = u == u
    if hu and a > u + 0.05 and a > 0.2:
      code[i] = C['BST']
      continue
    at_cap = a >= cs_[i] - CAP_TOL
    clipped = hu and u > a + CLIP_TOL and D.src[i] != 4
    if at_cap or clipped:
      thr = (not D.allow[i]) and cthr[i] < cturn[i] - COMP_TOL
      trn = cturn[i] < craw[i] - COMP_TOL
      if cs_[i] < cthr[i] - COMP_TOL:
        code[i] = C['SLW']
      elif thr:
        code[i] = C['THR']
      elif trn:
        code[i] = C['TRN']
      else:
        code[i] = C['CAP']
      if clipped and abs(a - cs_[i]) > 0.1:
        mism[i] = True
    elif D.src[i] == 4 or (D.p_exp[i] and hu and u > a + CLIP_TOL):
      code[i] = C['E2E']
    elif D.p_spsrc[i] > 0.5 and (D.p_spsrc[i] < 2.5 or D.p_target[i] - D.p_v[i] < SET_NEAR):
      # SCC vision / map also set the MPC's start accel: SP whenever they are the target. Speed Limit Assist only
      # lowers the target speed (its accel is aEgo): like a set speed, it holds the plan back only when close
      code[i] = C['SP']
    elif D.p_close[i]:
      code[i] = C['LEAD']
    elif D.p_set[i] - D.p_v[i] < SET_NEAR or D.p_target[i] - D.p_v[i] < SET_NEAR:
      code[i] = C['SET']
    elif D.p_rate[i] > RAMP_RATE:
      code[i] = C['RMP']
    else:
      code[i] = C['MPC']
  D.code, D.mism = code, mism


# ============================================================================================ pickups
def find_pickups(D):
  np = D.np
  cv, ct = D.v, D.cs_t
  D.skipped = {'driver on a pedal': 0, 'OP not driving': 0, 'shorter than 1 s': 0}
  if len(D.cc_t) == 0:
    return []
  long_cs = hold(np, D.cc_t, D.long.astype(float), ct) > 0.5
  with np.errstate(invalid='ignore'):
    vset = np.where((D.vcruise > 0) & (D.vcruise < 254), D.vcruise / 3.6, NAN)
  vlim = np.full(len(ct), NAN)
  if len(D.sp_t):   # Speed Limit Assist's target works like a set speed (SCC vision / map only for a bend)
    spv = hold(np, D.sp_t, D.sp_v, ct, stale=1.0)
    sps = hold(np, D.sp_t, D.sp_src, ct, stale=1.0)
    with np.errstate(invalid='ignore'):
      vlim = np.where(np.isfinite(spv) & (spv > 0.1) & (sps == SPSRC['speedLimitAssist']), spv, NAN)
  up = np.where((cv[:-1] < V_START) & (cv[1:] >= V_START))[0] + 1
  # when openpilot's long control last switched on (SET / RES pressed): carControl.longActive rising edges
  on = D.long.astype(np.int8)
  rise_t = D.cc_t[1:][np.diff(on) > 0] if len(on) > 1 else np.zeros(0)
  picks = []
  for i in up:
    t_a = ct[i]
    if picks and t_a < picks[-1]['t_b']:
      continue
    if not long_cs[i]:
      D.skipped['OP not driving'] += 1
      continue
    if D.gas[i] or D.brk[i]:
      D.skipped['driver on a pedal'] += 1
      continue
    j1 = int(np.searchsorted(ct, t_a + MAX_S, side='right'))
    seg = slice(i, j1)
    vv = cv[seg]
    with np.errstate(invalid='ignore'):
      stops = [
        (vv >= V_END, '25'),
        (D.gas[seg], 'GAS'),
        (D.brk[seg], 'BRAKE'),
        (~long_cs[seg], 'OP off'),
        (vv < V_SLOW, 'slowed'),
        ((vset[seg] < V_END) & (vv >= vset[seg] - SET_MARGIN), 'set'),
        ((vlim[seg] < V_END) & (vv >= vlim[seg] - SET_MARGIN), 'limit'),
      ]
    k_end, why = len(vv), ('25 s' if ct[j1 - 1] >= t_a + MAX_S - 0.1 else 'log end')
    for mask, w in stops:
      hit = np.where(mask)[0]
      if len(hit) and hit[0] < k_end:
        k_end, why = int(hit[0]), w
    k = min(i + k_end, len(cv) - 1)
    t_b = ct[k]
    if t_b - t_a < MIN_S:
      D.skipped['shorter than 1 s'] += 1
      continue
    j = int(np.searchsorted(rise_t, t_a, side='right')) - 1
    set0 = float(D.vcruise[i]) if 0 < D.vcruise[i] < 254 else NAN       # km/h
    set1 = float(D.vcruise[k]) if 0 < D.vcruise[k] < 254 else NAN
    picks.append({'t_a': t_a, 't_b': t_b, 'why': why, 'v_end': float(cv[k]), 'set0': set0, 'set1': set1,
                  'set40': bool(abs(set0 - V_CRUISE_INITIAL) < 0.3),     # SET pressed under 25 mph gives 40 km/h
                  'eng_ago': float(t_a - rise_t[j]) if j >= 0 else NAN})
  return picks


def nmed(x):
  x = x[np.isfinite(x)] if len(x) else x
  return float(np.median(x)) if len(x) else NAN


def nmax(x):
  x = x[np.isfinite(x)] if len(x) else x
  return float(np.max(x)) if len(x) else NAN


def analyse(D, pk, prof, pmax):
  np = D.np
  t_a, t_b = pk['t_a'], pk['t_b']
  pk['route'], pk['rid'], pk['clock'] = D.route, rid(D.route), clock(t_a - D.t0)
  pk['prof'], pk['pmax'] = prof, pmax
  pk['dur'] = t_b - t_a
  pk['t525'] = pk['dur'] if pk['why'] == '25' else NAN
  m = (D.lp_t >= t_a) & (D.lp_t < t_b)
  nf = int(m.sum())
  pk['nf'] = nf
  pk['codes'] = np.bincount(D.code[m], minlength=len(CODES)) if nf else np.zeros(len(CODES), dtype=np.int64)
  pk['mism'] = int(D.mism[m].sum())
  pk['lead_share'] = float(np.mean(D.p_lead[m])) if nf else 0.0
  pk['lead'] = pk['lead_share'] >= LEAD_SHARE
  room = D.p_room[m]
  pk['close'] = float(np.mean(room[np.isfinite(room)] <= 0.5)) if np.isfinite(room).any() else NAN
  pk['gap'] = nmed(D.p_d[m])
  pk['plan_med'], pk['plan_max'] = nmed(D.aT[m]), nmax(D.aT[m])
  pk['mpc_max'] = nmax(D.a_unc[m])
  pk['cap_med'] = nmed(D.c_s[m])
  pk['thr_off'] = float(np.mean(~D.allow[m] & (D.p_v[m] > 2.5))) if nf else NAN
  pk['set_min'] = nmin_set = float(np.nanmin(D.p_set[m])) if nf and np.isfinite(D.p_set[m]).any() else NAN
  pk['set_mph'] = nmin_set * MPH if nmin_set == nmin_set else NAN
  pk['sp'] = float(np.mean(D.p_spsrc[m] > 0.5)) if nf else 0.0
  pk['exp'] = bool(D.p_exp[m].any()) if nf else False
  # the car: 50 Hz grid over the pickup
  E = np.arange(t_a, t_b, DT)
  sent = hold(np, *D.cmd, E, stale=0.1)
  tsk = hold(np, *D.c['tsk_target'], E, stale=0.2)
  whl = grid_interp(np, D.g_t, D.a_whl, E) if len(D.g_t) else np.full(len(E), NAN)
  aego = np.interp(E, D.cs_t, D.aego)
  pk['sent_med'], pk['sent_max'] = nmed(sent), nmax(sent)
  pk['tsk_med'], pk['tsk_max'] = nmed(tsk), nmax(tsk)
  pk['whl_med'] = nmed(whl)
  sm = np.convolve(np.where(np.isfinite(whl), whl, 0.0), np.ones(15) / 15, mode='same') if len(whl) >= 15 else whl
  sm = np.where(np.isfinite(whl), sm, NAN)
  pk['whl_max'] = nmax(sm[7:-7]) if len(sm) > 20 else nmax(whl)
  pk['aego_med'], pk['aego_max'] = nmed(aego), nmax(aego)
  # delivered: the car's mean accel over the pickup / the mean request CAR_LAG s earlier
  El = E[E >= t_a + CAR_LAG]
  u = hold(np, *D.cmd, El - CAR_LAG, stale=0.1)
  y = grid_interp(np, D.g_t, D.a_whl, El) if len(D.g_t) else np.full(len(El), NAN)
  ok = np.isfinite(u) & np.isfinite(y)
  pk['car'] = float(np.mean(y[ok]) / np.mean(u[ok])) if ok.sum() > 25 and np.mean(u[ok]) > 0.3 else NAN
  with np.errstate(invalid='ignore'):
    short = ok & (u > 0.5) & (y < u - 0.3)
  pk['car_short'] = float(short.sum() / max((ok & (u > 0.5)).sum(), 1)) if ok.any() else NAN
  # the TSK's own target vs the request, where the request is steady and positive
  if np.isfinite(tsk).any() and len(sent) > 25:
    prev = np.concatenate([np.full(25, NAN), sent[:-25]])
    with np.errstate(invalid='ignore'):
      st_ = np.isfinite(sent) & np.isfinite(tsk) & (sent > 0.3) & (np.abs(sent - prev) < 0.15)
    pk['tsk_gap'] = float(np.median(sent[st_] - tsk[st_])) if st_.sum() > 10 else NAN
  else:
    pk['tsk_gap'] = NAN
  # the car's wheels against the TSK's own target (TSK -> engine / DSG), same lag
  ut = hold(np, *D.c['tsk_target'], El - CAR_LAG, stale=0.2)
  okt = np.isfinite(ut) & np.isfinite(y)
  pk['car_tsk'] = float(np.mean(y[okt]) / np.mean(ut[okt])) if okt.sum() > 25 and np.mean(ut[okt]) > 0.3 else NAN
  # TSK_06 TSK_Status: 3 = regulating; 5 = brake only (no engine torque); 4 = driver's pedal
  stat = hold(np, *D.c['tsk_status'], E, stale=0.5)
  sv = stat[np.isfinite(stat)].astype(int)
  pk['tsk_not3'] = float(np.mean(sv != 3)) if len(sv) else NAN
  pk['tsk5'] = float(np.mean(sv == 5)) if len(sv) else NAN
  pk['tsk_mode'] = int(np.bincount(sv).argmax()) if len(sv) else -1
  # gear lever (Getriebe_11 tells D from S; Gateway_73 shows S as D) and the DSG's target gear
  lev = hold(np, *D.c['lever_tx'], E, stale=0.5) if len(D.c['lever_tx'][0]) else hold(np, *D.c['lever'], E, stale=0.5)
  levs = lev[np.isfinite(lev)].astype(int)
  pk['lever'] = GEAR.get(int(np.bincount(levs).argmax()), '?') if len(levs) else '?'
  gz = hold(np, *D.c['gear'], E, stale=0.5)
  gz = gz[np.isfinite(gz) & (gz >= 1) & (gz <= 8)].astype(int)
  pk['gear_lo'], pk['gear_hi'] = (int(gz.min()), int(gz.max())) if len(gz) else (-1, -1)
  pk['rpm_med'] = nmed(hold(np, *D.c['rpm'], E, stale=0.2))
  # engine torque asked (MO_Mom_Soll_Roh) vs delivered (MO_Mom_Ist_Summe): a gap = engine / turbo lag
  er, ea = hold(np, *D.c['eng_req'], E, stale=0.2), hold(np, *D.c['eng'], E, stale=0.2)
  pk['eng_req'], pk['eng_got'] = nmed(er), nmed(ea)
  with np.errstate(invalid='ignore'):
    pk['eng_short'] = nmed(er - ea)
  pk['grade'] = nmed(hold(np, *D.c['grade'], E, stale=2.0))
  pk['dyn'] = nmax(hold(np, *D.c['dyn'], E, stale=0.2))
  # 5 -> 20 mph, which every pickup to a set speed of 25 mph or more passes
  mc = (D.cs_t >= t_a) & (D.cs_t <= t_b)
  hit = np.where(D.v[mc] >= V_20)[0]
  pk['t520'] = float(D.cs_t[mc][hit[0]] - t_a) if len(hit) else NAN
  # the same for a car that does exactly what was planned, after openpilot's own actuator delay (aTarget is
  # what openpilot asks now to get that accel CarParams.longitudinalActuatorDelay later)
  dl = D.info['delay'] if 0.0 < D.info['delay'] < 2.0 else 0.5
  oka = np.isfinite(D.aT)
  # up to 5 s past the pickup's end while openpilot still drives: a car slower than the real one (which got to
  # 20 mph inside the pickup) must not drop out of the medians just because it is slower
  mx = (D.lp_t >= t_a) & (D.lp_t < t_b + 5.0)
  lt = D.lp_t[mx]
  past = (lt >= t_b) & ~D.p_long[mx]
  if past.any():
    lt = lt[:int(np.argmax(past))]
  if len(lt) > 5 and oka.sum() > 5:
    la = np.interp(lt - dl, D.lp_t[oka], D.aT[oka])
    vp = float(D.v[mc][0]) + la[0] * (lt[0] - t_a) + np.concatenate([[0.0], np.cumsum(0.5 * (la[1:] + la[:-1]) * np.diff(lt))])
    hp = np.where(vp >= V_20)[0]
    pk['plan520'] = float(lt[hp[0]] - t_a) if len(hp) else NAN
  else:
    pk['plan520'] = NAN
  pk['set0_mph'] = pk['set0'] / 3.6 * MPH if pk['set0'] == pk['set0'] else NAN
  pk['set1_mph'] = pk['set1'] / 3.6 * MPH if pk['set1'] == pk['set1'] else NAN
  # detail rows: 4 per second from 1 s before to 0.5 s after
  G = np.arange(t_a - 1.0, t_b + 0.5 + 1e-9, 0.25)
  li = np.clip(np.searchsorted(D.lp_t, G, side='right') - 1, 0, max(len(D.lp_t) - 1, 0))
  okp = (np.abs(D.lp_t[li] - G) < 0.2) if len(D.lp_t) else np.zeros(len(G), bool)

  def lp_at(x):
    return np.where(okp, x[li], NAN) if len(D.lp_t) else np.full(len(G), NAN)
  pk['det'] = {
    't': G - t_a, 'mph': np.interp(G, D.cs_t, D.v) * MPH, 'set': lp_at(D.p_set) * MPH, 'plan': lp_at(D.aT),
    'mpc': lp_at(D.a_unc), 'cap': lp_at(D.c_s), 'sent': hold(np, *D.cmd, G, stale=0.1),
    'tsk': hold(np, *D.c['tsk_target'], G, stale=0.2),
    'whl': grid_interp(np, D.g_t, D.a_whl, G) if len(D.g_t) else np.full(len(G), NAN),
    'aego': np.interp(G, D.cs_t, D.aego), 'lead': lp_at(D.p_d), 'rpm': hold(np, *D.c['rpm'], G, stale=0.2),
    'eng': hold(np, *D.c['eng'], G, stale=0.2), 'eng_req': hold(np, *D.c['eng_req'], G, stale=0.2),
    'lever': (hold(np, *D.c['lever_tx'], G, stale=0.5) if len(D.c['lever_tx'][0]) else
              hold(np, *D.c['lever'], G, stale=0.5)),
    'gear': hold(np, *D.c['gear'], G, stale=0.5), 'st': hold(np, *D.c['tsk_status'], G, stale=0.5),
    'code': np.where(okp, D.code[li], C['?']) if len(D.lp_t) else np.full(len(G), C['?']),
    'allow': lp_at(D.allow.astype(float)), 'ang': lp_at(D.p_ang),
  }
  return pk


# ============================================================================================ output
def fmt(x, nd=2, w=5, plus=False):
  if x is None or x != x:
    return ' ' * (w - 1) + '-'
  x = round(x, nd) + 0.0     # no '-0.00'
  return f'{x:+{w}.{nd}f}' if plus else f'{x:{w}.{nd}f}'


def pct(x, w=3):
  return ' ' * (w - 1) + '-' if x is None or x != x else f'{x * 100:{w}.0f}'


def pcts(x, w=3):
  """Percent with its sign, or a lone '-' in the same width."""
  return ' ' * w + '-' if x is None or x != x else f'{x * 100:{w}.0f}%'


def clock(t):
  return f'{int(t // 60)}:{int(t % 60):02d}'


def med(vals):
  vals = [v for v in vals if v is not None and v == v]
  return st.median(vals) if vals else NAN


def result_txt(pk):
  w, mph, d = pk['why'], pk['v_end'] * MPH, pk['dur']
  if w == '25':
    return f'25 mph in {d:.1f} s'
  if w == 'set':
    return f'set speed {mph:.0f} mph, {d:.1f} s'
  if w == 'limit':
    return f'speed limit {mph:.0f} mph, {d:.1f} s'
  if w == 'slowed':
    return f'slowed after {d:.1f} s'
  return f'{w} at {mph:.0f} mph, {d:.1f} s'


def top_codes(counts, n=3, sep=' '):
  tot = int(counts.sum())
  if tot == 0:
    return '-'
  order = sorted(range(len(CODES)), key=lambda i: -counts[i])
  return sep.join(f'{CODES[i]} {counts[i] * 100.0 / tot:.0f}%' for i in order[:n] if counts[i] * 200 >= tot) or '-'


def lead_txt(pk):
  """'no lead', or 'lead N%' with N = share of the time the gap was at or under the MPC's wanted gap."""
  if not pk['lead']:
    return 'no lead'
  return 'lead' + (f" {pk['close'] * 100:.0f}%" if pk['close'] == pk['close'] else '')


def set_txt(pk):
  """Set speed at the start in mph; * = 40 km/h (SET pressed under 25 mph); >N = changed to N during the pickup."""
  s0, s1 = pk['set0_mph'], pk['set1_mph']
  if s0 != s0:
    return '-'
  txt = f'{s0:.0f}' + ('*' if pk['set40'] else '')
  if s1 == s1 and abs(s1 - s0) >= 1.0:
    txt += f'>{s1:.0f}'
  return txt


def tsk_txt(pk):
  """TSK_06 TSK_Status most of the time, plus the share of 5 (brake only) if any."""
  if pk['tsk_mode'] < 0:
    return '-'
  txt = str(pk['tsk_mode'])
  if pk['tsk5'] == pk['tsk5'] and pk['tsk5'] > 0.005 and pk['tsk_mode'] != 5:
    txt += f" 5:{pk['tsk5'] * 100:.0f}%"
  return txt


def gear_txt(lever, lo, hi):
  g = lever if lever != '?' else ''
  if lo > 0:
    g += f'{lo}' if lo == hi else f'{lo}-{hi}'
  return g or '-'


def eng_txt(req, got):
  if req != req and got != got:
    return '-'
  return f'{fmt(req, 0, 3)}/{fmt(got, 0, 3)}'


def print_pickups(picks):
  say('PICKUPS (accels in m/s2: plan = openpilot\'s planned accel, longitudinalPlan.aTarget; MPC = what it wanted before')
  say('  the cap; sent = ACC_06 that went out; TSK = cruise computer\'s own target (Motor_20); wheels = slope of the wheel')
  say('  speeds; car% = wheels / sent; set = set speed at the start in mph, * = 40 km/h = SET pressed under 25 mph,')
  say('  >N = changed during the pickup; lead N% = a car ahead, at or under the MPC\'s wanted gap N% of the time;')
  say('  5-20 = s from 5 to 20 mph; thr = time allowThrottle was off; TSK st = TSK_Status')
  say('  most of the time (3 = regulating, 5 = brake only); eng = engine torque asked/got Nm; gear = lever + DSG target')
  say('  gear; limits = share of planner frames held back by each limiter)')
  say('   #  drive     at  lead       set  result                    5-20  plan med/max   MPC  sent  TSK  wheels med/max'
      '  aEgo car%  thr  TSK st  eng Nm    gear  limits')
  for i, pk in enumerate(picks, 1):
    say(f"  {i:2d} {pk['rid']:>5} {pk['clock']:>6}  {lead_txt(pk):<9} {set_txt(pk):>5}  {result_txt(pk):<24}"
        f" {fmt(pk['t520'], 1, 5)}  {fmt(pk['plan_med'], 2, 5)}/{fmt(pk['plan_max'], 2, 4)}  {fmt(pk['mpc_max'], 2, 4)}"
        f" {fmt(pk['sent_max'], 2, 5)} {fmt(pk['tsk_max'], 1, 4)}  {fmt(pk['whl_med'], 2, 5)}/{fmt(pk['whl_max'], 2, 4)}"
        f"   {fmt(pk['aego_med'], 2, 4)} {pcts(pk['car'])} {pcts(pk['thr_off'])}  {tsk_txt(pk):<7} {eng_txt(pk['eng_req'], pk['eng_got']):>7}"
        f"  {gear_txt(pk['lever'], pk['gear_lo'], pk['gear_hi']):<5} {top_codes(pk['codes'])}")


def wmean(G, key):
  """Share pooled over all planner frames of the group (each pickup weighted by its length)."""
  num = sum(pk[key] * pk['nf'] for pk in G if pk[key] == pk[key])
  den = sum(pk['nf'] for pk in G if pk[key] == pk[key])
  return num / den if den else NAN


def group_stats(G):
  counts = sum((pk['codes'] for pk in G), np.zeros(len(CODES), dtype=np.int64))
  reached = [pk for pk in G if pk['why'] == '25']
  r20 = [pk for pk in G if pk['t520'] == pk['t520']]
  lo = [pk['gear_lo'] for pk in G if pk['gear_lo'] > 0]
  hi = [pk['gear_hi'] for pk in G if pk['gear_hi'] > 0]
  levs = [pk['lever'] for pk in G if pk['lever'] != '?']
  return {
    'n': len(G), 'reached': len(reached), 't525': med([pk['t525'] for pk in reached]),
    'ideal': med([pk['ideal'] for pk in G]), 'ideal20': med([pk['ideal20'] for pk in G]),
    'n20': len(r20), 't520': med([pk['t520'] for pk in r20]), 'plan520': med([pk['plan520'] for pk in r20]),
    'plan_med': med([pk['plan_med'] for pk in G]), 'plan_max': med([pk['plan_max'] for pk in G]),
    'mpc_max': med([pk['mpc_max'] for pk in G]), 'sent_max': med([pk['sent_max'] for pk in G]),
    'tsk_max': med([pk['tsk_max'] for pk in G]), 'whl_med': med([pk['whl_med'] for pk in G]),
    'whl_max': med([pk['whl_max'] for pk in G]), 'aego_med': med([pk['aego_med'] for pk in G]),
    'car': med([pk['car'] for pk in G]), 'car_short': med([pk['car_short'] for pk in G]),
    'car_tsk': med([pk['car_tsk'] for pk in G]), 'tsk_gap': med([pk['tsk_gap'] for pk in G]),
    'tsk_not3': wmean(G, 'tsk_not3'), 'tsk5': wmean(G, 'tsk5'), 'thr_off': wmean(G, 'thr_off'),
    'rpm': med([pk['rpm_med'] for pk in G]), 'eng_req': med([pk['eng_req'] for pk in G]),
    'eng_got': med([pk['eng_got'] for pk in G]), 'eng_short': med([pk['eng_short'] for pk in G]),
    'close': med([pk['close'] for pk in G]),
    'counts': counts, 'secs': float(counts.sum()) * DT_PLAN, 'cap_med': med([pk['cap_med'] for pk in G]),
    'set0': med([pk['set0_mph'] for pk in G]),
    'set40': sum(1 for pk in G if pk['set40']),
    'low_set': sum(1 for pk in G if pk['set0_mph'] == pk['set0_mph'] and pk['set0_mph'] <= 26.0),
    'taken': sum(1 for pk in G if pk['why'] in ('GAS', 'BRAKE', 'OP off')),
    'S': levs.count('S'), 'D': levs.count('D'), 'gear_lo': min(lo) if lo else -1, 'gear_hi': max(hi) if hi else -1,
    'lever': max(set(levs), key=levs.count) if levs else '?',
    'n_carlow': sum(1 for pk in G if pk['car'] == pk['car'] and pk['car'] < 0.85),
    'n_tskgap': sum(1 for pk in G if pk['tsk_gap'] == pk['tsk_gap'] and pk['tsk_gap'] > 0.15),
    'tskgap_max': max([pk['tsk_gap'] for pk in G if pk['tsk_gap'] == pk['tsk_gap']], default=NAN),
    'n_cartsk': sum(1 for pk in G if pk['car_tsk'] == pk['car_tsk'] and pk['car_tsk'] < 0.85),
    'n_engshort': sum(1 for pk in G if pk['eng_short'] == pk['eng_short'] and pk['eng_short'] > 20.0),
    'n_tsk5': sum(1 for pk in G if pk['tsk5'] == pk['tsk5'] and pk['tsk5'] > 0.02),
    'plan_cost': med([pk['plan520'] - pk['ideal20'] for pk in r20]),
    'car_cost': med([pk['t520'] - pk['plan520'] for pk in r20]),
    'plan_cost_max': max([pk['plan520'] - pk['ideal20'] for pk in r20 if pk['plan520'] == pk['plan520']], default=NAN),
    'car_cost_max': max([pk['t520'] - pk['plan520'] for pk in r20 if pk['plan520'] == pk['plan520']], default=NAN),
  }


def share(g, *codes):
  tot = g['counts'].sum()
  return float(sum(g['counts'][C[c]] for c in codes)) / tot if tot else 0.0


def print_medians(groups):
  say('MEDIANS per group, plan side (5-25 / 5-20 = s for the pickups that got there; "if as planned" = 5-20 if the car')
  say('  had done exactly the planned accel; "at cap" = 5-20 for a perfect car at the cap the whole way; SET* = pickups')
  say('  that started at 40 km/h, i.e. SET pressed under 25 mph; thr off = share of time allowThrottle was off)')
  say('  group        n  to25  5-25  5-20  if as planned  at cap   set SET*  plan med/max   MPC  thr off  taken over')
  for name, g in groups:
    if not g['n']:
      continue
    say(f"  {name:<10} {g['n']:3d}  {g['reached']:4d} {fmt(g['t525'], 1, 5)} {fmt(g['t520'], 1, 5)}  {fmt(g['plan520'], 1, 13)}"
        f"  {fmt(g['ideal20'], 1, 6)}  {fmt(g['set0'], 0, 4)} {g['set40']:4d}  {fmt(g['plan_med'], 2, 5)}/{fmt(g['plan_max'], 2, 4)}"
        f"  {fmt(g['mpc_max'], 2, 4)}  {pcts(g['thr_off'], 6)}  {g['taken']:10d}")
  say('MEDIANS per group, car side (TSK-sent = how far the TSK\'s own target sat under the steady request; car/TSK =')
  say('  wheels / TSK target; TSK5 = share of time TSK_Status was 5, brake only = no engine torque)')
  say('  group       sent   TSK  TSK-sent  TSK5  eng asked/got Nm   rpm   gear  wheels med/max  aEgo  car%  car/TSK')
  for name, g in groups:
    if not g['n']:
      continue
    say(f"  {name:<10} {fmt(g['sent_max'], 2, 5)} {fmt(g['tsk_max'], 1, 5)}  {fmt(g['tsk_gap'], 2, 7, plus=True)}  {pcts(g['tsk5'], 3)}"
        f"  {eng_txt(g['eng_req'], g['eng_got']):>16}  {fmt(g['rpm'], 0, 5)}  {gear_txt(g['lever'], g['gear_lo'], g['gear_hi']):>5}"
        f"  {fmt(g['whl_med'], 2, 6)}/{fmt(g['whl_max'], 2, 4)}   {fmt(g['aego_med'], 2, 4)}  {pcts(g['car'])}     {pcts(g['car_tsk'])}")
  say()
  say('LIMITS, share of all pickup time (planner frames) per group:')
  for name, g in groups:
    if not g['n']:
      continue
    say(f"  {name:<10} {g['secs']:6.1f} s: {top_codes(g['counts'], n=len(CODES))}")


def car_verdict(g):
  """Car-side findings for one group: from the medians, else how many pickups showed it."""
  out = []
  n = g['n']
  car, tg, ct, es, t5 = g['car'], g['tsk_gap'], g['car_tsk'], g['eng_short'], g['tsk5']
  if car == car and car < 0.85:
    out.append(f'the car gave only {car * 100:.0f}% of the request (median): car side (TSK / engine / DSG), not the planner')
  elif g['n_carlow']:
    out.append(f"the car gave under 85% of the request on {g['n_carlow']} of {n} pickups")
  if tg == tg and tg > 0.15:
    out.append(f'the TSK held its own target {tg:.2f} m/s2 under the request (median)')
  elif g['n_tskgap']:
    out.append(f"the TSK held its own target under the request on {g['n_tskgap']} of {n} pickups"
               f" (by up to {g['tskgap_max']:.2f} m/s2)")
  if ct == ct and ct < 0.85:
    out.append(f"the engine / DSG gave only {ct * 100:.0f}% of the TSK's own target (median)")
  elif g['n_cartsk']:
    out.append(f"the engine / DSG gave under 85% of the TSK's own target on {g['n_cartsk']} of {n} pickups")
  if es == es and es > 20.0:
    out.append(f'the engine delivered {es:.0f} Nm less torque than asked (median; engine / turbo lag)')
  elif g['n_engshort']:
    out.append(f"the engine delivered over 20 Nm less torque than asked on {g['n_engshort']} of {n} pickups"
               ' (engine / turbo lag)')
  if t5 == t5 and t5 > 0.05:
    out.append(f'TSK_Status was 5 (brake only, no engine torque) {t5 * 100:.0f}% of the time')
  elif g['n_tsk5']:
    out.append(f"TSK_Status went to 5 (brake only, no engine torque) on {g['n_tsk5']} of {n} pickups")
  return out


def bottom_line(groups, cap_txt, pmax_txt, room_txt):
  out = []
  g = dict(groups).get('no lead')
  if g and g['n']:
    parts = []
    tot = g['counts'].sum()
    order = sorted((i for i in range(len(CODES)) if g['counts'][i] * 200 >= tot), key=lambda i: -g['counts'][i])
    for i in order[:4]:
      txt = CODE_TEXT[CODES[i]]
      if CODES[i] == 'CAP':
        txt += f' ({cap_txt}, {pmax_txt})'
      parts.append(f"{g['counts'][i] * 100.0 / tot:.0f}% {txt}")
    out.append(f"With no car ahead ({g['n']} pickup{'s' if g['n'] != 1 else ''}, {g['secs']:.0f} s), the limit on the planned accel was: "
               + '; '.join(parts) + '.')
    out.append(f"  Plan side: openpilot planned {fmt(g['plan_med'], 2, 1)} median / {fmt(g['plan_max'], 2, 1)} max m/s2 (the MPC"
               f" wanted up to {fmt(g['mpc_max'], 2, 1)}); set speed at the start {fmt(g['set0'], 0, 1)} mph median,"
               f" {g['set40']} of {g['n']} started at 40 km/h (SET pressed under 25 mph); the throttle gate was off"
               f" {pcts(g['thr_off'], 1)} of the time.")
    cs_ = [f"openpilot sent up to {fmt(g['sent_max'], 2, 1)}"]
    if g['tsk_gap'] == g['tsk_gap']:
      cs_.append(f"the TSK's own target sat {fmt(g['tsk_gap'], 2, 1)} under it while steady")
    if g['tsk5'] == g['tsk5']:
      cs_.append(f"TSK_Status 5 (brake only) {pcts(g['tsk5'], 1)} of the time")
    if g['eng_req'] == g['eng_req']:
      cs_.append(f"engine torque asked/got {eng_txt(g['eng_req'], g['eng_got']).replace(' ', '')} Nm")
    cs_.append(f"the wheels gave {fmt(g['whl_med'], 2, 1)} median = {pcts(g['car'], 1)} of what was sent")
    out.append('  Car side: ' + '; '.join(cs_) + '.')
    if g['n20']:
      txt = f"  5->20 mph took {g['t520']:.1f} s (median of {g['n20']})"
      if g['plan520'] == g['plan520']:
        txt += (f"; {g['plan520']:.1f} s for a car doing exactly what was planned (after openpilot's assumed delay);"
                f" {g['ideal20']:.1f} s for a perfect car at the cap. Per pickup, the plan lost"
                f" {fmt(g['plan_cost'], 1, 1, plus=True)} s against the cap (worst {fmt(g['plan_cost_max'], 1, 1, plus=True)})"
                f" and the car {fmt(g['car_cost'], 1, 1, plus=True)} s against the plan"
                f" (worst {fmt(g['car_cost_max'], 1, 1, plus=True)}); medians.")
      out.append(txt)
    if g['reached']:
      out.append(f"  5->25 mph took {g['t525']:.1f} s (median of {g['reached']}); a perfect car at the cap would take "
                 f"{g['ideal']:.1f} s, at the panda's 2.0 about {(V_END - V_START) / PANDA_MAX:.1f} s.")
    verdict = []
    car, tg = g['car'], g['tsk_gap']
    if share(g, 'CAP') >= 0.5:
      if (car != car or car >= 0.9) and (tg != tg or tg <= 0.15):
        most = ' on most pickups' if g['n_carlow'] or g['n_tskgap'] else ''
        verdict.append(f'the planner cap is the limit and{most} the car delivers it; {room_txt}')
      else:
        verdict.append('the planner cap holds the plan')
    verdict += car_verdict(g)
    if share(g, 'SET') >= 0.2 or g['set40'] >= max(1, (g['n'] + 2) // 3):
      verdict.append(f"the set speed was close: {g['set40']} of {g['n']} pickups started at 40 km/h (24.9 mph, SET pressed"
                     f" under 25 mph) and the MPC eases off from ~14 mph then ({share(g, 'SET') * 100:.0f}% of the time);"
                     ' a higher set speed or RES lets it keep pulling')
    if share(g, 'THR') >= 0.1:
      verdict.append(f"the model's gas prediction cut the throttle {share(g, 'THR') * 100:.0f}% of the time"
                     ' (allowThrottle; the cap then needs ~2 s to come back)')
    if share(g, 'TRN', 'SLW') >= 0.15:
      verdict.append(f"turns (steering angle) cut the cap {share(g, 'TRN', 'SLW') * 100:.0f}% of the time")
    if share(g, 'MPC', 'RMP') >= 0.3:
      verdict.append(f"the MPC planned under the cap on its own {share(g, 'MPC', 'RMP') * 100:.0f}% of the time")
    if share(g, 'SP') >= 0.1:
      verdict.append(f"a sunnypilot speed target (SCC / SLA) was lower {share(g, 'SP') * 100:.0f}% of the time")
    if share(g, 'E2E') >= 0.1:
      verdict.append(f"Experimental mode's model accel was lower {share(g, 'E2E') * 100:.0f}% of the time")
    if verdict:
      out.append('  So: ' + '; '.join(verdict) + '.')
  else:
    out.append('No pickup without a car ahead in these drives.')
  g = dict(groups).get('lead')
  if g and g['n']:
    cv = car_verdict(g)
    out.append(f"Behind a car ({g['n']} pickup{'s' if g['n'] != 1 else ''}): {share(g, 'LEAD') * 100:.0f}% of the time the car ahead held it back"
               f" (at or under the MPC's wanted gap {pcts(g['close'], 1)} of the time), the cap {share(g, 'CAP') * 100:.0f}%;"
               f" plan {fmt(g['plan_med'], 2, 1)}/{fmt(g['plan_max'], 2, 1)}; car {pcts(g['car'], 1)} of what was sent"
               + ('; ' + '; '.join(cv) if cv else '') + '.')
  return out


def print_detail(i, pk):
  d = pk['det']
  say()
  say(f"==== PICKUP {i} (drive {pk['rid']} at {pk['clock']}): {lead_txt(pk)}"
      + (f" (gap {pk['gap']:.0f} m median)" if pk['lead'] and pk['gap'] == pk['gap'] else '') + f", set {set_txt(pk)} mph"
      + (f" (openpilot switched on {pk['eng_ago']:.0f} s before)" if pk['eng_ago'] == pk['eng_ago'] else '')
      + f", {result_txt(pk)}; {CAP_NAMES[pk['prof']]}" + (f" {pk['pmax']:.2f}" if pk['prof'] == 'pickup' else '')
      + (f"; grade {pk['grade']:.0f}%" if pk['grade'] == pk['grade'] else ''))
  say('      t   mph  set  plan   MPC   cap  sent  TSK wheels  aEgo  lead   rpm  engNm(asked) st gear thr steer  limit')
  for j in range(len(d['t'])):
    lev = GEAR.get(int(d['lever'][j]), '?') if d['lever'][j] == d['lever'][j] else '?'
    gear = f"{lev}{int(d['gear'][j]) if d['gear'][j] == d['gear'][j] else ''}"
    allow = '.' if d['allow'][j] != d['allow'][j] else ('ok' if d['allow'][j] > 0.5 else 'OFF')
    stt = str(int(d['st'][j])) if d['st'][j] == d['st'][j] else '-'
    say(f"  {d['t'][j]:+5.2f} {fmt(d['mph'][j], 1, 5)} {fmt(d['set'][j], 0, 4)} {fmt(d['plan'][j])} {fmt(d['mpc'][j])}"
        f" {fmt(d['cap'][j])} {fmt(d['sent'][j])} {fmt(d['tsk'][j], 1, 4)}  {fmt(d['whl'][j])} {fmt(d['aego'][j])}"
        f" {fmt(d['lead'][j], 0, 5)} {fmt(d['rpm'][j], 0, 5)}  {fmt(d['eng'][j], 0, 4)}({fmt(d['eng_req'][j], 0, 4)})"
        f"  {stt:>2} {gear:>4} {allow:>3} {fmt(d['ang'][j], 0, 5)}  {CODES[int(d['code'][j])] if d['t'][j] >= 0 else ''}")
  say('  (plan = aTarget, MPC = its wish before the cap, cap = the planner clip rebuilt from the log, st = TSK_Status,'
      ' gear = lever + DSG target gear, thr = allowThrottle, steer = wheel angle deg)')


def pace(pk):
  """Mean accel of a pickup in m/s2 (from 5 mph to where it ended)."""
  return (pk['v_end'] - V_START) / max(pk['dur'], 0.1)


def spread(idx, k):
  """k items of idx evenly spaced from the first to the last (all of them if they fit)."""
  if len(idx) <= k:
    return list(idx)
  if k <= 1:
    return list(idx[:k])
  return [idx[round(j * (len(idx) - 1) / (k - 1))] for j in range(k)]


def paste_pick(picks, room):
  """The pickups that get a P line, in time order: all if they fit, else the no-lead ones first, spread from the
  slowest to the fastest (so every part of the range and not only the oldest drive shows), then lead ones."""
  if len(picks) <= room:
    return list(range(len(picks)))
  nl = sorted((i for i in range(len(picks)) if not picks[i]['lead']), key=lambda i: pace(picks[i]))
  ld = sorted((i for i in range(len(picks)) if picks[i]['lead']), key=lambda i: pace(picks[i]))
  out = spread(nl, room)
  out += spread(ld, room - len(out))
  return sorted(out)


def all_notes(R):
  """The drives' notes, most important first and without repeats, and [(message, [drives])] for car messages
  that were not on the bus."""
  notes = sorted({n for r in R for n in r['notes']})
  missing = []
  for m in MISSING_TXT:
    ds = [r['rid'] for r in R if m in r['missing']]
    if ds and not (m == 'Gateway_73' and not any('Getriebe_11' in r['missing'] for r in R if r['rid'] in ds)):
      missing.append((m, ds))
  return notes, missing


def paste_block(R, picks, groups, cap_info, notes=(), missing=()):
  say()
  say('===== PASTE THIS TO CLAUDE (start) =====')
  lines = [f"mk7_pickup_check v{TOOL_VERSION.split()[0]} drives={','.join(r['rid'] for r in R)} pickups={len(picks)}"
           f" pickup_file_now={cap_info}"]
  for r in R:
    lines.append(f"D {r['rid']} {r['branch']}@{r['commit']} segs={r['segs']}/{r['nseg']} picks={r['n']} cap={r['prof']}"
                 + (f"{r['pmax']:.2f}" if r['prof'] == 'pickup' else '') + f"({r['how']}/n{r['n_clip']})"
                 + f" pers={r['pers']} exp={pcts(r['exp'], 1)} skip={r['skip']}" + (f" mism={r['mism']}" if r['mism'] else '')
                 + (' ' + r['src'] if r is R[0] or r['src'] != R[0]['src'] else ''))
  for name, g in groups:
    if not g['n']:
      continue
    key = name.replace(' ', '')
    lines.append(f"MED {key} n={g['n']} reached={g['reached']} t525={fmt(g['t525'], 1, 1)} ideal={fmt(g['ideal'], 1, 1)}"
                 f" t520={fmt(g['t520'], 1, 1)}/n{g['n20']} plan520={fmt(g['plan520'], 1, 1)} ideal20={fmt(g['ideal20'], 1, 1)}"
                 f" cost=plan{fmt(g['plan_cost'], 1, 1, plus=True)}(max{fmt(g['plan_cost_max'], 1, 1, plus=True)})"
                 f"/car{fmt(g['car_cost'], 1, 1, plus=True)}(max{fmt(g['car_cost_max'], 1, 1, plus=True)})"
                 f" set0={fmt(g['set0'], 0, 1)} set40={g['set40']} plan={fmt(g['plan_med'], 2, 1)}/{fmt(g['plan_max'], 2, 1)}"
                 f" mpc={fmt(g['mpc_max'], 2, 1)} throff={fmt(g['thr_off'], 2, 1)} close={fmt(g['close'], 2, 1)}"
                 f" taken={g['taken']}")
    lines.append(f"CAR {key} sent={fmt(g['sent_max'], 2, 1)} tsk={fmt(g['tsk_max'], 1, 1)} tskgap={fmt(g['tsk_gap'], 2, 1)}"
                 f" tsk!3={fmt(g['tsk_not3'], 2, 1)} tsk5={fmt(g['tsk5'], 2, 1)}"
                 f" eng={eng_txt(g['eng_req'], g['eng_got']).replace(' ', '')} engshort={fmt(g['eng_short'], 0, 1)}"
                 f" rpm={fmt(g['rpm'], 0, 1)} gear={gear_txt(g['lever'], g['gear_lo'], g['gear_hi'])} D/S={g['D']}/{g['S']}"
                 f" whl={fmt(g['whl_med'], 2, 1)}/{fmt(g['whl_max'], 2, 1)} aego={fmt(g['aego_med'], 2, 1)}"
                 f" car={fmt(g['car'], 2, 1)} cartsk={fmt(g['car_tsk'], 2, 1)} short={fmt(g['car_short'], 2, 1)}"
                 f" npk=car:{g['n_carlow']},tsk:{g['n_tskgap']},eng:{g['n_engshort']},st5:{g['n_tsk5']}")
    lines.append(f"LIM {key} secs={g['secs']:.0f} " + ' '.join(f"{CODES[i]}={g['counts'][i] * 100.0 / max(g['counts'].sum(), 1):.0f}"
                                                            for i in range(len(CODES)) if g['counts'][i]))
  # notes: the important ones first (at most 3), the car messages that are not on the bus in one line
  nl = [n for _, n in notes][:3]
  if len(notes) > 3:
    nl.append(f'(+{len(notes) - 3} more notes in the full output above)')
  if missing:
    nl.append('not on the bus: ' + ' '.join(f"{m}({'all' if len(ds) == len(R) else ','.join(ds)})" for m, ds in missing))
  room = max(28 - len(lines) - len(nl) - 1, 4)   # the block stays at about 30 lines
  order = paste_pick(picks, room)
  for i in order:
    pk = picks[i]
    lines.append(f"P{i + 1} {pk['rid']} {pk['clock']} {'L' if pk['lead'] else 'N'}{pct(pk['lead_share'], 1)}"
                 + (f"c{pct(pk['close'], 1)}" if pk['lead'] else '')
                 + f" set={set_txt(pk)} end={pk['why'].replace(' ', '')}@{pk['v_end'] * MPH:.0f}/{pk['dur']:.1f}s"
                 f" t520={fmt(pk['t520'], 1, 1)}/{fmt(pk['plan520'], 1, 1)}"
                 f" plan={fmt(pk['plan_med'], 2, 1)}/{fmt(pk['plan_max'], 2, 1)} mpc={fmt(pk['mpc_max'], 2, 1)}"
                 f" sent={fmt(pk['sent_max'], 2, 1)} tsk={fmt(pk['tsk_max'], 1, 1)} tg={fmt(pk['tsk_gap'], 2, 1)}"
                 f" whl={fmt(pk['whl_med'], 2, 1)}/{fmt(pk['whl_max'], 2, 1)} car={fmt(pk['car'], 2, 1)}"
                 f" thr={fmt(pk['thr_off'], 2, 1)} st={tsk_txt(pk).replace(' ', ',')}"
                 f" eng={eng_txt(pk['eng_req'], pk['eng_got']).replace(' ', '')}"
                 f" {gear_txt(pk['lever'], pk['gear_lo'], pk['gear_hi'])} g={fmt(pk['grade'], 0, 1)} "
                 + top_codes(pk['codes'], n=4, sep=',').replace(' ', '').replace('%', ''))
  if len(picks) > len(order):
    lines.append(f'... {len(picks) - len(order)} more pickups not shown (lead ones left out first; the ones shown span'
                 ' the slowest to the fastest)')
  lines += ['note: ' + n for n in nl]
  for ln in lines:
    say(ln)
  say('===== PASTE THIS TO CLAUDE (end) =====')


# ============================================================================================ main
def pickup_file_now():
  try:
    with open(PICKUP_FILE) as f:
      val = float(f.read().strip())
  except (OSError, ValueError):
    return PICKUP_DEFAULT, 'nofile'
  if not math.isfinite(val):
    return PICKUP_DEFAULT, 'bad'
  val = min(max(val, 0.0), PICKUP_LIMIT)
  return val, f'{val:.2f}'


def main():
  global np, PICKUP_FILE
  p = argparse.ArgumentParser(description='Read-only: what held the accel back on every pickup from 5 to 25 mph.')
  p.add_argument('drives', nargs='*', default=['0'], help='0 = newest (default), 1 = the one before, 0-4 = a range,'
                 ' or a route-name prefix; several are pooled')
  p.add_argument('--detail', type=int, help='show pickup N in detail (4 rows per second)')
  p.add_argument('--all', action='store_true', help='show every pickup in detail')
  p.add_argument('--pickup', type=float, help='the /data/pickup_accel value during these drives (default: seen in the'
                 ' logs, else the file now, else 1.8)')
  p.add_argument('--cap', choices=('auto', 'pickup', 'stock', 'cap17'), default='auto',
                 help='the planner cap the drives had (default: from the branch and the logs)')
  p.add_argument('--car-on', action='store_true', help='run even though the car is on (not recommended)')
  p.add_argument('--log-root', default=ROOT, help=argparse.SUPPRESS)
  p.add_argument('--op-dir', default=OP_DIR, help=argparse.SUPPRESS)
  p.add_argument('--pickup-file', default=PICKUP_FILE, help=argparse.SUPPRESS)
  args = p.parse_args()
  try:
    with open(ONROAD_PARAM, 'rb') as f:
      onroad = f.read().strip() in (b'1', b'\x01')
  except OSError:
    onroad = False
  if onroad and not args.car_on:
    raise UsageError('The car is on. Switch the ignition off first (the comma stays on and you can still SSH in).')
  PICKUP_FILE = args.pickup_file
  rt = setup(args.op_dir)
  np = rt.np
  dbc, _ = load_dbc(rt, args.op_dir)
  want = {dbc[m][0] for m in {m for _, m, _ in CAR + OP} if m in dbc}
  routes, segs = routes_on_disk(args.log_root)
  if not routes:
    raise UsageError(f'No drives found in {args.log_root}.')
  say(f'mk7_pickup_check {TOOL_VERSION} (read-only)  newest {LIST_N} drives (0 = newest):')
  for i, r in enumerate(routes[:LIST_N]):
    b = route_branch(rt, args.log_root, r, segs)
    say(f'  {i} {r} segs {len(segs[r]):3d} {b[0]}@{b[1]}')
  chosen = pick_routes(args.drives, routes)
  pmax_file, file_txt = pickup_file_now()
  if args.pickup is not None:
    pmax_file, file_txt = min(max(args.pickup, 0.0), PICKUP_LIMIT), f'{args.pickup:.2f}(--pickup)'
  picks, R = [], []
  for route in chosen:
    t_start = time.time()
    can, svc, info = read_route(rt, args.log_root, route, segs, want)
    D = build(rt, dbc, can, svc, info)
    del can, svc
    head = (f"=== drive {route}  {info['branch']}@{info['commit']}  {info['segs']}/{info['nseg']} segments read"
            + (f", {info['no_rlog']} without rlog" if info['no_rlog'] else '')
            + (f", {info['bad']} damaged messages" if info['bad'] else ''))
    if D is None or len(D.lp_t) < 20:
      say(head + ': no carState / longitudinalPlan in the rlogs, skipped')
      continue
    D.route = route
    plan_inputs(D)
    prior = args.cap if args.cap != 'auto' else BRANCH_CAP.get(info['branch'], 'pickup')
    if args.cap != 'auto':
      prof, pmax, how, n_clip = args.cap, pmax_file, 'given', 0
    else:
      prof, pmax, how, n_clip = infer_cap(D, prior, pmax_file)
      if info['branch'] not in BRANCH_CAP and how != 'seen':
        D.notes.append((0, f"drive {rid(route)}: branch {info['branch']} unknown and the plan was never clearly clipped:"
                           f" assumed the {CAP_NAMES[prof]}"))
    if args.pickup is not None and prof == 'pickup':
      pmax, how = pmax_file, 'given'
    rebuild_caps(D, prof, pmax)
    pers = [int(x) for x in D.ss_pers[D.ss_en] if x >= 0] if len(D.ss_t) else []
    pers_name = PERS_NAME.get(max(set(pers), key=pers.count), '?') if pers else '?'
    lead_inputs(D, T_FOLLOW.get(pers_name, 1.25))
    set_inputs(D)
    classify(D)
    found = find_pickups(D)
    pk_route = [analyse(D, pk, prof, pmax) for pk in found]
    ideal, ideal20 = ideal_time(np, prof, pmax), ideal_time(np, prof, pmax, V_20)
    for pk in pk_route:
      pk['ideal'], pk['ideal20'] = ideal, ideal20
    picks += pk_route
    exp = float(np.mean(D.ss_exp[D.ss_en])) if len(D.ss_t) and D.ss_en.any() else NAN
    mism = sum(pk['mism'] for pk in pk_route)
    if mism > 0.1 * max(sum(pk['nf'] for pk in pk_route), 1):
      D.notes.append((0, f"drive {rid(route)}: {mism} plan frames were clipped at another level than the rebuilt cap"
                     " (pass --pickup X or --cap)"))
    if exp == exp and exp > 0.05:
      D.notes.append((0, f"drive {rid(route)}: Experimental mode on {exp * 100:.0f}% of the engaged time"))
    dyn = [pk['dyn'] for pk in pk_route if pk['dyn'] == pk['dyn'] and pk['dyn'] > 0.5]
    if dyn:
      D.notes.append((1, f'drive {rid(route)}: ACC_06 ACC_DynErhoehung was set on {len(dyn)} pickups'))
    skip = ','.join(f"{k.replace(' ', '_')}:{v}" for k, v in D.skipped.items() if v) or '0'
    skip_txt = ', '.join(f'{v} {k}' for k, v in D.skipped.items() if v)
    R.append({'rid': rid(route), 'branch': info['branch'], 'commit': info['commit'], 'segs': info['segs'],
              'nseg': info['nseg'], 'n': len(pk_route), 'prof': prof, 'pmax': pmax, 'how': how, 'n_clip': n_clip,
              'pers': pers_name, 'exp': exp, 'skip': skip, 'mism': mism, 'notes': D.notes, 'missing': D.missing,
              'src': 'src ' + ' '.join(f'{m}={s.replace(" ", "")}' for m, s in sorted(D.srcs.items()))})
    cap_txt = CAP_NAMES[prof] + (f' {pmax:.2f}' if prof == 'pickup' else '')
    say(head + f": {len(pk_route)} pickups" + (f' (left out: {skip_txt})' if skip_txt else '')
        + f", {cap_txt} ({how}), personality {pers_name}, read in {time.time() - t_start:.0f} s")
    del D
  if not R:
    raise UsageError('None of the chosen drives has rlogs with carState and longitudinalPlan.')
  if len({r['src'] for r in R}) == 1:
    say('signals: ' + R[-1]['src'][4:])
  else:
    for r in R:
      say(f"signals drive {r['rid']}: {r['src'][4:]}")
  notes, missing = all_notes(R)
  for _, n in notes:
    say('note: ' + n)
  for m, ds in missing:
    say(f"note: {m} is not in the logs of {'these drives' if len(ds) == len(R) else 'drive ' + ','.join(ds)}: {MISSING_TXT[m]}")
  say(f'/data/pickup_accel now: {file_txt} (only used when a drive never clipped clearly)')
  say()
  if not picks:
    say('No pickup from 5 mph with openpilot driving in these drives.')
    paste_block(R, picks, [], file_txt, notes, missing)
    return 1
  print_pickups(picks)
  say()
  groups = [('no lead', group_stats([pk for pk in picks if not pk['lead']])),
            ('lead', group_stats([pk for pk in picks if pk['lead']])),
            ('all', group_stats(picks))]
  print_medians(groups)
  say()
  profs = {(r['prof'], round(r['pmax'], 2) if r['prof'] == 'pickup' else 0) for r in R}
  if len(profs) == 1:
    pr, pm = next(iter(profs))
    cap_txt, pmax_txt = CAP_NAMES[pr], (f'{pm:.2f} m/s2' if pr == 'pickup' else 'falls with speed')
  else:
    cap_txt, pmax_txt = 'mixed caps', 'see the D lines'
  if all(pr == 'pickup' for pr, _ in profs):
    pm = max(pm for _, pm in profs)
    room_txt = (f'the pickup cap is already at its {PICKUP_LIMIT} limit (the panda allows 2.0): no room left in the cap'
                if pm >= PICKUP_LIMIT - 0.01 else
                f'the only software room left is the cap itself (now {pm:.2f}; up to {PICKUP_LIMIT} in /data/pickup_accel,'
                ' under the panda\'s 2.0)')
  elif not any(pr == 'pickup' for pr, _ in profs):
    room_txt = ('the software room left is the cap itself (these drives had no pickup cap; the pickup cap on'
                f' t2_close_follow allows up to {PICKUP_LIMIT})')
  else:
    room_txt = f'the software room left is the cap itself (the drives had different caps: see the D lines; max {PICKUP_LIMIT})'
  say('BOTTOM LINE')
  for ln in bottom_line(groups[:2], cap_txt, pmax_txt, room_txt):
    say(ln)
  if args.detail:
    if 1 <= args.detail <= len(picks):
      print_detail(args.detail, picks[args.detail - 1])
    else:
      say(f'There is no pickup {args.detail}; there are {len(picks)}.')
  elif args.all:
    for i, pk in enumerate(picks, 1):
      print_detail(i, pk)
  else:
    nl = [pk for pk in picks if not pk['lead']] or picks
    # the slowest one that openpilot drove to its end (not one the driver took over or that slowed down again)
    worst = min([pk for pk in nl if pk['why'] in ('25', 'set', 'limit', '25 s')] or nl, key=pace)
    say()
    say('The slowest pickup with no car ahead in detail (another with --detail N, all with --all):')
    print_detail(next(i for i, pk in enumerate(picks, 1) if pk is worst), worst)
  paste_block(R, picks, groups[:2], file_txt, notes, missing)
  return 0


np = None

if __name__ == '__main__':
  try:
    sys.exit(main())
  except UsageError as e:
    say(f'ERROR: {e}')
    sys.exit(2)
  except KeyboardInterrupt:
    sys.exit(130)
  except Exception:   # a bug: exit 2 (error), never 1 (= "no pickups")
    import traceback
    sys.stdout.flush()
    traceback.print_exc()
    say(f'ERROR: mk7_pickup_check {TOOL_VERSION} failed (traceback above). Paste the last 20 lines to Claude.')
    sys.exit(2)
