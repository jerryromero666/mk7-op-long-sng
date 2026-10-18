#!/usr/bin/env python3
"""
mk7_stop_check.py - read-only: how openpilot brought the car to a stop, and every time the driver had to brake (car OFF).

INSTALL OR UPDATE (on the comma, over SSH; nothing is replaced if the download fails):
  cd /data/openpilot && git fetch sngrepo t2_set_min && git show FETCH_HEAD:mk7_stop_check.py > /data/mk7_stop_check.py.new && mv /data/mk7_stop_check.py.new /data/mk7_stop_check.py

RUN IT WITH THE CAR OFF:

  python3 /data/mk7_stop_check.py 0-4             the 5 newest drives together (0 = newest)
  python3 /data/mk7_stop_check.py                 newest drive
  python3 /data/mk7_stop_check.py 0 1 3           drives 0, 1 and 3
  python3 /data/mk7_stop_check.py 00000343        a drive can also be a route-name prefix
  python3 /data/mk7_stop_check.py 0-4 --detail 7  event 7 in detail (10 rows per second near the stop / press)
  python3 /data/mk7_stop_check.py 0-4 --all       every event in detail

The last block of the output, between "PASTE THIS TO CLAUDE (start)" and "(end)", is meant to be copied back as it is.

WHAT IT LOOKS AT
 A. Every stop by openpilot: the car's speed (vEgo) under 0.1 m/s for 0.5 s with openpilot's long control on in the 3 s
    before and still on the last time the car was at 5 mph or faster (a stop the driver took over above 5 mph is only a
    brake press, part B). For the approach (up to 20 s back) it shows the shape of the braking in six signals:
    openpilot's plan (longitudinalPlan.aTarget), its output (carControl accel = what the controller asked; VW: the same
    as the plan while driving, gains 0), what went out on the bus (ACC_06), the cruise computer's own target (Motor_20),
    the wheels (slope of the ESP_19 wheel speeds) and carState.aEgo. Numbers per stop:
      braking starts  first time the output went under -0.3 m/s2 in the final approach (gaps under 1 s bridged): speed,
                      seconds before the stop, the car ahead there (gap, its speed, time to collision)
      peak            the strongest braking asked, at what speed, how long before the stop; the steepest build-up (m/s3)
      release         after the peak, how far the braking eased (m/s2), for how long, at what speed
      into stopping   openpilot's longControlState pid -> stopping: when, at what speed, and the output it carried in.
                      The stopping state KEEPS that output to the end if it is stronger than VW's stopAccel -1.2 (it only
                      ramps toward -1.2 when it is weaker), so a stop that hands over at -2.5 stays at -2.5 until the
                      wheels stop
      last 1 s        the strongest braking asked / at the wheels in the last second, the wheels' braking at the moment
                      of the stop (the bump) and the steepest jerk there; whether the car rolled with the brakes eased
      also            shouldStop, ACC_06 / ACC_07 ACC_Anhalten, ACC_07 ACC_Anforderung_HMS changes, ESP_21 hold
                      confirmation (bit 53) and ACC stopping procedure (bit 63), the gap to the car ahead at the stop,
                      whether the driver pressed the brake
    Shape labels (constants below): hard-release-grab = peak at -2.0 or stronger, then the braking eased by 0.8 or more
    for 0.3 s or more, then grabbed again by 0.5 or more; late-hard = peak at -2.0 or stronger and the whole stop
    averaged 1.5 m/s2 or more from where braking started (started late, so it had to be hard); hard-into-stopping = the
    stopping state took over at -1.8 or stronger (and the stop was neither of the above); hard-eased = peak at -2.0 or
    stronger that eased into the stop; smooth = the rest; creep = a stop after rolling under 5 mph (left out of the
    numbers). The same rules on the wheels give the 'car' label ('into stopping' there = still braking at -1.8 or
    stronger in the last 0.3 s before the wheels stopped).
 B. Every driver brake press while openpilot drove (carState.brakePressed going on while long control was on or within
    0.2 s before; a press that switches openpilot off counts): speed, set speed and how it was set, what openpilot asked
    at the press and in the 3 s before, the wheels, the car ahead, the MPC's wanted gap, ACC_06 frames the panda blocked
    within 1 s, and what happened next. Classes: 'lead, OP braking too little', 'lead, OP braking late/tight', 'no lead
    (red light / stop sign / turn?)', 'OP accelerating', 'end of a stop, under 5 mph' (the car had been faster than
    5 mph since its last stop and stopped within 5 s: e.g. a touch while it rolled at the end), 'creeping under 5 mph'
    (the rest under 5 mph), 'other'. Presses at a standstill are only counted.
 C. Per branch: driving time, stops and presses per hour of openpilot driving, the shape labels, medians and worst cases,
    how often the car was above 25 / 30 / 35 mph, and panda-blocked ACC_06 frames right after a press vs elsewhere.
Only reads logs. Writes nothing. Exit codes: 0 = report printed, 1 = no openpilot stop and no brake press found, 2 = problem.
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

TOOL_VERSION = '1.1 (2026-10-18)'
OP_DIR = '/data/openpilot'
ROOT = '/data/media/0/realdata'
VENV_PYTHON = '/usr/local/venv/bin/python3'
REEXEC_FLAG = 'MK7_STOP_CHECK_REEXEC'
ONROAD_PARAM = '/data/params/d/IsOnroad'
DBC_REL = ('opendbc_repo/opendbc/dbc/vw_mqb.dbc', 'opendbc/dbc/vw_mqb.dbc')
ZSTD_MAGIC = b'\x28\xb5\x2f\xfd'
LIST_N = 8
NAN = float('nan')
MPH = 2.23694
PASTE_MAX = 30         # lines between the paste markers

DT = 0.02              # s: analysis grid (the car's messages come at 50 Hz)
# ---- a stop
STILL_V = 0.10         # m/s: stopped = vEgo under this ...
STILL_S = 0.5          # ... for at least this long
BLIP_S = 0.2           # a move shorter than this inside a stop is ignored
LONG_BEFORE_S = 3.0    # openpilot's long control on within this before the stop = an openpilot stop
LOOK_BACK_S = 20.0     # s: the approach is searched this far back (not past the previous stop)
SHAPE_S = 12.0         # s: the detail timeline shows this much before the stop
AFTER_S = 3.0          # s after the stop that are looked at (hold confirmation, rolling)
CREEP_TOP = 2.24       # m/s (5 mph): a stop whose speed never passed this since the previous one = creep stop
BRAKE_ON = -0.3        # m/s2: openpilot's output under this = braking (the start of the approach)
BRAKE_GAP_S = 1.0      # s: shorter breaks inside the braking are bridged
JERK_S = 0.2           # s: jerk = change over this long
# ---- shape labels (on openpilot's output; the same on the wheels for the 'car' label)
HARD_PEAK = -2.0       # m/s2: peak at this or stronger = a hard stop
RELEASE_DEPTH = 0.8    # m/s2: after the peak the braking eased by at least this ...
RELEASE_S = 0.3        # s: ... for at least this long ...
GRAB_DEPTH = 0.5       # m/s2: ... and then grew again by at least this before the stop = hard-release-grab
LATE_MEAN = 1.5        # m/s2: mean braking from the braking start to the stop at least this (with a hard peak) = late-hard
INTO_STOP_HARD = -1.8  # m/s2: the stopping state took over at this or stronger = hard-into-stopping
AT_STOP_S = 0.3        # s: the wheels' braking at the stop = their mean over the last 0.3 s before it
FINAL_S = 1.0          # s: the 'last second' before the stop
ROLL_DECEL = -0.5      # m/s2: wheels braking weaker than this after a hard part (peak <= ROLL_AFTER) ...
ROLL_AFTER = -1.5
ROLL_S = 0.3           # ... for at least this long above ROLL_V = the car rolled with the brakes eased
ROLL_V = 0.9           # m/s (2 mph; slower is the normal soft end of a stop)
MOVED_V = 0.3          # m/s: moved again within 2.5 s after the stop without gas / openpilot starting
LABELS = ['smooth', 'late-hard', 'hard-release-grab', 'hard-into-stopping', 'hard-eased', 'creep']
LABEL_SHORT = {'smooth': 'sm', 'late-hard': 'lh', 'hard-release-grab': 'rg', 'hard-into-stopping': 'is',
               'hard-eased': 'he', 'creep': 'cr', '?': '?'}
LABEL_RANK = {'hard-release-grab': 4, 'hard-into-stopping': 3, 'late-hard': 3, 'hard-eased': 1, 'smooth': 0, '?': -1,
              'creep': -2}
# ---- a driver brake press
PRESS_LONG_S = 0.2     # long control on at the press or this long before it
STAND_V = 0.3          # m/s: a press slower than this = at a standstill (only counted)
CREEP_V = 2.24         # m/s (5 mph): slower = 'end of a stop, under 5 mph' or 'creeping under 5 mph'
LEAD_D = 80.0          # m: a car ahead counts only within this ...
LEAD_CLOSING = 0.5     # ... and closing by this (m/s), or within ROOM_NEAR m of the MPC's wanted gap, or braking
ROOM_NEAR = 5.0        # m
LEAD_BRAKING = -1.0    # m/s2
STOP_GAP = 4.0         # m: 'needed braking' = constant braking to stay this far behind the car ahead
TOO_LITTLE = 0.5       # m/s2: openpilot asked at least this much less braking than needed = 'too little'
NEED_MIN = 0.5         # m/s2: ... and the needed braking was at least this
OP_ACCEL = 0.2         # m/s2: openpilot asking more than this (0.5 s before the press) = 'OP accelerating'
LATE_BRK_S = 1.5       # s: openpilot had started braking less than this before the press ...
TIGHT_ROOM = -2.0      # m: ... or the gap was this far under the MPC's wanted gap ...
TIGHT_TTC = 4.0        # s: ... or the time to collision was under this ...
TIGHT_D = 6.0          # m: ... or the gap was under this = 'lead, OP braking late/tight'
STOP_AFTER_S = 20.0    # s: did the car stop within this after the press
BLOCK_AFTER = (-0.2, 1.0)   # s: a blocked frame this close after a brake press is the expected one-to-three
CLASSES = ['lead, OP braking too little', 'lead, OP braking late/tight', 'no lead (red light / stop sign / turn?)',
           'OP accelerating', 'end of a stop, under 5 mph', 'creeping under 5 mph', 'other']
CLASS_SHORT = {CLASSES[0]: 'tl', CLASSES[1]: 'lt', CLASSES[2]: 'nl', CLASSES[3]: 'acc', CLASSES[4]: 'es', CLASSES[5]: 'cr',
               CLASSES[6]: 'ot'}
ES_STOP_S = 5.0        # s: a press under 5 mph after driving faster is 'end of a stop' when the car stopped within this
DRV_LATE_V = 0.9       # m/s (2 mph): a stop where the driver only touched the brake slower than this stays in the numbers
# ---- the MPC (selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py) and openpilot's VW settings
T_FOLLOW = {'aggressive': 1.25, 'standard': 1.45, 'relaxed': 1.75}
STOP_ACCEL, V_STOPPING, ACCEL_MIN, ACCEL_MAX = -1.2, 0.5, -3.5, 2.0
INACTIVE_ACCEL = 3.01
BTN = {'setCruise': 1, 'decelCruise': 1, 'resumeCruise': 2, 'accelCruise': 2}   # 1 = SET-type, 2 = RES-type
PERS = {'aggressive': 0, 'standard': 1, 'relaxed': 2}
PERS_NAME = {v: k for k, v in PERS.items()}
LCS = {'off': 0, 'pid': 1, 'stopping': 2, 'starting': 3}
LCS_SHORT = {0: 'off', 1: 'pid', 2: 'stop', 3: 'strt'}
HMS_TXT = {0: 'none', 1: 'hold request', 2: 'park', 3: 'standby', 4: 'release/launch', 5: 'ramp release'}

# key, message, signal (car -> comma, read from 'can' on the car's bus)
CAR = [
  ('ws_fl', 'ESP_19', 'ESP_VL_Radgeschw_02'),
  ('ws_fr', 'ESP_19', 'ESP_VR_Radgeschw_02'),
  ('ws_rl', 'ESP_19', 'ESP_HL_Radgeschw_02'),
  ('ws_rr', 'ESP_19', 'ESP_HR_Radgeschw_02'),
  ('tsk_target', 'Motor_20', 'TSK_a_Soll_gradientenbegrenzt'),
  ('hold53', 'ESP_21', 'ESP_Haltebestaetigung'),
  ('esp63', 'ESP_21', 'ESP_Anhaltevorgang_ACC_aktiv'),
]
VALID = {'ws_fl': (0.0, 400.0), 'ws_fr': (0.0, 400.0), 'ws_rl': (0.0, 400.0), 'ws_rr': (0.0, 400.0)}
# key, message, signal (comma -> car: the panda's echo of what went out, else sendcan)
OP = [
  ('sent', 'ACC_06', 'ACC_Sollbeschleunigung_02'),
  ('a06_stop', 'ACC_06', 'ACC_Anhalten'),
  ('a06_go', 'ACC_06', 'ACC_Anfahren'),
  ('hms', 'ACC_07', 'ACC_Anforderung_HMS'),
  ('a07_stop', 'ACC_07', 'ACC_Anhalten'),
  ('a07_go', 'ACC_07', 'ACC_Anfahren'),
]
OP_MSGS = ('ACC_06', 'ACC_07')
MISSING_TXT = {'ESP_19': 'wheel speeds (the carState.vEgo slope is used)', 'Motor_20': 'TSK target',
               'ESP_21': 'ESP hold confirmation (bit 53) / ACC stopping procedure (bit 63)',
               'ACC_06': 'ACC_06 as sent (carControl accel is used)', 'ACC_07': 'ACC_07 HMS / Anhalten'}
DENSE_GAP = 0.25       # s: a CAN frame with no neighbour this close is a qlog leftover, not a signal (dropped)


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


def read_route(rt, root, route, segs, want, op_addrs):
  """Only the fields needed, all segments of one drive (rlog; qlog where a segment has no rlog). Times in s.
  CAN keyed by (stream, address, src): car messages from buses 0-3, ACC_06 / ACC_07 only as sent / echoed / blocked."""
  np = rt.np
  T = {'cs': Table(7), 'cc': Table(5), 'ctl': Table(2), 'lp': Table(5), 'rs': Table(5), 'ss': Table(4), 'btn': Table(2)}
  cs_add, cc_add, ctl_add, lp_add = T['cs'].rows.append, T['cc'].rows.append, T['ctl'].rows.append, T['lp'].rows.append
  rs_add, ss_add, btn_add = T['rs'].rows.append, T['ss'].rows.append, T['btn'].rows.append
  can = {}
  info = {'route': route, 'branch': '?', 'commit': '?', 'delay': NAN, 'stop_accel': NAN, 'v_stopping': NAN,
          'decel_rate': NAN, 'segs': 0, 'q_segs': 0, 'none': 0, 'bad': 0, 'nseg': len(segs[route])}
  from_bytes = int.from_bytes
  ss_n = cs_n = cc_n = ctl_n = 0
  last_brk = last_lcs = last_la = last_ctl = None
  lcs_get = LCS.get
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
          t = e.logMonoTime * 1e-9
          is_can = w == 'can'
          for f in (e.can if is_can else e.sendcan):
            a = f.address
            if a in want:
              dat = f.dat
              if len(dat) <= 8:
                src = f.src
                if is_can and (src >= 4 if a not in op_addrs else src < 128):
                  continue
                k = (w, a, src)
                b = can.get(k)
                if b is None:
                  b = can[k] = CanBuf()
                b.t.append(t)
                b.d.append(from_bytes(dat, 'little'))
        elif w == 'carState':
          c = e.carState
          cs_n += 1
          brk, gas = c.brakePressed, c.gasPressed
          be = c.buttonEvents
          if len(be):      # which button switched openpilot on: SET (set / minus) or RES (resume / plus)
            for b in be:
              k = BTN.get(str(b.type), 0)
              if k:
                btn_add((e.logMonoTime * 1e-9, k))
          if q or cs_n % 2 == 0 or brk or gas or brk != last_brk:   # 50 Hz, every pedal frame and every brake edge
            cs_add((e.logMonoTime * 1e-9, c.vEgo, c.aEgo, c.vCruise, gas, brk, c.standstill))
          last_brk = brk
        elif w == 'carControl':
          c = e.carControl
          cc_n += 1
          la = c.longActive
          lcs = lcs_get(str(c.actuators.longControlState), -1)
          if q or cc_n % 2 == 0 or lcs != last_lcs or la != last_la:
            cc_add((e.logMonoTime * 1e-9, la, c.actuators.accel, c.enabled, lcs))
          last_lcs, last_la = lcs, la
        elif w == 'controlsState':
          ctl_n += 1
          lcs = lcs_get(str(e.controlsState.longControlState), -1)
          if q or lcs != last_ctl or ctl_n % 25 == 0:
            ctl_add((e.logMonoTime * 1e-9, lcs))
          last_ctl = lcs
        elif w == 'longitudinalPlan':
          lp = e.longitudinalPlan
          lp_add((e.logMonoTime * 1e-9, lp.aTarget, lp.shouldStop, lp.hasLead, 0.0))
        elif w == 'radarState':
          l1 = e.radarState.leadOne
          rs_add((e.logMonoTime * 1e-9, l1.status, l1.dRel, l1.vLead, l1.aLeadK))
        elif w == 'selfdriveState':
          ss_n += 1
          if q or ss_n % 10 == 0:
            s = e.selfdriveState
            ss_add((e.logMonoTime * 1e-9, s.enabled, s.experimentalMode, PERS.get(str(s.personality), -1)))
        elif w == 'carParams' and info['delay'] != info['delay']:
          cp = e.carParams
          info['delay'], info['stop_accel'] = float(cp.longitudinalActuatorDelay), float(cp.stopAccel)
          info['v_stopping'], info['decel_rate'] = float(cp.vEgoStopping), float(cp.stoppingDecelRate)
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


def slope_grid(np, t, v, half=5):
  """Speed on a uniform DT grid and its least-squares slope over +-half samples (+-0.1 s, centered)."""
  if len(t) < 2 * half + 2:
    return np.zeros(0), np.zeros(0)
  g = np.arange(t[0], t[-1], DT)
  vg = grid_interp(np, t, v, g)
  k = np.arange(-half, half + 1, dtype=np.float64)
  a = np.correlate(vg, k / (np.sum(k * k) * DT), mode='same')
  a[:half] = NAN
  a[-half:] = NAN
  return g, a


def fin(x):
  x = np.asarray(x, dtype=np.float64)
  return x[np.isfinite(x)]


def nmin(x):
  f = fin(x)
  return float(f.min()) if len(f) else NAN


def nmax(x):
  f = fin(x)
  return float(f.max()) if len(f) else NAN


def nmean(x):
  f = fin(x)
  return float(f.mean()) if len(f) else NAN


def nargmin(x):
  x = np.asarray(x, dtype=np.float64)
  if not np.isfinite(x).any():
    return None
  return int(np.nanargmin(x))


def nargmax(x):
  x = np.asarray(x, dtype=np.float64)
  if not np.isfinite(x).any():
    return None
  return int(np.nanargmax(x))


def isnum(x):
  return x is not None and x == x and not (isinstance(x, float) and math.isinf(x))


def dense(np, x):
  """(t, words) without isolated frames: qlogs keep 1 CAN message in thousands, too few to follow a signal."""
  t, d = x
  if len(t) < 3:
    return t[:0], d[:0]
  g = np.diff(t)
  keep = np.concatenate([[False], g <= DENSE_GAP]) | np.concatenate([g <= DENSE_GAP, [False]])
  return (t, d) if keep.all() else (t[keep], d[keep])


def build(rt, dbc, can, svc, info):
  np = rt.np
  D = Drive()
  D.np, D.info, D.notes = np, info, []      # notes: (priority 0 = must reach the paste block .. 1, text)
  dr = rid(info['route'])
  cs = svc['cs']
  if len(cs) < 50:
    return None
  D.t0 = cs[0, 0]
  D.cs_t, D.v, D.aego, D.vcruise = cs[:, 0], cs[:, 1], cs[:, 2], cs[:, 3]
  D.gas, D.brk, D.stand = cs[:, 4] > 0.5, cs[:, 5] > 0.5, cs[:, 6] > 0.5
  cc = svc['cc']
  D.cc_t, D.long, D.cc_a, D.cc_en, D.cc_lcs = cc[:, 0], cc[:, 1] > 0.5, cc[:, 2], cc[:, 3] > 0.5, cc[:, 4]
  ctl = svc['ctl']
  # openpilot's longControlState: controlsState (the state after this frame's update) if it is in the logs, else
  # carControl.actuators (the same state one 10 ms frame later)
  if len(ctl) >= 10 and (ctl[:, 1] >= 0).any():
    D.lcs_t, D.lcs, D.lcs_src = ctl[:, 0], np.where(ctl[:, 1] >= 0, ctl[:, 1], NAN), 'controlsState'
  else:
    D.lcs_t, D.lcs, D.lcs_src = D.cc_t, np.where(D.cc_lcs >= 0, D.cc_lcs, NAN), 'carControl'
    if len(cc) >= 100:
      D.notes.append((1, f'drive {dr}: controlsState is not in the logs: longControlState from carControl (10 ms late)'))
  D.btn_t, D.btn = svc['btn'][:, 0], svc['btn'][:, 1]
  lp = svc['lp']
  D.lp_t, D.aT, D.sstop, D.has_lead = lp[:, 0], lp[:, 1], lp[:, 2], lp[:, 3]
  rs = svc['rs']
  D.rs_t, D.l1, D.dRel, D.vLead, D.aLead = rs[:, 0], rs[:, 1], rs[:, 2], rs[:, 3], rs[:, 4]
  ss = svc['ss']
  D.ss_t, D.ss_en, D.ss_exp, D.ss_pers = ss[:, 0], ss[:, 1] > 0.5, ss[:, 2] > 0.5, ss[:, 3]
  addr = {name: a for name, (a, _) in dbc.items()}
  n_cs = max(len(cs), 1)
  if len(cc) < 50:
    D.notes.append((0, f'drive {dr}: carControl is not in the logs: cannot tell when openpilot drove (no events)'))
  if len(lp) < 0.05 * n_cs:
    D.notes.append((0, f'drive {dr}: longitudinalPlan is (mostly) not in the logs: no plan / shouldStop'))
  if len(rs) < 0.05 * n_cs:
    D.notes.append((0, f"drive {dr}: radarState is (mostly) not in the logs: the car ahead is unknown (lead classes"
                       " cannot be told apart)"))
  if info['q_segs']:
    D.notes.append((0, f"drive {dr}: {info['q_segs']} of {info['segs']} segments only have a qlog: 10 per second (plan 2"
                       " per second) and no CAN there, so jerk, release and car-side numbers are coarse or missing"
                       " (events marked q)"))
  D.q_spans = []
  # car bus: the lowest bus that carries ESP_19 at a real rate; other car messages from there if they are on it
  e19 = {src: len(x[0]) for (s, a, src), x in can.items() if s == 'can' and a == addr.get('ESP_19') and src < 4}
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

  car_src = {k for k in can if k[0] == 'can' and k[2] < 4}
  for k in list(can):      # signals only from the dense parts (rlog); blocked frames (src 192+) are kept as they are
    if k[0] == 'sendcan' or k[2] < 192:
      can[k] = dense(np, can[k])
  D.c, D.srcs = {}, {}
  for key, msg, sname in CAR:
    k, x = pick(msg, car_src)
    s = dbc.get(msg, (None, {}))[1].get(sname)
    if x is None or s is None or len(x[0]) < 10:
      D.c[key] = (np.zeros(0), np.zeros(0))
      continue
    _, val = decode(np, x[1], s)
    if key in VALID:
      lo, hi = VALID[key]
      val[(val < lo) | (val > hi)] = NAN
    D.c[key] = (x[0], val)
    D.srcs[msg] = f'bus {k[2]}'
  # what OP sent: the panda's echo of the frames that went out (same clock as the car's frames), else sendcan
  D.blocked = []      # (t, message, accel in the blocked frame)
  for msg in OP_MSGS:
    a = addr.get(msg)
    snd = {k: x for k, x in can.items() if k[0] == 'sendcan' and k[1] == a}
    op_bus = max(snd, key=lambda k: len(snd[k][0]))[2] if snd else 0
    echo = can.get(('can', a, 128 + op_bus))
    n_snd = len(snd[('sendcan', a, op_bus)][0]) if snd else 0
    sig = dbc.get(msg, (None, {}))[1]
    for (s_, a_, src), xb in can.items():      # the panda's safety blocked these (pandad: src + 0xC0)
      if s_ == 'can' and a_ == a and src >= 192 and len(xb[0]) and 'ACC_Sollbeschleunigung_02' in sig:
        _, vb = decode(np, xb[1], sig['ACC_Sollbeschleunigung_02'])
        D.blocked += [(float(tb), msg, float(v_)) for tb, v_ in zip(xb[0], vb, strict=True)]
    # the echo only when openpilot sent this message itself (sendcan): with openpilot long off the panda forwards the
    # radar's own ACC_06 / ACC_07 to bus 0, and those come back as echoes too
    if echo is not None and n_snd >= 10 and len(echo[0]) >= 0.5 * n_snd:
      x, D.srcs[msg] = echo, f'panda echo bus {op_bus}'
    elif snd and n_snd >= 10:
      x, D.srcs[msg] = snd[('sendcan', a, op_bus)], f'sendcan bus {op_bus}'
    else:
      x = None
    for key, m2, sname in OP:
      if m2 != msg:
        continue
      s = sig.get(sname)
      if x is None or s is None:
        D.c[key] = (np.zeros(0), np.zeros(0))
        continue
      raw, val = decode(np, x[1], s)
      if key == 'sent':
        val[raw >= 2046] = NAN      # 3.01 = openpilot not asking / error codes
      D.c[key] = (x[0], val)
  D.blocked.sort()
  if not len(D.c['sent'][0]):
    D.c['sent'] = (D.cc_t, np.where(D.long, D.cc_a, NAN))
  # the TSK target is "not on this car" if it never changes while OP drives
  tt, tv = D.c['tsk_target']
  if len(tt):
    tv = np.where(tv > 5.4, NAN, tv)          # raw 127 = no value
    D.c['tsk_target'] = (tt, tv)
    on = hold(np, D.cc_t, D.long.astype(float), tt) > 0.5 if len(D.cc_t) else np.zeros(len(tt), bool)
    if on.sum() >= 50 and (not np.isfinite(tv[on]).any() or np.nanstd(tv[on]) < 0.05):
      D.notes.append((1, f'drive {dr}: Motor_20 TSK target never moved while openpilot drove: left out'))
      D.c['tsk_target'] = (np.zeros(0), np.zeros(0))
      D.srcs.pop('Motor_20', None)
  D.missing = [m for m in MISSING_TXT if m not in D.srcs]
  # the wheels: one wheel speed in m/s on the 50 Hz grid and its slope; carState.vEgo's slope where there is none
  ws = [D.c[k] for k in ('ws_fl', 'ws_fr', 'ws_rl', 'ws_rr')]
  if all(len(w[0]) > 20 for w in ws):
    tw = ws[0][0]
    vw = np.mean([np.interp(tw, w[0], w[1]) for w in ws], axis=0) / 3.6
    D.g_t, D.a_whl = slope_grid(np, tw, vw)
  else:
    D.g_t, D.a_whl = np.zeros(0), np.zeros(0)
  D.gv_t, D.a_v = slope_grid(np, D.cs_t, D.v)
  # personality (the MPC's follow time)
  pers = [int(x) for x in D.ss_pers[D.ss_en] if x >= 0] if len(D.ss_t) else []
  D.pers = PERS_NAME.get(max(set(pers), key=pers.count), '?') if pers else '?'
  D.t_follow = T_FOLLOW.get(D.pers, 1.25)
  return D


# ============================================================================================ engages, set speed
def engages(D):
  """Every switch-on: (t, speed m/s just before, set speed km/h just after, 'set' / 'res' / '?' = button not in the logs)."""
  np = D.np
  on = D.cc_en if D.cc_en.any() else D.long
  if len(on) < 2:
    return []
  on = on.astype(np.int8)
  out = []
  for t in D.cc_t[1:][np.diff(on) > 0]:
    v = hold(np, D.cs_t, D.v, [t], stale=0.5)[0]
    sk = hold(np, D.cs_t, D.vcruise, [t + 0.3], stale=1.0)[0]
    b = D.btn[(D.btn_t >= t - 1.0) & (D.btn_t <= t + 0.05)]
    kind = '?' if not len(b) else ('res' if b[-1] == 2 else 'set')
    out.append((float(t), float(v), float(sk) if sk == sk and 0 < sk < 254 else NAN, kind))
  return out


def set_at(D, t):
  """(set speed mph at t, code, words): code S0 = SET at a stop, Sr = SET rolling, R0 / Rr = RES, ?0 / ?r = button not
  in the logs; '+-' appended when the set speed was changed with +/- after switching on."""
  np = D.np
  sk = hold(np, D.cs_t, D.vcruise, [t], stale=1.0)[0]
  mph = sk / 3.6 * MPH if sk == sk and 0 < sk < 254 else NAN
  eng = [e for e in D.engages if e[0] <= t]
  if not eng:
    return mph, '', 'switched on before the logs start'
  te, ve, se, kind = eng[-1]
  rolling = ve >= 0.5
  code = {'set': 'S', 'res': 'R'}.get(kind, '?') + ('r' if rolling else '0')
  words = ({'set': 'SET', 'res': 'RES'}.get(kind, 'switched on') + (f' rolling at {ve * MPH:.0f} mph' if rolling else ' at a stop'))
  if se == se and sk == sk and 0 < sk < 254 and abs(sk - se) >= 1.0:
    code += '+-'
    words += f', then +/- {se / 3.6 * MPH:.0f}>{mph:.0f}'
  return mph, code, words


# ============================================================================================ the event window
class Win:
  """All signals on a 50 Hz grid G around an event."""

  def __init__(self, D, t_lo, t_hi):
    np = D.np
    G = np.arange(t_lo, t_hi + 1e-9, DT)
    self.G, self.D = G, D
    self.v = grid_interp(np, D.cs_t, D.v, G)
    self.aego = grid_interp(np, D.cs_t, D.aego, G)
    self.long = hold(np, D.cc_t, D.long.astype(float), G, stale=0.1) > 0.5
    self.en = hold(np, D.cc_t, D.cc_en.astype(float), G, stale=0.1) > 0.5
    self.out = np.where(self.long, hold(np, D.cc_t, D.cc_a, G, stale=0.1), NAN)
    self.lcs = hold(np, D.lcs_t, D.lcs, G, stale=0.3)
    self.aT = hold(np, D.lp_t, D.aT, G, stale=0.2)
    self.ss = hold(np, D.lp_t, D.sstop, G, stale=0.2)
    sk = hold(np, D.cs_t, D.vcruise, G, stale=0.5)
    with np.errstate(invalid='ignore'):
      self.set = np.where((sk > 0) & (sk < 254), sk / 3.6 * MPH, NAN)
    self.brk = hold(np, D.cs_t, D.brk.astype(float), G, stale=0.1) > 0.5
    self.gas = hold(np, D.cs_t, D.gas.astype(float), G, stale=0.1) > 0.5
    l1 = hold(np, D.rs_t, D.l1, G, stale=0.3) > 0.5
    self.lead = l1
    self.d = np.where(l1, hold(np, D.rs_t, D.dRel, G, stale=0.3), NAN)
    self.vl = np.where(l1, hold(np, D.rs_t, D.vLead, G, stale=0.3), NAN)
    self.al = np.where(l1, hold(np, D.rs_t, D.aLead, G, stale=0.3), NAN)
    self.sent = hold(np, *D.c['sent'], G, stale=0.1)
    self.tsk = hold(np, *D.c['tsk_target'], G, stale=0.2)
    whl = grid_interp(np, D.g_t, D.a_whl, G, gap=0.1) if len(D.g_t) else np.full(len(G), NAN)
    av = grid_interp(np, D.gv_t, D.a_v, G, gap=0.1) if len(D.gv_t) else np.full(len(G), NAN)
    self.whl_src_v = ~np.isfinite(whl) & np.isfinite(av)
    self.whl = np.where(np.isfinite(whl), whl, av)
    for key in ('hms', 'a06_stop', 'a07_stop', 'hold53', 'esp63'):
      setattr(self, key, hold(np, *D.c[key], G, stale=0.2))
    bt = np.array([b[0] for b in D.blocked if b[1] == 'ACC_06'])
    self.blk = (np.histogram(bt, bins=np.append(G - DT / 2, G[-1] + DT / 2))[0] if len(bt) and len(G)
                else np.zeros(len(G), dtype=np.int64))

  def compact(self):
    """After the analysis: float32 copies and no link to the drive (whose arrays would otherwise stay in memory)."""
    np = self.D.np
    for key, x in list(vars(self).items()):
      if key != 'G' and isinstance(x, np.ndarray) and x.dtype == np.float64:
        setattr(self, key, x.astype(np.float32))
    self.D = None

  def k(self, t):
    return int(min(max(round((t - self.G[0]) / DT), 0), len(self.G) - 1))

  def ks(self, t0, t1):
    return slice(self.k(t0), self.k(t1) + 1)


def runs_of(np, mask):
  """[(start, end)) index pairs of the True runs of a boolean array."""
  if not len(mask):
    return []
  d = np.diff(np.concatenate([[0], mask.astype(np.int8), [0]]))
  s, e = np.flatnonzero(d == 1), np.flatnonzero(d == -1)
  return list(zip(s.tolist(), e.tolist(), strict=True))


def run_start(np, mask, k_last, gap_n):
  """(start, end) of the last True run at or before k_last, bridging False gaps of up to gap_n samples."""
  idx = np.flatnonzero(mask[:k_last + 1])
  if not len(idx):
    return None, None
  br = np.flatnonzero(np.diff(idx) > gap_n + 1)
  return int(idx[br[-1] + 1] if len(br) else idx[0]), int(idx[-1])


def wanted_gap(v, t_follow):
  """The MPC's wanted distance to the car ahead (get_safe_obstacle_distance; the 6 m is compiled into the solver)."""
  return v ** 2 / 5.0 + t_follow * v + 6.0


LEAD_BACK_S = 0.5      # s: a vision lead that dropped out this shortly before still counts (moved on by the closing speed)


def lead_at(W, k, t_follow):
  """Car ahead at grid index k: (dRel, vLead, aLead, ttc, wanted gap, room = have - wanted) or Nones."""
  d, vl, al, v = W.d[k], W.vl[k], W.al[k], W.v[k]
  if not (d == d and vl == vl):     # dropped out right now (vision flicker): its last reading up to 0.5 s back
    n = int(round(LEAD_BACK_S / DT))
    j0 = max(k - n, 0)
    idx = np.flatnonzero(np.isfinite(W.d[j0:k + 1]) & np.isfinite(W.vl[j0:k + 1]))
    if len(idx):
      j = j0 + int(idx[-1])
      age = (k - j) * DT
      vl, al = W.vl[j], W.al[j]
      d = W.d[j] - max(W.v[j] - vl, 0.0) * age
  if not (d == d and vl == vl and v == v):
    return None
  ttc = d / (v - vl) if v - vl > 0.1 else math.inf
  want = wanted_gap(v, t_follow)
  room = d + max(vl, 0.0) ** 2 / 5.0 - want
  return {'d': float(d), 'vl': float(vl), 'al': float(al) if al == al else 0.0, 'ttc': float(ttc), 'want': float(want),
          'room': float(room)}


def changes(np, t_src, v_src, a, b):
  """[(t, old, new)] where a CAN value changed between a and b (the value before a is the starting point)."""
  if not len(t_src):
    return []
  i0 = max(int(np.searchsorted(t_src, a, side='left')) - 1, 0)
  i1 = int(np.searchsorted(t_src, b, side='right'))
  tt, vv = t_src[i0:i1], v_src[i0:i1]
  if len(vv) < 2:
    return []
  ch = np.flatnonzero(np.diff(vv) != 0) + 1
  return [(float(tt[j]), int(vv[j - 1]), int(vv[j])) for j in ch if tt[j] >= a]


def first_rise(np, t_src, v_src, a, b):
  """First time a 0/1 CAN signal went on between a and b (or was already on at a: a)."""
  if not len(t_src):
    return NAN
  i0 = int(np.searchsorted(t_src, a, side='left'))
  i1 = int(np.searchsorted(t_src, b, side='right'))
  vv = v_src[i0:i1] > 0.5
  if not len(vv):
    return NAN
  j = int(np.argmax(vv))
  return float(t_src[i0 + j]) if vv[j] else NAN


# ============================================================================================ A. stops
def still_runs(D):
  np = D.np
  t, v = D.cs_t, D.v
  out = []
  for s, e in runs_of(np, v < STILL_V):
    a, b = t[s], t[min(e, len(t) - 1)]
    if out and a - out[-1][1] < BLIP_S:
      out[-1][1] = b
    else:
      out.append([float(a), float(b)])
  return [(a, b) for a, b in out if b - a >= STILL_S]


def shape(np, W, x, k0, ks, kend):
  """Peak, build-up jerk, release and grab of one braking signal x between k0 (braking start) and kend."""
  r = {'peak': NAN, 't_peak': NAN, 'v_peak': NAN, 'jerk': NAN, 'rel_val': NAN, 'rel_depth': NAN, 'rel_dur': 0.0,
       'rel_v': NAN, 'eased': NAN, 'grab_val': NAN, 'grab_depth': NAN, 'grab_t': NAN, 'grab_v': NAN, 'grab_in': '',
       'hrg': False}
  if k0 is None or kend <= k0:
    return r
  kp = nargmin(x[k0:kend + 1])
  if kp is None:
    return r
  kp += k0
  G = W.G
  r['peak'], r['t_peak'], r['v_peak'] = float(x[kp]), float(G[kp] - G[ks]), float(W.v[kp])
  n = int(round(JERK_S / DT))
  if kp - k0 >= n:
    r['jerk'] = nmin((x[k0 + n:kp + 1] - x[k0:kp + 1 - n]) / JERK_S)
  if ks > kp:
    r['rel_val'] = nmax(x[kp:ks + 1])
    r['rel_depth'] = r['rel_val'] - r['peak']
  with np.errstate(invalid='ignore'):
    m = np.isfinite(x[kp:kend + 1]) & (x[kp:kend + 1] >= r['peak'] + RELEASE_DEPTH)
  best = None
  for s, e in runs_of(np, m):
    if (e - s) * DT < RELEASE_S - 1e-9:
      continue
    # the grab is measured from the most eased point on, inside the eased stretch too: a grab that stays above
    # peak + 0.8 (e.g. -2.6 eased to -0.3, then the stopping state's -1.2) never ends the eased stretch
    ke = kp + s + nargmax(x[kp + s:kp + e])
    eased = float(x[ke])
    tail = x[ke:kend + 1]
    kg = nargmin(tail)
    g = float(tail[kg]) if kg is not None else NAN
    cand = (s, e, eased, g, ke + kg if kg is not None else None, ke)
    if g == g and eased - g >= GRAB_DEPTH - 1e-9:
      best, r['hrg'] = cand, True
      break
    if best is None or e - s > best[1] - best[0]:
      best = cand
  if best is not None:
    s, e, eased, g, kg, ke = best
    r['rel_dur'], r['rel_v'], r['eased'] = (e - s) * DT, float(W.v[kp + s]), eased
    if kg is not None and kg > ke:
      r['grab_val'], r['grab_depth'] = g, eased - g
      r['grab_t'], r['grab_v'] = float(G[kg] - G[ks]), float(W.v[kg])
      # where the grab came from: the state when the braking had grown GRAB_DEPTH past the eased point (the
      # planner's own grab is under way before the stopping state takes over), else at the strongest point
      with np.errstate(invalid='ignore'):
        deep = np.flatnonzero(x[ke:kg + 1] <= eased - GRAB_DEPTH + 1e-9)
      lc = W.lcs[ke + int(deep[0])] if len(deep) else W.lcs[kg]
      r['grab_in'] = LCS_SHORT.get(int(lc), '?') if lc == lc else '?'
  r['hrg'] = r['hrg'] and r['peak'] <= HARD_PEAK
  return r


def in_numbers(s):
  """A stop that counts in the shape numbers: not a creep stop, and the driver did not brake (a touch under 2 mph at
  the very end is allowed: openpilot had done the braking)."""
  return not s['creep'] and (s['drv'] != s['drv'] or s['drv_v'] < DRV_LATE_V)


def label_of(sh, mean_decel, into):
  if sh['peak'] != sh['peak']:
    return '?'
  hard = sh['peak'] <= HARD_PEAK
  if hard and sh['hrg']:
    return 'hard-release-grab'
  if hard and mean_decel == mean_decel and mean_decel >= LATE_MEAN:
    return 'late-hard'
  if into == into and into <= INTO_STOP_HARD:
    return 'hard-into-stopping'
  if hard:
    return 'hard-eased'
  return 'smooth'


def analyse_stop(D, t_still, t_still_end, prev_end):
  np = D.np
  t_lo = max(t_still - LOOK_BACK_S, prev_end + 0.1)
  if t_still - t_lo < 1.0:
    t_lo = t_still - 1.0
  W = Win(D, t_lo, t_still + AFTER_S)
  G = W.G
  ks = W.k(t_still)
  s = {'kind': 'stop', 't': t_still, 't_end': t_still_end, 'route': D.route, 'rid': rid(D.route),
       'branch': D.info['branch'], 'clock': clock(t_still - D.t0), 'W': W, 'ks': ks}
  s['q'] = any(a <= t_still <= b for a, b in D.q_spans)
  m_prev = (D.cs_t >= prev_end) & (D.cs_t <= t_still)
  s['top'] = float(np.max(D.v[m_prev])) if m_prev.any() else NAN
  s['creep'] = not (s['top'] >= CREEP_TOP)
  # ACC_07 / ESP_21 changes are looked for from the car's fastest moment since the previous stop on (at most 4 s
  # back): earlier changes are the previous stop's launch (HMS 3>4>0), not this stop
  t_top = float(D.cs_t[m_prev][int(np.argmax(D.v[m_prev]))]) if m_prev.any() else t_still - 4.0
  h_lo = max(t_still - 4.0, t_top)
  # braking start: the last run of output < -0.3 that reaches the stop, gaps under 1 s bridged
  with np.errstate(invalid='ignore'):
    braking = np.isfinite(W.out) & (W.out < BRAKE_ON)
  k_last = min(ks + int(round(0.3 / DT)), len(G) - 1)
  k0, k1 = run_start(np, braking, k_last, int(round(BRAKE_GAP_S / DT)))
  s['k0'] = k0
  s['clipped'] = k0 is not None and k0 == 0 and G[0] > prev_end + 0.2
  if k0 is not None:
    s['t_brk'], s['v_brk'] = float(G[k0] - t_still), float(W.v[k0])
    dur = t_still - G[k0]
    s['mean_decel'] = s['v_brk'] / dur if dur > 0.2 else NAN
    s['lead_brk'] = lead_at(W, k0, D.t_follow)
    t_ref = G[k0]
  else:
    s['t_brk'] = s['v_brk'] = s['mean_decel'] = NAN
    s['lead_brk'] = None
    t_ref = max(t_still - 3.0, G[0])
  s['set_mph'], s['set_code'], s['set_words'] = set_at(D, t_ref)
  # longControlState pid -> stopping: the output it carried in (stopping keeps it if stronger than stopAccel)
  lc = W.lcs
  with np.errstate(invalid='ignore'):
    tr = np.flatnonzero((lc[1:] == 2) & (lc[:-1] == 1)) + 1
  # the change that led into this stop: the last one up to 0.5 s after the wheels stopped (on a hill it can come
  # late), else the first one after that (up to 3 s); never a later pid/stopping flip at the standstill
  before = tr[G[tr] <= t_still + 0.5] if len(tr) else tr
  after = tr[(G[tr] > t_still + 0.5) & (G[tr] <= t_still + AFTER_S)] if len(tr) else tr
  tr = before[-1:] if len(before) else after[:1]
  if len(tr):
    kst = int(tr[0])
    s['t_stp'], s['v_stp'] = float(G[kst] - t_still), float(W.v[kst])
    s['carried'] = float(W.out[kst - 1]) if W.out[kst - 1] == W.out[kst - 1] else float(W.out[kst])
    s['held'] = float(W.out[min(kst + int(round(0.3 / DT)), len(G) - 1)])
  else:
    kst = None
    s['t_stp'] = s['v_stp'] = s['carried'] = s['held'] = NAN
  s['kst'] = kst
  # shouldStop: the last time it came on before the stop
  with np.errstate(invalid='ignore'):
    on = (W.ss > 0.5).astype(np.int8)
  rise = np.flatnonzero(np.diff(on) > 0) + 1
  rise = rise[G[rise] <= t_still + 0.5] if len(rise) else rise
  s['t_ss'] = float(G[rise[-1]] - t_still) if len(rise) else NAN
  s['v_ss'] = float(W.v[rise[-1]]) if len(rise) else NAN
  # the shape: openpilot's output (to 0.1 s after the stop) and the wheels (to the stop)
  kend_out = min(ks + int(round(0.1 / DT)), len(G) - 1)
  s['op'] = shape(np, W, W.out, k0, ks, kend_out)
  if k0 is not None:
    kw0 = k0
  else:   # no openpilot braking (driver): the wheels' own braking start
    with np.errstate(invalid='ignore'):
      kw0, _ = run_start(np, np.isfinite(W.whl) & (W.whl < BRAKE_ON), ks, int(round(BRAKE_GAP_S / DT)))
  s['car'] = shape(np, W, W.whl, kw0, ks, ks)
  n1, n3 = int(round(FINAL_S / DT)), int(round(AT_STOP_S / DT))
  s['fin_out'] = nmin(W.out[max(ks - n1, 0):kend_out + 1])
  s['fin_whl'] = nmin(W.whl[max(ks - n1, 0):ks + 1])
  s['at_stop'] = nmean(W.whl[max(ks - n3, 0):ks + 1])
  h = int(round(JERK_S / 2 / DT))
  jw = np.full(len(G), NAN)
  if len(G) > 2 * h:
    jw[h:-h] = (W.whl[2 * h:] - W.whl[:-2 * h]) / JERK_S
  s['fin_jerk'] = nmin(jw[max(ks - n1, 0):ks + 1])
  s['bump'] = nmax(jw[max(ks - n3, 0):min(ks + n3, len(G) - 1) + 1])
  sig = {'plan': W.aT, 'op': W.out, 'sent': W.sent, 'tsk': W.tsk, 'whl': W.whl, 'aego': W.aego}
  ka = k0 if k0 is not None else max(ks - int(round(SHAPE_S / DT)), 0)
  s['peaks'] = {k: nmin(x[ka:kend_out + 1]) for k, x in sig.items()}
  s['fins'] = {k: nmin(x[max(ks - n1, 0):kend_out + 1]) for k, x in sig.items()}
  # the car rolled with the brakes eased: after a hard part, wheels weaker than -0.5 for 0.3 s above 0.3 m/s
  s['roll_dur'] = 0.0
  s['roll_v'] = NAN
  kp = nargmin(W.whl[kw0:ks + 1]) if kw0 is not None else None
  if kp is not None and W.whl[kw0 + kp] <= ROLL_AFTER:
    kp += kw0
    with np.errstate(invalid='ignore'):
      m = np.isfinite(W.whl[kp:ks + 1]) & (W.whl[kp:ks + 1] > ROLL_DECEL) & (W.v[kp:ks + 1] > ROLL_V)
    for a, b in runs_of(np, m):
      if (b - a) * DT >= ROLL_S - 1e-9 and (b - a) * DT > s['roll_dur']:
        s['roll_dur'], s['roll_v'] = (b - a) * DT, float(W.v[kp + a])
  ka2 = W.k(t_still + 0.5)
  kb2 = W.k(t_still + 2.5)
  with np.errstate(invalid='ignore'):
    mv = W.v[ka2:kb2 + 1] > MOVED_V
    mv &= ~W.gas[ka2:kb2 + 1] & ~(W.lcs[ka2:kb2 + 1] == 3)
  s['moved'] = nmax(np.where(mv, W.v[ka2:kb2 + 1], NAN))
  # labels
  s['label_op'] = 'creep' if s['creep'] else label_of(s['op'], s['mean_decel'], s['carried'])
  s['label_car'] = 'creep' if s['creep'] else label_of(s['car'], s['mean_decel'], s['at_stop'])
  # the car ahead at the stop, the driver
  k5 = W.k(t_still + 0.5)
  s['gap'] = float(W.d[k5]) if W.d[k5] == W.d[k5] else NAN
  s['gap_min'] = nmin(W.d[max(ks - int(round(2.0 / DT)), 0):k5 + 1])
  kd0 = max((k0 if k0 is not None else ks - int(round(6.0 / DT))) - int(round(1.0 / DT)), 0)
  db = np.flatnonzero(W.brk[kd0:W.k(t_still + 0.2) + 1])
  s['drv'] = float(G[kd0 + db[0]] - t_still) if len(db) else NAN
  s['drv_v'] = float(W.v[kd0 + db[0]]) if len(db) else NAN
  s['gas'] = bool(W.gas[kd0:ks + 1].any())
  # ACC_06 / ACC_07 / ESP_21 around the stop
  c = D.c
  s['hms_ch'] = [(t - t_still, a, b) for t, a, b in changes(np, *c['hms'], h_lo, t_still + AFTER_S)]
  s['a07s'] = first_rise(np, *c['a07_stop'], h_lo, t_still + AFTER_S) - t_still
  s['a06s'] = first_rise(np, *c['a06_stop'], h_lo, t_still + AFTER_S) - t_still
  s['h53'] = first_rise(np, *c['hold53'], max(t_still - 2.0, h_lo), t_still + AFTER_S) - t_still
  s['e63'] = first_rise(np, *c['esp63'], h_lo, t_still + AFTER_S) - t_still
  s['has_hms'] = len(c['hms'][0]) > 0
  s['has_esp21'] = len(c['hold53'][0]) > 0
  s['whl_v'] = bool(W.whl_src_v[max(ks - n1, 0):ks + 1].any())
  W.compact()
  return s


def find_stops(D):
  out = []
  runs = still_runs(D)
  np = D.np
  prev_end = D.cs_t[0] - 1.0
  for a, b in runs:
    i0, i1 = np.searchsorted(D.cc_t, a - LONG_BEFORE_S), np.searchsorted(D.cc_t, a, side='right')
    if i1 > i0 and D.long[i0:i1].any():
      # long control must also have been on the last time the car was at 5 mph or faster: a stop the driver
      # took over above 5 mph (and finished within 3 s) is a brake press, not an openpilot stop
      fast = np.flatnonzero((D.cs_t > prev_end) & (D.cs_t < a) & (D.v >= CREEP_TOP))
      if not len(fast) or hold(np, D.cc_t, D.long.astype(float), [D.cs_t[fast[-1]]], stale=0.3)[0] > 0.5:
        out.append(analyse_stop(D, a, b, prev_end))
    prev_end = b
  return out


# ============================================================================================ B. brake presses
def find_presses(D):
  np = D.np
  t, b = D.cs_t, D.brk
  out, stand = [], 0
  if len(t) < 2 or len(D.cc_t) == 0:
    return out, stand
  falls = t[1:][~b[1:] & b[:-1]]
  for i in (np.flatnonzero(b[1:] & ~b[:-1]) + 1).tolist():
    tp = float(t[i])
    # long control must have been on since the last release: a brake switch that bounces (on, off for a few frames,
    # on) is one press, not two (openpilot was switched off by the first)
    j = int(np.searchsorted(falls, tp, side='right')) - 1
    i0 = np.searchsorted(D.cc_t, max(tp - PRESS_LONG_S - 0.02, float(falls[j]) if j >= 0 else -math.inf))
    i1 = np.searchsorted(D.cc_t, tp, side='right')
    if not (i1 > i0 and D.long[i0:i1].any()):
      continue
    if D.v[i] < STAND_V:
      stand += 1
      continue
    out.append(analyse_press(D, tp))
  return out, stand


def analyse_press(D, tp):
  np = D.np
  W = Win(D, tp - 4.0, tp + STOP_AFTER_S)
  G = W.G
  k = W.k(tp)
  kb = max(k - 1, 0)        # the frame before the press (openpilot still in control)
  p = {'kind': 'press', 't': tp, 'route': D.route, 'rid': rid(D.route), 'branch': D.info['branch'],
       'clock': clock(tp - D.t0), 'W': W, 'kp': k}
  p['q'] = any(a <= tp <= b for a, b in D.q_spans)
  v = float(W.v[k])
  p['v'] = v
  prev_end = max([r[1] for r in D.runs if r[1] <= tp], default=float(D.cs_t[0]))
  m_prev = (D.cs_t >= prev_end) & (D.cs_t <= tp)
  p['top'] = float(np.max(D.v[m_prev])) if m_prev.any() else v
  p['set_mph'], p['set_code'], p['set_words'] = set_at(D, tp - 0.05)
  n05, n3 = int(round(0.5 / DT)), int(round(3.0 / DT))
  op_now = W.out[kb] if W.out[kb] == W.out[kb] else nmean(W.out[max(k - n05, 0):k])
  p['op'] = float(op_now) if op_now == op_now else NAN
  p['plan'] = float(W.aT[kb]) if W.aT[kb] == W.aT[kb] else NAN
  p['op_mean05'] = nmean(W.out[max(k - n05, 0):k])
  p['op_min3'] = nmin(W.out[max(k - n3, 0):k])
  p['op_max3'] = nmax(W.out[max(k - n3, 0):k])
  with np.errstate(invalid='ignore'):
    braking = np.isfinite(W.out) & (W.out < BRAKE_ON)
  if braking[kb]:
    a, _ = run_start(np, braking, kb, int(round(0.2 / DT)))
    p['brk_ago'] = float(tp - G[a])
  else:
    p['brk_ago'] = NAN
  p['whl'] = nmean(W.whl[max(k - 10, 0):k + 1])
  p['whl_min3'] = nmin(W.whl[max(k - n3, 0):k + 1])
  ld = lead_at(W, kb, D.t_follow)
  if ld is not None and not (ld['d'] < LEAD_D):
    ld = None
  p['lead'] = ld
  # the braking needed to stay STOP_GAP behind the car ahead (constant braking; if it brakes, until both stop)
  need = 0.0
  if ld is not None:
    if v > ld['vl']:
      need = (v - ld['vl']) ** 2 / (2.0 * max(ld['d'] - STOP_GAP, 0.5))
    if ld['al'] < LEAD_BRAKING and ld['vl'] > 0:     # (vision aLeadK is noisy: only clear braking counts)
      room = ld['d'] + ld['vl'] ** 2 / (2.0 * -ld['al']) - STOP_GAP
      need = max(need, v ** 2 / (2.0 * max(room, 0.5)))
  p['need'] = need if ld is not None else NAN
  relevant = ld is not None and (v - ld['vl'] > LEAD_CLOSING or ld['room'] < ROOM_NEAR or ld['al'] < LEAD_BRAKING)
  p['lead_rel'] = relevant
  # blocked ACC_06 frames within 1 s
  p['blocked'] = sum(1 for tb, m, _ in D.blocked if m == 'ACC_06' and abs(tb - tp) <= 1.0)
  # what happened next
  nxt = [r for r in D.runs if tp <= r[0] <= tp + STOP_AFTER_S]
  if nxt:
    ts = nxt[0][0]
    p['stop_in'] = ts - tp
    kst = W.k(ts)
    p['stop_by'] = 'openpilot' if W.long[max(kst - 5, 0):kst + 1].any() else 'driver'
    p['after_min'] = nmin(W.whl[k:kst + 1])
  else:
    p['stop_in'], p['stop_by'] = NAN, ''
    p['after_min'] = nmin(W.whl[k:W.k(tp + 5.0) + 1])
  ken = np.flatnonzero(W.en[k + 1:] & ~W.en[k:-1]) if len(G) > k + 1 else []
  p['reengaged'] = float(G[k + 1 + ken[0]] - tp) if len(ken) else NAN
  p['still_on'] = bool(W.long[min(k + int(round(0.3 / DT)), len(G) - 1)])
  # class
  op_req = p['op'] if p['op'] == p['op'] else p['plan']
  if v < CREEP_V:
    c = CLASSES[4] if p['top'] >= CREEP_TOP and p['stop_in'] <= ES_STOP_S else CLASSES[5]
  elif relevant and need >= NEED_MIN and op_req == op_req and op_req > -need + TOO_LITTLE:
    c = CLASSES[0]
  elif (p['op_mean05'] == p['op_mean05'] and p['op_mean05'] > OP_ACCEL) or (op_req == op_req and op_req > OP_ACCEL):
    c = CLASSES[3]
  elif relevant and ((p['brk_ago'] == p['brk_ago'] and p['brk_ago'] < LATE_BRK_S) or ld['room'] < TIGHT_ROOM
                     or ld['ttc'] < TIGHT_TTC or ld['d'] < TIGHT_D):
    c = CLASSES[1]
  elif not relevant:
    c = CLASSES[2]
  else:
    c = CLASSES[6]
  p['cls'] = c
  W.compact()
  return p


# ============================================================================================ blocked frames, drive stats
def blocked_split(D, press_t):
  """ACC_06 / ACC_07 frames the panda blocked: (n right after a brake press, clusters elsewhere with context)."""
  np = D.np
  after, rest = 0, []
  for tb, m, val in D.blocked:
    if any(BLOCK_AFTER[0] <= tb - tp <= BLOCK_AFTER[1] for tp in press_t):
      after += m == 'ACC_06'
      continue
    if rest and tb - rest[-1]['t1'] < 1.0:
      c = rest[-1]
      c['t1'] = tb
      c['n'][m] = c['n'].get(m, 0) + 1
      c['vals'].append(val)
      continue
    rest.append({'t': tb, 't1': tb, 'n': {m: 1}, 'vals': [val]})
  for c in rest:
    t = c['t']
    c['rid'], c['clock'] = rid(D.route), clock(t - D.t0)
    c['v'] = float(hold(np, D.cs_t, D.v, [t], stale=1.0)[0])
    c['long'] = bool(hold(np, D.cc_t, D.long.astype(float), [t - 0.02], stale=0.3)[0] > 0.5)
    c['en'] = bool(hold(np, D.cc_t, D.cc_en.astype(float), [t - 0.02], stale=0.3)[0] > 0.5)
    c['gas'] = bool(hold(np, D.cs_t, D.gas.astype(float), [t], stale=0.3)[0] > 0.5)
    c['brk'] = bool(hold(np, D.cs_t, D.brk.astype(float), [t], stale=0.3)[0] > 0.5)
    c['out'] = float(hold(np, D.cc_t, D.cc_a, [t - 0.02], stale=0.3)[0])
    eng = [e for e in D.engages if abs(e[0] - t) <= 1.0]
    c['eng_edge'] = bool(eng)
    lo, hi = min(c['vals']), max(c['vals'])
    if hi > ACCEL_MAX + 0.01 and hi < INACTIVE_ACCEL - 0.005 or lo < ACCEL_MIN - 0.01:
      c['why'] = f'accel {lo:.2f}..{hi:.2f} outside the panda limits -3.5..2.0'
    elif not c['long'] or c['eng_edge'] or c['gas']:
      c['why'] = ('sent while openpilot was switching on / off or the driver was on the gas'
                  if c['eng_edge'] or c['gas'] else 'sent while openpilot was not in control')
    else:
      c['why'] = 'inside the limits with openpilot in control (the panda did not allow longitudinal right then)'
  return after, rest


def drive_stats(D):
  """Moving time, openpilot moving time, and openpilot moving time above 25 / 30 / 35 mph (s)."""
  np = D.np
  t = D.cs_t
  dt = np.minimum(np.diff(t), 0.5)
  v = D.v[:-1]
  lng = hold(np, D.cc_t, D.long.astype(float), t[:-1], stale=0.3) > 0.5
  mov = v > 0.5
  r = {'move': float(dt[mov].sum()), 'op': float(dt[mov & lng].sum())}
  for mph in (25, 30, 35):
    r[f'op{mph}'] = float(dt[mov & lng & (v > mph / MPH)].sum())
  return r


# ============================================================================================ output helpers
def fmt(x, nd=2, w=5, plus=False):
  if x is None or x != x or (isinstance(x, float) and math.isinf(x)):
    return ' ' * (w - 1) + '-'
  x = round(x, nd) + 0.0     # no '-0.00'
  return f'{x:+{w}.{nd}f}' if plus else f'{x:{w}.{nd}f}'


def f1(x, nd=1):
  """Compact number for key=value text."""
  if x is None or x != x or (isinstance(x, float) and math.isinf(x)):
    return '-'
  return f'{round(x, nd) + 0.0:.{nd}f}'


def clock(t):
  return f'{int(t // 60)}:{int(t % 60):02d}'


def med(vals):
  vals = [v for v in vals if v is not None and v == v and not (isinstance(v, float) and math.isinf(v))]
  return st.median(vals) if vals else NAN


def hrs(h):
  return f'{h:.1f}' if h >= 1.0 else f'{h:.2f}'


def mph(v):
  return v * MPH if v == v else NAN


def lead_txt(ld, w=16):
  if ld is None:
    return f"{'none':<{w}}"
  ttc = f"{ld['ttc']:.1f}s" if ld['ttc'] < 99 else '-'
  return f"{ld['d']:3.0f}m {mph(ld['vl']):3.0f} {ttc:>5}"[:w].ljust(w)


def set_txt(s):
  return (f"{s['set_mph']:.0f}" if s['set_mph'] == s['set_mph'] else '-') + s['set_code']


def hms_txt(s, sep=' ', compact=False):
  if not s['has_hms']:
    return '-'
  ch = s['hms_ch']
  if not ch:
    return 'no change'
  if compact:
    return ','.join(f'{b}@{t:+.2f}' for t, a, b in ch[:5]) + ('..' if len(ch) > 5 else '')
  return sep.join(f'{a}>{b} {t:+.2f}' for t, a, b in ch[:5]) + (' ..' if len(ch) > 5 else '')


def stop_flags(s):
  f = ''
  if s['q']:
    f += 'q'
  if s['clipped']:
    f += '>'
  if s['whl_v']:
    f += 'v'
  return f


# ============================================================================================ tables
def print_stops(stops):
  say('STOPS BY OPENPILOT (car under 0.1 m/s for 0.5 s with openpilot\'s long control on in the 3 s before and at the')
  say('  last moment at 5 mph or faster; m/s2, minus = braking; OP = what openpilot asked (carControl accel), car = the')
  say('  wheels (ESP_19 slope); set = set speed mph when braking started + how it was set: S0 = SET at a stop, Sr = SET')
  say('  rolling, R0 / Rr = RES, +- = changed with +/- after; top = fastest mph since the previous stop; braking starts =')
  say('  first time OP asked under -0.3 in the final approach: mph, s before the stop, then the car ahead there: gap m, its')
  say('  mph, time to collision; peak = strongest braking OP')
  say('  asked @ mph, s before the stop / the wheels\' peak; jerk = OP\'s steepest build-up m/s3; release = how far OP eased')
  say('  after the peak (m/s2) > to what, for how long it stayed 0.8 or more above the peak (s) @ mph; into stopping = OP\'s')
  say('  accel when its stopping state took over (it KEEPS anything stronger than -1.2 to the end) @ mph, s before the stop;')
  say('  last 1 s = strongest braking in the last second OP / wheels, then the wheels\' braking at the stop (the bump);')
  say('  gap = to the car ahead at the stop; drv = driver pressed the brake (s before the stop); flags: q = from a qlog,')
  say('  > = braking started 20 s or more before, v = carState speed slope used for the wheels)')
  say('   #  drive     at  set     top  braking starts                       peak OP / car          jerk  release'
      '               into stopping       last 1 s OP/car stop    gap   drv  shape OP / car')
  for s in stops:
    op, car = s['op'], s['car']
    brk = (f"{fmt(mph(s['v_brk']), 0, 3)}mph {fmt(s['t_brk'], 1, 5)}s " + lead_txt(s['lead_brk'])
           if s['t_brk'] == s['t_brk'] else f"{'-':<36}")
    peak = (f"{fmt(op['peak'], 2, 5)}@{fmt(mph(op['v_peak']), 0, 2)} {fmt(op['t_peak'], 1, 4)}s/{fmt(car['peak'], 2, 5)}"
            if op['peak'] == op['peak'] else f"{'-':<17}/{fmt(car['peak'], 2, 5)}")
    rel = (f"{fmt(op['rel_depth'], 1, 3, plus=True)}>{fmt(op['rel_val'], 1, 4)} {fmt(op['rel_dur'], 1, 3)}s@{fmt(mph(op['rel_v']), 0, 2)}"
           if op['rel_depth'] == op['rel_depth'] else f"{'-':<19}")
    into = (f"{fmt(s['carried'], 2, 5)}@{fmt(mph(s['v_stp']), 1, 3)} {fmt(s['t_stp'], 1, 4)}s"
            if s['carried'] == s['carried'] else f"{'-':<18}")
    drv = f"{s['drv']:+.1f}" if s['drv'] == s['drv'] else '-'
    say(f"  {s['n']:2d} {s['rid']:>5} {s['clock']:>6}  {set_txt(s):<7} {fmt(mph(s['top']), 0, 3)}  {brk} {peak}"
        f"  {fmt(op['jerk'], 1, 5)}  {rel:<19}  {into:<18}  {fmt(s['fin_out'], 1, 4)}/{fmt(s['fin_whl'], 1, 4)}"
        f" {fmt(s['at_stop'], 1, 4)}  {fmt(s['gap'], 1, 5)}  {drv:>4}  {s['label_op']} / {s['label_car']}"
        + (f"  [{stop_flags(s)}]" if stop_flags(s) else ''))


def print_stop_signals(stops):
  say('STOP SHAPE IN SIX SIGNALS (peak = strongest braking from the braking start to the stop / last 1 s = strongest in the')
  say('  last second: plan = longitudinalPlan.aTarget, OP = carControl accel, sent = ACC_06, TSK = Motor_20 target, whl =')
  say('  wheels, aEgo = carState; grab = after the release, how far the braking grew again and where: pid = the planner')
  say('  asked for it, stop = the stopping state\'s ramp to -1.2; roll = the wheels eased under 0.5 m/s2 after a hard part,')
  say('  for s @ mph; jerk = the wheels\' steepest build-up in the last second m/s3; ss = shouldStop on (s before the stop,')
  say('  mph); HMS = ACC_07 ACC_Anforderung_HMS changes (s from the stop; 1 = hold request, 3 = standby, 4 = release);')
  say('  hold = ESP_21 hold confirmed (bit 53), 63 = ESP ACC stopping procedure (bit 63), A06 / A07 = ACC_06 / ACC_07')
  say('  ACC_Anhalten on)')
  say('   #  peak: plan    OP  sent   TSK   whl  aEgo   last 1 s: plan    OP  sent   TSK   whl  aEgo   grab'
      '               roll         jerk   ss              HMS                         hold    63   A06   A07')
  for s in stops:
    P, F, op = s['peaks'], s['fins'], s['op']
    grab = (f"{fmt(op['grab_depth'], 1, 3, plus=True)}>{fmt(op['grab_val'], 1, 4)} {op['grab_in']:<4}@{fmt(mph(op['grab_v']), 0, 2)}"
            if op['grab_depth'] == op['grab_depth'] else '-')
    roll = f"{s['roll_dur']:.1f}s@{mph(s['roll_v']):.0f}" if s['roll_dur'] > 0 else '-'
    ss_ = f"{s['t_ss']:+.1f}s@{mph(s['v_ss']):.1f}" if s['t_ss'] == s['t_ss'] else '-'
    say(f"  {s['n']:2d}  " + ' '.join(fmt(P[k], 2, 5) for k in ('plan', 'op', 'sent', 'tsk', 'whl', 'aego'))
        + '            ' + ' '.join(fmt(F[k], 2, 5) for k in ('plan', 'op', 'sent', 'tsk', 'whl', 'aego'))
        + f"   {grab:<18} {roll:<11} {fmt(s['fin_jerk'], 1, 5)}  {ss_:<14}  {hms_txt(s):<26}  {fmt(s['h53'], 2, 5, plus=True)}"
        f" {fmt(s['e63'], 2, 5, plus=True)} {fmt(s['a06s'], 2, 5, plus=True)} {fmt(s['a07s'], 2, 5, plus=True)}")


def print_presses(presses, n_stand):
  say('DRIVER BRAKE PRESSES WHILE OPENPILOT DROVE (brakePressed going on with long control on at that moment or 0.2 s')
  say('  before; set = set speed mph + how it was set (as above); OP now = openpilot\'s accel just before the press,')
  say('  3 s min/max = the strongest braking / most accel it asked in the 3 s before, brk = how long it had been braking')
  say('  (under -0.3); wheels = the car\'s accel at the press; car ahead: gap m, its mph, time to collision; wanted / room =')
  say('  the MPC\'s wanted gap and how far the real one was above (+) or under (-) it, m; need = constant braking needed to')
  say('  stay 4 m behind it; blk = ACC_06 frames the panda blocked within 1 s; then = what happened next)')
  if n_stand:
    say(f'  ({n_stand} more press(es) at a standstill are only counted)')
  say('   #  drive     at   mph  set      class                                    OP now  3 s min/max   brk  plan  wheels'
      '  car ahead          wanted/room   need  blk  then')
  for p in presses:
    ld = p['lead']
    wr = f"{ld['want']:3.0f}/{ld['room']:+4.0f}" if ld else '    -    '
    if p['stop_in'] == p['stop_in']:
      then = f"stopped in {p['stop_in']:.1f} s ({p['stop_by']}), wheels {fmt(p['after_min'], 1, 1)} max"
    else:
      then = f"no stop in {STOP_AFTER_S:.0f} s" + (f", on again after {p['reengaged']:.1f} s" if p['reengaged'] == p['reengaged'] else '')
    if not p['still_on'] and p['stop_in'] != p['stop_in'] and p['reengaged'] != p['reengaged']:
      then += ', openpilot off'
    say(f"  {p['n']:2d} {p['rid']:>5} {p['clock']:>6}  {fmt(mph(p['v']), 0, 4)}  {set_txt(p):<7}  {p['cls']:<40}"
        f" {fmt(p['op'], 2, 5)}  {fmt(p['op_min3'], 2, 5)}/{fmt(p['op_max3'], 2, 5)} {fmt(p['brk_ago'], 1, 4)}"
        f" {fmt(p['plan'], 2, 5)}  {fmt(p['whl'], 2, 5)}   {lead_txt(ld, 17)}  {wr:<11}  {fmt(p['need'], 1, 4)}"
        f"  {p['blocked']:3d}  {then}" + ('  [q]' if p['q'] else ''))


# ============================================================================================ groups
def group_stats(name, R, stops, presses, blk_rest, blk_after):
  rs = [r for r in R if r['branch'] == name]
  S = [s for s in stops if s['branch'] == name]
  shp = [s for s in S if in_numbers(s)]
  P = [p for p in presses if p['branch'] == name]
  op_h = sum(r['stats']['op'] for r in rs) / 3600.0
  g = {'name': name, 'drives': [r['rid'] for r in rs], 'commits': sorted({r['commit'] for r in rs}),
       'move_h': sum(r['stats']['move'] for r in rs) / 3600.0, 'op_h': op_h,
       'n_stops': len(S), 'n_creep': sum(1 for s in S if s['creep']),
       'n_drv': sum(1 for s in S if not s['creep'] and not in_numbers(s)), 'n_shape': len(shp),
       'n_press': len(P), 'n_stand': sum(r['n_stand'] for r in rs),
       'blk_after': sum(blk_after.get(r['rid'], 0) for r in rs),
       'blk_rest': sum(sum(c['n'].get('ACC_06', 0) for c in blk_rest if c['rid'] == r['rid']) for r in rs)}
  g['stop_rate'] = (g['n_stops'] - g['n_creep']) / op_h if op_h > 0.01 else NAN
  g['press_rate'] = g['n_press'] / op_h if op_h > 0.01 else NAN
  g['labels_op'] = {lb: sum(1 for s in shp if s['label_op'] == lb) for lb in LABELS[:5] + ['?']}
  g['labels_car'] = {lb: sum(1 for s in shp if s['label_car'] == lb) for lb in LABELS[:5] + ['?']}
  g['cls'] = {c: sum(1 for p in P if p['cls'] == c) for c in CLASSES}
  g['grab_pid'] = sum(1 for s in shp if s['label_op'] == 'hard-release-grab' and s['op']['grab_in'] == 'pid')
  g['grab_stop'] = sum(1 for s in shp if s['label_op'] == 'hard-release-grab' and s['op']['grab_in'] == 'stop')

  def mw(vals, worst):
    vals = [v for v in vals if v == v and not math.isinf(v)]
    return (med(vals), worst(vals) if vals else NAN)
  g['v_brk'] = mw([mph(s['v_brk']) for s in shp], max)
  g['peak'] = mw([s['op']['peak'] for s in shp], min)
  g['peak_car'] = mw([s['car']['peak'] for s in shp], min)
  g['rel'] = mw([s['op']['rel_depth'] for s in shp], max)
  g['into'] = mw([s['carried'] for s in shp], min)
  g['n_into'] = sum(1 for s in shp if s['carried'] == s['carried'] and s['carried'] <= INTO_STOP_HARD)
  g['fin_out'] = mw([s['fin_out'] for s in shp], min)
  g['fin_whl'] = mw([s['fin_whl'] for s in shp], min)
  g['at_stop'] = mw([s['at_stop'] for s in shp], min)
  g['jerk'] = mw([s['fin_jerk'] for s in shp], min)
  g['gap'] = mw([s['gap'] for s in shp], min)
  g['mean_decel'] = mw([s['mean_decel'] for s in shp], max)
  g['set'] = med([s['set_mph'] for s in shp])
  g['n_roll'] = sum(1 for s in shp if s['roll_dur'] > 0)
  g['top'] = {m: sum(1 for s in S if not s['creep'] and mph(s['top']) > m) for m in (25, 30, 35)}
  opt = sum(r['stats']['op'] for r in rs)
  g['above'] = {m: (sum(r['stats'][f'op{m}'] for r in rs) / opt if opt > 1 else NAN) for m in (25, 30, 35)}
  return g


def print_groups(groups):
  say('PER BRANCH (stop numbers from openpilot stops that are not creep stops and where the driver did not brake (a')
  say('  touch under 2 mph at the very end is kept); med / worst; h = hours moving / with openpilot driving)')
  for g in groups:
    say(f"  {g['name']} (drives {','.join(g['drives'])}; {','.join(g['commits'])}): {g['move_h']:.2f} h moving,"
        f" {g['op_h']:.2f} h with openpilot driving")
    say(f"    stops: {g['n_stops'] - g['n_creep']} ({fmt(g['stop_rate'], 1, 1)} per openpilot hour) + {g['n_creep']} creep;"
        f" {g['n_drv']} with the driver braking; shapes (OP): "
        + ', '.join(f"{lb} {g['labels_op'][lb]}" for lb in LABELS[:5] if g['labels_op'][lb])
        + '; car: ' + ', '.join(f"{lb} {g['labels_car'][lb]}" for lb in LABELS[:5] if g['labels_car'][lb]))
    say(f"    brake presses while openpilot drove: {g['n_press']} ({fmt(g['press_rate'], 1, 1)} per openpilot hour)"
        + (': ' + ', '.join(f"{c} {g['cls'][c]}" for c in CLASSES if g['cls'][c]) if g['n_press'] else '')
        + f"; {g['n_stand']} at a standstill")
    say(f"    braking started at {fmt(g['v_brk'][0], 0, 1)} mph (fastest {fmt(g['v_brk'][1], 0, 1)}); peak"
        f" {fmt(g['peak'][0], 2, 1)} (strongest {fmt(g['peak'][1], 2, 1)}), wheels {fmt(g['peak_car'][0], 2, 1)}"
        f" ({fmt(g['peak_car'][1], 2, 1)}); mean braking from the start {fmt(g['mean_decel'][0], 2, 1)}"
        f" ({fmt(g['mean_decel'][1], 2, 1)}); release {fmt(g['rel'][0], 1, 1)} ({fmt(g['rel'][1], 1, 1)})")
    say(f"    into stopping {fmt(g['into'][0], 2, 1)} (strongest {fmt(g['into'][1], 2, 1)}; {g['n_into']} at -1.8 or"
        f" stronger); last 1 s OP {fmt(g['fin_out'][0], 2, 1)} ({fmt(g['fin_out'][1], 2, 1)}), wheels"
        f" {fmt(g['fin_whl'][0], 2, 1)} ({fmt(g['fin_whl'][1], 2, 1)}), at the stop {fmt(g['at_stop'][0], 2, 1)}"
        f" ({fmt(g['at_stop'][1], 2, 1)}), jerk {fmt(g['jerk'][0], 1, 1)} ({fmt(g['jerk'][1], 1, 1)}) m/s3;"
        f" gap at the stop {fmt(g['gap'][0], 1, 1)} m (closest {fmt(g['gap'][1], 1, 1)}); rolled with the brakes eased"
        f" on {g['n_roll']}")
    n_ap = g['n_stops'] - g['n_creep']
    say(f"    set speed at the stops {fmt(g['set'], 0, 1)} mph median; stops coming from above 25/30/35 mph:"
        f" {g['top'][25]}/{g['top'][30]}/{g['top'][35]} of {n_ap}; openpilot driving time above 25/30/35 mph:"
        f" {'/'.join(f1(g['above'][m] * 100, 0) for m in (25, 30, 35))}%; panda-blocked ACC_06 frames:"
        f" {g['blk_after']} right after a brake press, {g['blk_rest']} elsewhere")


def pick_compare(groups):
  names = [g['name'] for g in groups]
  if len(groups) < 2:
    return None, groups[0] if groups else None
  old = next((g for g in groups if g['name'] == 't2_close_follow'), groups[0])
  new = next((g for g in groups if g['name'] == 't2_set_min'), groups[-1])
  if old is new:
    old, new = (groups[0], groups[-1]) if names[0] != new['name'] else (groups[-1], groups[0])
  return old, new


def hard_n(g):
  return sum(g['labels_op'][lb] for lb in ('late-hard', 'hard-release-grab', 'hard-into-stopping', 'hard-eased'))


def bottom_line(groups, stops):
  out = []
  old, new = pick_compare(groups)
  if new is None:
    return ['No openpilot stop or brake press in these drives.']

  def lab(g):
    n = g['n_shape']
    return ', '.join(f"{lb} {g['labels_op'][lb]}" for lb in LABELS[:5] if g['labels_op'][lb]) + f" (of {n})"

  def cls(g):
    return ', '.join(f"{c} {g['cls'][c]}" for c in CLASSES if g['cls'][c]) or 'none'
  if old is None:
    g = new
    out.append(f"{g['name']}: {g['n_stops'] - g['n_creep']} stops by openpilot in {hrs(g['op_h'])} h of openpilot driving;"
               f" braking started at {f1(g['v_brk'][0], 0)} mph median (fastest {f1(g['v_brk'][1], 0)}); peak braking"
               f" {f1(g['peak'][0], 2)} median (strongest {f1(g['peak'][1], 2)}); handed to the stopping state at"
               f" {f1(g['into'][0], 2)} median, {g['n_into']} at -1.8 or stronger; shapes: {lab(g)}.")
    out.append(f"  Driver brake presses while openpilot drove: {g['n_press']} ({f1(g['press_rate'], 1)} per hour): {cls(g)}.")
  else:
    a, b = old, new
    out.append(f"Stops by openpilot: {b['name']} {b['n_stops'] - b['n_creep']} in {hrs(b['op_h'])} h of openpilot driving"
               f" ({f1(b['stop_rate'], 1)}/h), {a['name']} {a['n_stops'] - a['n_creep']} in {hrs(a['op_h'])} h"
               f" ({f1(a['stop_rate'], 1)}/h). Numbers are {b['name']} first, then {a['name']}.")
    out.append(f"  Faster? Braking for a stop started at {f1(b['v_brk'][0], 0)} vs {f1(a['v_brk'][0], 0)} mph median (fastest"
               f" {f1(b['v_brk'][1], 0)} vs {f1(a['v_brk'][1], 0)}); set speed at the stops {f1(b['set'], 0)} vs"
               f" {f1(a['set'], 0)} mph; openpilot drove above 30 mph {f1(b['above'][30] * 100, 0)}% vs"
               f" {f1(a['above'][30] * 100, 0)}% of the time; stops coming from above 30 mph {b['top'][30]} vs {a['top'][30]}.")
    out.append(f"  Harder? Peak braking {f1(b['peak'][0], 2)} vs {f1(a['peak'][0], 2)} median (strongest {f1(b['peak'][1], 2)}"
               f" vs {f1(a['peak'][1], 2)}); handed to the stopping state at {f1(b['into'][0], 2)} vs {f1(a['into'][0], 2)}"
               f" ({b['n_into']} vs {a['n_into']} at -1.8 or stronger); wheels in the last second {f1(b['fin_whl'][0], 2)} vs"
               f" {f1(a['fin_whl'][0], 2)} (strongest {f1(b['fin_whl'][1], 2)} vs {f1(a['fin_whl'][1], 2)}).")
    out.append(f"  Shapes: {b['name']}: {lab(b)}; {a['name']}: {lab(a)}.")
    out.append(f"  Driver brake presses while openpilot drove: {b['n_press']} ({f1(b['press_rate'], 1)}/h: {cls(b)}) vs"
               f" {a['n_press']} ({f1(a['press_rate'], 1)}/h: {cls(a)}).")
  # So: plain-words reading of the numbers
  so = []
  g = new
  if old is not None:
    a = old
    dv = g['v_brk'][0] - a['v_brk'][0] if g['v_brk'][0] == g['v_brk'][0] and a['v_brk'][0] == a['v_brk'][0] else NAN
    if dv == dv and dv >= 3.0:
      so.append(f"stops on {g['name']} started from about {dv:.0f} mph faster: with a higher set speed the car speeds up"
                " more between stops, and the same planner (its code is the same on both) has to brake harder to stop"
                " from there")
    sh_b = hard_n(g) / g['n_shape'] if g['n_shape'] else NAN
    sh_a = hard_n(a) / a['n_shape'] if a['n_shape'] else NAN
    if sh_b == sh_b and sh_a == sh_a and sh_b - sh_a >= 0.15:
      so.append(f"more stops were hard on {g['name']} ({hard_n(g)} of {g['n_shape']} vs {hard_n(a)} of {a['n_shape']})")
    elif sh_b == sh_b and sh_a == sh_a and not (dv == dv and dv >= 3.0):
      so.append(f"the stops look much the same on both ({hard_n(g)} of {g['n_shape']} vs {hard_n(a)} of {a['n_shape']} hard)")
    if g['press_rate'] == g['press_rate'] and a['press_rate'] == a['press_rate'] and g['n_press'] >= a['n_press'] + 2 \
       and g['press_rate'] >= 1.5 * a['press_rate']:
      top = max(CLASSES, key=lambda c: g['cls'][c])
      so.append(f"the driver had to brake more often on {g['name']} ({f1(g['press_rate'], 1)} vs {f1(a['press_rate'], 1)} per"
                f" hour), mostly '{top}'")
  groups_n = [x for x in groups if x['n_shape']]
  n_into = sum(x['n_into'] for x in groups_n)
  if n_into:
    so.append(f"{n_into} stop(s) handed over to the stopping state while still braking at -1.8 m/s2 or harder; that"
              " state keeps that braking until the wheels stop (it only eases toward -1.2 when it is weaker), which is"
              " a hard final stop / bump")
  n_rg = sum(x['labels_op']['hard-release-grab'] for x in groups_n)
  if n_rg:
    gp, gs = sum(x['grab_pid'] for x in groups_n), sum(x['grab_stop'] for x in groups_n)
    so.append(f"{n_rg} stop(s) braked hard, eased off, then grabbed again before stopping ('rolling, HARDER BRAKE,"
              f" slight roll, stop'): the grab came from the planner on {gp} and from the stopping state's -1.2 on {gs}")
  if so:
    out.append('  So: ' + '; '.join(so) + '.')
  return out


# ============================================================================================ detail
def print_detail(e):
  W = e['W']
  say()
  if e['kind'] == 'stop':
    s = e
    op = s['op']
    ld = s['lead_brk']
    say(f"==== EVENT {s['n']}: STOP (drive {s['rid']} at {s['clock']}, {s['branch']}), shape OP {s['label_op']} / car"
        f" {s['label_car']}, set {set_txt(s)} mph ({s['set_words']}), fastest {fmt(mph(s['top']), 0, 1)} mph since the last stop")
    if s['t_brk'] == s['t_brk']:
      say(f"  braking started {-s['t_brk']:.1f} s before the stop at {mph(s['v_brk']):.0f} mph"
          + (f", the car ahead {ld['d']:.0f} m away at {mph(ld['vl']):.0f} mph"
             + (f" (time to collision {ld['ttc']:.1f} s)" if ld['ttc'] < 99 else '') + f", the MPC wanted {ld['want']:.0f} m"
             if ld else ', no car ahead') + f"; it averaged {f1(s['mean_decel'], 2)} m/s2 from there to the stop")
    say(f"  peak {f1(op['peak'], 2)} at {f1(mph(op['v_peak']), 0)} mph {f1(-op['t_peak'], 1)} s before the stop (wheels"
        f" {f1(s['car']['peak'], 2)}), built up at {f1(op['jerk'], 1)} m/s3; after it the braking eased by {f1(op['rel_depth'], 1)}"
        + (f", 0.8 or more for {op['rel_dur']:.1f} s from {mph(op['rel_v']):.0f} mph" if op['rel_dur'] > 0 else '')
        + (f", then grew by {op['grab_depth']:.1f} to {op['grab_val']:.2f} ({op['grab_in']}) at {mph(op['grab_v']):.1f} mph"
           if op['grab_depth'] == op['grab_depth'] else ''))
    if s['carried'] == s['carried']:
      say(f"  pid -> stopping {-s['t_stp']:.2f} s before the stop at {mph(s['v_stp']):.1f} mph carrying {s['carried']:.2f}"
          f" (0.3 s later {f1(s['held'], 2)}); shouldStop on {f1(-s['t_ss'], 2)} s before")
    else:
      say(f"  no pid -> stopping change before the stop; shouldStop on {f1(-s['t_ss'], 2)} s before")
    say(f"  last second: OP {f1(s['fin_out'], 2)}, wheels {f1(s['fin_whl'], 2)}, at the stop {f1(s['at_stop'], 2)}, jerk"
        f" {f1(s['fin_jerk'], 1)} m/s3" + (f"; rolled {s['roll_dur']:.1f} s at {mph(s['roll_v']):.0f} mph with the brakes eased"
                                         if s['roll_dur'] > 0 else '')
        + (f"; moved again at {mph(s['moved']):.1f} mph after the stop" if s['moved'] == s['moved'] else '')
        + f"; gap at the stop {f1(s['gap'], 1)} m"
        + (f"; driver braked {-s['drv']:.1f} s before at {mph(s['drv_v']):.0f} mph" if s['drv'] == s['drv'] else ''))
    say(f"  ACC_07 HMS: {hms_txt(s, sep=', ')}; ACC_07 Anhalten {f1(s['a07s'], 2)}, ACC_06 Anhalten {f1(s['a06s'], 2)},"
        f" ESP_21 hold {f1(s['h53'], 2)}, bit 63 {f1(s['e63'], 2)} s from the stop")
    t_ref = s['t']
    a, b = max(W.G[0], t_ref - SHAPE_S), t_ref + 2.0
    fine = (t_ref - 4.0, t_ref + 2.0)
  else:
    p = e
    ld = p['lead']
    say(f"==== EVENT {p['n']}: BRAKE PRESS (drive {p['rid']} at {p['clock']}, {p['branch']}): {p['cls']}, at"
        f" {mph(p['v']):.0f} mph, set {set_txt(p)} mph ({p['set_words']})")
    say(f"  openpilot asked {f1(p['op'], 2)} (plan {f1(p['plan'], 2)})"
        + (f", braking for {p['brk_ago']:.1f} s" if p['brk_ago'] == p['brk_ago'] else '')
        + f"; in the 3 s before {f1(p['op_min3'], 2)}..{f1(p['op_max3'], 2)}; wheels {f1(p['whl'], 2)}"
        + (f"; the car ahead {ld['d']:.0f} m at {mph(ld['vl']):.0f} mph ({ld['al']:+.1f} m/s2)"
           + (f", time to collision {ld['ttc']:.1f} s" if ld['ttc'] < 99 else '')
           + f", the MPC wanted {ld['want']:.0f} m ({abs(ld['room']):.0f} m {'short' if ld['room'] < 0 else 'to spare'},"
           f" counting the car ahead's own braking room), braking needed to stay 4 m behind it {p['need']:.1f}" if ld else
           '; no car ahead within 80 m')
        + f"; {p['blocked']} blocked ACC_06 frame(s) within 1 s")
    t_ref = p['t']
    a, b = t_ref - 3.0, min(t_ref + (p['stop_in'] + 2.0 if p['stop_in'] == p['stop_in'] else 6.0), W.G[-1])
    fine = (t_ref - 1.0, t_ref + 3.0)
  say('      t   mph  set  lead   vL   plan     OP   sent   TSK  wheels  aEgo  state  ss Anh HMS hold 63  brk gas lng blk')
  t = a
  while t <= b + 1e-9:
    k = W.k(t)
    lc = W.lcs[k]
    bits = []
    for x in (W.ss[k], W.a06_stop[k], W.hms[k], W.hold53[k], W.esp63[k]):
      bits.append('.' if x != x else str(int(x)))
    say(f"  {round(t - t_ref, 2) + 0.0:+6.2f} {fmt(mph(W.v[k]), 1, 5)} {fmt(W.set[k], 0, 4)} {fmt(W.d[k], 0, 5)} {fmt(mph(W.vl[k]), 0, 4)}"
        f" {fmt(W.aT[k], 2, 6)} {fmt(W.out[k], 2, 6)} {fmt(W.sent[k], 2, 6)} {fmt(W.tsk[k], 1, 5)} {fmt(W.whl[k], 2, 7)}"
        f" {fmt(W.aego[k], 2, 5)}  {LCS_SHORT.get(int(lc), '?') if lc == lc else '-':>5}  {bits[0]:>2} {bits[1]:>3}"
        f" {bits[2]:>3} {bits[3]:>4} {bits[4]:>2}  {'B' if W.brk[k] else '.':>3} {'G' if W.gas[k] else '.':>3}"
        f" {'on' if W.long[k] else '.':>3} {int(W.blk[max(k - 2, 0):k + 3].sum()) or '.':>3}")
    t += 0.1 if fine[0] - 1e-9 <= t < fine[1] - 1e-9 else 0.25
  say('  (t = s from the ' + ('stop' if e['kind'] == 'stop' else 'press') + '; plan = aTarget, OP = carControl accel, sent ='
      ' ACC_06, TSK = Motor_20, state = longControlState, ss = shouldStop, Anh = ACC_06 Anhalten, HMS = ACC_07 HMS,'
      ' hold = ESP_21 bit 53, 63 = bit 63, B = brake, G = gas, lng = long control on, blk = blocked ACC_06 frames)')


# ============================================================================================ paste
def paste_block(R, stops, presses, groups, notes, missing, blk_rest):
  say()
  say('===== PASTE THIS TO CLAUDE (start) =====')
  n_creep = sum(1 for s in stops if s['creep'])
  lines = [f"mk7_stop_check v{TOOL_VERSION.split()[0]} drives={','.join(r['rid'] for r in R)} stops={len(stops)}"
           f"(creep{n_creep}) presses={len(presses)}(+{sum(r['n_stand'] for r in R)}standstill)"
           " labels sm=smooth lh=late-hard rg=hard-release-grab is=hard-into-stopping he=hard-eased cr=creep;"
           " classes tl=lead-too-little lt=lead-late/tight nl=no-lead acc=OP-accel es=end-of-stop(<5mph) cr=creep ot=other;"
           " m/s2, mph, s"
           " from the stop/press"]
  shown_r = R if len(R) <= 6 else R[:3] + R[-2:]
  for r in shown_r:
    lines.append(f"D {r['rid']} {r['branch']}@{r['commit']} segs={r['segs']}/{r['nseg']}" + (f" q={r['q_segs']}" if r['q_segs'] else '')
                 + f" min={r['stats']['move'] / 60:.0f}/{r['stats']['op'] / 60:.0f} stops={r['n_stops']}(cr{r['n_creep']})"
                 f" presses={r['n_press']}(+{r['n_stand']}st) blk={r['blk_after']}/{r['blk_rest']} pers={r['pers']}"
                 + (f" lcs={r['lcs_src']}" if r['lcs_src'] != 'controlsState' else '')
                 + (f" sa={f1(r['stop_accel'], 1)}" if r['stop_accel'] == r['stop_accel'] and abs(r['stop_accel'] - STOP_ACCEL) > 0.01 else ''))
  if len(R) > len(shown_r):
    lines.append(f"D ... {len(R) - len(shown_r)} more drives")
  for g in groups:
    lines.append(f"G {g['name']} h={g['move_h']:.2f}/{g['op_h']:.2f} stops={g['n_stops'] - g['n_creep']}({f1(g['stop_rate'], 1)}/h)"
                 f"+cr{g['n_creep']} drv={g['n_drv']} OP=" + ','.join(f"{LABEL_SHORT[lb]}{g['labels_op'][lb]}" for lb in LABELS[:5])
                 + ' car=' + ','.join(f"{LABEL_SHORT[lb]}{g['labels_car'][lb]}" for lb in LABELS[:5])
                 + f" grab=pid{g['grab_pid']}/stop{g['grab_stop']} presses={g['n_press']}({f1(g['press_rate'], 1)}/h) "
                 + ','.join(f"{CLASS_SHORT[c]}{g['cls'][c]}" for c in CLASSES) + f" st={g['n_stand']}")
    lines.append(f"M {g['name']} brk={f1(g['v_brk'][0], 0)}/{f1(g['v_brk'][1], 0)} peak={f1(g['peak'][0], 2)}/{f1(g['peak'][1], 2)}"
                 f" whl={f1(g['peak_car'][0], 2)}/{f1(g['peak_car'][1], 2)} mean={f1(g['mean_decel'][0], 2)}/{f1(g['mean_decel'][1], 2)}"
                 f" rel={f1(g['rel'][0], 1)}/{f1(g['rel'][1], 1)} into={f1(g['into'][0], 2)}/{f1(g['into'][1], 2)}(n{g['n_into']})"
                 f" fin={f1(g['fin_out'][0], 1)}/{f1(g['fin_out'][1], 1)},{f1(g['fin_whl'][0], 1)}/{f1(g['fin_whl'][1], 1)}"
                 f" stop={f1(g['at_stop'][0], 1)}/{f1(g['at_stop'][1], 1)} j={f1(g['jerk'][0], 0)}/{f1(g['jerk'][1], 0)}"
                 f" gap={f1(g['gap'][0], 1)}/{f1(g['gap'][1], 1)} roll={g['n_roll']} set={f1(g['set'], 0)}"
                 f" top>25/30/35={g['top'][25]}/{g['top'][30]}/{g['top'][35]}"
                 f" t>25/30/35={'/'.join(f1(g['above'][m] * 100, 0) for m in (25, 30, 35))}% blk={g['blk_after']}/{g['blk_rest']}")
  xl = []
  for c in blk_rest[:2]:
    xl.append(f"X {c['rid']} {c['clock']} n={','.join(f'{k}:{v}' for k, v in c['n'].items())} acc={f1(min(c['vals']), 2)}"
              f"..{f1(max(c['vals']), 2)} v={f1(mph(c['v']), 0)} long={int(c['long'])} en={int(c['en'])}"
              f" gas={int(c['gas'])} brk={int(c['brk'])} out={f1(c['out'], 2)} edge={int(c['eng_edge'])}")
  if len(blk_rest) > 2:
    xl.append(f"X ... {len(blk_rest) - 2} more blocked clusters elsewhere")
  nl = [n for _, n in notes][:3]
  if len(notes) > 3:
    nl.append(f'(+{len(notes) - 3} more notes in the full output above)')
  if missing:
    nl.append('not on the bus: ' + ' '.join(f"{m}({'all' if len(ds) == len(R) else ','.join(ds)})" for m, ds in missing))
  room = PASTE_MAX - len(lines) - len(xl) - len(nl) - 1
  if room < 4:     # notes go first, then blocked clusters
    nl = nl[:max(0, len(nl) - (4 - room))]
    room = PASTE_MAX - len(lines) - len(xl) - len(nl) - 1
  if room < 4:
    xl = xl[:1]
    room = PASTE_MAX - len(lines) - len(xl) - len(nl) - 1
  room = max(room, 1)
  # presses first (up to 60% of the room, the most telling classes first), then the hardest stops
  pr = sorted(presses, key=lambda p: (CLASSES.index(p['cls']) if p['cls'] in CLASSES else 9, p['t']))
  n_p = min(len(pr), max(room * 3 // 5, room - len([s for s in stops if not s['creep']])))
  sel_p = sorted(pr[:n_p], key=lambda p: p['n'])
  cand = sorted([s for s in stops if not s['creep']], key=lambda s: (-LABEL_RANK.get(s['label_op'], 0), -abs(s['fin_whl'])
                                                                     if s['fin_whl'] == s['fin_whl'] else 0))
  sel_s = sorted(cand[:room - len(sel_p)], key=lambda s: s['n'])
  ev = sorted(sel_s + sel_p, key=lambda e: e['n'])
  more = (len(cand) - len(sel_s)) + (len(presses) - len(sel_p))     # (room already leaves a line for this)
  for e in ev:
    if e['kind'] == 'stop':
      s, op = e, e['op']
      ld = s['lead_brk']
      lines.append(f"S{s['n']} {s['rid']} {s['clock']} {LABEL_SHORT.get(s['label_op'], '?')}/{LABEL_SHORT.get(s['label_car'], '?')}"
                   f" set={set_txt(s)} top={f1(mph(s['top']), 0)} brk={f1(mph(s['v_brk']), 0)}@{f1(s['t_brk'], 1)}"
                   + (f" L{ld['d']:.0f}/{mph(ld['vl']):.0f}/{f1(ld['ttc'], 1) if ld['ttc'] < 99 else '-'}r{ld['room']:+.0f}" if ld else ' L-')
                   + f" mean={f1(s['mean_decel'], 2)} pk={f1(op['peak'], 2)}@{f1(mph(op['v_peak']), 0)}/{f1(op['t_peak'], 1)}"
                   f" whl={f1(s['car']['peak'], 2)} j={f1(op['jerk'], 0)} rel={f1(op['rel_depth'], 1)}/{f1(op['rel_dur'], 1)}"
                   + (f" grab={f1(op['grab_depth'], 1)}>{f1(op['grab_val'], 1)}{op['grab_in']}@{f1(mph(op['grab_v']), 0)}"
                      if op['grab_depth'] == op['grab_depth'] else '')
                   + f" stp={f1(s['carried'], 2)}@{f1(mph(s['v_stp']), 1)}/{f1(s['t_stp'], 2)} ss={f1(s['t_ss'], 2)}"
                   f" hms={hms_txt(s, compact=True)} a7={f1(s['a07s'], 2)} h53={f1(s['h53'], 2)} e63={f1(s['e63'], 2)}"
                   f" fin={f1(s['fin_out'], 1)}/{f1(s['fin_whl'], 1)} stop={f1(s['at_stop'], 1)} fj={f1(s['fin_jerk'], 0)}"
                   + (f" roll={s['roll_dur']:.1f}@{mph(s['roll_v']):.0f}" if s['roll_dur'] > 0 else '')
                   + f" gap={f1(s['gap'], 1)}" + (f" drv={f1(s['drv'], 1)}@{f1(mph(s['drv_v']), 0)}" if s['drv'] == s['drv'] else '')
                   + (f" [{stop_flags(s)}]" if stop_flags(s) else ''))
    else:
      p = e
      ld = p['lead']
      lines.append(f"B{p['n']} {p['rid']} {p['clock']} {CLASS_SHORT[p['cls']]} v={f1(mph(p['v']), 0)} set={set_txt(p)}"
                   f" op={f1(p['op'], 2)} min3={f1(p['op_min3'], 2)} max3={f1(p['op_max3'], 2)} brk={f1(p['brk_ago'], 1)}"
                   f" plan={f1(p['plan'], 2)} whl={f1(p['whl'], 2)}"
                   + (f" L{ld['d']:.0f}/{mph(ld['vl']):.0f}/{f1(ld['ttc'], 1) if ld['ttc'] < 99 else '-'} aL={f1(ld['al'], 1)}"
                      f" want={ld['want']:.0f} room={ld['room']:+.0f} need={f1(p['need'], 1)}" if ld else ' L-')
                   + f" blk={p['blocked']}"
                   + (f" stop={f1(p['stop_in'], 1)}{p['stop_by'][:3]} max={f1(p['after_min'], 1)}" if p['stop_in'] == p['stop_in']
                      else ' nostop') + (' [q]' if p['q'] else ''))
  if more > 0:
    lines.append(f'... {more} more events not shown (presses first, then the hardest stops)')
  lines += xl
  lines += ['note: ' + n for n in nl]
  for ln in lines[:PASTE_MAX]:
    say(ln)
  say('===== PASTE THIS TO CLAUDE (end) =====')


# ============================================================================================ main
def all_notes(R):
  notes = sorted({n for r in R for n in r['notes']})
  missing = []
  for m in MISSING_TXT:
    ds = [r['rid'] for r in R if m in r['missing']]
    if ds:
      missing.append((m, ds))
  return notes, missing


def parse(argv):
  p = argparse.ArgumentParser(description='Read-only: how openpilot stopped the car and every driver brake press.')
  p.add_argument('drives', nargs='*', default=['0'], help='0 = newest (default), 1 = the one before, 0-4 = a range,'
                 ' or a route-name prefix; several are pooled')
  p.add_argument('--detail', type=int, help='show event N in detail (stop or brake press; numbers in the tables)')
  p.add_argument('--all', action='store_true', help='show every event in detail')
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
  dbc, _ = load_dbc(rt, args.op_dir)
  want_msgs = {m for _, m, _ in CAR + OP}
  want = {dbc[m][0] for m in want_msgs if m in dbc}
  op_addrs = {dbc[m][0] for m in OP_MSGS if m in dbc}
  routes, segs = routes_on_disk(args.log_root)
  if not routes:
    raise UsageError(f'No drives found in {args.log_root}.')
  say(f'mk7_stop_check {TOOL_VERSION} (read-only)  newest {LIST_N} drives (0 = newest):')
  for i, r in enumerate(routes[:LIST_N]):
    b = route_branch(rt, args.log_root, r, segs)
    say(f'  {i} {r} segs {len(segs[r]):3d} {b[0]}@{b[1]}')
  chosen = pick_routes(args.drives, routes)
  stops, presses, R, blk_rest, blk_after = [], [], [], [], {}
  for route in chosen:
    t_start = time.time()
    can, svc, info = read_route(rt, args.log_root, route, segs, want, op_addrs)
    D = build(rt, dbc, can, svc, info)
    del can, svc
    head = (f"=== drive {route}  {info['branch']}@{info['commit']}  {info['segs']}/{info['nseg']} segments read"
            + (f" ({info['q_segs']} qlog only)" if info['q_segs'] else '')
            + (f", {info['none']} without a log" if info['none'] else '')
            + (f", {info['bad']} damaged messages" if info['bad'] else ''))
    if D is None:
      say(head + ': no carState in the logs, skipped')
      continue
    D.route = route
    if info['q_segs']:      # time spans read from qlogs (events there are coarse)
      D.q_spans = ([(D.cs_t[0] - 1.0, D.cs_t[-1] + 1.0)] if info['q_segs'] == info['segs'] else qlog_spans(D))
    D.engages = engages(D)
    D.runs = still_runs(D)
    st_route = find_stops(D)
    pr_route, n_stand = find_presses(D)
    press_t = [float(t) for t in D.cs_t[1:][D.brk[1:] & ~D.brk[:-1]]]
    after, rest = blocked_split(D, press_t)
    blk_after[rid(route)] = after
    blk_rest += rest
    stats = drive_stats(D)
    r = {'rid': rid(route), 'route': route, 'branch': info['branch'], 'commit': info['commit'], 'segs': info['segs'],
         'nseg': info['nseg'], 'q_segs': info['q_segs'], 'stats': stats, 'n_stops': len(st_route),
         'n_creep': sum(1 for s in st_route if s['creep']), 'n_press': len(pr_route), 'n_stand': n_stand,
         'blk_after': after, 'blk_rest': sum(c['n'].get('ACC_06', 0) for c in rest), 'pers': D.pers,
         'lcs_src': D.lcs_src, 'stop_accel': info['stop_accel'], 'notes': D.notes, 'missing': D.missing,
         'src': 'src ' + ' '.join(f'{m}={s.replace(" ", "")}' for m, s in sorted(D.srcs.items()))}
    if info['stop_accel'] == info['stop_accel'] and abs(info['stop_accel'] - STOP_ACCEL) > 0.01:
      D.notes.append((0, f"drive {rid(route)}: carParams.stopAccel is {info['stop_accel']:.2f}, not -1.2"))
    R.append(r)
    stops += st_route
    presses += pr_route
    say(head + f": {len(st_route)} openpilot stops ({r['n_creep']} creep), {len(pr_route)} brake presses while it drove"
        f" (+{n_stand} at a standstill), {stats['move'] / 60:.0f} min moving ({stats['op'] / 60:.0f} with openpilot),"
        f" personality {D.pers}, read in {time.time() - t_start:.0f} s")
    del D
  if not R:
    raise UsageError('None of the chosen drives has carState in its logs.')
  events = sorted(stops + presses, key=lambda e: (chosen.index(e['route']), e['t']))
  for i, e in enumerate(events, 1):
    e['n'] = i
  names = []
  for r in R:
    if r['branch'] not in names:
      names.append(r['branch'])
  groups = [group_stats(nm, R, stops, presses, blk_rest, blk_after) for nm in names]
  notes, missing = all_notes(R)
  return {'args': args, 'R': R, 'stops': stops, 'presses': presses, 'events': events, 'groups': groups,
          'notes': notes, 'missing': missing, 'blk_rest': blk_rest, 'blk_after': blk_after}


def qlog_spans(D):
  """Time spans where carState came at qlog rate (10 per second or slower)."""
  np = D.np
  t = D.cs_t
  if len(t) < 3:
    return []
  dt = np.diff(t)
  slow = np.convolve((dt > 0.06).astype(float), np.ones(21) / 21, mode='same') > 0.8
  return [(float(t[a]), float(t[min(b, len(t) - 1)])) for a, b in runs_of(np, slow)]


def report(ctx):
  R, stops, presses, events, groups = ctx['R'], ctx['stops'], ctx['presses'], ctx['events'], ctx['groups']
  args = ctx['args']
  if len({r['src'] for r in R}) == 1:
    say('signals: ' + R[-1]['src'][4:])
  else:
    for r in R:
      say(f"signals drive {r['rid']}: {r['src'][4:]}")
  for _, n in ctx['notes']:
    say('note: ' + n)
  for m, ds in ctx['missing']:
    say(f"note: {m} is not in the logs of {'these drives' if len(ds) == len(R) else 'drive ' + ','.join(ds)}: no {MISSING_TXT[m]}")
  say()
  if not events:
    say('No openpilot stop and no driver brake press while openpilot drove in these drives.')
    paste_block(R, stops, presses, groups, ctx['notes'], ctx['missing'], ctx['blk_rest'])
    return 1
  if stops:
    print_stops(stops)
    say()
    print_stop_signals(stops)
  else:
    say('No stop by openpilot in these drives.')
  say()
  if presses:
    n_stand = sum(r['n_stand'] for r in R)
    print_presses(presses, n_stand)
  else:
    say(f"No driver brake press while openpilot drove ({sum(r['n_stand'] for r in R)} at a standstill).")
  say()
  br = ctx['blk_rest']
  tot_after = sum(ctx['blk_after'].values())
  say(f"PANDA-BLOCKED ACC_06 FRAMES: {tot_after} right after a brake press (expected: openpilot sends 1-3 more frames after"
      f" the press before it lets go), {sum(c['n'].get('ACC_06', 0) for c in br)} elsewhere"
      + (':' if br else '.'))
  for c in br[:12]:
    say(f"  drive {c['rid']} at {c['clock']}: {', '.join(f'{v} {k}' for k, v in c['n'].items())} frame(s), accel in them"
        f" {f1(min(c['vals']), 2)}..{f1(max(c['vals']), 2)}, {f1(mph(c['v']), 0)} mph, long control {'on' if c['long'] else 'off'},"
        f" openpilot {'on' if c['en'] else 'off'}{', gas' if c['gas'] else ''}{', brake' if c['brk'] else ''}, it asked"
        f" {f1(c['out'], 2)}: {c['why']}")
  if len(br) > 12:
    say(f'  ... {len(br) - 12} more')
  say()
  print_groups(groups)
  say()
  say('BOTTOM LINE')
  for ln in bottom_line(groups, stops):
    say(ln)
  if args.detail:
    ev = [e for e in events if e['n'] == args.detail]
    if ev:
      print_detail(ev[0])
    else:
      say(f'There is no event {args.detail}; there are {len(events)}.')
  elif args.all:
    for e in events:
      print_detail(e)
  else:
    cand = [s for s in stops if in_numbers(s)] or [s for s in stops if not s['creep']]
    if cand:
      worst = max(cand, key=lambda s: (LABEL_RANK.get(s['label_op'], 0), -s['fin_whl'] if s['fin_whl'] == s['fin_whl'] else 0,
                                       -s['op']['peak'] if s['op']['peak'] == s['op']['peak'] else 0))
      say()
      say('The hardest stop in detail (another with --detail N, all with --all):')
      print_detail(worst)
  paste_block(R, stops, presses, groups, ctx['notes'], ctx['missing'], ctx['blk_rest'])
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
  except Exception:   # a bug: exit 2 (error), never 1 (= "no events")
    import traceback
    sys.stdout.flush()
    traceback.print_exc()
    say(f'ERROR: mk7_stop_check {TOOL_VERSION} failed (traceback above). Paste the last 20 lines to Claude.')
    sys.exit(2)
