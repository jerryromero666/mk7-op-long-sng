#!/usr/bin/env python3
"""
mk7_button_check.py - read-only: what happens on the car's CAN when a cruise button (+ / - / SET / RES) is pressed while
the car stands still with openpilot on (car OFF). Made to find what moves the tachometer to ~3000 rpm when +/- is
pressed at a stop.

INSTALL OR UPDATE (on the comma, over SSH; nothing is replaced if the download fails):
  cd /data/openpilot && git fetch sngrepo t2_set_min && git show FETCH_HEAD:mk7_button_check.py > /data/mk7_button_check.py.new && mv /data/mk7_button_check.py.new /data/mk7_button_check.py

RUN IT WITH THE CAR OFF:
  python3 /data/mk7_button_check.py              newest drive: every button press at a standstill
  python3 /data/mk7_button_check.py 1            the drive before (0 = newest)
  python3 /data/mk7_button_check.py 0 --moving   presses while driving too
  python3 /data/mk7_button_check.py 0 --press 2  only press 2 (numbers from the list)

For every press it prints:
  - a timeline (10 rows per second, 1 s before to 4 s after): speed, set speed, the planner (shouldStop, its request),
    openpilot's long control state and request, what openpilot sent in ACC_06 / ACC_07 (as the panda sent it), the
    TSK's status and target, the engine speed (Motor_12 MO_Drehzahl_01);
  - every CAN signal on the car's buses (0, 1, 2) that sat still in the 1.5 s before the press and moved in the 4 s
    after it, with before -> after, when it moved and for how long (counters and checksums left out), and messages
    the DBC does not know whose bytes changed the same way;
  - signals that moved on most presses (the suspects).
The last block, between "PASTE THIS TO CLAUDE (start)" and "(end)", is meant to be copied back as it is.
Only reads logs. Writes nothing. Exit codes: 0 = report printed, 1 = no press found, 2 = problem.
"""
import sys
sys.dont_write_bytecode = True   # never leave .pyc files behind on the device

import argparse  # noqa: E402
import bz2       # noqa: E402
import os        # noqa: E402
import re        # noqa: E402

TOOL_VERSION = '1.0 (2026-11-09)'
OP_DIR = '/data/openpilot'
ROOT = '/data/media/0/realdata'
VENV_PYTHON = '/usr/local/venv/bin/python3'
REEXEC_FLAG = 'MK7_BUTTON_CHECK_REEXEC'
ONROAD_PARAM = '/data/params/d/IsOnroad'
DBC_REL = ('opendbc_repo/opendbc/dbc/vw_mqb.dbc', 'opendbc/dbc/vw_mqb.dbc')
ZSTD_MAGIC = b'\x28\xb5\x2f\xfd'
MPH = 2.23694
KPH_PER_MPH = 1.609344   # carState.vCruise is in km/h
NAN = float('nan')

STILL_V = 0.1          # m/s: a press counts as 'at a standstill' under this
PRE = (-1.5, -0.1)     # s from the press: the 'before' window
POST = (0.05, 4.0)     # s from the press: the 'after' window
READ = (-2.0, 6.0)     # s from the press: CAN read around it
ROWS = (-0.5, 3.5)     # s: timeline
ROW_DT = 0.1           # s
MIN_MOVED_S = 0.15     # s a signal must be off its 'before' values to be listed
MAX_PRESSES = 3        # timelines printed without --press
TOP_N = 25             # changed signals listed per press
SKIP_SIG = re.compile(r'(CHK|BZ$|_BZ|Checksum|CRC|Counter|_ZK|Zaehler)', re.I)
BTN_NAME = {'accelCruise': '+', 'decelCruise': '-', 'setCruise': 'SET', 'resumeCruise': 'RES', 'cancel': 'CANCEL',
            'gapAdjustCruise': 'GAP', 'mainCruise': 'MAIN'}
LCS = {'off': 'off', 'pid': 'pid', 'stopping': 'stop', 'starting': 'strt'}
# timeline signals: (column, message, signal, decimals)
TL_CAN = (('A6acc', 'ACC_06', 'ACC_Sollbeschleunigung_02', 2), ('Anh6', 'ACC_06', 'ACC_Anhalten', 0),
          ('Anf6', 'ACC_06', 'ACC_Anfahren', 0), ('SS6', 'ACC_06', 'ACC_StartStopp_Info', 0),
          ('St6', 'ACC_06', 'ACC_Status_ACC', 0), ('HMS', 'ACC_07', 'ACC_Anforderung_HMS', 0),
          ('Anh7', 'ACC_07', 'ACC_Anhalten', 0), ('Anf7', 'ACC_07', 'ACC_Anfahren', 0),
          ('Weg7', 'ACC_07', 'ACC_Anhalteweg', 2), ('TSKst', 'TSK_06', 'TSK_Status', 0),
          ('TSKa', 'Motor_20', 'TSK_a_Soll_gradientenbegrenzt', 2), ('rpm', 'Motor_12', 'MO_Drehzahl_01', 0),
          ('tach', 'Motor_04', 'MO_Anzeigedrehz', 0))
OP_MSGS = ('ACC_06', 'ACC_07', 'ACC_02', 'ACC_04', 'GRA_ACC_01')


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
      return msgs
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
  return msgs


def decode(np, words, s):
  """Physical values of one signal from 8-byte little-endian words."""
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
  return raw.astype(np.float64) * s.factor + s.offset


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


def pick_route(arg, routes):
  if re.fullmatch(r'\d{1,3}', arg) and int(arg) < len(routes):
    return routes[int(arg)]
  picked = [r for r in routes if r.startswith(arg)]
  if not picked:
    raise UsageError(f'No drive matches {arg!r}.')
  return picked[0]


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


# ============================================================================================ reading
def read_pass1(rt, root, route, segs):
  """openpilot's side for the whole drive (small): button presses, speed, plan, long control. Segment time spans."""
  cs, cc, lp, presses, span = [], [], [], [], {}
  info = {'branch': '?', 'commit': '?', 'q': 0}
  for n in sorted(segs):
    p = seg_file(root, route, n, RLOGS)
    if p is None:
      p = seg_file(root, route, n, QLOGS)
      info['q'] += p is not None
    if p is None:
      continue
    t0 = t1 = None
    for e in events_of(rt, p):
      try:
        t = e.logMonoTime * 1e-9
        t0 = t if t0 is None else t0
        t1 = t
        w = e.which()
        if w == 'carState':
          c = e.carState
          cs.append((t, c.vEgo, c.vCruise, c.brakePressed, c.gasPressed))
          for b in c.buttonEvents:
            if b.pressed:
              presses.append((t, BTN_NAME.get(str(b.type), str(b.type)), n))
        elif w == 'carControl':
          c = e.carControl
          cc.append((t, c.longActive, c.enabled, c.actuators.accel, str(c.actuators.longControlState)))
        elif w == 'longitudinalPlan':
          lp.append((t, e.longitudinalPlan.shouldStop, e.longitudinalPlan.aTarget))
        elif w == 'initData' and info['branch'] == '?':
          info['branch'], info['commit'] = e.initData.gitBranch, e.initData.gitCommit[:7]
      except Exception:
        pass
    if t0 is not None:
      span[n] = (t0, t1)
  return cs, cc, lp, presses, span, info


def read_can(rt, root, route, seg_list, windows):
  """CAN frames inside the windows: received on buses 0-2, and openpilot's own ACC frames as the panda sent them
  (echo, src 128+) or refused them (src 192+). Key (kind, bus, address) -> list of (t, word)."""
  out = {}
  from_bytes = int.from_bytes
  for n in seg_list:
    p = seg_file(root, route, n, RLOGS) or seg_file(root, route, n, QLOGS)
    if p is None:
      continue
    for e in events_of(rt, p):
      try:
        if e.which() != 'can':
          continue
        t = e.logMonoTime * 1e-9
        if not any(a <= t <= b for a, b in windows):
          continue
        for f in e.can:
          if len(f.dat) > 8:
            continue
          src = f.src
          kind = 'rx' if src < 4 else ('tx' if 128 <= src < 132 else ('blk' if src >= 192 else None))
          if kind is None:
            continue
          out.setdefault((kind, src & 3, f.address), []).append((t, from_bytes(f.dat, 'little')))
      except Exception:
        pass
  return out


# ============================================================================================ analysis
def held(np, t_src, v_src, t):
  """value at or before each t (nan before the first sample)"""
  if len(t_src) == 0:
    return np.full(len(t), NAN)
  i = np.searchsorted(t_src, t, side='right') - 1
  return np.where(i >= 0, v_src[np.clip(i, 0, None)], NAN)


def fmt(x, d):
  if x != x:
    return '-'
  return f'{x:.{d}f}' if d else f'{int(round(x))}'


class Press:
  pass


def analyse(rt, dbc, can, cs, cc, lp, P):
  np = rt.np
  by_addr = {a: (name, sigs) for name, (a, sigs) in dbc.items()}
  t0 = P.t
  rows_t = t0 + np.arange(ROWS[0], ROWS[1] + 1e-9, ROW_DT)
  cs_t = np.array([r[0] for r in cs])
  P.v = held(np, cs_t, np.array([r[1] for r in cs]), rows_t)
  P.vc = held(np, cs_t, np.array([r[2] for r in cs]), rows_t)
  cc_t = np.array([r[0] for r in cc])
  P.la = held(np, cc_t, np.array([float(r[1]) for r in cc]), rows_t)
  P.opa = held(np, cc_t, np.array([r[3] for r in cc]), rows_t)
  lcs_idx = np.searchsorted(cc_t, rows_t, side='right') - 1
  P.lcs = [LCS.get(cc[i][4], '?') if i >= 0 else '-' for i in lcs_idx]
  lp_t = np.array([r[0] for r in lp])
  P.ss = held(np, lp_t, np.array([float(r[1]) for r in lp]), rows_t)
  P.at = held(np, lp_t, np.array([r[2] for r in lp]), rows_t)
  P.rows_t = rows_t

  def series(msg, sig, kinds=('tx', 'rx')):
    """(t, values) of one signal: openpilot's ACC frames as sent (echo), car messages on the bus they appear most"""
    a, sigs = dbc.get(msg, (None, {}))
    if a is None or sig not in sigs:
      return None
    best = None
    for kind in kinds:
      for bus in range(4):
        fr = can.get((kind, bus, a))
        if fr and (best is None or len(fr) > len(best)):
          best = fr
      if best:
        break
    if not best:
      return None
    t = np.array([f[0] for f in best])
    return t, decode(np, np.array([f[1] for f in best], dtype=np.uint64), sigs[sig])

  P.tl = {}
  for col, msg, sig, d in TL_CAN:
    s = series(msg, sig, ('tx', 'rx') if msg in OP_MSGS else ('rx',))
    P.tl[col] = (held(np, s[0], s[1], rows_t) if s else np.full(len(rows_t), NAN), d)

  # openpilot's own frames refused by the panda around the press
  P.blocked = sum(1 for (kind, _b, a), fr in can.items() if kind == 'blk'
                  for t, _w in fr if PRE[0] <= t - t0 <= POST[1])

  # what moved: every signal still before the press and off those values after it
  moved, raw_moved = [], []
  for (kind, bus, a), fr in can.items():
    if kind != 'rx' or not fr:
      continue
    t = np.array([f[0] for f in fr]) - t0
    w = np.array([f[1] for f in fr], dtype=np.uint64)
    pre, post = (t >= PRE[0]) & (t <= PRE[1]), (t >= POST[0]) & (t <= POST[1])
    if pre.sum() < 3 or post.sum() < 3:
      continue
    dt_frame = float(np.median(np.diff(t))) if len(t) > 2 else 0.02
    known = by_addr.get(a)
    if known is None:
      bp = w[pre]
      if len(set(bp.tolist())) <= 2:
        continue      # counters make most unknown frames change every time: only bytes are compared below
      for byte in range(8):
        sh = np.uint64(8 * byte)
        b_pre = ((bp >> sh) & np.uint64(0xFF)).astype(int)
        b_post = ((w[post] >> sh) & np.uint64(0xFF)).astype(int)
        lo, hi = b_pre.min(), b_pre.max()
        if hi - lo > 2:
          continue
        off = (b_post < lo) | (b_post > hi)
        if off.sum() * dt_frame >= MIN_MOVED_S:
          k = int(np.argmax(off))
          raw_moved.append((bus, a, byte, f'{lo}..{hi}', int(b_post[off][0]), float(t[post][k]), float(off.sum() * dt_frame)))
      continue
    name, sigs = known
    for sn, sd in sigs.items():
      if SKIP_SIG.search(sn):
        continue
      v = decode(np, w, sd)
      vp, va = v[pre], v[post]
      lo, hi = float(np.min(vp)), float(np.max(vp))
      step = abs(sd.factor) or 1.0
      if hi - lo > 3 * step and hi - lo > 0.02 * max(abs(hi), abs(lo), 1e-9):
        continue      # moves on its own before the press
      tol = 0.5 * step
      off = (va < lo - tol) | (va > hi + tol)
      dur = float(off.sum() * dt_frame)
      if dur < MIN_MOVED_S:
        continue
      vo = va[off]
      ext = float(vo[np.argmax(np.abs(vo - (lo + hi) / 2))])
      k = int(np.argmax(off))
      first = float(t[post][k])
      # last time it was off (how long until it settled back)
      last = float(t[post][np.nonzero(off)[0][-1]])
      rpm_like = 'Drehzahl' in sn or 'Anzeigedrehz' in sn
      moved.append(dict(bus=bus, msg=name, sig=sn, before=(lo + hi) / 2, after=ext, first=first, last=last, dur=dur,
                        rpm=rpm_like, score=dur * (1 + abs(ext - (lo + hi) / 2) / (abs((lo + hi) / 2) + step))))
  moved.sort(key=lambda m: (-m['rpm'], m['first'], -m['score']))
  P.moved, P.raw_moved = moved, raw_moved
  # openpilot-side changes in the 4 s after (from the rows)
  after = P.rows_t >= t0
  P.lcs_after = sorted(set(x for x, a_ in zip(P.lcs, after, strict=True) if a_))
  P.ss_after = sorted(set(int(x) for x, a_ in zip(P.ss, after, strict=True) if a_ and x == x))
  P.opa_max = float(np.nanmax(P.opa[after])) if after.any() else NAN
  # the launch edge: any ACC_06 frame with Anhalten while ACC_07 says HMS 4, in the 4 s after (every frame, not the rows)
  s_anh, s_hms = series('ACC_06', 'ACC_Anhalten', ('tx',)), series('ACC_07', 'ACC_Anforderung_HMS', ('tx',))
  P.edge = 0
  if s_anh and s_hms:
    k = (s_anh[0] >= t0 + POST[0]) & (s_anh[0] <= t0 + POST[1])
    h = held(np, s_hms[0], s_hms[1], s_anh[0][k])
    P.edge = int(np.sum((s_anh[1][k] == 1) & (h == 4)))
  for key in ('rpm', 'tach'):
    x = P.tl[key][0]
    setattr(P, key + '_before', float(np.nanmedian(x[~after])) if np.any(~np.isnan(x[~after])) else NAN)
    setattr(P, key + '_max', float(np.nanmax(x[after])) if np.any(~np.isnan(x[after])) else NAN)


# ============================================================================================ report
def timeline(P):
  cols = ['t', 'mph', 'set', 'lng', 'sS', 'plan', 'lcs', 'OPacc'] + [c for c, *_ in TL_CAN]
  say('  ' + ' '.join(f'{c:>6}' for c in cols))
  for i, t in enumerate(P.rows_t):
    vals = [f'{t - P.t:+.1f}', fmt(P.v[i] * MPH, 1), fmt(P.vc[i] / KPH_PER_MPH, 0),
            'on' if P.la[i] == 1 else '.', fmt(P.ss[i], 0), fmt(P.at[i], 2), P.lcs[i], fmt(P.opa[i], 2)]
    vals += [fmt(P.tl[c][0][i], P.tl[c][1]) for c, *_ in TL_CAN]
    say('  ' + ' '.join(f'{v:>6}' for v in vals))
  say('  (t = s from the press; set = set speed; lng = openpilot long control; sS = planner shouldStop; plan = planner')
  say('   request; lcs = long control state; OPacc = openpilot request; A6acc/Anh6/Anf6/SS6/St6 = ACC_06 accel /')
  say('   Anhalten / Anfahren / StartStopp_Info / Status_ACC as the panda sent it; HMS/Anh7/Anf7/Weg7 = ACC_07 hold')
  say('   request / Anhalten / Anfahren / Anhalteweg; TSKst = TSK_06 status; TSKa = Motor_20 TSK target; rpm = engine')
  say('   speed, Motor_12 MO_Drehzahl_01; tach = the speed the engine ECU tells the cluster to show, Motor_04 MO_Anzeigedrehz)')


def report(route, info, presses, args):
  say(f'=== drive {route}  {info["branch"]}@{info["commit"]}  {len(presses)} press(es) '
      f'{"at a standstill" if not args.moving else "(standstill and moving)"}')
  for i, P in enumerate(presses, 1):
    say(f'  {i:2d}  {P.clock}  {P.name:6s}  {P.v0 * MPH:4.1f} mph  openpilot {"ON" if P.on else "off"}  '
        f'set {P.vc0 / KPH_PER_MPH:.0f} mph  segment {P.seg}')
  shown = [presses[args.press - 1]] if args.press else presses[:MAX_PRESSES]
  for P in shown:
    say('')
    say(f'==== PRESS {P.i}: {P.name} at {P.clock} ({P.v0 * MPH:.1f} mph, openpilot {"on" if P.on else "off"})')
    say(f'  openpilot after it: long states {"/".join(P.lcs_after) or "-"}, shouldStop {P.ss_after}, highest request '
        f'{fmt(P.opa_max, 2)}, ACC_06 frames with Anhalten under HMS 4 (the launch edge) {P.edge}, '
        f'panda-refused frames {P.blocked}')
    say(f'  engine speed (Motor_12): {fmt(P.rpm_before, 0)} before, highest after {fmt(P.rpm_max, 0)}; tachometer value '
        f'(Motor_04): {fmt(P.tach_before, 0)} before, highest after {fmt(P.tach_max, 0)}')
    timeline(P)
    say(f'  CAN signals that sat still before the press and moved after it ({len(P.moved)}; first {TOP_N}):')
    for m in P.moved[:TOP_N]:
      say(f'    bus{m["bus"]} {m["msg"]}.{m["sig"]}: {m["before"]:g} -> {m["after"]:g}  from +{m["first"]:.2f} s, '
          f'off for {m["dur"]:.2f} s, last off +{m["last"]:.2f} s{"  <- ENGINE SPEED" if m["rpm"] else ""}')
    for bus, a, byte, rng, val, first, dur in P.raw_moved[:8]:
      say(f'    bus{bus} 0x{a:03X} (not in the DBC) byte {byte}: {rng} -> {val}  from +{first:.2f} s, off for {dur:.2f} s')
  # suspects: moved on most presses
  cnt = {}
  done = [P for P in presses if P in shown or not args.press]
  for P in done:
    for key in {(m['bus'], m['msg'], m['sig']) for m in P.moved}:
      cnt[key] = cnt.get(key, 0) + 1
  sus = sorted(((n, k) for k, n in cnt.items() if n >= max(1 if len(done) == 1 else 2, (len(done) + 1) // 2)),
               reverse=True)
  say('')
  say(f'SUSPECTS (moved on at least half of the {len(done)} press(es) looked at):')
  for n, (bus, msg, sig) in sus[:20]:
    say(f'  {n}/{len(done)}  bus{bus} {msg}.{sig}')
  if not sus:
    say('  none')
  # paste block
  say('')
  say('===== PASTE THIS TO CLAUDE (start) =====')
  say(f'mk7_button_check {TOOL_VERSION} drive={route} {info["branch"]}@{info["commit"]} presses={len(presses)}')
  for P in done[:8]:
    top = '; '.join(f'b{m["bus"]} {m["msg"]}.{m["sig"]} {m["before"]:g}>{m["after"]:g} @+{m["first"]:.2f}/{m["dur"]:.1f}s'
                    for m in P.moved[:6])
    say(f'P{P.i} {P.clock} {P.name} v={P.v0 * MPH:.1f} on={int(P.on)} lcs={"/".join(P.lcs_after)} sS={P.ss_after} '
        f'opmax={fmt(P.opa_max, 2)} edge={int(P.edge)} blk={P.blocked} rpm={fmt(P.rpm_before, 0)}>{fmt(P.rpm_max, 0)} '
        f'tach={fmt(P.tach_before, 0)}>{fmt(P.tach_max, 0)} | {top}')
  for n, (bus, msg, sig) in sus[:10]:
    say(f'S {n}/{len(done)} b{bus} {msg}.{sig}')
  say('===== PASTE THIS TO CLAUDE (end) =====')


# ============================================================================================ main
def parse(argv):
  p = argparse.ArgumentParser(description='Read-only: what happens on CAN when a cruise button is pressed at a stop.')
  p.add_argument('drive', nargs='?', default='0', help='0 = newest (default), 1 = the one before, or a route-name prefix')
  p.add_argument('--moving', action='store_true', help='also presses while driving')
  p.add_argument('--press', type=int, help='only press N (numbers from the list)')
  p.add_argument('--car-on', action='store_true', help='run even though the car is on (not recommended)')
  p.add_argument('--log-root', default=ROOT, help=argparse.SUPPRESS)
  p.add_argument('--op-dir', default=OP_DIR, help=argparse.SUPPRESS)
  p.add_argument('--onroad-param', default=ONROAD_PARAM, help=argparse.SUPPRESS)
  return p.parse_args(argv)


def main(argv=None):
  args = parse(argv)
  try:
    if not args.car_on and os.path.isfile(args.onroad_param):
      with open(args.onroad_param, 'rb') as f:
        if f.read().strip() in (b'1', b'\x01'):
          raise UsageError('The car is on. Run this with the car off (or add --car-on).')
    rt = setup(args.op_dir)
    np = rt.np
    dbc = load_dbc(rt, args.op_dir)
    routes, segs = routes_on_disk(args.log_root)
    if not routes:
      raise UsageError(f'No drives in {args.log_root}.')
    route = pick_route(args.drive, routes)
    say(f'mk7_button_check {TOOL_VERSION} (read-only)  drive {route}: reading openpilot\'s side ...')
    cs, cc, lp, raw_presses, span, info = read_pass1(rt, args.log_root, route, segs[route])
    if not cs:
      raise UsageError('No carState in this drive.')
    cs_t = np.array([r[0] for r in cs])
    cs_v = np.array([r[1] for r in cs])
    cs_vc = np.array([r[2] for r in cs])
    cc_t = np.array([r[0] for r in cc]) if cc else np.zeros(0)
    t_start = min(s[0] for s in span.values())
    presses = []
    for t, name, n in raw_presses:
      v0 = float(held(np, cs_t, cs_v, np.array([t]))[0])
      if not args.moving and not v0 < STILL_V:
        continue
      P = Press()
      P.t, P.name, P.seg, P.v0 = t, name, n, v0
      P.vc0 = float(held(np, cs_t, cs_vc, np.array([t]))[0])
      i = int(np.searchsorted(cc_t, t, side='right') - 1) if len(cc_t) else -1
      P.on = bool(cc[i][2]) if i >= 0 else False
      m, s = divmod(int(round(t - t_start)), 60)
      P.clock = f'{m}:{s:02d}'
      presses.append(P)
    for i, P in enumerate(presses, 1):
      P.i = i
    if args.press and not 1 <= args.press <= len(presses):
      raise UsageError(f'There are {len(presses)} presses; --press {args.press} does not exist.')
    if not presses:
      say(f'No cruise button press {"" if args.moving else "at a standstill "}in this drive '
          f'({len(raw_presses)} press(es) in all{"" if args.moving else "; --moving shows those while driving"}).')
      return 1
    todo = [presses[args.press - 1]] if args.press else presses
    windows = [(P.t + READ[0], P.t + READ[1]) for P in todo]
    seg_list = sorted(n for n, (a, b) in span.items() if any(a - 1 <= w1 and w0 <= b + 1 for w0, w1 in windows))
    say(f'  {len(presses)} press(es); reading the car\'s CAN around them (segments {seg_list}) ...')
    can = read_can(rt, args.log_root, route, seg_list, windows)
    for P in todo:
      analyse(rt, dbc, can, cs, cc, lp, P)
    for P in presses:
      if not hasattr(P, 'moved'):
        P.moved, P.raw_moved, P.lcs_after, P.ss_after = [], [], [], []
        P.opa_max = P.rpm_before = P.rpm_max = P.tach_before = P.tach_max = NAN
        P.edge, P.blocked = False, 0
    report(route, info, presses, args)
    return 0
  except UsageError as e:
    say(f'ERROR: {e}')
    return 2


if __name__ == '__main__':
  sys.exit(main())
