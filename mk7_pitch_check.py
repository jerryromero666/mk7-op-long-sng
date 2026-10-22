#!/usr/bin/env python3
"""
mk7_pitch_check.py - READ-ONLY check of the comma 3X camera's up/down aim (pitch) on the Mk7 GTI build.

This build locks the camera calibration in selfdrive/locationd/calibrationd.py
(FIXED_PITCH = 0.0236 rad, yaw -0.0070 rad), so openpilot never re-learns the pitch if the mount moves
or the car sits differently. If the real aim is LOWER than the lock, openpilot sees far-away cars as
FARTHER than they are. This script measures how far the real aim is from the lock.

It only READS drive logs. It never changes, creates or deletes anything on the device. The only file it
can write is the optional --csv file that you name yourself.

INSTALL OR UPDATE (on the comma, car OFF):
  cd /data/openpilot && git fetch sngrepo mk7_tools && git show FETCH_HEAD:mk7_pitch_check.py > /data/mk7_pitch_check.py

  Normal check (default, no measuring needed):
      python3 /data/mk7_pitch_check.py
  Uses your most recent drive. Needs a few minutes of fairly straight road at 15+ mph (highway ideal).

  Tape-measure check (see --help for the full procedure):
      python3 mk7_pitch_check.py approach --end-gap 3.2
"""
import sys
sys.dont_write_bytecode = True   # never leave .pyc files behind on the device

import argparse  # noqa: E402
import bz2       # noqa: E402
import gc        # noqa: E402
import math      # noqa: E402
import os        # noqa: E402
import re        # noqa: E402
import time      # noqa: E402
import warnings  # noqa: E402

EXIT_OK, EXIT_NO_DATA, EXIT_USAGE = 0, 1, 2

DEFAULT_OP_DIR = '/data/openpilot'
DEFAULT_LOG_ROOT = '/data/media/0/realdata'          # system/hardware/hw.py Paths.log_root() on device
VENV_PYTHON = os.environ.get('MK7_PITCH_VENV_PYTHON', '/usr/local/venv/bin/python3')
REEXEC_FLAG = 'MK7_PITCH_CHECK_REEXEC'
PARAMS_DIR = os.environ.get('MK7_PITCH_PARAMS_DIR', '/data/params/d')    # read only: IsOnroad

# --- values copied from this build (read from the files at run time where possible) ---
LOCK_PITCH_DEFAULT = 0.0236     # calibrationd.py:46  FIXED_PITCH
LOCK_YAW = -0.0070              # calibrationd.py:146/211/249 locked yaw
MPH = 0.44704
MIN_SPEED_FILTER = 15 * MPH                  # calibrationd.py:23
MAX_VEL_ANGLE_STD = math.radians(0.25)       # calibrationd.py:24
MAX_YAW_RATE_FILTER = math.radians(2)        # calibrationd.py:25
MAX_HEIGHT_STD = math.exp(-3.5)              # calibrationd.py:27
SPEED_YAW_START, SPEED_YAW_SPAN, SPEED_YAW_FULL = 13.0, 18.0, -0.0130   # camera_offset_helper.py:45-46
RADAR_TO_CAMERA = 1.52                       # radard.py:30, dRel = lead.x - 1.52
DEFAULT_HEIGHT = 1.22                        # calibrationd.py HEIGHT_INIT
DEFAULT_LENS_TO_BUMPER = 2.1
# qlog keeps 1 in N messages (cereal/services.py); replaced by the real table when it can be imported
QLOG_DECIMATION = {'cameraOdometry': (20.0, 10), 'carState': (100.0, 10), 'radarState': (20.0, 5),
                   'liveCalibration': (4.0, 4)}

FINE_DEG, WATCH_DEG = 0.10, 0.25
MIN_SAMPLES_HARD = 200      # below this: "not enough data" (exit 1)
MIN_SAMPLES_GOOD = 1000     # below this: results shown with a warning
ZSTD_MAGIC = b'\x28\xb5\x2f\xfd'

APPROACH_HELP = """
TAPE-MEASURE CHECK ("approach" mode) - uses the car's own wheel-speed odometer, so you only need a
short tape measure:
  1. Find a flat, straight, empty lot or quiet road with one parked car (or any car-sized vehicle).
  2. Car on, openpilot running, but NOT engaged. Line up straight behind the parked car about 50 m
     (160 ft) back. Wait a couple of seconds.
  3. Roll straight toward it at a slow, steady 5-15 mph. Do not steer much.
  4. Brake gently and stop 2-5 m (7-16 ft) behind it. Stay stopped at least 5 seconds.
  5. Measure the gap from your front bumper to its rear bumper with the tape. Turn the car off.
  6. Run:   python3 mk7_pitch_check.py approach --end-gap 3.2      (your measured gap; m by default,
            or write it like 10.5ft or 126in)
  Reverse variant: start close (2-5 m behind, stopped, measure the gap first), then reverse straight
  back slowly about 50 m and stop:   python3 mk7_pitch_check.py approach --start-gap 3.0
  Both use the newest drive unless you add --route NAME (see --list).
"""

EPILOG = APPROACH_HELP + """
OTHER OPTIONS EXPLAINED
  --list           show recent drives (newest first) with their number of 1-minute segments
  --route NAME     use one particular drive: its name from --list, or its number in that list (1 = newest)
  --routes N       normal check only: combine the N newest drives
  --csv FILE       also save the individual samples to FILE (a new file; never inside the log or openpilot folders)

Exit codes: 0 = ok, 1 = not enough usable data, 2 = problem with the command or setup.
This script only reads logs and never changes anything on the device.
"""


class UsageError(Exception):
  pass


class NotEnoughData(Exception):
  pass


# ============================================================================================ helpers
def say(msg=''):
  print(msg, flush=True)


def fmt_deg(rad, plus=True):
  d = math.degrees(rad)
  return (f'{d:+.2f}' if plus else f'{d:.2f}') + ' deg'


def fmt_m(m):
  return f'{m:.1f} m ({m * 3.2808:.0f} ft)'


def parse_length(text):
  """'3.2' / '3.2m' / '320cm' / '10.5ft' / '126in' -> metres."""
  m = re.match(r'^\s*([0-9]*\.?[0-9]+)\s*(m|cm|mm|ft|feet|foot|in|inch|inches|\'|")?\s*$', str(text).lower())
  if not m:
    raise argparse.ArgumentTypeError(f"'{text}' is not a distance. Use metres like 3.2, or add a unit: 10.5ft, 126in, 320cm")
  val = float(m.group(1))
  unit = m.group(2) or 'm'
  scale = {'m': 1.0, 'cm': 0.01, 'mm': 0.001, 'ft': 0.3048, 'feet': 0.3048, 'foot': 0.3048, "'": 0.3048,
           'in': 0.0254, 'inch': 0.0254, 'inches': 0.0254, '"': 0.0254}[unit]
  return val * scale


def speed_yaw(v):
  """camera_offset_helper.py: extra yaw applied to the model input above 29 mph (13 m/s)."""
  return min(max((v - SPEED_YAW_START) / SPEED_YAW_SPAN, 0.0), 1.0) * SPEED_YAW_FULL


def d_ground_plane(np, d, delta, h, d_max=250.0):
  """Distance a pure ground-plane reader reports for a car truly d metres from the camera when the camera
  is aimed delta rad lower than the calibration assumes:  d' = h / tan(atan(h/d) - delta)."""
  a = np.arctan2(h, d) - delta
  out = np.where(a > h / d_max, h / np.tan(np.maximum(a, 1e-9)), d_max)
  return np.minimum(out, d_max)


# --------------------------------------------------------------- openpilot rotation conventions (local copy)
# Same math as common/transformations/transformations.py (euler2rot_single / rot2quat_single /
# quat2euler_single), so the result can be computed even off-device. Cross-checked at run time.
def rot_from_euler_local(np, e):
  phi, theta, psi = e
  cx, sx, cy, sy, cz, sz = math.cos(phi), math.sin(phi), math.cos(theta), math.sin(theta), math.cos(psi), math.sin(psi)
  Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
  Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
  Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
  return Rz @ Ry @ Rx


def euler_from_rot_local(np, rot):
  tr = np.trace(rot)
  if tr > 0:
    s = 0.5 / math.sqrt(tr + 1.0)
    w, x, y, z = 0.25 / s, (rot[2, 1] - rot[1, 2]) * s, (rot[0, 2] - rot[2, 0]) * s, (rot[1, 0] - rot[0, 1]) * s
  elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
    s = 2.0 * math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2])
    w, x, y, z = (rot[2, 1] - rot[1, 2]) / s, 0.25 * s, (rot[0, 1] + rot[1, 0]) / s, (rot[0, 2] + rot[2, 0]) / s
  elif rot[1, 1] > rot[2, 2]:
    s = 2.0 * math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2])
    w, x, y, z = (rot[0, 2] - rot[2, 0]) / s, (rot[0, 1] + rot[1, 0]) / s, 0.25 * s, (rot[1, 2] + rot[2, 1]) / s
  else:
    s = 2.0 * math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1])
    w, x, y, z = (rot[1, 0] - rot[0, 1]) / s, (rot[0, 2] + rot[2, 0]) / s, (rot[1, 2] + rot[2, 1]) / s, 0.25 * s
  if w < 0:
    w, x, y, z = -w, -x, -y, -z
  roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
  pitch = math.asin(min(max(2 * (w * y - z * x), -1.0), 1.0))
  yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
  return np.array([roll, pitch, yaw])


def corrected_rpy(rt, locked_rpy, residual_pitch, residual_yaw):
  """calibrationd.py:208 composition: euler_from_rot(rot_from_euler(locked) . rot_from_euler(observed))."""
  np = rt.np
  obs = [0.0, residual_pitch, residual_yaw]
  local = euler_from_rot_local(np, rot_from_euler_local(np, locked_rpy) @ rot_from_euler_local(np, obs))
  check = 'not available here (computed with the built-in copy of the same math)'
  if rt.op_orient is not None:
    rfe, efr = rt.op_orient
    theirs = np.asarray(efr(rfe(np.array(locked_rpy, dtype=float)).dot(rfe(np.array(obs, dtype=float)))), dtype=float)
    diff = float(np.max(np.abs(theirs - local)))
    if diff < 1e-9:
      check = "matches openpilot's own common/transformations/orientation.py"
    else:   # should never happen; trust openpilot's version
      check = f"WARNING: built-in math differs from openpilot's by {diff:.2e} rad; used openpilot's"
      local = theirs
  return local, check


# ============================================================================================ log discovery
SEG_DIR_RE = re.compile(r'^(?P<route>.+)--(?P<num>\d+)$')
LOG_NAMES = (('rlog', ('rlog.zst', 'rlog.bz2', 'rlog')), ('qlog', ('qlog.zst', 'qlog.bz2', 'qlog')))


class Segment:
  def __init__(self, route, num, path):
    self.route, self.num, self.path = route, num, path
    self.files = {}
    mt = [os.stat(path).st_mtime]
    for kind, names in LOG_NAMES:
      for fn in names:
        fp = os.path.join(path, fn)
        try:
          st = os.stat(fp)
        except OSError:
          continue
        if os.path.isfile(fp) and st.st_size > 0:
          self.files[kind] = fp
          mt.append(st.st_mtime)
          break
    self.mtime = max(mt)
    self.recording = os.path.exists(os.path.join(path, 'rlog.lock'))

  @property
  def label(self):
    return f'{self.route}--{self.num}'


class Route:
  def __init__(self, name):
    self.name = name
    self.segments = []

  def finish(self):
    self.segments.sort(key=lambda s: s.num)
    self.mtime = max(s.mtime for s in self.segments)
    self.n_rlog = sum(1 for s in self.segments if 'rlog' in s.files)
    self.n_qlog_only = sum(1 for s in self.segments if 'rlog' not in s.files and 'qlog' in s.files)
    self.n_none = sum(1 for s in self.segments if not s.files)


def discover_routes(log_root):
  if not os.path.isdir(log_root):
    raise UsageError(f"The log folder {log_root} does not exist. On the comma 3X it is {DEFAULT_LOG_ROOT}; "
                     "use --log-root to point somewhere else.")
  routes = {}
  for name in os.listdir(log_root):
    m = SEG_DIR_RE.match(name)
    full = os.path.join(log_root, name)
    if not m or not os.path.isdir(full):
      continue
    try:
      seg = Segment(m.group('route'), int(m.group('num')), full)
    except OSError:
      continue
    routes.setdefault(seg.route, Route(seg.route)).segments.append(seg)
  out = list(routes.values())
  for r in out:
    r.finish()
  # newest first by the route's hex counter (the device clock can be wrong at boot), mtime as fallback
  def key(r):
    m = re.match(r'^([0-9a-f]{8})--', r.name)
    return (1, int(m.group(1), 16), r.mtime) if m else (0, 0, r.mtime)
  out.sort(key=key, reverse=True)
  return out


def describe_route(r):
  n = len(r.segments)
  start = time.strftime('%Y-%m-%d %H:%M', time.localtime(r.mtime - 60 * n))
  logs = f'rlog {r.n_rlog}/{n}'
  if r.n_qlog_only:
    logs += f', qlog only {r.n_qlog_only}'
  if r.n_none:
    logs += f', no log {r.n_none}'
  return n, start, logs


def print_route_list(routes, count):
  say(f'Recent drives (newest first). Times are approximate (from file dates; each segment is ~1 minute).')
  say(f"{'#':>3}  {'drive name':<28} {'segments':>8}  {'started about':<17}  logs")
  for i, r in enumerate(routes[:count], 1):
    n, start, logs = describe_route(r)
    say(f'{i:>3}  {r.name:<28} {n:>8}  {start:<17}  {logs}')
  if len(routes) > count:
    say(f'... and {len(routes) - count} older drives.')
  say()
  say('Use one with:  --route NAME   or   --route NUMBER   (for example --route 2 for the second newest)')


def resolve_route(routes, text):
  t = os.path.basename(text.strip().split('|')[-1].rstrip('/'))
  m = SEG_DIR_RE.match(t)
  names = {r.name: r for r in routes}
  if t in names:
    return names[t]
  if m and m.group('route') in names:
    return names[m.group('route')]
  # a short number like 1, 2, 12 is a position in --list (route names start with 8 hex digits, e.g. 0000002f)
  if t.isdigit() and len(t) <= 3 and not t.startswith('0'):
    i = int(t)
    if 1 <= i <= len(routes):
      return routes[i - 1]
    raise UsageError(f'There is no drive number {i}; there are {len(routes)} drives. Run with --list to see them.')
  cands = [r for r in routes if r.name.startswith(t)]
  if len(cands) == 1:
    return cands[0]
  if len(cands) > 1:
    raise UsageError(f"'{text}' matches {len(cands)} drives ({', '.join(c.name for c in cands[:5])}...). Give more of the name.")
  raise UsageError(f"No drive called '{text}' was found. Run with --list to see the drive names.")


# ============================================================================================ runtime / reader
class Runtime:
  np = None
  LogReader = None
  log = None
  capnp = None
  zstd = None
  op_orient = None
  reader_name = ''


def maybe_reexec():
  """If capnp / openpilot's LogReader can't be imported, retry with openpilot's own Python (the device venv)."""
  if os.environ.get(REEXEC_FLAG) == '1':
    return
  if not (os.path.isfile(VENV_PYTHON) and os.access(VENV_PYTHON, os.X_OK)):
    return
  venv_prefix = os.path.dirname(os.path.dirname(os.path.abspath(VENV_PYTHON)))
  if os.path.abspath(sys.prefix) == venv_prefix:
    return
  env = dict(os.environ)
  env[REEXEC_FLAG] = '1'
  env['PYTHONDONTWRITEBYTECODE'] = '1'
  sys.stdout.flush()
  sys.stderr.flush()
  os.execve(VENV_PYTHON, [VENV_PYTHON, os.path.abspath(__file__)] + sys.argv[1:], env)


def setup_runtime(op_dir):
  rt = Runtime()
  op_dir = os.path.abspath(op_dir)
  # parent first so op_dir ends up first: 'cereal' lives in op_dir, 'openpilot' is op_dir/openpilot or op_dir itself
  for p in (os.path.dirname(op_dir), op_dir):
    if os.path.isdir(p) and p not in sys.path:
      sys.path.insert(0, p)
  first_err = None
  try:
    import numpy as np
    import capnp  # noqa: F401
    from openpilot.tools.lib.logreader import LogReader
    rt.np, rt.LogReader = np, LogReader
    rt.reader_name = "openpilot's LogReader (tools/lib/logreader.py)"
  except Exception as e:     # missing module, wrong python, ...
    first_err = e
    maybe_reexec()
  try:
    import numpy as np
    import capnp
    import zstandard
    from cereal import log
    rt.np, rt.capnp, rt.zstd, rt.log = np, capnp, zstandard, log
  except Exception as e:
    raise UsageError("This needs openpilot's Python libraries (numpy, pycapnp, zstandard and the cereal folder).\n"
                     f"  Could not load them: {first_err or e}\n"
                     f"  Run it on the comma device (openpilot in {DEFAULT_OP_DIR}), or point --op-dir at an openpilot checkout.") from None
  if rt.LogReader is None:
    rt.reader_name = f"built-in minimal reader using openpilot's cereal schema (LogReader unavailable: {first_err})"
  else:
    try:   # make sure nothing is sent to the device's log daemon
      from openpilot.common import swaglog
      swaglog.cloudlog.removeHandler(swaglog.ipchandler)
    except Exception:
      pass
  try:
    from openpilot.common.transformations.orientation import rot_from_euler, euler_from_rot
    rt.op_orient = (rot_from_euler, euler_from_rot)
  except Exception:
    rt.op_orient = None
  try:
    from cereal.services import SERVICE_LIST
    for k in list(QLOG_DECIMATION):
      if k in SERVICE_LIST and SERVICE_LIST[k].decimation:
        QLOG_DECIMATION[k] = (SERVICE_LIST[k].frequency, SERVICE_LIST[k].decimation)
  except Exception:
    pass
  return rt


def _minimal_read(rt, path):
  with open(path, 'rb') as f:
    raw = f.read()
  damaged = False
  if raw[:3] == b'BZh' or path.endswith('.bz2'):
    d = bz2.BZ2Decompressor()
    try:
      dat = d.decompress(raw)
    except (OSError, EOFError, ValueError):
      dat, damaged = b'', True
    damaged = damaged or not d.eof
  elif raw[:4] == ZSTD_MAGIC or path.endswith('.zst'):
    parts, data = [], raw
    while data:
      dobj = rt.zstd.ZstdDecompressor().decompressobj()
      try:
        parts.append(dobj.decompress(data))
      except rt.zstd.ZstdError:
        damaged = True
        break
      if not getattr(dobj, 'eof', True):
        damaged = True
        break
      data = dobj.unused_data
    dat = b''.join(parts)
  else:
    dat = raw
  events = []
  try:
    for e in rt.log.Event.read_multiple_bytes(dat):
      events.append(e)
  except Exception:
    damaged = True
  return events, damaged


def load_events(rt, path):
  """Returns (events, damaged, how). Raises on an unreadable file."""
  if rt.LogReader is not None:
    try:
      with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter('always')
        events = list(rt.LogReader(path))
      damaged = any('orrupt' in str(w.message) for w in caught)
      if events:
        return events, damaged, 'LogReader'
    except Exception:
      pass        # e.g. zstd error on a damaged file: try the tolerant reader below
  events, damaged = _minimal_read(rt, path)
  if not events:
    raise ValueError('no readable messages')
  return events, damaged, 'tolerant reader'


def extract(rt, events, want_radar, want_extra):
  """Pull only the fields we need. Messages of other types are skipped after a cheap which() check."""
  np = rt.np
  cs, co, lc, rs = [], [], [], []
  for ev in events:
    try:
      w = ev.which()
      if w == 'carState':
        c = ev.carState
        if want_extra:
          try:
            gear = str(c.gearShifter)
          except Exception:
            gear = 'unknown'
          cs.append((ev.logMonoTime, c.vEgo, c.vEgoRaw, c.aEgo, c.steeringAngleDeg, gear))
        else:
          cs.append((ev.logMonoTime, c.vEgo))
      elif w == 'cameraOdometry':
        o = ev.cameraOdometry
        tr, rot, ts, rts = list(o.trans), list(o.rot), list(o.transStd), list(o.roadTransformTransStd)
        if len(tr) != 3 or len(rot) != 3 or len(ts) < 2:
          continue
        hstd = rts[2] if len(rts) == 3 else float('nan')
        co.append((ev.logMonoTime, tr[0], tr[1], tr[2], rot[2], ts[1], hstd, o.timestampEof))
      elif w == 'liveCalibration':
        l = ev.liveCalibration
        rpy = list(l.rpyCalib)
        if len(rpy) == 3:
          hh = list(l.height)
          lc.append((ev.logMonoTime, rpy[0], rpy[1], rpy[2], hh[0] if len(hh) == 1 else float('nan')))
      elif want_radar and w == 'radarState':
        r = ev.radarState
        ld = r.leadOne
        rs.append((ev.logMonoTime, r.mdMonoTime, 1.0 if ld.status else 0.0, ld.modelProb, ld.dRel))
    except Exception:
      continue   # one bad message: skip it

  def arr(rows, ncol):
    if not rows:
      return np.zeros((0, ncol))
    a = np.array([r[:ncol] for r in rows], dtype=float)
    return a[np.argsort(a[:, 0], kind='stable')]
  out = {'cs': arr(cs, 5 if want_extra else 2), 'co': arr(co, 8), 'lc': arr(lc, 5), 'rs': arr(rs, 5)}
  if want_extra:
    order = np.argsort(np.array([r[0] for r in cs], dtype=float), kind='stable') if cs else []
    out['gear'] = [cs[i][5] for i in order]
  return out


class SegRead:
  def __init__(self):
    self.kind = None
    self.damaged = False
    self.data = None
    self.problems = []
    self.how = ''


def read_segment(rt, seg, want_radar=False, want_extra=False):
  res = SegRead()
  for kind, _ in LOG_NAMES:
    path = seg.files.get(kind)
    if not path:
      continue
    try:
      events, damaged, how = load_events(rt, path)
    except Exception as e:
      res.problems.append(f'{os.path.basename(path)} is damaged and unreadable')
      continue
    res.kind, res.damaged, res.how = kind, damaged, how
    res.data = extract(rt, events, want_radar, want_extra)
    del events
    gc.collect()
    return res
  if not seg.files:
    res.problems.append('no rlog/qlog file in this segment')
  return res


def seg_status_text(seg, res):
  bits = []
  if res.kind is None:
    bits.append('SKIPPED: ' + '; '.join(res.problems))
  else:
    if res.problems:
      bits.append('; '.join(res.problems) + f' -> used {res.kind}')
    if res.damaged:
      bits.append('file cut off or damaged, used the readable part')
    if seg.recording:
      bits.append('was still being recorded')
  return ('  [' + ' | '.join(bits) + ']') if bits else ''


def qlog_note(used_qlog_segments, total):
  if not used_qlog_segments:
    return []
  f_co, d_co = QLOG_DECIMATION['cameraOdometry']
  f_cs, d_cs = QLOG_DECIMATION['carState']
  f_rs, d_rs = QLOG_DECIMATION['radarState']
  return [f'NOTE: {used_qlog_segments} of {total} segments had no usable full log (rlog), so the smaller qlog was used.',
          f'      A qlog keeps only 1 in {d_co} camera-motion messages ({f_co / d_co:g} per second instead of {f_co:g}),',
          f'      1 in {d_cs} carState and 1 in {d_rs} radarState messages, so you need a longer drive for the same accuracy.']


# ============================================================================================ lock info
def find_lock_in_source(op_dir):
  """Read (never write) calibrationd.py to find the FIXED_PITCH line."""
  path = os.path.join(op_dir, 'selfdrive', 'locationd', 'calibrationd.py')
  try:
    with open(path) as f:
      for i, line in enumerate(f, 1):
        m = re.match(r'^\s*FIXED_PITCH\s*=\s*([-+0-9.eE]+)', line)
        if m:
          return path, i, line.rstrip('\n'), float(m.group(1))
  except (OSError, ValueError):
    pass
  return path, None, None, None


def lock_check(rt, lc_all, lock_pitch):
  """Returns (logged_rpy or None, height, lines, ok)."""
  np = rt.np
  lines = []
  if len(lc_all) == 0:
    lines.append('The log has no liveCalibration messages, so the lock could not be checked. Assuming the lock')
    lines.append(f'values (pitch {lock_pitch:.4f}, yaw {LOCK_YAW:.4f}) were in use.')
    return None, DEFAULT_HEIGHT, lines, True
  roll, pitch, yaw = (float(np.median(lc_all[:, k])) for k in (1, 2, 3))
  hts = lc_all[:, 4]
  hts = hts[np.isfinite(hts) & (hts > 0.5) & (hts < 3.0)]
  height = float(np.median(hts)) if len(hts) else DEFAULT_HEIGHT
  spread = float(np.max(lc_all[:, 2]) - np.min(lc_all[:, 2]))
  ok = abs(pitch - lock_pitch) < 5e-5 and spread < 5e-5
  lines.append(f'Logged camera pitch in use (liveCalibration.rpyCalib): {pitch:.4f} rad   lock in calibrationd.py: {lock_pitch:.4f} rad')
  if ok:
    lines.append('  -> matches: the pitch lock is what was running. Good.')
  else:
    lines += ['',
              '!' * 78,
              '!!  WARNING: the pitch actually used while driving is NOT the locked value.',
              f'!!  Logged: {pitch:.4f} rad' + (f' (it varied by {spread:.4f} rad across the drive(s))' if spread >= 5e-5 else '')
              + f'; lock in calibrationd.py: {lock_pitch:.4f} rad.',
              '!!  So the software that recorded the drive(s) was not running this build\'s lock (different branch,',
              '!!  older install, or the lock was edited). The numbers below are relative to what WAS used.',
              '!' * 78, '']
  if abs(yaw - LOCK_YAW) > 5e-5:
    lines.append(f'  Note: logged yaw {yaw:.4f} rad differs from the locked yaw {LOCK_YAW:.4f} rad.')
  lines.append(f'Logged roll {roll:+.4f} rad, camera height {height:.2f} m'
               + ('' if len(hts) else f' (not logged; using {DEFAULT_HEIGHT} m)'))
  return [roll, pitch, yaw], height, lines, ok


def vs_build_lock(delta_obs, true_pitch, lock_pitch, lock_ok):
  """The measured error is relative to the calibration that was LOGGED. If that is not this build's lock,
  re-express it against the lock (true pitch estimate minus FIXED_PITCH), since that is what the user asked."""
  if lock_ok:
    return delta_obs, []
  d = true_pitch - lock_pitch
  return d, ['',
             f'!!  Because the drive did not use the lock: relative to what was used the error is {fmt_deg(delta_obs)},',
             f'!!  the camera\'s real pitch is about {true_pitch:.4f} rad, which is {fmt_deg(d)} from this build\'s',
             f'!!  FIXED_PITCH ({lock_pitch:.4f}). The verdict below uses that {fmt_deg(d)}.']


def verdict_lines(delta, suggestion, src_info, approach=False):
  """Plain-English verdict for a pitch error delta (rad, + = aimed lower than the lock)."""
  a = round(abs(math.degrees(delta)), 2)       # judge the same 2-decimal number the user sees
  path, lineno, line, cur = src_info
  out = []
  if a < FINE_DEG:
    out.append(f'VERDICT: FINE. The error is under {FINE_DEG} deg; no action needed.')
  elif a <= WATCH_DEG:
    out.append(f'VERDICT: WORTH WATCHING. The error is between {FINE_DEG} and {WATCH_DEG} deg. Nothing urgent;')
    out.append('         check again after the next few drives (and after carrying heavy loads or passengers).')
  else:
    out.append(f'VERDICT: RE-CAPTURE RECOMMENDED. The error is more than {WATCH_DEG} deg.')
    out.append(f'   Suggested new value:   FIXED_PITCH = {suggestion:.4f}')
    if lineno:
      out.append(f'   File: {path}, line {lineno}:')
      out.append(f'       {line.strip()}')
      out.append(f'   Change the number {cur:g} to {suggestion:.4f} on that line only (leave the yaw lines alone),')
    else:
      out.append(f'   File: {path} (the line that starts with FIXED_PITCH =, around line 46);')
      out.append(f'   change its number to {suggestion:.4f} (leave the yaw lines alone),')
    out.append('   then reboot the comma 3X and run this check again after a drive: it should then say about 0.')
    if approach:
      out.append('   Before changing anything, confirm with the normal check (no "approach"): this tape method')
      out.append('   measures the combined effect on distance, which also depends on how the driving model works.')
    out.append('   This script did NOT change anything.')
  return out


def meaning_lines(rt, delta, h):
  np = rt.np
  out = []
  a = round(abs(math.degrees(delta)), 2)
  if a == 0:
    out.append('Meaning: the camera aim matches the locked setting.')
  elif delta > 0:
    out.append(f'Meaning: the camera is aimed {a:.2f} deg LOWER than the locked setting assumes, so')
    out.append('openpilot sees far-away cars as FARTHER than they really are. That is the risky direction: it can')
    out.append('follow closer than intended and start braking later for stopped traffic.')
  else:
    out.append(f'Meaning: the camera is aimed {a:.2f} deg HIGHER than the locked setting assumes, so')
    out.append('openpilot sees far-away cars as CLOSER than they really are (the cautious direction: more gap,')
    out.append('possibly earlier braking).')
  if 0 < a < FINE_DEG:
    out.append('At this size the effect is very small.')
  out.append(f'Size of the effect on distance (UPPER BOUND, flat road, camera height {h:.2f} m; the real error is')
  out.append('usually smaller because the driving model also judges distance from how big the car looks):')
  for d in (30.0, 60.0):
    a = math.atan2(h, d) - delta
    if a <= h / 250.0:
      out.append(f'   a car really {d:.0f} m ({d * 3.2808:.0f} ft) ahead could be read as extremely far away (past the horizon)')
    else:
      dd = float(d_ground_plane(np, np.array([d]), delta, h)[0])
      out.append(f'   a car really {d:.0f} m ({d * 3.2808:3.0f} ft) ahead could be read as {dd:5.1f} m ({dd - d:+.1f} m)')
  return out


# ============================================================================================ residual mode
def residual_samples(rt, data, last_v):
  """calibrationd.py handle_cam_odom filters on one segment. Returns (samples dict, last_v, counts)."""
  np = rt.np
  cs, co = data['cs'], data['co']
  counts = dict(total=len(co), slow=0, turning=0, unsure=0, height=0, used=0)
  empty = {k: np.zeros(0) for k in ('t', 'v', 'pitch', 'yaw', 'syaw', 'lp')}
  if len(co) == 0:
    return empty, (float(cs[-1, 1]) if len(cs) else last_v), counts
  t = co[:, 0]
  if len(cs):
    idx = np.searchsorted(cs[:, 0], t, side='right') - 1
    v = np.where(idx >= 0, cs[np.clip(idx, 0, len(cs) - 1), 1], last_v if last_v is not None else 0.0)
    new_last = float(cs[-1, 1])
  else:
    v = np.full(len(t), last_v if last_v is not None else 0.0)
    new_last = last_v
  tx, ty, tz, rz, sy, hstd = co[:, 1], co[:, 2], co[:, 3], co[:, 4], co[:, 5], co[:, 6]
  fast = (v > MIN_SPEED_FILTER) & (tx > MIN_SPEED_FILTER)
  straight = np.abs(rz) < MAX_YAW_RATE_FILTER
  rpy_certain = np.arctan2(sy, tx) < MAX_VEL_ANGLE_STD
  height_certain = np.where(np.isfinite(hstd), hstd < MAX_HEIGHT_STD, True)
  ok = fast & straight & rpy_certain & height_certain
  counts['slow'] = int(np.sum(~fast))
  counts['turning'] = int(np.sum(fast & ~straight))
  counts['unsure'] = int(np.sum(fast & straight & ~rpy_certain))
  counts['height'] = int(np.sum(fast & straight & rpy_certain & ~height_certain))
  counts['used'] = int(np.sum(ok))
  vv = v[ok]
  lc = data['lc']                 # calibration pitch in use at each sample (latest liveCalibration before it)
  if len(lc):
    j = np.clip(np.searchsorted(lc[:, 0], t[ok], side='right') - 1, 0, len(lc) - 1)
    lp = lc[j, 2]
  else:
    lp = np.full(int(np.sum(ok)), np.nan)
  s = dict(t=t[ok] * 1e-9, v=vv, pitch=-np.arctan2(tz[ok], tx[ok]), yaw=np.arctan2(ty[ok], tx[ok]),
           syaw=np.array([speed_yaw(x) for x in vv]), lp=lp)
  return s, new_last, counts


def run_residual(rt, args, routes, src_info, lock_pitch):
  np = rt.np
  say('=' * 78)
  say(' Mk7 camera pitch check - normal check (uses ordinary driving, no measuring)')
  say('=' * 78)
  say(f'Reading drive logs in {args.log_root} (read-only).')
  say(f'Log reader: {rt.reader_name}')
  all_s, seg_meds, route_meds, lc_list = [], [], [], []
  tot = dict(total=0, slow=0, turning=0, unsure=0, height=0, used=0)
  n_used, qlog_segs, read_segs, stopped_early = 0, 0, 0, False
  for r in routes:
    n, start, logs = describe_route(r)
    say()
    say(f'Drive {r.name}: {n} segments (~{n} min), started about {start}; {logs}')
    last_v = None
    r_samples = []
    for seg in r.segments:
      if n_used >= args.max_samples:
        stopped_early = True
        break
      res = read_segment(rt, seg)
      if res.kind is None:
        say(f'  segment {seg.num:>3}: ----{seg_status_text(seg, res)}')
        continue
      read_segs += 1
      qlog_segs += res.kind == 'qlog'
      s, last_v, c = residual_samples(rt, res.data, last_v)
      for k in tot:
        tot[k] += c[k]
      if len(res.data['lc']):
        lc_list.append(res.data['lc'])
      if c['used']:
        s['route'], s['seg'] = r.name, seg.num
        all_s.append(s)
        r_samples.append(s['pitch'])
        if c['used'] >= 20:
          seg_meds.append((seg.label, c['used'], float(np.median(s['pitch']))))
      n_used += c['used']
      say(f"  segment {seg.num:>3}: {res.kind}  {c['used']:>5} usable samples (total {n_used:>6}){seg_status_text(seg, res)}")
      res.data = None
      gc.collect()
    if r_samples:
      route_meds.append((r.name, sum(len(x) for x in r_samples), float(np.median(np.concatenate(r_samples)))))
    if stopped_early:
      say(f'  (stopped reading: already have {n_used} samples, the --max-samples limit)')
      break
  say()
  for line in qlog_note(qlog_segs, read_segs):
    say(line)

  # ---------------------------------------------------------------- lock check
  say('--- Calibration that was actually in use ' + '-' * 37)
  lc_all = np.concatenate(lc_list) if lc_list else np.zeros((0, 5))
  logged_rpy, height, lines, lock_ok = lock_check(rt, lc_all, lock_pitch)
  for line in lines:
    say(line)
  locked_rpy = logged_rpy if logged_rpy is not None else [0.0, lock_pitch, LOCK_YAW]
  say()

  if n_used == 0 or n_used < MIN_SAMPLES_HARD:
    say('--- RESULT ' + '-' * 67)
    say(f'NOT ENOUGH DATA: only {n_used} usable samples (need at least {MIN_SAMPLES_HARD}, ideally {MIN_SAMPLES_GOOD}+).')
    if tot['total']:
      say(f"Of {tot['total']} camera-motion messages: {tot['slow']} were below 15 mph, {tot['turning']} were in turns,")
      say(f"{tot['unsure'] + tot['height']} were rejected as uncertain by openpilot's own calibration rules.")
    say('What to do: drive a few minutes on fairly straight road at 15+ mph (a highway is ideal), then run this')
    say('again. Use --list to check that the drive you want is there, and --route to pick it.')
    raise NotEnoughData()

  P = np.concatenate([s['pitch'] for s in all_s])
  Y = np.concatenate([s['yaw'] for s in all_s])
  V = np.concatenate([s['v'] for s in all_s])
  SY = np.concatenate([s['syaw'] for s in all_s])
  T = np.concatenate([s['t'] for s in all_s])
  med = float(np.median(P))
  q25, q75 = (float(x) for x in np.percentile(P, [25, 75]))
  iqr = q75 - q25
  secs = len(np.unique(np.floor(T)))
  good_meds = [m for (_, n, m) in seg_meds if n >= 50]
  if len(good_meds) >= 3:
    se = float(np.std(good_meds, ddof=1) / math.sqrt(len(good_meds)))
    se_how = f'from the spread between {len(good_meds)} segments'
  else:
    se = float(1.2533 * 1.4826 * np.median(np.abs(P - med)) / math.sqrt(max(secs, 1)))
    se_how = 'from the sample scatter'
  yaw_true = float(np.median(Y + SY))       # mount yaw error with the deliberate speed shift removed
  new_rpy, orient_check = corrected_rpy(rt, locked_rpy, med, yaw_true)
  suggestion = float(new_rpy[1])
  LP = np.concatenate([s['lp'] for s in all_s])
  if not lock_ok and np.all(np.isfinite(LP)):
    # drives recorded with a different (or changing) calibration: add each sample's own logged pitch
    suggestion = float(np.median(LP + P))
    orient_check += '; samples combined with the calibration logged at their own time'
  judged, judge_lines = vs_build_lock(med, suggestion, lock_pitch, lock_ok)

  say('--- RESULT: camera pitch error ' + '-' * 47)
  say(f"Usable samples: {n_used} (about {secs // 60} min {secs % 60} s of straight 15+ mph driving).")
  if tot['total']:
    say(f"  (of {tot['total']} camera-motion messages: {tot['slow']} too slow, {tot['turning']} in turns, "
        f"{tot['unsure'] + tot['height']} uncertain - the same rules openpilot's calibration uses)")
  say('Pitch error = real camera aim minus the locked aim (+ means aimed lower):')
  say(f'   median        {fmt_deg(med)}    <- the number that matters')
  say(f'   middle half   {fmt_deg(q25)} to {fmt_deg(q75)}  (spread of single samples; this is normal noise)')
  say(f'   uncertainty   about +/-{math.degrees(se):.2f} deg ({se_how})')
  for line in judge_lines:
    say(line)
  say()
  for line in meaning_lines(rt, judged, height):
    say(line)
  say()
  for line in verdict_lines(judged, suggestion, src_info):
    say(line)
  say(f'   (Re-captured pitch = locked calibration combined with the measured error, exactly the way')
  say(f'    calibrationd.py line ~208 does it. Math check: {orient_check}.)')

  warn = []
  if n_used < MIN_SAMPLES_GOOD:
    warn.append(f'Only {n_used} usable samples (want {MIN_SAMPLES_GOOD}+). Treat the result as rough. For a solid answer drive a few')
    warn.append('minutes of straight road at 15+ mph (a highway is ideal) and run again.')
  if math.degrees(iqr) > 0.6:
    warn.append(f'The single samples are very scattered (middle half spans {math.degrees(iqr):.2f} deg). Rough roads or')
    warn.append('a loose mount can do this. Run it again on another drive and compare.')
  big = [(lbl, n, m) for (lbl, n, m) in seg_meds if n >= 200]
  if len(big) >= 2:
    rng_deg = math.degrees(max(m for _, _, m in big) - min(m for _, _, m in big))
    if rng_deg > 0.3:
      warn.append(f'Segments disagree by up to {rng_deg:.2f} deg. Something may have changed during the drive(s)')
      warn.append('(mount bumped, passengers or cargo added/removed). Compare the segment list below.')
  if len(route_meds) >= 2:
    rr = math.degrees(max(m for _, _, m in route_meds) - min(m for _, _, m in route_meds))
    if rr > 0.25:
      warn.append(f'The drives disagree by {rr:.2f} deg; load or mount differed between them.')
  if warn:
    say()
    say('WARNINGS:')
    for w in warn:
      say('  ' + w)

  say()
  say('--- Consistency: median pitch error per 1-minute segment ' + '-' * 21)
  shown = seg_meds if len(seg_meds) <= 40 else seg_meds[:20] + [None] + seg_meds[-19:]
  for item in shown:
    if item is None:
      say(f'   ... {len(seg_meds) - 39} more segments ...')
      continue
    lbl, n, m = item
    say(f'   {lbl:<34} {n:>5} samples   {fmt_deg(m)}')
  if not seg_meds:
    say('   (no segment had 20+ usable samples)')
  if len(route_meds) >= 2:
    say('   Per drive:')
    for name, n, m in route_meds:
      say(f'   {name:<34} {n:>5} samples   {fmt_deg(m)}')

  say()
  say('--- Yaw (left/right aim): for information only - do NOT change the yaw ' + '-' * 5)
  lo = V < SPEED_YAW_START
  say('(+ means the camera points further LEFT than the lock assumes.)')
  if np.any(lo):
    say(f'   15-29 mph ({int(np.sum(lo))} samples, no speed shift active): median {fmt_deg(float(np.median(Y[lo])))}')
  else:
    say('   15-29 mph: no samples')
  if np.any(~lo):
    say(f'   29+ mph   ({int(np.sum(~lo))} samples): median {fmt_deg(float(np.median(Y[~lo])))}'
        f'  ({fmt_deg(float(np.median(Y[~lo] + SY[~lo])))} after removing the deliberate speed shift)')
  else:
    say('   29+ mph: no samples')
  say('   Above 29 mph this build deliberately shifts the picture sideways (camera_offset_helper.py,')
  say('   up to -0.0130 rad = -0.74 deg at 70 mph) to keep the car centred, so a bigger high-speed number is')
  say('   expected and intended. The yaw was tuned for lane centring: leave it alone.')

  if args.csv:
    rows = []
    for s in all_s:
      for i in range(len(s['pitch'])):
        rows.append((s['route'], s['seg'], f"{s['t'][i]:.3f}", f"{s['v'][i]:.3f}", f"{math.degrees(s['pitch'][i]):.4f}",
                     f"{math.degrees(s['yaw'][i]):.4f}", f"{math.degrees(s['syaw'][i]):.4f}"))
    write_csv(args.csv, ['route', 'segment', 'logtime_s', 'vEgo_ms', 'pitch_err_deg', 'yaw_resid_deg', 'speed_yaw_deg'], rows)
  return EXIT_OK


# ============================================================================================ approach mode
def runs_of(np, mask):
  """[(i0, i1)] inclusive index ranges where mask is True."""
  if len(mask) == 0:
    return []
  m = np.concatenate([[False], mask.astype(bool), [False]])
  d = np.diff(m.astype(int))
  starts, ends = np.where(d == 1)[0], np.where(d == -1)[0] - 1
  return list(zip(starts.tolist(), ends.tolist()))


def merge_segments(np, parts):
  out = {}
  for k in ('cs', 'co', 'lc', 'rs'):
    arrs = [p[k] for p in parts if len(p[k])]
    out[k] = np.concatenate(arrs) if arrs else parts[0][k]
  gears = []
  for p in parts:
    gears += p.get('gear', [])
  order = np.argsort(out['cs'][:, 0], kind='stable') if len(out['cs']) else np.zeros(0, dtype=int)
  out['cs'] = out['cs'][order] if len(out['cs']) else out['cs']
  out['gear'] = [gears[i] for i in order] if gears else []
  for k in ('co', 'lc', 'rs'):
    if len(out[k]):
      out[k] = out[k][np.argsort(out[k][:, 0], kind='stable')]
  return out


def find_anchor(np, data, end_mode):
  """end_mode: last standstill (>= 2 s) after motion.  start mode: last standstill (>= 2 s) followed by >= 10 m motion."""
  cs = data['cs']
  if len(cs) < 10:
    return None
  t = cs[:, 0] * 1e-9
  spd = np.abs(cs[:, 2])
  s = np.concatenate([[0.0], np.cumsum(0.5 * (spd[1:] + spd[:-1]) * np.diff(t))])
  runs = [(a, b) for (a, b) in runs_of(np, spd <= 0.01) if t[b] - t[a] >= 2.0]
  if end_mode:
    cands = [(a, b) for (a, b) in runs if s[a] - s[0] > 3.0]
    if not cands:
      return None
    a, b = cands[-1]
    return dict(t=t, s=s, i0=a, i1=b, t0=t[a], t1=t[b], covered=(s[a] - s[0] >= 65.0), win=(t[0], t[b]))
  for k in range(len(runs) - 1, -1, -1):
    a, b = runs[k]
    nxt = runs[k + 1] if k + 1 < len(runs) else None
    end_i = nxt[0] if nxt else len(t) - 1
    if s[end_i] - s[b] >= 10.0:
      win_end = t[nxt[1]] if nxt else t[-1]
      return dict(t=t, s=s, i0=a, i1=b, t0=t[a], t1=t[b], covered=True, win=(t[a], win_end), next_run=nxt)
  return None


def approach_pairs(rt, data, anc, end_mode, known_gap, odo_scale, latency):
  """True gap (tape + odometry) paired with radarState.leadOne.dRel at the camera-frame time."""
  np = rt.np
  t, s, cs = anc['t'], anc['s'], data['cs']
  rs = data['rs']
  r_t = rs[:, 0] * 1e-9
  md = rs[:, 1] * 1e-9
  t_img = np.where((md > 0) & (np.abs(md - r_t) < 1.0), md, r_t - 0.005) - latency
  s_img = np.interp(t_img, t, s)
  a_img = np.interp(t_img, t, cs[:, 3])
  v_img = np.interp(t_img, t, np.abs(cs[:, 2]))
  if end_mode:
    s_ref = s[anc['i0']]
    gap = known_gap + odo_scale * (s_ref - s_img)
  else:
    s_ref = s[anc['i1']]
    gap = known_gap + odo_scale * (s_img - s_ref)
  in_win = (t_img >= anc['win'][0]) & (t_img <= anc['win'][1])
  lead = (rs[:, 2] > 0.5) & (rs[:, 3] > 0.5)
  calm = np.abs(a_img) <= 1.0
  rng = (gap >= 4.0) & (gap <= 60.0)
  used = in_win & lead & calm & rng
  return dict(t=t_img, gap=gap, drel=rs[:, 4], a=a_img, v=v_img, in_win=in_win, lead=lead, calm=calm, rng=rng, used=used)


def fit_pitch(np, gap, drel, L, h):
  """Fit dRel = d_geo(gap + L; delta) - 1.52 + c on distance-bin medians (1 m bins; wider when the log is
  sparse, e.g. a qlog). Medians per bin make it robust to odd samples and stop a long standstill dominating."""
  x = y = None
  for width in (1.0, 2.0, 3.0, 5.0):
    idx = np.floor(gap / width).astype(int)
    xs, ys = [], []
    for b in np.unique(idx):
      m = idx == b
      if np.sum(m) >= 3:
        xs.append(float(np.median(gap[m])))
        ys.append(float(np.median(drel[m])))
    if len(xs) >= 8 and (max(xs) - min(xs)) >= 15.0 and max(xs) >= 20.0:
      x, y = np.array(xs), np.array(ys)
      break
  if x is None:
    return None

  def solve(x, y):
    def best_c(dl):
      g = d_ground_plane(np, x + L, dl, h) - RADAR_TO_CAMERA
      c = float(np.mean(y - g))
      return c, float(np.sum((y - g - c) ** 2))
    grid = np.radians(np.arange(-5.0, 5.0001, 0.01))
    sse = np.array([best_c(d)[1] for d in grid])
    k = int(np.argmin(sse))
    lo, hi = grid[max(k - 1, 0)], grid[min(k + 1, len(grid) - 1)]
    gr = (math.sqrt(5) - 1) / 2
    for _ in range(60):
      m1, m2 = hi - gr * (hi - lo), lo + gr * (hi - lo)
      if best_c(m1)[1] < best_c(m2)[1]:
        hi = m2
      else:
        lo = m1
    dl = 0.5 * (lo + hi)
    c, e = best_c(dl)
    res = y - (d_ground_plane(np, x + L, dl, h) - RADAR_TO_CAMERA + c)
    return dl, c, e, res
  dl, c, e, res = solve(x, y)
  mad = float(np.median(np.abs(res - np.median(res))))
  keep = np.abs(res) <= max(0.5, 4 * 1.4826 * mad)
  n_out = int(np.sum(~keep))
  if n_out and np.sum(keep) >= 8:
    x, y = x[keep], y[keep]
    dl, c, e, res = solve(x, y)
  eps = 1e-5
  J0 = (d_ground_plane(np, x + L, dl + eps, h) - d_ground_plane(np, x + L, dl - eps, h)) / (2 * eps)
  J = np.column_stack([J0, np.ones_like(x)])
  s2 = e / max(len(x) - 2, 1)
  try:
    cov = s2 * np.linalg.inv(J.T @ J)
    sd, sc = math.sqrt(max(cov[0, 0], 0.0)), math.sqrt(max(cov[1, 1], 0.0))
  except Exception:
    sd, sc = float('nan'), float('nan')
  return dict(delta=dl, c=c, sd=sd, sc=sc, nbins=len(x), n_out=n_out, rms=math.sqrt(s2), xmin=float(x.min()), xmax=float(x.max()),
              width=width)


def run_approach(rt, args, route, src_info, lock_pitch):
  np = rt.np
  end_mode = args.end_gap is not None
  known = args.end_gap if end_mode else args.start_gap
  L = args.lens_to_bumper
  say('=' * 78)
  say(' Mk7 camera pitch check - tape-measure check (' + ('drive up to a parked car' if end_mode else 'reverse away from a parked car') + ')')
  say('=' * 78)
  say(f'Reading drive logs in {args.log_root} (read-only).')
  say(f'Log reader: {rt.reader_name}')
  n, start, logs = describe_route(route)
  say(f'Drive {route.name}: {n} segments (~{n} min), started about {start}; {logs}')
  say('Reading from the end of the drive backwards until the whole run is found:')
  parts, anc, qlog_segs, read_segs = [], None, 0, 0
  for seg in reversed(route.segments):
    res = read_segment(rt, seg, want_radar=True, want_extra=True)
    if res.kind is None:
      say(f'  segment {seg.num:>3}: ----{seg_status_text(seg, res)}')
      continue
    read_segs += 1
    qlog_segs += res.kind == 'qlog'
    parts.insert(0, res.data)
    say(f"  segment {seg.num:>3}: {res.kind}  {len(res.data['cs'])} carState, {len(res.data['rs'])} radarState messages{seg_status_text(seg, res)}")
    data = merge_segments(np, parts)
    anc = find_anchor(np, data, end_mode)
    if anc is not None and anc['covered']:
      break
  say()
  for line in qlog_note(qlog_segs, read_segs):
    say(line)
  if not parts:
    say('NOT ENOUGH DATA: none of the log files in this drive could be read.')
    raise NotEnoughData()
  data = merge_segments(np, parts)
  if anc is None:
    if end_mode:
      say('NOT ENOUGH DATA: could not find the final stop (the car standing still for at least 2 seconds after')
      say('driving). Make sure you stopped behind the parked car for ~5 s before turning the car off, and that')
      say('you picked the right drive (--list, --route).')
    else:
      say('NOT ENOUGH DATA: could not find a stop of at least 2 seconds followed by at least 10 m of reversing.')
      say('Start stopped behind the parked car for a few seconds, then reverse slowly; check --list / --route.')
    raise NotEnoughData()
  if len(data['rs']) == 0:
    say('NOT ENOUGH DATA: the log has no radarState (lead car) messages. openpilot must be running (car on).')
    raise NotEnoughData()

  co = data['co']
  lat_ok = co[:, 7] > 0 if len(co) else np.zeros(0, dtype=bool)
  lat = float(np.median((co[lat_ok, 0] - co[lat_ok, 7]) * 1e-9)) if np.any(lat_ok) else float('nan')
  lat_note = f'{lat * 1000:.0f} ms (from cameraOdometry timestamps)'
  if not (0.0 <= lat <= 0.3):
    lat, lat_note = 0.06, '60 ms (assumed; the log timestamps did not give a usable value)'
  logged_rpy, height, lines, lock_ok = lock_check(rt, data['lc'], lock_pitch)
  t0 = data['cs'][0, 0] * 1e-9
  i0, i1 = anc['i0'], anc['i1']
  pr = approach_pairs(rt, data, anc, end_mode, known, 1.0, lat)

  say('--- Calibration that was actually in use ' + '-' * 37)
  for line in lines:
    say(line)
  say()
  say('--- The run ' + '-' * 66)
  if end_mode:
    say(f"Final stop found {anc['t0'] - t0:.0f} s into the loaded part of the drive, stopped for {anc['t1'] - anc['t0']:.1f} s.")
  else:
    say(f"Starting stop found {anc['t0'] - t0:.0f} s into the loaded part of the drive (stopped {anc['t1'] - anc['t0']:.1f} s),")
    tr = anc['s'][anc['next_run'][0] if anc.get('next_run') else -1] - anc['s'][i1]
    say(f'then {tr:.1f} m of travel by the wheel odometer.')
  say(f'Your tape measurement: {fmt_m(known)} bumper to bumper.  Lens-to-bumper used: {L:.2f} m.  Camera height: {height:.2f} m.')
  win_m = pr['in_win']
  gears = data.get('gear') or []
  if gears:
    tt = anc['t']
    mov = (tt >= anc['win'][0]) & (tt <= anc['win'][1]) & (np.abs(data['cs'][:, 2]) > 0.3)
    gm = [g for g, m in zip(gears, mov) if m]
    if gm and any(g != 'unknown' for g in gm):
      rev = sum(1 for g in gm if g == 'reverse') / len(gm)
      if end_mode and rev > 0.5:
        say('WARNING: the car was mostly in REVERSE before the final stop. For the reverse method use --start-gap.')
      if not end_mode and rev < 0.5:
        say('WARNING: the car does not seem to have been in reverse after the starting stop. For driving up to the')
        say('         parked car use --end-gap instead.')
  steer = data['cs'][:, 4]
  near = (anc['t'] >= anc['win'][0]) & (anc['t'] <= anc['win'][1])
  dist_from_ref = np.abs(anc['s'] - (anc['s'][i0] if end_mode else anc['s'][i1])) + known
  ss = np.abs(steer[near & (dist_from_ref <= 60.0)])
  if len(ss) and float(np.max(ss)) > 30.0:
    say(f'NOTE: the steering wheel was turned up to {float(np.max(ss)):.0f} deg during the run; the method assumes a straight line.')
  stand = win_m & pr['lead'] & (pr['t'] >= anc['t0'] + 0.5) & (pr['t'] <= anc['t1'])
  if np.any(stand):
    off = float(np.median(pr['drel'][stand] - known))
    say(f"While stopped there, openpilot reported the car ahead at {float(np.median(pr['drel'][stand])):.2f} m: that is {off:+.2f} m compared")
    say('with your tape (you saw about +0.8 m before). This includes the fixed camera-to-bumper offset.')
    if abs(off) > 3.0:
      say('WARNING: that is a big difference. Check the --end-gap/--start-gap number and that this is the right drive.')
  say(f'Camera-to-log delay used to line up the timestamps: {lat_note}.')
  say()
  used = pr['used']
  say(f"Samples: {int(np.sum(win_m))} in the run, {int(np.sum(win_m & pr['lead']))} with a lead car, "
      f"{int(np.sum(win_m & pr['lead'] & ~pr['calm']))} dropped for hard accel/braking (body pitch),")
  say(f"         {int(np.sum(win_m & pr['lead'] & pr['calm'] & ~pr['rng']))} outside 4-60 m;  {int(np.sum(used))} used.")
  if int(np.sum(used)) < 40:
    say('NOT ENOUGH DATA: fewer than 40 usable samples. Start about 50 m back, roll slowly and steadily, and make')
    say('sure openpilot shows the parked car as a lead (the marker under it) during the run.')
    raise NotEnoughData()

  g, d = pr['gap'][used], pr['drel'][used]
  fit = fit_pitch(np, g, d, L, height)
  say('--- Reported distance minus true (tape + odometer) distance ' + '-' * 18)
  say(f"{'true gap':>12} {'samples':>8} {'median error':>13} {'fitted model':>13}")
  for lo_b, hi_b in ((4, 10), (10, 20), (20, 30), (30, 40), (40, 60)):
    m = (g >= lo_b) & (g < hi_b) if hi_b < 60 else (g >= lo_b) & (g <= hi_b)
    if np.any(m):
      gm = float(np.median(g[m]))
      fm = ''
      if fit:
        fm = f"{float(d_ground_plane(np, np.array([gm + L]), fit['delta'], height)[0]) - RADAR_TO_CAMERA + fit['c'] - gm:+.2f} m"
      say(f'{lo_b:>5}-{hi_b:<3} m {int(np.sum(m)):>8} {float(np.median(d[m] - g[m])):>+11.2f} m {fm:>13}')
    else:
      say(f'{lo_b:>5}-{hi_b:<3} m {0:>8} {"-":>13}')
  say('(+ means openpilot thought the car was farther away than it really was.)')
  say()
  if fit is None:
    say('NOT ENOUGH DATA to fit the pitch: the run needs usable samples spread from close up to beyond 20 m.')
    say('Start about 50 m back and roll steadily; make sure openpilot shows the parked car as a lead.')
    raise NotEnoughData()

  # rough uncertainty: statistics + odometer scale + lens-to-bumper + camera height
  sys_parts = []
  for label, kw in (('wheel odometer +/-1.5%', 'odo'), ('lens-to-bumper +/-0.15 m', 'L'), ('camera height +/-0.05 m', 'h')):
    shifts = []
    for sgn in (-1, 1):
      if kw == 'odo':
        p2 = approach_pairs(rt, data, anc, end_mode, known, 1.0 + sgn * 0.015, lat)
        u2 = p2['used']
        f2 = fit_pitch(np, p2['gap'][u2], p2['drel'][u2], L, height)
      elif kw == 'L':
        f2 = fit_pitch(np, g, d, L + sgn * 0.15, height)
      else:
        f2 = fit_pitch(np, g, d, L, height + sgn * 0.05)
      if f2:
        shifts.append(abs(f2['delta'] - fit['delta']))
    sys_parts.append((label, max(shifts) if shifts else 0.0))
  stat = fit['sd'] * math.sqrt(2.0) if math.isfinite(fit['sd']) else 0.0   # neighbouring bins are not independent
  tot_unc = math.sqrt(stat ** 2 + sum(x ** 2 for _, x in sys_parts))
  delta = fit['delta']
  locked_rpy = logged_rpy if logged_rpy is not None else [0.0, lock_pitch, LOCK_YAW]
  new_rpy, orient_check = corrected_rpy(rt, locked_rpy, delta, 0.0)
  judged, judge_lines = vs_build_lock(delta, float(new_rpy[1]), lock_pitch, lock_ok)
  say('--- RESULT: camera pitch error from the tape run ' + '-' * 29)
  say(f'Fitted pitch error: {fmt_deg(delta)}  (rough uncertainty +/-{math.degrees(tot_unc):.2f} deg)')
  say(f"   statistics +/-{math.degrees(stat):.2f}; " + '; '.join(f'{lbl} {math.degrees(x):.2f}' for lbl, x in sys_parts))
  say(f"Constant offset c = {fit['c']:+.2f} m (+/-{fit['sc']:.2f}); at short range a perfect camera would read "
      f"{L - RADAR_TO_CAMERA + fit['c']:+.2f} m vs your tape,")
  say(f'   mostly because openpilot measures from a point {RADAR_TO_CAMERA} m ahead of the camera but your bumper is ~{L:.1f} m ahead.')
  say(f"Fit used {fit['nbins']} distance bins ({fit['width']:g} m wide) from {fit['xmin']:.0f} to {fit['xmax']:.0f} m (scatter {fit['rms']:.2f} m"
      + (f", {fit['n_out']} odd bins ignored" if fit['n_out'] else '') + ').')
  for gg in (30.0, 60.0):
    if fit['xmax'] >= gg - 5:
      rep = float(d_ground_plane(np, np.array([gg + L]), delta, height)[0]) - RADAR_TO_CAMERA + fit['c']
      say(f'With this fit, a car truly {gg:.0f} m ahead (bumper to bumper) is reported at about {rep:.1f} m.')
  say('Wheel-odometer distance is typically good to about 1-2%; that is included in the uncertainty above.')
  say('This measures the EFFECTIVE pitch error as it shows up in reported distances. If it is clearly smaller')
  say('than the normal check says, the driving model is partly using car size instead of road position.')
  for line in judge_lines:
    say(line)
  say()
  for line in meaning_lines(rt, judged, height):
    say(line)
  say()
  for line in verdict_lines(judged, float(new_rpy[1]), src_info, approach=True):
    say(line)
  if round(abs(math.degrees(judged)), 2) > WATCH_DEG:
    say(f'   (Math check for the suggested value: {orient_check}.)')
  if abs(math.degrees(delta)) > 4.9:
    say()
    say('WARNING: the fit ran into its +/-5 deg limit; the run probably did not go as planned (wrong drive,')
    say('wrong gap number, or openpilot followed a different car). Do not change anything based on this.')
  if tot_unc > math.radians(0.15):
    say()
    say(f'WARNING: the uncertainty (+/-{math.degrees(tot_unc):.2f} deg) is large. Repeat the run (slower, straighter,')
    say('starting farther back) or use the normal check after a highway drive.')

  if args.csv:
    rows = []
    for i in range(len(pr['t'])):
      if not pr['in_win'][i]:
        continue
      why = 'used' if pr['used'][i] else ('no lead' if not pr['lead'][i] else ('hard accel' if not pr['calm'][i] else 'out of 4-60 m'))
      rows.append((f"{pr['t'][i] - t0:.3f}", f"{pr['gap'][i]:.3f}", f"{pr['drel'][i]:.3f}", f"{pr['drel'][i] - pr['gap'][i]:.3f}",
                   f"{pr['a'][i]:.3f}", f"{pr['v'][i]:.3f}", why))
    write_csv(args.csv, ['time_s', 'true_gap_m', 'dRel_m', 'dRel_minus_gap_m', 'aEgo', 'speed_ms', 'status'], rows)
  return EXIT_OK


def onroad_now():
  """True while openpilot is driving (car on). Only reads the IsOnroad param file."""
  try:
    with open(os.path.join(PARAMS_DIR, 'IsOnroad'), 'rb') as f:
      return f.read().strip() == b'1'
  except OSError:
    return False


# ============================================================================================ csv / args / main
def check_csv_path(path, args):
  full = os.path.realpath(os.path.abspath(path))
  for protected in (args.log_root, args.op_dir, '/data/params', '/persist', '/data/media'):
    pr = os.path.realpath(os.path.abspath(protected))
    if full == pr or full.startswith(pr.rstrip('/') + '/'):
      raise UsageError(f'--csv must not point inside {protected}. Save it somewhere else, e.g. /tmp/pitch.csv')
  if os.path.exists(full):
    raise UsageError(f'--csv: {path} already exists; this script never overwrites files. Pick a new name.')
  if not os.path.isdir(os.path.dirname(full)):
    raise UsageError(f'--csv: the folder {os.path.dirname(full)} does not exist.')
  return full


def write_csv(path, header, rows):
  import csv
  with open(path, 'x', newline='') as f:     # 'x': refuse to overwrite
    w = csv.writer(f)
    w.writerow(header)
    w.writerows(rows)
  say()
  say(f'Saved {len(rows)} rows to {path}')


def build_parser():
  p = argparse.ArgumentParser(
    prog='mk7_pitch_check.py',
    description='Read-only check of how far the comma 3X camera pitch is from the locked FIXED_PITCH (0.0236 rad).\n'
                'Default ("residual"): uses an ordinary drive with some straight road at 15+ mph, no measuring.\n'
                '"approach": tape-measure method (experimental), see below.',
    epilog=EPILOG, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument('mode', nargs='?', default='residual', choices=['residual', 'approach'],
                 help='residual (default) or approach')
  p.add_argument('--list', action='store_true', help='list recent drives and exit')
  p.add_argument('--route', metavar='NAME', help='drive name from --list, or its number there (1 = newest)')
  p.add_argument('--routes', metavar='N', type=int, help='normal check: combine the N newest drives')
  p.add_argument('--end-gap', metavar='DIST', type=parse_length, help='approach: tape gap after stopping behind the parked car')
  p.add_argument('--start-gap', metavar='DIST', type=parse_length, help='approach (reverse variant): tape gap before reversing away')
  p.add_argument('--lens-to-bumper', metavar='DIST', type=parse_length, default=DEFAULT_LENS_TO_BUMPER,
                 help=f'approach: camera lens to front bumper, horizontally (default {DEFAULT_LENS_TO_BUMPER} m)')
  p.add_argument('--max-samples', metavar='N', type=int, default=20000, help='normal check: stop reading after N usable samples (default 20000)')
  p.add_argument('--csv', metavar='FILE', help='also save the samples to this NEW file')
  p.add_argument('--log-root', metavar='DIR', default=DEFAULT_LOG_ROOT, help=f'where the drive logs are (default {DEFAULT_LOG_ROOT})')
  p.add_argument('--op-dir', metavar='DIR', default=DEFAULT_OP_DIR, help=f'openpilot folder (default {DEFAULT_OP_DIR})')
  p.add_argument('--allow-onroad', action='store_true', help='run even while the car is on and openpilot is driving (not recommended)')
  return p




def main(argv=None):
  parser = build_parser()
  args = parser.parse_args(argv)          # argparse exits with code 2 on bad options
  try:
    if args.mode == 'approach':
      if (args.end_gap is None) == (args.start_gap is None):
        raise UsageError('approach mode needs exactly one of --end-gap (drive up and stop) or --start-gap (reverse away).\n'
                         'Example: python3 mk7_pitch_check.py approach --end-gap 3.2      (see --help)')
      known = args.end_gap if args.end_gap is not None else args.start_gap
      if not (0.5 <= known <= 15.0):
        raise UsageError(f'The tape gap {known:.2f} m looks wrong; stop 2-5 m behind the parked car and measure that gap.')
      if args.routes:
        raise UsageError('approach mode uses one drive; use --route instead of --routes.')
      if not (0.5 <= args.lens_to_bumper <= 4.0):
        raise UsageError('--lens-to-bumper should be roughly 1.5-3 m for a windshield mount.')
    elif args.end_gap is not None or args.start_gap is not None:
      raise UsageError('--end-gap/--start-gap only make sense with the approach mode:  mk7_pitch_check.py approach --end-gap 3.2')
    if args.route and args.routes:
      raise UsageError('use either --route or --routes, not both.')
    if args.routes is not None and args.routes < 1:
      raise UsageError('--routes must be 1 or more.')
    if args.max_samples < MIN_SAMPLES_HARD:
      raise UsageError(f'--max-samples must be at least {MIN_SAMPLES_HARD}.')
    if args.csv:
      args.csv = check_csv_path(args.csv, args)

    routes = discover_routes(args.log_root)
    if args.list:
      if not routes:
        say(f'No drives found in {args.log_root}.')
        return EXIT_NO_DATA
      print_route_list(routes, max(args.routes or 0, 20))
      return EXIT_OK
    if not routes:
      say(f'No drives found in {args.log_root}. Drive first (car on, openpilot running), then run this again.')
      return EXIT_NO_DATA
    if args.route:
      chosen = [resolve_route(routes, args.route)]
    else:
      chosen = routes[:args.routes or 1]

    if onroad_now() and not args.allow_onroad:
      raise UsageError('openpilot is running right now (car on). Please run this with the car parked and turned off,\n'
                       'so reading the logs cannot slow openpilot down. (To run it anyway add --allow-onroad.)')
    try:
      os.nice(10)          # be gentle: lower priority for this process only
    except (OSError, AttributeError):
      pass
    rt = setup_runtime(args.op_dir)
    src = find_lock_in_source(args.op_dir)
    lock_pitch = src[3] if src[3] is not None else LOCK_PITCH_DEFAULT
    if args.mode == 'approach':
      return run_approach(rt, args, chosen[0], src, lock_pitch)
    return run_residual(rt, args, chosen, src, lock_pitch)
  except UsageError as e:
    sys.stderr.write(f'\nProblem: {e}\n')
    return EXIT_USAGE
  except NotEnoughData:
    return EXIT_NO_DATA
  except KeyboardInterrupt:
    sys.stderr.write('\nStopped.\n')
    return EXIT_USAGE


if __name__ == '__main__':
  sys.exit(main())
