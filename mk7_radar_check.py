#!/usr/bin/env python3
"""
mk7_radar_check.py - read-only: can the factory ACC radar give openpilot the distance to the car ahead? (car OFF)

INSTALL OR UPDATE (on the comma, over SSH):
  cd /data/openpilot && git fetch sngrepo claude/focused-babbage-y90aag && git show FETCH_HEAD:mk7_radar_check.py > /data/mk7_radar_check.py

RUN IT WITH THE CAR OFF:

  python3 /data/mk7_radar_check.py                 the newest drives with real openpilot driving (about 15 min in all)
  python3 /data/mk7_radar_check.py 0 1 2 3 4       the 5 newest drives together (0 = newest), driven or not
  python3 /data/mk7_radar_check.py 0-4             the same
  python3 /data/mk7_radar_check.py 00000340        a drive can also be a route-name prefix

It works on drives from ANY branch, also ones recorded before the radar lead existed: everything it needs (the
raw CAN from the radar, the camera's lead) is in every drive's rlog.

The radar does not send its raw targets anywhere the comma can read. It does send, for the instrument cluster,
the distance to the one car it is following: ACC_04 ACC_Abstand_Abstandswarner, in whole metres. The radar lead
(/data/radar_lead = 1) takes the distance of the camera's lead from that signal. This script answers, from your
own drives, the things that cannot be known without the car:
  1. is that signal there while openpilot drives, and at which speeds (also in stop and go)?
  2. is it in metres, i.e. does it agree with the camera where the camera is good (8-60 m)?
  3. what would the radar lead have done on these drives: how soon it is trusted, how much of the time the
     distance would have come from the radar, how far that moves the lead, how jumpy the lead's speed is
  4. what the radar's own cruise control does in the background while openpilot drives
  5. v1.1: if that one field is empty, is the distance anywhere else? It looks through every message on the radar's
     bus, every run of 8 to 16 bits, for a number that follows the camera's distance (or closing speed) to the car
     ahead, with our own speed taken out. It needs 2 minutes of driving behind a car, so parked drives say nothing.
  v1.2: v1.0 and v1.1 read every real drive as parked (a message stamped a little earlier than the one before it
     was taken for the start of a new drive, which reset the car's speed). Results from v1.0 / v1.1 don't count.
The replay uses the same maths as the car (opendbc volkswagen/radar_interface.py and selfdrive/controls/radard.py).
At the end there is a short block to paste back. Only reads logs. Writes nothing.
"""
import sys
sys.dont_write_bytecode = True   # never leave .pyc files behind on the device

import argparse  # noqa: E402
from array import array  # noqa: E402
import bz2       # noqa: E402
import os        # noqa: E402
import re        # noqa: E402

TOOL_VERSION = '1.2 (2026-11-17)'
OP_DIR = '/data/openpilot'
ROOT = '/data/media/0/realdata'
VENV_PYTHON = '/usr/local/venv/bin/python3'
REEXEC_FLAG = 'MK7_RADAR_CHECK_REEXEC'
ONROAD_PARAM = '/data/params/d/IsOnroad'
RADAR_LEAD_FILE = '/data/radar_lead'
SEG_RE = re.compile(r'^([0-9a-f]{8}--[0-9a-f]+)--(\d+)$')
RLOGS = ('rlog.zst', 'rlog.bz2', 'rlog')
QLOGS = ('qlog.zst', 'qlog.bz2', 'qlog')
DBC_REL = ('opendbc_repo/opendbc/dbc/vw_mqb.dbc', 'opendbc/dbc/vw_mqb.dbc')
ZSTD_MAGIC = b'\x28\xb5\x2f\xfd'
NAN = float('nan')
MPH = 2.23694

# CAN (vw_mqb.dbc). The radar is on bus 2 with the gateway harness, the wheel speeds on bus 0.
PT_BUS, RADAR_BUS = 0, 2
ESP_19, ACC_06, ACC_02, ACC_04 = 0x0B2, 0x122, 0x30C, 0x324
DISTANCE_MAX_VALID = 508     # 509 = free driving (no car), 510 = undefined, 511 = no display
SIGNAL_TIMEOUT = 0.5         # s without a fresh ACC_04 before the radar track is dropped
RADAR_TO_CAMERA = 1.52       # m, radard: the camera's lead distance is measured from the camera
DT_MDL = 0.05                # s, one camera / planner frame
ACC_STATUS = {0: 'off', 1: 'init', 2: 'standby', 3: 'active', 4: 'override', 5: 'shutting off', 6: 'fault', 7: 'fault (permanent)'}
SPEEDS = ((-1.0, 0.3, 'stopped'), (0.3, 5.0, 'under 11 mph'), (5.0, 15.0, '11-34 mph'), (15.0, 25.0, '34-56 mph'), (25.0, 99.0, 'over 56 mph'))
RANGES = ((0, 20), (20, 40), (40, 60), (60, 100), (100, 250))
COLS = ('t', 'v_ego', 'op', 'raw', 'idx', 'st', 'acc', 'trk_d', 'trk_v', 'prob', 'vis_d', 'vis_v', 'trusted', 'fus_d', 'fus_v', 'log_radar',
        'obj', 'typ')
# "driving behind a car": moving, the camera sure of a car this far ahead. What the search and the verdict need.
FOLLOW_PROB, FOLLOW_V, FOLLOW_D, FOLLOW_MIN_S = 0.9, 2.0, (4.0, 120.0), 120.0
STRONG = 0.9                 # search score from which a number really follows the car ahead
NEW_DRIVE_GAP = 5.0          # s
AUTO_MIN, AUTO_ROUTES = 15.0, 60   # auto: minutes of openpilot driving to collect, newest drives to look through

np = None


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


def setup(op_dir):
  op_dir = os.path.abspath(op_dir)
  for p in (os.path.dirname(op_dir), op_dir):
    if os.path.isdir(p) and p not in sys.path:
      sys.path.insert(0, p)
  try:
    import numpy
    import zstandard
    from cereal import log
  except Exception as e:
    maybe_reexec()
    raise UsageError(f"This needs openpilot's Python libraries (numpy, pycapnp, zstandard, cereal): {e}\n"
                     f"  Run it on the comma (openpilot in {OP_DIR}).") from None
  return numpy, zstandard, log


# ============================================================================================ logs
def routes_on_disk(root):
  if not os.path.isdir(root):
    raise UsageError(f'The log folder {root} does not exist.')
  segs = {}
  for d in os.listdir(root):
    m = SEG_RE.match(d)
    if m:
      segs.setdefault(m.group(1), []).append(int(m.group(2)))
  return sorted(segs, reverse=True), segs


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


def seg_file(root, route, n, kinds=RLOGS):
  for k in kinds:
    p = f'{root}/{route}--{n}/{k}'
    if os.path.isfile(p) and os.path.getsize(p) > 0:
      return p
  return None


def events_of(zstd, log, path):
  with open(path, 'rb') as f:
    raw = f.read()
  if raw[:3] == b'BZh':
    try:
      raw = bz2.BZ2Decompressor().decompress(raw)
    except (OSError, EOFError, ValueError):
      raw = b''
  elif raw[:4] == ZSTD_MAGIC:
    parts, data = [], raw
    while data:   # concatenated frames; a damaged tail keeps the readable part
      d = zstd.ZstdDecompressor().decompressobj()
      try:
        parts.append(d.decompress(data))
      except zstd.ZstdError:
        break
      if not getattr(d, 'eof', True):
        break
      data = d.unused_data
    raw = b''.join(parts)
  try:
    yield from log.Event.read_multiple_bytes(raw)
  except Exception:
    return


# ============================================================================================ CAN
def bits(dat, start, size):
  """A little-endian (Intel) DBC signal: raw value of `size` bits from bit `start`."""
  return (int.from_bytes(dat, 'little') >> start) & ((1 << size) - 1)


def wheel_speed(dat):
  """ESP_19: mean of the four wheel speeds, m/s (what carstate.py uses for vEgo)."""
  return sum(bits(dat, s, 16) for s in (0, 16, 32, 48)) * 0.0075 / 4.0 / 3.6


def acc_04(dat):
  """ACC_04: (distance raw, system text). Distance 1..508 = metres to the radar's lead."""
  return bits(dat, 42, 9), bits(dat, 62, 2)


def acc_02(dat):
  """ACC_02: the cluster's lead bar position (ACC_Abstandsindex, no unit)."""
  return bits(dat, 24, 10)


def acc_02_object(dat):
  """ACC_02: the cluster's lead symbol (ACC_Relevantes_Objekt, 0 = off)."""
  return bits(dat, 46, 2)


def acc_typ(dat):
  """ACC_06: ACC_Typ (0 basic, 1 follow to stop, 2 stop and go)."""
  return bits(dat, 58, 2)


def acc_06(dat):
  """ACC_06: (ACC_Status_ACC, ACC_Sollbeschleunigung_02 in m/s2; 3.01 = neutral)."""
  return bits(dat, 60, 3), bits(dat, 24, 11) * 0.005 - 7.22


# ============================================================================================ the car's maths
class LeadDistanceTracker:
  """Same as opendbc/car/volkswagen/radar_interface.py LeadDistanceTracker (kept equal by a unit test)."""
  BIN_HALF_WIDTH = 0.5
  LEAD_ACCEL_STD = 3.0
  IN_BIN_STD = 0.3
  IN_BIN_CORR_TIME = 0.5
  EDGE_STD = 0.1
  EDGE_CORR_TIME = 0.1
  JUMP_THRESHOLD = 2.0
  INIT_SPEED_STD = 5.0
  READY_SPEED_STD = 1.0
  READY_MIN_TIME = 0.5

  def __init__(self):
    self.track_id = 0
    self.reset()

  def reset(self):
    self.tracking = False
    self.d = self.v_lead = self.p_dd = self.p_dv = self.p_vv = self.age = self.since_meas = 0.0

  @property
  def ready(self):
    return self.tracking and self.age >= self.READY_MIN_TIME and self.p_vv < self.READY_SPEED_STD ** 2

  def predict(self, dt, v_ego):
    if not self.tracking or dt <= 0.0:
      return
    self.d += (self.v_lead - v_ego) * dt
    self.age += dt
    self.since_meas += dt
    q = self.LEAD_ACCEL_STD ** 2
    self.p_dd += 2.0 * dt * self.p_dv + dt * dt * self.p_vv + q * dt ** 4 / 4.0
    self.p_dv += dt * self.p_vv + q * dt ** 3 / 2.0
    self.p_vv += q * dt * dt

  def update(self, distance, v_ego):
    if not self.tracking:
      self._start(distance, v_ego)
      return
    innovation = distance - self.d
    if abs(innovation) > self.BIN_HALF_WIDTH + self.JUMP_THRESHOLD:
      self._start(distance, v_ego)
      return
    dt_meas = max(self.since_meas, 1e-3)
    if abs(innovation) <= self.BIN_HALF_WIDTH:
      innovation = 0.0
      r = self.IN_BIN_STD ** 2 * max(1.0, self.IN_BIN_CORR_TIME / dt_meas)
    else:
      innovation -= self.BIN_HALF_WIDTH if innovation > 0 else -self.BIN_HALF_WIDTH
      r = self.EDGE_STD ** 2 * max(1.0, self.EDGE_CORR_TIME / dt_meas)
    s = self.p_dd + r
    k_d = self.p_dd / s
    k_v = self.p_dv / s
    self.d += k_d * innovation
    self.v_lead += k_v * innovation
    self.p_vv -= k_v * self.p_dv
    self.p_dv *= (1.0 - k_d)
    self.p_dd *= (1.0 - k_d)
    self.since_meas = 0.0

  def _start(self, distance, v_ego):
    self.track_id += 1
    self.tracking = True
    self.d = distance
    self.v_lead = v_ego
    self.p_dd = self.IN_BIN_STD ** 2
    self.p_dv = 0.0
    self.p_vv = self.INIT_SPEED_STD ** 2
    self.age = 0.0
    self.since_meas = 0.0


class Fusion:
  """Same decisions as radard.py RadarTrustGate + RangeRadarFusion for leadOne, as if /data/radar_lead were 1."""
  MIN_VISION_PROB, MIN_COMPARE_DIST, MAX_COMPARE_DIST = 0.9, 8.0, 60.0
  MIN_COMPARE_TIME, SCORE_TAU, TRUST_SCORE, DISTRUST_SCORE = 10.0, 5.0, 0.8, 0.5
  SPEED_BIAS_TAU, SPEED_BIAS_MAX, FAR_BP = 5.0, 2.0, (30.0, 60.0)

  def __init__(self):
    self.score = 0.0
    self.compare_frames = 0
    self.trusted = False
    self.bias = 0.0
    self.bias_track = None
    self.confirmed = None

  def gate(self, trk, prob, vis_d):
    if trk is None or prob < self.MIN_VISION_PROB or not self.MIN_COMPARE_DIST <= vis_d <= self.MAX_COMPARE_DIST:
      return
    agree = abs(trk[1] - vis_d) < max(0.2 * vis_d, 3.0)
    self.score += (float(agree) - self.score) * DT_MDL / (self.SCORE_TAU + DT_MDL)
    self.compare_frames += 1
    if self.trusted and self.score < self.DISTRUST_SCORE:
      self.trusted = False
    elif not self.trusted and self.compare_frames * DT_MDL >= self.MIN_COMPARE_TIME and self.score >= self.TRUST_SCORE:
      self.trusted = True

  def lead(self, trk, prob, vis_d, vis_v, model_v):
    """trk = (id, distance, lead speed) or None. Returns (distance, lead speed) from the radar lead, or None when
    the lead would have stayed the camera's. vis_v = the camera's lead speed as radard uses it, model_v = lead.v[0]."""
    if prob <= 0.5 or not self.trusted or trk is None:
      self.confirmed = None
      return None
    tid, d, v = trk
    same_car = abs(d - vis_d) < max(vis_d * 0.25, 5.0) and (abs(v - model_v) < 10 or v > 3)
    if not (same_car or (self.confirmed == tid and d < vis_d)):
      self.confirmed = None
      return None
    self.confirmed = tid
    if self.bias_track != tid:
      self.bias, self.bias_track = 0.0, tid
    self.bias += (min(max(v - vis_v, -self.SPEED_BIAS_MAX), self.SPEED_BIAS_MAX) - self.bias) * DT_MDL / (self.SPEED_BIAS_TAU + DT_MDL)
    far = min(max((d - self.FAR_BP[0]) / (self.FAR_BP[1] - self.FAR_BP[0]), 0.0), 1.0)
    return d, (1.0 - far) * (vis_v + self.bias) + far * v


# ============================================================================================ replay
class Replay:
  def __init__(self):
    self.rows = []
    self.bus2 = {}            # radar-side bus: address -> newest payload as one little-endian number
    self.bus2_n = {}          # address -> frames seen
    self.a04 = {}             # ACC_04 payloads (without checksum and counter) -> count
    self.scan_n = 0           # frames kept for the search: moving, camera sure of a car 4-120 m ahead
    self.scan_d, self.scan_v, self.scan_e = array('d'), array('d'), array('d')
    self.scan_addr = {}       # address -> array('Q'), one payload per kept frame
    self.branch = '?'
    self.op_long = None
    self.last_t = -1.0
    self.new_drive()

  def new_drive(self):
    self.tracker = LeadDistanceTracker()
    self.fusion = Fusion()
    self.v_wheel = self.v_ego = 0.0
    self.radar_t = -1e9
    self.raw = self.idx = self.obj = self.st = self.acc = self.typ = NAN
    self.op = False
    self.log_radar = False

  def on_can(self, t, evt):
    got_radar = False
    for c in evt.can:
      src = c.src
      if src == PT_BUS:
        if c.address == ESP_19:
          self.v_wheel = wheel_speed(c.dat)
      elif src == RADAR_BUS:
        addr, dat = c.address, c.dat
        if len(dat) > 8:
          continue
        val = int.from_bytes(dat, 'little')
        self.bus2[addr] = val
        self.bus2_n[addr] = self.bus2_n.get(addr, 0) + 1
        if addr == ACC_04:
          self.raw, text = acc_04(dat)
          got_radar = True
          if len(self.a04) < 5000 or (val >> 16) in self.a04:
            self.a04[val >> 16] = self.a04.get(val >> 16, 0) + 1
        elif addr == ACC_02:
          self.idx, self.obj = acc_02(dat), acc_02_object(dat)
        elif addr == ACC_06:
          self.st, self.acc = acc_06(dat)
          self.typ = acc_typ(dat)
    # the same steps as RadarInterface.update
    self.tracker.predict(min(max(t - self.last_t, 0.0), 0.2), self.v_wheel)
    if got_radar:
      self.radar_t = t
      if 0 < self.raw <= DISTANCE_MAX_VALID and text == 0:
        self.tracker.update(float(self.raw), self.v_wheel)
      else:
        self.tracker.reset()
    elif t - self.radar_t > SIGNAL_TIMEOUT:
      self.tracker.reset()

  def on_model(self, t, model):
    if len(model.leadsV3) < 2:
      return
    lead = model.leadsV3[0]
    model_v_ego = model.velocity.x[0] if len(model.velocity.x) else self.v_ego
    vis_d = lead.x[0] - RADAR_TO_CAMERA
    vis_v = self.v_ego + (lead.v[0] - model_v_ego)
    trk = (self.tracker.track_id, self.tracker.d, self.tracker.v_lead) if self.tracker.ready else None
    self.fusion.gate(trk, lead.prob, vis_d)
    fused = self.fusion.lead(trk, lead.prob, vis_d, vis_v, lead.v[0])
    self.rows.append((t, self.v_ego, float(self.op), self.raw, self.idx, self.st, self.acc,
                      trk[1] if trk else NAN, trk[2] if trk else NAN, lead.prob, vis_d, vis_v,
                      float(self.fusion.trusted), fused[0] if fused else NAN, fused[1] if fused else NAN, float(self.log_radar),
                      self.obj, self.typ))
    if lead.prob > FOLLOW_PROB and FOLLOW_D[0] < vis_d < FOLLOW_D[1] and self.v_ego > FOLLOW_V:
      if len(self.scan_addr) != len(self.bus2):
        for addr in self.bus2:
          if addr not in self.scan_addr:
            self.scan_addr[addr] = array('Q', bytes(8 * self.scan_n))
      for addr, arr in self.scan_addr.items():
        arr.append(self.bus2[addr])
      self.scan_d.append(vis_d)
      self.scan_v.append(vis_v - self.v_ego)
      self.scan_e.append(self.v_ego)
      self.scan_n += 1

  def read(self, events):
    for evt in events:
      t = evt.logMonoTime * 1e-9
      # A log is not in strict time order: a message is stamped when it is made and written when it is sent
      # (modelV2 tens of ms later). Only a big jump either way is a new drive, which starts from scratch as the car does.
      if abs(t - self.last_t) > NEW_DRIVE_GAP:
        self.new_drive()
        self.last_t = t
      which = evt.which()
      if which == 'can':
        self.on_can(t, evt)
      elif which == 'modelV2':
        self.on_model(t, evt.modelV2)
      elif which == 'carState':
        self.v_ego = evt.carState.vEgo
      elif which == 'carControl':
        self.op = evt.carControl.longActive
      elif which == 'radarState':
        self.log_radar = evt.radarState.leadOne.status and evt.radarState.leadOne.radar
      elif which == 'carParams':
        self.op_long = evt.carParams.openpilotLongitudinalControl
      elif which == 'initData':
        self.branch = evt.initData.gitBranch
      self.last_t = max(self.last_t, t)


def driven_by_segment(zstd, log, root, route, seg_nums):
  """From the small logs (qlog): for each minute of a drive, the share spent moving with cruise engaged.
  None for a minute without a qlog."""
  out = {}
  for n in seg_nums:
    p = seg_file(root, route, n, QLOGS)
    if p is None:
      out[n] = None
      continue
    tot = drv = 0
    for evt in events_of(zstd, log, p):
      if evt.which() == 'carState':
        tot += 1
        drv += bool(evt.carState.vEgo > FOLLOW_V and evt.carState.cruiseState.enabled)
    out[n] = drv / tot if tot else 0.0
  return out


def pick_auto(zstd, log, root, routes, segs):
  """The newest drives with real openpilot driving, newest first, until AUTO_MIN minutes are together.
  Returns [(route, [segment numbers worth reading])], oldest first."""
  picked, total = [], 0.0
  for route in routes[:AUTO_ROUTES]:
    share = driven_by_segment(zstd, log, root, route, sorted(segs[route]))
    minutes = sum(s for s in share.values() if s)
    if minutes < 1.0:
      continue
    picked.append((route, [n for n, s in sorted(share.items()) if s is None or s >= 0.1]))
    say(f"drive {rid(route)}: {minutes:.0f} min of openpilot driving")
    total += minutes
    if total >= AUTO_MIN:
      break
  return sorted(picked)


# ============================================================================================ search
def msg_names(op_dir):
  for rel in DBC_REL:
    try:
      with open(os.path.join(op_dir, rel)) as f:
        return {int(m.group(1)): m.group(2) for m in re.finditer(r'^BO_ (\d+) (\w+):', f.read(), re.M)}
    except OSError:
      continue
  return {}


def search(replay):
  """Looks through every message on the radar's bus for a number that follows the camera's distance to the car
  ahead (and one that follows the closing speed). Every run of 8, 10, 12 or 16 bits is tried. Our own speed is
  taken out first: at a steady time gap the distance follows our speed, and so would any copy of our speed.
  Returns {'d': [...], 'v': [...]}, best first: (score 0-1, address, first bit, bits, units per metre or m/s, offset,
  different values, signed)."""
  n = replay.scan_n
  if n * DT_MDL < FOLLOW_MIN_S:
    return None
  step = max(1, n // 20000)
  e = np.array(replay.scan_e)[::step]
  ec = e - e.mean()
  ee = float(ec @ ec)

  def without_speed(x):
    xc = x - x.mean(axis=1, keepdims=True)
    return xc - ((xc @ ec) / ee)[:, None] * ec if ee > 0 else xc

  out = {}
  for key, src, signed_too in (('d', replay.scan_d, False), ('v', replay.scan_v, True)):
    target = np.array(src)[::step]
    tr = without_speed(target[None])[0]
    tn = float(np.linalg.norm(tr))
    best = []
    for addr, arr in replay.scan_addr.items():
      v = np.frombuffer(arr, dtype=np.uint64)[::step]
      if tn == 0 or v.min() == v.max():
        continue
      fields, meta = [], []
      for size in (8, 10, 12, 16):
        for start in range(12, 65 - size):    # bits 0-11 are the checksum and the counter
          f = ((v >> np.uint64(start)) & np.uint64((1 << size) - 1)).astype(np.float32)
          if f.min() == f.max():
            continue
          fields.append(f)
          meta.append((start, size, False))
          if signed_too:
            fields.append(np.where(f >= (1 << (size - 1)), f - (1 << size), f).astype(np.float32))
            meta.append((start, size, True))
      if not fields:
        continue
      X = np.stack(fields)
      Xr = without_speed(X)
      r = (Xr @ tr) / (np.linalg.norm(Xr, axis=1) * tn + 1e-9)
      i = int(np.argmax(np.abs(r)))
      slope, offset = (float(c) for c in np.polyfit(target, X[i], 1))
      best.append((float(abs(r[i])), addr, meta[i][0], meta[i][1], slope, offset, len(np.unique(X[i])), meta[i][2]))
    out[key] = sorted(best, reverse=True)
  return out


# ============================================================================================ report
def pc(part, whole):
  return f'{100.0 * part / whole:.0f}%' if whole else 'n/a'


def states(st, sel):
  sel = sel & np.isfinite(st)
  return sorted(((int(s), int((st[sel] == s).sum())) for s in np.unique(st[sel])), key=lambda kv: -kv[1]), int(sel.sum())


def analyse(replay):
  """Everything the report and the paste block need, as a dict of plain numbers."""
  rows = replay.rows
  a = {name: np.array([r[i] for r in rows], dtype=float) for i, name in enumerate(COLS)}
  n = len(rows)
  n04 = replay.bus2_n.get(ACC_04, 0)
  R = {'min': n * DT_MDL / 60.0, 'op_min': float(a['op'].sum()) * DT_MDL / 60.0, 'n04': n04, 'n06': replay.bus2_n.get(ACC_06, 0),
       'hz': n04 / max(n * DT_MDL, 1.0), 'moving_min': float((a['v_ego'] > FOLLOW_V).sum()) * DT_MDL / 60.0,
       'follow_s': replay.scan_n * DT_MDL, 'bus2': len(replay.bus2_n)}
  raw, got = a['raw'], np.isfinite(a['raw'])
  has = got & (raw > 0) & (raw <= DISTANCE_MAX_VALID)
  R.update(dist=pc(has.sum(), got.sum()), free=pc((raw == 509).sum(), got.sum()), nodisp=pc((raw >= 510).sum(), got.sum()),
           zero=pc((raw == 0).sum(), got.sum()), has_s=float(has.sum()) * DT_MDL)
  # what ACC_04 carries at all: how many different payloads, which bits ever change, the most common one
  changed, first = 0, None
  for val in replay.a04:
    first = val if first is None else first
    changed |= val ^ first
  top = max(replay.a04.items(), key=lambda kv: kv[1]) if replay.a04 else (0, 0)
  R.update(a04_kinds=len(replay.a04), a04_changed=f'{changed:012x}', a04_top=f'{top[0]:012x}', a04_top_share=pc(top[1], n04))

  sure = a['prob'] > FOLLOW_PROB
  R['by_speed'] = []
  for lo, hi, label in SPEEDS:
    sel = sure & (a['v_ego'] >= lo) & (a['v_ego'] < hi)
    R['by_speed'].append((label, int(sel.sum()), int((has & sel).sum())))

  both = sure & has & (a['vis_d'] >= 8.0) & (a['vis_d'] <= 60.0)
  R['both_s'] = float(both.sum()) * DT_MDL
  R['slope'] = R['offset'] = R['corr'] = R['med'] = R['band'] = NAN
  if both.sum() >= 200:
    x, y = a['vis_d'][both], raw[both]
    R['slope'], R['offset'] = (float(v) for v in np.polyfit(x, y, 1)) if np.std(x) > 0 else (NAN, NAN)
    R['corr'] = float(np.corrcoef(x, y)[0, 1]) if np.std(x) > 0 and np.std(y) > 0 else NAN
    R['med'] = float(np.median(y - x))
    R['band'] = float(np.mean(np.abs(y - x) < np.maximum(0.2 * x, 3.0)))

  # the cluster's lead bar and lead symbol (ACC_02), whatever ACC_04 does
  got02 = np.isfinite(a['idx'])
  bar = got02 & (a['idx'] >= 1) & (a['idx'] <= 1021)
  follow = sure & (a['v_ego'] > FOLLOW_V) & (a['vis_d'] > FOLLOW_D[0]) & (a['vis_d'] < FOLLOW_D[1])
  R.update(bar=pc(bar.sum(), got02.sum()), bar_follow=pc((bar & follow).sum(), follow.sum()), sym=pc((a['obj'] > 0).sum(), got02.sum()),
           sym_follow=pc(((a['obj'] > 0) & follow).sum(), follow.sum()), idx=None)
  idx_ok = bar & follow
  if idx_ok.sum() >= 200 and np.std(a['idx'][idx_ok]) > 0:
    s, o = np.polyfit(a['vis_d'][idx_ok], a['idx'][idx_ok], 1)
    R['idx'] = (float(s), float(o), float(np.corrcoef(a['vis_d'][idx_ok], a['idx'][idx_ok])[0, 1]))

  trusted, fused, cam = a['trusted'] > 0, np.isfinite(a['fus_d']), a['prob'] > 0.5
  first_trust = np.flatnonzero(trusted)
  R.update(trusted=pc(trusted.sum(), n), cam_s=float(cam.sum()) * DT_MDL, fused=pc((fused & cam).sum(), cam.sum()),
           first_trust=float(a['t'][first_trust[0]] - a['t'][0]) if len(first_trust) else NAN)
  R['moved'] = []
  for lo, hi in RANGES:
    sel = fused & (a['vis_d'] >= lo) & (a['vis_d'] < hi)
    if sel.sum() >= 100:
      d = a['fus_d'][sel] - a['vis_d'][sel]
      R['moved'].append((lo, hi, float(sel.sum()) * DT_MDL, float(np.median(d)), float(np.percentile(np.abs(d), 90))))
  # how jumpy the lead's speed is from one frame to the next, where the camera is weakest
  far = fused & np.roll(fused, 1) & (a['fus_d'] >= 60.0) & (np.abs(a['t'] - np.roll(a['t'], 1) - DT_MDL) < 0.02)
  R['jit'] = None
  if far.sum() >= 200:
    R['jit'] = (float(np.std((a['vis_v'] - np.roll(a['vis_v'], 1))[far]) / DT_MDL),
                float(np.std((a['fus_v'] - np.roll(a['fus_v'], 1))[far]) / DT_MDL), float(far.sum()) * DT_MDL)

  R['acc_op'], R['acc_op_n'] = states(a['st'], a['op'] > 0)
  R['acc_off'], R['acc_off_n'] = states(a['st'], a['op'] == 0)
  typ = a['typ'][np.isfinite(a['typ'])]
  R['typ'] = int(np.bincount(typ.astype(int)).argmax()) if len(typ) else -1
  drv3 = (a['op'] > 0) & (a['st'] == 3)
  R['acc_own'] = float(np.mean(a['acc'][drv3] < 3.0)) if drv3.sum() else NAN
  R['logged'] = pc((a['log_radar'] > 0).sum(), cam.sum())
  R['search'] = search(replay)
  return R


def verdict(R):
  best = R['search']['d'][0] if R['search'] and R['search']['d'] else None
  if R['n04'] == 0 and R['bus2'] == 0:
    return 'NO-BUS', (f"Nothing at all was received on bus {RADAR_BUS} in these drives, so the radar cannot be read here.")
  if R['follow_s'] < FOLLOW_MIN_S:
    return 'TOO-LITTLE', (f"NOT A VERDICT YET: these drives have only {R['follow_s']:.0f} s of driving behind a car (2 minutes needed);"
                          f" the car was moving for {R['moving_min']:.1f} of {R['min']:.1f} min. Run it with no drive numbers so it picks drives"
                          " with real openpilot driving:  python3 /data/mk7_radar_check.py")
  if R['has_s'] >= 10.0 and R['both_s'] >= 10.0:
    if R['band'] >= 0.8 and 0.8 < R['slope'] < 1.2:
      return 'GOOD', ("The radar's distance is there and agrees with the camera in metres. The radar lead will work on this"
                      " car. To switch it on (on this branch): echo 1 > /data/radar_lead")
    return 'MISMATCH', ("The radar reports distances in ACC_04, but they do not follow the camera's closely enough to be trusted."
                        " The car checks the same thing while driving and would keep the radar lead off by itself. Paste the"
                        " block below so the scaling can be worked out.")
  if best is not None and best[0] >= STRONG:
    return 'OTHER-SIGNAL', ("The distance field the radar lead reads (ACC_04) stays empty on this car, but another number the radar"
                            " sends does follow the distance to the car ahead (section 4). The radar lead as built stays idle;"
                            " paste the block below and it can be pointed at that number.")
  return 'NOTHING', (f"In {R['follow_s'] / 60.0:.1f} min of driving behind a car, nothing the radar sends follows the distance to that car:"
                     " the ACC_04 field stays empty and no other number on its bus tracks it. On this car the radar does not share"
                     " its target while openpilot drives, so the radar lead cannot work. With /data/radar_lead set nothing would"
                     " change: the lead stays the camera's.")


def state_txt(pairs, n):
  return ', '.join(f"{ACC_STATUS.get(s, s)} {pc(k, n)}" for s, k in pairs[:4]) if n else 'no ACC_06 from the radar'


def field_txt(hit, names, unit):
  score, addr, start, size, slope, offset, kinds, signed = hit
  return (f"{names.get(addr, '?'):12s} 0x{addr:03X} bits {start}-{start + size - 1}{' signed' if signed else ''}: follows it {score:.2f}"
          f"  ({slope:+.2f} per {unit} {offset:+.1f}, {kinds} different values)")


def report(R, names, branch, file_now, msgs):
  say(f"mk7_radar_check v{TOOL_VERSION.split()[0]}   drives: {', '.join(names)}   (recorded on branch {branch})")
  say(f"{R['min']:.1f} min of logs: car moving {R['moving_min']:.1f} min, openpilot driving {R['op_min']:.1f} min,"
      f" driving behind a car {R['follow_s'] / 60.0:.1f} min.   /data/radar_lead right now: "
      + ('1 (on)' if file_now == '1' else 'no file (off)' if file_now == 'none' else f'{file_now} (off)'))
  say()
  say("1. THE DISTANCE FIELD THE RADAR LEAD READS (ACC_04)")
  if R['n04'] == 0:
    say(f"   no ACC_04 from the radar on bus {RADAR_BUS} ({R['bus2']} other messages there)")
  else:
    say(f"   ACC_04 from the radar (bus {RADAR_BUS}): about {R['hz']:.0f} per second")
    say(f"   it shows a distance {R['dist']} of the time, 'no car ahead' {R['free']}, 'no display' {R['nodisp']}, zero {R['zero']}")
    say(f"   the whole message: {R['a04_kinds']} different contents, most common {R['a04_top']} ({R['a04_top_share']}),"
        f" bits that ever change {R['a04_changed']}")
    say("   by speed, with the camera sure of a car ahead:")
    for label, sel, hit in R['by_speed']:
      say(f"     {label:12s} " + (f"radar has it {pc(hit, sel):>4s} of {sel * DT_MDL:5.0f} s" if sel >= 40 else 'not enough driving'))
    if R['both_s'] >= 10.0:
      say(f"   in metres? radar = {R['slope']:.2f} x camera {R['offset']:+.1f} m over {R['both_s']:.0f} s (metres: close to 1.00 x),"
          f" correlation {R['corr']:.3f}; {100 * R['band']:.0f}% within the trust band (80% needed)")
  say()
  say("2. THE CLUSTER'S LEAD DISPLAY FROM THE RADAR (ACC_02)")
  say(f"   lead bar filled in {R['bar']} of the time ({R['bar_follow']} while driving behind a car), lead symbol on {R['sym']} ({R['sym_follow']})")
  if R['idx'] is not None:
    say(f"   the bar against the camera: {R['idx'][0]:.2f} per metre {R['idx'][1]:+.1f}, correlation {R['idx'][2]:.3f}")
  say()
  say("3. THE RADAR'S OWN CRUISE CONTROL (ACC_06 from the radar)")
  say(f"   while openpilot drives: {state_txt(R['acc_op'], R['acc_op_n'])}")
  say(f"   the rest of the time:   {state_txt(R['acc_off'], R['acc_off_n'])}" + (f"      (ACC type {R['typ']})" if R['typ'] >= 0 else ''))
  if R['acc_own'] == R['acc_own']:
    say(f"   while 'active' it works out its own acceleration {100 * R['acc_own']:.0f}% of the time")
  say()
  say(f"4. SEARCH: DOES ANYTHING ON THE RADAR'S BUS FOLLOW THE CAR AHEAD?   ({R['bus2']} messages, every run of 8-16 bits)")
  if R['search'] is None:
    say(f"   needs 2 minutes of driving behind a car, these drives have {R['follow_s']:.0f} s")
  else:
    say(f"   scores run 0 to 1; a real distance or speed reading scores {STRONG:.1f} or more, chance gives up to about 0.5")
    say("   the distance to the car ahead (camera):")
    for hit in R['search']['d'][:5]:
      say('     ' + field_txt(hit, msgs, 'm'))
    say("   the closing speed (camera):")
    for hit in R['search']['v'][:3]:
      say('     ' + field_txt(hit, msgs, 'm/s'))
  if R['first_trust'] == R['first_trust']:
    say()
    say("5. WHAT THE RADAR LEAD WOULD HAVE DONE ON THESE DRIVES   (as if /data/radar_lead were 1)")
    say(f"   radar trusted {R['first_trust']:.0f} s into the first drive, and {R['trusted']} of all the driving")
    say(f"   lead distance from the radar {R['fused']} of the {R['cam_s'] / 60.0:.1f} min the camera had a lead")
    for lo, hi, secs, med, p90 in R['moved']:
      say(f"   camera says {lo:3d}-{hi:3d} m ({secs:4.0f} s): radar puts the car {abs(med):.1f} m {'closer' if med < 0 else 'farther'}"
          f" (median), {p90:.1f} m or more 10% of the time")
    if R['jit'] is not None:
      say(f"   lead speed beyond 60 m ({R['jit'][2]:.0f} s): the camera's jumps by {R['jit'][0]:.2f} m/s2 frame to frame, with the radar {R['jit'][1]:.2f}")
    if R['logged'] not in ('n/a', '0%'):
      say(f"   in these drives the radar lead was on: the planner's lead distance came from the radar {R['logged']} of the time")
  code, text = verdict(R)
  say()
  say('VERDICT: ' + text)
  say()
  say('PASTE THIS BLOCK BACK:')
  moved = ' '.join(f"{lo}-{hi}:{med:+.1f}/{p90:.1f}" for lo, hi, _, med, p90 in R['moved']) or 'none'
  speed = ' '.join(f"{pc(hit, sel)}/{sel * DT_MDL:.0f}s" for _, sel, hit in R['by_speed'])
  jit = f"{R['jit'][0]:.2f}>{R['jit'][1]:.2f}" if R['jit'] else 'n/a'
  idx = f"{R['idx'][0]:.2f}/{R['idx'][1]:+.1f}/{R['idx'][2]:.2f}" if R['idx'] else 'n/a'

  def st(pairs, k):
    return ','.join(f"{s}:{pc(c, k)}" for s, c in pairs[:4]) or 'none'

  def hits(key, k):
    return ' '.join(f"{a:03X}/{b}+{s}{'s' if sg else ''}={sc:.2f}({sl:+.2f},{of:+.0f},{kn})"
                    for sc, a, b, s, sl, of, kn, sg in R['search'][key][:k]) if R['search'] else 'n/a'

  say(f"mk7_radar_check v{TOOL_VERSION.split()[0]} drives={','.join(names)} branch={branch} file={file_now} verdict={code}")
  say(f"min={R['min']:.1f} moving={R['moving_min']:.1f} op={R['op_min']:.1f} follow={R['follow_s']:.0f}s bus2_msgs={R['bus2']} acc04_hz={R['hz']:.0f}"
      f" dist={R['dist']} free={R['free']} nodisp={R['nodisp']} zero={R['zero']}")
  say(f"acc04 kinds={R['a04_kinds']} top={R['a04_top']}({R['a04_top_share']}) changed={R['a04_changed']}"
      f" by_speed[stop,<11,11-34,34-56,>56]={speed}")
  say(f"fit={R['slope']:.2f}x{R['offset']:+.1f} corr={R['corr']:.3f} med={R['med']:+.1f} band={100 * R['band']:.0f}% both={R['both_s']:.0f}s"
      f" bar={R['bar']}/{R['bar_follow']} sym={R['sym']}/{R['sym_follow']} idx={idx}")
  say(f"radar_acc op={st(R['acc_op'], R['acc_op_n'])} off={st(R['acc_off'], R['acc_off_n'])} typ={R['typ']}"
      f" trusted={R['trusted']} first={R['first_trust']:.0f}s fused={R['fused']} moved[m]={moved} jit60={jit} logged={R['logged']}")
  say(f"search_d={hits('d', 5)}")
  say(f"search_v={hits('v', 3)}")
  return code


def main():
  global np
  p = argparse.ArgumentParser(description='Read-only: can the factory ACC radar give openpilot the distance to the car ahead?')
  p.add_argument('drives', nargs='*', default=['auto'], help='nothing or auto = the newest drives with real openpilot driving (default),'
                 ' 0 = newest, 1 = the one before, 0-4 = a range, or a route-name prefix; several are pooled')
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
  np, zstd, log = setup(args.op_dir)

  routes, segs = routes_on_disk(args.log_root)
  if not routes:
    raise UsageError(f'No drives in {args.log_root}.')
  if args.drives == ['auto']:
    say(f'looking for the newest drives with real openpilot driving (up to {AUTO_MIN:.0f} min in all) ...')
    picked = pick_auto(zstd, log, args.log_root, routes, segs)
    if not picked:
      raise UsageError('None of the drives on the device has a minute of openpilot driving. Drive with openpilot on behind'
                       ' traffic, then run this again.')
  else:
    picked = [(r, sorted(segs[r])) for r in pick_routes(args.drives, routes)]
  replay = Replay()
  for route, seg_nums in picked:
    files = [f for f in (seg_file(args.log_root, route, n) for n in seg_nums) if f]
    say(f"reading drive {rid(route)}: {len(files)} of {len(seg_nums)} minutes have a full log (rlog)")
    for f in files:
      replay.read(events_of(zstd, log, f))
  if not replay.rows:
    raise UsageError('These drives have no full logs (rlog) with camera frames left on the device. Try a newer drive.')
  if replay.op_long is False:
    say('NOTE: these drives were on stock ACC, not openpilot longitudinal. The radar behaves differently then.')
  try:
    with open(RADAR_LEAD_FILE) as f:
      file_now = f.read().strip().split()[0][:8]
  except (OSError, IndexError):
    file_now = 'none'
  say()
  report(analyse(replay), [rid(r) for r, _ in picked], replay.branch, file_now, msg_names(args.op_dir))


if __name__ == '__main__':
  try:
    main()
  except UsageError as e:
    say(str(e))
    sys.exit(1)
