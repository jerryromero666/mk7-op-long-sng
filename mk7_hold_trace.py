#!/usr/bin/env python3
"""
mk7_hold_trace.py - READ-ONLY look at what openpilot sent to the ESP/TSK (brake and cruise computers) and what
they answered, around every stop of a drive, with the checks that separate the hill-hold hypotheses H1-H9.
Built for the Mk7 GTI with openpilot longitudinal control (ACC type 1, DSG, manual handbrake, no electronic
parking brake).

INSTALL OR UPDATE (on the comma, over SSH; nothing is replaced if the download fails):
    cd /data/openpilot && git fetch sngrepo mk7_tools && git show sngrepo/mk7_tools:mk7_hold_trace.py > /data/mk7_hold_trace.py.new && mv /data/mk7_hold_trace.py.new /data/mk7_hold_trace.py

RUN IT WITH THE CAR OFF (ignition off; the comma stays on for a while and you can still SSH in):
    python3 /data/mk7_hold_trace.py                 newest drive
    python3 /data/mk7_hold_trace.py --list          recent drives
    python3 /data/mk7_hold_trace.py --route 2       2nd newest drive (or a name from --list)
    python3 /data/mk7_hold_trace.py --event 3       show stop #3 in detail
    python3 /data/mk7_hold_trace.py --all-timelines detail for every stop and fault

The last block of the output, between the lines "PASTE THIS TO CLAUDE (start)" and "(end)", is meant to be
copied and pasted back to Claude as it is.

It only READS drive logs. It never changes, creates or deletes anything on the comma.
The only file it can write is the optional --csv file that you name yourself.
"""
import sys
sys.dont_write_bytecode = True   # never leave .pyc files behind on the device

import argparse  # noqa: E402
import bz2       # noqa: E402
import csv       # noqa: E402
import gc        # noqa: E402
import math      # noqa: E402
import os        # noqa: E402
import re        # noqa: E402
import time      # noqa: E402
import warnings  # noqa: E402

TOOL_VERSION = '2.0 (2026-10-14)'
EXIT_OK, EXIT_NO_DATA, EXIT_USAGE = 0, 1, 2

DEFAULT_OP_DIR = '/data/openpilot'
DEFAULT_LOG_ROOT = '/data/media/0/realdata'      # system/hardware/hw.py Paths.log_root() on the device
VENV_PYTHON = os.environ.get('MK7_HOLD_VENV_PYTHON', '/usr/local/venv/bin/python3')
REEXEC_FLAG = 'MK7_HOLD_TRACE_REEXEC'
PARAMS_DIR = os.environ.get('MK7_HOLD_PARAMS_DIR', '/data/params/d')   # read only: IsOnroad
DBC_REL = ('opendbc_repo/opendbc/dbc/vw_mqb.dbc', 'opendbc/dbc/vw_mqb.dbc')
ZSTD_MAGIC = b'\x28\xb5\x2f\xfd'
NS = 1_000_000_000
INSTALL_CMD = ('cd /data/openpilot && git fetch sngrepo mk7_tools && git show sngrepo/mk7_tools:mk7_hold_trace.py '
               '> /data/mk7_hold_trace.py.new && mv /data/mk7_hold_trace.py.new /data/mk7_hold_trace.py')

# ---- stop / roll rules (see --help) ----
STILL_V = 0.10           # m/s: "stopped" means vEgo below this ...
STOP_MIN_S = 0.5         # ... for at least this long
BLIP_S = 0.2             # a move shorter than this inside a stop is ignored (sensor blip)
LONG_BEFORE_S = 3.0      # the stop counts as an OP stop if OP long control was on within 3 s before ...
LONG_AFTER_S = 0.5       # ... or 0.5 s after the car stopped
ROLL_V = 0.30            # m/s: unrequested movement faster than this = the car rolled (flat ground, direction unknown)
REQ_LOOKBACK_S = 1.5     # a request to go counts if it was there in the 1.5 s before the wheels moved ...
REQ_SUSTAIN_S = 0.2      # ... for at least 0.2 s without a break ...
LCS_TOGGLE_MAX = 3       # ... and OP's long state changed at most 3 times in that window (more = chatter, not a request)
GAS_LOOKBACK_S = 1.0
RESTOP_MERGE_S = 8.0     # after a roll, a new stop within 8 s belongs to the same event
SLIP_MAX_M = 0.30        # backward movement shorter than this, followed by driving off, is a launch "slip" ...
SLIP_GRADE_MAX_M = 0.08  # ... but on a grade of 3 % or more only up to this (more = ROLLED BACK on the launch)
WHEEL_RUN_S = 0.25       # all four ESP_19 wheel speeds above 0 this long while "stopped" = the car moved
PULSE_MISMATCH_M = 0.05  # speed says the car moved this far but the pulse counters (almost) did not -> use speed
GRADE_UNKNOWN_DIR = 3.0  # with no valid direction signal, any movement on a grade of 3 % or more = possible rollback
PULSE_MIN = 2            # wheel pulses (median of 4 wheels) that count as real movement while "stopped"
PULSE_GAP_S = 0.5        # pulse changes closer than this belong to one movement
FLIP_GAP_S = 0.3         # changes closer together than this are reported as one "flipping" burst
FULL_RATE_HZ = 10.0      # CAN thinner than this around a moment (qlog) is not enough to say what happened
CAN_MAX_STALE_S = 1.0    # a CAN value is shown for at most 1 s after its last frame
SEND_GAP_NS = 100_000_000   # OP sends ACC_06/07 every 20 ms; a pause longer than 100 ms is reported
FLIP_MIN = 4
FAULT_WIN_S = 2.0        # TSK fault classification looks at the 2 s before the fault
H1_EDGE_S = 0.10         # an OP change at most 5 frames (0.1 s) before the hold dropped = OP-triggered release
H2_STEADY_S = 0.20       # OP unchanged for at least 10 frames (0.2 s) before the drop = ESP limit
H5_ZERO_RAW = 2          # TSK_Radbremsmom raw at or below this = no brake torque requested
H5_WITHIN_S = 0.5
# physics for the brake-torque check (study 04 / RJ's state machine). Mass comes from 0x65F when valid.
WHEEL_R_M = 0.328
DEFAULT_MASS_KG = 1566.0
GRAV = 9.81

# qlog keeps 1 in N messages (cereal/services.py). Replaced by the real table when it can be imported.
QLOG_DECIM = {'can': 2053, 'sendcan': 139, 'carState': 10, 'carControl': 10, 'longitudinalPlan': 10,
              'radarState': 5, 'selfdriveState': 10, 'controlsState': 10, 'pandaStates': 1}

# The positions you verified by hand against vw_mqb.dbc. The tool reads every position from the DBC itself
# and only uses this list to cross-check it (a mismatch is printed as a warning).
USER_VERIFIED = [
  # message, address, signal, start bit, size, factor, offset
  ('ESP_33', 0x1AB, 'ESC_TSK_SRBM_Anf', 28, 1, 1.0, 0.0),
  ('ESP_33', 0x1AB, 'ESC_TSK_SRBM_nicht_verfuegbar', 29, 1, 1.0, 0.0),
  ('ESP_21', 0x0FD, 'ESP_Haltebestaetigung', 53, 1, 1.0, 0.0),
  ('ACC_07', 0x12E, 'ACC_Anforderung_HMS', 28, 3, 1.0, 0.0),
  ('TSK_06', 0x120, 'TSK_Status', 24, 3, 1.0, 0.0),
  ('Motor_16', 0x65F, 'TSK_Steigung', 56, 8, 0.8, -101.6),
]

# key, message, signal, required?   (car -> comma, read from 'can' on the powertrain bus)
CAR_SIGNALS = [
  ('hold', 'ESP_21', 'ESP_Haltebestaetigung', True),            # bit 53
  ('esp_acc_stop', 'ESP_21', 'ESP_Anhaltevorgang_ACC_aktiv', False),   # bit 63
  ('esp_eingriff', 'ESP_21', 'ESP_Eingriff', False),
  ('esp_v_kph', 'ESP_21', 'ESP_v_Signal', False),
  ('anf', 'ESP_33', 'ESC_TSK_SRBM_Anf', True),                  # bit 28
  ('nv', 'ESP_33', 'ESC_TSK_SRBM_nicht_verfuegbar', True),      # bit 29
  ('esc_verz_reg', 'ESP_33', 'ESC_Verz_Reg_aktiv', False),
  ('kons_acc', 'ESP_33', 'ESC_Konsistenz_ACC', False),          # bit 38
  ('kons_tsk33', 'ESP_33', 'ESC_Konsistenz_TSK', False),        # bit 45
  ('tsk', 'TSK_06', 'TSK_Status', True),
  ('tsk_brake_nm', 'TSK_06', 'TSK_Radbremsmom', False),         # bits 12-23, x8 Nm
  ('tsk_freig', 'TSK_06', 'TSK_Freig_Verzoeg_Anf', False),      # bit 30
  ('tsk_standby', 'TSK_06', 'TSK_Standby_Anf_ESP', False),      # bit 28
  ('tsk_srbm_anf', 'TSK_06', 'TSK_SRBM_Anf_ASIL', False),       # bits 44-46
  ('grade_raw', 'Motor_16', 'TSK_Steigung', True),
  ('grade_qbit', 'Motor_16', 'TSK_QBit_Steigung', False),
  ('mass', 'Motor_16', 'TSK_Fahrzeugmasse_02', False),
  ('mass_qbit', 'Motor_16', 'TSK_QBit_Fahrzeugmasse', False),
  ('mo_torque', 'Motor_11', 'MO_Mom_Ist_Summe', False),
  ('mo_brake', 'Motor_14', 'MO_Fahrer_bremst', False),
  ('pedal', 'Motor_20', 'MO_Fahrpedalrohwert_01', False),
  ('gear_code', 'Getriebe_11', 'GE_Fahrstufe', False),
  ('esp_brake', 'ESP_05', 'ESP_Fahrer_bremst', False),          # bit 26
  ('brake_bar', 'ESP_05', 'ESP_Bremsdruck', False),             # bits 16-25
  ('verz_tsk', 'ESP_05', 'ESP_Verz_TSK_aktiv', False),          # bit 27
  ('kons_tsk05', 'ESP_05', 'ESP_Konsistenz_TSK', False),        # bit 29
  ('esp_reverse', 'ESP_05', 'ESP_Rueckwaertsfahrt_erkannt', False),   # bit 39
  ('anfahrhilfe', 'ESP_05', 'ESP_Status_Anfahrhilfe', False),   # bit 40
  ('ws_fl', 'ESP_19', 'ESP_VL_Radgeschw_02', False),
  ('ws_fr', 'ESP_19', 'ESP_VR_Radgeschw_02', False),
  ('ws_rl', 'ESP_19', 'ESP_HL_Radgeschw_02', False),
  ('ws_rr', 'ESP_19', 'ESP_HR_Radgeschw_02', False),
  ('dir_fl', 'ESP_10', 'ESP_VL_Fahrtrichtung', False),
  ('dir_fr', 'ESP_10', 'ESP_VR_Fahrtrichtung', False),
  ('dir_rl', 'ESP_10', 'ESP_HL_Fahrtrichtung', False),
  ('dir_rr', 'ESP_10', 'ESP_HR_Fahrtrichtung', False),
  ('pul_fl', 'ESP_10', 'ESP_Wegimpuls_VL', False),              # 10-bit counters, bits 16/26/36/46
  ('pul_fr', 'ESP_10', 'ESP_Wegimpuls_VR', False),
  ('pul_rl', 'ESP_10', 'ESP_Wegimpuls_HL', False),
  ('pul_rr', 'ESP_10', 'ESP_Wegimpuls_HR', False),
  ('pq_fl', 'ESP_10', 'ESP_QBit_Wegimpuls_VL', False),
  ('pq_fr', 'ESP_10', 'ESP_QBit_Wegimpuls_VR', False),
  ('pq_rl', 'ESP_10', 'ESP_QBit_Wegimpuls_HL', False),
  ('pq_rr', 'ESP_10', 'ESP_QBit_Wegimpuls_HR', False),
]
WHEELS = ('fl', 'fr', 'rl', 'rr')
# comma -> car, read from 'sendcan' (what openpilot asked the panda to send). The same signals are also decoded
# separately for every 'can' source that carries ACC_06/ACC_07 (radar src 2, OP's echo src 128, ...), never mixed.
OP_SIGNALS = [
  ('acc06_accel', 'ACC_06', 'ACC_Sollbeschleunigung_02', True),
  ('acc06_stop', 'ACC_06', 'ACC_Anhalten', True),               # bit 57
  ('acc06_go', 'ACC_06', 'ACC_Anfahren', True),                 # bit 56
  ('acc06_ssi', 'ACC_06', 'ACC_StartStopp_Info', False),
  ('acc06_status', 'ACC_06', 'ACC_Status_ACC', False),          # bits 60-62
  ('acc06_jerk_neg', 'ACC_06', 'ACC_neg_Sollbeschl_Grad_02', False),
  ('acc06_jerk_pos', 'ACC_06', 'ACC_pos_Sollbeschl_Grad_02', False),
  ('acc06_typ', 'ACC_06', 'ACC_Typ', False),
  ('hms', 'ACC_07', 'ACC_Anforderung_HMS', True),               # bits 28-30
  ('acc07_dist', 'ACC_07', 'ACC_Anhalteweg', False),            # bits 12-22
  ('acc07_accel', 'ACC_07', 'ACC_Sollbeschleunigung_02', False),   # bits 53-63
  ('acc07_stop', 'ACC_07', 'ACC_Anhalten', False),              # bit 23
  ('acc07_go', 'ACC_07', 'ACC_Anfahren', False),                # bit 31
]
NEUTRAL_SIGNALS = ('ACC_Sollbeschleunigung_02', 'ACC_Anhalteweg')   # raw 2046 "Neutralwert" / 2047 "Fehler" -> no value

HMS_TEXT = {0: 'none', 1: 'HOLD request', 2: 'park', 3: 'standby', 4: 'release/launch', 5: 'ramp release'}
LCS_NAMES = ['off', 'pid', 'stopping', 'starting']
LCS_SHORT = {0: 'off', 1: 'pid', 2: 'stop', 3: 'strt'}
TSK_TEXT = {0: 'init', 1: 'disabled', 2: 'enabled', 3: 'regulating', 4: 'gas override', 5: 'brake only',
            6: 'TEMPORARY FAULT', 7: 'PERMANENT FAULT'}
GEARS = ['unknown', 'park', 'drive', 'neutral', 'reverse', 'sport', 'low', 'brake', 'eco', 'manumatic']

EPILOG = f"""
RUN IT WITH THE CAR OFF
  Switch the ignition off first (the comma stays on for a while, so you can still SSH in). The script reads
  big log files and can use a few hundred MB of memory; with the car off it cannot get in openpilot's way.
  If the car is on, it stops and says so (add --car-on to run anyway at low priority).

INSTALL OR UPDATE
  {INSTALL_CMD}
  The new copy is first saved as /data/mk7_hold_trace.py.new and only moved into place if the download
  worked, so a failed download never leaves you with an empty or broken script.

WHAT COUNTS AS A STOP
  The car's speed (vEgo) stays under 0.1 m/s for at least 0.5 s, and openpilot's longitudinal control
  was on at that moment or within the 3 s before. Times are measured from the first moment all four wheel
  speeds (ESP_19) read 0 ("+0.00").

WHEN DID THE CAR START TO MOVE ("onset")
  The first change of the ESP_10 wheel-pulse counters after the stop (these see a few cm of movement long
  before the speed shows it). If the counters are missing or never change while driving, the first wheel
  speed above 0 is used instead. The header says which one this drive has. Either way, all four ESP_19
  wheel speeds above 0 for 0.25 s or more while the speed still reads "stopped" is a (slow) movement.

WHAT COUNTS AS ASKED-FOR MOVEMENT
  The driver pressed the gas in the 1 s before, or openpilot's planner wanted to GO: shouldStop off and a
  positive acceleration target for at least 0.2 s in the 1.5 s before, with OP's long state not flipping
  back and forth. A flipping long state (pid/stopping chatter) or an "Anfahren" bit alone is NOT a request.

WHAT COUNTS AS A ROLL
  - Wheels turning backward (ESP_10 direction, only used if this drive proves it works, i.e. "backward"
    shows up while moving) or the ESP's own reverse flag (ESP_05) = ROLLED BACK, even if a launch was
    asked for. A short backward slip followed by driving off is reported as a launch slip: under 30 cm on
    flat ground, but on a grade of 3 % or more only under 8 cm (more = "ROLLED BACK ... on OP launch").
  - No valid direction signal and a grade of 3 % or more: any movement is reported as
    "moved, direction unknown, possible rollback" - never as held or drove off.
  - Movement after openpilot was switched off (driver brake or gas, or a fault) is labelled
    "after OP was switched off" and counted separately.

OTHER OPTIONS EXPLAINED
  --list            recent drives (newest first, by the drive counter in the name) with their segment count
  --route NAME      a drive by name from --list, or by its number in that list (1 = newest)
  --segments A-B    only read segments A to B of the drive (e.g. 3-5, 4, 6-)
  --event N         show stop number N in detail (numbers are in the summary table)
  --fault N         show cruise/TSK fault number N in detail
  --all-timelines   detail for every stop and every fault
  --before S        timeline seconds before OP's "stopping" state began (default 3)
  --after S         timeline seconds after the car started to move (default 5; 8 after the stop if it never moved)
  --hz              timeline rows per second: 10 (default), 25 or 50
  --csv FILE        also save every 50-per-second row of the drive to FILE (a NEW file; never inside the
                    log or openpilot folders). Open it in a spreadsheet.

Exit codes: 0 = report printed, 1 = no usable data or nothing to report, 2 = problem with the command or setup.
This script only reads logs and never changes anything on the device.
"""


class UsageError(Exception):
  pass


class NoData(Exception):
  pass


# ============================================================================================ small helpers
def say(msg=''):
  print(msg, flush=True)


def isnum(x):
  return x is not None and not (isinstance(x, float) and math.isnan(x))


def fnum(x, fmt='{:.2f}', w=0, missing='-'):
  s = fmt.format(x) if isnum(x) else missing
  return s.rjust(w) if w else s


def fbool(x, on='1', off='.'):
  if not isnum(x):
    return '-'
  return on if x >= 0.5 else off


def b01(x):
  """1 / 0 / ? for key=value text"""
  if not isnum(x):
    return '?'
  return '1' if x >= 0.5 else '0'


def ival(x, missing='?'):
  return str(int(round(x))) if isnum(x) else missing


def rel(sec):
  return f'{sec:+.2f} s'


def relc(sec):
  """compact relative time for key=value text"""
  return f'{sec:+.2f}' if isnum(sec) else '?'


def clock(sec):
  """seconds into the drive -> m:ss (or h:mm:ss)"""
  if not isnum(sec):
    return '?'
  sec = int(round(max(sec, 0)))
  h, r = divmod(sec, 3600)
  m, s = divmod(r, 60)
  return f'{h}:{m:02d}:{s:02d}' if h else f'{m}:{s:02d}'


def plural(n, word, many=None):
  return f'{n} {word if n == 1 else (many or word + "s")}'


def lcs_name(x):
  return LCS_NAMES[int(x)] if isnum(x) and 0 <= int(x) < len(LCS_NAMES) else '?'


# ============================================================================================ log discovery
SEG_DIR_RE = re.compile(r'^(?P<route>.+)--(?P<num>\d+)$')
ROUTE_COUNTER_RE = re.compile(r'^(?P<cnt>[0-9a-f]{8})--')     # system/loggerd: '%08x--%s' from the RouteCount param
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
    self.recording = os.path.exists(os.path.join(path, 'rlog.lock')) or os.path.exists(os.path.join(path, 'qlog.lock'))

  @property
  def label(self):
    return f'{self.route}--{self.num}'


class Route:
  def __init__(self, name):
    self.name = name
    self.segments = []
    m = ROUTE_COUNTER_RE.match(name)
    self.counter = int(m.group('cnt'), 16) if m else None

  def finish(self):
    self.segments.sort(key=lambda s: s.num)
    self.mtime = max(s.mtime for s in self.segments)
    self.n_rlog = sum(1 for s in self.segments if 'rlog' in s.files)
    self.n_qlog_only = sum(1 for s in self.segments if 'rlog' not in s.files and 'qlog' in s.files)
    self.n_none = sum(1 for s in self.segments if not s.files)


def route_sort_key(r):
  """Newest first by the hex drive counter in the name; names without one (old software) by file time, after them."""
  return (1, r.counter, r.mtime) if r.counter is not None else (0, r.mtime, 0)


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
  out.sort(key=route_sort_key, reverse=True)
  return out


def order_note(routes):
  """A plain note when the drive counter and the file dates disagree about which drive is newest."""
  if len(routes) < 2:
    return ''
  by_time = max(routes, key=lambda r: r.mtime)
  if by_time is not routes[0] and routes[0].counter is not None:
    return (f'Note: "newest" = highest drive counter ({routes[0].name}); the files of {by_time.name} have a newer '
            'date (the clock may have been wrong, or the counter was reset). Use --route to pick another drive.')
  return ''


def describe_route(r):
  n = len(r.segments)
  first = r.segments[0]
  start = time.strftime('%Y-%m-%d %H:%M', time.localtime(first.mtime - 60))
  logs = f'rlog {r.n_rlog}/{n}'
  if r.n_qlog_only:
    logs += f', qlog only {r.n_qlog_only}'
  if r.n_none:
    logs += f', no log {r.n_none}'
  if any(s.recording for s in r.segments):
    logs += ', still recording'
  return n, start, logs


def print_route_list(routes, count):
  say('Recent drives, newest first (by the drive counter in the name). Start times are approximate '
      '(from file dates; a segment is ~1 minute).')
  say(f"{'#':>3}  {'drive name':<28} {'segs':>4}  {'started about':<16}  logs")
  for i, r in enumerate(routes[:count], 1):
    n, start, logs = describe_route(r)
    say(f'{i:>3}  {r.name:<28} {n:>4}  {start:<16}  {logs}')
  if len(routes) > count:
    say(f'... and {len(routes) - count} older drives.')
  note = order_note(routes)
  if note:
    say(note)
  say()
  say('Pick one with:  --route NAME   or   --route NUMBER   (for example --route 2 = second newest)')


def resolve_route(routes, text):
  t = os.path.basename(text.strip().split('|')[-1].rstrip('/'))
  m = SEG_DIR_RE.match(t)
  names = {r.name: r for r in routes}
  if t in names:
    return names[t]
  if m and m.group('route') in names:
    return names[m.group('route')]
  # a short number like 1, 2, 12 is a position in --list (route names start with 8 hex digits)
  if t.isdigit() and len(t) <= 3 and not t.startswith('0'):
    i = int(t)
    if 1 <= i <= len(routes):
      return routes[i - 1]
    raise UsageError(f'There is no drive number {i}; there are {len(routes)} drives. Run with --list to see them.')
  cands = [r for r in routes if r.name.startswith(t)]
  if len(cands) == 1:
    return cands[0]
  if len(cands) > 1:
    raise UsageError(f"'{text}' matches {len(cands)} drives ({', '.join(c.name for c in cands[:4])} ...). "
                     "Give more of the name.")
  raise UsageError(f"No drive called '{text}' was found. Run with --list to see the drive names.")


def parse_segments(text):
  m = re.match(r'^\s*(\d*)\s*(-?)\s*(\d*)\s*$', text or '')
  if not m or (not m.group(1) and not m.group(3)):
    raise UsageError(f"--segments '{text}' is not understood. Use a range like 3-5, a single segment like 4, "
                     "or an open range like 6- (6 to the end).")
  a = int(m.group(1)) if m.group(1) else 0
  if m.group(2):
    b = int(m.group(3)) if m.group(3) else 10**9
  else:
    b = a
  if b < a:
    raise UsageError(f'--segments {text}: the first number must not be bigger than the second.')
  return a, b


# ============================================================================================ runtime
class Runtime:
  np = None
  LogReader = None
  log = None
  zstd = None
  reader_name = ''
  odbc_DBC = None
  odbc_get_raw = None
  odbc_err = None


def maybe_reexec():
  """If capnp / openpilot's cereal can't be imported, retry once with openpilot's own Python."""
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
  # parent first so op_dir ends up first: 'cereal' and 'opendbc' live in op_dir, 'openpilot.*' via the parent
  for p in (os.path.dirname(op_dir), op_dir):
    if os.path.isdir(p) and p not in sys.path:
      sys.path.insert(0, p)
  try:
    import numpy as np
    import capnp  # noqa: F401
    import zstandard
    from cereal import log
  except Exception as e:     # missing module, wrong python, ...
    maybe_reexec()
    raise UsageError("This needs openpilot's Python libraries (numpy, pycapnp, zstandard and the cereal folder).\n"
                     f"  Could not load them: {e}\n"
                     f"  Run it on the comma device (openpilot in {DEFAULT_OP_DIR}), or point --op-dir at an "
                     "openpilot checkout.") from None
  rt.np, rt.zstd, rt.log = np, zstandard, log
  rt.reader_name = "streaming reader with openpilot's cereal schema (one segment in memory at a time)"
  try:
    from openpilot.tools.lib.logreader import LogReader      # only a fallback for files the streaming reader can't read
    rt.LogReader = LogReader
    try:   # make sure nothing is sent to the device's log daemon
      from openpilot.common import swaglog
      swaglog.cloudlog.removeHandler(swaglog.ipchandler)
    except Exception:
      pass
  except Exception:
    rt.LogReader = None
  try:
    from opendbc.can.dbc import DBC
    from opendbc.can.parser import get_raw_value
    rt.odbc_DBC, rt.odbc_get_raw = DBC, get_raw_value
  except Exception as e:
    rt.odbc_err = e
  try:
    from cereal.services import SERVICE_LIST
    for k in list(QLOG_DECIM):
      if k in SERVICE_LIST and SERVICE_LIST[k].decimation:
        QLOG_DECIM[k] = SERVICE_LIST[k].decimation
  except Exception:
    pass
  return rt


# ============================================================================================ DBC
class SigDef:
  __slots__ = ('name', 'start', 'size', 'lsb', 'msb', 'le', 'signed', 'factor', 'offset')

  def __init__(self, name, start, size, lsb, msb, le, signed, factor, offset):
    self.name, self.start, self.size, self.lsb, self.msb = name, start, size, lsb, msb
    self.le, self.signed, self.factor, self.offset = le, signed, factor, offset


class MsgDef:
  def __init__(self, name, addr, size):
    self.name, self.addr, self.size, self.sigs = name, addr, size, {}


BO_RE = re.compile(r'^BO_ (\w+) (\w+) *: (\w+) (\w+)')
SG_RE = re.compile(r'^SG_ (\w+)(?: \w+)? *: (\d+)\|(\d+)@(\d)([+-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\)')
VAL_RE = re.compile(r'^VAL_ (\w+) (\w+) (.*);')
VAL_PAIR_RE = re.compile(r'(\d+) "([^"]*)"')
BE_BITS = [j + i * 8 for i in range(64) for j in range(7, -1, -1)]


def _text_parse_dbc(path):
  """Plain-text DBC reader using the same rules as opendbc/can/dbc.py (used only if opendbc can't be imported)."""
  msgs, cur = {}, None
  with open(path, encoding='utf-8', errors='replace') as f:
    for line in f:
      line = line.strip()
      if line.startswith('BO_ '):
        m = BO_RE.match(line)
        if m:
          cur = MsgDef(m.group(2), int(m.group(1), 0), int(m.group(3), 0))
          msgs[cur.name] = cur
      elif line.startswith('SG_ ') and cur is not None:
        m = SG_RE.match(line)
        if not m:
          continue
        start, size, le = int(m.group(2)), int(m.group(3)), m.group(4) == '1'
        if le:
          lsb, msb = start, start + size - 1
        else:
          idx = BE_BITS.index(start)
          lsb, msb = BE_BITS[idx + size - 1], start
        cur.sigs[m.group(1)] = SigDef(m.group(1), start, size, lsb, msb, le, m.group(5) == '-',
                                      float(m.group(6)), float(m.group(7)))
  return msgs


def _parse_vals(path):
  vals = {}
  try:
    with open(path, encoding='utf-8', errors='replace') as f:
      for line in f:
        line = line.strip()
        if line.startswith('VAL_ '):
          m = VAL_RE.match(line)
          if m:
            vals[(int(m.group(1), 0), m.group(2))] = {int(a): b for a, b in VAL_PAIR_RE.findall(m.group(3))}
  except OSError:
    pass
  return vals


class Dbc:
  def __init__(self, rt, path):
    self.path = path
    self.how = ''
    self.msgs = None
    if rt.odbc_DBC is not None:
      try:
        d = rt.odbc_DBC(path)
        msgs = {}
        for addr, m in d.msgs.items():
          md = MsgDef(m.name, addr, m.size)
          for sn, s in m.sigs.items():
            md.sigs[sn] = SigDef(sn, s.start_bit, s.size, s.lsb, s.msb, s.is_little_endian, s.is_signed,
                                 float(s.factor), float(s.offset))
          msgs[m.name] = md
        self.msgs = msgs
        self.how = "openpilot's own DBC parser (opendbc/can/dbc.py)"
      except Exception as e:
        rt.odbc_err = e
    if self.msgs is None:
      self.msgs = _text_parse_dbc(path)
      self.how = f'built-in DBC text reader (opendbc not importable: {rt.odbc_err})'
    self.vals = _parse_vals(path)

  def sig(self, msg, name):
    m = self.msgs.get(msg)
    if m is None:
      return None, None
    return m, m.sigs.get(name)

  def val_name(self, msg, name, raw):
    m = self.msgs.get(msg)
    if m is None or not isnum(raw):
      return None
    return self.vals.get((m.addr, name), {}).get(int(raw))


def find_dbc(op_dir, override):
  if override:
    if not os.path.isfile(override):
      raise UsageError(f'--dbc {override}: file not found.')
    return override
  for r in DBC_REL:
    p = os.path.join(op_dir, r)
    if os.path.isfile(p):
      return p
  raise UsageError(f"Could not find vw_mqb.dbc under {op_dir} (looked in {' and '.join(DBC_REL)}).\n"
                   "  The decoding must come from the real DBC, so the script stops here. Use --op-dir to point at "
                   "your openpilot folder, or --dbc to give the file directly.")


def cross_check(dbc):
  """Compare the DBC with the user's hand-verified list. Returns (ok_lines, problems)."""
  problems = []
  n_ok = 0
  for msg, addr, sname, start, size, factor, offset in USER_VERIFIED:
    m, s = dbc.sig(msg, sname)
    if m is None:
      problems.append(f'{msg} is not in the DBC')
      continue
    if m.addr != addr:
      problems.append(f'{msg} has address 0x{m.addr:X} in the DBC, your list says 0x{addr:X}')
    if s is None:
      problems.append(f'{msg}.{sname} is not in the DBC')
      continue
    if (s.start, s.size) != (start, size) or not s.le or abs(s.factor - factor) > 1e-9 or abs(s.offset - offset) > 1e-9:
      problems.append(f'{msg}.{sname}: DBC says bit {s.start}, {s.size} bit(s), x{s.factor:g}{s.offset:+g}'
                      f'{"" if s.le else " big-endian"}; your list says bit {start}, {size} bit(s), x{factor:g}{offset:+g}')
    else:
      n_ok += 1
  return n_ok, problems


# ============================================================================================ reading logs
def _decompress(rt, path):
  """Whole file -> decompressed bytes. Tolerant: a cut-off or damaged file gives the readable part."""
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
  del raw
  return dat, damaged


class EventStream:
  """Iterates the events of one log file one at a time (the decoded messages are never all kept in memory)."""

  def __init__(self, rt, path):
    self.rt = rt
    self.dat, self.damaged = _decompress(rt, path)
    self.n = 0

  def __iter__(self):
    try:
      for e in self.rt.log.Event.read_multiple_bytes(self.dat):
        self.n += 1
        yield e
    except Exception:
      self.damaged = True


class SegData:
  def __init__(self):
    self.kind = None
    self.damaged = False
    self.how = ''
    self.problems = []
    self.t_first = None
    self.t_last = None
    self.can = {}          # (stream, addr, src) -> (t int64 array, value uint64 array)
    self.svc = {}          # name -> (t int64 array, numpy 2-D float array)
    self.sd = []           # selfdriveState changes: (t, state, alertType, text1, text2)
    self.oe = []           # onroadEvents changes: (t, tuple of names)
    self.init = None
    self.counts = {}


CS_COLS = ['t', 'vEgo', 'vEgoRaw', 'aEgo', 'standstill', 'brakePressed', 'gasPressed', 'cruiseEnabled',
           'cruiseAvailable', 'gear', 'parkingBrake', 'accFaulted', 'espActive']
CC_COLS = ['t', 'ccEnabled', 'longActive', 'ccAccel', 'lcs', 'pitch']
CTL_COLS = ['t', 'lcs_ctrl']
LP_COLS = ['t', 'shouldStop', 'aTarget', 'hasLead', 'allowThrottle', 'vPlan0']
RS_COLS = ['t', 'leadStatus', 'dRel', 'vLead']
PS_COLS = ['t', 'controlsAllowed', 'controlsAllowedLong', 'safetyTxBlocked', 'safetyRxInvalid']
NAN = float('nan')


def _opt(fn):
  try:
    return fn()
  except Exception:
    return NAN


def extract(rt, events, want):
  """Pull only what we need out of one segment. Everything else is skipped after a cheap which()."""
  np = rt.np
  sd = SegData()
  canbuf = {}
  cs, cc, ctl, lp, rs, ps = [], [], [], [], [], []
  last_sd = last_oe = None
  tmin, tmax = None, None
  n = {'can': 0, 'sendcan': 0, 'carState': 0, 'carControl': 0, 'controlsState': 0, 'pandaStates': 0,
       'longitudinalPlan': 0, 'bad': 0, 'long_frames': 0}
  from_bytes = int.from_bytes
  gear_idx = {g: i for i, g in enumerate(GEARS)}
  lcs_idx = {g: i for i, g in enumerate(LCS_NAMES)}
  for ev in events:
    try:
      w = ev.which()
      t = ev.logMonoTime
      if w == 'can' or w == 'sendcan':
        n[w] += 1
        for f in (ev.can if w == 'can' else ev.sendcan):
          a = f.address
          if a in want:
            d = f.dat
            if len(d) > 8:
              n['long_frames'] += 1
              continue
            key = (w, a, f.src)
            b = canbuf.get(key)
            if b is None:
              b = canbuf[key] = ([], [])
            b[0].append(t)
            b[1].append(from_bytes(d, 'little'))
      elif w == 'carState':
        n['carState'] += 1
        c = ev.carState
        cr = c.cruiseState
        cs.append((t, c.vEgo, c.vEgoRaw, c.aEgo, c.standstill, c.brakePressed, c.gasPressed, cr.enabled,
                   cr.available, gear_idx.get(str(c.gearShifter), 0), c.parkingBrake, c.accFaulted, c.espActive))
      elif w == 'carControl':
        n['carControl'] += 1
        c = ev.carControl
        a = c.actuators
        pitch = _opt(lambda c=c: float(c.orientationNED[1]) if len(c.orientationNED) > 1 else NAN)
        cc.append((t, c.enabled, c.longActive, a.accel, lcs_idx.get(str(a.longControlState), 0), pitch))
      elif w == 'controlsState':
        n['controlsState'] += 1
        ctl.append((t, _opt(lambda ev=ev: lcs_idx.get(str(ev.controlsState.longControlState), NAN))))
      elif w == 'longitudinalPlan':
        n['longitudinalPlan'] += 1
        p = ev.longitudinalPlan
        v0 = _opt(lambda p=p: float(p.speeds[0]) if len(p.speeds) else NAN)
        lp.append((t, p.shouldStop, p.aTarget, p.hasLead, p.allowThrottle, v0))
      elif w == 'radarState':
        ld = ev.radarState.leadOne
        rs.append((t, ld.status, ld.dRel, ld.vLead))
      elif w == 'pandaStates':
        n['pandaStates'] += 1
        pl = ev.pandaStates
        if len(pl):
          p0 = pl[0]
          ps.append((t, _opt(lambda p0=p0: p0.controlsAllowed), _opt(lambda p0=p0: p0.controlsAllowedLongitudinal),
                     _opt(lambda p0=p0: p0.safetyTxBlocked), _opt(lambda p0=p0: p0.safetyRxInvalid)))
      elif w == 'selfdriveState':
        s = ev.selfdriveState
        row = (str(s.state), s.alertType, s.alertText1, s.alertText2)
        if row != last_sd:
          sd.sd.append((t,) + row)
          last_sd = row
      elif w == 'onroadEvents':
        names = []
        for e in ev.onroadEvents:
          nm = str(e.name)
          if e.immediateDisable or e.softDisable:
            nm += '(disable)'
          elif e.noEntry and not e.warning:
            nm += '(no-entry)'
          names.append(nm)
        names = tuple(sorted(names))
        if names != last_oe:
          sd.oe.append((t, names))
          last_oe = names
      elif w == 'initData' and sd.init is None:
        i = ev.initData
        sd.init = (i.gitBranch, i.gitCommit, i.dirty, i.version)
      else:
        if tmin is None or t < tmin:
          tmin = t
        continue
    except Exception:
      n['bad'] += 1
      continue
    if tmin is None or t < tmin:
      tmin = t
    if tmax is None or t > tmax:
      tmax = t
  sd.t_first, sd.t_last = tmin, tmax
  for key, (ts, vs) in canbuf.items():
    sd.can[key] = (np.array(ts, dtype=np.int64), np.array(vs, dtype=np.uint64))
  for name, rows in (('cs', cs), ('cc', cc), ('ctl', ctl), ('lp', lp), ('rs', rs), ('ps', ps)):
    if rows:
      arr = np.array(rows, dtype=np.float64)
      arr_t = np.array([r[0] for r in rows], dtype=np.int64)   # keep exact ns times
      sd.svc[name] = (arr_t, arr[:, 1:])
  sd.counts = n
  return sd


def read_segment(rt, seg, want):
  res = None
  problems = []
  for kind, _ in LOG_NAMES:
    path = seg.files.get(kind)
    if not path:
      continue
    try:
      stream = EventStream(rt, path)
      res = extract(rt, stream, want)
      res.damaged, res.how = stream.damaged, 'streaming reader'
      n_ev = stream.n
      del stream
      if n_ev == 0 and rt.LogReader is not None:      # last resort: openpilot's own LogReader
        with warnings.catch_warnings(record=True) as caught:
          warnings.simplefilter('always')
          events = list(rt.LogReader(path))
        if events:
          res = extract(rt, events, want)
          res.damaged, res.how = any('orrupt' in str(w.message) for w in caught), 'LogReader'
          n_ev = len(events)
        del events
      if n_ev == 0:
        raise ValueError('no readable messages')
    except Exception:
      problems.append(f'{os.path.basename(path)} is damaged and unreadable')
      res = None
      continue
    res.kind = kind
    gc.collect()
    break
  if res is None:
    res = SegData()
    if not seg.files:
      problems.append('no rlog/qlog file in this segment')
  res.problems = problems
  return res


# ============================================================================================ signal series
class Sig:
  """A time-sorted signal: t = int64 ns, v = float64. Sampling holds the last value (until it goes stale)."""

  def __init__(self, np, t, v, stale_s=None, max_stale=5.0):
    self.np = np
    t = np.asarray(t, dtype=np.int64)
    v = np.asarray(v, dtype=np.float64)
    if len(t) > 1 and np.any(np.diff(t) < 0):
      o = np.argsort(t, kind='stable')
      t, v = t[o], v[o]
    self.t, self.v = t, v
    if stale_s is None:
      med = 1.0
      if len(t) > 3:
        dt = np.diff(t)
        dt = dt[dt > 0]
        if len(dt):
          med = float(np.median(dt)) / NS
      stale_s = min(max(3.0 * med, 0.3), max_stale)
    self.stale = int(stale_s * NS)

  def __len__(self):
    return len(self.t)

  def rate(self, a, b):
    """frames per second inside [a, b]"""
    if b <= a:
      return 0.0
    t, _ = self.sl(a, b)
    return len(t) / ((b - a) / NS)

  def at(self, T):
    np = self.np
    T = np.asarray(T, dtype=np.int64)
    if len(self.t) == 0:
      return np.full(len(T), np.nan)
    idx = np.searchsorted(self.t, T, side='right') - 1
    idc = np.clip(idx, 0, len(self.t) - 1)
    out = self.v[idc].copy()
    out[(idx < 0) | ((T - self.t[idc]) > self.stale)] = np.nan
    return out

  def at1(self, t):
    if len(self.t) == 0 or t is None:
      return float('nan')
    i = int(self.np.searchsorted(self.t, t, side='right')) - 1
    if i < 0 or t - self.t[i] > self.stale:
      return float('nan')
    return float(self.v[i])

  def sl(self, a, b):
    np = self.np
    i0 = int(np.searchsorted(self.t, a, side='left'))
    i1 = int(np.searchsorted(self.t, b, side='right'))
    return self.t[i0:i1], self.v[i0:i1]

  def any_on(self, a, b, thr=0.5):
    _, v = self.sl(a, b)
    return bool(len(v)) and bool(self.np.any(v >= thr))

  def first(self, a, b, cond):
    if a is None:
      return None
    t, v = self.sl(a, b)
    if not len(v):
      return None
    m = cond(v)
    if not self.np.any(m):
      return None
    return int(t[int(self.np.argmax(m))])

  def last(self, a, b, cond):
    t, v = self.sl(a, b)
    if not len(v):
      return None
    m = cond(v)
    if not self.np.any(m):
      return None
    return int(t[len(m) - 1 - int(self.np.argmax(m[::-1]))])

  def median(self, a, b):
    _, v = self.sl(a, b)
    v = v[~self.np.isnan(v)] if len(v) else v
    return float(self.np.median(v)) if len(v) else float('nan')

  def changes(self, a, b):
    """[(t, old, new)] for every value change inside [a, b] (the value just before a is the starting point)."""
    np = self.np
    i0 = int(np.searchsorted(self.t, a, side='left'))
    i1 = int(np.searchsorted(self.t, b, side='right'))
    j0 = max(i0 - 1, 0)
    t, v = self.t[j0:i1], self.v[j0:i1]
    if len(v) < 2:
      return []
    vf = np.where(np.isnan(v), -9999.0, v)
    idx = np.nonzero(vf[1:] != vf[:-1])[0] + 1
    out = []
    for i in idx:
      if t[i] >= a:
        out.append((int(t[i]), float(v[i - 1]), float(v[i])))
    return out


class Drive:
  pass


def decode_raw(np, words, sdef):
  """Raw integer value of one signal from 8-byte little-endian words (vectorised)."""
  if sdef.le:
    raw = (words >> np.uint64(sdef.lsb)) & np.uint64((1 << sdef.size) - 1)
    raw = raw.astype(np.int64)
  else:
    raw = np.array([be_raw(int(w).to_bytes(8, 'little'), sdef) for w in words], dtype=np.int64)
  if sdef.signed:
    raw = np.where(raw >= (1 << (sdef.size - 1)), raw - (1 << sdef.size), raw)
  return raw


def be_raw(dat, s):
  """Generic raw value (same walk as opendbc/can/parser.py get_raw_value), used for big-endian signals."""
  ret, i, bits = 0, s.msb // 8, s.size
  while 0 <= i < len(dat) and bits > 0:
    lsb = s.lsb if (s.lsb // 8) == i else i * 8
    msb = s.msb if (s.msb // 8) == i else (i + 1) * 8 - 1
    size = msb - lsb + 1
    d = (dat[i] >> (lsb - (i * 8))) & ((1 << size) - 1)
    ret |= d << (bits - size)
    bits -= size
    i = i - 1 if s.le else i + 1
  return ret


def decode_value(np, dbc, msg, sname, words):
  """(raw int array, physical float array) with the DBC's neutral/error codes turned into NaN."""
  m, sdef = dbc.sig(msg, sname)
  rawv = decode_raw(np, words, sdef)
  val = rawv.astype(np.float64) * sdef.factor + sdef.offset
  if sname in NEUTRAL_SIGNALS:
    vt = dbc.vals.get((m.addr, sname), {})
    if sname == 'ACC_Anhalteweg':      # 20.46 m ("Neutralwert") is what OP sends when not stopping: keep it visible
      bad = [r for r, lab in vt.items() if 'fehler' in lab.lower()] or [2047]
    else:
      bad = [r for r, lab in vt.items() if 'neutral' in lab.lower() or 'fehler' in lab.lower()] or [2046, 2047]
    val[np.isin(rawv, bad)] = np.nan
  return rawv, val


def src_label(src, pt_bus=0):
  if src >= 192:
    return f'src {src} (OP frames the panda REFUSED, bus {src - 192})'
  if src >= 128:
    return f"src {src} (OP's own frames as sent, panda echo bus {src - 128})"
  if src == pt_bus:
    return f'src {src} (received on the car bus: ANOTHER sender)'
  return f'src {src} (received on bus {src}: the stock radar\'s own frames)'


def src_short(src):
  if src == 'sendcan':
    return 'sendcan'
  if src == 'mixed':
    return 'all-can-mixed'
  return f'src{src}'


def unwrap_counter(np, raw, modulus):
  """10-bit wheel pulse counter -> running count of steps in EITHER direction (handles wrap-around both ways).
  Some ESPs count the pulses down while the wheel turns backward: a step down is movement too, not a glitch."""
  if not len(raw):
    return np.zeros(0)
  r = np.asarray(raw, dtype=np.int64)
  d = np.mod(np.diff(r), modulus)
  d = np.minimum(d, modulus - d)
  return np.concatenate([[0], np.cumsum(d)]).astype(np.float64)


def build_drive(rt, dbc, segs_data, segs):
  """Turn the per-segment pieces into whole-drive signals, pick the CAN buses and decode."""
  np = rt.np
  D = Drive()
  D.np = np
  D.dbc = dbc
  D.notes = []
  # each segment starts its change lists afresh: drop the repeats at segment boundaries
  D.sd, D.oe = [], []
  for x in sorted((x for s in segs_data for x in s.sd), key=lambda r: r[0]):
    if not D.sd or D.sd[-1][1:] != x[1:]:
      D.sd.append(x)
  for x in sorted((x for s in segs_data for x in s.oe), key=lambda r: r[0]):
    if not D.oe or D.oe[-1][1] != x[1]:
      D.oe.append(x)
  D.init = next((s.init for s in segs_data if s.init), None)
  D.svc_counts = {}
  for s in segs_data:
    for k, v in s.counts.items():
      D.svc_counts[k] = D.svc_counts.get(k, 0) + v

  # ---- services
  def cat_svc(name, cols):
    ts = [s.svc[name][0] for s in segs_data if name in s.svc]
    vs = [s.svc[name][1] for s in segs_data if name in s.svc]
    out = {}
    if not ts:
      for c in cols[1:]:
        out[c] = Sig(np, [], [])
      return out
    t = np.concatenate(ts)
    v = np.concatenate(vs, axis=0)
    o = np.argsort(t, kind='stable')
    t, v = t[o], v[o]
    for i, c in enumerate(cols[1:]):
      out[c] = Sig(np, t, v[:, i])
    return out

  D.s = {}
  for name, cols in (('cs', CS_COLS), ('cc', CC_COLS), ('ctl', CTL_COLS), ('lp', LP_COLS), ('rs', RS_COLS),
                     ('ps', PS_COLS)):
    D.s.update(cat_svc(name, cols))
  lead_ok = D.s['leadStatus']
  for k in ('dRel', 'vLead'):            # a distance only means something while radard has a lead
    s = D.s[k]
    st = lead_ok.at(s.t)
    s.v = np.where(st >= 0.5, s.v, np.nan)
  D.has_plan = len(D.s['shouldStop']) > 0
  D.has_panda = len(D.s['controlsAllowed']) > 0
  D.has_ctl = len(D.s['lcs_ctrl']) > 0
  D.has_pitch = len(D.s['pitch']) > 0 and bool(np.any(~np.isnan(D.s['pitch'].v)))
  D.has_vplan = len(D.s['vPlan0']) > 0 and bool(np.any(~np.isnan(D.s['vPlan0'].v)))

  # ---- raw CAN, all segments together
  raw = {}
  for s in segs_data:
    for key, (t, v) in s.can.items():
      raw.setdefault(key, []).append((t, v))
  rawcat = {}
  for key, parts in raw.items():
    t = np.concatenate([p[0] for p in parts])
    v = np.concatenate([p[1] for p in parts])
    o = np.argsort(t, kind='stable')
    rawcat[key] = (t[o], v[o])
  del raw

  addr_of = {name: m.addr for name, m in dbc.msgs.items()}
  D.addr_of = addr_of
  esp21 = addr_of.get('ESP_21')
  # powertrain bus = the lowest bus number that carries ESP_21 at a real rate (openpilot reads it from CAN.pt)
  counts = {src: len(v[0]) for (st, a, src), v in rawcat.items() if st == 'can' and a == esp21 and src < 128}
  D.pt_bus = None
  D.esp21_counts = counts
  if counts:
    mx = max(counts.values())
    D.pt_bus = min(b for b, c in counts.items() if c >= 0.25 * mx)
  acc07 = addr_of.get('ACC_07')
  acc06 = addr_of.get('ACC_06')
  scounts = {src: len(v[0]) for (st, a, src), v in rawcat.items() if st == 'sendcan' and a == acc07}
  D.op_bus = max(scounts, key=lambda b: scounts[b]) if scounts else None

  def pick(stream, msg):
    a = addr_of.get(msg)
    if a is None:
      return None, None
    if stream == 'can':
      cands = {src: v for (st, aa, src), v in rawcat.items() if st == 'can' and aa == a and src < 128}
      pref = D.pt_bus
    else:
      cands = {src: v for (st, aa, src), v in rawcat.items() if st == 'sendcan' and aa == a}
      pref = D.op_bus
    if not cands:
      return None, None
    if pref in cands:
      return pref, cands[pref]
    if stream == 'can' and msg in ('ACC_06', 'ACC_07'):
      return None, None
    b = max(cands, key=lambda k: len(cands[k][0]))
    return b, cands[b]

  D.sig_src = {}
  D.missing = []
  D.c = {}
  decode_checks = []
  for stream, table in (('can', CAR_SIGNALS), ('sendcan', OP_SIGNALS)):
    for key, msg, sname, _required in table:
      m, sdef = dbc.sig(msg, sname)
      if sdef is None:
        D.missing.append(f'{msg}.{sname} (not in the DBC)')
        D.c[key] = Sig(np, [], [])
        continue
      bus, data = pick(stream, msg)
      if data is None:
        D.c[key] = Sig(np, [], [])
        continue
      t, words = data
      rawv, val = decode_value(np, dbc, msg, sname, words)
      if key == 'grade_raw':
        D.c['grade_rawcode'] = Sig(np, t, rawv.astype(np.float64))
      if key == 'mass':
        D.c['mass_rawcode'] = Sig(np, t, rawv.astype(np.float64))
      D.c[key] = Sig(np, t, val, max_stale=CAN_MAX_STALE_S)
      D.sig_src[key] = (stream, bus, msg)
      if len(decode_checks) < 400:
        decode_checks.append((words[:12], sdef))
  D.decode_checks = decode_checks
  D.c['tsk_brake_raw'] = Sig(np, D.c['tsk_brake_nm'].t, D.c['tsk_brake_nm'].v / 8.0, max_stale=CAN_MAX_STALE_S)

  # ---- ACC_06 / ACC_07 from every 'can' source, decoded separately (never mixed)
  D.acc_src = {}
  D.acc_src_counts = {}
  mixed_t, mixed_v, mixed_s = [], [], []
  for (st, a, src), (t, words) in sorted(rawcat.items(), key=lambda kv: kv[0][2]):
    if st != 'can' or a not in (acc06, acc07):
      continue
    msg = 'ACC_06' if a == acc06 else 'ACC_07'
    d = D.acc_src.setdefault(src, {})
    D.acc_src_counts.setdefault(src, {})[msg] = len(t)
    for key, mm, sname, _ in OP_SIGNALS:
      if mm != msg or dbc.sig(mm, sname)[1] is None:
        continue
      _, val = decode_value(np, dbc, mm, sname, words)
      d[key] = Sig(np, t, val, max_stale=CAN_MAX_STALE_S)
      if key == 'hms':
        mixed_t.append(t)
        mixed_v.append(val)
        mixed_s.append(np.full(len(t), src))
  for d in D.acc_src.values():
    for key, _, _, _ in OP_SIGNALS:
      d.setdefault(key, Sig(np, [], []))
  if mixed_t:
    t = np.concatenate(mixed_t)
    o = np.argsort(t, kind='stable')
    D.hms_mixed = (t[o], np.concatenate(mixed_v)[o], np.concatenate(mixed_s)[o])
  else:
    D.hms_mixed = (np.zeros(0, np.int64), np.zeros(0), np.zeros(0))
  D.echo_src = (D.op_bus + 128) if D.op_bus is not None else 128
  D.radar_srcs = sorted(s for s in D.acc_src if s < 128 and s != D.pt_bus)

  # grade: invalid when raw 0 (-101.6 %), raw 255 (error) or the quality bit says so
  g = D.c['grade_raw']
  if len(g):
    rawcode = D.c['grade_rawcode'].v
    v = g.v.copy()
    bad = (rawcode == 0) | (rawcode == 255)
    q = D.c.get('grade_qbit')
    if q is not None and len(q):
      bad |= q.at(g.t) >= 0.5
    v[bad] = np.nan
    dt = np.diff(g.t)
    med = float(np.median(dt[dt > 0])) / NS if np.any(dt > 0) else 1.0
    D.grade = Sig(np, g.t, v, stale_s=min(max(3.0 * med, 3.0), 5.0))   # grade comes slowly; hold it up to 5 s
    D.grade_invalid_frac = float(np.mean(bad)) if len(bad) else 0.0
  else:
    D.grade = Sig(np, [], [])
    D.grade_invalid_frac = 1.0
  ms = D.c['mass']
  if len(ms):
    rc = D.c['mass_rawcode'].v
    v = ms.v.copy()
    bad = (rc == 0) | (rc == 255)
    q = D.c.get('mass_qbit')
    if q is not None and len(q):
      bad |= q.at(ms.t) >= 0.5
    v[bad] = np.nan
    D.mass = Sig(np, ms.t, v, stale_s=5.0)
  else:
    D.mass = Sig(np, [], [])

  # standstill anchor: all four ESP_19 wheel speeds = 0
  ws = [D.c[f'ws_{w}'] for w in WHEELS]
  if all(len(x) for x in ws):
    t19 = ws[0].t
    wv = np.vstack([x.at(t19) for x in ws])
    allz = np.all(wv == 0, axis=0)
    D.wz = Sig(np, t19, allz.astype(np.float64), max_stale=CAN_MAX_STALE_S)
    # all four wheels above 0 (NaN = no frame -> not "above 0"): a slow roll the 0.1 m/s speed threshold misses
    D.wnz = Sig(np, t19, np.all(np.nan_to_num(wv, nan=0.0) > 0, axis=0).astype(np.float64), max_stale=CAN_MAX_STALE_S)
    D.anchor_how = 'all four ESP_19 wheel speeds = 0'
  elif len(D.s['vEgoRaw']):
    vr = D.s['vEgoRaw']
    D.wz = Sig(np, vr.t, (vr.v == 0).astype(np.float64))
    D.anchor_how = 'carState vEgoRaw = 0 (ESP_19 not in the log)'
    D.wnz = Sig(np, [], [])
  else:
    D.wnz = Sig(np, [], [])
    D.wz = Sig(np, [], [])
    D.anchor_how = 'speed under 0.1 m/s (no wheel speeds in the log)'

  build_pulses(np, D)
  D.dir = build_direction(np, dbc, D)
  rv = D.c['esp_reverse']
  D.rev_seen = bool(len(rv)) and bool(np.any(rv.v >= 0.5))

  # panda echo of OP's own frames: src = bus+128 sent on the wire, bus+192 refused by panda safety
  D.echo = {}
  for msg in ('ACC_06', 'ACC_07'):
    a = addr_of.get(msg)
    ok, rej = [], []
    for (st, aa, src), (t, _) in rawcat.items():
      if st == 'can' and aa == a and src >= 128:
        (rej if src >= 192 else ok).append(t)
    D.echo[msg] = (np.sort(np.concatenate(ok)) if ok else np.zeros(0, np.int64),
                   np.sort(np.concatenate(rej)) if rej else np.zeros(0, np.int64))
  D.has_echo = any(len(v[0]) or len(v[1]) for v in D.echo.values())
  # ACC_06/ACC_07 RECEIVED on the car bus = a second sender (the stock radar leaking through, for example)
  D.foreign = {}
  for msg in ('ACC_06', 'ACC_07'):
    data = rawcat.get(('can', addr_of.get(msg), D.pt_bus)) if D.pt_bus is not None else None
    D.foreign[msg] = data[0] if data is not None else np.zeros(0, np.int64)
  D.has_tsk = len(D.c['tsk']) > 0
  D.has_can = len(D.c['hold']) > 0 or len(D.c['tsk']) > 0 or len(D.c['anf']) > 0
  D.has_sendcan = len(D.c['hms']) > 0
  cst = D.s['vEgo'].t
  dur = (cst[-1] - cst[0]) / NS if len(cst) > 1 else 0.0
  esp_ref = D.c['hold'] if len(D.c['hold']) else (D.c['anf'] if len(D.c['anf']) else D.c['tsk'])
  D.can_full = dur > 0 and len(esp_ref) / dur >= FULL_RATE_HZ
  D.tsk_full = dur > 0 and len(D.c['tsk']) / dur >= FULL_RATE_HZ
  D.op_full = dur > 0 and len(D.c['hms']) / dur >= FULL_RATE_HZ
  # gaps in what OP sent: ACC_07 normally goes out every 20 ms
  D.send_gaps = []
  if D.op_full:
    ht = D.c['hms'].t
    dts = np.diff(ht)
    for i in np.nonzero((dts > SEND_GAP_NS) & (dts < 5 * NS))[0]:   # longer = a missing log segment, not OP
      D.send_gaps.append((int(ht[i]), int(dts[i])))

  # time base: "time into the drive"
  firsts = [(sg.num, s.t_first) for sg, s in zip(segs, segs_data) if s.t_first is not None]
  if firsts:
    n0, t0 = firsts[0]
    D.t_route0 = t0 - int(n0 * 60 * NS)
  else:
    D.t_route0 = 0
  D.seg_bounds = [(sg.num, s.t_first, s.t_last, s.kind) for sg, s in zip(segs, segs_data) if s.t_first is not None]
  return D


def build_pulses(np, D):
  """ESP_10 wheel pulse counters -> running counts per wheel + a validity check for this drive."""
  D.pulse_ok = False
  D.m_per_pulse = None
  D.pulse_mismatch = []            # (a, b, metres from pulses, metres from speed) where the pulses missed movement
  D.pul = Sig(np, [], [])          # median of the four wheels' running counts
  D.pul_any = Sig(np, [], [])      # running count of frames in which ANY wheel's counter changed
  keys = [f'pul_{w}' for w in WHEELS]
  if not all(len(D.c[k]) for k in keys):
    D.pulse_info = 'ESP_10 wheel pulse counters not in the log'
    return
  t = D.c[keys[0]].t
  raws = [D.c[k].at(t) for k in keys]
  if any(np.any(np.isnan(r)) for r in raws):
    raws = [np.nan_to_num(r, nan=0.0) for r in raws]
  maxv = max(float(np.max(r)) for r in raws)
  modulus = 1024 if maxv > 1000 else 1001
  cums = np.vstack([unwrap_counter(np, r, modulus) for r in raws])
  med = np.median(cums, axis=0)
  anych = np.concatenate([[0.0], np.cumsum(np.any(np.diff(cums, axis=1) > 0, axis=0).astype(np.float64))])
  D.pul = Sig(np, t, med, max_stale=CAN_MAX_STALE_S)
  D.pul_any = Sig(np, t, anych, max_stale=CAN_MAX_STALE_S)
  D.pul_cums = (t, cums)
  v = D.s['vEgo'].at(t)
  fast = v > 1.0
  steps = np.diff(anych)
  fast_steps = fast[1:] & fast[:-1]
  n_fast = int(np.sum(fast_steps))
  frac = float(np.mean(steps[fast_steps] > 0)) if n_fast else 0.0
  qb = [D.c[f'pq_{w}'] for w in WHEELS]
  qfrac = None
  if all(len(x) for x in qb):
    qfrac = float(np.mean(np.any(np.vstack([x.at(t) >= 0.5 for x in qb]), axis=0)))
  if n_fast < 50:
    D.pulse_info = 'ESP_10 wheel pulse counters present, but not enough driving above 1 m/s to check them; not used'
    return
  if frac < 0.5:
    D.pulse_info = (f'ESP_10 wheel pulse counters present but they did not change in {100 - frac * 100:.0f}% of '
                    'frames while driving; NOT used (roll onset taken from wheel speed instead)')
    return
  D.pulse_ok = True
  # cm per pulse from this drive: distance (integrated speed) / pulses while driving above 2 m/s
  dtt = np.diff(t) / NS
  vm = 0.5 * (v[1:] + v[:-1])
  sel = (v[1:] > 2.0) & (v[:-1] > 2.0) & (dtt < 0.2) & ~np.isnan(vm)
  pulses = float(np.sum(np.diff(med)[sel]))
  dist = float(np.sum(vm[sel] * dtt[sel]))
  if pulses >= 100:
    D.m_per_pulse = dist / pulses
  D.pulse_info = (f'ESP_10 wheel pulse counters VALID (they change in {frac * 100:.0f}% of frames while driving'
                  + (f'; about {D.m_per_pulse * 100:.1f} cm per pulse, measured on this drive' if D.m_per_pulse else '')
                  + (f'; quality bit set in {qfrac * 100:.0f}% of frames' if qfrac else '') + ')')


def direction_codes(dbc):
  """(forward code, backward code, where the meaning came from)."""
  m, s = dbc.sig('ESP_10', 'ESP_VL_Fahrtrichtung')
  table = dbc.vals.get((m.addr, 'ESP_VL_Fahrtrichtung')) if m else None
  src = 'vw_mqb.dbc'
  if not table and m is not None:
    # vw_mqb.dbc has no value table for it; the MEB DBC next to it has one for the same message/bits
    other = os.path.join(os.path.dirname(dbc.path), 'vw_meb.dbc')
    if os.path.isfile(other):
      om = _text_parse_dbc(other).get('ESP_10')
      osd = om.sigs.get('ESP_VL_Fahrtrichtung') if om else None
      if om and osd and om.addr == m.addr and (osd.start, osd.size) == (s.start, s.size):
        table = _parse_vals(other).get((m.addr, 'ESP_VL_Fahrtrichtung'))
        src = 'vw_meb.dbc (same message and bits; vw_mqb.dbc has no value names for it)'
  fwd = back = None
  if table:
    for raw, lab in table.items():
      ll = lab.lower()
      if 'vorw' in ll:
        fwd = raw
      elif 'rueckw' in ll or 'rückw' in ll:
        back = raw
  if fwd is None or back is None:
    fwd, back = 0, 1
    src = 'ASSUMED 0=forward 1=backward (no value names found)'
  return fwd, back, src


def build_direction(np, dbc, D):
  """Wheel direction from ESP_10: +1 forward, -1 BACKWARD, 0 unknown - but only if this drive shows it works."""
  D.dir_valid = False
  D.dir_codes_seen = {}
  D.dir_raw = Sig(np, [], [])
  keys = ('dir_fl', 'dir_fr', 'dir_rl', 'dir_rr')
  if not all(len(D.c[k]) for k in keys):
    D.dir_info = 'ESP_10 wheel direction NOT in the log: direction unknown'
    D.dir_short = 'missing'
    return Sig(np, [], [])
  fwd, back, src = direction_codes(dbc)
  D.dir_codes = (fwd, back)
  t = D.c['dir_fl'].t
  vals = np.vstack([D.c[k].at(t) for k in keys])
  nf = np.sum(vals == fwd, axis=0)
  nb = np.sum(vals == back, axis=0)
  d = np.where(nb >= 2, -1.0, np.where((nf >= 3) | ((nf >= 2) & (nb == 0)), 1.0, 0.0))
  D.dir_raw = Sig(np, t, d, max_stale=CAN_MAX_STALE_S)
  v = D.s['vEgo'].at(t)
  fast = v > 0.3
  moving = v >= 0.05
  codes, cnt = np.unique(vals[:, fast].astype(np.int64), return_counts=True) if np.any(fast) else ([], [])
  D.dir_codes_seen = {int(c): int(n) for c, n in zip(codes, cnt)}
  n_fast = int(np.sum(fast))
  n_back = int(np.sum((d < 0) & moving))
  n_fwd = int(np.sum((d > 0) & fast))
  bad = sum(n for c, n in D.dir_codes_seen.items() if c not in (fwd, back))
  bad_frac = bad / max(1, sum(D.dir_codes_seen.values()))
  seen = ', '.join(f'{c}{"=fwd" if c == fwd else ("=back" if c == back else "=invalid")} x{n}'
                   for c, n in sorted(D.dir_codes_seen.items())) or 'none'
  why = None
  if n_fast < 50:
    why = 'not enough driving to check it'
  elif bad_frac > 0.2:
    why = f'codes other than forward/backward (init/invalid) are {bad_frac * 100:.0f}% of what the wheels report'
  elif n_back < 3:
    why = '"backward" never appears in this drive (stuck at forward, or this car has no direction sensors)'
  elif n_fwd < 0.2 * n_fast:
    why = '"forward" is rare while driving forward'
  D.dir_seen_txt = f'raw codes on the 4 wheels while faster than 0.3 m/s: {seen}'
  if why:
    D.dir_info = (f'ESP_10 wheel direction NOT VALIDATED: {why} ({D.dir_seen_txt}). Direction is ignored; on grades of '
                  f'{GRADE_UNKNOWN_DIR:.0f}% or more any movement is reported as a possible rollback')
    D.dir_short = 'NOT_VALIDATED'
    return Sig(np, [], [])
  D.dir_valid = True
  D.dir_info = (f'ESP_10 wheel direction VALIDATED on this drive ({fwd}=forward, {back}=backward from {src}; '
                f'backward seen in {n_back} moving frames; {D.dir_seen_txt})')
  D.dir_short = f'VALID(back_frames={n_back})'
  return D.dir_raw


# ============================================================================================ analysis
class Stop:
  pass


def runs_of(np, mask):
  """[(i0, i1)] index runs where mask is True (i1 exclusive)."""
  if not len(mask):
    return []
  m = np.concatenate([[False], mask, [False]])
  d = np.diff(m.astype(np.int8))
  starts = np.nonzero(d == 1)[0]
  ends = np.nonzero(d == -1)[0]
  return list(zip(starts.tolist(), ends.tolist()))


def find_still_runs(D):
  """Standing-still periods from carState vEgo: [(t_start, t_end_or_None, t_last, dur)]; t_end = first moving sample."""
  np = D.np
  sv = D.s['vEgo']
  t, v = sv.t, sv.v
  if not len(t):
    return []
  still = v < STILL_V
  runs = runs_of(np, still)
  # glue together runs separated by a blip shorter than BLIP_S
  merged = []
  for i0, i1 in runs:
    if merged:
      p0, p1 = merged[-1]
      if p1 < len(t) and t[i0] - t[p1] < BLIP_S * NS and t[i0] - t[p1 - 1] < (BLIP_S + 0.5) * NS:
        merged[-1] = (p0, i1)
        continue
    merged.append((i0, i1))
  out = []
  for i0, i1 in merged:
    t_start = int(t[i0])
    t_end = int(t[i1]) if i1 < len(t) else None
    t_last = int(t[i1 - 1])
    dur = (t_end if t_end is not None else t_last) - t_start
    out.append((t_start, t_end, t_last, dur))
  return out


def vsig(D):
  return D.s['vEgoRaw'] if len(D.s['vEgoRaw']) else D.s['vEgo']


def integrate(np, t, v):
  if len(t) < 2:
    return 0.0
  v = np.nan_to_num(v, nan=0.0)
  return float(np.sum(0.5 * (v[1:] + v[:-1]) * (np.diff(t) / NS)))


def distance(D, a, b):
  """metres travelled in [a, b]: wheel pulses when they are valid and calibrated, otherwise integrated speed."""
  np = D.np
  if b is None or a is None or b <= a:
    return 0.0, 'none'
  t, v = vsig(D).sl(a, b)
  ds = max(integrate(np, t, v), 0.0)
  if D.pulse_ok and D.m_per_pulse:
    p0, p1 = D.pul.at1(a), D.pul.at1(b)
    if isnum(p0) and isnum(p1):
      dp = max(p1 - p0, 0.0) * D.m_per_pulse
      if ds >= PULSE_MISMATCH_M and dp < 0.3 * ds:
        # the counters (almost) did not move while the speed clearly did: trust the speed, and say so
        D.pulse_mismatch.append((a, b, dp, ds))
        return ds, 'speed (pulse counters did not count)'
      return dp, 'pulses'
  return ds, 'speed'


def pulses_between(D, a, b):
  if not D.pulse_ok or a is None or b is None:
    return None
  p0, p1 = D.pul.at1(a), D.pul.at1(b)
  return (p1 - p0) if isnum(p0) and isnum(p1) else None


def find_anchor(D, t_still, t_end):
  """First frame where all four wheel speeds read 0, inside this standstill (fallback: speed under 0.1 m/s)."""
  if len(D.wz):
    ta = D.wz.first(t_still - int(0.3 * NS), t_end if t_end is not None else t_still + 5 * NS, lambda v: v >= 0.5)
    if ta is not None:
      return ta, D.anchor_how
  return t_still, 'speed under 0.1 m/s (wheel speeds never all 0 here)'


def pulse_clusters(D, a, b):
  """Movements seen in the pulse counters in [a, b]: [(t_first_change, t_last_change, median_pulses)]."""
  np = D.np
  t, v = D.pul_any.sl(a, b)
  base = D.pul_any.at1(a)
  if not len(t) or not isnum(base):
    return []
  d = np.diff(np.concatenate([[base], v]))
  ch = t[d > 0]
  if not len(ch):
    return []
  out = []
  s = ch[0]
  prev = ch[0]
  for x in ch[1:]:
    if x - prev > int(PULSE_GAP_S * NS):
      out.append((int(s), int(prev)))
      s = x
    prev = x
  out.append((int(s), int(prev)))
  res = []
  for s, e in out:
    p = pulses_between(D, s - int(0.05 * NS), e)
    res.append((s, e, p if p is not None else 0.0))
  return res


def wheel_runs(D, a, b):
  """Stretches in [a, b] with all four ESP_19 wheel speeds above 0 for WHEEL_RUN_S or longer: [(t_start, t_end)].
  t_end = first frame with a wheel at 0 again (or the last frame of the run if it lasts to b)."""
  np = D.np
  if not len(getattr(D, 'wnz', ())) or b is None or b <= a:
    return []
  t, v = D.wnz.sl(a, b)
  out = []
  for i0, i1 in runs_of(np, v >= 0.5):
    te = int(t[i1]) if i1 < len(t) else int(t[i1 - 1])
    if te - int(t[i0]) >= int(WHEEL_RUN_S * NS) and D.wnz.rate(int(t[i0]), te) >= FULL_RATE_HZ:   # not from a thin qlog
      out.append((int(t[i0]), te))
  return out


def find_onset(D, st):
  """When did the wheels start to turn after this stop? -> (time or None, how, earlier small movements)."""
  a = st.t0 + int(0.04 * NS)
  t1 = st.t1
  end = (t1 + int(0.1 * NS)) if t1 is not None else st.t_last_still
  early = []
  st.onset_end = None
  # all four ESP_19 wheel speeds above 0 for a while, then 0 again, all while vEgo stayed under 0.1 m/s: a slow
  # creep/roll. It counts even when the pulse counters are missing, frozen or did not show it.
  # (a run that ends less than PULSE_GAP_S before the car drives off is part of that launch, e.g. a launch slip)
  inner = [w for w in wheel_runs(D, a, t1 if t1 is not None else st.t_last_still)
           if t1 is None or w[1] < t1 - int(PULSE_GAP_S * NS)]
  if D.pulse_ok:      # only the runs the pulse counters did NOT show; the ones they did show are handled below
    inner = [w for w in inner if (pulses_between(D, w[0] - int(0.05 * NS), w[1]) or 0.0) < PULSE_MIN]
  w_how = ('all four ESP_19 wheel speeds above 0 for {:.2f} s (speed stayed under 0.1 m/s'
           + ('; the ESP_10 pulse counters did not show it)' if D.pulse_ok else ')'))
  if D.pulse_ok:
    cl = pulse_clusters(D, a, end)
    final = cl[-1] if (t1 is not None and cl and t1 + int(0.1 * NS) - cl[-1][1] <= int(PULSE_GAP_S * NS)) else None
    big = [c for c in cl if c[2] >= PULSE_MIN]
    if big and (final is None or big[0][0] < final[0]) and not (inner and inner[0][1] < big[0][0]):
      # the wheels turned while the speed still read "stopped" (a slow creep or roll), then stood still again:
      # that movement is the onset, even if the car drove off later
      c0 = big[0]
      early = [c for c in cl if c[0] < c0[0] and c[2] >= 1]
      st.onset_end = c0[1]
      return c0[0], 'first ESP_10 wheel-pulse change (speed stayed under 0.1 m/s)', early
    if inner and (final is None or inner[0][0] < final[0]):
      w0 = inner[0]
      early = [c for c in cl if c[0] < w0[0] and c[2] >= 1]
      st.onset_end = w0[1]
      return w0[0], w_how.format((w0[1] - w0[0]) / NS), early
    if final is not None:
      early = [c for c in cl[:-1] if c[2] >= 1]
      return final[0], 'first ESP_10 wheel-pulse change', early
    if t1 is None:
      return None, '', [c for c in cl if c[2] >= 1]
  elif inner:
    w0 = inner[0]
    st.onset_end = w0[1]
    return w0[0], w_how.format((w0[1] - w0[0]) / NS), early
  if t1 is None:
    return None, '', early
  if len(D.wz):
    tz = D.wz.last(st.t0, t1, lambda v: v >= 0.5)
    if tz is not None:
      tm = D.wz.first(tz + 1, t1 + int(0.1 * NS), lambda v: v < 0.5)
      if tm is not None and tm <= t1 + int(0.1 * NS):
        return tm, 'first wheel speed above 0 (ESP_19)', early
  return t1, 'speed above 0.1 m/s (carState)', early


def planner_go(D, a, b):
  """Did openpilot's planner want to GO in [a, b]? -> (bool, plain-English how / why not)."""
  np = D.np
  # evaluate at every carControl / plan sample (a fixed grid could alias a 100 Hz pid/stopping chatter)
  T = np.unique(np.concatenate([D.s['ccAccel'].sl(a, b)[0], D.s['shouldStop'].sl(a, b)[0],
                                np.arange(a, b + 1, NS // 50, dtype=np.int64)]))
  if not len(T):
    return False, ''
  la = D.s['longActive'].at(T) >= 0.5
  acc = D.s['ccAccel'].at(T)
  lcs = D.s['lcs'].at(T)
  pos = (np.nan_to_num(acc, nan=0.0) > 0.05) | (lcs == 3)
  if D.has_plan:
    ss = D.s['shouldStop'].at(T)
    at = np.nan_to_num(D.s['aTarget'].at(T), nan=0.0)
    go = la & (np.nan_to_num(ss, nan=1.0) < 0.5) & (pos | (at > 0.05))
  else:
    go = la & pos
  best = 0.0
  for i0, i1 in runs_of(np, go):
    end = T[i1] if i1 < len(T) else T[i1 - 1] + NS // 50
    best = max(best, (end - T[i0]) / NS)
  n_lcs = len(D.s['lcs'].changes(a, b))
  maxacc = float(np.nanmax(acc[la])) if np.any(la) and np.any(~np.isnan(acc[la])) else float('nan')
  if best >= REQ_SUSTAIN_S and n_lcs <= LCS_TOGGLE_MAX:
    how = ('planner wanted to go (shouldStop off' if D.has_plan else 'OP wanted to go (no planner data;')
    how += f', OP asked for +{maxacc:.2f} m/s2 for {best:.2f} s)' if isnum(maxacc) and maxacc > 0 else f' for {best:.2f} s)'
    return True, how
  why = []
  if np.any(la & pos):
    if n_lcs > LCS_TOGGLE_MAX:
      why.append(f'OP long state flipped {n_lcs} times (pid/stopping chatter, not a request)')
    if D.has_plan and not np.any(go):
      why.append('the planner still wanted to stop (shouldStop on)')
    elif best < REQ_SUSTAIN_S:
      why.append(f'the go request lasted only {best:.2f} s')
    if isnum(maxacc):
      why.append(f'OP output up to {maxacc:+.2f} m/s2')
  return False, '; '.join(why)


def off_cause(D, t, a=None):
  """Why OP long control went off around t: driver brake, driver gas, cruise fault, or unknown."""
  if t is None:
    return ''
  w0, w1 = (a if a is not None else t - int(0.4 * NS)), t + int(0.3 * NS)
  for key, name in (('brakePressed', 'driver brake'), ('gasPressed', 'driver gas')):
    tb = D.s[key].first(w0, w1, lambda v: v >= 0.5)
    if tb is not None:
      return name
  for key, name in (('esp_brake', 'driver brake'), ('mo_brake', 'driver brake')):
    if len(D.c[key]) and D.c[key].first(w0, w1, lambda v: v >= 0.5) is not None:
      return name
  if len(D.c['tsk']) and D.c['tsk'].first(w0, w1, lambda v: v >= 6) is not None:
    return 'cruise fault (TSK)'
  if D.s['accFaulted'].first(w0, w1, lambda v: v >= 0.5) is not None:
    return 'cruise fault'
  return 'not a pedal; reason not in the log'


def classify_motion(D, st, nxt_real):
  """What happened when the wheels moved after a stop. Returns a dict."""
  np = D.np
  on = st.onset
  r = {'t1': on, 'onset': on, 'speed_t1': st.t1}
  sv = vsig(D)
  creep = getattr(st, 'onset_end', None) is not None
  if creep:                             # pulses moved while the speed read 0, then the car stood still again
    hz_end = st.onset_end + int(0.3 * NS)
  elif st.t1 is None:                   # pulses moved but the speed never showed it
    hz_end = st.t_last_still
  else:
    hz_end = st.t1 + 10 * NS
    if nxt_real is not None:
      hz_end = min(hz_end, nxt_real)
  r['hz_end'] = hz_end
  # ---- direction
  ddir, dsrc = None, None
  t_fwd = None
  dwin = (on - int(0.1 * NS), min(on + 2 * NS, hz_end + int(0.3 * NS)))
  if D.dir_valid and D.dir.rate(*dwin) >= FULL_RATE_HZ:
    dt, dv = D.dir.sl(dwin[0], hz_end + int(0.3 * NS))
    if len(dt):
      vm = sv.at(dt)
      moved = (np.nan_to_num(vm, nan=0.0) >= 0.05)
      if D.pulse_ok:
        moved |= (D.pul_any.at(dt) - D.pul_any.at(dt - int(0.1 * NS))) > 0
      sel = moved & (dv != 0)
      first = dv[sel][:25] if np.any(sel) else dv[dv != 0][:25]
      if len(first):
        nb, nf = int(np.sum(first < 0)), int(np.sum(first > 0))
        ddir = -1 if nb > nf else (1 if nf > nb else None)
        dsrc = 'wheel direction (ESP_10)'
      if ddir == -1:
        tsel = dt[sel & (dv > 0)]
        tsel = tsel[tsel > on]
        t_fwd = int(tsel[0]) if len(tsel) else None
  rwin = (on - int(0.3 * NS), min(on + 2 * NS, hz_end + int(0.3 * NS)))
  rev = D.c['esp_reverse']
  rev_on = rev.rate(*rwin) >= FULL_RATE_HZ and rev.any_on(*rwin)
  r['rev_flag'] = rev_on
  if ddir is None and rev_on:
    ddir, dsrc = -1, 'ESP reverse flag (ESP_05)'
    toff = rev.first(rev.first(*rwin, lambda v: v >= 0.5), hz_end, lambda v: v < 0.5)
    if toff is not None and isnum(sv.at1(toff + int(0.3 * NS))) and sv.at1(toff + int(0.3 * NS)) >= 0.1:
      t_fwd = toff
  r['dir'], r['dir_source'] = ddir, dsrc
  gear = D.s['gear'].at1(on)
  r['reverse_gear'] = isnum(gear) and GEARS[int(gear)] == 'reverse'
  # ---- who asked for movement?
  r['gas'] = D.s['gasPressed'].any_on(on - int(GAS_LOOKBACK_S * NS), on + int(0.1 * NS))
  op_req, how = planner_go(D, on - int(REQ_LOOKBACK_S * NS), on + int(0.1 * NS))
  r['op_req'] = op_req
  r['op_req_how'] = how if op_req else ''
  r['op_not_req_why'] = '' if op_req else how
  sent = []
  a06 = D.c['acc06_accel']
  _, av = a06.sl(on - int(REQ_LOOKBACK_S * NS), on)
  av = av[~np.isnan(av)] if len(av) else av
  if len(av) and float(np.max(av)) > 0.05:
    sent.append(f'ACC_06 accel up to +{float(np.max(av)):.2f}')
  if D.c['acc06_go'].any_on(on - int(REQ_LOOKBACK_S * NS), on):
    sent.append('ACC_06 Anfahren')
  if D.c['acc07_go'].any_on(on - int(REQ_LOOKBACK_S * NS), on):
    sent.append('ACC_07 Anfahren/HMS 4')
  r['op_sent_go'] = ', '.join(sent)
  r['requested'] = r['gas'] or op_req or r['reverse_gear']
  # ---- OP switched off before the wheels moved?
  la = D.s['longActive']
  r['op_off'] = None
  if st.op_at_stop:
    toff = la.first(st.t0, on, lambda v: v < 0.5)
    if toff is not None and not (la.at1(on - int(0.02 * NS)) >= 0.5):
      r['op_off'] = (toff, off_cause(D, toff))
  else:
    toff = la.last(st.t_still - int(LONG_BEFORE_S * NS), st.t0 + int(LONG_AFTER_S * NS), lambda v: v >= 0.5)
    if not (la.at1(on - int(0.02 * NS)) >= 0.5):
      r['op_off'] = (toff if toff is not None else st.t0, off_cause(D, toff, (toff or st.t0) - int(0.5 * NS)) + ', before the car stopped')
  # ---- speed dip = the wheels stopped for a moment (a change of direction)
  dip_t, went_on = None, False
  if st.t1 is not None and not creep:
    vt, vv = sv.sl(st.t1, hz_end)
    if len(vt):
      moving = vv >= STILL_V
      k = np.nonzero(~moving & np.concatenate([[False], moving[:-1]]))[0]
      if len(k):
        dip_t = int(vt[k[0]])
        went_on = bool(np.any(vv[k[0]:] >= ROLL_V))
  r['dip_t'] = dip_t
  # ---- how far / fast
  t_end = hz_end
  if ddir == -1 and t_fwd is not None:
    t_end = min(t_end, t_fwd)
  if not r['requested']:
    tg = D.s['gasPressed'].first(on, hz_end, lambda v: v >= 0.5)
    if tg is not None:
      t_end = min(t_end, tg)
  r['t_end'] = max(t_end, on)
  r['dist'], r['dist_how'] = distance(D, on, r['t_end'])
  r['pulses'] = pulses_between(D, on - int(0.05 * NS), r['t_end'])
  vt, vv = sv.sl(on, r['t_end'])
  r['peak'] = float(np.nanmax(vv)) if len(vv) and np.any(~np.isnan(vv)) else 0.0
  r['t03'] = sv.first(on, hz_end, lambda v: v > ROLL_V)
  r['brake_t'] = D.s['brakePressed'].first(on, r['t_end'] + int(0.5 * NS), lambda v: v >= 0.5)
  r['restopped'] = creep or (nxt_real is not None and st.t1 is not None and nxt_real <= r['t_end'] + int(0.05 * NS))
  r['creep_then'] = None
  if creep and st.t1 is not None:      # the car drove off later, after standing still again
    later_gas = D.s['gasPressed'].any_on(st.t1 - int(GAS_LOOKBACK_S * NS), st.t1 + int(0.1 * NS))
    later_op, _ = planner_go(D, st.t1 - int(REQ_LOOKBACK_S * NS), st.t1 + int(0.1 * NS))
    r['creep_then'] = (st.t1, 'OP launch' if later_op else ('driver gas' if later_gas else 'nothing asked for it'))
  # backward part of a launch
  if t_fwd is not None:
    back_end = min(t_fwd, max(r['t_end'], on))
  else:
    back_end = min(x for x in (dip_t, r['t_end']) if x is not None)
  back_end = max(back_end, on)
  r['back_dist'], _ = distance(D, on, back_end)
  bt, bv = sv.sl(on, back_end)
  r['back_peak'] = float(np.nanmax(bv)) if len(bv) and np.any(~np.isnan(bv)) else 0.0
  went_fwd = False
  if ddir == -1:
    if t_fwd is not None:
      went_fwd = bool(np.any(sv.sl(t_fwd, hz_end)[1] >= ROLL_V)) and not (r['restopped'] and nxt_real < t_fwd)
    elif dsrc and 'reverse flag' in dsrc and dip_t is not None:
      went_fwd = went_on
  r['went_fwd'] = went_fwd
  r['slip'] = None
  grade = st.grade
  # ---- verdict
  if r['reverse_gear']:
    r['kind'] = 'reverse'
  elif ddir == -1:
    on_grade = isnum(grade) and abs(grade) >= GRADE_UNKNOWN_DIR
    slip_max = SLIP_GRADE_MAX_M if on_grade else SLIP_MAX_M
    if r['requested'] and went_fwd and r['back_dist'] < slip_max:
      r['kind'] = 'drive'
      r['slip'] = (on, r['back_dist'], r['back_peak'])
    else:
      r['kind'] = 'back'
      r['launch'] = r['requested']
      # rolled back further than a launch slip on a grade, then drove off: a rollback on the launch, not a slip
      r['launch_roll'] = bool(r['requested'] and went_fwd and on_grade)
      if r['requested']:
        r['dist'], r['peak'] = r['back_dist'], r['back_peak']
        r['t_end'] = back_end
  elif ddir == 1:
    if r['requested']:
      r['kind'] = 'drive'
    else:
      # downhill (3 % or more) with nothing asking for it: a roll at any speed
      downhill = isnum(grade) and grade <= -GRADE_UNKNOWN_DIR
      r['kind'] = 'fwd' if (r['peak'] > ROLL_V or r['dist'] >= 0.10 or downhill) else 'creep'
  else:
    uphill_unasked = isnum(grade) and grade >= 1.0 and not r['requested']
    if isnum(grade) and (abs(grade) >= GRADE_UNKNOWN_DIR or uphill_unasked):
      r['kind'] = 'unk'
      if r['requested'] and dip_t is not None and went_on:
        r['dist'], _ = distance(D, on, dip_t)
        tt, vv2 = sv.sl(on, dip_t)
        r['peak'] = float(np.nanmax(vv2)) if len(vv2) else r['peak']
        r['t_end'] = dip_t
        r['unk_note'] = 'speed fell to 0 once and rose again: probably slipped back, then drove off'
      elif r['requested'] and not r['restopped']:
        # it drove away: only the first second can be a slip; the rest of the drive is not a roll
        t1s = on + NS
        r['dist'], _ = distance(D, on, t1s)
        tt, vv2 = sv.sl(on, t1s)
        r['peak'] = float(np.nanmax(vv2)) if len(vv2) and np.any(~np.isnan(vv2)) else r['peak']
        r['t_end'] = t1s
        r['dist_first_s'] = True
        r['unk_note'] = ('then drove away with no stop-and-reverse in the speed, so a slip back cannot be seen '
                         'without a direction signal; distance = first 1 s')
      elif r['restopped'] and not creep:
        r['unk_note'] = 'stopped again after it'
    elif r['requested']:
      r['kind'] = 'drive'
    else:
      r['kind'] = 'roll?' if r['t03'] is not None else 'creep'
      r['guess'] = None
      if isnum(grade) and grade >= 1.0:
        r['guess'] = 'backward'
      elif isnum(grade) and grade <= -1.0:
        r['guess'] = 'forward'
  return r


MOVE_KINDS = ('back', 'fwd', 'roll?', 'unk', 'creep')


def category(st):
  m = st.motion
  if m is None:
    return 'held'
  k = m['kind']
  if k in MOVE_KINDS and m.get('op_off'):
    return 'op_off_roll'
  return {'back': 'rolled_back', 'unk': 'possible_roll', 'roll?': 'possible_roll', 'fwd': 'rolled_fwd',
          'creep': 'moved', 'reverse': 'held'}.get(k, 'slip' if m.get('slip') else 'held')


def is_unasked_roll(st):
  """Moved after the stop with nothing asking for it (or direction unknown): the stops the hypotheses are about."""
  m = st.motion
  return m is not None and m['kind'] in MOVE_KINDS and not m.get('requested') and category(st) != 'op_off_roll'


def is_roll(st):
  return category(st) != 'held'


def analyse(D):
  np = D.np
  runs = find_still_runs(D)
  la = D.s['longActive']
  stops, other = [], 0
  i = 0
  while i < len(runs):
    t0s, t1, tl, dur = runs[i]
    if dur < STOP_MIN_S * NS:
      i += 1
      continue
    op = la.any_on(t0s - int(LONG_BEFORE_S * NS), t0s + int(LONG_AFTER_S * NS))
    if not op:
      other += 1
      i += 1
      continue
    st = Stop()
    st.t_still, st.t1, st.t_last_still, st.still_s = t0s, t1, tl, dur / NS
    st.t0, st.anchor_how = find_anchor(D, t0s, t1 if t1 is not None else tl)
    st.cut_start = t0s - int(D.s['vEgo'].t[0]) < int(0.3 * NS)     # the log (or --segments) begins mid-stop
    st.op_at_stop = la.at1(st.t0) >= 0.5 if isnum(la.at1(st.t0)) else False
    tstill_end = t1 if t1 is not None else tl
    gt, gv = D.grade.sl(st.t0, max(tstill_end, st.t0 + int(0.5 * NS)))
    gv = gv[~np.isnan(gv)] if len(gv) else gv
    if not len(gv):
      gt, gv = D.grade.sl(st.t0 - 5 * NS, tstill_end + 5 * NS)
      gv = gv[~np.isnan(gv)] if len(gv) else gv
    st.grade = float(np.median(gv)) if len(gv) else float('nan')
    st.restops = []
    nxt_real = next((r[0] for r in runs[i + 1:] if r[3] >= STOP_MIN_S * NS), None)
    st.onset, st.onset_how, st.early_moves = find_onset(D, st)
    st.motion = None
    j = i + 1
    if st.onset is not None:
      st.motion = classify_motion(D, st, nxt_real)
      # a roll that ended in a new standstill: that standstill is part of this event - unless the movement was
      # asked for (an OP creep/launch, the driver) and OP itself brought the car to a stop again: that is a new OP
      # stop with its own hold and is analysed on its own (otherwise a let-go/rollback there would be hidden).
      if st.motion['kind'] in MOVE_KINDS and t1 is not None and st.onset_end is None:
        while j < len(runs) and runs[j][0] - t1 <= RESTOP_MERGE_S * NS:
          if st.motion['requested'] and runs[j][3] >= STOP_MIN_S * NS and la.at1(runs[j][0]) >= 0.5:
            break
          st.restops.append(runs[j])
          j += 1
    if st.motion is None:
      st.t_end = st.t_last_still
    else:
      st.t_end = max(st.motion['t_end'], st.onset + int(0.5 * NS))
      if st.restops:
        rs_end = st.restops[-1][1] or st.restops[-1][2]
        st.t_end = max(st.t_end, min(rs_end, st.restops[-1][0] + 5 * NS))
    stops.append(st)
    i = j
  for n, st in enumerate(stops, 1):
    st.no = n
  D.stops, D.other_stops = stops, other
  nv = D.c['nv']
  D.nv_first = None
  if len(nv):
    D.nv_first = int(nv.t[0]) if nv.v[0] >= 0.5 else nv.first(int(nv.t[0]), int(nv.t[-1]), lambda v: v >= 0.5)
  for st in stops:
    esp_metrics(D, st)
    mm = [x for x in getattr(D, 'pulse_mismatch', []) if st.t0 - NS <= x[0] <= st.t_end + NS]
    st.pul_mm = max(mm, key=lambda x: x[3]) if mm else None     # the biggest pulse/speed disagreement
  D.b53_any = any(s.hold_on is not None for s in stops if s.esp_ok)
  D.faults = find_faults(D)
  D.route_stats = route_stats(D)
  D.h7 = hms_switch_stats(D)


def op_edges(D, a, b):
  """Every change in what OP sent (sendcan) or OP's long state in [a, b]: [(t, text)]."""
  out = []
  c = D.c
  for key, name in (('hms', 'HMS'), ('acc07_stop', 'ACC_07 Anhalten'), ('acc07_go', 'ACC_07 Anfahren'),
                    ('acc06_stop', 'ACC_06 Anhalten'), ('acc06_go', 'ACC_06 Anfahren'), ('acc06_status', 'ACC_06 status')):
    for t, o, n in c[key].changes(a, b):
      out.append((t, f'{name} {ival(o, "-")}->{ival(n, "-")}'))
  for t, o, n in c['acc07_dist'].changes(a, b):
    if not (isnum(o) and isnum(n) and abs(o - n) < 0.05):
      out.append((t, f'ACC_07 Anhalteweg {fnum(o, "{:.2f}")}->{fnum(n, "{:.2f}")}'))
  a06 = c['acc06_accel']
  t6, v6 = a06.sl(a, b)
  prev = a06.at1(a - 1)
  for t, v in zip(t6.tolist(), v6.tolist()):
    if isnum(prev) != isnum(v) or (isnum(prev) and isnum(v) and (abs(v - prev) >= 0.3 or (v > 0.05) != (prev > 0.05))):
      out.append((t, f'ACC_06 accel {fnum(prev, "{:+.2f}")}->{fnum(v, "{:+.2f}")}'))
    prev = v
  for t, o, n in D.s['lcs'].changes(a, b):
    out.append((t, f'OP long state {lcs_name(o)}->{lcs_name(n)}'))
  out.sort(key=lambda e: (e[0], 0 if e[1].startswith("HMS") else 1))
  return out


def op_wants_stop(D, t):
  """Did OP still ask to STAND (not to go) in the 0.1 s from t? Judged from what it sent in ACC_06 (fallback: carControl)."""
  c = D.c
  if len(c['acc06_stop']) and c['acc06_stop'].rate(t, t + int(0.1 * NS)) > 0:
    if c['acc06_go'].any_on(t, t + int(0.1 * NS)):
      return False
    _, av = c['acc06_accel'].sl(t, t + int(0.1 * NS))
    if len(av) and D.np.any(av > 0.05):
      return False
    return c['acc06_stop'].any_on(t, t + int(0.1 * NS))
  lc = D.s['lcs'].at1(t + int(0.03 * NS))
  return lc == 2


def first_go(D, a, b):
  """First moment in [a, b] OP stopped asking to stand: ACC_06 Anfahren, a positive accel, or shouldStop off in pid."""
  if b is None or b <= a:
    return None
  c, s = D.c, D.s
  cands = []
  if len(c['acc06_go']):
    cands.append(c['acc06_go'].first(a, b, lambda v: v >= 0.5))
    cands.append(c['acc06_accel'].first(a, b, lambda v: v > 0.05))
  else:
    cands.append(s['ccAccel'].first(a, b, lambda v: v > 0.05))
  if D.has_plan:
    tss = s['shouldStop'].first(a, b, lambda v: v < 0.5)
    if tss is not None and s['lcs'].at1(tss + int(0.05 * NS)) == 1:
      cands.append(tss)
  cands = [x for x in cands if x is not None]
  return min(cands) if cands else None


def frames_at(D, sig, a, b):
  return sig.sl(a, b)


def esp_metrics(D, st):
  np = D.np
  c, s = D.c, D.s
  t0 = st.t0
  m = st.motion
  on = st.onset
  # OP's "stopping" state began
  lc = [x for x in s['lcs'].changes(t0 - 10 * NS, t0 + int(0.5 * NS)) if x[2] == 2]
  st.t_lcs_stop = lc[-1][0] if lc else None
  if st.t_lcs_stop is None and s['lcs'].at1(t0) == 2:
    st.t_lcs_stop = s['lcs'].first(t0 - 10 * NS, t0, lambda v: v == 2)
  pre = min(st.t_lcs_stop or t0, t0) - int(LONG_BEFORE_S * NS)
  st.pre = pre
  b = st.t_end
  stand_end = on if on is not None else (st.t1 if st.t1 is not None else st.t_last_still)
  st.esp_ok = c['hold'].rate(pre, max(b, t0 + NS)) >= FULL_RATE_HZ
  st.op_ok = c['hms'].rate(pre, max(b, t0 + NS)) >= FULL_RATE_HZ
  st.tsk_ok = c['tsk'].rate(pre, max(b, t0 + NS)) >= FULL_RATE_HZ
  st.anf_on = c['anf'].first(t0 - NS, b, lambda v: v >= 0.5) if len(c['anf']) else None
  st.anf_pre = c['anf'].at1(t0 - NS) >= 0.5 if len(c['anf']) and isnum(c['anf'].at1(t0 - NS)) else False
  st.anf_off = c['anf'].first(st.anf_on, b, lambda v: v < 0.5) if st.anf_on is not None else None
  st.hold_on = c['hold'].first(t0 - NS, b, lambda v: v >= 0.5) if len(c['hold']) else None
  st.hold_off = c['hold'].first(st.hold_on, b, lambda v: v < 0.5) if st.hold_on is not None else None
  st.b63_on = c['esp_acc_stop'].first(pre, b, lambda v: v >= 0.5) if len(c['esp_acc_stop']) else None
  st.b63_off = c['esp_acc_stop'].first(st.b63_on, b, lambda v: v < 0.5) if st.b63_on is not None else None
  nv = c['nv']
  st.nv_pre = len(nv) > 0 and isnum(nv.at1(pre)) and nv.at1(pre) >= 0.5
  st.nv_at_stop = len(nv) > 0 and isnum(nv.at1(t0)) and nv.at1(t0) >= 0.5
  st.nv_on = None if st.nv_pre or not len(nv) else nv.first(pre, b, lambda v: v >= 0.5)
  tsk = c['tsk']
  st.tsk_max = None
  st.tsk_fault = None
  if len(tsk):
    _, tv = tsk.sl(pre, b)
    if len(tv):
      st.tsk_max = int(np.nanmax(tv))
      tf = tsk.first(pre, b, lambda v: v >= 6)
      if tf is not None:
        st.tsk_fault = (tf, int(tsk.at1(tf)))
  # what OP sent
  hms = c['hms']
  st.hms_seq = '-'
  st.hms_changes = []
  st.hms_max_s = 0
  st.hms_per1000 = None
  if len(hms):
    ht, hv = hms.sl(t0, b)
    v0 = hms.at1(t0)
    st.hms_seq = seq_text(([v0] if isnum(v0) else []) + list(hv))
    st.hms_changes = [x[0] for x in hms.changes(t0, b)]
    st.hms_max_s = max_per_second(np, st.hms_changes)
    vs = s['vEgo'].at(ht)
    mm = vs < STILL_V
    if np.sum(mm) >= 10:
      hh = hv[mm]
      st.hms_per1000 = 1000.0 * float(np.sum(hh[1:] != hh[:-1])) / float(np.sum(mm))
  st.rejected = sum(int(np.sum((r >= pre) & (r <= b))) for _, r in D.echo.values())
  st.driver_brake = s['brakePressed'].first(t0 - NS, stand_end, lambda v: v >= 0.5)
  st.op_off = None
  la = s['longActive']
  if st.op_at_stop:
    st.op_off = la.first(t0, stand_end, lambda v: v < 0.5)
  st.handbrake = s['parkingBrake'].any_on(t0, stand_end)

  # ---- H1: how long OP asked for a hold, and how much of it with the wheels at 0
  a1 = st.t_lcs_stop if st.t_lcs_stop is not None else t0 - NS
  st.latch_t = None
  st.t_go = first_go(D, t0, stand_end)
  for t, _o, n in hms.changes(a1, b):
    if n == 4 and op_wants_stop(D, t) and (st.t_go is None or t < st.t_go):
      st.latch_t = t                         # HMS 4 while OP still wants to stop = the latch (not a launch)
      break
  e1 = st.latch_t or stand_end
  ht, hv = hms.sl(a1, e1)
  fr = (np.median(np.diff(hms.t)) / NS) if len(hms) > 3 else 0.02
  st.hms1_s = float(np.sum(hv == 1)) * fr if len(hv) else 0.0
  st.hms1_still_s = float(np.sum((hv == 1) & (D.wz.at(ht) >= 0.5))) * fr if len(hv) and len(D.wz) else None
  # ---- H8: pid frames with vEgo < 0.5 that sent HMS 4 / Anfahren before standstill
  st.h8 = st.h8_pid_slow = 0
  if st.op_ok:
    ht, hv = hms.sl(pre, t0)
    if len(ht):
      lcs_f = s['lcs'].at(ht)
      v_f = s['vEgo'].at(ht)
      go_f = c['acc06_go'].at(ht)
      pid_slow = (lcs_f == 1) & (v_f < 0.5)
      st.h8_pid_slow = int(np.sum(pid_slow))
      st.h8 = int(np.sum(pid_slow & ((hv == 4) | (go_f >= 0.5))))
  # ---- H6: what OP sent while the car rolled, by speed band; long state from every carControl sample
  st.h6 = None
  if m is not None and m['kind'] in MOVE_KINDS and st.op_ok:
    a6, b6 = on, min(on + 5 * NS, max(m['t_end'], on + NS))
    ht, hv = hms.sl(a6, b6)
    if len(ht):
      v_f = s['vEgo'].at(ht)
      stop_f = np.nan_to_num(c['acc06_stop'].at(ht), nan=0.0) >= 0.5
      go_f = np.nan_to_num(c['acc06_go'].at(ht), nan=0.0) >= 0.5
      ct, cl_ = s['lcs'].sl(a6, b6)
      cv = s['vEgo'].at(ct) if len(ct) else np.zeros(0)
      h6 = {}
      for band, sel, csel in (('<1.0', v_f < 1.0, cv < 1.0), ('>=1.0', v_f >= 1.0, cv >= 1.0)):
        hs = {}
        for h in np.unique(hv[sel & ~np.isnan(hv)]):
          hs[int(h)] = int(np.sum(sel & (hv == h)))
        ls = {}
        for lv in np.unique(cl_[csel & ~np.isnan(cl_)]) if len(cl_) else []:
          ls[lcs_name(lv)] = int(np.sum(csel & (cl_ == lv)))
        h4_stop = int(np.sum(sel & (hv == 4) & stop_f & ~go_f))
        h6[band] = (int(np.sum(sel)), hs, ls, h4_stop)
      st.h6 = h6
      st.h6_lcs_flips = len(s['lcs'].changes(a6, b6))
      st.h6_window = (a6, b6)
  # ---- H4 / H5: wheel brake torque the TSK asked for at standstill, while OP still wanted to stand
  brm = c['tsk_brake_raw']
  hold_end = min(x for x in (stand_end, st.t_go, st.op_off, t0 + 30 * NS) if x is not None)
  st.stand_win_end = hold_end
  st.brm_ok = len(brm) > 0 and brm.rate(t0, max(stand_end, t0 + NS)) >= FULL_RATE_HZ
  wa = t0 + int(0.2 * NS)
  wb = max(hold_end, wa + int(0.1 * NS))
  st.brm_stand = st.brm_note = st.brm_key = None
  st.brm_parts = {}
  st.brm_onset = brm.at1(on) if on is not None else float('nan')
  if st.brm_ok:
    lt = st.latch_t
    pre_b = min(wb, lt) if lt is not None else wb
    parts = {}
    bt, bv = brm.sl(wa, pre_b)
    if len(bt):
      hf = np.nan_to_num(c['hold'].at(bt), nan=0.0) >= 0.5
      ok = ~np.isnan(bv)
      if np.sum(ok & ~hf) >= 5:
        parts['pre_nohold'] = float(np.median(bv[ok & ~hf]))
      if np.sum(ok & hf) >= 5:
        parts['pre_hold'] = float(np.median(bv[ok & hf]))
    if lt is not None:
      bt, bv = brm.sl(lt + int(0.1 * NS), wb)
      bv = bv[~np.isnan(bv)] if len(bv) else bv
      if len(bv) >= 5:
        parts['post'] = float(np.median(bv))
    st.brm_parts = parts
    for key, note in (('pre_nohold', 'without the ESP hold, before any HMS 4 latch'),
                      ('pre_hold', 'while the ESP hold was confirmed'),
                      ('post', 'after OP\'s HMS 4 latch')):
      if key in parts:
        st.brm_stand, st.brm_note, st.brm_key = parts[key], note, key
        break
  a06 = c['acc06_accel'] if len(c['acc06_accel']) else s['ccAccel']
  st.a_req = a06.median(wa, wb)
  mass = D.mass.median(t0 - 5 * NS, wb + 5 * NS) if len(D.mass) else float('nan')
  st.mass_src = 'TSK_Fahrzeugmasse_02 (0x65F)' if isnum(mass) else f'assumed {DEFAULT_MASS_KG:.0f} kg'
  st.mass = mass if isnum(mass) else DEFAULT_MASS_KG
  st.brm_exp = torque_expectation(st.grade, st.mass, st.a_req)
  st.brm_class = classify_torque(st.brm_stand, st.brm_exp, st.grade)
  # H5: torque after OP latched HMS 4
  st.h5 = None
  if st.latch_t is not None and st.brm_ok:
    lt = st.latch_t
    pre_v = brm.median(lt - int(0.5 * NS), lt - int(0.02 * NS))
    # H5 is about the TSK dropping its torque while OP is still in control and before the car moves. Torque that
    # goes to 0 because the driver braked or OP switched off (or long after the roll began) does not count.
    caps = [(lt + 2 * NS, 'latch +2 s')]
    if on is not None:
      caps.append((max(on, lt) + int(0.3 * NS), 'the wheels moved (+0.3 s)'))
    t_la_off = la.first(lt, lt + 2 * NS, lambda v: v < 0.5)
    if t_la_off is not None:
      caps.append((t_la_off, 'OP switched off'))
    t_brk = s['brakePressed'].first(lt, lt + 2 * NS, lambda v: v >= 0.5)
    if len(c['esp_brake']):
      tb2 = c['esp_brake'].first(lt, lt + 2 * NS, lambda v: v >= 0.5)
      t_brk = tb2 if t_brk is None or (tb2 is not None and tb2 < t_brk) else t_brk
    if t_brk is not None:
      caps.append((t_brk, 'driver brake'))
    win_end, win_why = min(caps, key=lambda x: x[0])
    judge = win_end - lt >= int(0.1 * NS)
    tz = brm.first(lt, win_end, lambda v: v <= H5_ZERO_RAW) if judge else None
    if tz is not None and not (la.at1(tz) >= 0.5):
      tz = None
    tz_late = brm.first(win_end, lt + 30 * NS, lambda v: v <= H5_ZERO_RAW) if tz is None else None
    end_chk = win_end
    after_v = brm.median(lt + int(0.1 * NS), max(end_chk, lt + int(0.2 * NS)))
    stays = None
    if tz is not None:
      zt, zv = brm.sl(tz, max(end_chk, tz + int(0.2 * NS)))
      stays = bool(len(zv)) and float(np.mean(zv <= H5_ZERO_RAW)) >= 0.8
    rate = brm.rate(lt, lt + NS) or 50.0
    st.h5 = {'pre': pre_v, 'after': after_v, 'zero_t': tz, 'judge': judge, 'win_end': win_end, 'win_why': win_why,
             'zero_late': tz_late,
             'frames': int(round((tz - lt) / NS * rate)) if tz is not None else None, 'stays': stays,
             'a06_stop': c['acc06_stop'].at1(lt + int(0.05 * NS)), 'a06_acc': c['acc06_accel'].at1(lt + int(0.05 * NS)),
             'freig_after': c['tsk_freig'].median(lt + int(0.1 * NS), lt + NS) if len(c['tsk_freig']) else float('nan'),
             'verz_after': c['verz_tsk'].median(lt + int(0.1 * NS), lt + NS) if len(c['verz_tsk']) else float('nan')}
  st.bar_stand = float('nan')
  if len(c['brake_bar']):
    pt_, pv_ = c['brake_bar'].sl(wa, max(wb, wa + 1))
    if len(pt_):
      nh = np.nan_to_num(c['hold'].at(pt_), nan=0.0) < 0.5
      sel = pv_[nh] if np.sum(nh) >= 3 else pv_
      sel = sel[~np.isnan(sel)]
      st.bar_stand = float(np.median(sel)) if len(sel) else float('nan')
  # ---- H1 vs H2: what OP did right before the ESP hold dropped
  st.drop = None
  rolled = m is not None and m['kind'] in MOVE_KINDS and not m.get('requested')
  if not rolled:
    pass
  elif on is not None and st.hold_on is not None and st.hold_off is not None and st.hold_off <= on + int(0.05 * NS):
    st.drop = drop_analysis(D, st.hold_off, 'bit 53 (hold confirmed)')
  elif on is not None and st.hold_on is None and st.anf_on is not None and st.anf_off is not None \
          and st.anf_off <= on + int(0.05 * NS):
    st.drop = drop_analysis(D, st.anf_off, 'bit 28 (ESP hold active)')
  st.hold_at_onset = c['hold'].at1(on) if on is not None else float('nan')
  # ---- H9: pedal presses and OP switch-off before the wheels moved
  h9a = min(st.t_lcs_stop or t0, t0) - NS
  h9b = on if on is not None else st.t_end
  st.h9 = []
  for key, name in (('brakePressed', 'driver brake pressed'), ('gasPressed', 'driver gas pressed')):
    for t, o, n in s[key].changes(h9a, h9b):
      if n >= 0.5 and not (isnum(o) and o >= 0.5):
        st.h9.append((t, name))
  for t, o, n in la.changes(h9a, h9b):
    if n < 0.5 and isnum(o) and o >= 0.5:
      st.h9.append((t, 'OP long control switched OFF'))
  st.h9.sort()
  st.pedal_after = []
  if on is not None:
    for key, name in (('brakePressed', 'driver brake'), ('gasPressed', 'driver gas')):
      tb = s[key].first(on, st.t_end + int(0.5 * NS), lambda v: v >= 0.5)
      if tb is not None:
        st.pedal_after.append((tb, name))


def torque_expectation(grade, mass, a_req):
  """Expected TSK_Radbremsmom raw (x8 Nm) at standstill if the TSK subtracts the grade from OP's request
  (compensated) or not, and what holding the car needs."""
  if not isnum(grade) or not isnum(a_req):
    return None
  th = math.atan(grade / 100.0)
  gs = GRAV * math.sin(th)              # + = gravity pulls the car backward (uphill)
  mr = mass * WHEEL_R_M
  comp = max(0.0, -a_req - gs) * mr if grade >= 0 else max(0.0, -a_req + gs) * mr
  uncomp = max(0.0, -a_req) * mr
  need = abs(gs) * mr
  return {'comp': comp / 8.0, 'uncomp': uncomp / 8.0, 'need': need / 8.0, 'a_req': a_req, 'mass': mass, 'gs': gs}


def classify_torque(raw, exp, grade):
  if raw is None or not isnum(raw):
    return None
  if raw <= H5_ZERO_RAW:
    return 'zero'
  if exp is None:
    return None
  if not isnum(grade) or abs(grade) < 2.0 or abs(exp['comp'] - exp['uncomp']) < 4:
    return 'flat'
  dc, du = abs(raw - exp['comp']), abs(raw - exp['uncomp'])
  tol = max(4.0, 0.35 * max(exp['comp'], exp['uncomp']))
  if min(dc, du) > tol:
    return 'other'
  return 'comp' if dc < du else 'uncomp'


def drop_analysis(D, t_drop, what):
  """What OP changed in the 0.5 s before the ESP let go (and right after)."""
  # an OP change less than one CAN frame (15 ms) before the drop cannot have caused it: that is OP reacting
  cut = t_drop - int(0.015 * NS)
  before = op_edges(D, t_drop - int(1.5 * NS), cut)
  after = op_edges(D, cut + 1, t_drop + int(0.5 * NS))
  last = before[-1] if before else None
  steady = (t_drop - last[0]) / NS if last else 1.5
  rate = 50.0
  same = [e for e in before if last is not None and last[0] - e[0] <= int(0.025 * NS)]
  out = {'t': t_drop, 'what': what, 'last': last, 'steady_s': steady, 'frames': int(round(steady * rate)),
         'same': same, 'after': after, 'steady_full': last is None}
  if last is not None and steady <= H1_EDGE_S:
    out['kind'] = 'H1'
  elif steady >= H2_STEADY_S:
    out['kind'] = 'H2'
  else:
    out['kind'] = 'mid'
  return out


def max_per_second(np, times):
  if len(times) < 2:
    return len(times)
  ts = np.asarray(times, dtype=np.int64)
  j = np.searchsorted(ts, ts + NS, side='left')
  return int(np.max(j - np.arange(len(ts))))


def seq_text(vals, maxn=5):
  runs = []
  for v in vals:
    if not isnum(v):
      continue
    v = int(v)
    if not runs or runs[-1] != v:
      runs.append(v)
  if not runs:
    return '-'
  if len(runs) <= maxn:
    return '>'.join(str(x) for x in runs)
  return '>'.join(str(x) for x in runs[:maxn - 1]) + '>..'


def hms_changes(D, a, b):
  return [c[0] for c in D.c['hms'].changes(a, b)]


def edges(sig, a, b, rising_only=False):
  out = []
  for t, o, n in sig.changes(a, b):
    if not isnum(n) or not isnum(o):
      continue
    if (n >= 0.5) != (o >= 0.5) and (not rising_only or n >= 0.5):
      out.append(t)
  return out


def find_faults(D):
  np = D.np
  tsk = D.c['tsk']
  out = []
  if not len(tsk):
    return out
  for i0, i1 in runs_of(np, tsk.v >= 6):
    f = Stop()
    f.t0 = int(tsk.t[i0])
    f.t_endf = int(tsk.t[i1 - 1])
    f.value = int(np.max(tsk.v[i0:i1]))
    f.before = int(tsk.v[i0 - 1]) if i0 > 0 else None
    f.speed = D.s['vEgo'].at1(f.t0)
    f.during_stop = next((s.no for s in D.stops if s.t0 - LONG_BEFORE_S * NS <= f.t0 <= s.t_end), None)
    ch = hms_changes(D, f.t0 - 3 * NS, f.t0)
    f.hms_changes_3s = len(ch)
    f.hms_max_s = max_per_second(np, ch)
    f.lcs = D.s['lcs'].at1(f.t0)
    f.long_active = D.s['longActive'].at1(f.t0)
    f.grade = D.grade.at1(f.t0)
    f.nv = D.c['nv'].at1(f.t0)
    f.hms = D.c['hms'].at1(f.t0)
    f.lasts_to_end = i1 == len(tsk.v)
    g = [d for t, d in D.send_gaps if f.t0 - 3 * NS <= t + d and t <= f.t0]
    f.max_gap_ms = max(g) / 1e6 if g else None
    f.foreign = sum(int(np.sum((x >= f.t0 - 3 * NS) & (x <= f.t0))) for x in D.foreign.values())
    classify_fault(D, f)
    out.append(f)
  for n, f in enumerate(out, 1):
    f.no = n
  return out


def classify_fault(D, f):
  """Decision rule 6: the 2 s before a TSK 6/7 edge -> chatter / panda block / pedal toggling / ESP give-up."""
  np = D.np
  a, b = f.t0 - int(FAULT_WIN_S * NS), f.t0
  c, s = D.c, D.s
  ev = {}
  ev['hms_chg'] = len(c['hms'].changes(a, b)) if len(c['hms']) else None
  echo = D.acc_src.get(D.echo_src, {}).get('hms')
  ev['hms_chg_echo'] = len(echo.changes(a, b)) if echo is not None and len(echo) else None
  ev['a06_chg'] = (len(c['acc06_go'].changes(a, b)) + len(c['acc06_stop'].changes(a, b))) if len(c['acc06_go']) else None
  ev['lcs_chg'] = len(s['lcs'].changes(a, b)) if len(s['lcs']) else None
  ev['gas_edges'] = len(edges(s['gasPressed'], a, b))
  ev['brake_edges'] = len(edges(s['brakePressed'], a, b))
  ev['la_off'] = len([t for t, o, n in s['longActive'].changes(a, b) if n < 0.5 and isnum(o) and o >= 0.5])
  if D.has_panda:
    _, tb = s['safetyTxBlocked'].sl(a - int(0.2 * NS), b)
    ev['txblk'] = int(np.nanmax(tb) - np.nanmin(tb)) if len(tb) else 0
    ev['ctrl_drop'] = len([t for t, o, n in s['controlsAllowed'].changes(a, b) if n < 0.5 and isnum(o) and o >= 0.5])
  else:
    ev['txblk'] = ev['ctrl_drop'] = None
  ev['rej'] = sum(int(np.sum((r >= a) & (r <= b))) for _, r in D.echo.values()) if D.has_echo else None
  gaps = [d for t, d in D.send_gaps if a <= t + d and t <= b]
  ev['gap_ms'] = max(gaps) / 1e6 if gaps else 0.0
  ev['foreign'] = sum(int(np.sum((x >= a) & (x <= b))) for x in D.foreign.values())
  bits = {}
  for key, name in (('nv', 'b29'), ('kons_acc', 'b38'), ('kons_tsk33', 'b45'), ('kons_tsk05', 'esp05_b29')):
    sg = c[key]
    if not len(sg):
      bits[name] = None
      continue
    rise = [t for t in edges(sg, a, b, rising_only=True)]
    bits[name] = ((rise[0] - f.t0) / NS if rise else None, sg.at1(b))
  ev['bits'] = bits
  labels = []
  if (ev['lcs_chg'] or 0) >= 4 or (ev['hms_chg'] or 0) >= 4 or (ev['a06_chg'] or 0) >= 4:
    labels.append('OP chatter (trap 4)')
  if (ev['txblk'] or 0) > 0 or (ev['rej'] or 0) > 0 or (ev['ctrl_drop'] or 0) > 0:
    labels.append('panda block')
  if ev['gas_edges'] >= 2 or ev['gas_edges'] + ev['brake_edges'] >= 4:
    labels.append('pedal toggling (#2874)')
  b29 = bits.get('b29')
  give = (b29 and b29[0] is not None) or any(bits.get(k) and bits[k][0] is not None for k in ('b38', 'b45'))
  if give:
    labels.append('ESP give-up')
  extra = []
  if ev['gap_ms'] > SEND_GAP_NS / 1e6:
    extra.append(f'OP send pause {ev["gap_ms"]:.0f} ms')
  if ev['foreign']:
    extra.append(f'second ACC sender ({ev["foreign"]} frames)')
  f.ev, f.labels, f.extra = ev, labels, extra


def route_stats(D):
  np = D.np
  out = {}
  hms = D.c['hms']
  if len(hms):
    vs = D.s['vEgo'].at(hms.t)
    m = vs < STILL_V
    out['frames'] = int(len(hms.t))
    out['stand_frames'] = int(np.sum(m))
    hv = hms.v
    chg = np.concatenate([[False], hv[1:] != hv[:-1]])
    out['stand_switches'] = int(np.sum(chg & m))
    g = D.grade.at(hms.t)
    gm = m & (np.abs(np.nan_to_num(g, nan=0.0)) >= 3.0)
    out['grade_frames'] = int(np.sum(gm))
    out['grade_switches'] = int(np.sum(chg & gm))
    out['all_switches'] = int(np.sum(chg))
  nv = D.c['nv']
  if len(nv):
    out['nv_rises'] = [c[0] for c in nv.changes(nv.t[0], nv.t[-1]) if c[2] >= 0.5 and c[1] < 0.5]
    out['nv_always'] = bool(np.all(nv.v >= 0.5))
    out['nv_frac'] = float(np.mean(nv.v >= 0.5))
  out['rejected'] = {k: int(len(v[1])) for k, v in D.echo.items()}
  out['returned'] = {k: int(len(v[0])) for k, v in D.echo.items()}
  return out


def switch_count(D, t, v):
  """The user's measure on one stream of ACC_07 frames: HMS switches per 1000 grade-standstill frames."""
  np = D.np
  if not len(t):
    return None
  vs = D.s['vEgo'].at(t)
  g = D.grade.at(t)
  gm = (vs < STILL_V) & (np.abs(np.nan_to_num(g, nan=0.0)) >= 3.0)
  vf = np.where(np.isnan(v), -1.0, v)
  chg = np.concatenate([[False], vf[1:] != vf[:-1]])
  sel = chg & gm
  frames = int(np.sum(gm))
  sw = int(np.sum(sel))
  pairs = {}
  idx = np.nonzero(sel)[0]
  for i in idx.tolist():
    p = tuple(sorted((int(vf[i - 1]), int(vf[i]))))
    pairs[p] = pairs.get(p, 0) + 1
  vals = {}
  for x in np.unique(vf[gm]):
    vals[int(x)] = int(np.sum(vf[gm] == x))
  return {'frames': frames, 'switches': sw, 'per1000': (1000.0 * sw / frames) if frames else None,
          'pairs': sorted(pairs.items(), key=lambda kv: -kv[1]), 'values': vals}


def hms_switch_stats(D):
  """Decision rule 5 (H7): HMS switch counts per source over the same grade-standstill definition."""
  out = []
  if len(D.c['hms']):
    out.append(('sendcan', switch_count(D, D.c['hms'].t, D.c['hms'].v)))
  for src in sorted(D.acc_src):
    h = D.acc_src[src].get('hms')
    if h is not None and len(h):
      out.append((src, switch_count(D, h.t, h.v)))
  t, v, srcs = D.hms_mixed
  if len(t) and len(set(srcs.tolist())) > 1:
    out.append(('mixed', switch_count(D, t, v)))
  res = {'rows': out, 'verdict': None, 'kind': None}
  rows = dict(out)
  mixed = rows.get('mixed')
  own = rows.get(D.echo_src) or rows.get('sendcan')
  if not any(r and r['frames'] for _, r in out):
    res['kind'] = 'none'
    res['verdict'] = ('no frames while standing on a grade of 3% or more in this drive, so the switch count cannot '
                      'be checked here (H7 needs a stop on a hill)')
    return res
  own_rate = own['per1000'] if own and own['per1000'] is not None else 0.0
  if mixed and mixed['per1000'] is not None and mixed['frames'] >= 50:
    if mixed['per1000'] >= 100 and mixed['per1000'] >= 5 * max(own_rate, 1.0):
      res['kind'] = 'artifact'
      res['verdict'] = (f'consistent with H7: reading every 0x12E frame regardless of source gives '
                        f'{mixed["per1000"]:.0f} switches per 1000 grade-standstill frames, but OP\'s own frames '
                        f'switch only {own_rate:.0f} per 1000. The high figure comes from mixing sources, not from OP.')
      return res
  if own_rate >= 100:
    res['kind'] = 'real'
    res['verdict'] = (f'OP\'s own frames really switch {own_rate:.0f} per 1000 grade-standstill frames -> not a log '
                      'artifact (H7 does not explain it); compare with the rollback frames (H6)')
  else:
    res['kind'] = 'low'
    res['verdict'] = ('no source switches often in this drive; the 812-per-1000 figure is not reproduced here' +
                      (' (only one source of 0x12E in the log, so mixing cannot be checked)' if not mixed else ''))
  return res


# ============================================================================================ snapshot at onset
def snapshot(D, t):
  """Everything decision rule 2 needs, at one moment. Returns an ordered list of (key, text)."""
  c, s = D.c, D.s
  out = []

  def add(k, v):
    out.append((k, v))

  def thin(sig):
    return sig.rate(t - NS // 2, t + NS // 2) < FULL_RATE_HZ / 2      # qlog / too few frames: do not guess

  def bit(sig):
    if not len(sig):
      return '-'
    return '?' if thin(sig) else b01(sig.at1(t))

  def num(sig, fmt):
    if not len(sig):
      return '-'
    x = sig.at1(t)
    return fmt.format(x) if isnum(x) and not thin(sig) else '?'

  add('b53', bit(c['hold']))
  add('b63', bit(c['esp_acc_stop']))
  add('b28', bit(c['anf']))
  add('b29', bit(c['nv']))
  add('b38', bit(c['kons_acc']))
  add('b45', bit(c['kons_tsk33']))
  add('hms_sent', num(c['hms'], '{:.0f}'))
  for src in sorted(D.acc_src):
    h = D.acc_src[src].get('hms')
    if src < 192 and h is not None and len(h):
      add(f'hms_src{src}', num(h, '{:.0f}'))
  add('a06_stp', bit(c['acc06_stop']))
  add('a06_go', bit(c['acc06_go']))
  add('a06_acc', num(c['acc06_accel'], '{:+.2f}'))
  add('a06_st', num(c['acc06_status'], '{:.0f}'))
  add('a06_jneg', num(c['acc06_jerk_neg'], '{:.2f}'))
  add('a07_stp', bit(c['acc07_stop']))
  add('a07_go', bit(c['acc07_go']))
  add('a07_dist', num(c['acc07_dist'], '{:.2f}'))
  add('a07_acc', num(c['acc07_accel'], '{:+.2f}'))
  add('tsk', num(c['tsk'], '{:.0f}'))
  add('brm_raw', num(c['tsk_brake_raw'], '{:.0f}'))
  add('brm_nm', num(c['tsk_brake_nm'], '{:.0f}'))
  add('freig30', bit(c['tsk_freig']))
  add('stby28', bit(c['tsk_standby']))
  add('srbm', num(c['tsk_srbm_anf'], '{:.0f}'))
  add('bar', num(c['brake_bar'], '{:.1f}'))
  add('verz27', bit(c['verz_tsk']))
  add('kons29', bit(c['kons_tsk05']))
  add('rev39', bit(c['esp_reverse']))
  add('anfh40', bit(c['anfahrhilfe']))
  add('drvbrk26', bit(c['esp_brake']))
  add('grade', fnum(D.grade.at1(t), '{:+.1f}'))
  add('mass', fnum(D.mass.at1(t), '{:.0f}') if len(D.mass) else '-')
  add('mo_nm', num(c['mo_torque'], '{:.0f}'))
  add('pedal', num(c['pedal'], '{:.1f}'))
  gc_ = c['gear_code'].at1(t) if len(c['gear_code']) else float('nan')
  gname = D.dbc.val_name('Getriebe_11', 'GE_Fahrstufe', gc_) if isnum(gc_) else None
  add('gear', (f'{int(gc_)}' + (f'({gname})' if gname else '')) if isnum(gc_) else ('-' if not len(c['gear_code']) else '?'))
  add('lcs', lcs_name(s['lcs'].at1(t)) if len(s['lcs']) else '-')
  add('lcs_cs', lcs_name(s['lcs_ctrl'].at1(t)) if D.has_ctl else '-')
  add('ss', bit(s['shouldStop']))
  add('aT', num(s['aTarget'], '{:+.2f}'))
  add('vplan0', num(s['vPlan0'], '{:.2f}') if D.has_vplan else '-')
  add('la', bit(s['longActive']))
  add('brk', bit(s['brakePressed']))
  add('gas', bit(s['gasPressed']))
  add('v', num(s['vEgoRaw'] if len(s['vEgoRaw']) else s['vEgo'], '{:.2f}'))
  add('pitch', num(s['pitch'], '{:+.3f}') if D.has_pitch else '-')
  if D.has_panda:
    add('ctrl_ok', bit(s['controlsAllowed']))
    add('ctrl_long', bit(s['controlsAllowedLong']))
    add('txblk', num(s['safetyTxBlocked'], '{:.0f}'))
  return out


def snapshot_lines(D, snap):
  """Plain-English rendering of a snapshot (one topic per line)."""
  d = dict(snap)
  srcs = ' '.join(f'{k[4:]}={v}' for k, v in snap if k.startswith('hms_src'))
  lines = [
    f"ESP_21: hold confirmed (bit 53)={d['b53']}, ACC stopping active (bit 63)={d['b63']}",
    f"ESP_33: hold active (bit 28)={d['b28']}, gave up (bit 29)={d['b29']}, consistency ACC (bit 38)={d['b38']}, "
    f"TSK (bit 45)={d['b45']}",
    f"OP sent (sendcan): HMS {d['hms_sent']}; ACC_06 Anhalten {d['a06_stp']}, Anfahren {d['a06_go']}, accel "
    f"{d['a06_acc']}, status {d['a06_st']}, neg jerk {d['a06_jneg']}; ACC_07 Anhalten {d['a07_stp']}, Anfahren "
    f"{d['a07_go']}, Anhalteweg {d['a07_dist']}",
    f"HMS per source (never mixed): {srcs or 'no ACC_07 in the car CAN log'}",
    f"TSK_06: status {d['tsk']}, wheel brake torque raw {d['brm_raw']} = {d['brm_nm']} Nm, Freig_Verzoeg_Anf "
    f"{d['freig30']}, Standby_Anf_ESP {d['stby28']}, SRBM_Anf {d['srbm']}",
    f"ESP_05: brake pressure {d['bar']} bar, Verz_TSK_aktiv {d['verz27']}, Konsistenz_TSK {d['kons29']}, "
    f"reverse flag {d['rev39']}, Anfahrhilfe {d['anfh40']}, driver brakes {d['drvbrk26']}",
    f"Car: grade {d['grade']} %, mass {d['mass']} kg, engine torque {d['mo_nm']} Nm, gas pedal {d['pedal']} %, "
    f"gear code {d['gear']}, speed {d['v']} m/s, pitch {d['pitch']}",
    f"openpilot: long state {d['lcs']} (controlsState {d['lcs_cs']}), shouldStop {d['ss']}, aTarget {d['aT']}, "
    f"plan speed {d['vplan0']}, long control {d['la']}, brake pedal {d['brk']}, gas pedal {d['gas']}"
    + (f"; panda controlsAllowed {d['ctrl_ok']}, long {d['ctrl_long']}, safetyTxBlocked {d['txblk']}" if 'txblk' in d else ''),
  ]
  return lines


def snapshot_compact(snap, keys):
  d = dict(snap)
  return ' '.join(f'{k}={d[k]}' for k in keys if k in d)


SNAP_PASTE_1 = ['b53', 'b63', 'b28', 'b29', 'b38', 'b45', 'hms_sent']
SNAP_PASTE_2 = ['a06_stp', 'a06_go', 'a06_acc', 'a06_st', 'a06_jneg', 'a07_stp', 'a07_go', 'a07_dist']
SNAP_PASTE_3 = ['tsk', 'brm_raw', 'brm_nm', 'freig30', 'stby28', 'bar', 'verz27', 'kons29', 'rev39', 'anfh40']
SNAP_PASTE_4 = ['grade', 'mass', 'mo_nm', 'lcs', 'lcs_cs', 'ss', 'aT', 'vplan0', 'la', 'brk', 'gas', 'ctrl_ok', 'txblk']


# ============================================================================================ decisions
def drive_bit53_line(D):
  known = [s for s in D.stops if s.esp_ok]
  conf = [s for s in known if s.hold_on is not None]
  return f'bit 53 was set on {len(conf)} of {len(known)} stops in this drive' if known else ''


def decisions(D, st):
  """The study's decision rules, computed for one stop, in plain English. -> [(tag, text)]"""
  out = []
  m = st.motion
  on = st.onset
  moved = m is not None and (m['kind'] in MOVE_KINDS or m.get('slip') or m.get('launch'))
  rolled = is_unasked_roll(st)
  t0 = st.t0
  # ---- rule 1
  if not st.esp_ok:
    out.append(('R1', 'Hold confirmation (bit 53): not enough car CAN data around this stop to tell.'))
  elif st.hold_on is None:
    txt = ('Hold confirmation (bit 53) was never set during this stop -> the ESP hold never formed '
           f'(H1/H4/H5 path; {drive_bit53_line(D)}).')
    if st.op_ok and st.hms1_s > 0:
      txt += f' OP asked for a hold (HMS 1) for {st.hms1_s:.2f} s'
      if st.hms1_still_s is not None:
        txt += f', {st.hms1_still_s:.2f} s of it with all four wheels at 0'
      if st.latch_t is not None:
        txt += f', then switched to HMS 4 at {rel((st.latch_t - t0) / NS)}'
        if st.hms1_still_s is not None and st.hms1_still_s < 1.0 and rolled:
          txt += ' -> consistent with H1 (the hold-request window ran out around the moment the wheels stopped).'
        elif st.hms1_still_s is not None and st.hms1_still_s < 1.0:
          txt += (' -> the H1 pattern (the hold-request window ran out around the moment the wheels stopped) is '
                  'present, but this stop did not roll, so something else held the car (see the TSK torque line).')
        elif st.hms1_still_s is not None:
          txt += ' -> HMS 1 ran 1 s or more at standstill and the ESP still did not confirm: H1\'s short-window ' \
                 'explanation does not fit.'
        else:
          txt += '.'
      else:
        txt += '.'
    out.append(('R1', txt))
  else:
    txt = f'Hold confirmation (bit 53) set at {rel((st.hold_on - t0) / NS)}'
    if st.hold_off is not None:
      txt += f', dropped at {rel((st.hold_off - t0) / NS)} after {(st.hold_off - st.hold_on) / NS:.2f} s'
    else:
      txt += ', stayed on'
    txt += ' -> the ESP hold did form on this stop (H1\'s "never forms" path is out for this stop).'
    out.append(('R1', txt))
  # ---- rule 2: at the onset
  if moved and st.esp_ok and m.get('requested') and m.get('kind') != 'drive':
    out.append(('R2', f'{who_asked(m)} had asked to go before the wheels moved, so the ESP releasing its hold was '
                      'expected; the roll happened during the launch (engine/launch too slow), not while standing '
                      '(not H1/H2).'))
  elif moved and st.esp_ok and m.get('kind') != 'drive':
    if isnum(st.hold_at_onset) and st.hold_at_onset >= 0.5:
      out.append(('R2', f'The ESP still confirmed the hold (bit 53 = 1) when the wheels started to turn, brake '
                        f'pressure {fnum(st.bar_stand, "{:.1f}")} bar -> the car moved through the ESP hold: '
                        'consistent with H2 (hold pressure not enough).'))
    elif st.drop:
      dr = st.drop
      dt_drop = (dr['t'] - t0) / NS
      hold_len = ''
      if st.hold_on is not None and dr['what'].startswith('bit 53'):
        hold_len = f'; the hold lasted {(dr["t"] - st.hold_on) / NS:.2f} s'
      if dr['kind'] == 'H1':
        txt = (f'OP changed {", ".join(x[1] for x in dr["same"])} {dr["frames"]} frame(s) ({dr["steady_s"]:.2f} s) before {dr["what"]} '
               f'dropped at {rel(dt_drop)} -> OP-triggered release (consistent with H1).')
      elif dr['kind'] == 'H2':
        why = ('no OP change in the 1.5 s before' if dr['steady_full'] else
               f'last OP change {dr["last"][1]} at {rel((dr["last"][0] - t0) / NS)}')
        txt = (f'{dr["what"][0].upper() + dr["what"][1:]} dropped at {rel(dt_drop)} with OP steady for '
               f'{dr["steady_s"]:.2f} s before ({why}) -> consistent with an ESP time/pressure limit (H2){hold_len}.')
      else:
        txt = (f'OP last changed {dr["last"][1]} {dr["steady_s"]:.2f} s before {dr["what"]} dropped at '
               f'{rel(dt_drop)}: between the H1 (0.10 s or less) and H2 (0.20 s or more) patterns.')
      if dr['after']:
        a0 = dr['after'][0]
        txt += f' OP then sent {a0[1]} {(a0[0] - dr["t"]) / NS:+.2f} s after the drop.'
      txt += f' The wheels started to move {(on - dr["t"]) / NS:.2f} s after the drop.'
      out.append(('R2', txt))
    elif st.hold_on is None:
      out.append(('R2', 'There was no ESP hold to drop when the wheels moved (bit 53 never set; see R1).'))
  elif moved and not st.esp_ok:
    out.append(('R2', 'Not enough car CAN data at the onset to check the ESP side.'))
  # H3
  if st.esp_ok and len(D.c['nv']):
    if st.nv_pre or st.nv_at_stop:
      first = D.nv_first
      when = f' (first set {clock((first - D.t_route0) / NS)} into the drive, {(t0 - first) / NS:.1f} s before this stop)' \
        if first is not None else ''
      if rolled:
        out.append(('R2', f'Bit 29 (ESP gave up standstill hold) was already set before this stop{when} -> consistent with H3.'))
      else:
        out.append(('R2', f'Bit 29 (ESP gave up standstill hold) was already set before this stop{when}, but this stop '
                          'did not roll (the H3 condition was present without causing a roll here).'))
    else:
      txt = 'Bit 29 (ESP gave up) was not set when the car stopped'
      if st.nv_on is not None:
        rel_on = (st.nv_on - t0) / NS
        txt += f' (it came on at {rel(rel_on)}' + (', after the wheels started to move)' if on is not None and st.nv_on > on else ')')
      out.append(('R2', txt + ' -> H3 not supported.'))
  # H4: torque
  if not D.has_tsk:
    out.append(('R2', 'TSK_06 is not in the log: the TSK wheel brake torque (H4/H5) cannot be checked.'))
  elif not len(D.c['tsk_brake_raw']) or not st.brm_ok:
    out.append(('R2', 'TSK_Radbremsmom not available around this stop: H4/H5 cannot be checked.'))
  elif st.brm_stand is not None:
    e = st.brm_exp
    raw = st.brm_stand
    txt = f'TSK wheel brake torque at standstill ({st.brm_note}) = {raw * 8:.0f} Nm (raw {raw:.0f})'
    if isnum(st.grade):
      txt += f' on {st.grade:+.1f}%'
    if e:
      txt += (f' with OP asking {e["a_req"]:+.2f} m/s2; expected raw {e["comp"]:.0f} if the TSK grade-compensates, '
              f'raw {e["uncomp"]:.0f} if not; holding needs raw {e["need"]:.0f} ({e["mass"]:.0f} kg from '
              f'{st.mass_src.split(" (")[0]}, wheel radius {WHEEL_R_M} m)')
    others = [f'{lab} raw {st.brm_parts[k]:.0f}' for k, lab in (('pre_nohold', 'no ESP hold'), ('pre_hold', 'ESP holding'),
                                                                ('post', 'after the HMS 4 latch'))
              if k in st.brm_parts and k != st.brm_key]
    if others:
      txt += '; other phases: ' + ', '.join(others)
    if on is not None and isnum(st.brm_onset):
      txt += f'; raw {st.brm_onset:.0f} at the onset'
    cl = st.brm_class
    if cl == 'comp' and e and e['comp'] < e['need']:
      txt += ' -> matches grade-compensated: the request is too weak here once the ESP hold is gone (consistent with H4).'
    elif cl == 'comp':
      txt += ' -> matches grade-compensated, and that is enough to hold on this grade (H4 predicts this stop holds).'
    elif cl == 'uncomp':
      txt += ' -> matches NOT grade-compensated (same on every grade): H4 not supported; look at brake pressure and slip.'
    elif cl == 'zero' and st.brm_key == 'pre_hold':
      txt += ' -> about 0 while the ESP was holding: the ESP, not the TSK, held the car here (H4 cannot be judged from it).'
    elif cl == 'zero' and st.brm_key == 'post':
      txt += ' -> about 0 after the latch: the TSK asked for no brake torque (see the H5 line).'
    elif cl == 'zero':
      txt += ' -> about 0: the TSK asked for no brake torque even before any HMS 4 latch.'
    elif cl == 'flat':
      txt += ' -> on flat ground both models give the same number, so this stop cannot separate them.'
    elif cl == 'other':
      txt += ' -> fits neither model; compare the numbers by hand.'
    else:
      txt += '.'
    out.append(('R2', txt))
  elif D.has_tsk and st.brm_ok:
    out.append(('R2', 'No standstill frames where OP still asked to stand (it asked to go or switched off at once): '
                      'H4 cannot be checked on this stop.'))
  # H5
  if D.has_tsk and st.brm_ok:
    if st.latch_t is None:
      out.append(('R2', 'OP never latched HMS 4 while it still asked to stand here (an HMS 4 at a launch does not count) '
                        '-> H5 (nobody owns the hold after the latch) does not apply to this stop.'))
    else:
      h = st.h5
      lt = rel((st.latch_t - t0) / NS)
      a06 = ''
      if isnum(h['a06_stop']):
        a06 = f' while ACC_06 said Anhalten={int(h["a06_stop"])}, accel {fnum(h["a06_acc"], "{:+.2f}")}'
      late = ''
      if h.get('zero_late') is not None:
        late = (f'; it fell to 0 only at {rel((h["zero_late"] - t0) / NS)}, after {h["win_why"]}, which does not '
                'count for H5')
      if not h['judge']:
        out.append(('R2', f'HMS->4 at {lt}, but {h["win_why"]} within 0.1 s of it: the TSK torque cannot be judged '
                          'for H5 on this stop.'))
      elif h['zero_t'] is not None and h['stays']:
        out.append(('R2', f'TSK wheel brake torque fell to 0 within {h["frames"]} frames of HMS->4 at {lt} (raw '
                          f'{fnum(h["pre"], "{:.0f}")} before){a06}, while OP was still in control and before the wheels '
                          'moved, and stayed there -> nobody owns the hold (consistent with H5).'))
      elif h['zero_t'] is not None:
        out.append(('R2', f'TSK wheel brake torque touched 0 {h["frames"]} frames after HMS->4 at {lt} but did not stay '
                          f'there (raw {fnum(h["after"], "{:.0f}")} after) -> H5 only partly.'))
      else:
        out.append(('R2', f'After HMS->4 at {lt} the TSK wheel brake torque stayed at raw {fnum(h["after"], "{:.0f}")} '
                          f'(raw {fnum(h["pre"], "{:.0f}")} before){a06} until {h["win_why"]} at '
                          f'{rel((h["win_end"] - t0) / NS)}{late} -> H5 not supported.'))
  # ---- rule 3: H8
  if st.op_ok:
    if st.h8:
      out.append(('R3', f'{st.h8} frame(s) with OP in pid and vEgo under 0.5 m/s sent HMS 4/Anfahren before the car '
                        f'stopped ({st.h8_pid_slow} pid frames under 0.5 m/s in all) -> consistent with H8.'))
    else:
      out.append(('R3', f'No pid frame under 0.5 m/s sent HMS 4/Anfahren before the car stopped '
                        f'({st.h8_pid_slow} pid frames under 0.5 m/s) -> H8 not seen on this stop.'))
  else:
    out.append(('R3', 'Not enough sendcan data to count H8 frames.'))
  # ---- rule 4: H6
  if moved and m['kind'] != 'drive':
    if st.h6:
      parts = []
      for band, label in (('<1.0', 'under 1.0 m/s'), ('>=1.0', 'at 1.0 m/s or more')):
        n, hs, ls, h4s = st.h6[band]
        if n:
          tot = sum(ls.values()) or 1
          parts.append(f'{label} OP sent ' + ', '.join(f'HMS {h} x{k}' for h, k in sorted(hs.items())) +
                       (f' ({h4s} of the HMS 4 frames with ACC_06 still saying Anhalten)' if hs.get(4) else '') +
                       '; long state ' + (', '.join(f'{lname} {100 * k // tot}%' for lname, k in sorted(ls.items())) or '-'))
      txt = 'While the wheels turned (onset to +5 s): ' + ('; '.join(parts) if parts else 'no ACC_07 frames')
      lo = st.h6['<1.0'][3]
      hi1 = st.h6['>=1.0'][1].get(1, 0)
      verdict = []
      if m.get('requested'):
        verdict.append(f'{who_asked(m)} had asked to go, so HMS 4 here is the launch itself (H6 is about rolls while OP '
                       'still wants to stand)')
      else:
        if lo:
          verdict.append('OP kept telling the ESP to drive off (HMS 4) while the car rolled and OP still asked to stand '
                         '(consistent with H6 as an amplifier)')
        if hi1:
          verdict.append('it asked for a hold again at 1.0 m/s or more (H6 roll-catch-release loop)')
      if st.h6_lcs_flips >= FLIP_MIN:
        verdict.append(f'OP\'s long state flipped {st.h6_lcs_flips} times in this window (pid/stopping chatter, trap 4)')
      out.append(('R4', txt + (' -> ' + '; '.join(verdict) + '.' if verdict else ' -> no H6 pattern.')))
    elif st.op_ok:
      out.append(('R4', 'OP sent no ACC_07 frames while the wheels turned.'))
  # ---- H9
  if m is not None and m.get('op_off') and category(st) == 'op_off_roll':
    toff, cause = m['op_off']
    out.append(('R9', f'OP long control was switched off at {rel((toff - t0) / NS)} ({cause}); the wheels moved at '
                      f'{rel((on - t0) / NS)} -> rolled after OP was switched off (consistent with H9, not an OP-hold failure).'))
  elif m is not None and m.get('op_off'):
    toff, cause = m['op_off']
    out.append(('R9', f'OP long control was switched off at {rel((toff - t0) / NS)} ({cause}) and the car then drove '
                      'off as asked -> not a roll, so H9 does not apply here.'))
  else:
    before = [(t, n) for t, n in st.h9 if on is None or t <= on]
    if before:
      out.append(('R9', 'Before the wheels moved: ' + ', '.join(f'{n} {rel((t - t0) / NS)}' for t, n in before[:4]) +
                  ' -> pedal/disengage interaction (H9) is possible here.'))
    else:
      txt = 'No pedal press and no OP switch-off ' + ('before the wheels moved' if on is not None else 'during this stop')
      txt += ' -> H9 not supported'
      if st.pedal_after:
        txt += ' (' + ', '.join(f'{n} {rel((t - t0) / NS)}' for t, n in st.pedal_after) + ', after the roll began)'
      out.append(('R9', txt + '.'))
  return out


# ============================================================================================ narrative
def key_moments(D, a, b, t_ref):
  """Plain-English list of what changed in [a, b]. Rapid back-and-forth changes are summarised."""
  np = D.np
  ev = []

  def add(t, text, prio=5):
    ev.append((t, prio, text))

  thin = []

  def track(sig, fmt_change, prio=5, flip_name=None, valfmt=None, can=False):
    if not len(sig):
      return
    if can and sig.rate(a, b) < FULL_RATE_HZ:
      thin.append(flip_name or 'CAN')
      return
    ch = sig.changes(a, b)
    groups, cur = [], []
    for c in ch:
      if cur and c[0] - cur[-1][0] > FLIP_GAP_S * NS:
        groups.append(cur)
        cur = []
      cur.append(c)
    if cur:
      groups.append(cur)
    for g in groups:
      if len(g) >= FLIP_MIN and flip_name:
        vals = sorted({int(x[2]) for x in g if isnum(x[2])} | {int(x[1]) for x in g if isnum(x[1])})
        vf = valfmt or (lambda v: str(v))
        dur = (g[-1][0] - g[0][0]) / NS
        mx = max_per_second(np, [x[0] for x in g])
        add(g[0][0], f'{flip_name} FLIPPING between {"/".join(vf(v) for v in vals)}: {len(g)} changes in '
                     f'{dur:.2f} s (up to {mx} per second), ended at {vf(int(g[-1][2])) if isnum(g[-1][2]) else "-"}',
            prio)
      else:
        for t, old, new in g:
          s = fmt_change(old, new)
          if s:
            add(t, s, prio)

  c, s = D.c, D.s

  def onoff(on, off):
    return lambda o, n: (on if n >= 0.5 else off) if isnum(n) else None

  track(s['longActive'], onoff('OP longitudinal control ON', 'OP longitudinal control OFF'), 3)
  track(s['lcs'], lambda o, n: f'OP long state -> {lcs_name(n)}' if isnum(n) else None, 6,
        flip_name='OP long state', valfmt=lambda v: LCS_NAMES[v] if 0 <= v < 4 else str(v))
  track(s['shouldStop'], onoff('planner: shouldStop ON (wants to stay stopped)', 'planner: shouldStop OFF (wants to go)'), 6,
        flip_name='planner shouldStop')
  a06 = c['acc06_accel']
  if len(a06) and a06.rate(a, b) >= FULL_RATE_HZ:
    sgn = Sig(np, a06.t, np.where(np.isnan(a06.v), np.nan, np.where(a06.v > 0.05, 1.0, 0.0)))
    ch = sgn.changes(a, b)
    if len(ch) >= FLIP_MIN * 3:
      add(ch[0][0], f'OP accel request sign FLIPPING: {len(ch)} changes', 6)
    else:
      for t, _old, n in ch:
        if isnum(n):
          val = a06.at1(t)
          add(t, (f'OP accel request turned POSITIVE ({val:+.2f} m/s2)' if n >= 0.5 else
                  f'OP accel request back to zero/braking ({val:+.2f} m/s2)') if isnum(val) else
              'OP accel request: no value (OP not in control)', 6)
  track(c['acc06_stop'], onoff('OP sent ACC_06 Anhalten=1 (stop request to the TSK)', 'OP sent ACC_06 Anhalten=0'), 6,
        flip_name='ACC_06 Anhalten', can=True)
  track(c['acc06_go'], onoff('OP sent ACC_06 Anfahren=1 (start request to the TSK)', 'OP sent ACC_06 Anfahren=0'), 6,
        flip_name='ACC_06 Anfahren', can=True)
  track(c['acc07_stop'], onoff('OP sent ACC_07 Anhalten=1 (stop request to the ESP)', 'OP sent ACC_07 Anhalten=0'), 6,
        flip_name='ACC_07 Anhalten', can=True)
  track(c['hms'], lambda o, n: (f'OP HMS -> {int(n)} ({HMS_TEXT.get(int(n), "?")})' if isnum(n) else None), 4,
        flip_name='OP HMS', valfmt=lambda v: str(v), can=True)
  track(c['esp_acc_stop'], onoff('ESP: ACC stopping procedure active (bit 63) ON', 'ESP: ACC stopping procedure (bit 63) OFF'),
        4, flip_name='ESP bit 63', can=True)
  track(c['anf'], onoff('ESP: SRBM Anf ON (ESP standstill hold active, bit 28)', 'ESP: SRBM Anf OFF (bit 28)'), 3,
        flip_name='ESP SRBM Anf', can=True)
  track(c['hold'], onoff('ESP: hold CONFIRMED (Haltebestaetigung, bit 53 = 1)', 'ESP: hold confirmation DROPPED (bit 53 = 0)'), 2,
        flip_name='ESP hold confirmation', can=True)
  track(c['nv'], onoff('ESP: nicht_verfuegbar ON (ESP has given up standstill hold, bit 29)', 'ESP: nicht_verfuegbar cleared'), 2,
        flip_name='ESP nicht_verfuegbar', can=True)
  track(c['kons_acc'], onoff('ESP: consistency ACC (bit 38) ON', 'ESP: consistency ACC (bit 38) OFF'), 4, can=True)
  track(c['kons_tsk33'], onoff('ESP: consistency TSK (ESP_33 bit 45) ON', 'ESP: consistency TSK (bit 45) OFF'), 4, can=True)
  track(c['kons_tsk05'], onoff('ESP_05: Konsistenz_TSK (bit 29) ON', 'ESP_05: Konsistenz_TSK OFF'), 4, can=True)
  track(c['tsk_freig'], onoff('TSK: deceleration request released (Freig_Verzoeg_Anf=1)', 'TSK: Freig_Verzoeg_Anf=0'), 5, can=True)
  track(c['verz_tsk'], onoff('ESP_05: TSK deceleration active (bit 27) ON', 'ESP_05: TSK deceleration active (bit 27) OFF'), 5, can=True)
  brm = c['tsk_brake_raw']
  if len(brm) and brm.rate(a, b) >= FULL_RATE_HZ:
    z = Sig(np, brm.t, np.where(np.isnan(brm.v), np.nan, (brm.v <= H5_ZERO_RAW).astype(np.float64)))
    for t, o, n in z.changes(a, b):
      if isnum(n) and isnum(o):
        if n >= 0.5:
          add(t, f'TSK wheel brake torque fell to 0 (was raw {fnum(brm.at1(t - int(0.03 * NS)), "{:.0f}")})', 3)
        else:
          add(t, f'TSK wheel brake torque back up (raw {fnum(brm.at1(t), "{:.0f}")} = {fnum(brm.at1(t) * 8, "{:.0f}")} Nm)', 4)
  track(c['tsk'], lambda o, n: (f'TSK status {int(o) if isnum(o) else "-"} -> {int(n)} ({TSK_TEXT.get(int(n), "?")})'
                                if isnum(n) else None), 1, flip_name='TSK status', can=True)
  track(s['brakePressed'], onoff('driver BRAKE pedal pressed', 'driver brake released'), 3, flip_name='driver brake')
  track(s['gasPressed'], onoff('driver GAS pedal pressed', 'driver gas released'), 3, flip_name='driver gas')
  track(s['gear'], lambda o, n: f'gear -> {GEARS[int(n)]}' if isnum(n) else None, 4)
  track(s['parkingBrake'], onoff('handbrake ON', 'handbrake OFF'), 4)
  if D.has_panda:
    track(s['controlsAllowed'], onoff('panda: controls allowed', 'panda: controls NOT allowed'), 3)
    tb = s['safetyTxBlocked']
    for t, o, n in tb.changes(a, b):
      if isnum(o) and isnum(n) and n > o:
        add(t, f'panda blocked {int(n - o)} of OP\'s frames (safetyTxBlocked +{int(n - o)})', 2)
  if len(D.dir) and D.dir.rate(a, b) >= FULL_RATE_HZ:
    sv = vsig(D)
    dt, dv = D.dir.sl(a, b)
    if len(dt):
      vm = sv.at(dt)
      mv = np.where(vm >= 0.05, dv, 0.0)
      last = float(mv[0])
      for t, x in zip(dt.tolist(), mv.tolist()):
        if x != 0 and x != last:
          add(t, 'wheels turning BACKWARD (ESP_10)' if x < 0 else 'wheels turning forward (ESP_10)', 2)
        if x != 0:
          last = x
  track(c['esp_reverse'], onoff('ESP: reverse travel detected (ESP_05 bit 39)', 'ESP: reverse travel flag cleared'), 4, can=True)
  sv = s['vEgo']
  vt, vv = sv.sl(a, b)
  if len(vt):
    state = None
    for t, v in zip(vt.tolist(), vv.tolist()):
      ns = 'still' if v < STILL_V else ('slow' if v <= ROLL_V else 'moving')
      if state is not None and ns != state:
        if ns == 'still':
          add(t, 'car STOPPED (speed under 0.1 m/s)', 1)
        elif state == 'still':
          add(t, f'car started MOVING (speed {v:.2f} m/s)', 1)
        elif ns == 'moving' and state == 'slow':
          add(t, 'speed passed 0.3 m/s', 5)
      state = ns
  if len(D.wz):
    for t, o, n in D.wz.changes(a, b):
      if isnum(n) and isnum(o) and n >= 0.5:
        add(t, 'all four wheel speeds read 0 (ESP_19)', 2)
  for t, d in D.send_gaps:
    if a <= t <= b:
      add(t, f'OP sent NO ACC_06/ACC_07 for {d / 1e6:.0f} ms (normally every 20 ms)', 2)
  for msg, ft in D.foreign.items():
    ff = ft[(ft >= a) & (ft <= b)]
    if len(ff):
      add(int(ff[0]), f'{plural(len(ff), msg + " frame")} from ANOTHER sender on the car bus in this window', 1)
  for msg, (_, rej) in D.echo.items():
    rr = rej[(rej >= a) & (rej <= b)]
    if len(rr):
      add(int(rr[0]), f'panda REFUSED {plural(len(rr), "frame")} of OP\'s {msg} (safety limit) in this window', 1)
  prev_sd = None
  for t, state, atype, t1, t2 in D.sd:
    if a <= t <= b:
      bits = []
      if prev_sd is None or state != prev_sd[0]:
        bits.append(f'openpilot state -> {state}')
      if t1 or t2:
        if prev_sd is None or (t1, t2) != (prev_sd[2], prev_sd[3]):
          bits.append(f'alert: "{t1}{(" / " + t2) if t2 else ""}"' + (f' ({atype})' if atype else ''))
      elif atype and (prev_sd is None or atype != prev_sd[1]):
        bits.append(f'alert: {atype}')
      elif prev_sd is not None and (prev_sd[2] or prev_sd[3] or prev_sd[1]) and not atype:
        bits.append('alert cleared')
      if bits:
        add(t, ' | '.join(bits), 3)
    prev_sd = (state, atype, t1, t2)
  prev = None
  for t, names in D.oe:
    if t < a:
      prev = names
      continue
    if t > b:
      break
    new = [n for n in names if prev is None or n not in prev]
    if new:
      add(t, 'events: ' + ', '.join(new), 4)
    prev = names
  if thin:
    add(a, 'ESP/HMS changes are not listed here: too few CAN frames in this window (qlog only?)', 0)
  ev.sort(key=lambda x: (x[0], x[1]))
  return [(t, txt) for t, _, txt in ev]


# ============================================================================================ timeline
def legend_lines():
  return [
    'Timeline columns: t = seconds from the moment all four wheel speeds read 0 (- before)   v = wheel speed m/s',
    '  pul = wheel pulses since +0.00 (ESP_10, median of 4 wheels; "-" = counters not usable on this drive)',
    '  LCS = OP long state (off/pid/stop/strt)   sS = planner shouldStop   OPacc = accel OP sent (ACC_06)',
    '  A6 = ACC_06 bits sent: S Anhalten, G Anfahren, B both, . none   HMS = hold request sent (ACC_07, sendcan):',
    '  0 none, 1 HOLD, 3 standby, 4 release/launch; "4*6" = changed 6 times inside this row   r2 = radar src 2 HMS',
    '  53 = ESP hold confirmed   63 = ESP ACC stopping active   28 = ESP hold active (SRBM Anf)   29 = ESP gave up',
    '  TSK = cruise status, 6/7 = FAULT   Brm = TSK wheel brake torque raw (x8 Nm)   bar = ESP brake pressure',
    '  B/G = driver brake/gas   dir = wheels F forward, R BACKWARD ("?" = direction not validated on this drive,',
    '  "r" = ESP reverse flag)   "1" = on, "." = off, "-" = no data',
  ]


TL_HEAD = (f"{'t':>6} {'v':>5} {'pul':>4} {'LCS':>4} {'sS':>2} {'OPacc':>5} {'A6':>2} {'HMS':>4} {'r2':>2} "
           f"{'53':>2} {'63':>2} {'28':>2} {'29':>2} {'TSK':>3} {'Brm':>4} {'bar':>5} B G {'dir':>3}")


def timeline(D, t_ref, a, b, hz):
  np = D.np
  step = int(NS / hz)
  T = np.arange(a, b + 1, step, dtype=np.int64)
  if not len(T):
    return []
  c, s = D.c, D.s
  r2sig = D.acc_src.get(D.radar_srcs[0], {}).get('hms') if D.radar_srcs else None
  col = {
    'v': vsig(D).at(T), 'lcs': s['lcs'].at(T), 'la': s['longActive'].at(T),
    'ss': s['shouldStop'].at(T), 'acc': c['acc06_accel'].at(T), 'stp': c['acc06_stop'].at(T), 'go': c['acc06_go'].at(T),
    'hms': c['hms'].at(T), 'r2': r2sig.at(T) if r2sig is not None else np.full(len(T), np.nan),
    'h53': c['hold'].at(T), 'h63': c['esp_acc_stop'].at(T), 'anf': c['anf'].at(T), 'nv': c['nv'].at(T),
    'tsk': c['tsk'].at(T), 'brm': c['tsk_brake_raw'].at(T), 'bar': c['brake_bar'].at(T),
    'brk': s['brakePressed'].at(T), 'gas': s['gasPressed'].at(T), 'dir': D.dir.at(T), 'dirraw': D.dir_raw.at(T),
    'rev': c['esp_reverse'].at(T),
  }
  if D.pulse_ok:
    p0 = D.pul.at1(t_ref)
    col['pul'] = D.pul.at(T) - (p0 if isnum(p0) else 0.0)
  else:
    col['pul'] = np.full(len(T), np.nan)
  if not len(c['acc06_accel']):
    col['acc'] = np.where(col['la'] >= 0.5, s['ccAccel'].at(T), np.nan)
  hms_t = np.asarray(hms_changes(D, a - step, b), dtype=np.int64)
  nchg = np.searchsorted(hms_t, T, side='right') - np.searchsorted(hms_t, T - step, side='right')
  lines = []
  for i, t in enumerate(T.tolist()):
    hv = col['hms'][i]
    h = '-' if not isnum(hv) else str(int(hv))
    if nchg[i] > 1 or (nchg[i] == 1 and hz < 50):
      h += f'*{int(nchg[i])}'
    v = col['v'][i]
    vmov = isnum(v) and v >= 0.05
    if D.dir_valid:
      d = col['dir'][i]
      dtxt = '-' if not isnum(d) else ('R' if d < 0 and vmov else ('F' if d > 0 and vmov else '.'))
    elif isnum(col['rev'][i]) and col['rev'][i] >= 0.5:
      dtxt = 'r'
    elif len(D.dir_raw):
      dtxt = '?'
    else:
      dtxt = '-'
    st_, go_ = col['stp'][i], col['go'][i]
    a6 = '-' if not (isnum(st_) and isnum(go_)) else ('B' if st_ >= 0.5 and go_ >= 0.5 else
                                                      ('S' if st_ >= 0.5 else ('G' if go_ >= 0.5 else '.')))
    lc = col['lcs'][i]
    tsk = col['tsk'][i]
    lines.append(
      f"{(t - t_ref) / NS:+6.2f} {fnum(v, '{:.2f}', 5)} {fnum(col['pul'][i], '{:.0f}', 4)} "
      f"{(LCS_SHORT.get(int(lc), '?') if isnum(lc) else '-'):>4} {fbool(col['ss'][i]):>2} "
      f"{fnum(col['acc'][i], '{:+.2f}', 5)} {a6:>2} {h:>4} {fnum(col['r2'][i], '{:.0f}', 2)} "
      f"{fbool(col['h53'][i]):>2} {fbool(col['h63'][i]):>2} {fbool(col['anf'][i]):>2} {fbool(col['nv'][i]):>2} "
      f"{(str(int(tsk)) + ('!' if tsk >= 6 else ' ')) if isnum(tsk) else '- ':>3} "
      f"{fnum(col['brm'][i], '{:.0f}', 4)} {fnum(col['bar'][i], '{:.1f}', 5)} "
      f"{fbool(col['brk'][i])} {fbool(col['gas'][i])} {dtxt:>3}")
  return lines


def print_timeline(D, t_ref, a, b, hz, marks):
  lines = timeline(D, t_ref, a, b, hz)
  step = NS / hz
  mk = {}
  for t, txt in marks:
    k = int(round((t - a) / step + 0.4999))
    mk.setdefault(k, txt)
  for i, ln in enumerate(lines):
    if i % 30 == 0:
      say(TL_HEAD)
    say(ln + (f'  <{mk[i]}' if i in mk else ''))


def short_marks(D, st):
  """Tags shown at the right edge of the timeline."""
  out = [(st.t0, 'WHEELS 0')]
  if st.t_lcs_stop is not None:
    out.append((st.t_lcs_stop, 'lcs->stopping'))
  if st.latch_t is not None:
    out.append((st.latch_t, 'HMS 4'))
  if st.hold_on is not None:
    out.append((st.hold_on, 'hold on'))
  if st.hold_off is not None:
    out.append((st.hold_off, 'hold OFF'))
  if st.anf_off is not None:
    out.append((st.anf_off, 'Anf off'))
  if st.nv_on is not None:
    out.append((st.nv_on, 'nv ON'))
  if st.tsk_fault:
    out.append((st.tsk_fault[0], f'TSK {st.tsk_fault[1]}'))
  m = st.motion
  if m:
    out.append((st.onset, 'ONSET'))
    if m.get('brake_t') is not None:
      out.append((m['brake_t'], 'brake'))
    if m.get('op_off'):
      out.append((m['op_off'][0], 'OP off'))
  out.sort(key=lambda x: x[0])
  return out


# ============================================================================================ result text
def who_asked(m):
  if m['gas'] and not m['op_req']:
    return 'driver gas'
  if m['op_req']:
    return 'OP launch'
  if m['reverse_gear']:
    return 'reverse gear'
  return 'requested'


def stop_result(D, st, short=False):
  pre = '[log starts mid-stop] ' if st.cut_start else ''
  return pre + _stop_result(D, st, short)


def _stop_result(D, st, short=False):
  m = st.motion
  extra = []
  if m is None:
    base = 'held, still stopped at end of log'
    if st.op_off is not None:
      base = f'stopped; OP turned off {rel((st.op_off - st.t0) / NS)}'
    if st.early_moves:
      p = sum(c[2] for c in st.early_moves)
      extra.append(f'wheels ticked {p:.0f} pulse(s) while stopped')
  else:
    k = m['kind']
    dist = m.get('dist', 0.0)
    if k == 'back':
      base = f'ROLLED BACK {dist:.2f} m'
      if m.get('launch_roll'):
        base = (f'ROLLED BACK {dist * 100:.0f} cm on {"OP launch" if m["op_req"] else "a " + who_asked(m) + " launch"} (more than a launch slip on '
                f'{fnum(st.grade, "{:+.1f}%")}), then drove off')
      elif m.get('launch'):
        base += f' after {"OP" if m["op_req"] else "the driver"} asked to go ({who_asked(m)})'
        if m.get('went_fwd'):
          base += ', then drove off'
      extra.append(f'peak {m["peak"]:.2f} m/s')
    elif k == 'unk':
      base = f'moved {dist:.2f} m, direction unknown, possible rollback'
      if m['requested']:
        base = f'moved {dist:.2f} m after {who_asked(m)} asked to go, direction unknown, possible rollback'
      if m.get('unk_note'):
        extra.append(m['unk_note'])
      extra.append(f'peak {m["peak"]:.2f} m/s')
    elif k == 'fwd':
      base = f'ROLLED FORWARD {dist:.2f} m (nothing asked for it)'
      extra.append(f'peak {m["peak"]:.2f} m/s')
    elif k == 'roll?':
      g = m.get('guess')
      base = f'ROLLED {dist:.2f} m, no gas & no OP request' + (f' (probably {g}; no direction signal)' if g else ' (direction unknown)')
      extra.append(f'peak {m["peak"]:.2f} m/s')
    elif k == 'creep':
      dname = {-1: ' back', 1: ' forward'}.get(m['dir'], '')
      base = f'moved{dname} {dist * 100:.0f} cm slowly (<0.3 m/s), nothing asked for it'
    elif k == 'reverse':
      base = 'driver reversed (R gear)'
    else:
      who = who_asked(m)
      base = f'held, then drove off ({who})'
      if m.get('slip'):
        _, dist_s, pk = m['slip']
        base = f'held, then {who}: slipped back {dist_s * 100:.0f} cm first (launch slip), then drove off'
    if k in MOVE_KINDS and not short:
      if m.get('brake_t') is not None:
        extra.append(f'driver braked {rel((m["brake_t"] - st.t0) / NS)}')
      elif m.get('restopped'):
        extra.append('stopped again by itself')
    if k in MOVE_KINDS and m.get('op_off'):
      toff, cause = m['op_off']
      base += f' - rolled after OP was switched off ({cause} at {rel((toff - st.t0) / NS)})'
    if m.get('creep_then') and not short:
      tl, who = m['creep_then']
      extra.append(f'stood still again, later drove off {rel((tl - st.t0) / NS)} ({who})')
  if short:
    return base
  return base + (f' ({", ".join(extra)})' if extra else '')


CAT_TEXT = [('held', 'held without rolling'), ('rolled_back', 'ROLLED BACK'),
            ('possible_roll', 'POSSIBLE ROLLBACK (moved, direction unknown)'), ('rolled_fwd', 'rolled forward'),
            ('moved', 'moved without being asked'), ('op_off_roll', 'rolled after OP was switched off'),
            ('slip', 'slipped back on launch')]


# ============================================================================================ report sections
def print_bottom_line(D):
  st = D.stops
  say()
  say('BOTTOM LINE')
  if not st:
    say('  No stops with openpilot in control in this part of the drive.')
  else:
    cats = {}
    for s in st:
      cats.setdefault(category(s), []).append(s)
    parts = []
    for c, txt in CAT_TEXT:
      if not cats.get(c):
        continue
      if c == 'possible_roll':
        nr = sum(1 for x in cats[c] if x.motion.get('requested'))
        if nr:
          txt += f': {len(cats[c]) - nr} with nothing asking to go, {nr} after a request to go'
      parts.append(f'{len(cats[c])} {txt}')
    say(f'  {plural(len(st), "stop")} with openpilot in control: {", ".join(parts)}.')
    for s in st:
      if category(s) in ('rolled_back', 'possible_roll', 'rolled_fwd', 'moved', 'op_off_roll'):
        say(f'  Stop #{s.no} at {fnum(s.grade, "{:+.1f}%")}: {stop_result(D, s, short=True)}, wheels moved '
            f'{(s.onset - s.t0) / NS:.1f} s after stopping.')
    known = [s for s in st if s.esp_ok]
    if known:
      conf = [s for s in known if s.hold_on is not None]
      lost = []
      for s in conf:
        if s.hold_off is None:
          continue
        m = s.motion
        if m is not None and s.hold_off >= s.onset - int(0.2 * NS):
          continue
        if m is not None and (m.get('op_off') and m['op_off'][0] <= s.hold_off):
          continue
        if m is not None and m['requested'] and s.onset - s.hold_off < int(1.0 * NS):
          continue
        if s.op_off is not None and s.op_off <= s.hold_off:
          continue
        lost.append(s)
      say(f'  The ESP confirmed a hold (bit 53) in {len(conf)} of {len(known)} stops' +
          (f'; while OP was in control it LET GO before the car was asked to move in {plural(len(lost), "stop")} '
           f'(#{", #".join(str(s.no) for s in lost)}).' if lost else '; it never let go early while OP was in control.'))
  if not D.has_tsk:
    say('  TSK_06 (cruise status, 0x120) is NOT in the log: TSK faults cannot be checked.')
  elif D.has_can:
    say(f'  TSK faults: {len(D.faults)}' + (f' (F1 at {clock((D.faults[0].t0 - D.t_route0) / NS)}).' if D.faults else '.'))


def print_summary(D):
  stops = D.stops
  say()
  if not stops:
    say('STOPS: none found where openpilot was in control' + (f' ({plural(D.other_stops, "stop")} without OP)' if D.other_stops else '') + '.')
    return
  say(f'STOPS WITH OP IN CONTROL: {len(stops)}' + (f'  (+ {plural(D.other_stops, "stop")} without OP, not shown)' if D.other_stops else ''))
  say('Sorted steepest uphill first. Times are from the moment all four wheels read 0. "-" = did not happen / no data.')
  say(f"{'#':>3} {'at':>5} {'grade':>6} {'still':>6} {'Anf':>6} {'Hold':>11} {'nv':>5} {'TSK':>5} {'HMS sent':<11} {'/s':>3}  result")
  for st in sorted(stops, key=lambda s: (-(s.grade if isnum(s.grade) else -999), s.t0)):
    if st.esp_ok:
      anf = ('pre' if st.anf_pre else f'{(st.anf_on - st.t0) / NS:+.1f}') if st.anf_on is not None else '-'
      if st.hold_on is not None:
        hold = f'{(st.hold_on - st.t0) / NS:+.1f}..' + (f'{(st.hold_off - st.t0) / NS:+.1f}' if st.hold_off else 'on')
      else:
        hold = 'never'
      nv = 'pre' if st.nv_pre else (f'{(st.nv_on - st.t0) / NS:+.1f}' if st.nv_on else '-')
      tsk = ('-' if st.tsk_max is None else (f'{st.tsk_fault[1]}!' if st.tsk_fault else 'ok')) if D.has_tsk else 'n/a'
    else:
      anf = hold = nv = tsk = '?'
    hseq = st.hms_seq if st.op_ok else '?'
    if len(hseq) > 11:
      hseq = hseq[:10] + '+'
    say(f"{st.no:>3} {clock((st.t0 - D.t_route0) / NS):>5} {fnum(st.grade, '{:+.1f}%', 6)} {st.still_s:>5.1f}s "
        f"{anf:>6} {hold:>11} {nv:>5} {tsk:>5} {hseq:<11} {st.hms_max_s if st.op_ok else '?':>3}  {stop_result(D, st)}")
  say('  at = time into the drive   Hold = ESP hold confirmed from..until   /s = most HMS changes in one second')


def fault_numbers(f):
  ev = f.ev

  def n(x):
    return '-' if x is None else str(x)
  bits = []
  for k, label in (('b29', 'bit 29'), ('b38', 'bit 38'), ('b45', 'bit 45'), ('esp05_b29', 'ESP_05 bit 29')):
    v = ev['bits'].get(k)
    if v is None:
      continue
    rise, now = v
    bits.append(f'{label} ' + (f'rose {rise:+.2f} s' if rise is not None else f'={b01(now)}'))
  return (f'HMS changes {n(ev["hms_chg"])} (sendcan) / {n(ev["hms_chg_echo"])} (echo), ACC_06 bit changes '
          f'{n(ev["a06_chg"])}, long-state changes {n(ev["lcs_chg"])}, gas/brake edges {ev["gas_edges"]}/'
          f'{ev["brake_edges"]}, OP switched off {ev["la_off"]}x, panda safetyTxBlocked '
          f'{"+" + str(ev["txblk"]) if ev["txblk"] is not None else "not logged"}, refused frames {n(ev["rej"])}, '
          + ', '.join(bits))


def fault_label_text(f):
  if f.labels:
    txt = 'consistent with ' + ' + '.join(f.labels)
  else:
    txt = 'no OP chatter, panda block, pedal toggling or ESP give-up in the 2 s before (unclassified)'
  if f.extra:
    txt += '; also: ' + ', '.join(f.extra)
  return txt


def print_faults(D):
  rs = D.route_stats
  say()
  if not D.has_tsk:
    say('CRUISE/TSK FAULTS: TSK_06 (0x120) is not in the log, so faults cannot be checked.')
  else:
    if not D.tsk_full:
      say('Note: TSK_06 is thin in these logs (qlog?), so faults below may be incomplete.')
    if D.faults:
      say(f'CRUISE/TSK FAULTS (TSK status 6 or 7): {len(D.faults)}')
      for f in D.faults:
        where = f'during stop #{f.during_stop}' if f.during_stop else f'at {fnum(f.speed, "{:.1f}")} m/s, not during an OP stop'
        say(f'  F{f.no}  {clock((f.t0 - D.t_route0) / NS):>5}  TSK {f.before if f.before is not None else "?"} -> {f.value} '
            f'({TSK_TEXT.get(f.value, "?")}), {where}; HMS changes in the 3 s before: {f.hms_changes_3s} '
            f'(max {f.hms_max_s}/s)' + ('; lasted to end of log' if f.lasts_to_end else f'; lasted {(f.t_endf - f.t0) / NS:.1f} s'))
        say(f'       2 s before (rule 6): {fault_label_text(f)}')
        say(f'       numbers: {fault_numbers(f)}')
    else:
      say('CRUISE/TSK FAULTS: none (TSK status never 6 or 7).')
  if 'nv_rises' in rs:
    if rs.get('nv_always'):
      say('ESP nicht_verfuegbar (bit 29): ON for the whole drive.')
    elif rs['nv_rises']:
      say(f'ESP nicht_verfuegbar (bit 29) came on {plural(len(rs["nv_rises"]), "time")}; first at '
          f'{clock((rs["nv_rises"][0] - D.t_route0) / NS)} into the drive.')
    else:
      say('ESP nicht_verfuegbar (bit 29): never came on.' if rs.get('nv_frac', 0) == 0 else
          f'ESP nicht_verfuegbar (bit 29): on {rs["nv_frac"] * 100:.0f}% of the time (already on at the start).')


def h7_src_name(src):
  if src == 'sendcan':
    return 'sendcan (what OP asked the panda to send)'
  if src == 'mixed':
    return 'ALL 0x12E frames in the car CAN log, sources mixed together'
  return src_label(src)


def print_h7(D):
  h = D.h7
  say()
  say('HMS SWITCH COUNTS PER SOURCE (decision rule 5, H7) - your measure: HMS switches per 1000 grade-standstill frames')
  say('  Grade-standstill frame = an ACC_07 frame logged while carState.vEgo < 0.1 m/s and |grade| >= 3% (0x65F).')
  if not h['rows']:
    say('  No ACC_07 frames in this log.')
  for src, r in h['rows']:
    if r is None:
      continue
    vals = ', '.join(f'{k} x{n}' for k, n in sorted(r['values'].items(), key=lambda kv: -kv[1])[:4]) or '-'
    pr = ', '.join(f'{a}<->{b} x{n}' for (a, b), n in r['pairs'][:2])
    rate = f'{r["per1000"]:.0f} per 1000' if r['per1000'] is not None else 'no frames'
    say(f'  {h7_src_name(src)}: {r["switches"]} switches / {r["frames"]} frames = {rate}; values {vals}'
        + (f'; mostly {pr}' if pr else ''))
  say(f'  -> {h["verdict"]}')


def print_route_stats(D):
  rs = D.route_stats
  if 'frames' not in rs:
    return
  say()
  if not D.op_full:
    say('HMS statistics skipped: not enough sendcan data in these logs (qlog only).')
    return
  if D.send_gaps:
    t, d = max(D.send_gaps, key=lambda x: x[1])
    say(f'OP paused its ACC_06/ACC_07 messages {plural(len(D.send_gaps), "time")} for more than 100 ms '
        f'(longest {d / 1e6:.0f} ms at {clock((t - D.t_route0) / NS)}).')
  else:
    say('OP sent ACC_06/ACC_07 without pauses (never more than 100 ms apart).')
  nf = {k: len(v) for k, v in D.foreign.items() if len(v)}
  if nf:
    first = min(int(v[0]) for v in D.foreign.values() if len(v))
    say('WARNING: ' + ' and '.join(f'{n} {k}' for k, n in nf.items()) + ' frames from ANOTHER sender were seen on the car bus '
        f'(first at {clock((first - D.t_route0) / NS)}). Only OP should send these; two senders confuse the ESP/TSK.')
  if D.has_echo:
    rej = sum(rs['rejected'].values())
    say(f'Panda refused {rej} of OP\'s ACC_06/ACC_07 frames in this drive.' if rej else
        'Panda refused none of OP\'s ACC_06/ACC_07 frames.')


def onset_src(st):
  h = st.onset_how or ''
  return 'esp19_wheels' if 'ESP_19 wheel speeds above' in h else ('pulses' if 'pulse' in h else 'speed')


def onset_compact(D, st):
  """Two key=value lines: the decision-rule-2 snapshot at the onset (or while standing if the car never moved)."""
  if st.motion is not None:
    t = st.onset
    head = (f'AT ROLLBACK ONSET {relc((t - st.t0) / NS)} s ({onset_src(st)}'
            + (', OP had asked to go' if st.motion.get('requested') else '') + '):')
  else:
    t = st.t0 + int(min(2.0, max(st.still_s / 2, 0.2)) * NS)
    head = f'AT ROLLBACK ONSET: none (wheels never moved); standing at {relc((t - st.t0) / NS)} s:'
  snap = snapshot(D, t)
  srcs = ' '.join(f'{k}={v}' for k, v in snap if k.startswith('hms_src'))
  return [f'{head} ' + snapshot_compact(snap, SNAP_PASTE_1) + ' ' + srcs + ' ' + snapshot_compact(snap, SNAP_PASTE_2),
          '   ' + snapshot_compact(snap, SNAP_PASTE_3) + ' ' + snapshot_compact(snap, SNAP_PASTE_4)]


def print_decisions_all(D):
  if not D.stops:
    return
  say()
  say('DECISIONS PER STOP (the study\'s rules R1-R9 -> hypotheses H1-H9; "consistent with" means the numbers fit,')
  say('not proof). R5 (H7) is the per-source switch count above.')
  for st in sorted(D.stops, key=lambda s: (-(s.grade if isnum(s.grade) else -999), s.t0)):
    say(f' Stop #{st.no} ({fnum(st.grade, "{:+.1f}%")}, {stop_result(D, st, short=True)}):')
    for ln in onset_compact(D, st):
      say(f'  {ln}')
    for tag, txt in decisions(D, st):
      say(f'  {tag} {txt}')


def motion_lines(D, st):
  m = st.motion
  out = []
  if m is None:
    if st.early_moves:
      out.append('The wheel pulse counters ticked ' + ', '.join(f'{c[2]:.0f} at {rel((c[0] - st.t0) / NS)}' for c in st.early_moves)
                 + ' while the car was stopped (too little to count as a roll).')
    return out
  who = []
  who.append('gas pressed' if m['gas'] else 'no gas')
  if m['op_req']:
    who.append(m['op_req_how'])
  else:
    txt = 'no request to go from OP'
    if m['op_not_req_why']:
      txt += f' ({m["op_not_req_why"]})'
    if m['op_sent_go']:
      txt += f'; OP did send {m["op_sent_go"]}'
    who.append(txt)
  dsrc = {-1: 'BACKWARD', 1: 'forward', None: 'unknown'}[m['dir']]
  if m.get('slip'):
    dsrc = 'BACKWARD first, then forward'
  if m['dir'] is None:
    dsrc += (' (no valid direction signal: ' + ('ESP_10 not validated' if len(D.dir_raw) else 'ESP_10 not in the log')
             + ('' if D.rev_seen else ', ESP_05 reverse flag never set in this drive') + ')')
  out.append(f'Wheels started to move {rel((st.onset - st.t0) / NS)} after the stop ({st.onset_how}); direction {dsrc}'
             + (f' (from {m["dir_source"]})' if m['dir_source'] else '') + f'; {", ".join(who)}.')
  dd = f'{m["dist"]:.2f} m' + (f' ({m["pulses"]:.0f} wheel pulses)' if m.get('pulses') is not None else '')
  how_d = {'pulses': 'wheel pulses', 'speed': 'integrated wheel speed'}.get(
    m['dist_how'], 'integrated wheel speed (the ESP_10 pulse counters showed almost no movement here: MISMATCH)')
  out.append(f'Distance {dd}, measured from {how_d}; peak {m["peak"]:.2f} m/s.')
  if st.early_moves:
    out.append('Before that the pulse counters already ticked ' +
               ', '.join(f'{c[2]:.0f} at {rel((c[0] - st.t0) / NS)}' for c in st.early_moves) + '.')
  if m.get('op_off'):
    toff, cause = m['op_off']
    out.append(f'OP long control was OFF before the wheels moved: switched off {rel((toff - st.t0) / NS)} ({cause}).')
  return out


def stop_window(D, st, args):
  a = min(st.t_lcs_stop or st.t0, st.t0) - int(args.before * NS)
  if st.motion is not None:
    b = st.onset + int(args.after * NS)
  else:
    b = st.t0 + int(max(args.after, 8.0) * NS)
  return a, b


def print_stop_detail(D, st, args, title=None):
  say()
  say('=' * 78)
  seg = seg_of(D, st.t0)
  say((title or f'STOP #{st.no} IN DETAIL') + f' - {clock((st.t0 - D.t_route0) / NS)} into the drive'
      + (f' (segment {seg[0]}, {seg[1]:.1f} s into it)' if seg else ''))
  say('=' * 78)
  g = st.grade
  gtxt = f'{g:+.1f}% ({"uphill" if g > 0.5 else ("downhill" if g < -0.5 else "flat")})' if isnum(g) else 'not available'
  pit = D.s['pitch'].median(st.t0, st.t0 + NS) if D.has_pitch else float('nan')
  say(f'Grade while stopped: {gtxt}' + (f'; openpilot pitch (orientationNED[1]) {pit:+.3f} rad' if isnum(pit) else ''))
  ctl = 'ON at the stop' if st.op_at_stop else 'OFF at the stop (it was on within the 3 s before)'
  say(f'OP longitudinal control: {ctl}' + (f'; OP turned off {rel((st.op_off - st.t0) / NS)}' if st.op_off else ''))
  say(f'+0.00 = {st.anchor_how}' + (f'; OP\'s stopping state began {rel((st.t_lcs_stop - st.t0) / NS)}'
                                    if st.t_lcs_stop is not None else '') + '.')
  say(f'Result: {stop_result(D, st)}')
  for ln in motion_lines(D, st):
    say(f'  {ln}')
  say(f'Stood still for {st.still_s:.2f} s (speed under 0.1 m/s)' + (' (log ended while still stopped)' if st.t1 is None else ''))
  if st.esp_ok:
    hold = 'never confirmed'
    if st.hold_on is not None:
      hold = f'confirmed {rel((st.hold_on - st.t0) / NS)}'
      hold += f', DROPPED {rel((st.hold_off - st.t0) / NS)} ({(st.hold_off - st.hold_on) / NS:.2f} s)' if st.hold_off else ', stayed on'
    anf = 'never on'
    if st.anf_on is not None:
      anf = ('already on before the stop' if st.anf_pre else f'on {rel((st.anf_on - st.t0) / NS)}')
      anf += f', off {rel((st.anf_off - st.t0) / NS)}' if st.anf_off else ', stayed on'
    b63 = 'never on' if st.b63_on is None else (f'on {rel((st.b63_on - st.t0) / NS)}' +
                                                (f', off {rel((st.b63_off - st.t0) / NS)}' if st.b63_off else ', stayed on'))
    nv = 'was ALREADY ON before this stop' if st.nv_pre else (f'came ON {rel((st.nv_on - st.t0) / NS)}' if st.nv_on else 'never on')
    if not D.has_tsk:
      tsk = 'TSK_06 not in the log'
    else:
      tsk = 'no data' if st.tsk_max is None else (f'FAULT {st.tsk_fault[1]} at {rel((st.tsk_fault[0] - st.t0) / NS)}'
                                                  if st.tsk_fault else f'ok (highest {st.tsk_max})')
    say(f'ESP hold (Haltebestaetigung, bit 53): {hold}')
    say(f'ESP ACC stopping procedure (bit 63): {b63}')
    say(f'ESP SRBM Anf (bit 28): {anf}')
    say(f'ESP nicht_verfuegbar (bit 29): {nv}')
    say(f'TSK status: {tsk}')
  else:
    say('ESP side: not enough car CAN data around this stop to tell (qlog only, or no CAN in the log).')
  if not st.op_ok:
    say('What OP sent (HMS): not enough sendcan data around this stop to tell.')
  else:
    per = f'; {st.hms_per1000:.0f} switches per 1000 standstill frames' if st.hms_per1000 is not None else ''
    say(f'OP HMS sent: {st.hms_seq}, {plural(len(st.hms_changes), "change")}, max {st.hms_max_s} per second{per}')
  if D.has_echo:
    say(f"Panda REFUSED {st.rejected} of OP's ACC_06/ACC_07 frames around this stop (panda safety limit)." if st.rejected
        else "Panda refused none of OP's ACC frames around this stop.")
  if st.driver_brake is not None:
    say(f'Driver brake pressed {rel((st.driver_brake - st.t0) / NS)}')
  if st.handbrake:
    say('Handbrake was ON during this stop.')
  say()
  if st.motion is not None:
    say(f'AT ROLLBACK ONSET ({rel((st.onset - st.t0) / NS)}, {st.onset_how}):')
    snap_t = st.onset
  else:
    snap_t = st.t0 + int(min(2.0, max(st.still_s / 2, 0.2)) * NS)
    say(f'AT ROLLBACK ONSET: the wheels never moved during this stop. While standing ({rel((snap_t - st.t0) / NS)}):')
  for ln in snapshot_lines(D, snapshot(D, snap_t)):
    say(f'  {ln}')
  say()
  say('DECISIONS (study rules; "consistent with" = the numbers fit that hypothesis, not proof):')
  for tag, txt in decisions(D, st):
    say(f'  {tag} {txt}')
  a, b = stop_window(D, st, args)
  say()
  say('What happened (times from +0.00):')
  marks = [(st.onset, f'WHEELS START TO TURN ({st.onset_how})')] if st.motion is not None else []
  moments = key_moments(D, a, max(b, st.t_end + NS), st.t0)
  for t, txt in sorted(moments + marks, key=lambda x: x[0]):
    say(f'  {(t - st.t0) / NS:+7.2f} s  {txt}')
  say()
  print_windowed_timeline(D, st, a, b, args, st.t0)


def print_windowed_timeline(D, st, a, b, args, t_ref, marks=None):
  """The study window; a long quiet middle is skipped so the phone screen is not flooded."""
  marks = marks if marks is not None else short_marks(D, st)
  say(f'Timeline, {args.hz} rows per second, from {args.before:g} s before OP\'s stopping state to '
      f'{args.after:g} s after the wheels moved (aligned on +0.00):')
  mid_a = t_ref + 2 * NS
  mid_b = (st.onset - 2 * NS) if st.motion is not None else None
  if mid_b is not None and mid_b - mid_a > 4 * NS and mid_a > a and mid_b < b:
    print_timeline(D, t_ref, a, mid_a, args.hz, marks)
    say(f'... skipping {rel((mid_a - t_ref) / NS)} to {rel((mid_b - t_ref) / NS)} (see "What happened" for changes) ...')
    print_timeline(D, t_ref, mid_b, b, args.hz, marks)
  else:
    print_timeline(D, t_ref, a, b, args.hz, marks)


def choose_default(D):
  order = ['rolled_back', 'possible_roll', 'op_off_roll', 'rolled_fwd', 'moved', 'slip', 'held']
  for cat in order:
    pool = [s for s in D.stops if category(s) == cat]
    if pool:
      return max(pool, key=lambda s: (not s.motion.get('requested') if s.motion else True,
                                      abs(s.grade) if isnum(s.grade) else -1, s.t0))
  return None


def choose_compare(D, worst):
  """A stop that held, closest to 5% uphill (the study's comparison)."""
  pool = [s for s in D.stops if s is not worst and category(s) in ('held', 'slip') and isnum(s.grade)]
  if not pool:
    return None
  return min(pool, key=lambda s: (abs(s.grade - 5.0), s.t0))


def compare_rows(D, w, c):
  rel_on = (w.onset - w.t0) if w.motion is not None else int(2 * NS)

  def hold(st):
    if not st.esp_ok:
      return '?'
    if st.hold_on is None:
      return 'never'
    return f'{(st.hold_on - st.t0) / NS:+.2f}..' + (f'{(st.hold_off - st.t0) / NS:+.2f}' if st.hold_off else 'on')

  def onoff(on, off, st):
    if on is None:
      return 'never'
    return f'{(on - st.t0) / NS:+.2f}..' + (f'{(off - st.t0) / NS:+.2f}' if off else 'on')

  def torque(st):
    if st.brm_stand is None:
      return '-'
    e = st.brm_exp
    return f'{st.brm_stand:.0f}' + (f' (comp {e["comp"]:.0f}/not {e["uncomp"]:.0f})' if e else '')

  def at(st, key):
    d = dict(snapshot(D, st.t0 + rel_on))
    return d.get(key, '-')

  rows = [
    ('grade %', fnum(w.grade, '{:+.1f}'), fnum(c.grade, '{:+.1f}')),
    ('result', category(w), category(c)),
    ('lcs->stopping', relc((w.t_lcs_stop - w.t0) / NS) if w.t_lcs_stop else '-', relc((c.t_lcs_stop - c.t0) / NS) if c.t_lcs_stop else '-'),
    ('bit 53 hold', hold(w), hold(c)),
    ('bit 28 hold active', onoff(w.anf_on, w.anf_off, w), onoff(c.anf_on, c.anf_off, c)),
    ('bit 63 ACC stopping', onoff(w.b63_on, w.b63_off, w), onoff(c.b63_on, c.b63_off, c)),
    ('HMS sent', w.hms_seq, c.hms_seq),
    ('HMS 1 s (at 0 speed)', f'{w.hms1_s:.2f} ({fnum(w.hms1_still_s, "{:.2f}")})', f'{c.hms1_s:.2f} ({fnum(c.hms1_still_s, "{:.2f}")})'),
    ('HMS->4 at', relc((w.latch_t - w.t0) / NS) if w.latch_t else '-', relc((c.latch_t - c.t0) / NS) if c.latch_t else '-'),
    ('H8 frames', str(w.h8), str(c.h8)),
    ('TSK brake raw (stand)', torque(w), torque(c)),
    ('brake bar (stand)', fnum(w.bar_stand, '{:.1f}'), fnum(c.bar_stand, '{:.1f}')),
    (f'at {relc(rel_on / NS)}: b53', at(w, 'b53'), at(c, 'b53')),
    (f'at {relc(rel_on / NS)}: HMS', at(w, 'hms_sent'), at(c, 'hms_sent')),
    (f'at {relc(rel_on / NS)}: TSK raw', at(w, 'brm_raw'), at(c, 'brm_raw')),
    (f'at {relc(rel_on / NS)}: bar', at(w, 'bar'), at(c, 'bar')),
    (f'at {relc(rel_on / NS)}: pulses', fnum(pulses_between(D, w.t0, w.t0 + rel_on), '{:.0f}'),
     fnum(pulses_between(D, c.t0, c.t0 + rel_on), '{:.0f}')),
  ]
  return rows


def print_compare(D, w, c, args):
  say()
  say('=' * 78)
  say(f'COMPARISON: stop #{c.no} ({fnum(c.grade, "{:+.1f}%")}, held) lined up with stop #{w.no} '
      f'({fnum(w.grade, "{:+.1f}%")}) on the moment all four wheels read 0')
  say('=' * 78)
  say(f"{'':<24} {'#' + str(w.no):<26} {'#' + str(c.no)}")
  for name, a, b in compare_rows(D, w, c):
    say(f'{name:<24} {a:<26} {b}')
  rel_on = (w.onset - w.t0) if w.motion is not None else int(2 * NS)
  say()
  say(f'Stop #{c.no} at {rel(rel_on / NS)} (the same time after stopping as the onset on stop #{w.no}):')
  for ln in snapshot_lines(D, snapshot(D, c.t0 + rel_on)):
    say(f'  {ln}')
  say(f'DECISIONS for stop #{c.no}:')
  for tag, txt in decisions(D, c):
    say(f'  {tag} {txt}')
  wa, wb = stop_window(D, w, args)
  a, b = c.t0 + (wa - w.t0), c.t0 + (wb - w.t0)
  say()
  say(f'Timeline of stop #{c.no}, same window as stop #{w.no} ({rel((wa - w.t0) / NS)} to {rel((wb - w.t0) / NS)}):')
  print_timeline(D, c.t0, a, b, args.hz, short_marks(D, c))


def print_fault_detail(D, f, args):
  say()
  say('=' * 78)
  seg = seg_of(D, f.t0)
  say(f'FAULT F{f.no} IN DETAIL - TSK status {f.before if f.before is not None else "?"} -> {f.value} '
      f'({TSK_TEXT.get(f.value, "?")}) at {clock((f.t0 - D.t_route0) / NS)} into the drive'
      + (f' (segment {seg[0]}, {seg[1]:.1f} s into it)' if seg else ''))
  say('=' * 78)
  say(f'Speed {fnum(f.speed, "{:.2f}")} m/s, grade {fnum(f.grade, "{:+.1f}")}%, '
      + (f'during stop #{f.during_stop}' if f.during_stop else 'not during an OP stop'))
  say(f'OP at that moment: long control {"ON" if isnum(f.long_active) and f.long_active >= 0.5 else "off"}, '
      f'state {lcs_name(f.lcs)}, HMS sent {int(f.hms) if isnum(f.hms) else "-"}, nv {fbool(f.nv)}')
  say(f'In the 2 s before (rule 6): {fault_label_text(f)}')
  say(f'  numbers: {fault_numbers(f)}')
  say(f'In the 3 s before: OP changed HMS {plural(f.hms_changes_3s, "time")} (max {f.hms_max_s} per second)')
  if f.max_gap_ms:
    say(f'In the 3 s before: OP stopped sending ACC_07 for up to {f.max_gap_ms:.0f} ms (normal: every 20 ms)')
  if f.foreign:
    say(f'In the 3 s before: {f.foreign} ACC_06/ACC_07 frames from ANOTHER sender were on the car bus')
  say('The fault lasted to the end of the log (a permanent TSK fault needs a key cycle).' if f.lasts_to_end else
      f'The fault lasted {(f.t_endf - f.t0) / NS:.1f} s.')
  a = f.t0 - int(max(args.before, 2.0) * NS)
  b = f.t0 + int(args.after * NS)
  say()
  say('What happened (times from the fault):')
  for t, txt in key_moments(D, a, b, f.t0):
    say(f'  {(t - f.t0) / NS:+7.2f} s  {txt}')
  say()
  say(f'Timeline, {args.hz} rows per second (t from the fault):')
  print_timeline(D, f.t0, a, b, args.hz, [(f.t0, f'TSK {f.value}')])


def seg_of(D, t):
  for num, a, b, _ in D.seg_bounds:
    if a <= t <= b + NS:
      return num, (t - a) / NS
  return None


# ============================================================================================ paste-back block
def hyp_tags(D, st):
  """For / against per hypothesis, from the same numbers as decisions()."""
  f, a = [], []
  m = st.motion
  moved = m is not None and m['kind'] in MOVE_KINDS
  if st.esp_ok and st.hold_on is None and st.latch_t is not None and st.hms1_still_s is not None:
    (f if st.hms1_still_s < 1.0 else a).append('H1')
  if st.drop:
    if st.drop['kind'] == 'H1':
      f.append('H1')
    elif st.drop['kind'] == 'H2':
      f.append('H2')
      a.append('H1')
  if moved and isnum(st.hold_at_onset) and st.hold_at_onset >= 0.5:
    f.append('H2')
  if st.esp_ok and len(D.c['nv']):
    (f if (st.nv_pre or st.nv_at_stop) else a).append('H3')
  _ = moved
  e = st.brm_exp
  if st.brm_class == 'comp' and e and e['comp'] < e['need']:
    f.append('H4')
  elif st.brm_class == 'uncomp':
    a.append('H4')
  if st.h5 and st.h5['judge']:
    if st.h5['zero_t'] is not None and st.h5['stays']:
      f.append('H5')
    elif st.h5['zero_t'] is None:
      a.append('H5')
  if st.op_ok:
    (f if st.h8 else a).append('H8')
  if st.h6 and m is not None and not m.get('requested') and (st.h6['<1.0'][3] or st.h6['>=1.0'][1].get(1, 0)):
    f.append('H6')
  if m is not None and m.get('op_off') and category(st) == 'op_off_roll':
    f.append('H9')
  elif m is not None and m.get('op_off'):
    pass
  elif not [x for x in st.h9 if st.onset is None or x[0] <= st.onset]:
    a.append('H9')
  dedup = lambda xs: sorted(set(xs), key=lambda x: int(x[1:]))  # noqa: E731
  fs = dedup(f)
  return fs, [x for x in dedup(a) if x not in fs]


def paste_stop_line(D, st):
  hold = 'never' if st.hold_on is None else (f'{relc((st.hold_on - st.t0) / NS)}..' +
                                              (relc((st.hold_off - st.t0) / NS) if st.hold_off else 'on'))
  if not st.esp_ok:
    hold = '?'
  m = st.motion
  f, a = hyp_tags(D, st)
  how = ''
  if m is not None:
    how = f' kind={m["kind"]} dist={m["dist"]:.2f}' + (f' slip={m["slip"][1] * 100:.0f}cm' if m.get('slip') else '') + \
          (f' by={who_asked(m).replace(" ", "_")}' if m['requested'] else '') + \
          (f' opoff={relc((m["op_off"][0] - st.t0) / NS)}' if m.get('op_off') else '')
  return (f'S#{st.no} g={fnum(st.grade, "{:+.1f}")} {category(st)}{how} onset='
          f'{relc((st.onset - st.t0) / NS) if st.onset else "-"} b53={hold} hms={st.hms_seq if st.op_ok else "?"} '
          f'brm={fnum(st.brm_stand, "{:.0f}")}/{st.brm_class or "-"} h8={st.h8} '
          + (f'for={",".join(f) or "-"}' if is_unasked_roll(st) or category(st) == 'op_off_roll'
             else f'no_roll_seen={",".join(f) or "-"}'))


def build_paste(D, args, route, used, n_ok, problems):
  L = []
  br, cm, dirty, ver = D.init if D.init else ('?', '?', False, '?')
  L.append(f'mk7_hold_trace v{TOOL_VERSION.split()[0]} route={route.name} segs={used[0].num}-{used[-1].num} '
           f'logs=rlog{route.n_rlog}/{len(route.segments)}' + (f' qlog_only={route.n_qlog_only}' if route.n_qlog_only else ''))
  L.append(f'op={ver or "?"} branch={br or "?"} commit={(cm or "?")[:10]} dirty={int(bool(dirty))}')
  srcs = ','.join(f'{s}' + ('(radar)' if s in D.radar_srcs else ('(echo)' if 128 <= s < 192 else
                                                                   ('(REFUSED)' if s >= 192 else '(FOREIGN)')))
                  for s in sorted(D.acc_src)) or 'none'
  L.append(f'can: pt_bus={D.pt_bus} sendcan_bus={D.op_bus} acc_src={srcs} panda={"yes" if D.has_panda else "no"} '
           f'ctrlState={"yes" if D.has_ctl else "no"} plan={"yes" if D.has_plan else "no"} '
           f'pitch={"yes" if D.has_pitch else "no"}')
  L.append(f'esp10_dir={D.dir_short} pulses={"VALID" if D.pulse_ok else "NOT_USED"}'
           + (f'({D.m_per_pulse * 100:.1f}cm)' if D.pulse_ok and D.m_per_pulse else '')
           + f' tsk06={"yes" if D.has_tsk else "NO"} m16={"yes" if len(D.grade) else "no"} anchor='
           + ('esp19' if D.anchor_how.startswith('all four') else 'fallback')
           + f' dbc_check={n_ok}/{len(USER_VERIFIED)}' + (f' MISMATCH={len(problems)}' if problems else '')
           + ''.join(f' PULSE_MISMATCH=#{x.no}(pul{x.pul_mm[2]:.2f}/spd{x.pul_mm[3]:.2f}m)' for x in D.stops if x.pul_mm)[:120])
  cats = {}
  for s in D.stops:
    cats[category(s)] = cats.get(category(s), 0) + 1
  L.append(f'stops={len(D.stops)} held={cats.get("held", 0)} ' + ' '.join(f'{c}={cats[c]}' for c, _ in CAT_TEXT
                                                                     if c != 'held' and cats.get(c)) +
           f' b53_any={int(D.b53_any)} faults={len(D.faults) if D.has_tsk else "n/a"}')
  w = choose_default(D)
  if w is not None:
    m = w.motion
    L.append(f'WORST #{w.no} g={fnum(w.grade, "{:+.1f}")} {category(w)} '
             + (f'dist={m["dist"]:.2f}m({m["dist_how"].split()[0] + ("_PULSE_MISMATCH" if "did not" in m["dist_how"] else "")}) peak={m["peak"]:.2f} '
                f'dir={ {-1: "back", 1: "fwd", None: "unknown"}[m["dir"]]}({(m["dir_source"] or "none").split(" (")[0].replace(" ", "_")}) '
                f'onset={relc((w.onset - w.t0) / NS)}({onset_src(w)}) '
                f'req={int(m["requested"])} '
                if m else 'no_motion ')
             + f'lcs_stop={relc((w.t_lcs_stop - w.t0) / NS) if w.t_lcs_stop else "-"} '
             + f'latch4={relc((w.latch_t - w.t0) / NS) if w.latch_t else "-"}')
    snap_t = w.onset if m is not None else w.t0 + int(min(2.0, max(w.still_s / 2, 0.2)) * NS)
    snap = snapshot(D, snap_t)
    srcs = ' '.join(f'{k}={v}' for k, v in snap if k.startswith('hms_src'))
    L.append(' onset1: ' + snapshot_compact(snap, SNAP_PASTE_1) + ' ' + srcs)
    L.append(' onset2: ' + snapshot_compact(snap, SNAP_PASTE_2))
    L.append(' onset3: ' + snapshot_compact(snap, SNAP_PASTE_3))
    L.append(' onset4: ' + snapshot_compact(snap, SNAP_PASTE_4))
    hold = 'never' if w.hold_on is None else (f'{relc((w.hold_on - w.t0) / NS)}..' +
                                               (relc((w.hold_off - w.t0) / NS) if w.hold_off else 'on'))
    L.append(f' R1 b53={hold if w.esp_ok else "?"} hms1={w.hms1_s:.2f}s still={fnum(w.hms1_still_s, "{:.2f}")}s '
             f'b28={relc((w.anf_on - w.t0) / NS) if w.anf_on else "never"} b63={relc((w.b63_on - w.t0) / NS) if w.b63_on else "never"}')
    if w.drop:
      dr = w.drop
      L.append(f' R2 drop={dr["what"].split(" (")[0].replace(" ", "")}@{relc((dr["t"] - w.t0) / NS)} kind={dr["kind"]} '
               f'steady={dr["steady_s"]:.2f}s last={(dr["last"][1] if dr["last"] else "none").replace(" ", "_")} '
               f'after={(dr["after"][0][1] if dr["after"] else "none").replace(" ", "_")}')
    else:
      L.append(f' R2 drop=none hold_at_onset={b01(w.hold_at_onset)}')
    L.append(f' R2 H3 b29_at_stop={int(bool(w.nv_pre or w.nv_at_stop))} b29_on='
             f'{relc((w.nv_on - w.t0) / NS) if w.nv_on else "-"} nv_first='
             f'{clock((D.nv_first - D.t_route0) / NS) if D.nv_first is not None else "-"}')
    e = w.brm_exp
    L.append(f' R2 H4 brm_stand={fnum(w.brm_stand, "{:.1f}")} brm_onset={fnum(w.brm_onset, "{:.0f}")} '
             + (f'exp_comp={e["comp"]:.0f} exp_uncomp={e["uncomp"]:.0f} need={e["need"]:.0f} a_req={e["a_req"]:+.2f} '
                f'mass={e["mass"]:.0f} ' if e else 'exp=? ')
             + f'class={w.brm_class or "-"} bar={fnum(w.bar_stand, "{:.1f}")}')
    if w.h5:
      h = w.h5
      L.append(f' R2 H5 brm_pre={fnum(h["pre"], "{:.0f}")} after={fnum(h["after"], "{:.0f}")} zero_frames='
               f'{h["frames"] if h["frames"] is not None else "-"} stays={h["stays"]} a06_stp={b01(h["a06_stop"])} '
               f'a06_acc={fnum(h["a06_acc"], "{:+.2f}")} freig={fnum(h["freig_after"], "{:.0f}")} '
               f'verz27={fnum(h["verz_after"], "{:.0f}")} win={relc((h["win_end"] - w.t0) / NS)}:'
               f'{h["win_why"].split(" (")[0].replace(" ", "_")}' + ('' if h['judge'] else ' UNJUDGED')
               + (f' zero_late={relc((h["zero_late"] - w.t0) / NS)}' if h.get('zero_late') is not None else ''))
    else:
      L.append(' R2 H5 no_latch' if D.has_tsk else ' R2 H5 no_tsk06')
    L.append(f' R3 H8 frames={w.h8} pid_v<0.5={w.h8_pid_slow}')
    if w.h6:
      def band(bn):
        n, hs, ls, h4s = w.h6[bn]
        return (f'n={n} hms{{{",".join(f"{k}:{v}" for k, v in sorted(hs.items()))}}} hms4_while_stop={h4s} '
                f'lcs{{{",".join(f"{k}:{v}" for k, v in sorted(ls.items()))}}}')
      L.append(f' R4 H6 v<1: {band("<1.0")} | v>=1: {band(">=1.0")} lcs_flips={w.h6_lcs_flips}')
    h9 = ','.join(f'{n.split()[1] if n.startswith("driver") else "OPoff"}{relc((t - w.t0) / NS)}' for t, n in w.h9
                  if w.onset is None or t <= w.onset) or 'none'
    after = ','.join(f'{n.split()[1]}{relc((t - w.t0) / NS)}' for t, n in w.pedal_after) or 'none'
    off = f'{m["op_off"][1].replace(" ", "_")}{relc((m["op_off"][0] - w.t0) / NS)}' if m and m.get('op_off') else 'none'
    L.append(f' R9 H9 op_off={off} before_onset={h9} after={after}')
    f, a = hyp_tags(D, w)
    if is_unasked_roll(w) or category(w) == 'op_off_roll':
      L.append(f' VERDICT consistent_with={",".join(f) or "-"} not={",".join(a) or "-"}')
    else:
      L.append(f' VERDICT no_unasked_roll; patterns_seen={",".join(f) or "-"}')
    c = choose_compare(D, w)
    if c is not None:
      e = c.brm_exp
      hold = 'never' if c.hold_on is None else (f'{relc((c.hold_on - c.t0) / NS)}..' +
                                                 (relc((c.hold_off - c.t0) / NS) if c.hold_off else 'on'))
      L.append(f'COMPARE #{c.no} g={fnum(c.grade, "{:+.1f}")} {category(c)} b53={hold} hms={c.hms_seq} '
               f'brm_stand={fnum(c.brm_stand, "{:.1f}")}' + (f'(comp{e["comp"]:.0f}/not{e["uncomp"]:.0f})' if e else '')
               + f' class={c.brm_class or "-"} bar={fnum(c.bar_stand, "{:.1f}")} h8={c.h8} latch4='
               f'{relc((c.latch_t - c.t0) / NS) if c.latch_t else "-"}')
  rows = dict(D.h7['rows'])
  parts = []
  for src, r in D.h7['rows']:
    if r and r['per1000'] is not None:
      pr = f',{r["pairs"][0][0][0]}-{r["pairs"][0][0][1]}' if r['pairs'] else ''
      parts.append(f'{src_short(src)}={r["per1000"]:.0f}({r["frames"]}fr{pr})')
  L.append('H7 per1000 gradeStill(vEgo<0.1,|g|>=3): ' + (' '.join(parts) or 'no_frames') + f' -> {D.h7["kind"]}')
  _ = rows
  order = sorted(D.stops, key=lambda s: (0 if is_roll(s) else 1, -(abs(s.grade) if isnum(s.grade) else 0)))
  for st in order[:10]:
    L.append(paste_stop_line(D, st))
  if len(order) > 10:
    L.append(f'... +{len(order) - 10} more stops')
  for f in D.faults[:5]:
    ev = f.ev
    b29 = ev['bits'].get('b29')
    L.append(f'F#{f.no} t={clock((f.t0 - D.t_route0) / NS)} {f.before}->{f.value} v={fnum(f.speed, "{:.1f}")} '
             f'stop={f.during_stop or "-"} labels={"+".join(x.split(" (")[0].replace(" ", "_") for x in f.labels) or "none"} '
             f'hms2s={ev["hms_chg"]} lcs2s={ev["lcs_chg"]} a06={ev["a06_chg"]} gas={ev["gas_edges"]} brk={ev["brake_edges"]} '
             f'txblk={ev["txblk"] if ev["txblk"] is not None else "-"} rej={ev["rej"]} gap={ev["gap_ms"]:.0f}ms '
             f'b29={(f"{b29[0]:+.2f}" if b29 and b29[0] is not None else (b01(b29[1]) if b29 else "-"))}')
  if len(D.faults) > 5:
    L.append(f'... +{len(D.faults) - 5} more faults')
  return L


def print_paste(lines):
  say()
  say('===== PASTE THIS TO CLAUDE (start) =====')
  for ln in lines:
    say(ln)
  say('===== PASTE THIS TO CLAUDE (end) =====')


# ============================================================================================ csv
def check_csv_path(path, args):
  ap = os.path.abspath(path)
  for root in (args.log_root, args.op_dir, os.path.dirname(os.path.abspath(args.op_dir)) + '/params'):
    r = os.path.abspath(root)
    if ap == r or ap.startswith(r.rstrip('/') + '/'):
      raise UsageError(f'--csv {path}: please save it somewhere else, not inside {r} (the script never writes there).')
  if os.path.exists(ap):
    raise UsageError(f'--csv {path}: that file already exists. Pick a new name (the script never overwrites files).')
  if not os.path.isdir(os.path.dirname(ap)):
    raise UsageError(f'--csv {path}: the folder {os.path.dirname(ap)} does not exist.')
  return ap


def write_csv(D, path):
  np = D.np
  sv = D.s['vEgo']
  if not len(sv):
    return 0
  step = NS // 50
  T = np.arange(int(sv.t[0]), int(sv.t[-1]) + 1, step, dtype=np.int64)
  cols = [('vEgo', D.s['vEgo']), ('vEgoRaw', D.s['vEgoRaw']), ('aEgo', D.s['aEgo']), ('standstill', D.s['standstill']),
          ('all_wheels_zero', D.wz), ('wheel_pulses_median', D.pul), ('grade_pct', D.grade), ('mass_kg', D.mass),
          ('longActive', D.s['longActive']), ('ccEnabled', D.s['ccEnabled']), ('longControlState', D.s['lcs']),
          ('controlsState.longControlState', D.s['lcs_ctrl']), ('cc_accel', D.s['ccAccel']), ('pitch', D.s['pitch']),
          ('shouldStop', D.s['shouldStop']), ('aTarget', D.s['aTarget']), ('plan_speed0', D.s['vPlan0']),
          ('hasLead', D.s['hasLead']), ('lead_dRel', D.s['dRel']), ('lead_vLead', D.s['vLead']),
          ('brakePressed', D.s['brakePressed']), ('gasPressed', D.s['gasPressed']), ('gear', D.s['gear']),
          ('parkingBrake', D.s['parkingBrake']), ('accFaulted', D.s['accFaulted']), ('cruiseEnabled', D.s['cruiseEnabled']),
          ('panda.controlsAllowed', D.s['controlsAllowed']), ('panda.controlsAllowedLongitudinal', D.s['controlsAllowedLong']),
          ('panda.safetyTxBlocked', D.s['safetyTxBlocked'])]
  for key, msg, sname, _ in OP_SIGNALS:
    cols.append((f'sendcan.{msg}.{sname}', D.c[key]))
  for src in sorted(D.acc_src):
    for key, msg, sname, _ in OP_SIGNALS:
      if key in ('hms', 'acc06_stop', 'acc06_go', 'acc06_accel', 'acc07_stop', 'acc07_go', 'acc06_status'):
        cols.append((f'can_src{src}.{msg}.{sname}', D.acc_src[src][key]))
  for key, msg, sname, _ in CAR_SIGNALS:
    cols.append((f'{msg}.{sname}', D.c[key]))
  cols.append(('TSK_06.TSK_Radbremsmom_raw', D.c['tsk_brake_raw']))
  cols.append(('wheel_direction_validated', D.dir))
  cols.append(('wheel_direction_raw_vote', D.dir_raw))
  hms_t = np.asarray(hms_changes(D, int(T[0]) - step, int(T[-1])), dtype=np.int64)
  nchg = np.searchsorted(hms_t, T, side='right') - np.searchsorted(hms_t, T - step, side='right')
  rej = np.sort(np.concatenate([v[1] for v in D.echo.values()])) if D.echo else np.zeros(0, np.int64)
  nrej = np.searchsorted(rej, T, side='right') - np.searchsorted(rej, T - step, side='right')
  stop_no = np.zeros(len(T), dtype=np.int64)
  t_from = np.full(len(T), np.nan)
  for st in D.stops:
    mm = (T >= st.t0 - 3 * NS) & (T <= st.t_end + 3 * NS)
    stop_no[mm] = st.no
    t_from[mm] = (T[mm] - st.t0) / NS
  data = [c.at(T) for _, c in cols]
  segnum = np.full(len(T), -1, dtype=np.int64)
  for num, a, b, _ in D.seg_bounds:
    segnum[(T >= a) & (T <= b)] = num
  with open(path, 'x', newline='') as f:
    w = csv.writer(f)
    w.writerow(['drive_time_s', 'segment', 'logMonoTime_ns', 'stop_no', 't_from_stop_s', 'hms_changes_in_row',
                'panda_refused_in_row'] + [n for n, _ in cols])
    for i in range(len(T)):
      row = [f'{(T[i] - D.t_route0) / NS:.2f}', int(segnum[i]), int(T[i]), int(stop_no[i]) or '',
             '' if not isnum(t_from[i]) else f'{t_from[i]:.2f}', int(nchg[i]), int(nrej[i])]
      for (name, _), arr in zip(cols, data):
        x = arr[i]
        if not isnum(x):
          row.append('')
        elif name == 'gear':
          row.append(GEARS[int(x)])
        elif name.endswith('ongControlState'):
          row.append(lcs_name(x))
        else:
          row.append(int(x) if float(x).is_integer() else f'{x:.4f}')
      w.writerow(row)
  return len(T)


# ============================================================================================ main
def onroad_now():
  try:
    with open(os.path.join(PARAMS_DIR, 'IsOnroad'), 'rb') as f:
      return f.read().strip() == b'1'
  except OSError:
    return False


def build_parser():
  p = argparse.ArgumentParser(
    prog='mk7_hold_trace.py',
    description='READ-ONLY: what openpilot sent to the ESP/TSK and what they answered, around every stop (and any '
                'roll-back) of a drive, with the hill-hold decision rules. Default: the newest drive. '
                'Run it with the car OFF.',
    epilog=EPILOG, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument('--list', nargs='?', const=15, type=int, metavar='N', help='list the N newest drives (default 15) and stop')
  p.add_argument('--route', metavar='NAME', help='drive name from --list, or its number there (1 = newest)')
  p.add_argument('--segments', metavar='A-B', help='only read these segments (e.g. 3-5, 4, 6-)')
  p.add_argument('--event', type=int, metavar='N', help='show stop number N in detail')
  p.add_argument('--fault', type=int, metavar='N', help='show fault number N in detail')
  p.add_argument('--all-timelines', action='store_true', help='detail for every stop and every fault')
  p.add_argument('--before', type=float, default=3.0, metavar='S', help="timeline seconds before OP's stopping state (default 3)")
  p.add_argument('--after', type=float, default=5.0, metavar='S', help='timeline seconds after the wheels moved (default 5)')
  p.add_argument('--hz', type=int, default=10, choices=(10, 25, 50), help='timeline rows per second (default 10)')
  p.add_argument('--csv', metavar='FILE', help='also write every 50-per-second row to this NEW file')
  p.add_argument('--car-on', action='store_true', help='run even though the car is on (not recommended)')
  p.add_argument('--log-root', default=DEFAULT_LOG_ROOT, help=f'where the drive logs are (default {DEFAULT_LOG_ROOT})')
  p.add_argument('--op-dir', default=DEFAULT_OP_DIR, help=f'openpilot folder (default {DEFAULT_OP_DIR})')
  p.add_argument('--dbc', metavar='FILE', help='vw_mqb.dbc to decode with (default: the one inside --op-dir)')
  return p


def main(argv=None):
  p = build_parser()
  try:
    args = p.parse_args(argv)
  except SystemExit as e:
    return EXIT_OK if e.code in (0, None) else EXIT_USAGE
  try:
    return run(args)
  except UsageError as e:
    say(f'PROBLEM: {e}')
    return EXIT_USAGE
  except NoData as e:
    say(f'NOTHING TO SHOW: {e}')
    return EXIT_NO_DATA
  except KeyboardInterrupt:
    say('Stopped.')
    return EXIT_USAGE
  except BrokenPipeError:      # output piped into head/less and the reader quit: not an error
    try:
      os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())
    except OSError:
      pass
    return EXIT_OK


def run(args):
  if args.before < 0 or args.after <= 0 or args.before > 120 or args.after > 600:
    raise UsageError('--before must be 0-120 s and --after 0-600 s.')
  if args.event is not None and args.fault is not None:
    raise UsageError('Use --event or --fault, not both.')
  if args.list is not None and args.list < 1:
    raise UsageError('--list needs a positive number.')
  csv_path = check_csv_path(args.csv, args) if args.csv else None
  routes = discover_routes(args.log_root)
  if args.list is not None:
    if not routes:
      raise NoData(f'no drives found in {args.log_root}.')
    print_route_list(routes, args.list)
    return EXIT_OK
  if not routes:
    raise NoData(f'no drives found in {args.log_root}.')
  onroad = onroad_now()
  if onroad and not args.car_on:
    raise UsageError('the car is ON (openpilot is running). Please switch the ignition off and run this again: it reads '
                     'big log files and can use a few hundred MB of memory, which must not compete with openpilot. '
                     '(To run it anyway at low priority, add --car-on.)')
  route = resolve_route(routes, args.route) if args.route else routes[0]
  segs = route.segments
  if args.segments:
    a, b = parse_segments(args.segments)
    segs = [s for s in segs if a <= s.num <= b]
    if not segs:
      raise UsageError(f'Drive {route.name} has segments {route.segments[0].num}-{route.segments[-1].num}; '
                       f'none in {args.segments}.')
  rt = setup_runtime(args.op_dir)
  dbc_path = find_dbc(os.path.abspath(args.op_dir), args.dbc)
  dbc = Dbc(rt, dbc_path)
  for msg in ('ESP_21', 'ESP_33', 'TSK_06', 'Motor_16', 'ACC_06', 'ACC_07'):
    if msg not in dbc.msgs:
      raise UsageError(f'{dbc_path} has no {msg} message, so it is not the Volkswagen MQB DBC this script needs.')
  missing = [f'{m}.{n}' for _, m, n, req in CAR_SIGNALS + OP_SIGNALS if req and dbc.sig(m, n)[1] is None]
  if missing:
    raise UsageError(f'{dbc_path} is missing signals this script needs: {", ".join(missing)}.')
  if onroad:
    try:
      os.nice(15)        # lower priority for this process only, so openpilot is not slowed down
    except OSError:
      pass

  say(f'mk7_hold_trace v{TOOL_VERSION} (read-only) - drive {route.name}')
  if onroad:
    say('WARNING: the car is on (openpilot is running) and --car-on was given. Running at low priority; the newest '
        'minute may be cut short.')
  if not args.route:
    note = order_note(routes)
    if note:
      say(note)
  want = {dbc.msgs[m].addr for m in {x[1] for x in CAR_SIGNALS + OP_SIGNALS} if m in dbc.msgs}
  datas, used = [], []
  qlog_segs, bad_segs, notes = [], [], []
  t_start = time.time()
  tty = sys.stdout.isatty()
  for i, seg in enumerate(segs):
    if tty:
      sys.stdout.write(f'\r  reading segment {seg.num} ({i + 1} of {len(segs)}) ...')
      sys.stdout.flush()
    sd = read_segment(rt, seg, want)
    if sd.kind is None:
      bad_segs.append((seg.num, '; '.join(sd.problems) or 'unreadable'))
      continue
    if sd.kind == 'qlog':
      qlog_segs.append(seg.num)
    if sd.problems:
      notes.append(f'segment {seg.num}: ' + '; '.join(sd.problems) + f' -> used the {sd.kind}')
    if sd.damaged:
      notes.append(f'segment {seg.num}: file cut off or damaged' + (' (it was still being recorded)' if seg.recording else '')
                   + ', used the readable part')
    if sd.how == 'LogReader':
      notes.append(f'segment {seg.num}: read with openpilot\'s LogReader (the streaming reader found nothing)')
    datas.append(sd)
    used.append(seg)
  if tty:
    sys.stdout.write('\r' + ' ' * 60 + '\r')
    sys.stdout.flush()
  if not datas:
    raise NoData('none of the segments could be read: ' + '; '.join(f'{n}: {w}' for n, w in bad_segs))
  D = build_drive(rt, dbc, datas, used)
  del datas
  gc.collect()

  # ---- header
  segtxt = f'segments {used[0].num}-{used[-1].num}' if len(used) > 1 else f'segment {used[0].num}'
  say(f'Read {segtxt} of {route.segments[0].num}-{route.segments[-1].num} in {time.time() - t_start:.0f} s '
      f'using the {rt.reader_name}.')
  if D.init:
    br, cm, dirty, ver = D.init
    say(f'Logged by openpilot {ver or "?"}, branch {br or "?"}, commit {(cm or "?")[:10]}{" (modified)" if dirty else ""}.')
  for n, w in bad_segs:
    say(f'WARNING: segment {n} skipped: {w}')
  for nt in notes:
    say(f'Note: {nt}')
  if qlog_segs:
    which = 'ALL segments' if len(qlog_segs) == len(used) else f'segment(s) {", ".join(map(str, qlog_segs))}'
    say(f'WARNING: only the small qlog exists for {which}. A qlog keeps 1 in {QLOG_DECIM["can"]} car-CAN messages and 1 in '
        f'{QLOG_DECIM["sendcan"]} sendcan messages (cereal/services.py), so the ESP and HMS columns are mostly "-" there. '
        f'Stops are still found from carState (1 in {QLOG_DECIM["carState"]} kept, about 10 per second).')
  n_ok, problems = cross_check(dbc)
  say(f'Decoding: bit positions read from {os.path.relpath(dbc_path, os.path.abspath(args.op_dir)) if dbc_path.startswith(os.path.abspath(args.op_dir)) else dbc_path} '
      f'with {dbc.how}.')
  if problems:
    for pr in problems:
      say(f'WARNING: DBC does not match your verified list: {pr}. Using the DBC.')
  else:
    say(f'  All {n_ok} signals on your verified list match the DBC (ESP_33 bits 28/29, ESP_21 bit 53, '
        'ACC_07 bits 28-30, TSK_06 bits 24-26, 0x65F byte 7).')
  agree = decoder_self_check(rt, D)
  if agree:
    say(f'  {agree}')
  for mm in D.missing:
    say(f'Note: {mm}; that column stays "-".')
  if D.has_can:
    say(f'Car CAN found on bus {D.pt_bus} (ESP_21 count per bus: '
        + ', '.join(f'{b}: {c}' for b, c in sorted(D.esp21_counts.items())) + ').')
    odd = sorted({f'{k} from bus {v[1]}' for k, v in D.sig_src.items() if v[0] == 'can' and v[1] != D.pt_bus})
    if odd:
      say('  Some messages only appeared on another bus: ' + ', '.join(odd))
    say(f'Wheel direction: {D.dir_info}.')
    say(f'Wheel pulses: {D.pulse_info}.')
    say(f'Standstill moment (+0.00): {D.anchor_how}.')
    if not D.has_tsk:
      say('WARNING: TSK_06 (0x120, cruise status and wheel brake torque) is NOT in the log: TSK faults and the H4/H5 '
          'checks cannot be done.')
    if len(D.grade) and D.grade_invalid_frac > 0.5:
      say(f'Note: the grade signal was invalid {D.grade_invalid_frac * 100:.0f}% of the time.')
    elif not len(D.grade):
      say('Note: no grade signal (0x65F) in the log.')
  else:
    say('WARNING: no car CAN data (ESP_21 / TSK_06) in these logs, so nothing about the ESP side can be shown.')
  if D.has_sendcan:
    say(f'OP commands found in sendcan bus {D.op_bus}.' + ('' if D.has_echo else ' (No panda echo frames in the log.)'))
  else:
    say('WARNING: no ACC_07 in sendcan: what OP sent cannot be shown (openpilot long control off, or qlog only).')
  if D.acc_src:
    say('ACC_06/ACC_07 in the car CAN log, decoded per source (never mixed): ' +
        '; '.join(f'{src_label(s, D.pt_bus)}: {", ".join(f"{m} {n}" for m, n in sorted(D.acc_src_counts[s].items()))}'
                  for s in sorted(D.acc_src)) + '.')
  extras = []
  extras.append('pandaStates ' + ('yes' if D.has_panda else 'NOT in the log (safetyTxBlocked unknown)'))
  extras.append('controlsState ' + ('yes' if D.has_ctl else 'not in the log (long state from carControl)'))
  extras.append('longitudinalPlan ' + ('yes' if D.has_plan else 'NOT in the log (requests judged from carControl only)'))
  extras.append('orientationNED pitch ' + ('yes' if D.has_pitch else 'no'))
  say('openpilot messages: ' + ', '.join(extras) + '.')
  if not len(D.s['vEgo']):
    raise NoData('these logs have no carState (speed) messages.')

  analyse(D)
  mm = [x for x in D.stops if x.pul_mm]
  if mm:
    say('WARNING: wheel-pulse MISMATCH: the ESP_10 pulse counters showed (almost) no movement while the wheel speed '
        'clearly did (' + ', '.join(f'stop #{x.no}: pulses {x.pul_mm[2]:.2f} m, speed {x.pul_mm[3]:.2f} m' for x in mm[:4])
        + '). Distances there are taken from the wheel speed; the counters may stop or misbehave at low speed.')
  print_bottom_line(D)
  print_summary(D)
  print_faults(D)
  print_route_stats(D)
  print_h7(D)
  print_decisions_all(D)

  # ---- details
  shown = False
  if args.all_timelines:
    for st in D.stops:
      print_stop_detail(D, st, args)
    for f in D.faults:
      print_fault_detail(D, f, args)
    shown = bool(D.stops or D.faults)
  elif args.event is not None:
    if not D.stops:
      raise UsageError('there are no OP stops in this drive, so --event cannot be used.')
    st = next((s for s in D.stops if s.no == args.event), None)
    if st is None:
      raise UsageError(f'there is no stop #{args.event}; this drive has stops 1-{len(D.stops)}.')
    print_stop_detail(D, st, args)
    shown = True
  elif args.fault is not None:
    if not D.faults:
      raise UsageError('there are no TSK faults in this drive, so --fault cannot be used.' if D.has_tsk else
                       'TSK_06 is not in this log, so faults (and --fault) cannot be checked.')
    f = next((x for x in D.faults if x.no == args.fault), None)
    if f is None:
      raise UsageError(f'there is no fault F{args.fault}; this drive has F1-F{len(D.faults)}.')
    print_fault_detail(D, f, args)
    shown = True
  else:
    st = choose_default(D)
    if st is not None:
      say()
      why = {'rolled_back': 'the steepest stop where the car rolled back',
             'possible_roll': 'the steepest stop with a possible rollback',
             'op_off_roll': 'the steepest roll (after OP was switched off)',
             'rolled_fwd': 'the steepest stop where the car rolled forward',
             'moved': 'the steepest stop where the car moved without being asked',
             'slip': 'the steepest launch slip; no stop rolled',
             'held': 'the steepest stop; no stop rolled'}[category(st)]
      say(f'Showing stop #{st.no} in detail ({why}). Pick another with --event N, or --all-timelines.')
      print_stop_detail(D, st, args)
      c = choose_compare(D, st)
      if c is not None and is_roll(st):
        print_compare(D, st, c, args)
      shown = True
    elif D.faults:
      say()
      say('No OP stops; showing the first fault in detail (others: --fault N).')
      print_fault_detail(D, D.faults[0], args)
      shown = True
  if shown:
    say()
    for ln in legend_lines():
      say(ln)
  if csv_path:
    n = write_csv(D, csv_path)
    say()
    say(f'Saved {n} rows (50 per second) to {csv_path}')
  print_paste(build_paste(D, args, route, used, n_ok, problems))
  if not D.stops and not D.faults:
    raise NoData('no stops with openpilot in control and no TSK faults in ' + segtxt + ' of this drive.'
                 + (f' ({plural(D.other_stops, "stop")} without OP.)' if D.other_stops else ''))
  return EXIT_OK


def decoder_self_check(rt, D):
  """Compare this script's fast bit decoding with openpilot's own get_raw_value on real frames from the log."""
  if rt.odbc_get_raw is None or not D.decode_checks:
    return ''
  np = rt.np
  n = bad = 0
  for words, sdef in D.decode_checks:
    mine = decode_raw(np, words, sdef)
    for w, v in zip(words.tolist(), mine.tolist()):
      dat = int(w).to_bytes(8, 'little')
      try:
        theirs = rt.odbc_get_raw(dat, _OdbcSig(sdef))
      except Exception:
        return ''
      if sdef.signed and theirs >= (1 << (sdef.size - 1)):
        theirs -= 1 << sdef.size
      n += 1
      bad += int(theirs != v)
  if not n:
    return ''
  if bad:
    return f'WARNING: this script\'s decoding differs from openpilot\'s CAN parser on {bad} of {n} checked values.'
  return f"Decoding agrees with openpilot's own CAN parser on all {n} values checked from this drive."


class _OdbcSig:
  def __init__(self, s):
    self.lsb, self.msb, self.size, self.is_little_endian = s.lsb, s.msb, s.size, s.le


if __name__ == '__main__':
  sys.exit(main())
