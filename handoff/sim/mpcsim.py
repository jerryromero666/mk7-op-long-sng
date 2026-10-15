"""
Closed-loop longitudinal sim for the Mk7 GTI branches.

Runs the REAL long_mpc.py (from any git ref) against the device's own c_generated_code,
compiled for x86 (cgen/). Stubs only imports that don't affect the math (cereal enums,
swaglog, radard constant). Planner / longcontrol / carcontroller logic is a line-by-line
replay of selfdrive/controls/lib/longitudinal_planner.py, longcontrol.py and the VW
carcontroller clip, with constants read from the same git ref.

Plant: requested accel -> pure delay + first-order lag -> velocity. Brake holds at v=0.
Absolute numbers depend on that plant model; compare variants against baseline.
"""
import ast
import importlib.util
import os
import re
import subprocess
import sys
import types
from types import SimpleNamespace as NS

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = "/home/user/mk7-op-long-sng"
MPC_PATH = "selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py"
PLANNER_PATH = "selfdrive/controls/lib/longitudinal_planner.py"
CC_PATH = "opendbc_repo/opendbc/car/volkswagen/carcontroller.py"

# ---------------------------------------------------------------- stubs

def _stub(name, **attrs):
  m = sys.modules.get(name) or types.ModuleType(name)
  m.__dict__.update(attrs)
  m.__path__ = []
  sys.modules[name] = m
  return m

class LongitudinalPersonality:
  relaxed = 0
  standard = 1
  aggressive = 2

class LongitudinalPlanSource:
  lead0 = 0
  lead1 = 1
  cruise = 2
  e2e = 3

_stub("cereal", log=NS(LongitudinalPersonality=LongitudinalPersonality,
                       LongitudinalPlan=NS(LongitudinalPlanSource=LongitudinalPlanSource)))
_stub("opendbc"); _stub("opendbc.car")
_stub("opendbc.car.interfaces", ACCEL_MIN=-3.5, ACCEL_MAX=2.5)   # real values, interfaces.py:28-29
_stub("openpilot"); _stub("openpilot.common")
_stub("openpilot.common.realtime", DT_MDL=0.05, DT_CTRL=0.01)
_stub("openpilot.common.swaglog", cloudlog=NS(warning=lambda *a, **k: None, info=lambda *a, **k: None,
                                              error=lambda *a, **k: None))
_stub("openpilot.selfdrive"); _stub("openpilot.selfdrive.modeld")
_spec = importlib.util.spec_from_file_location("openpilot.selfdrive.modeld.constants",
                                               os.path.join(REPO, "selfdrive/modeld/constants.py"))
_mc = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_mc)
sys.modules["openpilot.selfdrive.modeld.constants"] = _mc
_stub("openpilot.selfdrive.controls")
_stub("openpilot.selfdrive.controls.radard", _LEAD_ACCEL_TAU=1.5)   # radard.py:21
_stub("openpilot.selfdrive.controls.lib")
_stub("openpilot.selfdrive.controls.lib.longitudinal_mpc_lib")
_stub("openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code")
import ctypes
for _lib in ("libblasfeo.so", "libhpipm.so", "libqpOASES_e.so.3.1", "libacados.so"):
  ctypes.CDLL(os.path.join(REPO, "third_party/acados/x86_64/lib", _lib), mode=ctypes.RTLD_GLOBAL)
_ext = [f for f in os.listdir(os.path.join(HERE, "cgen")) if f.startswith("acados_ocp_solver_pyx") and f.endswith(".so")][0]
_spec = importlib.util.spec_from_file_location(
  "openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx",
  os.path.join(HERE, "cgen", _ext))
_pyx = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_pyx)
sys.modules[_spec.name] = _pyx

# ---------------------------------------------------------------- loading a ref

def git_show(ref, path):
  if ref is None:
    return open(os.path.join(REPO, path)).read()
  return subprocess.check_output(["git", "-C", REPO, "show", f"{ref}:{path}"], text=True)

_load_count = 0
def load_mpc_module(ref=None, src=None, overrides=None):
  """Exec long_mpc.py from a git ref (or raw source) as a fresh module."""
  global _load_count
  _load_count += 1
  src = src if src is not None else git_show(ref, MPC_PATH)
  name = f"long_mpc_sim_{_load_count}"
  mod = types.ModuleType(name)
  mod.__file__ = os.path.join(REPO, MPC_PATH)
  exec(compile(src, f"{ref}:{MPC_PATH}", "exec"), mod.__dict__)
  for k, v in (overrides or {}).items():
    assert hasattr(mod, k), k
    setattr(mod, k, v)
  return mod

def _const(src, name):
  tree = ast.parse(src)
  for node in tree.body:
    if isinstance(node, ast.Assign) and any(getattr(t, "id", None) == name for t in node.targets):
      return ast.literal_eval(node.value)
  for node in ast.walk(tree):   # class attributes
    if isinstance(node, ast.Assign) and any(getattr(t, "id", None) == name for t in node.targets):
      try:
        return ast.literal_eval(node.value)
      except ValueError:
        pass
  raise KeyError(name)

def load_ref(ref=None, mpc_overrides=None, planner_overrides=None, cc_overrides=None):
  mpc_mod = load_mpc_module(ref, overrides=mpc_overrides)
  psrc = git_show(ref, PLANNER_PATH)
  ccsrc = git_show(ref, CC_PATH)
  cfg = dict(
    A_CRUISE_MAX_VALS=_const(psrc, "A_CRUISE_MAX_VALS"),
    A_CRUISE_MAX_BP=_const(psrc, "A_CRUISE_MAX_BP"),
    UPHILL_ACCEL_FLOOR=_const(ccsrc, "UPHILL_ACCEL_FLOOR"),
  )
  cfg.update(planner_overrides or {})
  cfg.update(cc_overrides or {})
  return mpc_mod, cfg

# ---------------------------------------------------------------- car params (VW interface.py + interfaces.py defaults)

VW = NS(stopAccel=-1.2, vEgoStarting=0.1, vEgoStopping=0.5, longitudinalActuatorDelay=0.5,
        stoppingDecelRate=6.0, startingState=False)
ACCEL_MIN, ACCEL_MAX = -3.5, 2.5          # interfaces.py (MPC a_max, pid limits)
CCP_ACCEL_MIN, CCP_ACCEL_MAX = -3.5, 2.0  # volkswagen/values.py CarControllerParams
DT_MDL, DT_CTRL = 0.05, 0.01
CONTROL_N = 17
CONTROL_N_T_IDX = np.array(_mc.ModelConstants.T_IDXS[:CONTROL_N])

def get_accel_from_plan(speeds, accels, t_idxs, action_t, vEgoStopping):   # drive_helpers.py:42
  v_now = speeds[0]; a_now = accels[0]
  v_target = np.interp(action_t, t_idxs, speeds)
  a_target = 2 * (v_target - v_now) / action_t - a_now
  should_stop = (v_now < vEgoStopping and a_target < 0.1)
  return a_target, should_stop

# ---------------------------------------------------------------- planner replay (longitudinal_planner.py:89-175, no e2e/SCC)

class Planner:
  def __init__(self, mpc_mod, cfg, personality=LongitudinalPersonality.standard, init_v=0.0):
    self.m = mpc_mod
    self.cfg = cfg
    self.mpc = mpc_mod.LongitudinalMpc(dt=DT_MDL)
    self.personality = personality
    self.a_desired = 0.0
    self.v_f_x = init_v
    self.alpha = DT_MDL / (2.0 + DT_MDL)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False

  def get_max_accel(self, v):
    return np.interp(v, self.cfg["A_CRUISE_MAX_BP"], self.cfg["A_CRUISE_MAX_VALS"])

  def update(self, v_ego, standstill, radarstate, v_cruise):
    prev_accel_constraint = not standstill
    accel_clip = [ACCEL_MIN, self.get_max_accel(v_ego)]
    # limit_accel_in_turns (longitudinal_planner.py:36-47) with a straight wheel: caps at
    # interp(v, [20, 40], [1.7, 3.2]) even at zero steering angle (added after review)
    accel_clip[1] = min(accel_clip[1], float(np.interp(v_ego, [20., 40.], [1.7, 3.2])))
    self.v_f_x = max(0.0, (1 - self.alpha) * self.v_f_x + self.alpha * v_ego)
    self.mpc.set_weights(prev_accel_constraint, personality=self.personality)
    self.mpc.set_cur_state(self.v_f_x, self.a_desired)
    self.mpc.update(radarstate, v_cruise, personality=self.personality)
    T = self.m.T_IDXS
    v_traj = np.interp(CONTROL_N_T_IDX, T, self.mpc.v_solution)
    a_traj = np.interp(CONTROL_N_T_IDX, T, self.mpc.a_solution)
    a_prev = self.a_desired
    self.a_desired = float(np.interp(DT_MDL, CONTROL_N_T_IDX, a_traj))
    self.v_f_x = self.v_f_x + DT_MDL * (self.a_desired + a_prev) / 2.0
    action_t = VW.longitudinalActuatorDelay + DT_MDL
    a_t, ss = get_accel_from_plan(v_traj, a_traj, CONTROL_N_T_IDX, action_t, VW.vEgoStopping)
    for i in range(2):
      accel_clip[i] = np.clip(accel_clip[i], self.prev_accel_clip[i] - 0.05, self.prev_accel_clip[i] + 0.05)
    self.output_a_target = float(np.clip(a_t, accel_clip[0], accel_clip[1]))
    self.prev_accel_clip = accel_clip
    self.output_should_stop = ss
    return self.output_a_target, ss

# ---------------------------------------------------------------- longcontrol replay (longcontrol.py, kp=ki=0 -> feedforward)

STOPPING, PID = "stopping", "pid"

class LongControl:
  def __init__(self):
    self.state = STOPPING
    self.last = 0.0

  def update(self, v_ego, a_target, should_stop):
    starting_condition = not should_stop     # cruise_standstill hardwired False under OP long (trap #1)
    if self.state == STOPPING:
      if starting_condition:
        self.state = PID
    else:
      if should_stop:
        self.state = STOPPING
    if self.state == STOPPING:
      out = self.last
      if out > VW.stopAccel:
        out = min(out, 0.0)
        out -= VW.stoppingDecelRate * DT_CTRL
    else:
      out = a_target
    self.last = float(np.clip(out, ACCEL_MIN, ACCEL_MAX))
    return self.last

# ---------------------------------------------------------------- plant

class Plant:
  def __init__(self, v0=0.0, delay=0.15, tau=0.35):
    self.v = v0
    self.x = 0.0
    self.a = 0.0
    self.buf = [0.0] * max(1, int(round(delay / DT_CTRL)))
    self.tau = tau

  def step(self, a_cmd, held=False):
    self.buf.append(a_cmd)
    a_req = self.buf.pop(0)
    self.a += (a_req - self.a) * DT_CTRL / self.tau
    if held:
      self.v = 0.0   # ESP hold: drive torque builds, car does not move until release
    elif self.v <= 0.0 and self.a <= 0.0:
      self.v = 0.0; self.a = min(self.a, 0.0)   # brake/ESP hold
    else:
      self.v = max(0.0, self.v + self.a * DT_CTRL)
    self.x += self.v * DT_CTRL

# ---------------------------------------------------------------- lead

def lead_msg(dRel, vLead, aLead, status=True):
  return NS(status=status, dRel=float(dRel), vLead=float(vLead), aLeadK=float(aLead), aLeadTau=1.5, modelProb=1.0)

NO_LEAD = NS(status=False, dRel=0.0, vLead=0.0, aLeadK=0.0, aLeadTau=1.5, modelProb=0.0)

# ---------------------------------------------------------------- closed loop

def run(mpc_mod, cfg, T, lead_fn, v0=0.0, gap0=6.0, v_cruise=35.0,
        personality=LongitudinalPersonality.standard, noise=None, hold_release_s=0.0, seed=0,
        record=False, lead2='mirror', plant_delay=0.15, plant_tau=0.35):
  """lead_fn(t, x_lead, v_lead) -> (a_lead) ; lead starts gap0 ahead at speed of lead_fn's v_init."""
  rng = np.random.default_rng(seed)
  plant = Plant(v0, plant_delay, plant_tau)
  planner = Planner(mpc_mod, cfg, personality, init_v=v0)
  lc = LongControl()
  lc.state = STOPPING if v0 < 0.1 else PID
  lx, lv = gap0, lead_fn.v_init
  a_target, should_stop = 0.0, v0 < 0.1
  a_send = 0.0
  hold_frames = 0
  out = NS(t=[], v=[], gap=[], a_cmd=[], should_stop=[], state=[], a_target=[], lv=[], crash_cnt=[])
  prev_state = lc.state
  stop_to_pid = 0
  n = int(round(T / DT_CTRL))
  for k in range(n):
    t = k * DT_CTRL
    la = lead_fn(t, lx, lv)
    lv = max(0.0, lv + la * DT_CTRL)
    lx += lv * DT_CTRL
    gap = lx - plant.x
    if k % 5 == 0:
      d, vl = gap, lv
      if noise is not None:
        d += rng.normal(0, noise[0]); vl += rng.normal(0, noise[1])
      l1 = lead_msg(d, vl, la)
      rs = NS(leadOne=l1, leadTwo=l1 if lead2 == 'mirror' else NO_LEAD)
      a_target, should_stop = planner.update(plant.v, plant.v == 0.0, rs, v_cruise)
    a_out = lc.update(plant.v, a_target, should_stop)
    if lc.state != prev_state and prev_state == STOPPING:
      stop_to_pid += 1
    prev_state = lc.state
    # ESP hold model: confirmed while stopped in STOPPING; after OP leaves STOPPING the ESP
    # takes hold_release_s to let go. Floor applies only while hold is still confirmed.
    if plant.v == 0.0 and lc.state == STOPPING:
      hold_frames = int(round(hold_release_s / DT_CTRL))
    elif hold_frames > 0:
      hold_frames -= 1
    held = hold_frames > 0 and plant.v == 0.0
    # carcontroller (50 Hz): clip to CCP limits, then launch floor while hold confirmed
    if k % 2 == 0:
      a_cc = float(np.clip(a_out, CCP_ACCEL_MIN, CCP_ACCEL_MAX))
      if a_cc > 0 and held and plant.v < 1.0:
        a_cc = min(max(a_cc, cfg["UPHILL_ACCEL_FLOOR"]), CCP_ACCEL_MAX)
      a_send = a_cc
    plant.step(a_send, held)
    if record and k % 5 == 0:
      out.t.append(t); out.v.append(plant.v); out.gap.append(gap); out.a_cmd.append(a_send)
      out.should_stop.append(should_stop); out.state.append(lc.state); out.a_target.append(a_target)
      out.lv.append(lv); out.crash_cnt.append(planner.mpc.crash_cnt)
  out.stop_to_pid = stop_to_pid
  out.final_gap = lx - plant.x
  out.final_v = plant.v
  return out

# ---------------------------------------------------------------- lead profiles

def lead_const(v):
  f = lambda t, x, vl: 0.0
  f.v_init = v
  return f

def lead_launch(t0=5.0, a=2.0, v_max=13.4):
  def f(t, x, vl):
    return a if (t >= t0 and vl < v_max) else 0.0
  f.v_init = 0.0
  return f

def lead_brake(v0, t0, decel, v_end=0.0):
  def f(t, x, vl):
    return -decel if (t >= t0 and vl > v_end) else 0.0
  f.v_init = v0
  return f
