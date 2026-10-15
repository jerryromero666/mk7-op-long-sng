#!/usr/bin/env python3
"""Frame-by-frame replay of the VW MQB ACC_06/ACC_07 standstill path for several git refs.

Pulls from each ref with `git show` (no checkout):
  - MQBStandstillManager            (opendbc_repo/opendbc/car/volkswagen/carcontroller.py)
  - the ACC block of CarController.update, exec'd verbatim (same text on all refs, md5 checked)
  - acc_control_value, create_acc_accel_control (mqbcan.py)
  - long_control_state_trans, LongControl    (selfdrive/controls/lib/longcontrol.py)
and drives them at 100 Hz (controlsd) with the ACC block running on even frames (50 Hz),
exactly as on the device. Stub CS/CC/CP, numpy only.

controlsd ordering reproduced: actuators.longControlState = LoC.long_control_state (state from the
PREVIOUS frame), then actuators.accel = LoC.update(...) (this frame).
CarState: vEgo/aEgo from the same 2-state Kalman filter as CarStateBase.update_speed_kf,
standstill = (vEgoRaw == 0), cruiseState.standstill = pcmCruise(False) and esp_hold -> False.
"""
import ast
import subprocess
import sys
import textwrap
import types
import numpy as np

REPO = "/home/user/mk7-op-long-sng"
REFS = ["hms_latch_fix", "claude/determined-hypatia-pnidyz", "hold_tsk_owned"]
SHORT = {"hms_latch_fix": "HLF", "claude/determined-hypatia-pnidyz": "PAR", "hold_tsk_owned": "T0"}
CC_PATH = "opendbc_repo/opendbc/car/volkswagen/carcontroller.py"
CAN_PATH = "opendbc_repo/opendbc/car/volkswagen/mqbcan.py"
LC_PATH = "selfdrive/controls/lib/longcontrol.py"
DT_CTRL = 0.01


def git_show(ref, path):
  return subprocess.check_output(["git", "-C", REPO, "show", f"{ref}:{path}"], text=True)


def extract_top(src, name):
  tree = ast.parse(src)
  for node in tree.body:
    if isinstance(node, (ast.ClassDef, ast.FunctionDef)) and node.name == name:
      return ast.get_source_segment(src, node)
  raise KeyError(name)


def extract_acc_block(src):
  lines = src.split("\n")
  start = next(i for i, l in enumerate(lines) if l.strip() == "if self.CP.openpilotLongitudinalControl:")
  end = next(i for i in range(start, len(lines)) if lines[i].strip().startswith("#if self.aeb_available"))
  return textwrap.dedent("\n".join(lines[start:end]))


class LCS:  # LongControlState stub
  off = "off"; pid = "pid"; stopping = "stopping"; starting = "starting"


class PIDStub:
  """VW kpV = kiV = [0.] -> PIDController output = clip(feedforward, neg_limit, pos_limit)."""
  def __init__(self, *a, **k):
    self.neg_limit, self.pos_limit = -1e9, 1e9
  def reset(self):
    pass
  def update(self, error, speed=0.0, feedforward=0.0, **k):
    return float(np.clip(feedforward, self.neg_limit, self.pos_limit))


class Packer:
  def make_can_msg(self, name, bus, values):
    return (name, bus, dict(values))


def ns(**kw):
  return types.SimpleNamespace(**kw)


CP = ns(vEgoStopping=0.5, vEgoStarting=0.1, stopAccel=-1.2, stoppingDecelRate=6.0, startingState=False,
        startAccel=0.0, openpilotLongitudinalControl=True,
        longitudinalTuning=ns(kpBP=[0.], kpV=[0.], kiBP=[0.], kiV=[0.]))
CP_SP = ns(enableGasInterceptor=False)
CCP = ns(ACCEL_MAX=2.0, ACCEL_MIN=-3.5, ACC_CONTROL_STEP=2)


def load_ref(ref):
  cc_src = git_show(ref, CC_PATH)
  can_src = git_show(ref, CAN_PATH)
  lc_src = git_show(ref, LC_PATH)
  g = {"np": np, "DT_CTRL": DT_CTRL, "LongCtrlState": LCS, "PIDController": PIDStub,
       "CONTROL_N_T_IDX": None, "car": None}
  exec(extract_top(lc_src, "long_control_state_trans"), g)
  exec(extract_top(lc_src, "LongControl"), g)
  mq = types.ModuleType("mqbcan")
  exec(extract_top(can_src, "acc_control_value"), mq.__dict__)
  exec(extract_top(can_src, "create_acc_accel_control"), mq.__dict__)
  mg = {"np": np}
  exec(extract_top(cc_src, "MQBStandstillManager"), mg)
  acc_block = extract_acc_block(cc_src)
  return {"LongControl": g["LongControl"], "mqbcan": mq, "Mgr": mg["MQBStandstillManager"],
          "acc_block": compile(acc_block, f"<{ref}:acc_block>", "exec"), "acc_text": acc_block,
          "timeout": mg["MQBStandstillManager"].HOLD_REQUEST_TIMEOUT}


class SpeedKF:
  """Steady-state gain of CarStateBase's KF1D (Q=[[0,0],[0,100]], R=0.3, dt=0.01)."""
  def __init__(self):
    A = np.array([[1.0, DT_CTRL], [0.0, 1.0]]); C = np.array([[1.0, 0.0]])
    Q = np.array([[0.0, 0.0], [0.0, 100.0]]); R = 0.3
    P = np.eye(2)
    for _ in range(5000):
      P = A @ P @ A.T + Q
      S = C @ P @ C.T + R
      K = P @ C.T / S
      P = (np.eye(2) - K @ C) @ P
    self.A, self.C, self.K = A, C, K
    self.x = np.zeros((2, 1))
  def update(self, z):
    if abs(z - self.x[0, 0]) > 2.0:
      self.x = np.array([[z], [0.0]])
    xp = self.A @ self.x
    self.x = xp + self.K * (z - (self.C @ xp)[0, 0])
    return float(self.x[0, 0]), float(self.x[1, 0])


def run(ref_mod, scen):
  """scen: dict of per-100Hz-frame arrays: t, vraw, should_stop, a_target, brake, long_active, esp_hold."""
  LoC = ref_mod["LongControl"](CP, CP_SP)
  mgr = ref_mod["Mgr"](CCP)
  mq = ref_mod["mqbcan"]
  kf = SpeedKF()
  selfo = ns(CP=CP, CCP=CCP, CCS=mq, standstill_manager=mgr, packer_pt=Packer(), CAN=ns(pt=0), frame=0)
  rows = []
  N = len(scen["t"])
  for i in range(N):
    vraw = float(scen["vraw"][i])
    vEgo, aEgo = kf.update(vraw)
    esp_hold = bool(scen["esp_hold"][i])
    out = ns(vEgo=vEgo, aEgo=aEgo, vEgoRaw=vraw, standstill=(vraw == 0.0), brakePressed=bool(scen["brake"][i]),
             accFaulted=False, cruiseState=ns(available=True, standstill=False))
    CS = ns(out=out, esp_hold_confirmation=esp_hold, acc_type=1)
    # controlsd
    long_active = bool(scen["long_active"][i])
    actuators = ns(longControlState=LoC.long_control_state)
    lcs_cs = LoC.long_control_state
    if not long_active:
      LoC.reset()
    actuators.accel = float(LoC.update(long_active, out, float(scen["a_target"][i]), bool(scen["should_stop"][i]),
                                       (CCP.ACCEL_MIN, CCP.ACCEL_MAX)))
    CC = ns(longActive=long_active, actuators=actuators)
    # carcontroller ACC block (verbatim), runs when frame % 2 == 0
    loc = {"self": selfo, "CC": CC, "CS": CS, "actuators": actuators, "can_sends": [], "LongCtrlState": LCS,
           "np": np, "mqbcan": mq}
    exec(ref_mod["acc_block"], loc)
    if loc["can_sends"]:
      a6 = next(v for n, b, v in loc["can_sends"] if n == "ACC_06")
      a7 = next(v for n, b, v in loc["can_sends"] if n == "ACC_07")
      rows.append(dict(i=i, t=float(scen["t"][i]), vraw=vraw, v=vEgo, lcs=lcs_cs, la=long_active, brk=out.brakePressed,
                       hold=esp_hold, stopping=bool(loc["stopping"]), starting=bool(loc["starting"]),
                       eso=loc["esp_starting_override"], est=loc["esp_stopping_override"],
                       st=a6["ACC_Status_ACC"], a6_stp=bool(a6["ACC_Anhalten"]), a6_go=bool(a6["ACC_Anfahren"]),
                       acc=float(a6["ACC_Sollbeschleunigung_02"]), hms=int(a7["ACC_Anforderung_HMS"]),
                       a7_stp=bool(a7["ACC_Anhalten"]), a7_go=bool(a7["ACC_Anfahren"]),
                       dist=float(a7["ACC_Anhalteweg"]), latched=bool(getattr(mgr, "_hms4_latched", False)),
                       reqf=int(mgr._hold_request_frames)))
    selfo.frame += 1
  return rows
