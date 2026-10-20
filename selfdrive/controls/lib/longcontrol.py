import numpy as np
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState

# Soft stop. In the stopping state the stock request moves from what the planner handed over (about -0.3..-0.45 at
# 0.9-1.1 m/s on the Mk7) to stopAccel -1.2 at stoppingDecelRate 6 m/s3: 0.13 s, and the wheels grab at the end of
# every stop (real drives 343-347: 21 of 21 stops, -0.41 -> -1.23, wheels -1.7 m/s3 median, -6.8 worst). With soft
# stop, while the car still rolls the request moves at the soft stop rate toward the planner's stop request (the
# decel that stops the car where the plan wants it: longitudinal_planner.py stop placement), never weaker than
# SOFT_MIN nor than what stops the car within SOFT_DIST, never stronger than stopAccel, and it may ease back (at
# SOFT_EASE_RATE) when the stop request eases, but never from a stronger brake it was handed. Once the wheels stop
# the stock ramp to stopAccel holds the car (nothing moves, nothing to feel), so the standstill hold, the HMS / TSK
# handling and the launch are unchanged. A planner request stronger than SOFT_URGENT (a car cutting in, a lead
# braking hard in the last metres) gives the stock ramp. More than SOFT_MAX_T s rolling in the stopping state (a car
# slow to answer the soft request, a steep downhill) moves the request on to stopAccel, but at the soft stop rate:
# a long soft stop must not end in the stock 0.13 s snap (sim: 4-8 % of the soft stops lasted over 3 s).
# Live: /data/stop_soft holds the soft stop rate in m/s3 (default 1.5, clamped SOFT_RATE_LIMITS); no file or an
# unreadable file = the default; "0" (or any number <= 0) = off = the stock stopping ramp, bit for bit.
SOFT_FILE = '/data/stop_soft'
SOFT_DEFAULT = 1.5
SOFT_RATE_LIMITS = (0.5, 6.0)  # m/s3
SOFT_PERIOD = 100              # updates between file reads (1 s at 100 Hz)
SOFT_MIN = 0.4                 # m/s2
SOFT_DIST = 1.0                # m
SOFT_EASE_RATE = 1.0           # m/s3
SOFT_URGENT = -1.25            # m/s2 (just past stopAccel: the soft stop tracks requests down to stopAccel)
SOFT_MAX_T = 5.0               # s


def read_soft_stop():
  # soft stop rate in m/s3, 0.0 = off
  try:
    with open(SOFT_FILE) as f:
      vals = [float(v) for v in f.read().split()]
  except (OSError, ValueError):
    return SOFT_DEFAULT
  if len(vals) != 1 or not np.isfinite(vals[0]):
    return SOFT_DEFAULT
  if vals[0] <= 0.0:
    return 0.0
  return float(np.clip(vals[0], *SOFT_RATE_LIMITS))


def long_control_state_trans(CP, CP_SP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill):
  # Gas Interceptor
  cruise_standstill = cruise_standstill and not CP_SP.enableGasInterceptor

  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid
  return long_control_state

class LongControl:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    self.soft_rate = SOFT_DEFAULT
    self.soft_cnt = 0
    self.stop_t = 0.0

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    if self.soft_cnt % SOFT_PERIOD == 0:
      self.soft_rate = read_soft_stop()
    self.soft_cnt += 1

    self.long_control_state = long_control_state_trans(self.CP, self.CP_SP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if self.long_control_state != LongCtrlState.stopping:
      self.stop_t = 0.0
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      self.stop_t += DT_CTRL
      soft = self.soft_rate > 0.0 and CS.vEgo > self.CP.vEgoStarting and a_target >= SOFT_URGENT
      if soft and self.stop_t <= SOFT_MAX_T:
        # soft stop (see SOFT_* above)
        target = min(a_target, -SOFT_MIN, -CS.vEgo**2 / (2 * SOFT_DIST))
        target = float(np.clip(target, self.CP.stopAccel, -SOFT_MIN))
        if output_accel > target:
          output_accel = max(min(output_accel, 0.0) - self.soft_rate * DT_CTRL, target)
        elif self.CP.stopAccel < output_accel < target:
          output_accel = min(output_accel + SOFT_EASE_RATE * DT_CTRL, target)
      elif soft:
        # rolling for longer than SOFT_MAX_T: on to stopAccel at the soft stop rate
        if output_accel > self.CP.stopAccel:
          output_accel = max(min(output_accel, 0.0) - self.soft_rate * DT_CTRL, self.CP.stopAccel)
      elif output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target)

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
