import numpy as np
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

# Soft stop. In the stopping state the brake request moves down to stopAccel at stoppingDecelRate,
# -1.2 m/s2 at 6 m/s3 on the Mk7. The planner hands the stop over while it brakes lightly (about
# -0.3 m/s2 at 0.5-0.7 m/s: it eases off as the car creeps up to its stop distance), so the request
# jumps to -1.2 within 0.15 s and the car stops while braking at about 1.1 m/s2: a bump at the very
# end of every stop. With soft stop set, while the car still rolls (vEgo above vEgoStarting) the
# request only moves down to the roll decel, at the soft stop rate, and never to less braking than
# stops the car within SOFT_STOP_DIST, so a hand-over at 0.85 m/s or faster keeps the stock -1.2 and
# rate. Once the wheels have stopped the stock ramp to stopAccel follows (nothing moves, nothing to
# feel), so the standstill hold is the same as stock. A stop handed over braking harder than the roll
# decel keeps that braking, as stock. Soft stop gives way to the stock ramp when the planner asks for
# clearly more braking than it gives (SOFT_STOP_PLAN_MARGIN: a lead braking hard or a car cutting in
# during the last metres) and after SOFT_STOP_MAX_T in the stopping state (a car that does not slow
# down, e.g. a steep downhill where the brakes give less than asked). Use it together with stop hold
# (/data/stop_hold, stop_hold.py): on its own it gives up 5-10 cm in the tightest stops.
# Live: /data/soft_stop holds "<roll decel m/s2> [<rate m/s3>]", e.g. "0.6" or "0.6 1.5" (decel at
# least 0.3 and under -stopAccel, rate 0.3-6.0, default 1.5); no file or 0 = off (stock).
SOFT_STOP_FILE = '/data/soft_stop'
SOFT_STOP_DECEL_MIN = 0.3
SOFT_STOP_DIST = 0.3     # m
SOFT_STOP_RATE = 1.5     # m/s3
SOFT_STOP_RATE_LIMITS = (0.3, 6.0)
SOFT_STOP_PERIOD = 100   # updates between file reads (1 Hz at 100 Hz)
SOFT_STOP_PLAN_MARGIN = 0.3  # m/s2: planner asks for this much more braking than soft stop gives -> stock
SOFT_STOP_MAX_T = 1.5    # s in the stopping state, then stock (flat stops take 0.4-1.3 s from hand-over)


def read_soft_stop():
  # -> (roll decel m/s2, rate m/s3); decel 0 = off
  try:
    with open(SOFT_STOP_FILE) as f:
      vals = [float(v) for v in f.read().split()]
  except (OSError, ValueError):
    return 0.0, SOFT_STOP_RATE
  if not 1 <= len(vals) <= 2 or not all(np.isfinite(v) for v in vals) or vals[0] <= 0.0:
    return 0.0, SOFT_STOP_RATE
  rate = float(np.clip(vals[1], *SOFT_STOP_RATE_LIMITS)) if len(vals) == 2 else SOFT_STOP_RATE
  return max(float(vals[0]), SOFT_STOP_DECEL_MIN), rate


LongCtrlState = car.CarControl.Actuators.LongControlState


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
    self.soft_stop = (0.0, SOFT_STOP_RATE)
    self.soft_stop_cnt = 0
    self.soft_stop_t = 0.0

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    if self.soft_stop_cnt % SOFT_STOP_PERIOD == 0:
      self.soft_stop = read_soft_stop()
    self.soft_stop_cnt += 1

    self.long_control_state = long_control_state_trans(self.CP, self.CP_SP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if self.long_control_state != LongCtrlState.stopping:
      self.soft_stop_t = 0.0
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      self.soft_stop_t += DT_CTRL
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        soft_decel, soft_rate = self.soft_stop
        stop_decel = CS.vEgo**2 / (2 * SOFT_STOP_DIST)   # m/s2 that stops the car within SOFT_STOP_DIST
        roll_accel = max(self.CP.stopAccel, -max(soft_decel, stop_decel))
        if (0.0 < soft_decel < -self.CP.stopAccel and CS.vEgo > self.CP.vEgoStarting and
            self.soft_stop_t <= SOFT_STOP_MAX_T and a_target >= min(output_accel, roll_accel) - SOFT_STOP_PLAN_MARGIN):
          # soft stop (see SOFT_STOP_* above): still rolling, only down to the roll decel
          rate = float(np.interp(stop_decel, [soft_decel, -self.CP.stopAccel], [soft_rate, self.CP.stoppingDecelRate]))
          output_accel = max(output_accel - rate * DT_CTRL, min(output_accel, roll_accel))
        else:
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
