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
# Live: /data/stop_soft holds "<rate> [<end>]": the soft stop rate in m/s3 (default 1.5, clamped SOFT_RATE_LIMITS) and
# the end floor, the weakest braking it eases to before the wheels stop, in m/s2 (default SOFT_MIN, clamped
# SOFT_MIN_LIMITS); no file or an unreadable file = the defaults; a rate of "0" (or any number <= 0) = off = the stock
# stopping ramp, bit for bit. The end floor was 0.4: drives 395-3a3 (21 stops) felt the brakes let go just before the
# stop ('stop gap'); the stop check showed no TSK release there, but the request eased from -0.95 to -0.40 in the last
# 0.5 s and the car floated at 1 -> 0 mph (sim: braking dropped 0.2-0.35 before the stop, 1.4 s from 1 mph to 0 when
# rolling up to a stopped car). At 0.8 it drops 0.07-0.12 and stops 0.6-0.7 s after 1 mph (stock -1.2: 0.5 s, the
# bump). "1.5 0.4" = the earlier soft stop.
SOFT_FILE = '/data/stop_soft'
SOFT_DEFAULT = 1.5
SOFT_RATE_LIMITS = (0.5, 6.0)  # m/s3
SOFT_PERIOD = 100              # updates between file reads (1 s at 100 Hz)
SOFT_MIN = 0.8                 # m/s2: default end floor
SOFT_MIN_LIMITS = (0.4, 1.2)   # m/s2
SOFT_DIST = 1.0                # m
SOFT_EASE_RATE = 1.0           # m/s3
SOFT_URGENT = -1.25            # m/s2 (just past stopAccel: the soft stop tracks requests down to stopAccel)
SOFT_MAX_T = 5.0               # s

# Roll guard. Once the wheels have read 0 in the stopping state (or vEgo stayed under ROLL_STILL_V for ROLL_ARM_T),
# any movement while openpilot still wants to be stopped (still in the stopping state: no launch, no creep decision)
# is a roll the car did not ask for (downhill: the brakes let go; uphill: vEgo is unsigned, a rollback reads the same):
# the request goes to ROLL_ACCEL at ROLL_RATE at once and stays there until openpilot leaves the stopping state, so
# the car stops again within centimetres instead of rolling on at -1.2 (or at the soft stop's -0.4). A launch or a
# creep leaves the stopping state first, so it never brakes them. Short stopping <-> pid flickers at a standstill
# (camera noise) keep it armed for ROLL_KEEP_T. Live: on with /data/decline_stop (longitudinal_planner.py DS_*);
# "0" there = off = origin/t2_stop_smooth exactly.
ROLL_FILE = '/data/decline_stop'
ROLL_V = 0.15          # m/s: wheels moving
ROLL_STILL_V = 0.05    # m/s: standing still (when the wheels do not read exactly 0)
ROLL_ARM_T = 0.1       # s
ROLL_KEEP_T = 0.5      # s
ROLL_CLEAR_V = 1.5     # m/s: driving again, disarm
ROLL_ACCEL = -2.0      # m/s2
ROLL_RATE = 10.0       # m/s3


def read_roll_guard():
  try:
    with open(ROLL_FILE) as f:
      vals = [float(v) for v in f.read().split()]
  except (OSError, ValueError):
    return True
  if len(vals) != 1 or not np.isfinite(vals[0]):
    return True
  return vals[0] > 0.0


def read_soft_stop():
  # (soft stop rate in m/s3, 0.0 = off; end floor in m/s2)
  try:
    with open(SOFT_FILE) as f:
      vals = [float(v) for v in f.read().split()]
  except (OSError, ValueError):
    return SOFT_DEFAULT, SOFT_MIN
  if len(vals) not in (1, 2) or not all(np.isfinite(vals)):
    return SOFT_DEFAULT, SOFT_MIN
  end = float(np.clip(vals[1], *SOFT_MIN_LIMITS)) if len(vals) == 2 else SOFT_MIN
  if vals[0] <= 0.0:
    return 0.0, end
  return float(np.clip(vals[0], *SOFT_RATE_LIMITS)), end


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
    self.soft_min = SOFT_MIN
    self.soft_cnt = 0
    self.stop_t = 0.0
    self.roll_on = True
    self.still_t = 0.0      # standing still in the stopping state
    self.away_t = 0.0       # out of the stopping state since the car was armed
    self.roll_armed = False
    self.roll_hold = False  # a roll was caught: hold ROLL_ACCEL until the stopping state ends

  def reset(self):
    self.pid.reset()

  def update_roll_guard(self, CS):
    stopping = self.long_control_state == LongCtrlState.stopping
    if not self.roll_on or self.long_control_state == LongCtrlState.off or CS.vEgo > ROLL_CLEAR_V:
      self.roll_armed = self.roll_hold = False
      self.still_t = self.away_t = 0.0
      return
    if not stopping:
      self.roll_hold = False
      self.still_t = 0.0
      self.away_t += DT_CTRL
      if self.away_t > ROLL_KEEP_T:
        self.roll_armed = False
      return
    self.away_t = 0.0
    if CS.standstill:
      self.roll_armed = True        # the wheels read 0
    elif CS.vEgo < ROLL_STILL_V:
      self.still_t += DT_CTRL
      if self.still_t >= ROLL_ARM_T:
        self.roll_armed = True
    else:
      self.still_t = 0.0

  def update(self, active, CS, a_target, should_stop, accel_limits):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    if self.soft_cnt % SOFT_PERIOD == 0:
      self.soft_rate, self.soft_min = read_soft_stop()
      self.roll_on = read_roll_guard()
    self.soft_cnt += 1

    self.long_control_state = long_control_state_trans(self.CP, self.CP_SP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if self.long_control_state != LongCtrlState.stopping:
      self.stop_t = 0.0
    self.update_roll_guard(CS)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      self.stop_t += DT_CTRL
      soft = self.soft_rate > 0.0 and CS.vEgo > self.CP.vEgoStarting and a_target >= SOFT_URGENT
      if soft and self.stop_t <= SOFT_MAX_T:
        # soft stop (see SOFT_* above)
        target = min(a_target, -self.soft_min, -CS.vEgo**2 / (2 * SOFT_DIST))
        target = float(np.clip(target, self.CP.stopAccel, -self.soft_min))
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
      if self.roll_armed and CS.vEgo > ROLL_V:
        self.roll_hold = True
      if self.roll_hold:
        # roll guard (see ROLL_* above)
        output_accel = min(output_accel, max(min(self.last_output_accel, 0.0) - ROLL_RATE * DT_CTRL, ROLL_ACCEL))
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
