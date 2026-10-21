#!/usr/bin/env python3
import math
import numpy as np

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource, CRUISE_MAX_ACCEL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.controls.lib.launch_boost import LaunchBoost
from openpilot.selfdrive.controls.lib.pickup_accel import PickupAccel
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP, a_total_min=0.0):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = max(np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V), a_total_min)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


# Stop placement (with the stop distance, long_mpc.py SD_*; off with /data/stop_gap "0"). The stock stop is declared
# where the PLAN's own speed drops under vEgoStopping. In a crawl or a slow queue the plan's speed (the MPC starts
# from a 2 s filtered vEgo, and the VW long control has no speed feedback) runs ahead of the car, so the stop was
# declared up to 2 m before the spot the plan aimed for, and the car stayed there; under 0.9 m/s the VW carcontroller
# also announces the stop while openpilot still brakes without the stopping state (HMS 4 latched without ACC_06
# Anhalten), the state in which the TSK lets go and the car pushes. Here, under ST_V_MAX:
#  - the stop is declared where the plan wants the car to stop: once the plan's remaining travel to its rest point
#    (MPC x_sol / v_solution) is no more than this car needs to stop (ST_T_LAG s + a stop at ST_A_STOP) + ST_TOL,
#    and held off while more is left (a plan braking harder than ST_A_PASS is never held off);
#  - while held off and slower than ST_V_ANN (above the VW announcement speed) the request is not negative, so the
#    stop is announced together with the stopping state (ACC_06 Anhalten), not before it; a car crawling slower than
#    the plan catches up gently (ST_CREEP_*);
#  - in the stopping state the request carries the decel that stops the car at the plan's rest point (never stronger
#    than ST_A_DIST_MAX: anything stronger is the plan's own request) for longcontrol's soft stop;
#  - at a standstill the car does not move off for a plan that wants less than ST_GO_TRAVEL (camera noise: stop,
#    creep, stop); a lead moving off makes the plan stop farther away or not at all, and the car goes as before.
ST_V_MAX = 2.0        # m/s: above this, stock
ST_V_EARLY = 1.2      # m/s: the stop may be declared under this
ST_V_ANN = 1.0        # m/s: above the VW stop announcement speed (carcontroller ANNOUNCE_SPEED 0.9)
ST_A_PASS = -0.6      # m/s2: a plan braking harder than this is never held off
ST_T_LAG = 0.5        # s: actuator lag + soft-stop ramp
ST_A_STOP = 0.6       # m/s2: the soft stop
ST_TOL = 0.3          # m
ST_V_REST = 0.2       # m/s: plan speed that counts as at rest (the MPC creeps the last centimetres slowly)
ST_D_MIN = 0.15       # m
ST_A_DIST_MAX = 0.95  # m/s2
ST_AWAY_N = 10        # updates (0.5 s): a plan that does not stop for this long drops the stop (lead moved off)
ST_CREEP_DV = 0.1     # m/s
ST_CREEP_TAU = 1.0    # s
ST_CREEP_A = 0.3      # m/s2
ST_GO_TRAVEL = 1.0    # m
ST_GO_AWAY_N = 2      # updates (0.1 s): at a standstill a plan no longer at rest, or a lead faster than ST_V_LEAD,
                      # for this long lets the car go
ST_V_LEAD = 0.5       # m/s: the stop is declared early (before the plan's speed is under vEgoStopping) only behind a
                      # lead this slow (a stopped car), never for a crawling queue that only slows down
ST_ROOM = 1.0         # m: a car ahead closer than this + what this car needs to stop at ST_A_DIST_MAX (its own stop
                      # at ST_A_LEAD counted as room) -- a cut-in, an overrun -- ends the cap: the stop is declared at once
                      # and asks the decel that stops the car ST_ROOM behind it (no cap, no easing); from longcontrol's
                      # SOFT_URGENT on that is the stock firm stopping ramp
ST_A_LEAD = 2.0       # m/s2
ST_A_URGENT = -1.2    # m/s2: a plan braking harder than this (more than the stopping state's stopAccel) stays in pid (no
                      # stopping state), so its request goes through


def plan_stop_travel(x_sol, v_sol):
  """metres the plan travels before it is at rest (inf if it is not at rest within the horizon)"""
  idx = np.nonzero(v_sol < ST_V_REST)[0]
  if not len(idx):
    return float('inf')
  i = int(idx[0])
  if i == 0:
    return 0.0
  v0, v1 = v_sol[i - 1], v_sol[i]
  f = (v0 - ST_V_REST) / max(v0 - v1, 1e-6)
  return float(x_sol[i - 1] + f * (x_sol[i] - x_sol[i - 1]))


class StopPlace:
  def __init__(self):
    self.declared = False
    self.away = 0                 # updates in a row the plan was not at rest within the horizon
    self.lead_go = 0              # updates in a row the lead was faster than ST_V_LEAD (at a standstill)
    self.travel = float('inf')
    self.d_prev = float('inf')    # last update's lead distance (ST_ROOM needs two close readings in a row)

  def reset(self):
    self.declared = False
    self.away = 0
    self.lead_go = 0
    self.travel = float('inf')
    self.d_prev = float('inf')

  def update(self, a_target, should_stop, v_ego, travel, v_plan, CP, v_lead=0.0, d_lead=float('inf')):
    """(a_target, should_stop) from get_accel_from_plan -> placed ones. travel = plan_stop_travel(), v_plan = the
    plan's speed at action_t. Call once per planner update while it is on."""
    d_close, self.d_prev = max(d_lead, self.d_prev), d_lead
    if v_ego > ST_V_MAX:
      self.reset()
      return a_target, should_stop
    # the plan's rest point, kept through short flickers of a plan that does not stop (camera lead speed noise)
    self.away = 0 if np.isfinite(travel) else self.away + 1
    if np.isfinite(travel):
      self.travel = float(travel)
    elif self.away >= ST_AWAY_N:
      self.travel = float('inf')
    travel = self.travel
    approaching = np.isfinite(travel)
    if v_ego <= CP.vEgoStarting:
      self.declared = False
      self.lead_go = self.lead_go + 1 if v_lead >= ST_V_LEAD else 0
      if not should_stop and self.away < ST_GO_AWAY_N and travel < ST_GO_TRAVEL and self.lead_go < ST_GO_AWAY_N:
        should_stop = True
      return a_target, should_stop
    d_car = v_ego * ST_T_LAG + v_ego**2 / (2 * ST_A_STOP)
    if self.declared and not approaching:
      self.declared = False
    if not self.declared:
      if approaching and v_ego < ST_V_EARLY and travel <= d_car + ST_TOL and a_target < 0.1 and v_lead < ST_V_LEAD:
        self.declared = True
      elif should_stop and approaching and travel > d_car + ST_TOL and a_target >= ST_A_PASS:
        should_stop = False
      elif should_stop:
        self.declared = True
    if self.declared:
      should_stop = True
      d_eff = max(travel - v_ego * ST_T_LAG, ST_D_MIN)
      a_target = min(a_target, -min(v_ego**2 / (2 * d_eff), ST_A_DIST_MAX))
    elif approaching and ST_A_PASS <= a_target < 0.1:
      if v_ego < ST_V_ANN:
        a_target = max(a_target, 0.0)
      if v_ego < v_plan - ST_CREEP_DV:
        a_target = max(a_target, min((v_plan - v_ego) / ST_CREEP_TAU, ST_CREEP_A))
    # a car this close: no hold-off, no cap, no easing (see ST_ROOM)
    room = d_close + max(v_lead, 0.0)**2 / (2 * ST_A_LEAD) - ST_ROOM - v_ego * ST_T_LAG
    if v_ego**2 / (2 * max(room, 0.05)) > ST_A_DIST_MAX:
      if a_target < ST_A_URGENT:
        # the plan brakes harder than the stopping state can (it holds at most stopAccel): leave it to pid
        self.declared = False
        should_stop = False
      else:
        self.declared = True
        should_stop = True
        a_target = min(a_target, -min(v_ego**2 / (2 * max(room, 0.05)), 3.5))
    return a_target, should_stop


# Downhill stops ("slow down, stop, roll, stop"). The VW request is vehicle accel and openpilot's VW long control is
# pure feedforward, so on a decline whatever lets the brakes go even briefly turns into a roll: gravity adds
# g * grade (-10 %: +0.98 m/s2) on top of the idle creep. After a stop the plan may still want to close up to its
# stop spot (more than ST_GO_TRAVEL left, e.g. a stop that ended 6 m instead of 4 m back): openpilot leaves the
# stopping state at a standstill with +0.1..0.2 = ACC_06 Anfahren + HMS 4, the TSK lets the brakes go, and downhill
# the car rolls 0 -> 2+ mph in about a second before the stopping state catches it again. And the soft stop eases
# the brake at the very end (to -0.4), which needs the TSK to compensate the whole grade at a crawl. On a decline
# (grade from the calibrated pitch openpilot already uses for the coast model, filtered, decided while faster than
# DS_V_DECIDE or at a standstill and held through the stop's own nose dive):
#  - at a standstill the car does not move off to close up; only a lead that moves off (faster than ST_V_LEAD) or a
#    plan that no longer comes to rest, for DS_GO_N updates in a row, lets it go: "slow down, stop" and that's it;
#  - in the stopping state under DS_V_MAX the stop request never eases (it keeps the strongest request of this
#    stop) and is at least DS_A_MIN: the soft stop still blends in at its rate, but does not let go at the end;
#  - a request to go under DS_V_LAUNCH is capped at DS_LAUNCH_A (real -12 %: +1.3 asked -> 0 to 2.6 m/s in 1.2 s,
#    the TSK lets the brakes go on a launch and gravity pushes too): creeping after the lead without the surge.
# Live: /data/decline_stop holds the decline threshold in % (default DS_DEFAULT, clamped DS_LIMITS); "0" (or any
# number <= 0) = off = origin/t2_stop_smooth exactly (with longcontrol's roll guard and the VW carcontroller's decline
# brake, which read the same file; the carcontroller judges the grade from the TSK's own Motor_16 grade).
DS_FILE = '/data/decline_stop'
DS_DEFAULT = 2.5        # % downhill
DS_LIMITS = (1.0, 10.0)
DS_HYST = 1.0           # %: off again once the grade is DS_HYST flatter than the threshold
DS_PERIOD = 20          # planner updates between file reads (1 s at 20 Hz)
DS_TAU = 1.0            # s: grade filter
DS_V_DECIDE = 2.0       # m/s: the grade decision is taken above this or at a standstill, held in between
DS_V_MAX = 2.0          # m/s: no-easing applies under this in the stopping state
DS_A_MIN = 0.6          # m/s2
DS_V_LAUNCH = 1.0       # m/s: under this on a decline a request to go (launch, creep after the lead) is capped ...
DS_LAUNCH_A = 0.6       # m/s2: ... here: the TSK lets the brakes go on a launch and gravity adds g * grade on top
DS_V_LAUNCH_Q = 3.0     # m/s: ... and under this behind a car ahead slower than DS_V_LEAD_Q (a creeping queue that will
DS_V_LEAD_Q = 2.0       # m/s  stop again); behind a car that drives off the cap ends at DS_V_LAUNCH (the TSK compensates
                        # the grade above it: pulling away feels as on the flat)
DS_GO_N = 3             # updates (0.15 s): at a standstill on a decline a lead faster than ST_V_LEAD, or a plan that
                        # does not come to rest, for this long lets the car go (camera speed noise: single updates)
DS_GO_AWAY_N = 6        # updates (0.3 s): ... a plan that does not come to rest (camera noise makes 0.15 s ones)
DS_GO_NOLEAD_N = 40     # updates (2 s): ... with no car ahead seen at all a plan that does not come to rest must last
                        # this long (a camera dropout at a standstill; a car ahead that drives off is seen moving first)
DS_GO_HOLD_N = 30       # updates (1.5 s): once a car ahead was seen moving off at a standstill the car stays let go
                        # this long or until it moves (camera speed noise must not take a launch back before the car
                        # has started: stop/go chatter). Not for a plan that does not come to rest (camera noise there
                        # would let the car close up to its stop spot: the downhill re-launch roll)


def read_decline_stop():
  try:
    with open(DS_FILE) as f:
      vals = [float(v) for v in f.read().split()]
  except (OSError, ValueError):
    return DS_DEFAULT
  if len(vals) != 1 or not np.isfinite(vals[0]):
    return DS_DEFAULT
  if vals[0] <= 0.0:
    return 0.0
  return float(np.clip(vals[0], *DS_LIMITS))


def launch_cap_speed(v_lead, has_lead):
  return DS_V_LAUNCH_Q if (has_lead and v_lead < DS_V_LEAD_Q) else DS_V_LAUNCH


class DeclineStop:
  def __init__(self, dt=DT_MDL):
    self.threshold = DS_DEFAULT
    self.cnt = 0
    self.grade = FirstOrderFilter(0.0, DS_TAU, dt)
    self.grade_init = False
    self.decline = False
    self.lead_go = 0
    self.away = 0
    self.go_hold = 0
    self.a_stop = 0.0

  def update_grade(self, pitch):
    """pitch: carControl.orientationNED[1] (rad, < 0 nose down) or None. Every planner update."""
    if self.cnt % DS_PERIOD == 0:
      self.threshold = read_decline_stop()
    self.cnt += 1
    if pitch is not None and np.isfinite(pitch):
      g = -100.0 * math.tan(pitch)     # % downhill
      if not self.grade_init:
        self.grade.x = g
        self.grade_init = True
      self.grade.update(g)

  def reset(self):
    self.lead_go = 0
    self.away = 0
    self.go_hold = 0
    self.a_stop = 0.0

  def update(self, a_target, should_stop, v_ego, travel, CP, v_lead=0.0, has_lead=True):
    """(a_target, should_stop) -> downhill ones. Call after update_grade while openpilot drives."""
    if self.threshold <= 0.0 or not self.grade_init:
      self.decline = False
      self.reset()
      return a_target, should_stop
    if v_ego > DS_V_DECIDE or v_ego <= CP.vEgoStarting:
      if self.grade.x >= self.threshold:
        self.decline = True
      elif self.grade.x < self.threshold - DS_HYST:
        self.decline = False
    if v_ego <= CP.vEgoStarting:
      self.a_stop = 0.0
      self.lead_go = self.lead_go + 1 if v_lead >= ST_V_LEAD else 0
      self.away = 0 if np.isfinite(travel) else self.away + 1
      away_n = DS_GO_AWAY_N if has_lead else DS_GO_NOLEAD_N
      if self.lead_go >= DS_GO_N:
        self.go_hold = DS_GO_HOLD_N
      elif self.go_hold > 0:
        self.go_hold -= 1
      if self.decline and not should_stop and self.go_hold <= 0 and self.away < away_n:
        should_stop = True
      if self.decline and not should_stop:
        a_target = min(a_target, DS_LAUNCH_A)
      return a_target, should_stop
    self.lead_go = 0
    self.away = 0
    self.go_hold = 0
    if self.decline and not should_stop and v_ego < launch_cap_speed(v_lead, has_lead):
      a_target = min(a_target, DS_LAUNCH_A)
    if self.decline and should_stop and v_ego < DS_V_MAX:
      self.a_stop = min(self.a_stop, a_target, -DS_A_MIN)
      a_target = self.a_stop
    else:
      self.a_stop = 0.0
    return a_target, should_stop


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.launch_boost = LaunchBoost(dt)
    self.pickup_accel = PickupAccel()
    self.stop_place = StopPlace()
    self.decline_stop = DeclineStop(dt)
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  @staticmethod
  def parse_model(model_msg):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    reset_state = reset_state or not v_cruise_initialized

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    # Pickup accel (pickup_accel.py): allow more accel up to 25 mph, with or without a car ahead, in
    # the planner cap, the turn cap and the MPC's cruise fake obstacle. 0 when turned off.
    pickup = self.pickup_accel.update(v_ego)
    accel_clip = [ACCEL_MIN, max(get_max_accel(v_ego), pickup)]
    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
    accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP, a_total_min=pickup)
    self.mpc.cruise_max_accel = max(CRUISE_MAX_ACCEL, pickup)

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    _, _, _, _, throttle_prob = self.parse_model(sm['modelV2'])
    # Don't clip at low speeds since throttle_prob doesn't account for creep
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    # Get new v_cruise and a_desired from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.a_desired = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.a_desired, v_cruise)

    if force_slow_decel:
      v_cruise = 0.0

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(sm['radarState'], v_cruise, personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, vEgoStopping=self.CP.vEgoStopping)
    # Launch boost from a dead stop behind a departing lead (launch_boost.py), ACC mode only. It only adds
    # accel, before the accel clip, and while it is on longcontrol leaves stopping. The boosted car runs
    # ahead of the MPC's own state, so the MPC is put on the car's real speed and commanded accel: what
    # it plans, also right after an abort, starts from the car as it is.
    long_active_acc = sm['carControl'].longActive and not self.is_e2e(sm)
    launch_boost = self.launch_boost.update(long_active_acc, v_ego, sm['carState'].gasPressed,
                                            sm['carState'].brakePressed, sm['radarState'].leadOne,
                                            sm['radarState'].leadTwo, output_a_target_mpc, self.mpc.crash_cnt)
    if self.launch_boost.event:
      cloudlog.info(f"launch boost: {self.launch_boost.event}")
    if launch_boost > 0.0:
      output_a_target_mpc = max(output_a_target_mpc, launch_boost)
      output_should_stop_mpc = False
      self.a_desired = max(self.a_desired, launch_boost)
    # Stop placement (see ST_* above), with the stop distance of long_mpc.py (off with /data/stop_gap "0")
    if self.mpc.stop_distance_on and not reset_state and launch_boost <= 0.0:
      travel = plan_stop_travel(self.mpc.x_sol[:, 0], self.mpc.v_solution)
      v_plan = float(np.interp(action_t, CONTROL_N_T_IDX, self.v_desired_trajectory))
      l1 = sm['radarState'].leadOne
      v_lead = float(l1.vLead) if (l1.status and np.isfinite(l1.vLead)) else 0.0
      d_lead = float(l1.dRel) if (l1.status and np.isfinite(l1.dRel)) else float('inf')
      output_a_target_mpc, output_should_stop_mpc = self.stop_place.update(output_a_target_mpc, output_should_stop_mpc,
                                                                           v_ego, travel, v_plan, self.CP, v_lead, d_lead)
    else:
      self.stop_place.reset()
    # Downhill stops (see DS_* above), with or without the stop distance
    self.decline_stop.update_grade(sm['carControl'].orientationNED[1] if len(sm['carControl'].orientationNED) == 3 else None)
    if not reset_state and launch_boost <= 0.0:
      l1 = sm['radarState'].leadOne
      v_lead = float(l1.vLead) if (l1.status and np.isfinite(l1.vLead)) else 0.0
      travel = plan_stop_travel(self.mpc.x_sol[:, 0], self.mpc.v_solution)
      output_a_target_mpc, output_should_stop_mpc = self.decline_stop.update(output_a_target_mpc, output_should_stop_mpc,
                                                                             v_ego, travel, self.CP, v_lead,
                                                                             bool(l1.status))
    else:
      self.decline_stop.reset()
      l1 = sm['radarState'].leadOne
      v_cap = launch_cap_speed(float(l1.vLead) if (l1.status and np.isfinite(l1.vLead)) else 0.0, bool(l1.status))
      if not reset_state and self.decline_stop.decline and self.decline_stop.threshold > 0.0 and v_ego < v_cap:
        output_a_target_mpc = min(output_a_target_mpc, DS_LAUNCH_A)   # a launch boost downhill: capped too
    if self.launch_boost.mpc_behind(self.v_desired_filter.x, v_ego):
      self.v_desired_filter.x = v_ego
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    if self.is_e2e(sm):
      output_a_target = min(output_a_target_e2e, output_a_target_mpc)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc
      if output_a_target < output_a_target_mpc:
        self.mpc.source = LongitudinalPlanSource.e2e
    else:
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc

    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_sp(sm, pm)
