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
