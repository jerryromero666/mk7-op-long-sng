#!/usr/bin/env python3
import os
import time
import numpy as np
from cereal import log
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
# WARNING: imports outside of constants will not trigger a rebuild
from openpilot.selfdrive.modeld.constants import index_function
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU

if __name__ == '__main__':  # generating code
  from openpilot.third_party.acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython

from casadi import SX, vertcat

MODEL_NAME = 'long'
LONG_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LONG_MPC_DIR, "c_generated_code")
JSON_FILE = os.path.join(LONG_MPC_DIR, "acados_ocp_long.json")

LongitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource
MPC_SOURCES = (LongitudinalPlanSource.lead0, LongitudinalPlanSource.lead1, LongitudinalPlanSource.cruise)

X_DIM = 3
U_DIM = 1
PARAM_DIM = 6
COST_E_DIM = 5
COST_DIM = COST_E_DIM + 1
CONSTR_DIM = 4

X_EGO_OBSTACLE_COST = 3.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.
A_CHANGE_COST = 75.
DANGER_ZONE_COST = 100.
CRASH_DISTANCE = .25
LEAD_DANGER_FACTOR = 0.75
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'

# Fewer timestamps don't hurt performance and lead to
# much better convergence of the MPC with low iterations
N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]

T_IDXS = np.array(T_IDXS_LST)
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.])
COMFORT_BRAKE = 2.5
STOP_DISTANCE = 7.5
CRUISE_MIN_ACCEL = -1.2
CRUISE_MAX_ACCEL = 1.6
MIN_X_LEAD_FACTOR = 0.5

def get_jerk_factor(personality=log.LongitudinalPersonality.standard):
  if personality==log.LongitudinalPersonality.relaxed:
    return 1.0
  elif personality==log.LongitudinalPersonality.standard:
    return 1.0
  elif personality==log.LongitudinalPersonality.aggressive:
    return 0.5
  else:
    raise NotImplementedError("Longitudinal personality not supported")


def get_T_FOLLOW(personality=log.LongitudinalPersonality.standard):
  if personality==log.LongitudinalPersonality.relaxed:
    return 1.75
  elif personality==log.LongitudinalPersonality.standard:
    return 1.45
  elif personality==log.LongitudinalPersonality.aggressive:
    return 1.25
  else:
    raise NotImplementedError("Longitudinal personality not supported")

# Close follow while the lead pulls away at low speed. Behind a lead the MPC keeps
# v^2/5 + t_follow*v + 6 m (the 6 m stop distance is compiled into the solver), so at 5-25 mph it
# lets a departing car pull away (route 33d: gap at or under that 95-100% of the time on the slow
# pickups, plan 0.1-0.6 m/s2 while the limit allowed 1.8). While leadOne moves (none at 1 m/s,
# full at 3 m/s) and this car is not closing on it (none when 2 m/s faster than the lead, full at
# 0.5 m/s), the lead obstacle is shifted forward (the car settles that much closer) and
# t_follow is shortened, but never so far that the wanted gap drops under a stopping-distance
# floor: if the lead brakes at CF_A_LEAD, this car reacts after CF_T_REACT and brakes at CF_A_EGO,
# it still stops CF_D_MIN behind. The floor grows with speed, so the effect is largest at
# 5-15 mph and gone by about 25 mph (aggressive). When the lead slows it goes back fast, so stops
# keep the gap. leadTwo is usually the same car: both obstacles move, by the smaller of the two.
# Live: /data/close_follow holds "<stop gap cut m> <t_follow s>" (default "3.0 0.9"; "0 1.75" = off).
CF_DEFAULT = (3.0, 0.9)
CF_FILE = '/data/close_follow'
CF_CUT_LIMITS = (0.0, 4.0)   # m: the stop distance never goes below 2 m
CF_TF_LIMITS = (0.8, 1.75)   # s: and t_follow is never longer than the personality's
CF_LEAD_V = [1.0, 3.0]       # m/s lead speed: none .. full
CF_CLOSING_V = [-2.0, -0.5]  # m/s lead speed minus this car's: none .. full
CF_SOLVER_STOP_DISTANCE = 6.0  # compiled into c_generated_code (STOP_DISTANCE above is not)
CF_T_REACT = 1.0             # s: floor reaction time
CF_A_EGO = 3.0               # m/s2: floor braking of this car
CF_A_LEAD = 5.0              # m/s2: floor braking of the lead
CF_D_MIN = 2.5               # m: floor gap left when both have stopped
CF_SHIFT_IN = 0.05           # m per update toward closer (1 m/s)
CF_SHIFT_OUT = 0.25          # m per update back (5 m/s)
CF_TF_IN = 0.01              # s per update shorter
CF_TF_OUT = 0.05             # s per update back
CF_FILE_PERIOD = 20          # updates between file reads


def read_close_follow():
  try:
    with open(CF_FILE) as f:
      vals = [float(v) for v in f.read().split()]
  except (OSError, ValueError):
    return CF_DEFAULT
  if len(vals) != 2 or not all(np.isfinite(v) for v in vals):
    return CF_DEFAULT
  return float(np.clip(vals[0], *CF_CUT_LIMITS)), float(np.clip(vals[1], *CF_TF_LIMITS))


def close_follow_weight(lead, v_ego, t_follow, cut_full):
  # 0..1 share of the close follow allowed behind this lead (see CF_* above)
  v_lead = max(lead.vLead, 0.0)
  d_base = CF_SOLVER_STOP_DISTANCE + t_follow * v_ego + (v_ego**2 - v_lead**2) / (2 * COMFORT_BRAKE)
  d_floor = CF_D_MIN + CF_T_REACT * v_ego + v_ego**2 / (2 * CF_A_EGO) - v_lead**2 / (2 * CF_A_LEAD)
  w = float(np.interp(v_lead, CF_LEAD_V, [0., 1.]) * np.interp(v_lead - v_ego, CF_CLOSING_V, [0., 1.]) *
            np.clip((d_base - d_floor) / cut_full, 0.0, 1.0))
  return w if np.isfinite(w) else 0.0  # a NaN / inf vLead must not latch _cf_shift and _cf_tf at NaN


# Stop distance: one standstill gap for every kind of stop. It replaces the stop gap of t2_set_min_gap /
# t2_stop_gap2, which moved the obstacle of a lead PREDICTED to be stopped back by the extra metres. That rule
# switched on by the camera lead's speed: late (or not at all) behind a car braking to a stop, which still stopped
# close, and fully behind a stopped car, a crawl or a slow queue, which already stopped far back (the MPC plans the
# whole compiled 6 m there, but only the 4.5 m danger floor behind a braking car, and the camera lag eats more).
# Here every lead obstacle is moved by one amount that depends only on this car's speed (no lead-speed switch):
#   at SD_V[0] and slower:  the standstill trim (aimed gap - SD_AIM_OFFSET; negative = closer than the 6 m),
#   from SD_V[1] up:        the trim + SD_MARGIN, a margin at speed behind a car that may stop (it absorbs the
#                           camera lag and the car's brake lag of a stop from speed),
# blended in between, so the margin goes back to the standstill trim while the car slows through SD_V: the end of
# a stop from speed eases instead of braking harder, and every kind of stop aims for the same spot (the planner's
# stop placement, longitudinal_planner.py, then declares the stop there). The margin is left out while the lead
# pulls away (SD_PULL_V: lead faster than this car), so pickups behind a departing car keep the close follow. The
# whole shift fades out from SD_FADE_MPH[0] to SD_FADE_MPH[1]: following at speed is unchanged. It is slewed.
# Live: /data/stop_gap holds the gap in metres the car aims to stop behind the car ahead (default 4.0, clamped
# SD_LIMITS); no file or an unreadable file = the default; "0" (or any number <= 0) = off = no stop distance and no
# stop placement: the stop distance of t2_set_min (and of t2_highway_follow with /data/stop_gap 0), bit for bit.
SD_FILE = '/data/stop_gap'
SD_DEFAULT = 4.0
SD_LIMITS = (3.0, 7.0)        # m: aimed gap
SD_AIM_OFFSET = 5.5           # m: the compiled 6 m stop distance less the ~0.5 m the soft stop rolls past the plan
SD_MARGIN = 3.5               # m: added at speed
SD_V = [1.0, 3.0]             # m/s this car's speed: standstill trim .. trim + margin
SD_PULL_V = [1.0, 3.0]        # m/s lead speed minus this car's: margin kept .. none (the lead pulls away)
SD_FADE_MPH = [25.0, 35.0]    # mph: full .. none
SD_SLEW = 0.1                 # m per update
SD_FILE_PERIOD = 20           # updates between file reads (1 s)


def read_stop_distance():
  # aimed standstill gap in m (see SD_* above), 0.0 = off
  try:
    with open(SD_FILE) as f:
      vals = [float(v) for v in f.read().split()]
  except (OSError, ValueError):
    return SD_DEFAULT
  if len(vals) != 1 or not np.isfinite(vals[0]):
    return SD_DEFAULT
  if vals[0] <= 0.0:
    return 0.0
  return float(np.clip(vals[0], *SD_LIMITS))


def stop_distance_shift(v_ego, aim, dv_lead):
  # metres to move every lead obstacle back (negative = forward) at this car's speed (see SD_* above);
  # dv_lead = leadOne's speed minus this car's (0 without a lead)
  if aim <= 0.0:
    return 0.0
  margin = SD_MARGIN * float(np.interp(v_ego, SD_V, [0., 1.])) * float(np.interp(dv_lead, SD_PULL_V, [1., 0.]))
  fade = float(np.interp(v_ego, [SD_FADE_MPH[0] * CV.MPH_TO_MS, SD_FADE_MPH[1] * CV.MPH_TO_MS], [1., 0.]))
  shift = (aim - SD_AIM_OFFSET + margin) * fade
  return shift if np.isfinite(shift) else 0.0


# Highway follow: a shorter time gap at highway speed. Behind a lead the MPC settles at
# t_follow * v + 6 m (aggressive 1.25 s: 42 m at 65 mph, 56 m at 90 mph); the close follow above cannot
# shorten that, its floor is above the wanted gap from about 30 mph up (aggressive; 35 standard, 44 relaxed). From HF full speed (default
# 45 mph) up, with no upper limit, t_follow is lowered to the time gap in the file, blended in over the
# HF_BAND_MPH below the full speed (none at 35 mph, full at 45 mph by default) so crossing the band does not
# step the wanted gap. The cut is slewed like the close follow (HF_TF_IN per update shorter, HF_TF_OUT per
# update back), so neither the band nor a change of the file jumps the request. It is applied after the
# close follow and before the cruise fake obstacle, which uses the same t_follow, so the set speed is kept
# as before. The approach term (v_ego^2 - v_lead^2) / 5 is compiled into the solver and does not change.
# While the cut is active the stopped phantom of a lead that drops out (process_lead) is not used: the gap
# can then be under its 25 m at highway speed, and a stopped car there is a brake stab, not a stand-in.
# Live: /data/highway_follow holds "<time gap s> [<full speed mph>]" (e.g. "0.8" or "0.8 45").
# No file, 0, or an unreadable file = off = the behaviour without this feature, exactly.
# The time gap is clamped to HF_T_LIMITS: the floor is what a lead braking as hard as this car can
# (-3.5 m/s2, the panda limit) still leaves room for after a 0.5 s reaction at 90 mph (0.5 s headway in all,
# 0.35 s + 6 m), rounded up to 0.5 s for the delays of the camera lead, the planner and the car. In the closed-loop
# simulation of this branch the stack takes 3-4 s to reach full braking, so what it absorbs on its own is far less
# (0.5 s: a lead braking ~1.5 m/s2 at 65 mph, ~1 at 90; 0.8: ~2.5 / 2.25; today's 1.25: ~3): the driver covers the
# rest, as under the gas override today. The full speed is clamped to HF_V_LIMITS; its floor (35 mph: the band is
# 25-35 mph at the lowest) keeps the cut out of the close follow / stop-and-go speeds, where it was not simulated.
HF_DEFAULT = (0.0, 45.0)
HF_FILE = '/data/highway_follow'
HF_T_LIMITS = (0.5, 1.75)    # s: time gap
HF_V_LIMITS = (35.0, 90.0)   # mph: full speed
HF_BAND_MPH = 10.0           # mph below the full speed over which the cut blends in
HF_TF_IN = CF_TF_IN          # s per update shorter
HF_TF_OUT = CF_TF_OUT        # s per update back
HF_FILE_PERIOD = 20          # updates between file reads (1 s)


def read_highway_follow():
  try:
    with open(HF_FILE) as f:
      vals = [float(v) for v in f.read().split()]
  except (OSError, ValueError):
    return HF_DEFAULT
  if len(vals) not in (1, 2) or not all(np.isfinite(v) for v in vals) or vals[0] <= 0.0:
    return HF_DEFAULT
  v_full = float(np.clip(vals[1], *HF_V_LIMITS)) if len(vals) == 2 else HF_DEFAULT[1]
  return float(np.clip(vals[0], *HF_T_LIMITS)), v_full


def highway_follow_weight(v_ego, v_full_mph):
  # 0..1 share of the highway follow cut at this speed (see HF_* above)
  v_full = v_full_mph * CV.MPH_TO_MS
  return float(np.interp(v_ego, [v_full - HF_BAND_MPH * CV.MPH_TO_MS, v_full], [0., 1.]))


# Coast-first approach: at highway speed, while closing on a slower car with a large reserve, do not ask for more slowing
# than this car gets by lifting off. On this car (mk7_follow_check F, drive 34f) a request down to about -0.55 m/s2 is met
# with no brakes (overrun fuel cut / engine drag); under about -0.6 the brakes come on. Today's 45+ mph approaches ask
# -0.9..-1.2 at ~100 m, mostly because the camera's far-range lead speed is noisy and the compiled approach term
# (v_ego^2 - v_lead^2) / 5 turns 1 m/s of it into ~12 m of wanted gap. So a FLOOR on the accel the solver may plan
# (a_min, and the request the planner sends) is raised to minus the larger of the coast decel and CA_NEED_FACTOR x the
# decel needed to end the approach at the following gap behind the lead at its present speed. Distances and speeds only:
# the far aLeadK is never used to allow less braking, and the lead speed used is the lower of now and the last second's
# mean, so an upward jump of a noisy far lead does not open anything. The floor is ON only while all of these hold
# (on / off thresholds with a dead band in between, where the state is kept):
#  - RESERVE: this car could still stop CA_RES_D_MIN behind the lead if the lead braked at CA_RES_A_LEAD from its present
#    speed right now and this car reached CA_RES_A_EGO after an equivalent delay of CA_RES_T s (camera + planner + car,
#    and the floor's own ramp): on from CA_RES[1] m of reserve, off under CA_RES[0]. A fast approach on much slower
#    traffic never has it, so it keeps today's earlier, firmer braking.
#  - speed (on from CA_V_EGO[1], off under CA_V_EGO[0]; under CA_V_MIN the floor is dropped at once), closing (CA_CLOSING),
#    time to collision (CA_TTC), room beyond the following gap (CA_ROOM), the set speed not under this car's speed, and
#    in the last CA_BLOCK updates no cut-in (the gap shrank CA_CUTIN_D more than closing explains), no dropout and no
#    braking lead (aLeadK under CA_LB_A AND its speed fell CA_LB_DV within CA_LB_N updates, CA_LB_CONFIRM updates in a row);
#    to turn it ON, also: the lead seen for CA_SETTLE updates and its aLeadK not under CA_LB_A (aLeadK alone never turns it
#    off: on a far camera lead it swings by +-3 m/s2 with the speed noise).
# No steps: the floor rises (less braking) at most CA_J_UP and falls (more braking) at most CA_J_DOWN per second, starting
# under the plan already running, so the request never jumps; once the plan no longer touches a falling floor it is
# removed and the solver is exactly today's. Turned off, the floor is ramped away the same way.
# Live: /data/coast_approach holds the coast decel in m/s2 (e.g. "0.55"; clamped to CA_A_LIMITS), or "1" (any number
# from 1 up) = on with CA_ON_DEFAULT. No file, 0, a negative number or junk = off = the behaviour without this feature.
CA_FILE = '/data/coast_approach'
CA_ON_DEFAULT = 0.55         # m/s2: the coast decel "1" stands for (the value the sims were run with)
CA_A_LIMITS = (0.2, 0.6)     # m/s2: coast decel (under about 0.6 this car does not brake)
CA_NEED_FACTOR = 1.15        # decel allowed = this x the needed decel (when that is above the coast decel)
CA_T_REACT = 1.0             # s of closing taken off the room before the needed decel is counted
CA_V_MIN = 40.0 * CV.MPH_TO_MS                          # under this the floor is dropped at once
CA_V_EGO = (42.0 * CV.MPH_TO_MS, 45.0 * CV.MPH_TO_MS)   # this car's speed: off under .. on from
CA_CLOSING = (0.3, 1.0)      # m/s closing: off under .. on from
CA_TTC = (8.0, 10.0)         # s time to collision: off under .. on from
CA_ROOM = (2.0, 6.0)         # m of room beyond the following gap: off under .. on from
CA_RES = (0.0, 10.0)         # m of hard-brake reserve: off under .. on from
CA_RES_A_LEAD = 3.0          # m/s2 the lead is assumed to brake at
CA_RES_A_EGO = 3.5           # m/s2 this car brakes at after the delay
CA_RES_T = 1.8               # s equivalent delay
CA_RES_D_MIN = 3.0           # m left
CA_LB_A = -1.0               # m/s2 aLeadK under which a lead counts as braking ...
CA_LB_DV = 3.0               # m/s ... if its speed also fell this much ...
CA_LB_N = 20                 # ... within this many updates (1 s) ...
CA_LB_CONFIRM = 3            # ... in this many updates in a row ...
CA_LB_ROOM_A = 0.0           # ... unless it could brake at this (m/s2) and the reserve would still hold (0 = always)
CA_SETTLE = 20               # updates a lead must be seen before the floor turns on (at most CA_LB_N)
CA_CUTIN_D = 8.0             # m the gap may shrink in one update beyond what the closing speed explains
CA_BLOCK = 20                # updates (1 s) off after a cut-in, a dropout or a braking lead
CA_J_UP = 0.8                # m/s3: the floor rises (less braking) at most this fast
CA_J_DOWN = 2.5              # m/s3: and falls (more braking) at most this fast ...
CA_J_DOWN_FAST = 5.0         # m/s3: ... or this fast once the reserve is gone or the lead cut in / brakes
CA_RELEASE = 0.3             # m/s2: a falling floor this far under the whole plan is removed
CA_FILE_PERIOD = 20          # updates between file reads (1 s)


def read_coast_approach():
  try:
    with open(CA_FILE) as f:
      vals = [float(v) for v in f.read().split()]
  except (OSError, ValueError):
    return 0.0
  if len(vals) != 1 or not np.isfinite(vals[0]) or vals[0] <= 0.0:
    return 0.0
  if vals[0] >= 1.0:
    return CA_ON_DEFAULT
  return float(np.clip(vals[0], *CA_A_LIMITS))


def coast_reserve(d_rel, v_lead, v_ego):
  # m this car would still have behind a lead braking at CA_RES_A_LEAD from now (see CA_* above)
  v_lead = max(v_lead, 0.0)
  return d_rel - (CA_RES_D_MIN + CA_RES_T * v_ego + v_ego**2 / (2.0 * CA_RES_A_EGO) - v_lead**2 / (2.0 * CA_RES_A_LEAD))


def _band(x, off_on):
  # 0 under the off threshold, 1 from the on threshold, None in between (keep the state)
  return 0 if not x >= off_on[0] else (1 if x >= off_on[1] else None)


def coast_gate(d_rel, v_lead, v_ego, t_follow, a_coast):
  # (floor accel, gate) behind one lead from distances and speeds only (see CA_* above); gate 0 off, 1 on, None keep
  if not (np.isfinite(d_rel) and np.isfinite(v_lead) and np.isfinite(v_ego)):
    return ACCEL_MIN, 0
  v_lead = max(v_lead, 0.0)
  dv = v_ego - v_lead
  room = d_rel - (CF_SOLVER_STOP_DISTANCE + t_follow * v_lead) - max(dv, 0.0) * CA_T_REACT
  if dv <= 0.0 or room <= 0.0:
    return ACCEL_MIN, 0
  a_allow = min(max(a_coast, CA_NEED_FACTOR * dv**2 / (2.0 * room)), -ACCEL_MIN)
  bands = (_band(v_ego, CA_V_EGO), _band(dv, CA_CLOSING), _band(d_rel / dv, CA_TTC), _band(room, CA_ROOM),
           _band(coast_reserve(d_rel, v_lead, v_ego), CA_RES))
  gate = 0 if 0 in bands else (1 if all(b == 1 for b in bands) else None)
  return -a_allow, gate


def get_stopped_equivalence_factor(v_lead):
  return (v_lead**2) / (2 * COMFORT_BRAKE)

def get_safe_obstacle_distance(v_ego, t_follow):
  return (v_ego**2) / (2 * COMFORT_BRAKE) + t_follow * v_ego + STOP_DISTANCE

def gen_long_model():
  model = AcadosModel()
  model.name = MODEL_NAME

  # states
  x_ego, v_ego, a_ego = SX.sym('x_ego'), SX.sym('v_ego'), SX.sym('a_ego')
  model.x = vertcat(x_ego, v_ego, a_ego)

  # controls
  j_ego = SX.sym('j_ego')
  model.u = vertcat(j_ego)

  # xdot
  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  a_ego_dot = SX.sym('a_ego_dot')
  model.xdot = vertcat(x_ego_dot, v_ego_dot, a_ego_dot)

  # live parameters
  a_min = SX.sym('a_min')
  a_max = SX.sym('a_max')
  x_obstacle = SX.sym('x_obstacle')
  a_prev = SX.sym('a_prev')
  lead_t_follow = SX.sym('lead_t_follow')
  lead_danger_factor = SX.sym('lead_danger_factor')
  model.p = vertcat(a_min, a_max, x_obstacle, a_prev, lead_t_follow, lead_danger_factor)

  # dynamics model
  f_expl = vertcat(v_ego, a_ego, j_ego)
  model.f_impl_expr = model.xdot - f_expl
  model.f_expl_expr = f_expl
  return model

def gen_long_ocp():
  ocp = AcadosOcp()
  ocp.model = gen_long_model()

  Tf = T_IDXS[-1]

  # set dimensions
  ocp.dims.N = N

  # set cost module
  ocp.cost.cost_type = 'NONLINEAR_LS'
  ocp.cost.cost_type_e = 'NONLINEAR_LS'

  QR = np.zeros((COST_DIM, COST_DIM))
  Q = np.zeros((COST_E_DIM, COST_E_DIM))

  ocp.cost.W = QR
  ocp.cost.W_e = Q

  x_ego, v_ego, a_ego = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
  j_ego = ocp.model.u[0]

  a_min, a_max = ocp.model.p[0], ocp.model.p[1]
  x_obstacle = ocp.model.p[2]
  a_prev = ocp.model.p[3]
  lead_t_follow = ocp.model.p[4]
  lead_danger_factor = ocp.model.p[5]

  ocp.cost.yref = np.zeros((COST_DIM, ))
  ocp.cost.yref_e = np.zeros((COST_E_DIM, ))

  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow)

  # The main cost in normal operation is how close you are to the "desired" distance
  # from an obstacle at every timestep. This obstacle can be a lead car
  # or other object. In e2e mode we can use x_position targets as a cost
  # instead.
  costs = [((x_obstacle - x_ego) - (desired_dist_comfort)) / (v_ego + 10.),
           x_ego,
           v_ego,
           a_ego,
           a_ego - a_prev,
           j_ego]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])

  # Constraints on speed, acceleration and desired distance to
  # the obstacle, which is treated as a slack constraint so it
  # behaves like an asymmetrical cost.
  constraints = vertcat(v_ego,
                        (a_ego - a_min),
                        (a_max - a_ego),
                        ((x_obstacle - x_ego) - lead_danger_factor * (desired_dist_comfort)) / (v_ego + 10.))
  ocp.model.con_h_expr = constraints

  x0 = np.zeros(X_DIM)
  ocp.constraints.x0 = x0
  ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0, get_T_FOLLOW(), LEAD_DANGER_FACTOR])


  # We put all constraint cost weights to 0 and only set them at runtime
  cost_weights = np.zeros(CONSTR_DIM)
  ocp.cost.zl = cost_weights
  ocp.cost.Zl = cost_weights
  ocp.cost.Zu = cost_weights
  ocp.cost.zu = cost_weights

  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4*np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)

  # The HPIPM solver can give decent solutions even when it is stopped early
  # Which is critical for our purpose where compute time is strictly bounded
  # We use HPIPM in the SPEED_ABS mode, which ensures fastest runtime. This
  # does not cause issues since the problem is well bounded.
  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = 1

  # More iterations take too much time and less lead to inaccurate convergence in
  # some situations. Ideally we would run just 1 iteration to ensure fixed runtime.
  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3

  # set prediction horizon
  ocp.solver_options.tf = Tf
  ocp.solver_options.shooting_nodes = T_IDXS

  ocp.code_export_directory = EXPORT_DIR
  return ocp


class LongitudinalMpc:
  def __init__(self, dt=DT_MDL):
    self.dt = dt
    self.cruise_max_accel = CRUISE_MAX_ACCEL   # the planner raises it below 25 mph (pickup_accel.py)
    self._cf = CF_DEFAULT
    self._cf_cnt = 0
    self._cf_shift = 0.0
    self._cf_tf = None
    self._sd_aim = SD_DEFAULT
    self._sd_cnt = 0
    self._sd = 0.0
    self.stop_distance_on = True   # read by the planner's stop placement
    self._hf = HF_DEFAULT
    self._hf_cnt = 0
    self._hf_cut = 0.0
    self._ca = 0.0
    self._ca_cnt = 0
    self.ca_floor = ACCEL_MIN   # the planner reads it
    self._ca_on = False
    self._ca_block = 0
    self._ca_vl = [[], []]
    self._ca_dl = [None, None]
    self._ca_lb = [0, 0]
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.reset()
    self.source = LongitudinalPlanSource.cruise

  def reset(self):
    self.solver.reset()

    self.x_sol = np.zeros((N+1, X_DIM))
    self.u_sol = np.zeros((N, 1))
    self.v_solution = np.zeros(N+1)
    self.a_solution = np.zeros(N+1)
    self.j_solution = np.zeros(N)
    self.a_prev = np.array(self.a_solution)
    self.yref = np.zeros((N+1, COST_DIM))

    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params = np.zeros((N+1, PARAM_DIM))
    for i in range(N+1):
      self.solver.set(i, 'x', np.zeros(X_DIM))

    self.last_cloudlog_t = 0
    self.status = False
    self.crash_cnt = 0.0
    self.solution_status = 0
    # timers
    self.solve_time = 0.0
    self.time_qp_solution = 0.0
    self.time_linearization = 0.0
    self.time_integrator = 0.0
    self.x0 = np.zeros(X_DIM)
    self._last_dRel = [50.0, 50.0]          # per lead: [leadOne, leadTwo]
    self._lead_dropout_frames = [0, 0]
    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      # TODO don't hardcode A_CHANGE_COST idx
      # reduce the cost on (a-a_prev) later in the horizon.
      W[4,4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
      self.solver.cost_set(i, 'W', W)
    # Setting the slice without the copy make the array not contiguous,
    # causing issues with the C interface.
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))

    # Set L2 slack cost on lower bound constraints
    Zl = np.array(constraint_cost_weights)
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def set_weights(self, prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard):
    jerk_factor = get_jerk_factor(personality)
    a_change_cost = A_CHANGE_COST if prev_accel_constraint else 0
    cost_weights = [X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST, jerk_factor * a_change_cost, jerk_factor * J_EGO_COST]
    constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]
    self.set_cost_weights(cost_weights, constraint_cost_weights)

  def set_cur_state(self, v, a):
    v_prev = self.x0[1]
    self.x0[1] = v
    self.x0[2] = a
    if abs(v_prev - v) > 2.:  # probably only helps if v < v_prev
      for i in range(N+1):
        self.solver.set(i, 'x', self.x0)

  @staticmethod
  def extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau):
    a_lead_traj = a_lead * np.exp(-a_lead_tau * (T_IDXS**2)/2.)
    v_lead_traj = np.clip(v_lead + np.cumsum(T_DIFFS * a_lead_traj), 0.0, 1e8)
    x_lead_traj = x_lead + np.cumsum(T_DIFFS * v_lead_traj)
    lead_xv = np.column_stack((x_lead_traj, v_lead_traj))
    return lead_xv

  def process_lead(self, lead, idx):
    # Dropout state is per lead (idx 0 = leadOne, 1 = leadTwo). When it was shared, an absent
    # lead reused the other lead's distance and became a stopped phantom car at that spot.
    v_ego = self.x0[1]
    if lead is not None and lead.status:
      x_lead = lead.dRel
      last_dRel = self._last_dRel[idx]
      if last_dRel < 20.0 and last_dRel > 0:
        # Only limit how fast a lead can appear to move AWAY (that is what makes the car go).
        # A lead getting closer is taken at once, so cut-ins and closer leads are not delayed.
        max_increase = 0.5 if lead.vLead < 0.5 else 2.0
        x_lead = float(min(x_lead, last_dRel + max_increase))
      self._last_dRel[idx] = x_lead
      self._lead_dropout_frames[idx] = 0
      v_lead = lead.vLead
      a_lead = lead.aLeadK
      a_lead_tau = lead.aLeadTau
    else:
      # Fake a fast lead car, so mpc can keep running in the same mode
      self._lead_dropout_frames[idx] += 1
      # Highway follow: with its cut active the gap can be under 25 m at highway speed, where a stopped phantom
      # (clipped to v_ego^2/14 ahead) would be a brake stab (-3.5 m/s2 for the 0.5 s of a camera dropout at 65 mph in
      # the sim). Today's gap is only under 25 m below about 33 mph, so with the cut at 0 nothing changes.
      if (self._last_dRel[idx] < 25.0 and v_ego > 1.0 and self._lead_dropout_frames[idx] <= 10 and
              self._hf_cut == 0.0):
        x_lead = self._last_dRel[idx]
        v_lead = 0.0
        a_lead = 0.0
        a_lead_tau = _LEAD_ACCEL_TAU
      else:
        x_lead = 50.0
        v_lead = v_ego + 10.0
        a_lead = 0.0
        a_lead_tau = _LEAD_ACCEL_TAU

    # MPC will not converge if immediate crash is expected
    # Clip lead distance to what is still possible to brake for
    min_x_lead = MIN_X_LEAD_FACTOR * (v_ego + v_lead) * (v_ego - v_lead) / (-ACCEL_MIN * 2)
    x_lead = np.clip(x_lead, min_x_lead, 1e8)
    v_lead = np.clip(v_lead, 0.0, 1e8)
    a_lead = np.clip(a_lead, -10., 5.)
    lead_xv = self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)
    return lead_xv

  def update(self, radarstate, v_cruise, personality=log.LongitudinalPersonality.standard):
    t_follow = get_T_FOLLOW(personality)
    v_ego = self.x0[1]

    # close follow (see CF_* above)
    if self._cf_cnt % CF_FILE_PERIOD == 0:
      self._cf = read_close_follow()
    self._cf_cnt += 1
    cf_cut, cf_tf = self._cf
    cut_full = cf_cut + max(t_follow - cf_tf, 0.0) * v_ego
    w = 0.0
    if radarstate.leadOne.status and cut_full > 0.0:
      w = close_follow_weight(radarstate.leadOne, v_ego, t_follow, cut_full)
      if radarstate.leadTwo.status:
        w = min(w, close_follow_weight(radarstate.leadTwo, v_ego, t_follow, cut_full))
    shift_target = cf_cut * w
    tf_target = t_follow - max(t_follow - cf_tf, 0.0) * w
    self._cf_shift += float(np.clip(shift_target - self._cf_shift, -CF_SHIFT_OUT, CF_SHIFT_IN))
    if self._cf_tf is None:
      self._cf_tf = t_follow
    self._cf_tf = min(self._cf_tf + float(np.clip(tf_target - self._cf_tf, -CF_TF_IN, CF_TF_OUT)), t_follow)
    t_follow = self._cf_tf

    # highway follow (see HF_* above): a cut of t_follow, slewed, never below the file's time gap
    if self._hf_cnt % HF_FILE_PERIOD == 0:
      self._hf = read_highway_follow()
    self._hf_cnt += 1
    hf_t, hf_v = self._hf
    hf_on = hf_t > 0.0
    hf_cut_target = max(t_follow - hf_t, 0.0) * highway_follow_weight(v_ego, hf_v) if hf_on else 0.0
    hf_cut_max = max(t_follow - (hf_t if hf_on else HF_T_LIMITS[0]), 0.0)   # a personality change cannot undercut it
    self._hf_cut = min(self._hf_cut + float(np.clip(hf_cut_target - self._hf_cut, -HF_TF_OUT, HF_TF_IN)), hf_cut_max)
    t_follow = t_follow - self._hf_cut

    # stop distance (see SD_* above)
    if self._sd_cnt % SD_FILE_PERIOD == 0:
      self._sd_aim = read_stop_distance()
    self._sd_cnt += 1
    self.stop_distance_on = self._sd_aim > 0.0
    l1 = radarstate.leadOne
    dv_lead = float(l1.vLead) - v_ego if (l1.status and np.isfinite(l1.vLead)) else 0.0
    sd_target = stop_distance_shift(v_ego, self._sd_aim, dv_lead)
    self._sd += float(np.clip(sd_target - self._sd, -SD_SLEW, SD_SLEW))
    self.status = radarstate.leadOne.status or radarstate.leadTwo.status

    lead_xv_0 = self.process_lead(radarstate.leadOne, 0)
    lead_xv_1 = self.process_lead(radarstate.leadTwo, 1)

    # To estimate a safe distance from a moving lead, we calculate how much stopping
    # distance that lead needs as a minimum. We can add that to the current distance
    # and then treat that as a stopped car/obstacle at this new distance.
    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1]) + self._cf_shift - self._sd
    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1]) + self._cf_shift - self._sd

    # Fake an obstacle for cruise, this ensures smooth acceleration to set speed
    # when the leads are no factor.
    v_lower = v_ego + (T_IDXS * CRUISE_MIN_ACCEL * 1.05)
    # TODO does this make sense when max_a is negative?
    v_upper = v_ego + (T_IDXS * self.cruise_max_accel * 1.05)
    v_cruise_clipped = np.clip(v_cruise * np.ones(N+1), v_lower, v_upper)
    cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, t_follow)

    x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])
    self.source = MPC_SOURCES[np.argmin(x_obstacles[0])]

    self.yref[:,:] = 0.0
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])

    # coast-first approach (see CA_* above): a floor on the planned accel while closing with a large reserve
    if self._ca_cnt % CA_FILE_PERIOD == 0:
      self._ca = read_coast_approach()
    self._ca_cnt += 1
    if self._ca > 0.0 or self.ca_floor > ACCEL_MIN:
      self._coast_update(radarstate, v_ego, v_cruise, t_follow)

    self.params[:,0] = ACCEL_MIN if self.ca_floor <= ACCEL_MIN else self.ca_floor
    self.params[:,1] = ACCEL_MAX
    self.params[:,2] = np.min(x_obstacles, axis=1)
    self.params[:,3] = np.copy(self.a_prev)
    self.params[:,4] = t_follow
    self.params[:,5] = LEAD_DANGER_FACTOR

    self.run()
    if (np.any(lead_xv_0[FCW_IDXS,0] - self.x_sol[FCW_IDXS,0] < CRASH_DISTANCE) and
            radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

  def _coast_watch(self, lead, idx, v_ego):
    # per lead: True when it cut in, dropped out or brakes (confirmed by its speed); keeps the last second of its speed
    hist = self._ca_vl[idx]
    if lead is None or not lead.status or not (np.isfinite(lead.dRel) and np.isfinite(lead.vLead)):
      bad = len(hist) > 0     # a lead that was there and is gone
      hist.clear()
      self._ca_dl[idx] = None
      self._ca_lb[idx] = 0
      return bad
    last_d = self._ca_dl[idx]
    cut_in = last_d is not None and (last_d - max(v_ego - lead.vLead, 0.0) * self.dt - lead.dRel) > CA_CUTIN_D
    self._ca_dl[idx] = float(lead.dRel)
    hist.append(float(lead.vLead))
    if len(hist) > CA_LB_N + 1:
      hist.pop(0)
    hard_ok = CA_LB_ROOM_A > 0.0 and (lead.dRel - (CA_RES_D_MIN + CA_RES_T * v_ego + v_ego**2 / (2.0 * CA_RES_A_EGO) -
                                                   max(lead.vLead, 0.0)**2 / (2.0 * CA_LB_ROOM_A))) >= 0.0
    if np.isfinite(lead.aLeadK) and lead.aLeadK < CA_LB_A and max(hist) - hist[-1] >= CA_LB_DV and not hard_ok:
      self._ca_lb[idx] += 1
    else:
      self._ca_lb[idx] = 0
    return bool(cut_in or self._ca_lb[idx] >= CA_LB_CONFIRM or not np.isfinite(lead.aLeadK))

  def _coast_v_lead(self, lead, idx):
    # the lead speed the floor is worked out from: the lower of now and the last second's mean
    hist = self._ca_vl[idx]
    return min(float(lead.vLead), float(np.mean(hist))) if hist else float(lead.vLead)

  def _coast_update(self, radarstate, v_ego, v_cruise, t_follow):
    lead_one, lead_two = radarstate.leadOne, radarstate.leadTwo
    bad = self._coast_watch(lead_one, 0, v_ego)
    bad = self._coast_watch(lead_two, 1, v_ego) or bad
    self._ca_block = CA_BLOCK if bad else max(self._ca_block - 1, 0)
    a_fl, gate = ACCEL_MIN, 0
    urgent = bad or self._ca_block > 0
    if self._ca > 0.0 and self._ca_block == 0 and lead_one.status and v_cruise >= v_ego - 1.0:
      a_fl, gate = coast_gate(lead_one.dRel, self._coast_v_lead(lead_one, 0), v_ego, t_follow, self._ca)
      urgent = coast_reserve(lead_one.dRel, self._coast_v_lead(lead_one, 0), v_ego) < CA_RES[0]
      if lead_two.status:
        a_fl2, gate2 = coast_gate(lead_two.dRel, self._coast_v_lead(lead_two, 1), v_ego, t_follow, self._ca)
        a_fl = min(a_fl, a_fl2)
        urgent = urgent or coast_reserve(lead_two.dRel, self._coast_v_lead(lead_two, 1), v_ego) < CA_RES[0]
        gate = 0 if 0 in (gate, gate2) else (1 if gate == gate2 == 1 else None)
    if gate == 1 and (len(self._ca_vl[0]) <= CA_SETTLE or not lead_one.aLeadK >= CA_LB_A or
                      (lead_two.status and not lead_two.aLeadK >= CA_LB_A)):
      gate = None    # a lead seen for less than a second, or one that looks like it brakes, does not turn the floor on
    if gate is not None:
      self._ca_on = gate == 1
    target = a_fl if self._ca_on else ACCEL_MIN
    if v_ego < CA_V_MIN or not np.isfinite(target):
      self.ca_floor = ACCEL_MIN    # inert under 40 mph
      self._ca_on = False
      return
    if self.ca_floor <= ACCEL_MIN:
      if target <= ACCEL_MIN:
        return
      # start under the plan already running (and its next second), so the request does not jump up
      self.ca_floor = max(min(float(np.min(self.a_solution[T_IDXS <= 1.0])), self.x0[2], target), ACCEL_MIN)
    j_down = CA_J_DOWN_FAST if urgent else CA_J_DOWN
    step = float(np.clip(target - self.ca_floor, -j_down * self.dt, CA_J_UP * self.dt))
    self.ca_floor = max(self.ca_floor + step, ACCEL_MIN)
    if step < 0.0 and self.ca_floor < float(np.min(self.a_solution)) - CA_RELEASE:
      self.ca_floor = ACCEL_MIN    # the plan no longer touches it: gone, the solver is today's

  def run(self):
    for i in range(N+1):
      self.solver.set(i, 'p', self.params[i])
    self.solver.constraints_set(0, "lbx", self.x0)
    self.solver.constraints_set(0, "ubx", self.x0)

    self.solution_status = self.solver.solve()
    self.solve_time = float(self.solver.get_stats('time_tot')[0])
    self.time_qp_solution = float(self.solver.get_stats('time_qp')[0])
    self.time_linearization = float(self.solver.get_stats('time_lin')[0])
    self.time_integrator = float(self.solver.get_stats('time_sim')[0])

    for i in range(N+1):
      self.x_sol[i] = self.solver.get(i, 'x')
    for i in range(N):
      self.u_sol[i] = self.solver.get(i, 'u')

    self.v_solution = self.x_sol[:,1]
    self.a_solution = self.x_sol[:,2]
    self.j_solution = self.u_sol[:,0]

    self.a_prev = np.interp(T_IDXS + self.dt, T_IDXS, self.a_solution)

    t = time.monotonic()
    if self.solution_status != 0:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning(f"Long mpc reset, solution_status: {self.solution_status}")
      self.reset()


if __name__ == "__main__":
  ocp = gen_long_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
