#!/usr/bin/env python3
import os
import time
import numpy as np
from cereal import log
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
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

# Set speed ease at low set speeds. With no car ahead the MPC eases off toward the set speed very
# early: with SET pressed at a stop (40 km/h) it leaves the pickup cap at ~14 mph and creeps up the
# last 10 mph at ~0.4 m/s3 (the obstacle-distance cost makes that approach so soft; the jerk costs
# and the cruise obstacle's shape hardly matter). Here the cruise fake obstacle aims above the set
# speed while the speed still left is more than what rolling the MPC's accel off at EASE_JERK adds:
#   target = v_cruise + min(EASE_GAIN * max(v_cruise - v - a^2 / (2 * EASE_JERK), 0), EASE_MAX)
# with v, a the MPC's own speed and accel. The extra is gone before the set speed, so the plan
# still ends at the set speed (sim: overshoot up to ~1.1 mph instead of ~0.8 with the car giving all
# that is asked, ~0.6 instead of ~0.5 at the 90 % this car gives). It only
# moves the cruise obstacle further away: wherever a lead obstacle is closer, the MPC still uses
# the lead. Full for set speeds up to 30 mph, none from 39.6 mph (so SET at a stop = 64 km/h on
# t2_set_min is exactly as before); nothing at or above the set speed.
# EASE_JERK is for aggressive; standard and relaxed (twice the jerk cost) use half of it.
# Live: /data/cruise_ease holds EASE_JERK in m/s3 (default 0.8; 0 = off, exactly as before;
# more = holds the accel longer and rolls off later, with more overshoot: 1.0 -> up to 1.2 mph).
EASE_DEFAULT = 0.8           # m/s3
EASE_JERK_FACTOR = 0.5       # get_jerk_factor() that EASE_JERK is meant for (aggressive)
EASE_FILE = '/data/cruise_ease'
EASE_LIMITS = (0.0, 1.5)     # m/s3
EASE_GAIN = 2.0              # m/s of extra target per m/s of speed left
EASE_MAX = 3.0               # m/s
EASE_V_CRUISE = [13.4, 17.7]  # m/s set speed (30 / 39.6 mph): full .. none (64 km/h = 17.78 m/s gets none)
EASE_FILE_PERIOD = 20        # updates between file reads


def read_cruise_ease():
  try:
    with open(EASE_FILE) as f:
      val = float(f.read().strip())
  except (OSError, ValueError):
    return EASE_DEFAULT
  if not np.isfinite(val):
    return EASE_DEFAULT
  return float(np.clip(val, *EASE_LIMITS))


def cruise_ease_target(v_cruise, v_ego, a_ego, jerk):
  # the cruise obstacle's target speed (see EASE_* above); v_cruise itself when off or not low speed
  if jerk <= 0.0:
    return v_cruise
  fade = float(np.interp(v_cruise, EASE_V_CRUISE, [1.0, 0.0]))
  a = max(a_ego, 0.0)
  room = v_cruise - v_ego - a * a / (2 * jerk)
  if fade <= 0.0 or room <= 0.0:
    return v_cruise
  return v_cruise + fade * min(EASE_GAIN * room, EASE_MAX)


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
    self._ease = EASE_DEFAULT
    self._ease_cnt = 0
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
      if self._last_dRel[idx] < 25.0 and v_ego > 1.0 and self._lead_dropout_frames[idx] <= 10:
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
    if self._ease_cnt % EASE_FILE_PERIOD == 0:
      self._ease = read_cruise_ease()
    self._ease_cnt += 1
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
    self.status = radarstate.leadOne.status or radarstate.leadTwo.status

    lead_xv_0 = self.process_lead(radarstate.leadOne, 0)
    lead_xv_1 = self.process_lead(radarstate.leadTwo, 1)

    # To estimate a safe distance from a moving lead, we calculate how much stopping
    # distance that lead needs as a minimum. We can add that to the current distance
    # and then treat that as a stopped car/obstacle at this new distance.
    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1]) + self._cf_shift
    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1]) + self._cf_shift

    # Fake an obstacle for cruise, this ensures smooth acceleration to set speed
    # when the leads are no factor.
    v_lower = v_ego + (T_IDXS * CRUISE_MIN_ACCEL * 1.05)
    # TODO does this make sense when max_a is negative?
    v_upper = v_ego + (T_IDXS * self.cruise_max_accel * 1.05)
    ease_jerk = self._ease * EASE_JERK_FACTOR / get_jerk_factor(personality)
    v_cruise_clipped = np.clip(cruise_ease_target(v_cruise, v_ego, self.x0[2], ease_jerk) * np.ones(N+1), v_lower, v_upper)
    cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, t_follow)

    x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])
    self.source = MPC_SOURCES[np.argmin(x_obstacles[0])]

    self.yref[:,:] = 0.0
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params[:,0] = ACCEL_MIN
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
