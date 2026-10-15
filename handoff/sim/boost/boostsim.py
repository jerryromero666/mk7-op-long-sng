"""
Closed-loop launch-boost harness. Reuses ../mpcsim.py (real long_mpc.py from a git ref, compiled
acados solver, line-by-line Planner/LongControl replay, delay+lag plant) unchanged, and adds:

  * BoostPlanner: wraps mpcsim.Planner. After the replayed planner update it applies the
    launch_boost module loaded from a git ref exactly like the branch's longitudinal_planner.py:
      a_mpc, ss_mpc = get_accel_from_plan(...)            (captured from the replay)
      boost = LaunchBoost.update(longActive, vEgo, gas, brake, leadOne, leadTwo, a_mpc, crash_cnt)
      if boost > 0: a_mpc = max(a_mpc, boost); ss_mpc = False; a_desired = max(a_desired, boost)
      if LaunchBoost.mpc_behind(v_desired_filter.x, vEgo): v_desired_filter.x = vEgo
      a_out = clip(a_mpc, accel_clip)                     (same rate-limited clip as the replay)
    check_wiring() asserts those lines appear in this order in the ref's planner.
    clip(max(clip(x), b)) == clip(max(x, b)) because the clip is monotone, so post-processing the
    replay's clipped output with the same clip is identical to the in-planner wiring.
  * vision-like lead measurements: white + correlated noise on dRel / vLead / aLeadK, first-order
    lags on vLead and aLeadK, aLeadTau 0.3 (radard vision value); 'ideal' = exact, tau 1.5.
  * scenario hooks: second car (cut-in or leadTwo), lead dropout, driver brake, gas.
  * optional ECU accel-gradient limit (ACC_06 pos 4.0 / neg 1.0 m/s3 on this lineage).
"""
import os
import subprocess
import sys
import types
from types import SimpleNamespace as NS

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import mpcsim as M  # noqa: E402

REPO = M.REPO
BOOST_PATH = "selfdrive/controls/lib/launch_boost.py"
WT = "/tmp/claude-0/-home-user-mk7-op-long-sng/48e1abac-3f14-57e3-a7c0-d6a536957af8/scratchpad/wt_boost"


def load_boost(ref, overrides=None):
  """launch_boost.py from a git ref ('WT' = worktree file). Live file pointed at a missing path."""
  if ref == 'WT':
    src = open(os.path.join(WT, BOOST_PATH)).read()
  else:
    src = subprocess.check_output(["git", "-C", REPO, "show", f"{ref}:{BOOST_PATH}"], text=True)
  mod = types.ModuleType(f"launch_boost_{ref}")
  exec(compile(src, f"{ref}:{BOOST_PATH}", "exec"), mod.__dict__)
  mod.LIVE_FILE = "/nonexistent/launch_boost"
  check_wiring(ref)
  for k, v in (overrides or {}).items():
    assert hasattr(mod, k), k
    setattr(mod, k, v)
  return mod


def check_wiring(ref):
  """Textual check that the ref's planner wires the boost exactly the way BoostPlanner models it."""
  src = M.git_show(ref, M.PLANNER_PATH) if ref != 'WT' else open(os.path.join(WT, M.PLANNER_PATH)).read()
  lines = [
    "output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(",
    "launch_boost = self.launch_boost.update(sm['carControl'].longActive, v_ego, sm['carState'].gasPressed,",
    "sm['carState'].brakePressed, sm['radarState'].leadOne,",
    "sm['radarState'].leadTwo, output_a_target_mpc, self.mpc.crash_cnt)",
    "if launch_boost > 0.0:",
    "output_a_target_mpc = max(output_a_target_mpc, launch_boost)",
    "output_should_stop_mpc = False",
    "self.a_desired = max(self.a_desired, launch_boost)",
    "if self.launch_boost.mpc_behind(self.v_desired_filter.x, v_ego):",
    "self.v_desired_filter.x = v_ego",
    "if self.is_e2e(sm):",
    "self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])",
  ]
  i = src.index("    # Launch boost from a dead stop")
  for x in lines[1:]:
    i = src.index(x, i) + len(x)   # each line must follow the previous one (ValueError if missing)
  assert src.count("self.launch_boost.update(") == 1 and src.index(lines[0]) < src.index(lines[1])
  return True


class BoostPlanner(M.Planner):
  def __init__(self, mpc_mod, cfg, boost_mod=None, **kw):
    super().__init__(mpc_mod, cfg, **kw)
    self.lb = boost_mod.LaunchBoost(dt=M.DT_MDL) if boost_mod is not None else None
    self.boost = 0.0
    self.a_mpc = 0.0

  def update(self, v_ego, standstill, rs, v_cruise, drv=None):
    cap = {}
    orig = M.get_accel_from_plan

    def spy(*a, **k):
      r = orig(*a, **k)
      cap['a'], cap['ss'] = r
      return r
    M.get_accel_from_plan = spy
    try:
      a_out, ss = super().update(v_ego, standstill, rs, v_cruise)
    finally:
      M.get_accel_from_plan = orig
    self.a_mpc = float(cap['a'])
    self.boost = 0.0
    if self.lb is not None:
      drv = drv or NS(long_active=True, gas=False, brake=False)
      self.boost = self.lb.update(drv.long_active, v_ego, drv.gas, drv.brake, rs.leadOne, rs.leadTwo,
                                  self.a_mpc, self.mpc.crash_cnt)
      if self.boost > 0.0:
        a_out = float(np.clip(max(self.a_mpc, self.boost), self.prev_accel_clip[0], self.prev_accel_clip[1]))
        ss = False
        self.a_desired = max(self.a_desired, self.boost)
      if self.lb.mpc_behind(self.v_f_x, v_ego):
        self.v_f_x = v_ego
    self.output_a_target, self.output_should_stop = a_out, ss
    return a_out, ss


# ---------------------------------------------------------------- measurement noise

class Noise:
  """white + AR(1) noise with total sigma, half the variance each (corr tau in s, 20 Hz)."""
  def __init__(self, rng, sigma, tau=0.5, white_frac=0.5):
    self.rng, self.sw, self.sc = rng, sigma * np.sqrt(white_frac), sigma * np.sqrt(1 - white_frac)
    self.phi = np.exp(-M.DT_MDL / tau)
    self.c = self.rng.normal(0, self.sc) if self.sc > 0 else 0.0

  def __call__(self):
    if self.sc > 0:
      self.c = self.phi * self.c + self.rng.normal(0, self.sc * np.sqrt(1 - self.phi ** 2))
    return self.c + (self.rng.normal(0, self.sw) if self.sw > 0 else 0.0)


class Sensor:
  """One tracked car as radard would report it."""
  def __init__(self, rng, mode, sd=0.3, sv=0.4, sa=0.3, tau_v=0.2, tau_a=0.5, white_frac=0.5):
    self.mode = mode
    if mode == 'vision':
      self.nd, self.nv, self.na = Noise(rng, sd, 0.5, white_frac), Noise(rng, sv, 0.5, white_frac), Noise(rng, sa, 0.5, white_frac)
    self.tau_v, self.tau_a = tau_v, tau_a
    self.vl = None
    self.al = 0.0

  def read(self, d, v, a, status=True):
    if not status:
      self.vl = None
      return M.NO_LEAD
    if self.mode == 'ideal':
      return NS(status=True, dRel=float(d), vLead=float(v), aLeadK=float(a), aLeadTau=1.5, modelProb=1.0)
    if self.vl is None:
      self.vl, self.al = v, a
    self.vl += (v - self.vl) * M.DT_MDL / (self.tau_v + M.DT_MDL)
    self.al += (a - self.al) * M.DT_MDL / (self.tau_a + M.DT_MDL)
    return NS(status=True, dRel=float(d + self.nd()), vLead=float(max(self.vl + self.nv(), 0.0)),
              aLeadK=float(self.al + self.na()), aLeadTau=0.3, modelProb=1.0)


# ---------------------------------------------------------------- scenario + closed loop

class Car:
  def __init__(self, x, v, a_fn):
    self.x, self.v, self.a_fn, self.a = x, v, a_fn, 0.0

  def step(self, t):
    self.a = self.a_fn(t, self)
    if self.v <= 0 and self.a < 0:
      self.a = 0.0
    self.v = max(0.0, self.v + self.a * M.DT_CTRL)
    self.x += self.v * M.DT_CTRL


def run(mpc_mod, cfg, boost_mod, T, lead_a, gap0=4.5, mode='vision', seed=0, noise_kw=None,
        lead2=None, lead1_lost=None, driver_brake_t=None, gas_t=None, ecu_grad=None,
        plant_delay=0.15, plant_tau=0.35, v_cruise=35.0, lead2_mode='mirror', probe=None):
  """
  lead_a(t, car) -> accel of lead car (starts stopped, gap0 ahead of a stopped ego).
  lead2: optional dict(t_on, t_off, d=<dRel at t_on> or d_rel_lead1=<offset vs lead1>, v, a, role)
         role 'cutin'  -> becomes leadOne (and leadTwo = old lead) while closer than lead1
         role 'lead2'  -> reported as leadTwo only
  lead1_lost: (t_from, t_to) leadOne.status False (leadTwo too, as radard would).
  driver_brake_t / gas_t: time of pedal press (held to the end).
  ecu_grad: (pos, neg) m/s3 accel-request gradient limit inside the car.
  lead2_mode: 'mirror' = leadTwo is the same car as leadOne with its own noise; 'none'.
  """
  rng = np.random.default_rng(seed)
  nk = noise_kw or {}
  s1, s2, s3 = Sensor(rng, mode, **nk), Sensor(rng, mode, **nk), Sensor(rng, mode, **nk)
  plant = M.Plant(0.0, plant_delay, plant_tau)
  planner = BoostPlanner(mpc_mod, cfg, boost_mod, init_v=0.0)
  lc = M.LongControl()
  lead = Car(gap0, 0.0, lead_a)
  other = None
  a_target, ss = 0.0, True
  a_send, a_ecu = 0.0, 0.0
  rec = NS(t=[], v=[], a=[], gap=[], gap_true=[], a_cmd=[], a_target=[], a_mpc=[], boost=[], ss=[], lv=[],
           events=[], clip_hi=[], stop_to_pid=0)
  prev_state = lc.state
  for k in range(int(round(T / M.DT_CTRL))):
    t = k * M.DT_CTRL
    lead.step(t)
    if lead2 is not None and t >= lead2['t_on'] and other is None and plant.v >= lead2.get('v_on', -1.0) and not lead2.get('_done'):
      lead2['_done'] = True
      d0 = lead2['d'] if 'd' in lead2 else (lead.x - plant.x) + lead2['d_rel_lead1']
      other = Car(plant.x + d0, lead2['v'] if lead2['v'] is not None else plant.v, lambda tt, c: lead2.get('a', 0.0))
    if other is not None:
      other.step(t)
      if lead2.get('t_off') is not None and t >= lead2['t_off']:
        other = None
    gap = lead.x - plant.x
    gap_true = gap
    if other is not None and lead2['role'] == 'cutin':
      gap_true = min(gap, other.x - plant.x)
    brake = driver_brake_t is not None and t >= driver_brake_t
    gas = gas_t is not None and t >= gas_t
    long_active = not brake
    drv = NS(long_active=long_active, gas=gas, brake=brake)
    if k % 5 == 0:
      lost = lead1_lost is not None and lead1_lost[0] <= t < lead1_lost[1]
      l1 = s1.read(gap, lead.v, lead.a, status=not lost)
      if lead2_mode == 'mirror':
        l2 = s2.read(gap, lead.v, lead.a, status=not lost)
      else:
        l2 = M.NO_LEAD
      if other is not None:
        od = other.x - plant.x
        lo = s3.read(od, other.v, other.a, status=not lost)
        if lead2['role'] == 'cutin' and od < gap:
          l1, l2 = lo, l1
        elif lead2['role'] == 'lead2':
          l2 = lo
      rs = NS(leadOne=l1, leadTwo=l2)
      a_target, ss = planner.update(plant.v, plant.v == 0.0, rs, v_cruise, drv)
      if planner.lb is not None and planner.lb.event:
        rec.events.append((round(t, 2), planner.lb.event))
      if probe is not None:
        probe(t, planner, plant, rs)
    if long_active:
      a_out = lc.update(plant.v, a_target, ss)
    else:
      lc.state, lc.last, a_out = M.STOPPING, 0.0, 0.0
    if lc.state != prev_state and prev_state == M.STOPPING:
      rec.stop_to_pid += 1
    prev_state = lc.state
    if k % 2 == 0:
      a_send = float(np.clip(a_out, M.CCP_ACCEL_MIN, M.CCP_ACCEL_MAX)) if long_active else 0.0
    if ecu_grad is not None:
      a_ecu = float(np.clip(a_send, a_ecu - ecu_grad[1] * M.DT_CTRL, a_ecu + ecu_grad[0] * M.DT_CTRL))
    else:
      a_ecu = a_send
    plant.step(-3.0 if brake else a_ecu, False)
    if k % 5 == 0:
      rec.t.append(t); rec.v.append(plant.v); rec.a.append(plant.a); rec.gap.append(gap); rec.gap_true.append(gap_true)
      rec.a_cmd.append(a_send); rec.a_target.append(a_target); rec.a_mpc.append(planner.a_mpc)
      rec.boost.append(planner.boost); rec.ss.append(ss); rec.lv.append(lead.v); rec.clip_hi.append(planner.prev_accel_clip[1])
  for f in ('t', 'v', 'a', 'gap', 'gap_true', 'a_cmd', 'a_target', 'a_mpc', 'boost', 'lv', 'clip_hi'):
    setattr(rec, f, np.array(getattr(rec, f)))
  rec.ss = np.array(rec.ss)
  return rec


# ---------------------------------------------------------------- lead profiles

def launch(t0, a, v_max=13.4):
  return lambda t, c: a if (t >= t0 and c.v < v_max) else 0.0


def launch_then_brake(t0, a, t_brake, decel):
  return lambda t, c: (a if t < t0 + t_brake else -decel) if t >= t0 else 0.0


def creep(t0, v_creep, t_creep, a=0.5):
  def f(t, c):
    if t < t0:
      return 0.0
    if t < t0 + t_creep:
      return a if c.v < v_creep else 0.0
    return -a
  return f


def stationary():
  return lambda t, c: 0.0


# ---------------------------------------------------------------- metrics

def launch_metrics(r, T0):
  t, v, a, g = r.t, r.v, r.a, r.gap_true
  i = lambda tt: min(np.searchsorted(t, T0 + tt), len(t) - 1)
  mv = np.where((t >= T0) & (v > 0.1))[0]
  delay = t[mv[0]] - T0 if len(mv) else np.nan
  if len(mv):
    j0 = mv[0]; j1 = np.searchsorted(t, t[j0] + 1.0)
    a1 = float(np.mean(a[j0:j1]))
  else:
    a1 = np.nan
  v5 = np.where((t >= T0) & (v >= 5.0))[0]
  post = t >= T0
  jerk_cmd = np.max(np.abs(np.diff(r.a_cmd[post]))) / M.DT_MDL
  jerk_act = np.max(np.abs(np.diff(a[post]))) / M.DT_MDL
  moving = np.where(post & (v > 0.1))[0]          # after ego starts: excludes the stopping->pid step
  jerk_run = np.max(np.abs(np.diff(r.a_target[moving[0]:]))) / M.DT_MDL if len(moving) > 1 else np.nan
  fired = [e for e in r.events if e[1] == 'fired']
  ends = [e for e in r.events if e[1] != 'fired']
  return dict(delay=delay, a1=a1, v2=v[i(2)], v4=v[i(4)], t5=(t[v5[0]] - T0) if len(v5) else np.nan,
              gg4=g[i(4)] - g[i(0) - 1], gmin=float(np.min(g[post])), apk=float(np.max(a[post])),
              acmd=float(np.max(r.a_cmd[post])), jcmd=jerk_cmd, jact=jerk_act, jrun=jerk_run,
              fire=(fired[0][0] - T0) if fired else np.nan, end=ends[0][1] if ends else '',
              end_t=(ends[0][0] - T0) if ends else np.nan,
              over_clip=float(np.max(r.a_target - r.clip_hi)), bmax=float(np.max(r.boost)))
