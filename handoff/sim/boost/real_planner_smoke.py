# Runs the REAL selfdrive/controls/lib/longitudinal_planner.py from a git ref (with the real long_mpc.py,
# drive_helpers.py, filter_simple.py, constants.py and launch_boost.py from that ref; only cereal messaging,
# longcontrol's enum, cruise constants and the sunnypilot base class are stubbed) in the same closed loop as
# boostsim, and compares its aTarget/shouldStop with the harness's BoostPlanner (mpcsim replay + boost
# post-processing). Proves the planner wiring executes and that the harness models it exactly.
# usage: real_planner_smoke.py <ref>
import sys, types, importlib, numpy as np
from types import SimpleNamespace as NS
import boostsim as B
M = B.M
REF = sys.argv[1] if len(sys.argv) > 1 else 'WT'
T0 = 8.0


def src_of(path):
  return open(f"{B.WT}/{path}").read() if REF == 'WT' else M.git_show(REF, path)


def load_as(name, path, post=None):
  mod = types.ModuleType(name)
  exec(compile(src_of(path), f"{REF}:{path}", "exec"), mod.__dict__)
  if post:
    post(mod)
  sys.modules[name] = mod
  return mod


def stub(name, **kw):
  m = sys.modules.get(name) or types.ModuleType(name)
  m.__dict__.update(kw)
  m.__path__ = []
  sys.modules[name] = m
  return m


LongCtrlState = NS(off=0, pid=1, stopping=2, starting=3)


class LongitudinalPlannerSP:   # sunnypilot base: SCC/SLA/DEC off -> targets pass through
  def __init__(self, CP, CP_SP, mpc):
    pass

  def update(self, sm):
    pass

  def update_targets(self, sm, v_ego, a_ego, v_cruise):
    return v_cruise, a_ego

  def is_e2e(self, sm):
    return sm['selfdriveState'].experimentalMode


stub('cereal.messaging')
sys.modules['cereal'].messaging = sys.modules['cereal.messaging']
load_as('openpilot.common.constants', 'common/constants.py')
load_as('openpilot.common.filter_simple', 'common/filter_simple.py')
stub('openpilot.selfdrive.controls.lib.longcontrol', LongCtrlState=LongCtrlState)
load_as('openpilot.selfdrive.controls.lib.drive_helpers', 'selfdrive/controls/lib/drive_helpers.py')
sys.modules['openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc'] = M.load_mpc_module('t2_lead2_phantom_fix' if REF == 'WT' else REF)
load_as('openpilot.selfdrive.controls.lib.launch_boost', 'selfdrive/controls/lib/launch_boost.py',
        post=lambda m: setattr(m, 'LIVE_FILE', '/nonexistent/launch_boost'))
stub('openpilot.selfdrive.car'); stub('openpilot.selfdrive.car.cruise', V_CRUISE_MAX=145, V_CRUISE_UNSET=255)
for n in ('openpilot.sunnypilot', 'openpilot.sunnypilot.selfdrive', 'openpilot.sunnypilot.selfdrive.controls',
          'openpilot.sunnypilot.selfdrive.controls.lib'):
  stub(n)
stub('openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner', LongitudinalPlannerSP=LongitudinalPlannerSP)
LP = load_as('openpilot.selfdrive.controls.lib.longitudinal_planner', 'selfdrive/controls/lib/longitudinal_planner.py')

CP = NS(steerRatio=15.6, wheelbase=2.63, longitudinalActuatorDelay=0.5, vEgoStopping=0.5, openpilotLongitudinalControl=True)
MODEL = NS(position=NS(x=[]), velocity=NS(x=[]), acceleration=NS(x=[]),
           meta=NS(disengagePredictions=NS(gasPressProbs=[1.0, 1.0])), action=NS(desiredAcceleration=0.0, shouldStop=False))


def run_real(T, lead_a, gap0, lead2=None, driver_brake_t=None, experimental=False):
  planner = LP.LongitudinalPlanner(CP, None)
  plant = M.Plant(0.0); lc = M.LongControl()
  lead = B.Car(gap0, 0.0, lead_a); other = None
  a_t, ss, a_send = 0.0, True, 0.0
  out = []
  for k in range(int(round(T / M.DT_CTRL))):
    t = k * M.DT_CTRL
    lead.step(t)
    if lead2 is not None and other is None and t >= lead2['t_on']:
      other = B.Car(plant.x + lead2['d'], lead2['v'], lambda tt, c: 0.0)
    if other is not None:
      other.step(t)
    brake = driver_brake_t is not None and t >= driver_brake_t
    if k % 5 == 0:
      gap = lead.x - plant.x
      l1 = M.lead_msg(gap, lead.v, lead.a); l2 = l1
      if other is not None and other.x - plant.x < gap:
        l1, l2 = M.lead_msg(other.x - plant.x, other.v, 0.0), l1
      sm = {'carControl': NS(orientationNED=[0.0, 0.0, 0.0], longActive=not brake, enabled=True),
            'carState': NS(vEgo=plant.v, vCruise=126.0, vCruiseCluster=126.0, standstill=plant.v == 0.0, steeringAngleDeg=0.0,
                           aEgo=plant.a, gasPressed=False, brakePressed=brake),
            'controlsState': NS(longControlState=LongCtrlState.stopping if lc.state == M.STOPPING else LongCtrlState.pid, forceDecel=False),
            'selfdriveState': NS(enabled=True, personality=1, experimentalMode=experimental),
            'liveParameters': NS(angleOffsetDeg=0.0), 'modelV2': MODEL, 'radarState': NS(leadOne=l1, leadTwo=l2)}
      planner.update(sm)
      a_t, ss = float(planner.output_a_target), bool(planner.output_should_stop)
      out.append((t, a_t, ss, planner.launch_boost.boost, planner.launch_boost.event))
    if brake:
      lc.state, lc.last, a_out = M.STOPPING, 0.0, 0.0
    else:
      a_out = lc.update(plant.v, a_t, ss)
    if k % 2 == 0:
      a_send = float(np.clip(a_out, M.CCP_ACCEL_MIN, M.CCP_ACCEL_MAX)) if not brake else 0.0
    plant.step(-3.0 if brake else a_send, False)
  return out


if __name__ == '__main__':
  m, cfg = M.load_ref('t2_lead2_phantom_fix' if REF == 'WT' else REF)
  bm = B.load_boost(REF)
  cases = [('launch aL1.6 gap4.5', dict(lead_a=B.launch(T0, 1.6), gap0=4.5)),
           ('launch aL1.0 gap3.5', dict(lead_a=B.launch(T0, 1.0), gap0=3.5)),
           ('launch then brake 4.0 after 2 s', dict(lead_a=B.launch_then_brake(T0, 1.6, 2.0, 4.0), gap0=4.5)),
           ('cut-in 4 m v=1 at t+2 s', dict(lead_a=B.launch(T0, 1.6), gap0=4.5, lead2=dict(t_on=T0 + 2.0, d=4.0, v=1.0))),
           ('driver brake at t+1.5 s', dict(lead_a=B.launch(T0, 1.6), gap0=4.5, driver_brake_t=T0 + 1.5))]
  ok = True
  for name, kw in cases:
    real = run_real(T0 + 8, **kw)
    lead2 = kw.get('lead2'); h_kw = {}
    if lead2:
      h_kw['lead2'] = dict(t_on=lead2['t_on'], d=lead2['d'], v=lead2['v'], a=0.0, role='cutin')
    if kw.get('driver_brake_t'):
      h_kw['driver_brake_t'] = kw['driver_brake_t']
    r = B.run(m, cfg, bm, T0 + 8, kw['lead_a'], gap0=kw['gap0'], mode='ideal', **h_kw)
    a_real = np.array([x[1] for x in real]); ss_real = np.array([x[2] for x in real])
    da = np.max(np.abs(a_real - r.a_target)); dss = int(np.sum(ss_real != r.ss))
    ev_real = [(round(x[0], 2), x[4]) for x in real if x[4]]
    good = da < 1e-6 and dss == 0 and ev_real == r.events
    ok &= good
    print(f"{'OK  ' if good else 'DIFF'} {name:34s} max|aTarget real - harness| {da:.1e}  shouldStop mismatches {dss}  events real {ev_real}  harness {r.events}")
  # experimental (e2e) mode: boost must stay off
  real = run_real(T0 + 8, B.launch(T0, 1.6), 4.5, experimental=True)
  e2e_ok = all(x[3] == 0.0 for x in real) and not any(x[4] for x in real)
  ok &= e2e_ok
  print(f"{'OK  ' if e2e_ok else 'DIFF'} experimental mode: boost never on")
  print('REAL PLANNER == HARNESS' if ok else 'MISMATCH')
