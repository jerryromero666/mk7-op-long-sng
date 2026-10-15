"""
Launch boost: pull away more briskly from a dead stop behind a departing lead.

From a stop the MPC answers a departing lead with a slow, jerk-limited ramp: on this car the plan
peaks ~1.4 m/s2 and the first second averages ~0.6 m/s2. No MPC weight, accel cap or follow
time changes that much (closed-loop sim). This adds a feed-forward accel driven by the lead.

The planner sends max(MPC, boost) before its e2e blend and accel clip, so both still apply, as
do the car's limits. The boost only ever adds accel; it never brakes.

  arm      ego stopped (long active, no pedals) behind a stopped lead for ARM_TIME; latch the gap
  trigger  the MPC already asks to go (so the boost never takes the car out of stopping by
           itself), and the lead is pulling away: vLead > V_TRIGGER and the gap has opened
           D_TRIGGER beyond the stop gap, for TRIGGER_FRAMES in a row
  boost    the smallest of three bounds, each needing its own evidence that the lead is leaving:
             speed  K_SPEED * (vLead - vEgo - V_MARGIN)   0 once ego nears the lead's speed
             gap    K_GAP * (gap - stop gap - D_GAP0 - T_GAP * vEgo)   never closes in on it
             accel  aLeadK + A_MARGIN                     about the lead's own accel
           ramped at J_UP / J_DOWN, capped at the live max (default BOOST_MAX)
  handover fade out at J_DOWN: ego above V_HANDOVER, MPC asks for at least the speed bound,
           or BOOST_TIME
  abort    boost 0 at once: lead lost, leadTwo closer than leadOne, gap shrinking, closing on
           the lead, lead falls back below V_CONFIRM, MPC asks to brake, crash risk, gas or
           brake pressed, long inactive
After it fires it stays off until ego has driven off and stopped again.

While it boosts, the planner also puts the MPC's speed and accel state on the car's (mpc_behind),
so an abort hands the MPC the car as it really is, not its own slower plan.

Live tuning: /data/launch_boost holds one number, the max boost in m/s2 (0 turns it off).
Re-read about once a second, clamped to [0, BOOST_MAX_LIMIT]. No file: BOOST_MAX.
"""
import numpy as np

from openpilot.common.realtime import DT_MDL

V_EGO_STOPPED = 0.1      # m/s, ego counts as stopped below this
V_EGO_DROVE = 1.0        # m/s, ego must pass this before the next stop can arm again
V_ARMED_MAX = 2.0        # m/s, once armed it may still trigger while the MPC rolls ego up to this
ARM_TIME = 1.0           # s
V_LEAD_STOPPED = 0.5     # m/s, filtered vLead below this is a stopped lead
A_MPC_GO = 0.1           # m/s2, MPC asks for more than this (its should_stop is False)
V_TRIGGER = 0.9          # m/s
A_TRIGGER = 0.6          # m/s2
D_TRIGGER = 0.4          # m
TRIGGER_FRAMES = 3
K_SPEED = 1.5            # 1/s
V_MARGIN = 0.3           # m/s
K_GAP = 3.0              # 1/s2
D_GAP0 = 0.2             # m
T_GAP = 1.0              # s
A_MARGIN = 0.3           # m/s2
BOOST_MAX = 1.5          # m/s2, below the planner's own clip at launch (1.6 at 0 m/s)
BOOST_MAX_LIMIT = 1.6    # m/s2, upper clamp for the live value
J_UP = 2.5               # m/s3
J_DOWN = 1.5             # m/s3
V_HANDOVER = 5.0         # m/s
BOOST_TIME = 6.0         # s
LEAD2_MARGIN = 1.5       # m, leadTwo this much closer than leadOne
D_SHRINK = 1.0           # m, gap this far below its peak since the trigger
V_CLOSING = 0.5          # m/s, lead this much slower than ego
V_CONFIRM = 0.6          # m/s, lead back below this: noise or a creeping lead, not a departure
A_MPC_BRAKE = -0.3       # m/s2
TAU_D = 0.15             # s, dRel filter
TAU_V = 0.1              # s, vLead filter
TAU_A = 0.1              # s, aLeadK filter
TAU_LATCH = 1.0          # s, stop gap filter while the lead is stopped
LIVE_FILE = '/data/launch_boost'
LIVE_PERIOD = 20         # updates between file reads (1 Hz at 20 Hz)


def read_live_max():
  try:
    with open(LIVE_FILE) as f:
      val = float(f.read().strip())
  except (OSError, ValueError):
    return BOOST_MAX
  if not np.isfinite(val):
    return BOOST_MAX
  return float(np.clip(val, 0.0, BOOST_MAX_LIMIT))


class LaunchBoost:
  def __init__(self, dt=DT_MDL):
    self.dt = dt
    self.boost_max = BOOST_MAX
    self.live_cnt = 0
    self.spent = False       # fired since ego last drove off
    self.event = ''          # set on the update the boost fires, hands over or aborts (for logging)
    self.d_f = None          # filtered leadOne dRel, vLead, aLeadK (d_f None while there is no lead)
    self.v_f = 0.0
    self.a_f = 0.0
    self.sync = False
    self._reset()

  def _reset(self):
    self.stopped_t = 0.0
    self.d_stop = None       # latched stop gap
    self.armed = False
    self.trigger_cnt = 0
    self.active = False
    self.fading = False
    self.boost = 0.0
    self.boost_t = 0.0
    self.d_peak = 0.0

  def _end(self, event):
    self.event = event
    self._reset()
    return 0.0

  def update(self, long_active, v_ego, gas_pressed, brake_pressed, lead_one, lead_two, a_target_mpc, crash_cnt):
    """Returns the boost accel in m/s2, 0 when off. Call once per planner update."""
    self.event = ''
    if self.live_cnt % LIVE_PERIOD == 0:
      self.boost_max = read_live_max()
    self.live_cnt += 1

    lead_ok = bool(lead_one.status)
    if not lead_ok:
      self.d_f = None
    elif self.d_f is None:
      self.d_f, self.v_f, self.a_f = float(lead_one.dRel), float(lead_one.vLead), float(lead_one.aLeadK)
    else:
      self.d_f += (lead_one.dRel - self.d_f) * self.dt / (TAU_D + self.dt)
      self.v_f += (lead_one.vLead - self.v_f) * self.dt / (TAU_V + self.dt)
      self.a_f += (lead_one.aLeadK - self.a_f) * self.dt / (TAU_A + self.dt)

    if v_ego > V_EGO_DROVE and not self.active:
      self.spent = False

    if self.active:
      return self._update_active(long_active, v_ego, gas_pressed, brake_pressed, lead_ok, lead_two, a_target_mpc, crash_cnt)

    # Arm only at a full stop. Once armed, stay armed while the MPC starts to roll ego forward
    # (below V_ARMED_MAX), so a slow MPC start does not pre-empt the boost.
    v_max = V_ARMED_MAX if self.armed else V_EGO_STOPPED
    if v_ego > v_max or not long_active or gas_pressed or brake_pressed or not lead_ok or self.spent:
      self._reset()
      return 0.0

    if self.v_f < V_LEAD_STOPPED:
      self.stopped_t += self.dt
      if self.d_stop is None:
        self.d_stop = self.d_f
      self.d_stop += (self.d_f - self.d_stop) * self.dt / (TAU_LATCH + self.dt)
    elif not self.armed:
      self.stopped_t = 0.0   # lead still moving or creeping: wait for it to settle
    self.armed = self.armed or self.stopped_t >= ARM_TIME
    if not self.armed:
      return 0.0

    departing = (a_target_mpc > A_MPC_GO and self.v_f > V_TRIGGER and self.a_f > A_TRIGGER and
                 self.d_f > self.d_stop + D_TRIGGER)
    self.trigger_cnt = self.trigger_cnt + 1 if departing else 0
    if self.trigger_cnt < TRIGGER_FRAMES:
      return 0.0
    self.active = True
    self.spent = True
    self.d_peak = self.d_f
    self.boost = float(np.clip(a_target_mpc, 0.0, self.boost_max))   # ramp on from where the MPC already is
    self.event = 'fired'
    return self._update_active(long_active, v_ego, gas_pressed, brake_pressed, lead_ok, lead_two, a_target_mpc, crash_cnt)

  def mpc_behind(self, v_mpc, v_ego):
    """True from the trigger until the MPC's own speed state has caught up with the boosted car."""
    self.sync = (self.active or self.sync) and v_mpc < v_ego and v_ego > V_EGO_STOPPED
    return self.sync

  def _update_active(self, long_active, v_ego, gas_pressed, brake_pressed, lead_ok, lead_two, a_target_mpc, crash_cnt):
    # abort: drop the boost at once and leave the MPC output as it is
    if not long_active or gas_pressed or brake_pressed:
      return self._end('abort: driver or long inactive')
    if not lead_ok:
      return self._end('abort: lead lost')
    if lead_two.status and lead_two.dRel < self.d_f - LEAD2_MARGIN:
      return self._end('abort: leadTwo closer')
    self.d_peak = max(self.d_peak, self.d_f)
    if self.d_f < self.d_peak - D_SHRINK or self.v_f < max(V_CONFIRM, v_ego - V_CLOSING):
      return self._end('abort: lead not pulling away')
    if a_target_mpc < A_MPC_BRAKE or crash_cnt > 0:
      return self._end('abort: mpc brakes')

    self.boost_t += self.dt
    speed_bound = K_SPEED * (self.v_f - v_ego - V_MARGIN)
    gap_bound = K_GAP * (self.d_f - self.d_stop - D_GAP0 - T_GAP * v_ego)
    accel_bound = self.a_f + A_MARGIN
    if v_ego > V_HANDOVER or self.boost_t > BOOST_TIME or a_target_mpc >= speed_bound:
      self.fading = True
    target = 0.0 if self.fading else float(np.clip(min(speed_bound, gap_bound, accel_bound), 0.0, self.boost_max))
    self.boost = float(np.clip(target, self.boost - J_DOWN * self.dt, self.boost + J_UP * self.dt))
    if self.fading and self.boost <= 0.0:
      return self._end('handed over')
    return self.boost
