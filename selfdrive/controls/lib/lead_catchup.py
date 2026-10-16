"""
Lead catch-up: let the car accelerate harder while it follows a car ahead that is pulling away,
in the 5-25 mph range where the stock caps (1.6 m/s2 at 0 falling to 1.2 by 22 mph, plus the
cruise fake obstacle growing at CRUISE_MAX_ACCEL 1.6) make it fall behind.

Only with a lead: leadOne seen, closer than D_MAX and not slower than ego by more than V_SLOWER.
Then the planner's accel cap becomes max(stock cap, catch-up cap) and the MPC's cruise fake
obstacle may grow at the catch-up cap too. Without a lead everything is stock. The MPC still
keeps its follow distance and never plans past the set speed; this only lets it use more of the
accel it already plans (its own bound is ACCEL_MAX 2.5).

  catch-up cap = interp(vEgo, CAP_BP, [max, max, 0]): full up to 25 mph, faded out by ~45 mph,
  where the stock cap is higher anyway (~34 mph).

Live tuning: /data/lead_accel holds one number, the catch-up max in m/s2 (0 turns it off).
Re-read about once a second, clamped to [0, MAX_LIMIT]. No file: DEFAULT_MAX.
"""
import numpy as np

DEFAULT_MAX = 1.8        # m/s2
MAX_LIMIT = 1.95         # m/s2, below the panda / CarControllerParams limit of 2.0
CAP_BP = [0., 11.2, 20.]  # m/s: 0, 25 mph, 45 mph
D_MAX = 60.0             # m, lead closer than this
V_SLOWER = 0.5           # m/s, lead not slower than ego by more than this
LIVE_FILE = '/data/lead_accel'
LIVE_PERIOD = 20         # updates between file reads (1 Hz at 20 Hz)


def read_live_max():
  try:
    with open(LIVE_FILE) as f:
      val = float(f.read().strip())
  except (OSError, ValueError):
    return DEFAULT_MAX
  if not np.isfinite(val):
    return DEFAULT_MAX
  return float(np.clip(val, 0.0, MAX_LIMIT))


class LeadCatchup:
  def __init__(self):
    self.max = DEFAULT_MAX
    self.live_cnt = 0

  def update(self, lead_one, v_ego):
    """Returns the catch-up accel cap in m/s2, 0 when off (no lead, slower lead, or disabled)."""
    if self.live_cnt % LIVE_PERIOD == 0:
      self.max = read_live_max()
    self.live_cnt += 1
    if not lead_one.status or lead_one.dRel > D_MAX or lead_one.vLead < v_ego - V_SLOWER:
      return 0.0
    return float(np.interp(v_ego, CAP_BP, [self.max, self.max, 0.0]))
