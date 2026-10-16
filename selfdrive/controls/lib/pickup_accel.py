"""
Pickup accel: let the car accelerate harder in the 5-25 mph range, with or without a car ahead.

Stock caps hold it back there: the planner caps accel at 1.6 m/s2 falling to 1.2 by 22 mph, the
turn limit caps it at 1.7 even with the wheel straight, and the MPC's cruise fake obstacle grows at
CRUISE_MAX_ACCEL 1.6, so the car picks up slowly from a stop to the set speed and falls behind a
car pulling away.

The planner's accel cap becomes max(stock cap, pickup cap), the turn limit is raised to the same,
and the MPC's cruise fake obstacle may grow at the pickup cap. The MPC still keeps its follow
distance behind a car and never plans past the set speed; this only lets it use more of the accel
it already plans (its own bound is ACCEL_MAX 2.5).

  pickup cap = interp(vEgo, CAP_BP, [max, max, 0]): full up to 25 mph, faded out by 45 mph
  (the stock cap is higher from ~34 mph).

Live tuning: /data/pickup_accel holds one number, the pickup max in m/s2 (0 turns it off).
Re-read about once a second, clamped to [0, MAX_LIMIT]. No file: DEFAULT_MAX.
"""
import numpy as np

DEFAULT_MAX = 1.8        # m/s2
MAX_LIMIT = 1.95         # m/s2, below the panda / CarControllerParams limit of 2.0
CAP_BP = [0., 11.2, 20.]  # m/s: 0, 25 mph, 45 mph
LIVE_FILE = '/data/pickup_accel'
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


class PickupAccel:
  def __init__(self):
    self.max = DEFAULT_MAX
    self.live_cnt = 0

  def update(self, v_ego):
    """Returns the pickup accel cap in m/s2, 0 when off."""
    if self.live_cnt % LIVE_PERIOD == 0:
      self.max = read_live_max()
    self.live_cnt += 1
    return float(np.interp(v_ego, CAP_BP, [self.max, self.max, 0.0]))
