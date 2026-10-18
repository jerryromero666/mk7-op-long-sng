"""
Stop hold: no brake let-go in the last metres of a stop.

Near a stop the plan's own speed runs ahead of the car (the car follows the plan about 0.5 s late), so
after a firm stop the plan reaches 0 while the car still rolls at 0.3-0.8 m/s. get_accel_from_plan looks
action_t (0.55 s) ahead, finds the plan already stopped and returns about 0 or even +0.25, and shouldStop
waits until that drops under 0.1: the brakes let go in the last ~0.5 s, the car rolls on, then the
stopping state brakes again ("hard brake, slight roll, stop").

With stop hold set, from the moment the plan brakes into its stop while the car still rolls (a_now < 0,
v_target under vEgoStopping, vEgo above vEgoStarting) and until the plan stops braking or wants to move
off again, the request may only come back up (less braking) at the stop hold rate in m/s3 and never above
0, so the stop is handed to the stopping state as soon as the plan's speed is under vEgoStopping and it is
not taken back at the standstill. It only ever brakes more than stock, never less; gentle stops ease off
slower than this rate anyway, so they are unchanged.

Live: /data/stop_hold holds the rate in m/s3 (clamped to 0.5-5.0), re-read about once a second;
no file or 0 = off (stock).
"""
import numpy as np

STOP_HOLD_FILE = '/data/stop_hold'
STOP_HOLD_LIMITS = (0.5, 5.0)
STOP_HOLD_PERIOD = 20    # updates between file reads (1 Hz at 20 Hz)


def read_stop_hold():
  try:
    with open(STOP_HOLD_FILE) as f:
      val = float(f.read().strip())
  except (OSError, ValueError):
    return 0.0
  if not np.isfinite(val) or val <= 0.0:
    return 0.0
  return float(np.clip(val, *STOP_HOLD_LIMITS))


class StopHold:
  def __init__(self, dt):
    self.dt = dt
    self.rate = 0.0
    self.cnt = 0
    self.prev = None
    self.active = False

  def update(self, a_target, should_stop, v_now, v_target, a_now, v_ego, reset_state, CP):
    """a_target / should_stop from get_accel_from_plan -> (a_target, should_stop). Call once per planner update."""
    if self.cnt % STOP_HOLD_PERIOD == 0:
      self.rate = read_stop_hold()
    self.cnt += 1
    self.active = (self.rate > 0.0 and not reset_state and a_now < 0.0 and v_target < CP.vEgoStopping and
                   (v_ego > CP.vEgoStarting or self.active))
    if self.active:
      if self.prev is not None:
        a_target = min(a_target, self.prev + self.rate * self.dt, 0.0)
      should_stop = v_now < CP.vEgoStopping and a_target < 0.1
    self.prev = a_target
    return a_target, should_stop
