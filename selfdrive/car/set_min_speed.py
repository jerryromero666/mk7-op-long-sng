"""
Set min: SET (or RES with no earlier set speed) never starts below SET_MIN mph.

Stock engages at the current speed but at least 25 mph (V_CRUISE_INITIAL 40 km/h), so SET at a stop
gives 25 mph. The planner eases into the set speed from ~14 mph, so a pickup from a light with no car
ahead stops pushing early and only pushes again after the set speed is raised by hand.

With the minimum at 40 mph the planner keeps pushing through 25 mph and only eases off toward 40.
Above the minimum SET still takes the current speed, RES still restores the earlier set speed, and
+/- adjust from there as stock (down to V_CRUISE_MIN). Experimental mode's 65 mph start is kept.

Live tuning: /data/set_min_mph holds one number, the minimum in mph (25 = stock). Read when
openpilot engages, clamped to [MIN_LIMIT, MAX_LIMIT]. No file: DEFAULT_MPH.
"""
import numpy as np

DEFAULT_MPH = 40.
MIN_LIMIT = 25.   # mph, stock V_CRUISE_INITIAL
MAX_LIMIT = 45.   # mph
LIVE_FILE = '/data/set_min_mph'


def read_set_min_mph():
  try:
    with open(LIVE_FILE) as f:
      val = float(f.read().strip())
  except (OSError, ValueError):
    return DEFAULT_MPH
  if not np.isfinite(val):
    return DEFAULT_MPH
  return float(np.clip(val, MIN_LIMIT, MAX_LIMIT))
