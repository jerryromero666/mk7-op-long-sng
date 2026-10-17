"""
Set min: SET pressed at a stop or under APPLY_BELOW_MPH never starts below SET_MIN mph.

Stock engages at the current speed but at least 25 mph (V_CRUISE_INITIAL 40 km/h), so SET at a stop
gives 25 mph. The planner eases into the set speed from ~14 mph, so a pickup from a light with no car
ahead stops pushing early and only pushes again after the set speed is raised by hand.

With the minimum at 40 mph the planner keeps pushing through 25 mph and only eases off toward 40.
Only SET at a stop or under APPLY_BELOW_MPH (pulling away from a light) gets the minimum: SET while
rolling at APPLY_BELOW_MPH or more is stock (the current speed, at least 25 mph), so SET at 30 mph in a
30 zone still holds 30 instead of speeding up to 40. RES restores the earlier set speed (openpilot
blocks RES when there is none), +/- adjust from there as stock (down to V_CRUISE_MIN). Experimental
mode's 65 mph start is kept.

Live tuning: /data/set_min_mph holds one number, the minimum in mph (25 = stock). Read when
openpilot engages, clamped to [MIN_LIMIT, MAX_LIMIT]. No file: DEFAULT_MPH.
"""
import numpy as np

from openpilot.common.constants import CV

DEFAULT_MPH = 40.
MIN_LIMIT = 25.   # mph, stock V_CRUISE_INITIAL
MAX_LIMIT = 45.   # mph
LIVE_FILE = '/data/set_min_mph'
APPLY_BELOW_MPH = 10.   # mph: SET under this (a stop, pulling away) gets the minimum; at or above it, stock


def read_set_min_mph():
  try:
    with open(LIVE_FILE) as f:
      val = float(f.read().strip())
  except (OSError, ValueError):
    return DEFAULT_MPH
  if not np.isfinite(val):
    return DEFAULT_MPH
  return float(np.clip(val, MIN_LIMIT, MAX_LIMIT))


def applies(v_ego: float) -> bool:
  """v_ego m/s: True when SET at this speed gets the set minimum (a stop or pulling away from one)."""
  return v_ego < APPLY_BELOW_MPH * CV.MPH_TO_MS
