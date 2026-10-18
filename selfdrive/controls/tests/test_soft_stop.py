import numpy as np
import pytest

from cereal import car, custom
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import longcontrol
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState


def make_cp():
  # Mk7 values: volkswagen/interface.py + interfaces.py get_std_params, PID gains 0
  CP = car.CarParams.new_message(stopAccel=-1.2, stoppingDecelRate=6.0, vEgoStopping=0.5, vEgoStarting=0.1,
                                 startingState=False)
  CP.longitudinalTuning.kpBP = [0.]
  CP.longitudinalTuning.kpV = [0.]
  CP.longitudinalTuning.kiBP = [0.]
  CP.longitudinalTuning.kiV = [0.]
  return CP


@pytest.fixture
def live(tmp_path, monkeypatch):
  """live(text or None): point SOFT_STOP_FILE at a file holding text (None: a missing file)."""
  def _set(text):
    f = tmp_path / "soft_stop"
    if text is None:
      monkeypatch.setattr(longcontrol, "SOFT_STOP_FILE", str(tmp_path / "missing"))
    else:
      f.write_text(text)
      monkeypatch.setattr(longcontrol, "SOFT_STOP_FILE", str(f))
  return _set


def run_stop(v_profile, a_entry=-0.3, a_plan=0.0):
  """pid at a_entry, then shouldStop with the car speed following v_profile (one value per 10 ms) while the
  planner asks a_plan (a number or one value per step)."""
  lc = LongControl(make_cp(), custom.CarParamsSP.new_message())
  CS = car.CarState.new_message(vEgo=float(v_profile[0]))
  for _ in range(10):
    lc.update(True, CS, a_entry, False, (-3.5, 2.5))
  a_plan = np.broadcast_to(a_plan, len(v_profile))
  out = []
  for v, a in zip(v_profile, a_plan, strict=True):
    CS = car.CarState.new_message(vEgo=float(v))
    out.append(float(lc.update(True, CS, float(a), True, (-3.5, 2.5))))
    assert lc.long_control_state == LongCtrlState.stopping
  return np.array(out)


def stock_ramp(a_entry, n):
  # the stock stopping state, with the CarParams values as the car sees them (Float32)
  CP = make_cp()
  out, a = [], a_entry
  for _ in range(n):
    if a > CP.stopAccel:
      a = min(a, 0.0) - CP.stoppingDecelRate * DT_CTRL
    out.append(a)
  return np.array(out)


def test_missing_or_zero_is_stock(live):
  v = np.linspace(0.6, 0.0, 80)
  for text in (None, "0", "0 1.5", "-1", "garbage", "nan", "", "0.6 1.5 2", "5"):
    live(text)
    out = run_stop(v)
    assert np.array_equal(out, stock_ramp(-0.3, len(v))), text


def test_parse(live):
  for text, want in (("0.6", (0.6, 1.5)), ("0.6 1.0", (0.6, 1.0)), ("0.1", (0.3, 1.5)), ("0.6 99", (0.6, 6.0)),
                     ("0.6 0", (0.6, 0.3)), ("inf", (0.0, 1.5)), ("1 2 3", (0.0, 1.5))):
    live(text)
    assert longcontrol.read_soft_stop() == want, text


def test_rolling_stops_at_roll_decel(live):
  # hand-over at 0.5 m/s: the request goes down to -0.6 at 1.5 m/s3 and stays there while rolling
  live("0.6")
  out = run_stop(np.full(60, 0.5))
  assert out.min() >= -0.6 - 1e-9
  assert abs(out[-1] + 0.6) < 1e-9
  assert np.all(np.diff(out) <= 1e-12)                 # never eases off
  assert np.max(-np.diff(out)) <= 1.5 * DT_CTRL + 1e-9  # 1.5 m/s3


def test_standstill_gets_stock_hold(live):
  live("0.6")
  out = run_stop(np.concatenate([np.full(30, 0.4), np.zeros(60)]))
  assert out[29] >= -0.6 - 1e-9
  assert abs(out[-1] + 1.2) < 0.061                     # stock stopAccel once stopped (stock may overshoot 1 step)
  assert np.max(-np.diff(out[30:])) <= 6.0 * DT_CTRL + 1e-9


def test_fast_handover_keeps_stock(live):
  # at 0.85 m/s or more the car must still stop within SOFT_STOP_DIST: stock target and rate
  live("0.6")
  out = run_stop(np.full(40, 1.0))
  assert out[-1] <= -1.2 + 1e-9
  assert np.max(-np.diff(out[:10])) >= 6.0 * DT_CTRL - 1e-9


def test_hard_entry_is_held(live):
  live("0.6")
  out = run_stop(np.full(40, 0.4), a_entry=-2.0)
  assert np.allclose(out, -2.0)


def test_planner_wants_more_braking_gets_stock(live):
  # a lead braking hard in the last metres: the planner asks -2, soft stop gives way to the stock ramp
  live("0.6")
  v = np.full(40, 0.5)
  out = run_stop(v, a_plan=-2.0)
  assert np.array_equal(out, stock_ramp(-0.3, len(v)))
  # the planner asks for more only later: soft until then, then the stock ramp from where it is, never easing
  a_plan = np.concatenate([np.zeros(30), np.full(30, -2.0)])
  out = run_stop(np.full(60, 0.5), a_plan=a_plan)
  assert out[29] >= -0.6 - 1e-9
  assert out[-1] <= -1.2 + 1e-9
  assert np.all(np.diff(out) <= 1e-12)


def test_time_limit_gets_stock(live):
  # a car that does not slow down (steep downhill, weak brakes): stock ramp after SOFT_STOP_MAX_T
  live("0.6")
  n = int(round(longcontrol.SOFT_STOP_MAX_T / DT_CTRL))
  out = run_stop(np.full(n + 40, 0.5))
  assert out[n - 2] >= -0.6 - 1e-9
  assert out[-1] <= -1.2 + 1e-9
  assert np.all(np.diff(out) <= 1e-12)
