from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib import stop_hold
from openpilot.selfdrive.controls.lib.stop_hold import StopHold

CP = SimpleNamespace(vEgoStopping=0.5, vEgoStarting=0.1)
DT = 0.05


@pytest.fixture
def live(tmp_path, monkeypatch):
  def _set(text):
    f = tmp_path / "stop_hold"
    if text is None:
      monkeypatch.setattr(stop_hold, "STOP_HOLD_FILE", str(tmp_path / "missing"))
    else:
      f.write_text(text)
      monkeypatch.setattr(stop_hold, "STOP_HOLD_FILE", str(f))
  return _set


def run(frames):
  """frames: (a_target, should_stop, v_now, v_target, a_now, v_ego, reset_state) per planner update."""
  sh = StopHold(DT)
  return [sh.update(*f, CP) for f in frames]


def end_of_stop(n=12):
  # the plan has reached its stop and eases off (a_target climbs to +0.25) while the car still rolls at 0.6 m/s
  a = np.linspace(-1.5, 0.25, n)
  return [(float(x), False, 0.4, 0.2, -0.3, 0.6, False) for x in a]


def test_missing_or_zero_is_stock(live):
  frames = end_of_stop()
  for text in (None, "0", "-2", "nan", "abc", ""):
    live(text)
    assert run(frames) == [(f[0], f[1]) for f in frames], text


def test_parse(live):
  for text, want in (("2.5", 2.5), ("0.1", 0.5), ("9", 5.0), ("0", 0.0), ("inf", 0.0), ("x", 0.0)):
    live(text)
    assert stop_hold.read_stop_hold() == want, text


def test_no_let_go_at_the_end_of_a_stop(live):
  live("2.5")
  out = run(end_of_stop())
  a = np.array([o[0] for o in out])
  assert np.all(a <= 0.0)                                   # never asks to speed up
  assert np.max(np.diff(a)) <= 2.5 * DT + 1e-9              # eases off at most 2.5 m/s3
  assert all(o[1] for o in out)                             # hands the stop to the stopping state at once


def test_only_ever_more_braking(live):
  live("2.5")
  frames = end_of_stop()
  for (a, _), f in zip(run(frames), frames, strict=True):
    assert a <= f[0]


def test_moving_off_releases(live):
  # the plan wants to go again (a_now >= 0, or its speed in action_t back over vEgoStopping): untouched
  live("2.5")
  frames = end_of_stop(6) + [(0.8, False, 0.4, 0.7, 0.2, 0.6, False)] * 3
  out = run(frames)
  assert out[-1] == (0.8, False)
  frames = end_of_stop(6) + [(0.8, False, 0.4, 0.7, -0.1, 0.6, False)] * 3
  assert run(frames)[-1] == (0.8, False)


def test_reset_state_and_standstill(live):
  live("2.5")
  # long off (reset_state): untouched
  frames = [(0.3, False, 0.4, 0.2, -0.3, 0.6, True)] * 3
  assert run(frames) == [(0.3, False)] * 3
  # it does not start at a standstill (re-engaging behind a stopped car) ...
  frames = [(0.3, False, 0.0, 0.0, -0.01, 0.0, False)] * 3
  assert run(frames) == [(0.3, False)] * 3
  # ... but a stop it holds stays held through the standstill until the plan wants to move off
  frames = end_of_stop() + [(0.3, False, 0.0, 0.0, -0.01, 0.0, False)] * 3 + [(0.6, False, 0.0, 0.3, 0.05, 0.0, False)]
  out = run(frames)
  assert all(o[0] <= 0.0 and o[1] for o in out[:-1])
  assert out[-1] == (0.6, False)
