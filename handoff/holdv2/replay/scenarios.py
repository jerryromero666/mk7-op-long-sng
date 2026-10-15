"""Scripted (open-loop) inputs per 100 Hz frame. Grade never enters the code under test directly;
it only shapes vEgoRaw (rollback/creep dither), the plan lag, and how slowly vEgo crosses 1.0/0.5.
t = 0 is the moment vEgoRaw first reads 0 (all wheels 0)."""
import numpy as np

DT = 0.01


def _q20(x):
  """plan outputs (shouldStop, aTarget) update at 20 Hz: hold each value for 5 frames."""
  x = np.asarray(x).copy()
  for i in range(len(x)):
    x[i] = x[(i // 5) * 5]
  return x


def base(t0, t1):
  t = np.round(np.arange(t0, t1, DT), 2)
  n = len(t)
  return dict(t=t, vraw=np.zeros(n), should_stop=np.zeros(n, bool), a_target=np.zeros(n),
              brake=np.zeros(n, bool), long_active=np.ones(n, bool), esp_hold=np.zeros(n, bool))


def approach(s, pts):
  """piecewise-linear vEgoRaw through (t, v) points; after the last point v stays at its value."""
  tp, vp = zip(*pts)
  s["vraw"] = np.interp(s["t"], tp, vp)
  s["vraw"][s["t"] >= tp[-1]] = vp[-1]
  s["vraw"] = np.round(s["vraw"], 3)


def set_window(s, key, ta, tb, val):
  m = (s["t"] >= ta - 1e-9) & (s["t"] < tb - 1e-9)
  s[key][m] = val


def launch(s, t_go, t_move=0.6, a=1.3):
  """plan says go at t_go: shouldStop off, aTarget ramps to 1.4; wheels move t_move later."""
  t = s["t"]
  set_window(s, "should_stop", t_go, 1e9, False)
  m = t >= t_go
  s["a_target"][m] = np.minimum(0.2 + (t[m] - t_go) * 1.5, 1.4)
  mv = t >= t_go + t_move
  s["vraw"][mv] = np.round(np.minimum((t[mv] - (t_go + t_move)) * a, 6.0), 3)


def dither(s, ta, tb, amp, period=0.20):
  """vEgoRaw rolls 0 -> amp -> 0 triangles (rollback/creep; ESP_19 speeds are unsigned)."""
  t = s["t"]
  m = (t >= ta) & (t < tb)
  ph = ((t[m] - ta) % period) / period
  s["vraw"][m] = np.round(amp * (1 - np.abs(2 * ph - 1)), 3)


def finish(s):
  s["should_stop"] = _q20(s["should_stop"]).astype(bool)
  s["a_target"] = _q20(s["a_target"])
  return s


FLAT_PTS = [(-6.0, 5.6), (-2.38, 1.01), (-1.88, 0.60), (-1.28, 0.27), (-0.98, 0.12), (-0.58, 0.05), (-0.01, 0.02), (0.0, 0.0)]


def flat_stop(hold=10.0, name="flat_stop"):
  # route 333 stop #2 shape: stopping state at vEgo ~1.01 (-2.38 s), wheels 0 at 0.00
  s = base(-6.0, hold + 3.0)
  approach(s, FLAT_PTS)
  s["a_target"][:] = np.interp(s["t"], [-6, -2.4, 0], [-1.7, -0.55, -0.3])
  set_window(s, "should_stop", -2.40, hold, True)
  launch(s, hold)
  return name, finish(s)


def uphill(grade, lag, amp, name):
  # uphill: gravity stops the car fast; plan lags -> lcs stays pid (aTarget ~ -0.25) until t=lag
  s = base(-4.0, 10.0)
  approach(s, [(-4.0, 3.5), (-0.6, 0.45), (-0.2, 0.12), (-0.01, 0.02), (0.0, 0.0)])
  s["a_target"][:] = np.interp(s["t"], [-4, -0.6, 0], [-1.2, -0.4, -0.25])
  set_window(s, "should_stop", lag, 7.0, True)
  dither(s, 0.10, 0.70, amp)          # rollback/creep right after the stop
  dither(s, 3.00, 3.60, amp / 3)      # small creep later in the hold
  launch(s, 7.0)
  return name, finish(s)


def downhill(stop_at_v, name):
  # -6%: brakes fight gravity, vEgo crosses 1.0 and 0.5 slowly; KF noise -> several crossings
  s = base(-8.0, 10.0)
  pts = [(-8.0, 3.0), (-6.0, 1.15), (-5.0, 1.03), (-4.0, 0.99), (-3.0, 0.80), (-2.2, 0.52), (-1.4, 0.48),
         (-0.6, 0.20), (-0.01, 0.02), (0.0, 0.0)]
  approach(s, pts)
  rng = np.random.default_rng(6)
  m = s["t"] < 0
  s["vraw"][m] = np.round(np.maximum(s["vraw"][m] + rng.normal(0, 0.012, m.sum()), 0.001), 3)
  s["a_target"][:] = np.interp(s["t"], [-8, -3, 0], [-0.6, -0.3, -0.2])
  # shouldStop turns on the first time vraw drops below stop_at_v
  i0 = int(np.argmax(s["vraw"] < stop_at_v))
  set_window(s, "should_stop", s["t"][i0], 7.0, True)
  launch(s, 7.0)
  return name, finish(s)


def chatter(name="chatter_at_standstill"):
  # planner flips stopping<->pid at standstill (hill: lead/vision/aEgo noise). aTarget during pid blips.
  s = base(-6.0, 14.0)
  approach(s, FLAT_PTS)
  s["a_target"][:] = np.interp(s["t"], [-6, -2.4, 0], [-1.7, -0.55, -0.3])
  set_window(s, "should_stop", -2.40, 11.0, True)
  for ta, tb, a in [(1.0, 1.15, 0.15), (2.5, 2.8, -0.10), (4.0, 4.05, 0.05), (5.0, 5.5, 0.30), (6.0, 6.1, -0.05),
                    (8.0, 8.2, 0.20)]:
    set_window(s, "should_stop", ta, tb, False)
    set_window(s, "a_target", ta, tb, a)
  launch(s, 11.0)
  return name, finish(s)


def brake_during_stop(name="brake_during_stop"):
  # stopped; driver presses brake at +3.0 s -> selfdrived pedalPressed -> longActive False 2 frames later;
  # brake held to +6.0; driver re-engages (resume) at +8.0 at standstill; plan go at +11.0
  s = base(-6.0, 14.0)
  approach(s, FLAT_PTS)
  s["a_target"][:] = np.interp(s["t"], [-6, -2.4, 0], [-1.7, -0.55, -0.3])
  set_window(s, "should_stop", -2.40, 11.0, True)
  set_window(s, "brake", 3.0, 6.0, True)
  set_window(s, "long_active", 3.02, 8.0, False)
  launch(s, 11.0)
  return name, finish(s)


def rollback_after_hold(amp, name):
  # held at 0, then wheels turn again (rollback) while OP still stopping: amp < 1.0 or > 1.0
  s = base(-6.0, 14.0)
  approach(s, FLAT_PTS)
  s["a_target"][:] = np.interp(s["t"], [-6, -2.4, 0], [-1.7, -0.55, -0.3])
  set_window(s, "should_stop", -2.40, 11.0, True)
  t = s["t"]
  m = (t >= 4.0) & (t < 5.2)
  s["vraw"][m] = np.round(amp * np.sin(np.pi * (t[m] - 4.0) / 1.2), 3)
  launch(s, 11.0)
  return name, finish(s)


def esp_hold_hypo(name="HYPO_esp_hold_confirms"):
  # never seen on this car (bit 53 never set); shows the floor/HMS=3 paths
  s = base(-6.0, 9.0)
  approach(s, FLAT_PTS)
  s["a_target"][:] = np.interp(s["t"], [-6, -2.4, 0], [-1.7, -0.55, -0.3])
  set_window(s, "should_stop", -2.40, 6.0, True)
  set_window(s, "esp_hold", 0.10, 6.8, True)
  launch(s, 6.0, t_move=0.8)
  return name, finish(s)


def all_scenarios():
  return [
    flat_stop(10.0, "flat_stop"),
    uphill(5, 0.04, 0.15, "uphill5_lag0.04_dither0.15"),
    uphill(5, 0.50, 0.15, "uphill5_lag0.50_dither0.15"),
    uphill(11, 0.04, 0.30, "uphill11_lag0.04_dither0.30"),
    uphill(11, 0.50, 0.30, "uphill11_lag0.50_dither0.30"),
    downhill(1.10, "downhill6_stop_at_v1.10"),
    downhill(0.40, "downhill6_stop_at_v0.40"),
    chatter(),
    brake_during_stop(),
    flat_stop(30.0, "long_stop_30s"),
    rollback_after_hold(0.4, "rollback_while_stopped_0.4"),
    rollback_after_hold(1.2, "rollback_while_stopped_1.2"),
    esp_hold_hypo(),
  ]
