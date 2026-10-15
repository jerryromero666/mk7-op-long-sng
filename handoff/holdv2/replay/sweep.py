#!/usr/bin/env python3
"""Sensitivity sweeps for the two timeout-0 specific effects.
 A) stopping-state onset speed vs ACC_07 stop announcement (HMS=1 / Anhalten / Anhalteweg 0.3) per stop.
 B) HMS 1<->4 cycling when vEgo crosses 1.0 m/s slowly in the stopping state (noise on vEgoRaw)."""
import numpy as np
from replay import REFS, SHORT, load_ref, run
from scenarios import base, set_window, finish, launch

mods = {r: load_ref(r) for r in ("claude/determined-hypatia-pnidyz", "hold_tsk_owned")}


def stop_profile(v_on, decel=0.6, pre=-1.2):
  # constant decel to 0 at t=0; lcs stopping begins when vraw first < v_on (v_on=0 -> after wheels 0)
  s = base(-8.0, 6.0)
  t = s["t"]
  s["vraw"] = np.round(np.clip(-decel * t, 0, None), 3)
  s["a_target"][:] = pre * 0 - 0.4
  if v_on > 0:
    i0 = int(np.argmax(s["vraw"] < v_on))
  else:
    i0 = int(np.argmax(t >= 0.04))
  set_window(s, "should_stop", t[i0], 4.0, True)
  launch(s, 4.0)
  return finish(s)


print("A) per stop: s of ACC_07 stop announcement (HMS=1 & Anhalten & Anhalteweg 0.3) and where it happens")
print("   v_on = vEgo when OP's stopping state begins (decel 0.6 m/s2)")
for v_on in (1.5, 1.2, 1.05, 1.01, 0.99, 0.9, 0.6, 0.45, 0.2, 0.0):
  s = stop_profile(v_on)
  cells = []
  for ref, m in mods.items():
    rows = run(m, s)
    h1 = [r for r in rows if r["hms"] == 1 and r["a7_stp"] and r["dist"] == 0.3]
    still = sum(r["vraw"] == 0 for r in h1) * 0.02
    hi = sum(r["v"] >= 1.0 for r in h1) * 0.02
    seq = ">".join(str(h) for i, h in enumerate([r["hms"] for r in rows]) if i == 0 or h != rows[i - 1]["hms"])
    cells.append(f"{SHORT[ref]}: {len(h1) * 0.02:.2f}s (v>=1 {hi:.2f}, wheels0 {still:.2f}) seq {seq}")
  print(f"  v_on={v_on:4.2f}  " + " | ".join(cells))


def slow_cross(decel, noise, seed):
  s = base(-12.0, 6.0)
  t = s["t"]
  v = np.clip(1.0 - decel * (t + 2.0), 0, None)   # crosses 1.0 at t=-2
  v[t < -2.0] = np.minimum(1.0 + decel * (-2.0 - t[t < -2.0]), 2.0)
  rng = np.random.default_rng(seed)
  v = v + rng.normal(0, noise, len(v)) * (v > 0)
  s["vraw"] = np.round(np.clip(v, 0, None), 3)
  s["vraw"][t >= 0] = 0.0
  set_window(s, "should_stop", -10.0, 4.0, True)   # stopping well before 1.0 (downhill: plan leads)
  s["a_target"][:] = -0.3
  launch(s, 4.0)
  return finish(s)


print("\nB) stopping state already on, vEgo crosses 1.0 slowly: HMS 4->1 round trips (mean over 5 seeds)")
for decel in (0.05, 0.1, 0.2, 0.5, 1.0):
  for noise in (0.0, 0.005, 0.01, 0.02):
    res = {}
    for ref, m in mods.items():
      trips = []
      for seed in range(5):
        rows = run(m, slow_cross(decel, noise, seed))
        h = [r["hms"] for r in rows]
        trips.append(sum(1 for a, b in zip(h, h[1:]) if a == 4 and b == 1))
      res[SHORT[ref]] = np.mean(trips), max(trips)
    print(f"  decel {decel:4.2f} m/s2 noise {noise:.3f}: " + "  ".join(f"{k} 4->1 trips mean {a:.1f} max {b}" for k, (a, b) in res.items()))
