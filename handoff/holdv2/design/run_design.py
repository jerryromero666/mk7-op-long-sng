#!/usr/bin/env python3
"""Replay the hill-hold v2 candidates through the parent's real ACC block, LongControl and mqbcan,
with a simple ESP model for ESP_21 bit 63 (route 333/332 timing):
  bit 63 turns ON 2 ACC frames (0.04 s) after ACC_07 HMS=1 + Anhalten starts, OFF 2 frames after it ends.
TSK torque is annotated, not simulated: route 333 shows Brm -> 0 about 0.05 s after HMS 1 (with ACC_06
Anhalten) and back about 0.05 s after HMS 4.
Usage: run_design.py [scenario-substring ...] [-v]
"""
import sys
import types
sys.path.insert(0, "/tmp/claude-0/-home-user-mk7-op-long-sng/48e1abac-3f14-57e3-a7c0-d6a536957af8/scratchpad/holdv2/replay")
import numpy as np
from replay import load_ref, CP, CP_SP, CCP, SpeedKF, Packer, LCS, ns
import scenarios as S
from candidates import CANDIDATES

PARENT = "claude/determined-hypatia-pnidyz"
T0REF = "hold_tsk_owned"
FR = 0.02


def with_mgr(base, src):
  g = {"np": np}
  exec(src, g)
  m = dict(base)
  m["Mgr"] = g["MQBStandstillManager"]
  return m


def run(mod, scen):
  LoC = mod["LongControl"](CP, CP_SP)
  mgr = mod["Mgr"](CCP)
  mq = mod["mqbcan"]
  kf = SpeedKF()
  selfo = ns(CP=CP, CCP=CCP, CCS=mq, standstill_manager=mgr, packer_pt=Packer(), CAN=ns(pt=0), frame=0)
  rows = []
  b63 = False
  hms1_run = 0      # consecutive ACC frames with HMS 1 + A7 Anhalten sent
  off_run = 0
  for i in range(len(scen["t"])):
    vraw = float(scen["vraw"][i])
    vEgo, aEgo = kf.update(vraw)
    out = ns(vEgo=vEgo, aEgo=aEgo, vEgoRaw=vraw, standstill=(vraw == 0.0), brakePressed=bool(scen["brake"][i]),
             accFaulted=False, cruiseState=ns(available=True, standstill=False))
    CS = ns(out=out, esp_hold_confirmation=bool(scen["esp_hold"][i]), esp_stopping=b63, acc_type=1)
    long_active = bool(scen["long_active"][i])
    actuators = ns(longControlState=LoC.long_control_state)
    lcs_cs = LoC.long_control_state
    if not long_active:
      LoC.reset()
    actuators.accel = float(LoC.update(long_active, out, float(scen["a_target"][i]), bool(scen["should_stop"][i]),
                                       (CCP.ACCEL_MIN, CCP.ACCEL_MAX)))
    CC = ns(longActive=long_active, actuators=actuators)
    loc = {"self": selfo, "CC": CC, "CS": CS, "actuators": actuators, "can_sends": [], "LongCtrlState": LCS,
           "np": np, "mqbcan": mq}
    exec(mod["acc_block"], loc)
    if loc["can_sends"]:
      a6 = next(v for n, b, v in loc["can_sends"] if n == "ACC_06")
      a7 = next(v for n, b, v in loc["can_sends"] if n == "ACC_07")
      req = a7["ACC_Anforderung_HMS"] == 1 and bool(a7["ACC_Anhalten"])
      rows.append(dict(t=float(scen["t"][i]), vraw=vraw, v=vEgo, lcs=lcs_cs, la=long_active, b63=b63,
                       a6_stp=bool(a6["ACC_Anhalten"]), a6_go=bool(a6["ACC_Anfahren"]),
                       acc=float(a6["ACC_Sollbeschleunigung_02"]), hms=int(a7["ACC_Anforderung_HMS"]),
                       a7_stp=bool(a7["ACC_Anhalten"]), a7_go=bool(a7["ACC_Anfahren"]),
                       dist=float(a7["ACC_Anhalteweg"]), st=a6["ACC_Status_ACC"]))
      # ESP model, evaluated on ACC frames
      if req:
        hms1_run += 1
        off_run = 0
      else:
        hms1_run = 0
        off_run += 1
      if not b63 and hms1_run >= 2:
        b63 = True
      elif b63 and off_run >= 2:
        b63 = False
    selfo.frame += 1
  return rows


def fmt(r):
  a6 = ("S" if r["a6_stp"] else ".") + ("G" if r["a6_go"] else ".")
  a7 = ("S" if r["a7_stp"] else ".") + ("G" if r["a7_go"] else ".")
  return (f"{r['t']:+7.2f} v={r['v']:.2f} lcs={r['lcs'][:4]:4s} HMS={r['hms']} A6={a6} A7={a7} d={r['dist']:5.2f} "
          f"acc={r['acc']:+.2f} b63={int(r['b63'])}")


def key(r):
  return (r["la"], r["lcs"], r["hms"], r["a6_stp"], r["a6_go"], r["a7_stp"], r["a7_go"], r["dist"], r["b63"], r["st"])


def summary(rows):
  seq = ">".join(str(h) for i, h in enumerate([r["hms"] for r in rows]) if i == 0 or h != rows[i - 1]["hms"])
  still = [r for r in rows if r["vraw"] == 0.0 or r["v"] < 0.05]
  h1 = [r for r in rows if r["hms"] == 1]
  h1_still = sum(1 for r in h1 if r["vraw"] == 0.0 or r["v"] < 0.05) * FR
  h1_v = [r["v"] for r in h1]
  b63 = [r for r in rows if r["b63"]]
  b63_still = sum(1 for r in b63 if r["vraw"] == 0.0 or r["v"] < 0.05) * FR
  # end of the last bit-63 episode relative to the first standstill frame
  t_still = next((r["t"] for r in rows if r["vraw"] == 0.0), None)
  b63_end_before = None
  if b63 and t_still is not None:
    pre = [r["t"] for r in b63 if r["t"] <= t_still]
    b63_end_before = (t_still - max(pre)) if pre else None
  chg_still = sum(1 for a, b in zip(rows, rows[1:]) if a["hms"] != b["hms"] and (b["vraw"] == 0.0 or b["v"] < 0.05))
  a6g_a7s = sum(1 for r in rows if r["a6_go"] and r["a7_stp"]) * FR
  return (f"HMS seq {seq} | HMS1 {len(h1) * FR:.2f}s (v {min(h1_v) if h1_v else 0:.2f}-{max(h1_v) if h1_v else 0:.2f}, "
          f"at standstill {h1_still:.2f}s) | modelled b63 {len(b63) * FR:.2f}s, at standstill {b63_still:.2f}s, "
          f"last b63 ends {('%.2fs before wheels 0' % b63_end_before) if b63_end_before is not None else 'never before wheels 0'} "
          f"| HMS changes at standstill {chg_still} | A6 Anfahren with A7 Anhalten {a6g_a7s:.2f}s")


def crawl(name="crawl_no_stop"):
  # slow traffic: 4.0 -> 1.2 m/s braking, then speeds back up to 3.5 without stopping (no shouldStop)
  s = S.base(-6.0, 6.0)
  S.approach(s, [(-6.0, 4.0), (-2.0, 1.2), (-1.0, 1.2), (2.0, 3.5), (6.0, 3.5)])
  s["a_target"][:] = np.interp(s["t"], [-6, -2.0, -1.0, -0.9, 2.0], [-0.8, -0.6, -0.05, 0.6, 0.3])
  return name, S.finish(s)


def scenario_list():
  return [
    S.flat_stop(10.0, "flat_stop"),
    S.uphill(11, 0.04, 0.30, "uphill11_lag0.04_dither0.30"),
    S.uphill(11, 0.50, 0.30, "uphill11_lag0.50_dither0.30"),
    S.downhill(1.10, "downhill6_stop_at_v1.10"),
    S.downhill(0.40, "downhill6_stop_at_v0.40"),
    S.chatter(),
    S.brake_during_stop(),
    S.rollback_after_hold(1.2, "rollback_while_stopped_1.2"),
    crawl(),
  ]


def main():
  base = load_ref(PARENT)
  t0 = load_ref(T0REF)
  mods = {"PAR": base, "T0": t0}
  for k, src in CANDIDATES.items():
    mods[k] = with_mgr(base, src)
  only = [a for a in sys.argv[1:] if not a.startswith("-")]
  for name, s in scenario_list():
    if only and not any(o in name for o in only):
      continue
    print("=" * 120)
    print("SCENARIO", name)
    for k, mod in mods.items():
      rows = run(mod, s)
      print(f"-- {k}: {summary(rows)}")
      last = None
      n = 0
      for r in rows:
        kk = key(r)
        if kk != last:
          if n < 40:
            print("     " + fmt(r))
          n += 1
          last = kk
      if n >= 40:
        print(f"     ... {n - 40} more transitions")
    print()


if __name__ == "__main__":
  main()
