#!/usr/bin/env python3
import sys
import hashlib
from replay import REFS, SHORT, load_ref, run
from scenarios import all_scenarios

FR = 0.02  # one ACC frame = 20 ms


def key(r):
  return (r["la"], r["lcs"], r["stopping"], r["starting"], r["hms"], r["a6_stp"], r["a6_go"], r["a7_stp"], r["a7_go"],
          r["dist"], r["st"])


def fmt(r):
  a6 = ("S" if r["a6_stp"] else ".") + ("G" if r["a6_go"] else ".")
  a7 = ("S" if r["a7_stp"] else ".") + ("G" if r["a7_go"] else ".")
  return (f"{r['t']:+7.2f} v={r['v']:.2f} raw={r['vraw']:.2f} la={int(r['la'])} lcs={r['lcs'][:4]:4s} "
          f"stp={int(r['stopping'])} go={int(r['starting'])} HMS={r['hms']} A6={a6} A7={a7} "
          f"d={r['dist']:5.2f} st={r['st']} acc={r['acc']:+.2f}" + (" L4" if r["latched"] else ""))


def transitions(rows, max_rows=40):
  out, last = [], None
  for r in rows:
    k = key(r)
    if k != last:
      out.append(fmt(r))
      last = k
  if len(out) > max_rows:
    out = out[:max_rows // 2] + [f"   ... {len(out) - max_rows} transitions skipped ..."] + out[-max_rows // 2:]
  return out


def runs(mask_rows):
  """list of (t_start, t_end, n) contiguous runs of True."""
  res, cur = [], None
  for t, m in mask_rows:
    if m and cur is None:
      cur = [t, t, 1]
    elif m:
      cur[1] = t; cur[2] += 1
    elif cur is not None:
      res.append(tuple(cur)); cur = None
  if cur is not None:
    res.append(tuple(cur))
  return res


def still(r):
  return r["vraw"] == 0.0 or r["v"] < 0.05


def metrics(rows):
  m = {}
  # Q1: HMS=4 + ACC_07 Anfahren while still rolling 0.05 < vEgo < 1.0 (vraw > 0)
  q1 = [r for r in rows if r["hms"] == 4 and r["a7_go"] and 0.05 < r["v"] < 1.0 and r["vraw"] > 0 and r["la"]]
  by = {}
  for r in q1:
    by.setdefault(r["lcs"], []).append(r)
  m["Q1"] = {k: (len(v) * FR, min(x["v"] for x in v), max(x["v"] for x in v), sum(x["acc"] < 0 for x in v) * FR,
                 v[0]["t"]) for k, v in by.items()}
  # Q2: HMS transitions at standstill, and while v < 1.0
  q2s, q2v = [], []
  for a, b in zip(rows, rows[1:]):
    if a["hms"] != b["hms"]:
      tag = f"{b['t']:+.2f}:{a['hms']}->{b['hms']}"
      if still(b):
        q2s.append(tag)
      elif b["v"] < 1.0:
        q2v.append(tag + f"@v{b['v']:.2f}")
  m["Q2s"], m["Q2v"] = q2s, q2v
  # Q3: ACC_06 Anhalten=1 together with ACC_07 Anfahren=1 / HMS=4
  q3 = runs([(r["t"], r["a6_stp"] and (r["a7_go"] or r["hms"] == 4)) for r in rows])
  m["Q3"] = (sum(x[2] for x in q3) * FR, max((x[2] for x in q3), default=0) * FR, len(q3),
             q3[0][0] if q3 else None)
  q3b = runs([(r["t"], r["a6_go"] and (r["a7_stp"] or r["hms"] == 1)) for r in rows])
  m["Q3b"] = (sum(x[2] for x in q3b) * FR, len(q3b))
  # Q4: start request (HMS=4 or ACC_07 Anfahren or ACC_06 Anfahren) with accel < 0 at standstill
  q4 = [r for r in rows if still(r) and r["la"] and (r["hms"] == 4 or r["a7_go"] or r["a6_go"]) and r["acc"] < 0]
  q4a6 = [r for r in q4 if r["a6_stp"]]
  q4go6 = [r for r in q4 if r["a6_go"]]
  m["Q4"] = (len(q4) * FR, len(q4a6) * FR, len(q4go6) * FR, min((r["acc"] for r in q4), default=0.0))
  # extra: HMS=1 / ACC_07 Anhalten totals, by speed
  h1 = [r for r in rows if r["hms"] == 1]
  m["HMS1"] = (len(h1) * FR, sum(still(r) for r in h1) * FR, sum((not still(r)) and r["v"] < 1.0 for r in h1) * FR,
               sum(r["v"] >= 1.0 for r in h1) * FR)
  m["A7stp"] = sum(r["a7_stp"] for r in rows) * FR
  m["hms_seq"] = ">".join(str(h) for i, h in enumerate([r["hms"] for r in rows]) if i == 0 or h != rows[i - 1]["hms"])
  return m


def diff_windows(ra, rb, fields=("hms", "a7_stp", "a7_go", "dist", "a6_stp", "a6_go", "acc")):
  mask = [(a["t"], any(a[f] != b[f] for f in fields)) for a, b in zip(ra, rb)]
  return runs(mask)


def main():
  mods = {ref: load_ref(ref) for ref in REFS}
  md5 = {ref: hashlib.md5(mods[ref]["acc_text"].encode()).hexdigest()[:8] for ref in REFS}
  print("ACC block md5 per ref:", md5, "| HOLD_REQUEST_TIMEOUT:", {SHORT[r]: mods[r]["timeout"] for r in REFS})
  print("legend: A6/A7 = ACC_06/ACC_07 Anhalten(S)/Anfahren(G); d = ACC_07 Anhalteweg; st = ACC_Status_ACC; "
        "L4 = _hms4_latched; lcs = actuators.longControlState as sent (1-frame lag)\n")
  verbose = "-v" in sys.argv
  only = [a for a in sys.argv[1:] if not a.startswith("-")]
  for name, s in all_scenarios():
    if only and not any(o in name for o in only):
      continue
    res = {ref: run(mods[ref], s) for ref in REFS}
    print("=" * 110)
    print("SCENARIO", name)
    for ref in REFS:
      rows = res[ref]
      m = metrics(rows)
      print(f"-- {SHORT[ref]} ({ref}, timeout {mods[ref]['timeout']})  HMS seq: {m['hms_seq']}")
      if verbose or ref != "hms_latch_fix":
        for line in transitions(rows):
          print("   " + line)
      q1 = ", ".join(f"{k}: {d:.2f}s v{lo:.2f}-{hi:.2f} acc<0 {n:.2f}s first {t0:+.2f}" for k, (d, lo, hi, n, t0) in m["Q1"].items()) or "none"
      print(f"   Q1 HMS4+A7Anfahren while rolling 0.05<v<1.0: {q1}")
      print(f"   Q2 HMS changes at standstill: {' '.join(m['Q2s']) or 'none'} | rolling v<1.0: {' '.join(m['Q2v']) or 'none'}")
      print(f"   Q3 A6 Anhalten with A7 Anfahren/HMS4: total {m['Q3'][0]:.2f}s longest {m['Q3'][1]:.2f}s runs {m['Q3'][2]} "
            f"first {m['Q3'][3]} | reverse (A6 Anfahren with A7 Anhalten/HMS1): {m['Q3b'][0]:.2f}s")
      print(f"   Q4 start request (HMS4/A7G/A6G) with accel<0 at standstill: {m['Q4'][0]:.2f}s "
            f"(with A6 Anhalten {m['Q4'][1]:.2f}s, with A6 Anfahren {m['Q4'][2]:.2f}s, min acc {m['Q4'][3]:+.2f})")
      print(f"   HMS1 total {m['HMS1'][0]:.2f}s (standstill {m['HMS1'][1]:.2f}, rolling v<1 {m['HMS1'][2]:.2f}, "
            f"v>=1 {m['HMS1'][3]:.2f}); ACC_07 Anhalten total {m['A7stp']:.2f}s")
    for a, b in (("hms_latch_fix", "claude/determined-hypatia-pnidyz"), ("claude/determined-hypatia-pnidyz", "hold_tsk_owned")):
      w = diff_windows(res[a], res[b])
      ws = " ".join(f"[{x[0]:+.2f}..{x[1]:+.2f}]" for x in w[:12]) + (" ..." if len(w) > 12 else "")
      print(f"   DIFF {SHORT[a]} vs {SHORT[b]}: {sum(x[2] for x in w) * FR:.2f}s in {len(w)} windows {ws}")
    print()


if __name__ == "__main__":
  main()
