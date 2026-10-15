#!/usr/bin/env python3
"""Write every 50 Hz ACC frame per scenario/ref to csv/<scenario>__<ref>.csv"""
import csv, os
from replay import REFS, SHORT, load_ref, run
from scenarios import all_scenarios
os.makedirs("csv", exist_ok=True)
mods = {r: load_ref(r) for r in REFS}
cols = ["t", "vraw", "v", "la", "lcs", "brk", "hold", "stopping", "starting", "eso", "est", "st", "a6_stp", "a6_go", "acc",
        "hms", "a7_stp", "a7_go", "dist", "latched", "reqf"]
for name, s in all_scenarios():
  for r in REFS:
    with open(f"csv/{name}__{SHORT[r]}.csv", "w", newline="") as f:
      w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
      w.writeheader()
      for row in run(mods[r], s):
        w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in row.items()})
print("ok", len(os.listdir("csv")))
