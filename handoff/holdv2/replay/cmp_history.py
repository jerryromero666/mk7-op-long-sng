#!/usr/bin/env python3
"""Does d861e6b (HMS=4 latch) change anything vs its parent f30a396 when ESP hold never confirms?"""
import replay
from replay import load_ref, run
from scenarios import all_scenarios
refs = ["f30a396", "d861e6b", "claude/determined-hypatia-pnidyz", "hold_tsk_owned"]
mods = {r: load_ref(r) for r in refs}
F = ("hms", "a7_stp", "a7_go", "dist", "a6_stp", "a6_go", "acc")
for name, s in all_scenarios():
  res = {r: run(mods[r], s) for r in refs}
  def nd(a, b):
    return sum(any(x[f] != y[f] for f in F) for x, y in zip(res[a], res[b])) * 0.02
  seq = {r: ">".join(str(h) for i, h in enumerate([x["hms"] for x in res[r]]) if i == 0 or h != res[r][i - 1]["hms"]) for r in refs}
  print(f"{name:32s} f30a396 vs d861e6b differ {nd('f30a396','d861e6b'):.2f}s | d861e6b vs PAR {nd('d861e6b','claude/determined-hypatia-pnidyz'):.2f}s | PAR vs T0 {nd('claude/determined-hypatia-pnidyz','hold_tsk_owned'):.2f}s")
  print(f"{'':32s} HMS seq f30a396 {seq['f30a396']} | T0 {seq['hold_tsk_owned']}")
