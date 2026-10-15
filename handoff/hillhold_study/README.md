# Hill-hold study: why the Mk7 GTI rolls back after an OP stop on 11% but holds on ~5%

Branch studied: claude/determined-hypatia-pnidyz (HEAD 1478030). Read-only: nothing in the repo was
changed.

Files
- 01_hold_chain.md: planner -> longcontrol -> MQBStandstillManager -> ACC_06/ACC_07 -> ESP/TSK, with a
  per-state table of exactly what goes on the wire (from running the branch's own code).
- 02_history.md: every hold-related commit and branch, the evidence each cites, and what that says
  about this ESP.
- 03_external.md: upstream opendbc, RJWadley's PR #2764/#3322 (the "RJ" the code credits), the MEB
  notes, the openpilot VW wiki, and physics numbers. Facts and inferences are marked separately.
- 04_hypotheses.md: ranked hypotheses, each with its 50 Hz signature, a minimal fix and the risks.
- 05_trace_signals.md: the exact signals and the decision table.
- sim/: `hms_sim.py` (branch code replay; run with `python3 -I hms_sim.py` or `... C2`),
  `physics.py`, outputs `*_out*.txt`, and branch_* source copies.
- ext/: downloaded upstream/RJ/iqpilot files (untrusted data, reference only). vw_mqb.dbc: copy from
  the branch.

Top findings
1. A target acceleration is not a brake force. If the TSK grade-compensates (it broadcasts grade and
   mass in 0x65F), -1.2 leaves 0.71 m/s^2 of brake on 5% (enough: 0.49 needed) but only 0.13 on 11%
   (1.07 needed). Crossover is ~6.1%. That one line matches "holds ~5%, rolls 11%".
2. The ESP-hold handshake is not anchored to standstill. The 20-frame HMS 1 window starts at plan
   v < 0.5 and mostly expires before the wheels stop. TIMEOUT or RELEASING then latches HMS 4 /
   ACC_07 Anfahren while ACC_06 keeps Anhalten. That is the opposite of RJ's working split (ACC_06
   starting so the TSK keeps braking, ACC_07 START), and RJ notes that a TSK left in stopping lets its
   brake torque zero out.
3. HOLDING drops ACC_07 Anhalten and sets Anhalteweg 0.3 -> 20.46; upstream keeps Anhalten=1 with
   HMS 3. This may be what d861e6b saw as "ESP reads HMS=3 as release".
4. During rollback (unsigned vEgo < 1.0) OP keeps sending HMS 4 = drive off. At >= 1.0 it re-requests
   hold, then re-latches 4 after 20 frames, a roll-catch-release loop.
5. HMS 0 next to HMS 4 is impossible from this code at true standstill while long is active. The
   "812 switches/1000, mostly 0<->4" figure is either rollback frames (pid, vEgo >= 0.5, hold bit
   flicker) or the radar's src-2 ACC_07 mixed with OP's src-128 echo. Check per-src counts first.
6. History disagrees about whether ESP_21 bit 53 ever sets on this car ("2-5 frames" vs "never in 45k+
   frames"). Nobody has logged TSK_Radbremsmom, ESP_05 brake pressure, ESP_21 bit 63 or ESP_10 pulses
   at a grade stop; those decide between the hypotheses.
