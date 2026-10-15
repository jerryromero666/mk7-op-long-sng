# Mk7 GTI openpilot tuning: handoff note (2026-10-15, update 2)

## 0. WHERE THINGS STAND RIGHT NOW (read first)

- **Launch boost:** the build is committed as `1973928` on local branch `t2_launch_boost`, based on `t2_lead2_phantom_fix`. It has NOT been pushed and has NOT been reviewed.
  - The exact commit is in `handoff/launch_boost_wip/0001-*.patch`. Recreate the branch with:
    `git checkout -b t2_launch_boost origin/t2_lead2_phantom_fix && git am handoff/launch_boost_wip/0001-*.patch`
  - The final simulator runs were still going when this note was written. Next: run the two adversarial reviews in section 8, fix what they find, then push and give the user `/data/mk7_switch.sh t2_launch_boost`.
  - The user is waiting to test this branch next.
- **Hill-hold study:** done. The result is in section 9 and in `handoff/holdv2/hold_design_result.json`. The recommended fix is candidate A, but build it ONLY after the user's fault traces confirm it. Those commands are in section 9.
- The user said their usage is low (89% of the weekly limit used). Keep replies short and spend effort only on the boost review and the hill-hold fix.


Paste this whole file into a new Claude Code session (any account) that has access to
`github.com/jerryromero666/mk7-op-long-sng`. Everything referenced here is on the
`mk7_tools` branch under `handoff/`, unless it says "on origin as branch X".

---

## 1. The car, the device, the repo

- **Car:** 2017 VW GTI Mk7 Autobahn.
  - MQB platform, ACC type 1.
  - Manual handbrake, so no EPB and no ESP_15 (reading ESP_15 crashes this car).
  - DSG gearbox.
- **Device:** comma 3X running a sunnypilot fork (v2026.001.000 staging base).
  - SSH: `ssh comma@192.168.0.195`. It's on the home LAN, so a cloud session can't reach it. The user runs commands and pastes the output back.
  - The repo on the device is `/data/openpilot`.
  - Device git remote `sngrepo` = github.com/jerryromero666/mk7-op-long-sng (`origin` in a cloud session).
  - Remote `myfork` = jerryromero666/sunnypilot (branch mqb_mk7_stop_n_go). Never pushed to.
- **Logs on the device:** `/data/media/0/realdata/<8-hex-counter>--<id>--<seg>/rlog.zst`.
- **Python on the device:** `/usr/local/venv/bin/python3`, run from `/data/openpilot`.
  - Use `from openpilot.tools.lib.logreader import LogReader`.

## 2. User goals and hard rules

**Goals:**
- **A.** Hill hold at ANY incline or decline (11% included). No rollback, no fault, no grade special-casing.
- **B.** Quicker launch from a dead stop behind a departing car.
- **C.** Closer following at higher speeds.
- **D.** SNG (stop-and-go) that feels like the factory system.

**Rules:**
- One change per branch per drive. The user A/B tests each branch on the road.
- Never commit to `hms_latch_fix`. That's the user's old baseline (d861e6b).
- No new Params keys. Live tuning only via plain `/data/<file>` files, re-read at most once a second and clamped.
- acados cannot be regenerated on the device. These solver constants are compiled in:
  - STOP_DISTANCE 6.0. The Python value of 7.5 never reaches the solver.
  - COMFORT_BRAKE 2.5.

  These are live at runtime:
  - `t_follow` (solver param p[4]).
  - The cost weights W (set through `cost_set`).
- Only Python changes take effect. The checkout has a `prebuilt` marker, so it doesn't recompile.
- Limits:
  - Panda accel limit 2.0 m/s². Frames above it are blocked, which kills the whole ACC_06/07 frame.
  - CarControllerParams ACCEL_MAX 2.0.
- Planner caps:
  - `A_CRUISE_MAX_VALS` [1.6,1.2,0.8,0.6] at [0,10,25,40] m/s.
  - `limit_accel_in_turns` caps accel at 1.7 below 20 m/s even with the wheel straight.
  - The VW long PID gains are 0, so the longcontrol output equals aTarget.
- The user's preferences for replies:
  - Plain-English explanations.
  - Exact copy-paste commands.
  - Car OFF for any device command.
  - Read-only diagnostic scripts.
- Commits end with the two attribution lines the session gives. Don't put model names in commits.

## 3. Camera calibration (keep in any simulation)

- `selfdrive/locationd/calibrationd.py` hard-locks camera pitch `FIXED_PITCH = 0.0236` rad and yaw `-0.0070` rad.
- Pitch error makes far leads read farther than they are. The error grows roughly with distance squared. Near leads (under ~20 m) are fine.
- There's a speed-dependent yaw taper (speed_yaw up to -0.0130 rad, ramping from 13 to 31 m/s). It corrects the left drift on the highway caused by the mount position (parallax). It only shifts yRel and has no longitudinal effect.
- Identical on every branch. Don't press "Reset Calibration" between A/B drives.

## 4. Branches (all on origin; each is ONE change on top of its parent)

```
hms_latch_fix (d861e6b, old baseline)
└─ claude/determined-hypatia-pnidyz (1478030)  Task 1: fixed UPHILL_ACCEL_FLOOR 1.5 (no grade/aEgo taper); HOLD_REQUEST_TIMEOUT 20
   └─ t2_lead2_phantom_fix (e558ce7)  per-lead dropout state (ghost leadTwo fix); dRel clamp only limits increases
      ├─ t2_launch_floor_2p0       UPHILL_ACCEL_FLOOR 2.0 (floor only applies when ESP hold confirmed, which never happens on this car -> no effect)
      ├─ t2_accel_cap_launch (f0722e0)  A_CRUISE_MAX_VALS [1.7,1.7,1.2,0.8,0.6] @ [0,6,10,25,40]   <- TESTED drive 339: no felt change
      ├─ t2_achange_cost_launch    A_CHANGE_COST 40 below 3 m/s, back to 75 by 8 m/s
      ├─ t2_cruise_max_accel_2p0   CRUISE_MAX_ACCEL 2.0  (tested drive 336 = launch baseline)
      ├─ hold_tsk_owned (9439ecc)  HOLD_REQUEST_TIMEOUT 0   <- TESTED drives 337/338: SOFT FAULT on any hill. User: DO NOT pursue.
      └─ fix_brake_ramp_4p0        ACC_neg_Sollbeschl_Grad_02 back to 4.0 (89a239f had set 1.0 m/s3; whether the ESP honors it is unverified)
         ├─ t3_highway_aggressive
         └─ t3_stop_distance_taper  /data/stop_distance "low high" metres (clamp 4.5-9.0), shifts lead obstacles; no shrink at standstill
```

- Dropped after review: `t2_jerk_cost_4` (stop/go chatter at standstill) and the original `t3_tfollow_taper`.
- Other origin branches are old history: MK7YAHEARD, SNG_YA_DIG, _Mk7_95_Percenter_yahhuurd, climb_taper_fix, srbm_grade_hold_fix, staging, live_stop_distance.
- **Not on origin (lost if the cloud session dies):**
  - `t2_launch_gap_match`: latches the stop gap as a lead shift. The simulator gain was only 0.05 to 0.2 s, so it isn't worth a drive. The idea is described in section 7.
  - `t2_launch_boost`: work in progress. A snapshot is in `handoff/launch_boost_wip/`.

## 5. Device tools (branch `mk7_tools` on origin; install with car OFF)

```
cd /data/openpilot && git fetch sngrepo mk7_tools && git show FETCH_HEAD:mk7_switch.sh > /data/mk7_switch.sh && chmod +x /data/mk7_switch.sh
cd /data/openpilot && git fetch sngrepo mk7_tools && git show FETCH_HEAD:mk7_hold_trace.py > /data/mk7_hold_trace.py
cd /data/openpilot && git fetch sngrepo mk7_tools && git show FETCH_HEAD:mk7_launch2.py > /data/mk7_launch2.py
```

**`/data/mk7_switch.sh`:** switches branches from a numbered menu or by name.
- Curated menu:
  1. claude/determined…
  2. phantom
  3. floor
  4. accel_cap_launch
  5. achange
  6. cruise_max
  7. brake_ramp
  8. hwy_aggr
  9. stop_dist
  10. hms_latch_fix
- Usage: `/data/mk7_switch.sh 4`, `/data/mk7_switch.sh <branch>`, `prev`, `status`, `list`, `update`.
- It refuses to switch while the car is on, stashes edits, checks that the Python compiles (rolls back if it doesn't), logs to `/data/mk7_branch_log.txt`, then asks to reboot.
- To add a menu item, edit the `BRANCHES=(...)` list near the top of the script.

**`/data/mk7_launch2.py N`:** read-only; 0 = newest drive. Prints:
- The branch and commit.
- Every new fault or alert event with its speed.
- Per launch from a stop:
  - planMax (peak longitudinalPlan.aTarget in the first 4 s).
  - actMax and act1s (carState.aEgo peak and mean over about 1 s).
  - v4 (speed at 4 s).
  - Time to reach 5 m/s.
  - Lead dRel and aL (peak aLeadK).

**`/data/mk7_hold_trace.py`:** read-only, v2.0.
- Usage: `--list N`, `--route N` (1 = newest), `--fault N`, `--event N`, `--all-timelines`, `--csv FILE`.
- Prints what OP sent to the ESP/TSK and how the car responded around each stop, plus a PASTE block of at most about 40 lines.

**`/data/mk7_launch.py N`:** an older, user-written launch script (start delay and gap growth).

## 6. What the drive data says

**Routes 332/333 (`mk7_hold_trace`), hill hold:**
- The ESP hold confirmation (ESP_21 bit 53) was never set on any stop, so on this car the ESP hold never forms.
- HMS=1 (ACC_07 Anhalten / hold request) makes the TSK drop its wheel brake torque (TSK_Radbremsmom) to 0.
- With HMS=4 latched, the TSK holds raw ~117–143 (~1000 Nm) for 20–26 s.
- On the +11.2% stop:
  - OP's stopping state began 0.04 s after the wheels reached 0 (the plan lags).
  - Torque was 0 during the HMS=1 window.
  - The car rolled back 0.11 m at +0.16 s until the driver braked.
- Root cause of the rollback is the HMS=1 handoff gap: with timeout 20 there's about 0.4 s of zero brake torque.

**Routes 337/338 (`hold_tsk_owned`, timeout 0):**
- `accFaulted` while stopped:
  - Route 338 at 0:23.
  - Route 337 at 3:35.
- The user says it faulted on any hill and they had to turn cruise off and on to recover. Same symptom as faults seen before.
- Early state-machine replay result (`handoff/holdv2/design/out_design.txt`): with timeout 0, the HMS=1 window is only 0.02 s, and HMS=4 plus ACC_07 Anfahren (start) goes out at v≈1.0 m/s **while the car is still rolling**. That's because `at_standstill = CS.out.standstill or vEgo < 1.0`.
  - At the same time, ACC_06 still says Anhalten (stopping) with negative accel. That contradictory start-while-braking command is the leading suspect for the ESP plausibility fault.
  - Old commit messages disagree: f30a396 era says "permanent HMS=4 → SRBM", while d861e6b says "the ESP tolerates long steady HMS=4; cycling 4→1→3→4 is what faults".
  - Not confirmed yet. The user still needs to run the hold traces in section 9.

**Route 339 (`t2_accel_cap_launch`), launch:** 8 real launches.

| | Median |
|---|---|
| planMax | 1.38 m/s² |
| actMax | 1.67 m/s² |
| v4 | 3.98 m/s |

- The new 1.7 cap was reached on only **1 of 8** launches.
- First-second mean accel was only **0.6–0.7 m/s²** on 3 launches.
- The lead was 4–6 m ahead, with aL 1.3–2.3.

**Route 336 (`t2_cruise_max_accel_2p0`) launch baseline:** median start delay 1.05 s after the lead moves, and the gap grows 7.8 m in 4 s.

**Ghost-car fix (route 336/334 analysis):**
- The ghost condition occurred in 4.7% / 19.2% of close-lead moving frames.
- The fix is real but rarely active. The user felt no difference.

## 7. Simulation findings (launch)

- **Harness:** `handoff/sim/mpcsim.py`. It runs the REAL `long_mpc.py` from any git ref against the device's compiled solver, built for x86 in `sim/cgen`.
  - The planner, longcontrol and carcontroller clip are line-by-line replays.
  - Plant: pure delay 0.15 s plus a first-order lag of 0.35 s. No grade.
  - Venv needs casadi 3.7.2, Cython 3.3, numpy, scipy, Jinja2, future-fstrings.
  - The cgen `.so` files aren't committed. Rebuild from `handoff/sim/cgen` with its Makefile against the repo's acados (third_party/acados).
  - The scripts hard-code the old scratchpad path and `REPO=/home/user/mk7-op-long-sng`; change both.
- **No tuning knob fixes the launch.** Every variant tested (gap0 4–6.5 m, lead aL 1.4–2.2) has a start delay of **1.05–1.5 s**, and v@4s improves by at most about +0.3 m/s.
  - Variants: accel cap 1.7/2.0, A_CHANGE_COST 30/40, J_EGO_COST 2, floor 2.0, t_follow 1.45 down to 0.8, stop-gap latch.
  - The delay is structural: the MPC's jerk-limited reaction plus 0.5 s actuator delay.
- **Fix in progress: a feed-forward "launch boost".** It reacts to the lead departing instead of waiting for the MPC.

## 8. `t2_launch_boost`: status and design (BUILT as 1973928, NOT REVIEWED, NOT PUSHED)

- **Snapshot:** `handoff/launch_boost_wip/0001-*.patch` (the full commit, apply with `git am`) and `launch_boost.py`. It adds a new module and wires it into `selfdrive/controls/lib/longitudinal_planner.py`. The constants in the design below are from the first draft; the committed module is the source of truth.
- **Base:** `origin/t2_lead2_phantom_fix`.
- **Simulator scripts:** `handoff/sim/boost/` (boostsim.py, eval_launch.py, safety.py, falsefire.py, creepcheck.py …).

**Design** (see the module docstring):
- **Arm:** ego stopped for 1 s behind a stopped lead, long active, no pedals. Latch the stop gap.
- **Trigger:** all of these, for 3 frames in a row:
  - The MPC already asks to go (a > 0.1, so the boost never takes the car out of stopping by itself).
  - vLead > 0.9.
  - The gap has opened ≥ 0.4 m beyond the stop gap.
- **Boost:** the minimum of three bounds:
  - Speed: K 1.5 × (vLead − vEgo − 0.3).
  - Gap: K 3.0 × (gap − stop gap − 0.2 − 1.0 × vEgo).
  - Accel: aLeadK + 0.3.

  It ramps up at 2.5 m/s³ and down at 1.5 m/s³, capped at `BOOST_MAX` 1.5 (live file `/data/launch_boost`, one number, 0 = off, clamped to 1.6).
- **Planner output:** max(MPC, boost), applied before the e2e blend and the accel clip, so all the existing limits still apply. The boost never brakes.
- **Handover:** fades out when vEgo > 5 m/s, when the MPC asks for at least the speed bound, or after 6 s.
- **Abort** (boost goes to 0 immediately):
  - Lead lost, or leadTwo closer than leadOne.
  - Gap shrinking or closing on the lead.
  - The lead falls back below its confirm speed.
  - The MPC wants to brake, or a crash risk.
  - Gas or brake pressed, or long control inactive.

  Once it has fired, it doesn't re-arm until the car has driven off and stopped again.
- While boosting, the planner puts the MPC's speed and accel state on the car's real state, so an abort hands over cleanly.

**Remaining work:**
- Finish the build.
- Run a simulator comparison against `t2_lead2_phantom_fix` with gap 3.5/4.5/5.5 m, lead aL 1.0/1.6/2.3/3.0, and vision-like noise (dRel σ0.3, vLead σ0.3–0.5, lagged aLeadK).
- Safety battery:
  - The lead departs, then stops again after 1, 2 or 3 s.
  - The lead creeps at 0.3–0.5 m/s.
  - 60 s stationary with noise: **zero false fires** across seeds.
  - Cut-in at 4–6 m mid-boost.
  - LeadOne lost mid-boost.
  - LeadTwo closer.
  - Driver brakes.
  - Stop-and-go crawl.
- Then two adversarial reviews:
  - Safety and integration: is the boost in the planner the car actually runs (sunnypilot vs openpilot planner, e2e/experimental mode)? Does it handle the should_stop and longcontrol STOPPING↔PID transitions, and the HMS=4 departure flag in MQBStandstillManager?
  - Real-car benefit: does it fire in the drive-339 conditions given vision lag? Target a first-second mean accel ≥ 1.2 m/s² and a start delay cut of ≥ 0.3 s.

  Watch the planner's accel_clip slew (0.05 per 50 ms), which may clip it.
- Only after both reviews pass:
  - Push, add it to the `mk7_switch.sh` menu.
  - Tell the user: `/data/mk7_switch.sh t2_launch_boost` → reboot → drive with stop-and-go → `python3 /data/mk7_launch2.py 0`.
  - Compare against route 339: act1s 0.6–1.5, planMax 1.38, v4 3.98.

## 9. Hill hold v2: study DONE, waiting on user traces (goal A, still unsolved)

**Study result (full detail in `handoff/holdv2/hold_design_result.json`; raw studies in `hold_studies_raw.json`; frame-replay tools in `holdv2/replay` and `holdv2/design/candidates.py`):**
- The only CAN difference between `hold_tsk_owned` (faults) and its parent (no faults) is the parent's 0.40 s **ACC_07 stop announcement**: HMS=1, Anhalten=1 and Anhalteweg 0.3, sent while the car is still moving. Every other frame is identical.
- `hold_tsk_owned` latches HMS=4 on the first stopping frame below 1.0 m/s. On hills the planner's stopping state starts late, at or after the wheels hit 0, so the announcement is never sent.
- **Root-cause ranking:**
  - **H1 (~55%):** without the announcement, the ESP's ACC stopping procedure (ESP_21 bit 63) never runs, so the ESP treats the standstill as time-limited. TSK goes 3→6 a few seconds into the stop. Route 338 faulted about 3–4 s after stopping.
  - **H3 (~15%):** a plausibility rejection of start-without-stop. Same fix as H1.
  - **H2 (~15%):** TSK torque too low or slip on the grade.
  - **H4 (~7%):** HMS 1↔4 cycling as speed crosses 1.0 m/s.
  - **H5 (~8%):** confounds such as startup handover, a second ACC sender or pedal toggling.
- Every fault-free hold on record (routes 332/333, holds of 11–26 s) had that announcement while the car was moving, with bit 63 on and then off before HMS=4.

**Recommended fix, candidate A ("hold_announce_early"):**
- New branch off `t2_lead2_phantom_fix`. Change only `MQBStandstillManager`.
- **Announcement:** send the parent's proven 20-frame (0.4 s) announcement on the first frame with vEgo < 1.5 m/s while braking (accel < 0 or stopping). This trigger is speed-based instead of OP's stopping state, so planner lag can't skip it.
- **Window:** cut it short if vEgo falls below 0.5 m/s, so it always ends while the car is still moving.
- **Hold:** latch HMS=4, ACC_07 Anfahren and Anhalteweg 20.46 until vEgo > 2.6 m/s or long control goes off.
- **Never:**
  - Send HMS=1 at standstill.
  - Change HMS at standstill.
  - Use any grade input.
- The code sketch is in the JSON (candidate A).
- Risk: ACC_06 shows the pid state for most of the window. If the ESP needs a consistent ACC_06, force ACC_06 to S or G during the window.
- Alternative B: end the window on ESP_21 bit 63 (`ESP_Anhaltevorgang_ACC_aktiv`), RJ's handshake. This needs a one-line carstate field.
- Rejected: C, D and E (they would still fault or roll back; reasons in the JSON).
- **Gate:** build or drive A ONLY if the faulting stops on 337/338 show "ESP ACC stopping procedure (bit 63): never on".
  - If bit 63 was on and the stop still faulted, H1 is refuted. Look at H2/H4/H5 instead.

**Commands for the user** (car OFF). Check the numbering first: 339 = 1, 338 = 2, 337 = 3. A route name prefix also works (`--route 00000338`).
```
python3 /data/mk7_hold_trace.py --list 6
python3 /data/mk7_hold_trace.py --route 2
python3 /data/mk7_hold_trace.py --route 2 --fault 1 --hz 25 --before 6 --after 2
python3 /data/mk7_hold_trace.py --route 2 --all-timelines | grep -F -e 'IN DETAIL' -e 'Grade while' -e 'stopping state began' -e 'Stood still' -e '(bit 63)' -e 'TSK status' -e 'OP HMS sent' -e 'during stop' -e '2 s before'
python3 /data/mk7_hold_trace.py --route 3
python3 /data/mk7_hold_trace.py --route 3 --fault 1 --hz 25 --before 6 --after 2
python3 /data/mk7_hold_trace.py --route 1     # control: route 339, parent hold code, no faults
```
**What to look for:**
- "TSK status 3 -> 6" (a 7 would mean chatter).
- When the fault happens: "FAULT 6 at +X s". H1 predicts a similar X in both drives.
- "(bit 63): never on" is the go-ahead for A.
- "stopping state began" at or after +0.00 s, with no "OP HMS -> 1" before the wheels reach 0.
- The ESP bits in the 2 s before the fault:
  - Bit 38 or 45 first means H3.
  - Bit 29 first means the ESP withdrew.
  - Chatter or pedal toggling means H4/H5.
  - "second ACC sender" points to the startup handover, which matters for route 338's fault at 0:23.

After building A: drive flat first, then a steep hill with a foot ready. Check with `mk7_hold_trace --route 1`: did it hold over 20 s on 11%, with no TSK 6?

Background notes: `handoff/hillhold_study/` (01_hold_chain, 02_history, 03_external, 04_hypotheses, 05_trace_signals, `vw_mqb.dbc`).

## 10. Other pending items (lower priority)

- **`mk7_pitch_check.py`:** reviewed as "ship with fixes"; 3 fixes are pending before publishing it:
  - An approach-mode consistency check.
  - Sort routes by the hex counter.
  - Fix the README/help install text.
- **ESP brake-ramp check:** confirm whether the ESP honors ACC_neg_Sollbeschl_Grad_02 at 1.0 m/s³ (compare decel onset on a `fix_brake_ramp_4p0` drive).
- **Restack branches:** put the eventual hill-hold fix and `fix_brake_ramp_4p0` under all the others once proven.
- **Launch script glitch:** fix the 131 s outlier in `mk7_launch.py`.
- **Goal C** (closer following at speed): `t3_highway_aggressive` and `t3_stop_distance_taper` are built but haven't been driven yet.
- **Unexplained items:** a foreign ACC sender, panda-refused frames, and a bit29 event seen in earlier traces.
