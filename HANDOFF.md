# Handoff: MK7 GTI openpilot longitudinal tuning (as of 2026-10-21)

Paste this whole note into the new session as the first message. It is written for the next Claude session.

## Who / what
- Car: 2017 VW GTI Mk7 (MQB, 2.0T, DSG, ACC type 1, **no electronic parking brake**: manual handbrake), comma 3X.
- Software: sunnypilot fork, openpilot longitudinal, personality **aggressive**, camera-only leads (no radar), Experimental mode never used.
- Repo: **github.com/jerryromero666/mk7-op-long-sng**. On the device the remote is called `sngrepo`.
- The user is **not a programmer**. Use plain English, short answers and exact copy-paste commands for the device with the **car OFF**.

## Working rules the user set
- One change per branch, with a new branch for each fix (allowed). Never push to a branch that wasn't agreed on, never open a PR unless asked.
- **No new Params keys.** Live tuning only through plain files in `/data/<name>`. The code re-reads them about once a second and clamps them. No file = the behaviour without the feature.
- Panda accel limits [-3.5, 2.0] m/s². The acados solver constants are compiled (6 m stop distance, the v²/5 approach term), so only Python runtime params can change.
- Diagnostic scripts on the device must be **read-only**.
- No AI model names in commits or code. Commits end with a Co-Authored-By trailer.
- Be honest when something isn't safe or isn't shippable. The user prefers real results to promises.

## Device commands
- Switch branch: `/data/mk7_switch.sh <branch>` (fetches from sngrepo and checks out; reboot/restart after).
- Install a tool from the `t2_set_min` branch (example):
  `cd /data/openpilot && git fetch sngrepo t2_set_min && git show FETCH_HEAD:mk7_stop_check.py > /data/mk7_stop_check.py.new && mv /data/mk7_stop_check.py.new /data/mk7_stop_check.py`
- Report tools (read-only, run with the car off):
  - `python3 /data/mk7_stop_check.py 0`: every stop in the latest drive (v1.3.1: grade, roll after the stop, soft-stop profile, gap at standstill).
  - `python3 /data/mk7_follow_check.py 0`: highway following at 45+ mph, coast vs brake per approach (v1.2.1).
  - `mk7_pickup_check.py`: what limited each pickup. Hold trace (`mk7_hold_trace.py`) is on the `mk7_tools` branch.

## CURRENT STATE: the user is about to drive `t2_coast` (a726f2d)
Lineage (each branch is built on the previous one):
`t2_close_follow` (the original starting branch) → `t2_set_min` → `t2_set_min_gap` → `t2_highway_follow` (3be8b9f) → `t2_stop_smooth` (e507456) → `t2_decline_stop` (6fe4add) → **`t2_coast` (a726f2d)**

What `t2_coast` contains, beyond `t2_close_follow`:

| Feature | File in /data (default without file) | Notes |
|---|---|---|
| SET at a stop starts at 40 mph (only under 10 mph) | `set_min_mph` (40) | |
| Close follow (about 3 m closer at 5-20 mph while the lead pulls away) + NaN guard | `close_follow` ("3.0 0.9") | from t2_close_follow |
| Highway follow: shorter time gap from 45 mph up | `highway_follow` (off; **user has 0.8**) | gap = 6 m + T·v, clamp 0.5-1.75 s, full at 35-90 mph |
| Smooth, consistent stops: about 4 m at standstill every time, soft end, firm hold once stopped | `stop_gap` (4.0 m, 3-7; 0 = old) and `stop_soft` (1.5; 0 = old firm end) | replaced the old stop-gap 1.5/2.0 m versions |
| Downhill stops: no "slow down, stop, roll, stop" | `decline_stop` (2.5 % slope threshold, 1-10; 0 = off) | grade from Motor_16 TSK_Steigung; keeps brakes on near the stop, no go-flickers at standstill, roll guard -2.0 |
| **Coast-first highway approach (NEW)** | `coast_approach` (**off**; user told to `echo 1 > /data/coast_approach`) | "1" = 0.55 m/s² lift-off decel; 0.2-0.6 accepted |
| Pickup accel up to 25 mph | `pickup_accel` (1.8) | |

Also inherited from t2_close_follow: hill hold v2 (announce at about 2 mph, ESP hold gets HMS 3), launch kick/boost, braking ramp restore (4.0), HMS4 latch fix, silent wrongGear, screen-wake suppression, steering speed/yaw tweaks.

### The coast feature (what to tell the user / what to check)
- Before this, the user had **never driven** any coasting. Two earlier designs failed the safety check in the sim and were never shipped.
- How it works: at 45+ mph, while closing on a slower car, it puts a floor under the planned accel and the sent request at the car's lift-off decel. That floor only turns on while the car could still stop 3 m short if the lead braked -3 m/s² right now (1.8 s delay assumed). It drops at once on a cut-in, a camera dropout, a confirmed braking lead or a lost reserve, and it is inert under 40 mph.
- Sim on this branch (2306 runs): no file = identical; 336 hard-brake approaches with no new contacts; one tie in cut-ins (today 0.0 m left, coast 0.16 mph); 1 of 12 hardest-braking cases 0.25 m/s² worse.
- Expected feel is **modest**. 77→68 mph from 110 m with a noisy camera: brakes on 0.4 s instead of 0.9 s. Fast approaches on much slower traffic still brake like today on purpose (coasting there was what failed safety).
- Real car facts: there's no braking for requests down to about -0.55 m/s² (fuel-cut overrun) and the brakes come on below about -0.6. Lift-off decel is -0.29 to -0.37 at 55-75 mph. The early highway braking is probably the camera misjudging lead speed at 100+ m; only better far-lead data would fix that properly.

## Next steps (waiting on the user's drive)
1. After a drive on `t2_coast` with `coast_approach` = 1, ask for `python3 /data/mk7_follow_check.py 0` and `python3 /data/mk7_stop_check.py 0` output.
2. Highway: compare coast vs brake per approach with earlier drives (34d/34f). The user's top complaint is that it "brakes at high speed instead of coasting".
3. Stops: check the gap at standstill (target about 4 m, consistent), the soft end, and on declines (< -3 %) that there's **no roll after the first stop**. v1.3.1 prints grade and roll kinds slow/stop/held/go and a go-chatter count.
4. Possible later work, only if the user wants it: far-range lead logging (dRel / vLead / aLeadK / modelProb around each early brake) to build a lead plausibility filter, and a stock phantom stab at 25-30 mph.

## User verdicts on past branches (don't repeat these)
- `t2_announce_brake`: "much more phantom braking". Abandoned (its idea survives only on declines inside decline_stop).
- `t2_stop_gap2`: "worst one we've had", with inconsistent stop distance and a rough standstill. Replaced by t2_stop_smooth.
- Earlier stop-gap versions (1.5 m) barely added distance behind a car braking to a stop.
- `t2_cruise_ease`, `t2_stop_comfort`: built, never driven.

## Car-side facts worth keeping (VW MQB standstill)
- MQBStandstillManager (opendbc carcontroller.py): early stop announcement at 0.9 m/s (HMS 1 + ACC_07 Anhalten + ACC_06 Anfahren until ESP_21 bit 63), then the announced hold HMS 4 + ACC_07 Anfahren.
- Hill-hold history rules: the ESP holds only after HMS 1; no HMS 4 at standstill without bit 63 (it faulted the TSK); 4>1>3>4 cycling faults the TSK; ACC_06 Anhalten in the HMS 1 window drops brake torque.
- With openpilot long the hold is continuous stopping messages (no 3 s release trick needed; that resume-spam path only runs with stock ACC / pcmCruise).
- Brake signals: ESP_05 brake-for-TSK bit, TSK_06 brake torque, Motor_20 TSK target; ESP_05 pressure = driver pedal only.

## Note on lost material
The previous session's simulator harness and patch files lived in an ephemeral scratchpad and do **not** carry over. Everything that shipped is in the branches above, with full explanations in the commit messages (`git log origin/t2_coast`). An older sim harness snapshot is in `handoff/` on the `mk7_tools` branch.
