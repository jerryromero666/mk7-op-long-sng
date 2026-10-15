# 1. Hold chain on branch claude/determined-hypatia-pnidyz (HEAD 1478030)

All code read with `git show claude/determined-hypatia-pnidyz:<path>`. Per-frame CAN output was
reproduced by running the branch's own `MQBStandstillManager` and `create_acc_accel_control`
(extracted verbatim) in `sim/hms_sim.py`; outputs in `sim/hms_sim_out*.txt`.

## 1.1 Planner -> longcontrol (controlsd, 100 Hz; planner 20 Hz)

- `drive_helpers.get_accel_from_plan`: `should_stop = v_now < vEgoStopping(0.5) and a_target < 0.1`.
  `v_now` is the MPC start state = `v_desired_filter.x`, which mostly integrates the plan's own accel
  and is pulled toward `vEgo` by a first-order filter (rc 2.0 s, k = 0.024 per 50 ms step). It is not
  the measured wheel speed, so it can lead or lag `vEgo`.
- `long_control_state_trans`: stopping -> pid only when `not should_stop and not cruise_standstill and
  not brake_pressed`. `cruise_standstill` is hardwired False under OP long (trap 1). `CP.startingState`
  is False for VW, so departure goes straight to `pid`. pid -> stopping whenever `should_stop`.
- stopping output: ramps from the last output toward `stopAccel = -1.2` at `stoppingDecelRate = 6.0`
  m/s^3 (6.0 was set globally in interfaces.py by 89a239f; upstream default 0.8). Holds -1.2
  indefinitely. If the last pid output was already below -1.2 it keeps the harder value.
- pid at standstill: VW has kp = ki = 0, so output = plan a_target (pure feed-forward).
- `actuators.longControlState` is set before `LoC.update`, so carcontroller sees the state one
  controlsd frame late.

## 1.2 carcontroller.py (ACC_06/ACC_07 at 50 Hz, every 2nd controls frame)

```
stopping = lcs == stopping
starting = lcs == pid and (CS.esp_hold_confirmation or CS.out.vEgo < CP.vEgoStopping)   # 0.5; upstream now uses 0.25
```
`MQBStandstillManager.update` (ACC type 1 only):
- `at_standstill = CS.out.standstill or vEgo < 1.0`. `CS.out.standstill` is `vEgoRaw == 0`
  (carstate.py:127), not "pcmCruise and hold" as the code comment says.
- `hms_active = CC.longActive`; brakePressed only zeroes the local `long_active` (which gates only
  the launch floor). In practice a brake rising edge disengages OP (selfdrived.py:239 pedalPressed),
  so `CC.longActive` goes False a frame later anyway.
- Launch floor: if `long_active and accel > 0 and _prev_hold_confirmation and at_standstill`:
  `accel = min(max(accel, 1.5), 2.0)`.
- Latch reset: `if not at_standstill or starting: _hms4_latched = False`.
- Branches, in order:
  1. latched and at_standstill and not starting -> override START (HMS 4).
  2. at_standstill and stopping and not starting:
     - hold confirmed -> HOLDING: override (start False, stop False) -> HMS 3; `_hold_acquired=True`.
     - hold just dropped (`_prev_hold_confirmation`) -> RELEASING: START, latch=True.
     - `_hold_acquired` (4a): START for 29 frames, then reset everything -> REQUESTING again.
       The docstring's "after 100 frames (~2 s)" and "don't reset while rolling (vEgo > 0.3)" are NOT
       implemented; code uses 30 frames (0.6 s) and has no vEgo check.
     - `_hold_request_frames >= 20` (4b TIMEOUT) -> START, latch=True.
     - else REQUESTING: no override (HMS 1), counter += 1.
  3. at_standstill and starting -> START, counter reset (does not clear `_hold_acquired`).
  4. else (moving >= 1.0 m/s, or not hms_active): counters reset, no override.
- Key timing fact: the 20-frame REQUESTING counter starts when lcs enters `stopping` (plan v_now < 0.5)
  and vEgo < 1.0, i.e. while the wheels are still turning. At -1.2 m/s^2 from 0.5 m/s the car needs
  ~0.42 s (~21 frames) to stop. In sim B, 15 of the 20 frames were spent moving and only 5 at
  vEgoRaw == 0 before the TIMEOUT latch. The window is not anchored to standstill.

## 1.3 mqbcan.create_acc_accel_control: what goes on the wire

Constant while `acc_enabled` (= `CC.longActive`): ACC_06 `ACC_Typ` 1, `ACC_Status_ACC` 3
("ACC_AKTIV_regelt"), `ACC_StartStopp_Info` 1 ("Motoranlauf_nicht_zwingend_notwendig_Stoppverbot_
keine_Startanforderung"), comfort band 0.2/0.2 m/s^2, `ACC_neg_Sollbeschl_Grad_02` 1.0 m/s^3 (upstream
4.0; lowered by 89a239f), `ACC_pos_Sollbeschl_Grad_02` 4.0. ACC_07 `ACC_Freilauf_Info` 2
("Freilauf_nicht_freigegeben"), `ACC_Folgebeschl` 3.02 (neutral, panda-enforced). Both carry the same
`ACC_Sollbeschleunigung_02`. HMS = 4 if acc07_starting, elif esp_hold 3, elif acc07_stopping 1, else 0.
ACC_06 bits always use RAW stopping/starting; ACC_07 bits use the overrides.

HMS value table (VAL_ 302): 0 keine_Anforderung, 1 halten, 2 parken, 3 halten_Standby, 4 anfahren,
5 Loesen_ueber_Rampe.

| State (sim) | lcs | HMS | ACC_07 Anhalten / Anfahren / Anhalteweg | ACC_06 Anhalten / Anfahren | accel |
|---|---|---|---|---|---|
| approach, vEgo >= 0.5 | pid | 0 | 0 / 0 / 20.46 | 0 / 0 | plan |
| approach, vEgo < 0.5 but plan v_now >= 0.5 | pid | **4** | 0 / **1** / 20.46 | 0 / **1** | plan (< 0) |
| REQUESTING (<= 20 frames) | stopping | 1 | 1 / 0 / 0.30 m | 1 / 0 | -> -1.2 |
| HOLDING (ESP_21 bit 53 = 1) | stopping | 3 | **0** / 0 / **20.46** | 1 / 0 | -1.2 |
| RELEASING or TIMEOUT, then latched | stopping | 4 | 0 / **1** / 20.46 | **1** / 0 | -1.2 |
| latched, car rolling back 0..1.0 m/s | stopping | **4** | 0 / 1 / 20.46 | 1 / 0 | -1.2 |
| rolling back >= 1.0 m/s, plan still stopping | stopping | 1 | 1 / 0 / 0.30 | 1 / 0 | -1.2 |
| rolling back, plan flipped to pid, vEgo >= 0.5 | pid | 0 (4 on any hold-bit frame) | 0/0 or 0/1 | 0/0 or 0/1 | plan |
| departure (pid and (hold or vEgo < 0.5)) | pid | 4 | 0 / 1 / 20.46 | 0 / 1 | >= 1.5 if hold was confirmed |
| OP disengaged (brake/gas) | off | 3 if hold else 0 | 0 / 0 / 20.46 | 0 / 0, status 2 | 3.01 (inactive) |

Two things stand out:
1. In every latched standstill frame ACC_06 says "Anhalten" (stop, to the TSK) while ACC_07 says
   "Anfahren"/HMS 4 (drive off, to the ESP). RJ's working implementation does the opposite split
   (ACC_06 starting so the TSK keeps commanding brake torque, ACC_07 START) and comments that a TSK left
   in the stopping procedure lets its brake torque "zero itself out".
2. HOLDING withdraws the stop request from ACC_07 (Anhalten 1 -> 0, Anhalteweg 0.3 -> 20.46). Upstream
   openpilot keeps `Anhalten = 1` and `Anhalteweg = 0.3` alongside HMS 3. The d861e6b observation "ESP
   reads HMS=3 as release" may really be the ESP reacting to this withdrawal.

## 1.4 What happens when the car rolls back (sim C, C2, D)

- vEgo is unsigned wheel speed. A car rolling backward reads as moving forward.
- 0 <= vEgo < 1.0: still `at_standstill`, latch holds, so OP keeps sending HMS 4 / ACC_07 Anfahren 1
  (drive off) for the whole rollback up to 1.0 m/s. At 11% with only 0.13 m/s^2 of effective brake
  (see 04, H4) that is about 1.06 s and 0.53 m of travel.
- vEgo >= 1.0: latch, `_hold_acquired` and the counter are cleared. If the plan still says stop, HMS
  becomes 1 (request hold). If the ESP catches the car and vEgo falls below 1.0, REQUESTING runs 20
  frames, TIMEOUT re-latches HMS 4 and the car is released again: a 4 -> 1 -> 4 roll-catch-release
  loop (sim C2: HMS 4 for 1.26 s, HMS 1 for 0.68 s, repeat).
- Planner: rollback pulls `v_now` up slowly (2 s filter). If it passes 0.5, `should_stop` drops,
  lcs -> pid. In pid with vEgo >= 0.5, `starting` = hold bit, so HMS is 0 on frames without the
  hold bit and 4 on frames with it. A flickering ESP_21 bit 53 then gives strict 0 <-> 4 alternation
  at 50 Hz (sim D: 61 switches in 70 frames). This is the only code path that can put HMS 0 next to
  HMS 4 while long is active; it needs vEgo >= 0.5, which means the car is moving.
- With lcs chattering pid <-> stopping at true standstill (trap 4), HMS alternates 1 <-> 4, not
  0 <-> 4 (sim E).

## 1.5 ESP / TSK side (signals, from vw_mqb.dbc)

- 0xFD ESP_21: bit 53 `ESP_Haltebestaetigung` (hold confirmed); bit 63 `ESP_Anhaltevorgang_ACC_aktiv`
  (ESP running the ACC stopping procedure; RJ's trigger signal); bit 61 `ESP_Eingriff`.
- 0x1AB ESP_33: bit 28 `ESC_TSK_SRBM_Anf` (ESP standstill hold active); bit 29
  `ESC_TSK_SRBM_nicht_verfuegbar` (ESP has given up hold; nothing OP sends clears it); bit 38
  `ESC_Konsistenz_ACC`; bit 45 `ESC_Konsistenz_TSK`; bits 30-33 `ESC_Verz_Reg_aktiv`.
- 0x120 TSK_06: bits 24-26 `TSK_Status` (0 init, 1 disabled, 2 enabled, 3 regulating,
  4 accel_pedal_override, 5 brake_only, 6 temp_fault, 7 perm_fault); bits 12-23 `TSK_Radbremsmom`
  (x8 Nm, wheel brake torque the TSK asks of the ESP); bit 30 `TSK_Freig_Verzoeg_Anf`; bit 28
  `TSK_Standby_Anf_ESP`; bits 44-46 `TSK_SRBM_Anf_ASIL`.
- 0x106 ESP_05: bits 16-25 `ESP_Bremsdruck` (x0.3 - 30 bar); bit 26 `ESP_Fahrer_bremst`; bit 27
  `ESP_Verz_TSK_aktiv`; bit 29 `ESP_Konsistenz_TSK`; bit 39 `ESP_Rueckwaertsfahrt_erkannt`; bit 40
  `ESP_Status_Anfahrhilfe`.
- 0x116 ESP_10: 10-bit wheel pulse counters `ESP_Wegimpuls_VL/VR/HL/HR` (bits 16/26/36/46) and 2-bit
  `ESP_xx_Fahrtrichtung` (bits 56/58/60/62; RJ reads 1 = backward, 0 = forward). jyoung8607 notes FtS
  cars may lack bidirectional rear sensors, so direction may be invalid on this car; pulses still
  show creep that wheel speed rounds to 0.
- 0x65F Motor_16: bits 56-63 `TSK_Steigung` (x0.8 - 101.6 %), bit 12 `TSK_QBit_Steigung`, bits 48-55
  `TSK_Fahrzeugmasse_02` (x32 kg). The TSK broadcasting its own grade and mass estimate is what you
  would expect if it converts the accel request to wheel torque with grade compensation (inference).

## 1.6 Panda (opendbc safety, volkswagen_mqb.h)

- ACC_06/ACC_07/ACC_02 on bus 0 are `check_relay`: the radar's copies arriving on bus 2 are never
  forwarded, and any 0x12E received on bus 0 would trip relay malfunction. So the log holds two 0x12E
  streams: src 2 (radar, its own values, normally HMS 0) and src 128 (OP's TX echo).
- Accel outside [-3.5, 2.0], unless exactly 3.01, or any active accel while `controls_allowed` is false
  or `gas_pressed_prev`, blocks the whole frame (`pandaStates.safetyTxBlocked`). ACC_07 Folgebeschl
  must be 3.02. `controls_allowed` drops if TSK_Status leaves {2,3,4,5}.

## 1.7 Stale or incorrect comments in the current code

- "at_standstill = standstill OR vEgo < 0.5": the code uses 1.0.
- "CS.out.standstill = pcmCruise AND esp_hold_confirmation": it is `vEgoRaw == 0`.
- 4a "After 100 frames (~2s)" and the vEgo > 0.3 rolling exception: the code uses 30 frames and has
  no rolling check.
- "REQUESTING window <= 20 frames (400 ms)" is counted from lcs = stopping, not from standstill.
