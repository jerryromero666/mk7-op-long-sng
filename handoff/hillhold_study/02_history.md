# 2. History of hold-related work on origin

Source: `git log --all -- opendbc_repo/opendbc/car/volkswagen/{carcontroller,mqbcan,carstate,interface,values}.py`
plus the named branches. "In HEAD" = ancestor of claude/determined-hypatia-pnidyz.

| Commit / branch | Date | In HEAD | What it did | Evidence cited |
|---|---|---|---|---|
| eabb2aa staging = SNG_YA_DIG | 2026-04-18 | yes (base) | Stock sunnypilot v2026.001.000: upstream VW long. stopAccel -0.55, HMS 1 (stopping) -> 3 (hold) -> 4 (starting), neg jerk 4.0, no standstill manager. | none |
| 1ba17f6 MK7YAHEARD | 2025-12-02 | no (older FrogPilot-era tree, root-level files) | "Standstill Reset Patch": a ResetSignal timer machine (GENTLE nudges at frames 5/10, EXTENDED 20-60, PRE/POST_DISABLE emulating ACC's own timeout) that briefly sends ACC_06 Anfahren=1/Anhalten=0 with accel 0.01/0.0 and ACC_07 HMS=0 to reset the standstill timer. Also swapped the HMS map (hold -> 1, stopping -> 3). | none. Implies the author believed a standstill timer exists and can be reset by start-like requests. |
| 89a239f _Mk7_95_Percenter_yahhuurd | 2026-06-05 | yes | Introduced MQBStandstillManager ("ported from RJ") without RJ's core (TSK-owned hold, rollback blending, ESP_21 bit 63 trigger). stopAccel -0.55 -> -1.2, longitudinalActuatorDelay 0.5, ACC_06 neg jerk 4.0 -> 1.0, stoppingDecelRate 0.8 -> 6.0 (global), interfaces ACCEL_MAX 2.5. ACC_07 overrides introduced. Original 4a reset jumped straight to TIMEOUT. | Docstring only: "zero hold frames in drive logs" when HMS 4/0 was sent at standstill (cause of rollback); "ESP engages hold within 2-5 frames on flat/hill"; "HMS=1 without hold only accumulates SRBM after hundreds of frames"; "3-cycle REQUESTING->TIMEOUT loop at -14.4%". No routes or counts. |
| 1438380 | 06-05 | yes (reverted by 1478030) | Grade-adaptive launch floor from -aEgo, up to 2.5. | none |
| 92b7fb4 | 06-06 | yes | HMS/acc_enabled use CC.longActive so brakePressed doesn't force HMS 0. | claim: HMS 0 from brake -> "SRBM accumulation" |
| ac1cf21 | 06-06 | reverted next day | Force stopping before hold acquired. | none |
| f30a396 | 06-07 | yes | 4a reset now returns to REQUESTING (HMS 1) instead of TIMEOUT. | claim: "permanent HMS=4 -> SRBM" |
| a9dbc58 + 8613283 srbm_grade_hold_fix | 08-02 | no | "Powertrain hold": on grade (abs(aEgo) > 0.3) suppress the HMS machine, send stopping=starting=False (HMS 0), hold with positive accel floor up to 2.5. | "ESP never confirms hold (esp_hold_confirmation always False in 45k+ frames)"; "HMS=0 resting state runs fault-free (11,461 grade frames, zero faults on clean drives)"; "taper positive accel -> pid/starting -> HMS 4 while stopping machine wants 1 -> HMS chatters 0<->4 at 50 Hz -> TSK=7". aEgo gating is noise (trap 2) and floor > 2.0 is panda-blocked. |
| 6bacc4d live_stop_distance | 08-08 | no | TIMEOUT and 4a send HMS 0 (both overrides False) instead of 4; never re-request. | "HMS 4->1, 13 SRBM frames, TSK=6"; "HMS=0 fault-free (11,461 grade frames)"; "both HMS=1 and HMS=4 accumulate SRBM at standstill and eventually trip TSK". |
| d861e6b hms_latch_fix | 08-10 | yes | Latch HMS 4 at standstill after RELEASING or TIMEOUT until departure or vEgo >= 1.0. | Route 0000022b, 5.6%: "ESP tolerates 60+ frames steady HMS=4 with ESC_TSK_SRBM_Anf=1"; "each HMS=3 is read as release, ESP drops Anf, re-engages on 4; after three 4->1->3->4 round trips it sets SRBM_nicht_verfuegbar and TSK faults". |
| 1478030 | 10-14 | yes (HEAD) | Fixed 1.5 m/s^2 launch floor, only after confirmed hold, clamped to 2.0. | aEgo is 0.00 on a 5.6% grade and +/-1.6 on flat. |
| f6d0edc fix_brake_ramp_4p0 | 10-14 | no | ACC_06 neg jerk 1.0 -> 4.0. | sim only; whether the ESP honours it is unverified |
| climb_taper_fix | - | ancestor of HEAD | No unique commits (tip 4eb1540). | - |

User notes after d861e6b: 812 HMS switches per 1000 grade-standstill frames, mostly 0 <-> 4; 3 TSK
faults on routes with zero grade-standstill frames.

## What the history says about this ESP (fact = cited log evidence, claim = asserted without data)

1. Hold confirmation is disputed. 89a239f claims bit 53 comes "within 2-5 frames" (claim). 8613283 says
   it was never seen in 45k+ frames (evidence, but the frame selection is not described). The trace has
   to settle this: if bit 53 is never 1, HOLDING and RELEASING are dead code and every stop is
   REQUESTING (<= 20 frames) -> TIMEOUT -> HMS 4 latched.
2. "SRBM accumulation" in older messages fits the user's verified meaning: bit 28 = the ESP is holding at
   standstill. "Accumulating SRBM" therefore means using up a finite ESP hold-time budget, after which
   bit 29 sets and TSK faults. This matches RJ: "MQB cars without an EPB will fault if the ESP holds the
   brakes for too long at a standstill".
3. HMS 4 does not make the ESP drop its hold immediately, at least on 5.6% (d861e6b: 60+ frames of
   bit 28 = 1 under steady HMS 4). Transitions are what it reacts to: sending HMS 3 (together with
   ACC_07 Anhalten 1 -> 0 in this code) dropped bit 28.
4. Re-requesting (4 -> 1) after a release makes TSK fault quickly (6bacc4d: TSK=6 after 13 SRBM frames).
5. 0 <-> 4 chatter at 50 Hz causes TSK=7 (8613283, trap 4).
6. Steady HMS 0 with ACC active is reported fault-free (11,461 grade frames), but on that branch the
   car was held by positive engine torque, and that logic keyed on aEgo, which is noise at standstill.
7. Nobody in the history measured brake pressure (ESP_05), TSK_Radbremsmom, wheel pulses or ESP_21
   bit 63 at a grade stop. Every earlier conclusion was drawn from HMS, bit 53, bit 28 and TSK_Status
   alone.
