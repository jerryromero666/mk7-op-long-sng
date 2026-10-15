# 3. External knowledge (facts marked F with source, inferences marked I)

Downloaded copies (untrusted data, read only) are in `ext/`: commaai_mqbcan.py, commaai_carcontroller.py,
commaai_interface.py, commaai_mebcan.py, rj_*.py (RJWadley/opendbc branch standstill-workaround),
iqinc_*.py (iqinc/iqpilot), iq_*.py (DSparks156x/iq-pilot-tt).

## Upstream commaai/opendbc (master, fetched 2026-10-14)
- F: MQB still sends HMS 4 if starting, elif hold 3, elif stopping 1, else 0, with ACC_07 Anhalten =
  stopping and Anhalteweg 0.3 while stopping. So upstream keeps Anhalten=1 alongside HMS 3.
  ACC_06 neg/pos jerk 4.0. (ext/commaai_mqbcan.py)
- F: `starting` now uses `vEgo < 0.25` (this build uses vEgoStopping 0.5). stopAccel -0.55.
  (ext/commaai_carcontroller.py:150, commaai_interface.py:139)
- F (MEB, same HMS enum on ACC_18): "HALTEN -> KEINE_ANFORDERUNG causes the car to fault into park";
  "ANFAHREN faults TSK if the hold manager isn't holding"; "HMS_Status leaves holding as soon as we send
  anfahren"; MEB keeps HALTEN for the whole stop and uses LOESEN_UEBER_RAMPE (5) to release.
  (ext/commaai_mebcan.py MebLongStateMachine). I: MQB is a different ESP, but this says ANFAHREN with
  no active hold is a known TSK fault trigger on VW.
- F: PR #3862 (MEB): the hold manager reached "holding" before ESC standstill in all 316 OP long stops
  across 4 cars, median 0.85 s earlier. I: a hold window counted from standstill would be too late on
  MEB. For MQB bit 53 the timing is unknown.
- F: PR #3633 (MEB): `HALTEN` holds with ESP hydraulics at standstill and "does nothing above a few
  km/h".

## RJ's implementation (RJWadley = the "RJ" in the docstring)
- commaai/opendbc PR #2764 "VW MQB: work around standstill limit" (open):
  F: "MQB cars without an EPB will fault if the ESP holds the brakes for too long at a standstill."
  F: "If we transition from stopping to starting before the car has reached a standstill, we can ask for
     brake indefinitely without consequence. The car will happily comply with braking requests even in a
     starting state."
  F: "we are prone to rollback on very steep hills. The car takes about a second to build braking force."
  F: comments: uphill needs a longer "start request" before the car acknowledges; on very steep hills
     "the request might be impossible to send as we don't have enough time before faulting"; RJ leans
     toward asking the driver to brake on steeper hills.
  F: jyoung8607: FtS and SnG cars differ in bidirectional rear wheel-speed sensors, used against hill-start
     rollback; he suspects the ESP holds until the powertrain overcomes the hill; he "wouldn't trust the
     car's road-grade signal" and suggests openpilot pitch.
  F: sshane questions supporting SnG on cars without EPB (door open -> car rolls).
- PR #3322 (closed): F: "In order to hold at standstill forever, we must use TSK braking at low speeds";
  "when using the TSK, we have no native rollback protection, unlike EPB cars"; "the MQB TSK will only
  ever command brake or gas, never both"; "brake force commanded by the TSK is jerk-limited and takes about
  a second to accumulate"; "releasing the brake is always instant"; "any attempt to balance the
  acceleration against gravity will earn the wrath of the check engine light".
- Code (ext/rj_mqbcan.py MqbLongStateMachine, same as iqinc/iqpilot):
  * Below 9.5 km/h send ACC_07 STOP (Anhalten, HMS 1) to start the ESP stopping procedure. When ESP_21
    bit 63 `ESP_Anhaltevorgang_ACC_aktiv` appears, latch ACC_07 START (HMS 4, Anfahren) for the rest of
    the stop ("can_stop_forever").
  * Keep ACC_06 in starting (Anfahren=1, Anhalten=0) "to prevent brake torque from zeroing itself out by
    keeping TSK in starting state". ACC_06 still carries negative accel, so the TSK brakes.
  * Rollback protection: pitch from `CC.orientationNED[1]`; hold deficit = g sin(pitch) - TSK brake
    torque/(m r); safe speed = 1.5 deficit^2 / (2000 Nm/s / (m r)); below it, blend accel toward
    ACCEL_MIN (-3.5). On ESP_10 Fahrtrichtung == 1 (backward) force ACCEL_MIN + stopping. At standstill
    never relax braking: `accel = min(accel, prev_accel)`.
  * Launch: a "start commit" with accel >= max(0.2, 0.1 x grade%), only if the plan's speed exceeds the
    safe speed (needs openpilot PR #37800 actuators.speed).
  * Neg jerk 4.0, starting threshold vEgo < 0.25, brakePressed suppresses long_active.
- I: this build borrowed RJ's names but kept the ESP-owned hold and added a 20-frame window and an
  HMS 4 latch, with ACC_06 still saying Anhalten. That is close to the opposite of RJ's split.

## Factory behaviour
- F: openpilot VW wiki: "ACC low" (any car with a manual parking brake) "will require the driver to take
  over and hold the brake after a very short delay" and "will not hold themselves at a stop"; "ACC high"
  needs an EPB and resumes automatically if the stop is under three seconds.
  https://github.com/commaai/openpilot/wiki/Volkswagen
- F (anecdotal, VW T6.1 with handbrake): ACC holds about 3 s, warns, then releases the brakes; RES
  presses extend the hold. https://www.t6forum.com/threads/t6-1-abs-fts-s-g-theory.37977/ (search
  snippet only; the page itself was not reachable).
- F: issue #2874: Golf R manual, alpha long, rapid throttle on/off "will occasionally cause a cruise
  control fault". I: a TSK fault source that has nothing to do with grade, relevant to the 3
  unexplained faults.
- Not found: a published maximum ESP hold time or grade limit for MQB FtS, and an expansion of "SRBM".
  The only time figures are the anecdotal 3 s factory release and RJ's "about a second" brake build.

## Physics (sim/physics.py; m = 1566 kg incl. 136 kg cargo, r = 0.328 m)
- Holding torque needed: 5%: 252 Nm; 5.6%: 282; 11%: 551; -14.4%: 718.
- If the TSK treats -1.2 as net accel and subtracts gravity (I: plausible, because it broadcasts its own
  grade and mass in 0x65F), the brake share is 1.2 - g sin(theta): 5% -> 0.71 m/s^2 (365 Nm, holds),
  11% -> 0.13 m/s^2 (65 Nm, does not hold). Crossover 6.1%. That matches "holds ~5%, rolls at 11%".
- If not grade-compensated, -1.2 gives 616 Nm: holds 11% with 12% margin, fails on -14.4% downhill.
- A grade-compensated -3.5 holds uphill to 18.1% and any downhill. Non-compensated -3.5 holds to about
  36%.
- 11% with 0.13 m/s^2 of brake: rollback accel 0.94 m/s^2, reaches 0.5 m/s in 0.53 s and 1.0 m/s in
  1.06 s (0.53 m).
- RJ safe speed at 11% with zero brake: 0.44 m/s, close to vEgoStopping 0.5. At 5%: 0.09 m/s.
