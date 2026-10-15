# 4. Hypotheses for "holds on ~5%, rolls back on 11%" (ranked)

Framing: something has to hold the car at standstill. In this build that is either (a) the ESP's
standstill hold (ESP_21 bit 53 / ESP_33 bit 28), which is time-limited on a no-EPB car, or (b) the TSK's
brake torque from OP's -1.2 accel request, plus DSG creep. A rollback on 11% needs (a) to be absent or
gone AND (b) to be too weak. H1-H3 cover (a), H4-H5 cover (b), H6-H9 are amplifiers and confounders.
Priors are rough, for picking what to look at first.

"src128" = OP's own ACC frames as echoed in rlog `can` (bus 0 + 128). Times are 50 Hz ACC frames
unless stated.

## H1 (prior ~30%). OP never lets the ESP hold get established, then tells it to drive off
Mechanism: the 20-frame HMS 1 window starts at lcs = stopping (plan v_now < 0.5, wheels still turning)
and mostly expires before or right at vEgoRaw == 0. TIMEOUT then latches HMS 4 with ACC_07 Anhalten=0
and Anfahren=1 for the rest of the stop, while ACC_06 keeps Anhalten=1. If bit 53 does come, HOLDING
immediately withdraws ACC_07 Anhalten and sets Anhalteweg to 20.46 (upstream keeps Anhalten=1). The ESP
probably reads that as release, and RELEASING then latches HMS 4 as well. Either way the ESP is not
holding for long on a steep stop. Per RJ, uphill needs longer requests before the car acknowledges.
- Confirms: src128 HMS = 1 for <= 20 frames starting at the lcs -> stopping edge, and HMS -> 4 at or
  before the first frame with all four ESP_19 wheel speeds = 0. ESP_21 bit 53 stays 0, or is 1 only
  between the Anhalten 1 -> 0 edge and its own drop a few frames later. ESP_21 bit 63 is 1 during
  HMS 1 and falls within a few frames of HMS -> 4. ESP_05 Bremsdruck decays after that. ESP_10 pulse
  counters start moving 0.2-1.5 s later while ACC accel is still -1.2. On the 5% stop the CAN pattern
  is the same but no pulses move (the fallback holds there, see H4/H5).
- Kills: bit 53 or bit 28 = 1 and steady when the rollback starts (then see H3); or >= 1 s of HMS 1 at
  true standstill before the latch.
- Candidate fix: (i) count HOLD_REQUEST_TIMEOUT only while CS.out.standstill (vEgoRaw == 0) and make
  it >= 50 frames; (ii) in HOLDING keep ACC_07 Anhalten=1 and Anhalteweg=0.3 with HMS 3 (upstream).
  Neither gives an unlimited hold on a no-EPB car, so pair it with the H4/H5 fix.
- Risks: a longer HMS 1 without confirmation is what 6bacc4d blamed for TSK=6 (4 -> 1 after a release,
  13 frames). A longer ESP hold uses up its time budget, then bit 29 sets and TSK faults (d861e6b).
  Do not reintroduce 1/3/4 cycling.

## H2 (prior ~12%). The ESP hold engages but its time or pressure limit runs out
Mechanism: on a car without an EPB the ESP hold is a finite hydraulic hold (factory "very short delay",
about 3 s on a T6.1; RJ: "will fault if the ESP holds too long"). On 11% the hold pressure is higher, so
the budget may run out sooner, or the hold pressure from the ESP's grade estimate may slip.
- Confirms: bit 53 and/or bit 28 = 1 for T seconds, then drop with no change in OP's HMS, Anhalten or
  accel in the previous >= 10 frames. Bit 29 rises at or after the drop. T is repeatable and shorter on
  11% than on 5%. For slip: bit 53 = 1 while ESP_10 pulses increment and Bremsdruck is flat.
- Kills: every drop follows an OP-side edge within 1-2 frames (that is H1).
- Candidate fix: stop depending on the ESP hold (RJ, see H4/H5 fix). Time-limited holds can't be
  extended from OP; the RES-spam timer reset (MK7YAHEARD) is untested here and is exactly the 1/3/4
  cycling the history links to faults.
- Risks: as H4/H5.

## H3 (prior ~7%). The ESP had already given up (bit 29 set) before the 11% stop
Mechanism: earlier cycling in the same drive (or the stop's own 1/3/4 transitions) set
ESC_TSK_SRBM_nicht_verfuegbar, which OP cannot clear, so no ESP hold on this stop.
- Confirms: ESP_33 bit 29 = 1 at the first standstill frame of the 11% stop, with its first rising edge
  earlier in the drive. Bit 28 never rises during the stop. Possibly TSK_Status 6 nearby.
- Kills: bit 29 = 0 until after the rollback starts.
- Candidate fix: same as H4/H5 (the TSK hold does not need the ESP hold). Log the bit-29 edge and its
  preceding 2 s of HMS to find the trigger.
- Risks: none extra.

## H4 (prior ~20%). OP's -1.2 standstill request is too weak at 11% once the ESP hold is gone
Mechanism: comparing 1.2 with g sin(atan 0.11) = 1.07 is the wrong test. The request is a target
acceleration. If the TSK subtracts gravity using its own grade and mass (it broadcasts both in 0x65F),
the brake share is 1.2 - 1.07 = 0.13 m/s^2: about 65 Nm against the 551 Nm needed. At 5% the share is
0.71 m/s^2 (365 Nm against 252 needed), so it holds. Crossover is about 6.1%. Rolling back also
"satisfies" a -1.2 target (gravity alone gives -0.94), so a closed-loop accel controller sees little
error. The 1.0 m/s^3 neg jerk limit sent in ACC_06 slows any brake build further.
- Confirms: with bit 53 = 0 at standstill, TSK_06 TSK_Radbremsmom tracks m(1.2 - g sin theta) r: about
  365 Nm (raw ~46) on 5% and ~65 Nm (raw ~8) on 11%, with TSK_Freig_Verzoeg_Anf = 1 and ESP_05 bit 27
  = 1. ESP_05 Bremsdruck scales the same way. Motor_16 TSK_Steigung reads about +11 / +5.
- Kills: TSK_Radbremsmom is about 616 Nm (raw ~77) whatever the grade (not compensated). Then -1.2
  should hold 11%, so look at H5.
- Candidate fix (grade-agnostic): once stopped and the plan wants to stay stopped, request ACCEL_MIN
  (-3.5) and never relax it until OP wants to go (RJ: `accel = min(accel, prev_accel)` at standstill).
  That holds to about 18% uphill if compensated (36% if not) and any downhill, with no grade branching.
  Raise ACC_neg_Sollbeschl_Grad_02 to 4.0 (branch fix_brake_ramp_4p0) so it builds in under 1 s.
- Risks: needs the TSK to actually be braking (H5); launch must release from -3.5 cleanly (brake
  release is instant per RJ; on 11% the car can roll back before engine torque builds, so it needs
  RJ's start-commit or the 1.5-2.0 floor); unknown whether a long TSK brake at standstill trips anything
  on this ESP (RJ says no limit in TSK starting state); panda min is -3.5, so no block. It must not touch
  longControlState, or trap 4 comes back.

## H5 (prior ~15%). ACC_06/ACC_07 split brain: nobody owns the hold after the latch
Mechanism: after RELEASING or TIMEOUT, ACC_06 says Anhalten=1 (stop; the TSK hands the stop to the
ESP's stopping procedure) while ACC_07 says HMS 4 + Anfahren (the ESP is told to drive off). RJ's
comment says a TSK left in the stopping state lets its brake torque zero itself out, which is why he
does the opposite split. Result: no ESP hold and no TSK brake. 5% would then be held only by DSG creep
and residual pressure.
- Confirms: within a few frames of the HMS -> 4 latch, TSK_Radbremsmom -> 0 and/or
  TSK_Freig_Verzoeg_Anf -> 0 and ESP_05 bit 27 -> 0 while src128 ACC_06 accel = -1.2 and Anhalten = 1.
  ESP_05 Bremsdruck goes to ~0. On the 5% stop Motor_11 MO_Mom_Ist_Summe shows creep torque and the
  ESP_10 pulses stay still. ESP_33 bit 38/45 or ESP_05 bit 29 (consistency flags) may toggle at the
  latch.
- Kills: TSK_Radbremsmom stays at its pre-latch level after HMS -> 4.
- Candidate fix: make the latched state consistent, RJ style. ACC_06 Anfahren=1, Anhalten=0 (TSK in
  starting, keeps braking), ACC_07 HMS 4/Anfahren steady, accel as in H4. Drive these bits from a
  latched standstill owner, not from lcs. Ideally enter it before standstill (STOP until ESP_21 bit 63
  appears, then START), so the ESP's timed hold never starts.
- Risks: trap 4. If ACC_06 Anfahren follows lcs flips it chatters at 50 Hz and causes TSK=7, so it must
  be latched. Not verified on this car that the TSK accepts Anfahren with negative accel (RJ reports it
  does on MQB FtS). MEB comment "ANFAHREN faults TSK if the hold manager isn't holding" means this needs a
  careful first test on flat ground with a driver's foot ready. The consistency flags must stay quiet.
  No EPB: when OP disengages (driver brakes then lifts, door opens) nothing holds the car; the factory
  car also releases, but this is a safety point (sshane).

## H6 (prior ~5% as root cause; high as amplifier). Rollback flips the state machine into "drive off"
Mechanism: vEgo is unsigned. During rollback below 1.0 m/s the latch keeps HMS 4 (drive off). At
>= 1.0 m/s the latch and counters clear: HMS 1 if the plan still says stop. If the ESP catches the car,
20 frames later TIMEOUT re-latches HMS 4, giving a roll-catch-release loop (sim C2). If v_now rises past
0.5, should_stop drops, lcs -> pid, `starting` = hold bit (vEgo >= 0.5), so HMS is 0 or 4 on alternate
frames when the hold bit flickers (sim D, strict 0 <-> 4 at 50 Hz).
- Confirms: src128 HMS = 4 while ESP_19 speeds > 0 and ESP_10 pulses climb with lcs = stopping; HMS 1
  episodes starting exactly when carState.vEgo crosses 1.0; 0 <-> 4 alternation only in frames with
  lcs = pid and vEgo >= 0.5, in phase with ESP_21 bit 53.
- Kills: 0 <-> 4 frames with vEgo < 0.5 and CC.longActive true (impossible in this code; that would
  point to H7 or a different binary on the device).
- Candidate fix: once stopped with the plan still wanting stop, any wheel motion is rollback. Keep the
  latch, command ACCEL_MIN, never re-enter HMS 1 within the stop. Use ESP_10 direction only if it is
  valid on this car (jyoung8607: FtS may lack bidirectional sensors); otherwise use pulses plus the
  "stopped and plan says stop" latch. Use vEgo < 0.25 (upstream) for `starting`.
- Risks: forward creep into the lead is also "motion", but braking hard is correct in both directions
  while the plan wants to be stopped. The latch must still release cleanly on a real departure, or
  launches will drag.

## H7 (evidence quality, prior ~45% that the 812/1000 figure is partly artifact). Two 0x12E streams mixed
Mechanism: rlog `can` holds the radar's own ACC_07 on src 2 (normally HMS 0, because the radar isn't
controlling) and OP's echo on src 128. Decoding 0x12E without filtering by src interleaves them at 50 Hz
each, which gives about 0.8-1.0 switches per frame, "mostly 0 <-> 4", whenever OP is at a steady HMS 4.
In the branch code, HMS 0 next to HMS 4 at true standstill is impossible while long is active (see H6).
- Confirms: per-src counts. src 2 HMS constant 0, src 128 steady 4 over the same frames.
- Kills: src 128 alone alternates 0/4 (then H6 or CC.longActive toggling).
- Fix: analysis only. Decode OP frames from src 128 (or `sendcan`), radar frames from src 2.
- Risks: none to the car. But if it is an artifact, the conclusion that "the latch didn't work" is wrong,
  and the latch may be behaving as designed.

## H8 (prior ~6%). Pre-stop drive-off window
Mechanism: with vEgoStopping 0.5 used for `starting`, any frame with lcs = pid and vEgo < 0.5 sends
HMS 4 + Anfahren on both ACC_06 and ACC_07 while the car is still braking to a stop. On an uphill the
car decelerates faster than the plan (gravity), so the plan's v_now lags and these frames are more
likely. That can abort the ESP stopping procedure (bit 63) just before the stop. Sim: 5 frames with a
5-frame plan lag.
- Confirms: frames right before standstill with controlsState.longControlState = pid, carState.vEgo
  < 0.5, src128 HMS 4 and ACC_06 Anfahren 1. More of them on the 11% stop than the 5% stop. Bit 63
  falls on them.
- Kills: zero such frames on the 11% stop.
- Candidate fix: `starting` threshold 0.25 (upstream) and/or require `not longitudinalPlan.shouldStop`.
- Risks: slightly later departure signal. It doesn't fix rollback by itself.

## H9 (prior ~5%). Driver or pedal interaction
Mechanism: a brake tap or gas touch at the stop disengages OP long. ACC_06 goes to status 2 with accel
3.01, all bits 0, HMS 0/3. When the driver lifts off the brake there is no ACC stop owner and the car
rolls (factory FtS does the same). The panda can also block one or two frames during the disengage race.
Rapid throttle on/off is a known MQB alpha-long TSK fault source (issue #2874), which may explain the
3 faults on routes with no grade standstill.
- Confirms: carState.brakePressed / Motor_14 bit 28 / ESP_05 bit 26 or carState.gasPressed edges in the
  stop; carControl.longActive false; src128 ACC_06 status 2 and accel 3.01;
  pandaStates.controlsAllowed false; safetyTxBlocked increments; rollback starts after the pedal is
  released.
- Kills: no pedal edges, longActive true throughout.
- Fix: none in OP for the driver's own disengage (it is a driver-hold situation). Optionally keep the TSK
  brake and show a "press brake" alert for ~3 s before giving up (RJ).
- Risks: none extra.

## Combined most-likely story (to be confirmed by the trace)
On 11% the car stops mostly on gravity, so there is little brake torque at the moment of standstill. The
ESP hold window is too short or is withdrawn (H1), and/or the ESP hold is time-limited (H2/H3). From then
on the only brake is the TSK's share of a -1.2 target, roughly 0.13 m/s^2 at 11% (H4), or nothing if
ACC_06 Anhalten has put the TSK into the stopping procedure (H5). The car rolls. Below 1.0 m/s OP keeps
telling the ESP "drive off", and above it starts a roll-catch-release loop (H6). On 5% the same CAN
sequence is masked because the TSK share (0.71 vs 0.49 m/s^2) and/or creep is enough. A fix that holds on
any grade without grade branching is RJ's model. The TSK owns standstill (ACC_06 starting, ACC_07 START,
latched, entered before standstill so the ESP timer never starts), with ACCEL_MIN held at standstill, a
4.0 m/s^3 brake ramp and rollback detection. Every one of these steps must first be checked against the
trace signals in 05.
