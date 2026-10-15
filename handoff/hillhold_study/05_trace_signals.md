# 5. What the 50 Hz trace must show

Window: from 3 s before the lcs -> stopping edge to 5 s after the rollback starts (or the departure).
Take one 11% stop that rolled and one ~5% stop that held from the same drive, and line them up on the
first frame with all four ESP_19 wheel speeds = 0. Bit numbers are DBC start bits (little-endian, LSB
of byte 0 = bit 0). Decode OP's frames from rlog `can` src 128 (or `sendcan`) and the car's from src 0.
Print src 2 0x12E/0x122 separately (radar), never mixed.

## A. What OP sent (src 128 / sendcan)
| Msg | Signal (bits, scale) | Separates |
|---|---|---|
| 0x12E ACC_07 | ACC_Anforderung_HMS (28-30) | all |
| 0x12E | ACC_Anhalten (23), ACC_Anfahren (31), ACC_Anhalteweg (12-22, x0.01 m) | H1 (withdrawal at HOLDING), H5, H6, H8 |
| 0x12E | ACC_Sollbeschleunigung_02 (53-63, x0.005 - 7.22) | H4 |
| 0x122 ACC_06 | ACC_Anhalten (57), ACC_Anfahren (56), ACC_Status_ACC (60-62), ACC_Sollbeschleunigung_02 (24-34) | H5 (split), H8, H9 |
| 0x122 | ACC_neg_Sollbeschl_Grad_02 (40-47, x0.05) | H4 brake build |
| 0x12E src 2 vs src 128 | HMS per source; switch counts per source | H7 |

## B. What the ESP / TSK answered (src 0)
| Msg | Signal (bits, scale) | Separates |
|---|---|---|
| 0xFD ESP_21 | ESP_Haltebestaetigung (53) | H1 vs H2; settles "never confirms" vs "2-5 frames" |
| 0xFD | ESP_Anhaltevorgang_ACC_aktiv (63) | H1, H8, entry point for an RJ-style fix |
| 0x1AB ESP_33 | ESC_TSK_SRBM_Anf (28), ESC_TSK_SRBM_nicht_verfuegbar (29) | H2, H3 (time from 28 rising to 29 rising) |
| 0x1AB | ESC_Konsistenz_ACC (38), ESC_Konsistenz_TSK (45), ESC_Verz_Reg_aktiv (30-33) | H5 (does the split trip consistency checks) |
| 0x120 TSK_06 | TSK_Status (24-26) | fault timing (6 temp, 7 perm) |
| 0x120 | TSK_Radbremsmom (12-23, x8 Nm), TSK_Freig_Verzoeg_Anf (30), TSK_Standby_Anf_ESP (28), TSK_SRBM_Anf_ASIL (44-46) | H4 (grade-compensated or not), H5 (zeroed after latch) |
| 0x106 ESP_05 | ESP_Bremsdruck (16-25, x0.3 - 30 bar) | who is braking, and how hard |
| 0x106 | ESP_Verz_TSK_aktiv (27), ESP_Konsistenz_TSK (29), ESP_Fahrer_bremst (26), ESP_Rueckwaertsfahrt_erkannt (39), ESP_Status_Anfahrhilfe (40) | H4/H5, H9, rollback detection |
| 0x116 ESP_10 | ESP_Wegimpuls_VL/VR/HL/HR (16/26/36/46, 10-bit counters), ESP_xx_Fahrtrichtung (56/58/60/62, 2-bit) | exact rollback onset (before wheel speed leaves 0); whether direction is valid on this FtS car |
| 0xB2 ESP_19 | four wheel speeds (x0.0075 km/h) | vEgoRaw == 0 frame (standstill anchor) |
| 0x65F Motor_16 | TSK_Steigung (56-63, x0.8 - 101.6 %), TSK_QBit_Steigung (12), TSK_Fahrzeugmasse_02 (48-55, x32 kg) | grade of each stop; mass for the torque check |
| 0xA7 Motor_11 | MO_Mom_Ist_Summe (22-31, -509 offset Nm) | DSG creep holding the 5% stop (H5) |
| 0x3BE Motor_14 / 0x121 Motor_20 | MO_Fahrer_bremst (28) / MO_Fahrpedalrohwert_01 (12-19) | H9 |
| 0xAD Getriebe_11 | GE_Fahrstufe (42-45) | stays in D |

## C. openpilot side (cereal)
- controlsState.longControlState and carControl.actuators.longControlState (one frame apart).
- carControl.longActive, carControl.actuators.accel, carControl.orientationNED[1] (pitch, a second grade
  source that is independent of 0x65F).
- longitudinalPlan.shouldStop, aTarget, speeds[0] (plan v_now). This shows how far the plan lags the
  wheels (H8) and when rollback flips should_stop (H6).
- carState.vEgo, vEgoRaw, standstill, brakePressed, gasPressed (aEgo only to show it is noise).
- pandaStates.controlsAllowed, controlsAllowedLongitudinal, safetyTxBlocked (blocked ACC frames ->
  TSK faults).
- onroadEvents around the stop (pedalPressed, overrides).

## D. Decision table (what kills what)
1. Is bit 53 ever 1 on any stop? If never, H1's HOLDING path is dead and every stop is REQUESTING ->
   TIMEOUT -> latch, which makes H4/H5 the whole story.
2. When the car starts rolling (first ESP_10 pulse change after standstill), what is the state of
   bits 53, 28, 29, 63; HMS; ACC_06 Anhalten/Anfahren; TSK_Radbremsmom; Bremsdruck?
   - bit 53/28 dropped with OP steady before it -> H2. Dropped right after an OP edge -> H1.
     29 already 1 -> H3.
   - Radbremsmom about 8 raw at 11% and about 46 raw at 5% -> H4 (compensated). About 0 after the latch
     -> H5. About 77 everywhere -> neither, look at Bremsdruck and slip.
3. Count frames with pid, vEgo < 0.5 and HMS 4 before standstill -> H8.
4. During rollback, HMS by vEgo band (< 1.0 / >= 1.0) and lcs -> H6.
5. Switch counts per src -> H7.
6. For each TSK_Status 6/7 edge (including the 3 on non-grade routes): the 2 s before it of HMS (src
   128), ACC_06 bits, lcs, gas/brake edges, safetyTxBlocked, bits 29/38/45 -> classify as chatter
   (trap 4), panda block, pedal toggling (#2874), or ESP give-up.
