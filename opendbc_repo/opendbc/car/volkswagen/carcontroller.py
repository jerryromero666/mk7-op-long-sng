import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mlbcan, mqbcan, pqcan
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# Decline brake (MQBStandstillManager). Real hold trace, drive 350 stop #2 at -12 % (Motor_16): the early stop
# announcement latched HMS=4 / ACC_07 Anfahren at 0.96 m/s while openpilot was still in pid asking -0.59 and ACC_06
# did not say Anhalten; 0.04 s later the TSK wheel brake torque went 150 -> 0 for 0.3 s and took 0.7 s to come back,
# and on the grade the car sped up 0.86 -> 1.32 m/s and needed 3 s / 2.5 m more to stop: "slow down, stop, roll,
# stop". On the flat the same release costs 0.2-0.5 mph (event 10); downhill gravity adds g * grade. On a decline
# (the TSK's own grade, Motor_16, at least DECLINE_FILE's threshold), from the HMS=4 latch on and while openpilot
# does not ask to accelerate, ACC_06 says Anhalten with openpilot's own request (never positive): the state every
# announced stop is in a moment later anyway (HMS=4 + ACC_07 Anfahren + ACC_06 Anhalten: the TSK follows the
# request, event 10 from -0.96 s), only entered at the latch. Never in the HMS=1 announcement window (ACC_06 Anhalten
# there dropped the TSK torque), never with the ESP's own hold (bit 53), ACC_07 / HMS untouched. Once openpilot
# pushed harder than DECLINE_PUSH it waits DECLINE_HOLDOFF before braking with Anhalten again (a lead lost and found
# in the last metre: with Anhalten the TSK comes down from the push at its 4 m/s3 gradient). A real launch (request
# above DECLINE_PUSH) gets today's frames. Live: /data/decline_stop (the planner's file): the decline threshold in %
# (default DECLINE_DEFAULT, clamped DECLINE_LIMITS), "0" = off = frames as before, bit for bit; read ~once a second.
DECLINE_FILE = '/data/decline_stop'
DECLINE_DEFAULT = 2.5        # % downhill
DECLINE_LIMITS = (1.0, 10.0)
DECLINE_HYST = 1.0           # %
DECLINE_PERIOD = 50          # standstill manager updates between file reads (1 s at 50 Hz)


def read_decline_threshold() -> float:
  try:
    with open(DECLINE_FILE) as f:
      vals = [float(v) for v in f.read().split()]
  except (OSError, ValueError):
    return DECLINE_DEFAULT
  if len(vals) != 1 or not np.isfinite(vals[0]):
    return DECLINE_DEFAULT
  if vals[0] <= 0.0:
    return 0.0
  return float(np.clip(vals[0], *DECLINE_LIMITS))


class HCAMitigation:
  """
  Manages HCA fault mitigations for VW/Audi EPS racks:
    * Reduces torque by 1 for a single frame after commanding the same torque value for too long
  """

  def __init__(self, CCP):
    self._max_same_torque_frames = CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP)
    self._same_torque_frames = 0

  def update(self, apply_torque, apply_torque_last):
    if apply_torque != 0 and apply_torque_last == apply_torque:
      self._same_torque_frames += 1
      if self._same_torque_frames > self._max_same_torque_frames:
        apply_torque -= (1, -1)[apply_torque < 0]
        self._same_torque_frames = 0
    else:
      self._same_torque_frames = 0

    return apply_torque


class MQBStandstillManager:
  """
  Standstill handling for MQB ACC type 1. Ported from RJ's implementation, adapted for
  sunnypilot and cars without ESP_15 (no EPB, no hold torque signal).

  Hold request state machine (ACC_07 HMS signal):
    The ESP only engages hill hold in response to HMS=1 (hold request). Previous versions
    sent HMS=4 (start) or HMS=0 immediately at standstill, preventing ESP from ever
    engaging hold — confirmed cause of rollback on hills (zero hold frames in drive logs).

    State machine per standstill event:
      1. REQUESTING (no hold yet, within timeout):
         No override → raw stopping=True → HMS=1 (hold request)
         ESP engages hold within 2-5 frames on flat/hill.
      2. HOLDING (hold confirmed):
         esp_starting_override=False → HMS=3 (standby, never fights active hold)
      3. RELEASING (hold was confirmed, now dropped — normal ESP cycling):
         esp_starting_override=True → HMS=4 (release, harmless noop without hold)
      4. TIMEOUT (no hold after HOLD_REQUEST_TIMEOUT frames ~400ms):
         Fall back to HMS=4 permanently for this stop. Prevents SRBM accumulation
         on stops where ESP never engages hold (e.g. very gentle grade, slow creep).

    SRBM safety: HMS=1 without hold only accumulates SRBM after hundreds of sustained
    frames. The requesting window is ≤20 frames (400ms) before either hold confirms
    or we fall back to HMS=4. Zero SRBM risk confirmed from prior drive log analysis.

  Sunnypilot vs RJ differences:
    - Removed CS.esp_hold_uphill (ESP_15 — crashes this car variant)
    - Removed CS.esp_hold_torque_nm / CS.actual_torque_nm (ESP_15)
    - Replaced I-controller with fixed UPHILL_ACCEL_FLOOR
    - Simplified standstill detection: at_standstill = standstill OR vEgo < 0.5
    - Removed CS.distance_button_pressed (debug helper)
    - Signatures: __init__(CCP) unchanged; CarController uses (CP_SP) and (CC_SP)
  """

  HOLD_REQUEST_TIMEOUT = 20    # frames of HMS=1 before fallback to HMS=4 (~400ms at 50Hz)
  # Early stop announcement (hill hold): announce every stop while the car still rolls, started by
  # speed instead of OP's stopping state, which starts late on hills (at or after the wheels stop,
  # so the parent's HMS=1 window fell at standstill with 0 TSK torque: 0.11 m rollback at +11.2%).
  # The TSK only owns the hold after HMS=4 if the ESP ran its ACC stopping procedure (bit 63) first;
  # without it HMS=4 at standstill faulted (hold_tsk_owned, routes 337/338). As in RJWadley's
  # standstill-workaround (MqbLongStateMachine): the announcement ends as soon as the ESP reports
  # bit 63, and during it ACC_06 says Anfahren (not Anhalten) so the TSK keeps its brake torque
  # instead of dropping it to 0. The announced stop latches HMS=4 only if bit 63 was seen; otherwise,
  # and for stops that could not be announced while rolling (re-engaging at standstill), the parent
  # state machine below runs unchanged. RJ's rollback guard is not used: it reads the ESP_10 wheel
  # direction, which is not valid on this car (half the codes are 'invalid', 'backward' while driving).
  # Route 33d (announcing at RJ's 2.6 m/s): once HMS=4 latched the car stopped slowing (2.59 -> 2.55
  # m/s in 0.4 s with -0.8 asked) and the driver had to brake on every approach. HMS=4 only brakes weakly
  # on this car, so it must latch at the parent's speed (it latched at 0.6-1.0 m/s and stopped fine).
  ANNOUNCE_SPEED = 0.9         # m/s: start the announcement below this while braking (parent latched at 0.6-1.0)
  ANNOUNCE_MIN_SPEED = 0.3     # m/s: never announce slower, so it always ends before the wheels stop
  ANNOUNCE_FRAMES = 20         # 0.4 s at 50 Hz: the window in front of every fault-free hold (332/333)
  # SET at a stop: openpilot engaged (preEnabled) with the car already stopped and the driver on the brake, so the
  # rolling announcement above can no longer happen. Without it the ESP grants only a short standstill and gives up
  # after ~3.5 s (route 355 22:20, +5.6 %: bit 29, TSK 3 -> 6, car rolled 1 m). The same announcement is sent here,
  # but only while the driver's brake holds the car (HMS 1 drops the TSK brake torque), and only once per stop.
  # It ends on bit 63 (announced hold, as on a rolling stop), after ANNOUNCE_FRAMES, or when the brake is released;
  # without bit 63 the stop continues exactly as before.
  # It starts only on the frame openpilot engages (never after HMS 4 went out: no 4 -> 1 at a standstill), so a brake
  # press at an engaged stop never starts it.
  STILL_V = 0.1                # m/s (vEgoStarting): the car counts as stopped below this
  RELEASE_SPEED = 1.5          # m/s: the announced stop ends once driving again; a new slowdown re-announces
  UPHILL_ACCEL_FLOOR = 1.5     # m/s² accel floor on launch from confirmed hold (any grade)
  DECLINE_PUSH = 0.3           # m/s2: a request above this is a launch / push: today's frames (see DECLINE_FILE)
  DECLINE_HOLDOFF = 25         # frames (0.5 s at 50 Hz) after a push before the decline brake comes back
  # At a standstill openpilot asking to go (above 0) for DECLINE_GO_FRAMES = a launch: today's frames from then on
  # (shorter go requests = chatter: held with Anhalten)
  DECLINE_GO_FRAMES = 15       # frames (0.3 s)
  DECLINE_STILL_V = 0.1        # m/s (vEgoStarting)

  def __init__(self, CCP):
    self._CCP = CCP
    self._prev_hold_confirmation = False
    self._hold_request_frames = 0
    self._hold_acquired = False  # latches True once hold confirms; prevents re-requesting
    self._hold_lost_frames = 0   # frames since hold last confirmed; resets _hold_acquired after timeout
    self._hms4_latched = False   # once HMS=4 at standstill, hold it steady (see update)
    self._announce_frames = 0    # HMS=1 frames sent in the early announcement of this stop
    self._announce_seen = False  # ESP bit 63 seen during that announcement
    self._stop_announced = False # this stop's announcement is over (until vEgo > RELEASE_SPEED)
    self._still_announce = False # the current announcement is the SET-at-a-stop one (see STILL_V)
    self._prev_hms_active = False
    self._announced_hold = False # announced and acknowledged: HMS=4 until vEgo > RELEASE_SPEED
    self._decl_threshold = DECLINE_DEFAULT
    self._decl_cnt = 0
    self._decline = False        # on a decline (Motor_16)
    self._decl_on = False        # decline brake latched in this announced hold
    self._decl_push = self.DECLINE_HOLDOFF   # frames since openpilot asked for more than DECLINE_PUSH
    self._decl_go = 0            # frames openpilot asked to go from a standstill (see DECLINE_GO_FRAMES)

  def _update_decline(self, CS, accel: float) -> None:
    if self._decl_cnt % DECLINE_PERIOD == 0:
      self._decl_threshold = read_decline_threshold()
    self._decl_cnt += 1
    self._decl_push = 0 if accel > self.DECLINE_PUSH else min(self._decl_push + 1, self.DECLINE_HOLDOFF)
    if self._decl_threshold <= 0.0 or not getattr(CS, 'tsk_grade_valid', False):
      self._decline = False
      return
    downhill = -float(CS.tsk_grade)
    if downhill >= self._decl_threshold:
      self._decline = True
    elif downhill < self._decl_threshold - DECLINE_HYST:
      self._decline = False

  def _decline_brake(self, accel: float, stopping: bool, v: float) -> float | None:
    """announced hold, HMS=4 latched: openpilot's request for ACC_06 with Anhalten, or None (today's bits)"""
    if not self._decline or self._decl_push < self.DECLINE_HOLDOFF:
      self._decl_on = False
      self._decl_go = 0
      return None
    # a launch from a standstill: openpilot asking to go for DECLINE_GO_FRAMES (a lead creeping off slowly asks less
    # than DECLINE_PUSH and would otherwise be held for as long as it creeps); go-pulses shorter than that stay held
    if accel > 0.0 and not stopping and (v < self.DECLINE_STILL_V or self._decl_go > 0):
      self._decl_go = min(self._decl_go + 1, self.DECLINE_GO_FRAMES)
    else:
      self._decl_go = 0
    if self._decl_go >= self.DECLINE_GO_FRAMES:
      self._decl_on = False
      return None
    if accel <= 0.0 or stopping:
      self._decl_on = True
    return min(accel, 0.0) if self._decl_on else None

  def update(self, CS, long_active: bool, accel: float, stopping: bool, starting: bool
             ) -> tuple[bool, float, bool, bool, bool | None, bool | None]:
    esp_starting_override: bool | None = None
    esp_stopping_override: bool | None = None

    # CS.out.standstill = pcmCruise AND esp_hold_confirmation, always False on a hill
    # before hold is acquired. Use vEgo threshold so the override engages as soon as
    # the car is physically stopped regardless of ESP hold state.
    at_standstill = CS.out.standstill or CS.out.vEgo < 1.0

    # acc type 1 is sensitive to control signals when brake is pressed (preEnabled).
    # hms_active keeps HMS state machine alive even when brakePressed kills long_active.
    # Without this, brakePressed on a grade causes HMS=0 -> SRBM -> fault.
    hms_active = long_active
    if CS.out.brakePressed:
      long_active = False



    # Launch accel floor: fixed, no grade branching, so behavior is identical on every
    # incline and decline. Builds engine torque before the ESP releases hold.
    # Replaces a grade-adaptive floor keyed on CS.out.aEgo, which is noise at standstill
    # (reads 0.00 on a 5.6% grade, swings +/-1.6 on flat). That floor could also reach
    # 2.5 m/s2, above the 2.0 panda limit, which blocks the whole ACC_06/ACC_07 frame.
    # If a grade-aware floor is wanted later, key it on the 0x65F grade, never aEgo.
    # Clamped to ACCEL_MAX so raising UPHILL_ACCEL_FLOOR can never trip panda safety.
    if long_active and accel > 0 and self._prev_hold_confirmation and at_standstill:
      accel = min(max(accel, self.UPHILL_ACCEL_FLOOR), self._CCP.ACCEL_MAX)

    # Early stop announcement (see the class constants).
    v = CS.out.vEgo
    self._update_decline(CS, accel)
    if not hms_active or v > self.RELEASE_SPEED:
      self._announce_frames = 0
      self._announce_seen = False
      self._stop_announced = False
      self._still_announce = False
      self._announced_hold = False
    held = hms_active and v < self.STILL_V and CS.out.brakePressed
    if held and not self._prev_hms_active and not self._stop_announced:
      self._still_announce = True       # SET at a stop with the brake held (see STILL_V)
    self._prev_hms_active = hms_active
    # while the driver's brake holds the car nothing can drive off, so a 'starting' plan does not skip it (route 355:
    # after SET at the stop the plan sat in pid under 0.5 m/s, i.e. 'starting', and HMS went straight to 4)
    if hms_active and not self._stop_announced and (not starting or held):
      if self._announce_frames > 0 or self._still_announce or \
         (self.ANNOUNCE_MIN_SPEED <= v < self.ANNOUNCE_SPEED and (accel < 0 or stopping)):
        if self._announce_frames > 0:   # bit 63 counts only as an answer to our HMS 1
          self._announce_seen = self._announce_seen or bool(getattr(CS, 'esp_stopping', False))
        if self._still_announce:
          end = v >= self.STILL_V or not CS.out.brakePressed
        else:
          end = v < self.ANNOUNCE_MIN_SPEED
        if self._announce_seen or self._announce_frames >= self.ANNOUNCE_FRAMES or end:
          self._stop_announced = True
          self._announced_hold = self._announce_seen
        else:
          self._announce_frames += 1
          self._prev_hold_confirmation = CS.esp_hold_confirmation
          # HMS 1, ACC_07 Anhalten, d 0.3; ACC_06 Anfahren so the TSK keeps brake torque (RJ)
          return long_active, accel, False, True, False, True
    # If the ESP confirms its own hold (bit 53), never answer with HMS=4: route 33d #3 (+16%), 338 F1:
    # bit 53 under HMS=4 with 0 TSK torque, dropped after ~1.5 s -> TSK fault. Route 33d #5: HMS=3 under
    # bit 53 -> the ESP handed over within 0.04 s, HMS=4 and the TSK held. The parent's State 2 does that.
    if hms_active and self._announced_hold and not CS.esp_hold_confirmation:
      self._prev_hold_confirmation = CS.esp_hold_confirmation
      decl = self._decline_brake(accel, stopping, v)
      if decl is not None:
        return long_active, decl, True, False, True, False              # decline brake: ACC_06 Anhalten (see DECLINE_FILE)
      return long_active, accel, stopping, starting, True, False        # HMS 4, ACC_07 Anfahren, d 20.46 (ACC_06: OP's bits)
    self._decl_on = False
    self._decl_go = 0

    # HMS=4 LATCH.
    # Drive data (0000022b, 5.6% grade) shows the ESP tolerates a long steady HMS=4 --
    # 60+ frames with ESC_TSK_SRBM_Anf=1 and no complaint. What it will NOT tolerate is
    # our state machine cycling 4 -> 1 -> 3 -> 4. Each time we send HMS=3 (standby) the
    # ESP reads it as "release", drops Anf, and re-engages when we go back to 4. After
    # three such round trips it sets ESC_TSK_SRBM_nicht_verfuegbar and TSK faults.
    # We move first in every cycle, so this is self-inflicted. Once we reach HMS=4 at
    # standstill, latch it: no branch may pull us to 1 or 3 until the car departs.
    if not at_standstill or starting:
      self._hms4_latched = False

    if hms_active and at_standstill and self._hms4_latched and not starting and not CS.esp_hold_confirmation:
      esp_starting_override = True
      esp_stopping_override = False
    elif hms_active and at_standstill and stopping and not starting:
      if CS.esp_hold_confirmation:
        # State 2: HOLDING — standby, never fight active hold → HMS=3
        esp_starting_override = False
        esp_stopping_override = False
        self._hold_request_frames = 0
        self._hold_acquired = True   # latch: prevents re-requesting on rapid-cycle grades
        self._hold_lost_frames = 0
      elif self._prev_hold_confirmation:
        # State 3: RELEASING — hold just dropped, signal release → HMS=4
        esp_starting_override = True
        esp_stopping_override = False
        self._hold_request_frames = 0
        self._hms4_latched = True   # stay at 4 rather than cycling back through 1/3
      elif self._hold_acquired:
        # State 4a: POST-ACQUISITION — hold was acquired but not currently confirmed
        # Count frames since hold last confirmed. After 100 frames (~2s), reset
        # _hold_acquired so a fresh REQUESTING cycle can re-engage hold.
        # Exception: if the car is still rolling (vEgo > 0.3 m/s) we're on a grade
        # too steep for hold to engage — don't reset, stay on HMS=4 permanently
        # until departure. Resetting causes repeated REQUESTING→TIMEOUT oscillation
        # on steep declines (confirmed: 3-cycle loop at -14.4% grade).
        self._hold_lost_frames += 1
        if self._hold_lost_frames >= 30:
          self._hold_acquired = False
          self._hold_lost_frames = 0
          self._hold_request_frames = 0
          # reset fully — allow fresh REQUESTING (HMS=1) cycle to re-establish hold
          # previously used HOLD_REQUEST_TIMEOUT which caused permanent HMS=4 → SRBM
        else:
          esp_starting_override = True
          esp_stopping_override = False
      elif self._hold_request_frames >= self.HOLD_REQUEST_TIMEOUT:
        # State 4b: TIMEOUT — hold never came, safe HMS=4 fallback
        esp_starting_override = True
        esp_stopping_override = False
        self._hms4_latched = True   # hold 4 steady from here; do not cycle back to 1/3
      else:
        # State 1: REQUESTING — no override, allow HMS=1 to request hold from ESP
        self._hold_request_frames += 1
        # esp_starting_override remains None → raw stopping=True → HMS=1
    elif hms_active and at_standstill and starting:
      # Departure: OP has transitioned to starting state — immediately signal HMS=4
      # regardless of hold state so ESP releases without waiting for hold to drop first
      esp_starting_override = True
      esp_stopping_override = False
      self._hold_request_frames = 0
    else:
      self._hold_request_frames = 0
      self._hold_acquired = False   # reset when leaving standstill
      self._hold_lost_frames = 0

    self._prev_hold_confirmation = CS.esp_hold_confirmation

    return long_active, accel, stopping, starting, esp_starting_override, esp_stopping_override


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.CCP = CarControllerParams(CP)
    self.CAN = CanBus(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.aeb_available = not CP.flags & VolkswagenFlags.PQ

    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MLB:
      self.CCS = mlbcan
    else:
      self.CCS = mqbcan

    self.apply_torque_last = 0
    self.gra_acc_counter_last = None
    self.hca_mitigation = HCAMitigation(self.CCP)
    self.standstill_manager = MQBStandstillManager(self.CCP)

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      apply_torque = 0
      if CC.latActive:
        new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)

      apply_torque = self.hca_mitigation.update(apply_torque, self.apply_torque_last)
      hca_enabled = apply_torque != 0
      self.apply_torque_last = apply_torque
      can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, apply_torque, hca_enabled))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = float(np.clip(apply_torque * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, self.CAN.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Acceleration Controls ******************************************** #

    if self.CP.openpilotLongitudinalControl:
      if self.frame % self.CCP.ACC_CONTROL_STEP == 0:
        long_active = CC.longActive
        accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if long_active else 0)
        stopping = actuators.longControlState == LongCtrlState.stopping
        starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)

        esp_starting_override = None
        esp_stopping_override = None
        if self.CCS == mqbcan and CS.acc_type == 1:
          long_active, accel, stopping, starting, esp_starting_override, esp_stopping_override = \
            self.standstill_manager.update(CS, CC.longActive, accel, stopping, starting)

        # Use CC.longActive for HMS signaling — returned long_active may be False from brakePressed
        # which would force HMS=0 and prevent hill hold from engaging via SRBM accumulation
        long_active_hms = CC.longActive if (self.CCS == mqbcan and CS.acc_type == 1) else long_active
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, long_active_hms)

        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, long_active_hms, accel,
                                                           acc_control, stopping, starting, CS.esp_hold_confirmation,
                                                           esp_starting_override, esp_stopping_override))

      #if self.aeb_available:
      #  if self.frame % self.CCP.AEB_CONTROL_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_control(self.packer_pt, False, False, 0.0))
      #  if self.frame % self.CCP.AEB_HUD_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_hud(self.packer_pt, False, False))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive,
                                                       CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      lead_distance = 0
      if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
        lead_distance = 512 if CS.upscale_lead_car_signal else 8
      acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
      # FIXME: PQ may need to use the on-the-wire mph/kmh toggle to fix rounding errors
      # FIXME: Detect clusters with vEgoCluster offsets and apply an identical vCruiseCluster offset
      set_speed = hud_control.setSpeed * CV.MS_TO_KPH
      can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, set_speed,
                                                       lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.CAN.ext, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
