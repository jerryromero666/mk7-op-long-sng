from cereal import car
from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import apply_driver_steer_torque_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.volkswagen import mqbcan, pqcan
from openpilot.selfdrive.car.volkswagen.mqbcan import ResetSignal
from openpilot.selfdrive.car.volkswagen.values import CANBUS, CarControllerParams, VolkswagenFlags

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.CCP = CarControllerParams(CP)
    self.CCS = pqcan if CP.flags & VolkswagenFlags.PQ else mqbcan
    self.packer_pt = CANPacker(dbc_name)
    self.ext_bus = CANBUS.pt if CP.networkLocation == car.CarParams.NetworkLocation.fwdCamera else CANBUS.cam

    self.apply_steer_last = 0
    self.gra_acc_counter_last = None
    self.frame = 0
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0
    self.frames_at_standstill = 0

  def _compute_mqb_standstill_reset_signal(self, CS, longActive):
    # Standstill reset state machine (approx. ACC_CONTROL_STEP * DT_CTRL Hz)
    # We:
    #   - try gentle resets early
    #   - then extended resets
    #   - then emulate ACC's own pre/post-disable behavior to avoid faults
    GENTLE_TICKS = (5, 10)
    EXTENDED_START = 20
    DISABLE_AT = 60
    PRE_DISABLE_DURATION = 3
    PRE_DISABLE_END = DISABLE_AT + PRE_DISABLE_DURATION - 1

    # Defensive access to fields (play nice with future changes)
    acc_type = getattr(CS, "acc_type", 0)
    esp_hold = getattr(CS, "esp_hold_confirmation", False)
    esp_vEgo = getattr(CS, "esp_vEgo_confirmation", 0.0)

    cs_out = getattr(CS, "out", None)
    standstill = getattr(cs_out, "standstill", False) if cs_out is not None else False

    reset_signal = ResetSignal.NONE

    # Only MQB ACC type 1 gets this behavior
    resettable_car = not (self.CP.flags & VolkswagenFlags.PQ) and acc_type == 1
    in_window = longActive and resettable_car and (esp_hold or standstill)

    if in_window:
      self.frames_at_standstill = min(self.frames_at_standstill + 1, PRE_DISABLE_END + 1)
      frames = self.frames_at_standstill

      # Gentle reset twice – comfortable, may fail on steep hills
      if frames in GENTLE_TICKS:
        reset_signal = ResetSignal.GENTLE_RESET

      # Extended reset – hold timer reset while not actually rolling
      elif EXTENDED_START <= frames < DISABLE_AT:
        reset_signal = ResetSignal.EXTENDED_RESET

      # Fault prevention region
      if frames < DISABLE_AT and not esp_hold:
        # Hold dropped early, reset the timer
        self.frames_at_standstill = 0
      else:
        if DISABLE_AT <= frames <= PRE_DISABLE_END:
          reset_signal = ResetSignal.PRE_DISABLE
        elif frames > PRE_DISABLE_END:
          reset_signal = ResetSignal.POST_DISABLE
    else:
      # Not in the standstill reset window
      self.frames_at_standstill = 0

    # If esp-confirmed vEgo > 0 with no hold, definitely reset
    if esp_vEgo > 0.0 and not esp_hold:
      self.frames_at_standstill = 0

    return reset_signal

  def update(self, CC, CS, now_nanos, frogpilot_toggles):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      # Logic to avoid HCA state 4 "refused":
      #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
      #   * Don't steer at standstill
      #   * Don't send > 3.00 Newton-meters torque
      #   * Don't send the same torque for > 6 seconds
      #   * Don't send uninterrupted steering for > 360 seconds
      # MQB racks reset the uninterrupted steering timer after a single frame
      # of HCA disabled; this is done whenever output happens to be zero.

      if CC.latActive:
        new_steer = int(round(actuators.steer * self.CCP.STEER_MAX))
        apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.CCP)
        self.hca_frame_timer_running += self.CCP.STEER_STEP
        if self.apply_steer_last == apply_steer:
          self.hca_frame_same_torque += self.CCP.STEER_STEP
          if self.hca_frame_same_torque > self.CCP.STEER_TIME_STUCK_TORQUE / DT_CTRL:
            apply_steer -= (1, -1)[apply_steer < 0]
            self.hca_frame_same_torque = 0
        else:
          self.hca_frame_same_torque = 0
        hca_enabled = abs(apply_steer) > 0
      else:
        hca_enabled = False
        apply_steer = 0

      if not hca_enabled:
        self.hca_frame_timer_running = 0

      self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
      self.apply_steer_last = apply_steer
      can_sends.append(self.CCS.create_steering_control(self.packer_pt, CANBUS.pt, apply_steer, hca_enabled))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = clip(apply_steer * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX)
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, CANBUS.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Acceleration Controls ******************************************** #

    if self.frame % self.CCP.ACC_CONTROL_STEP == 0 and self.CP.openpilotLongitudinalControl:
      # ACC is picky about what we send while brake is pressed
      longActive = CC.longActive and not CS.out.brakePressed

      acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, longActive)
      accel = clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if longActive else 0.0

      stopping = longActive and actuators.longControlState == LongCtrlState.stopping
      rollout_in_progress = CS.out.vEgo < frogpilot_toggles.vEgoStopping and accel > 0.0
      starting = longActive and actuators.longControlState == LongCtrlState.pid and (
        CS.esp_hold_confirmation or rollout_in_progress
      )

      if self.CP.flags & VolkswagenFlags.PQ:
        # PQ path: stock behavior, no standstill reset
        can_sends.extend(self.CCS.create_acc_accel_control(
          self.packer_pt, CANBUS.pt, CS.acc_type, longActive, accel,
          acc_control, stopping, starting, CS.esp_hold_confirmation
        ))
      else:
        # MQB standstill reset path
        reset_signal = self._compute_mqb_standstill_reset_signal(CS, longActive)
        can_sends.extend(self.CCS.create_acc_accel_control(
          self.packer_pt, CANBUS.pt, CS.acc_type, longActive, accel,
          acc_control, stopping, starting, CS.esp_hold_confirmation, reset_signal
        ))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, CANBUS.pt, CS.ldw_stock_values, CC.latActive,
                                                       CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      lead_distance = 0
      if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
        lead_distance = 512 if CS.upscale_lead_car_signal else 8
      acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
      # FIXME: follow the recent displayed-speed updates, also use mph_kmh toggle to fix display rounding problem?
      set_speed = hud_control.setSpeed * CV.MS_TO_KPH
      can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, CANBUS.pt, acc_hud_status, set_speed,
                                                       lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.ext_bus, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    new_actuators = actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.CCP.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
