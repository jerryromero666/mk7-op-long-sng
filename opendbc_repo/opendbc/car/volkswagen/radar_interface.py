"""
MQB front radar lead interface.

The MQB ACC radar (Bosch MRR; in the lower grille on the Mk7, behind the front emblem from the
Mk7.5 on) does not put raw tracks on any bus the
comma can reach. What it does broadcast, on the extended CAN next to its ACC control messages,
is the distance to the object it has selected as the in-path lead: ACC_04.ACC_Abstand_Abstandswarner,
whole metres, produced by Front Assist whether or not the radar's own ACC is engaged. That is
enough to give openpilot longitudinal a radar-ranged lead while openpilot keeps authoring
ACC_06/ACC_07, so the standstill/HMS handling in carcontroller.py is untouched.

One signal means one track, with two things to work around:
  * No relative speed is broadcast. It is estimated here by a small Kalman filter over the
    distance, using our own wheel speed as a known input so that only the lead's speed has to
    be estimated (it changes far more slowly than the relative speed does).
  * 1 m resolution. A reading of N only says the lead is somewhere in [N-0.5, N+0.5], so the
    filter treats a prediction inside that bin as "no new information" instead of chasing the
    staircase, and corrects toward the nearest bin edge once the prediction leaves it.

This only reads CAN and publishes the track in liveTracks (so it is in every log). Whether the
track is used is decided in radard: off without /data/radar_lead, and even with it only after the
signal has proven itself against the camera on that drive (selfdrive/controls/radard.py,
RangeRadarFusion). CarParams are not changed: radarUnavailable stays set.
"""
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.volkswagen.values import DBC, CanBus, NetworkLocation, VolkswagenFlags

DISTANCE_MAX_VALID = 508       # 509 = Freifahrt (no lead), 510 = undefined, 511 = no display
PUBLISH_DECIMATION = 5         # card runs at 100 Hz, liveTracks is a 20 Hz service
SIGNAL_TIMEOUT = 0.5           # s without a fresh ACC_04 before the track is dropped


class LeadDistanceTracker:
  """Estimates lead distance and lead speed from a 1 m quantized range signal and ego speed."""

  BIN_HALF_WIDTH = 0.5         # m, half of the signal resolution
  LEAD_ACCEL_STD = 3.0         # m/s^2, how hard we allow the lead to change speed
  IN_BIN_STD = 0.3             # m, uncertainty of a prediction that agrees with the reading
  IN_BIN_CORR_TIME = 0.5       # s, repeated identical readings are one observation, not many
  EDGE_STD = 0.1               # m, uncertainty of a bin-edge correction
  EDGE_CORR_TIME = 0.1         # s
  JUMP_THRESHOLD = 2.0         # m beyond the bin edge: radar switched to a different object
  INIT_SPEED_STD = 5.0         # m/s, initial uncertainty of lead speed relative to ego
  READY_SPEED_STD = 1.0        # m/s, publish only once lead speed is known this well
  READY_MIN_TIME = 0.5         # s, and the object has been held at least this long

  def __init__(self):
    self.track_id = 0
    self.reset()

  def reset(self) -> None:
    self.tracking = False
    self.d = 0.0
    self.v_lead = 0.0
    self.p_dd = 0.0
    self.p_dv = 0.0
    self.p_vv = 0.0
    self.age = 0.0
    self.since_meas = 0.0

  @property
  def ready(self) -> bool:
    return self.tracking and self.age >= self.READY_MIN_TIME and self.p_vv < self.READY_SPEED_STD ** 2

  def predict(self, dt: float, v_ego: float) -> None:
    if not self.tracking or dt <= 0.0:
      return
    self.d += (self.v_lead - v_ego) * dt
    self.age += dt
    self.since_meas += dt

    q = self.LEAD_ACCEL_STD ** 2
    self.p_dd += 2.0 * dt * self.p_dv + dt * dt * self.p_vv + q * dt ** 4 / 4.0
    self.p_dv += dt * self.p_vv + q * dt ** 3 / 2.0
    self.p_vv += q * dt * dt

  def update(self, distance: float, v_ego: float) -> None:
    if not self.tracking:
      self._start(distance, v_ego)
      return

    innovation = distance - self.d
    if abs(innovation) > self.BIN_HALF_WIDTH + self.JUMP_THRESHOLD:
      # Not explainable by motion between two readings: a cut-in, or the lead left and exposed
      # the next car. Start a new track so radard does not smear two objects into one.
      self._start(distance, v_ego)
      return

    dt_meas = max(self.since_meas, 1e-3)
    if abs(innovation) <= self.BIN_HALF_WIDTH:
      innovation = 0.0
      r = self.IN_BIN_STD ** 2 * max(1.0, self.IN_BIN_CORR_TIME / dt_meas)
    else:
      innovation -= self.BIN_HALF_WIDTH if innovation > 0 else -self.BIN_HALF_WIDTH
      r = self.EDGE_STD ** 2 * max(1.0, self.EDGE_CORR_TIME / dt_meas)

    s = self.p_dd + r
    k_d = self.p_dd / s
    k_v = self.p_dv / s
    self.d += k_d * innovation
    self.v_lead += k_v * innovation
    self.p_vv -= k_v * self.p_dv
    self.p_dv *= (1.0 - k_d)
    self.p_dd *= (1.0 - k_d)
    self.since_meas = 0.0

  def _start(self, distance: float, v_ego: float) -> None:
    self.track_id += 1
    self.tracking = True
    self.d = distance
    self.v_lead = v_ego
    self.p_dd = self.IN_BIN_STD ** 2
    self.p_dv = 0.0
    self.p_vv = self.INIT_SPEED_STD ** 2
    self.age = 0.0
    self.since_meas = 0.0


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.tracker = LeadDistanceTracker()
    self.pt_cp: CANParser | None = None
    self.last_t = 0.0
    self.last_radar_t = 0.0
    self.v_ego = 0.0

    # openpilot longitudinal through the gateway (J533) harness: the radar is on the ext bus and still running
    mqb = not CP.flags & (VolkswagenFlags.PQ | VolkswagenFlags.MLB)
    if mqb and CP.openpilotLongitudinalControl and CP.networkLocation == NetworkLocation.gateway:
      can_bus = CanBus(CP)
      self.rcp = CANParser(DBC[CP.carFingerprint][Bus.pt], [("ACC_04", 0)], can_bus.ext)
      self.pt_cp = CANParser(DBC[CP.carFingerprint][Bus.pt], [("ESP_19", 100)], can_bus.pt)
      self.trigger_msg = self.rcp.dbc.name_to_msg["ACC_04"].address

  def update(self, can_packets):
    if self.rcp is None or self.pt_cp is None:
      return super().update(None)

    radar_updated = self.rcp.update(can_packets)
    self.pt_cp.update(can_packets)
    self.frame += 1

    if len(can_packets):
      t = can_packets[-1][0] * 1e-9
      dt = min(max(t - self.last_t, 0.0), 0.2)
      self.last_t = t

      wheels = self.pt_cp.vl["ESP_19"]
      self.v_ego = CV.KPH_TO_MS * (wheels["ESP_VL_Radgeschw_02"] + wheels["ESP_VR_Radgeschw_02"] +
                                   wheels["ESP_HL_Radgeschw_02"] + wheels["ESP_HR_Radgeschw_02"]) / 4.0

      self.tracker.predict(dt, self.v_ego)

      if self.trigger_msg in radar_updated:
        self.last_radar_t = t
        msg = self.rcp.vl["ACC_04"]
        distance = msg["ACC_Abstand_Abstandswarner"]
        # ACC_Texte_Abstandswarner != 0: system fault / no sensor view / restricted
        if 0 < distance <= DISTANCE_MAX_VALID and msg["ACC_Texte_Abstandswarner"] == 0:
          self.tracker.update(distance, self.v_ego)
        else:
          self.tracker.reset()
      elif t - self.last_radar_t > SIGNAL_TIMEOUT:
        self.tracker.reset()

    if self.frame % PUBLISH_DECIMATION != 0:
      return None

    # No errors are ever raised from here: a missing or blind radar must leave the camera-only
    # lead openpilot long already uses on this car, never block engagement.
    ret = structs.RadarData()
    if self.tracker.ready:
      pt = structs.RadarData.RadarPoint()
      pt.trackId = self.tracker.track_id
      pt.dRel = self.tracker.d
      pt.yRel = 0.0
      pt.vRel = self.tracker.v_lead - self.v_ego
      pt.aRel = float('nan')
      pt.yvRel = float('nan')
      pt.measured = True
      ret.points = [pt]
    return ret
