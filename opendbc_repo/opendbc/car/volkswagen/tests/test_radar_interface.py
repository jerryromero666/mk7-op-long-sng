import math

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.radar_interface import LeadDistanceTracker, RadarInterface
from opendbc.car.volkswagen.values import CAR, DBC, CanBus

DT = 0.01
FREE_DRIVING = 509


def gateway_params(alpha_long=True):
  fingerprint = gen_empty_fingerprint()
  fingerprint[1][0x40] = 8  # Airbag_01 on the powertrain bus: gateway (J533) harness
  return CarInterface.get_params(CAR.VOLKSWAGEN_GOLF_MK7, fingerprint, [], alpha_long, False, False)


def run_tracker(d0, v_ego, v_lead, seconds, rate=17, floor=False, tracker=None):
  """Feeds a tracker the quantized distance to a lead moving at constant speed."""
  tracker = tracker or LeadDistanceTracker()
  step = round(1 / rate / DT)
  d = d0
  for i in range(int(seconds / DT)):
    d += (v_lead - v_ego) * DT
    tracker.predict(DT, v_ego)
    if i % step == 0:
      tracker.update(float(math.floor(d) if floor else round(d)), v_ego)
  return tracker, d


class TestLeadDistanceTracker:
  def test_constant_gap(self):
    tracker, d = run_tracker(40.2, 30.0, 30.0, 10)
    assert tracker.ready
    assert abs(tracker.d - d) < 0.6
    assert abs(tracker.v_lead - 30.0) < 0.3

  def test_closing(self):
    for rate in (5, 17, 25):
      tracker, d = run_tracker(120.0, 28.0, 20.0, 8, rate=rate)
      assert tracker.ready
      assert abs(tracker.d - d) < 1.0, rate
      assert abs(tracker.v_lead - 20.0) < 0.5, rate

  def test_opening(self):
    tracker, d = run_tracker(10.0, 5.0, 8.0, 8)
    assert abs(tracker.d - d) < 1.0
    assert abs(tracker.v_lead - 8.0) < 0.5

  def test_truncating_sensor(self):
    # if the radar truncates instead of rounding, distance reads short by under a metre, speed is unaffected
    tracker, d = run_tracker(60.0, 25.0, 22.0, 8, floor=True)
    assert -1.0 < tracker.d - d <= 0.1
    assert abs(tracker.v_lead - 22.0) < 0.5

  def test_not_ready_until_speed_is_known(self):
    tracker = LeadDistanceTracker()
    tracker.update(40.0, 30.0)
    assert tracker.tracking and not tracker.ready
    tracker, _ = run_tracker(40.0, 30.0, 30.0, 0.3, tracker=LeadDistanceTracker())
    assert not tracker.ready

  def test_quantization_does_not_ripple_speed(self):
    # a slowly closing lead crosses a 1 m bin every few seconds; the speed estimate must not spike at each step
    tracker = LeadDistanceTracker()
    d, v_ego, v_lead = 50.4, 30.0, 29.7
    worst = 0.0
    for i in range(int(30 / DT)):
      d += (v_lead - v_ego) * DT
      tracker.predict(DT, v_ego)
      if i % 6 == 0:
        tracker.update(float(round(d)), v_ego)
      if i * DT > 10:
        worst = max(worst, abs(tracker.v_lead - v_lead))
    assert worst < 0.5

  def test_new_object_gets_new_track(self):
    tracker, _ = run_tracker(60.0, 30.0, 30.0, 5)
    first = tracker.track_id
    tracker.predict(DT, 30.0)
    tracker.update(25.0, 30.0)  # cut-in
    assert tracker.track_id == first + 1
    assert not tracker.ready
    assert tracker.d == 25.0

  def test_reset(self):
    tracker, _ = run_tracker(60.0, 30.0, 30.0, 5)
    tracker.reset()
    assert not tracker.tracking and not tracker.ready


class TestRadarInterface:
  def setup_method(self):
    self.CP = gateway_params()
    self.packer = CANPacker(DBC[self.CP.carFingerprint][Bus.pt])
    self.bus = CanBus(self.CP)
    self.t = 0
    self.frame = 0

  def step(self, RI, distance, v_ego, text=0, radar_bus=None, send_radar=True):
    """One 100 Hz card iteration: wheel speeds every frame, ACC_04 at ~17 Hz."""
    kph = v_ego * CV.MS_TO_KPH
    wheels = {s: kph for s in ("ESP_VL_Radgeschw_02", "ESP_VR_Radgeschw_02", "ESP_HL_Radgeschw_02", "ESP_HR_Radgeschw_02")}
    msgs = [self.packer.make_can_msg("ESP_19", self.bus.pt, wheels)]
    if send_radar and self.frame % 6 == 0:
      values = {"ACC_Abstand_Abstandswarner": distance, "ACC_Texte_Abstandswarner": text}
      msgs.append(self.packer.make_can_msg("ACC_04", self.bus.ext if radar_bus is None else radar_bus, values))
    self.t += int(DT * 1e9)
    self.frame += 1
    return RI.update([(self.t, msgs)])

  def drive(self, RI, d0, v_ego, v_lead, seconds, **kwargs):
    d, out = d0, []
    for _ in range(int(seconds / DT)):
      d += (v_lead - v_ego) * DT
      rd = self.step(RI, round(d), v_ego, **kwargs)
      if rd is not None:
        out.append(rd)
    return d, out

  def test_car_params_unchanged(self):
    # the radar lead is a live-file feature in radard; the car's parameters stay those of a radarless car
    assert self.CP.radarUnavailable
    assert self.CP.openpilotLongitudinalControl

  def test_publishes_at_20hz(self):
    RI = RadarInterface(self.CP, None)
    _, out = self.drive(RI, 40.0, 20.0, 20.0, 2)
    assert len(out) == 40

  def test_lead(self):
    RI = RadarInterface(self.CP, None)
    d, out = self.drive(RI, 80.0, 30.0, 25.0, 6)
    pts = out[-1].points
    assert len(pts) == 1
    assert abs(pts[0].dRel - d) < 1.0
    assert abs(pts[0].vRel - -5.0) < 0.5
    assert pts[0].yRel == 0.0
    assert pts[0].measured
    assert not any(any(rd.errors.to_dict().values()) for rd in out)

  def test_no_lead(self):
    RI = RadarInterface(self.CP, None)
    _, out = self.drive(RI, 40.0, 20.0, 20.0, 3)
    assert len(out[-1].points) == 1
    for _ in range(20):
      rd = self.step(RI, FREE_DRIVING, 20.0)
    while rd is None:
      rd = self.step(RI, FREE_DRIVING, 20.0)
    assert len(rd.points) == 0
    assert not any(rd.errors.to_dict().values())

  def test_sensor_blind(self):
    RI = RadarInterface(self.CP, None)
    _, out = self.drive(RI, 40.0, 20.0, 20.0, 3, text=2)  # keine_Sensorsicht
    assert all(len(rd.points) == 0 for rd in out)
    assert not any(any(rd.errors.to_dict().values()) for rd in out)

  def test_signal_goes_away(self):
    RI = RadarInterface(self.CP, None)
    _, out = self.drive(RI, 40.0, 20.0, 20.0, 3)
    assert len(out[-1].points) == 1
    _, out = self.drive(RI, 40.0, 20.0, 20.0, 1, send_radar=False)
    assert len(out[-1].points) == 0
    assert not any(any(rd.errors.to_dict().values()) for rd in out)

  def test_wrong_bus_is_ignored(self):
    RI = RadarInterface(self.CP, None)
    _, out = self.drive(RI, 40.0, 20.0, 20.0, 3, radar_bus=self.bus.pt)
    assert all(len(rd.points) == 0 for rd in out)

  def test_only_with_openpilot_long_on_the_gateway_harness(self):
    # stock ACC: the radar is in charge of longitudinal; camera harness: no openpilot longitudinal
    camera_harness = CarInterface.get_params(CAR.VOLKSWAGEN_GOLF_MK7, gen_empty_fingerprint(), [], True, False, False)
    for CP in (gateway_params(alpha_long=False), camera_harness):
      RI = RadarInterface(CP, None)
      assert RI.rcp is None
      _, out = self.drive(RI, 40.0, 20.0, 20.0, 3)
      assert len(out) > 0 and all(len(rd.points) == 0 for rd in out)
