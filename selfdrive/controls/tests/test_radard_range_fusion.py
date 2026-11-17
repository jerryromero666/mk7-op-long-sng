import pytest

from cereal import log
from opendbc.car import structs
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls import radard
from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA, KalmanParams, RadarD, RadarTrustGate, RangeRadarFusion, Track, \
                                                 get_lead, read_radar_lead_on

V_EGO = 20.0


def vision_lead(d_rel, v_lead, prob=0.99, a_lead=0.0):
  return log.ModelDataV2.LeadDataV3.new_message(prob=prob, x=[d_rel + RADAR_TO_CAMERA], y=[0.3], v=[v_lead], a=[a_lead],
                                                xStd=[2.0], yStd=[0.5], vStd=[1.0])


def radar_track(d_rel, v_lead, identifier=1, v_ego=V_EGO):
  track = Track(identifier, v_lead, KalmanParams(DT_MDL))
  track.update(d_rel, 0.0, v_lead - v_ego, v_lead, True)
  return track


def fusion_in_use():
  fusion = RangeRadarFusion()
  fusion.on = True
  fusion.gate.trusted = True
  return fusion


class TestLiveFile:
  @pytest.mark.parametrize("content, on", [("1", True), ("1\n", True), ("1.0", True), ("0", False), ("2", False), ("", False),
                                           ("on", False), ("nan", False), ("inf", False), ("-1", False)])
  def test_values(self, tmp_path, monkeypatch, content, on):
    path = tmp_path / "radar_lead"
    path.write_text(content)
    monkeypatch.setattr(radard, "RADAR_LEAD_FILE", str(path))
    assert read_radar_lead_on() is on

  def test_no_file_is_off(self, tmp_path, monkeypatch):
    monkeypatch.setattr(radard, "RADAR_LEAD_FILE", str(tmp_path / "missing"))
    assert read_radar_lead_on() is False

  def test_file_is_followed_live(self, tmp_path, monkeypatch):
    path = tmp_path / "radar_lead"
    monkeypatch.setattr(radard, "RADAR_LEAD_FILE", str(path))
    fusion = RangeRadarFusion()
    track, lead = radar_track(30.0, V_EGO), vision_lead(30.0, V_EGO)

    def run(seconds):
      for _ in range(int(seconds / DT_MDL)):
        fusion.update(True, track, lead)
      return fusion.get_lead(0, V_EGO, True, {1: track}, lead, V_EGO)['radar']

    assert not run(15)          # no file: trusted by now, still unused
    assert fusion.gate.trusted
    path.write_text("1")
    assert run(1.1)
    path.unlink()
    assert not run(1.1)


class TestRadarTrustGate:
  def run(self, gate, seconds, radar_dist, vision_dist, prob=0.99):
    for _ in range(int(seconds / DT_MDL)):
      gate.update(radar_track(radar_dist, V_EGO) if radar_dist is not None else None, vision_lead(vision_dist, V_EGO, prob))
    return gate.trusted

  def test_trust_takes_sustained_agreement(self):
    gate = RadarTrustGate()
    assert not self.run(gate, RadarTrustGate.MIN_COMPARE_TIME - 1, 30.0, 31.0)
    assert self.run(gate, 2, 30.0, 31.0)

  def test_wrong_units_never_trusted(self):
    # e.g. a signal that is not metres: reads half the true distance
    gate = RadarTrustGate()
    assert not self.run(gate, 120, 15.0, 30.0)

  def test_wrong_units_not_trusted_from_a_standstill_queue(self):
    # sitting 5 m behind a stopped car, half the true distance is only 2.5 m off: too close to tell, so it must not count
    gate = RadarTrustGate()
    assert not self.run(gate, 120, 2.5, 5.0)
    assert gate.compare_frames == 0

  def test_stuck_value_never_trusted(self):
    gate = RadarTrustGate()
    for vision_dist in (20.0, 35.0, 50.0, 28.0) * 5:
      assert not self.run(gate, 5, 8.0, vision_dist)

  def test_no_comparison_without_confident_vision(self):
    gate = RadarTrustGate()
    assert not self.run(gate, 60, 30.0, 30.0, prob=0.6)
    assert not self.run(gate, 60, None, 30.0)
    assert not self.run(gate, 60, 90.0, 90.0)  # beyond the range where the camera can judge
    assert gate.compare_frames == 0

  def test_trust_is_withdrawn_and_regained(self):
    gate = RadarTrustGate()
    assert self.run(gate, 15, 30.0, 30.0)
    assert self.run(gate, 1, 60.0, 30.0)      # a brief disagreement is tolerated
    assert not self.run(gate, 6, 60.0, 30.0)  # a sustained one is not
    assert self.run(gate, 10, 30.0, 30.0)


class TestRangeRadarFusion:
  @pytest.mark.parametrize("on, trusted", [(False, False), (False, True), (True, False)])
  @pytest.mark.parametrize("prob", [0.99, 0.6, 0.4])
  @pytest.mark.parametrize("v_ego, radar_dist", [(V_EGO, 40.0), (1.0, 6.0)])
  def test_off_or_untrusted_is_exactly_camera_only(self, on, trusted, prob, v_ego, radar_dist):
    # what radard did on this car before: the stock lead selection with no radar tracks
    fusion = RangeRadarFusion()
    fusion.on, fusion.gate.trusted = on, trusted
    lead_msg = vision_lead(43.0, 17.0, prob=prob, a_lead=-0.7)
    CP = structs.CarParams(brand="volkswagen")
    for idx, low_speed_override in ((0, True), (1, False)):
      before = get_lead(v_ego, True, {}, lead_msg, v_ego, CP, None, low_speed_override=low_speed_override)
      assert fusion.get_lead(idx, v_ego, True, {1: radar_track(radar_dist, 18.0, v_ego=v_ego)}, lead_msg, v_ego) == before

  def test_not_ready(self):
    assert fusion_in_use().get_lead(0, V_EGO, False, {1: radar_track(40.0, 18.0)}, vision_lead(40.0, 18.0), V_EGO) == {'status': False}

  def test_radar_distance_camera_dynamics_up_close(self):
    fusion = fusion_in_use()
    lead = fusion.get_lead(0, V_EGO, True, {1: radar_track(20.0, 18.0)}, vision_lead(22.0, 17.0, a_lead=-1.5), V_EGO)
    assert lead['status'] and lead['radar'] and lead['radarTrackId'] == 1
    assert lead['dRel'] == pytest.approx(20.0)
    # first frame: speed is still the camera's, the radar has only begun to trim it
    assert lead['vLead'] == pytest.approx(17.0, abs=0.05)
    assert lead['vLeadK'] == lead['vLead']
    assert lead['vRel'] == pytest.approx(lead['vLead'] - V_EGO)
    assert lead['aLeadK'] == pytest.approx(-1.5)
    assert lead['aLeadTau'] == pytest.approx(0.3)
    assert lead['modelProb'] == pytest.approx(0.99)

  def test_radar_speed_far_away(self):
    fusion = fusion_in_use()
    lead = fusion.get_lead(0, 30.0, True, {1: radar_track(80.0, 27.0, v_ego=30.0)}, vision_lead(88.0, 24.0, a_lead=-0.8), 30.0)
    assert lead['dRel'] == pytest.approx(80.0)
    assert lead['vLead'] == pytest.approx(27.0)
    assert lead['aLeadK'] == pytest.approx(-0.8)

  def test_speed_blends_between_near_and_far(self):
    fusion = fusion_in_use()
    lead = fusion.get_lead(0, V_EGO, True, {1: radar_track(45.0, 18.0)}, vision_lead(46.0, 16.0), V_EGO)
    assert lead['vLead'] == pytest.approx(17.0, abs=0.05)

  def test_radar_trims_slow_camera_speed_error(self):
    fusion = fusion_in_use()
    track = radar_track(20.0, 18.0)
    for _ in range(int(30 / DT_MDL)):
      lead = fusion.get_lead(0, V_EGO, True, {1: track}, vision_lead(21.0, 17.0), V_EGO)
    assert lead['vLead'] == pytest.approx(18.0, abs=0.05)

  def test_speed_trim_is_bounded(self):
    fusion = fusion_in_use()
    track = radar_track(20.0, 14.0)
    for _ in range(int(60 / DT_MDL)):
      lead = fusion.get_lead(0, V_EGO, True, {1: track}, vision_lead(21.0, 18.0), V_EGO)
    assert lead['vLead'] == pytest.approx(18.0 - RangeRadarFusion.SPEED_BIAS_MAX, abs=0.05)

  def test_late_radar_speed_does_not_hide_a_braking_lead_up_close(self):
    # the radar's differentiated speed lags a braking lead; up close that must not delay what the camera sees
    fusion = fusion_in_use()
    for _ in range(int(20 / DT_MDL)):
      fusion.get_lead(0, V_EGO, True, {1: radar_track(20.0, 18.0)}, vision_lead(20.0, 18.0), V_EGO)
    for _ in range(int(1 / DT_MDL)):  # lead brakes: the camera sees 15 m/s, the radar estimate still says 18
      lead = fusion.get_lead(0, V_EGO, True, {1: radar_track(19.0, 18.0)}, vision_lead(19.0, 15.0, a_lead=-3.0), V_EGO)
    assert lead['vLead'] < 15.5
    assert lead['aLeadK'] == pytest.approx(-3.0)

  def test_new_radar_object_restarts_speed_trim(self):
    fusion = fusion_in_use()
    for _ in range(int(30 / DT_MDL)):
      fusion.get_lead(0, V_EGO, True, {1: radar_track(25.0, 18.0)}, vision_lead(26.0, 17.0), V_EGO)
    lead = fusion.get_lead(0, V_EGO, True, {2: radar_track(15.0, 15.0, identifier=2)}, vision_lead(15.0, 15.0), V_EGO)
    assert lead['vLead'] == pytest.approx(15.0, abs=0.05)

  def test_radar_never_creates_a_lead(self):
    fusion = fusion_in_use()
    track = radar_track(40.0, 18.0)
    fusion.get_lead(0, V_EGO, True, {1: track}, vision_lead(41.0, 18.0), V_EGO)   # camera confirms the car
    assert fusion.get_lead(0, V_EGO, True, {1: track}, vision_lead(41.0, 18.0, prob=0.2), V_EGO) == {'status': False}
    slow = radar_track(6.0, 0.0, v_ego=1.0)
    assert fusion.get_lead(0, 1.0, True, {1: slow}, vision_lead(40.0, 0.0, prob=0.1), 1.0) == {'status': False}

  def test_radar_never_removes_a_lead(self):
    fusion = fusion_in_use()
    lead = fusion.get_lead(0, V_EGO, True, {}, vision_lead(30.0, 17.0), V_EGO)
    assert lead['status'] and not lead['radar'] and lead['dRel'] == pytest.approx(30.0)

  def test_different_object_is_ignored(self):
    fusion = fusion_in_use()
    lead = fusion.get_lead(0, V_EGO, True, {1: radar_track(90.0, 18.0)}, vision_lead(30.0, 17.0), V_EGO)
    assert lead['status'] and not lead['radar']
    assert lead['dRel'] == pytest.approx(30.0)
    # a closer radar object the camera never matched is not taken either
    lead = fusion.get_lead(0, V_EGO, True, {1: radar_track(12.0, 18.0)}, vision_lead(40.0, 17.0), V_EGO)
    assert not lead['radar'] and lead['dRel'] == pytest.approx(40.0)

  def test_confirmed_car_is_kept_while_the_radar_reads_closer(self):
    # far away the camera over-reads distance by more than the match band; that must not flip the lead back and forth
    fusion = fusion_in_use()
    track = radar_track(70.0, 25.0, v_ego=30.0)
    assert fusion.get_lead(0, 30.0, True, {1: track}, vision_lead(75.0, 25.0), 30.0)['radar']
    lead = fusion.get_lead(0, 30.0, True, {1: track}, vision_lead(105.0, 25.0), 30.0)
    assert lead['radar'] and lead['dRel'] == pytest.approx(70.0)

  def test_camera_seeing_something_closer_wins(self):
    # cut-in: the camera has a new, closer car while the radar still holds the old one
    fusion = fusion_in_use()
    track = radar_track(70.0, 25.0, v_ego=30.0)
    assert fusion.get_lead(0, 30.0, True, {1: track}, vision_lead(72.0, 25.0), 30.0)['radar']
    lead = fusion.get_lead(0, 30.0, True, {1: track}, vision_lead(25.0, 24.0), 30.0)
    assert not lead['radar'] and lead['dRel'] == pytest.approx(25.0)
    # and the old radar object has to be matched again before it counts
    lead = fusion.get_lead(0, 30.0, True, {1: track}, vision_lead(110.0, 25.0), 30.0)
    assert not lead['radar']

  def test_lead_dict_is_a_valid_radar_state_lead(self):
    rs = log.RadarState.new_message()
    rs.leadOne = fusion_in_use().get_lead(0, V_EGO, True, {1: radar_track(40.0, 18.0)}, vision_lead(41.0, 18.0), V_EGO)
    assert rs.leadOne.status and rs.leadOne.radar and rs.leadOne.dRel == pytest.approx(40.0)


class TestRadarDSelection:
  def test_only_volkswagen_uses_range_fusion(self):
    assert RadarD(structs.CarParams(brand="volkswagen", radarUnavailable=True), None).range_fusion is not None
    assert RadarD(structs.CarParams(brand="toyota"), None).range_fusion is None
