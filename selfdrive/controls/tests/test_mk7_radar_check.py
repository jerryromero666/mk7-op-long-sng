"""mk7_radar_check.py carries its own copy of the radar lead maths so it runs from /data on any branch. Keep the copy equal."""
import importlib.util
import math
import os
import random

import pytest

from cereal import log
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.volkswagen import radar_interface
from opendbc.car.volkswagen.values import CAR, DBC
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls import radard

TOOL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "mk7_radar_check.py")
spec = importlib.util.spec_from_file_location("mk7_radar_check", TOOL_PATH)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


def test_constants():
  for name in ("BIN_HALF_WIDTH", "LEAD_ACCEL_STD", "IN_BIN_STD", "IN_BIN_CORR_TIME", "EDGE_STD", "EDGE_CORR_TIME", "JUMP_THRESHOLD",
               "INIT_SPEED_STD", "READY_SPEED_STD", "READY_MIN_TIME"):
    assert getattr(tool.LeadDistanceTracker, name) == getattr(radar_interface.LeadDistanceTracker, name), name
  for name in ("MIN_VISION_PROB", "MIN_COMPARE_DIST", "MAX_COMPARE_DIST", "MIN_COMPARE_TIME", "SCORE_TAU", "TRUST_SCORE", "DISTRUST_SCORE"):
    assert getattr(tool.Fusion, name) == getattr(radard.RadarTrustGate, name), name
  for name in ("SPEED_BIAS_TAU", "SPEED_BIAS_MAX"):
    assert getattr(tool.Fusion, name) == getattr(radard.RangeRadarFusion, name), name
  assert list(tool.Fusion.FAR_BP) == radard.RangeRadarFusion.FAR_BP
  assert tool.DISTANCE_MAX_VALID == radar_interface.DISTANCE_MAX_VALID
  assert tool.SIGNAL_TIMEOUT == radar_interface.SIGNAL_TIMEOUT
  assert tool.RADAR_TO_CAMERA == radard.RADAR_TO_CAMERA
  assert tool.RADAR_LEAD_FILE == radard.RADAR_LEAD_FILE
  assert tool.DT_MDL == DT_MDL


def test_can_decoding():
  packer = CANPacker(DBC[CAR.VOLKSWAGEN_GOLF_MK7][Bus.pt])
  for distance, text in ((1, 0), (37, 0), (508, 0), (509, 0), (511, 3)):
    _, dat, _ = packer.make_can_msg("ACC_04", 2, {"ACC_Abstand_Abstandswarner": distance, "ACC_Texte_Abstandswarner": text,
                                                  "ACC_Zeitluecke_Abstandswarner": 6.3, "ACC_Tempolimit": 31, "ACC_Charisma_FahrPr": 15})
    assert tool.acc_04(dat) == (distance, text)
  _, dat, _ = packer.make_can_msg("ACC_02", 2, {"ACC_Abstandsindex": 777, "ACC_Wunschgeschw_02": 327.04, "ACC_Gesetzte_Zeitluecke": 7})
  assert tool.acc_02(dat) == 777
  _, dat, _ = packer.make_can_msg("ACC_06", 2, {"ACC_Status_ACC": 3, "ACC_Sollbeschleunigung_02": -1.25, "ACC_Typ": 1, "ACC_Anhalten": 1})
  status, accel = tool.acc_06(dat)
  assert status == 3 and accel == pytest.approx(-1.25, abs=0.003)
  _, dat, _ = packer.make_can_msg("ESP_19", 0, {"ESP_VL_Radgeschw_02": 72.0, "ESP_VR_Radgeschw_02": 72.6, "ESP_HL_Radgeschw_02": 71.4,
                                                "ESP_HR_Radgeschw_02": 72.0})
  assert tool.wheel_speed(dat) == pytest.approx(20.0, abs=0.01)


def test_tracker_copy_is_identical():
  rng = random.Random(0)
  ours, theirs = tool.LeadDistanceTracker(), radar_interface.LeadDistanceTracker()
  d, v_ego, v_lead = 60.0, 25.0, 23.0
  for i in range(6000):
    if i % 700 == 0:
      v_lead = rng.uniform(0.0, 30.0)
    if i == 3000:
      d = 22.0   # cut-in
    v_ego = max(0.0, v_ego + rng.uniform(-0.03, 0.03))
    d = max(1.0, d + (v_lead - v_ego) * 0.01)
    for t in (ours, theirs):
      t.predict(0.01, v_ego)
      if i % 6 == 0:
        if 4000 <= i < 4100:
          t.reset()
        else:
          t.update(float(round(d)), v_ego)
    assert (ours.tracking, ours.ready, ours.track_id) == (theirs.tracking, theirs.ready, theirs.track_id), i
    assert ours.d == theirs.d and ours.v_lead == theirs.v_lead and ours.p_vv == theirs.p_vv, i
  assert theirs.track_id >= 3


def test_fusion_copy_decides_the_same(monkeypatch):
  monkeypatch.setattr(radard, "read_radar_lead_on", lambda: True)
  rng = random.Random(1)
  ours, theirs = tool.Fusion(), radard.RangeRadarFusion()
  v_ego = 22.0
  used = unused = 0
  for i in range(int(180 / DT_MDL)):
    t = i * DT_MDL
    true_d = 35.0 + 30.0 * math.sin(t / 9.0) + (40.0 if 100 < t < 110 else 0.0)
    true_v = v_ego + 30.0 / 9.0 * math.cos(t / 9.0)
    prob = 0.3 if 60 < t < 63 else 0.97
    vis_d = true_d * (1.0 + (0.5 if 130 < t < 150 else 0.08)) + rng.uniform(-0.5, 0.5)   # 130-150 s: beyond the match band
    model_v = true_v + rng.uniform(-0.8, 0.8)
    model_v_ego = v_ego + 0.2
    have_radar = not 80 < t < 84
    track_id = 1 + int(t > 100) + int(t > 110)
    radar_d = float(round(true_d * (0.5 if 150 < t < 165 else 1.0)))   # 150-165 s: a radar that has stopped making sense
    radar_v = true_v + 0.4

    lead_msg = log.ModelDataV2.LeadDataV3.new_message(prob=prob, x=[vis_d + radard.RADAR_TO_CAMERA], y=[0.0], v=[model_v], a=[0.0],
                                                      xStd=[2.0], yStd=[0.5], vStd=[1.0])
    tracks = {}
    if have_radar:
      track = radard.Track(track_id, radar_v, radard.KalmanParams(DT_MDL))
      track.update(radar_d, 0.0, radar_v - v_ego, radar_v, True)
      tracks[track_id] = track
    theirs.update(True, tracks.get(track_id), lead_msg)
    want = theirs.get_lead(0, v_ego, True, tracks, lead_msg, model_v_ego)

    trk = (track_id, radar_d, radar_v) if have_radar else None
    ours.gate(trk, prob, vis_d)
    got = ours.lead(trk, prob, vis_d, v_ego + (model_v - model_v_ego), model_v)

    assert ours.trusted == theirs.gate.trusted, t
    assert (got is not None) == bool(want['status'] and want['radar']), t
    if got is not None:
      used += 1
      assert got[0] == pytest.approx(want['dRel']) and got[1] == pytest.approx(want['vLead']), t
    else:
      unused += 1
  assert used > 1000 and unused > 300
