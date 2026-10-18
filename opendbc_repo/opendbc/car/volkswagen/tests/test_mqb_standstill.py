"""MQBStandstillManager + mqbcan.create_acc_accel_control: the ACC_06 / ACC_07 bits per phase of a stop, today and with
announce brake (/data/announce_brake)."""
from types import SimpleNamespace as NS

import pytest

from opendbc.car.volkswagen import carcontroller, mqbcan
from opendbc.car.volkswagen.carcontroller import MQBStandstillManager, read_announce_brake
from opendbc.car.volkswagen.values import CarControllerParams as CCP

STEP = 1.5 * 0.02   # announce brake rate 1.5 m/s3, per 50 Hz frame


@pytest.fixture(autouse=True)
def _restore_live_file(monkeypatch):
  # the tests point ANNOUNCE_BRAKE_FILE at their own files; put the real path back after each one
  monkeypatch.setattr(carcontroller, "ANNOUNCE_BRAKE_FILE", carcontroller.ANNOUNCE_BRAKE_FILE)


class Packer:
  def make_can_msg(self, name, bus, values):
    return name, dict(values)


def frame(ssm, v, accel, lcs, b63=False, b53=False, long_active=True):
  """One 50 Hz ACC frame as CarController.update builds it. lcs: 'pid' / 'stopping'."""
  CS = NS(out=NS(vEgo=v, standstill=v == 0.0, brakePressed=False), esp_hold_confirmation=b53, esp_stopping=b63)
  stopping = lcs == "stopping"
  starting = lcs == "pid" and (b53 or v < 0.5)   # vEgoStopping 0.5
  _, accel, stopping, starting, so, sto = ssm.update(CS, long_active, accel, stopping, starting)
  acc_control = mqbcan.acc_control_value(True, False, long_active)
  a06, a07 = mqbcan.create_acc_accel_control(Packer(), 0, 1, long_active, accel, acc_control, stopping, starting, b53,
                                             so, sto)
  a06, a07 = a06[1], a07[1]
  return dict(accel=a06["ACC_Sollbeschleunigung_02"], anh06=a06["ACC_Anhalten"], anf06=a06["ACC_Anfahren"],
              hms=a07["ACC_Anforderung_HMS"], anh07=a07["ACC_Anhalten"], anf07=a07["ACC_Anfahren"],
              weg=a07["ACC_Anhalteweg"], accel07=a07["ACC_Sollbeschleunigung_02"])


# A stop as on the car (drive 341 stop 10): pid braking through 2 mph, the ESP answers the announcement (bit 63) after
# 2 frames of HMS 1, openpilot stays in pid easing from -0.47 for ~0.5 s, then its stopping state ramps to -1.2,
# standstill, the lead leaves (pid +0.6), driving off past RELEASE_SPEED.
def script():
  s = []
  v = 1.1
  for _ in range(10):                       # P0: pid, braking, above ANNOUNCE_SPEED
    s.append(dict(v=v, accel=-0.55, lcs="pid"))
    v -= 0.55 * 0.02
  for i in range(40):                        # P1 / P2: announcement, then announced hold while openpilot is in pid
    s.append(dict(v=v, accel=-0.47 + 0.003 * i, lcs="pid"))
    v -= 0.4 * 0.02
  a = -0.35
  while v > 0.0:                             # P3: openpilot's stopping state while rolling
    a = max(a - 6.0 * 0.02, -1.2)
    s.append(dict(v=v, accel=a, lcs="stopping"))
    v = max(0.0, v - 0.9 * 0.02)
  for _ in range(50):                        # P4: standstill hold
    s.append(dict(v=0.0, accel=-1.2, lcs="stopping"))
  v = 0.0
  for _ in range(150):                       # P5: launch
    s.append(dict(v=v, accel=0.6, lcs="pid"))
    v += 0.6 * 0.02
  return s


def run(tmp_path, live, sc=None, **kw):
  """The script through the manager and the builder, with the ESP answering HMS 1 + ACC_07 Anhalten while rolling
  with bit 63 after 2 frames (drive 341: HMS 0>1 at -1.48 s, 1>4 at -1.44 s)."""
  f = tmp_path / "announce_brake"
  if live is None:
    carcontroller.ANNOUNCE_BRAKE_FILE = str(tmp_path / "missing")
  else:
    f.write_text(live)
    carcontroller.ANNOUNCE_BRAKE_FILE = str(f)
  ssm = MQBStandstillManager(CCP)
  out, n1, b63 = [], 0, False
  for st in (sc or script()):
    fr = frame(ssm, b63=b63, **dict(st, **kw))
    n1 = n1 + 1 if (fr["hms"] == 1 and fr["anh07"] and st["v"] > 0) else 0
    b63 = b63 or n1 >= 2
    out.append(fr)
  return out


def phases(out, sc=None):
  sc = sc or script()
  i_p1 = next(i for i, f in enumerate(out) if f["hms"] == 1)
  i_p2 = next(i for i, f in enumerate(out) if f["hms"] == 4)
  i_p3 = next(i for i, s in enumerate(sc) if s["lcs"] == "stopping")
  i_p4 = next(i for i, s in enumerate(sc) if s["v"] == 0.0)
  i_p5 = next(i for i, s in enumerate(sc) if s["accel"] > 0)
  return i_p1, i_p2, i_p3, i_p4, i_p5


def test_phases_today(tmp_path):
  sc = script()
  out = run(tmp_path, None, sc)
  i1, i2, i3, i4, i5 = phases(out, sc)
  # P0 pid above 0.9 m/s: nothing special
  for f in out[:i1]:
    assert (f["hms"], f["anh06"], f["anf06"], f["anh07"], f["anf07"], f["weg"]) == (0, False, False, False, False, 20.46)
  # P1 the announcement: HMS 1, ACC_07 Anhalten d 0.3, ACC_06 Anfahren (not Anhalten), openpilot's accel, 2 frames
  assert i2 - i1 == 2
  for f in out[i1:i2]:
    assert (f["hms"], f["anh06"], f["anf06"], f["anh07"], f["anf07"], f["weg"]) == (1, False, True, True, False, 0.3)
  # P2 announced hold while openpilot is still in pid: HMS 4 / ACC_07 Anfahren and ACC_06 WITHOUT Anhalten, with
  # openpilot's small negative accel -- the state in which the TSK target went to +0.2 on the car
  assert i3 > i2
  for k in range(i2, i3):
    f = out[k]
    assert (f["hms"], f["anh06"], f["anf07"], f["anh07"], f["weg"]) == (4, False, True, False, 20.46)
    assert f["accel"] == sc[k]["accel"] < 0
  # P3 / P4 stopping state, rolling and at standstill: HMS 4 / ACC_07 Anfahren + ACC_06 Anhalten, openpilot's ramp
  for k in range(i3, i5):
    f = out[k]
    assert (f["hms"], f["anh06"], f["anf06"], f["anf07"], f["anh07"]) == (4, True, False, True, False)
    assert f["accel"] == sc[k]["accel"]
  # P5 launch: HMS 4 + Anfahren both, positive accel, until RELEASE_SPEED, then HMS 0
  for k in range(i5, len(sc)):
    f = out[k]
    if sc[k]["v"] <= MQBStandstillManager.RELEASE_SPEED:
      assert (f["hms"], f["anh06"], f["anf07"], f["accel"]) == (4, False, True, 0.6)
    elif sc[k]["v"] > 1.0 + 0.02:
      assert (f["hms"], f["anh06"], f["anf06"], f["anf07"]) == (0, False, False, False)


def release_state(f, v):
  # what made the TSK target +0.2 on the car: HMS 4 / ACC_07 Anfahren, ACC_06 without Anhalten, braking, rolling
  return v > 0 and f["hms"] == 4 and f["anf07"] and not f["anh06"] and f["accel"] <= 0


def test_today_has_the_release_state(tmp_path):
  sc = script()
  out = run(tmp_path, None, sc)
  assert sum(release_state(f, s["v"]) for f, s in zip(out, sc, strict=True)) >= 20


def test_announce_brake_never_sends_the_release_state(tmp_path):
  sc = script()
  out = run(tmp_path, "1.5", sc)
  assert not any(release_state(f, s["v"]) for f, s in zip(out, sc, strict=True))


def test_announce_brake_phases(tmp_path):
  sc = script()
  ref = run(tmp_path, None, sc)
  out = run(tmp_path, "1.5", sc)
  i1, i2, i3, i4, i5 = phases(out, sc)
  assert phases(ref, sc)[:2] == (i1, i2)
  # before the latch and the announcement itself: unchanged
  assert out[:i2] == ref[:i2]
  # from the latch to the standstill: ACC_06 Anhalten (not Anfahren), ACC_07 unchanged (HMS 4 / Anfahren / d 20.46)
  for k in range(i2, i4):
    f = out[k]
    assert (f["hms"], f["anh06"], f["anf06"], f["anf07"], f["anh07"], f["weg"]) == (4, True, False, True, False, 20.46)
    assert (f["hms"], f["anf07"], f["anh07"], f["weg"]) == (ref[k]["hms"], ref[k]["anf07"], ref[k]["anh07"], ref[k]["weg"])
    assert f["accel"] == f["accel07"]
  # openpilot still in pid: exactly its own request
  for k in range(i2, i3):
    assert out[k]["accel"] == sc[k]["accel"] == ref[k]["accel"]
  # openpilot's stopping state while rolling: its ramp to -1.2 (6 m/s3) followed at 1.5 m/s3, never easing, then exactly
  prev = out[i3 - 1]["accel"]
  k_hit = None
  for k in range(i3, i4):
    a = out[k]["accel"]
    assert a <= prev + 1e-9 and prev - a <= STEP + 1e-9
    assert a >= sc[k]["accel"] - 1e-9
    if k_hit is None and a == pytest.approx(-1.2):
      k_hit = k
    prev = a
  assert k_hit is not None and k_hit - i3 <= int((1.2 - 0.35) / STEP + 0.999) + 1
  for k in range(k_hit, i4):
    assert out[k]["accel"] == sc[k]["accel"]
  # standstill and launch: today's frames
  assert out[i4:] == ref[i4:]


def test_announce_brake_never_eases_in_stopping(tmp_path):
  # openpilot's stopping request jumps back up while the car still rolls: announce brake keeps the braking it reached
  sc = script()
  i3 = next(i for i, s in enumerate(sc) if s["lcs"] == "stopping")
  i4 = next(i for i, s in enumerate(sc) if s["v"] == 0.0)
  for k in range(i3 + 15, i4):
    sc[k]["accel"] = -0.6
  out = run(tmp_path, "1.5", sc)
  low = min(out[k]["accel"] for k in range(i3, i3 + 15))
  assert low < -0.6
  for k in range(i3 + 15, i4):
    assert out[k]["accel"] == pytest.approx(low)


def test_announce_brake_second_stop_starts_fresh(tmp_path):
  # a second stop in which the stopping state starts BEFORE the latch (most stops on the car): announce brake starts
  # from openpilot's request at the latch (its stopping ramp, still above -1.2), not from what it sent at the end of
  # the first stop (-1.2)
  first = script()
  second = []
  v, a = 0.905, -0.3
  while v > 0.0:                                 # stopping state from just above ANNOUNCE_SPEED, ramp to -1.2 at 6 m/s3
    a = max(a - 6.0 * 0.02, -1.2)
    second.append(dict(v=v, accel=a, lcs="stopping"))
    v = max(0.0, v - 0.8 * 0.02)
  sc = first + second
  ref = run(tmp_path, None, sc)
  out = run(tmp_path, "1.5", sc)
  n = len(first)
  j2 = next(k for k in range(n, len(sc)) if out[k]["hms"] == 4)
  assert sc[j2]["lcs"] == "stopping" and ref[j2]["anh06"] and sc[j2]["accel"] > -1.2
  assert out[j2]["accel"] == sc[j2]["accel"]
  assert out[n:j2] == ref[n:j2]


def test_announce_brake_yields_to_any_positive_request(tmp_path):
  # openpilot asks for +0.1 for a moment (the planner letting go in the last metres): announce brake never brakes
  # while openpilot asks to accelerate, so these frames are today's (small requests do not start the hold-off)
  sc = script()
  i2 = phases(run(tmp_path, None, sc), sc)[1]
  for k in range(i2 + 5, i2 + 10):
    sc[k]["accel"] = 0.1
  ref = run(tmp_path, None, sc)
  out = run(tmp_path, "1.5", sc)
  assert out[i2 + 5:i2 + 10] == ref[i2 + 5:i2 + 10]
  assert all(out[k]["anh06"] for k in range(i2 + 10, i2 + 15))


def test_announce_brake_waits_after_a_push(tmp_path):
  # openpilot pushed (+1.0, e.g. the camera lost the car ahead) and brakes again: today's frames for
  # ANNOUNCE_BRAKE_HOLDOFF frames after the push (the TSK target drops to +0.2 at once there), then Anhalten
  sc = script()
  i2 = phases(run(tmp_path, None, sc), sc)[1]
  for k in range(i2 + 3, i2 + 8):
    sc[k]["accel"] = 1.0
  ref = run(tmp_path, None, sc)
  out = run(tmp_path, "1.5", sc)
  hold = MQBStandstillManager.ANNOUNCE_BRAKE_HOLDOFF
  assert out[i2 + 3:i2 + 8 + hold - 1] == ref[i2 + 3:i2 + 8 + hold - 1]
  assert out[i2 + 8 + hold - 1]["anh06"]


def test_announce_brake_passes_harder_braking(tmp_path):
  # openpilot brakes hard in pid inside the announced hold (car ahead): sent as is, with Anhalten
  sc = script()
  i2 = phases(run(tmp_path, None, sc), sc)[1]
  for k in range(i2 + 3, i2 + 8):
    sc[k]["accel"] = -2.0
  out = run(tmp_path, "1.5", sc)
  for k in range(i2 + 3, i2 + 8):
    assert out[k]["accel"] == -2.0 and out[k]["anh06"]


def test_announce_brake_rate_6_is_only_the_bit(tmp_path):
  # rate 6 m/s3 = openpilot's own stopping ramp: only ACC_06 Anhalten / Anfahren change, every request is openpilot's
  sc = script()
  ref = run(tmp_path, None, sc)
  out = run(tmp_path, "6", sc)
  i2 = phases(out, sc)[1]
  for k, (f, r) in enumerate(zip(out, ref, strict=True)):
    assert f["accel"] == r["accel"]
    if k < i2 or sc[k]["accel"] > 0:
      assert f == r


def test_announce_brake_yields_to_moving_off(tmp_path):
  # the lead leaves while the car still rolls in the announced hold: openpilot asks +0.5 -> today's frames at once
  sc = script()
  i2 = phases(run(tmp_path, None, sc), sc)[1]
  for k in range(i2 + 5, len(sc)):
    sc[k] = dict(v=max(sc[k]["v"], 0.6), accel=0.5, lcs="pid")
  ref = run(tmp_path, None, sc)
  out = run(tmp_path, "1.5", sc)
  assert all(out[k]["anh06"] for k in range(i2, i2 + 5))
  assert out[i2 + 5:] == ref[i2 + 5:]
  assert out[i2 + 5]["accel"] == 0.5 and out[i2 + 5]["hms"] == 4 and not out[i2 + 5]["anh06"]


def test_announce_brake_esp_hold_is_parents(tmp_path):
  # bit 53 (the ESP's own hold) at standstill: the parent's HMS 3, exactly as today
  sc = script()
  i4 = next(i for i, s in enumerate(sc) if s["v"] == 0.0)
  for k in range(i4 + 10, i4 + 40):
    sc[k]["b53"] = True
  ref = run(tmp_path, None, sc)
  out = run(tmp_path, "1.5", sc)
  assert out[i4 + 10:] == ref[i4 + 10:]
  assert out[i4 + 10]["hms"] == 3


@pytest.mark.parametrize("live", [None, "0", "-1.5", "nan", "inf", "", "junk", "1.5 2"])
def test_off_is_today(tmp_path, live):
  sc = script()
  assert run(tmp_path, live, sc) == run(tmp_path, None, sc)


def test_parse(tmp_path):
  f = tmp_path / "announce_brake"
  carcontroller.ANNOUNCE_BRAKE_FILE = str(f)
  for text, want in (("1.5", 1.5), (" 2.5\n", 2.5), ("0.2", 1.0), ("9", 6.0), ("0", 0.0), ("-1", 0.0), ("nan", 0.0),
                     ("inf", 0.0), ("1 2", 0.0), ("", 0.0)):
    f.write_text(text)
    assert read_announce_brake() == want, text
  f.unlink()
  assert read_announce_brake() == 0.0


def test_file_read_about_once_a_second(tmp_path):
  f = tmp_path / "announce_brake"
  f.write_text("0")
  carcontroller.ANNOUNCE_BRAKE_FILE = str(f)
  ssm = MQBStandstillManager(CCP)
  frame(ssm, 1.2, -0.5, "pid")
  f.write_text("1.5")
  for _ in range(carcontroller.ANNOUNCE_BRAKE_PERIOD - 2):
    frame(ssm, 1.2, -0.5, "pid")
  assert ssm._ab_rate == 0.0
  frame(ssm, 1.2, -0.5, "pid")
  frame(ssm, 1.2, -0.5, "pid")
  assert ssm._ab_rate == 1.5


def test_real_packer(tmp_path):
  # the frames pack with the real DBC (accel and bits in range)
  from opendbc.can import CANPacker
  out = run(tmp_path, "1.5")
  packer = CANPacker("vw_mqb")
  for f in out:
    a06, a07 = mqbcan.create_acc_accel_control(packer, 0, 1, True, f["accel"], 3, f["anh06"], f["anf06"], False,
                                               f["anf07"], f["anh07"])
    assert len(a06[1]) == 8 and len(a07[1]) == 8
