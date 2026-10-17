import pytest
import itertools
import numpy as np

from openpilot.common.parameterized import parameterized_class
from cereal import log
from openpilot.selfdrive.car import set_min_speed
from openpilot.selfdrive.car.cruise import VCruiseHelper, V_CRUISE_MIN, V_CRUISE_MAX, V_CRUISE_INITIAL, IMPERIAL_INCREMENT, \
                                           V_CRUISE_INITIAL_EXPERIMENTAL_MODE
from cereal import car, custom
from openpilot.common.constants import CV
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
SET_MIN_DEFAULT_MPH = set_min_speed.DEFAULT_MPH


def run_cruise_simulation(cruise, e2e, personality, t_end=20.):
  man = Maneuver(
    '',
    duration=t_end,
    initial_speed=max(cruise - 1., 0.0),
    lead_relevancy=True,
    initial_distance_lead=100,
    cruise_values=[cruise],
    prob_lead_values=[0.0],
    breakpoints=[0.],
    e2e=e2e,
    personality=personality,
  )
  valid, output = man.evaluate()
  assert valid
  return output[-1, 3]


@parameterized_class(("e2e", "personality", "speed"), itertools.product(
                      [True, False], # e2e
                      log.LongitudinalPersonality.schema.enumerants, # personality
                      [5,35])) # speed
class TestCruiseSpeed:
  def test_cruise_speed(self):
    print(f'Testing {self.speed} m/s')
    cruise_speed = float(self.speed)

    simulation_steady_state = run_cruise_simulation(cruise_speed, self.e2e, self.personality)
    assert simulation_steady_state == pytest.approx(cruise_speed, abs=.01), f'Did not reach {self.speed} m/s'


# TODO: test pcmCruise and pcmCruiseSpeed
@parameterized_class(('pcm_cruise', 'pcm_cruise_speed'), [(False, True)])
class TestVCruiseHelper:
  @pytest.fixture(autouse=True)
  def stock_set_min(self, monkeypatch):
    # the other tests expect stock SET (25 mph minimum); test_set_min sets its own
    monkeypatch.setattr(set_min_speed, "LIVE_FILE", "/nonexistent/set_min_mph")
    monkeypatch.setattr(set_min_speed, "DEFAULT_MPH", set_min_speed.MIN_LIMIT)

  def setup_method(self):
    self.CP = car.CarParams(pcmCruise=self.pcm_cruise)
    self.CP_SP = custom.CarParamsSP(pcmCruiseSpeed=self.pcm_cruise_speed)
    self.v_cruise_helper = VCruiseHelper(self.CP, self.CP_SP)
    self.reset_cruise_speed_state()

  def reset_cruise_speed_state(self):
    # Two resets previous cruise speed
    for _ in range(2):
      self.v_cruise_helper.update_v_cruise(car.CarState(cruiseState={"available": False}), enabled=False, is_metric=False)

  def enable(self, v_ego, experimental_mode, dynamic_experimental_control):
    # Simulates user pressing set with a current speed
    self.v_cruise_helper.initialize_v_cruise(car.CarState(vEgo=v_ego), experimental_mode, dynamic_experimental_control)

  def test_adjust_speed(self):
    """
    Asserts speed changes on falling edges of buttons.
    """

    self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False, False)

    for btn in (ButtonType.accelCruise, ButtonType.decelCruise):
      for pressed in (True, False):
        CS = car.CarState(cruiseState={"available": True})
        CS.buttonEvents = [ButtonEvent(type=btn, pressed=pressed)]

        self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False)
        assert pressed == (self.v_cruise_helper.v_cruise_kph == self.v_cruise_helper.v_cruise_kph_last)

  def test_rising_edge_enable(self):
    """
    Some car interfaces may enable on rising edge of a button,
    ensure we don't adjust speed if enabled changes mid-press.
    """

    # NOTE: enabled is always one frame behind the result from button press in controlsd
    for enabled, pressed in ((False, False),
                             (False, True),
                             (True, False)):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.decelCruise, pressed=pressed)]
      self.v_cruise_helper.update_v_cruise(CS, enabled=enabled, is_metric=False)
      if pressed:
        self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False, False)

      # Expected diff on enabling. Speed should not change on falling edge of pressed
      assert not pressed == self.v_cruise_helper.v_cruise_kph == self.v_cruise_helper.v_cruise_kph_last

  def test_resume_in_standstill(self):
    """
    Asserts we don't increment set speed if user presses resume/accel to exit cruise standstill.
    """

    self.enable(0, False, False)

    for standstill in (True, False):
      for pressed in (True, False):
        CS = car.CarState(cruiseState={"available": True, "standstill": standstill})
        CS.buttonEvents = [ButtonEvent(type=ButtonType.accelCruise, pressed=pressed)]
        self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False)

        # speed should only update if not at standstill and button falling edge
        should_equal = standstill or pressed
        assert should_equal == (self.v_cruise_helper.v_cruise_kph == self.v_cruise_helper.v_cruise_kph_last)

  def test_set_gas_pressed(self):
    """
    Asserts pressing set while enabled with gas pressed sets
    the speed to the maximum of vEgo and current cruise speed.
    """

    for v_ego in np.linspace(0, 100, 101):
      self.reset_cruise_speed_state()
      self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False, False)

      # first decrement speed, then perform gas pressed logic
      expected_v_cruise_kph = self.v_cruise_helper.v_cruise_kph - IMPERIAL_INCREMENT
      expected_v_cruise_kph = max(expected_v_cruise_kph, v_ego * CV.MS_TO_KPH)  # clip to min of vEgo
      expected_v_cruise_kph = float(np.clip(round(expected_v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX))

      CS = car.CarState(vEgo=float(v_ego), gasPressed=True, cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.decelCruise, pressed=False)]
      self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False)

      # TODO: fix skipping first run due to enabled on rising edge exception
      if v_ego == 0.0:
        continue
      assert expected_v_cruise_kph == self.v_cruise_helper.v_cruise_kph

  def test_initialize_v_cruise(self):
    """
    Asserts allowed cruise speeds on enabling with SET.
    """

    for experimental_mode in (True, False):
      for dynamic_experimental_control in (True, False):
        for v_ego in np.linspace(0, 100, 101):
          self.reset_cruise_speed_state()
          assert not self.v_cruise_helper.v_cruise_initialized

          self.enable(float(v_ego), experimental_mode, dynamic_experimental_control)
          assert V_CRUISE_INITIAL <= self.v_cruise_helper.v_cruise_kph <= V_CRUISE_MAX
          assert self.v_cruise_helper.v_cruise_initialized

  @pytest.mark.parametrize("file_text, min_mph", [(None, 40), ("40", 40), ("35\n", 35), ("25", 25), ("10", 25),
                                                  ("99", 45), ("nan", 40), ("junk", 40)])
  def test_set_min(self, tmp_path, monkeypatch, file_text, min_mph):
    """
    Asserts SET never starts below /data/set_min_mph: SET under it gives it, above it gives vEgo,
    RES restores the earlier set speed, experimental mode keeps its own higher start.
    """
    live_file = tmp_path / "set_min_mph"
    if file_text is not None:
      live_file.write_text(file_text)
    monkeypatch.setattr(set_min_speed, "LIVE_FILE", str(live_file))
    monkeypatch.setattr(set_min_speed, "DEFAULT_MPH", SET_MIN_DEFAULT_MPH)
    min_kph = int(round(min_mph * IMPERIAL_INCREMENT))

    for v_ego_mph in (0, 5, 20, min_mph - 1, min_mph + 5, 70):
      self.reset_cruise_speed_state()
      self.enable(v_ego_mph * CV.MPH_TO_MS, False, False)
      assert self.v_cruise_helper.v_cruise_kph == max(min_kph, int(round(v_ego_mph * CV.MPH_TO_KPH)))
      assert round(self.v_cruise_helper.v_cruise_kph * CV.KPH_TO_MPH) == max(min_mph, v_ego_mph)

    # experimental mode start is higher than any minimum
    self.reset_cruise_speed_state()
    self.enable(0., True, False)
    assert self.v_cruise_helper.v_cruise_kph == V_CRUISE_INITIAL_EXPERIMENTAL_MODE

    # RES at a stop restores the earlier set speed, even below the minimum
    self.reset_cruise_speed_state()
    self.v_cruise_helper.v_cruise_kph = 30 * IMPERIAL_INCREMENT
    self.v_cruise_helper.v_cruise_kph_last = 30 * IMPERIAL_INCREMENT
    CS = car.CarState(vEgo=0.)
    CS.buttonEvents = [ButtonEvent(type=ButtonType.resumeCruise)]
    self.v_cruise_helper.initialize_v_cruise(CS, False, False)
    assert self.v_cruise_helper.v_cruise_kph == 30 * IMPERIAL_INCREMENT
