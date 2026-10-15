"""Candidate MQBStandstillManager variants for hill hold v2 (design study only, never committed).

Each is a drop-in class with the same update() signature as the parent's manager, so the parent's ACC block
(CarController.update) can run it unchanged. The only new car input is CS.esp_stopping (ESP_21 bit 63,
ESP_Anhaltevorgang_ACC_aktiv), used by candidate B only.
"""

COMMON_FLOOR = '''
    # Launch accel floor, unchanged from the parent (dead on this car: needs bit 53).
    if long_active and accel > 0 and self._prev_hold_confirmation and (CS.out.standstill or CS.out.vEgo < 1.0):
      accel = min(max(accel, self.UPHILL_ACCEL_FLOOR), self._CCP.ACCEL_MAX)
'''

# ---------------------------------------------------------------------------------------------- A
# Early stop announcement (timed): one 20-frame HMS=1 window per low-speed episode, started by SPEED
# (vEgo < 1.5 while braking), never below 0.5 m/s, then HMS=4 latched until vEgo > 3.0 or long off.
CAND_A = '''
class MQBStandstillManager:
  ANNOUNCE_SPEED = 1.5      # m/s: send the ESP stop announcement below this, while still moving
  ANNOUNCE_MIN_SPEED = 0.5  # m/s: never start or continue an announcement slower than this
  ANNOUNCE_FRAMES = 20      # 0.4 s at 50 Hz: the window that preceded every fault-free hold (routes 332/333)
  RELEASE_SPEED = 2.6       # m/s (9.5 km/h, RJ's ESP_OVERRIDE_SPEED): forget the episode once driving again
  UPHILL_ACCEL_FLOOR = 1.5

  def __init__(self, CCP):
    self._CCP = CCP
    self._prev_hold_confirmation = False
    self._announce_frames = 0
    self._hms4_latched = False
    self._hold_request_frames = 0

  def update(self, CS, long_active, accel, stopping, starting):
    esp_starting_override = None
    esp_stopping_override = None
    hms_active = long_active
    if CS.out.brakePressed:
      long_active = False
''' + COMMON_FLOOR + '''
    v = CS.out.vEgo
    if not hms_active or v > self.RELEASE_SPEED:
      self._hms4_latched = False
      self._announce_frames = 0
    if not hms_active:
      stopping = starting = False   # OP long off: HMS 0, never one stray HMS 1 frame (lcs lags a frame on brake)

    if hms_active:
      if self._hms4_latched:
        esp_starting_override, esp_stopping_override = True, False          # HMS 4, ACC_07 Anfahren
      elif self._announce_frames > 0 or (v < self.ANNOUNCE_SPEED and (accel < 0 or stopping)):
        if self._announce_frames >= self.ANNOUNCE_FRAMES or v < self.ANNOUNCE_MIN_SPEED:
          self._hms4_latched = True
          esp_starting_override, esp_stopping_override = True, False
        else:
          self._announce_frames += 1
          esp_starting_override, esp_stopping_override = False, True        # HMS 1, ACC_07 Anhalten, d 0.3

    self._hold_request_frames = self._announce_frames
    self._prev_hold_confirmation = CS.esp_hold_confirmation
    return long_active, accel, stopping, starting, esp_starting_override, esp_stopping_override
'''

# ---------------------------------------------------------------------------------------------- B
# RJ handshake: same trigger as A, but the announcement ends on the first frame the ESP reports its
# stopping procedure (ESP_21 bit 63), capped at 20 frames. Needs CS.esp_stopping.
CAND_B = CAND_A.replace(
  "        if self._announce_frames >= self.ANNOUNCE_FRAMES or v < self.ANNOUNCE_MIN_SPEED:",
  "        if CS.esp_stopping or self._announce_frames >= self.ANNOUNCE_FRAMES or v < self.ANNOUNCE_MIN_SPEED:")
assert CAND_B != CAND_A

# ---------------------------------------------------------------------------------------------- C
# lcs-triggered announcement (the parent's trigger) made safe: 20 frames of HMS=1 from the first stopping
# frame at ANY speed, but only if vEgo >= 0.3 then (else latch HMS 4 at once, like hold_tsk_owned);
# ACC_06 held in Anfahren with the negative accel during the window (RJ: TSK keeps brake torque);
# HMS 4 latched until vEgo > 3.0 or long off (no 1<->4 cycling around 1.0 m/s).
CAND_C = '''
class MQBStandstillManager:
  ANNOUNCE_FRAMES = 20
  MIN_ANNOUNCE_SPEED = 0.3
  RELEASE_SPEED = 3.0
  UPHILL_ACCEL_FLOOR = 1.5

  def __init__(self, CCP):
    self._CCP = CCP
    self._prev_hold_confirmation = False
    self._announce_frames = 0
    self._hms4_latched = False

  def update(self, CS, long_active, accel, stopping, starting):
    esp_starting_override = None
    esp_stopping_override = None
    hms_active = long_active
    if CS.out.brakePressed:
      long_active = False
''' + COMMON_FLOOR + '''
    v = CS.out.vEgo
    if not hms_active or v > self.RELEASE_SPEED:
      self._hms4_latched = False
      self._announce_frames = 0

    if hms_active:
      if self._hms4_latched:
        esp_starting_override, esp_stopping_override = True, False
      elif self._announce_frames > 0 or stopping:
        if self._announce_frames >= self.ANNOUNCE_FRAMES or (self._announce_frames == 0 and v < self.MIN_ANNOUNCE_SPEED):
          self._hms4_latched = True
          esp_starting_override, esp_stopping_override = True, False
        else:
          self._announce_frames += 1
          esp_starting_override, esp_stopping_override = False, True
          stopping, starting = False, True        # ACC_06 Anfahren + negative accel during the window (RJ)

    self._prev_hold_confirmation = CS.esp_hold_confirmation
    return long_active, accel, stopping, starting, esp_starting_override, esp_stopping_override
'''

# ---------------------------------------------------------------------------------------------- D
# Latch HMS 4 only at true standstill: the parent's HMS=1 request runs from lcs=stopping until the wheels
# read 0 (CS.out.standstill or vEgo < 0.05), no 20-frame timeout; then HMS 4 latched as in the parent.
CAND_D = '''
class MQBStandstillManager:
  UPHILL_ACCEL_FLOOR = 1.5

  def __init__(self, CCP):
    self._CCP = CCP
    self._prev_hold_confirmation = False
    self._hold_request_frames = 0
    self._hms4_latched = False

  def update(self, CS, long_active, accel, stopping, starting):
    esp_starting_override = None
    esp_stopping_override = None
    hms_active = long_active
    if CS.out.brakePressed:
      long_active = False
''' + COMMON_FLOOR + '''
    v = CS.out.vEgo
    at_standstill = CS.out.standstill or v < 1.0
    true_still = CS.out.standstill or v < 0.05
    if not at_standstill or starting:
      self._hms4_latched = False

    if hms_active and at_standstill and self._hms4_latched and not starting:
      esp_starting_override, esp_stopping_override = True, False
    elif hms_active and at_standstill and stopping and not starting:
      if true_still:
        esp_starting_override, esp_stopping_override = True, False
        self._hms4_latched = True
      else:
        self._hold_request_frames += 1          # raw stopping -> HMS 1 until the wheels read 0
    elif hms_active and at_standstill and starting:
      esp_starting_override, esp_stopping_override = True, False
      self._hold_request_frames = 0
    else:
      self._hold_request_frames = 0

    self._prev_hold_confirmation = CS.esp_hold_confirmation
    return long_active, accel, stopping, starting, esp_starting_override, esp_stopping_override
'''

# ---------------------------------------------------------------------------------------------- E
# Announcement only at true standstill: while rolling in the stopping state send HMS 4 (no announcement);
# once the wheels read 0 send 20 frames of HMS=1, then HMS 4 latched until departure (parent latch rules).
CAND_E = '''
class MQBStandstillManager:
  HOLD_REQUEST_TIMEOUT = 20
  UPHILL_ACCEL_FLOOR = 1.5

  def __init__(self, CCP):
    self._CCP = CCP
    self._prev_hold_confirmation = False
    self._hold_request_frames = 0
    self._hms4_latched = False

  def update(self, CS, long_active, accel, stopping, starting):
    esp_starting_override = None
    esp_stopping_override = None
    hms_active = long_active
    if CS.out.brakePressed:
      long_active = False
''' + COMMON_FLOOR + '''
    v = CS.out.vEgo
    at_standstill = CS.out.standstill or v < 1.0
    true_still = CS.out.standstill or v < 0.05
    if not at_standstill or starting:
      self._hms4_latched = False

    if hms_active and at_standstill and self._hms4_latched and not starting:
      esp_starting_override, esp_stopping_override = True, False
    elif hms_active and at_standstill and stopping and not starting:
      if not true_still and self._hold_request_frames == 0:
        esp_starting_override, esp_stopping_override = True, False     # rolling: HMS 4, no announcement yet
      elif self._hold_request_frames >= self.HOLD_REQUEST_TIMEOUT:
        esp_starting_override, esp_stopping_override = True, False
        self._hms4_latched = True
      else:
        self._hold_request_frames += 1                                  # raw stopping -> HMS 1 at standstill
    elif hms_active and at_standstill and starting:
      esp_starting_override, esp_stopping_override = True, False
      self._hold_request_frames = 0
    else:
      self._hold_request_frames = 0

    self._prev_hold_confirmation = CS.esp_hold_confirmation
    return long_active, accel, stopping, starting, esp_starting_override, esp_stopping_override
'''

CANDIDATES = {"A": CAND_A, "B": CAND_B, "C": CAND_C, "D": CAND_D, "E": CAND_E}
