"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
from openpilot.common.transformations.camera import DEVICE_CAMERAS
class CameraOffsetHelper:
  def __init__(self):
    self.camera_offset = 0.0
    self.actual_camera_offset = 0.0
  @staticmethod
  def apply_camera_offset(model_transform, intrinsics, height, offset_param):
    cy = intrinsics[1, 2]
    shear = np.eye(3, dtype=np.float32)
    shear[0, 1] = offset_param / height
    shear[0, 2] = -offset_param / height * cy
    model_transform = (shear @ model_transform).astype(np.float32)
    return model_transform
  @staticmethod
  def apply_yaw_correction(model_transform, intrinsics, yaw_rad):
    # Uniform horizontal pixel shift = small yaw rotation about vertical axis.
    # Unlike shear, this preserves curve geometry — correct fix for speed-dependent
    # yaw parallax from off-center mount.
    fx = intrinsics[0, 0]
    shift = np.eye(3, dtype=np.float32)
    shift[0, 2] = yaw_rad * fx
    return (shift @ model_transform).astype(np.float32)
  def set_offset(self, offset):
    self.camera_offset = offset
  def update(self, model_transform_main, model_transform_extra, sm, main_wide_camera):
    self.actual_camera_offset = (0.9 * self.actual_camera_offset) + (0.1 * self.camera_offset)
    dc = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['roadCameraState'].sensor))]
    height = sm["liveCalibration"].height[0] if sm['liveCalibration'].height else 1.22
    # Speed-dependent YAW correction (replaces shear-based speed correction).
    # 0 rad at 0mph, +0.0055 rad (~0.32 deg) at 35 m/s (78mph).
    # Equivalent lateral effect ~0.25m at 45m lookahead, but curve-safe.
    try:
      v_ego = sm['carState'].vEgo
    except Exception:
      v_ego = 0.0
    # Quadratic ramp: gentle at 60mph (curve-safe), stronger at 75mph+ where left drift concentrates.
    # norm^2 keeps low-speed correction small while boosting high-speed correction.
    _norm = float(np.clip(v_ego / 35.0, 0.0, 1.0))
    speed_yaw = (_norm ** 2) * -0.0075
    intrinsics_main = dc.ecam.intrinsics if main_wide_camera else dc.fcam.intrinsics
    model_transform_main = self.apply_camera_offset(model_transform_main, intrinsics_main, height, self.actual_camera_offset)
    model_transform_main = self.apply_yaw_correction(model_transform_main, intrinsics_main, speed_yaw)
    intrinsics_extra = dc.ecam.intrinsics
    model_transform_extra = self.apply_camera_offset(model_transform_extra, intrinsics_extra, height, self.actual_camera_offset)
    model_transform_extra = self.apply_yaw_correction(model_transform_extra, intrinsics_extra, speed_yaw)
    return model_transform_main, model_transform_extra
