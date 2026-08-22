"""Pure joint-target mapping for robosuite JOINT_POSITION control."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .calibration import GelloCalibration, modular_count_delta
from .dynamixel import COUNTS_PER_REVOLUTION


@dataclass(frozen=True)
class ControlConfig:
    action_scale_rad: float = 0.20
    joint_limit_margin_rad: float = 0.02
    warning_lag_rad: float = 0.25
    gripper_close_ratio: float = 0.5
    maximum_continuity_error_rad: float = 0.05

    def validate(self) -> None:
        if self.action_scale_rad <= 0 or self.joint_limit_margin_rad < 0:
            raise ValueError("invalid action scale or joint-limit margin")
        if self.warning_lag_rad <= 0 or self.maximum_continuity_error_rad <= 0:
            raise ValueError("lag and continuity thresholds must be positive")


class TargetDiscontinuity(RuntimeError):
    def __init__(self, diagnostic: dict):
        self.diagnostic = diagnostic
        super().__init__("calibrated absolute target changed discontinuously")


class _FollowerBase:
    mode = "base"

    def __init__(self, calibration: GelloCalibration, config: ControlConfig | None = None):
        self.calibration = calibration
        self.config = config or ControlConfig()
        self.config.validate()
        self._previous_counts = None
        self._previous_raw_target = None
        self._leader_anchor = None
        self._follower_anchor = None

    @property
    def ids(self) -> tuple[int, ...]:
        return self.calibration.arm_ids + (self.calibration.gripper_id,)

    def anchor(self, counts: dict[int, int], follower_joints) -> dict:
        follower = np.asarray(follower_joints, dtype=np.float64)
        raw_counts = np.asarray([counts[i] for i in self.ids], dtype=np.int64)
        leader = self.calibration.mapped_joints(raw_counts[:7])
        if follower.shape != (7,) or not np.isfinite(follower).all():
            raise ValueError("follower joint vector must be finite with shape (7,)")
        self._previous_counts = raw_counts[:7].copy()
        self._previous_raw_target = leader.copy()
        self._leader_anchor = leader.copy()
        self._follower_anchor = follower.copy()
        return {"counts": raw_counts, "leader_joints": leader, "follower_joints": follower}

    def _finish(self, counts, current, raw_target) -> tuple[np.ndarray, dict]:
        limits = np.asarray(self.calibration.joint_limits)
        margin = self.config.joint_limit_margin_rad
        target = np.clip(raw_target, limits[:, 0] + margin, limits[:, 1] - margin)
        error = target - current
        normalized = np.clip(error / self.config.action_scale_rad, -1.0, 1.0)
        ratio = self.calibration.gripper_ratio(counts)
        gripper = 1.0 if ratio >= self.config.gripper_close_ratio else -1.0
        action = np.concatenate((normalized, [gripper]))
        return action, {
            "counts": [int(counts[i]) for i in self.ids],
            "raw_target_joint_positions": raw_target.tolist(),
            "target_joint_positions": target.tolist(),
            "current_follower_joint_positions": current.tolist(),
            "joint_errors": error.tolist(),
            "max_abs_joint_error_rad": float(np.abs(error).max()),
            "tracking_warning": bool(np.abs(error).max() >= self.config.warning_lag_rad),
            "target_joint_limit_clipped": (~np.isclose(raw_target, target, atol=1e-12)).tolist(),
            "normalized_arm_action": normalized.tolist(),
            "gripper_ratio": ratio,
            "gripper_action": gripper,
            "mapping_mode": self.mode,
        }


class RelativeClutchFollower(_FollowerBase):
    """Re-anchor on Enter; the first action after every clutch is exactly zero."""

    mode = "relative-clutch"

    def action(self, counts: dict[int, int], follower_joints):
        if self._previous_counts is None:
            raise RuntimeError("controller is not anchored")
        current = np.asarray(follower_joints, dtype=np.float64)
        arm_counts = np.asarray([counts[i] for i in self.calibration.arm_ids])
        delta = modular_count_delta(arm_counts, self._previous_counts)
        delta_rad = (
            np.asarray(self.calibration.joint_signs) * delta * 2.0 * np.pi / COUNTS_PER_REVOLUTION
        )
        raw_target = self._follower_anchor + delta_rad
        action, diagnostic = self._finish(counts, current, raw_target)
        diagnostic["motion_from_anchor_radians"] = delta_rad.tolist()
        return action, diagnostic


class AbsoluteJointFollower(_FollowerBase):
    """Always follow calibrated GELLO joints in Panda coordinates."""

    mode = "absolute"

    def action(self, counts: dict[int, int], follower_joints):
        if self._previous_counts is None:
            raise RuntimeError("controller is not anchored")
        current = np.asarray(follower_joints, dtype=np.float64)
        arm_counts = np.asarray([counts[i] for i in self.calibration.arm_ids])
        raw_target = self.calibration.mapped_joints(arm_counts)
        delta = modular_count_delta(arm_counts, self._previous_counts)
        expected = (
            self._previous_raw_target
            + np.asarray(self.calibration.joint_signs) * delta * 2.0 * np.pi / COUNTS_PER_REVOLUTION
        )
        residual = raw_target - expected
        diagnostic = {
            "target_continuity_error_radians": residual.tolist(),
            "max_abs_target_continuity_error_rad": float(np.abs(residual).max()),
        }
        if np.abs(residual).max() > self.config.maximum_continuity_error_rad:
            raise TargetDiscontinuity(diagnostic)
        self._previous_counts = arm_counts.copy()
        self._previous_raw_target = raw_target.copy()
        action, common = self._finish(counts, current, raw_target)
        return action, {**common, **diagnostic}
