"""Official-style GELLO quarter-turn calibration and Panda joint mapping."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from .dynamixel import COUNTS_PER_REVOLUTION

SCHEMA_VERSION = "gello-hil-franka-calibration-v1"
PANDA_REFERENCE_POSE = (0.0, 0.0, 0.0, -np.pi / 2.0, 0.0, np.pi / 2.0, 0.0)
PANDA_JOINT_LIMITS = (
    (-2.8973, 2.8973),
    (-1.7628, 1.7628),
    (-2.8973, 2.8973),
    (-3.0718, -0.0698),
    (-2.8973, 2.8973),
    (-0.0175, 3.7525),
    (-2.8973, 2.8973),
)


def modular_count_delta(current, reference) -> np.ndarray:
    current = np.asarray(current, dtype=np.int64)
    reference = np.asarray(reference, dtype=np.int64)
    return (current - reference + 2048) % COUNTS_PER_REVOLUTION - 2048


def counts_to_radians(counts) -> np.ndarray:
    values = np.asarray(counts, dtype=np.int64)
    if np.any(values < 0) or np.any(values >= COUNTS_PER_REVOLUTION):
        raise ValueError("encoder counts must be in [0, 4095]")
    return values.astype(np.float64) * (2.0 * np.pi / COUNTS_PER_REVOLUTION)


def normalize_joint_positions(joints, limits=PANDA_JOINT_LIMITS) -> np.ndarray:
    values = np.asarray(joints, dtype=np.float64)
    ranges = np.asarray(limits, dtype=np.float64)
    if values.shape != (7,) or ranges.shape != (7, 2):
        raise ValueError("expected seven joints and 7x2 limits")
    midpoints = ranges.mean(axis=1)
    return (values - midpoints + np.pi) % (2.0 * np.pi) - np.pi + midpoints


def infer_quarter_turn_offsets(counts, reference_pose, signs, limits=PANDA_JOINT_LIMITS) -> dict:
    counts = np.asarray(counts, dtype=np.int64)
    reference = np.asarray(reference_pose, dtype=np.float64)
    signs = np.asarray(signs, dtype=np.int64)
    ranges = np.asarray(limits, dtype=np.float64)
    if counts.shape != reference.shape or counts.shape != signs.shape or counts.shape != (7,):
        raise ValueError("calibration requires seven counts, reference joints, and signs")
    if np.any(~np.isin(signs, (-1, 1))):
        raise ValueError("joint signs must be -1 or +1")
    raw = counts_to_radians(counts)
    offsets = np.arange(4, dtype=np.float64) * (np.pi / 2.0)
    chosen = np.empty(7, dtype=np.float64)
    mapped = np.empty(7, dtype=np.float64)
    residual = np.empty(7, dtype=np.float64)
    for joint in range(7):
        candidates = []
        for offset in offsets:
            probe = np.zeros(7, dtype=np.float64)
            probe[joint] = (raw[joint] - offset) * signs[joint]
            candidates.append(normalize_joint_positions(probe, ranges)[joint])
        candidates = np.asarray(candidates)
        index = int(np.argmin(np.abs(reference[joint] - candidates)))
        chosen[joint] = offsets[index]
        mapped[joint] = candidates[index]
        residual[joint] = reference[joint] - mapped[joint]
    return {
        "assembly_offsets_radians": chosen,
        "mapped_joint_positions": mapped,
        "calibration_residual_radians": residual,
        "required_count_deltas": np.rint(
            signs * residual * COUNTS_PER_REVOLUTION / (2.0 * np.pi)
        ).astype(np.int64),
        "max_abs_calibration_residual_rad": float(np.abs(residual).max()),
    }


@dataclass(frozen=True)
class GelloCalibration:
    arm_ids: tuple[int, ...]
    gripper_id: int
    calibration_counts: tuple[int, ...]
    reference_pose: tuple[float, ...]
    assembly_offsets_radians: tuple[float, ...]
    joint_signs: tuple[int, ...]
    joint_limits: tuple[tuple[float, float], ...]
    gripper_open_count: int
    gripper_closed_count: int

    def validate(self) -> None:
        if {
            len(self.arm_ids),
            len(self.calibration_counts),
            len(self.reference_pose),
            len(self.assembly_offsets_radians),
            len(self.joint_signs),
            len(self.joint_limits),
        } != {7}:
            raise ValueError("calibration must describe exactly seven Panda arm joints")
        if len(set(self.arm_ids + (self.gripper_id,))) != 8:
            raise ValueError("all motor IDs must be unique")
        if any(sign not in (-1, 1) for sign in self.joint_signs):
            raise ValueError("joint signs must be -1 or +1")
        if any(not isinstance(i, (int, np.integer)) or not 0 <= i <= 252
               for i in self.arm_ids + (self.gripper_id,)):
            raise ValueError("motor IDs must be integers in 0..252")
        ranges = np.asarray(self.joint_limits, dtype=float)
        if (ranges.shape != (7, 2) or not np.isfinite(ranges).all()
                or np.any(ranges[:, 1] <= ranges[:, 0])
                or not np.isfinite(self.reference_pose).all()
                or not np.isfinite(self.assembly_offsets_radians).all()):
            raise ValueError("calibration ranges, offsets and reference must be finite")
        counts = self.calibration_counts + (self.gripper_open_count, self.gripper_closed_count)
        if any(count < 0 or count >= COUNTS_PER_REVOLUTION for count in counts):
            raise ValueError("calibration counts must be in [0, 4095]")
        if abs(int(modular_count_delta(self.gripper_closed_count, self.gripper_open_count))) < 20:
            raise ValueError("gripper calibration span is too small")
        expected = infer_quarter_turn_offsets(
            self.calibration_counts, self.reference_pose, self.joint_signs, self.joint_limits
        )["assembly_offsets_radians"]
        if not np.allclose(expected, self.assembly_offsets_radians, atol=1e-9, rtol=0):
            raise ValueError("stored quarter-turn offsets do not match the calibration capture")

    @classmethod
    def create(
        cls,
        *,
        arm_ids,
        gripper_id,
        calibration_counts,
        joint_signs,
        gripper_open_count,
        gripper_closed_count,
        reference_pose=PANDA_REFERENCE_POSE,
        joint_limits=PANDA_JOINT_LIMITS,
    ) -> GelloCalibration:
        inferred = infer_quarter_turn_offsets(
            calibration_counts, reference_pose, joint_signs, joint_limits
        )
        result = cls(
            arm_ids=tuple(int(v) for v in arm_ids),
            gripper_id=int(gripper_id),
            calibration_counts=tuple(int(v) for v in calibration_counts),
            reference_pose=tuple(float(v) for v in reference_pose),
            assembly_offsets_radians=tuple(float(v) for v in inferred["assembly_offsets_radians"]),
            joint_signs=tuple(int(v) for v in joint_signs),
            joint_limits=tuple((float(a), float(b)) for a, b in joint_limits),
            gripper_open_count=int(gripper_open_count),
            gripper_closed_count=int(gripper_closed_count),
        )
        result.validate()
        return result

    def mapped_joints(self, counts: dict[int, int] | np.ndarray) -> np.ndarray:
        if isinstance(counts, dict):
            raw = np.asarray([counts[motor_id] for motor_id in self.arm_ids])
        else:
            raw = np.asarray(counts)
        joints = counts_to_radians(raw) - np.asarray(self.assembly_offsets_radians)
        joints *= np.asarray(self.joint_signs)
        return normalize_joint_positions(joints, self.joint_limits)

    def gripper_ratio(self, counts: dict[int, int]) -> float:
        total = float(modular_count_delta(self.gripper_closed_count, self.gripper_open_count))
        current = float(modular_count_delta(counts[self.gripper_id], self.gripper_open_count))
        return float(np.clip(current / total, 0.0, 1.0))

    @property
    def joint_position_limits(self):
        """Compatibility with the independently tested alignment safety core."""
        return self.joint_limits

    def mapped_joint_positions(self, counts):
        return self.mapped_joints(counts)

    def alignment_diagnostics(self, counts, follower_joints, *, tolerance_rad):
        target = np.asarray(follower_joints, dtype=np.float64)
        if target.shape != (7,) or not np.isfinite(target).all():
            raise ValueError("alignment target must contain seven finite joints")
        if not np.isfinite(tolerance_rad) or tolerance_rad <= 0:
            raise ValueError("alignment tolerance must be finite and positive")
        mapped = self.mapped_joints(counts)
        error = target - mapped
        return {
            "mapped_leader_joint_positions": mapped,
            "alignment_error_radians": error,
            "required_count_deltas": np.rint(
                np.asarray(self.joint_signs) * error * COUNTS_PER_REVOLUTION / (2 * np.pi)
            ).astype(np.int64),
            "alignment_ready": bool(np.max(np.abs(error)) <= tolerance_rad),
        }

    def to_json(self, path: Path) -> None:
        payload = {"schema_version": SCHEMA_VERSION, **asdict(self)}
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    @classmethod
    def from_json(cls, path: Path) -> GelloCalibration:
        payload = json.loads(path.read_text(encoding="utf-8"))
        if payload.pop("schema_version", None) != SCHEMA_VERSION:
            raise ValueError("unsupported calibration schema")
        result = cls(
            arm_ids=tuple(payload["arm_ids"]),
            gripper_id=payload["gripper_id"],
            calibration_counts=tuple(payload["calibration_counts"]),
            reference_pose=tuple(payload["reference_pose"]),
            assembly_offsets_radians=tuple(payload["assembly_offsets_radians"]),
            joint_signs=tuple(payload["joint_signs"]),
            joint_limits=tuple(tuple(row) for row in payload["joint_limits"]),
            gripper_open_count=payload["gripper_open_count"],
            gripper_closed_count=payload["gripper_closed_count"],
        )
        result.validate()
        return result

    @classmethod
    def from_session_json(cls, path: Path) -> GelloCalibration:
        """Explicit import of the user's historical D31 capture, never a new calibration."""
        payload = json.loads(path.read_text(encoding="utf-8"))
        if payload.get("schema_version") != "d31-gello-session-discrete-assembly-offset-calibration-v15":
            raise ValueError("unsupported historical session calibration")
        result = cls.create(
            arm_ids=payload["arm_ids"], gripper_id=payload["gripper_id"],
            calibration_counts=payload["calibration_counts"],
            joint_signs=payload["joint_signs"],
            reference_pose=payload["calibration_pose_joint_positions"],
            joint_limits=payload["joint_position_limits"],
            gripper_open_count=payload["gripper_open_count"],
            gripper_closed_count=payload["gripper_closed_count"],
        )
        if not np.allclose(result.assembly_offsets_radians,
                           payload["assembly_offsets_radians"], atol=1e-9, rtol=0):
            raise ValueError("historical capture and offsets disagree")
        return result
