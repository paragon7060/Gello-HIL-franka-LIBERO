"""Operator-driven calibration using the original read-only capture procedure."""

from __future__ import annotations

import json
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from .calibration import (
    SCHEMA_VERSION,
    GelloCalibration,
    PANDA_REFERENCE_POSE,
    infer_quarter_turn_offsets,
)
from .review import atomic_json


def selected_calibration(root, explicit=None, fallback=None):
    if explicit:
        return str(Path(explicit).expanduser().resolve())
    settings = Path(root) / "gui-settings.json"
    if settings.is_file():
        return json.loads(settings.read_text()).get("calibration") or fallback
    return fallback


def remember_calibration(root, path):
    path = Path(path).expanduser().resolve()
    GelloCalibration.from_json(path)
    atomic_json(Path(root) / "gui-settings.json", {"calibration": str(path)}, replace=True)
    return str(path)


class CalibrationCapture:
    """Three explicit captures, no motor writes, no overwrite of existing files."""

    def __init__(self, args, signs, *, bus_factory=None):
        from .cli import _bus

        self.args = args
        self.signs = tuple(int(s) for s in signs)
        if len(self.signs) != 7 or any(s not in (-1, 1) for s in self.signs):
            raise ValueError("Choose one sign (+1 or -1) for each of the seven joints.")
        self.bus_factory = bus_factory or _bus
        self.rows = []
        self.residual = None

    def capture_next(self):
        from .cli import _sample_positions

        if len(self.rows) == 3:
            raise ValueError("All captures are complete. Save or restart calibration.")
        if not self.args.port:
            raise ValueError("Enter the GELLO serial port in Connection first.")
        with self.bus_factory(self.args) as bus:
            row = _sample_positions(bus, tuple(range(1, 9)), 25, 0.02)
        if not self.rows:
            inferred = infer_quarter_turn_offsets(
                [row[i] for i in range(1, 8)], PANDA_REFERENCE_POSE, self.signs
            )
            if inferred["max_abs_calibration_residual_rad"] > 0.60:
                raise ValueError(
                    "Reference pose residual exceeds 0.60 rad. Reposition and capture again."
                )
            self.residual = np.asarray(inferred["calibration_residual_radians"]).tolist()
        if len(self.rows) == 2:
            # Validate the gripper span before accepting the final capture.
            self._build([*self.rows, row])
        self.rows.append(row)
        return row

    def _build(self, rows):
        return GelloCalibration.create(
            arm_ids=tuple(range(1, 8)),
            gripper_id=8,
            calibration_counts=[rows[0][i] for i in range(1, 8)],
            joint_signs=self.signs,
            gripper_open_count=rows[1][8],
            gripper_closed_count=rows[2][8],
        )

    def save(self, path):
        from .recording import source_provenance

        if len(self.rows) != 3:
            raise ValueError(
                "Capture reference pose, open gripper and closed gripper before saving."
            )
        calibration = self._build(self.rows)
        payload = {
            "schema_version": SCHEMA_VERSION,
            **asdict(calibration),
            "capture_provenance": {
                "created_at_utc": datetime.now(timezone.utc).isoformat(),
                "method": "original_quarter_turn_reference_open_closed",
                "samples_per_step": 25,
                "sample_interval_seconds": 0.02,
                "max_reference_residual_rad": 0.60,
                "reference_residual_radians": self.residual,
                "capture_counts": self.rows,
                "motor_register_writes": 0,
                "requires_physical_direction_and_pose_check": True,
                "source": source_provenance(),
            },
        }
        path = Path(path).expanduser().resolve()
        atomic_json(path, payload)
        return str(path)
