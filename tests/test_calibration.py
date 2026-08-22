import numpy as np

from gello_hil.calibration import (
    PANDA_REFERENCE_POSE,
    GelloCalibration,
    normalize_joint_positions,
)


def calibration():
    return GelloCalibration.create(
        arm_ids=range(1, 8),
        gripper_id=8,
        calibration_counts=[0, 0, 0, 1024, 0, 1024, 0],
        joint_signs=[1, 1, 1, -1, 1, 1, 1],
        gripper_open_count=100,
        gripper_closed_count=900,
    )


def test_calibration_capture_maps_back_to_reference_pose():
    cal = calibration()
    mapped = cal.mapped_joints(np.asarray(cal.calibration_counts))
    np.testing.assert_allclose(mapped, PANDA_REFERENCE_POSE, atol=1e-12)


def test_normalization_uses_asymmetric_panda_joint_midpoints():
    values = np.asarray([0, 0, 0, 3 * np.pi / 2, 0, -3 * np.pi / 2, 0])
    normalized = normalize_joint_positions(values)
    assert -3.0718 <= normalized[3] <= -0.0698
    assert -0.0175 <= normalized[5] <= 3.7525


def test_json_roundtrip(tmp_path):
    original = calibration()
    path = tmp_path / "calibration.json"
    original.to_json(path)
    assert GelloCalibration.from_json(path) == original
