import numpy as np

from gello_hil.calibration import GelloCalibration
from gello_hil.control import AbsoluteJointFollower, RelativeClutchFollower


def calibration():
    return GelloCalibration.create(
        arm_ids=range(1, 8),
        gripper_id=8,
        calibration_counts=[0, 0, 0, 1024, 0, 1024, 0],
        joint_signs=[1, 1, 1, -1, 1, 1, 1],
        gripper_open_count=100,
        gripper_closed_count=900,
    )


def counts(cal, arm=None):
    arm = list(cal.calibration_counts if arm is None else arm)
    return {motor_id: value for motor_id, value in zip(cal.arm_ids, arm)} | {8: 100}


def test_relative_anchor_has_zero_arm_action_and_encoder_motion_is_bounded():
    cal = calibration()
    follower = RelativeClutchFollower(cal)
    current = np.asarray(cal.reference_pose)
    base = counts(cal)
    follower.anchor(base, current)
    action, _ = follower.action(base, current)
    np.testing.assert_allclose(action[:7], 0.0)
    moved = list(cal.calibration_counts)
    moved[0] += 100
    action, _ = follower.action(counts(cal, moved), current)
    assert action[0] > 0
    assert np.abs(action).max() <= 1.0


def test_relative_reanchor_returns_to_zero_action():
    cal = calibration()
    follower = RelativeClutchFollower(cal)
    moved = list(cal.calibration_counts)
    moved[0] += 100
    current = np.asarray(cal.reference_pose).copy()
    current[0] += 0.2
    at_moved = counts(cal, moved)
    follower.anchor(at_moved, current)
    action, _ = follower.action(at_moved, current)
    np.testing.assert_allclose(action[:7], 0.0)


def test_absolute_mapping_targets_calibrated_panda_pose():
    cal = calibration()
    follower = AbsoluteJointFollower(cal)
    current = np.asarray(cal.reference_pose)
    base = counts(cal)
    follower.anchor(base, current)
    action, diagnostic = follower.action(base, current)
    np.testing.assert_allclose(action[:7], 0.0, atol=1e-12)
    np.testing.assert_allclose(diagnostic["target_joint_positions"], current, atol=1e-12)
