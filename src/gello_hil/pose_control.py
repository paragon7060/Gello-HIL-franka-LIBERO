"""GELLO joint targets -> native OSC_POSE commands, without stepping an FK simulator."""
from __future__ import annotations

import numpy as np

from .control import EnterAnchoredFollower


def matrix_rotvec(matrix):
    """Principal rotation vector, including the pi branch (no scipy dependency)."""
    m = np.asarray(matrix, dtype=np.float64)
    if m.shape != (3, 3) or not np.isfinite(m).all():
        raise ValueError("rotation must be a finite 3x3 matrix")
    if not np.allclose(m.T @ m, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(m), 1):
        raise ValueError("rotation is not in SO(3)")
    angle = np.arccos(np.clip((np.trace(m) - 1) / 2, -1, 1))
    skew = np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])
    if angle < 1e-7:
        return skew / 2
    if np.pi - angle < 1e-5:
        _, vectors = np.linalg.eigh((m + m.T) / 2)
        axis = vectors[:, -1]
        if np.dot(axis, skew) < 0:
            axis = -axis
        return angle * axis
    return angle * skew / (2 * np.sin(angle))


class PoseFollower:
    """Reuse calibrated continuity and Enter/clutch joint anchors; command EEF deltas."""
    mode = "enter-anchored-osc-pose"

    def __init__(self, calibration, kinematics, *, output_min, output_max):
        self.joint_follower = EnterAnchoredFollower(calibration)
        self.kinematics = kinematics
        self.output_min = np.broadcast_to(np.asarray(output_min, dtype=float), (6,)).copy()
        self.output_max = np.broadcast_to(np.asarray(output_max, dtype=float), (6,)).copy()
        if not np.isfinite([self.output_min, self.output_max]).all():
            raise ValueError("nonfinite controller scale")
        # Native delta controller must map neutral zero to zero motion.
        if np.any(self.output_max <= 0) or not np.allclose(self.output_min, -self.output_max):
            raise ValueError("OSC controller requires symmetric positive output bounds")
        self.contract = {"type": "OSC_POSE", "input_min": [-1]*6, "input_max": [1]*6,
                         "output_min": self.output_min.tolist(), "output_max": self.output_max.tolist(),
                         "rotation": "axis_angle_left_multiply_world_frame", "impedance": "fixed"}
        self.first_neutral = True

    def anchor(self, counts, follower_joints):
        self.first_neutral = True
        return self.joint_follower.anchor(counts, follower_joints)

    def action(self, counts, follower_joints):
        joint_action, diagnostic = self.joint_follower.action(counts, follower_joints)
        current_pos, current_rot = self.kinematics(np.asarray(follower_joints))
        target_pos, target_rot = self.kinematics(np.asarray(diagnostic["target_joint_positions"]))
        delta = np.r_[target_pos - current_pos, matrix_rotvec(target_rot @ current_rot.T)]
        if delta.shape != (6,) or not np.isfinite(delta).all():
            raise ValueError("invalid forward kinematics")
        action = np.r_[np.clip(delta / self.output_max, -1, 1), joint_action[-1]]
        if self.first_neutral:
            action[:] = 0
            self.first_neutral = False
        diagnostic.update(
            mapping_mode=self.mode, controller="OSC_POSE", eef_delta=delta.tolist(),
            action_clipped=(np.abs(delta) > self.output_max).tolist(),
            action=action.tolist(),
        )
        return action, diagnostic


class PandaKinematics:
    """Independent MuJoCo data; never edit live simulation state for FK."""

    def __init__(self, env):
        import mujoco
        self.mujoco = mujoco
        self.env = env
        self.robot = env.env.robots[0]
        self.model = getattr(env.sim.model, "_model", env.sim.model)
        self.data = mujoco.MjData(self.model)
        self.site_id = int(self.robot.eef_site_id)

    def __call__(self, joints):
        joints = np.asarray(joints, dtype=float)
        if joints.shape != (7,) or not np.isfinite(joints).all():
            raise ValueError("FK requires seven finite joints")
        self.data.qpos[:] = self.env.sim.data.qpos
        self.data.qpos[self.robot._ref_joint_pos_indexes] = joints
        self.data.qvel[:] = 0
        self.mujoco.mj_forward(self.model, self.data)
        return self.data.site_xpos[self.site_id].copy(), self.data.site_xmat[self.site_id].reshape(3, 3).copy()


def make_pose_follower(env, calibration):
    controller = env.env.robots[0].controller
    if controller.name != "OSC_POSE" or not controller.use_delta:
        raise ValueError("Position demonstrations require native delta OSC_POSE")
    if controller.control_dim != 6 or controller.impedance_mode != "fixed":
        raise ValueError("OSC_POSE must use the native fixed-impedance six-axis contract")
    if not np.allclose(controller.input_min, -1) or not np.allclose(controller.input_max, 1):
        raise ValueError("unexpected native OSC input range")
    return PoseFollower(calibration, PandaKinematics(env),
                        output_min=controller.output_min, output_max=controller.output_max)
