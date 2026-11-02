"""Thin, optional LIBERO adapter imported only by simulation commands."""

from __future__ import annotations

import os

import numpy as np


def make_env(suite_name: str, task_id: int, state_index: int, *, resolution: int, seed: int,
             controller="JOINT_POSITION", horizon=1000):
    from libero.libero import benchmark, get_libero_path
    from libero.libero.envs import OffScreenRenderEnv

    suites = benchmark.get_benchmark_dict()
    if suite_name not in suites:
        raise ValueError(f"unknown LIBERO suite {suite_name!r}; choose one of {sorted(suites)}")
    suite = suites[suite_name]()
    task = suite.get_task(int(task_id))
    initial_states = suite.get_task_init_states(int(task_id))
    if state_index < 0 or state_index >= len(initial_states):
        raise IndexError(f"state-index must be in [0, {len(initial_states) - 1}]")
    bddl = os.path.join(get_libero_path("bddl_files"), task.problem_folder, task.bddl_file)
    env = OffScreenRenderEnv(
        bddl_file_name=bddl,
        controller=controller,
        camera_heights=int(resolution),
        camera_widths=int(resolution),
        horizon=horizon,
        ignore_done=True,
    )
    try:
        env.seed(int(seed))
        env.reset()
        obs = env.set_init_state(initial_states[int(state_index)])
        return env, obs, str(task.language)
    except BaseException:
        env.close()
        raise


def follower_joints(env) -> np.ndarray:
    robot = env.env.robots[0]
    return np.asarray(env.sim.data.qpos[robot._ref_joint_pos_indexes], dtype=np.float64).copy()


def rgb_frame(obs) -> np.ndarray:
    # LIBERO / OpenVLA convention rotates raw robosuite camera pixels by 180 degrees.
    return np.asarray(obs["agentview_image"], dtype=np.uint8)[::-1, ::-1].copy()


def wrist_frame(obs) -> np.ndarray | None:
    """Eye-in-hand camera in the same 180-degree convention, for the operator only."""
    if "robot0_eye_in_hand_image" not in obs:
        return None
    return np.asarray(obs["robot0_eye_in_hand_image"], dtype=np.uint8)[::-1, ::-1].copy()


def render_joint_pose(env, joints) -> np.ndarray:
    """Render a Panda pose without a control action or physics step."""

    robot = env.env.robots[0]
    values = np.asarray(joints, dtype=np.float64)
    if values.shape != (7,) or not np.isfinite(values).all():
        raise ValueError("render pose must contain seven finite joints")
    limits = np.asarray(env.sim.model.jnt_range[robot._ref_joint_indexes], dtype=np.float64)
    env.sim.data.qpos[robot._ref_joint_pos_indexes] = np.clip(values, limits[:, 0], limits[:, 1])
    env.sim.data.qvel[robot._ref_joint_vel_indexes] = 0.0
    env.sim.forward()
    env._update_observables(force=True)
    return rgb_frame(env.env._get_observations())
