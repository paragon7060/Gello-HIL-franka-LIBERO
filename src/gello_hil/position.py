"""Explicit LIBERO-Pro Position cases; no collection scheduler or automatic retries."""
from __future__ import annotations

import hashlib
import inspect
import json
import re
import subprocess
import sys
from pathlib import Path

import numpy as np


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def resolve_case(plan_path, task_key, draw, shot):
    plan = json.loads(Path(plan_path).read_text())
    schema = plan.get("schema_version")
    if schema == "gello-position-pool-v2":
        if draw is not None or type(shot) is not int or not 1 <= shot <= 10:
            raise ValueError("10-demo pool requires shot 1..10 and no --draw")
        if plan.get("support_init_indices") != list(range(10)):
            raise ValueError("10-demo pool must use native support init0..9")
        state_index = shot - 1
    elif schema == "gello-position-support-v1":
        if draw is None or not 1 <= draw <= 3 or not 1 <= shot <= 5:
            raise ValueError("legacy support requires draw 1..3 and shot 1..5")
        state_index = (draw - 1) * 5 + shot - 1
    else:
        raise ValueError("unknown Position plan")
    tasks = {t["key"]: t for t in plan["tasks"]}
    if len(tasks) != len(plan["tasks"]) or task_key not in tasks:
        raise ValueError("unknown or duplicated task key")
    task = tasks[task_key]
    if task["suite"] not in ("libero_spatial_swap", "libero_object_swap",
                             "libero_goal_swap", "libero_10_swap"):
        raise ValueError("Position tasks require official _swap suites")
    return {**task, "draw": draw, "shot": shot, "state_index": state_index,
            "split": "support", "benchmark": "libero_pro", "collection_method": "human_gello",
            "plan_schema": schema,
            "plan_sha256": sha256(plan_path),
            "expected_benchmark_commit": plan["expected_benchmark_commit"]}


def make_position_env(args):
    from .libero_runtime import make_env

    root = Path(args.libero_source).resolve()
    revision = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    if revision != args.position_case["expected_benchmark_commit"]:
        raise ValueError("LIBERO-Pro commit differs from the registered plan")
    if subprocess.check_output(["git", "-C", str(root), "diff", "HEAD", "--"], text=True):
        raise ValueError("LIBERO-Pro has tracked source changes")
    sys.path.insert(0, str(root))
    from libero.libero import benchmark, get_libero_path
    if not Path(inspect.getfile(benchmark)).resolve().is_relative_to(root):
        raise ValueError("a different LIBERO package is already imported")
    suite = benchmark.get_benchmark_dict()[args.suite]()
    task = suite.get_task(args.task_id)
    if task.name != args.position_case["task_name"]:
        raise ValueError("official task identity differs from plan")
    bddl = Path(get_libero_path("bddl_files")) / task.problem_folder / task.bddl_file
    match = re.search(r"\(:language\s+([^)]*)\)", bddl.read_text(), flags=re.S)
    if match is None:
        raise ValueError("native BDDL instruction is missing")
    instruction = " ".join(match[1].split()).strip('"')
    if not instruction:
        raise ValueError("native instruction is empty")
    env, obs, _ = make_env(args.suite, args.task_id, args.state_index,
                          resolution=args.resolution, seed=args.seed, controller="OSC_POSE",
                          horizon=args.max_active_steps + 10)
    try:
        source_state = np.asarray(suite.get_task_init_states(args.task_id)[args.state_index])
        # Match the existing native post-warmup boundary; no object/fixture translation.
        for _ in range(10):
            obs, *_ = env.step([0, 0, 0, 0, 0, 0, -1])
        state = np.asarray(env.get_sim_state()).copy()
        obs = env.regenerate_obs_from_state(state)
        initial = np.asarray(env.get_sim_state()).copy()
        env.env.timestep = 0
        env.env.done = False
        env.env.robots[0].controller.reset_goal()
        if not np.array_equal(initial, env.get_sim_state()) or env.check_success():
            raise ValueError("invalid native support initial state")
        args.position_case.update(
            benchmark_commit=revision, bddl_sha256=sha256(bddl), native_instruction=instruction,
            init_file_sha256=sha256(Path(get_libero_path("init_states")) / task.problem_folder / task.init_states_file),
            initial_source_sha256=hashlib.sha256(source_state.tobytes()).hexdigest(),
            initial_state_sha256=hashlib.sha256(initial.tobytes()).hexdigest(), warmup_steps=10,
            controller="OSC_POSE", resolution=args.resolution,
        )
        return env, obs, instruction
    except BaseException:
        env.close()
        raise
