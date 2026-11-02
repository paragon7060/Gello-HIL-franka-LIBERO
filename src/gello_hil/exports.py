"""Explicit successful support export. Raw attempts always remain unchanged."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from .pose_control import matrix_rotvec


def load_support(attempt, *, require_review=True):
    """Validate the raw support boundary before export or pool acceptance."""
    attempt = Path(attempt)
    meta = json.loads((attempt / "attempt.json").read_text())
    if meta.get("success") is not True or meta.get("error") or meta.get("aborted"):
        raise ValueError("failed/aborted/error attempts cannot be exported as support")
    if meta.get("controller") != "OSC_POSE" or not meta.get("position_case"):
        raise ValueError("only registered Position OSC_POSE attempts are support")
    case = meta["position_case"]
    if case["split"] != "support" or not 0 <= case["state_index"] <= 14:
        raise ValueError("development/final scenes cannot become support")
    if require_review:
        from .review import require_confirmation
        require_confirmation(attempt, meta)
    with np.load(attempt / "trajectory.npz", allow_pickle=False) as t:
        actions = t["actions"].copy()
        counts = t["counts"].copy()
        joints = t["follower_joints"].copy()
        action_times = t["action_elapsed_seconds"].copy()
    with np.load(attempt / "observations.npz", allow_pickle=False) as d:
        arrays = {k: d[k].copy() for k in d.files}
    length = len(actions)
    if length == 0 or actions.shape != (length, 7) or not np.isfinite(actions).all():
        raise ValueError("support requires actual finite 7D OSC actions")
    if np.any(np.abs(actions) > 1 + 1e-6):
        raise ValueError("actions exceed the native input range")
    if counts.shape != (length, 8) or joints.shape != (length, 7):
        raise ValueError("missing aligned encoder/follower trajectory")
    if (action_times.shape != (length,) or not np.isfinite(action_times).all()
            or not np.isfinite(joints).all() or not np.isfinite(counts).all()
            or np.any(np.diff(action_times) < 0)):
        raise ValueError("invalid trajectory timing or hardware telemetry")
    if any(len(v) != length + 1 for v in arrays.values()):
        raise ValueError("unaligned observation boundaries")
    required = {"agentview_image", "wrist_image", "state", "sim_states", "elapsed_seconds"}
    if not required.issubset(arrays):
        raise ValueError("missing observation payload")
    if arrays["state"].shape != (length + 1, 9) or not np.isfinite(arrays["state"]).all():
        raise ValueError("invalid proprioception")
    times = arrays["elapsed_seconds"]
    if (times.shape != (length + 1,) or not np.isfinite(times).all()
            or np.any(np.diff(times) < 0) or not np.isfinite(arrays["sim_states"]).all()):
        raise ValueError("invalid observation timing/state")
    if np.any(action_times < times[:-1]) or np.any(action_times > times[1:]):
        raise ValueError("action timestamps must lie between paired observations")
    for name in ("agentview_image", "wrist_image"):
        pixels = arrays[name]
        if pixels.dtype != np.uint8 or pixels.ndim != 4 or pixels.shape[-1] != 3:
            raise ValueError("invalid RGB observations")
    return meta, case, actions, arrays


def export_support(attempt, output, *, format="native-npz"):
    attempt, output = Path(attempt), Path(output)
    meta, case, actions, arrays = load_support(attempt)
    length = len(actions)
    if format not in ("native-npz", "oat-hdf5"):
        raise ValueError("unknown support export format")
    sidecar = output.with_suffix(output.suffix + ".json")
    if output.exists() or sidecar.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if format == "native-npz":
        # Matches the existing GR00T native support adapter's 180-degree RGB convention.
        with output.open("xb") as f:
            np.savez_compressed(f, image=arrays["agentview_image"][:-1, ::-1, ::-1],
                                wrist_image=arrays["wrist_image"][:-1, ::-1, ::-1],
                                state=arrays["state"][:-1], actions=actions,
                                initial_sim_state=arrays["sim_states"][0])
    else:
        import h5py
        from . import __version__
        state = arrays["state"][:-1]
        quat = state[:, 3:7].astype(float)
        if np.any(np.linalg.norm(quat, axis=1) < 1e-8):
            raise ValueError("invalid quaternion")
        quat /= np.linalg.norm(quat, axis=1, keepdims=True)
        rotations = []
        for x, y, z, w in quat:
            matrix = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                               [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                               [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
            rotations.append(matrix_rotvec(matrix))
        with h5py.File(output, "x") as f:
            group = f.create_group("data")
            group.attrs.update(num_demos=1, bddl_file_name=case["task_name"] + ".bddl",
                               collection_method="human_gello", exporter_version=__version__)
            demo = group.create_group("demo_0")
            demo.create_dataset("actions", data=actions)
            obs = demo.create_group("obs")
            # OAT's existing loader applies a vertical flip on reading these raw pixels.
            obs.create_dataset("agentview_rgb", data=arrays["agentview_image"][:-1], compression="gzip")
            obs.create_dataset("eye_in_hand_rgb", data=arrays["wrist_image"][:-1], compression="gzip")
            obs.create_dataset("ee_pos", data=state[:, :3])
            obs.create_dataset("ee_ori", data=np.asarray(rotations, dtype=np.float32))
            obs.create_dataset("gripper_states", data=state[:, 7:9])
    manifest = {"schema_version": "gello-support-export-v1", "format": format,
                "attempt": str(attempt.resolve()), "position_case": case,
                "instruction": meta["instruction"], "source": meta["source"],
                "calibration": meta["calibration"], "controller_contract": meta["controller_contract"],
                "actions": length, "normalization": "none_native_actions",
                "output_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
                "events_file": str((attempt / "attempt.json").resolve()),
                "training_input_excludes": ["sim_states", "success", "events", "motor_telemetry"]}
    with sidecar.open("x") as f:
        f.write(json.dumps(manifest, indent=2) + "\n")
    return manifest
