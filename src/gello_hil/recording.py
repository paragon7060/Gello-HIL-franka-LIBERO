"""Shared observation recording and source provenance for all experiment consumers."""
from __future__ import annotations

import hashlib
import subprocess
import time
from pathlib import Path

import numpy as np

UPSTREAM_COMMIT = "26be20212ff0709668b776ebefabe7da1809aa73"


def source_provenance():
    root = Path(__file__).resolve().parents[2]
    def git(*args):
        try:
            return subprocess.check_output(["git", "-C", str(root), *args], text=True,
                                           stderr=subprocess.DEVNULL).strip()
        except (OSError, subprocess.CalledProcessError):
            return None
    files = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
             for p in sorted((root / "src/gello_hil").glob("*.py"))}
    return {"upstream_commit": UPSTREAM_COMMIT, "commit": git("rev-parse", "HEAD"),
            "branch": git("branch", "--show-current"),
            "working_tree_dirty": bool(git("status", "--porcelain")), "source_sha256": files}


class ObservationRecorder:
    """Raw RGB/state/simulator states at T+1 boundaries, paired with T actual actions."""

    def __init__(self, *, clock=time.monotonic, started_at=None):
        self.clock = clock
        self.started = clock() if started_at is None else started_at
        self.rows = []

    def append(self, observation, sim_state):
        images = []
        for key in ("agentview_image", "robot0_eye_in_hand_image"):
            value = np.asarray(observation[key])
            if value.ndim != 3 or value.shape[-1] != 3 or value.dtype != np.uint8:
                raise ValueError(f"{key} must be HWC uint8 RGB")
            images.append(value.copy())
        state = np.r_[observation["robot0_eef_pos"], observation["robot0_eef_quat"],
                      observation["robot0_gripper_qpos"]].astype(np.float32)
        sim_state = np.asarray(sim_state).copy()
        if state.shape != (9,) or not np.isfinite(state).all() or not np.isfinite(sim_state).all():
            raise ValueError("invalid observation/simulator state")
        self.rows.append((*images, state, sim_state, self.clock() - self.started))

    def save(self, output, action_count):
        if len(self.rows) != action_count + 1:
            raise ValueError("observation/action alignment must be T+1/T")
        np.savez_compressed(
            Path(output) / "observations.npz",
            agentview_image=np.stack([r[0] for r in self.rows]),
            wrist_image=np.stack([r[1] for r in self.rows]),
            state=np.stack([r[2] for r in self.rows]),
            sim_states=np.stack([r[3] for r in self.rows]),
            elapsed_seconds=np.asarray([r[4] for r in self.rows]),
        )
        return {"file": "observations.npz", "boundaries": len(self.rows),
                "alignment": "observation[t], action[t], observation[t+1]",
                "camera_transform": "raw_simulator_rgb_no_flip_no_resize",
                "state": "eef_xyz+quaternion_xyzw+gripper_qpos2",
                "timing_reference": "session_start_monotonic",
                "sim_state_is_privileged_metadata": True}
