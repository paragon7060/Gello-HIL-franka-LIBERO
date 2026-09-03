"""Adapters for the shared alignment UI; no motor writes or simulator stepping."""

import hashlib

import numpy as np

from .session import OperatorSession as GelloOperatorSession  # shared UI type
from .ui import close_window as close_operator_window
from .ui import operator_panel

__all__ = ["GelloOperatorSession", "close_operator_window"]


def sha256_array(value):
    return hashlib.sha256(np.asarray(value).tobytes()).hexdigest()


def prepare_observation(observation, resize_size):
    from .libero_runtime import rgb_frame

    return None, rgb_frame(observation)


def operator_camera_frame_from_policy_frame(frame):
    # The public runtime already returns its consistent operator camera frame.
    return frame


def collection_window_name(protocol, *, task_id, state_id, attempt_index):
    return f"{protocol} TASK {task_id} STATE {state_id} ATTEMPT {attempt_index}"


def draw_attempt_panel(frame, **kwargs):
    return operator_panel(
        frame, instruction=kwargs["instruction"], status=kwargs["handoff_status"],
        active_steps=0, max_steps=kwargs["action_budget"], mapping_mode="absolute",
        notice="Support GELLO. Simulator/budget FROZEN. Fresh Enter after torque-off.",
        scale=kwargs["window_scale"],
        controls=("SPACE: release + PAUSE | q: release + ABORT",
                  "ENTER ignored until torque-off verified"),
    )
