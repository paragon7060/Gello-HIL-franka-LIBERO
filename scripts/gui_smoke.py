"""Render/operator-window QA only. No hardware, simulation step, or motor writes."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from gello_hil.actuation import GelloAlignmentAssistLimits
from gello_hil.alignment import draw_alignment_panel
from gello_hil.ui import operator_panel, poll_command


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--display", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    frame = np.zeros((256, 256, 3), np.uint8)
    frame[:, :128, 0] = 200
    frame[:, 128:, 2] = 200
    instruction = "pick up the alphabet soup and place it in the basket; GUI LAYOUT CHECK (synthetic camera)"
    panels = {
        "collection_low_resolution": operator_panel(frame[::4, ::4], instruction=instruction,
            status="PAUSED", active_steps=0, max_steps=1000, mapping_mode="enter-anchored-osc-pose",
            joints=np.zeros(7), errors=np.zeros(7), scale=2,
            notice="Ready screen: simulator is frozen; Enter starts."),
        "collection_live_release": operator_panel(frame, instruction=instruction,
            status="ACTIVE", active_steps=3, max_steps=1000, mapping_mode="enter-anchored-osc-pose",
            joints=np.zeros(7), errors=np.zeros(7), scale=2.5,
            notice="ACTIVE: following GELLO while reducing force over 2 seconds"),
        "alignment_ready": draw_alignment_panel(frame, instruction=instruction,
            task_id=7, state_id=2, attempt_index=1, status="ALIGNMENT READY",
            diagnostic={"target_error_radians": [0]*7, "target_joint_positions": [0]*7,
                        "mapped_gello_joint_positions": [0]*7, "present_position_counts": [2048]*7,
                        "max_abs_target_error_rad": 0, "rms_target_error_rad": 0, "ready_streak": 5},
            preview_error=None, notice="ENTER: START control now, soften over 2s (no second ENTER)",
            limits=GelloAlignmentAssistLimits(profile="GELLO-ALIGN-A6",
                ready_max_abs_error_rad=.60, ready_rms_error_rad=.60), passive_start_ready=False),
    }
    result = {"synthetic_camera": True, "motor_register_writes": 0, "simulator_steps": 0, "panels": {}}
    for name, canvas in panels.items():
        if not cv2.imwrite(str(args.output / (name + ".png")), canvas):
            raise OSError("PNG save failed")
        info = {"shape": list(canvas.shape), "display_tested": args.display}
        if args.display:
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            cv2.imshow(name, canvas)
            cv2.waitKey(150)
            info["window_visible"] = cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) >= 1
            cv2.destroyWindow(name)
            info["closed_window_aborts"] = poll_command(name) == "abort"
            if not info["window_visible"] or not info["closed_window_aborts"]:
                raise RuntimeError("GUI window lifecycle check failed")
        result["panels"][name] = info
    (args.output / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
