"""Desktop GUI QA in a separate synthetic pool; never opens a real serial device."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from gello_hil import cli, pool
from gello_hil.collector_qt import CollectorWindow, application
from PySide6 import QtCore as C
from gello_hil.position import resolve_case, sha256


def fixture(root, shot, *, success, name):
    case = resolve_case(root / "pool-plan.json", "spatial_00", None, shot)
    case["benchmark_commit"] = case["expected_benchmark_commit"]
    directory = root / "attempts/spatial_00" / f"shot{shot:02d}" / name
    directory.mkdir(parents=True)
    (directory / "calibration.json").write_text('{"synthetic_gui_fixture": true}\n')
    meta = {
        "success": success,
        "aborted": False,
        "error": None,
        "motor_assistance_requested": True,
        "operator_review_required": True,
        "position_case": case,
        "controller": "OSC_POSE",
        "controller_contract": {"type": "OSC_POSE"},
        "instruction": "SYNTHETIC GUI QA: pick up the bowl and place it on the plate",
        "success_hold_steps": 20,
        "native_success_streak": 20 if success else 0,
        "source": {
            "commit": "synthetic-qa-only",
            "source_sha256": {"fixture": "synthetic"},
        },
        "session": {"active_step_count": 20},
        "calibration": {"snapshot_sha256": sha256(directory / "calibration.json")},
    }
    (directory / "attempt.json").write_text(json.dumps(meta))
    frames = np.zeros((21, 256, 256, 3), np.uint8)
    frames[:, :, :128, 0] = 150
    frames[:, :, 128:, 2] = 190
    for index, frame in enumerate(frames):
        cv2.rectangle(frame, (20 + index * 4, 100), (60 + index * 4, 140), (220, 220, 50), -1)
    np.savez_compressed(
        directory / "trajectory.npz",
        actions=np.zeros((20, 7)),
        counts=np.zeros((20, 8), np.int64),
        follower_joints=np.zeros((20, 7)),
        action_elapsed_seconds=np.arange(20) / 20,
    )
    np.savez_compressed(
        directory / "observations.npz",
        agentview_image=frames,
        wrist_image=frames,
        state=np.tile([0, 0, 0, 0, 0, 0, 1, 0.04, 0.04], (21, 1)),
        sim_states=np.zeros((21, 5)),
        elapsed_seconds=np.arange(21) / 20,
    )
    writer = cv2.VideoWriter(
        str(directory / "operator-view.mp4"),
        cv2.VideoWriter_fourcc(*"mp4v"),
        10,
        (256, 256),
    )
    if not writer.isOpened():
        raise RuntimeError("QA video encoder failed")
    for frame in frames[:-1]:
        writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    writer.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--native-scene", action="store_true")
    parser.add_argument("--libero-source")
    parser.add_argument("--libero-config-path")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    dataset = args.output / "synthetic-pool"
    pool.init_pool(dataset)
    if not args.native_scene:
        fixture(dataset, 1, success=False, name="001-failed")
        fixture(dataset, 1, success=True, name="002-success")
        fixture(dataset, 2, success=True, name="001-success")
    command = ["position-pool-gui", "--dataset-root", str(dataset), "--port", "UNCONNECTED_GUI_QA"]
    if args.libero_source:
        command += ["--libero-source", args.libero_source]
    if args.libero_config_path:
        command += ["--libero-config-path", args.libero_config_path]
    options = cli.build_parser().parse_args(command)
    app = application()
    window = CollectorWindow(options)
    window.show()
    result = {
        "motor_register_writes": 0,
        "real_serial_device_used": False,
        "native_scene": args.native_scene,
        "interface_language": "English",
        "toolkit": "Qt6",
        "screenshots": [],
    }
    started = time.monotonic()
    stage = 0

    def screenshot(name):
        path = args.output / name
        if not window.grab().save(str(path)):
            raise OSError("window screenshot failed")
        result["screenshots"].append(name)
        result["viewport"] = [window.preview.width(), window.preview.height()]

    def check():
        nonlocal stage
        try:
            if args.native_scene:
                if window.state.get("phase") == "PREVIEW":
                    screenshot("initial-native-scene.png")
                    result["initial_scene_visible"] = True
                    result["motor_buttons_disabled"] = not window.buttons["assist"].isEnabled()
                    window.close()
                    return
                if time.monotonic() - started > 45:
                    raise RuntimeError(
                        "native scene preview did not become ready: " + window.status.text()
                    )
            elif stage == 0:
                assert window.capture and window.capture.isOpened()
                assert window.buttons["confirm"].isEnabled()
                screenshot("review-candidate.png")
                window.toggle_video()
                stage = 1
            elif stage == 1:
                assert window.seek.value() > 0
                window.toggle_video()
                window.buttons["confirm"].click()
                stage = 2
            elif stage == 2 and pool.inspect_pool(dataset)["accepted"] == 1 and window.shot == 2:
                screenshot("confirmed-next-shot.png")
                result.update(
                    video_playback=True,
                    confirm_button=True,
                    automatically_selected_next_shot=True,
                    failed_attempt_preserved=True,
                    accepted=1,
                )
                window.close()
                return
            if not args.native_scene and time.monotonic() - started > 20:
                raise RuntimeError("review workflow did not finish: " + window.status.text())
        except Exception as error:
            result["error"] = f"{type(error).__name__}: {error}"
            window.close()
            return
        C.QTimer.singleShot(600, check)

    C.QTimer.singleShot(1000, check)
    app.exec()
    result["worker_cleaned_up"] = not window.worker.running
    result["passed"] = "error" not in result and result["worker_cleaned_up"]
    (args.output / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
