"""Finite worker IPC for the desktop collector; buttons use the existing safety loop."""

from __future__ import annotations

import json
import os
import time
from pathlib import Path

import cv2
import numpy as np

from .review import atomic_json


def json_safe(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    return value


class FileDisplay:
    """Display adapter, never a motor transport. Abort has priority over other buttons."""

    def __init__(self, root, *, motor_permitted):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.motor_permitted = bool(motor_permitted)
        self.state = {
            "mode": "loading",
            "phase": "LOADING",
            "notice": "Preparing the initial scene",
        }
        self.sequence = 0
        self.events = []
        self.last_button = None
        self.last_publish = 0.0

    def set_state(self, **fields):
        self.state.update(json_safe(fields))

    def publish(self, frame=None, *, force=False):
        if not force and time.monotonic() - self.last_publish < 0.10:
            return
        self.last_publish = time.monotonic()
        if frame is not None:
            frame = np.asarray(frame, dtype=np.uint8)
            if frame.shape[1] > 900:
                frame = cv2.resize(frame, (900, round(frame.shape[0] * 900 / frame.shape[1])))
            okay, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 88])
            if not okay:
                raise OSError("live preview encoding failed")
            temp = self.root / "frame.tmp"
            temp.write_bytes(encoded.tobytes())
            os.replace(temp, self.root / "frame.jpg")
        atomic_json(
            self.root / "state.json",
            {
                **self.state,
                "updated_monotonic": time.monotonic(),
                "motor_permitted": self.motor_permitted,
            },
            replace=True,
        )

    def namedWindow(self, *args):
        pass

    def resizeWindow(self, *args):
        pass

    def destroyWindow(self, *args):
        pass

    def getWindowProperty(self, *args):
        return 1

    def imshow(self, window, canvas):
        # Alignment and legacy handoff panels place the camera in the left square.
        width = int(self.state.get("camera_width", 768))
        self.publish(canvas[:width, :width])

    def waitKey(self, milliseconds=10):
        if (self.root / "abort.json").exists():
            return ord("q")
        commands = sorted((self.root / "commands").glob("*.json"))
        for path in commands:
            sequence = int(path.name.split("_")[0])
            if sequence <= self.sequence:
                continue
            self.sequence = sequence
            request = json.loads(path.read_text())
            command = request["command"]
            mode, phase = self.state.get("mode"), self.state.get("phase")
            allowed = command == "abort" or (
                self.motor_permitted
                and (
                    command == "pause"
                    or command == "assist"
                    and mode == "alignment"
                    and phase == "PAUSED"
                    or command == "start"
                    and (
                        mode == "collection"
                        and phase == "PAUSED"
                        or mode == "alignment"
                        and (
                            phase == "ALIGNMENT READY"
                            or phase == "PAUSED"
                            and self.state.get("passive_start_ready")
                        )
                    )
                )
            )
            self.events.append({**request, "accepted": bool(allowed), "mode": mode, "phase": phase})
            if allowed:
                self.last_button = command
                return {"abort": ord("q"), "pause": 32, "assist": 13, "start": 13}[command]
        time.sleep(max(0, min(milliseconds, 20)) / 1000)
        return -1

    def summary(self):
        return {
            "button_events": self.events,
            "confirmation_required": True,
            "motor_authorization_source": "explicit_gui_assist_or_start_button",
            "commands_directory": str(self.root / "commands"),
        }


def preview_scene(args):
    """Initial scene before calibration selection; no serial bus is opened."""
    from .position import make_position_env
    from .libero_runtime import rgb_frame

    display = FileDisplay(args.gui_control_dir, motor_permitted=False)
    environment = None
    try:
        environment, observation, instruction = make_position_env(args)
        display.set_state(
            mode="preview",
            phase="PREVIEW",
            instruction=instruction,
            notice="Initial scene. Choose a calibration to enable alignment and collection.",
        )
        display.publish(cv2.cvtColor(rgb_frame(observation), cv2.COLOR_RGB2BGR), force=True)
        while display.waitKey(20) != ord("q"):
            pass
    finally:
        if environment is not None:
            environment.close()
        display.set_state(mode="preview", phase="CLOSED")
        display.publish(force=True)
    return 0
