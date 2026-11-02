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

    def __init__(self, root, *, motor_permitted, operator_view="camera"):
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
        self.initial_operator_view = operator_view
        # The GUI owns this worker; if it dies the worker must release, not idle.
        self.parent_pid = os.getppid()
        self.parent_lost = False

    def set_state(self, **fields):
        self.state.update(json_safe(fields))

    def _write_image(self, name, frame):
        frame = np.asarray(frame, dtype=np.uint8)
        if frame.shape[1] > 900:
            frame = cv2.resize(frame, (900, round(frame.shape[0] * 900 / frame.shape[1])))
        okay, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 88])
        if not okay:
            raise OSError("live preview encoding failed")
        temp = self.root / f"{name}.tmp"
        temp.write_bytes(encoded.tobytes())
        os.replace(temp, self.root / f"{name}.jpg")

    def publish(self, frame=None, *, wrist=None, force=False):
        """Publish BGR operator frames; wrist is the eye-in-hand camera in RGB."""
        if not force and time.monotonic() - self.last_publish < 0.10:
            return
        self.last_publish = time.monotonic()
        if frame is not None:
            self._write_image("frame", frame)
        if wrist is not None:
            self._write_image("wrist", cv2.cvtColor(np.asarray(wrist, dtype=np.uint8), cv2.COLOR_RGB2BGR))
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
        if os.getppid() != self.parent_pid:
            if not self.parent_lost:
                self.parent_lost = True
                self.events.append({"command": "abort", "accepted": True,
                                    "operator_source": "gui_process_lost",
                                    "mode": self.state.get("mode"), "phase": self.state.get("phase")})
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
            if command == "view":
                accepted = request.get("value") in ("camera", "policy")
                self.events.append({**request, "accepted": accepted, "mode": mode, "phase": phase})
                continue
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
            "initial_operator_view": self.initial_operator_view,
            "worker_and_video_pixel_convention": "openvla_raw_180_degrees",
            "camera_view_transform": "horizontal_flip_of_policy_view_display_only",
        }


def preview_scene(args):
    """Initial scene without serial, or explicit read-only joint mapping preview."""
    from .position import make_position_env
    from .libero_runtime import rgb_frame, render_joint_pose, wrist_frame
    from .calibration import GelloCalibration
    from .cli import _bus

    display = FileDisplay(
        args.gui_control_dir,
        motor_permitted=False,
        operator_view=getattr(args, "gui_operator_view", "camera"),
    )
    environment = None
    bus = None
    try:
        environment, observation, instruction = make_position_env(args)
        display.set_state(
            mode="preview",
            phase="PREVIEW",
            instruction=instruction,
            notice="Initial scene. Choose a calibration to enable alignment and collection.",
        )
        display.publish(cv2.cvtColor(rgb_frame(observation), cv2.COLOR_RGB2BGR),
                        wrist=wrist_frame(observation), force=True)
        if getattr(args, "gui_mapping_check", False):
            calibration = GelloCalibration.from_json(Path(args.calibration))
            bus = _bus(args)
        while display.waitKey(20) != ord("q"):
            if bus is not None:
                counts = bus.read_positions(calibration.arm_ids + (calibration.gripper_id,))
                joints = calibration.mapped_joints(counts)
                frame = render_joint_pose(environment, joints)
                display.set_state(
                    mode="mapping_check",
                    phase="MAPPING CHECK",
                    joint_degrees=np.rad2deg(joints).tolist(),
                    notice="Move one joint at a time; read-only pose preview, no physics steps.",
                )
                display.publish(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                time.sleep(0.18)
    except Exception as error:
        display.set_state(error=f"{type(error).__name__}: {error}")
        raise
    finally:
        if bus is not None:
            bus.close()
        if environment is not None:
            environment.close()
        display.set_state(mode="preview", phase="CLOSED")
        display.publish(force=True)
    return 0
