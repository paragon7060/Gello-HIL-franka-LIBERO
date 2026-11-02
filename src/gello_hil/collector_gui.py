"""One desktop window for scene preview, collection, attempt review and confirmation."""

from __future__ import annotations

import glob
import json
import os
import subprocess
import sys
import time
import uuid
from pathlib import Path

from .pool import DEFAULT_PLAN
from .review import atomic_json, pool_lock, record_event


def button_permissions(state, *, writable, running, candidate_success=False, frozen=False):
    phase, mode = state.get("phase"), state.get("mode")
    return {
        "assist": writable and running and mode == "alignment" and phase == "PAUSED",
        "start": writable
        and running
        and (
            mode == "alignment"
            and (
                phase == "ALIGNMENT READY" or phase == "PAUSED" and state.get("passive_start_ready")
            )
            or mode == "collection"
            and phase == "PAUSED"
        ),
        "pause": writable
        and running
        and (
            mode == "collection" and phase == "ACTIVE" or mode == "alignment" and phase != "PAUSED"
        ),
        "retry": writable and not frozen,
        "confirm": not running and candidate_success and not frozen,
    }


class SceneWorker:
    """Own one finite child and one pool lock. Stop requests use the child's cleanup path."""

    def __init__(self, args, *, process_factory=subprocess.Popen):
        self.args = args
        self.root = Path(args.dataset_root).resolve()
        self.process_factory = process_factory
        self.process = None
        self.lock = None
        self.log = None
        self.control = None
        self.slot = None
        self.sequence = 0
        self.last_state = {}

    @property
    def running(self):
        return self.process is not None and self.process.poll() is None

    def open(self, key, shot, *, retry=False, previous=None):
        if self.running:
            raise RuntimeError("Wait for the previous scene to release and save.")
        if (self.root / "pool-manifest.json").exists():
            raise ValueError(
                "This dataset is frozen. Video review is available; collection is locked."
            )
        for field in ("libero_source", "libero_config_path"):
            if not getattr(self.args, field, None) or not Path(getattr(self.args, field)).exists():
                raise ValueError(f"Check setting: {field}")
        from .position import resolve_case

        resolve_case(self.root / "pool-plan.json", key, None, shot)
        self.lock = pool_lock(self.root)
        self.lock.__enter__()
        try:
            self.slot = (key, shot)
            token = f"{time.time_ns()}_{uuid.uuid4().hex[:8]}"
            self.control = self.root / "gui-sessions" / token
            self.control.mkdir(parents=True)
            self.sequence = 0
            self.last_state = {"mode": "loading", "phase": "LOADING"}
            record_event(
                self.root,
                key,
                shot,
                "retry_requested" if retry else "scene_opened",
                previous_attempt=previous,
                session=token,
            )
            command = [
                sys.executable,
                "-u",
                "-B",
                "-m",
                "gello_hil.cli",
                "assisted-position",
                "--port",
                self.args.port or "",
                "--baudrate",
                str(self.args.baudrate),
                "--serial-timeout",
                str(self.args.serial_timeout),
                "--calibration",
                self.args.calibration or "",
                "--libero-source",
                self.args.libero_source,
                "--libero-config-path",
                self.args.libero_config_path,
                "--plan",
                str(self.root / "pool-plan.json"),
                "--task-key",
                key,
                "--shot",
                str(shot),
                "--output-root",
                str(self.root / "attempts" / key / f"shot{shot:02d}"),
                "--gui-control-dir",
                str(self.control),
                "--operator-review-required",
                "--success-hold-steps",
                "20",
                "--window-scale",
                "3",
                "--max-active-steps",
                str(self.args.max_active_steps),
            ]
            command += ["--apply"] if self.args.apply else ["--gui-preview-only"]
            if not self.args.calibration or not Path(self.args.calibration).is_file():
                command += ["--gui-scene-preview-only"]
            self.log = (self.control / "worker.log").open("w")
            environment = dict(os.environ)
            environment["PYTHONUNBUFFERED"] = "1"
            self.process = self.process_factory(
                command,
                stdin=subprocess.DEVNULL,
                stdout=self.log,
                stderr=subprocess.STDOUT,
                env=environment,
            )
        except BaseException:
            self.release()
            raise

    def send(self, command):
        if not self.running:
            return
        if command == "abort":
            atomic_json(
                self.control / "abort.json",
                {"command": "abort", "operator": True},
                replace=True,
            )
        else:
            self.sequence += 1
            atomic_json(
                self.control / "commands" / f"{self.sequence:06d}_{uuid.uuid4().hex[:8]}.json",
                {
                    "command": command,
                    "requested_at_monotonic": time.monotonic(),
                    "operator_source": "desktop_button",
                },
            )

    def state(self):
        if self.control is not None and (self.control / "state.json").exists():
            self.last_state = json.loads((self.control / "state.json").read_text())
        return self.last_state

    def finish(self):
        if self.process is None or self.running:
            return None
        result = {
            "returncode": self.process.returncode,
            "state": self.state(),
            "slot": self.slot,
            "log_file": str(self.control / "worker.log"),
        }
        self.release()
        self.process = None
        return result

    def release(self):
        if self.log:
            self.log.close()
            self.log = None
        if self.lock:
            self.lock.__exit__(None, None, None)
            self.lock = None


def launch_gui(args):
    from .collector_qt import launch

    return launch(args)


def add_gui_command(sub):
    parser = sub.add_parser(
        "position-pool-gui",
        help="one-window 40-task collection, video review and confirmation",
    )
    parser.add_argument("--dataset-root", default="runs/position40-pool10")
    parser.add_argument("--plan", default=str(DEFAULT_PLAN))
    parser.add_argument("--calibration")
    ports = glob.glob("/dev/serial/by-id/*")
    parser.add_argument("--port", default=ports[0] if len(ports) == 1 else "")
    parser.add_argument("--baudrate", type=int, default=57600, choices=(57600,))
    parser.add_argument("--serial-timeout", type=float, default=0.12)
    parser.add_argument(
        "--libero-source",
        default=str(Path(__file__).resolve().parents[3] / "LIBERO-PRO"),
    )
    parser.add_argument("--libero-config-path", default="runs/position40-assets/config")
    parser.add_argument("--max-active-steps", type=int, default=1000)
    parser.add_argument(
        "--apply",
        action="store_true",
        help="enable explicit GUI motor/collection buttons",
    )
    parser.set_defaults(func=launch_gui)
