"""Finite Qt Stop/review stress check with synthetic workers; never opens a device."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path


def synthetic_worker(control, dataset, index):
    from collector_gui_smoke import fixture
    from gello_hil.gui_bridge import FileDisplay
    from gello_hil.review import atomic_json

    control, dataset = Path(control), Path(dataset)
    display = FileDisplay(control, motor_permitted=False)
    display.set_state(
        mode="collection",
        phase="ACTIVE",
        active_steps=20,
        instruction="SYNTHETIC Stop stress / no real device",
        notice="Waiting for the actual GUI Stop button.",
    )
    display.publish(force=True)
    started = time.monotonic()
    while not (control / "abort.json").exists():
        if time.monotonic() - started > 30:
            raise RuntimeError("synthetic worker did not receive Stop")
        time.sleep(0.01)
    name = f"synthetic-{index:04d}"
    fixture(dataset, 1, success=False, name=name)
    directory = dataset / "attempts/spatial_00/shot01" / name
    header = directory / "attempt.json"
    meta = json.loads(header.read_text())
    meta["aborted"] = True
    atomic_json(header, meta, replace=True)
    display.set_state(
        mode="review",
        phase="REVIEW",
        success=False,
        aborted=True,
        error=None,
        attempt_path=str(directory.resolve()),
    )
    display.publish(force=True)
    return 0


def run(args):
    from PySide6 import QtCore as C, QtWidgets as W
    from gello_hil import cli, pool
    from gello_hil.collector_gui import SceneWorker
    from gello_hil.collector_qt import CollectorWindow, application
    from gello_hil.qt_runtime import verify_qt_runtime

    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    dataset = root / "synthetic-pool"
    (root / "unused-source").mkdir()
    (root / "unused-config").mkdir()
    options = cli.build_parser().parse_args(
        [
            "position-pool-gui",
            "--dataset-root",
            str(dataset),
            "--port",
            "UNCONNECTED_QA",
            "--libero-source",
            str(root / "unused-source"),
            "--libero-config-path",
            str(root / "unused-config"),
            "--apply",
        ]
    )
    if args.task_examples:
        options.task_examples = str(args.task_examples)
    # A stale local calibration preference must never connect this test to hardware.
    os.environ.pop("GELLO_CALIBRATION", None)
    launches = 0

    def factory(command, **kwargs):
        nonlocal launches
        launches += 1
        control = command[command.index("--gui-control-dir") + 1]
        return subprocess.Popen(
            [
                sys.executable,
                "-B",
                str(Path(__file__).resolve()),
                "--synthetic-worker",
                "--control",
                control,
                "--dataset",
                str(dataset),
                "--index",
                str(launches),
            ],
            **kwargs,
        )

    app = application()
    runtime = verify_qt_runtime()
    window = CollectorWindow(
        options, worker_factory=lambda a: SceneWorker(a, process_factory=factory)
    )
    window.show()
    stop = next(b for b in window.findChildren(W.QPushButton) if b.text().startswith("Stop"))
    started = time.monotonic()
    completed, polls, before, after = 0, 0, None, None
    stage = "active"
    result = {"runtime": runtime, "real_serial_used": False, "motor_register_writes": 0}

    def check():
        nonlocal completed, polls, stage, before, after
        try:
            if time.monotonic() - started > 120:
                raise RuntimeError("Stop stress timed out: " + window.status.text())
            if stage == "active" and window.state.get("phase") == "ACTIVE":
                if args.task_examples and not window.example.timer.isActive():
                    window.example.restart()
                    assert window.example.capture is not None and window.example.timer.isActive()
                # This deliberately exceeds the old runtime's cumulative void-call
                # failure threshold while keeping the real UI/worker transition.
                before = sys.getrefcount(None) if before is None else before
                for _ in range(args.polls_per_cycle):
                    window.poll()
                    polls += 1
                stop.click()
                stage = "saved"
            elif stage == "saved" and not window.worker.running and window.worker.process is None:
                assert window.capture is not None and window.capture.isOpened()
                assert not window.buttons["confirm"].isEnabled()
                slot = window.slot()
                assert len(slot["attempts"]) == completed + 1
                assert slot["details"][-1]["aborted"]
                if args.task_examples:
                    assert window.example.capture.isOpened() and window.example.timer.isActive()
                    result["task_example_survived_stop"] = True
                completed += 1
                if completed == args.cycles:
                    after = sys.getrefcount(None)
                    assert pool.inspect_pool(dataset)["accepted"] == 0
                    assert window.grab().save(str(root / "stop-review.png"))
                    result.update(
                        passed=True,
                        stop_cycles=completed,
                        explicit_gui_polls=polls,
                        raw_attempts_preserved=completed,
                        video_review_opened=completed,
                        none_refcount_before=before,
                        none_refcount_after=after,
                    )
                    window.close()
                    return
                window.open_scene(retry=True)
                stage = "active"
        except Exception as error:
            result.update(passed=False, error=f"{type(error).__name__}: {error}")
            window.close()
            return
        C.QTimer.singleShot(25, check)

    C.QTimer.singleShot(350, check)
    app.exec()
    result["worker_cleaned_up"] = not window.worker.running
    (root / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))
    return 0 if result.get("passed") and result["worker_cleaned_up"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--cycles", type=int, default=20)
    parser.add_argument("--polls-per-cycle", type=int, default=200)
    parser.add_argument("--task-examples", type=Path)
    parser.add_argument("--synthetic-worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--control", help=argparse.SUPPRESS)
    parser.add_argument("--dataset", help=argparse.SUPPRESS)
    parser.add_argument("--index", type=int, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.synthetic_worker:
        return synthetic_worker(args.control, args.dataset, args.index)
    if args.output is None or args.cycles < 1 or args.polls_per_cycle < 1:
        parser.error("--output, positive --cycles and --polls-per-cycle are required")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
