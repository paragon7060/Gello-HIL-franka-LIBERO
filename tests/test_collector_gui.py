import json
from types import SimpleNamespace

import numpy as np
import pytest

from gello_hil import cli, pool
from gello_hil.collector_gui import SceneWorker, button_permissions
from gello_hil.exports import export_support
from gello_hil.gui_bridge import FileDisplay
from gello_hil.position import sha256
from gello_hil.review import (
    atomic_json,
    confirm_attempt,
    current_confirmation,
    enable_review,
    pool_lock,
    record_event,
)
from test_position_pool import fake_attempt


def new_pool(path):
    pool.init_pool(path)
    assert enable_review(path)
    return path


def test_success_stays_pending_until_explicit_confirmation(tmp_path):
    root = new_pool(tmp_path)
    attempt = fake_attempt(root, "object_07", 1)
    before = {name: sha256(attempt / name) for name in pool.PAYLOADS}
    status = pool.inspect_pool(root)
    assert status["accepted"] == 0 and status["failed_attempts"] == 0
    assert next(s for s in status["cases"] if s["task_key"] == "object_07")["pending_review"] == 1
    with pytest.raises(ValueError, match="confirmation required"):
        export_support(attempt, root / "unconfirmed.npz")
    decision = confirm_attempt(root, str(attempt.relative_to(root)))
    assert pool.inspect_pool(root)["accepted"] == 1
    assert before == {name: sha256(attempt / name) for name in pool.PAYLOADS}
    assert decision == confirm_attempt(root, str(attempt.relative_to(root)))
    export_support(attempt, root / "confirmed.npz")


def test_retry_and_confirm_replacement_preserve_previous_raw_and_decisions(tmp_path):
    root = new_pool(tmp_path)
    original = fake_attempt(root, "object_07", 1)
    first = confirm_attempt(root, str(original.relative_to(root)))
    hashes = {n: sha256(original / n) for n in pool.PAYLOADS}
    with pool_lock(root):
        record_event(root, "object_07", 1, "retry_requested", previous_attempt=first["attempt"])
    assert (
        pool.inspect_pool(root)["accepted"] == 1
    )  # keep confirmed data until replacement is accepted
    replacement = fake_attempt(root, "object_07", 1, name="002")
    assert (
        pool.inspect_pool(root)["accepted"] == 1
    )  # native success alone cannot replace a selection
    second = confirm_attempt(root, str(replacement.relative_to(root)))
    assert second["replaces"] == first["attempt"]
    slot = next(s for s in pool.inspect_pool(root)["cases"] if s["task_key"] == "object_07")
    assert slot["accepted"] == second["attempt"] and slot["pending_review"] == 0
    assert hashes == {n: sha256(original / n) for n in pool.PAYLOADS}
    assert (root / first["event_file"]).exists()
    with pytest.raises(ValueError, match="confirmation required"):
        export_support(original, root / "superseded.npz")


def test_failed_aborted_foreign_and_tampered_attempts_cannot_be_confirmed(tmp_path):
    root = new_pool(tmp_path)
    failed = fake_attempt(root, "object_07", 1, success=False)
    with pytest.raises(ValueError, match="failed/aborted"):
        confirm_attempt(root, str(failed.relative_to(root)))
    success = fake_attempt(root, "object_07", 1, name="002")
    event = confirm_attempt(root, str(success.relative_to(root)))
    (success / "trajectory.npz").write_bytes(b"changed")
    with pytest.raises(ValueError, match="confirmed raw attempt changed"):
        export_support(success, root / "changed.npz")
    value = json.loads((root / event["event_file"]).read_text())
    value["previous_event_sha256"] = "invalid"
    (root / event["event_file"]).write_text(json.dumps(value))
    with pytest.raises(ValueError, match="checksum chain"):
        current_confirmation(root, "object_07", 1)
    with pytest.raises(ValueError, match="belong"):
        confirm_attempt(root, "../elsewhere")


def test_frozen_pool_confirmation_is_read_only(tmp_path):
    root = new_pool(tmp_path)
    attempt = fake_attempt(root, "object_07", 1)
    atomic_json(root / "pool-manifest.json", {})
    with pytest.raises(ValueError, match="frozen"):
        confirm_attempt(root, str(attempt.relative_to(root)))


@pytest.mark.parametrize("motor_permitted", [False, True])
def test_gui_bridge_requires_correct_phase_and_motor_permission(tmp_path, motor_permitted):
    bridge = FileDisplay(tmp_path, motor_permitted=motor_permitted)
    bridge.set_state(mode="alignment", phase="PAUSED", passive_start_ready=False)

    def send(sequence, name):
        atomic_json(tmp_path / "commands" / f"{sequence:06d}_request.json", {"command": name})
        return bridge.waitKey(0)

    assert send(1, "start") == -1  # start must never accidentally arm assistance
    assert send(2, "assist") == (13 if motor_permitted else -1)
    bridge.set_state(phase="ALIGNMENT READY")
    assert send(3, "start") == (13 if motor_permitted else -1)
    bridge.set_state(mode="collection", phase="ACTIVE")
    assert send(4, "start") == -1
    assert send(5, "pause") == (32 if motor_permitted else -1)
    atomic_json(tmp_path / "abort.json", {"command": "abort"})
    assert bridge.waitKey(0) == ord("q")


def test_dashboard_button_states_match_worker_safety_phases():
    far = button_permissions({"mode": "alignment", "phase": "PAUSED"}, writable=True, running=True)
    assert far["assist"] and not far["start"] and not far["confirm"]
    near = button_permissions(
        {"mode": "alignment", "phase": "PAUSED", "passive_start_ready": True},
        writable=True,
        running=True,
    )
    assert near["start"]
    active = button_permissions(
        {"mode": "collection", "phase": "ACTIVE"}, writable=True, running=True
    )
    assert active["pause"] and not active["start"] and not active["confirm"]
    review = button_permissions({}, writable=False, running=False, candidate_success=True)
    assert review["confirm"] and not review["assist"] and not review["retry"]
    frozen = button_permissions(
        {}, writable=True, running=False, candidate_success=True, frozen=True
    )
    assert not frozen["confirm"] and not frozen["retry"]


def test_scene_worker_is_one_owned_process_and_abort_waits_for_cleanup(tmp_path):
    root = new_pool(tmp_path / "pool")
    calibration = tmp_path / "calibration.json"
    calibration.write_text("{}")
    calls = []

    class Process:
        returncode = None

        def poll(self):
            return self.returncode

    process = Process()

    def launch(command, **kwargs):
        calls.append((command, kwargs))
        return process

    args = SimpleNamespace(
        dataset_root=str(root),
        calibration=str(calibration),
        libero_source=str(tmp_path),
        libero_config_path=str(tmp_path),
        port="unused",
        baudrate=57600,
        serial_timeout=0.12,
        max_active_steps=1000,
        apply=True,
    )
    worker = SceneWorker(args, process_factory=launch)
    worker.open("object_07", 1)
    assert len(calls) == 1 and "--gui-control-dir" in calls[0][0] and "--apply" in calls[0][0]
    with pytest.raises(BlockingIOError), pool_lock(root):
        pass
    with pytest.raises(RuntimeError):
        worker.open("object_07", 2)
    worker.send("assist")
    worker.send("abort")
    assert (worker.control / "abort.json").is_file() and worker.running
    process.returncode = 1
    assert worker.finish()["returncode"] == 1
    with pool_lock(root):
        pass


def test_collection_gui_controls_to_pending_candidate_to_confirmation(tmp_path, monkeypatch):
    from gello_hil import assisted, position
    from gello_hil import libero_runtime as runtime
    from gello_hil import pose_control as pose
    from test_repa_position import FakeEnv, observation, counts, fake_fk
    from test_alignment_safety import POSE, calibration

    root = new_pool(tmp_path / "pool")
    cal = tmp_path / "calibration.json"
    calibration().to_json(cal)
    ipc = tmp_path / "ipc"
    ipc.mkdir()
    atomic_json(ipc / "commands/000001_start.json", {"command": "start"})
    env = FakeEnv([True] * 30)

    class Bus:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def read_positions(self, ids):
            return counts()

    class Alignment:
        live_handoff = None

        def __init__(self, args):
            self.args = args

        def __call__(self, **kw):
            display = self.args.gui_display
            display.set_state(mode="alignment", phase="ALIGNMENT READY")
            assert display.waitKey(0) == 13
            return {
                "collection_start_eligible": True,
                "aborted": False,
                "collection_start_requested_by_alignment_enter": True,
            }

    def make_env(args):
        args.position_case["benchmark_commit"] = args.position_case["expected_benchmark_commit"]
        return env, observation(), "test native instruction"

    monkeypatch.setattr(position, "make_position_env", make_env)
    monkeypatch.setattr(assisted, "make_alignment_runner", lambda a, c: Alignment(a))
    monkeypatch.setattr(runtime, "follower_joints", lambda e: POSE.copy())
    monkeypatch.setattr(
        pose,
        "make_pose_follower",
        lambda e, c: pose.PoseFollower(
            c,
            fake_fk,
            output_min=[-0.05] * 3 + [-0.5] * 3,
            output_max=[0.05] * 3 + [0.5] * 3,
        ),
    )
    monkeypatch.setattr(cli, "_bus", lambda a: Bus())
    monkeypatch.setattr(cli, "close_window", lambda *a: None)
    args = cli.build_parser().parse_args(
        [
            "assisted-position",
            "--port",
            "unused",
            "--calibration",
            str(cal),
            "--libero-source",
            "unused",
            "--plan",
            str(root / "pool-plan.json"),
            "--task-key",
            "object_07",
            "--shot",
            "1",
            "--apply",
            "--gui-control-dir",
            str(ipc),
            "--output-root",
            str(root / "attempts/object_07/shot01"),
        ]
    )
    assert cli.position_collect(args) == 0 and env.closed
    assert pool.inspect_pool(root)["accepted"] == 0
    attempt = next((root / "attempts/object_07/shot01").iterdir())
    meta = json.loads((attempt / "attempt.json").read_text())
    assert meta["operator_review_required"] and meta["operator_gui"]["button_events"][0]["accepted"]
    with pytest.raises(ValueError, match="confirmation required"):
        export_support(attempt, tmp_path / "before.npz")
    confirm_attempt(root, str(attempt.relative_to(root)))
    assert pool.inspect_pool(root)["accepted"] == 1
    export_support(attempt, tmp_path / "after.npz")
    with np.load(tmp_path / "after.npz") as data:
        assert data["actions"].shape == (20, 7)


def test_assist_button_at_near_pose_waits_for_separate_start(monkeypatch, tmp_path):
    import gello_hil.alignment as module
    from gello_hil.actuation import GelloAlignmentAssistLimits
    from gello_hil.assisted import make_alignment_runner
    from test_alignment_safety import POSE, setup

    limits = GelloAlignmentAssistLimits(
        profile="GELLO-ALIGN-A6",
        ready_max_abs_error_rad=0.6,
        ready_rms_error_rad=0.6,
    )
    bus, controller, _ = setup(target_offset=0, limits=limits)
    next_bus, next_controller, _ = setup(target_offset=0, limits=limits)
    clock = controller._clock
    bus.close = lambda: None
    next_bus.close = lambda: None
    display = FileDisplay(tmp_path, motor_permitted=True)
    commands = iter(["assist", "start"])

    def click(_):
        button = next(commands)
        if button == "start":
            assert controller.summary()["motor_register_writes"] == 0
            assert display.state["phase"] == "PAUSED"
            assert "press Start" in display.state["notice"]
        display.last_button = button
        return 13

    monkeypatch.setattr(display, "waitKey", click)
    args = SimpleNamespace(
        baudrate=57600,
        port="unused",
        serial_timeout=0.12,
        window_scale=3,
        max_active_steps=1000,
        gui_display=display,
    )
    runner = make_alignment_runner(args, controller.calibration)
    buses = iter([bus, next_bus])
    monkeypatch.setattr(runner, "_bus", lambda: next(buses))
    monkeypatch.setattr(
        runner, "_controller", lambda b: controller if b is bus else next_controller
    )
    monkeypatch.setattr(
        runner,
        "_read_only_bus",
        lambda: SimpleNamespace(
            read_positions=lambda ids: {
                i: bus.registers[i]["present_position_counts"] for i in ids
            },
            close=lambda: None,
        ),
    )
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=clock, sleep=clock.sleep))
    monkeypatch.setattr(
        module.passive, "prepare_observation", lambda *a: (None, np.zeros((64, 64, 3), np.uint8))
    )
    result = runner(
        env=SimpleNamespace(get_sim_state=lambda: np.zeros(10)),
        observation={},
        follower_target_joints=POSE,
        task_instruction="native",
        task_id=0,
        state_id=0,
        attempt_index=1,
        resize_size=64,
    )
    assert result["collection_start_eligible"] and not result["aborted"], result.get("failure")
    assert result["simulator_steps"] == 0
    assert all(s["motor_register_writes"] == 0 for s in result["sessions"])


def test_qt_english_review_confirm_and_next(monkeypatch, tmp_path):
    import time

    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtWidgets as W
    from gello_hil.collector_qt import CollectorWindow, application
    import cv2

    root = new_pool(tmp_path)
    attempt = fake_attempt(root, "spatial_00", 1)
    fake_attempt(root, "spatial_00", 2)
    writer = cv2.VideoWriter(
        str(attempt / "operator-view.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 10, (16, 16)
    )
    assert writer.isOpened()
    for _ in range(20):
        writer.write(np.zeros((16, 16, 3), np.uint8))
    writer.release()
    args = cli.build_parser().parse_args(
        [
            "position-pool-gui",
            "--dataset-root",
            str(root),
            "--port",
            "UNCONNECTED_QA",
            "--libero-source",
            str(tmp_path / "no-source"),
            "--libero-config-path",
            str(tmp_path / "no-config"),
        ]
    )
    app = application()
    window = CollectorWindow(args)
    try:
        window.show()
        app.processEvents()
        window.poll()
        assert window.tasks.topLevelItemCount() == 40
        assert window.shots.topLevelItemCount() == 10
        assert window.buttons["confirm"].isEnabled()
        assert not window.buttons["assist"].isEnabled()
        assert all(
            not any("\uac00" <= c <= "\ud7a3" for c in w.text())
            for w in window.findChildren(W.QLabel) + window.findChildren(W.QPushButton)
        )
        window.buttons["confirm"].click()
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline and window.shot != 2:
            app.processEvents()
            window.poll()
            time.sleep(0.01)
        assert pool.inspect_pool(root)["accepted"] == 1
        assert window.shot == 2 and window.worker.process is None
    finally:
        window.close()
        app.processEvents()
