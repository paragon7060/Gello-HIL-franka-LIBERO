from types import SimpleNamespace

import numpy as np
import pytest

from gello_hil import cli, pool
from gello_hil.collector_gui import button_permissions
from gello_hil.gui_bridge import FileDisplay
from gello_hil.review import confirm_attempt, enable_review
from test_position_pool import fake_attempt
from test_repa_position import collect_fake, counts


def test_recovering_reader_retries_reopens_and_reports_persistent_fault():
    opened, results = [], iter([TimeoutError("drop"), {1: 7}, OSError("gone"), OSError("gone"),
                                OSError("gone")])

    class Port:
        def __enter__(self):
            opened.append(True)
            return self

        def __exit__(self, *args):
            pass

        def read_positions(self, ids):
            value = next(results)
            if isinstance(value, Exception):
                raise value
            return value

    reader = cli.RecoveringReader(Port, retries=2)
    assert reader.read_positions((1,)) == {1: 7}
    assert len(opened) == 2 and reader.recovered_faults == 1
    with pytest.raises(cli.SerialReadFault, match="gone"):
        reader.read_positions((1,))
    reader.close()


def test_encoder_dropout_pauses_the_attempt_and_resumes_after_reanchor(tmp_path, monkeypatch):
    reads = []

    def read_positions(ids):
        reads.append(True)
        if len(reads) in (4, 5, 6):  # one read plus both retries fail
            raise TimeoutError("Dynamixel SYNC_READ missing IDs [1, 2, 3, 4, 5, 6, 7, 8]")
        return counts()

    monkeypatch.setattr(cli.time, "sleep", lambda seconds: None)
    env, code, _, meta = collect_fake(
        tmp_path,
        monkeypatch,
        keys=[None, "resume", None, None, "resume", None, None],
        successes=[False, True, True],
        bus=SimpleNamespace(read_positions=read_positions),
    )
    assert code == 0 and meta["success"] and meta["error"] is None and env.steps == 3
    assert len(meta["encoder_read_faults"]) == 1
    events = [e for e in meta["session"]["events"] if e["event_type"] == "clutch_pause"]
    assert "missing IDs" in events[0]["encoder_read_fault"]


def test_worker_aborts_when_the_gui_process_disappears(tmp_path, monkeypatch):
    import gello_hil.gui_bridge as bridge

    display = FileDisplay(tmp_path, motor_permitted=True)
    display.set_state(mode="alignment", phase="ALIGNMENT READY")
    assert display.waitKey(0) == -1
    monkeypatch.setattr(bridge.os, "getppid", lambda: 1)
    assert display.waitKey(0) == ord("q")
    assert display.events[-1]["operator_source"] == "gui_process_lost"


def test_start_button_never_arms_assistance_after_the_pose_drifts(monkeypatch, tmp_path):
    import gello_hil.alignment as module
    from gello_hil.assisted import make_alignment_runner
    from test_alignment_safety import POSE, setup

    bus, controller, _ = setup(target_offset=0)
    bus.close = lambda: None
    display = FileDisplay(tmp_path, motor_permitted=True)
    clicks = iter(["start", "abort"])

    def click(_):
        button = next(clicks)
        if button == "abort":
            assert display.state["phase"] == "PAUSED"
            assert "Not aligned" in display.state["notice"]
            assert controller.summary()["motor_register_writes"] == 0
            return ord("q")
        display.last_button = button
        return 13

    monkeypatch.setattr(display, "waitKey", click)
    args = SimpleNamespace(baudrate=57600, port="unused", serial_timeout=0.12, window_scale=3,
                           max_active_steps=1000, gui_display=display)
    runner = make_alignment_runner(args, controller.calibration)
    monkeypatch.setattr(runner, "_bus", lambda: bus)
    monkeypatch.setattr(runner, "_controller", lambda b: controller)
    monkeypatch.setattr(
        runner,
        "_read_only_bus",
        lambda: SimpleNamespace(
            read_positions=lambda ids: {i: bus.registers[i]["present_position_counts"] for i in ids},
            close=lambda: None,
        ),
    )
    clock = controller._clock
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=clock, sleep=clock.sleep))
    monkeypatch.setattr(
        module.passive, "prepare_observation", lambda *a: (None, np.zeros((64, 64, 3), np.uint8))
    )
    result = runner(
        env=SimpleNamespace(get_sim_state=lambda: np.zeros(10)),
        observation={},
        follower_target_joints=POSE + 0.5,  # beyond the near-pose start gate
        task_instruction="native",
        task_id=0,
        state_id=0,
        attempt_index=1,
        resize_size=64,
    )
    assert result["aborted"] and not result["collection_start_eligible"]
    assert all(s["motor_register_writes"] == 0 for s in result["sessions"])


def test_retry_cannot_abort_a_running_attempt():
    state = {"mode": "collection", "phase": "PAUSED"}
    assert not button_permissions(state, writable=True, running=True)["retry"]
    assert button_permissions(state, writable=True, running=False)["retry"]


def test_incomplete_attempt_is_listed_without_blocking_review(tmp_path):
    pool.init_pool(tmp_path)
    enable_review(tmp_path)
    success = fake_attempt(tmp_path, "object_07", 1)
    (tmp_path / "attempts/object_07/shot02/20261102T000000Z").mkdir(parents=True)
    with pytest.raises(ValueError, match="unresolved"):
        pool.inspect_pool(tmp_path)
    status = pool.inspect_pool(tmp_path, strict=False)
    slot = next(s for s in status["cases"] if (s["task_key"], s["shot"]) == ("object_07", 2))
    assert slot["details"][0]["incomplete"] and not slot["attempts"]
    confirm_attempt(tmp_path, str(success.relative_to(tmp_path)))
    assert pool.inspect_pool(tmp_path, strict=False)["accepted"] == 1


def test_attempt_header_cache_follows_file_changes(tmp_path):
    import json

    pool.init_pool(tmp_path)
    attempt = fake_attempt(tmp_path, "object_07", 1)
    assert pool.inspect_pool(tmp_path)["failed_attempts"] == 0
    meta = json.loads((attempt / "attempt.json").read_text())
    (attempt / "attempt.json").write_text(json.dumps({**meta, "success": False, "note": "changed"}))
    assert pool.inspect_pool(tmp_path)["failed_attempts"] == 1


def test_live_handoff_maps_signed_motor_positions_to_one_turn():
    from test_live_handoff_a6 import ready

    bus, _, handoff, _, _ = ready()
    handoff.tick = lambda *a: None
    original = bus.sync_read_control_item
    bus.sync_read_control_item = lambda name, ids: {i: -1028 for i in ids}
    assert handoff.read_positions((1, 5)) == {1: 3068, 5: 3068}
    bus.sync_read_control_item = original
    handoff.close()


def test_live_hud_shows_budget_gripper_success_hold_and_encoder_faults():
    pytest.importorskip("PySide6")
    from gello_hil.collector_qt import collection_summary

    text = collection_summary({"mode": "collection", "active_steps": 12, "max_active_steps": 1000,
                               "success_streak": 3, "success_hold_steps": 20, "gripper_ratio": 0.9,
                               "max_abs_joint_error_rad": 0.05, "control_hz": 20.0})
    assert "Step 12 / 1000" in text and "Success hold 3 / 20" in text
    assert "Gripper CLOSED" in text and "20.0 Hz" in text
    fault = collection_summary({"mode": "collection", "notice": "ENCODER LINK LOST",
                                "encoder_fault": "TimeoutError: missing IDs"})
    assert "ENCODER LINK LOST" in fault and "missing IDs" in fault
