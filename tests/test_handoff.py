from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from gello_hil.actuation import GelloAlignmentAssistLimits, HomingState
from gello_hil.handoff import (
    HandoffCancelled, visible_hold_then_release,
)
from test_alignment_safety import ARM_IDS, POSE, arm, setup
import gello_hil.alignment as MODULE
from gello_hil.assisted import make_alignment_runner


A5 = GelloAlignmentAssistLimits(
    profile="GELLO-ALIGN-A5", ready_max_abs_error_rad=0.15, ready_rms_error_rad=0.15,
)


def ready():
    bus, controller, plan = setup(target_offset=0.0, limits=A5)
    arm(bus, controller, plan)
    for _ in range(A5.ready_stable_cycles):
        controller.guide_once()
    assert controller.state == HomingState.HOLDING_AT_TARGET
    return bus, controller


def test_a5_tolerance_lock_preserves_legacy_and_force_limits():
    A5.validate()
    for invalid in (
        replace(A5, ready_max_abs_error_rad=0.151),
        replace(A5, ready_rms_error_rad=0.151),
        replace(A5, profile="GELLO-ALIGN-A4"),
        replace(A5, profile="unknown"),
        replace(A5, goal_current_ma=101),
        replace(A5, position_p_gain=751),
    ):
        with pytest.raises(ValueError):
            invalid.validate()
    GelloAlignmentAssistLimits().validate()


@pytest.mark.parametrize("error,expected", [(0.149, True), (0.15, True), (0.151, False)])
def test_per_joint_inclusive_error_boundary(error, expected):
    bus, controller = ready()
    target = POSE.copy()
    target[0] += error
    controller.alignment_target_joint_positions = target
    diagnostic = controller._alignment_diagnostic(controller.snapshot(ARM_IDS))
    assert diagnostic["stable_now"] is expected
    assert controller.summary()["protocol_version"] == "GELLO-ALIGN-A5"
    controller.finish(reason="test")


def test_two_seconds_after_render_no_force_writes_and_verified_release():
    bus, controller = ready()
    clock = controller._clock
    events, frames = [], []
    original_writes = bus.motor_register_writes
    original_reads = bus.instruction_counts["SYNC_READ"]

    def show(phase, remaining, diagnostic):
        frames.append((phase, remaining, clock()))
        if phase == "POWERED HOLD":
            assert bus.motor_register_writes == original_writes
            assert all(bus.registers[i]["torque_enable"] == 1 for i in ARM_IDS)
            assert bus.registers[8]["torque_enable"] == 0
        if len(frames) == 1:
            clock.sleep(0.04)  # display setup must not eat into the two seconds

    verification = visible_hold_then_release(
        controller, show_frame=show, poll_key=lambda: 13, events=events,
        clock=clock, sleep=clock.sleep,
    )
    assert verification["passed"]
    assert verification["torque_off_verified_all_ids"]
    assert bus.instruction_counts["SYNC_READ"] >= original_reads + 20
    visible = next(e for e in events if e["event_type"] == "teleop_hold_visible")
    complete = next(e for e in events if e["event_type"] == "teleop_hold_complete")
    assert visible["monotonic_elapsed_seconds"] >= 0.04
    assert complete["visible_elapsed_seconds"] == pytest.approx(2.0)
    assert all(e["active_step_index"] == e["active_action_budget_used"] == e["simulator_steps"] == 0 for e in events)
    assert controller.state == HomingState.RELEASED_FOR_TELEOP
    assert all(bus.registers[i]["torque_enable"] == 0 for i in (*ARM_IDS, 8))


@pytest.mark.parametrize("key,abort", [(32, False), (ord("q"), True)])
@pytest.mark.parametrize("during_release", [False, True])
def test_operator_cancel_during_hold_or_release(key, abort, during_release):
    bus, controller = ready()
    events = []
    phase = None

    def show(current_phase, *args):
        nonlocal phase
        phase = current_phase

    try:
        with pytest.raises(HandoffCancelled) as caught:
            visible_hold_then_release(
                controller, show_frame=show,
                poll_key=lambda: key if (not during_release or phase != "POWERED HOLD") else -1,
                events=events, clock=controller._clock, sleep=controller._sleep,
            )
        assert caught.value.abort is abort
    finally:
        controller.finish(reason="caller_cleanup")
    assert events[-1]["event_type"] == "teleop_handoff_cancelled"
    assert all(bus.registers[i]["torque_enable"] == 0 for i in (*ARM_IDS, 8))


def test_readiness_loss_does_not_release_or_start_collection():
    bus, controller = ready()
    writes = bus.motor_register_writes
    events = []

    def show(*args):
        controller.alignment_target_joint_positions = POSE + 0.16

    result = visible_hold_then_release(
        controller, show_frame=show, poll_key=lambda: -1, events=events,
        clock=controller._clock, sleep=controller._sleep,
    )
    assert result is None
    assert controller.state == HomingState.APPROACHING
    assert not controller.collection_start_eligible
    assert bus.motor_register_writes == writes
    assert events[-1]["event_type"] == "teleop_hold_ready_lost"
    controller.finish(reason="test")


def test_gui_failure_is_recorded_before_caller_cleanup():
    bus, controller = ready()
    events = []

    def broken_gui(*args):
        raise RuntimeError("GUI failed")

    try:
        with pytest.raises(RuntimeError, match="GUI failed"):
            visible_hold_then_release(
                controller, show_frame=broken_gui, poll_key=lambda: -1,
                events=events, clock=controller._clock, sleep=controller._sleep,
            )
    finally:
        controller.finish(reason="gui_failure_cleanup")
    assert events[-1]["event_type"] == "teleop_handoff_cancelled"
    assert not controller.collection_start_eligible
    assert all(bus.registers[i]["torque_enable"] == 0 for i in (*ARM_IDS, 8))


@pytest.mark.parametrize("cancel", [None, "q", "space", "gui_failure", "closed_window"])
def test_runner_owns_powered_handoff_cleanup_and_shared_window(monkeypatch, cancel):
    bus, controller, _ = setup(target_offset=0.0, limits=A5)
    clock = controller._clock
    args = SimpleNamespace(baudrate=57600, port="unused", serial_timeout=0.12, window_scale=3, max_active_steps=1000)
    runner = make_alignment_runner(args, controller.calibration)
    # Keep this legacy A5 fixture explicit; the shipped CLI now defaults to A6.
    from dataclasses import asdict
    from gello_hil.handoff import HANDOFF_CONTRACT
    runner = MODULE.AlignmentRunner(
        config={**runner.config, "alignment_limits": asdict(A5), "alignment_handoff": dict(HANDOFF_CONTRACT)},
        calibration=controller.calibration,
    )
    closed, displays, window_closes = [], [], []
    current = {"phase": "PAUSED", "cancelled": False}

    def close():
        assert all(bus.registers[i]["torque_enable"] == 0 for i in (*ARM_IDS, 8))
        closed.append(True)

    bus.close = close
    read_bus = SimpleNamespace(
        read_positions=lambda ids: {i: bus.registers[i]["present_position_counts"] for i in ids},
        close=lambda: None,
    )
    monkeypatch.setattr(runner, "_bus", lambda: bus)
    monkeypatch.setattr(runner, "_read_only_bus", lambda: read_bus)
    monkeypatch.setattr(runner, "_controller", lambda transport: controller)
    monkeypatch.setattr(MODULE, "time", SimpleNamespace(monotonic=clock, sleep=clock.sleep))
    # Exercise powered alignment even though this synthetic pose starts near.
    monkeypatch.setattr(MODULE, "near_pose_passive_start_ready", lambda *args: False)
    monkeypatch.setattr(MODULE.passive, "prepare_observation", lambda *args: (None, np.zeros((256, 256, 3), dtype=np.uint8)))
    alignment_panel = MODULE.draw_alignment_panel
    collection_panel = MODULE.passive.draw_attempt_panel

    def draw_alignment(*args, **kwargs):
        current["phase"] = kwargs["status"]
        return alignment_panel(*args, **kwargs)

    def draw_collection(*args, **kwargs):
        current["phase"] = kwargs["handoff_status"]
        if cancel == "gui_failure":
            raise RuntimeError("simulated GUI failure")
        return collection_panel(*args, **kwargs)

    def key(delay):
        if current["phase"].startswith("POWERED HOLD") and cancel in ("q", "space"):
            current["cancelled"] = True
            return ord("q") if cancel == "q" else 32
        if current["cancelled"] and current["phase"] == "PAUSED":
            return ord("q")
        return 13

    monkeypatch.setattr(MODULE, "draw_alignment_panel", draw_alignment)
    monkeypatch.setattr(MODULE.passive, "draw_attempt_panel", draw_collection)
    monkeypatch.setattr(MODULE.cv2, "namedWindow", lambda *args: None)
    monkeypatch.setattr(MODULE.cv2, "resizeWindow", lambda *args: None)
    monkeypatch.setattr(MODULE.cv2, "imshow", lambda name, display: displays.append(name))
    monkeypatch.setattr(MODULE.cv2, "waitKey", key)
    monkeypatch.setattr(MODULE.cv2, "getWindowProperty", lambda *args: 0 if cancel == "closed_window" and current["phase"].startswith("POWERED HOLD") else 1)
    monkeypatch.setattr(MODULE.passive, "close_operator_window", lambda name: window_closes.append(name))
    sim_state = np.arange(9, dtype=np.float64)
    env = SimpleNamespace(get_sim_state=lambda: sim_state.copy())  # no step method
    result = runner(
        env=env, observation={}, follower_target_joints=POSE.copy(),
        task_instruction="put the ketchup in the basket", task_id=4, state_id=3,
        attempt_index=1, resize_size=256,
    )
    expected_window = MODULE.passive.collection_window_name(
        "GELLO-HIL-ASSIST-v1", task_id=4, state_id=3, attempt_index=1,
    )
    assert set(displays) == {expected_window}
    assert closed
    assert result["protocol_version"] == "GELLO-ALIGN-A5"
    assert result["simulator_state_byte_exact"]
    assert result["simulator_steps"] == result["active_action_budget_used"] == 0
    assert result["motor_write_transport_closed_before_collection"]
    assert result["collection_start_eligible"] is (cancel is None)
    assert result["same_window_handed_to_collection"] is (cancel is None)
    assert window_closes == ([] if cancel is None else [expected_window])
    assert all(bus.registers[i]["torque_enable"] == 0 for i in (*ARM_IDS, 8))
    if cancel is None:
        session = MODULE.passive.GelloOperatorSession(clock=clock)
        assert not session.active
        assert session.active_step_count == 0
        assert result["teleop_handoff_events"][-1]["verification"]["passed"]
    elif cancel == "space":
        assert result["pause_count"] == 1


