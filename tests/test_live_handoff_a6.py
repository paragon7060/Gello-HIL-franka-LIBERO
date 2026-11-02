from dataclasses import replace
from types import SimpleNamespace

import pytest

from gello_hil.actuation import GelloAlignmentAssistLimits, GelloActuationSafetyError, HomingState
from gello_hil.live_handoff import LiveTorqueHandoff
from test_alignment_safety import ARM_IDS, arm, setup


def ready():
    limits = GelloAlignmentAssistLimits(profile="GELLO-ALIGN-A6",
        ready_max_abs_error_rad=.60, ready_rms_error_rad=.60)
    bus, controller, plan = setup(target_offset=0, limits=limits)
    arm(bus, controller, plan)
    for _ in range(limits.ready_stable_cycles):
        controller.guide_once()
    assert controller.state == HomingState.HOLDING_AT_TARGET
    closed = []
    bus.close = lambda: closed.append(True)
    handoff = LiveTorqueHandoff(controller, bus, clock=controller._clock)
    opened = []
    def reader():
        assert closed and all(bus.registers[i]["torque_enable"] == 0 for i in (*ARM_IDS, 8))
        opened.append(True)
        return SimpleNamespace(read_positions=lambda ids: {i: bus.registers[i]["present_position_counts"] for i in ids}, close=lambda: None)
    handoff.read_only_factory = reader
    return bus, controller, handoff, opened, closed


def test_a6_single_writer_monotonic_fade_and_gripper_off():
    bus, controller, handoff, opened, closed = ready()
    handoff.begin()
    currents = []
    for step in range(12):
        handoff.tick(step)
        handoff.read_positions((*ARM_IDS, 8))
        currents.append(controller.current_goal_ma)
        assert bus.registers[8]["torque_enable"] == 0
        controller._clock.sleep(.21)
    assert currents == sorted(currents, reverse=True)
    assert max(currents) <= 100
    assert not handoff.powered and closed and opened
    assert handoff.summary()["torque_off_verified"]
    handoff.close()


def test_a6_backdrive_cap_releases_early_instead_of_raising_force():
    bus, controller, handoff, _, _ = ready()
    handoff.begin()
    bus.registers[1]["goal_position_counts"] = bus.registers[1]["present_position_counts"] + 200
    bus.registers[1]["present_velocity_raw"] = 0
    handoff.tick()
    assert not handoff.powered
    assert any(e.get("reason") == "operator_backdrive_early_release" for e in handoff.events)
    assert all(bus.registers[i]["torque_enable"] == 0 for i in (*ARM_IDS, 8))


def test_a6_safety_fault_and_keyboard_interrupt_cleanup():
    bus, controller, handoff, _, closed = ready()
    with pytest.raises(GelloActuationSafetyError):
        with handoff:
            handoff.begin()
            bus.registers[1]["temperature_c"] = 60
            handoff.tick()
    assert closed
    assert all(bus.registers[i]["torque_enable"] == 0 for i in (*ARM_IDS, 8))
    bus, controller, handoff, _, closed = ready()
    with pytest.raises(KeyboardInterrupt):
        with handoff:
            handoff.begin()
            raise KeyboardInterrupt()
    assert closed and handoff.closed


def test_a6_clutch_and_start_timeout_cleanup():
    bus, controller, handoff, _, closed = ready()
    handoff.begin()
    handoff.release_now("operator_clutch")
    assert closed and not handoff.powered
    bus, controller, handoff, _, closed = ready()
    controller._clock.sleep(2.1)
    with pytest.raises(GelloActuationSafetyError, match="initialization"):
        with handoff:
            handoff.tick()
    assert closed


def test_a6_does_not_raise_existing_force_limits():
    original = GelloAlignmentAssistLimits(profile="GELLO-ALIGN-A6",
                                        ready_max_abs_error_rad=.60, ready_rms_error_rad=.60)
    for changes in [{"goal_current_ma": 101}, {"maximum_tracking_error_counts": 41},
                    {"position_p_gain": 751}, {"maximum_guide_offset_counts": 21}]:
        with pytest.raises(ValueError):
            replace(original, **changes).validate()


@pytest.mark.parametrize("cancel", [None, "ready_window_closed"])
def test_a6_runner_transfers_bus_or_closes_on_gui_abort(monkeypatch, cancel):
    import numpy as np
    import gello_hil.alignment as module
    from gello_hil.assisted import make_alignment_runner
    from test_alignment_safety import POSE
    limits = GelloAlignmentAssistLimits(profile="GELLO-ALIGN-A6",
                                        ready_max_abs_error_rad=.6, ready_rms_error_rad=.6)
    bus, controller, _ = setup(target_offset=0, limits=limits)
    clock = controller._clock
    closed = []
    bus.close = lambda: closed.append(True)
    args = SimpleNamespace(baudrate=57600, port="unused", serial_timeout=.12,
                           window_scale=3, max_active_steps=1000)
    runner = make_alignment_runner(args, controller.calibration)
    monkeypatch.setattr(runner, "_bus", lambda: bus)
    monkeypatch.setattr(runner, "_controller", lambda b: controller)
    monkeypatch.setattr(runner, "_read_only_bus", lambda: SimpleNamespace(
        read_positions=lambda ids: {i: bus.registers[i]["present_position_counts"] for i in ids},
        close=lambda: None))
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=clock, sleep=clock.sleep))
    monkeypatch.setattr(module, "near_pose_passive_start_ready", lambda *a: False)
    monkeypatch.setattr(module.passive, "prepare_observation", lambda *a: (None, np.zeros((64,64,3),np.uint8)))
    current = {"phase": "PAUSED"}
    draw = module.draw_alignment_panel
    def panel(*a, **kw):
        current["phase"] = kw["status"]
        return draw(*a, **kw)
    monkeypatch.setattr(module, "draw_alignment_panel", panel)
    for name in ["namedWindow", "resizeWindow", "imshow"]:
        monkeypatch.setattr(module.cv2, name, lambda *a: None)
    monkeypatch.setattr(module.cv2, "waitKey", lambda *a: 13)
    monkeypatch.setattr(module.cv2, "getWindowProperty", lambda *a: 0 if cancel and current["phase"] == "ALIGNMENT READY" else 1)
    monkeypatch.setattr(module.passive, "close_operator_window", lambda *a: None)
    env = SimpleNamespace(get_sim_state=lambda: np.zeros(10))  # no step method
    result = runner(env=env, observation={}, follower_target_joints=POSE,
                    task_instruction="native", task_id=0, state_id=0, attempt_index=1, resize_size=64)
    assert result["simulator_state_byte_exact"] and result["simulator_steps"] == 0
    assert result["collection_start_eligible"] is (cancel is None)
    if cancel:
        assert result["aborted"] and closed and runner.live_handoff is None
    else:
        assert not closed and runner.live_handoff is not None
        runner.live_handoff.close()
        assert closed
    assert all(bus.registers[i]["torque_enable"] == 0 for i in (*ARM_IDS, 8))
