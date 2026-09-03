from fake_motors import FakeXL330Transport
import numpy as np
import pytest

from gello_hil.calibration import GelloCalibration
from gello_hil.dynamixel import COUNTS_PER_REVOLUTION
from gello_hil.actuation import (
    GelloActuationSafetyError,
    GelloAlignmentAssistController,
    GelloAlignmentAssistLimits,
    HomingState,
    plan_follower_joint_alignment,
)


ARM_IDS = tuple(range(1, 8))
MODELS = {**{motor_id: 1200 for motor_id in ARM_IDS}, 8: 1190}
LIMITS = (
    (-2.8973, 2.8973),
    (-1.7628, 1.7628),
    (-2.8973, 2.8973),
    (-3.0718, -0.0698),
    (-2.8973, 2.8973),
    (-0.0175, 3.7525),
    (-2.8973, 2.8973),
)
POSE = np.asarray((0.0, 0.0, 0.0, -np.pi / 2, 0.0, np.pi / 2, 0.0))
OFFSETS = np.asarray((0, 1, 2, 3, 0, 1, 2), dtype=np.float64) * np.pi / 2
SIGNS = np.asarray((1, 1, 1, -1, 1, -1, 1), dtype=np.int64)


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += float(seconds)


def counts_for(joints):
    return (
        np.rint((OFFSETS + SIGNS * np.asarray(joints)) * COUNTS_PER_REVOLUTION / (2 * np.pi))
        .astype(np.int64)
        % COUNTS_PER_REVOLUTION
    )


def calibration():
    value = GelloCalibration(
        arm_ids=ARM_IDS,
        gripper_id=8,
        calibration_counts=tuple(int(x) for x in counts_for(POSE)),
        reference_pose=tuple(float(x) for x in POSE),
        assembly_offsets_radians=tuple(float(x) for x in OFFSETS),
        joint_limits=LIMITS,
        joint_signs=tuple(int(x) for x in SIGNS),
        gripper_open_count=4000,
        gripper_closed_count=200,
    )
    value.validate()
    return value


def setup(target_offset=0.08, *, limits=None):
    initial = counts_for(POSE)
    positions = {**{motor_id: int(initial[motor_id - 1]) for motor_id in ARM_IDS}, 8: 4000}
    bus = FakeXL330Transport(
        model_numbers=MODELS,
        present_positions=positions,
        maximum_follow_counts_per_tick=2,
        control_hz=5.0,
    )
    for motor_id in ARM_IDS:
        bus.registers[motor_id]["operating_mode"] = 5
    clock = Clock()
    limits = limits or GelloAlignmentAssistLimits()
    controller = GelloAlignmentAssistController(
        bus=bus,
        calibration=calibration(),
        expected_model_numbers=MODELS,
        limits=limits,
        clock=clock,
        sleep=clock.sleep,
    )
    target = POSE + float(target_offset)
    plan = plan_follower_joint_alignment(
        calibration(), target, initial, selected_motor_ids=ARM_IDS, limits=controller.limits
    )
    return bus, controller, plan


def arm(bus, controller, plan):
    controller.read_only_preflight()
    controller.verify_safe_off()
    controller.prepare_alignment(plan)
    before = [bus.registers[motor_id]["present_position_counts"] for motor_id in ARM_IDS]
    controller.arm_alignment()
    after = [bus.registers[motor_id]["present_position_counts"] for motor_id in ARM_IDS]
    assert before == after


def test_assist_starts_zero_jump_and_never_torques_gripper():
    bus, controller, plan = setup()
    arm(bus, controller, plan)
    torque_on = {
        entry["motor_ids"][0]
        for entry in bus.command_log
        if entry["item"] == "torque_enable" and entry["value"] == 1
    }
    assert torque_on == set(ARM_IDS)
    assert bus.registers[8]["torque_enable"] == 0


def test_guide_waypoints_are_bounded_and_point_toward_target():
    bus, controller, plan = setup()
    arm(bus, controller, plan)
    before = {motor_id: bus.registers[motor_id]["present_position_counts"] for motor_id in ARM_IDS}
    result = controller.guide_once()
    for motor_id in ARM_IDS:
        goal = result["commanded_waypoint_counts"][str(motor_id)]
        target = controller.target_goal_counts[motor_id]
        assert abs(goal - before[motor_id]) <= 20
        assert np.sign(goal - before[motor_id]) == np.sign(target - before[motor_id])


def test_reported_error_is_target_minus_actual_in_panda_joint_coordinates():
    bus, controller, plan = setup(target_offset=0.08)
    arm(bus, controller, plan)

    result = controller.guide_once()
    target = np.asarray(result["target_joint_positions"], dtype=np.float64)
    actual = np.asarray(result["mapped_gello_joint_positions"], dtype=np.float64)
    error = np.asarray(result["target_error_radians"], dtype=np.float64)

    assert np.allclose(error, target - actual, atol=1e-12)
    # J4 and J6 have negative encoder signs in this calibration. The operator
    # display must still report positive Panda-joint error for a positive
    # seven-joint target offset, rather than raw motor-space signs.
    assert np.all(error > 0.0)
    assert len(result["present_position_counts"]) == 7


def test_ready_requires_stable_dwell_and_release_enables_collection():
    bus, controller, plan = setup(target_offset=0.01)
    arm(bus, controller, plan)
    for index in range(controller.assist_limits.ready_stable_cycles):
        result = controller.guide_once()
        assert result["alignment_ready"] is (index + 1 == controller.assist_limits.ready_stable_cycles)
    assert controller.state == HomingState.HOLDING_AT_TARGET
    for motor_id in ARM_IDS:
        bus.registers[motor_id]["present_velocity_raw"] = 0
    verification = controller.release_and_verify_for_collection()
    assert verification["passed"]
    assert controller.summary()["collection_start_eligible"]
    assert all(bus.registers[motor_id]["torque_enable"] == 0 for motor_id in (*ARM_IDS, 8))


def test_post_release_drift_blocks_collection_start():
    bus, controller, plan = setup(target_offset=0.0)
    arm(bus, controller, plan)
    for _ in range(controller.assist_limits.ready_stable_cycles):
        controller.guide_once()
    # Inject drift only after soften-and-release, when the post-release
    # read-only samples begin.
    original_snapshot = controller.snapshot
    drifted = False

    def drifting_snapshot(*args, **kwargs):
        nonlocal drifted
        if not drifted:
            bus.registers[1]["present_position_counts"] += 35
            drifted = True
        return original_snapshot(*args, **kwargs)

    controller.snapshot = drifting_snapshot
    verification = controller.release_and_verify_for_collection()
    assert not verification["passed"]
    assert not controller.summary()["collection_start_eligible"]


def test_ready_screen_keeps_monitoring_and_returns_active_if_alignment_is_lost():
    bus, controller, plan = setup(target_offset=0.0)
    arm(bus, controller, plan)
    for _ in range(controller.assist_limits.ready_stable_cycles):
        controller.guide_once()
    sample = controller.maintain_ready_once()
    assert sample["alignment_ready"]
    assert sample["ready_hold_sample"]
    bus.registers[1]["present_position_counts"] += 35
    sample = controller.maintain_ready_once()
    assert not sample["alignment_ready"]
    assert controller.state == HomingState.APPROACHING


def test_clutch_cleanup_is_all_id_torque_off_and_consumes_no_d31_budget():
    bus, controller, plan = setup()
    arm(bus, controller, plan)
    controller.guide_once()
    controller.finish(reason="operator_clutch")
    summary = controller.summary()
    assert summary["torque_off_verified_all_ids"]
    assert summary["simulator_steps"] == 0
    assert summary["active_action_budget_used"] == 0
    assert all(bus.registers[motor_id]["torque_enable"] == 0 for motor_id in (*ARM_IDS, 8))


def test_passive_motion_at_enter_waits_for_stable_dwell_without_writes():
    bus, controller, _ = setup()
    bus.registers[1]["present_velocity_raw"] = 100
    controller.read_only_preflight()
    first = controller.passive_settle_once()
    assert not first["stable_now"]

    bus.registers[1]["present_position_counts"] += 10
    moved = controller.passive_settle_once()
    assert not moved["stable_now"]
    assert moved["stable_streak"] == 0

    samples = [
        controller.passive_settle_once()
        for _ in range(controller.assist_limits.pre_arm_stable_cycles)
    ]
    assert samples[-1]["settled_for_arm"]
    assert bus.motor_register_writes == 0
    assert bus.command_log == []
    assert all(bus.registers[motor_id]["torque_enable"] == 0 for motor_id in (*ARM_IDS, 8))


def test_passive_settle_uses_two_aggregate_reads_and_no_sequential_reads():
    bus, controller, _ = setup()
    controller.read_only_preflight()
    before = dict(bus.instruction_counts)

    controller.passive_settle_once()

    assert bus.instruction_counts["SYNC_READ"] - before["SYNC_READ"] == 2
    assert bus.instruction_counts["READ"] - before["READ"] == 0


def test_alignment_arming_refreshes_unchanged_watchdog_between_sequential_stages():
    bus, controller, plan = setup()
    arm(bus, controller, plan)

    event = next(
        item for item in controller.events
        if item["event_type"] == "arming_watchdogs_refreshed"
    )
    assert event["refresh_count"] == 21
    assert event["bus_watchdog_ticks"] == 20
    assert all(bus.registers[motor_id]["bus_watchdog"] == 20 for motor_id in ARM_IDS)


def test_human_guided_velocity_cap_allows_measured_motion_but_still_faults_runaway():
    bus, controller, plan = setup()
    arm(bus, controller, plan)
    original = bus.sync_read_active_snapshots

    def snapshot_with_velocity(raw_velocity):
        def read(motor_ids):
            result = original(motor_ids)
            result[1]["present_velocity_raw"] = raw_velocity
            return result

        return read

    bus.sync_read_active_snapshots = snapshot_with_velocity(62)  # about 1.49 rad/s
    controller.monitor_once()
    bus.sync_read_active_snapshots = snapshot_with_velocity(63)  # about 1.51 rad/s
    with pytest.raises(GelloActuationSafetyError, match="velocity exceeds"):
        controller.monitor_once()
    controller.finish(reason="test_cleanup")


def test_powered_assist_monitor_uses_two_sync_reads_and_no_sequential_reads():
    bus, controller, plan = setup()
    arm(bus, controller, plan)
    before = dict(bus.instruction_counts)

    controller.monitor_once()

    assert bus.instruction_counts["SYNC_READ"] - before["SYNC_READ"] == 2
    assert bus.instruction_counts["READ"] - before["READ"] == 0
    assert controller.monitor_deadline_miss_count == 0


def test_two_transaction_monitor_meets_five_hz_with_physical_bus_scale_latency():
    class TimedSyncTransport(FakeXL330Transport):
        delay_enabled = False

        def sync_read_control_item(self, name, motor_ids):
            if self.delay_enabled:
                clock.sleep(0.02)
            return super().sync_read_control_item(name, motor_ids)

        def sync_read_active_snapshots(self, motor_ids):
            if self.delay_enabled:
                clock.sleep(0.12)
            return super().sync_read_active_snapshots(motor_ids)

    initial = counts_for(POSE)
    positions = {**{motor_id: int(initial[motor_id - 1]) for motor_id in ARM_IDS}, 8: 4000}
    clock = Clock()
    bus = TimedSyncTransport(model_numbers=MODELS, present_positions=positions, control_hz=5.0)
    for motor_id in ARM_IDS:
        bus.registers[motor_id]["operating_mode"] = 5
    controller = GelloAlignmentAssistController(
        bus=bus,
        calibration=calibration(),
        expected_model_numbers=MODELS,
        limits=GelloAlignmentAssistLimits(),
        clock=clock,
        sleep=clock.sleep,
    )
    plan = plan_follower_joint_alignment(
        calibration(), POSE + 0.08, initial, selected_motor_ids=ARM_IDS, limits=controller.limits
    )
    arm(bus, controller, plan)
    bus.delay_enabled = True

    controller.monitor_once()

    assert controller.monitor_cycle_seconds[-1] == pytest.approx(0.14)
    assert controller.monitor_deadline_miss_count == 0


def test_pre_arm_stability_allows_small_serial_latch_motion():
    bus, controller, plan = setup()
    controller.read_only_preflight()
    for _ in range(controller.assist_limits.pre_arm_stable_cycles + 1):
        settled = controller.passive_settle_once()
    assert settled["settled_for_arm"]
    controller.verify_safe_off()

    original = bus.read_control_item
    position_reads = 0

    def read_with_one_latch_shift(motor_id, name):
        nonlocal position_reads
        if motor_id == 1 and name == "present_position_counts":
            position_reads += 1
            if position_reads == 2:
                bus.registers[1]["present_position_counts"] += 22
        return original(motor_id, name)

    bus.read_control_item = read_with_one_latch_shift
    prepared = controller.prepare_alignment(plan)
    assert prepared["positions"][1] == int(counts_for(POSE)[0]) + 22
    controller.finish(reason="test_cleanup")


@pytest.mark.parametrize(
    "override,match",
    [
        ({"goal_current_ma": 101}, "100 mA"),
        ({"position_p_gain": 751}, "750"),
        ({"maximum_guide_offset_counts": 21}, "20 counts"),
        ({"maximum_latch_motion_counts": 33}, "latch tolerance"),
        ({"maximum_velocity_rad_s": 1.51}, "velocity cap"),
    ],
)
def test_registered_assist_caps_cannot_be_raised(override, match):
    payload = {**GelloAlignmentAssistLimits().__dict__, **override}
    with pytest.raises(ValueError, match=match):
        GelloAlignmentAssistLimits.from_dict(payload)

