"""Pre-collection alignment UI; live environment is frozen throughout."""
from __future__ import annotations
import time
import cv2
import numpy as np
from .ui import wrap_pixels
from . import assist_ui as passive
from .calibration import GelloCalibration
from .dynamixel import COUNTS_PER_REVOLUTION, ReadOnlyDynamixelBus
from .actuation import (GelloActuationSafetyError, GelloActuationShutdownUnverified,
    GelloAlignmentAssistController, GelloAlignmentAssistLimits, SafeXL330Transport,
    plan_follower_joint_alignment)
from .handoff import HANDOFF_CONTRACT, HandoffCancelled, visible_hold_then_release
from .live_handoff import LIVE_HANDOFF_CONTRACT, LiveTorqueHandoff
PROTOCOL_VERSION = "GELLO-HIL-ASSIST-v1"


def _json_safe(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value

def target_error_diagnostic(
    calibration: GelloCalibration,
    follower_target_joints: np.ndarray,
    counts: dict[int, int] | np.ndarray,
    *,
    tolerance_rad: float,
) -> dict:
    """Describe target-minus-actual GELLO error in Panda joint coordinates."""

    target = np.asarray(follower_target_joints, dtype=np.float64)
    raw_counts = (
        np.asarray([counts[motor_id] for motor_id in calibration.arm_ids], dtype=np.int64)
        if isinstance(counts, dict)
        else np.asarray(counts, dtype=np.int64)
    )
    alignment = calibration.alignment_diagnostics(
        np.mod(raw_counts, COUNTS_PER_REVOLUTION),
        target,
        tolerance_rad=float(tolerance_rad),
    )
    mapped = np.asarray(
        alignment["mapped_leader_joint_positions"], dtype=np.float64
    )
    errors = np.asarray(alignment["alignment_error_radians"], dtype=np.float64)
    return {
        "present_position_counts": raw_counts.tolist(),
        "target_joint_positions": target.tolist(),
        "mapped_gello_joint_positions": mapped.tolist(),
        "target_error_radians": errors.tolist(),
        "required_count_deltas": np.asarray(
            alignment["required_count_deltas"], dtype=np.int64
        ).tolist(),
        "max_abs_target_error_rad": float(np.max(np.abs(errors))),
        "rms_target_error_rad": float(np.sqrt(np.mean(np.square(errors)))),
        "alignment_ready": bool(alignment["alignment_ready"]),
    }

def near_pose_passive_start_ready(diagnostic: dict | None, contract: dict) -> bool:
    """Gate direct passive HITL start without weakening powered-assist safety."""

    if diagnostic is None:
        return False
    errors = np.asarray(diagnostic.get("target_error_radians", []), dtype=np.float64)
    mapped = np.asarray(
        diagnostic.get("mapped_gello_joint_positions", []), dtype=np.float64
    )
    if errors.shape != (7,) or mapped.shape != (7,):
        return False
    if not np.isfinite(errors).all() or not np.isfinite(mapped).all():
        return False
    return bool(
        float(diagnostic.get("max_abs_target_error_rad", np.inf))
        <= float(contract["max_abs_target_error_rad"])
        and float(diagnostic.get("rms_target_error_rad", np.inf))
        <= float(contract["rms_target_error_rad"])
    )

def draw_alignment_panel(
    frame: np.ndarray,
    *,
    instruction: str,
    task_id: int,
    state_id: int,
    attempt_index: int,
    status: str,
    diagnostic: dict | None,
    preview_error: str | None,
    notice: str,
    limits: GelloAlignmentAssistLimits,
    passive_start_ready: bool,
) -> np.ndarray:
    scale = 3
    camera = cv2.resize(
        cv2.cvtColor(frame, cv2.COLOR_RGB2BGR),
        (int(frame.shape[1]) * scale, int(frame.shape[0]) * scale),
        interpolation=cv2.INTER_NEAREST,
    )
    panel = np.zeros((camera.shape[0], 780, 3), dtype=np.uint8)
    color = (0, 220, 255) if status == "PAUSED" else (80, 230, 80)
    if status == "ALIGNMENT READY":
        color = (0, 255, 255)
    lines = [
        f"GELLO-HIL-ASSIST-v1 / RESUMABLE + ENTER RECHECK + {limits.profile}",
        f"TASK: {instruction}",
        f"task {task_id}  state {state_id}  attempt {attempt_index}",
        "",
        f"{status}",
        notice,
        (
            "ENTER: START PASSIVE HITL NOW (torque stays off; no settle)"
            if passive_start_ready and status in ("PAUSED", "ARMING SETTLE")
            else "ENTER: START control + 2s force fade at READY"
            if limits.profile == "GELLO-ALIGN-A6" and status == "ALIGNMENT READY"
            else "ENTER: settle+assist / release at READY"
        ),
        "SPACE: torque-off clutch | q: abort",
        "Move slowly; keep one hand at the physical 5 V disconnect.",
        "Simulator/action budget: FROZEN (0 consumed)",
        f"Assist: {limits.goal_current_ma} mA, P={limits.position_p_gain}, "
        f"waypoint <= {limits.maximum_guide_offset_counts} counts",
        f"READY tolerance max/RMS: {limits.ready_max_abs_error_rad:.2f}/"
        f"{limits.ready_rms_error_rad:.2f} rad",
    ]
    if diagnostic is not None:
        if status == "ARMING SETTLE":
            lines.extend(
                [
                    "",
                    f"SETTLE {int(diagnostic.get('stable_streak', 0))}/{limits.pre_arm_stable_cycles} | "
                    f"max vel raw={max((abs(int(v)) for v in diagnostic.get('present_velocity_raw', [0])), default=0)} | "
                    f"max drift={max((abs(int(v)) for v in diagnostic.get('drift_counts', [0])), default=0)} counts",
                ]
            )
        errors = diagnostic.get("target_error_radians")
    else:
        errors = None
    if diagnostic is not None and errors is not None:
        targets = diagnostic.get("target_joint_positions", [0.0] * 7)
        actual = diagnostic.get("mapped_gello_joint_positions", [0.0] * 7)
        counts = diagnostic.get("present_position_counts", [0] * 7)
        lines.extend(
            [
                "",
                (
                    "NEAR-POSE PASSIVE START: READY"
                    if passive_start_ready
                    else "NEAR-POSE PASSIVE START: not ready"
                ),
                "TARGET - ACTUAL GELLO error (rad)  (+ means increase Panda joint)",
                f"max={float(diagnostic.get('max_abs_target_error_rad', 0.0)):.3f}  "
                f"rms={float(diagnostic.get('rms_target_error_rad', 0.0)):.3f}  "
                f"READY={int(diagnostic.get('ready_streak', 0))}/{limits.ready_stable_cycles}",
                "       count   target   GELLO    error",
                *[
                    f"J{i + 1}  {int(counts[i]):6d}  {float(targets[i]):+7.3f}  "
                    f"{float(actual[i]):+7.3f}  {float(errors[i]):+7.3f}"
                    for i in range(7)
                ],
            ]
        )
    elif preview_error:
        lines.extend(["", f"TARGET ERROR PREVIEW UNAVAILABLE: {preview_error}"])
    styled = []
    for i, line in enumerate(lines):
        size, thickness = (.72 if i < 3 else .58), (2 if i in (0, 4) else 1)
        for part in wrap_pixels(line, 740, font_scale=size, thickness=thickness):
            styled.append((part, size, thickness, color if i == 4 else (235, 235, 235),
                           38 if i < 3 else 25))
    height = max(camera.shape[0], 44 + sum(item[4] for item in styled) + 10)
    camera = np.pad(camera, ((0, height-camera.shape[0]), (0, 0), (0, 0)))
    panel = np.zeros((height, 780, 3), dtype=np.uint8)
    y = 44
    for line, font_scale, thickness, line_color, line_height in styled:
        cv2.putText(
            panel,
            line,
            (18, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            line_color,
            thickness,
            cv2.LINE_AA,
        )
        y += line_height
    return np.hstack((camera, panel))

class AlignmentRunner:
    def __init__(self, *, config: dict, calibration: GelloCalibration, display=None):
        self.config = config
        self.calibration = calibration
        self.display = display if display is not None else cv2
        self.limits = GelloAlignmentAssistLimits.from_dict(config["alignment_limits"])
        self.passive_start_contract = dict(config["near_pose_passive_start"])
        self.live_release = self.limits.profile == "GELLO-ALIGN-A6"
        self.visible_handoff = self.limits.profile in ("GELLO-ALIGN-A5", "GELLO-ALIGN-A6")
        self.live_handoff = None
        if self.visible_handoff and config.get("alignment_handoff") != (
            LIVE_HANDOFF_CONTRACT if self.live_release else HANDOFF_CONTRACT
        ):
            raise ValueError("GELLO-ALIGN-A5 requires its locked handoff contract")

    def _bus(self) -> SafeXL330Transport:
        hardware = self.config["hardware"]
        return SafeXL330Transport(
            str(hardware["port"]),
            baudrate=int(hardware["baudrate"]),
            timeout_seconds=float(hardware["serial_timeout_seconds"]),
        )

    def _read_only_bus(self) -> ReadOnlyDynamixelBus:
        hardware = self.config["hardware"]
        return ReadOnlyDynamixelBus(
            str(hardware["port"]),
            baudrate=int(hardware["baudrate"]),
            timeout_seconds=float(hardware["serial_timeout_seconds"]),
        )

    def _controller(self, bus) -> GelloAlignmentAssistController:
        return GelloAlignmentAssistController(
            bus=bus,
            calibration=self.calibration,
            expected_model_numbers={
                int(key): int(value)
                for key, value in self.config["hardware"]["expected_model_numbers"].items()
            },
            limits=self.limits,
        )

    def __call__(
        self,
        *,
        env,
        observation,
        follower_target_joints: np.ndarray,
        task_instruction: str,
        task_id: int,
        state_id: int,
        attempt_index: int,
        resize_size: int,
    ) -> dict:
        initial_sim_state = np.asarray(env.get_sim_state()).copy()
        if self.live_handoff is not None and not self.live_handoff.closed:
            raise RuntimeError("previous live handoff still owns the motor bus")
        self.live_handoff = None
        initial_sim_state_sha256 = passive.sha256_array(initial_sim_state)
        policy_frame = passive.prepare_observation(observation, int(resize_size))[1]
        frame = passive.operator_camera_frame_from_policy_frame(policy_frame)
        window = (
            f"GELLO-HIL-ASSIST-v1 ALIGNMENT TASK {int(task_id)} STATE {int(state_id)} "
            f"ATTEMPT {int(attempt_index)}"
        )
        if self.visible_handoff:
            window = passive.collection_window_name(
                PROTOCOL_VERSION, task_id=task_id, state_id=state_id,
                attempt_index=attempt_index,
            )
        self.display.namedWindow(window, cv2.WINDOW_NORMAL)
        self.display.resizeWindow(window, frame.shape[1] * 3 + 780, frame.shape[0] * 3)
        status = "PAUSED"
        notice = "PRE-ATTEMPT READY: press ENTER only while supporting the GELLO"
        diagnostic = None
        sessions: list[dict] = []
        pause_started = time.monotonic()
        total_paused_seconds = 0.0
        active_assist_seconds = 0.0
        active_started = None
        pause_count = 0
        started = time.monotonic()
        active_controller = None
        active_bus = None
        aborted = False
        collection_start_eligible = False
        failure = None
        stop_collection = False
        physical_power_cut_required = False
        retryable_faults: list[dict] = []
        direct_passive_start_events: list[dict] = []
        handoff_events: list[dict] = []
        handoff_session = passive.GelloOperatorSession()
        collection_start_mode = None
        read_only_start_check_seconds = 0.0
        paused_bus = None
        paused_preview_error = None
        paused_preview_samples: list[dict] = []
        next_paused_preview_at = 0.0

        def show_handoff_frame(phase, remaining, current_diagnostic):
            countdown = f" / {remaining:.1f}s" if phase == "POWERED HOLD" else ""
            display = passive.draw_attempt_panel(
                policy_frame, instruction=task_instruction, task_id=task_id,
                state_id=state_id, attempt_index=attempt_index,
                session=handoff_session,
                action_budget=int(self.config["max_actions_per_attempt"]),
                paused_target=None, preview_frame=None, preview=None,
                active_diagnostic=None, success_diagnostic={},
                success_review_pending=False, retry_available=False,
                notice="PAUSED: support GELLO; control starts only after a new ENTER",
                window_scale=3.0, panel_width=780,
                handoff_status=f"{phase}{countdown}",
            )
            if hasattr(self.display, "set_state"):
                self.display.set_state(mode="alignment", phase=phase,
                                       notice="Handing off motor assistance", passive_start_ready=False)
            self.display.imshow(window, display)

        def poll_handoff_key():
            key = int(self.display.waitKey(10)) & 0xFF
            if self.display.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                raise HandoffCancelled(abort=True)
            return key

        def close_paused_bus() -> None:
            nonlocal paused_bus
            if paused_bus is not None:
                paused_bus.close()
                paused_bus = None

        def refresh_paused_target_error() -> None:
            nonlocal paused_bus, paused_preview_error, diagnostic
            nonlocal next_paused_preview_at
            now = time.monotonic()
            if now < next_paused_preview_at:
                return
            next_paused_preview_at = now + 1.0 / float(self.limits.control_hz)
            try:
                if paused_bus is None:
                    paused_bus = self._read_only_bus()
                counts = paused_bus.read_positions(self.calibration.arm_ids)
                diagnostic = target_error_diagnostic(
                    self.calibration,
                    follower_target_joints,
                    counts,
                    tolerance_rad=float(self.limits.ready_max_abs_error_rad),
                )
                sample = {
                    **diagnostic,
                    "event_type": "paused_target_error_preview",
                    "sample_index": len(paused_preview_samples),
                    "monotonic_elapsed_seconds": max(
                        0.0, time.monotonic() - started
                    ),
                    "motor_register_writes": 0,
                    "simulator_steps": 0,
                    "active_action_budget_used": 0,
                }
                paused_preview_samples.append(sample)
                paused_preview_error = None
            except Exception as exc:
                close_paused_bus()
                diagnostic = None
                paused_preview_error = f"{type(exc).__name__}: {exc}"
                # A disconnected adapter should not spin/open at 5 Hz.
                next_paused_preview_at = time.monotonic() + 1.0

        def close_active_session(reason: str, session_failure: str | None = None) -> bool:
            nonlocal active_controller, active_bus, active_started
            nonlocal active_assist_seconds, physical_power_cut_required, failure
            clean = True
            if active_started is not None:
                active_assist_seconds += max(0.0, time.monotonic() - active_started)
                active_started = None
            if active_controller is not None:
                if session_failure is not None:
                    active_controller.failure = session_failure
                try:
                    active_controller.finish(reason=reason)
                except GelloActuationShutdownUnverified as cleanup_exc:
                    physical_power_cut_required = True
                    failure = f"{type(cleanup_exc).__name__}: {cleanup_exc}"
                    clean = False
                except BaseException as cleanup_exc:
                    failure = f"{type(cleanup_exc).__name__}: {cleanup_exc}"
                    clean = False
                sessions.append(active_controller.summary())
            if active_bus is not None:
                active_bus.close()
            active_controller = None
            active_bus = None
            return clean

        def finish_near_pose_passive_start(
            current_diagnostic: dict,
            *,
            source_status: str,
            check_started: float,
        ) -> bool:
            """Close a never-written setup bus and hand off to passive LIBERO."""

            nonlocal active_controller, active_bus, collection_start_eligible
            nonlocal collection_start_mode, notice, read_only_start_check_seconds
            if not near_pose_passive_start_ready(
                current_diagnostic, self.passive_start_contract
            ):
                return False
            if active_controller is None or active_bus is None:
                raise GelloActuationSafetyError(
                    "near-pose passive start requires a current read-only preflight"
                )
            verification = active_controller.verify_near_pose_passive_start()
            summary = active_controller.summary()
            if int(summary.get("motor_register_writes", -1)) != 0 or int(
                summary.get("write_packets", -1)
            ) != 0:
                raise GelloActuationSafetyError(
                    "near-pose passive start setup unexpectedly wrote a motor register"
                )
            event = {
                "event_type": "near_pose_direct_passive_start",
                "source_status": source_status,
                "monotonic_elapsed_seconds": max(
                    0.0, time.monotonic() - started
                ),
                "read_only_start_check_seconds": max(
                    0.0, time.monotonic() - check_started
                ),
                "thresholds": dict(self.passive_start_contract),
                "diagnostic": dict(current_diagnostic),
                "torque_off_readback_all_ids": bool(
                    verification["torque_off_verified_all_ids"]
                ),
                "motor_register_writes": 0,
                "simulator_steps": 0,
                "active_action_budget_used": 0,
            }
            read_only_start_check_seconds += event["read_only_start_check_seconds"]
            direct_passive_start_events.append(event)
            sessions.append(summary)
            active_bus.close()
            active_controller = None
            active_bus = None
            collection_start_eligible = True
            collection_start_mode = "near_pose_direct_passive"
            notice = "TORQUE-OFF VERIFIED: opening passive LIBERO without assist"
            return True

        def pause_after_retryable_fault(exc: BaseException) -> bool:
            nonlocal pause_count, pause_started, status, notice, diagnostic
            message = f"{type(exc).__name__}: {exc}"
            retryable_faults.append(
                {
                    "event_type": "alignment_setup_or_assist_safety_pause",
                    "monotonic_elapsed_seconds": max(0.0, time.monotonic() - started),
                    "status_before_pause": status,
                    "error": message,
                    "hil_attempt_consumed": False,
                }
            )
            if not close_active_session("retryable_alignment_safety_pause", message):
                return False
            pause_count += 1
            pause_started = time.monotonic()
            status = "PAUSED"
            diagnostic = None
            notice = f"SAFETY PAUSE (attempt not consumed): {exc}; ENTER retries"
            return True

        try:
            while True:
                if status == "PAUSED":
                    refresh_paused_target_error()
                display = draw_alignment_panel(
                    frame,
                    instruction=task_instruction,
                    task_id=task_id,
                    state_id=state_id,
                    attempt_index=attempt_index,
                    status=status,
                    diagnostic=diagnostic,
                    preview_error=paused_preview_error,
                    notice=notice,
                    limits=self.limits,
                    passive_start_ready=near_pose_passive_start_ready(
                        diagnostic, self.passive_start_contract
                    ),
                )
                if hasattr(self.display, "set_state"):
                    self.display.set_state(mode="alignment", phase=status, notice=notice,
                        instruction=task_instruction, diagnostic=diagnostic,
                        preview_error=paused_preview_error,
                        passive_start_ready=near_pose_passive_start_ready(diagnostic, self.passive_start_contract))
                self.display.imshow(window, display)
                key = int(self.display.waitKey(10)) & 0xFF
                try:
                    closed = self.display.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1
                except cv2.error:
                    closed = True
                if closed or key in (ord("q"), ord("Q"), 27):
                    aborted = True
                    stop_collection = True
                    notice = "ABORTED"
                    break
                if status == "PAUSED":
                    if key not in (10, 13):
                        continue
                    total_paused_seconds += max(0.0, time.monotonic() - pause_started)
                    close_paused_bus()
                    diagnostic = None
                    paused_preview_error = None
                    try:
                        read_only_check_started = time.monotonic()
                        active_bus = self._bus()
                        active_controller = self._controller(active_bus)
                        preflight = active_controller.read_only_preflight()
                        preflight_counts = np.asarray(
                            [
                                preflight["snapshot"][motor_id][
                                    "present_position_counts"
                                ]
                                for motor_id in self.calibration.arm_ids
                            ],
                            dtype=np.int64,
                        )
                        diagnostic = target_error_diagnostic(
                            self.calibration,
                            follower_target_joints,
                            preflight_counts,
                            tolerance_rad=float(
                                self.limits.ready_max_abs_error_rad
                            ),
                        )
                        button = getattr(self.display, "last_button", None)
                        near = near_pose_passive_start_ready(diagnostic, self.passive_start_contract)
                        if button == "assist" and near or button == "start" and not near:
                            # Assist is not the Start button, and Start never arms
                            # assistance: a pose that drifted away stays paused.
                            active_controller.verify_near_pose_passive_start()
                            summary = active_controller.summary()
                            if int(summary.get("motor_register_writes", -1)) != 0:
                                raise GelloActuationSafetyError("unexpected write during passive alignment check")
                            sessions.append(summary)
                            active_bus.close()
                            active_controller = None
                            active_bus = None
                            pause_started = time.monotonic()
                            status = "PAUSED"
                            notice = (
                                "Already aligned, torque OFF: press Start to collect"
                                if near else
                                "Not aligned at Start (max "
                                f"{float(diagnostic['max_abs_target_error_rad']):.2f} rad): "
                                "align GELLO or press Motor assist"
                            )
                            continue
                        if finish_near_pose_passive_start(
                            diagnostic,
                            source_status="PAUSED",
                            check_started=read_only_check_started,
                        ):
                            break
                    except GelloActuationShutdownUnverified as exc:
                        failure = f"{type(exc).__name__}: {exc}"
                        physical_power_cut_required = True
                        stop_collection = True
                        break
                    except GelloActuationSafetyError as exc:
                        if not pause_after_retryable_fault(exc):
                            stop_collection = True
                            break
                        continue
                    status = "ARMING SETTLE"
                    notice = "TORQUE OFF: hold GELLO still; assist arms after 3 stable reads"
                    continue
                if key == 32:
                    if not close_active_session("operator_clutch"):
                        stop_collection = True
                        break
                    pause_count += 1
                    pause_started = time.monotonic()
                    status = "PAUSED"
                    notice = "CLUTCHED: torque off; reposition/support, then ENTER"
                    continue
                if status == "ARMING SETTLE":
                    tick_started = time.monotonic()
                    try:
                        settle = active_controller.passive_settle_once()
                        diagnostic = {
                            **settle,
                            **target_error_diagnostic(
                                self.calibration,
                                follower_target_joints,
                                np.asarray(
                                    settle["present_position_counts"], dtype=np.int64
                                ),
                                tolerance_rad=float(
                                    self.limits.ready_max_abs_error_rad
                                ),
                            ),
                        }
                        if key in (10, 13) and finish_near_pose_passive_start(
                            diagnostic,
                            source_status="ARMING SETTLE",
                            check_started=tick_started,
                        ):
                            break
                        if diagnostic["settled_for_arm"]:
                            reference = diagnostic["present_position_counts"]
                            active_controller.verify_safe_off()
                            plan = plan_follower_joint_alignment(
                                self.calibration,
                                follower_target_joints,
                                reference,
                                selected_motor_ids=self.calibration.arm_ids,
                                limits=active_controller.limits,
                            )
                            active_controller.prepare_alignment(plan)
                            active_controller.arm_alignment()
                            active_started = time.monotonic()
                            status = "ASSIST ACTIVE"
                            diagnostic = None
                            notice = "Assist active: guide GELLO toward zero error"
                        else:
                            stable = int(diagnostic.get("stable_streak", 0))
                            required = int(self.limits.pre_arm_stable_cycles)
                            notice = (
                                f"TORQUE OFF: settled {stable}/{required}; hold still for "
                                f"{required - stable} more stable read(s)"
                            )
                    except GelloActuationShutdownUnverified as exc:
                        failure = f"{type(exc).__name__}: {exc}"
                        physical_power_cut_required = True
                        stop_collection = True
                        break
                    except GelloActuationSafetyError as exc:
                        if not pause_after_retryable_fault(exc):
                            stop_collection = True
                            break
                        continue
                    elapsed = time.monotonic() - tick_started
                    time.sleep(max(0.0, 1.0 / self.limits.control_hz - elapsed))
                    continue
                if status == "ALIGNMENT READY":
                    if key in (10, 13):
                        if self.live_release:
                            self.live_handoff = LiveTorqueHandoff(
                                active_controller, active_bus, clock=time.monotonic,
                            )
                            if active_started is not None:
                                active_assist_seconds += max(0.0, time.monotonic() - active_started)
                                active_started = None
                            # Transfer ownership, never disarm in this runner's finally.
                            active_controller = None
                            active_bus = None
                            collection_start_eligible = True
                            collection_start_mode = "A6_enter_starts_live_release"
                            break
                        try:
                            if self.visible_handoff:
                                verification = visible_hold_then_release(
                                    active_controller, show_frame=show_handoff_frame,
                                    poll_key=poll_handoff_key, events=handoff_events,
                                    clock=time.monotonic, sleep=time.sleep,
                                )
                                if verification is None:
                                    status = "ASSIST ACTIVE"
                                    notice = "HOLD CANCELLED: alignment lost; guide GELLO back"
                                    continue
                            else:
                                verification = active_controller.release_and_verify_for_collection()
                        except HandoffCancelled as exc:
                            clean = close_active_session("handoff_operator_cancel")
                            if exc.abort or not clean:
                                aborted = True
                                stop_collection = True
                                break
                            pause_count += 1
                            pause_started = time.monotonic()
                            status = "PAUSED"
                            notice = "HANDOFF CLUTCHED: torque off; ENTER retries"
                            continue
                        except GelloActuationShutdownUnverified as exc:
                            failure = f"{type(exc).__name__}: {exc}"
                            physical_power_cut_required = True
                            stop_collection = True
                            break
                        except GelloActuationSafetyError as exc:
                            if not pause_after_retryable_fault(exc):
                                stop_collection = True
                                break
                            continue
                        if active_started is not None:
                            active_assist_seconds += max(0.0, time.monotonic() - active_started)
                            active_started = None
                        sessions.append(active_controller.summary())
                        active_bus.close()
                        active_controller = None
                        active_bus = None
                        if verification["passed"]:
                            collection_start_eligible = True
                            collection_start_mode = "assist_release_verified"
                            notice = "VERIFIED TORQUE-OFF: opening read-only LIBERO control"
                            break
                        pause_count += 1
                        pause_started = time.monotonic()
                        status = "PAUSED"
                        notice = "POST-RELEASE DRIFT: realign; collection remains frozen"
                        continue
                    tick_started = time.monotonic()
                    try:
                        diagnostic = active_controller.maintain_ready_once()
                    except GelloActuationShutdownUnverified as exc:
                        failure = f"{type(exc).__name__}: {exc}"
                        physical_power_cut_required = True
                        stop_collection = True
                        break
                    except GelloActuationSafetyError as exc:
                        if not pause_after_retryable_fault(exc):
                            stop_collection = True
                            break
                        continue
                    if not diagnostic["alignment_ready"]:
                        status = "ASSIST ACTIVE"
                        notice = "READY LOST: guide GELLO back toward zero error"
                    elapsed = time.monotonic() - tick_started
                    time.sleep(max(0.0, 1.0 / self.limits.control_hz - elapsed))
                    continue
                tick_started = time.monotonic()
                try:
                    diagnostic = active_controller.guide_once()
                except GelloActuationShutdownUnverified as exc:
                    failure = f"{type(exc).__name__}: {exc}"
                    physical_power_cut_required = True
                    stop_collection = True
                    break
                except GelloActuationSafetyError as exc:
                    if not pause_after_retryable_fault(exc):
                        stop_collection = True
                        break
                    continue
                if diagnostic["alignment_ready"]:
                    status = "ALIGNMENT READY"
                    notice = (
                        "ENTER: START control now, soften over 2s (no second ENTER)"
                        if self.live_release else
                        "ENTER: teleop screen + 2s hold, then release and verify"
                        if self.visible_handoff
                        else "Hold gently, then ENTER to soften/release and verify"
                    )
                elapsed = time.monotonic() - tick_started
                time.sleep(max(0.0, 1.0 / self.limits.control_hz - elapsed))
        except BaseException as exc:
            failure = f"{type(exc).__name__}: {exc}"
            aborted = bool(aborted or isinstance(exc, KeyboardInterrupt))
            stop_collection = True
            if active_controller is not None:
                active_controller.failure = failure
        finally:
            close_paused_bus()
            close_active_session("alignment_runner_cleanup", failure)
            final_sim_state = np.asarray(env.get_sim_state()).copy()
            simulator_state_exact = bool(
                np.array_equal(initial_sim_state, final_sim_state)
            )
            if not simulator_state_exact:
                failure = failure or (
                    f"RuntimeError: {self.limits.profile} changed simulator state"
                )
                collection_start_eligible = False
            if self.live_handoff is not None and (failure or not collection_start_eligible):
                try:
                    self.live_handoff.close()
                except BaseException as cleanup_exc:
                    failure = f"{type(cleanup_exc).__name__}: {cleanup_exc}"
                    physical_power_cut_required = True
            if not (self.visible_handoff and collection_start_eligible and not failure):
                try:
                    if self.display is cv2:
                        passive.close_operator_window(window)
                    else:
                        self.display.destroyWindow(window)
                except cv2.error:
                    pass
            if status == "PAUSED" and pause_started is not None:
                total_paused_seconds += max(0.0, time.monotonic() - pause_started)
        return _json_safe(
            {
                "schema_version": "hil-gello-alignment-assist-" + self.limits.profile.rsplit("-", 1)[-1].lower(),
                "protocol_version": self.limits.profile,
                "alignment_handoff_contract": self.config.get("alignment_handoff"),
                "teleop_handoff_events": handoff_events,
                "same_window_handed_to_collection": bool(
                    self.visible_handoff and collection_start_eligible and not failure
                ),
                "hil_protocol_version": self.config["protocol_version"],
                "session_calibration": self.config.get("resolved_session_calibration"),
                "task_id": task_id,
                "state_id": state_id,
                "attempt_index": attempt_index,
                "task_instruction": task_instruction,
                "follower_target_joint_positions": follower_target_joints,
                "collection_start_eligible": collection_start_eligible,
                "collection_start_mode": collection_start_mode,
                "collection_start_requested_by_alignment_enter": bool(
                    self.live_release and collection_start_eligible
                ),
                "aborted": aborted,
                "stop_collection": stop_collection,
                "failure": failure,
                "retryable_faults": retryable_faults,
                "direct_passive_start_events": direct_passive_start_events,
                "read_only_start_check_seconds": read_only_start_check_seconds,
                "paused_target_error_samples": paused_preview_samples,
                "physical_power_cut_required": physical_power_cut_required,
                "pause_count": pause_count,
                "total_paused_seconds": total_paused_seconds,
                "active_assist_seconds": active_assist_seconds,
                "monotonic_elapsed_seconds": time.monotonic() - started,
                "simulator_steps": 0,
                "active_action_budget_used": 0,
                "initial_sim_state_sha256": initial_sim_state_sha256,
                "final_sim_state_sha256": passive.sha256_array(final_sim_state),
                "simulator_state_byte_exact": simulator_state_exact,
                "motor_write_transport_closed_before_collection": bool(
                    active_bus is None and self.live_handoff is None
                ),
                "sessions": sessions,
                "final_hidden_exposure": [],
            }
        )
