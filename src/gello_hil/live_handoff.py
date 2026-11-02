"""Single-owner live torque release; caller must close on EVERY exit path."""

from __future__ import annotations

import math
import time

from .actuation import (
    GelloActuationSafetyError, GelloTrackingError, HomingState,
)
from .dynamixel import COUNTS_PER_REVOLUTION


LIVE_HANDOFF_CONTRACT = {
    "protocol_version": "GELLO-ALIGN-A6",
    "release_seconds": 2.0,
    "start_key": "ENTER",
    "second_enter_required": False,
    "mapping": "enter_anchored_joint_delta",
    "transition_actions_consume_budget": True,
    "single_serial_owner": True,
    "near_pose_passive_start_enables_torque": False,
}


class LiveTorqueHandoff:
    """Serve positions through one bus while reducing force, then become read-only.

    No background thread touches the bus. The caller services tick once per GUI
    loop and closes before closing its simulator or window. Hardware watchdog and
    monitor caps are not extended for slow GUI/simulator iterations.
    """

    def __init__(self, controller, bus, *, clock=time.monotonic):
        if controller.assist_limits.profile != "GELLO-ALIGN-A6":
            raise ValueError("live release requires the A6 profile")
        if controller.state != HomingState.HOLDING_AT_TARGET:
            raise GelloActuationSafetyError("live handoff requires READY hold")
        self.controller = controller
        self.bus = bus
        self.clock = clock
        self.created_at = clock()
        self.started_at = None
        self.initial_current_ma = int(controller.current_goal_ma)
        self.next_tick = 0.0
        self.read_only_factory = None
        self.reader = None
        self.events = []
        self.active_step_index = 0
        self.released = False
        self.closed = False
        self.failure = None
        self.record("powered_handoff_owned")

    def record(self, event_type, **details):
        self.events.append({
            "event_type": event_type,
            "active_step_index": int(self.active_step_index),
            "monotonic_elapsed_seconds": max(0.0, self.clock() - self.created_at),
            "release_elapsed_seconds": None if self.started_at is None else max(
                0.0, self.clock() - self.started_at
            ),
            **details,
        })

    @property
    def powered(self):
        return not self.released and not self.closed

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def begin(self):
        if self.started_at is not None or not self.powered:
            raise GelloActuationSafetyError("live handoff start is not repeatable")
        self.started_at = self.clock()
        self.controller._set_state(HomingState.SOFTENING, "live_control_softening_started")
        self.record("collection_started_while_powered", current_ma=self.initial_current_ma)

    def tick(self, active_step_index=None):
        if active_step_index is not None:
            self.active_step_index = int(active_step_index)
        if not self.powered or self.clock() < self.next_tick:
            return
        self.next_tick = self.clock() + 1.0 / self.controller.assist_limits.control_hz
        if self.started_at is None and self.clock() - self.created_at > 2.0:
            raise GelloActuationSafetyError("live handoff initialization exceeded two seconds")
        try:
            snapshot = self.controller.monitor_once()
        except GelloTrackingError:
            if self.started_at is None:
                raise
            self.release_now("operator_backdrive_early_release")
            return
        if self.started_at is None:
            return
        elapsed = max(0.0, self.clock() - self.started_at)
        desired = max(0, int(math.ceil(self.initial_current_ma * (1.0 - elapsed / 2.0))))
        desired = min(int(self.controller.current_goal_ma), desired)
        if desired != int(self.controller.current_goal_ma):
            for motor_id in self.controller.selected_motor_ids:
                self.bus.set_goal_current_ma(motor_id, desired)
            self.controller.current_goal_ma = desired
        self.record(
            "live_release_tick", current_ma=desired,
            present_position_counts={str(i): int(v["present_position_counts"]) for i, v in snapshot.items()},
        )
        if elapsed >= 2.0:
            self.release_now("two_second_live_release_complete")

    def read_positions(self, motor_ids):
        if self.closed:
            raise RuntimeError("live handoff bus is closed")
        self.tick()
        if self.powered:
            # Match the read-only successor's one-turn mapping input contract.
            # Raw signed positions remain in tick telemetry for safety monitoring.
            positions = self.bus.sync_read_control_item("present_position_counts", motor_ids)
            return {motor_id: int(value) % COUNTS_PER_REVOLUTION
                    for motor_id, value in positions.items()}
        if self.reader is None:
            if self.read_only_factory is None:
                raise RuntimeError("read-only successor was not configured")
            self.reader = self.read_only_factory()
            self.record("read_only_successor_opened")
        return self.reader.read_positions(motor_ids)

    def release_now(self, reason):
        if not self.powered:
            return
        try:
            result = self.controller.finish(reason=reason)
            self.released = True
            self.record("torque_off_verified", reason=reason, verification=result)
        except BaseException as exc:
            self.failure = f"{type(exc).__name__}: {exc}"
            self.record("release_failed", reason=reason, error=self.failure)
            raise
        finally:
            self.bus.close()

    def close(self):
        if self.closed:
            return
        try:
            self.release_now("collection_exit_cleanup")
        finally:
            self.closed = True
            if self.reader is not None:
                self.reader.close()

    def summary(self):
        return {
            "protocol_version": "GELLO-ALIGN-A6",
            "contract": dict(LIVE_HANDOFF_CONTRACT),
            "events": list(self.events),
            "release_started": self.started_at is not None,
            "torque_off_verified": self.released,
            "failure": self.failure,
            "motor_session": self.controller.summary(),
        }
