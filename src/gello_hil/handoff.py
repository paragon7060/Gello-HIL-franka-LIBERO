"""Visible, bounded pre-collection hold; no simulator or motor-write interface."""

from __future__ import annotations

import time

from .actuation import GelloActuationSafetyError


HANDOFF_CONTRACT = {
    "protocol_version": "GELLO-ALIGN-A5",
    "visible_hold_seconds": 2.0,
    "same_attempt_window": True,
    "simulator_steps": 0,
    "active_action_budget_used": 0,
    "collection_requires_torque_off_and_fresh_enter": True,
    "near_pose_passive_start_enables_torque": False,
}


class HandoffCancelled(Exception):
    def __init__(self, *, abort: bool):
        self.abort = abort
        super().__init__("operator abort" if abort else "operator clutch")


def visible_hold_then_release(
    controller, *, show_frame, poll_key, events: list,
    clock=time.monotonic, sleep=time.sleep,
) -> dict | None:
    """Caller owns fail-closed cleanup of its bus, including callback failures.

    None means READY was lost: continue realignment, never start collection.
    Only the existing monitor is called in HOLD; softening owns later writes.
    Events are appended to caller-owned storage even when an exception occurs.
    """
    if controller.assist_limits.profile != "GELLO-ALIGN-A5":
        raise ValueError("visible powered handoff requires GELLO-ALIGN-A5")
    hold_seconds = HANDOFF_CONTRACT["visible_hold_seconds"]
    entered_at = clock()
    visible_at = None
    phase = "POWERED HOLD"
    diagnostic = None

    def record(event_type, **details):
        now = clock()
        events.append({
            "event_type": event_type,
            "monotonic_elapsed_seconds": max(0.0, now - entered_at),
            "visible_elapsed_seconds": (
                None if visible_at is None else max(0.0, now - visible_at)
            ),
            "active_step_index": 0,
            "simulator_steps": 0,
            "active_action_budget_used": 0,
            **details,
        })

    def display_and_poll():
        remaining = hold_seconds if visible_at is None else max(
            0.0, hold_seconds - (clock() - visible_at)
        )
        show_frame(phase, remaining, diagnostic)
        key = poll_key()
        if key in (ord("q"), ord("Q"), 32):
            raise HandoffCancelled(abort=key != 32)
        # Enter is deliberately consumed, never forwarded to passive control.
        return False

    record("teleop_handoff_requested", hold_seconds=hold_seconds)
    try:
        diagnostic = controller.maintain_ready_once()
        if not diagnostic["alignment_ready"]:
            record("teleop_hold_ready_lost", diagnostic=diagnostic)
            return None
        display_and_poll()
        visible_at = clock()  # first teleop frame has been shown AND pumped
        record("teleop_hold_visible", diagnostic=diagnostic)
        interval = 1.0 / controller.assist_limits.control_hz
        while True:
            tick_started = clock()
            if clock() - controller.started_at > controller.assist_limits.maximum_alignment_seconds:
                raise GelloActuationSafetyError("GELLO-ALIGN-A5 handoff alignment timeout")
            diagnostic = controller.maintain_ready_once()
            record("teleop_hold_monitor", diagnostic=diagnostic)
            if not diagnostic["alignment_ready"]:
                record("teleop_hold_ready_lost", diagnostic=diagnostic)
                return None
            display_and_poll()
            if clock() - visible_at >= hold_seconds:
                break
            sleep(max(0.0, min(
                interval - (clock() - tick_started),
                hold_seconds - (clock() - visible_at),
            )))
        record("teleop_hold_complete")
        phase = "SOFTENING / TORQUE-OFF CHECK"
        display_and_poll()
        verification = controller.release_and_verify_for_collection(
            stop_requested=display_and_poll,
        )
        record("teleop_release_verified", verification=verification)
        return verification
    except BaseException as exc:
        record("teleop_handoff_cancelled", reason=f"{type(exc).__name__}: {exc}")
        raise

