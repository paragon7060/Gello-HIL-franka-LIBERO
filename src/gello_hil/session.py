"""Operator-controlled PAUSED/ACTIVE state with provenance."""

from __future__ import annotations

import time

import numpy as np


class OperatorSession:
    """A pause/clutch state machine that never steps a simulator itself."""

    def __init__(self, *, clock=None):
        self._clock = time.monotonic if clock is None else clock
        self._started_at = float(self._clock())
        self._pause_started_at = self._started_at
        self._active_started_at = None
        self._paused_total = 0.0
        self._active_total = 0.0
        self.active_step_count = 0
        self.pause_count = 0
        self.active = False
        self.terminal = False
        self.aborted = False
        self.events: list[dict] = []
        self._event("initial_pause")

    @property
    def status(self) -> str:
        if self.aborted:
            return "ABORTED"
        if self.terminal:
            return "FINISHED"
        return "ACTIVE" if self.active else "PAUSED"

    def _timing(self, now: float) -> tuple[float, float]:
        paused = self._paused_total
        active = self._active_total
        if self._pause_started_at is not None:
            paused += max(0.0, now - self._pause_started_at)
        if self._active_started_at is not None:
            active += max(0.0, now - self._active_started_at)
        return paused, active

    def _event(self, event_type: str, *, now=None, **payload) -> dict:
        now = float(self._clock()) if now is None else float(now)
        paused, active = self._timing(now)
        event = {
            "event_type": str(event_type),
            "active_step_index": int(self.active_step_count),
            "monotonic_elapsed_seconds": max(0.0, now - self._started_at),
            "pause_count": int(self.pause_count),
            "total_paused_seconds": float(paused),
            "active_control_seconds": float(active),
            **payload,
        }
        self.events.append(event)
        return event

    @staticmethod
    def _anchor(anchor_counts, follower_joints, leader_joints) -> dict:
        counts = np.asarray(anchor_counts, dtype=np.int64)
        follower = np.asarray(follower_joints, dtype=np.float64)
        leader = np.asarray(leader_joints, dtype=np.float64)
        if counts.shape != (8,) or follower.shape != (7,) or leader.shape != (7,):
            raise ValueError("anchor requires 8 counts and two seven-joint vectors")
        if not np.isfinite(follower).all() or not np.isfinite(leader).all():
            raise ValueError("anchor joint vectors must be finite")
        return {
            "anchor_counts": counts.tolist(),
            "follower_anchor_joint_positions": follower.tolist(),
            "mapped_leader_joint_positions": leader.tolist(),
        }

    def resume(self, *, anchor_counts, follower_joints, leader_joints, mode: str) -> dict:
        if self.terminal or self.active:
            raise RuntimeError("session cannot resume from its current state")
        anchor = self._anchor(anchor_counts, follower_joints, leader_joints)
        now = float(self._clock())
        pause_duration = max(0.0, now - self._pause_started_at)
        self._paused_total += pause_duration
        self._pause_started_at = None
        self._active_started_at = now
        self.active = True
        event_type = (
            "initial_anchor_resume"
            if not any(e["event_type"].endswith("anchor_resume") for e in self.events)
            else "reanchor_resume"
        )
        return self._event(
            event_type,
            now=now,
            pause_duration_seconds=float(pause_duration),
            mapping_mode=str(mode),
            **anchor,
        )

    def pause(self, **payload) -> dict:
        if self.terminal or not self.active:
            raise RuntimeError("session cannot pause from its current state")
        now = float(self._clock())
        self._active_total += max(0.0, now - self._active_started_at)
        self._active_started_at = None
        self._pause_started_at = now
        self.active = False
        self.pause_count += 1
        return self._event("clutch_pause", now=now, **payload)

    def record_active_step(self) -> int:
        if self.terminal or not self.active:
            raise RuntimeError("paused or finished sessions cannot consume active budget")
        self.active_step_count += 1
        return self.active_step_count

    def finish(self, event_type="finished") -> dict:
        return self._terminate(event_type, aborted=False)

    def abort(self) -> dict:
        return self._terminate("abort", aborted=True)

    def _terminate(self, event_type: str, *, aborted: bool) -> dict:
        if self.terminal:
            return self.events[-1]
        now = float(self._clock())
        payload = {}
        if self.active:
            self._active_total += max(0.0, now - self._active_started_at)
            self._active_started_at = None
        else:
            duration = max(0.0, now - self._pause_started_at)
            self._paused_total += duration
            self._pause_started_at = None
            payload["pause_duration_seconds"] = duration
        self.active = False
        self.terminal = True
        self.aborted = bool(aborted)
        return self._event(event_type, now=now, **payload)

    def summary(self) -> dict:
        now = float(self._clock())
        paused, active = self._timing(now)
        return {
            "status": self.status,
            "active_step_count": self.active_step_count,
            "pause_count": self.pause_count,
            "monotonic_elapsed_seconds": max(0.0, now - self._started_at),
            "total_paused_seconds": paused,
            "active_control_seconds": active,
            "events": [dict(event) for event in self.events],
        }


def decode_key(key: int) -> str | None:
    key &= 0xFF
    if key in (10, 13):
        return "resume"
    if key == ord(" "):
        return "pause"
    if key in (ord("q"), ord("Q"), 27):
        return "abort"
    return None
