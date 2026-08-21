import numpy as np
import pytest

from gello_hil.session import OperatorSession, decode_key


class Clock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value


def anchor(session, clock):
    return session.resume(
        anchor_counts=np.arange(8),
        follower_joints=np.zeros(7),
        leader_joints=np.zeros(7),
        mode="relative-clutch",
    )


def test_starts_paused_and_paused_time_never_consumes_active_budget():
    clock = Clock()
    session = OperatorSession(clock=clock)
    clock.value = 10.0
    assert session.status == "PAUSED"
    assert session.active_step_count == 0
    assert session.summary()["total_paused_seconds"] == 10.0
    with pytest.raises(RuntimeError):
        session.record_active_step()


def test_pause_and_reanchor_provenance_uses_active_step_index():
    clock = Clock()
    session = OperatorSession(clock=clock)
    clock.value = 2.0
    first = anchor(session, clock)
    session.record_active_step()
    clock.value = 3.5
    pause = session.pause()
    clock.value = 8.5
    second = anchor(session, clock)
    assert first["event_type"] == "initial_anchor_resume"
    assert pause["active_step_index"] == 1
    assert second["event_type"] == "reanchor_resume"
    assert second["pause_duration_seconds"] == 5.0
    assert session.pause_count == 1


def test_keys_are_deliberately_small():
    assert decode_key(13) == "resume"
    assert decode_key(ord(" ")) == "pause"
    assert decode_key(ord("q")) == "abort"
    assert decode_key(ord("x")) is None
