import json
from types import SimpleNamespace

import numpy as np
import pytest

from gello_hil import cli, gui_bridge, position
from gello_hil.libero_runtime import rgb_frame
from gello_hil.review import atomic_json
from test_alignment_safety import POSE, calibration, counts_for


def test_camera_view_removes_only_policy_horizontal_mirror(monkeypatch):
    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtGui as G
    from gello_hil.collector_qt import ImageView, application

    app = application()
    raw = np.arange(18, dtype=np.uint8).reshape(2, 3, 3) * 10
    untouched = raw.copy()
    policy = rgb_frame({"agentview_image": raw})
    view = ImageView()
    view.show_image(
        G.QImage(policy.data, 3, 2, policy.strides[0], G.QImage.Format.Format_RGB888).copy()
    )

    def pixels():
        image = view.oriented_pixmap().toImage()
        return np.asarray([[image.pixelColor(x, y).red() for x in range(3)] for y in range(2)])

    assert app is not None
    assert np.array_equal(pixels(), raw[::-1, :, 0])
    view.set_camera_view(False)
    assert np.array_equal(pixels(), policy[:, :, 0])
    assert np.array_equal(raw, untouched)
    view.close()


def test_display_orientation_command_is_logged_without_motor_permission(tmp_path):
    display = gui_bridge.FileDisplay(tmp_path, motor_permitted=False)
    atomic_json(tmp_path / "commands/000001_view.json", {"command": "view", "value": "policy"})
    assert display.waitKey(0) == -1
    assert display.summary()["initial_operator_view"] == "camera"
    assert display.events[-1]["accepted"]
    assert display.events[-1]["command"] == "view"


def test_mapping_preview_reads_only_and_never_steps_physics(monkeypatch, tmp_path):
    import gello_hil.libero_runtime as runtime

    file = tmp_path / "calibration.json"
    calibration().to_json(file)
    closed = []
    states = []
    keys = iter([-1, ord("q")])
    original_publish = gui_bridge.FileDisplay.publish
    monkeypatch.setattr(gui_bridge.FileDisplay, "waitKey", lambda *a: next(keys))

    def publish(display, frame=None, **kwargs):
        states.append(dict(display.state))
        return original_publish(display, frame, force=True)

    monkeypatch.setattr(gui_bridge.FileDisplay, "publish", publish)
    env = SimpleNamespace(close=lambda: closed.append("env"))  # deliberately no step method
    monkeypatch.setattr(
        position,
        "make_position_env",
        lambda a: (env, {"agentview_image": np.zeros((16, 16, 3), np.uint8)}, "native instruction"),
    )
    calls = []

    def render(environment, joints):
        assert environment is env and np.max(np.abs(joints - POSE)) < 0.002
        calls.append(True)
        return np.zeros((16, 16, 3), np.uint8)

    monkeypatch.setattr(runtime, "render_joint_pose", render)
    monkeypatch.setattr(
        cli,
        "_bus",
        lambda a: SimpleNamespace(
            read_positions=lambda ids: {**dict(zip(range(1, 8), counts_for(POSE))), 8: 4000},
            close=lambda: closed.append("bus"),
        ),
    )
    args = SimpleNamespace(
        gui_control_dir=str(tmp_path / "ipc"), gui_mapping_check=True, calibration=str(file)
    )
    assert gui_bridge.preview_scene(args) == 0
    assert calls and closed == ["bus", "env"]
    assert any(s.get("phase") == "MAPPING CHECK" for s in states)
    value = json.loads((tmp_path / "ipc/state.json").read_text())
    assert value["motor_permitted"] is False and value["phase"] == "CLOSED"


def test_handoff_normalizes_signed_multi_turn_counts_without_extra_writes(monkeypatch):
    from test_live_handoff_a6 import ready

    bus, controller, handoff, _, _ = ready()
    raw = {1: -1, 2: 8193, 3: 4096, 4: 2048, 5: 0, 6: 4095, 7: -4097, 8: 4000}
    writes = bus.motor_register_writes
    monkeypatch.setattr(handoff, "tick", lambda *a: None)
    original_read = bus.sync_read_control_item
    monkeypatch.setattr(
        bus,
        "sync_read_control_item",
        lambda item, ids: raw if item == "present_position_counts" else original_read(item, ids),
    )
    try:
        counts = handoff.read_positions(tuple(range(1, 9)))
        assert counts == {i: value % 4096 for i, value in raw.items()}
        assert bus.motor_register_writes == writes
        assert np.isfinite(calibration().mapped_joints(counts)).all()
        assert raw[1] == -1  # safety telemetry is not overwritten by the mapping view
    finally:
        handoff.close()
