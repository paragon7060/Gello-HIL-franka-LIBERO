import json
import time
from types import SimpleNamespace
from pathlib import Path

import numpy as np
import pytest

from gello_hil import cli
from gello_hil.calibration import GelloCalibration
from gello_hil.calibration_setup import (
    CalibrationCapture,
    remember_calibration,
    selected_calibration,
)
from test_alignment_safety import POSE, SIGNS, calibration, counts_for


def capture_fixture(monkeypatch, *, bad_reference=False, bad_span=False):
    monkeypatch.setattr(cli, "time", SimpleNamespace(sleep=lambda seconds: None))
    arm = counts_for(POSE)
    if bad_reference:
        arm[0] = (arm[0] + 512) % 4096
    rows = [
        {**{i: int(arm[i - 1]) for i in range(1, 8)}, 8: gripper}
        for gripper in (4000, 4000, 4001 if bad_span else 200)
    ]
    opened, closed, reads = [], [], []

    class ReadOnlyBus:
        def __init__(self, args):
            self.row = rows[len(opened)]
            opened.append(self)

        def __enter__(self):
            return self

        def __exit__(self, *args):
            closed.append(self)

        def read_positions(self, ids):
            assert tuple(ids) == tuple(range(1, 9))
            reads.append(tuple(ids))
            return dict(self.row)

    args = SimpleNamespace(port="UNCONNECTED_TEST", baudrate=57600, serial_timeout=0.12)
    return args, ReadOnlyBus, opened, closed, reads


def test_three_original_read_only_captures_save_new_valid_file_without_overwrite(
    monkeypatch, tmp_path
):
    args, factory, opened, closed, reads = capture_fixture(monkeypatch)
    capture = CalibrationCapture(args, SIGNS, bus_factory=factory)
    with pytest.raises(ValueError, match="Capture reference"):
        capture.save(tmp_path / "incomplete.json")
    for step in range(3):
        capture.capture_next()
        assert len(capture.rows) == step + 1
        assert opened == closed
    assert len(reads) == 75
    file = tmp_path / "session/calibration.json"
    capture.save(file)
    result = GelloCalibration.from_json(file)
    assert result.joint_signs == tuple(SIGNS)
    assert np.max(np.abs(result.mapped_joints(counts_for(POSE)) - POSE)) < 0.002
    value = json.loads(file.read_text())
    assert value["capture_provenance"]["motor_register_writes"] == 0
    assert (
        value["capture_provenance"]["source"]["upstream_commit"]
        == "26be20212ff0709668b776ebefabe7da1809aa73"
    )
    before = file.read_bytes()
    with pytest.raises(FileExistsError):
        capture.save(file)
    assert file.read_bytes() == before
    with pytest.raises(ValueError, match="complete"):
        capture.capture_next()


@pytest.mark.parametrize("bad_reference,bad_span", [(True, False), (False, True)])
def test_failed_reference_or_gripper_capture_is_not_accepted(monkeypatch, bad_reference, bad_span):
    args, factory, opened, closed, _ = capture_fixture(
        monkeypatch, bad_reference=bad_reference, bad_span=bad_span
    )
    capture = CalibrationCapture(args, SIGNS, bus_factory=factory)
    for _ in range(0 if bad_reference else 2):
        capture.capture_next()
    with pytest.raises(ValueError, match="residual|span"):
        capture.capture_next()
    assert len(capture.rows) == (0 if bad_reference else 2)
    assert opened == closed


def test_explicit_calibration_overrides_last_selection_and_local_fallback(tmp_path):
    file = tmp_path / "calibration.json"
    calibration().to_json(file)
    assert selected_calibration(tmp_path, fallback="historical.json") == "historical.json"
    remember_calibration(tmp_path, file)
    assert selected_calibration(tmp_path, fallback="historical.json") == str(file)
    assert selected_calibration(tmp_path, explicit="another.json") == str(
        Path("another.json").resolve()
    )
    settings = (tmp_path / "gui-settings.json").read_bytes()
    with pytest.raises(FileNotFoundError):
        remember_calibration(tmp_path, tmp_path / "missing.json")
    assert (tmp_path / "gui-settings.json").read_bytes() == settings


def test_qt_wizard_capture_save_and_cancel_wait_for_read(monkeypatch, tmp_path):
    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtWidgets as W
    from gello_hil.collector_qt import application
    from gello_hil import calibration_dialog as module
    from concurrent.futures import Future

    app = application()
    args, factory, opened, closed, _ = capture_fixture(monkeypatch)
    monkeypatch.setattr(
        module,
        "CalibrationCapture",
        lambda args, signs: CalibrationCapture(args, signs, bus_factory=factory),
    )
    file = tmp_path / "new/calibration.json"
    monkeypatch.setattr(W.QFileDialog, "getSaveFileName", lambda *a: (str(file), ""))
    dialog = module.CalibrationDialog(args, tmp_path)
    try:
        dialog.show()
        for step in range(3):
            dialog.next.click()
            deadline = time.monotonic() + 3
            while dialog.future and time.monotonic() < deadline:
                app.processEvents()
                dialog.poll()
                time.sleep(0.005)
            assert not dialog.future and len(dialog.capture.rows) == step + 1
        assert opened == closed
        dialog.next.click()
        assert dialog.saved_path == str(file)
        assert dialog.result() == W.QDialog.DialogCode.Accepted
        GelloCalibration.from_json(file)
    finally:
        dialog.close()
    cancel = module.CalibrationDialog(args, tmp_path)
    cancel.show()
    cancel.future = Future()
    cancel.reject()
    assert cancel.isVisible() and cancel.cancel_after_read
    cancel.future.set_result({i: 0 for i in range(1, 9)})
    cancel.poll()
    assert not cancel.isVisible()
