"""English three-step desktop calibration wizard; capture is always read-only."""

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path

from PySide6 import QtCore as C, QtWidgets as W

from .calibration_setup import CalibrationCapture


class CalibrationDialog(W.QDialog):
    def __init__(self, args, root, parent=None):
        super().__init__(parent)
        self.args, self.root = args, Path(root)
        self.capture = None
        self.future = None
        self.saved_path = None
        self.cancel_after_read = False
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="gello-calibration")
        self.setWindowTitle("New GELLO calibration")
        self.setMinimumWidth(680)
        layout = W.QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(16)
        title = W.QLabel("Create a calibration for this GELLO assembly")
        title.setObjectName("heading")
        layout.addWidget(title)
        text = W.QLabel(
            "Read-only encoder capture. Support GELLO manually; motor assistance stays off.\n"
            "Joint signs depend on your physical assembly. Defaults are examples."
        )
        text.setWordWrap(True)
        layout.addWidget(text)
        row = W.QHBoxLayout()
        row.addWidget(W.QLabel("Joint signs"))
        self.signs = []
        from .cli import DEFAULT_SIGNS

        for index, sign in enumerate(DEFAULT_SIGNS, 1):
            row.addWidget(W.QLabel(f"J{index}"))
            box = W.QComboBox()
            box.addItems(["+1", "-1"])
            box.setCurrentText(f"{sign:+d}")
            self.signs.append(box)
            row.addWidget(box)
        layout.addLayout(row)
        self.step = W.QLabel()
        self.step.setObjectName("heading")
        layout.addWidget(self.step)
        self.instructions = W.QLabel()
        self.instructions.setWordWrap(True)
        self.instructions.setMinimumHeight(90)
        layout.addWidget(self.instructions)
        self.feedback = W.QLabel("25 encoder samples per capture; no register writes.")
        self.feedback.setWordWrap(True)
        self.feedback.setTextInteractionFlags(C.Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.feedback)
        controls = W.QHBoxLayout()
        self.restart = W.QPushButton("Restart captures")
        self.restart.clicked.connect(self.reset)
        controls.addWidget(self.restart)
        controls.addStretch()
        self.cancel = W.QPushButton("Cancel")
        self.cancel.clicked.connect(self.reject)
        controls.addWidget(self.cancel)
        self.next = W.QPushButton("Capture reference pose")
        self.next.setObjectName("primary")
        self.next.clicked.connect(self.advance)
        controls.addWidget(self.next)
        layout.addLayout(controls)
        self.timer = C.QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(60)
        self.render_step()

    def render_step(self):
        step = len(self.capture.rows) if self.capture else 0
        titles = [
            "1 / 3 - Reference joint pose",
            "2 / 3 - Open gripper",
            "3 / 3 - Close gripper",
            "Calibration ready to save",
        ]
        instructions = [
            "Match every GELLO joint to the Panda reference:\n"
            "J1 0 deg   J2 0 deg   J3 0 deg   J4 -90 deg\n"
            "J5 0 deg   J6 +90 deg   J7 0 deg\nHold still, then capture.",
            "Keep the arm in its reference pose. Fully OPEN the passive GELLO gripper, then capture.",
            "Keep the arm in its reference pose. Fully CLOSE the passive GELLO gripper, then capture.",
            "Save a new JSON file and connect it to GELLO Studio.\n"
            "Check joint directions and the mapping before using motor assistance.",
        ]
        self.step.setText(titles[step])
        self.instructions.setText(instructions[step])
        self.next.setText(
            [
                "Capture reference pose",
                "Capture open gripper",
                "Capture closed gripper",
                "Save && use calibration",
            ][step]
        )
        for box in self.signs:
            box.setEnabled(step == 0 and not self.future)

    def reset(self):
        if not self.future:
            self.capture = None
            self.feedback.setText("Captures cleared. Existing calibration files remain unchanged.")
            self.render_step()

    def advance(self):
        if self.future:
            return
        if self.capture and len(self.capture.rows) == 3:
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            default = self.root / "calibrations" / f"{stamp}.json"
            file, _ = W.QFileDialog.getSaveFileName(
                self, "Save a new calibration", str(default), "Calibration (*.json)"
            )
            if file:
                try:
                    self.saved_path = self.capture.save(file)
                    self.accept()
                except Exception as error:
                    self.feedback.setText(f"Not saved: {error}")
            return
        if not self.capture or not self.capture.rows:
            self.capture = CalibrationCapture(
                self.args, [int(box.currentText()) for box in self.signs]
            )
        self.future = self.executor.submit(self.capture.capture_next)
        self.next.setEnabled(False)
        self.restart.setEnabled(False)
        for box in self.signs:
            box.setEnabled(False)
        self.feedback.setText("Reading encoder samples... Hold the requested pose.")

    def poll(self):
        if not self.future or not self.future.done():
            return
        future, self.future = self.future, None
        try:
            row = future.result()
            self.feedback.setText(
                "Captured IDs 1-8: " + ", ".join(str(row[i]) for i in range(1, 9))
            )
        except Exception as error:
            self.feedback.setText(
                f"Capture failed: {error}\nReposition or check the connection, then capture again."
            )
        self.next.setEnabled(True)
        self.restart.setEnabled(True)
        self.render_step()
        if self.cancel_after_read:
            self.reject()

    def done(self, result):
        if self.future:
            self.cancel_after_read = True
            self.feedback.setText("Waiting for the read-only capture to close before cancelling...")
            return
        self.timer.stop()
        self.executor.shutdown(wait=False)
        super().done(result)

    def closeEvent(self, event):
        if self.future:
            self.cancel_after_read = True
            self.feedback.setText("Waiting for the read-only capture to close before cancelling...")
            event.ignore()
        else:
            super().closeEvent(event)
