"""Independent local video player; no worker, device or collection dependencies."""

from __future__ import annotations

import cv2
from PySide6 import QtCore as C, QtGui as G, QtWidgets as W

from .task_examples import NOTICE, load_example


class TaskExamplePanel(W.QWidget):
    def __init__(self, directory, parent=None, *, enlarged=False):
        super().__init__(parent)
        self.directory = directory
        self.task = None
        self.capture = None
        self.frame_index = -1
        self.next_frame = None
        self.frames = 0
        self.fps = 20.0
        self.pixmap = None
        self.dialog = None
        self.timer = C.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.setMinimumWidth(230)
        self.setMinimumHeight(265)
        self.setStyleSheet(
            "QPushButton { padding:6px 8px; font-size:12px; } "
            "QLabel#muted { font-size:11px; }"
        )
        layout = W.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(5)
        self.title = W.QLabel("TASK EXAMPLE")
        self.title.setObjectName("eyebrow")
        layout.addWidget(self.title)
        self.caption = W.QLabel(NOTICE)
        self.caption.setWordWrap(True)
        self.caption.setObjectName("muted")
        layout.addWidget(self.caption)
        self.image = W.QLabel("Select a task")
        self.image.setAlignment(C.Qt.AlignmentFlag.AlignCenter)
        self.image.setMinimumSize(160, 96)
        self.image.setSizePolicy(W.QSizePolicy.Policy.Ignored, W.QSizePolicy.Policy.Expanding)
        self.image.setWordWrap(True)
        layout.addWidget(self.image, 1)
        self.seek = W.QSlider(C.Qt.Orientation.Horizontal)
        self.seek.valueChanged.connect(self.seek_frame)
        timeline = W.QHBoxLayout()
        timeline.addWidget(self.seek, 1)
        self.time = W.QLabel("00:00 / 00:00")
        self.time.setObjectName("muted")
        timeline.addWidget(self.time)
        layout.addLayout(timeline)
        row = W.QHBoxLayout()
        self.play = W.QPushButton("Play example")
        self.play.setObjectName("primary")
        self.play.clicked.connect(self.toggle)
        row.addWidget(self.play)
        self.replay = W.QPushButton("Replay")
        self.replay.clicked.connect(self.restart)
        row.addWidget(self.replay)
        layout.addLayout(row)
        options = W.QHBoxLayout()
        self.loop = W.QCheckBox("Loop")
        self.loop.setChecked(True)
        options.addWidget(self.loop)
        self.speed = W.QComboBox()
        for speed in (0.5, 1.0, 1.5):
            self.speed.addItem(f"{speed:g}x", speed)
        self.speed.setCurrentIndex(1)
        self.speed.currentIndexChanged.connect(self.update_speed)
        options.addWidget(self.speed)
        if not enlarged:
            expand = W.QPushButton("Enlarge")
            expand.clicked.connect(self.enlarge)
            options.addWidget(expand)
        layout.addLayout(options)
        self.set_controls(False)

    def set_controls(self, enabled):
        for control in (self.play, self.replay, self.seek, self.speed, self.loop):
            control.setEnabled(enabled)

    def set_task(self, task, *, force=False):
        if self.task == task and not force:
            return
        self.close_video()
        self.task = dict(task)
        self.title.setText(f"{task['key'].replace('_', ' ').upper()} / EXAMPLE")
        self.pixmap = None
        self.image.clear()
        self.time.setText("00:00 / 00:00")
        self.seek.setRange(0, 0)
        try:
            path, entry = load_example(self.directory, task)
            self.capture = cv2.VideoCapture(str(path))
            self.frames = int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT))
            self.fps = self.capture.get(cv2.CAP_PROP_FPS)
            if not self.capture.isOpened() or self.frames < 1 or not 0 < self.fps <= 120:
                raise ValueError("Task example video cannot be decoded")
            self.seek.setRange(0, self.frames - 1)
            self.frame_index = -1
            self.next_frame = 0
            self.show_frame(0)
            self.caption.setText(NOTICE)
            self.caption.setToolTip(
                f"{entry['source_repository']} @ {entry['source_revision']}\n"
                f"{entry['source_path']} / {entry['episode']}\n{path}"
            )
            self.set_controls(True)
        except (OSError, ValueError, KeyError) as error:
            self.close_video()
            self.image.setText("Example unavailable")
            self.caption.setText(
                f"{error}\nPrepare videos with scripts/prepare_task_examples.py."
            )
            self.caption.setToolTip(str(self.directory))
        if self.dialog:
            self.dialog.player.set_task(task, force=force)

    def show_frame(self, index):
        if self.capture is None or index == self.frame_index:
            return
        if index != self.next_frame:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, index)
        ok, frame = self.capture.read()
        if not ok:
            self.pause()
            raise ValueError("Task example frame could not be decoded")
        # Prepared examples are upright already: no policy/mirror transforms.
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = G.QImage(
            rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], G.QImage.Format.Format_RGB888
        ).copy()
        self.pixmap = G.QPixmap.fromImage(image)
        self.frame_index, self.next_frame = index, index + 1
        self.paint_frame()
        self.seek.blockSignals(True)
        self.seek.setValue(index)
        self.seek.blockSignals(False)
        def clock(frame):
            seconds = int(frame / self.fps)
            return f"{seconds // 60:02d}:{seconds % 60:02d}"
        self.time.setText(f"{clock(index)} / {clock(self.frames)}")

    def paint_frame(self):
        if self.pixmap is not None:
            self.image.setPixmap(self.pixmap.scaled(
                self.image.size(), C.Qt.AspectRatioMode.KeepAspectRatio,
                C.Qt.TransformationMode.SmoothTransformation,
            ))

    def seek_frame(self, index):
        try:
            self.show_frame(index)
        except ValueError as error:
            self.caption.setText(str(error))
            self.close_video()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.paint_frame()

    def update_speed(self):
        self.timer.setInterval(max(1, round(1000 / self.fps / self.speed.currentData())))

    def pause(self):
        self.timer.stop()
        self.play.setText("Play example")

    def toggle(self):
        if self.timer.isActive():
            self.pause()
        elif self.capture is not None:
            if self.frame_index == self.frames - 1:
                self.show_frame(0)
            self.update_speed()
            self.play.setText("Pause example")
            self.timer.start()

    def restart(self):
        if self.capture is not None:
            self.show_frame(0)
            if not self.timer.isActive():
                self.toggle()

    def tick(self):
        if self.capture is None:
            return
        index = self.frame_index + 1
        if index >= self.frames:
            if not self.loop.isChecked():
                self.pause()
                return
            index = 0
        try:
            self.show_frame(index)
        except ValueError as error:
            self.caption.setText(str(error))

    def enlarge(self):
        self.pause()
        if self.dialog is None:
            self.dialog = TaskExampleDialog(self.directory, self)
        if self.task:
            self.dialog.player.set_task(self.task)
        self.dialog.show()
        self.dialog.raise_()
        self.dialog.activateWindow()

    def close_video(self):
        self.pause()
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        self.set_controls(False)

    def shutdown(self):
        self.close_video()
        if self.dialog:
            self.dialog.close()


class TaskExampleDialog(W.QDialog):
    def __init__(self, directory, parent):
        super().__init__(parent)
        self.setWindowTitle("Task example / Original LIBERO")
        self.resize(720, 760)
        layout = W.QVBoxLayout(self)
        self.player = TaskExamplePanel(directory, self, enlarged=True)
        layout.addWidget(self.player)

    def closeEvent(self, event):
        self.player.close_video()
        self.player.task = None  # reopen must restore the released capture
        event.accept()
