"""Qt presentation layer for GELLO Studio. All hardware stays in the owned worker."""

from __future__ import annotations

import json
import os
from pathlib import Path

import cv2
from PySide6 import QtCore as C, QtGui as G, QtWidgets as W

from .collector_gui import SceneWorker, button_permissions
from .pool import init_pool, inspect_pool
from .review import confirm_attempt, enable_review


STYLE = """
QWidget { color:#e8eff8; font-family:'DejaVu Sans'; font-size:13px; }
QMainWindow, QWidget#workspace { background:#0b111b; }
QFrame#card { background:#131e2e; border:1px solid #223147; border-radius:12px; }
QFrame#viewer { background:#080e17; border:1px solid #223147; border-radius:12px; }
QLabel#brand { font-size:25px; font-weight:700; letter-spacing:-0.5px; }
QLabel#logo { background:#42dcba; color:#063b32; border-radius:12px; font-size:26px; font-weight:800; }
QLabel#eyebrow { color:#8296b0; font-size:10px; font-weight:700; letter-spacing:1.4px; }
QLabel#muted { color:#8da2bc; font-size:12px; }
QLabel#metric { font-size:22px; font-weight:600; }
QLabel#heading { font-size:17px; font-weight:600; }
QLabel#badge { background:#1a3b34; color:#6ae9c9; border-radius:7px; padding:6px 10px; font-size:10px; font-weight:700; }
QLabel#reviewTitle { font-size:16px; font-weight:600; }
QLineEdit { background:#0e1725; border:1px solid #2a3a50; border-radius:7px; padding:9px 12px; selection-background-color:#275148; }
QLineEdit:focus { border:1px solid #42dcba; }
QPushButton { background:#202e42; border:1px solid #2d3e56; border-radius:8px; padding:10px 15px; font-weight:600; }
QPushButton:hover { background:#2a3c54; border-color:#4b647f; }
QPushButton:pressed { background:#16243a; }
QPushButton:disabled { background:#142031; border-color:#1d2c3f; color:#52667f; }
QPushButton#primary { background:#42dcba; color:#073c32; border-color:#42dcba; }
QPushButton#primary:hover { background:#6ceace; border-color:#6ceace; }
QPushButton#primary:disabled { background:#183c34; color:#4d8174; border-color:#21493f; }
QPushButton#stop { color:#f0a1a6; background:#261e2b; border-color:#483044; }
QPushButton#quiet { background:transparent; border-color:transparent; color:#a9bfd7; }
QTreeWidget { background:transparent; border:0; outline:0; alternate-background-color:#152234; show-decoration-selected:1; }
QTreeWidget::item { height:29px; padding:2px 5px; border:0; border-radius:5px; }
QTreeWidget::item:hover { background:#1d3045; }
QTreeWidget::item:selected { background:#20473e; color:#9af3da; }
QHeaderView::section { background:#131e2e; border:0; color:#8296b0; font-size:10px; font-weight:600; padding:7px 5px; }
QScrollBar:vertical { background:transparent; width:6px; margin:2px; }
QScrollBar::handle:vertical { background:#34475d; border-radius:2px; min-height:28px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height:0px; }
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background:transparent; }
QProgressBar { background:#24334a; border:0; border-radius:2px; max-height:4px; min-height:4px; }
QProgressBar::chunk { background:#42dcba; border-radius:2px; }
QSlider::groove:horizontal { height:4px; background:#29394e; border-radius:2px; }
QSlider::sub-page:horizontal { background:#42dcba; border-radius:2px; }
QSlider::handle:horizontal { background:#99f4db; width:10px; height:10px; margin:-3px 0; border-radius:5px; }
QToolTip { background:#22334a; color:#e8eff8; border:1px solid #415775; padding:6px; }
"""


def label(text="", name=None, *, wrap=False):
    widget = W.QLabel(text)
    if name:
        widget.setObjectName(name)
    widget.setWordWrap(wrap)
    return widget


def card(name="card", margins=(16, 15, 16, 15)):
    widget = W.QFrame()
    widget.setObjectName(name)
    layout = W.QVBoxLayout(widget)
    layout.setContentsMargins(*margins)
    layout.setSpacing(10)
    return widget, layout


class ImageView(W.QLabel):
    def __init__(self):
        super().__init__("Preparing your scene...")
        self.setAlignment(C.Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setSizePolicy(W.QSizePolicy.Policy.Expanding, W.QSizePolicy.Policy.Expanding)
        self.original = None

    def show_image(self, image):
        self.original = G.QPixmap.fromImage(image)
        self.repaint_image()

    def repaint_image(self):
        if self.original is not None:
            self.setPixmap(
                self.original.scaled(
                    self.size(),
                    C.Qt.AspectRatioMode.KeepAspectRatio,
                    C.Qt.TransformationMode.SmoothTransformation,
                )
            )

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.repaint_image()


class CollectorWindow(W.QMainWindow):
    def __init__(self, args, *, worker_factory=SceneWorker):
        super().__init__()
        self.args = args
        self.root = Path(args.dataset_root).resolve()
        self.plan = init_pool(self.root, args.plan)
        enable_review(self.root)
        self.frozen = (self.root / "pool-manifest.json").exists()
        self.worker = worker_factory(args)
        self.key, self.shot = self.plan["tasks"][0]["key"], 1
        self.selected_attempt = None
        self.pending = None
        self.closing = False
        self.refreshing = False
        self.capture = None
        self.playing = False
        self.frame_stamp = None
        self.state = {}
        self.review_future = None
        from concurrent.futures import ThreadPoolExecutor

        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="gello-review")
        self.setWindowTitle("GELLO Studio - Position demonstrations")
        self.setStyleSheet(STYLE)
        screen = W.QApplication.primaryScreen().availableGeometry()
        self.resize(min(1510, screen.width() - 60), min(1000, screen.height() - 60))
        self.setMinimumSize(1120, 800)
        self._layout()
        self.refresh()
        pending = next((s for s in self.data["cases"] if not s["accepted"]), self.data["cases"][0])
        self.key, self.shot = pending["task_key"], pending["shot"]
        self.refresh()
        self.timer = C.QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(120)
        self.video_timer = C.QTimer(self)
        self.video_timer.timeout.connect(self.play_tick)
        if self.slot()["attempts"]:
            self.show_attempt(self.slot()["details"][-1])
        elif (
            not self.frozen
            and Path(args.libero_source).is_dir()
            and Path(args.libero_config_path).is_dir()
        ):
            C.QTimer.singleShot(250, self.open_scene)

    def tree(self, headings, widths):
        tree = W.QTreeWidget()
        tree.setHeaderLabels(headings)
        tree.setRootIsDecorated(False)
        tree.setSelectionMode(W.QAbstractItemView.SelectionMode.SingleSelection)
        tree.setUniformRowHeights(True)
        tree.setAlternatingRowColors(False)
        tree.setHorizontalScrollBarPolicy(C.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        for i, width in enumerate(widths):
            tree.setColumnWidth(i, width)
        tree.header().setStretchLastSection(True)
        return tree

    def button(self, text, callback, name=None):
        button = W.QPushButton(text)
        button.clicked.connect(callback)
        button.setCursor(C.Qt.CursorShape.PointingHandCursor)
        if name:
            button.setObjectName(name)
        return button

    def _layout(self):
        body = W.QWidget()
        body.setObjectName("workspace")
        self.setCentralWidget(body)
        outer = W.QVBoxLayout(body)
        outer.setContentsMargins(24, 20, 24, 16)
        outer.setSpacing(16)
        top = W.QHBoxLayout()
        logo = label("G", "logo")
        logo.setAlignment(C.Qt.AlignmentFlag.AlignCenter)
        logo.setFixedSize(48, 48)
        top.addWidget(logo)
        brand = W.QVBoxLayout()
        brand.setSpacing(3)
        brand.addWidget(label("GELLO Studio", "brand"))
        brand.addWidget(label("LIBERO-PRO  /  POSITION DEMONSTRATIONS", "eyebrow"))
        top.addLayout(brand)
        top.addStretch()
        stats = W.QVBoxLayout()
        stats.setSpacing(8)
        self.progress = label("000 / 400 confirmed", "metric")
        stats.addWidget(self.progress)
        self.progress_bar = W.QProgressBar()
        self.progress_bar.setRange(0, 400)
        self.progress_bar.setTextVisible(False)
        stats.addWidget(self.progress_bar)
        top.addLayout(stats)
        outer.addLayout(top)
        connection, box = card(margins=(14, 10, 14, 10))
        row = W.QHBoxLayout()
        row.setSpacing(10)
        row.addWidget(label("CONNECTION", "eyebrow"))
        self.calibration = W.QLineEdit(self.args.calibration or "")
        self.calibration.setPlaceholderText("Choose the calibration for this GELLO assembly")
        row.addWidget(self.calibration, 3)
        self.calibration_button = self.button("Calibration...", self.choose_calibration)
        row.addWidget(self.calibration_button)
        row.addSpacing(8)
        self.port = W.QLineEdit(self.args.port or "")
        self.port.setPlaceholderText("Serial port")
        row.addWidget(self.port, 2)
        row.addWidget(
            label("MOTOR CONTROLS ENABLED" if self.args.apply else "READ-ONLY REVIEW", "badge")
        )
        box.addLayout(row)
        outer.addWidget(connection)
        workspace = W.QHBoxLayout()
        workspace.setSpacing(18)
        sidebar, side = card()
        sidebar.setFixedWidth(310)
        side.addWidget(label("TASK LIBRARY", "eyebrow"))
        side.addWidget(label("40 tasks / 10 demonstrations per task", "muted"))
        search = W.QLineEdit()
        search.setPlaceholderText("Search tasks...")
        search.textChanged.connect(self.search_tasks)
        side.addWidget(search)
        self.tasks = self.tree(["TASK", "SAVED", "TRIES"], [145, 65, 45])
        self.tasks.itemSelectionChanged.connect(self.select_task)
        side.addWidget(self.tasks, 3)
        side.addSpacing(7)
        side.addWidget(label("DEMONSTRATION SLOTS", "eyebrow"))
        self.shots = self.tree(["SHOT", "INIT", "STATUS", "TRIES"], [44, 39, 110, 35])
        self.shots.itemSelectionChanged.connect(self.select_shot)
        side.addWidget(self.shots, 2)
        nav = W.QHBoxLayout()
        nav.addWidget(self.button("Refresh", self.refresh, "quiet"))
        nav.addWidget(self.button("Next pending", self.pick_next))
        side.addLayout(nav)
        workspace.addWidget(sidebar)
        main = W.QVBoxLayout()
        main.setSpacing(12)
        self.task_title = label("COLLECTION WORKSPACE", "eyebrow")
        main.addWidget(self.task_title)
        self.instruction = label("Open a scene to begin", "heading", wrap=True)
        self.instruction.setMinimumHeight(42)
        main.addWidget(self.instruction)
        viewer, display = card("viewer", (12, 10, 12, 10))
        viewer_top = W.QHBoxLayout()
        self.view_badge = label("INITIAL SCENE", "eyebrow")
        viewer_top.addWidget(self.view_badge)
        viewer_top.addStretch()
        viewer_top.addWidget(label("AGENT CAMERA", "eyebrow"))
        display.addLayout(viewer_top)
        self.preview = ImageView()
        display.addWidget(self.preview, 1)
        playback = W.QHBoxLayout()
        self.play_button = self.button("Play", self.toggle_video, "quiet")
        self.play_button.setFixedWidth(66)
        playback.addWidget(self.play_button)
        self.seek = W.QSlider(C.Qt.Orientation.Horizontal)
        self.seek.valueChanged.connect(self.seek_video)
        playback.addWidget(self.seek, 1)
        self.video_time = label("00:00 / 00:00", "muted")
        playback.addWidget(self.video_time)
        display.addLayout(playback)
        main.addWidget(viewer, 1)
        capture = W.QHBoxLayout()
        capture.setSpacing(8)
        capture.addWidget(self.button("Open scene", self.open_scene))
        self.buttons = {}
        for name, title, command, appearance in [
            ("assist", "Motor assist", lambda: self.send("assist"), None),
            ("start", "Start collection", lambda: self.send("start"), "primary"),
            ("pause", "Pause", lambda: self.send("pause"), None),
        ]:
            self.buttons[name] = self.button(title, command, appearance)
            capture.addWidget(self.buttons[name])
        capture.addStretch()
        capture.addWidget(self.button("Stop && review", self.stop, "stop"))
        main.addLayout(capture)
        review, review_layout = card()
        review_row = W.QHBoxLayout()
        review_row.setSpacing(22)
        history = W.QVBoxLayout()
        history.addWidget(label("ATTEMPT HISTORY", "eyebrow"))
        self.attempts = self.tree(["ATTEMPT", "RESULT", "SELECTION"], [120, 95, 100])
        self.attempts.setMinimumHeight(105)
        self.attempts.setMaximumHeight(145)
        self.attempts.itemSelectionChanged.connect(self.select_attempt)
        history.addWidget(self.attempts)
        review_row.addLayout(history, 1)
        decision = W.QVBoxLayout()
        self.phase = label("Ready", "reviewTitle", wrap=True)
        decision.addWidget(self.phase)
        self.detail = label(
            "Every attempt is preserved. Confirm only after reviewing the recording.",
            "muted",
            wrap=True,
        )
        self.detail.setMinimumWidth(290)
        self.detail.setMaximumWidth(350)
        decision.addWidget(self.detail, 1)
        choices = W.QHBoxLayout()
        self.buttons["retry"] = self.button("Retry shot", self.retry)
        self.buttons["confirm"] = self.button("Confirm && next", self.confirm, "primary")
        choices.addWidget(self.buttons["retry"])
        choices.addWidget(self.buttons["confirm"])
        decision.addLayout(choices)
        review_row.addLayout(decision)
        review_layout.addLayout(review_row)
        main.addWidget(review)
        workspace.addLayout(main, 1)
        outer.addLayout(workspace, 1)
        footer = W.QHBoxLayout()
        footer.addWidget(label("STATUS", "eyebrow"))
        self.status = label(
            "Preview first. Motor assist and collection require an explicit button press.",
            "muted",
            wrap=True,
        )
        footer.addWidget(self.status, 1)
        outer.addLayout(footer)

    def slot(self):
        return next(
            s for s in self.data["cases"] if (s["task_key"], s["shot"]) == (self.key, self.shot)
        )

    def error(self, error):
        self.status.setText(f"Needs attention: {error}")

    def refresh(self):
        try:
            self.data = inspect_pool(self.root)
        except Exception as error:
            self.error(error)
            return
        self.refreshing = True
        self.tasks.clear()
        for task in self.plan["tasks"]:
            slots = [s for s in self.data["cases"] if s["task_key"] == task["key"]]
            count = sum(bool(s["accepted"]) for s in slots)
            item = W.QTreeWidgetItem(
                [
                    task["key"].replace("_", " ").title(),
                    f"{count:02d} / 10",
                    str(sum(len(s["attempts"]) for s in slots)),
                ]
            )
            item.setData(0, C.Qt.ItemDataRole.UserRole, task["key"])
            self.tasks.addTopLevelItem(item)
            if task["key"] == self.key:
                self.tasks.setCurrentItem(item)
                self.tasks.scrollToItem(item)
        self.shots.clear()
        for slot in self.data["cases"]:
            if slot["task_key"] != self.key:
                continue
            status = (
                "Confirmed"
                if slot["accepted"]
                else "Review"
                if slot["pending_review"]
                else "Retry"
                if slot["attempts"]
                else "Empty"
            )
            item = W.QTreeWidgetItem(
                [
                    f"{slot['shot']:02d}",
                    str(slot["state_index"]),
                    status,
                    str(len(slot["attempts"])),
                ]
            )
            item.setData(0, C.Qt.ItemDataRole.UserRole, slot["shot"])
            if slot["accepted"] or slot["pending_review"]:
                item.setForeground(2, G.QColor("#62dfbb" if slot["accepted"] else "#e9c17b"))
            self.shots.addTopLevelItem(item)
            if slot["shot"] == self.shot:
                self.shots.setCurrentItem(item)
        self.attempts.clear()
        for index, detail in enumerate(reversed(self.slot()["details"])):
            outcome = (
                "Error"
                if detail["error"]
                else "Aborted"
                if detail["aborted"]
                else "Success"
                if detail["success"]
                else "Failed"
            )
            selection = (
                "Accepted"
                if detail["confirmed"]
                else "Candidate"
                if detail["success"] and not self.slot()["accepted"]
                else "Archived"
            )
            item = W.QTreeWidgetItem(
                [f"Attempt {len(self.slot()['details']) - index:02d}", outcome, selection]
            )
            item.setData(0, C.Qt.ItemDataRole.UserRole, detail["path"])
            item.setToolTip(0, Path(detail["path"]).name)
            item.setForeground(1, G.QColor("#62dfbb" if detail["success"] else "#ee9b9d"))
            self.attempts.addTopLevelItem(item)
            if detail["path"] == self.selected_attempt:
                self.attempts.setCurrentItem(item)
        self.task_title.setText(
            f"{self.key.replace('_', ' ').upper()}   /   DEMONSTRATION {self.shot:02d} OF 10   /   INIT {self.shot - 1}"
        )
        self.progress.setText(f"{self.data['accepted']:03d} / 400 confirmed")
        self.progress_bar.setValue(self.data["accepted"])
        self.refreshing = False

    def search_tasks(self, text):
        for index in range(self.tasks.topLevelItemCount()):
            item = self.tasks.topLevelItem(index)
            item.setHidden(text.lower() not in item.text(0).lower())

    def select_task(self):
        if self.refreshing or not self.tasks.currentItem():
            return
        key = self.tasks.currentItem().data(0, C.Qt.ItemDataRole.UserRole)
        if key != self.key:
            slots = [s for s in self.data["cases"] if s["task_key"] == key]
            self.navigate(key, next((s["shot"] for s in slots if not s["accepted"]), 1))

    def select_shot(self):
        if self.refreshing or not self.shots.currentItem():
            return
        shot = self.shots.currentItem().data(0, C.Qt.ItemDataRole.UserRole)
        if shot != self.shot:
            self.navigate(self.key, shot)

    def select_attempt(self):
        if self.refreshing or not self.attempts.currentItem():
            return
        if self.worker.running:
            self.status.setText("Stop the current scene before reviewing previous attempts.")
            return
        path = self.attempts.currentItem().data(0, C.Qt.ItemDataRole.UserRole)
        self.show_attempt(next(d for d in self.slot()["details"] if d["path"] == path))

    def navigate(self, key, shot):
        if self.worker.running:
            if self.state.get("mode") == "collection" and self.state.get("phase") == "ACTIVE":
                self.refresh()
                self.status.setText("Collection is active. Pause or stop before switching tasks.")
                return
            self.pending = (key, shot, False)
            self.worker.send("abort")
            self.status.setText("Releasing the current scene before switching...")
            return
        self.key, self.shot, self.selected_attempt = key, shot, None
        self.state = {}
        self.refresh()
        if self.slot()["attempts"]:
            self.show_attempt(self.slot()["details"][-1])
        elif not self.frozen:
            self.open_scene()

    def pick_next(self):
        pending = next((s for s in self.data["cases"] if not s["accepted"]), None)
        if pending:
            self.navigate(pending["task_key"], pending["shot"])
        else:
            self.status.setText("400/400 confirmed. Freeze the pool to generate k-shot selections.")

    def choose_calibration(self):
        if self.worker.running and self.state.get("mode") != "preview":
            self.status.setText("Stop the current scene before changing the device calibration.")
            return
        file, _ = W.QFileDialog.getOpenFileName(
            self, "Choose the calibration for this physical assembly", "", "Calibration (*.json)"
        )
        if file:
            self.calibration.setText(file)
            self.open_scene()

    def open_scene(self, checked=False, *, retry=False):
        try:
            self.args.calibration = self.calibration.text().strip()
            self.args.port = self.port.text().strip()
            if self.worker.running:
                if (
                    self.worker.slot == (self.key, self.shot)
                    and not retry
                    and self.state.get("mode") != "preview"
                ):
                    return
                self.pending = (self.key, self.shot, retry)
                self.worker.send("abort")
                return
            self.close_video()
            self.worker.open(self.key, self.shot, retry=retry, previous=self.selected_attempt)
            self.selected_attempt = None
            self.frame_stamp = None
            self.phase.setText("Loading initial scene")
            self.view_badge.setText("INITIAL SCENE")
            self.status.setText(
                "Preview only. Motors and collection begin only after an explicit button press."
            )
        except Exception as error:
            self.error(error)

    def send(self, command):
        self.worker.send(command)
        self.status.setText(
            {
                "assist": "Support GELLO during alignment. Press Start collection when ready.",
                "start": "Collection requested. Pause and Stop & review remain available.",
                "pause": "Pause requested. Motor force is released before re-anchoring.",
            }[command]
        )

    def stop(self):
        self.pending = None
        if self.worker.running:
            self.worker.send("abort")
            self.status.setText("Releasing motors and saving the raw attempt...")

    def retry(self):
        if not self.frozen and self.args.apply:
            self.open_scene(retry=True)

    def confirm(self):
        if not self.selected_attempt or self.worker.running or self.review_future:
            return
        self.status.setText("Validating and confirming the selected demonstration...")
        self.review_future = self.executor.submit(confirm_attempt, self.root, self.selected_attempt)
        self.buttons["confirm"].setEnabled(False)

    def show_attempt(self, detail):
        self.close_video()
        self.selected_attempt = detail["path"]
        directory = self.root / detail["path"]
        meta = json.loads((directory / "attempt.json").read_text())
        self.instruction.setText(meta.get("instruction", "Recorded demonstration"))
        self.view_badge.setText("ATTEMPT REVIEW")
        self.phase.setText(
            "Confirmed demonstration"
            if detail["confirmed"]
            else "Ready for review"
            if detail["success"]
            else "Retry available"
        )
        self.detail.setText(
            f"{'SUCCESS' if detail['success'] else 'NOT ACCEPTED'} / {detail['steps']} steps\n"
            + (meta.get("error") or "Review the recording before confirming.")
        )
        self.capture = cv2.VideoCapture(str(directory / "operator-view.mp4"))
        if self.capture.isOpened():
            self.frame_count = int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT))
            self.fps = max(1, self.capture.get(cv2.CAP_PROP_FPS))
            self.seek.blockSignals(True)
            self.seek.setRange(0, max(0, self.frame_count - 1))
            self.seek.setValue(0)
            self.seek.blockSignals(False)
            self.video_frame(0)
        else:
            self.capture.release()
            self.capture = None
            self.preview.original = None
            self.preview.setText(
                "No video / zero-action attempt\nCheck the result and preserved error details."
            )
        self.refresh()

    def video_frame(self, index):
        if self.capture is None:
            return
        self.capture.set(cv2.CAP_PROP_POS_FRAMES, int(index))
        okay, bgr = self.capture.read()
        if okay:
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            height, width = rgb.shape[:2]
            self.preview.show_image(
                G.QImage(
                    rgb.data, width, height, rgb.strides[0], G.QImage.Format.Format_RGB888
                ).copy()
            )
            position, total = int(index / self.fps), int(self.frame_count / self.fps)
            self.video_time.setText(
                f"{position // 60:02d}:{position % 60:02d} / {total // 60:02d}:{total % 60:02d}"
            )

    def seek_video(self, value):
        if not self.playing:
            self.video_frame(value)

    def toggle_video(self):
        if self.capture is None or self.worker.running:
            return
        self.playing = not self.playing
        self.play_button.setText("Pause" if self.playing else "Play")
        if self.playing:
            self.video_timer.start(max(20, int(1000 / self.fps)))
        else:
            self.video_timer.stop()

    def play_tick(self):
        if not self.playing or self.capture is None:
            return
        index = (self.seek.value() + 1) % max(1, self.frame_count)
        self.video_frame(index)
        self.seek.blockSignals(True)
        self.seek.setValue(index)
        self.seek.blockSignals(False)

    def close_video(self):
        self.playing = False
        if hasattr(self, "video_timer"):
            self.video_timer.stop()
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        self.play_button.setText("Play")

    def poll(self):
        try:
            if self.review_future and self.review_future.done():
                future, self.review_future = self.review_future, None
                decision = future.result()
                self.refresh()
                self.status.setText(
                    f"Confirmed {decision['attempt']}. Moving to the next pending shot."
                )
                if not self.closing:
                    self.pick_next()
            if self.worker.process is not None:
                self.state = self.worker.state()
                if self.worker.running:
                    phase = self.state.get("phase", "LOADING")
                    title = {
                        "LOADING": "Loading initial scene",
                        "PREVIEW": "Initial scene / select calibration",
                        "PAUSED": "Ready to collect / paused",
                        "ACTIVE": "Recording demonstration",
                        "ALIGNMENT READY": "Alignment ready",
                    }.get(phase, phase)
                    self.phase.setText(title)
                    self.view_badge.setText("RECORDING" if phase == "ACTIVE" else "LIVE PREVIEW")
                    self.detail.setText(
                        self.state.get("preview_error") or self.state.get("notice", "")
                    )
                    if self.state.get("instruction"):
                        self.instruction.setText(self.state["instruction"])
                    image = self.worker.control / "frame.jpg"
                    if image.exists() and image.stat().st_mtime_ns != self.frame_stamp:
                        self.frame_stamp = image.stat().st_mtime_ns
                        self.preview.show_image(G.QImage.fromData(image.read_bytes()))
                else:
                    result = self.worker.finish()
                    self.refresh()
                    path = result["state"].get("attempt_path")
                    if path and Path(path).is_relative_to(self.root / "attempts"):
                        relative = str(Path(path).relative_to(self.root))
                        detail = next(
                            (d for d in self.slot()["details"] if d["path"] == relative), None
                        )
                        if detail:
                            self.show_attempt(detail)
                        self.status.setText(
                            "Review the video. Confirm & next saves the selection; Retry shot preserves the previous attempt."
                        )
                    elif result["returncode"] and not self.pending and not self.closing:
                        self.phase.setText("Scene unavailable")
                        self.detail.setText("Check the renderer and LIBERO-Pro configuration.")
                        self.error(f"Scene preparation failed. Log: {result['log_file']}")
                    if self.pending:
                        self.key, self.shot, retry = self.pending
                        self.pending = None
                        self.refresh()
                        self.open_scene(retry=retry)
            candidate = next(
                (d for d in self.slot()["details"] if d["path"] == self.selected_attempt), {}
            )
            permissions = button_permissions(
                self.state,
                writable=self.args.apply,
                running=self.worker.running,
                candidate_success=candidate.get("success", False),
                frozen=self.frozen,
            )
            for key, allowed in permissions.items():
                self.buttons[key].setEnabled(
                    bool(allowed and not self.review_future and not self.closing)
                )
            settings_editable = (
                not self.closing
                and not self.review_future
                and (not self.worker.running or self.state.get("mode") == "preview")
            )
            for widget in (self.calibration, self.calibration_button, self.port):
                widget.setEnabled(bool(settings_editable))
            if self.closing and not self.worker.running and not self.review_future:
                self.close()
        except Exception as error:
            self.error(error)

    def closeEvent(self, event):
        if self.worker.running or self.review_future:
            self.closing = True
            self.pending = None
            self.worker.send("abort")
            self.status.setText("Releasing motors and preserving the raw attempt before closing...")
            event.ignore()
            return
        self.close_video()
        self.worker.finish()
        self.executor.shutdown(wait=False)
        self.timer.stop()
        event.accept()


def application():
    # OpenCV's Qt5 plugin path is incompatible with this independent Qt6 app.
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = C.QLibraryInfo.path(
        C.QLibraryInfo.LibraryPath.PluginsPath
    )
    os.environ.pop("QT_QPA_FONTDIR", None)
    app = W.QApplication.instance() or W.QApplication([])
    app.setFont(G.QFont("DejaVu Sans", 10))
    return app


def launch(args):
    app = application()
    window = CollectorWindow(args)
    window.show()
    return app.exec()
