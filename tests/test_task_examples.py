import importlib.util
import io
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from gello_hil.http_range import RangeReader
from gello_hil.pool import DEFAULT_PLAN
from gello_hil.task_examples import REPOSITORY, REVISION, SCHEMA, load_example, source_path


TASKS = json.loads(DEFAULT_PLAN.read_text())["tasks"]


def video_fixture(root, tasks=TASKS[:2]):
    root.mkdir(exist_ok=True)
    manifest = {"schema_version": SCHEMA, "tasks": {}}
    for offset, task in enumerate(tasks):
        file = root / f"{task['key']}.mp4"
        writer = cv2.VideoWriter(str(file), cv2.VideoWriter_fourcc(*"mp4v"), 20, (128, 128))
        for i in range(12):
            frame = np.zeros((128, 128, 3), np.uint8)
            frame[:64, :64] = (20, 40, 200)  # asymmetric corners detect accidental mirrors
            frame[64:, 64:] = (200, 40, 20)
            frame[50:70, 50:70] = i * 15 + offset
            writer.write(frame)
        writer.release()
        from gello_hil.position import sha256
        manifest["tasks"][task["key"]] = {
            "suite": task["suite"], "task_name": task["task_name"], "file": file.name,
            "sha256": sha256(file), "source_path": source_path(task), "episode": "demo_0",
            "source_repository": REPOSITORY, "source_revision": REVISION,
            "pixel_orientation": "upright_unmirrored",
        }
    (root / "manifest.json").write_text(json.dumps(manifest))
    return manifest


def test_all_40_original_task_paths_are_unique_and_preserve_names():
    paths = [source_path(task) for task in TASKS]
    assert len(set(paths)) == 40
    for task, path in zip(TASKS, paths):
        assert Path(path).stem == task["task_name"] + "_demo"
        assert path.split('/')[0] == task["suite"].removesuffix("_swap")
    catalog = json.loads((DEFAULT_PLAN.parent / "libero_task_example_sources.json").read_text())
    assert catalog["repository"] == REPOSITORY and catalog["revision"] == REVISION
    assert set(paths) == set(catalog["files"])
    assert all(len(v["sha256"]) == 64 and v["bytes"] > 0 for v in catalog["files"].values())


@pytest.mark.parametrize("mutation", ["name", "source", "revision", "orientation", "escape", "bytes"])
def test_mismatched_or_tampered_video_is_not_shown(tmp_path, mutation):
    task = TASKS[0]
    manifest = video_fixture(tmp_path)
    path, _ = load_example(tmp_path, task)
    entry = manifest["tasks"][task["key"]]
    if mutation == "name":
        entry["task_name"] = TASKS[1]["task_name"]
    elif mutation == "source":
        entry["source_path"] = source_path(TASKS[1])
    elif mutation == "revision":
        entry["source_revision"] = "main"
    elif mutation == "orientation":
        entry["pixel_orientation"] = "policy"
    elif mutation == "escape":
        entry["file"] = "../outside.mp4"
    else:
        path.write_bytes(b"changed")
    (tmp_path / "manifest.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError):
        load_example(tmp_path, task)


class FakeHTTP:
    def __init__(self, data, *, status=206, wrong_range=False):
        self.data, self.status, self.wrong_range = data, status, wrong_range
        self.calls = []

    def get(self, url, *, headers, **kwargs):
        start, end = map(int, headers['Range'][6:].split('-'))
        end = min(end, len(self.data) - 1)
        self.calls.append((start, end))
        class Response:
            status_code = self.status
            raw = io.BytesIO(self.data[start:end + 1])
            def __enter__(response):
                response.headers = {
                    "Content-Range": f"bytes {start + int(self.wrong_range)}-{end}/{len(self.data)}"
                }
                return response
            def __exit__(self, *a):
                pass
        return Response()


def test_http_range_file_seeks_cache_and_bounds_reads():
    data = bytes(range(256)) * 3000
    session = FakeHTTP(data)
    with RangeReader("https://example.invalid/file", session=session) as file:
        assert file.read(8) == data[:8]
        assert len(session.calls) == 1
        file.seek(300_001)
        assert file.read(9000) == data[300_001:309_001]
        file.seek(-4, 2)
        assert file.read(20) == data[-4:]
        assert file.read() == b""
    assert all(last - start + 1 < len(data) for start, last in session.calls)


@pytest.mark.parametrize("status,wrong", [(200, False), (206, True)])
def test_range_server_must_not_return_a_whole_file_or_wrong_block(status, wrong):
    with pytest.raises(ValueError, match="Range|range"):
        RangeReader("https://example.invalid/file", session=FakeHTTP(b'x' * 300_000, status=status, wrong_range=wrong))


def test_http_download_budget_is_finite():
    with RangeReader("unused", session=FakeHTTP(b'x' * 1_000_000), byte_limit=300_000) as file:
        file.seek(500_000)
        with pytest.raises(ValueError, match="budget"):
            file.read(10)


def test_player_is_upright_loops_changes_task_and_releases(tmp_path, monkeypatch):
    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from gello_hil.collector_qt import application
    from gello_hil.task_example_qt import TaskExamplePanel
    app = application()
    video_fixture(tmp_path)
    panel = TaskExamplePanel(tmp_path)
    panel.set_task(TASKS[0])
    image = panel.pixmap.toImage()
    assert image.pixelColor(20, 20).red() > 180
    assert image.pixelColor(100, 20).red() < 20
    panel.show_frame(11)
    panel.tick()
    assert panel.frame_index == 0
    panel.loop.setChecked(False)
    panel.show_frame(11)
    panel.toggle()
    assert panel.timer.isActive() and panel.frame_index == 0
    panel.show_frame(11)
    panel.tick()
    assert not panel.timer.isActive()
    panel.toggle()
    old = panel.capture
    panel.set_task(TASKS[1])
    assert not old.isOpened() and not panel.timer.isActive() and panel.frame_index == 0
    panel.speed.setCurrentIndex(0)
    assert panel.timer.interval() == 100
    panel.enlarge()
    app.processEvents()
    assert panel.dialog.player.capture.isOpened()
    panel.set_task(TASKS[0])
    assert panel.dialog.player.task["key"] == TASKS[0]["key"]
    panel.dialog.close()
    assert panel.dialog.player.capture is None
    panel.enlarge()
    assert panel.dialog.player.capture.isOpened()
    panel.shutdown()
    assert panel.capture is None and not panel.timer.isActive()
    assert panel.dialog.player.capture is None
    panel.close()


def test_missing_example_disables_only_player(tmp_path, monkeypatch):
    pytest.importorskip("PySide6")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from gello_hil.collector_qt import application
    from gello_hil.task_example_qt import TaskExamplePanel
    app = application()
    panel = TaskExamplePanel(tmp_path)
    panel.set_task(TASKS[0])
    assert app is not None
    assert not panel.play.isEnabled() and panel.capture is None
    assert "unavailable" in panel.image.text()
    panel.shutdown()


def test_extraction_uses_only_demo0_and_flips_opengl_vertically(tmp_path):
    h5py = pytest.importorskip("h5py")
    spec = importlib.util.spec_from_file_location(
        "prepare_examples_test", Path(__file__).resolve().parents[1] / "scripts/prepare_task_examples.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    source = tmp_path / 'input.hdf5'
    pixels = np.zeros((5, 128, 128, 3), np.uint8)
    pixels[:, :64, :64, 0] = 220
    with h5py.File(source, 'w') as file:
        data = file.create_group('data')
        data.attrs['bddl_file_name'] = TASKS[0]['task_name'] + '.bddl'
        data.attrs['macros_image_convention'] = 'opengl'
        data.attrs['problem_info'] = json.dumps({'language_instruction': 'test task'})
        demo = data.create_group('demo_0')
        demo.create_dataset('obs/agentview_rgb', data=pixels)
        demo.create_dataset('dones', data=[0, 0, 0, 0, 1])
        demo.create_dataset('rewards', data=[0, 0, 0, 0, 1])
    result = module.extract(source, TASKS[0], tmp_path / 'example.mp4')
    capture = cv2.VideoCapture(str(tmp_path / 'example.mp4'))
    ok, frame = capture.read()
    capture.release()
    assert ok and frame[100, 20, 2] > 200 and frame[20, 20, 2] < 20
    assert result['episode'] == 'demo_0' and result['frames'] == 5
    with h5py.File(source, 'r+') as file:
        file['data/demo_0/rewards'][-1] = 0
    with pytest.raises(ValueError, match='terminal success'):
        module.extract(source, TASKS[0], tmp_path / 'example2.mp4')
