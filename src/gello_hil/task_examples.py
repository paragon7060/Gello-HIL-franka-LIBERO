"""Read-only task guidance assets, kept outside the demonstration pool."""

from __future__ import annotations

import json
from pathlib import Path

from .position import sha256


DEFAULT_EXAMPLES = Path(__file__).resolve().parents[2] / "runs/task-examples"
REPOSITORY = "yifengzhu-hf/LIBERO-datasets"
REVISION = "f13aa24a3da8c43c7225569f28c562979fa0e35a"
SCHEMA = "gello-task-examples-v1"
NOTICE = "Original LIBERO demonstration. Position layouts can differ. Guidance only."


def source_path(task):
    suite = task["suite"].removesuffix("_swap")
    if suite not in {"libero_spatial", "libero_object", "libero_goal", "libero_10"}:
        raise ValueError("No original LIBERO reference suite for this task")
    name = task["task_name"]
    if Path(name).name != name or ".." in name:
        raise ValueError("Invalid reference task name")
    return f"{suite}/{name}_demo.hdf5"


def load_example(directory, task):
    """Require exact task identity and bytes; never infer matching from list order."""
    root = Path(directory).expanduser().resolve()
    manifest = json.loads((root / "manifest.json").read_text())
    if manifest.get("schema_version") != SCHEMA:
        raise ValueError("Unsupported task example manifest")
    entry = manifest["tasks"][task["key"]]
    for key in ("suite", "task_name"):
        if entry.get(key) != task[key]:
            raise ValueError(f"Task example {key} does not match the selected task")
    if entry.get("source_path") != source_path(task):
        raise ValueError("Task example source identity does not match")
    if entry.get("source_repository") != REPOSITORY or entry.get("source_revision") != REVISION:
        raise ValueError("Task example must use the pinned original demonstration source")
    if entry.get("pixel_orientation") != "upright_unmirrored":
        raise ValueError("Task example camera orientation is unspecified")
    path = (root / entry["file"]).resolve()
    if not path.is_relative_to(root) or path.suffix.lower() != ".mp4":
        raise ValueError("Task example must be an MP4 inside the asset directory")
    if sha256(path) != entry["sha256"]:
        raise ValueError("Task example video checksum changed; prepare it again")
    return path, entry
