"""Prepare one original LIBERO task video per Position task; never runs a simulator."""

from __future__ import annotations

import argparse
import json
import os
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from urllib.parse import quote

import cv2
import h5py
import numpy as np

from gello_hil.http_range import RangeReader
from gello_hil.pool import DEFAULT_PLAN
from gello_hil.position import sha256
from gello_hil.review import atomic_json, pool_lock
from gello_hil.task_examples import (
    DEFAULT_EXAMPLES, NOTICE, REPOSITORY, REVISION, SCHEMA, load_example, source_path,
)


def extract(source, task, output):
    """Select demo_0 before inspecting performance; verify recorded terminal signals."""
    with h5py.File(source, "r") as file:
        group = file["data"]
        recorded_name = Path(str(group.attrs["bddl_file_name"])).stem
        if recorded_name != task["task_name"]:
            raise ValueError("Source HDF5 task identity mismatch")
        if group.attrs.get("macros_image_convention") != "opengl":
            raise ValueError("Source image convention must be declared as OpenGL")
        episode = group["demo_0"]
        images = episode["obs/agentview_rgb"]
        if images.dtype != np.uint8 or len(images.shape) != 4 or images.shape[-1] != 3:
            raise ValueError("Unsupported original demonstration images")
        if not 1 < images.shape[0] <= 6000 or images.shape[1:3] != (128, 128):
            raise ValueError("Unexpected demonstration image size")
        dones, rewards = episode["dones"], episode["rewards"]
        if not bool(dones[-1]) or not float(rewards[-1]) > 0:
            raise ValueError("demo_0 has no recorded terminal success; do not select another silently")
        pixels = images[:]
        instruction = json.loads(group.attrs["problem_info"])["language_instruction"]
    temporary = output.with_suffix(".partial.mp4")
    writer = cv2.VideoWriter(str(temporary), cv2.VideoWriter_fourcc(*"mp4v"), 20, (128, 128))
    if not writer.isOpened():
        raise RuntimeError("MP4 encoder could not be opened")
    try:
        for frame in pixels:
            # LIBERO HDF5 declares OpenGL bottom-up pixels. Flip vertically only.
            writer.write(cv2.cvtColor(frame[::-1], cv2.COLOR_RGB2BGR))
    finally:
        writer.release()
    capture = cv2.VideoCapture(str(temporary))
    try:
        if not capture.isOpened() or int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) != len(pixels):
            raise ValueError("Prepared reference frame coverage mismatch")
        for index in (0, len(pixels) - 1):
            capture.set(cv2.CAP_PROP_POS_FRAMES, index)
            if not capture.read()[0]:
                raise ValueError("Prepared reference cannot be decoded")
    finally:
        capture.release()
    os.replace(temporary, output)
    return {
        "suite": task["suite"], "task_name": task["task_name"],
        "file": output.name, "sha256": sha256(output),
        "source_repository": REPOSITORY, "source_revision": REVISION,
        "source_path": source_path(task), "episode": "demo_0",
        "instruction": instruction, "source_camera": "agentview_rgb",
        "source_image_convention": "opengl", "pixel_orientation": "upright_unmirrored",
        "transform": "vertical_flip_only", "fps": 20, "frames": len(pixels),
        "source_terminal_done": True, "source_terminal_reward_positive": True,
        "role": "operator_guidance_only_not_support_or_query", "notice": NOTICE,
    }


def prepare(task, output, local_source=None):
    path = source_path(task)
    destination = output / f"{task['key']}.mp4"
    catalog = json.loads((DEFAULT_PLAN.parent / "libero_task_example_sources.json").read_text())
    if catalog["repository"] != REPOSITORY or catalog["revision"] != REVISION:
        raise ValueError("Original demonstration catalog revision mismatch")
    expected = catalog["files"][path]
    if local_source and (local_source / path).is_file():
        local_file = local_source / path
        if local_file.stat().st_size != expected["bytes"] or sha256(local_file) != expected["sha256"]:
            raise ValueError("Local HDF5 differs from the pinned original; no silent fallback")
        result = extract(local_file, task, destination)
        result.update(source_access="verified_local_hdf5", source_hdf5_sha256=expected["sha256"])
        return result
    url = f"https://huggingface.co/datasets/{REPOSITORY}/resolve/{REVISION}/{quote(path)}"
    with RangeReader(url) as source:
        if source.size != expected["bytes"]:
            raise ValueError("Original HDF5 size differs from the pinned catalog")
        result = extract(source, task, destination)
        result.update(
            source_access="pinned_https_range", source_hdf5_sha256_expected=expected["sha256"],
            full_source_hash_verified=False,
            http_bytes_read=source.bytes_read, http_range_requests=source.requests,
        )
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_EXAMPLES)
    parser.add_argument("--plan", type=Path, default=DEFAULT_PLAN)
    parser.add_argument("--local-datasets", type=Path, help="optional existing LIBERO HDF5 root")
    parser.add_argument("--workers", type=int, choices=range(1, 5), default=4)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    tasks = json.loads(args.plan.read_text())["tasks"]
    with pool_lock(args.output):
        manifest_path = args.output / "manifest.json"
        manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {
            "schema_version": SCHEMA, "tasks": {}, "errors": {}, "notice": NOTICE,
        }
        pending = []
        for task in tasks:
            try:
                load_example(args.output, task)
                print(f"cached {task['key']}", flush=True)
            except (OSError, ValueError, KeyError):
                pending.append(task)
        with ThreadPoolExecutor(max_workers=args.workers) as executor:
            work = {executor.submit(prepare, task, args.output, args.local_datasets): task for task in pending}
            for future in as_completed(work):
                key = work[future]["key"]
                try:
                    manifest["tasks"][key] = future.result()
                    manifest["errors"].pop(key, None)
                    print(f"ready {key}", flush=True)
                except Exception as error:
                    manifest["errors"][key] = f"{type(error).__name__}: {error}"
                    print(f"unavailable {key}: {error}", flush=True)
                atomic_json(manifest_path, manifest, replace=True)
        complete = all(task["key"] in manifest["tasks"] and task["key"] not in manifest["errors"] for task in tasks)
        print(json.dumps({"complete": complete, "ready": len(manifest["tasks"]), "tasks": len(tasks)}))
        return 0 if complete else 1


if __name__ == "__main__":
    raise SystemExit(main())
