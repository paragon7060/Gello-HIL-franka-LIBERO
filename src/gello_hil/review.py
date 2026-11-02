"""Append-only operator decisions; raw attempts remain immutable."""

from __future__ import annotations

import fcntl
import json
import os
import uuid
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path

from .position import resolve_case, sha256

POLICY = "review-policy.json"


def atomic_json(path, value, *, replace=False):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("x") as f:
            f.write(json.dumps(value, indent=2, allow_nan=False) + "\n")
            f.flush()
            os.fsync(f.fileno())
        if replace:
            os.replace(temporary, path)
        else:
            os.link(temporary, path)  # atomic publication without overwriting a decision
    finally:
        temporary.unlink(missing_ok=True)


@contextmanager
def pool_lock(root):
    with (Path(root) / ".collection.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield lock


def review_required(root):
    path = Path(root) / POLICY
    if not path.exists():
        return False
    policy = json.loads(path.read_text())
    if policy.get("schema_version") != "gello-operator-review-v1":
        raise ValueError("unknown review policy")
    return policy.get("operator_confirmation_required") is True


def enable_review(root):
    root = Path(root)
    with pool_lock(root):
        if (root / "pool-manifest.json").exists():
            return review_required(root)
        if not (root / POLICY).exists():
            atomic_json(
                root / POLICY,
                {
                    "schema_version": "gello-operator-review-v1",
                    "operator_confirmation_required": True,
                    "created_at_utc": datetime.now(timezone.utc).isoformat(),
                },
            )
    return review_required(root)


def slot_events(root, key, shot):
    folder = Path(root) / "reviews" / key / f"shot{shot:02d}"
    previous = None
    events = []
    for path in sorted(folder.glob("*.json")):
        value = json.loads(path.read_text())
        if (
            value.get("schema_version") != "gello-review-event-v1"
            or value.get("previous_event_sha256") != previous
            or value.get("task_key") != key
            or value.get("shot") != shot
        ):
            raise ValueError("invalid review history/checksum chain")
        previous = sha256(path)
        events.append((path, value))
    return events


def current_confirmation(root, key, shot):
    selected = None
    for path, event in slot_events(root, key, shot):
        if event["decision"] == "confirmed":
            selected = {
                **event,
                "event_file": str(path.relative_to(root)),
                "event_sha256": sha256(path),
            }
    return selected


def record_event(root, key, shot, decision, **fields):
    """Caller must hold the shared collection lock."""
    events = slot_events(root, key, shot)
    value = {
        "schema_version": "gello-review-event-v1",
        "task_key": key,
        "shot": shot,
        "decision": decision,
        "operator_source": "dataset_gui_button",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "previous_event_sha256": sha256(events[-1][0]) if events else None,
        **fields,
    }
    path = (
        Path(root)
        / "reviews"
        / key
        / f"shot{shot:02d}"
        / f"{len(events) + 1:06d}_{uuid.uuid4().hex}.json"
    )
    atomic_json(path, value)
    return {
        **value,
        "event_file": str(path.relative_to(root)),
        "event_sha256": sha256(path),
    }


def find_pool(attempt):
    for parent in Path(attempt).resolve().parents:
        if (parent / "pool-plan.json").is_file():
            return parent
    return None


def require_confirmation(attempt, meta):
    root = find_pool(attempt)
    if not meta.get("operator_review_required") and (root is None or not review_required(root)):
        return None
    if root is None:
        raise ValueError("review-required attempt must remain in its registered pool")
    case = meta["position_case"]
    event = current_confirmation(root, case["key"], case["shot"])
    relative = str(Path(attempt).resolve().relative_to(root))
    if event is None or event["attempt"] != relative:
        raise ValueError("operator confirmation required before training/export")
    from .pool import PAYLOADS

    if any(sha256(Path(attempt) / name) != event["sha256"][name] for name in PAYLOADS):
        raise ValueError("confirmed raw attempt changed")
    return event


def confirm_attempt(root, relative):
    from .exports import load_support
    from .pool import PAYLOADS, inspect_pool

    root = Path(root).resolve()
    directory = (root / relative).resolve()
    if not directory.is_relative_to(root / "attempts"):
        raise ValueError("attempt must belong to this pool")
    with pool_lock(root):
        if (root / "pool-manifest.json").exists():
            raise ValueError("frozen pool cannot change confirmations")
        if not review_required(root):
            raise ValueError("operator review policy is required")
        # This also rejects foreign/misfiled task/shot/init metadata.
        status = inspect_pool(root)
        relative = str(directory.relative_to(root))
        if not any(relative in slot["attempts"] for slot in status["cases"]):
            raise ValueError("unregistered attempt")
        meta, case, actions, _ = load_support(directory, require_review=False)
        expected = resolve_case(root / "pool-plan.json", case["key"], None, case["shot"])
        if (
            meta.get("success_hold_steps", 0) < 20
            or len(actions) < meta["success_hold_steps"]
            or meta.get("native_success_streak", 0) < meta["success_hold_steps"]
            or case.get("benchmark_commit") != expected["expected_benchmark_commit"]
            or not meta.get("source", {}).get("commit")
            or not meta["source"].get("source_sha256")
        ):
            raise ValueError("attempt does not meet native pool success/provenance contract")
        hashes = {}
        for name in PAYLOADS:
            file = directory / name
            if not file.is_file() or file.stat().st_size == 0:
                raise ValueError(f"missing/nonempty raw artifact: {name}")
            hashes[name] = sha256(file)
        if hashes["calibration.json"] != meta["calibration"].get("snapshot_sha256"):
            raise ValueError("calibration snapshot checksum mismatch")
        previous = current_confirmation(root, case["key"], case["shot"])
        if previous and previous["attempt"] == relative and previous["sha256"] == hashes:
            return previous
        return record_event(
            root,
            case["key"],
            case["shot"],
            "confirmed",
            attempt=relative,
            sha256=hashes,
            replaces=previous["attempt"] if previous else None,
        )
