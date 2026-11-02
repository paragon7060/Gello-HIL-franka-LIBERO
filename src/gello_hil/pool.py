"""Operator-driven 40-task demo pool and immutable, nested k-shot selections."""

from __future__ import annotations

import copy
import fcntl
import hashlib
import json
from pathlib import Path

from .position import resolve_case, sha256

DEFAULT_PLAN = Path(__file__).parent / "plans/libero_pro_position40_pool10.json"
PAYLOADS = (
    "attempt.json",
    "calibration.json",
    "trajectory.npz",
    "observations.npz",
    "operator-view.mp4",
)
SUITES = {
    "libero_spatial_swap",
    "libero_object_swap",
    "libero_goal_swap",
    "libero_10_swap",
}


def write_new_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as f:
        f.write(json.dumps(value, indent=2, allow_nan=False) + "\n")


def load_plan(path):
    plan = json.loads(Path(path).read_text())
    tasks = plan.get("tasks", [])
    if (
        plan.get("schema_version") != "gello-position-pool-v2"
        or plan.get("shots_per_task") != 10
        or plan.get("motor_assistance_required") is not True
        or plan.get("support_init_indices") != list(range(10))
        or len(tasks) != 40
        or len({t["key"] for t in tasks}) != 40
        or {t["suite"] for t in tasks} != SUITES
    ):
        raise ValueError(
            "pool must register all four Position suites, 40 tasks and 10 support states"
        )
    for suite in SUITES:
        group = [t for t in tasks if t["suite"] == suite]
        if len(group) != 10 or {t["task_id"] for t in group} != set(range(10)):
            raise ValueError("each Position suite must contain official task IDs0..9")
    for task in tasks:
        # Keys become directory names, so a plan cannot escape the dataset root.
        if not task["key"] or any(
            c not in "abcdefghijklmnopqrstuvwxyz0123456789_" for c in task["key"]
        ):
            raise ValueError("invalid pool task key")
        resolve_case(path, task["key"], None, 10)
    return plan


def init_pool(root, plan_path=DEFAULT_PLAN):
    root, plan_path = Path(root), Path(plan_path)
    plan = load_plan(plan_path)
    target = root / "pool-plan.json"
    root.mkdir(parents=True, exist_ok=True)
    if target.exists():
        if target.read_bytes() != plan_path.read_bytes():
            raise ValueError("dataset already belongs to a different immutable plan")
    else:
        with target.open("xb") as f:
            f.write(plan_path.read_bytes())
    return plan


_HEADERS = {}


def _attempt_header(header):
    """Small attempt.json summary, parsed again only after the file changes."""
    stat = header.stat()
    version = (stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns)
    cached = _HEADERS.get(str(header))
    if cached and cached[0] == version:
        return cached[1]
    meta = json.loads(header.read_text())
    summary = {
        "position_case": meta.get("position_case") or {},
        "motor_assistance_requested": meta.get("motor_assistance_requested"),
        "success": meta.get("success"),
        "aborted": meta.get("aborted"),
        "error": meta.get("error"),
        "instruction": meta.get("instruction", ""),
        "steps": meta.get("session", {}).get("active_step_count", len(meta.get("diagnostics", []))),
        "created_at_utc": meta.get("created_at_utc"),
        "sha256": None,
    }
    _HEADERS[str(header)] = (version, summary)
    return summary


def inspect_pool(root, *, strict=True):
    """Read metadata only; full raw validation happens when freezing the pool.

    strict=False lists attempt folders without attempt.json (an interrupted save)
    as incomplete instead of stopping, so review can continue around them.
    """
    root = Path(root)
    plan_path = root / "pool-plan.json"
    plan = load_plan(plan_path)
    from .review import current_confirmation, review_required
    manual = review_required(root)
    cases = []
    failed_count = 0
    for task in plan["tasks"]:
        for shot in range(1, 11):
            case = resolve_case(plan_path, task["key"], None, shot)
            folder = root / "attempts" / task["key"] / f"shot{shot:02d}"
            attempts, accepted, unresolved, details = [], [], [], []
            confirmation = current_confirmation(root, task["key"], shot) if manual else None
            for directory in sorted(folder.iterdir()) if folder.exists() else []:
                if not directory.is_dir():
                    continue
                header = directory / "attempt.json"
                if not header.is_file():
                    unresolved.append(str(directory.relative_to(root)))
                    details.append({"path": unresolved[-1], "success": False, "aborted": False,
                                    "error": "Incomplete attempt: attempt.json is missing",
                                    "confirmed": False, "incomplete": True, "instruction": "",
                                    "steps": 0, "created_at_utc": directory.name})
                    continue
                meta = _attempt_header(header)
                actual = meta["position_case"]
                if any(
                    actual.get(k) != case[k]
                    for k in (
                        "key",
                        "suite",
                        "task_id",
                        "task_name",
                        "shot",
                        "state_index",
                        "plan_sha256",
                    )
                ):
                    raise ValueError(f"foreign case in pool: {header}")
                if meta.get("motor_assistance_requested") is not True:
                    raise ValueError(
                        f"passive attempt cannot enter motor-assisted pool: {header}"
                    )
                attempts.append(str(directory.relative_to(root)))
                successful = (
                    meta.get("success") is True
                    and not meta.get("error")
                    and not meta.get("aborted")
                )
                relative = str(directory.relative_to(root))
                selected = successful and (not manual or (
                    confirmation is not None and confirmation["attempt"] == relative))
                if selected:
                    if meta["sha256"] is None:
                        meta["sha256"] = sha256(header)
                    if manual and confirmation["sha256"].get("attempt.json") != meta["sha256"]:
                        raise ValueError("confirmed attempt metadata changed")
                    accepted.append(relative)
                failed_count += not successful
                details.append({"path": relative, "success": bool(successful),
                                "aborted": bool(meta.get("aborted")), "error": meta.get("error"),
                                "confirmed": bool(selected), "instruction": meta["instruction"],
                                "steps": meta["steps"],
                                "created_at_utc": meta.get("created_at_utc") or directory.name})
            if manual and confirmation and confirmation["attempt"] not in attempts:
                raise ValueError("confirmed attempt is missing")
            if len(accepted) > 1:
                raise ValueError(
                    f"duplicate successful support slot: {case['key']} shot{shot}"
                )
            if unresolved and strict:
                raise ValueError(
                    f"unresolved attempt ownership/artifacts: {unresolved}"
                )
            cases.append(
                {
                    "task_key": task["key"],
                    "shot": shot,
                    "state_index": shot - 1,
                    "accepted": accepted[0] if accepted else None,
                    "attempts": attempts,
                    "details": details,
                    "pending_review": (sum(d["success"] for d in details) if not accepted else 0),
                }
            )
    return {
        "schema_version": "gello-position-pool-status-v1",
        "plan_sha256": sha256(plan_path),
        "target": 400,
        "accepted": sum(c["accepted"] is not None for c in cases),
        "failed_attempts": failed_count,
        "operator_confirmation_required": manual,
        "cases": cases,
    }


def collect_pool(args):
    from .cli import position_collect

    if not args.apply:
        raise ValueError(
            "collect-position-pool can move motors; read SAFETY.md and add --apply"
        )
    if args.max_attempts < 1:
        raise ValueError("max-attempts must be positive")
    root = Path(args.dataset_root).resolve()
    init_pool(root, args.plan)
    with (root / ".collection.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if (root / "pool-manifest.json").exists():
            raise ValueError("pool is frozen; use its manifest for training selections")
        status = inspect_pool(root)
        pending = [c for c in status["cases"] if not c["accepted"]]
        if args.task_key:
            if args.task_key not in {c["task_key"] for c in status["cases"]}:
                raise ValueError("unknown task key")
            pending = [c for c in pending if c["task_key"] == args.task_key]
        print(
            f"Accepted {status['accepted']}/400; failed/aborted attempts {status['failed_attempts']}"
        )
        for slot in pending[: args.max_attempts]:
            if slot["attempts"] and not args.retry_failed:
                print(
                    f"STOP: {slot['task_key']} shot{slot['shot']} has a preserved failed attempt. "
                    "Inspect it; --retry-failed explicitly permits one new attempt for this slot."
                )
                return 1
            current = copy.copy(args)
            current.plan = str(root / "pool-plan.json")
            current.draw, current.shot, current.task_key = (
                None,
                slot["shot"],
                slot["task_key"],
            )
            current.mapping, current.assisted = "absolute", True
            current.operator_review_required = status["operator_confirmation_required"]
            current.output_root = str(
                root / "attempts" / slot["task_key"] / f"shot{slot['shot']:02d}"
            )
            print(
                f"\nNEXT {slot['task_key']} shot{slot['shot']:02d}/10, native init{slot['state_index']}"
            )
            # Each invocation retains the terminal motor gate and GUI READY/Enter gate.
            if position_collect(current) != 0:
                print("STOP: failure/abort preserved; no automatic retry or next task.")
                return 1
            accepted = inspect_pool(root)
            case = next(
                c
                for c in accepted["cases"]
                if (c["task_key"], c["shot"]) == (slot["task_key"], slot["shot"])
            )
            if not case["accepted"]:
                if status["operator_confirmation_required"]:
                    print("Saved review candidate. Open position-pool-gui and confirm or retry.")
                    return 0
                raise ValueError(
                    "collector returned success without an accepted attempt"
                )
        print(
            json.dumps(
                {k: v for k, v in inspect_pool(root).items() if k != "cases"}, indent=2
            )
        )
    return 0


def freeze_pool(root):
    from .exports import load_support

    root = Path(root).resolve()
    with (root / ".collection.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        output = root / "pool-manifest.json"
        if output.exists():
            raise FileExistsError(output)
        status = inspect_pool(root)
        if status["accepted"] != 400:
            raise ValueError(
                f"pool incomplete: {status['accepted']}/400; partial pools cannot freeze"
            )
        episodes = []
        for slot in status["cases"]:
            directory = root / slot["accepted"]
            meta, case, actions, _ = load_support(directory)
            if (
                case.get("plan_schema") != "gello-position-pool-v2"
                or meta.get("motor_assistance_requested") is not True
                or meta.get("success_hold_steps", 0) < 20
                or meta.get("native_success_streak", 0) < meta["success_hold_steps"]
                or len(actions) < meta["success_hold_steps"]
            ):
                raise ValueError(
                    "support does not meet pool collection/success contract"
                )
            if case.get("benchmark_commit") != case["expected_benchmark_commit"]:
                raise ValueError("missing pinned benchmark provenance")
            if not meta.get("source", {}).get("commit") or not meta["source"].get(
                "source_sha256"
            ):
                raise ValueError("missing collector commit/source provenance")
            files = {}
            for name in PAYLOADS:
                path = directory / name
                if not path.is_file() or not path.stat().st_size:
                    raise ValueError(f"missing/nonempty pool artifact: {path}")
                files[name] = sha256(path)
            if files["calibration.json"] != meta["calibration"].get("snapshot_sha256"):
                raise ValueError("calibration snapshot checksum mismatch")
            episodes.append(
                {
                    "episode_id": f"{slot['task_key']}/shot{slot['shot']:02d}",
                    "task_key": slot["task_key"],
                    "shot": slot["shot"],
                    "suite": case["suite"],
                    "task_id": case["task_id"],
                    "task_name": case["task_name"],
                    "state_index": case["state_index"],
                    "attempt": slot["accepted"],
                    "actions": len(actions),
                    "sha256": files,
                }
            )
            from .review import require_confirmation
            decision = require_confirmation(directory, meta)
            if decision:
                episodes[-1]["confirmation"] = {"file": decision["event_file"],
                                                "sha256": decision["event_sha256"]}
        result = {
            "schema_version": "gello-position-frozen-pool-v1",
            "plan_sha256": status["plan_sha256"],
            "collection_method": "human_gello_motor_assisted",
            "tasks": 40,
            "shots_per_task": 10,
            "episodes": episodes,
            "failed_attempts_preserved": status["failed_attempts"],
            "support_init_indices": list(range(10)),
            "query_used_for_selection": False,
        }
        write_new_json(output, result)
    return result


def select_shots(manifest, output_dir, shots=(1, 3, 5, 10), seeds=(42,)):
    manifest, output_dir = Path(manifest).resolve(), Path(output_dir)
    pool = json.loads(manifest.read_text())
    if pool.get("schema_version") != "gello-position-frozen-pool-v1":
        raise ValueError("selection requires a complete frozen pool manifest")
    episodes = pool["episodes"]
    keys = sorted({e["task_key"] for e in episodes})
    if len(episodes) != 400 or len(keys) != 40:
        raise ValueError("selection requires 40 complete tasks")
    for key in keys:
        group = [e for e in episodes if e["task_key"] == key]
        if len(group) != 10 or {e["shot"] for e in group} != set(range(1, 11)):
            raise ValueError("incomplete or duplicated task shots")
        if any(e["state_index"] != e["shot"] - 1 for e in group):
            raise ValueError("non-support init in frozen pool")
    if (
        not shots
        or not seeds
        or any(type(k) is not int or not 1 <= k <= 10 for k in shots)
        or any(type(s) is not int or s < 0 for s in seeds)
        or len(set(shots)) != len(shots)
        or len(set(seeds)) != len(seeds)
    ):
        raise ValueError("unique shots1..10 and nonnegative integer seeds required")
    outputs = [
        output_dir / f"seed{seed}_k{k:02d}.json" for seed in seeds for k in shots
    ]
    if any(path.exists() for path in outputs):
        raise FileExistsError("selection outputs already exist")
    # Recheck every accepted raw artifact once before deriving any subset.
    for episode in episodes:
        directory = (manifest.parent / episode["attempt"]).resolve()
        if not directory.is_relative_to(manifest.parent):
            raise ValueError("attempt escapes pool directory")
        for name in PAYLOADS:
            if sha256(directory / name) != episode["sha256"].get(name):
                raise ValueError("frozen raw artifact changed")
        if episode.get("confirmation"):
            confirmation = episode["confirmation"]
            if sha256(manifest.parent / confirmation["file"]) != confirmation["sha256"]:
                raise ValueError("frozen operator confirmation changed")
    results = []
    for seed in seeds:
        ordered = {}
        for key in keys:
            ordered[key] = sorted(
                (e for e in episodes if e["task_key"] == key),
                key=lambda e: hashlib.sha256(
                    f"gello-pool-v1:{seed}:{e['episode_id']}".encode()
                ).hexdigest(),
            )
        for k in shots:
            selected = [episode for key in keys for episode in ordered[key][:k]]
            result = {
                "schema_version": "gello-kshot-selection-v1",
                "seed": seed,
                "shots_per_task": k,
                "tasks": 40,
                "episode_count": 40 * k,
                "pool_manifest": str(manifest),
                "pool_manifest_sha256": sha256(manifest),
                "selection_rule": "SHA256 rank(gello-pool-v1:seed:episode_id), prefix k per task",
                "nested_with_same_seed": True,
                "query_used_for_selection": False,
                "episodes": selected,
            }
            path = output_dir / f"seed{seed}_k{k:02d}.json"
            write_new_json(path, result)
            results.append(str(path))
    return results


def add_pool_commands(sub):
    from .cli import _hardware_arguments

    init = sub.add_parser(
        "init-position-pool", help="create an immutable 40-task x 10-demo pool"
    )
    init.add_argument("--dataset-root", required=True)
    init.add_argument("--plan", default=str(DEFAULT_PLAN))
    init.set_defaults(
        func=lambda a: (
            init_pool(a.dataset_root, a.plan),
            print("Immutable pool plan ready; use position-pool-status for progress"),
            0,
        )[-1]
    )
    status = sub.add_parser(
        "position-pool-status", help="read 400-slot progress without hardware"
    )
    status.add_argument("--dataset-root", required=True)
    status.set_defaults(
        func=lambda a: (print(json.dumps(inspect_pool(a.dataset_root), indent=2)), 0)[
            -1
        ]
    )
    run = sub.add_parser(
        "collect-position-pool",
        help="interactive A6 collection; skip complete slots, stop on failure",
    )
    _hardware_arguments(run)
    run.add_argument("--calibration", required=True)
    run.add_argument("--libero-source", required=True)
    run.add_argument("--libero-config-path", required=True)
    run.add_argument("--dataset-root", required=True)
    run.add_argument("--plan", default=str(DEFAULT_PLAN))
    run.add_argument("--task-key", help="optional one-task session, e.g. object_07")
    run.add_argument("--max-attempts", type=int, default=400)
    run.add_argument(
        "--retry-failed",
        action="store_true",
        help="explicitly permit one new attempt per failed slot",
    )
    run.add_argument("--apply", action="store_true")
    run.add_argument("--seed", type=int, default=0)
    run.add_argument("--resolution", type=int, default=256)
    run.add_argument("--window-scale", type=float, default=3.0)
    run.add_argument("--max-active-steps", type=int, default=1000)
    run.add_argument("--success-hold-steps", type=int, default=20)
    run.set_defaults(func=collect_pool)
    freeze = sub.add_parser(
        "freeze-position-pool", help="validate all 400 raw episodes and freeze hashes"
    )
    freeze.add_argument("--dataset-root", required=True)
    freeze.set_defaults(
        func=lambda a: (freeze_pool(a.dataset_root), print("Frozen 400/400"), 0)[-1]
    )
    subset = sub.add_parser(
        "select-shots", help="select nested k-shot manifests from a frozen pool"
    )
    subset.add_argument("--pool-manifest", required=True)
    subset.add_argument("--shots", type=int, nargs="+", default=[1, 3, 5, 10])
    subset.add_argument("--seeds", type=int, nargs="+", default=[42])
    subset.add_argument("--output-dir", required=True)
    subset.set_defaults(
        func=lambda a: (
            print(
                json.dumps(
                    select_shots(a.pool_manifest, a.output_dir, a.shots, a.seeds),
                    indent=2,
                )
            ),
            0,
        )[-1]
    )
