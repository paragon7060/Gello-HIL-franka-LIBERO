import json
from pathlib import Path

import numpy as np
import pytest

from gello_hil import cli, pool
from gello_hil.position import resolve_case, sha256


def fake_attempt(root, key, shot, *, success=True, assisted=True, name="001"):
    case = resolve_case(root / "pool-plan.json", key, None, shot)
    case["benchmark_commit"] = case["expected_benchmark_commit"]
    directory = root / "attempts" / key / f"shot{shot:02d}" / name
    directory.mkdir(parents=True)
    (directory / "calibration.json").write_text("{}\n")
    meta = {
        "success": success,
        "error": None,
        "aborted": False,
        "motor_assistance_requested": assisted,
        "position_case": case,
        "controller": "OSC_POSE",
        "success_hold_steps": 20,
        "native_success_streak": 20,
        "source": {
            "commit": "test-collector-commit",
            "source_sha256": {"collector.py": "test"},
        },
        "calibration": {"snapshot_sha256": sha256(directory / "calibration.json")},
    }
    (directory / "attempt.json").write_text(json.dumps(meta))
    np.savez(directory / "trajectory.npz", actions=np.zeros((20, 7), dtype=np.float32),
             counts=np.zeros((20, 8), dtype=np.int64), follower_joints=np.zeros((20, 7)),
             action_elapsed_seconds=np.arange(20) / 20)
    np.savez(
        directory / "observations.npz",
        agentview_image=np.zeros((21, 2, 2, 3), np.uint8),
        wrist_image=np.zeros((21, 2, 2, 3), np.uint8),
        state=np.tile([0, 0, 0, 0, 0, 0, 1, 0.04, 0.04], (21, 1)),
        sim_states=np.zeros((21, 3)),
        elapsed_seconds=np.arange(21) / 20,
    )
    (directory / "operator-view.mp4").write_bytes(b"synthetic-test-video")
    return directory


def test_full_pool_has_all_400_slots_and_excludes_query(tmp_path):
    plan = pool.init_pool(tmp_path)
    status = pool.inspect_pool(tmp_path)
    assert status["accepted"] == 0 and len(status["cases"]) == 400
    assert {c["state_index"] for c in status["cases"]} == set(range(10))
    assert len({(c["task_key"], c["state_index"]) for c in status["cases"]}) == 400
    assert {t["suite"] for t in plan["tasks"]} == pool.SUITES
    assert not set(plan["support_init_indices"]) & set(plan["excluded_init_indices"])
    with pytest.raises(ValueError, match="no --draw"):
        resolve_case(pool.DEFAULT_PLAN, "object_07", 1, 10)
    with pytest.raises(ValueError):
        resolve_case(pool.DEFAULT_PLAN, "object_07", None, 11)
    with pytest.raises(ValueError, match="incomplete"):
        pool.freeze_pool(tmp_path)


def test_foreign_passive_duplicate_and_unresolved_attempts_stop(tmp_path):
    pool.init_pool(tmp_path)
    directory = fake_attempt(tmp_path, "object_07", 1, assisted=False)
    with pytest.raises(ValueError, match="passive"):
        pool.inspect_pool(tmp_path)
    header = directory / "attempt.json"
    meta = json.loads(header.read_text())
    meta["motor_assistance_requested"] = True
    meta["position_case"]["state_index"] = 25
    header.write_text(json.dumps(meta))
    with pytest.raises(ValueError, match="foreign"):
        pool.inspect_pool(tmp_path)
    meta["position_case"]["state_index"] = 0
    header.write_text(json.dumps(meta))
    duplicate = fake_attempt(tmp_path, "object_07", 1, name="002")
    with pytest.raises(ValueError, match="duplicate"):
        pool.inspect_pool(tmp_path)
    (duplicate / "attempt.json").unlink()
    with pytest.raises(ValueError, match="unresolved"):
        pool.inspect_pool(tmp_path)


def test_interactive_queue_skips_success_stops_failure_requires_explicit_retry(
    tmp_path, monkeypatch
):
    pool.init_pool(tmp_path)
    fake_attempt(tmp_path, "object_07", 1)
    fake_attempt(tmp_path, "object_07", 2, success=False)
    args = cli.build_parser().parse_args(
        [
            "collect-position-pool",
            "--dataset-root",
            str(tmp_path),
            "--port",
            "unused",
            "--calibration",
            "unused",
            "--libero-source",
            "unused",
            "--libero-config-path",
            "unused",
            "--task-key",
            "object_07",
            "--apply",
            "--max-attempts",
            "3",
        ]
    )
    calls = []

    def collect(current):
        calls.append(current.shot)
        assert current.assisted and current.apply and current.draw is None
        fake_attempt(
            tmp_path,
            current.task_key,
            current.shot,
            success=current.shot == 2,
            name="002",
        )
        return 0 if current.shot == 2 else 1

    monkeypatch.setattr(cli, "position_collect", collect)
    assert pool.collect_pool(args) == 1 and calls == []
    args.retry_failed = True
    assert pool.collect_pool(args) == 1 and calls == [2, 3]
    status = pool.inspect_pool(tmp_path)
    assert status["accepted"] == 2 and status["failed_attempts"] == 2


def test_pool_motor_gate_precedes_hardware(tmp_path, monkeypatch):
    args = cli.build_parser().parse_args(
        [
            "collect-position-pool",
            "--dataset-root",
            str(tmp_path),
            "--port",
            "unused",
            "--calibration",
            "unused",
            "--libero-source",
            "unused",
            "--libero-config-path",
            "unused",
        ]
    )
    monkeypatch.setattr(
        cli, "position_collect", lambda args: pytest.fail("no hardware allowed")
    )
    with pytest.raises(ValueError, match="add --apply"):
        pool.collect_pool(args)


def test_freeze_all_400_nested_subsets_integrity_and_no_overwrites(tmp_path):
    root = tmp_path / "pool"
    plan = pool.init_pool(root)
    for task in plan["tasks"]:
        for shot in range(1, 11):
            fake_attempt(root, task["key"], shot)
    frozen = pool.freeze_pool(root)
    assert len(frozen["episodes"]) == 400
    manifest = root / "pool-manifest.json"
    outputs = pool.select_shots(manifest, tmp_path / "selections", seeds=[42, 43])
    assert len(outputs) == 8
    selected = {Path(path).stem: json.loads(Path(path).read_text()) for path in outputs}
    for seed in (42, 43):
        sets = []
        for k in (1, 3, 5, 10):
            selection = selected[f"seed{seed}_k{k:02d}"]
            assert selection["episode_count"] == 40 * k
            assert selection["pool_manifest_sha256"] == sha256(manifest)
            sets.append({e["episode_id"] for e in selection["episodes"]})
        assert sets[0] < sets[1] < sets[2] < sets[3]
    assert selected["seed42_k03"]["episodes"] != selected["seed43_k03"]["episodes"]
    again = pool.select_shots(manifest, tmp_path / "same-seed", shots=[3], seeds=[42])
    assert (
        json.loads(Path(again[0]).read_text())["episodes"]
        == selected["seed42_k03"]["episodes"]
    )
    with pytest.raises(FileExistsError):
        pool.select_shots(manifest, tmp_path / "selections")
    with pytest.raises(FileExistsError):
        pool.freeze_pool(root)
    episode = frozen["episodes"][0]
    (root / episode["attempt"] / "observations.npz").write_bytes(b"changed")
    with pytest.raises(ValueError, match="artifact changed"):
        pool.select_shots(manifest, tmp_path / "tampered")
