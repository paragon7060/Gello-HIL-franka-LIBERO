"""Fetch only the official 40 Position BDDL/init pairs, without robot or simulator use."""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import subprocess
import urllib.request
from pathlib import Path

from gello_hil.pool import DEFAULT_PLAN, load_plan, write_new_json
from gello_hil.position import sha256

REPOSITORY = "zhouxueyang/LIBERO-Pro"
REVISION = "c86fc3b8293185a6f373677018ff3e37f8391602"


def prepare(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    plan = load_plan(DEFAULT_PLAN)
    revision = subprocess.check_output(
        ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
    ).strip()
    if revision != plan["expected_benchmark_commit"]:
        raise ValueError("use the registered LIBERO-Pro source commit")
    catalog = source / plan["catalog_source"]
    if sha256(catalog) != plan["catalog_source_sha256"]:
        raise ValueError("official task catalog differs")
    if subprocess.check_output(
        ["git", "-C", str(source), "diff", "HEAD", "--"], text=True
    ):
        raise ValueError("LIBERO-Pro source has tracked changes")
    if output.exists():
        raise FileExistsError(
            "choose a new asset directory; existing assets are never overwritten"
        )
    api = f"https://huggingface.co/api/datasets/{REPOSITORY}/tree/{REVISION}?recursive=true&limit=1000"
    with urllib.request.urlopen(api, timeout=30) as response:
        listing = {
            entry["path"]: entry
            for entry in json.load(response)
            if entry["type"] == "file"
        }
    names = []
    for task in plan["tasks"]:
        names.extend(
            [
                f"bddl_files/{task['suite']}/{task['task_name']}.bddl",
                f"init_files/{task['suite']}/{task['task_name']}.pruned_init",
            ]
        )
    if any(name not in listing for name in names):
        raise ValueError("official dataset is missing registered Position assets")
    output.mkdir(parents=True)

    def fetch(name):
        entry = listing[name]
        url = f"https://huggingface.co/datasets/{REPOSITORY}/resolve/{REVISION}/{name}"
        with urllib.request.urlopen(url, timeout=30) as response:
            data = response.read()
        if not data or len(data) != entry["size"]:
            raise ValueError(f"asset size mismatch: {name}")
        digest = hashlib.sha256(data).hexdigest()
        if entry.get("lfs"):
            valid = digest == entry["lfs"]["oid"]
        else:
            git_blob = hashlib.sha1(
                b"blob " + str(len(data)).encode() + b"\0" + data
            ).hexdigest()
            valid = git_blob == entry["oid"]
        if not valid:
            raise ValueError(f"official asset checksum mismatch: {name}")
        target = output / name
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("xb") as f:
            f.write(data)
        return {"path": name, "sha256": digest, "bytes": len(data)}

    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
        files = list(executor.map(fetch, names))
    benchmark_root = source / "libero/libero"
    config = {
        "benchmark_root": str(benchmark_root),
        "bddl_files": str(output / "bddl_files"),
        "init_states": str(output / "init_files"),
        "datasets": str(output / "datasets"),
        "assets": str(benchmark_root / "assets"),
    }
    (output / "datasets").mkdir()
    # JSON is valid YAML; avoids importing LIBERO and its first-run interactive config.
    write_new_json(output / "config/config.yaml", config)
    result = {
        "repository": REPOSITORY,
        "revision": REVISION,
        "source_commit": revision,
        "plan_sha256": sha256(DEFAULT_PLAN),
        "files": files,
        "motor_register_writes": 0,
        "simulator_steps": 0,
    }
    write_new_json(output / "download-ledger.json", result)
    print(
        f"Prepared {len(files)} official files. LIBERO_CONFIG_PATH={output / 'config'}"
    )
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--libero-source", required=True)
    parser.add_argument("--output", required=True)
    arguments = parser.parse_args()
    prepare(arguments.libero_source, arguments.output)
