"""Finite, read-only encoder input connection; no actuation imports."""
from __future__ import annotations

import hashlib
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

from .calibration import GelloCalibration
from .dynamixel import ReadOnlyDynamixelBus
from .recording import source_provenance
from .ui import close_window, poll_command


def import_calibration(args):
    source, output = Path(args.source), Path(args.output)
    if output.exists():
        raise FileExistsError(output)
    cal = GelloCalibration.from_session_json(source)
    cal.to_json(output)
    payload = json.loads(output.read_text())
    payload["import_provenance"] = {
        "source": str(source.resolve()), "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "historical_capture_not_fresh_calibration": True,
        "requires_physical_direction_and_pose_check": True,
    }
    output.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"Imported historical calibration: {output.resolve()}")
    return 0


def encoder_panel(row):
    canvas = np.zeros((520, 850, 3), np.uint8)
    lines = ["GELLO INPUT / READ ONLY / NO MOTOR WRITES", "q / ESC / close: stop",
             "ID   ENCODER COUNT   CALIBRATED JOINT (rad)"]
    for index, (motor_id, count) in enumerate(row["counts"].items()):
        mapped = f'{row["mapped_joints"][index]:+.4f}' if index < 7 else f'gripper {row["gripper_ratio"]:.3f}'
        lines.append(f"{motor_id:>2}       {count:>4}           {mapped}")
    lines += ["Historical calibration: verify assembly signs / offsets before teleop.",
              "No simulator is being stepped. No follower commands are sent."]
    for i, line in enumerate(lines):
        cv2.putText(canvas, line, (18, 35 + 35*i), cv2.FONT_HERSHEY_SIMPLEX,
                    .56, (70, 240, 100) if i == 0 else (230, 230, 230), 1, cv2.LINE_AA)
    return canvas


def read_input(args):
    if args.samples < 1 or not 0 < args.hz <= 50:
        raise ValueError("read-input requires finite samples>=1 and 0<hz<=50")
    output = Path(args.output)
    if output.exists():
        raise FileExistsError(output)
    cal = GelloCalibration.from_json(Path(args.calibration))
    ids = cal.arm_ids + (cal.gripper_id,)
    result = {"created_at_utc": datetime.now(timezone.utc).isoformat(), "source": source_provenance(),
              "baudrate": args.baudrate, "calibration_sha256": hashlib.sha256(Path(args.calibration).read_bytes()).hexdigest(),
              "allowed_instructions": ["PING", "READ", "SYNC_READ"], "motor_register_writes": 0,
              "gui_requested": bool(args.gui), "requested_samples": args.samples,
              "samples": [], "error": None}
    window = "GELLO encoder input (read only)"
    started = time.monotonic()
    try:
        if args.gui:
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        with ReadOnlyDynamixelBus(args.port, baudrate=args.baudrate,
                                 timeout_seconds=args.serial_timeout) as bus:
            result["ping"] = [bus.ping(i) for i in ids]
            torque = {}
            for i in ids:
                error, value = bus.read(i, 64, 1)
                if error:
                    raise OSError(f"ID {i} torque read error {error}")
                torque[i] = int.from_bytes(value, "little")
            result["torque_enable_readback"] = torque
            for _ in range(args.samples):
                tick = time.monotonic()
                counts = {int(i): int(v) for i, v in bus.read_positions(ids).items()}
                row = {"elapsed_seconds": tick - started, "counts": counts,
                       "mapped_joints": cal.mapped_joints(counts).tolist(),
                       "gripper_ratio": cal.gripper_ratio(counts)}
                result["samples"].append(row)
                if args.gui:
                    cv2.imshow(window, encoder_panel(row))
                    if poll_command(window) == "abort":
                        break
                time.sleep(max(0, 1/args.hz - (time.monotonic() - tick)))
    except (Exception, KeyboardInterrupt) as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if args.gui:
            close_window(window)
        result["completed_samples"] = len(result["samples"])
        result["passed"] = bool(result["samples"] and result["error"] is None)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("x") as f:
            json.dump(result, f, indent=2, allow_nan=False)
        if result["samples"] and getattr(args, "preview", None):
            if not cv2.imwrite(str(args.preview), encoder_panel(result["samples"][-1])):
                raise OSError("failed to save encoder preview")
    print(json.dumps({"passed": result["passed"], "samples": result["completed_samples"],
                      "error": result["error"], "output": str(output.resolve())}))
    return 0 if result["passed"] else 1
