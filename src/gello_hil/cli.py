"""Command line interface for probe, calibration, mapping, and LIBERO HIL."""

from __future__ import annotations

import argparse
import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

from .calibration import PANDA_REFERENCE_POSE, GelloCalibration, infer_quarter_turn_offsets
from .control import AbsoluteJointFollower, RelativeClutchFollower, TargetDiscontinuity
from .dynamixel import ReadOnlyDynamixelBus
from .session import OperatorSession, decode_key
from .ui import close_window, operator_panel

DEFAULT_SIGNS = (1, 1, 1, -1, 1, -1, 1)


def _bus(args):
    return ReadOnlyDynamixelBus(
        args.port, baudrate=args.baudrate, timeout_seconds=args.serial_timeout
    )


def _configure_libero(args) -> None:
    if getattr(args, "libero_config_path", None):
        os.environ["LIBERO_CONFIG_PATH"] = str(Path(args.libero_config_path).resolve())


def _sample_positions(bus, ids, samples: int, interval: float) -> dict[int, int]:
    rows = []
    for _ in range(samples):
        values = bus.read_positions(ids)
        rows.append([values[motor_id] for motor_id in ids])
        if interval:
            time.sleep(interval)
    median = np.rint(np.median(np.asarray(rows), axis=0)).astype(np.int64)
    return {motor_id: int(value) for motor_id, value in zip(ids, median)}


def probe(args) -> int:
    ids = tuple(args.arm_ids) + (args.gripper_id,)
    with _bus(args) as bus:
        reports = [bus.ping(motor_id) for motor_id in ids]
        positions = bus.read_positions(ids)
    print(json.dumps({"ping": reports, "present_position": positions}, indent=2))
    return 0


def calibrate(args) -> int:
    output = Path(args.output)
    if output.exists() and not args.overwrite:
        answer = input(f"Calibration exists at {output}. Overwrite it? [y/N]: ")
        if answer.strip().lower() != "y":
            print("Calibration preserved.")
            return 1
    ids = tuple(args.arm_ids) + (args.gripper_id,)
    with _bus(args) as bus:
        print("\nSTEP 1/3 — Match GELLO to the Panda reference joint-by-joint:")
        print("  J1=0, J2=0, J3=0, J4=-90deg, J5=0, J6=+90deg, J7=0")
        input("Press Enter only after all seven joints match: ")
        pose = _sample_positions(bus, ids, args.samples, args.sample_interval)
        arm_counts = [pose[i] for i in args.arm_ids]
        inferred = infer_quarter_turn_offsets(arm_counts, PANDA_REFERENCE_POSE, args.joint_signs)
        print("Calibration residual (rad):", np.round(inferred["calibration_residual_radians"], 3))
        print("Requested count correction:", inferred["required_count_deltas"])
        if inferred["max_abs_calibration_residual_rad"] > args.max_residual:
            raise RuntimeError(
                "reference pose is too far from a quarter-turn branch; reposition GELLO and retry"
            )
        print("\nSTEP 2/3 — Fully OPEN the passive GELLO gripper.")
        input("Press Enter to capture open: ")
        opened = _sample_positions(bus, ids, args.samples, args.sample_interval)
        print("\nSTEP 3/3 — Fully CLOSE the passive GELLO gripper.")
        input("Press Enter to capture closed: ")
        closed = _sample_positions(bus, ids, args.samples, args.sample_interval)
    calibration = GelloCalibration.create(
        arm_ids=args.arm_ids,
        gripper_id=args.gripper_id,
        calibration_counts=arm_counts,
        joint_signs=args.joint_signs,
        gripper_open_count=opened[args.gripper_id],
        gripper_closed_count=closed[args.gripper_id],
    )
    calibration.to_json(output)
    print(f"Wrote calibration: {output.resolve()}")
    return 0


def mapping_check(args) -> int:
    _configure_libero(args)
    from .libero_runtime import make_env, render_joint_pose

    calibration = GelloCalibration.from_json(Path(args.calibration))
    ids = calibration.arm_ids + (calibration.gripper_id,)
    window = "GELLO mapping check (read-only)"
    env = None
    try:
        env, _, _ = make_env(
            args.suite,
            args.task_id,
            args.state_index,
            resolution=args.resolution,
            seed=args.seed,
        )
        with _bus(args) as bus:
            while True:
                counts = bus.read_positions(ids)
                joints = calibration.mapped_joints(counts)
                errors = np.asarray(PANDA_REFERENCE_POSE) - joints
                frame = render_joint_pose(env, joints)
                canvas = operator_panel(
                    frame,
                    instruction="DRY-RUN MAPPING CHECK — rendered pose only; no simulator step",
                    status="PAUSED",
                    active_steps=0,
                    max_steps=0,
                    mapping_mode="absolute calibrated joints",
                    joints=joints,
                    errors=errors,
                    notice="Move each joint and confirm its displayed direction. q exits.",
                    scale=args.window_scale,
                )
                cv2.imshow(window, canvas)
                if decode_key(cv2.waitKey(1)) == "abort":
                    break
    finally:
        close_window(window)
        if env is not None:
            env.close()
    return 0


def _json_safe(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _save_attempt(output, *, metadata, actions, counts, joints, frames) -> None:
    output.mkdir(parents=True, exist_ok=False)
    (output / "attempt.json").write_text(
        json.dumps(_json_safe(metadata), indent=2) + "\n", encoding="utf-8"
    )
    np.savez_compressed(
        output / "trajectory.npz",
        actions=np.asarray(actions, dtype=np.float32).reshape((-1, 8)),
        counts=np.asarray(counts, dtype=np.int64).reshape((-1, 8)),
        follower_joints=np.asarray(joints, dtype=np.float64).reshape((-1, 7)),
    )
    if frames:
        height, width = frames[0].shape[:2]
        writer = cv2.VideoWriter(
            str(output / "operator-view.mp4"),
            cv2.VideoWriter_fourcc(*"mp4v"),
            20.0,
            (width, height),
        )
        for frame in frames:
            writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        writer.release()


def collect(args) -> int:
    _configure_libero(args)
    from .libero_runtime import follower_joints, make_env, rgb_frame

    calibration = GelloCalibration.from_json(Path(args.calibration))
    controller_class = (
        AbsoluteJointFollower if args.mapping == "absolute" else RelativeClutchFollower
    )
    controller = controller_class(calibration)
    session = OperatorSession()
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    output = Path(args.output_root) / timestamp
    env = None
    actions, counts_log, joints_log, frames, diagnostics = [], [], [], [], []
    success = False
    error = None
    instruction = ""
    window = f"GELLO HIL {args.suite} task={args.task_id} state={args.state_index}"
    ids = calibration.arm_ids + (calibration.gripper_id,)
    try:
        env, obs, instruction = make_env(
            args.suite,
            args.task_id,
            args.state_index,
            resolution=args.resolution,
            seed=args.seed,
        )
        with _bus(args) as bus:
            latest = None
            while not session.terminal:
                counts = bus.read_positions(ids)
                current = follower_joints(env)
                target = calibration.mapped_joints(counts)
                errors = target - current
                frame = rgb_frame(obs)
                notice = (
                    "Ready screen: simulator is frozen; Enter starts." if not session.active else ""
                )
                canvas = operator_panel(
                    frame,
                    instruction=instruction,
                    status=session.status,
                    active_steps=session.active_step_count,
                    max_steps=args.max_active_steps,
                    mapping_mode=args.mapping,
                    joints=target,
                    errors=errors,
                    notice=notice,
                    scale=args.window_scale,
                )
                cv2.imshow(window, canvas)
                command = decode_key(cv2.waitKey(1))
                if command == "abort":
                    session.abort()
                    break
                if command == "pause" and session.active:
                    session.pause(
                        counts=[counts[i] for i in ids],
                        current_follower_joint_positions=current.tolist(),
                    )
                    continue
                if command == "resume" and not session.active:
                    anchor = controller.anchor(counts, current)
                    session.resume(
                        anchor_counts=anchor["counts"],
                        follower_joints=anchor["follower_joints"],
                        leader_joints=anchor["leader_joints"],
                        mode=controller.mode,
                    )
                    continue  # Enter never consumes an action or simulator step.
                if not session.active:
                    continue
                try:
                    action, latest = controller.action(counts, current)
                except TargetDiscontinuity as exc:
                    session.pause(**exc.diagnostic)
                    continue
                obs, reward, _done, _info = env.step(action.tolist())
                session.record_active_step()
                actions.append(action)
                counts_log.append([counts[i] for i in ids])
                joints_log.append(current)
                diagnostics.append(latest)
                frames.append(frame)
                success = bool(reward > 0 or env.check_success())
                if success:
                    session.finish("success")
                elif session.active_step_count >= args.max_active_steps:
                    session.finish("active_horizon_exhausted")
    except Exception as exc:  # noqa: BLE001 - every failed hardware attempt must be saved
        error = f"{type(exc).__name__}: {exc}"
        if not session.terminal:
            session.finish("runtime_error")
    finally:
        close_window(window)
        if env is not None:
            env.close()
        metadata = {
            "schema_version": "gello-hil-libero-attempt-v1",
            "created_at_utc": timestamp,
            "suite": args.suite,
            "task_id": args.task_id,
            "state_index": args.state_index,
            "instruction": instruction,
            "mapping_mode": args.mapping,
            "success": success,
            "aborted": session.aborted,
            "error": error,
            "session": session.summary(),
            "diagnostics": diagnostics,
            "hardware_contract": {
                "allowed_instructions": ["PING", "READ", "SYNC_READ"],
                "motor_register_writes": 0,
            },
        }
        _save_attempt(
            output,
            metadata=metadata,
            actions=actions,
            counts=counts_log,
            joints=joints_log,
            frames=frames,
        )
        print(f"Saved attempt: {output.resolve()}")
        print(
            f"success={success} aborted={session.aborted} active_steps={session.active_step_count}"
        )
        if error:
            print(error)
    return 0 if success else 1


def _hardware_arguments(parser) -> None:
    parser.add_argument("--port", required=True, help="stable /dev/serial/by-id/... path")
    parser.add_argument("--baudrate", type=int, default=57600, choices=(57600, 115200))
    parser.add_argument("--serial-timeout", type=float, default=0.12)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="gello-hil")
    sub = parser.add_subparsers(dest="command", required=True)
    probe_parser = sub.add_parser("probe", help="read-only PING and Present Position check")
    _hardware_arguments(probe_parser)
    probe_parser.add_argument("--arm-ids", type=int, nargs=7, default=range(1, 8))
    probe_parser.add_argument("--gripper-id", type=int, default=8)
    probe_parser.set_defaults(func=probe)

    cal = sub.add_parser("calibrate", help="capture launch-time quarter-turn calibration")
    _hardware_arguments(cal)
    cal.add_argument("--arm-ids", type=int, nargs=7, default=range(1, 8))
    cal.add_argument("--gripper-id", type=int, default=8)
    cal.add_argument("--joint-signs", type=int, nargs=7, default=DEFAULT_SIGNS)
    cal.add_argument("--samples", type=int, default=25)
    cal.add_argument("--sample-interval", type=float, default=0.02)
    cal.add_argument("--max-residual", type=float, default=0.60)
    cal.add_argument("--output", default="calibration.json")
    cal.add_argument("--overwrite", action="store_true")
    cal.set_defaults(func=calibrate)

    mapping = sub.add_parser("mapping-check", help="read-only joint mapping display")
    _hardware_arguments(mapping)
    mapping.add_argument("--calibration", default="calibration.json")
    mapping.add_argument("--resolution", type=int, default=256)
    mapping.add_argument("--window-scale", type=float, default=2.5)
    mapping.add_argument("--suite", default="libero_object")
    mapping.add_argument("--task-id", type=int, default=0)
    mapping.add_argument("--state-index", type=int, default=0)
    mapping.add_argument("--seed", type=int, default=0)
    mapping.add_argument(
        "--libero-config-path",
        help="directory containing LIBERO config.yaml; equivalent to LIBERO_CONFIG_PATH",
    )
    mapping.set_defaults(func=mapping_check)

    run = sub.add_parser("collect", help="collect one LIBERO HIL attempt")
    _hardware_arguments(run)
    run.add_argument("--calibration", default="calibration.json")
    run.add_argument("--suite", default="libero_object")
    run.add_argument("--task-id", type=int, required=True)
    run.add_argument("--state-index", type=int, default=0)
    run.add_argument("--seed", type=int, default=0)
    run.add_argument("--mapping", choices=("absolute", "relative-clutch"), default="absolute")
    run.add_argument("--resolution", type=int, default=256)
    run.add_argument("--window-scale", type=float, default=2.5)
    run.add_argument("--max-active-steps", type=int, default=1000)
    run.add_argument("--output-root", default="runs")
    run.add_argument(
        "--libero-config-path",
        help="directory containing LIBERO config.yaml; equivalent to LIBERO_CONFIG_PATH",
    )
    run.set_defaults(func=collect)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
