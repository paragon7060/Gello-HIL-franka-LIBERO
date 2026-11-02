"""Opt-in setup factory for the separately write-capable alignment workflow."""

from dataclasses import asdict

from .actuation import GelloAlignmentAssistLimits
from .alignment import AlignmentRunner
from .live_handoff import LIVE_HANDOFF_CONTRACT


def make_alignment_runner(args, calibration):
    calibration.validate()
    if args.baudrate != 57600:
        raise ValueError("assisted-collect is registered for 57600 baud only")
    if args.window_scale < 3:
        raise ValueError("assisted-collect requires --window-scale >=3")
    limits = GelloAlignmentAssistLimits(
        profile="GELLO-ALIGN-A6",
        ready_max_abs_error_rad=0.60, ready_rms_error_rad=0.60,
    )
    config = {
        "protocol_version": "GELLO-HIL-ASSIST-v1",
        "max_actions_per_attempt": args.max_active_steps,
        "hardware": {
            "port": args.port, "baudrate": args.baudrate,
            "serial_timeout_seconds": args.serial_timeout,
            "expected_model_numbers": {
                **{str(i): 1200 for i in calibration.arm_ids},
                str(calibration.gripper_id): 1190,
            },
        },
        "alignment_limits": asdict(limits),
        "alignment_handoff": dict(LIVE_HANDOFF_CONTRACT),
        "near_pose_passive_start": {
            "max_abs_target_error_rad": 0.60, "rms_target_error_rad": 0.60,
        },
    }
    return AlignmentRunner(config=config, calibration=calibration,
                           display=getattr(args, "gui_display", None))

