# Experimental assisted collection

The default commands remain passive. This optional workflow adds bounded
human-guided motor assistance before collection and a short **live-release
prefix during collection**; no autonomous
homing, private failure states, or research campaign manager are included.

## Prerequisites

1. Read [SAFETY.md](../SAFETY.md). Verify the documented servo models, supply,
   existing mode 5, IDs, and wiring; keep the physical disconnect accessible.
2. Run the read-only `probe`, `calibrate`, and `mapping-check` commands in the
   [README](../README.md). Recalibrate after assembly/sign changes and preferably
   at each session. A small numeric residual cannot certify a wrong physical pose.
3. Use the same calibration file for assisted collection. The CLI does not
   automatically recalibrate or reuse a private experiment artifact.

## Run

```bash
gello-hil assisted-collect \
  --port "$GELLO_PORT" --calibration calibration.json \
  --libero-config-path "$LIBERO_CONFIG_PATH" \
  --suite libero_object --task-id 4 --state-index 0 \
  --mapping absolute --window-scale 3 --max-active-steps 1000 \
  --output-root runs/assisted-a6 --apply
```

Without `--apply` the command refuses before opening hardware. With it, an
English safety prompt still asks for Enter. Type q then Enter to cancel.
No config exposes higher current/gain or weaker safety limits.

## On-screen sequence

| Phase | What happens | Your control |
| --- | --- | --- |
| PAUSED | Read-only target-minus-GELLO error preview | Support GELLO, Enter to continue |
| ARMING SETTLE | Three stationary aggregate samples with torque off | Hold still; Space pauses; q aborts |
| ASSIST ACTIVE | Short, current-limited waypoint toward target | Guide the arm by hand; Space releases |
| ALIGNMENT READY | Max/RMS error <=0.60/0.60 rad with stable dwell; force retained | Enter starts control immediately |
| ACTIVE / LIVE RELEASE | Same scene/window, neutral first action; current decreases for 2 s while controlling | Support arm; Space releases/pauses; q aborts |
| Collection PAUSED | Torque-off verified; simulator/budget frozen | Enter re-anchors current counts/follower pose |
| Collection ACTIVE | Joint-space teleoperation | Space clutch, q abort/save |

An already-near pose (max/RMS <=0.60/0.60 rad) can pass directly with verified
torque off and **zero motor writes**. It skips assist and the powered transition.
That Enter also starts anchored control without a second confirmation.
The Enter event itself consumes no action; subsequent live-release actions count
toward the ACTIVE horizon. Every Enter fixes a session offset from actual GELLO
and follower joints, so the residual pose mismatch does not become a target jump.
There is no post-release pose gate. The physical arm still needs human support.

Success saves and ends this one attempt. Rerun the command for another attempt
or select a different suite/task/state explicitly. No result is overwritten.

## Saved provenance

`runs/assisted-a6/<UTC>/attempt.json` contains an `alignment` section with the A6
profile, target joints, counts/errors, monitor timings, all attempted register
writes and cleanup outcome, pause/abort reasons, and `live_handoff.events`.
The calibration path and SHA-256 are recorded. `trajectory.npz` and video refer
to simulation control including the labeled powered-release prefix. Per-action
`diagnostics.motor_assisted_transition` and Enter offsets separate that prefix
from passive data. Setup failures and aborts also save metadata
and an empty trajectory; zero-action attempts may not have video.

No screenshot/video of the setup is required for the software gate. For a
physical validation report, separately record the actual device/assembly and
whether the operator touched/supported it; offline tests do not establish that
motor force will hold a given arm safely.

## Troubleshooting

- **Missing IDs:** check supply, cable chain, port ownership, IDs and baud with
  read-only probe first. Do not enable motors to debug a failed connection.
- **Mode/model/startup rejected:** this command does not reconfigure devices to
  force compatibility. Check the hardware specification and manufacturer tools.
- **Settle or READY repeatedly lost:** support the linkage, slow down, and read
  the displayed per-joint target-minus-actual errors. Do not disable velocity or
  drift checks. Large constant angular offsets require calibration review.
- **Release ends early:** backdriving exceeded the existing tracking cap. It
  releases safely instead of resisting harder; inspect the recorded reason.
- **Physical arm drops:** support it before READY Enter. This is a gradual
  handoff, not gravity compensation. No time delay can make an unpowered arm stand.
- **Unverified shutdown:** physically disconnect motor power. Software cannot
  prove torque-off when communication has failed; do not just restart.

## Port boundary

Calibration follows GELLO's signed-count/discrete-quarter-turn convention. The
read/write transports, monitor, bounded guide, GUI, and 2 s handoff are local
implementations. The reusable safety lifecycle is preserved, while legacy
one-motor probes, standalone homing/force-ramp commands, experiment manifests,
logs, host paths and hidden state data are excluded. No LeRobot/RoboCopilot
runtime or schema is used. See [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md).

The current A6 revision has not been run through further tests or on hardware,
per the user's request to stop validation. Do not treat older A5 test results
as validation of this new overlapping-release data collection mode.
