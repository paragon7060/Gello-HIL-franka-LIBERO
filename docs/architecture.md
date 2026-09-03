# Architecture and contracts

## Data flow

```text
Dynamixel Present Position counts
  -> quarter-turn calibration + joint signs
  -> Panda-range joint targets
  -> normalized JOINT_POSITION delta action
  -> LIBERO / robosuite Panda
```

The hardware reader and simulator controller are separate modules. The reader
does not know about MuJoCo and cannot write motor registers. The pure controller
does not own a serial port and can be tested with synthetic counts.

## Pause contract

The event loop evaluates keys before an action. While PAUSED it may read current
encoder counts and render diagnostics, but it does not call `env.step`, append
an action, or increment `active_step_count`. `Enter` captures current leader and
follower anchors and immediately returns to the display loop; the first action
is evaluated on the next iteration. In the optional assisted command, READY
Enter also starts the downward current ramp; there is no second start key.

## Mapping modes

`absolute` maps current counts directly into Panda coordinates. Encoder wrap is
handled with modular count deltas, and a continuity residual catches a change
to a different `2*pi` normalization branch.

`relative-clutch` maps the signed encoder displacement from the latest anchor
onto the follower joints captured at that anchor. It is useful when the passive
leader cannot comfortably remain in the exact simulated pose.

`assisted-collect` uses an Enter-anchored mapping: calibrated absolute joints
plus a fixed offset captured from the current follower minus leader pose.
The first action is neutral. Each later clutch Enter replaces the offset.
This removes the initial pose jump; it is not gravity compensation, and later
physical sag is still motion unless the operator pauses and re-anchors.

## Optional powered transition

`actuation` contains a separate bounded motor-writing transport and human-guided
alignment state machine. `alignment` owns the attempt window during setup;
`live_handoff` transfers its existing writer to the collection loop at READY
Enter. That single owner services safety telemetry and reduces existing current
over two seconds while collection runs. After verified all-ID torque-off, it
closes the writer before opening the default read-only input bus. Space/q cause
release before pause/abort. No background torque keeper is used.

Setup and pause remain zero-action; the live-release prefix consumes action
budget and is explicitly tagged as motor-assisted. A6 is an unvalidated
experimental port; see [the assisted guide](assisted.md) and [safety](../SAFETY.md).

## Provenance

Every session event records event type, active step index, monotonic elapsed
time, pause count, total paused time, active-control time, and anchor vectors
when applicable. Every attempt gets a new UTC directory and is saved even when
it aborts before the first action. Assisted attempts additionally retain the
alignment/release event stream, safety limits, writer counters, Enter offset,
and per-action motor-assisted-transition flag.
