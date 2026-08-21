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
is evaluated on the next iteration.

## Mapping modes

`absolute` maps current counts directly into Panda coordinates. Encoder wrap is
handled with modular count deltas, and a continuity residual catches a change
to a different `2*pi` normalization branch.

`relative-clutch` maps the signed encoder displacement from the latest anchor
onto the follower joints captured at that anchor. It is useful when the passive
leader cannot comfortably remain in the exact simulated pose.

## Provenance

Every session event records event type, active step index, monotonic elapsed
time, pause count, total paused time, active-control time, and anchor vectors
when applicable. Every attempt gets a new UTC directory and is saved even when
it aborts before the first action.
