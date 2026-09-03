# Safety

This software controls a simulated Franka Panda. Its default GELLO transport is
read-only, but hardware mistakes can still damage electronics or create pinch
hazards.

## Default boundary

`gello_hil.dynamixel` can construct only Dynamixel Protocol 2.0 `PING`, `READ`,
and `SYNC_READ` instructions. It has no API for torque, goal position, current,
gain, mode, ID, baud-rate, or other register writes. Unit tests reject common
write-class opcodes.

That boundary is unchanged in v0.2. The optional `gello_hil.actuation` transport
is separate and reachable through `assisted-collect --apply` only after an
interactive Enter authorization. Default `collect`, `probe`, `calibrate`, and
`mapping-check` never enable it.

## Experimental motor-assisted setup

The v0.2 assist is a project-local GELLO-ALIGN-A6 style port, **not an official
GELLO reproduction or physical safety certification**. Validate the exact servo
model, assembly, power supply, and serial timing on your hardware. All seven
arm joints can move together. Human support does not eliminate pinch hazards.

- Supported expected models: arm 1200 (XL330-M077), gripper 1190 (XL330-M288).
  The operator must verify this matches their hardware; mismatches fail closed.
- Requires existing current-based position mode 5 and torque off; no automatic
  EEPROM/mode migration, Current Limit change, or gain/current escalation CLI.
- Goal Current <=100 mA, monitored current <=150 mA; P<=750, I=0, D=150 by
  default; guide goal at most 20 encoder counts from Present Position.
- Goal-present tracking cap 40 counts, arm velocity cap 1.5 rad/s, input
  voltage 4.5–5.5 V, temperature <=45°C and rise <=10°C.
- 5 Hz monitoring, 400 ms bus watchdog, all-ID torque reads, bounded arm
  telemetry reads, zero-force arming latch, and verified disarm are retained.
- READY requires maximum/RMS pose error <=0.60/0.60 rad, velocity <=1 raw unit,
  and five stable samples. The wider pose tolerance is not a motor-safety cap.
- READY Enter starts anchored control and a nominal two-second downward current
  ramp concurrently. There is no post-release alignment gate or second Enter.
  The first action is neutral; calibrated motion plus the Enter offset follows.
- Setup/pause consume no simulation/action budget; the live-release prefix DOES
  consume budget and is labeled motor-assisted. Current never increases, goals
  and gains are not advanced during release. One writer owns the serial bus until
  verified torque-off and closes before the reader opens. Gripper stays off.
- Backdrive past the unchanged 40-count tracking cap during the release prefix
  causes early torque-off, not stronger resistance or a widened tracking cap.
  Other safety faults abort. A slow UI/simulator may still trip the watchdog.
- Space requests release/pause; q requests release/abort between bounded bus
  transactions. They are **not hard real-time emergency stops**. If cleanup
  cannot verify torque-off or anything behaves unexpectedly, use the physical
  power disconnect immediately. Never rely on the GUI or watchdog alone.

The hold is not gravity compensation and may not support the arm under load.
Do not remove safety gates or increase force to make a failed gate pass.
See [docs/assisted.md](docs/assisted.md) for the complete operator sequence.

Current A6 code was written without additional tests, lint, compile, smoke, or
physical execution at the user's request. Prior A5 test results are historical,
not evidence that the new powered collection prefix is validated.

## Operator checklist

1. Confirm servo model IDs and the manufacturer-rated supply voltage.
2. Keep cables and fingers clear of all joints.
3. Use the stable `/dev/serial/by-id/` path and ensure only one process owns it.
4. Run `probe`, `calibrate`, and `mapping-check` before collection.
5. Start every collection on the PAUSED ready screen.
6. Use `Space` before resting or repositioning GELLO; use `q` on uncertainty.
7. Inspect the saved attempt even after success. Automatic success is the
   LIBERO task predicate, not a hardware safety judgment.

The software is provided without warranty. You are responsible for the
physical system and local risk assessment.
