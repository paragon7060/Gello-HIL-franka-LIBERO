# Safety

This software controls a simulated Franka Panda. Its default GELLO transport is
read-only, but hardware mistakes can still damage electronics or create pinch
hazards.

## Default boundary

`gello_hil.dynamixel` can construct only Dynamixel Protocol 2.0 `PING`, `READ`,
and `SYNC_READ` instructions. It has no API for torque, goal position, current,
gain, mode, ID, baud-rate, or other register writes. Unit tests reject common
write-class opcodes.

Do not weaken that boundary to add motor holding or homing. Active GELLO motor
assistance needs a separate transport, explicit current/temperature/voltage and
watchdog limits, a reachable physical power disconnect, pinch-point controls,
and validation for the exact servo model, assembly, and power supply.

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
