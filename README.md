# Gello-HIL-franka-LIBERO

Human-in-the-loop joint teleoperation for a Franka Panda in
[LIBERO](https://github.com/Lifelong-Robot-Learning/LIBERO), using a passive
[GELLO](https://github.com/wuphilipp/gello_software) as the leader device.

[한국어 사용법](docs/usage.ko.md) · [Safety](SAFETY.md) ·
[Architecture](docs/architecture.md) · [Attribution](THIRD_PARTY_NOTICES.md)

> **Default `collect` is passive and read-only.** Version 0.2 also includes an
> **experimental, opt-in** `assisted-collect --apply` command that can move real
> GELLO arm motors during setup. Read [the assisted guide](docs/assisted.md) and
> [SAFETY.md](SAFETY.md) first. The current A6 live-release revision is
> **unvalidated: further tests and physical execution were explicitly deferred**.
> Older A5 tests do not certify A6. Autonomous homing/private failure states are excluded.

## Start here

First use: **install → probe → calibrate → mapping check → collect**.
Later sessions: probe again, preferably capture fresh calibration, check mapping
after any assembly change, then collect. Each collection command saves a new
attempt automatically; there is no output JSON to copy into the next command.

| Goal | Where to go |
| --- | --- |
| Install and locate GELLO | [Setup](#setup) and [serial device](#1-find-the-serial-device) |
| Fix joint direction or a constant angular offset | [Calibration](#3-calibrate) and [mapping check](#4-mapping-check) |
| Start a demonstration | [Collect](#5-collect-one-hil-attempt) |
| Use bounded motor assistance to align first | [Assisted collection](#6-optional-motor-assisted-alignment) |
| Pause, rest, or re-anchor | [Keys and mapping modes](#keys) |
| Repeat an attempt or inspect a result | [Repeat a session](#repeat-a-session) and [artifacts](#artifacts) |
| Connection, calibration, or display problem | [Troubleshooting](#troubleshooting) |

## What is included

- Strictly read-only Dynamixel input: only Protocol 2.0 `PING`, `READ`, and
  `SYNC_READ` instructions can be constructed.
- Official-style GELLO calibration: joint signs plus discrete multiples of
  `pi/2`, normalized around the asymmetric Panda joint ranges.
- A rendered, source-independent mapping check. Moving GELLO changes a Panda
  preview but does not step the simulation.
- Direct Franka `JOINT_POSITION` teleoperation in LIBERO.
- Operator-owned start and clutch semantics: the session starts paused,
  `Enter` arms/re-anchors, `Space` pauses, and `q` aborts.
- A 1000 **active-step** default horizon. Paused wall-clock time never consumes
  the action budget and never steps MuJoCo.
- Automatic success termination and complete attempt artifacts, including
  aborted, failed, and zero-action attempts.
- Two mapping modes:
  - `absolute` follows the current calibrated GELLO joint pose in Panda
    coordinates. Use this after physical pose alignment.
  - `relative-clutch` maps motion from the latest `Enter` anchor onto the
    current simulated pose. Re-anchoring produces a zero arm action.

This is a standalone interaction-layer release, not an official GELLO
reproduction. It contains no experiment logs, private failure states, model
checkpoints, workstation paths, or hidden evaluation data.

## Setup

### Requirements

- A seven-joint GELLO plus a passive gripper servo, normally IDs
  `1..7` and `8`.
- A stable serial path under `/dev/serial/by-id/`.
- Python 3.10, NumPy `<2`, robosuite 1.4.x, and a working LIBERO installation.
- MuJoCo off-screen rendering (EGL is recommended on a GPU workstation).
- A desktop/display for the OpenCV operator window. Off-screen MuJoCo rendering
  does not remove the need for a GUI display during interactive collection.
- Correctly assembled and externally supported GELLO; use the power and wiring
  specified for your hardware. Read [SAFETY.md](SAFETY.md) before connecting it.

The CLI currently reads all eight IDs, including the gripper. Its examples
assume IDs `1..7` for the arm and `8` for the gripper; a missing device is not
silently ignored.

### Install

Follow the official installation instructions for
[LIBERO](https://github.com/Lifelong-Robot-Learning/LIBERO) and
[robosuite](https://github.com/ARISE-Initiative/robosuite). Then install this
package in that same environment:

```bash
git clone https://github.com/paragon7060/Gello-HIL-franka-LIBERO.git
cd Gello-HIL-franka-LIBERO
python -m pip install -e .
```

Make sure LIBERO's `config.yaml` points to existing `assets`, `bddl_files`, and
`init_states` directories. Either export its parent directory:

```bash
export LIBERO_CONFIG_PATH=/absolute/path/to/libero-config-directory
```

or pass `--libero-config-path` to `mapping-check` and `collect`.

## 1. Find the serial device

```bash
ls -l /dev/serial/by-id/
```

Use the complete stable path in every command below:

```bash
export GELLO_PORT=/dev/serial/by-id/usb-FTDI_REPLACE_ME-if00-port0
```

If the device is permission-denied, add the user to the system's serial-access
group (often `dialout`), then log out and back in. Do not run the collector as
root just to bypass serial permissions.

## 2. Read-only hardware probe

```bash
gello-hil probe --port "$GELLO_PORT"
```

This pings IDs 1 through 8 and reads their Present Position values. It sends no
register writes and does not enable torque.

Proceed only when every expected ID responds. A missing device or timeout is a
connection check failure, not a calibration problem.

## 3. Calibrate

Calibration must be repeated whenever servo assembly, horn orientation, motor
IDs, or joint signs change. For the cleanest provenance, repeat it at the start
of each collection session.

```bash
gello-hil calibrate \
  --port "$GELLO_PORT" \
  --joint-signs 1 1 1 -1 1 -1 1 \
  --output calibration.json
```

At step 1, match each physical GELLO joint to this Panda reference:

```text
J1=0, J2=0, J3=0, J4=-90 deg, J5=0, J6=+90 deg, J7=0
```

The public calibration command uses **terminal prompts**, not a rendered
calibration wizard. Match the joints individually, not only the end-effector or
arm silhouette. Then press `Enter`, capture gripper-open, and capture
gripper-closed without moving the arm. The code
tests all four quarter-turn branches per joint and rejects a capture whose
residual is larger than the configured threshold.

A small residual is a consistency check, not proof that the physical reference
pose was correct. Recheck individual joints if the later preview disagrees.

Joint signs depend on motor installation. The example signs are the assembly
used to validate this release; they are not universal. If one rendered joint
moves in exactly the opposite direction, invert only that joint's sign and
recalibrate.

If `calibration.json` already exists, the command asks in English before
overwriting it. Add `--overwrite` only when replacement without a prompt is
intentional.

## 4. Mapping check

```bash
gello-hil mapping-check \
  --port "$GELLO_PORT" \
  --calibration calibration.json \
  --libero-config-path "$LIBERO_CONFIG_PATH" \
  --suite libero_object \
  --task-id 0
```

This opens a large Panda preview. Move one GELLO joint at a time and verify the
same joint and direction on screen. The display reports mapped joint values and
reference-pose error in radians. It modifies render-state joint positions only;
it issues no control action and calls no simulator step. Press `q` to exit.

Unlike collection, this preview updates immediately as GELLO moves; no Enter is
needed. It does not create a pass/fail attestation artifact. The displayed error
is relative to the fixed calibration reference, not a requirement to keep every
joint at zero error while inspecting motion.

## 5. Collect one HIL attempt

`--suite`, `--task-id`, and `--state-index` select an ordinary LIBERO initial
state. Task IDs are suite-specific: check the natural-language instruction in
the window rather than assuming that a number always names the same object.
This public command does not restore a private policy-failure checkpoint.

Absolute mapping, matching the current calibrated GELLO pose:

```bash
gello-hil collect \
  --port "$GELLO_PORT" \
  --calibration calibration.json \
  --libero-config-path "$LIBERO_CONFIG_PATH" \
  --suite libero_object \
  --task-id 4 \
  --state-index 0 \
  --mapping absolute \
  --max-active-steps 1000 \
  --output-root runs
```

Ergonomic clutch-relative mapping, which starts from the current simulated
pose even if the passive GELLO pose is different:

```bash
gello-hil collect \
  --port "$GELLO_PORT" \
  --calibration calibration.json \
  --libero-config-path "$LIBERO_CONFIG_PATH" \
  --suite libero_object \
  --task-id 4 \
  --mapping relative-clutch
```

The ready screen is always `PAUSED`. No action, active-step budget, or MuJoCo
step is consumed until the operator presses `Enter`.

### Keys

| Key | PAUSED | ACTIVE |
| --- | --- | --- |
| `Enter` | Capture current GELLO counts and current follower joints; resume | No effect |
| `Space` | No effect | Clutch/pause before issuing another action |
| `q` | Abort and save | Abort and save before issuing another action |

In `relative-clutch` mode, the first arm action after every `Enter` is zero. In
`absolute` mode, `Enter` records the current calibrated target rather than
rebasing it; align the physical GELLO to the simulated Panda before resuming.

LIBERO success ends the attempt automatically. Otherwise it ends at 1000
active steps, a runtime error, or when `q` is pressed. There is no post-success
review/recollect key or automatic advance to another task in the public CLI.

### Repeat a session

Run the same `gello-hil collect ...` command again to retry the same task/state.
Every attempt gets its own timestamped directory, including failures and
aborts. Change `--task-id` or `--state-index` explicitly for another episode.

For a fresh calibration, run the same calibration command and answer `y` to
the English overwrite prompt. To preserve separate calibration files, pass a
new `--output` name and use that same name with `--calibration` in both later
commands. Calibration is **not automatically repeated** by `collect`.

To enlarge the visualized environment without increasing render resolution,
add `--window-scale 3 --resolution 256` to `mapping-check` or `collect`.

## Artifacts

Each launch creates a fresh UTC directory under `runs/`:

```text
runs/<UTC timestamp>/
  attempt.json
  trajectory.npz
  operator-view.mp4
```

`attempt.json` includes task identity, instruction, outcome, error, mapping
mode, pause/re-anchor event provenance, active and paused time, pause count, and
per-step target/follower diagnostics. `trajectory.npz` stores actions, raw
encoder counts, and follower joints without Python pickles. Every outcome is
preserved; the collector never overwrites a previous attempt.

Check `success`, `aborted`, `error`, and `session` in `attempt.json`; the presence
of a saved directory alone does not mean the task succeeded. The command exits
with code `0` for success and `1` otherwise. A zero-action attempt need not have
a video. Recorded actions have eight components: seven joint commands and one
gripper command; they are not seven-dimensional OSC end-effector actions.

## 6. Optional motor-assisted alignment (A6, unvalidated)

After calibration and mapping verification, the assisted command helps you
manually bring GELLO near the **ordinary LIBERO initial pose**, then hands the
same scene/window to Enter-anchored joint teleoperation:

```bash
gello-hil assisted-collect \
  --port "$GELLO_PORT" \
  --calibration calibration.json \
  --libero-config-path "$LIBERO_CONFIG_PATH" \
  --suite libero_object --task-id 4 --state-index 0 \
  --max-active-steps 1000 \
  --apply
```

This command is restricted to the registered XL330 setup (arm model `1200`,
gripper model `1190`, `57600` baud). It checks identities and existing mode-5,
torque-off, power/temperature and startup state before arming. It does not
automatically convert an incompatible device's operating mode.

- READY allows **0.60 rad maximum and RMS joint error**, with the existing
  stationary dwell checks. Directional assistance remains capped at 100 mA,
  P=750, and a 20-count waypoint; this is not autonomous full-pose homing.
- At READY, **Enter starts control immediately and reduces existing force over
  2 seconds while you control**. There is no release-then-wait gap or second Enter.
  Current counts and the frozen follower pose define the start offset; the first
  action is neutral. Each later clutch Enter re-anchors the same way.
- Setup stays frozen, but live-release actions consume the normal ACTIVE budget
  and are explicitly labeled motor-assisted. Space releases/pauses; q releases/aborts.
- Already-near passive starts do not turn motors on just to show a countdown.
- All outcomes are saved under `runs/assisted-a6/<UTC>/`; `attempt.json` includes
  alignment telemetry, timings, safety/cleanup results, and handoff events.

Support the physical arm throughout and keep the power disconnect reachable.
See [the full guide and troubleshooting](docs/assisted.md). The base collection
success/retry behavior is unchanged; calibration remains a separate CLI step.
The unchanged 40-count tracking cap can cause an early verified release if you
backdrive during the fade. Other hardware safety failures abort; no force cap
is increased to make alignment easier.

## Troubleshooting

- **`/dev/serial` does not exist:** connect the USB adapter and inspect the
  serial-device directory again; do not use another machine's saved device path.
- **`SYNC_READ missing IDs [1, ..., 8]`:** the serial port opened but no expected
  servos replied. Check hardware power, data-chain connections, configured IDs
  and baudrate, and whether another program owns the port. Retry the read-only
  probe before calibration; do not bypass the check or enable torque to fix it.
- **A joint moves backward:** invert that one entry in `--joint-signs`, then
  recalibrate and repeat mapping check.
- **A joint has a roughly constant 90°/180° offset:** suspect reference-pose or
  assembly-offset calibration before changing its sign. Return every joint to
  the reference and recalibrate. A sign error reverses the *direction of change*;
  a constant offset alone does not establish a sign error.
- **`Numba needs NumPy 2.0 or less`:** activate the intended LIBERO environment,
  ensure `numpy<2`, and prevent an incompatible user-site package from taking
  precedence (for example, `PYTHONNOUSERSITE=1`).
- **The viewer is small:** use `--window-scale 3 --resolution 256`. This scales
  the camera image itself, not just the window border.
- **It stops after success:** expected; rerun `collect` for another attempt.
- **It stops without success:** inspect `error` and the terminal session event in
  `attempt.json`. The active-step horizon and an abort also end the attempt.
- **Pause still advances:** file an issue with a minimal reproduction. The unit
  contract forbids action-budget or simulator-step consumption while paused.

## Development

```bash
python -m pip install -e '.[dev]'
python -m pytest
python -m ruff check src tests
```

These unit checks do not validate your physical assembly, power supply, or
mapping direction. Follow [CONTRIBUTING.md](CONTRIBUTING.md) for changes and
keep generated runs, calibration captures, and hardware identifiers out of Git.

## Attribution

The calibration convention follows the official GELLO software, and the
simulation integration targets LIBERO/robosuite. See
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and cite the upstream GELLO
and LIBERO work when using this repository in research.
