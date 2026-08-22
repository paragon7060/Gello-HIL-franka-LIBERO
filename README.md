# Gello-HIL-franka-LIBERO

Human-in-the-loop joint teleoperation for a Franka Panda in
[LIBERO](https://github.com/Lifelong-Robot-Learning/LIBERO), using a passive
[GELLO](https://github.com/wuphilipp/gello_software) as the leader device.

This repository is a cleaned, public release of the reusable GELLO/LIBERO
interaction layer. It contains no experiment logs, private failure states,
model checkpoints, absolute workstation paths, or hidden evaluation data.

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

The public v0.1 release intentionally does **not** enable GELLO motor torque or
automatic homing. Motorized alignment needs a separate, hardware-specific
safety review. See [SAFETY.md](SAFETY.md).

## Hardware and software

- A seven-joint GELLO plus an optional passive gripper servo, normally IDs
  `1..7` and `8`.
- A stable serial path under `/dev/serial/by-id/`.
- Python 3.10, NumPy `<2`, robosuite 1.4.x, and a working LIBERO installation.
- MuJoCo off-screen rendering (EGL is recommended on a GPU workstation).

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

For development:

```bash
python -m pip install -e '.[dev]'
python -m pytest
python -m ruff check src tests
```

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

Match the joints individually, not only the end-effector or arm silhouette.
Then press `Enter`, capture gripper-open, and capture gripper-closed. The code
tests all four quarter-turn branches per joint and rejects a capture whose
residual is larger than the configured threshold.

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

## 5. Collect one HIL attempt

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
active steps or when `q` is pressed.

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

## Troubleshooting

- **A joint moves backward:** invert that one entry in `--joint-signs`, then
  recalibrate and repeat mapping check.
- **J4 or J6 differs by about pi radians:** the wrong quarter-turn branch was
  captured. Return every joint to the exact reference pose and recalibrate.
- **`Numba needs NumPy 2.0 or less`:** activate the intended LIBERO environment,
  ensure `numpy<2`, and prevent an incompatible user-site package from taking
  precedence (for example, `PYTHONNOUSERSITE=1`).
- **The viewer is small:** raise `--window-scale`; simulation render resolution
  can stay at 256.
- **Pause still advances:** file an issue with a minimal reproduction. The unit
  contract forbids action-budget or simulator-step consumption while paused.

Korean usage notes are in [docs/usage.ko.md](docs/usage.ko.md). Architecture and
data contracts are in [docs/architecture.md](docs/architecture.md).

## Attribution

The calibration convention follows the official GELLO software, and the
simulation integration targets LIBERO/robosuite. See
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and cite the upstream GELLO
and LIBERO work when using this repository in research.
