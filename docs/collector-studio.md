# GELLO Studio

An English-language desktop collector for **LIBERO-Pro Position: 40 tasks × 10 demos**.
The Qt6 interface combines initial-scene preview, explicit motor-assist controls,
recording, video playback, attempt history and operator confirmation in one window.

The collector code and GUI have been tested. **Physical A6 assistance is still
unverified, and no real demonstrations were collected during GUI validation.**
Use the calibration for the current physical assembly and follow [SAFETY.md](../SAFETY.md).

## Install and open

Use Python 3.10 or newer in the same environment as LIBERO/robosuite 1.4.x:

```bash
python -m pip install -e '.[gui,export]'
export PYTHONNOUSERSITE=1
export PYTHONPATH="$PWD/src"
export NUMBA_CACHE_DIR="$PWD/runs/numba-cache"
```

Prepare the pinned LIBERO-Pro source and official Position assets once, as described
in the [40-task setup guide](position40-pool10.ko.md). Then open the application:

```bash
python -m gello_hil.cli position-pool-gui \
  --dataset-root runs/position40-pool10 \
  --libero-source ../LIBERO-PRO \
  --libero-config-path runs/position40-assets/config \
  --apply
```

There is no per-task command or retry flag. Select tasks and shots inside the GUI.
The application resumes at the first unconfirmed slot. If exactly one stable serial
device is visible, its port is filled automatically; otherwise enter its path in
**Connection**. Baud rate is fixed to 57600, with arm IDs 1–7 and gripper ID 8.

Without a calibration file, the initial scene opens without any serial communication.
Choose **Calibration...** to load a verified calibration, or supply
`--calibration /path/to/calibration.json` at launch. Device settings are locked while
the encoder/alignment/collection worker is open; stop that scene before changing them.

Omit `--apply` to disable motor and collection commands. Reviewing videos and
confirming existing candidates remains available. Merely opening the window never
starts motor assistance or records a demonstration.

## Screen and controls

| Area | Purpose |
| --- | --- |
| Task library | Search all 40 tasks; inspect confirmed counts and total attempts |
| Demonstration slots | Inspect shots 1–10, their registered init states and status |
| Main viewer | See the initial/live agent camera or play a recorded attempt |
| Attempt history | Browse every success, failure, abort and accepted selection |
| Progress | Count confirmed demonstrations toward 400 |

| Button | Action |
| --- | --- |
| **Open scene** | Load the selected task's registered initial state |
| **Motor assist** | Request the original bounded A6 alignment procedure |
| **Start collection** | Start from an aligned pose, or re-anchor and resume from pause |
| **Pause** | Release force and clutch; pause physics and the active-step budget |
| **Stop & review** | Release, end the attempt and preserve raw artifacts |
| **Play** / seek bar | Review the selected recording |
| **Retry shot** | Preserve the old attempt and reopen the same task/init for a new attempt |
| **Confirm & next** | Validate and accept the selected success, then select the next pending shot |
| **Next pending** | Navigate to an unconfirmed slot |

Typical flow:

1. Inspect the initial scene and instruction. Select the current assembly's calibration.
2. Support GELLO and click **Motor assist** if alignment is needed.
3. When ready, click **Start collection** and perform the demonstration.
4. Review the video after success or **Stop & review**.
5. Choose **Retry shot** to repeat, or **Confirm & next** to accept a successful recording.

If GELLO is already aligned, the existing passive-start path performs no motor writes.
**Motor assist** still waits for a separate **Start collection** click. After confirming,
only the next initial scene opens automatically; assistance and recording always wait
for the operator. Switching tasks during active recording is blocked until pause/stop.
Closing the app requests the worker's normal force-release and save path.

The original A6 current/gain/tracking/guide/watchdog limits and gripper torque-off
behavior remain in force. See the [assisted workflow](assisted.md) for the hardware
procedure and limitations. No actual motor validation is implied by simulated tests.

## Saving, retries and training selection

Every attempt is saved automatically as raw evidence, including failures and aborts.
**Confirmation** separately selects a successful attempt for training. Native success
alone does not increment the confirmed count. A candidate must also pass the raw-data,
native success-hold, calibration and provenance checks.

Failed or aborted attempts cannot be confirmed. Retry does not delete or overwrite
them. Retrying an already confirmed slot keeps the old selection until a replacement
is explicitly confirmed. Older raw files and confirmation events remain available.

```text
position40-pool10/
  pool-plan.json
  review-policy.json                   # explicit operator confirmation required
  attempts/<task>/shotNN/<timestamp>/
    attempt.json                       # result, instruction, events and source commits
    calibration.json
    trajectory.npz                     # T executed 7D actions, encoder/joints/time
    observations.npz                   # T+1 raw camera/state observations
    operator-view.mp4
  reviews/<task>/shotNN/<sequence>_<id>.json  # append-only, hash-linked decisions
  gui-sessions/<id>/                   # owned worker control and preview diagnostics
  pool-manifest.json                  # only after all 400 selections validate
```

The GUI never overwrites original attempt artifacts. Confirmations bind the raw-file
SHA256 values. Unconfirmed or superseded candidates cannot be exported for training.
Scene-only browsing before any assist/start request is kept under `gui-sessions` and
does not count as a demonstration attempt. Keep this private data outside Git.

After 400 successful confirmations, freeze the pool and generate nested subsets:

```bash
python -m gello_hil.cli freeze-position-pool --dataset-root runs/position40-pool10
python -m gello_hil.cli select-shots \
  --pool-manifest runs/position40-pool10/pool-manifest.json \
  --shots 1 3 5 10 --seeds 42 43 44 --output-dir runs/position40-subsets
```

Frozen pools support review but reject collection/replacement. Each seed gives
**1-shot ⊂ 3-shot ⊂ 5-shot ⊂ 10-shot**, shared by existing and new experiments.
The [data guide](position40-pool10.ko.md) describes export and the remaining OAT
few-shot loader integration. These commands do not launch training.

## Display troubleshooting

The application uses English labels and Qt6 text rendering. `gui` installs
`PySide6-Essentials`; QtWebEngine and a browser server are unnecessary. The app selects
the Qt6 plugin directory to avoid OpenCV's bundled Qt5 plugin conflict.

On the verified Linux desktop, CPU Mesa scene rendering worked with:

```bash
export MUJOCO_GL=glx
export PYOPENGL_PLATFORM=glx
export LIBGL_ALWAYS_SOFTWARE=1
export __GLX_VENDOR_LIBRARY_NAME=mesa
```

This requires a desktop display. In robosuite 1.4, other backend values can be forced
to EGL by its rendering setup. Select a working backend before launching the app;
do not mix `PYOPENGL_PLATFORM=glx` with an EGL context. No installed simulator source
needs editing. Worker errors are shown in the window and preserved in the session log.

Validation scope and local screenshot evidence are listed in the
[GUI validation report](../reports/REPA_VLA_GUI_VALIDATION_KO.md).
