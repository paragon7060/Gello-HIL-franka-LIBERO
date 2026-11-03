# GELLO Studio

An English-language desktop collector for **LIBERO-Pro Position: 40 tasks × 10 demos**.
The Qt6 interface combines initial-scene preview, explicit motor-assist controls,
recording, task example videos, attempt history and operator confirmation in one window.

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
Choose **Load calibration...** to load a verified calibration, or supply
`--calibration /path/to/calibration.json` at launch. Device settings are locked while
the encoder/alignment/collection worker is open; stop that scene before changing them.
The loaded file's full path appears below Connection and can be copied. The last
valid selection is remembered privately in the pool's `gui-settings.json`.
An explicit `--calibration` overrides the remembered file. `GELLO_CALIBRATION` is an
optional local fallback when no file has been selected; no workstation path is built
into the shared source. Historical imported captures are labeled as such.

### Create a new calibration

Click **New calibration**. The current scene releases and closes before the wizard
opens; recording and motor assistance do not run during calibration. The wizard uses
the original read-only calibration method and the same 25 samples per capture:

1. Choose each joint's **+1 / -1 sign** for the actual assembly. Defaults are examples.
2. Match GELLO joint-by-joint to **J1=0°, J2=0°, J3=0°, J4=-90°, J5=0°, J6=+90°, J7=0°**.
   Hold still and click **Capture reference pose**.
3. Keep the arm at the reference pose, fully open the gripper, then click **Capture open gripper**.
4. Fully close the gripper, then click **Capture closed gripper**.
5. Click **Save & use calibration** and save a new JSON file. It is immediately
   connected to the collector and remembered for the next launch.

The suggested location is `<dataset-root>/calibrations/<UTC timestamp>.json`.
Existing files are never overwritten. **Restart captures** lets you repeat the procedure;
an invalid reference residual or gripper span stays at the current step. Errors close
the serial reader before allowing another capture. Closing during a read waits for
that finite read to finish. The numerical check does not replace physical sign/mapping
verification before motor assistance. Creation never sends a motor-register write.

The existing terminal method also remains available:

```bash
python -m gello_hil.cli calibrate --port "$GELLO_PORT" --baudrate 57600 \
  --joint-signs 1 1 1 -1 1 -1 1 --output runs/my-new-session/calibration.json
```

Follow the three Enter prompts, using the signs for your assembly. Load the output
with **Load calibration...**. Keep calibration files and local settings out of Git.

Omit `--apply` to disable motor and collection commands. Reviewing videos and
confirming existing candidates remains available. Merely opening the window never
starts motor assistance or records a demonstration.

## Screen and controls

### Watch the task before collecting

The right side of the viewer has a **Task example** tab. Select any task in the
library and its example poster changes automatically. Click **Play example**;
**Replay**, the timeline, **Loop**, and **0.5x / 1x / 1.5x** control only the example.
**Enlarge** opens a separate modeless player. Switching tasks also updates that
window. The **Wrist camera** tab remains available when the live worker supplies it.
Playback never starts/stops an attempt, sends a motor command, or changes a review
decision. The example keeps its upright camera view when the policy-view toggle
changes the collection display.

The reference is **one original LIBERO human demonstration (`demo_0`) per task**,
matched by exact suite and task name for all 40 Position tasks. It illustrates the
goal; **LIBERO-Pro Position layouts can differ**. These are 128×128 source recordings,
so enlarging helps viewing but cannot add detail. The persistent caption identifies
them as original demonstrations. Hover over it for source revision, episode and
file details. References are operator guidance, not collected GELLO support or query
episodes, and never enter the 400-demo pool or k-shot export.

Prepare the offline cache once, from the collector repository:

```bash
python -m pip install -e '.[gui,examples]'
PYTHONPATH=src python scripts/prepare_task_examples.py
```

The helper reads only `demo_0` image data and metadata from the
[official LIBERO dataset](https://huggingface.co/datasets/yifengzhu-hf/LIBERO-datasets/tree/f13aa24a3da8c43c7225569f28c562979fa0e35a),
at the pinned revision. HTTP Range is required and each task has a finite transfer
budget; a server that ignores Range cannot trigger a full dataset download. Existing
local HDF5 files can be reused with `--local-datasets /path/to/LIBERO/datasets`.
Local files must match the pinned catalog's complete SHA256; HTTPS Range reads pin
the source revision and size without claiming a full-file hash verification.
The helper checks exact HDF5 task identity and recorded terminal done/reward, converts
OpenGL bottom-up images with a vertical flip, and writes MP4s plus SHA256 provenance
to `runs/task-examples/`. It never creates simulator rollouts or opens GELLO. Completed
assets are reused; failures stay in the manifest and are printed without silently
selecting a different episode. Running it again explicitly prepares missing/invalid
assets. Keep the videos and manifest out of Git.

The normal GUI command automatically uses this directory; no per-task arguments are
needed. An alternate cache can be selected with `--task-examples /path/to/cache`.
Missing or mismatched assets disable only that example player and show a preparation
message; collection controls continue to work. There are no startup network requests.

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
| **Check joint mapping** | Read encoders and render the calibrated joint pose without collection or motor writes |
| **Motor assist** | Request the original bounded A6 alignment procedure |
| **Start collection** | Start from an aligned pose, or re-anchor and resume from pause |
| **Pause** | Release force and clutch; pause physics and the active-step budget |
| **Stop & review** | Release, end the attempt and preserve raw artifacts |
| **Play** / seek bar | Review the selected recording |
| **Retry shot** | Preserve the old attempt and reopen the same task/init for a new attempt |
| **Confirm & next** | Validate and accept the selected success, then select the next pending shot |
| **Next pending** | Navigate to an unconfirmed slot |

Keyboard shortcuts (or a foot pedal mapped to the same keys) keep both hands on GELLO:
**Enter** = Start collection, **Space** = Pause, **M** = Motor assist, **Esc** = Stop & review.
Only Esc works while a text field has focus. During collection the review panel shows
active steps and budget, the success hold (n/20), gripper open/closed, the largest joint
tracking error and the measured loop rate. The wrist camera appears beside the scene
for the operator only; saved observations and videos are unchanged.

Typical flow:

1. Inspect the initial scene and instruction. Select the current assembly's calibration.
2. Support GELLO and click **Motor assist** if alignment is needed.
3. When ready, click **Start collection** and perform the demonstration.
4. Review the video after success or **Stop & review**.
5. Choose **Retry shot** to repeat, or **Confirm & next** to accept a successful recording.

If GELLO is already aligned, the existing passive-start path performs no motor writes.
**Motor assist** still waits for a separate **Start collection** click. After confirming,
only the next initial scene opens automatically; assistance and recording always wait
for the operator. Switching tasks during active recording is blocked until pause/stop. Leaving a paused
attempt that already recorded steps asks first; it is then saved as aborted. **Retry shot**
is disabled while a scene is running. **Start collection** never arms motor assistance:
if the pose drifted out of the near-pose gate when Start is pressed, the scene stays
paused and asks for alignment or **Motor assist**. The near-pose and A6 READY gates are
0.15 rad max / 0.10 rad RMS per joint, because the start offset persists for the whole
demonstration under the Enter-anchored mapping. If the GUI process dies, the worker
detects it and takes the normal abort path (release force, save).

Encoder read faults during collection are retried twice and then the port is reopened.
If the link stays down, the attempt pauses instead of ending (`encoder_read_faults`);
fix USB/power and press **Start collection** to re-anchor and continue. Simulator steps
are paced at the native 20 Hz control rate. `long_*` tasks use `--long-max-active-steps`
(default 1500); other tasks use `--max-active-steps` (default 1000).
Closing the app requests the worker's normal force-release and save path.

The original A6 current/gain/tracking/guide/watchdog limits and gripper torque-off
behavior remain in force. See the [assisted workflow](assisted.md) for the hardware
procedure and limitations. No actual motor validation is implied by simulated tests.

### Left/right motion and joint signs

The viewer now defaults to **Camera view (unmirrored)**. Previously the operator
display reused OpenVLA's policy preprocessing, which rotates raw pixels 180 degrees
and reverses their horizontal direction. **Policy view (180 deg)** remains available
for comparison. This selector changes only what the operator sees: raw observations,
saved actions, the video pixel convention, and export preprocessing stay unchanged.
The initial display mode and changes during collection are recorded in the GUI events.

The joint equation remains the official GELLO convention,
`q = (encoder_radians - assembly_offset) * joint_sign`, with equivalent 2π branches.
No global action-axis inversion or unverified joint-sign changes are applied.
The physical assembly still determines each sign; a single reference pose cannot
determine a joint's direction from motion.

Before collection, load/create a calibration and click **Check joint mapping**:

1. Move one physical joint at a time and compare the rendered joint and displayed J1–J7 angles.
2. Check J1/J3/J5 particularly if sideways motions appear reversed; confirm every joint,
   rather than changing a group of signs solely from end-effector motion.
3. If one joint moves in the wrong angular direction, select the correct sign in
   **New calibration**, repeat all three captures, save a new file and check again.
4. Press **Open scene** to return to the registered scene, then explicitly request
   assistance/start. The mapping preview itself cannot arm assistance or collect actions.

Only the registered scene's ten initialization warmup steps run before mapping preview;
subsequent pose rendering advances no physics. A numerical official-source/FK comparison
is documented in the [calibration and coordinate audit](../reports/CALIBRATION_MAPPING_VALIDATION_KO.md).

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
the validated **PySide6-Essentials 6.8.3** with matching Shiboken; QtWebEngine and a browser server are unnecessary. The app selects
the Qt6 plugin directory to avoid OpenCV's bundled Qt5 plugin conflict.

### `Fatal Python error: none_dealloc: deallocating None`

PySide6 6.12.0 was observed to decrement `None` once per void Qt call on the
collector's Python 3.10 runtime. Repeated UI updates exhausted its reference count
and aborted the GUI; Stop could coincide with this cumulative failure. This is a
Qt Python binding failure, rather than a recoverable collection exception.

The GUI dependency is now pinned to 6.8.3. At startup, the app rejects the known
unsafe version on Python 3.10/3.11, rejects mixed PySide/Shiboken versions, and
checks that actual Qt calls do not consume `None` references. These checks run
before any scene worker or serial connection is opened.

On the configured workstation, the existing launcher was switched to a separate
6.8.3 runtime directory. Restart with the same launch command. For another checkout,
install the matched runtime through the GUI extra:

```bash
python -m pip install -e '.[gui,export]'
```

The fix was checked with 100,000 `setEnabled` calls and 100,000 `setText` calls,
plus 20 real GUI Stop/review transitions using finite synthetic workers and 4,000
UI polls. [Crash repair evidence](../reports/QT_NONE_DEALLOC_REPAIR_20261103_KO.md).

Render on the GPU. On the RTX 4090 desktop a simulator step took about 10 ms with
`MUJOCO_GL=glx` (NVIDIA) or `egl`, versus about 92 ms with CPU Mesa
(`LIBGL_ALWAYS_SOFTWARE=1`, `__GLX_VENDOR_LIBRARY_NAME=mesa`), which held the control
loop near 7 Hz. Use CPU Mesa only on a desktop without a working GPU driver.
Set the FTDI latency timer to 1 ms (`echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`,
or a udev rule) so each SYNC_READ does not wait for the 16 ms default.

GLX requires a desktop display. In robosuite 1.4, other backend values can be forced
to EGL by its rendering setup. Select a working backend before launching the app;
do not mix `PYOPENGL_PLATFORM=glx` with an EGL context. No installed simulator source
needs editing. Worker errors are shown in the window and preserved in the session log.

Validation scope and local screenshot evidence are listed in the
[GUI validation report](../reports/REPA_VLA_GUI_VALIDATION_KO.md).
