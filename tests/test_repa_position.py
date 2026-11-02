from pathlib import Path
from types import SimpleNamespace
import json

import numpy as np
import pytest

from gello_hil import cli
from gello_hil.pose_control import PoseFollower, matrix_rotvec
from gello_hil.position import resolve_case
from gello_hil.exports import export_support
from gello_hil.session import OperatorSession
from gello_hil.ui import operator_panel, poll_command
from test_alignment_safety import calibration, counts_for, POSE

PLAN = Path(cli.__file__).parent / "plans/libero_pro_position12.json"


def test_plan_has_180_disjoint_support_cases():
    tasks = json.loads(PLAN.read_text())["tasks"]
    cases = [
        resolve_case(PLAN, t["key"], draw, shot)
        for t in tasks
        for draw in range(1, 4)
        for shot in range(1, 6)
    ]
    assert len(cases) == 180
    assert len({(c["key"], c["state_index"]) for c in cases}) == 180
    assert {c["state_index"] for c in cases} == set(range(15))
    assert {c["suite"] for c in cases} == {"libero_object_swap", "libero_goal_swap"}
    with pytest.raises(ValueError):
        resolve_case(PLAN, "object_milk", 4, 1)


def counts(pose=POSE):
    return {**dict(zip(range(1, 8), counts_for(pose))), 8: 4000}


def fake_fk(joints):
    return np.asarray(joints[:3]), np.eye(3)


def test_pose_follower_neutral_reanchor_and_actual_seven_actions():
    follower = PoseFollower(
        calibration(),
        fake_fk,
        output_min=[-0.05] * 3 + [-0.5] * 3,
        output_max=[0.05] * 3 + [0.5] * 3,
    )
    follower.anchor(counts(), POSE)
    neutral, _ = follower.action(counts(), POSE)
    assert np.array_equal(neutral, np.zeros(7))
    moved = POSE.copy()
    moved[0] += 0.02
    action, _ = follower.action(counts(moved), POSE)
    assert action.shape == (7,) and 0.35 < action[0] < 0.45
    assert action[-1] == -1
    follower.anchor(counts(moved), POSE)
    assert np.array_equal(follower.action(counts(moved), POSE)[0], np.zeros(7))


@pytest.mark.parametrize("angle", [0, 1e-9, 0.2, np.pi - 1e-7, np.pi])
def test_rotation_conversion_near_zero_and_pi(angle):
    matrix = np.array(
        [
            [1, 0, 0],
            [0, np.cos(angle), -np.sin(angle)],
            [0, np.sin(angle), np.cos(angle)],
        ]
    )
    vector = matrix_rotvec(matrix)
    assert np.linalg.norm(vector) == pytest.approx(angle, abs=1e-7)
    assert np.linalg.norm(vector[1:]) < 1e-7


def test_invalid_session_anchor_does_not_mutate_pause():
    session = OperatorSession()
    with pytest.raises(ValueError):
        session.resume(
            anchor_counts=[], follower_joints=POSE, leader_joints=POSE, mode="bad"
        )
    assert session.status == "PAUSED"


def test_circular_encoder_calibration_capture():
    samples = iter([4094, 4095, 0, 1, 2])
    bus = SimpleNamespace(read_positions=lambda ids: {i: next(samples) for i in ids})
    assert cli._sample_positions(bus, [1], 5, 0)[1] == 0


def observation(value=0):
    return {
        "agentview_image": np.full((16, 16, 3), value, np.uint8),
        "robot0_eye_in_hand_image": np.full((16, 16, 3), value + 1, np.uint8),
        "robot0_eef_pos": np.zeros(3),
        "robot0_eef_quat": [0, 0, 0, 1],
        "robot0_gripper_qpos": [0.04, 0.04],
    }


class FakeEnv:
    def __init__(self, successes):
        self.successes = successes
        self.steps = 0
        self.closed = False
        self.env = SimpleNamespace(control_freq=20)

    def get_sim_state(self):
        return np.asarray([self.steps], dtype=float)

    def step(self, action):
        assert len(action) == 7
        self.steps += 1
        return observation(self.steps), 1, False, {}

    def check_success(self):
        return self.successes[min(self.steps - 1, len(self.successes) - 1)]

    def close(self):
        self.closed = True


def collect_fake(tmp_path, monkeypatch, *, keys, successes, budget=10, hold=2):
    import gello_hil.libero_runtime as runtime
    import gello_hil.position as position
    import gello_hil.pose_control as pose

    calfile = tmp_path / "calibration.json"
    calibration().to_json(calfile)
    env = FakeEnv(successes)
    bus = SimpleNamespace(read_positions=lambda ids: counts())

    class Context:
        def __enter__(self):
            return bus

        def __exit__(self, *args):
            pass

    args = cli.build_parser().parse_args(
        [
            "collect-position",
            "--port",
            "unused",
            "--calibration",
            str(calfile),
            "--libero-source",
            "unused",
            "--task-key",
            "object_milk",
            "--draw",
            "1",
            "--shot",
            "3",
            "--output-root",
            str(tmp_path / "runs"),
            "--max-active-steps",
            str(budget),
            "--success-hold-steps",
            str(hold),
        ]
    )
    monkeypatch.setattr(
        position,
        "make_position_env",
        lambda args: (env, observation(), "native instruction"),
    )
    monkeypatch.setattr(
        pose,
        "make_pose_follower",
        lambda env, cal: PoseFollower(
            cal,
            fake_fk,
            output_min=[-0.05] * 3 + [-0.5] * 3,
            output_max=[0.05] * 3 + [0.5] * 3,
        ),
    )
    monkeypatch.setattr(runtime, "follower_joints", lambda env: POSE.copy())
    monkeypatch.setattr(cli, "_bus", lambda args: Context())
    monkeypatch.setattr(cli.cv2, "namedWindow", lambda *a: None)
    monkeypatch.setattr(cli.cv2, "imshow", lambda *a: None)
    monkeypatch.setattr(cli, "close_window", lambda *a: None)
    key_iter = iter(keys)
    monkeypatch.setattr(cli, "poll_command", lambda *a: next(key_iter, "abort"))
    code = cli.position_collect(args)
    attempt = next((tmp_path / "runs").iterdir())
    return env, code, attempt, json.loads((attempt / "attempt.json").read_text())


def test_collector_pause_reanchor_observation_pairing_and_export(tmp_path, monkeypatch):
    env, code, attempt, meta = collect_fake(
        tmp_path,
        monkeypatch,
        keys=[None, "resume", None, "pause", None, "resume", None, None],
        successes=[False, True, True],
    )
    assert code == 0 and env.closed and env.steps == 3
    assert meta["instruction"] == "native instruction"
    assert meta["position_case"]["state_index"] == 2
    events = [x["event_type"] for x in meta["session"]["events"]]
    assert events == [
        "initial_pause",
        "initial_anchor_resume",
        "clutch_pause",
        "reanchor_resume",
        "success",
    ]
    with np.load(attempt / "observations.npz", allow_pickle=False) as d:
        assert d["state"].shape == (4, 9)
        assert d["sim_states"][:, 0].tolist() == [0, 1, 2, 3]
    assert (
        meta["source"]["upstream_commit"] == "26be20212ff0709668b776ebefabe7da1809aa73"
    )
    exported = tmp_path / "support.npz"
    export_support(attempt, exported)
    with np.load(exported, allow_pickle=False) as d:
        assert d["actions"].shape == (3, 7) and d["image"].shape == (3, 16, 16, 3)
    with pytest.raises(FileExistsError):
        export_support(attempt, exported)


def test_transient_success_is_not_accepted(tmp_path, monkeypatch):
    env, code, attempt, meta = collect_fake(
        tmp_path,
        monkeypatch,
        keys=["resume"] + [None] * 4,
        successes=[True, False, False, False],
        budget=4,
    )
    assert code == 1 and not meta["success"] and env.steps == 4
    with pytest.raises(ValueError, match="failed"):
        export_support(attempt, tmp_path / "invalid.npz")


def test_abort_while_paused_preserves_zero_action_attempt(tmp_path, monkeypatch):
    env, code, attempt, meta = collect_fake(
        tmp_path, monkeypatch, keys=["abort"], successes=[False]
    )
    assert code == 1 and env.steps == 0 and meta["aborted"]
    with np.load(attempt / "trajectory.npz", allow_pickle=False) as d:
        assert d["actions"].shape == (0, 7)


def test_gui_low_resolution_and_closed_window(monkeypatch):
    panel = operator_panel(
        np.zeros((64, 64, 3), np.uint8),
        instruction="task " * 30,
        status="PAUSED",
        active_steps=0,
        max_steps=1000,
        mapping_mode="enter-anchored-osc-pose",
        joints=POSE,
        errors=POSE,
    )
    assert panel.shape[0] > 600
    monkeypatch.setattr(cli.cv2, "waitKey", lambda *a: -1)
    monkeypatch.setattr(cli.cv2, "getWindowProperty", lambda *a: 0)
    assert poll_command("closed") == "abort"


def test_real_mujoco_shadow_fk_leaves_live_state_untouched():
    mujoco = pytest.importorskip("mujoco")
    from gello_hil.pose_control import PandaKinematics

    bodies = "".join(
        '<body pos=".1 0 0"><joint type="hinge" axis="0 0 1"/><geom type="sphere" size=".02" mass="1"/>'
        for _ in range(7)
    )
    model = mujoco.MjModel.from_xml_string(
        "<mujoco><worldbody>"
        + bodies
        + '<site name="eef" pos=".1 0 0"/>'
        + "</body>" * 7
        + "</worldbody></mujoco>"
    )
    live = mujoco.MjData(model)
    live.qpos[:] = np.linspace(0, 0.2, 7)
    mujoco.mj_forward(model, live)
    env = SimpleNamespace(
        env=SimpleNamespace(
            robots=[SimpleNamespace(eef_site_id=0, _ref_joint_pos_indexes=np.arange(7))]
        ),
        sim=SimpleNamespace(model=SimpleNamespace(_model=model), data=live),
    )
    before = live.qpos.copy(), live.qvel.copy(), live.time
    fk = PandaKinematics(env)
    pos, rot = fk(np.zeros(7))
    assert np.allclose(pos, [0.8, 0, 0]) and np.allclose(rot, np.eye(3))
    assert np.array_equal(before[0], live.qpos) and np.array_equal(before[1], live.qvel)
    assert before[2] == live.time


def test_oat_export_preserves_raw_camera_and_axis_angle(tmp_path, monkeypatch):
    h5py = pytest.importorskip("h5py")
    _, _, attempt, _ = collect_fake(
        tmp_path, monkeypatch, keys=["resume", None, None], successes=[True, True]
    )
    output = tmp_path / "support.hdf5"
    export_support(attempt, output, format="oat-hdf5")
    with h5py.File(output) as f:
        assert f["data"].attrs["num_demos"] == 1
        assert f["data/demo_0/actions"].shape == (2, 7)
        assert f["data/demo_0/obs/ee_ori"].shape == (2, 3)
        assert np.array_equal(f["data/demo_0/obs/ee_ori"][:], np.zeros((2, 3)))


def test_historical_calibration_explicit_import_and_mismatch_rejection(tmp_path):
    from dataclasses import asdict
    from gello_hil.calibration import GelloCalibration

    cal = calibration()
    payload = asdict(cal)
    payload["schema_version"] = (
        "d31-gello-session-discrete-assembly-offset-calibration-v15"
    )
    payload["calibration_pose_joint_positions"] = payload.pop("reference_pose")
    payload["joint_position_limits"] = payload.pop("joint_limits")
    source = tmp_path / "session.json"
    source.write_text(json.dumps(payload))
    imported = tmp_path / "imported.json"
    cli.build_parser().parse_args(
        ["import-calibration", "--source", str(source), "--output", str(imported)]
    ).func(SimpleNamespace(source=str(source), output=str(imported)))
    assert GelloCalibration.from_json(imported) == cal
    assert json.loads(imported.read_text())["import_provenance"][
        "historical_capture_not_fresh_calibration"
    ]
    payload["assembly_offsets_radians"] = list(payload["assembly_offsets_radians"])
    payload["assembly_offsets_radians"][0] += 1
    source.write_text(json.dumps(payload))
    with pytest.raises(ValueError, match="offsets disagree"):
        GelloCalibration.from_session_json(source)


def test_input_monitor_only_uses_read_api(tmp_path, monkeypatch):
    import gello_hil.input_monitor as monitor

    source = tmp_path / "cal.json"
    calibration().to_json(source)

    class Reader:
        def __init__(self, *a, **kw):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *a):
            pass

        def ping(self, i):
            return {"id": i, "error": 0}

        def read(self, i, addr, length):
            assert addr == 64 and length == 1
            return 0, b"\0"

        def read_positions(self, ids):
            return counts()

    monkeypatch.setattr(monitor, "ReadOnlyDynamixelBus", Reader)
    monkeypatch.setattr(monitor.time, "sleep", lambda x: None)
    args = SimpleNamespace(
        samples=2,
        hz=5,
        output=tmp_path / "input.json",
        calibration=source,
        port="unused",
        baudrate=57600,
        serial_timeout=0.12,
        gui=False,
    )
    assert monitor.read_input(args) == 0
    assert json.loads(args.output.read_text())["motor_register_writes"] == 0


def test_alignment_camera_color_and_complete_panel():
    from gello_hil.alignment import draw_alignment_panel
    from gello_hil.actuation import GelloAlignmentAssistLimits

    image = np.zeros((64, 64, 3), np.uint8)
    image[:, :, 0] = 255
    canvas = draw_alignment_panel(
        image,
        instruction="pick up " * 40,
        task_id=0,
        state_id=0,
        attempt_index=1,
        status="PAUSED",
        diagnostic=None,
        preview_error="no input",
        notice="support GELLO",
        limits=GelloAlignmentAssistLimits(),
        passive_start_ready=False,
    )
    assert np.array_equal(canvas[0, 0], [0, 0, 255])
    assert canvas.shape[0] > 400


def test_native_position_setup_uses_official_case_and_ten_warmup_steps(
    tmp_path, monkeypatch
):
    import sys
    from types import ModuleType
    import gello_hil.position as position
    import gello_hil.libero_runtime as runtime

    root = tmp_path / "pro"
    root.mkdir()
    folder = root / "bddl/scene"
    folder.mkdir(parents=True)
    (folder / "milk.bddl").write_text(
        "(define (:language pick up the milk and place it in the basket))"
    )
    (folder / "milk.pruned_init").write_bytes(b"fake native init fixture")
    case = resolve_case(PLAN, "object_milk", 1, 3)
    spec = SimpleNamespace(
        name=case["task_name"], problem_folder="scene", bddl_file="milk.bddl",
        init_states_file="milk.pruned_init"
    )
    suite = SimpleNamespace(
        get_task=lambda i: spec, get_task_init_states=lambda i: np.zeros((50, 8))
    )
    benchmark = ModuleType("libero.libero.benchmark")
    benchmark.__file__ = str(root / "benchmark.py")
    benchmark.get_benchmark_dict = lambda: {"libero_object_swap": lambda: suite}
    inner = ModuleType("libero.libero")
    inner.benchmark = benchmark
    inner.get_libero_path = lambda name: str(root / "bddl")
    outer = ModuleType("libero")
    outer.libero = inner
    monkeypatch.setitem(sys.modules, "libero", outer)
    monkeypatch.setitem(sys.modules, "libero.libero", inner)
    monkeypatch.setitem(sys.modules, "libero.libero.benchmark", benchmark)
    monkeypatch.setattr(
        position.subprocess,
        "check_output",
        lambda args, **kw: (
            case["expected_benchmark_commit"] if "rev-parse" in args else ""
        ),
    )
    env = FakeEnv([False] * 20)
    env.env.robots = [
        SimpleNamespace(controller=SimpleNamespace(reset_goal=lambda: None))
    ]
    env.regenerate_obs_from_state = lambda state: observation()
    calls = []

    def factory(*a, **kw):
        calls.append((a, kw))
        return env, observation(), "ignore this cached language"

    monkeypatch.setattr(runtime, "make_env", factory)
    args = SimpleNamespace(
        libero_source=root,
        position_case=case,
        suite=case["suite"],
        task_id=7,
        state_index=2,
        resolution=256,
        seed=0,
        max_active_steps=1000,
    )
    _, _, language = position.make_position_env(args)
    assert language == "pick up the milk and place it in the basket"
    assert env.steps == 10 and args.position_case["warmup_steps"] == 10
    assert calls[0][1]["controller"] == "OSC_POSE"
    assert args.position_case["bddl_sha256"]
