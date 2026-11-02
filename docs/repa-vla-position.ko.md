# REPA-VLA: GELLO 공통 입력과 LIBERO-Pro Position 시연 수집

기준 GELLO commit은 `26be20212ff0709668b776ebefabe7da1809aa73`, 작업 branch는 `REPA-VLA`다.
이 저장소를 공통 teleoperation 패키지로 사용한다. 기존 GR00T 실험과 새 OAT 실험에
GELLO 소스를 복사하지 않고 같은 패키지와 원본 시연을 참조한다.

최신 수집 목표는 **Position 전체40과제 ×10개 성공 시연**이다.
[400-demo pool 실행 절차](position40-pool10.ko.md)를 먼저 따른다.
아래 12-task/3-draw 설명은 이전 plan과의 호환 경로다.

## 현재 확인한 범위

- 기존 `openvla-oft` Python 3.10 환경으로 실제 장치 ID 1–8에 PING/READ/SYNC_READ를 실행했다.
- arm 1–7은 model1200, gripper8은 model1190, firmware53이었다.
- encoder 20회/5Hz 읽기와 실제 OpenCV GUI 10회/5Hz 읽기가 오류 없이 완료됐다.
- 읽은 torque_enable은 8개 모두0이었다. 이 작업에서 torque/current/goal register 쓰기는0이다.
- 원본 사용자의 calibration은 D31-v15 형식이었다. 새 파일로 가져왔으며 실제 재보정한 것이 아니다.
- `HIL` 환경은 Python3.9다. 본 패키지는 Python>=3.10이므로 현재는 `openvla-oft`를 사용한다.
- A6는 모의 transport로 검증한다. 실제 모터 보조, 물리 조립 방향/offset 검증,
  실제 LIBERO-Pro Position 시연 성공은 이번 read-only 확인에 포함하지 않는다.
- 로컬의 기존 UnlearningVLA/LIBERO는 원본 LIBERO commit `8f1084e...`다.
  Position 실행에는 별도 LIBERO-PRO checkout과 공식 BDDL/init 자산을 지정해야 한다.
  환경이나 기존 데이터 경로를 자동 교체하지 않는다.
- 별도 pinned Pro checkout과 공식40과제 BDDL/init80개를 준비했고,
  과제 identity40개와 과제별 native 초기 상태50개를 확인했다. 실제 시연은0개다.

실제 확인 원본은 이 checkout의 `runs/device_check_20261102/`에 보존한다.
calibration, 장치별 기록, 시연/영상은 Git에서 제외한다.

## 공통 패키지와 데이터 경계

```text
src/gello_hil/
  dynamixel.py       # 재사용: 읽기 전용 encoder transport
  calibration.py     # 재사용 + 명시적 과거 calibration 가져오기
  session.py         # 재사용: 시작·pause·re-anchor
  input_monitor.py   # 새 기능: 유한 read-only 입력 확인
  pose_control.py    # 새 기능: 관절 target -> 독립 FK -> native OSC_POSE
  position.py        # 새 기능: 공식 Position task/init와 provenance
  recording.py       # 공통 카메라·상태·source 기록
  exports.py         # 기존 native NPZ / OAT 형식 HDF5 내보내기
  actuation.py       # 기존 제한된 모터 보조 transport
  live_handoff.py    # 기존 A6 단일 bus 소유권과 force fade
```

공통 원본은 각 `runs/position-gello/<timestamp>/`에 한 번만 저장한다.
기존 실험과 새 실험은 export sidecar의 episode/task/draw/shot 및 source 지문을 참조한다.
기존 N/W의 scripted-expert 결과와 실패 시도는 수정하지 않는다.
사람 GELLO 시연은 `human_gello`라는 새 수집 조건이다.

## 로컬 실행

아래 명령은 이 저장소 root에서 실행한다. `GELLO_PORT`에는 실제 stable by-id 포트를 넣는다.
환경 전역을 수정할 필요 없이 `PYTHONPATH=src`로 실행할 수 있다.

```bash
conda activate openvla-oft
export PYTHONNOUSERSITE=1
export PYTHONPATH="$PWD/src"
export GELLO_PORT=/dev/serial/by-id/YOUR_DEVICE
python -m gello_hil.cli probe --port "$GELLO_PORT" --baudrate 57600
```

새로 calibration할 때는 원래 `calibrate`를 사용하여 실제 각 관절의 sign과 조립 offset을 확인한다.
예제 sign을 모든 조립에 적용하지 않는다. 과거 성공 capture를 명시적으로 가져오려면:

```bash
python -m gello_hil.cli import-calibration \
  --source /absolute/path/to/session-calibration.json \
  --output runs/my-session/calibration.json

python -m gello_hil.cli read-input \
  --port "$GELLO_PORT" --baudrate 57600 \
  --calibration runs/my-session/calibration.json \
  --samples 300 --hz 20 --gui --output runs/my-session/input-check.json
```

`read-input`은 지정한 샘플 수에서 끝난다. q/ESC/창 닫기로 먼저 종료할 수 있다.
시뮬레이터를 import하거나 follower 명령을 보내지 않는다.
가져온 calibration에는 원본 파일 SHA와 과거 capture라는 사실을 기록한다.
이것을 현재 조립에 대한 calibration 성공으로 취급하지 않는다.

## Position support 한 시도

기본 plan은 `src/gello_hil/plans/libero_pro_position12.json`이다.
기존 설계와 같은 Object10 + Goal1/8의12 task이며, **draw1..3 × shot1..5**를
native init0..14에 대응시킨다. task/shot은 사람이 지정한다. 실패 자동 반복이나 다음 task 자동 실행은 없다.

LIBERO-Pro 기준 source commit은 `eafdb809426b13153aa1e4c42d6601844217dfec`다.
패키지가 다른 LIBERO checkout에서 이미 import됐거나 tracked source가 변경됐으면 실행을 거부한다.
`LIBERO_CONFIG_PATH`에는 해당 Pro용 config.yaml이 있는 디렉터리를 사용한다.
공식 안내: https://github.com/Zxy-MLlab/LIBERO-PRO

```bash
python -m gello_hil.cli collect-position \
  --port "$GELLO_PORT" --baudrate 57600 \
  --calibration runs/my-session/calibration.json \
  --libero-source /absolute/path/to/LIBERO-PRO \
  --libero-config-path /absolute/path/to/pro-config \
  --task-key object_milk --draw 1 --shot 3
```

입력은 기존 calibration/Enter/clutch로7관절 target을 만들고, live simulator와 독립된
MuJoCo data에서 FK를 계산한다. 실제 follower는 native `OSC_POSE`의7D EEF action으로
제어한다. 기존8D joint action을7D 데이터로 이름만 바꾸지 않는다.
controller의 실제 input/output scale과 fixed impedance 계약을 확인한다.
OSC 회전은 robosuite의 world-frame left composition에 맞춘 axis-angle delta다.

공식 BDDL/init와 native instruction을 사용하며 target/fixture/XY를 옮기지 않는다.
seed0/256px, 기존 계약과 같은10-step warmup 후 시작한다.
PAUSED/Enter 자체는 추가 simulator step을 소비하지 않는다.
첫 action은 neutral이며 pause 후 Enter는 다시 anchor한다.
순간적인 성공으로 끝내지 않고 기본20 active-step 연속 native success를 확인한다.
그 구간의 실제 hold action과 관측도 원본에 저장한다. 이 값은 metadata에 기록한다.

## 모터 보조

물리 검증을 진행할 때의 명령은 `assisted-position`이다. `--apply`가 없으면 bus를 열지 않는다.
기존 interactive authorization과 alignment READY Enter를 거쳐 A6를 사용한다.

```bash
python -m gello_hil.cli assisted-position \
  --port "$GELLO_PORT" --baudrate 57600 \
  --calibration runs/my-session/calibration.json \
  --libero-source /absolute/path/to/LIBERO-PRO \
  --libero-config-path /absolute/path/to/pro-config \
  --task-key object_milk --draw 1 --shot 3 --apply
```

100mA/current, P750, guide20counts, tracking40counts, 5Hz와 watchdog 제한을 유지한다.
gripper torque는 켜지 않는다. A6는 READY Enter 후 동일 창에서 제어하면서2초에 걸쳐
기존 force를 줄인다. 실물 검증 전까지 A6의 실제 동작이 검증됐다고 해석하지 않는다.
이번 사용자의 최신 요청은 **우선 read-only encoder 입력 연결**이므로 위 모터 명령은 실행하지 않았다.

## 저장과 학습용 내보내기

```text
attempt.json         # task/instruction, native success/error, 모든 이벤트, 실제 source commit+SHA
calibration.json     # 실제 사용한 calibration 값의 고정 snapshot
trajectory.npz       # 실제 action[T], encoder counts[T], follower joints[T]
observations.npz     # raw 양쪽 RGB, 9D proprio, sim state, 시간 [T+1]
operator-view.mp4    # operator 카메라 영상 (zero-action 시 없을 수 있음)
```

`observations[t] -> actions[t] -> observations[t+1]`을 유지한다.
관측과 action 시각은 session 시작을 기준으로 기록하며 pause/re-anchor 이벤트와 맞춘다.
sim state/성공 flag/모터 telemetry는 분석·재현 metadata이며 policy 입력에서 제외한다.
source는 upstream 기준 commit, 실행 HEAD/branch, dirty 상태 및 Python source별 SHA를 저장한다.
실패·abort·zero-action도 보존하고 성공한 registered support만 export한다.

```bash
python -m gello_hil.cli export-support \
  --attempt runs/position-gello/ATTEMPT --format native-npz --output exports/milk-d1-s3.npz
python -m gello_hil.cli export-support \
  --attempt runs/position-gello/ATTEMPT --format oat-hdf5 --output exports/milk-d1-s3.hdf5
```

native NPZ는 기존 GR00T용180도 RGB convention을 사용한다.
HDF5는 raw camera와 axis-angle state로 내보내며 OAT loader의 vertical flip과 구분해 기록한다.
HDF5 한 파일은 한 시연이다. 현재 OAT의 공식10task×50demo 개발 loader는 이 파일을
그대로 받지 않으므로 별도 few-shot manifest/loader 연결이 필요하다.
이 exporter를 사용했다고 기존 OAT의 full-data split/정규화 계약을 바꾼 것으로 취급하지 않는다.

## GUI 수정과 남은 확인

- 낮은 해상도에서 하단 관절/notice가 잘리던 문제: 내용에 따라 panel 높이를 확장한다.
- 긴 instruction/phase: rendered pixel 폭에 맞춰 줄바꿈하며 문장을 잘라 버리지 않는다.
- alignment RGB를 BGR로 변환하여 collection 화면과 색을 일치시켰다.
- collection/mapping/alignment의 창 닫기와 ESC를 abort로 처리하여 cleanup 경로로 보낸다.
- A6 READY 안내를 실제 동작인 START +2s force fade에 맞췄다.
- wrap 경계4095/0에서 calibration median이 잘못 반 바퀴로 계산되던 문제를 수정했다.

실제 확인한 GUI는 read-only encoder 화면이다. 전체 Pro task 화면, 각 관절 방향과
전체 가동범위, 실제 A6 fade/클러치의 조립별 동작은 operator가 함께 확인해야 한다.
