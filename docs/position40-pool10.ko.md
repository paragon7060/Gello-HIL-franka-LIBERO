# Position 전체 40과제 × 10-demo 수집과 k-shot 실험

목표는 **LIBERO-Pro Position 40과제마다 성공 시연 10개, 총 400개**를 GELLO 모터 보조
방식으로 수집하는 것이다. 원본 400개를 고정한 뒤 같은 데이터에서 1·3·5·10-shot
학습용 목록을 만든다. 영어 Qt6 GUI인 **GELLO Studio**에서 초기 장면, 수집, 영상 검토,
재시도와 성공 시연 확인을 관리한다. [English GUI guide](collector-studio.md).
GELLO 입력·보정·GUI·기록 코드는 이 저장소 `REPA-VLA` 브랜치에서
공통으로 관리하고, 기존 실험과 새 실험이 같은 원본 및 선택 manifest를 참조한다.

**현재는 수집 코드와 공식 자산을 준비한 단계다. 실제 성공 시연은 0/400이며 실물 A6
모터 보조는 아직 검증하지 않았다.** 실제 시연은 사람이 GELLO를 조작해서 수행한다.
코드/모의 transport 테스트를 실제 모터 검증이나 400개 수집 완료로 해석하지 않는다.

## 1. 데이터와 폴더

| Position suite | task key | 과제 수 | 과제별 성공 시연 | 합계 |
| --- | --- | ---: | ---: | ---: |
| `libero_spatial_swap` | `spatial_00`–`spatial_09` | 10 | 10 | 100 |
| `libero_object_swap` | `object_00`–`object_09` | 10 | 10 | 100 |
| `libero_goal_swap` | `goal_00`–`goal_09` | 10 | 10 | 100 |
| `libero_10_swap` | `long_00`–`long_09` | 10 | 10 | 100 |

과제 번호는 공식 suite의 기본 task order(0부터 시작)다. 예를 들어 우유는 `object_07`이다.
정확한 40개 이름은 [고정 plan](../src/gello_hil/plans/libero_pro_position40_pool10.json)에 있다.
기존 12-task × 3-draw × 5-shot plan은 그대로 보존한다.

```text
Gello-HIL-franka-LIBERO/                 # 공통 코드, REPA-VLA 브랜치
  src/gello_hil/                         # 입력/보정/A6/OSC/기록/export
  src/gello_hil/plans/                   # 과제와 support 상태 등록
  scripts/prepare_position_assets.py     # 공식 자산만 별도로 준비
  docs/                                 # 실행 절차
  runs/                                 # 전부 Git 제외
    position40-assets/                  # 공식 BDDL/init, 로컬 config와 다운로드 지문
    calibration-SESSION/                # 실제 조립의 보정 결과
    position40-pool10/
      pool-plan.json                    # 수집 시작 전에 고정한 plan
      .collection.lock                  # 동일 pool의 중복 수집/확인 방지
      review-policy.json                # GUI의 명시적 확인 필수
      reviews/<task>/shotNN/             # hash chain으로 연결한 확인/재시도 이력
      gui-sessions/<id>/                # 단일 worker, 초기 장면과 제어 진단
      attempts/
        object_07/shot01/<UTC timestamp>/
          attempt.json                  # instruction, outcome, source, 모든 이벤트
          calibration.json              # 실제 사용한 보정 snapshot
          trajectory.npz                # 실제 7D action, encoder counts, 관절
          observations.npz              # raw RGB 두 개, proprio, sim state, 시간
          operator-view.mp4
      pool-manifest.json                # 400/400 검증 후에만 생성
    position40-subsets/
      seed42_k01.json                    # 40개 episode
      seed42_k03.json                    # 120개 episode
      seed42_k05.json                    # 200개 episode
      seed42_k10.json                    # 400개 episode
```

shot 1–10은 **native init 0–9**에 고정한다. init 10–49는 이 수집에 넣지 않는다.
평가 split은 나중에 별도 manifest로 정한다. 과거에 관측한 query를 새로운 blind test라고
부르지 않으며, 다른 init 번호라고 XY가 반드시 다르다고 가정하지 않는다.
이 pool은 `human_gello_motor_assisted`라는 새 조건이다. 기존 scripted-expert/N/W 결과를
덮어쓰거나 그 support draw를 이 데이터로 바꾸지 않는다.

## 2. Python과 입력 확인

명령은 GELLO 저장소 root에서 실행한다. 로컬에 이미 `REPA-VLA` checkout이 있으면
그 checkout을 사용한다. 새 컴퓨터에서는 다음처럼 가져온다.

```bash
git clone --branch REPA-VLA https://github.com/paragon7060/Gello-HIL-franka-LIBERO.git
cd Gello-HIL-franka-LIBERO
conda activate openvla-oft
export PYTHONNOUSERSITE=1
export PYTHONPATH="$PWD/src"
export NUMBA_CACHE_DIR="$PWD/runs/numba-cache"
export GELLO_PORT=/dev/serial/by-id/YOUR_DEVICE
export SESSION="runs/calibration-$(date +%Y%m%dT%H%M%S)"
export POOL="$PWD/runs/position40-pool10"
```

Python>=3.10, NumPy<2, Qt6(`PySide6-Essentials`)와 LIBERO/robosuite 1.4.x가 필요하다.
기존 OpenCV GUI도 호환 경로로 유지한다.
현재 확인한 `HIL` 환경은 Python3.9여서 이 패키지를 실행할 때 `openvla-oft`를 사용했다.
Numba cache도 이 checkout의 `runs/`에 두므로 conda 설치 경로를 수정하지 않는다.
새 환경에서 패키지 설치가 필요하면 `python -m pip install -e '.[gui,export]'`를 사용한다.
기존 LIBERO나 사용자 전역 config를 자동 교체하지 않는다.

```bash
python -m gello_hil.cli probe --port "$GELLO_PORT" --baudrate 57600
```

ID1–7 arm과 ID8 gripper가 모두 응답해야 한다. 다른 프로그램이 같은 포트를 열어 둔
경우 종료한 뒤 진행한다. 장치별 포트·보정 파일은 공개 GitHub에 올리지 않는다.

## 3. 실제 조립 보정과 mapping 확인

아래 sign은 예시다. 실제 조립에서 각 관절의 방향을 확인해 사용한다.

```bash
python -m gello_hil.cli calibrate \
  --port "$GELLO_PORT" --baudrate 57600 \
  --joint-signs 1 1 1 -1 1 -1 1 \
  --output "$SESSION/calibration.json"
```

터미널 안내에 따라 GELLO의 각 관절을 Panda 기준
`J1=0°, J2=0°, J3=0°, J4=-90°, J5=0°, J6=+90°, J7=0°`에 맞춘 뒤 Enter를 누른다.
이어 arm을 유지한 채 gripper 완전 열림/닫힘을 각각 capture한다.
단순히 끝점만 비슷하게 맞추지 않는다. 각 관절 sign과 quarter-turn offset을 확인한다.

과거 D31 capture를 참고로 가져오는 경우에만 다음을 사용한다. 이는 현재 조립의
재보정이나 mapping 검증을 대신하지 않는다.

```bash
python -m gello_hil.cli import-calibration \
  --source /path/to/session-calibration.json --output "$SESSION/calibration.json"
```

`calibrate`와 `import-calibration` 중 사용할 경로 하나를 선택한다. 같은 파일을 덮어쓰지 않는다.

```bash
python -m gello_hil.cli read-input \
  --port "$GELLO_PORT" --baudrate 57600 \
  --calibration "$SESSION/calibration.json" \
  --samples 300 --hz 20 --gui --output "$SESSION/input-check.json"
```

읽기만 수행하며 모터 register를 쓰지 않는다. q/ESC/창 닫기로 종료할 수 있다.
현재 장치의 read-only 연결은 확인했으나, 실물 A6 전에 실제 조립의 mapping 확인이 필요하다.

## 4. 공식 LIBERO-Pro 자산 준비

기준 source는 `eafdb809426b13153aa1e4c42d6601844217dfec`다.
[공식 소스](https://github.com/Zxy-MLlab/LIBERO-PRO/tree/eafdb809426b13153aa1e4c42d6601844217dfec)와
[공식 자산](https://huggingface.co/datasets/zhouxueyang/LIBERO-Pro)을 사용한다.
기존 원본 LIBERO checkout에는 Position `_swap` 자산이 없으므로 별도 source를 지정한다.

```bash
git clone https://github.com/Zxy-MLlab/LIBERO-PRO.git ../LIBERO-PRO
git -C ../LIBERO-PRO checkout --detach eafdb809426b13153aa1e4c42d6601844217dfec
export LIBERO_PRO_ROOT="$(cd ../LIBERO-PRO && pwd)"
python scripts/prepare_position_assets.py \
  --libero-source "$LIBERO_PRO_ROOT" --output runs/position40-assets
export LIBERO_CONFIG_PATH="$PWD/runs/position40-assets/config"
```

이미 준비된 로컬 checkout에서는 clone/다운로드를 반복하지 않고 위 두 경로 변수만 지정한다.
준비 스크립트는 dataset revision `c86fc3b8293185a6f373677018ff3e37f8391602`에서
해당 40과제의 BDDL/init 80개만 받고 공식 blob checksum과 크기를 확인한다.
source 파일이나 기존 `.libero/config.yaml`을 바꾸지 않고 별도 config를 만든다.
이미 있는 출력 폴더를 덮어쓰지 않는다. 중간 다운로드 실패도 새 폴더에서 다시 준비한다.

Pro source를 함께 지정하여 mapping을 확인한다.

```bash
PYTHONPATH="$PWD/src:$LIBERO_PRO_ROOT" python -m gello_hil.cli mapping-check \
  --port "$GELLO_PORT" --calibration "$SESSION/calibration.json" \
  --libero-config-path "$LIBERO_CONFIG_PATH" \
  --suite libero_object_swap --task-id 7 --state-index 0
```

각 관절을 하나씩 움직여 화면과 방향·범위를 확인하고 q로 종료한다.
이 preview는 실제 모터를 구동하거나 physics step을 진행하지 않는다.

## 5. GELLO Studio로 수집·검토·재시도

이후에는 **GUI 명령 하나**로 관리한다. task 이름이나 재시도 flag를 매번 입력하지 않는다.
40과제 목록과 shot1–10 슬롯을 클릭하고, 이전 실행의 첫 미확인 슬롯부터 이어서 수집한다.

```bash
python -m gello_hil.cli position-pool-gui \
  --port "$GELLO_PORT" --baudrate 57600 \
  --libero-source "$LIBERO_PRO_ROOT" --libero-config-path "$LIBERO_CONFIG_PATH" \
  --dataset-root "$POOL" --apply
```

모든 화면 문구는 영어이며 Qt6 폰트 렌더링을 사용한다. 기존 Tk/저해상도 문구 대신
과제·shot 목록, 넓은 장면/영상 뷰어, 상태별 버튼과 시도 이력을 한 화면에 배치했다.
처음에는 실제 native 초기 장면이 보인다. 보정 파일을 아직 선택하지 않은 초기 장면은
serial 통신 없이 표시한다. **Calibration...**에서 현재 조립의 보정 JSON을 고른다.
`--calibration "$SESSION/calibration.json"`을 실행 시 지정해도 된다.
encoder/alignment worker가 열린 뒤에는 장치/보정 설정을 잠그므로 **Stop & review** 후 변경한다.

`--apply`를 생략하면 motor/collection 버튼을 비활성화한다. 영상 검토와 기존 후보 확인은
가능하다. `--apply`를 줘도 창을 열거나 다음 shot으로 이동하는 것만으로 모터를 켜지 않는다.

| 영어 버튼 | 수행 내용 |
| --- | --- |
| **Open scene** | 선택 task/shot의 등록된 초기 장면을 연다 |
| **Motor assist** | 명시적으로 기존 제한된 A6 정렬을 요청한다 |
| **Start collection** | 정렬된 상태에서 수집 시작, 또는 pause에서 현재 자세로 re-anchor/재개 |
| **Pause** | force 해제와 clutch/pause, physics·active budget 정지 |
| **Stop & review** | 해제·종료·원본 보존 후 같은 창에서 검토 |
| **Play**와 seek bar | 선택 시도의 영상 재생과 프레임 탐색 |
| **Retry shot** | 이전 시도를 보존하고 동일 task/init에 새 시도를 준비 |
| **Confirm & next** | 성공 시도의 원본 검증·채택 후 다음 미확인 shot 선택 |
| **Next pending** | 다음 미확인 슬롯으로 이동 |

**장면 확인 → Motor assist → Start collection → 영상 검토 → Retry shot 또는 Confirm & next**
순서다. 모터 보조가 필요 없을 정도로 가까운 자세이면 기존 passive-start를 사용하며
motor write0이다. 이때도 **Motor assist** 클릭만으로 수집을 시작하지 않는다.
다음 shot은 초기 장면만 자동으로 열고 보조·수집 시작은 다시 버튼을 기다린다.
ACTIVE 중 과제 이동은 차단하고 pause/stop을 먼저 사용한다.
창 닫기는 worker의 정상 force 해제·기록 완료를 기다리며 강제 kill하지 않는다.

[SAFETY.md](../SAFETY.md)와 [기존 A6 절차](assisted.md)를 따른다.
arm current100mA, P750, guide20counts, tracking40counts, 5Hz/watchdog 제한과
gripper torque-off는 그대로다. **GELLO를 지지하고 전원 차단에 접근할 수 있는 사람이 조작한다.**
원래 A6 motor register 구현은 변경하지 않았으며, 실물 A6 성공은 아직 검증하지 않았다.
`motor_assistance_requested=true`가 모든 시연의 실제 전류 인가를 뜻하지는 않는다.
실제 write/fade telemetry를 함께 보존한다.

원본은 실패·중단도 매번 자동 저장한다. **Confirm은 성공 원본을 학습용으로 채택하는 별도 결정**이다.
최소20 active-step 연속 native success와 최종 true, 관측/action 정렬, calibration/provenance를
통과해야 확인할 수 있다. 실패나 중단을 확인하여 성공으로 바꿀 수 없다.
미확인 성공은 진행률과 export에 포함하지 않는다. 장면/목표/XY/instruction도 수정하지 않는다.

실패 후 앱을 다시 실행하지 않고 **Retry shot**을 누른다. 이전 NPZ/영상/실패/이벤트는 남는다.
이미 확인한 slot을 재시도해도 새 후보를 확인하기 전까지 기존 채택은 유지한다.
새 성공을 확인하면 선택만 교체하고 이전 원본·확인 이력을 지우지 않는다.
`reviews/`의 append-only hash chain과 원본 SHA로 이를 기록한다.
초기 장면만 둘러본 경우는 `gui-sessions/`에 두고 실제 시도 수에 넣지 않는다.

진행률은 GUI나 아래 명령으로 확인한다. 주기적 watcher/자동화는 만들지 않는다.

```bash
python -m gello_hil.cli position-pool-status --dataset-root "$POOL"
```

기존 `collect-position-pool` CLI도 보존한다. GUI에서 연 pool은 수동 확인 정책을 적용하므로
CLI로 만든 성공도 후보로 저장한 뒤 GUI 확인을 기다린다. 레거시 CLI의 실패 gate는
`--retry-failed`이지만 GUI에서는 **Retry shot**이 그 명시적 사용자 동작이다.

## 6. 400개 완료 후 고정과 k-shot 선택

```bash
python -m gello_hil.cli freeze-position-pool --dataset-root "$POOL"
python -m gello_hil.cli select-shots \
  --pool-manifest "$POOL/pool-manifest.json" \
  --shots 1 3 5 10 --seeds 42 43 44 \
  --output-dir runs/position40-subsets
```

GUI pool은400개가 모두 성공하고 명시적으로 확인되어야 freeze가 가능하다. 각 원본의 실제 action/관측 정렬, calibration
snapshot, source commit과 파일 SHA, video 존재를 확인하고 SHA256을 고정한다.
freeze 후에는 GUI 영상 검토만 가능하며 같은 pool에 시연/확인을 추가하지 않는다. 선택 전에 원본 checksum을 다시 확인한다.

같은 seed에서는 과제별 SHA256 순서의 앞 k개를 선택한다. 따라서
**1-shot ⊂ 3-shot ⊂ 5-shot ⊂ 10-shot**이다. k는1–10 중 원하는 값을 사용할 수 있다.
seed는 demo 선택 seed이며 학습 seed와 별도로 기록한다. 각 모델/teacher는 같은 manifest를
사용한다. 성공률·query 결과를 보고 좋은 demo를 고르는 선택은 하지 않는다.
10개를 재사용한 여러 subset은 서로 독립된 수집 실험이 아니다.

| k | 과제당 시연 | 전체 40과제 시연 |
| ---: | ---: | ---: |
| 1 | 1 | 40 |
| 3 | 3 | 120 |
| 5 | 5 | 200 |
| 10 | 10 | 400 |

학습에서 `k`는 시연 수이며 trajectory frame 수가 아니다. k 간 비교 시 동일한 query,
모델 초기 checkpoint, 정규화/processor와 명시한 학습 update 예산을 사용한다.
선택 manifest SHA와 실제 학습 seed/update/target·source 노출을 결과에 기록한다.
이 문서의 명령은 데이터를 수집·선택하며 학습 job을 자동 실행하지 않는다.

## 7. 기존 실험과 새 실험에 연결

원본은 하나만 유지한다. 각 selection의 `pool_manifest` 기준 상대 `attempt` 경로와
`episode_id`를 통해 같은 시연을 참조한다. 기존 native 경로는 `native-npz`, OAT용
변환은 `oat-hdf5`를 사용한다. 확인까지 완료한 성공 한 시도의 변환 예시는 다음과 같다.

```bash
python -m gello_hil.cli export-support \
  --attempt "$POOL/attempts/object_07/shot01/ACTUAL_UTC_DIRECTORY" \
  --format native-npz --output runs/position40-exports/native/object_07-shot01.npz
python -m gello_hil.cli export-support \
  --attempt "$POOL/attempts/object_07/shot01/ACTUAL_UTC_DIRECTORY" \
  --format oat-hdf5 --output runs/position40-exports/oat/object_07-shot01.hdf5
```

실제 timestamp는 status/selection의 `attempt` 값을 사용한다. 같은 시연/형식은 한 번만
변환하고 k별로 원본을 복사하지 않는다. Export sidecar의 scene/source/action/camera 지문을
각 학습 manifest에 연결한다. raw image 변환과 action 정규화를 두 번 적용하지 않는다.

**현재 OAT의 full-data loader는 10task×50demo 계약이다.** 한 시연씩 내보낸 HDF5를
그 loader에 그대로 넣을 수는 없다. 실제 학습 연결 단계에서 이 selection manifest를 읽는
few-shot loader/adapter를 사용해야 한다. 기존 full-data loader의 계약을 조용히 변경하지 않는다.
관측은 T+1개, action은 T개다. sim state/success/이벤트/모터 telemetry는 policy 입력에서 제외한다.

## 8. 검증 범위와 문제 해결

- 로컬 실제 encoder/GUI 입력은 read-only로 확인했고 torque readback은 모두 OFF였다.
- 공식40 task identity와 과제별 native state50개를 확인했다. 실제 motor 시연은0개다.
- 실제 우유 Position 장면/init0도 CPU software EGL로 열어 공식10-step warmup과
  OSC_POSE/FK 연결을 확인하고 카메라 PNG를 검토했다. collection action0/모터 write0이다.
- 모의 수집으로400 슬롯, 성공 건너뛰기, 실패 중단/명시적 재시도,400개 freeze,
  nested subset/seed 재현성, 원본 변경 감지, 덮어쓰기 거부를 검증한다.
- 기존 GUI의 하단 잘림/긴 문장 clipping/카메라 색/창 닫기 처리와 A6 READY 안내를 수정했다.
- A6 force fade와 cleanup은 모의 transport로 검사했다. 실물 방향/범위/A6 작동은 별도 확인한다.

`HIL` Python3.9 실행 오류는 Python>=3.10 환경으로 바꿔 해결한다.
`different LIBERO package already imported`는 새 Python 프로세스에서 pinned source로 실행한다.
`pool incomplete`는 실패를 숨기지 않고 미완료 슬롯을 채운 뒤 다시 freeze한다.
레거시 pool의 `duplicate successful support slot`이나 불완전 폴더는 원본을 보존하며 원인을 확인한다.
GUI pool은 여러 성공 후보를 보존하고 마지막으로 명시적으로 확인한 후보만 채택한다.
source commit/plan이 다른 데이터는 새 pool 디렉터리에 모은다.

입력·OSC/저장 형식의 상세 설명은 [공통 연결 문서](repa-vla-position.ko.md),
확인 기록은 [검증 보고](../reports/REPA_VLA_VALIDATION_KO.md)를 참고한다.

Qt6 GUI의 실제 Spatial init0 장면과 영상→확인→다음 shot 흐름도 데스크톱에서 검증했다.
새 GUI의 문구·버튼·상태는 영어다. 상세 검증과 screenshot 위치는
[GUI 검증 보고](../reports/REPA_VLA_GUI_VALIDATION_KO.md)를 참고한다.

현재 데스크톱에서 CPU Mesa 장면 렌더링이 필요하면 실행 전에 다음을 지정한다.
robosuite1.4의 backend 선택 때문에 `glfw`가 EGL로 바뀌는 조합 대신 아래 GLX 조합을 확인했다.
설치된 simulator 코드는 수정하지 않는다.

```bash
export MUJOCO_GL=glx
export PYOPENGL_PLATFORM=glx
export LIBGL_ALWAYS_SOFTWARE=1
export __GLX_VENDOR_LIBRARY_NAME=mesa
```
