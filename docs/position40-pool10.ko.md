# Position 전체 40과제 × 10-demo 수집과 k-shot 실험

목표는 **LIBERO-Pro Position 40과제마다 성공 시연 10개, 총 400개**를 GELLO 모터 보조
방식으로 수집하는 것이다. 원본 400개를 고정한 뒤 같은 데이터에서 1·3·5·10-shot
학습용 목록을 만든다. GELLO 입력·보정·GUI·기록 코드는 이 저장소 `REPA-VLA` 브랜치에서
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
      .collection.lock                  # 동일 pool의 중복 수집 방지
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

Python>=3.10, NumPy<2, OpenCV GUI와 LIBERO/robosuite 1.4.x가 필요하다.
현재 확인한 `HIL` 환경은 Python3.9여서 이 패키지를 실행할 때 `openvla-oft`를 사용했다.
Numba cache도 이 checkout의 `runs/`에 두므로 conda 설치 경로를 수정하지 않는다.
새 환경에서 패키지 설치가 필요하면 `python -m pip install -e '.[export]'`를 사용한다.
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

## 5. 모터 보조로 수집

[SAFETY.md](../SAFETY.md)와 [기존 A6 절차](assisted.md)를 따른다.
arm current100mA, P750, guide20counts, tracking40counts, 5Hz/watchdog 제한과
gripper torque-off를 그대로 사용한다. `--apply` 뒤에도 매 시도 터미널 Enter와
GUI의 READY/Enter가 필요하다. **GELLO를 지지하고 전원 차단에 접근할 수 있는 사람이 조작한다.**

먼저 한 시도만 진행해 실제 장면·보정·A6 handoff를 확인한다.

```bash
python -m gello_hil.cli init-position-pool --dataset-root "$POOL"
python -m gello_hil.cli collect-position-pool \
  --port "$GELLO_PORT" --baudrate 57600 \
  --calibration "$SESSION/calibration.json" \
  --libero-source "$LIBERO_PRO_ROOT" --libero-config-path "$LIBERO_CONFIG_PATH" \
  --dataset-root "$POOL" --task-key object_07 --max-attempts 1 --apply
```

| 단계/키 | 수행 내용 |
| --- | --- |
| 터미널 Enter | 해당 시도에 대한 제한된 모터 보조 허용. 다른 문자열을 입력하면 취소 |
| alignment 창 Enter | 기존 A6 preflight/정렬 절차 시작 |
| READY에서 Enter | 같은 장면에서 제어 시작, 처음 action은 neutral, 기존 force를 2초간 줄임 |
| ACTIVE에서 Space | force 해제와 clutch/pause, physics와 active budget 정지 |
| PAUSED에서 Enter | 현재 GELLO/로봇 자세를 다시 anchor하고 재개; 보조 force를 다시 켜지 않음 |
| q / ESC / 창 닫기 | 해제·중단·원본 보존, 전체 수집 큐 종료 |

이미 충분히 가까운 자세이면 force를 켜지 않는 기존 passive start 경로를 사용한다.
따라서 `motor_assistance_requested=true`는 같은 A6 프로토콜을 사용했다는 뜻이며,
모든 시연에서 실제 전류를 인가했다는 뜻은 아니다. 실제 write/fade telemetry는 별도 보존한다.

실물 확인 후 전체 수집은 다음 명령이다. 한 세션에서 성공 10개까지만 하려면
`--max-attempts 10`을 추가한다. 특정 과제만 하려면 `--task-key object_07` 등을 추가한다.

```bash
python -m gello_hil.cli collect-position-pool \
  --port "$GELLO_PORT" --baudrate 57600 \
  --calibration "$SESSION/calibration.json" \
  --libero-source "$LIBERO_PRO_ROOT" --libero-config-path "$LIBERO_CONFIG_PATH" \
  --dataset-root "$POOL" --apply
```

성공 슬롯은 다음 실행에서 건너뛴다. 성공 판정은 최소20 active-step 연속 native success다.
장면/목표/XY/instruction을 수정하지 않고 실제7D OSC_POSE action을 기록한다.
순간 success 뒤 최종 false가 되면 성공 시연에 넣지 않는다.

실패·중단·오류가 발생하면 그 시도를 저장하고 큐를 멈춘다. 다음 실행도 같은 실패 슬롯에서
멈춘다. 원인을 확인한 뒤 같은 명령에 `--retry-failed`를 추가하면 해당 슬롯에 새 시도
한 번을 허용한다. 새 실패가 나면 즉시 다시 멈춘다. 이전 실패/영상/이벤트는 지우지 않는다.
성공한 동일 슬롯이 두 개 있거나 불완전한 시도 폴더가 있으면 임의로 선택하지 않고 거부한다.

진행률은 하드웨어를 열지 않고 확인한다. 주기적 watcher/자동화는 만들지 않는다.

```bash
python -m gello_hil.cli position-pool-status --dataset-root "$POOL"
```

## 6. 400개 완료 후 고정과 k-shot 선택

```bash
python -m gello_hil.cli freeze-position-pool --dataset-root "$POOL"
python -m gello_hil.cli select-shots \
  --pool-manifest "$POOL/pool-manifest.json" \
  --shots 1 3 5 10 --seeds 42 43 44 \
  --output-dir runs/position40-subsets
```

400개가 모두 성공해야 freeze가 가능하다. 각 원본의 실제 action/관측 정렬, calibration
snapshot, source commit과 파일 SHA, video 존재를 확인하고 SHA256을 고정한다.
freeze 후 같은 pool에 시연을 추가하지 않는다. 선택 전에 원본 checksum을 다시 확인한다.

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
변환은 `oat-hdf5`를 사용한다. 성공 한 시도의 변환 예시는 다음과 같다.

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
`duplicate successful support slot`이나 불완전 폴더는 원본을 보존하며 원인을 확인한다.
source commit/plan이 다른 데이터는 새 pool 디렉터리에 모은다.

입력·OSC/저장 형식의 상세 설명은 [공통 연결 문서](repa-vla-position.ko.md),
확인 기록은 [검증 보고](../reports/REPA_VLA_VALIDATION_KO.md)를 참고한다.
