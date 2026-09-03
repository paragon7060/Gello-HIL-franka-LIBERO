# 한국어 사용법

## 전체 순서

```bash
export GELLO_PORT=/dev/serial/by-id/usb-FTDI_REPLACE_ME-if00-port0
export LIBERO_CONFIG_PATH=/absolute/path/to/libero-config-directory

gello-hil probe --port "$GELLO_PORT"

gello-hil calibrate \
  --port "$GELLO_PORT" \
  --joint-signs 1 1 1 -1 1 -1 1 \
  --output calibration.json

gello-hil mapping-check \
  --port "$GELLO_PORT" \
  --calibration calibration.json \
  --libero-config-path "$LIBERO_CONFIG_PATH"

gello-hil collect \
  --port "$GELLO_PORT" \
  --calibration calibration.json \
  --libero-config-path "$LIBERO_CONFIG_PATH" \
  --suite libero_object \
  --task-id 4 \
  --state-index 0 \
  --mapping absolute \
  --max-active-steps 1000
```

## Calibration 자세

화면의 팔 끝 위치만 비슷하게 만들면 안 됩니다. 각 관절을 아래 값에
개별적으로 맞춘 뒤 Enter를 누릅니다.

```text
J1=0, J2=0, J3=0, J4=-90도, J5=0, J6=+90도, J7=0
```

그 다음 gripper 완전 open과 완전 close를 각각 캡처합니다. 조립 방향이
다르면 sign도 달라질 수 있습니다. 특정 관절만 화면과 정반대로 움직이면
그 관절 sign만 뒤집고 calibration부터 다시 수행합니다.

## Collection 키

- 시작 화면은 항상 `PAUSED`입니다.
- `Enter`: 현재 GELLO count와 simulator follower joint를 캡처하고 시작/재시작
- `Space`: active 상태에서 clutch/pause
- `q`: 즉시 abort하되 attempt artifact는 저장

PAUSED 중에는 시뮬레이터 step과 1000-step active budget이 전혀 증가하지
않습니다. 성공 predicate가 만족되면 자동 종료 및 저장됩니다.

`absolute`는 현재 GELLO joint를 Panda 좌표의 목표로 항상 사용합니다.
따라서 시작 전에 물리 GELLO와 simulation Panda 자세를 맞추는 용도입니다.
`relative-clutch`는 Enter 순간의 simulator 자세를 기준으로 이후 GELLO
변화량만 적용하므로, GELLO를 쉬었다가 다른 편한 자세에서 재시작할 수
있고 재시작 직후 arm action은 0입니다.

기본 `collect`는 GELLO motor torque를 켜지 않습니다.

## 선택 기능: 모터 보조 후 수집 (v0.2, 실험적)

[안전 문서](../SAFETY.md)와 [상세 assisted guide](assisted.md)를 먼저 읽으세요.
이식본은 새로운 물리 안전 인증을 받은 기능이 아닙니다. 지원 모터/전원/배선과
기존 mode 5 설정이 맞아야 하며, 팔을 지지하고 물리 전원 차단을 준비해야 합니다.

```bash
gello-hil assisted-collect \
  --port "$GELLO_PORT" \
  --calibration calibration.json \
  --libero-config-path "$LIBERO_CONFIG_PATH" \
  --suite libero_object --task-id 4 --state-index 0 \
  --max-active-steps 1000 --apply
```

CLI Enter 승인 → GUI PAUSED에서 Enter → 손으로 목표 자세에 맞추기 →
READY에서 Enter → **즉시 제어 시작과 동시에 2초 동안 힘을 줄입니다**.
두 번째 Enter나 해제 후 정렬 검사는 없습니다. 정렬 허용 오차는 최대/RMS
모두 `0.60 rad`입니다. Enter의 실제 counts/Panda를 기준으로 잡아 첫 action은 neutral이며,
이후 움직임을 추종합니다. 힘/게인을 높이지 않고 gripper torque도 켜지 않습니다.

보조/힘 해제 도중 Space는 해제·일시정지, q는 해제·중단입니다. 이미 가까운 자세로
passive 통과하면 모터를 새로 켜지 않으므로 powered transition도 생략합니다.
정렬/PAUSED 중에는 sim/ACTIVE budget이 증가하지 않습니다. 초기 live release의 action은
budget에 포함되고 모터 보조 구간으로 표시됩니다. Torque-off 확인 후 read-only로 전환합니다.
Clutch Enter마다 현재 자세를 다시 기준으로 잡습니다.

출력은 `runs/assisted-a6/<UTC>/`로 분리되며 성공/실패/중단을 보존합니다.
재시도하려면 같은 명령을 다시 실행하세요. 공개본은 일반 LIBERO initial state를
사용하며, 연구 프로젝트의 비공개 실패 상태·campaign 관리자·통합 calibration
GUI·성공 후 재수집 키는 포함하지 않습니다. Calibration은 앞의 CLI로 별도 수행합니다.

현재 A6 변경은 사용자 요청으로 추가 검증 및 물리 실행 없이 작성되었습니다.
