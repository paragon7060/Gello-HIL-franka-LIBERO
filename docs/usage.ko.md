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

기본 공개 버전은 GELLO motor torque를 켜지 않습니다. motor holding/homing
기능은 servo, 전원, current, watchdog에 대한 별도 안전 검증 없이는 사용하지
마세요.
