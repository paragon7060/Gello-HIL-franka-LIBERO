# Calibration 생성과 좌우 이동 검토 — 2026-11-02

이 변경은 `REPA-VLA` commit `e6f86235249aab18545e886a35a9b92cd2f16ed4` 위에서
보정 파일 선택/생성, 조작 화면 방향, 읽기 전용 관절 확인을 개선한다.

## 확인 결과와 수정

### 조작 화면의 좌우 반전

이전 GUI는 `rgb_frame()`의 `raw[::-1, ::-1]`을 그대로 표시했다.
이는 [OpenVLA-OFT의 학습 전처리](https://github.com/moojink/openvla-oft/blob/main/experiments/robot/libero/libero_utils.py)
와 같은180도 회전이며 가로 방향도 뒤집는다. robosuite의 기본 카메라는
[OpenGL image convention](https://github.com/ARISE-Initiative/robosuite/blob/v1.4.1/robosuite/macros.py)을 쓴다.

GUI는 이제 **Camera view (unmirrored)**를 기본으로 사용하여 raw 영상의 세로만 뒤집는다.
**Policy view (180 deg)**로 기존 학습용 방향을 비교할 수 있다. 이 변환은 Qt 표시층에만
적용한다. 원본 관측, 실제action, 저장된 영상의 기존180도 규칙, exporter는 바꾸지 않았다.
시작 표시 모드와 수집 중 view 변경을 `operator_gui` 이벤트에 남긴다.
작은 비대칭 픽셀 배열 검사로 가로 방향과 raw 불변을 확인했다.

이는 확인된 화면 처리 문제의 수정이다. 사용자가 느낀 모든 물리 동작 반전의 원인을
확정한 것은 아니다. GELLO 조립별 sign과 사용자/카메라의 관점도 실제로 확인해야 한다.

### 공식 GELLO 관절 수식과 native OSC

공식 upstream을 commit **`204f53a64bef89471a1e483b0f874f755fbd2d3a`**로 고정하여 비교했다.

- [Dynamixel 변환](https://github.com/wuphilipp/gello_software/blob/204f53a64bef89471a1e483b0f874f755fbd2d3a/gello/robots/dynamixel.py):
  `(encoder_radians - joint_offsets) * joint_signs`.
- [sign 설정](https://github.com/wuphilipp/gello_software/blob/204f53a64bef89471a1e483b0f874f755fbd2d3a/gello/agents/gello_agent.py):
  각 조립의 ±1 방향을 명시한다.
- [offset 계산](https://github.com/wuphilipp/gello_software/blob/204f53a64bef89471a1e483b0f874f755fbd2d3a/scripts/gello_get_offset.py):
  알려진 reference pose에서 pi/2 단위 offset을 고른다.

공식 `get_joint_state` 메서드만 추출해 가짜 driver로1000개 encoder 입력을 비교했다.
실제 driver 초기화나 upstream motor-write 코드는 실행하지 않았다.
현재 Panda 2pi branch를 맞춘 최대 오차는 **0 rad**였다.

실제 pinned LIBERO-Pro Spatial00/init0의 native controller에서도 확인했다.
X/Y/Z 각 양의0.25 입력은 해당 world 축으로+0.0125m goal을 만들었다.
7개 관절을 각각16counts 변화시켰을 때 지정 sign의 관절 변화와 FK→OSC 위치 delta가
1e-12 허용오차 안에서 일치했다. FK/goal 검사 전후 sim state도 byte-exact였다.
초기화의10 warmup 외에 physics step0, 실제 serial0, motor write0이다.
확인되지 않은 joint sign이나 action 축을 자동 반전하지 않았다.

### A6 인계 중 encoder 범위 오류

사용자 첫 시도의 `encoder counts must be in [0, 4095]` 로그를 보존해 조사했다.
signed raw position이 음수인 telemetry도 존재한다. A6 powered handoff의
`read_positions()`는 signed multi-turn 값을 그대로 반환했지만, 이후 read-only 경로는
이미 `%4096`한 한 바퀴 값을 반환하고 있었다. calibration 입력 계약이 달랐다.

handoff의 **mapping용 반환값**도 `%4096`으로 맞췄다. signed raw telemetry와 tracking/
전류/온도/watchdog 검사, 제어 목표와 motor write 제한은 그대로 보존한다.
음수 및 여러 바퀴 입력에서 입력 계약과 추가 write0을 모의 transport로 검증했다.
이 오류는 화면 좌우 반전과 별개의 문제다. 뒤 시도의 SYNC_READ timeout도 원본에
보존하며 이 변경으로 해결됐다고 주장하지 않는다. 자동 재시도는 실행하지 않았다.

## 새 GUI 흐름

- **Load calibration...**는 현재 파일의 폴더에서 열고, 전체 경로를 Connection 아래에 표시한다.
- 마지막 유효 파일을 pool의 private `gui-settings.json`에 기억한다. 명시적 CLI 경로가 우선이며,
  `GELLO_CALIBRATION`은 선택 이력이 없을 때만 로컬 fallback이다. 공개 코드에 장치 경로를 넣지 않는다.
- **New calibration**는 기존 scene의 종료/해제 후 read-only wizard를 연다.
  실제 조립의 sign을 선택하고 reference pose→gripper open→closed를25sample씩 캡처한다.
  기존 quarter-turn 알고리즘, residual<=0.60rad와 gripper span 검사를 재사용한다.
  새 timestamp JSON으로만 저장하고 기존 파일은 덮어쓰지 않는다. 저장 직후 연결한다.
  raw capture, 계산 residual, 사용 commit/source SHA를 calibration provenance에 기록한다.
- **Check joint mapping**는 encoder만 읽어 별도 preview의 로봇 관절을 렌더링하며 J1–J7 각도를 표시한다.
  초기10-step warmup 이후 physics/action0이다. Assist/Start 명령은 허용하지 않는다.
  한 관절씩 확인한 뒤 반대인 sign은 New calibration에서 재캡처해야 한다.

## 검증과 로컬 evidence

전체 **103 passed, 0 failed, 0 skipped**, Ruff와 whitespace 검사 통과.
새 검사는3단계 캡처/오류/파일 보존/선택 우선순위, Qt wizard 저장·취소 시 reader 종료,
가로 반전만 제거하는 표시, view 이벤트, read-only mapping, handoff counts를 포함한다.

실제 데스크톱에서 wizard의3단계와 실제 native mapping preview를 열어 PNG를 검토했다.
둘 다 **합성 encoder**만 사용했다. 실제 장치의 방향·범위를 검증하거나 새 보정 파일을
실물에서 캡처한 것은 아니다. 기존 사용자 실패 시도/NPZ/영상은 수정하지 않았다.

| Evidence | 로컬 위치 |
| --- | --- |
| 전체 pytest XML | `runs/calibration_mapping_validation_20261102/final-pytest.xml` |
| upstream exact SHA/source SHA와 수식/FK audit | `runs/official_gello_audit_20261102/` |
| calibration wizard PNG/validation | `runs/calibration_mapping_desktop_20261102/` |
| 실제 native scene + 합성 encoder mapping PNG/validation | `runs/mapping_native_gui_validation_20261102/` |

기계 판독 요약: [CALIBRATION_MAPPING_validation_20261102.json](CALIBRATION_MAPPING_validation_20261102.json).
원본/PNG/port/calibration/개인 경로는 Git에서 제외한다.
