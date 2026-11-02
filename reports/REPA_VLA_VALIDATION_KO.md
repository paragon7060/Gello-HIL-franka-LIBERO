# GELLO REPA-VLA 구현·검증 기록

2026-11-02 로컬 장치 기준이다. 기준 commit은
`26be20212ff0709668b776ebefabe7da1809aa73`, 구현 branch는 `REPA-VLA`다.
이 CLVLA 공통 checkout에서 작업했으며 원래 UnlearningVLA GELLO checkout은 변경하지 않았다.

## 실제 입력 연결

`openvla-oft` Python3.10 환경을 사용했다. `HIL` 환경의 Python3.9는 패키지의
Python>=3.10 요구조건과 맞지 않았다. 환경 자체의 패키지를 교체하지 않았다.

- PING에서 ID1–7 model1200, ID8 model1190, firmware53 및 error0을 읽었다.
- 유한 read-only 입력20회/5Hz와 실제 데스크톱 GUI 입력10회/5Hz가 오류 없이 완료됐다.
- 8개 motor의 torque_enable readback은 전부0이었다. motor register 쓰기는0이다.
- 사용자 성공 capture의 D31-v15 calibration을 명시적으로 가져왔다. 원본 SHA를 보존했다.
- 현재 장치 조립의 calibration 성공이나 전체 가동범위 방향 확인을 의미하지 않는다.

원본 evidence는 `runs/device_check_20261102/input.json`, `gui-input.json`,
`calibration.json`, `encoder-preview.png`다. Git에는 넣지 않는다.

## 구현

같은 수집기에 native Position OSC_POSE 연결, 12task×3draw×5shot case 선택,
공식 init0..14/BDDL instruction/10 warmup 경계, 연속 native success 확인을 추가했다.
무단 XY 이동이나 실패 자동 재수집, 과거 N/W run 수정은 없다.

source commit/branch/dirty/source SHA, calibration snapshot, task/scene 지문,
raw 양쪽 RGB/proprio/sim state, 실제 action, encoder, 시간 및 pause/re-anchor 이벤트를 저장한다.
기존 native NPZ와 OAT 형식 HDF5 exporter를 추가했다. HDF5 export는 현재 full-data
OAT loader의50demo 계약을 충족한다는 뜻이 아니며 few-shot manifest 연결이 별도로 필요하다.

A6 모터 보조는 기존 `--apply` 및 interactive authorization 경계를 통해 Position에 연결했다.
원래 current/gain/waypoint/tracking/watchdog 제한은 그대로다.
이번 실제 실행은 사용자 최신 지시에 따라 read-only 범위였다.

## 검증

- 기존 단위 테스트52개는 수정 전 통과했다.
- 최종 실제 사용 환경에서 **83 passed, 0 failed, 0 skipped**.
- A6 모의 transport: force가 증가하지 않는2초 fade, gripper torque-off,
  tracking cap 이탈 시 early release, 과열/KeyboardInterrupt/clutch/timeout cleanup,
  single bus 소유권 및 A6 GUI 전환·창 닫기를 확인했다.
- 실제 MuJoCo shadow FK가 live qpos/qvel/time을 변경하지 않는지 확인했다.
- 수집기의 pause/re-anchor/중단·순간 success 실패 보존, T+1/T 정렬,
  native NPZ 및 HDF5 내보내기와 180개 support case 분리를 검증했다.
- 전체 Python 정적 검사와 git diff whitespace 검사가 통과했다.

## 전체40과제 ×10-demo 확장

기존 12-task plan을 보존하면서 별도 `libero_pro_position40_pool10.json`을 추가했다.
공식 Spatial/Object/Goal/Long 각10과제와 native init0..9를 등록하여400개 슬롯을 만든다.
interactive A6 수집은 성공 슬롯을 건너뛰고 첫 실패/중단에서 멈춘다. 실패 재시도는
별도 `--retry-failed`가 필요하며 매 시도의 원래 터미널/GUI Enter 경계를 유지한다.
400/400 원본 검사 후 SHA를 고정하고, 같은 seed의1·3·5·10-shot이 포함 관계를 갖는
선택 manifest를 만든다. 실패/원본 checksum 변경/중복 성공/덮어쓰기를 거부한다.

추가 모의 테스트5개에서400 슬롯 분리, resume/failure gate,400개 freeze,
nested subset/seed 재현성/변조 감지를 검증했다. 실제 학습 job은 실행하지 않았다.
공식 Pro source를 별도 고정 checkout으로 준비했고, 공식 dataset revision
`c86fc3b8293185a6f373677018ff3e37f8391602`의 해당 BDDL/init80개만 내려받았다.
checksum과40개 task identity 및 과제별 초기 상태50개를 실제 benchmark API로 확인했다.
이는 모터나 physics step을 실행하지 않는 자산 검사다.

별도 CPU software EGL smoke에서 `object_07`/init0의 실제 Pro 장면을 열었다.
공식10-step warmup, native instruction, 실제 OSC_POSE input/output 범위와 독립 FK 생성이
통과했고 카메라 PNG도 검토했다. collection action0/모터 write0이며 성공 시연에 넣지 않는다.
Numba cache가 conda 설치 경로에 쓰기를 시도하던 실행 문제는 `NUMBA_CACHE_DIR`를
checkout의 `runs/numba-cache`로 지정하여 해결했다. source나 conda 패키지는 수정하지 않았다.
근거는 `runs/position40-scene-smoke/validation.json`과 카메라 PNG다.

실제 데스크톱에서 collection 저해상도 화면, ACTIVE release 화면, alignment READY
화면 세 종류를 합성 카메라로 열고 window visible 및 닫힌 창의 abort 처리를 확인했다.
PNG도 직접 검토했다. 기존 하단 잘림, 긴 문장 clipping, alignment 색상 및 READY 안내를 수정했다.
결과는 `runs/gui_validation_20261102_v2/validation.json`과 PNG3개에 있다.

기계 판독 종합 기록: `REPA_VLA_validation_20261102.json`.
테스트 XML: `runs/final_validation_20261102/pytest.xml`.

## 아직 실행하지 않은 범위

실제 motor assistance/A6, 조립별 재보정, 실제 LIBERO-Pro Position 시연 수집은 실행하지 않았다.
로컬의 기존 LIBERO는 원본 버전이며, 별도 pinned Pro checkout과 공식 자산을 준비했다.
사용자 조작을 동반한 보정·장면 확인·제한된 A6 실물 검증이
후속 단계다. 재개한 heartbeat나 새 반복 자동화는 없다.

최신 사용법은 `docs/position40-pool10.ko.md`를 따른다.
