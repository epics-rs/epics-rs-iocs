# ur-robot 드라이버 개선 구현 계획 (2026-08-20)

근거 조사: `drivers/ur-robot` + `iocs/ur-robot` 전수 (결함 47건), C 원본
`epics-modules/urRobot` 전체 표면, `~/work/cspace` (MoveIt 포트) 역량,
`~/work/robot-sample-changer` (UR3e 샘플체인저) 실사용 요구.

C 조사 기준 커밋: 093785d. 이후 upstream이 fe89715까지 7커밋 전진
(2026-08-20 pull 확인) — 재동기화가 Phase P0으로 선행된다.

배포 사실이 우선순위를 정한다: robot-sample-changer에서 이 IOC는
dashboard.db + rtde_receive.db만 싣는 **모니터링 IOC**다 (control/io/gripper
포트는 robot-sequencer가 독점). 따라서 모니터링 절반의 진실성 → 정확성 결함
→ control 절반 확장 순.

계획 수립 시 확정한 사실:

- `PortDriverBase::set_param_status(reason, addr, status, alarm_status,
  alarm_severity)` 존재 (asyn-rs port.rs:1151) — 알람 전파 즉시 구현 가능.
- epics-base-rs 레코드 목록에 `luascript` 없음 — C의 waypoint/path 시스템은
  그대로 이식 불가. sseq/calcout/busy/printf/transform/scalcout은 있음.
- `gripper.rs:408` activate는 이미 `!is_active` 가드 보유 (ur_rtde
  robotiq_gripper.cpp:123과 동일) — 멱등성은 라이브러리 레벨에서 이미 성립,
  남은 것은 PINI 경주(B3)뿐.

공통 규칙: 커밋 단위 = finding 하나. 매 커밋 `cargo fmt --all` + `clippy -p
ur-robot -p ur-robot-ioc --all-targets -- -D warnings` + `nextest` 동일
scope; push 전 workspace 전체. C 대비 의도적 이탈은 전부 §이탈 등록부에.

## Phase P0 — upstream 재동기화 (093785d → fe89715)

우리 포트는 093785d 기준이라 아래 세 upstream 변경을 역이식해야 한다.
write 핸들러·db를 건드리는 B3/B4보다 먼저 수행 (같은 코드를 두 번 손대지
않기 위해). RTDE 와이어 코덱은 무변경 — golden 테스트 영향 없음.

### P0-1. TCP 회전 표현 정정 (upstream c02490e, fe89715)

- UR의 TCP pose 회전 3요소는 본래 rotation vector(Rodrigues, rad)인데, 옛
  C(=우리 포트)는 이를 180/π 스케일해 "Roll/Pitch/Yaw(deg)"로 오표기했다.
  upstream이 원시 rad 통과로 정정하고 문서화 (docs/usage.md "TCP Pose" 절).
- 역이식:
  - `drivers/receive.rs` publish(): pose[3..5] rad→deg 변환 제거.
  - `drivers/control.rs` write_float64: POSE_CMD·JOG_SPEED의 addr>=3
    deg→rad 변환 제거 (TCP_OFFSET은 원래 rad 통과 — 무변경).
  - db 개명: `Receive:PoseRoll/Pitch/Yaw` → `PoseRx/Ry/Rz`,
    `Control:PoseRoll/Pitch/YawCmd` → `PoseRx/Ry/RzCmd`,
    `Control:TCPOffset_Roll/Pitch/Yaw` → `_Rx/_Ry/_Rz`,
    `Control:JogSpeedRoll/Pitch/Yaw` → `JogSpeedRx/Ry/Rz`(rad/s).
    EGU 추가: 관절 deg, xyz mm, 회전 rad (upstream rtde_receive.db 동일).
- **client-visible**: robot-sample-changer가 우리 rtde_receive.db를 로드
  하므로 `Robot:UR:Receive:PoseRoll` 등이 개명됨. 배포 GUI는
  ActualTCPForce(배열)만 읽어 무영향이나, 개명 목록을 배포측에 통지.
- 테스트: 배선 기계검증(B1 테스트를 이 시점으로 앞당겨 도입 가능), 단위
  변환 부재 확인 (publish 경계값).

### P0-2. busy 레코드 전환 (upstream 558b98f)

- `Control:moveJ/moveL` bo→busy; 드라이버가 수락 시 param=1, 거부(안전한계
  /모션 진행 중) 시 param=0 즉시 반영, `set_motion_task_done`에서 0.
  `caput -c -w`가 모션 완료까지 대기 가능해짐. epics-base-rs에 busy 레코드
  존재 확인(records/busy.rs).
- `RobotiqGripper:Open/Close` bo→busy + 사전검사 (이미 열림/외측 정지 →
  경고 후 busy 즉시 해제), `clear_busy_calc`+`clear_busy` dfanout 추가,
  핸들러는 value==1에만 동작.
- B2(typed 거부)와의 관계: upstream busy는 거부를 *보이게*만 한다(비지
  해제). B2는 여기에 거부 사유(CommandRefused)와 알람을 얹는다 — 중복
  아님, 선행 관계 (P0-2 → B2).

### P0-3. AutoMove 제거 + CmdU/Tweak 재구성 (upstream 61f08d0)

- 제거: `AutoMoveJ/L`, `auto_moveJ/L_calc`, `auto_sync_automovej/l`.
  `reset_after_async_move` → `sync_after_async_move`.
- 신설: `J1..J6CmdU`, `PoseX/Y/ZCmdU` (타깃 설정 + moveJ/L FLNK, GUI용).
  회전축(Rx/Ry/Rz)은 rotation vector라 CmdU/Tweak 미제공 (upstream 문서
  근거). Tweak은 TweakFwd/Rev가 calcout으로 직접 CmdU를 미는 구조로 단순화.
- `rtde_control_settings.req`(우리 C2b에서 이식 예정분)도 upstream 신판
  기준으로. `ur_soft_motor.substitutions`의 회전축 변경도 반영 대상 확인.

### P0 검증

- upstream 미수정 확인 완료: `TargetJointMoments` 오배선은 fe89715에도
  잔존 → B1과 upstream PR 제안 유효.
- P0 완료 후 fe89715를 새 parity 기준 커밋으로 이 문서에 기록.

## Phase A — 모니터링 진실성

### A1. RTDE 스트림 staleness 감지 + receive 자동 재연결

- `stream.rs`: reader가 매 데이터 패키지 수신 시 `last_package: Instant`
  기록; `StateStream::last_update()` / `is_alive(threshold)` 노출. 소켓이
  열린 채 데이터만 끊기는 경우(stream.rs:96-111)가 현재는 영구 침묵.
- 임계값: 명명 상수 `STALE_AFTER = 1 s` (기본 출력 주파수 125 Hz 기준 충분한
  마진; 새 설정 노브는 추가하지 않음). 구현 시 session의 output-setup
  frequency 지정 여부 확인.
- `drivers/receive.rs` poll: `connected==false` 또는 stale → `IS_CONNECTED=0`
  \+ A2 알람 + poll 스레드에서 backoff 재연결 (1 s → 2 s → 5 s cap, 각 시도는
  기존 connect/FIRST_STATE_TIMEOUT 바운드). 성공 시 알람 해제.
- control은 자동 재연결하지 않음 (스크립트 재업로드 부작용; C도 안 함).
  staleness는 IS_CONNECTED=0 반영만.
- 테스트(경계 기준): 패킷 정지 → stale 전이 / 소켓 단절 → 재연결 성공 /
  연속 실패 → backoff cap / connected-but-stale vs disconnected 구분.

### A2. 링크 단절·stale 시 COMM 알람 전파

- Invariant: 연결 상태 알람의 단일 소유자는 각 드라이버의 poll 스레드.
  write 경로는 알람을 만지지 않는다.
- 단절/stale 전이: 전체 readback param에 `set_param_status(.., Error,
  COMM, INVALID)` + 전 addr flush (asyn-rs PR #79 다중 addr flush 경로).
  복구: (Success, NO_ALARM) + 값 재게시. InterruptValue가
  aux_status/alarm_status/alarm_severity를 이미 운반하므로 레코드 SEVR까지
  도달 (interrupt.rs 확인 완료).
- receive 먼저, dashboard/gripper/control 동일 패턴 확산 (dashboard는 예외
  시 IS_CONNECTED=0까지는 이미 함).
- 테스트: 가짜 서버 단절 → 구독 레코드 SEVR=INVALID, 복구 → NO_ALARM.

### A3. RobotMode/SafetyMode/RuntimeState mbbi화 + safety severity [사용자 결정 필요]

- rtde_receive.db의 세 ai → mbbi (RVAL 매핑; RobotMode -1은 ZRVL이 DBF_LONG
  이라 표현 가능). 상태별 SEVR: C docs pv_reference_guide.md의 값 표 그대로,
  PROTECTIVE/EMERGENCY → MAJOR, REDUCED → MINOR 등.
- `SafetyStatusBits`는 비트마스크라 mbbi 부적합 → `Receive:SafetyOk`
  (calcout `A=1` → bi, ZSV MAJOR) 신설.
- **client-visible**: .VAL 의미가 원시값 → 상태 인덱스로 변경 (RVAL은 원시값
  유지). 배포 GUI는 이 레코드들을 읽지 않음(ActualTCPForce만) — 그래도 record
  type 변경이므로 승인 후 진행.
- 테스트: db 파싱 테스트에 상태 문자열·SEVR 존재 검증 추가. epics-base-rs
  mbbi + DTYP asynInt32 조합 동작 확인.

### A4. dashboard db 읽기전용 분리

- `db/dashboard.db` = 상태 절반 + Connect/Disconnect (모니터링 IOC도 연결
  관리는 필요). 신설 `db/dashboard_ctrl.db` = Play/Stop/Pause/Shutdown/
  ClosePopup/CloseSafetyPopup/PowerOn/PowerOff/BrakeRelease/
  UnlockProtectiveStop/RestartSafety/Popup/LoadURP.
- 레코드가 없으면 CA/PVA 어느 경로로도 쓰기 불가 — 권한 체크가 아니라 표면
  제거(구조적). 우리 st.cmd는 두 파일 다 로드해 현행 동일.
- 배포 효과: robot-sample-changer의 ur_monitor_ioc st.cmd가 dashboard.db만
  로드하면 PowerOff/Shutdown/UnlockProtectiveStop CA-쓰기 노출이 사라짐.
  (그 repo의 st.cmd 갱신은 별도 확인 항목.)
- 테스트: command_records.rs의 db 파일 카운트 갱신 + ctrl 레코드가 ctrl
  파일에만 존재함을 검증.

## Phase B — 정확성

### B1. TargetJointMoments 오배선 수정 (즉시 가능)

- rtde_receive.db:391 `TARGET_JOINT_CURRENTS` → `TARGET_JOINT_MOMENTS`,
  DESC 수정. C 원본 rtde_receive.db:389의 버그를 그대로 이식한 것 —
  doc/upstream-c-defects.md에 기록, upstream PR 후보.
- 가족 폐쇄: 레코드명 → param 명 대응을 기계 검증하는 테스트를 신설해 모든
  Target*/Actual* waveform의 배선을 전수 고정 (같은 유형 오배선 재발 차단).

### B2. 거부된 명령의 typed 전파

- `control.rs::send_command` → `Result<()>`; 거부는
  `UrError::CommandRefused { reason: NotRunning | SafetyStop | Timeout }`.
  `send_query`는 진짜 답(bool)과 질의 자체의 거부를 분리 — 거부는 Err.
- 효과: driver 9곳의 `Ok(false)` 폐기가 컴파일 단위로 불가능해짐(구조적).
  Err → asynError → A2 인프라로 레코드 SEVR.
- `drive_motion`: 거부된 move는 WaitingMotion 진입 금지 —
  `motion_task_done` + ERROR trace. 현재는 거부된 moveJ가 다음 poll에
  ASYNC_MOVE_DONE=1로 "완료"됨(drivers/control.rs:776-816). C보다 정확 —
  이탈 등록.
- `is_steady` 실패=false(#19), `is_pose_within_safety_limits`의 "한계 밖" vs
  "명령 거부" 혼동(#21)이 이 타입 변화로 함께 닫힘.
- 테스트: 가짜 컨트롤 서버 거부 응답 → Err 종별; drive_motion 상태 전이
  경계(거부/성공/타임아웃).

### B3. 연결-소유 설정 적용 (PINI 경주의 구조적 폐쇄)

- Invariant: 장치에 닿는 설정은 연결 소유자가 (재)연결 성립 시 param
  캐시에서 적용한다.
- 현재: `AfterScanInit` 직후 PINI가 poll 스레드와 경주 (ioc_ready 게이트 +
  ioc_app.rs:974-993 순서) → AUTO_ACTIVATE 무효, MIN/MAX_POS 미적용,
  TCPOffset 유실, control은 부팅 auto-connect가 구조적으로 불가
  (drivers/control.rs:351-366의 receive/dashboard 선행조건이 항상 미충족).
- 구현:
  - gripper connect 성공 시: MIN/MAX_POSITION → set_native_position_range,
    POSITION_UNIT, SET_SPEED/FORCE, AUTO_ACTIVATE 보류분 적용.
  - control connect 성공 시: TCP_OFFSET → set_tcp.
  - write 핸들러: 미연결 시 param 캐시에 저장하고 asynSuccess (C의 즉시
    실패 대신 — 이탈 등록). 로컬-전용 파라미터(speeds/blends/timeout/
    JogSpeed)는 connected 게이트 밖으로(#25/#26); write_float64의 addr
    범위검사를 param 종별로(#27).
- 테스트: 부팅 시나리오(미연결 → PINI 쓰기 → 연결 → 적용), 재연결 시
  재적용, 연결 도중 쓰기 경계.

### B4. write 경로 flush (C parity 복원)

- C asynPortDriver의 기본 writeInt32/writeFloat64는 끝에서
  callParamCallbacks(addr,addr) 호출 (asynPortDriver.cpp:2031 등). 우리
  오버라이드들이 이를 빠뜨려 JOGGING/CUSTOM_SCRIPT_* 등이 다음 poll까지
  레코드에 못 감; io 포트는 poll이 없어 영영 미통지.
- anchor: `fn write_int32|write_float64|write_octet` 전 사이트 rg 열거 후
  일괄 — 핸들러 끝 `call_param_callbacks` (다중 addr 세팅 시 PR #79 계약).
- B3 뒤에 수행 (write 핸들러를 한 번만 손대기 위해).
- 테스트: 각 드라이버 write → interrupt 콜백 도달.

### B5. gripper 프로토콜 라인 프레이밍

- `gripper.rs::transact`(:280-299)를 '\n' 단위 read 루프로 (타임아웃 유지);
  `set_vars`는 SET당 1줄 ack 파싱. 분할("ac"+"k\n")·병합("ack\nack\n") 오파싱
  제거. ur_rtde 자체도 단발 recv라 upstream 동종 결함 — wire-correctness
  수정으로 기록.
- 테스트: 분할/병합 전송 경계 (기존 loopback 인프라).

### B6. control 락 범위 감사 (조사 후 결정, 코드 변경은 조건부)

- C도 poll/write가 같은 포트 락에서 장치 I/O를 한다(parity). 우리가 C를
  초과하는 보유 구간만 확인·축소 (예: C는 custom script 실행 중 isSteady를
  건너뜀 — 이식 여부 확인). 측정 근거 없는 재구조화는 하지 않음. 산출물은
  doc 노트, 필요 시 후속 항목화.

## Phase C — 확장

### C1. GraspState

- gripper poll에서 장치 사실만으로 유도하는 `GRASP_STATE` param + mbbi:
  0 UNKNOWN / 1 INACTIVE / 2 MOVING / 3 OPEN / 4 CLOSED_EMPTY /
  5 HOLDING_INNER / 6 HOLDING_OUTER.
  MISGRIP/DROPPED는 명령 문맥이 필요해 의도적으로 제외 (sequencer 몫) —
  robot-sample-changer 위시리스트(vision_inspection_plan.md §0.2)의 IOC측
  절반.
- 테스트: (is_active, move_status, position) 경계표 전수.

### C2. C-parity 저비용 완성

- C2a. jog db 로드 복원 (st.cmd:37 주석 해제). 선행: B3(JogSpeed PINI),
  B4(JOGGING flush). 1 s 워치독 레코드는 db에 이미 있음.
- C2b. autosave .req 이식 (C의 `CustomScriptPath` 오기는 `CustomScriptFile`로
  수정하며 이식 — upstream 버그 기록).
- C2c. `calibrate_gripper` Rust bin (C PROD 대응): connect → activate →
  auto_calibrate → native range 출력. MIN_POS/MAX_POS 산출 플로우 완성.
- C2d. ControlInterface 누락 명령 (라이브러리 표면만, PV 없음 = C 동일):
  1차 servoJ/servo_stop/speedJ/watchdog-kick (C3b 의존성), 2차
  forceMode/zero_ftsensor/getInverseKin/moveUntilContact. input recipe와
  업로드 스크립트는 이미 존재 — Rust 명령 표면만 추가. golden 인코딩
  테스트 확장.
- C2e. 운영성: st.cmd의 dbLoadRecords 경로 $(URROBOT) 매크로화 + PREFIX/IP
  env 인자화 (main.rs:75의 URROBOT 기본값을 실제로 사용), RTDEInOutConfig
  poll 인자 poll_arg 검증 통일(#43), registry 중복 포트명 등록 에러화(#42).

### C3. cspace 연동 [사용자 결정 게이트]

배포 참고: robot-sample-changer에서는 sequencer가 planning을 소유하므로 이
기능의 수요자는 독립 실행 IOC 사용자(C 모듈의 원래 청중)다.

- C3a (1단계, 저위험): feature `cspace` (default off), URDF/SRDF 경로
  iocsh 인자. `Control:TargetValid`/`TargetValidMsg` — moveJ/L 대상의 IOC 내
  joint-limit + self-collision 검사 (`PlanningScene::is_state_valid`,
  동기 호출·ROS/tokio 불필요 확인 완료). 로봇 왕복
  `is*WithinSafetyLimits`(거부와 한계초과를 구분 못 함)의 선행 게이트.
  - 결정 필요: 의존 형태 (git dep `physwkim/cspace` vs path). 해석해 UR IK
    없음은 무관 (검증은 FK/충돌만 사용).
  - SRDF 함정: fixed-joint 체인 그룹이 빈 링크셋으로 조용히 통과 (kodex
    기록) — 로드 시 빈 그룹 감지·에러.
  - 자산: robot-sample-changer `model/`의 ur3e URDF/SRDF/collision STL 차용
    가능 (fixtures 복사 여부 결정).
- C3b (2단계, 이 계획에서는 설계까지만): named-pose planned move —
  cspace RRT-Connect + TOTG → servoJ 스트리밍 실행 (C2d 선행). C의 Lua
  waypoint/path 시스템은 luascript 레코드 부재로 이식 자체가 불가하므로 이
  경로가 waypoint 기능의 대체안. PV 표면 설계서 별도 제출 후 승인 시 착수.
  포즈 표현은 P0-1의 rotation vector 규약을 따른다 (cspace Isometry3 ↔
  rotvec 변환은 nalgebra 축각 API로 직결).

## 이탈(deviation) 등록부

구현하며 doc/port-parity-defects.md 옆에 `ur-robot` 이탈 표 신설. 등록 예정:
receive 자동 재연결, COMM 알람 전파, mbbi화(A3), 거부-즉-에러(B2),
연결-소유 설정 적용 + 미연결 쓰기 캐시(B3), gripper 라인 프레이밍(B5),
GraspState(C1), dashboard db 분리(A4). 각 행: C 동작 / 우리 동작 / 근거.

## 검증 체계

- 단위: 항목별 명시 테스트 (가짜 TCP 서버 인프라는 session/dashboard/gripper
  테스트에 기존).
- 레코드-파라미터 배선 기계검증(B1 도입)을 전 db로 확대.
- 통합: URSim 접근 가능 여부 확인 (robot-sample-changer가 192.168.56.101
  URSim 리허설 환경 보유) — 가능 시 receive/dashboard/gripper 스모크, 불가
  항목은 보고서에 미검증으로 명기.

## 순서·의존성

```
P0-1 ─→ P0-2 ─→ P0-3   (최우선; B3/B4/C2b가 만질 코드·db의 기준선)
A1 ─→ A2          A3, A4 독립
B1 독립 (즉시)
P0-2 ─→ B2 ─→ B3 ─→ B4    B5 독립     B6 조사
C1 독립
C2a ← B3,B4       C2b ← P0-3     C2c/C2e 독립     C2d ─→ C3b
C3a 독립 (결정 게이트)
```

규모: Phase P0 3커밋, A 4커밋, B 6커밋, C 8–10커밋 예상.

## 사용자 결정 필요

1. A3 mbbi화 — .VAL 의미가 바뀌는 client-visible 변경. 진행 여부.
2. A4 이후 robot-sample-changer deploy/ur_monitor_ioc/st.cmd 갱신을 이
   작업에 포함할지.
3. C3 cspace 의존 형태(git vs path)와 C3a 착수 여부. C3b는 설계 승인 별도.
4. URSim 통합 검증 환경 접근 가능 여부.
