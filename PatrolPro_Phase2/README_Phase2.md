# PatrolPro Phase2 Arduino

업로드 대상 스케치:

```text
PatrolPro_Phase2/PatrolPro_Phase2/PatrolPro_Phase2.ino
```

## 구조

- `Config.h`: 핀 번호, 센서 임계값, 서보 각도, 모터 PWM 튜닝값
- `MotorControl.*`: 4개 모터 전진, 후진, 좌회전, 우회전, 정지
- `SensorSuite.*`: IR, 가스, 초음파 센서 읽기와 화재/가스 위험 판단
- `StatusOutputs.*`: LED, 부저, OLED, 카메라 서보모터
- `SerialProtocol.*`: Serial Monitor와 Jetson 명령 파싱
- `PatrolController.*`: 순찰, 화재 알림, 사람 확인, 침입자 알림 상태머신

기존 친구 코드는 `hazard_monitor/hazard_monitor.ino`에 그대로 남겨뒀고, 새 코드는 위 모듈 구조로 분리했습니다.

## LED 연결

- 기존 상태 LED: Arduino pin `24`, LED `24개`
- 외부 LED light: Arduino pin `17`, LED `200개`
- 외부 LED `1-100`: 첫 번째 보드
- 외부 LED `101-200`: 두 번째 보드

기존 상태 LED와 외부 LED 200개는 분리되어 동작합니다.
앞쪽 24개 원형 상태 LED는 사람 얼굴 확인 중에 주황색 점이 한 칸씩 도는 scanner로 동작하고, 등록자 확인이 완료되면 전체 초록색으로 바뀝니다.
상판은 전방 좌측부터 local `1`, 전방 우측이 local `10`, 뒤로 갈수록 10씩 증가합니다. 실제 장착은 상판을 180도 돌리는 기준이라, 코드는 논리 좌표를 180도 회전해서 global `101-200`으로 변환합니다.
뒷판은 하단 우측이 `1`, 하단 좌측이 `10`, 위로 갈수록 10씩 증가합니다.
현재 기본 설정은 바퀴 구동이 활성화되어 있고, 대신 부팅 직후에는 `Config.h`의 `kStartInPatrol = false`로 안전 정지 상태에서 시작합니다. 자동 순찰은 `AUTO` 명령을 받은 뒤 시작됩니다.
뒷판 `41-100`은 상태 표시 패널로 동작합니다: 순찰 청록색 scanner 라인, 얼굴 확인 주황색 sweep, 등록자 초록 원, unknown 빨간 X, 위험 감지 빨간 느낌표 blink.
뒷판 `1-40`은 신호등 영역으로 남겨두고, 우측 `1`, `2`, `11`, `12`, `21`, `22`와 좌측 `9`, `10`, `19`, `20`, `29`, `30` 그룹을 브레이크/깜빡이에 사용합니다.
평상시에는 이 그룹들이 연한 빨강 미등으로 켜지고, 사람 확인/화재/unknown/장애물 정지처럼 감속 또는 정지하는 상태에서만 강한 빨강 브레이크등으로 켜집니다.
우회전 중에는 우측 그룹이 주황색으로 깜빡이고, 좌회전용 함수도 같은 방식으로 준비되어 있습니다.
상판은 모드별 그림을 표시합니다: 순찰 청록색 로딩 원, 얼굴 확인 깜빡이는 눈, 등록자 smile, unknown 물음표, 위험 감지 빨간 느낌표.

## Serial Monitor 명령

기본 baud rate는 `115200`입니다.

```text
h   도움말
o   센서값 1회 출력
p   센서값 자동 출력 on/off
b   부저 테스트
l   LED 테스트
v   서보 스윕 테스트
c   카메라 서보 기본 각도로 복귀
E:n:r:g:b      외부 LED n번 개별 제어, n=1-200
ER:a:b:r:g:b   외부 LED a-b 구간 제어
EA:r:g:b       외부 LED 1-200 전체 제어
EOFF           외부 LED 전체 끄기
MANUAL         수동 정지 모드
AUTO           자동 순찰 시작
STOP           즉시 정지
F 1000         1초 전진
B 500          0.5초 후진, 후진등 흰색
L 800          0.8초 좌회전
R 800          0.8초 우회전
D3             시간 기반 약 3m 전진
```

예시:

```text
E:1:255:0:0        1번 LED 빨강
E:101:0:0:255      101번 LED 파랑
ER:1:100:0:255:0   1-100번 초록
ER:101:200:255:80:0 101-200번 주황
EA:10:10:10        전체 아주 어두운 흰색
EOFF               전체 끄기
```

## Jetson 명령

친구 Jetson 코드 `detect_people.py`의 최신 `STATUS:` 패킷 방식을 기준으로 맞췄습니다. 이전 개별 이벤트도 수동 테스트용으로 계속 처리합니다.

```text
STATUS:CLEAR
STATUS:T0:SCANNING:LEFT
STATUS:T0:VERIFIED:Noah,T1:UNKNOWN
FACE_TIMEOUT
HEARTBEAT
```

Arduino가 Jetson으로 보내는 명령:

```text
MODE:<state>
PLAY:<clip_name>
```

Jetson 쪽은 친구 코드 그대로 `detect_people.py`, `face_id.py`, `arduino_link.py`, `snapshot_writer.py`를 사용합니다.

## 현재 동작

- 정상 상태: LED 흰색, OLED `PATROL SAFE`
- 센서 화재/가스 감지: 정지, LED 빨간색 깜빡임, 부저, OLED alert, `PLAY:alert_fire`
- 사람 감지 `STATUS:Tn:SCANNING:<LEFT|CENTER|RIGHT>`: 정지, 얼굴 확인 모드, 앞쪽 24개 LED scanner, OLED scanning
- 사람이 화면 좌우로 치우쳐 있으면 얼굴 스캔 전에 차체를 해당 방향으로 약 `300ms`만 살짝 회전하고, 정지 후 스캔을 시작합니다.
- 얼굴 확인이 늦으면: 약 `6초` 뒤 카메라 서보가 기본 각도 `100`도에서 `130`도까지 올라간 뒤 같은 속도로 다시 내려옴
- 등록자 `STATUS:Tn:VERIFIED:<name>`: 초록 LED, OLED 이름 표시, `PLAY:verified_<name>`, 1초 후 순찰 복귀
- 미등록자 `STATUS:Tn:UNKNOWN` 또는 `FACE_TIMEOUT`: 정지, 빨간 LED 깜빡임, 부저, `PLAY:alert_intruder`
