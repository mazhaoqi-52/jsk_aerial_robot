# HiH 키보드 노드

`script/hih_key.py`는 ground station에서 실행하며, 명시적으로 설정한 `/hih1`(Leader)과 `/hih2`(Follower)에 명령을 보냅니다. Leader 또는 Follower 한 대만 켜져 있어도 해당 기체를 조종할 수 있고, 두 기체가 모두 연결되기를 기다리지 않습니다. 기체를 자동 검색하지 않으며, `/assembly`나 기존 `/beetle…`에는 명령을 보내지 않습니다.

## 실행

WSL 작업 폴더에는 적용돼 있습니다. 새 환경을 적용한 뒤, 입력 포커스가 있는 터미널에서 실행합니다.

```bash
hw

# ROS 노드 생성이나 publish 없이 터미널 매핑만 확인
rosrun hih hih_key.py --dry-run

# 실제 명령 전송 (ground station에 roscore, 각 기체에 bringup 실행 후)
rosrun hih hih_key.py

# 다른 설정 파일 선택
rosrun hih hih_key.py --config /absolute/path/to/hih_key.yaml
```

이동 동작 이름은 Python과 YAML 모두 `+X`, `-X`, `+Y`, `-Y`, `+Z`, `-Z`, `+yaw`, `-yaw`로 맞춥니다. YAML 예: `'+X': w`. 이 이름은 설정을 찾는 키이므로 화면 표시만 바꾸는 것과 달리 양쪽이 일치해야 합니다.

기본 `config/hih_key.yaml`은 자동으로 읽습니다. 별도의 `rosparam load`는 필요 없습니다. YAML은 시작 시 읽으므로 변경 후 키보드 노드를 재실행합니다. 키보드는 대화형 터미널에서 실행해야 하며 detached roslaunch 노드로 실행하지 않습니다.

ROS master/IP 설정은 기존 다중 PC 구성과 같습니다. `hsh`처럼 환경을 초기화하는 셸을 쓰면 그 셸을 연 뒤 네트워크 환경을 적용합니다. YAML의 namespace는 기체 ID에 맞춘 ROS 토픽 경로이며 IP 주소가 아닙니다.

## 키 매핑

| 동작 | Follower | Leader |
|---|---|---|
| +X / -X | `w` / `s` | `i` / `k` |
| +Y / -Y | `a` / `d` | `j` / `l` |
| +Z / -Z | `r` / `f` | `p` / `;` |
| +Yaw / -Yaw | `q` / `e` | `u` / `o` |

| 공통 키 | 전송 명령 |
|---|---|
| `v` | arming → `teleop_command/start` |
| `b` | takeoff → `teleop_command/takeoff` |
| `n` | land → `teleop_command/land` |
| `m` | force landing → `teleop_command/force_landing` |
| `/` | 모터 즉시 정지 경로 → `teleop_command/halt` |
| Space | 키보드의 XYZ/yaw 속도 명령만 0으로 설정 |
| `?` | 현재 YAML에서 생성한 키 안내 재출력 |
| Ctrl+C | 키보드 속도를 0으로 보내고 종료; 착륙/모터 정지 명령은 보내지 않음 |

공통 비행 명령의 기본 대상은 두 기체입니다. `common_targets`로 대상을 제한할 수 있습니다. Space와 종료는 키보드가 다루는 두 기체의 이동 명령을 모두 취소합니다. 여러 호스트로의 명령 전송은 개별 ROS publish이며 하드웨어 동기 실행을 보장하는 트리거가 아닙니다.

## 이동 명령의 의미

- 기본 world 좌표계이며 병진 속도 0.10 m/s, yaw 속도 0.10 rad/s입니다. 기체별 `xy_speed`, `z_speed`, `yaw_speed`를 설정할 수 있습니다.
- `frame: local`로 바꾸면 XY 방향은 각 기체의 현재 yaw를 따릅니다. Z는 기존 navigator의 높이 명령, yaw는 기존 yaw-rate 명령입니다.
- `aerial_robot_msgs/FlightNav`의 CoG 목표와 XY/Z/yaw `VEL_MODE`를 사용합니다. 이 저장소의 navigator는 이를 기준 위치/yaw에 적분하면서 위치 제어를 유지합니다. 새 Admittance와 경쟁해서 절대 위치 기준을 덮어쓰지 않습니다.
- 움직이지 않는 축에는 속도 0을 넣고, roll/pitch 명령은 보내지 않습니다. 속도 0은 목표 위치를 원점이나 현재 측정 위치로 재설정하지 않습니다.
- 각 기체의 각 축은 마지막 입력부터 기본 0.35초 동안 유지됩니다. 입력이 끊기면 해당 축만 0이 되며, 모든 축이 멈추면 0 명령을 세 번 보낸 뒤 idle publish를 멈춥니다.
- 터미널은 키를 놓는 이벤트를 제공하지 않으므로 키 반복과 timeout으로 정지 시점을 판단합니다. 운영체제의 초기 키 반복 지연이 길면 처음 누른 뒤 잠깐 끊길 수 있습니다. 터미널에서 여러 키를 계속 동시에 누르는 것을 완전한 조이스틱처럼 보장하지는 않습니다. 두 기체/여러 축의 입력은 시간 내 교차 입력으로 독립 유지됩니다.
- 여러 축을 동시에 입력하면 축별 설정 속도가 적용되므로 벡터 크기는 단일 축 속도보다 커질 수 있습니다.
- Space는 키보드 기준 이동만 취소하며, Follower의 Admittance 접촉력 보정은 끄지 않습니다. 새 Admittance 활성/비활성 서비스나 운송 시퀀스는 이번 키보드에 추가하지 않았습니다.

## 구성과 입력 처리

`hih_key.yaml`에서 키, namespace, 속도, 발행 주기, timeout과 공통 비행 명령 대상을 관리합니다. 키 매핑 중복, namespace 중복과 잘못된 수치는 실행 전에 확인합니다. 설정 로더는 일반 yaml.safe_load를 사용합니다. Caps Lock을 끄고 소문자로 입력합니다. 붙여넣은 문자열과 화살표 escape sequence는 비행 키로 처리하지 않습니다.

비행 명령 키는 반복 입력을 억제하며 자동 재시도하지 않습니다. 명령 토픽에 subscriber가 없으면 경고를 출력합니다. 키보드를 켰다는 이유만으로 arming·이륙·착륙·halt를 보내지 않습니다. 비행 명령은 latched publish가 아닙니다. 연결이 끊기거나 프로세스가 강제 종료되면 마지막 0 명령이 전달된다고 보장할 수 없으며, 기존 navigator의 수신 timeout 처리는 그대로 유지됩니다.

## PID / MOB 보존과 검증

이번 키보드 작업에서 제어기 계산식이나 게인을 변경하지 않았습니다. 역할별 `hih_control.yaml`의 PID·MOB 설정을 원본 `beetle/config/BeetleControl.yaml`과 다시 비교했고, Leader/Follower 모두 동일했습니다.

검증은 모의 publisher와 실제 `FlightNav` 메시지를 사용하는 오프라인 테스트 13개(전체 이동 키, 한 대만 연결된 경우의 이동·공통 명령, timeout·정지 등), 실제 PTY 터미널에서의 dry-run, catkin 실행 파일 설치 빌드로 수행했습니다. dry-run에서는 ROS 노드나 publisher를 만들지 않습니다. 실제 기체나 모터를 작동시키는 테스트는 하지 않았습니다.

## 코드 스타일

첨부한 `keyboard_command_multi.py`의 흐름을 따라 `MultiPublisher`로 공통 비행 명령을 전송합니다. `main`에서 설정 읽기 → Publisher 생성 → `while` 키 입력 → 키별 `if` → 명령 전송 순서로 읽을 수 있습니다.

Leader와 Follower는 각각 별도의 `FlightNav` 메시지와 이동 Publisher를 사용합니다. 예를 들어 Follower의 +X 키에서는 `follower_nav.target_vel_x`를 직접 바꿉니다. 키보드 매핑을 일반화한 탐색 루프는 없앴습니다. 반복되는 메시지 초기화, 축별 timeout·정지, 터미널 입력/복원은 작은 함수로 분리했습니다.
