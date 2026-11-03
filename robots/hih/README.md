# Hand in Hand — hih 패키지

`robots/beetle` 복사본을 별도 ROS 1 패키지 `hih`로 분리한 초기 버전입니다.
Ubuntu 22.04 / ROS-O (`ROS_DISTRO=one`)에서 사용합니다.

## 이번 버전의 범위

- 원본 `robots/beetle`은 수정하지 않았습니다.
- 패키지, C++ 클래스, 라이브러리, 플러그인, 메시지, Python 모듈과 리소스 참조를 분리했습니다.
- PID, 모멘텀 기반 외력 추정, QP 계산, 하드웨어 캘리브레이션과 수치 게인은 복사본의 기존 내용을 유지했습니다.
- `HihFormationMomentumObserver`도 원래 계산 그대로이며, 동일 프로세스에서 Beetle 라이브러리와 심볼이 충돌하지 않도록 이름만 분리했습니다.
- Leader/Follower bringup은 기본 ID와 역할 표지만 다릅니다. 두 기체 모두 현재는 같은 기존 제어기를 사용합니다.
- 새 Leader/Follower 제어기, Admittance, `hih_control.yaml`, `hih_key.yaml`, 요청한 새 키 매핑의 `hih_key.py`는 다음 단계입니다. 이 버전에 구현된 것으로 간주하지 마세요.

## 주요 이름과 경로

| 항목 | hih |
|---|---|
| ROS / Python 패키지 | `hih` |
| 기본 기체 namespace / TF prefix | `hih1`, `hih2` |
| 제어기 | `aerial_robot_control::HihController` (C++), `aerial_robot_control/hih_controller` (플러그인 식별자) |
| navigation | `aerial_robot_navigation/hih_navigation` |
| 모델 플러그인 | `hih_robot_model` |
| IMU 플러그인 | `sensor_plugin/hih_imu` |
| 설정 | `config/HihControl.yaml`, `config/HihControl_sim.yaml` |
| 헤더 | `include/hih/` |
| Python 유틸리티 | `script/hih/` |
| 기본 하드웨어 모델 | `robots/hih.urdf.xacro` |

메시지 타입은 `hih/TaggedWrench` 등으로 분리했습니다. 따라서 두 기체 PC와 ground station 모두 같은 버전의 `hih`를 빌드해야 합니다. 원래 `beetle/…` 메시지를 구독하는 분석/외부 노드는 `hih/…`로 맞춰야 합니다. 모델의 기존 물리 link/joint 이름(예: `beetle_fang_link`)은 하드웨어 인터페이스 호환을 위해 유지했습니다.

## 배치 및 빌드

ZIP에는 최상위 `hih/` 폴더가 있습니다. 작업 공간에 아래처럼 배치합니다.

```text
~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/
├── beetle/       # 원본 보존
└── hih/          # 이 패키지
```

기존 복사본이 있으면 먼저 별도 위치에 백업하고 교체합니다. 서로 다른 버전을 겹쳐 풀지 마세요.

```bash
source /opt/ros/one/setup.bash
cd ~/ros/jsk_aerial_robot_ws
catkin build hih beetle -j2 -p2
source devel/setup.bash
rospack find hih
rospack find beetle
```

빌드 결과와 `__pycache__`는 ZIP에 포함하지 않습니다. 코드 수정 후에는 다시 빌드하고 각 터미널에서 작업 공간 환경을 적용합니다.

## 두 기체 PC + ground station

각 기체는 별도 PC이므로 `/dev/flight_controller` 경로가 같아도 충돌하지 않습니다. 각 PC의 udev/직렬 연결 설정은 기존 Beetle 설정을 유지합니다. `robot_id`는 기존 mocap 설정에 전달되므로 실제 마커 ID와 연결 설정도 확인해야 합니다.

공통 ROS master는 ground station 한 곳에서 실행합니다. 아래 IP는 예시이며 실제 주소로 바꿔야 합니다. ROS 통신은 세 PC 사이에서 양방향으로 가능해야 합니다. SSH만 연결된다고 ROS 데이터 통신이 보장되지는 않습니다. 특히 ground station이 WSL이면 기체에서 WSL의 광고 주소와 노드 포트로 접근 가능한 네트워크인지 확인해야 합니다.

`hsh` 등 환경을 초기화하는 셸을 사용할 경우 **그 셸을 연 뒤** 아래 네트워크 변수를 설정합니다. 새 SSH 터미널과 rosbag 터미널에도 설정해야 합니다.

### Ground station

```bash
source ~/ros/jsk_aerial_robot_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://192.168.10.10:11311
export ROS_IP=192.168.10.10
roscore
```

### Leader PC (SSH)

```bash
source ~/ros/jsk_aerial_robot_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://192.168.10.10:11311
export ROS_IP=192.168.10.11
roslaunch hih hih_leader_bringup.launch \
  simulation:=false real_machine:=true robot_id:=1 battery:=true
```

### Follower PC (SSH)

```bash
source ~/ros/jsk_aerial_robot_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://192.168.10.10:11311
export ROS_IP=192.168.10.12
roslaunch hih hih_follower_bringup.launch \
  simulation:=false real_machine:=true robot_id:=2 battery:=true
```

ROS 1 XML launch이므로 `.launch` 확장자를 사용합니다. `real_machine:=true`가 올바른 인자 문법입니다. 기본 기체 모델은 이전 bringup과 같은 기본형이며, 기존 실험에서 fang 모델을 사용했다면 `hih_model:=hih_fang` 또는 해당 실제 모델을 지정해야 합니다.

### Ground station의 기존 키보드 노드

현재는 복사본의 기존 키보드 동작만 보존합니다. 설정한 두 namespace만 명시해 자동 검색으로 다른 기체를 조종하지 않도록 합니다.

```bash
rosrun hih keyboard_command_multi.py _robot_namespaces:="['hih1', 'hih2']"
```

이 스크립트는 두 기체에 같은 명령을 전달하는 기존 방식입니다. 요청한 좌/우 분리 키 매핑은 아직 아닙니다.

| 기존 키 | 명령 |
|---|---|
| `r` / `t` / `l` | arming / takeoff / land |
| `f` / `h` | force landing / halt (서로 다른 명령) |
| `w`, `s`, `a`, `d` | 두 기체의 ±X, ±Y 속도 명령 |
| `q`, `e`, `[`, `]` | 두 기체의 ±Yaw, ±Z 속도 명령 |

`m`은 아직 새 동작에 연결하지 않았습니다. Ctrl+C는 키보드 노드 종료이며 모터 정지 명령으로 취급하지 않습니다. 개별 기체 MOB와 조립체 전체 MOB는 기존처럼 별도 의미를 유지합니다. 두 독립 기체에 조립체 unified mode를 자동으로 켜지 않습니다.

### 기록과 상태 확인

ground station의 별도 터미널에도 같은 네트워크 환경을 적용한 뒤:

```bash
rosnode list
rostopic list
rosbag record -a -O hih_experiment
```

두 기체의 토픽은 `/hih1/…`, `/hih2/…`로 구분합니다. 기존 조립체 진단의 `/assemble/…` 같은 전역 경로는 원래 제어 동작을 보존하기 위해 그대로 두었습니다. 실제 기체 통신과 비행은 이번 오프라인 검증 범위에 포함되지 않습니다.

## 기존 예제의 알려진 제한

복사본에 원래부터 있던 비활성 예제 두 개는 이번 이름 분리와 무관하므로 보존했습니다: 빈 `launch/aerial_towing_tasks/towing_formation_put_on.launch`, XML 태그가 맞지 않는 `urdf/valve_upward.urdf`. 본 문서의 Leader/Follower 기본 bringup은 이 두 파일을 사용하지 않습니다. 기존 데모/분석 스크립트 전체를 실험 검증한 것은 아닙니다.
