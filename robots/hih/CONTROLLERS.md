# Hand in Hand 역할 제어기 — XYZ+yaw 접촉력 Admittance

## 목적과 현재 구현 범위

Leader는 기존 Beetle PID로 기준 위치·자세를 추종합니다. Follower는 기존 PID와 MOB를 그대로 사용하면서, 추정 접촉 wrench와 목표 접촉 wrench의 차이로 위치·yaw 기준을 수정합니다. 두 기체는 물리적 접촉을 통해 화물을 운송하는 구조이며, 새 제어기는 Leader의 위치 명령을 네트워크로 복사하지 않습니다.

참고한 사용자 제공 코드: `HIH_sim-main.zip`의 `follower_admittance_controller.cpp`, `follower_admittance_core.hpp`, `follower_wrench_controller.cpp`, `leader_wrench_controller.cpp`. 시뮬레이션을 실행하거나 시뮬레이터 PID/allocator/second-order MOB를 실기용으로 교체하지 않았습니다. 시뮬레이션의 접촉 wrench 오차 의미와 모멘트 기준점 이동·접촉점 회전 보정·stale 입력 정지 동작을 반영했습니다.

## 코드 위치

- `src/control/hih_leader_controller.cpp`, `include/hih/control/hih_leader_controller.h`
- `src/control/hih_follower_controller.cpp`, `include/hih/control/hih_follower_controller.h`
- `include/hih/control/hih_admittance.h`: ROS와 독립적인 4자유도 수치 모델
- `include/hih/control/hih_contact_wrench.h`: 실제 제어기와 테스트가 함께 사용하는 좌표·모멘트 변환
- `include/hih/control/hih_role_parameters.h`: 부모 제어기 초기화 전 역할별 설정 적용
- `config/hih_control.yaml`: Leader/Follower PID·MOB 및 Follower Admittance 설정
- `plugins/hih_role_control_plugins.xml`: 별도 역할 제어기 등록
- `test/test_hih_admittance.cpp`: 수치·접촉력 부호·모멘트 기준점·yaw 보정 회귀 테스트

기존 include 관례에 맞춰 헤더는 `include/hih/control/`에 둡니다. 이름만 구분했던 이전 bringup이 이제 역할별 플러그인을 선택합니다. `hih_leader_controller`와 `hih_follower_controller` 모두 기존 `HihController`를 상속합니다.

## 공통 PID에 추가한 선택 인터페이스

기존 PID는 navigator에서 매 주기 기준 스냅샷을 읽습니다. 이 기준을 navigator에 다시 쓰면 키보드 명령과 경쟁하거나 보정이 누적될 수 있습니다. 이를 피하기 위해 공통 패키지에 다음 두 변경을 추가했습니다.

1. `aerial_robot_control/include/aerial_robot_control/control/base/target_reference_modifier.h`
2. `aerial_robot_control/src/control/base/pose_linear_controller.cpp`: 스냅샷을 읽은 직후 선택 인터페이스에 전달

Follower만 `TargetReferenceModifier`를 구현합니다. Leader, 원본 Beetle 및 다른 기존 제어기는 이 인터페이스를 구현하지 않으므로 기존 기준을 그대로 사용합니다. `PoseLinearController`와 `BaseNavigator`의 레이아웃·vtable은 바꾸지 않았고, PID 계산식도 바꾸지 않았습니다. 새 헤더를 추가하고 원래 함수를 수정하는 대신 선택적 cross-cast를 사용했습니다.

**다른 PC로 배포할 때 hih 폴더만 복사하면 안 됩니다.** 제공된 업데이트 ZIP에는 위 공통 패키지 변경도 포함됩니다. 같은 저장소의 해당 파일들과 `robots/hih/`를 함께 적용하고 빌드해야 Follower 기준 보정이 실제 PID에 전달됩니다.

## 접촉력과 좌표계

MOB의 힘은 환경이 기체에 가하는 외력입니다. 현재 실기 MOB는 힘을 world 좌표로, CoG 기준 토크를 CoG 좌표로 보관합니다. 모델의 `getCog2Cp<KDL::Frame>()`로 CoG에서 `contact_point`까지의 변환을 읽어 다음을 계산합니다.

```text
F_cog     = R_world_cog^T * F_world
tau_cp    = R_cog_cp^T * (tau_cog - r_cog_cp × F_cog)
F_cp      = R_cog_cp^T * F_cog
F_des_cp  = -F_normal_reference * normal_contact
error_F_w = R_world_cp * (F_cp - F_des_cp)
error_T_w = R_world_cp * tau_cp
```

`normal_contact`는 **기체에서 화물 쪽으로 향하는 방향**을 모델 `contact_point` 좌표로 표시한 단위벡터입니다. 기본 `[1,0,0]`은 +X 접촉판 가정입니다. 현재 기본 모델의 접촉점은 base_link 기준 +X 0.26 m이며, 시뮬레이션의 0.215 m를 하드코딩하지 않았습니다. 실제 접촉판 위치/방향이 다르면 모델과 이 설정을 먼저 맞춰야 합니다.

4자유도 입력은 `[error_F_w.x, error_F_w.y, error_F_w.z, error_T_w.z]`입니다. 따라서 가상 동역학은 **world XYZ와 world yaw**에서 계산합니다. 원본 시뮬레이션의 contact-frame 6자유도 적분과 완전히 같은 구현은 아닙니다. 요청된 4자유도로 투영했으며 roll/pitch 명령은 유지합니다. yaw는 Euler yaw 증분으로 취급하고 큰 기울기의 SO(3) 6자유도 동작을 구현했다고 간주하지 않습니다.

```text
M * delta_q_ddot + D * delta_q_dot + K * delta_q = filtered_wrench_error
q = [world X, world Y, world Z, world yaw]
```

작은 하위 스텝의 backward Euler 적분에 입력, 속도, 가속도 및 변위 제한을 적용합니다. 변위 제한에서 바깥 방향 속도를 제거해 숨은 누적을 막습니다. 기본 K=0은 목표 힘이 맞춰진 뒤 이전 위치로 당기는 가상 스프링을 두지 않기 위한 설정입니다. K>0을 쓰면 위치 복원력과 접촉력 오차 사이의 평형이 생기므로 일정 힘 추종 오차가 남을 수 있습니다.

MOB가 추정하는 것은 모델 오차·바람 등을 포함할 수 있는 합성 외력입니다. 이를 화물 접촉력으로 해석하려면 실제 시스템의 추정 바이어스와 접촉 조건을 확인해야 하며, 정확한 화물 힘 센서로 간주하지 않습니다.

## 기준 명령과 접촉점 보정

PID에 전달할 이번 주기의 스냅샷 복사본만 수정합니다. navigator의 원래 기준과 PID 적분 상태를 덮어쓰지 않습니다.

```text
yaw_command = nominal_yaw + admittance_yaw_offset
CoG_position_command = nominal_position + contact_translation_offset
                     + (R_nominal - R_command) * r_cog_cp
```

마지막 항은 yaw 변경으로 접촉판 중심이 쓸려 움직이는 것을 보정합니다. `apply_velocity=true`이면 위치/yaw 보정 속도도 기존 기준에 더합니다. 각속도는 부모 제어기가 기대하는 target-CoG 좌표 표현으로 변환하고, 접촉판 회전 보정의 속도 항도 반영합니다. 가속도 feedforward는 새로 추가하지 않습니다.

## 활성 조건과 입력 손실

- `enabled=false`가 기본입니다. 이 경우 Follower도 기존 PID/MOB 경로를 유지합니다.
- `enabled=true`로 시작하더라도 hover·위치 제어 모드에서만 동작합니다.
- 조립/unified mode에서는 동작하지 않습니다. 이 프로젝트의 Leader/Follower 역할은 기존 rigid-assembly 모듈 역할과 별개입니다.
- 최초 `contact_threshold` 이상의 압축력을 감지한 뒤 접촉력 기준을 현재 값에서 설정 목표까지 `normal_force_rate`로 증가시킵니다. 자동 접근은 없습니다.
- 접촉을 한 번 감지한 이후 접촉력이 부족하면 제한 범위에서 화물 쪽 보정을 만들고, 과하면 물러나는 방향 보정을 만듭니다.
- MOB/IMU 입력이 오래됐으면 이를 힘 0으로 해석하지 않습니다. 변위를 유지하고 보정 속도를 0으로 하며 힘 목표 증가도 멈춥니다.
- clock jump/너무 큰 dt, force landing, 착륙/비-hover, 제어 모드 변경 시 보정을 초기화하고 기존 비행 상태 처리를 우선합니다.
- 설정은 역할 제어기 초기화 시 읽습니다. 비행 중 `rosparam set enabled`로 토글하는 기능이나 연속적인 ON/OFF handoff 서비스는 아직 없습니다. 다음 키보드·실험 상태기 구현 단계에서 추가할 항목입니다.

## 파라미터와 실행

### 单机 Gazebo 起飞验证

无需生成或加载 `/tmp` 配置文件。普通 `bringup.launch` 增加了启动参数
`admittance:=true`，自动选择 Follower 控制器并启用 XYZ+yaw 导纳：

```bash
source /opt/ros/one/setup.bash
source ~/ros/jsk_aerial_robot_ws/devel/setup.bash
roslaunch hih bringup.launch simulation:=true real_machine:=false \
  robot_id:=2 headless:=false admittance:=true
```

该入口读取包内 `config/hih_control.yaml` 的导纳设置；仿真时用
`HihControl_sim.yaml` 覆盖角色的 PID/MOB 等已有控制参数，避免角色初始化
将仿真增益覆盖为真机增益。`admittance` 默认为 `false`，普通启动保持原有
控制器。该开关仅在启动时生效，不是飞行中的切换服务。

另一终端加载工作空间后运行 `rosrun hih hih_key.py`，按 `v` 解锁、`b` 起飞，
悬停后按 `n` 降落。仅启动一台 `/hih2` 即可，不需要 Leader。

```bash
rostopic echo /hih2/hih/admittance/status
```

正常无接触悬停时应为 `waiting_for_contact`，补偿量为零；检测到超过阈值的
压缩力后才进入 `active`。因此本步骤验证开启导纳后的起飞/悬停兼容性，
接触力跟踪还需随后在 Gazebo 中施加接触或外力验证。

### 기존 역할별 실행

`hih_control.yaml`에서 `leader/controller` 및 `follower/controller`의 PID/MOB 값은 기존 실기 `HihControl.yaml`과 동일합니다. bringup이 설정을 namespace 아래에 로드하고, 제어기가 자기 역할의 값을 부모 초기화 전에 적용합니다. `HihControl.yaml`에는 QP와 기존 하드웨어 관련 나머지 기본 설정이 유지됩니다.

Admittance 벡터 순서는 모두 `[X,Y,Z,yaw]`입니다. 병진은 m·N, 회전은 rad·Nm를 사용합니다. M/D/K의 초기값은 제공된 시뮬레이션의 해당 축 값을 참고했지만, 변위·속도 제한은 더 작게 두었습니다. **10 N 목표와 수치들은 실기 검증된 게인이 아닙니다.** 실제 기체에서 확인한 값으로 설정하고 실험 전 활성 여부를 결정해야 합니다.

```bash
source /opt/ros/one/setup.bash
cd ~/ros/jsk_aerial_robot_ws
catkin build hih beetle -j2 -p2
source devel/setup.bash
```

각 기체 PC에서 기존 공통 ROS master/IP 설정을 적용한 뒤:

```bash
# Leader PC
roslaunch hih hih_leader_bringup.launch \
  simulation:=false real_machine:=true robot_id:=1 battery:=true

# Follower PC (별도 터미널/별도 호스트)
roslaunch hih hih_follower_bringup.launch \
  simulation:=false real_machine:=true robot_id:=2 battery:=true
```

별도 설정 파일은 `hih_control_file:=/absolute/path/to/settings.yaml`로 선택할 수 있습니다. 새 `hih_key.py`가 추가됐습니다. 좌우 분리 매핑과 실행 방법은 [KEYBOARD.md](KEYBOARD.md)를 참고하세요.

## 기록

기존 PID 디버그·MOB 토픽과 함께 `/hih2/hih/admittance/` 아래를 기록합니다.

- `nominal_position`, `command_position`: 보정 전후 CoG 기준
- `position_offset`, `velocity_offset`: 가상 접촉점 XYZ 보정 상태
- `yaw_reference`: x=기준 yaw, y=명령 yaw, z=yaw 보정량 (모두 rad)
- `input_wrench_world`: 필터·deadband·제한을 거친 world wrench **오차** (원시 측정 힘이 아님)
- `normal_force_reference`, `normal_force_measured`: 적용 중인 목표와 추정 압축력 (N)
- `applying`, `status`: 적용 여부, 대기/비활성/stale/모드 상태

자동 접근, preload 적응, 화물 lift/trajectory, CoP 제어 및 운송 시퀀스는 아직 포함되지 않습니다. 키보드는 후속 작업으로 추가했습니다. 실기 센서나 모터를 작동시키는 검증은 하지 않았습니다.
