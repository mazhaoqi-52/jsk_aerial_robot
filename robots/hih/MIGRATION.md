# HiH 패키지 분리 작업 결과

## 적용 위치

WSL Ubuntu-22.04의 `/home/yujin/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/hih`에 직접 적용했습니다. 브랜치는 `develop/hih_init`입니다. 원본 `robots/beetle`의 추적 소스 파일은 변경하지 않았습니다. commit/push는 실행하지 않았습니다.

## 변경 사항

- `package.xml` / `project()` / Python 설치 모듈: `beetle` → `hih`
- 메시지 참조: `beetle/…`, `beetle::…`, `beetle.msg` → `hih` 전용 타입
- 헤더 디렉터리·소스 파일·모델 파일·Python 모듈 및 내부 import 경로 변경
- C++ 클래스: `BeetleController`, `BeetleNavigator`, `BeetleRobotModel`, `BeetleImu`, `BeetleUnifiedController` 등을 `Hih…`로 분리
- `FormationMomentumObserver` → `HihFormationMomentumObserver`: 라이브러리 공존 시 동일 심볼 방지
- 제어기·navigation·모델·센서 라이브러리 및 플러그인 등록 식별자 분리
- YAML의 플러그인 선택, launch include, xacro 및 mesh package URI 갱신
- 워크스페이스 전역 CMake 테스트 대상 이름 분리
- 헤더 설치 규칙 및 기존 키보드 스크립트 실행 파일 등록 추가
- `hih_leader_bringup.launch`, `hih_follower_bringup.launch` 추가: 기본 ID 1/2와 역할 표지로 구분, 제어기는 기존 것 사용
- 기본 bringup의 미사용 필수 인자 `direct_model_name`에 빈 기본값 지정
- 다중 PC ROS master/IP 설정과 현재 구현 범위를 `README.md`에 명시

이번 분리는 동일한 패키지를 두 번 빌드할 때 발생하는 충돌을 피하려는 작업입니다. 제어 식, PID 게인, 외력 추정 식, 하드웨어 설정 수치는 변경하지 않았습니다. 원래 물리 link/joint 이름은 유지했습니다. 복사본 백업과 이름 변환을 기준으로 소스·헤더·설정 29개 파일의 보존을 비교했습니다.

## 검증 결과

| 검증 | 결과 |
|---|---|
| `catkin build hih beetle -j2 -p2 --no-status` | 의존성 포함 22개 패키지 성공, 실패 없음, 약 3분 11초 |
| `rospack find hih` / `beetle` | 각각의 디렉터리로 구분 |
| 두 역할 launch의 `--dump-params` | 성공, `/hih1` 및 `/hih2`의 전용 플러그인·TF 설정 확인 |
| 원본 + HiH 라이브러리 8개 동시 로드 | 성공 (상위 `aerial_robot_base` 라이브러리 로드 후 확인) |
| 생성 메시지 import 및 이름 분리 | 성공, 원본 메시지 wire schema 보존 |
| 기존 키보드 실행 파일 탐색 | 성공 |
| 기존 오프라인 Python 테스트 | 50개 통과 |
| 새 Python 문법 오류 | 없음 |
| 원본 Beetle 추적 소스 diff | 없음 |

## 확인된 기존 제한과 남은 범위

- `test_docking_alignment` 모듈은 `gazebo_ros_link_attacher` 미설치로 import할 수 없습니다. 원본 Beetle에서도 동일한 실패를 재현했습니다. 이번 분리로 생긴 실패가 아니며 시뮬레이션 의존성을 추가 설치하지 않았습니다.
- 빈 towing launch와 잘못된 태그를 가진 valve URDF 예제는 복사본에 원래 있던 문제입니다. 기본 Leader/Follower bringup은 이를 참조하지 않으며, 해당 예제는 보존했습니다.
- 빌드 중 spinal의 rosserial 라이브러리 생성 단계에서 일부 메시지/서비스 관련 경고가 나왔지만 최종 catkin 빌드는 성공했습니다. 새 펌웨어 빌드나 업로드를 검증한 것은 아닙니다.
- 새 `hih_key.py`의 좌우 분리 매핑, `m`의 동작, Leader/Follower별 신규 제어기 및 Admittance는 아직 구현하지 않았습니다. 현재 제공되는 `keyboard_command_multi.py`는 원래의 두 기체 공통 명령 방식입니다.
- 실제 네트워크 연결, 센서·FC 통신과 비행은 시험하지 않았습니다. 하드웨어 제어 노드 및 시뮬레이션도 실행하지 않았습니다.

## 사용 시작

현재 노트북에는 변경 및 빌드가 이미 반영돼 있습니다. ROS-O 터미널에서 `hw`로 새 환경을 적용하면 됩니다. 다른 PC에는 ZIP의 `hih/`를 `robots/hih`로 배치하고 같은 의존성 환경에서 빌드합니다. 자세한 IP 설정·bringup 명령·기존 키보드 사용법은 패키지의 `README.md`를 참고하세요.
