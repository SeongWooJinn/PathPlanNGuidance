# 2026-10-30 병합 전 시뮬레이션 검증

검증 소스: `feature/ros2_mpc`, `ccc6b8f`.
실제 실행 파일명은 `nav2_plugins/launch/my_pathplanner.launch.py`이다.
ROS 2 Jazzy, TurtleBot3 Waffle, 기본 `maze_world`, 초기 위치 `(2, 0, 0)`에서 실행했다.
다른 실행과 분리하기 위해 `ROS_DOMAIN_ID=88`, `GZ_PARTITION=pathplan_261030`을 사용했다.

## 빌드 및 회귀 테스트

- `colcon build --symlink-install`: 4개 패키지 성공.
- `colcon test --packages-select nav2_controller_plugins`: TF 8개와 참조 궤적 6개 통과.
- `colcon test-result`: 상위 CTest 항목을 포함한 16 tests, 0 errors, 0 failures, 0 skipped.

## 개별 목표를 순차 전송

기존 `test/simulation_tracking_probe.py --suite`를 실행했다.

| 목표 (x, y, yaw rad) | 액션 상태 | 위치 오차 m | yaw 오차 rad | 최대 최근 경로 이탈 m |
| --- | --- | --- | --- | --- |
| (1.0, 0.0, 0.0) | SUCCEEDED (4) | 0.1797 | 0.0216 | 0.0233 |
| (2.0, 0.0, 0.0) | SUCCEEDED (4) | 0.2255 | 0.0290 | 0.0063 |
| (1.5, 0.5, 1.5708) | SUCCEEDED (4) | 0.2073 | 0.1801 | 0.0907 |

세 목표 모두 성공했다. 최대 명령 선속도는 0.26m/s였으며 후진 목표에서 음의 명령과 실제 odometry 후진을 관측했다.
위치와 yaw 오차는 액션 결과 수신 시점의 `map → base_link` TF를 기준으로 측정했다.
경로 이탈은 가장 최근 `/plan`과의 거리이며 ground truth 오차가 아니다.

## 4개 연속 목표를 한 번에 전송

동일 launch 세션에서 개별 목표 시험 직후 `/follow_waypoints`에 다음 목표들을 한 번에 전달했다.
임시 계측 스크립트 `/tmp/pathplan_261030_waypoints.py`에서 `FollowWaypoints` 액션과 TF를 측정했다.

| 순서 | 목표 (x, y, yaw rad) | 관측 위치 오차 m | 관측 yaw 오차 rad |
| --- | --- | --- | --- |
| 1 | (1.0, 0.0, 0) | 0.1491 | 0.1212 |
| 2 | (2.0, 0.0, 0) | 0.2504 | 0.2201 |
| 3 | (1.5, 0.5, π/2) | 0.1580 | 0.2309 |
| 4 | (1.0, 0.5, π) | 0.2383 | 0.1349 |

최종 액션 결과: `status=4 (SUCCEEDED)`, `error_code=0`, `missed_waypoints=[]`.
서버 로그에서도 `Completed all 4 waypoints requested.`를 확인했다.
1~3번 오차는 다음 waypoint의 feedback 수신 시점에 측정했고, 4번은 최종 결과 수신 시 측정했다.
2번 위치 오차 0.2504m는 설정 허용오차 0.25m보다 약 0.4mm 크다.
이 값은 내부 goal checker의 판정 시각과 다른 시점의 TF 측정값이며 액션 자체는 성공했다.

## 확인 범위와 실행 로그

- 설정된 goal checker 허용오차: 위치 0.25m, yaw 0.25rad.
- 기본 미로의 위 목표들에 대한 검증이며 전체 미로 또는 임의 목표의 성공을 보장하지 않는다.
- RViz 프로세스는 실행됐으나 `indexed_8bit_image` 셰이더 오류가 관측됐다.
  VMware/EGL 그래픽 경고도 남아 있어 화면 렌더링 정상 여부를 보장하지 않는다.
  본 결과는 ROS 액션, TF, 경로 및 odometry 계측을 근거로 한다.
- 초기 센서/TF 기동 중 대기 메시지는 있었으나 이후 Nav2가 활성화되어 시험을 완료했다.
- 소스 제어 로직의 추가 수정 없이 검증했다.

실행 명령:

```bash
source /opt/ros/jazzy/setup.bash
source /home/uj/PathPlan_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle ROS_DOMAIN_ID=88 GZ_PARTITION=pathplan_261030
ros2 launch nav2_plugins my_pathplanner.launch.py
# 같은 환경의 별도 터미널
python3 nav2_controller_plugins/test/simulation_tracking_probe.py --suite
```

원본 로그(임시 파일): `/tmp/pathplan_261030_build.log`, `/tmp/pathplan_261030_tests.log`,
`/tmp/pathplan_261030_launch.log`, `/tmp/pathplan_261030_tracking.log`,
`/tmp/pathplan_261030_waypoints.log`.
임시 파일은 정리될 수 있으므로 측정 결과를 이 문서에 보존했다.
검증용 launch 세션은 시험 후 종료한다.
