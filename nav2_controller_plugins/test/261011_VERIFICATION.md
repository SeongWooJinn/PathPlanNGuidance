<!-- 261011 수정: MPC 수정 범위와 실제 Gazebo/Nav2 검증 결과를 재현 가능한 형태로 기록한다. -->
# 261011 MPC 검증 기록

<!-- 261011 수정: 요청한 4개 연속 웨이포인트 액션의 실제 실행 결과를 추가한다. -->
## 4개 연속 웨이포인트 추가 검증

`my_pathplanner.launch.py`와 Gazebo GUI를 실행한 상태에서 `/follow_waypoints`에
4개 목표를 한 번에 전달했다. 화면을 직접 클릭하지 않고 RViz의 waypoint follower와
동일한 액션을 사용했고, 기존 RViz `/waypoints` MarkerArray에 번호/방향을 표시했다.
이번 실행은 기본 ROS domain을 사용했다.

| 순서 | map 목표 (x, y, yaw) | 결과 | 관측 위치 오차 | 관측 yaw 오차 |
| --- | --- | --- | --- | --- |
| 1 | (1.0, 0.0, 0) | 성공 | 0.122m | 0.005rad |
| 2 | (2.0, 0.0, 0) | 성공 | 0.246m | 0.013rad |
| 3 | (1.5, 0.5, π/2) | 성공 | 0.216m | 0.192rad |
| 4 | (1.0, 0.5, π) | 성공 | 0.206m | 0.033rad |

액션 결과 `status=4`, `error_code=0`, `missed_waypoints=[]`.
waypoint_follower 로그: `Completed all 4 waypoints requested.`
1~3번 오차는 다음 waypoint feedback 전환 시, 4번은 전체 액션 종료 시의
`map → base_link` TF 측정값이다. 정확한 내부 goal checker 판정 시각과는 다르다.
실제 `/odom`에서 전후진 속도 ±0.26m/s를 확인했다.
현재 도착 허용오차는 위치 0.25m, 방향 0.25rad이며 임의의 모든 경로 성공을 보장하지 않는다.
GUI 프로세스 실행과 ROS 데이터를 확인했으며 화면 자체를 직접 시각 검수하지는 못했다.
시험용 코드만 `/tmp/261011_waypoint_probe.py`에 작성했고 제어 코드는 변경하지 않았다.
로그: `/tmp/261011_waypoints_tracking.log`, `/tmp/261011_waypoints_launch.log`.

기본 `maze_world`, TurtleBot3 Waffle, ROS 2 Jazzy에서 검증했다.
기존 사용자 변경과 260904/260905 TF 보정은 유지했다.

## 수정 범위

- `src/mpc_controller_plugin.cpp`: 직선을 BicycleMode로 분류, 측정 속도/각속도로 초기 상태 구성,
  전후진 부호를 보존하는 곡률 기반 조향 참조, 솔버 실패 시 정지/예외 처리, 목표 위치 및 최종 yaw 정렬,
  AMCL 보정 시 위치 추종 재개, local_costmap footprint 검사, 동적 속도 제한 복원.
- `controller/mpc/include/mpc_controller.h`: 초기 측정 속도, 기어/모드 경계 검색 제한,
  다른 구간에 대한 예측 참조 정지, 위치 추종 비용 30/방향 비용 8/조향각속도 비용 2 적용.
- `controller/mpc/include/structs_mpc.h`: 참조 궤적에 명시적 gear 보존.
- `controller/mpc/include/recoveryfsm_mpc.h`: 스킵과 재계획 카운터 분리, 기어 경계 스킵 방지.
- `controller/velocity_planner.h`: 최소 0.01m 강제 이동 제거, 속도 적분에 따른 구간 보간,
  정지→정지 구간의 제한된 속도 프로파일, Spin 각속도 0.3rad/s, 종점/단일 점/조향 참조 보존.
- `config/nav2_params.yaml`: 전역 플래너 `delta_max: 0.7`로 MPC 조향 제약과 일치,
  `expected_planner_frequency: 0.3`으로 BT 재계획의 경고 기준 일치.
- 플러그인 헤더/CMake와 `test_reference_trajectory.cpp`, `simulation_tracking_probe.py`: 상태 및 검증 도구 추가.

변경 블록에는 `261011 수정`과 역할 주석을 남겼다.
재계획 주기는 기존 `custom_nav.xml`의 0.3Hz를 유지했다.
`expected_planner_frequency`는 재계획 스케줄링 설정이 아니다.

## 빌드 및 자동 테스트

```bash
cd /home/uj/PathPlan_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
colcon test --packages-select nav2_controller_plugins --event-handlers console_direct+
colcon test-result --test-result-base build/nav2_controller_plugins
```

4개 패키지 빌드 성공. TF 8개 + 참조 궤적 6개 = gtest 14개 통과.
colcon 집계의 16개는 CTest 상위 항목 2개를 포함한 수치다.
기존 acados/OpenCV 매크로, 미사용 변수 등의 컴파일 경고는 남아 있다.

## 시뮬레이션 검증

Gazebo 서버/GUI와 RViz2를 동시에 실행하고 NavigateToPose 액션, `/cmd_vel`, `/odom`,
`map → base_link` TF, 최근 `/plan` 대비 거리로 측정했다.
검증 환경은 다른 ROS 세션과 분리하기 위해 ROS_DOMAIN_ID=88을 사용했다.

| 목표 | 액션 결과 | 최종 위치 오차 | 최종 yaw 오차 | 최대 경로 이탈 |
| --- | --- | --- | --- | --- |
| 후진 (1.0, 0.0, 0.0) | SUCCEEDED | 0.232m | 0.002rad | 0.011m |
| 전진 (2.0, 0.0, 0.0) | SUCCEEDED | 0.237m | 0.026rad | 0.032m |
| 곡선 (1.5, 0.5, π/2) | SUCCEEDED | 0.237m | 0.161rad | 0.069m |
| 제자리 yaw (1.58, 0.27, 0.0) | SUCCEEDED | 0.105m | 0.242rad | 0.032m |

후진/전진 최대 명령 선속도는 각각 -0.26/+0.26m/s이며 odometry 부호도 일치했다.
제자리 yaw 시험은 선속도 명령 0.0m/s 및 odometry 선속도 약 0을 유지하고 -0.35rad/s로 회전했다.
map 좌표의 변화에는 회전 중 AMCL 보정이 포함된다.
최종 위치 오차는 현재 Nav2 허용오차 0.25m에 따른 종료 결과이며 정밀 정지를 의미하지 않는다.
경로 이탈은 재계획 중 가장 최근 경로와 TF 자세 사이의 거리이며 별도 ground-truth 오차가 아니다.

반복 시험 중 목표 정렬 상태가 AMCL 보정 이후에도 유지되는 정체를 발견해 해제 조건을 수정했다.
곡선 추종 오차는 참조 조향각과 실행 시 비용 가중치를 보완한 뒤 감소했다.
전체 미로 완주 및 모든 장애물 시나리오를 검증한 것은 아니다.

## 재현

첫 터미널:

```bash
cd /home/uj/PathPlan_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_plugins my_pathplanner.launch.py
```

같은 ROS 환경을 source한 두 번째 터미널:

```bash
python3 /home/uj/PathPlan_ws/src/nav2_controller_plugins/test/simulation_tracking_probe.py --suite
```

이 도구는 기본 maze_world에서 초기 위치 (2,0,0)를 설정하고 위 3개 목표를 순서대로 전송한다.
개별 목표는 `--goal x y yaw`로 지정한다. BT lifecycle ACTIVE까지 기다린다.
제자리 yaw 분기 검증은 위 3개 목표 이후 `--goal 1.58 0.27 0.0`으로 실행했다.
기존 launch는 Gazebo 서버만 실행하므로 GUI는 같은 GZ_PARTITION에서 `gz sim -g`로 연결한다.

## 화면 확인 범위

Gazebo GUI/RViz2 프로세스 실행 및 RViz에서 사용하는 TF 이동을 확인했다.
데스크톱의 스크린샷 요청은 AccessDenied로 거부되어 화면의 픽셀을 직접 검토하지 못했다.
RViz 최초 지도 수신 시 shader 경고와 VMware의 3D/EGL 경고가 있었다.
동일 shader 메시지는 [RViz upstream 이슈 #463](https://github.com/ros2/rviz/issues/463)에도 보고되어 있다.
따라서 시각적 렌더링이 완전히 정상이라고 판단하지 않는다.

<!-- 261011 수정: 검증 로그 위치와 종료 상태를 기록한다. -->
원본 실행 로그: `/tmp/261011_curve_fixed_launch.log`, `/tmp/261011_curve_fixed_tracking.log`,
`/tmp/261011_final_yaw_tracking.log`, `/tmp/261011_curve_fixed_gui.log`.
이 파일들은 임시 파일이므로 재부팅/정리 이후 없어질 수 있으며 위 표가 결과 요약이다.
검증용 ROS/Gazebo/RViz 세션은 시험 완료 후 종료한다.
