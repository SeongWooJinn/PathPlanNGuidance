import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 경로 탐색
    my_plugin_dir = get_package_share_directory('nav2_plugins')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # 2. 파일 경로 설정
    # 1) maze_world + spawn pos
    my_world_file = os.path.join(my_plugin_dir, 'worlds', 'maze_world.world')
    my_map_file = os.path.join(my_plugin_dir, 'maps', 'maze_world.yaml')
    x_pose = LaunchConfiguration('x_pose', default='2.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # # 2) turtlebot world + spawn pos
    # my_world_file = os.path.join(my_plugin_dir, 'worlds', 'turtlebot3_world.world')
    # my_map_file = os.path.join(my_plugin_dir, 'maps', 'turtlebot3_world.yaml')
    # x_pose = LaunchConfiguration('x_pose', default='-2.0')
    # y_pose = LaunchConfiguration('y_pose', default='0.0')

    # # 3) turtlebot house + spawn pos
    # my_world_file = os.path.join(my_plugin_dir, 'worlds', 'turtlebot3_house.world')
    # my_map_file = os.path.join(my_plugin_dir, 'maps', 'turtlebot3_house.yaml')
    # x_pose = LaunchConfiguration('x_pose', default='0.0')
    # y_pose = LaunchConfiguration('y_pose', default='0.0')

    my_params_file = os.path.join(my_plugin_dir, 'config', 'nav2_params.yaml')
    my_rviz_file = os.path.join(my_plugin_dir, 'rviz', 'my_rviz2_config2.rviz')

    print(f'complete world, param setting!')

    # 3. 환경 변수 설정 (가제보가 모델을 찾을 수 있도록)
    set_env_vars = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(my_plugin_dir, 'models')
    )

    # 4. Gazebo Sim 실행 (내가 만든 3D 월드 적용)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            # -s (Server Only): 물리 엔진과 서버만 실행
            # -g (GUI Only): 물리 엔진 없이 화면(클라이언트)만 실행
            # 인자 없음 (기본): 서버와 GUI를 동시에 실행
            'gz_args': f'-r -s -v2 {my_world_file}', # f-string으로 확실하게 결합
            # 'gz_args': f'-r -v2 {my_world_file}', # 화면(클라이언트)만 실행
            'on_exit_shutdown': 'True'              # 디버깅을 위해 False로 변경
        }.items()
    )

    # 5. Robot State Publisher (RViz2에서 로봇 뼈대를 볼 수 있도록 URDF 전달)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_plugin_dir, 'launch', 'robot_state_pub.launch.py')
        ),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    # # 6. 🚨 터틀봇 소환 & 브리지 연결 
    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_plugin_dir, 'launch', 'spawn_robot.launch.py')),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # 7. Nav2 Bringup (알고리즘 및 지도 서버 실행)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': my_params_file,
            'map': my_map_file,
            'use_rviz': 'False'
        }.items()
    )

    # 8. RViz2 독립 실행
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', my_rviz_file],
        parameters=[{'use_sim_time': True}], output='screen'
    )

    # LaunchDescription에 액션 추가 (순서대로)
    ld = LaunchDescription()
    # 1. set env
    ld.add_action(set_env_vars)

    # 2. set gazebo
    ld.add_action(gz_sim)
    # ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)

    # 3. set robot
    ld.add_action(spawn_turtlebot)
    ld.add_action(robot_state_publisher)

    # 4. set nav & viz
    ld.add_action(nav2_bringup)
    ld.add_action(rviz_node)

    return ld