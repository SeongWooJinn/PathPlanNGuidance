import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  

def generate_launch_description():
    # 1. nav2_bringup 패키지의 경로 찾기
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 2. 내 패키지(nav2_plugins)의 경로 찾기
    # 하드코딩된 절대 경로 대신, ROS2 패키지 시스템을 이용해 경로를 유연하게 잡습니다.
    my_plugin_dir = get_package_share_directory('nav2_plugins')
    my_params_file = os.path.join(my_plugin_dir, 'config', 'nav2_params.yaml')
    my_rviz_file = os.path.join(my_plugin_dir, 'rviz', 'my_rviz2_config.rviz')

    # 3. tb3_simulation_launch.py 가져오기 및 파라미터 덮어쓰기
    tb3_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'tb3_simulation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': my_params_file,
            'use_rviz': 'False'
            # 'rviz_config_file': my_rviz_file
            # 'headless': 'False' # 가제보 화면을 끄고 싶다면 True로 설정 가능
        }.items()
    )

    # create my rviz node 
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', my_rviz_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 4. 실행할 런치 파일들을 Description에 담아 반환
    ld = LaunchDescription()
    ld.add_action(tb3_sim_launch)
    ld.add_action(rviz_node)

    return ld