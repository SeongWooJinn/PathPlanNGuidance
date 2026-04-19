cmake_minimum_required(VERSION 3.8)
project(nav2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(extended_hastar REQUIRED)

############## START : CUSTOM PLUGIN INSTALL ##############
# 플러그인 라이브러리를 만듭니다.
add_library(nav2_extended_hastar_plugin SHARED
  src/Extended_HAStar_plugin.cpp
)

# ROS 2 패키지들과 우리가 만든 알고리즘 패키지의 헤더/라이브러리를 한 번에 플러그인 라이브러리에 자동 연결! 🔗
ament_target_dependencies(nav2_extended_hastar_plugin
  rclcpp
  nav2_core
  nav2_costmap_2d
  pluginlib
  geometry_msgs
  nav_msgs
  rclcpp_lifecycle
  extended_hastar  # 우리의 알고리즘 패키지도 이름만 적으면 됩니다!
)

# ament 패키지가 아닌 외부 패키지들의 헤더와 라이브러리는 수동으로 직접 연결
target_include_directories(nav2_extended_hastar_plugin PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)
target_link_libraries(nav2_extended_hastar_plugin
  ${OpenCV_LIBS}
  ${OMPL_LIBRARIES}
)

# 플러그인 라이브러리 설치
install(TARGETS nav2_extended_hastar_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
############## END : CUSTOM PLUGIN INSTALL ##############

# 폴더 복사
install(DIRECTORY
  config behavior_trees launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

# xml 파일을 패키지의 공유(share) 폴더로 복사
# ~PathPlan_ws/install/nav2_plugins/share/ament_index/resource_index에 nav2_core가 없어서 수동 설치
install(FILES 
  global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
# pluginlib이 이 xml 파일을 찾아 인덱스(전화번호부)를 생성하도록 명시합니다.
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)


ament_package()
