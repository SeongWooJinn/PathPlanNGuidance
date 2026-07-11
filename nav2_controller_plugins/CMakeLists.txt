cmake_minimum_required(VERSION 3.16)
project(nav2_controller_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)  # -g: for debug mode
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release) # ROS2 환경이므로 Release 또는 RelWithDebInfo 추천
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ROS 2용 빌드 도구 (추후 colcon build를 위해 유지)
find_package(ament_cmake REQUIRED) 
find_package(OpenCV REQUIRED)

set(CONTROLLER_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../controller)
set(COMMONS_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../commons)

# acados 설치 경로 (사용자 환경에 맞게 유지)
set(ACADOS_DIR "/home/uj/acados")

# 3. 파이썬이 생성한 동적 라이브러리(.so)들을 타겟으로 가져오기
add_library(acados_bicycle_lib SHARED IMPORTED)
set_target_properties(acados_bicycle_lib PROPERTIES 
    IMPORTED_LOCATION ${CONTROLLER_DIR}/mpc/state_scripts/c_generated_bicycle_model/libacados_solver_bicycle_model.so
)

add_library(acados_parallel_lib SHARED IMPORTED)
set_target_properties(acados_parallel_lib PROPERTIES 
    IMPORTED_LOCATION ${CONTROLLER_DIR}/mpc/state_scripts/c_generated_parallel_model/libacados_solver_parallel_model.so
)

add_library(acados_spin_lib SHARED IMPORTED)
set_target_properties(acados_spin_lib PROPERTIES 
    IMPORTED_LOCATION ${CONTROLLER_DIR}/mpc/state_scripts/c_generated_spin_model/libacados_solver_spin_model.so
)

# acados 기본 라이브러리 폴더 경로 지정
link_directories(${ACADOS_DIR}/lib)

# 4. Nav2 플러그인(Shared Library) 생성
# 기존 controller의 main_mpc.cpp를 제외한 알고리즘 구현 파일과
# 새로 작성할 ROS2 래퍼용 mpc_controller_plugin.cpp를 함께 빌드합니다.
add_library(${PROJECT_NAME} SHARED
    src/mpc_controller_plugin.cpp              # (신규) ROS2 플러그인 인터페이스 구현 파일
    ${COMMONS_DIR}/src/visualize.cpp           # 기존 commons 소스 파일
    # 만약 controller 내부에 추가적인 cpp 구현체가 있다면 여기에 추가하세요 (예: base_controller.cpp 등)
)

# 5. 헤더 파일(Include) 경로 통합
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CONTROLLER_DIR}
    ${CONTROLLER_DIR}/mpc/include
    ${CONTROLLER_DIR}/mpc/state_scripts/c_generated_bicycle_model
    ${CONTROLLER_DIR}/mpc/state_scripts/c_generated_parallel_model
    ${CONTROLLER_DIR}/mpc/state_scripts/c_generated_spin_model
    ${COMMONS_DIR}/include
    ${ACADOS_DIR}/include
    ${ACADOS_DIR}/include/acados
    ${ACADOS_DIR}/include/blasfeo/include
    ${ACADOS_DIR}/include/hpipm/include
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# 6. 라이브러리 링크 (Acados + OpenCV + ROS2)
target_link_libraries(${PROJECT_NAME} PUBLIC
    acados         
    hpipm          
    blasfeo        
    m              
    acados_bicycle_lib
    acados_parallel_lib
    acados_spin_lib
    ${OpenCV_LIBS}
)

# ROS 2 의존성 적용
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_lifecycle
    nav2_core
    nav2_costmap_2d
    nav2_util
    pluginlib
    geometry_msgs
    nav_msgs
    sensor_msgs
)

# 7. 플러그인 XML 파일 등록 (Nav2에서 플러그인을 찾을 수 있게 해줌)
pluginlib_export_plugin_description_file(nav2_core 
  local_planner_plugin.xml
)

# 8. Install 설정 (ROS2 빌드 후 install 폴더로 파일 복사)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# 중요: Acados에서 생성된 .so 파일들을 ROS2 런타임이 찾을 수 있도록 install/lib 폴더로 복사합니다.
install(FILES 
    ${CONTROLLER_DIR}/mpc/state_scripts/c_generated_bicycle_model/libacados_solver_bicycle_model.so
    ${CONTROLLER_DIR}/mpc/state_scripts/c_generated_parallel_model/libacados_solver_parallel_model.so
    ${CONTROLLER_DIR}/mpc/state_scripts/c_generated_spin_model/libacados_solver_spin_model.so
    DESTINATION lib
)

install(FILES 
  local_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_package()
