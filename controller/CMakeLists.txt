cmake_minimum_required(VERSION 3.16)    # pch(precompile header) is from version 3.16
project(mpc_controller)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# For Debugging
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
# 명시적으로 디버그 기호(-g) 컴파일 플래그 강제 적용
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# ROS 2용 빌드 도구 (추후 colcon build를 위해 유지)
find_package(ament_cmake REQUIRED) 
find_package(OpenCV REQUIRED)

############### START : Make main standalone exe file ###############
# 1. acados 설치 경로 수동 지정 (사용자 환경에 맞게 수정 필수!)
set(ACADOS_DIR "/home/uj/acados")

# 2. acados 기본 헤더 및 생성된 C 모델 헤더 경로 통합 추가
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    mpc/include
    ${ACADOS_DIR}/include
    ${ACADOS_DIR}/include/acados
    ${ACADOS_DIR}/include/blasfeo/include
    ${ACADOS_DIR}/include/hpipm/include
    # 파이썬이 생성한 C 코드 폴더 경로 (세 모델 모두 포함되도록 설정)
    mpc/state_scripts/c_generated_bicycle_model
    mpc/state_scripts/c_generated_parallel_model
    mpc/state_scripts/c_generated_spin_model
)

# 3. acados 기본 라이브러리 폴더 경로 지정
link_directories(${ACADOS_DIR}/lib)

# 4. 파이썬이 생성한 동적 라이브러리(.so)들을 타겟으로 가져오기
add_library(acados_bicycle_lib SHARED IMPORTED)
set_target_properties(acados_bicycle_lib PROPERTIES 
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/mpc/state_scripts/c_generated_bicycle_model/libacados_solver_bicycle_model.so
)

add_library(acados_parallel_lib SHARED IMPORTED)
set_target_properties(acados_parallel_lib PROPERTIES 
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/mpc/state_scripts/c_generated_parallel_model/libacados_solver_parallel_model.so
)

add_library(acados_spin_lib SHARED IMPORTED)
set_target_properties(acados_spin_lib PROPERTIES 
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/mpc/state_scripts/c_generated_spin_model/libacados_solver_spin_model.so
)
# 4-1) add maps_lib
add_library(maps_lib 
    ${CMAKE_CURRENT_SOURCE_DIR}/../maps/src/maps.cpp
)
target_include_directories(maps_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../maps/include    # maps의 include 폴더 지정
    ${CMAKE_CURRENT_SOURCE_DIR}/../commons/include            # for struct.h
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
# 4-2) add commons_lib
add_library(common_planner_lib 
    ${CMAKE_CURRENT_SOURCE_DIR}/../commons/src/visualize.cpp
)
target_include_directories(common_planner_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../commons/include    # maps의 include 폴더 지정
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# 5. 실행 파일(main standalone exe) 생성
add_executable(run_mpc
    mpc/src/main_mpc.cpp   
)

# 6. 필수 라이브러리 풀세트 링크 (acados 코어 + 수학 엔진 + 모델 라이브러리)
target_link_libraries(run_mpc
    acados         # 메인 솔버
    hpipm          # 2차 계획법(QP) 엔진
    blasfeo        # 고성능 선형대수 연산 엔진
    m              # C 표준 수학 라이브러리
    acados_bicycle_lib
    acados_parallel_lib
    acados_spin_lib
    maps_lib
    common_planner_lib
    ${OpenCV_LIBS}
)

# 7. Precompile Header Setting (빌드 속도 향상)
target_precompile_headers(run_mpc 
    PRIVATE
    <iostream>
    <vector>
    <cmath>
    <filesystem>
    <fstream>
    <algorithm>
    <opencv2/opencv.hpp>
    "base_controller.h"
    "velocity_planner.h"
    # "mpc/include/structs.h"
    # "mpc/include/mpc_controller.h"
    # "mpc/include/vehicles_mpc.h"
)
############### END : Make main standalone exe file ###############

# ############### START : Make Shared Libraries and install for ROS2 ###############
# # 2. 플러그인에서 사용할 공유 라이브러리(Shared Library : extended_hastar_lib) 생성
# add_library(extended_hastar_lib SHARED
#     src/maps.cpp
#     src/astar.cpp
#     src/vehicles.cpp
#     src/xhastar.cpp
#     src/visualize.cpp
# )

# # 3. 라이브러리가 헤더 파일을 찾을 수 있도록 경로 지정 (PUBLIC으로 설정해야 다른 패키지에서도 이 경로를 알 수 있습니다)
# target_include_directories(extended_hastar_lib PUBLIC
#     include
#     ${OpenCV_INCLUDE_DIRS}
#     ${EIGEN3_INCLUDE_DIRS}
#     ${OMPL_INCLUDE_DIRS}
# )

# # 4. 라이브러리 컴파일에 필요한 외부 패키지 링크
# target_link_libraries(extended_hastar_lib
#     ${OpenCV_LIBS}
#     ${OMPL_LIBRARIES}
# )

# # 5. 라이브러리와 include 폴더 안의 헤더 파일들을 다른 패키지(nav2_plugins)가 쓸 수 있도록 설치(install)
# install(TARGETS extended_hastar_lib
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )
# install(DIRECTORY include/
#   DESTINATION include
# )

# # 6. ament 패키지로 내보내기
# ament_export_include_directories(include)
# ament_export_libraries(extended_hastar_lib)
# ament_export_dependencies(OpenCV ompl Eigen3)
# ament_package()
# ############### END : Make Shared Libraries and install for ROS2 ###############

