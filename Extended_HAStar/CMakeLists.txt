# cmake_minimum_required(VERSION 3.10)
cmake_minimum_required(VERSION 3.16)    # pch(precompile header) is from version 3.16
project(extended_hastar)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 외부 패키지 찾기 (OpenCV, Eigen3, OMPL) 
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(ament_cmake REQUIRED) # ament_cmake <- for ROS2!!

############### START : Make main standalone exe file ###############
# 2. 실행 파일(main standalone exe) 생성 및 소스/헤더 명시
add_executable(run_xhastar
    # 소스 파일
    src/main_pathplan.cpp   
    src/maps.cpp
    src/astar.cpp
    src/vehicles.cpp
    src/xhastar.cpp
    src/visualize.cpp
    
    # 헤더 파일
    include/astar.h
    include/costmap.h
    include/maps.h
    include/structs.h
    include/vehicles.h
    include/visualize.h
    include/xhastar.h
)
# 3. 헤더 파일 경로 설정
target_include_directories(run_xhastar PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)

# 4. 라이브러리 링크 🔗
# Eigen은 대부분 헤더 파일만으로 동작하므로 여기에 명시하지 않아도 됩니다.
target_link_libraries(run_xhastar
    ${OpenCV_LIBS}
    ${OMPL_LIBRARIES}
)

# 5. key!! precompile header Setting
target_precompile_headers(run_xhastar 
    PRIVATE
    <iostream>
    <vector>
    <cmath>
    <opencv2/opencv.hpp>
    <Eigen/Dense>
    <ompl/base/spaces/ReedsSheppStateSpace.h>
    <ompl/base/spaces/DubinsStateSpace.h>
    <ompl/base/State.h>
    <filesystem>
)
############### END : Make main standalone exe file ###############

############### START : Make Shared Libraries and install for ROS2 ###############
# 2. 플러그인에서 사용할 공유 라이브러리(Shared Library : extended_hastar_lib) 생성
add_library(extended_hastar_lib SHARED
    src/maps.cpp
    src/astar.cpp
    src/vehicles.cpp
    src/xhastar.cpp
    src/visualize.cpp
)

# 3. 라이브러리가 헤더 파일을 찾을 수 있도록 경로 지정 (PUBLIC으로 설정해야 다른 패키지에서도 이 경로를 알 수 있습니다)
target_include_directories(extended_hastar_lib PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)

# 4. 라이브러리 컴파일에 필요한 외부 패키지 링크
target_link_libraries(extended_hastar_lib
    ${OpenCV_LIBS}
    ${OMPL_LIBRARIES}
)

# 5. 라이브러리와 include 폴더 안의 헤더 파일들을 다른 패키지(nav2_plugins)가 쓸 수 있도록 설치(install)
install(TARGETS extended_hastar_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

# 6. ament 패키지로 내보내기
ament_export_include_directories(include)
ament_export_libraries(extended_hastar_lib)
ament_export_dependencies(OpenCV ompl Eigen3)
ament_package()
############### END : Make Shared Libraries and install for ROS2 ###############

