cmake_minimum_required(VERSION 3.16.3)
project(ros_basalt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -DEIGEN_DONT_PARALLELIZE -Wall -Wextra -Werror -Wno-error=unused-parameter -ftemplate-backtrace-limit=0 -Wno-error=deprecated-declarations -Wno-error=maybe-uninitialized -Wno-error=implicit-fallthrough")
set(STD_CXX_FS stdc++fs)
# set(CMAKE_CXX_FLAGS_RELEASE "-g")


find_package(catkin REQUIRED COMPONENTS
roscpp
sensor_msgs
geometry_msgs
)
find_package(realsense2 REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
include
${catkin_INCLUDE_DIRS}
${realsense2_INCLUDE_DIR}
${EIGEN3_INCLUDE_DIR}
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS roscpp
)

add_library(${PROJECT_NAME}
src/ros_basalt/device/rs_t265.cpp
src/ros_basalt/linearization/landmark_block.cpp
src/ros_basalt/linearization/linearization_base.cpp
src/ros_basalt/linearization/linearization_abs_qr.cpp
src/ros_basalt/linearization/linearization_abs_sc.cpp
src/ros_basalt/linearization/linearization_rel_sc.cpp
src/ros_basalt/optical_flow/optical_flow.cpp
src/ros_basalt/utils/time_utils.cpp
src/ros_basalt/utils/system_utils.cpp
src/ros_basalt/utils/keypoints.cpp
src/ros_basalt/utils/vio_config.cpp
src/ros_basalt/vi_estimator/ba_base.cpp
src/ros_basalt/vi_estimator/sc_ba_base.cpp
src/ros_basalt/vi_estimator/sqrt_ba_base.cpp
src/ros_basalt/vi_estimator/landmark_database.cpp
src/ros_basalt/vi_estimator/marg_helper.cpp
src/ros_basalt/vi_estimator/vio_estimator.cpp
src/ros_basalt/vi_estimator/sqrt_keypoint_vo.cpp
src/ros_basalt/vi_estimator/sqrt_keypoint_vio.cpp
)
target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
${STD_CXX_FS}
realsense2
TBB::tbb
fmt
${OpenCV_LIBS}
opengv
)
target_compile_definitions(${PROJECT_NAME} PUBLIC
BASALT_INSTANTIATIONS_FLOAT
)


add_executable(${PROJECT_NAME}_rs_t265_vio scripts/rs_t265_vio.cpp)
target_link_libraries(${PROJECT_NAME}_rs_t265_vio
${PROJECT_NAME}
)