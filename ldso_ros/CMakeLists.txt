cmake_minimum_required (VERSION 2.8.11)

find_package(catkin REQUIRED)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(ldso)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  cv_bridge
)

set(LDSO_PATH $ENV{LDSO_PATH})

LIST(APPEND CMAKE_MODULE_PATH ${LDSO_PATH}/cmake)

message("\n\n---- LOOKING FOR LDSO at\n\"${LDSO_PATH}\"")


find_package(Pangolin 0.2 REQUIRED)
message("\n\n---- FOUND Pangolin Headers at\n\"${Pangolin_INCLUDE_DIRS}\"")
message("---- FOUND Pangolin Libs at\n\"${Pangolin_LIBRARIES}\"")

find_package(OpenCV REQUIRED)
message("\n\n---- FOUND OpenCV Headers at\n\"${OpenCV_INCLUDE_DIRS}\"")
message("---- FOUND OpenCV Libs at\n\"${OpenCV_LIBS}\"")

find_library(LDSO_LIBRARY ldso ${LDSO_PATH}/build/lib)

find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(LibZip QUIET)


catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    sensor_msgs
    cv_bridge
)

set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O2 -g -std=c++0x -march=native -fno-omit-frame-pointer"
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${LDSO_PATH}/src
  ${LDSO_PATH}/thirdparty/Sophus
  ${Glog_INCLUDE_DIRS}

  ${PROJECT_SOURCE_DIR}/include
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/examples
  ${PROJECT_SOURCE_DIR}/thirdparty/
  ${PROJECT_SOURCE_DIR}/thirdparty/g2o/
  ${PROJECT_SOURCE_DIR}/thirdparty/DBoW3/
)

## Declare a C++ executable
add_executable(ldso_live src/main.cpp)

target_link_libraries(ldso_live
    ${LDSO_LIBRARY}
    ${Pangolin_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    ${GLOG_LIBRARY}
    ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${PROJECT_SOURCE_DIR}/thirdparty/DBoW3/build/src/libDBoW3${CMAKE_SHARED_LIBRARY_SUFFIX}
    ${LIBZIP_LIBRARY})

