cmake_minimum_required(VERSION 2.8.3)

project(orb_slam3_wrapper)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        cv_bridge
        std_msgs
        sensor_msgs
        geometry_msgs
        image_transport
        nodelet
        pluginlib
        std_srvs
        rosbag
        nav_msgs
        )

find_package(Boost REQUIRED COMPONENTS system)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES orb_slam3_wrapper
        CATKIN_DEPENDS roscpp tf tf2 tf2_geometry_msgs tf2_ros cv_bridge std_msgs sensor_msgs geometry_msgs image_transport nodelet pluginlib
        DEPENDS Boost
)

set(ORB_SLAM3_DIR /home/user/lab3/3rdparty/ORB_SLAM3/)
set(ORB_SLAM3_LIBS
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
        ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
        -lboost_system
        )
#set(EIGEN_INCLUDE_DIRS "/lab/3rdparty/eigen")

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${ORB_SLAM3_DIR}
        ${ORB_SLAM3_DIR}/include
        ${Pangolin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
#        /lab/3rdparty/eigen
)


add_executable(orb_slam3_mono_inertial src/orb_slam3_mono_inertial.cpp)
target_link_libraries(orb_slam3_mono_inertial  ${catkin_LIBRARIES} ${ORB_SLAM3_LIBS})

add_executable(orb_slam3_mono src/orb_slam3_mono.cpp)
target_link_libraries(orb_slam3_mono ${catkin_LIBRARIES} ${ORB_SLAM3_LIBS})
