cmake_minimum_required(VERSION 3.0.2)
project(ORB_SLAM2)

IF(NOT ROS_BUILD_TYPE)
    SET(ROS_BUILD_TYPE Release)
ENDIF()
MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -w  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -O3 -march=native")
set(CMAKE_CXX_STANDARD 11)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        cv_bridge
        tf
        nav_msgs
        geometry_msgs
        )

find_package(OpenCV QUIET)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/examples/)
add_executable(Mono
        examples/ros_mono.cc
        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Map.cc
        src/MapDrawer.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Initializer.cc
        src/Viewer.cc
        src/CameraPoseVisualization.cc
        )
target_link_libraries(Mono
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        /usr/lib/x86_64-linux-gnu/libboost_system.so
        )
