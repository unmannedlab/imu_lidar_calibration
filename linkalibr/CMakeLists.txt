cmake_minimum_required(VERSION 2.8.8)
# Project name
project(linkalibr)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "RELEASE")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native ")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Find catkin (the ROS build system)
find_package(catkin QUIET COMPONENTS
        roscpp
        rosbag
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        ndt_omp
        tf
        imuPacket
        )

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(PCL 1.7 REQUIRED)
#find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# display message to user
message(STATUS "EIGEN VERSION: " ${EIGEN3_VERSION})
message(STATUS "BOOST VERSION: " ${Boost_VERSION})

# Describe catkin project
if (catkin_FOUND)
    add_definitions(-DROS_AVAILABLE=1)
    catkin_package(
            CATKIN_DEPENDS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs imuPacket
            INCLUDE_DIRS src
            LIBRARIES linkalibr_lib
    )
else()
    message(WARNING "CATKIN NOT FOUND BUILDING WITHOUT ROS!")
endif()

# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
#        ${CERES_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CERES_LIBRARIES}
        #        gtsam
        )

##################################################
# Make the core library
##################################################
add_library(linkalibr_lib SHARED
        src/init/InertialInitializer.cpp
        src/state/State.cpp
        src/state/StateHelper.cpp
        src/state/Propagator.cpp
        src/update/UpdaterLidarOdometry.cpp
        src/track/lidarOdometry.cpp
        src/track/PlaneDetector.cpp
        src/core/lincalibManager.cpp
        src/surfel_association/SurfelAssociation.cpp
        )
target_link_libraries(linkalibr_lib ${thirdparty_libraries})
target_include_directories(linkalibr_lib PUBLIC src)

##################################################
# Adding different executables
##################################################
add_executable(ros_test_node src/ros_linkalibr_test.cpp)
target_link_libraries(ros_test_node linkalibr_lib ${catkin_LIBRARIES})

add_executable(ros_ndt_lodom src/ros_ndt_lodom.cpp)
target_link_libraries(ros_ndt_lodom linkalibr_lib ${catkin_LIBRARIES})

add_executable(ros_calib_init src/ros_calib_init.cpp)
target_link_libraries(ros_calib_init linkalibr_lib ${catkin_LIBRARIES})

add_executable(ros_calib_init_optimizer src/ros_calib_init_optimizer.cpp)
target_link_libraries(ros_calib_init_optimizer
        gtsam
#        ${CERES_LIBRARIES}
        ${catkin_LIBRARIES})

#add_executable(ros_batchoptimizer src/ros_batchoptimization.cpp
#                                  src/batch_optimization/LoadPriorKnowledge.cpp
#                                  src/batch_optimization/BatchOptimizer.cpp
#                                  src/batch_optimization/PointToPlaneFactor.cpp
#                                  src/batch_optimization/PointToPlaneFactor2.cpp
#                                  src/batch_optimization/PointToPlaneFactor3.cpp)
#
#target_link_libraries(ros_batchoptimizer gtsam ${catkin_LIBRARIES})





