cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(normal_distributions_transform)

set(CMAKE_CXX_STANDARD 11)

FIND_PACKAGE(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        tf
        geometry_msgs
        )
include_directories(/opt/holo/3rdparty/include/pcl-1.8)
include_directories(
        ${catkin_INCLUDE_DIRS}
)
#或者
#include_directories(/usr/include/pcl-1.7)



include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(normal_distributions_transform normal_distributions_transform.cpp )
target_link_libraries (normal_distributions_transform ${PCL_LIBRARIES}    ${catkin_LIBRARIES})
