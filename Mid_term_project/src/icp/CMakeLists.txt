cmake_minimum_required(VERSION 2.8.3)
project(icp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  message_generation
  tf2
  tf_conversions
  eigen_conversions

)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs  # Or other packages containing msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pkg
  CATKIN_DEPENDS
  roscpp
  rospy
  pcl_ros
  geometry_msgs
  nav_msgs std_msgs
  tf2
  message_runtime
  eigen_conversions
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)





add_executable(pcl_load src/pcl_load.cpp )
add_dependencies(pcl_load ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(pcl_load ${catkin_LIBRARIES})


add_executable(downsample src/downsample.cpp )
add_dependencies(downsample ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(downsample ${catkin_LIBRARIES})


add_executable(icp_locolization src/icp_locolization.cpp )
add_dependencies(icp_locolization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(icp_locolization ${catkin_LIBRARIES})

add_executable(icp_locolization1 src/icp_locolization1.cpp )
add_dependencies(icp_locolization1 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(icp_locolization1 ${catkin_LIBRARIES})



add_executable(icp_locolization2 src/icp_locolization2.cpp )
add_dependencies(icp_locolization2 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(icp_locolization2 ${catkin_LIBRARIES})

