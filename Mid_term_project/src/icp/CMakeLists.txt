cmake_minimum_required(VERSION 2.8.3)
project(icp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

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



catkin_package(

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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(downsample src/downsample.cpp )
add_dependencies(downsample ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(downsample ${catkin_LIBRARIES})


add_executable(ICP_localization_1 src/ICP_localization_1.cpp )
add_dependencies(ICP_localization_1 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(ICP_localization_1 ${catkin_LIBRARIES})

add_executable(ICP_localization_2 src/ICP_localization_2.cpp )
add_dependencies(ICP_localization_2 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(ICP_localization_2 ${catkin_LIBRARIES})

add_executable(ICP_localization_3 src/ICP_localization_3.cpp )
add_dependencies(ICP_localization_3 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(ICP_localization_3 ${catkin_LIBRARIES})

add_executable(NDT_localization_3 src/NDT_localization_3.cpp )
add_dependencies(NDT_localization_3 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(NDT_localization_3 ${catkin_LIBRARIES})
