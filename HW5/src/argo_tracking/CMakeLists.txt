cmake_minimum_required(VERSION 2.8.3)
project(argo_tracking)


find_package(catkin REQUIRED
	pcl_ros
	roscpp
	sensor_msgs
	std_msgs
	pcl_conversions
	rospy
	tf
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS 
	pcl_ros
	roscpp
	sensor_msgs
	pcl_conversions
)

include_directories(
	include
	src
	${catkin_INCLUDE_DIRS}
)
#import to avoid json error
set(lib_DIR /usr/lib/x84_64-linux-gnu)
link_directories(${lib_DIR})

#remember to link libjsoncpp.a to your executable files.
add_executable(viz_track_result	src/viz_track_result.cpp)	
target_link_libraries(viz_track_result ${catkin_LIBRARIES} libjsoncpp.a) # add libjsoncpp.a



