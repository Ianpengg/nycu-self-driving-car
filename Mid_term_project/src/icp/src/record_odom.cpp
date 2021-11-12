#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <fstream>
#include <iostream>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  double quatx= msg->pose.pose.orientation.x;
  double quaty= msg->pose.pose.orientation.y;
  double quatz= msg->pose.pose.orientation.z;
  double quatw= msg->pose.pose.orientation.w;

  tf::Quaternion qq(quatx, quaty, quatz, quatw);
  tf::Matrix3x3 mm(qq);
  double roll, pitch, yaw;
  mm.getRPY(roll, pitch, yaw);
  // ROS_INFO(yaw);








}
int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_listener");
  
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("result_odom", 1000, chatterCallback);
    // Create a new file

    // Write to the file
    ;
    //newFile.close();




  ros::spin();
  return 0;

}