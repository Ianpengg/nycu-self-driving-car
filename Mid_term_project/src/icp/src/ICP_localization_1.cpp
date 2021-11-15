#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "math.h"
#include <fstream>
#include <string>
#include <sstream>


using namespace ros;
using namespace std;

class Localization {
  private:
    ros::Subscriber sub_lidar_scan;
    ros::Publisher pub_pc_after_icp, pub_result_odom, pub_map;
    ros::NodeHandle nh;

    sensor_msgs::PointCloud2 map_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map;
    pcl::VoxelGrid<pcl::PointXYZI> map_voxel;

    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;

    Eigen::Matrix4f initial_guess;
    Eigen::Quaterniond q;
    ofstream outFile;
    int pub_count=0;

  public:
    Localization();
    Eigen::Matrix4f get_initial_guess();
    Eigen::Matrix4f get_transfrom(std::string link_name);
    void cb_lidar_scan(const sensor_msgs::PointCloud2 &msg);
};

Localization::Localization() {
  // load the LiDAR map & readin
  map.reset(new pcl::PointCloud<pcl::PointXYZI>);

  if (pcl::io::loadPCDFile<pcl::PointXYZI> ("src/maps/map_downsample.pcd", *map) == -1) 
  {
    PCL_ERROR ("Couldn't read file map.pcd \n");
    exit(0);
  }
  cout << "map size:" << map->size() << endl;
  cout << "---------------------------" << endl;
  
  //=====================PassThrough Filter==================================
  // Cut the map with passthrough filter to lower the computation time
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud (map);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-1000, 50);
  pass.filter (*map);
  cout<<"Passthrough filter: "<<map->points.size()<<endl;
  pcl::toROSMsg(*map, map_cloud);


  // publishers and subscribers 
  // If there is any Rviz transform error please try to add / before each topic name 

  sub_lidar_scan = nh.subscribe("lidar_points", 400, &Localization::cb_lidar_scan, this);
  pub_pc_after_icp = nh.advertise<sensor_msgs::PointCloud2>("pc_after_icp", 1);
  pub_result_odom = nh.advertise<nav_msgs::Odometry>("result_odom", 1);
  pub_map = nh.advertise<sensor_msgs::PointCloud2>("map", 1);
  

  // Initial guess

  int init_x= -285.456721951;
  int init_y= 225.77162962;
  int init_z= -12.4146628257;
  double yaw=2.43;
  initial_guess<< cos(yaw), -sin(yaw), 0,  init_x,
                  sin(yaw), cos(yaw),  0,  init_y,
			            0,        0,         1,  init_z,
			            0,        0,         0,  1;
  // write the result to csv file
  outFile.open("Q1result.csv", ios::out);
  outFile << "id,x,y,z,yaw,pitch,roll"<< endl;
  printf("init done \n");
}

//===============Frame conversion from velodyne to base_link====================
Eigen::Matrix4f Localization::get_transfrom(std::string link_name){
	tf::StampedTransform transform;
	Eigen::Matrix4f trans;

	try {
		ros::Duration five_seconds(5.0);
		listener.waitForTransform("base_link", link_name, ros::Time(0), five_seconds);
		listener.lookupTransform("base_link", link_name, ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return trans;
	}
	Eigen::Quaternionf q(transform.getRotation().getW(), transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());
	Eigen::Matrix3f mat = q.toRotationMatrix();
	trans << mat(0,0), mat(0,1), mat(0,2), transform.getOrigin().getX(),
			     mat(1,0), mat(1,1), mat(1,2), transform.getOrigin().getY(),
			     mat(2,0), mat(2,1), mat(2,2), transform.getOrigin().getZ(),
			            0,        0,        0,        1;
	return trans;
}




void Localization::cb_lidar_scan(const sensor_msgs::PointCloud2 &msg) {
    
  pcl::PointCloud<pcl::PointXYZI>::Ptr bag_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PCLPointCloud2::Ptr bag_cloud_filtered (new pcl::PCLPointCloud2 ());


  Eigen::Matrix4f trans = get_transfrom("velodyne");
  pcl::fromROSMsg(msg, *bag_pointcloud);
	transformPointCloud (*bag_pointcloud, *bag_pointcloud, trans);

  
  ROS_INFO("transformed to base_link");
  cout << "original: " << bag_pointcloud->points.size() << endl;




  //=======================Voxelgrid filter=====================================
  pcl::toPCLPointCloud2(*bag_pointcloud, *bag_cloud_filtered);
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
  voxel.setInputCloud (bag_cloud_filtered);
  voxel.setLeafSize (0.13f, 0.13f, 0.13f);
  voxel.filter (*bag_cloud_filtered);
  pcl::fromPCLPointCloud2(*bag_cloud_filtered, *bag_pointcloud);
  cout << "voxel grid filter: " << bag_pointcloud->points.size() << endl;


  // //=======================PassThrough filter===================================
  // //=======================Filter X direction===================================
  // pcl::PassThrough<pcl::PointXYZI> pass;
  // pass.setInputCloud(bag_pointcloud);
  // pass.setFilterFieldName("x");
  // pass.setFilterLimits(-20.0, 130.0);
  // pass.filter(*bag_pointcloud);
  // cout<<"Passthrough filter: "<<bag_pointcloud->points.size()<<endl;


  // //=======================PassThrough filter===================================
  // //=======================Filter Y direction===================================
  // pass.setInputCloud(bag_pointcloud);
  // pass.setFilterFieldName("y");
  // pass.setFilterLimits(-20.0, 40.0);
  // pass.filter(*bag_pointcloud);


  //=====================ICP Implementation=====================================
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setInputSource(bag_pointcloud);
  icp.setInputTarget(map);
  icp.setMaximumIterations (1000);
  icp.setTransformationEpsilon (1e-13);
  icp.setMaxCorrespondenceDistance (1);
  icp.setEuclideanFitnessEpsilon (1e-5);
  icp.setRANSACOutlierRejectionThreshold (0.02);
  
  pcl::PointCloud<pcl::PointXYZI> Final;
  icp.align(Final, initial_guess);

  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  initial_guess = icp.getFinalTransformation();



  //=================TF transform broadcaster===================================
  tf::Matrix3x3 tf3d;
  tf3d.setValue((initial_guess(0,0)), (initial_guess(0,1)), (initial_guess(0,2)),
        (initial_guess(1,0)), (initial_guess(1,1)), (initial_guess(1,2)),
        (initial_guess(2,0)), (initial_guess(2,1)), (initial_guess(2,2)));
  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(initial_guess(0,3),initial_guess(1,3),initial_guess(2,3)));
  transform.setRotation(tfqt);
  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"world","base_link"));




  // Publish your lidar scan pointcloud after doing ICP.
  sensor_msgs::PointCloud2 matched_cloud;
  pcl::toROSMsg(Final, matched_cloud);
  matched_cloud.header=msg.header;
  matched_cloud.header.frame_id = "world";
  pub_pc_after_icp.publish(matched_cloud);

  //==========================Show map==========================================
  map_cloud.header.frame_id = "world";
  map_cloud.header.stamp = Time::now();
  pub_map.publish(map_cloud);

  // Publish localization result as nav_msgs/Odometry.msg message type.
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = initial_guess(0,3);
  odom.pose.pose.position.y = initial_guess(1,3);
  odom.pose.pose.position.z = initial_guess(2,3);
  tf2::Matrix3x3 m;
  m.setValue(initial_guess(0,0) ,initial_guess(0,1) ,initial_guess(0,2) ,
              initial_guess(1,0) ,initial_guess(1,1) ,initial_guess(1,2) ,
              initial_guess(2,0) ,initial_guess(2,1) ,initial_guess(2,2));
  tf2::Quaternion tfq2;
  m.getRotation(tfq2);
  odom.pose.pose.orientation.x = tfq2[0];
  odom.pose.pose.orientation.y = tfq2[1];
  odom.pose.pose.orientation.z = tfq2[2];
  odom.pose.pose.orientation.w = tfq2[3];
  pub_result_odom.publish(odom);


  // Transform the odom message to roll,pitch,yaw
  float linearposx=odom.pose.pose.position.x;
  float linearposy=odom.pose.pose.position.y;
  double quatx= odom.pose.pose.orientation.x;
  double quaty= odom.pose.pose.orientation.y;
  double quatz= odom.pose.pose.orientation.z;
  double quatw= odom.pose.pose.orientation.w;

  tf::Quaternion qua_element(quatx, quaty, quatz, quatw);
  tf::Matrix3x3 result(qua_element);
  double roll, pitch, yaw;
  result.getRPY(roll, pitch, yaw);

  
  cout<< "pub_count=" << pub_count + 1 << endl;
  outFile << pub_count + 1 << ',' << odom.pose.pose.position.x << ',' << 
  odom.pose.pose.position.y << ',' << odom.pose.pose.position.z <<
  ','<< yaw << ',' << pitch << ',' << roll << endl;
  
  pub_count++;
  
  if (pub_count==201){
    cout<<"close file"<<endl;
    outFile.close();
  }
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "Localization");
  Localization icp;
  ros::spin();
}