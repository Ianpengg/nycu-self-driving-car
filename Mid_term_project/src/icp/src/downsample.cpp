#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
int main (int argc, char** argv)
{
  ros::init (argc, argv, "Downsample");

  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  pcl::PointCloud<pcl::PointXYZI>::Ptr map (new pcl::PointCloud<pcl::PointXYZI>);

  // Fill in the cloud data
  pcl::PCDReader reader;

  // There are two origin map, ITRI and Nuscenes, you just need to replace the file_path.
  //reader.read ("/home/ee904/Documents/nycu-self-driving-car/Mid_term_project/src/icp/maps/nuscenes_map.pcd", *cloud); 
  reader.read ("/home/ee904/Documents/nycu-self-driving-car/Mid_term_project/src/icp/maps/itri_map.pcd", *cloud); 
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.5f, 0.5f, 0.5f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;
  pcl::PCDWriter writer;
  //writer.write ("/home/ee904/Documents/nycu-self-driving-car/Mid_term_project/src/icp/maps/nu_downsample.pcd", *cloud_filtered,
  //       Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
  writer.write ("/home/ee904/Documents/nycu-self-driving-car/Mid_term_project/src/icp/maps/itri_downsample.pcd", *cloud_filtered,
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
  printf("Downsample and write Done\n");

}
