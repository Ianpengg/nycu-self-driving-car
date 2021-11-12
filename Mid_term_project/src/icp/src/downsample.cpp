#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.
#include <pcl/filters/passthrough.h>
int main (int argc, char** argv)
{
  ros::init (argc, argv, "Downsample");

  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  pcl::PointCloud<pcl::PointXYZI>::Ptr map (new pcl::PointCloud<pcl::PointXYZI>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read ("src/maps/nuscenes_map.pcd", *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.5f, 0.5f, 0.5f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

  pcl::PassThrough<pcl::PCLPointCloud2> pass1;
  pass1.setInputCloud(cloud_filtered);
  pass1.setFilterFieldName("x");
  pass1.setFilterLimits(1650.0, 2100.0);
  pass1.filter(*cloud_filtered);//儲存處理之後的點雲
  pcl::PCDWriter writer;
  writer.write ("src/maps/nuscene_downsample.pcd", *cloud_filtered,
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  printf("Downsample and write Done\n");

}
