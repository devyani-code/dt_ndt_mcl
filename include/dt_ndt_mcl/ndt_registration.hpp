#ifndef NDT_2D__NDT_REGISTRATION_HPP_
#define NDT_2D__NDT_REGISTRATION_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dt_ndt_mcl/point.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <sstream>
#include <chrono> 
#include <ctime> 
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h> 
#include <pcl/point_cloud.h>
#include <geometry_msgs/msg/pose.hpp>

namespace ndt_2d
{
  class NDTRegistration
  {
  public:
    NDTRegistration();
    void addMapPCD(std::vector<Eigen::Vector3d> &points);
    void addScanPCD(std::vector<Eigen::Vector3d> &points);
    Eigen::Matrix4d matchScans( Eigen::Vector3d &pose);
  private:
  pcl::PointCloud<pcl::PointXYZ> scan_pts;
  pcl::PointCloud<pcl::PointXYZ> map_pts;
  
};