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
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
namespace ndt_2d
{
  class NDTRegistration
  {
  public:
    NDTRegistration();
    void setInitialPose(const Eigen::Vector3d &pose);
    void addMapPCD(const nav_msgs::msg::OccupancyGrid &map);
    void addScanPCD(std::vector<Eigen::Vector3d> &points);
    Eigen::Matrix4f transform3D(double x, double y, double z, double yaw);
    Eigen::Vector3d matchScans();
    void convertEigenToPose(const Eigen::Matrix4f &transformation, geometry_msgs::msg::Pose &pose);
    void convertEigenToXYTheta(const Eigen::Matrix4f &transformation, Eigen::Vector3d &pose);
  private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_pts;
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pts;
  Eigen::Vector3d m_initial_pose;
  
  
  };
} // namespace ndt_2d
#endif

