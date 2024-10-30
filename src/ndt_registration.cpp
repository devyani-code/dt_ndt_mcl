#include <dt_ndt_mcl/ndt_registration.hpp>

namespace ndt_2d
{
    NDTRegistration::NDTRegistration(): map_pts(new pcl::PointCloud<pcl::PointXYZ>), scan_pts(new pcl::PointCloud<pcl::PointXYZ>)
    {}
    
  void NDTRegistration::addMapPCD(std::vector<Eigen::Vector3d> &points)
  {

    for (auto &point : points)
    {
      map_pts->push_back(pcl::PointXYZ(point(0), point(1), point(2)));
    }
    
  }

  void NDTRegistration::addScanPCD(std::vector<Eigen::Vector3d> &points)
  {
    for (auto &point : points)
    {
      scan_pts->push_back(pcl::PointXYZ(point(0), point(1), point(2)));
    }
  }

  Eigen::Matrix4d NDTRegistration::transform3D(double x, double, double z, double yaw){
    Eigen::Matrix3d R_yaw;
    R_yaw << cos(yaw), -sin(yaw), 0,
              sin(yaw),  cos(yaw), 0,
              0,         0,        1;

    // Create the transformation matrix
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R_yaw;  // Set rotation
    T(0, 3) = x;                  // Set translation x
    T(1, 3) = y;                  // Set translation y
    T(2, 3) = z;                  // Set translation z

    return T;
  }

  Eigen::Matrix4d NDTRegistration::matchScans(Eigen::Vector3d &pose)
  {
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations(35);
    ndt.setInputSource(scan_pts);
    ndt.setInputTarget(map_pts);
    pcl::PointCloud<pcl::PointXYZ> output(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f init_guess = transform3D(pose[0], pose[1], 0.0, pose[2]).cast<float>();
    ndt.align(*output, init_guess);
    Eigen::Matrix4d final_transformation = ndt.getFinalTransformation().cast<double>();
    return final_transformation;
  }
};  // namespace ndt_2d