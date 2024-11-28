#include <dt_ndt_mcl/ndt_registration.hpp>
#include <thread>  // Add this line for std::this_thread


namespace ndt_2d
{
  NDTRegistration::NDTRegistration(): map_pts(new pcl::PointCloud<pcl::PointXYZ>), 
                                      scan_pts(new pcl::PointCloud<pcl::PointXYZ>),
                                      m_initial_pose(Eigen::Vector3d::Zero())
                                        {}

  void NDTRegistration::setInitialPose(const Eigen::Vector3d &pose){m_initial_pose = pose;}
    
  void NDTRegistration::addMapPCD(const nav_msgs::msg::OccupancyGrid &msg)
  {

    for (unsigned int i = 0; i < msg.info.height; ++i) {
            for (unsigned int j = 0; j < msg.info.width; ++j) {
                if (msg.data[i * msg.info.width + j] > 50) {  // Threshold for occupied cells
                    pcl::PointXYZ point;
                    point.x = msg.info.origin.position.x + j * msg.info.resolution;
                    point.y = msg.info.origin.position.y + i * msg.info.resolution;
                    point.z = 0.0;
                    map_pts->points.push_back(point);
                }
            }
        }

     

    
  }

  void NDTRegistration::addScanPCD(std::vector<Eigen::Vector3d> &points)
  {
    for (auto &point : points)
    {
      scan_pts->push_back(pcl::PointXYZ(point(0), point(1), 0.0));
    }
    // // Initialize the PCL Visualizer
    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // viewer->setBackgroundColor(0, 0, 0);  // Set background to black
    // viewer->addPointCloud<pcl::PointXYZ>(scan_pts, "sample cloud");  // Add the point cloud
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");  // Set point size
    // viewer->addCoordinateSystem(1.0);  // Add a coordinate system
    // viewer->initCameraParameters();  // Initialize camera

    // // Main loop to keep the viewer open
    // while (!viewer->wasStopped()) {
    //     viewer->spin();  // Update the viewer
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

  }

  Eigen::Matrix4f NDTRegistration::transform3D(double x, double y, double z, double yaw){
    Eigen::Matrix3f R_yaw;
    R_yaw << cos(yaw), -sin(yaw), 0,
              sin(yaw),  cos(yaw), 0,
              0,         0,        1;

    // Create the transformation matrix
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = R_yaw;  // Set rotation
    T(0, 3) = x;                  // Set translation x
    T(1, 3) = y;                  // Set translation y
    T(2, 3) = z;                  // Set translation z

    return T;
  }

  
  Eigen::Vector3d NDTRegistration::matchScans()
  {
    if (scan_pts->empty()) {
    std::cout << "Scan point cloud is empty!" << std::endl;
    }

    if (map_pts->empty()) {
    std::cout << "Map point cloud is empty!" << std::endl;
    }
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setInputTarget(map_pts);
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    ndt.setResolution(1);
    ndt.setMaximumIterations(150);
    ndt.setInputSource(scan_pts);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f init_guess = transform3D(m_initial_pose[0], m_initial_pose[1], 0.0, m_initial_pose[2]);
    ndt.align(*output, init_guess);
    Eigen::Matrix4f final_transformation = ndt.getFinalTransformation();
    // geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    // // pose_msg.header.stamp = msg->header.stamp;
    // pose_msg.header.frame_id = "map";
    // convertEigenToPose(final_transformation, pose_msg.pose.pose);
    Eigen::Vector3d final_pose;
    convertEigenToXYTheta(final_transformation, final_pose);
    return final_pose;
  }
  void NDTRegistration::convertEigenToPose(const Eigen::Matrix4f &transformation, geometry_msgs::msg::Pose &pose){
        Eigen::Affine3d transform(transformation.cast<double>());
        pose.position.x = transform.translation().x();
        pose.position.y = transform.translation().y();
        pose.position.z = transform.translation().z();
        Eigen::Quaterniond q(transform.rotation());
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
  }
  void NDTRegistration::convertEigenToXYTheta(const Eigen::Matrix4f &transformation, Eigen::Vector3d &pose){
    Eigen::Affine3d transform(transformation.cast<double>());
    pose(0) = transform.translation().x();
    pose(1) = transform.translation().y();
    pose(2) = atan2(transform(1,0), transform(0,0));
  }
};  // namespace ndt_2d