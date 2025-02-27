#ifndef PF_ROS_HPP_
#define PF_ROS_HPP_

#include <angles/angles.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <dt_ndt_mcl/conversions.hpp>
#include <dt_ndt_mcl/motion_model.hpp>
#include <dt_ndt_mcl/particle_filter.hpp>
#include <dt_ndt_mcl/scan_matcher_ndt.hpp>
#include <dt_ndt_mcl/ndt_registration.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class ParticleFilter2D : public rclcpp::Node
{
public:
  ParticleFilter2D();

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  void initPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  double computeTrace();

private:

  std::shared_ptr<ndt_2d::ScanMatcherNDT> m_scan_matcher_ptr;
  ndt_2d::MotionModelPtr m_motion_model;
  std::shared_ptr<ndt_2d::ParticleFilter> m_pf;
  std::shared_ptr<ndt_2d::NDTRegistration> m_scan_registration_ptr;

  ndt_2d::Pose2d m_prev_odom_pose;
  ndt_2d::Pose2d m_prev_robot_pose;

  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_init_pose_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_scan_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_best_pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_pose_particle_pub;

  bool m_received_map;
  bool m_received_init_pose;
  int m_scan_id;
  double m_kld_err;
  double m_kld_z;
  double m_min_travel_distance;
  double m_min_travel_rotation;
  Eigen::Vector3d mean;
  Eigen::Vector3d lag_delta;
  double pose_x;
  double pose_y;
  double pose_yaw;
  Eigen::Vector3d smoothed_pose;
  double total_distance;
};

#endif // PF_ROS_HPP_