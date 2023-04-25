#ifndef __DATA_COLLECTION__LIDAR_IMI__HPP__
#define __DATA_COLLECTION__LIDAR_IMI__HPP__

#include <memory>
#include <string>
#include <vector>

#include <fstream>

// ros2
#include <rclcpp/rclcpp.hpp>

// msgs
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

// tf2
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct RsPointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
  RsPointXYZIRT,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
  (double, timestamp, timestamp)
);

namespace data_collection
{
namespace lidar_imu
{

struct Param
{
  std::string pcd_dir_path;
  std::string pose_file_path;
  std::string pointcloud_topic;
  std::string pose_topic;
};

class LidarImu : public rclcpp::Node
{
private:
  // param
  Param param_;

  // imu odometry
  builtin_interfaces::msg::Time prev_t_;
  double time_diff_;
  bool is_initialized = false;
  geometry_msgs::msg::PoseStamped::SharedPtr initial_pose_ptr_;
  Eigen::Matrix4d initial_transform_matrix_;
  Eigen::Matrix4d shifted_transform_matrix_;

  // file output stream
  std::ofstream output_pose_file_;

  // subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr pointcloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::ConstSharedPtr gnss_pose_sub_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

public:
  LidarImu();
  ~LidarImu();

  // callback functions
  void callbackPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void callbackGnssPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg);
};

} // namespace lidar_imu
} // namespace data_collection

#endif // __DATA_COLLECTION__LIDAR_IMU__HPP__