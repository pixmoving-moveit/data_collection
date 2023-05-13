#include <ctime>
#include <chrono>
#include <iomanip>
#include <data_collection/lidar_imu.hpp>

namespace data_collection
{
namespace lidar_imu
{
LidarImu::LidarImu() : Node("lidar_imu")
{
  // param
  param_.pcd_dir_path = declare_parameter("pcd_dir_path", "/home/ahua20/Downloads/calibration_data/lidar_top");
  param_.pose_file_path = declare_parameter("pose_file_path", "/home/ahua20/Downloads/calibration_data/pose.txt");
  param_.pointcloud_topic =
    declare_parameter("pointcloud_topic", "/sensing/lidar/top/rslidar_sdk/rs/points");
  param_.pose_topic = declare_parameter("pose_topic", "/sensing/gnss/pose");

  output_pose_file_.open(param_.pose_file_path);

  // subscribers
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    param_.pointcloud_topic, 
    rclcpp::SensorDataQoS().keep_last(10),
    std::bind(&LidarImu::callbackPointCloud, this, std::placeholders::_1));
  gnss_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    param_.pose_topic, 1, std::bind(&LidarImu::callbackGnssPose, this, std::placeholders::_1));
}

LidarImu::~LidarImu()
{ 
  output_pose_file_.close();
}

std::string stampToString(const builtin_interfaces::msg::Time & stamp)
{
  // Convert the stamp message to a std::chrono::time_point object
  auto stamp_time_point = std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec);
  
  // Convert the time point to a std::tm object
  std::chrono::system_clock::time_point timepoint{std::chrono::duration_cast<std::chrono::system_clock::time_point::duration>(stamp_time_point)};
  auto stamp_time = std::chrono::system_clock::to_time_t(timepoint);
  std::tm stamp_tm = *std::localtime(&stamp_time);

  // Format the time as a string in the desired format
  std::stringstream stamp_string_stream;
  stamp_string_stream << std::put_time(&stamp_tm, "%Y-%m-%d-%H-%M-%S");
  stamp_string_stream << "-" << std::setw(3) << std::setfill('0') << (stamp.nanosec / 1000000);
  return stamp_string_stream.str();
}

void LidarImu::callbackPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  pcl::PointCloud<RsPointXYZIRT> pcl_pointcloud;
  pcl::fromROSMsg(*msg, pcl_pointcloud);
  // pcl::io::savePCDFileASCII(param_.pcd_dir_path + "/" + stampToString(msg->header.stamp) + ".pcd", pcl_pointcloud);
  if(is_initialized)
  {
    pcl::io::savePCDFileBinary(param_.pcd_dir_path + "/" + stampToString(msg->header.stamp) + ".pcd", pcl_pointcloud);
    output_pose_file_ << stampToString(msg->header.stamp) << " " << shifted_transform_matrix_(0, 0) << " "
              << shifted_transform_matrix_(0, 1) << " " << shifted_transform_matrix_(0, 2) << " "
              << shifted_transform_matrix_(0, 3) << " " << shifted_transform_matrix_(1, 0) << " "
              << shifted_transform_matrix_(1, 1) << " " << shifted_transform_matrix_(1, 2) << " "
              << shifted_transform_matrix_(1, 3) << " " << shifted_transform_matrix_(2, 0) << " "
              << shifted_transform_matrix_(2, 1) << " " << shifted_transform_matrix_(2, 2) << " "
              << shifted_transform_matrix_(2, 3) << std::endl;
  }
}

void LidarImu::callbackGnssPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
{
  if(!is_initialized)
  {
    initial_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(*msg);
    initial_transform_matrix_.setIdentity();
    Eigen::Matrix3d initial_r = Eigen::Quaterniond(
      initial_pose_ptr_->pose.orientation.w, initial_pose_ptr_->pose.orientation.x,
      initial_pose_ptr_->pose.orientation.y, initial_pose_ptr_->pose.orientation.z).toRotationMatrix();
    Eigen::Vector3d initial_t{initial_pose_ptr_->pose.position.x, initial_pose_ptr_->pose.position.y, initial_pose_ptr_->pose.position.z};
    initial_transform_matrix_.block<3, 3>(0, 0) = initial_r;
    initial_transform_matrix_.block<3, 1>(0, 3) = initial_t;
    is_initialized = true;
    return;
  }
  Eigen::Quaterniond current_q(
    msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
    msg->pose.orientation.z);
  Eigen::Vector3d current_t(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Matrix4d current_transform_matrix;
  current_transform_matrix.setIdentity();
  current_transform_matrix.block<3, 3>(0, 0) = current_q.toRotationMatrix();
  current_transform_matrix.block<3, 1>(0, 3) = current_t;
  shifted_transform_matrix_ =  current_transform_matrix;
}
} // namespace data_collection
} // namespace lidar_imu