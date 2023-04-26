#ifndef __DATA_COLLECTION__PCD_PNG_EXTRACTOR__HPP__
#define __DATA_COLLECTION__PCD_PNG_EXTRACTOR__HPP__
// system 
#include <chrono>
using namespace std::chrono_literals;

// ros2
#include <rclcpp/rclcpp.hpp>

// msgs
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

// opencv
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>

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
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp));

namespace data_collection
{
  namespace pcd_png_extractor
  {
    struct Param
    {
      std::string pcd_dir_path;
      std::string png_file_path;
      std::string pointcloud_topic;
      std::string image_topic;
      uint8_t collect_number;
    };

    class PcdPndExtractor : public rclcpp::Node
    {
    private:
      // ros param
      Param param_;
      uint8_t pcd_number = 0;
      uint8_t png_number = 0;

      // subscribers
      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr pointclude_sub_;
      rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr image_sub_;
      rclcpp::TimerBase::SharedPtr timer_;

    public:
      PcdPndExtractor(/* args */);
      ~PcdPndExtractor();

      // callback functions
      void callbackPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
      void callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
      void timer_callback();
    };

  } // namespace pcd_png_extractor
} // namespace data_collection

#endif // __DATA_COLLECTION__PCD_PNG_EXTRACTOR__HPP__