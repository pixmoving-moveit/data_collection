#include "data_collection/pcd_png_extractor.hpp"

namespace data_collection
{
  namespace pcd_png_extractor
  {

    PcdPndExtractor::PcdPndExtractor(/* args */) : Node("pcd_png_extractor")
    {
      // ros2 param
      param_.pointcloud_topic = declare_parameter("pointcloud_topic", "input/point_raw");
      param_.image_topic = declare_parameter("image_topic", "input/image_raw");
      param_.collect_number = declare_parameter("collect_number", 1);
      param_.pcd_dir_path = declare_parameter("pcd_dir_path", "output/pcd/");
      param_.png_file_path = declare_parameter("png_file_path", "output/png/");

      std::cout << (int)param_.collect_number << "\n";
      maximum_queue_size_ = static_cast<int>(declare_parameter("max_queue_size", 5));

      // ros2 subscribers
      pointclude_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          param_.pointcloud_topic, 
          rclcpp::SensorDataQoS().keep_last(maximum_queue_size_),
          std::bind(&PcdPndExtractor::callbackPointCloud, this, std::placeholders::_1));

      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
          param_.image_topic, 1,
          std::bind(&PcdPndExtractor::callbackImage, this, std::placeholders::_1));

      timer_ = create_wall_timer(500ms, std::bind(&PcdPndExtractor::timer_callback, this));
    }

    PcdPndExtractor::~PcdPndExtractor()
    {
    }

    std::string stampToString(const builtin_interfaces::msg::Time &stamp)
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

    void PcdPndExtractor::callbackPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
    {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), steady_clock, 5000, "One frame PointCloud:" << msg->header.stamp.sec);

      pcl::PointCloud<RsPointXYZIRT> pcl_pointcloud;
      pcl::fromROSMsg(*msg, pcl_pointcloud);
      if(pcd_number<param_.collect_number)
      {
        // pcl::io::savePCDFileASCII(param_.pcd_dir_path + "/" + stampToString(msg->header.stamp) + ".pcd", pcl_pointcloud);
        pcl::io::savePCDFileBinary(param_.pcd_dir_path + "/" + stampToString(msg->header.stamp) + ".pcd", pcl_pointcloud);
        pcd_number = pcd_number+1;
      }
    }

    void PcdPndExtractor::callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), steady_clock, 5000, "One frame Image:" << msg->header.stamp.sec);
      if(png_number<param_.collect_number)
      {
        // Convert ROS message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          return;
        }

        // Save image to file
        cv::imwrite(param_.png_file_path+ "/" + stampToString(msg->header.stamp) + ".png", cv_ptr->image);
        png_number = png_number+1;
      }
    }
    
    void PcdPndExtractor::timer_callback()
    {
      if(pcd_number >= param_.collect_number && png_number >= param_.collect_number)
      {
        rclcpp::shutdown();
      }
    }
  } // namespace pcd_png_extractor
} // namespace data_collection