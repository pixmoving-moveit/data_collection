#include <data_collection/pcd_png_extractor.hpp>

int main(int argc, char ** argv)
{ 
  rclcpp::init(argc, argv);
  auto node = std::make_shared<data_collection::pcd_png_extractor::PcdPndExtractor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}