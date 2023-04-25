cmake_minimum_required(VERSION 3.5)
project(data_collection)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Declare a C++ executable

ament_auto_add_library(data_collection_lidar_imu SHARED
  src/lidar_imu.cpp
)

ament_auto_add_executable(
  data_collection_lidar_imu_node
  src/lidar_imu_node.cpp
)

target_link_libraries(
  data_collection_lidar_imu_node
  data_collection_lidar_imu
)

# install
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  # config
)
