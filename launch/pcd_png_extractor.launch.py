import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
		pcd_folder_path = "./data/pcd/"
		png_folder_path = "./data/png/"

		if not os.path.exists(pcd_folder_path):
				os.makedirs(pcd_folder_path, exist_ok=True)
		if not os.path.exists(png_folder_path):
				os.makedirs(png_folder_path, exist_ok=True)

		decl_pcd_dir_path = DeclareLaunchArgument("pcd_dir_path", default_value=pcd_folder_path)
		decl_png_file_path = DeclareLaunchArgument("png_file_path", default_value=png_folder_path)

		pcd_png_extractor_node = Node(
			package='data_collection',
			executable='data_collection_pcd_png_extractor_node',
			name='pcd_png_extractor_node',
			parameters=[{
				"pointcloud_topic" : "/lidar/top/rslidar_sdk/rs/points",
				"image_topic" : "/image_raw",
				"collect_number" : 1,
				"pcd_dir_path" : LaunchConfiguration("pcd_dir_path"),
				"png_file_path" : LaunchConfiguration("png_file_path")
			}],
			output='screen')

		return LaunchDescription([decl_pcd_dir_path, decl_png_file_path, pcd_png_extractor_node])