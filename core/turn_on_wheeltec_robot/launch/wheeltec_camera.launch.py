import os
from pathlib import Path
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
	astra_dir = get_package_share_directory('astra_camera')
	astra_launch_dir = os.path.join(astra_dir,'launch')

	Astra_S = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(os.path.join(astra_launch_dir,'astra_mini.launch.py')),)

	Astra_Pro = IncludeLaunchDescription(
	PythonLaunchDescriptionSource(os.path.join(astra_launch_dir,'astra_pro.launch.py')),)

	Dabai = IncludeLaunchDescription(
	PythonLaunchDescriptionSource(os.path.join(astra_launch_dir,'dabai.launch.py')),)

	Gemini = IncludeLaunchDescription(
	PythonLaunchDescriptionSource(os.path.join(astra_launch_dir,'gemini.launch.py')),)

	# Create the launch description and populate
	ld = LaunchDescription()
	
	#Select your camera here, options include:
	#Astra_S、Astra_Pro、Dabai、Gemini
	ld.add_action(Astra_S)

	return ld