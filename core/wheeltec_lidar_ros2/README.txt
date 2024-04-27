Launch LiDAR driver nodes
	roslaunch  lslidar_driver  lslidar_c32.launch 
	roslaunch  pointcloud_to_laserscan  pointcloud_scan.launch


Adding LiDAR drivers to launch files
 	<include file="$(find lslidar_driver)/launch/lslidar_c32.launch" />
  	<include file="$(find pointcloud_to_laserscan)/launch/pointcloud_scan.launch" />

gmapping integration
    in your workspace src folder, run：
        	git clone https://github.com/ros-perception/slam_gmapping
         	git clone https://github.com/ros-perception/openslam_gmapping
    In openslam_gmapping/include/gmapping/scanmatcher/scanmatcher.h, change the following lines：
        	//#define LASER_MAXBEAMS 2048
        	#define LASER_MAXBEAMS 4096
    Build the openslam_gmapping package using catkin_make, followed by slam_gmapping


cartographer integration：
	Copy 2d_online.launch to path /opt/ros/melodic/share/cartographer_ros/launch/
	Copy 2d_online.lua to path /opt/ros/melodic/share/cartographer_ros/configuration_files/
	
	Integration of IMU requires data from the z-axis acceleration direction


TF static transform configuration
	rosrun tf2_ros static_transform_publisher <offset x> <offset y> <offset z> <yaw x> <pitch y> <roll x> <Parent Frame> <Child Frame>
Example:
	rosrun tf2_ros static_transform_publisher 0.2 0 0.5 0 0 0 /base_footprint  /laser



Visualize scan on polar map：
	1. Change the number of Laser scan channels
	2. Change the min_height parameter in pointcloud_to_laserscan pointcloud_scan.launch