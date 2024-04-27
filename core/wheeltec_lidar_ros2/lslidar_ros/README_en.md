# LSLIDAR_CX_V3.0.3_221118_ROS2

## 1.Introduction
​		LSLIDAR_CX_V3.0.3_221118_ROS2 is the lidar ros driver in linux environment, which is suitable for c16/c32 lidar(1212bytes ,version 2.6/2.8/3.0). The program has  tested under ubuntu18.04 ros dashing , ubuntu18.04 ros eloquent, ubuntu 20.04 ros foxy , ubuntu 20.04 ros galactic and ubuntu 22.04 ros humble .

## 2.Dependencies

1.ros

To run lidar driver in ROS environment, ROS related libraries need to be installed.

**Ubuntu 18.04**: ros-dashing-desktop-full

**Ubuntu 18.04**: ros-eloquent-desktop-full

**Ubuntu 20.04**: ros-foxy-desktop-full

**Ubuntu 20.04**: ros-galactic-desktop-full

**Ubuntu 22.04**: ros-humble-desktop-full

**Installation**: please refer to [http://wiki.ros.org](http://wiki.ros.org/)

2.ros dependencies

```bash
# install
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions
```

3.other dependencies

~~~bash
sudo apt-get install libpcap-dev
sudo apt-get install libboost${BOOST_VERSION}-dev   #Select the appropriate version
~~~

## 3.Compile & Run

### 3.1 Compile

~~~bash
mkdir -p ~/lidar_ws/src
~~~

Copy the whole lidar ROS driver directory into ROS workspace, i.e "~/lidar_ws/src".

~~~bash
cd ~/lidar_ws
colcon build
source install/setup.bash
~~~

### 3.2 Run

run with single c32 lidar(It is divided into 1212 bytes lidar and 1206 bytes lidar. Pay attention to modify lslidar_driver/params/lslidar_c32.yaml when start  lidar driver) :

~~~bash
ros2 launch lslidar_driver lslidar_c32_launch.py
~~~

run with double c32 lidar(It is divided into 1212 bytes lidar and 1206 bytes lidar. Pay attention to modify lslidar_driver/params/lslidar_c32_1.yaml and  lslidar_c32_2.yaml  when start  lidar  driver) :

~~~bash
ros2 launch lslidar_driver lslidar_c32_double_launch.py
~~~



run with single c16 lidar(It is divided into 1212 bytes lidar and 1206 bytes lidar. Pay attention to modify lslidar_driver/params/lslidar_c16.yaml when start  lidar driver) :

~~~bash
ros2 launch lslidar_driver lslidar_c16_launch.py
~~~

run with double c16 lidar(It is divided into 1212 bytes lidar and 1206 bytes lidar. Pay attention to modify lslidar_driver/params/lslidar_c16_1.yaml and  lslidar_c16_2.yaml  when start lidar  driver) :

~~~bash
ros2 launch lslidar_driver lslidar_c16_double_launch.py
~~~



## 4. Introduction to parameters

The content of the lslidar_c32.yaml file is as follows, and the meaning of each parameter is shown in the notes.

~~~bash
/c32/lslidar_driver_node:
  ros__parameters:
    packet_size: 1206                       #lidar data packet length (bytes)，write 1212 or 1206 
    device_ip: 192.168.1.200                #lidar ip
    msop_port: 2368                         # Main data Stream Output Protocol packet port
    difop_port: 2369                        # Device Information Output Protocol packet port
    frame_id: laser_link                    # lidar point cloud coordinate system name
    add_multicast: false                    # Whether to add multicast
    group_ip: 224.1.1.2                     #multicast ip
    use_gps_ts: true                        # Whether gps time synchronization
    lidar_type: c32                         #c16:c16 lidar；c32:c32 lidar
    c16_type: c16_2                         #c16_2:lidar with vertical angular resolution of 2 degrees，c16_1:lidar with vertical angular resolution of 1.33 degrees
    c32_type: c32_2                         #c32_2:lidar with vertical angular resolution of 1 degrees ，c32_1:lidar with vertical angular resolution of 2 degrees
    c32_fpga_type: 3                        #3:lidar with fpga version 2.7\2.8\3.0 ，2:lidar with fpga version 2.6
    min_range: 0.3                          #Unit: m. The minimum value of the lidar blind area, points smaller than this value are filtered
    max_range: 200.0                        #Unit: m. The maximum value of the lidar blind area, points smaller than this value are filtered
    distance_unit: 0.4                      #lidar range resolution
    angle_disable_min: 0                    #lidar clipping angle start value ，unit:0.01°
    angle_disable_max: 0                    #lidar clipping angle end value ，unit:0.01°
    scan_num: 16                            #laserscan line number
    topic_name: lslidar_point_cloud         #point cloud topic name, can be modified
    publish_scan: false                     #Whether to publish the scan
    pcl_type: false                         #pointcloud type，false: xyzirt,true:xyzi
    coordinate_opt: false                   #Default false. The zero degree angle of the lidar corresponds to the direction of the point cloud
    #pcap: /home/chris/Documents/leishen/1212bytes_c32/gps.pcap                        #Uncomment to read the data from the pcap file, and add the comment to read the data from the lidar
~~~

### Multicast mode:

- The host computer sets the lidar to enable multicast mode

- Modify the following parameters of the launch file

  ~~~xml
  add_multicast: true
  group_ip: 224.1.1.2    //The multicast ip address set by the host computer
  ~~~

- Run the following command to add the computer to the group (replace enp2s0 in the command with the network card name of the user's computer, you can use ifconfig to view the network card name)

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



### Offline pcap mode:

- Copy the recorded pcap file to the lslidar_ros/lslidar_driver/pcap folder

- Modify the following parameters of the launch file

  ~~~xml
  // uncomment
  pcap: xxx.pcap  // xxx.pcap is changed to the copied pcap file name
  ~~~



###  pcl point cloud type:

- Modify the following parameters of the launch file

  ~~~xml
  pcl_type: false      # pointcloud type，false: xyzirt,true:xyzi
  ~~~

- The default false is the custom point cloud type, which references the definition in the file of

  lslidar_driver/include/lslidar_driver.h

  Change it to true, which is the own type of pcl:

  ~~~c++
  pcl::PointCloud<pcl::PointXYZI>
  ~~~

## FAQ

Bug Report

version : LSLIDAR_CX_ 1212BYTES_V3.0.1_220719 ROS2

Modify:  original version

Date    : 2022-07-19

-------------------------------------------------------------------------------------------------------------------------

Original version : LSLIDAR_CX_V3.0.2_220809_ROS2

Modify:  1.Compatible with 1212 bytes and 1206 bytes C16 / C32 lidar(version 3.0 )

Date    : 2022-08-09

-------------------------

Original version : LSLIDAR_CX_V3.0.3_221118_ROS2

Modify:  1.Added support for ros2 humble/ros2 dashing / ros2 eloquent

Date    : 2022-11-18

