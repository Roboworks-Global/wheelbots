### LSLIDAR_CX_V3.0.3_221118_ROS2驱动说明

## 1.工程介绍

​		LSLIDAR_CX_V3.0.3_221118_ROS2为linux环境下雷达ros2驱动，适用于镭神c16/c32, 1212字节/1206字节,2.6/2.8/3.0版本的雷达 ，程序在ubuntu18.04 ros dashing , ubuntu18.04 ros eloquent ,ubuntu 20.04 ros foxy, ubuntu 20.04 ros galactic 以及ubuntu22.04 ros humble下测试通过。

## 2.依赖

1.ubuntu18.04 ros dashing/ubuntu18.04 ros eloquent/ubuntu 20.04 ros foxy/ubuntu 20.04 ros galactic/ubuntu22.04 ros humble

2.ros依赖

```bash
# 安装
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions
```

3.其他依赖

~~~bash
sudo apt-get install libpcap-dev
sudo apt-get install libboost${BOOST_VERSION}-dev   #选择适合的版本
~~~

## 3.编译与运行：

~~~shell
mkdir -p ~/lslidar_ws/src
cd ~/lslidar_ws/src
把驱动压缩包拷贝到src目录下，并解压
cd ~/lslidar_ws
colcon build
source install/setup.bash
#启动单个c32雷达,分为1212字节雷达和1206字节雷达，启动的时候注意修改lslidar_driver/params/lslidar_c32.yaml文件中packet_size参数
ros2 launch lslidar_driver lslidar_c32_launch.py
#启动两个c32雷达,分为1212字节雷达和1206字节雷达，启动的时候注意修改lslidar_driver/params/lslidar_c32_1.yaml和lslidar_c32_2.yaml文件中packet_size参数 
ros2 launch lslidar_driver lslidar_c32_double_launch.py

#启动单个c16雷达,分为1212字节雷达和1206字节雷达，启动的时候注意修改lslidar_driver/params/lslidar_c16.yaml文件中packet_size参数
ros2 launch lslidar_driver lslidar_c16_launch.py
#启动两个c16雷达，分为1212字节雷达和1206字节雷达，启动的时候注意修改lslidar_driver/params/lslidar_c16.yaml和lslidar_c16.yaml文件中packet_size参数
ros2 launch lslidar_driver lslidar_c16_double_launch.py
~~~



## 4.launch 文件参数说明：

~~~shell
/c32/lslidar_driver_node:
  ros__parameters:
    packet_size: 1206                       #雷达数据包长度(字节)，填1212/1206 
    device_ip: 192.168.1.200                #雷达ip
    msop_port: 2368                         #数据包目的端口
    difop_port: 2369                        #设备包目的端口
    frame_id: laser_link                    #坐标系id
    add_multicast: false                    #是否开启组播模式。
    group_ip: 224.1.1.2                     #组播ip地址
    use_gps_ts: true                        #雷达是否使用gps或ptp授时，使用改为true
    lidar_type: c32                         #c16表示机械式16线雷达；c32表示机械式32线雷达
    c16_type: c16_2                         #c16_2表示16线垂直角度分辨率为2度的雷达，c16_1表示16线垂直角度分辨率为1.33度的雷达
    c32_type: c32_2                         #c32_2表示32线垂直角度分辨率为1度的雷达，c32_1表示32线垂直角度分辨率为0.33度的雷达
    c32_fpga_type: 3                        #3表示32线fpga为2.7\2.8\3.0的版本的雷达，2表示32线fpga为2.6的版本的雷达
    min_range: 0.3                          #单位，米。雷达盲区最小值，小于此值的点被过滤
    max_range: 200.0                        #单位，米。雷达盲区最大值 ，大于此值的点被过滤
    distance_unit: 0.4                      #雷达距离分辨率
    angle_disable_min: 0                    #雷达裁剪角度开始值 ，单位0.01°
    angle_disable_max: 0                    #雷达裁剪角度结束值，单位0.01°
    scan_num: 16                            #laserscan线号
    topic_name: lslidar_point_cloud         #点云话题名称，可修改
    publish_scan: false                     #是否发布scan
    pcl_type: false                         #点云类型，默认false点云中的点为xyzirt字段。改为true，点云中的点为xyzi字段。
    coordinate_opt: false                   #默认false  雷达零度角对应点云方向
    #pcap: /home/chris/Documents/leishen/1212bytes_c32/gps.pcap                        #pcap包路径，加载pcap包时打开此注释
~~~

### 组播模式：

- 上位机设置雷达开启组播模式

- 修改launch文件以下参数

  ~~~shell
  add_multicast: true                    #是否开启组播模式。
  group_ip: 224.1.1.2                     #组播ip地址
  ~~~

- 运行以下指令将电脑加入组内（将指令中的enp2s0替换为用户电脑的网卡名,可用ifconfig查看网卡名)

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



### 离线pcap模式：

- 把录制好的pcap文件，拷贝到cx_4.0_ws/src/lslidar_ros/lslidar_driver/pcap文件夹下。（cx_4.0_ws是ros工作空间,根据实际工作空间修改）

- 修改launch文件以下参数

  ~~~shell
  #取消注释
  pcap: /home/chris/Documents/leishen/1212bytes_c32/gps.pcap                        #pcap包路径，加载pcap包时打开此注释
  ~~~



###  pcl点云类型：

- 修改launch文件以下参数

  ~~~shell
  pcl_type: false                         #点云类型，默认false点云中的点为xyzirt字段。改为true，点云中的点为xyzi字段。
  ~~~

  

- 默认false为自定义点云类型，定义参考lslidar_driver/include/lslidar_driver.h头文件

- 改为true,为pcl自带类型 :

  ~~~shell
  pcl::PointCloud<pcl::PointXYZI>
  ~~~

  

## FAQ

Bug Report

Original version : LSLIDAR_CX_1212BYTES_V3.0.1_220719_ROS2

Modify:  original version

Date    : 2022-07-19

-----------------------------------------------------------------------------------------

Original version : LSLIDAR_CX_V3.0.2_220809_ROS2

Modify:  1.兼容1212字节和1206字节 3.0版本c16/c32雷达

Date    : 2022-08-09

---------------------

Original version : LSLIDAR_CX_V3.0.2_221118_ROS2

Modify:  1.新增对ros2 humble, ros2 dashing, ros2 eloquent的支持

Date    : 2022-11-18
