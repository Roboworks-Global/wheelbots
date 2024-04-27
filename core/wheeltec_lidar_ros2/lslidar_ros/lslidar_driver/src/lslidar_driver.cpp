/******************************************************************************
 * This file is part of lslidar_cx driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "lslidar_driver/lslidar_driver.h"

namespace lslidar_driver {
    lslidarDriver::lslidarDriver() : lslidarDriver(rclcpp::NodeOptions()) {
        config_vert_num = 0;
        return;
    }

    lslidarDriver::lslidarDriver(const rclcpp::NodeOptions &options) : Node("lslidar_node", options){
        config_vert_num = 0;
        config_vert = true;
        socket_id = -1;
        last_azimuth = 0;
        sweep_end_time = 0.0;
        is_first_sweep = true;
        return_mode = 1;
        current_packet_time=0.0;
        last_packet_time=0.0;
        packet_size= 1212;
		is_update_gps_time = false;
		sweep_data.reset(new lslidar_msgs::msg::LslidarScan());

        return;
    }

    bool lslidarDriver::checkPacketValidity(const lslidar_driver::RawPacket *packet) {
        for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
            if (packet->blocks[blk_idx].header != UPPER_BANK) {
                return false;
            }
        }
        return true;
    }

    bool lslidarDriver::isPointInRange(const double &distance) {
        return (distance >= min_range && distance < max_range);
    }

    bool lslidarDriver::loadParameters() {
        this->declare_parameter<std::string>("pcap", "");
        this->declare_parameter<std::string>("frame_id", "laser_link");
        this->declare_parameter<std::string>("lidar_type", "c16");
        this->declare_parameter<std::string>("c16_type", "c16_2");
        this->declare_parameter<std::string>("c32_type", "c32_2");
        this->declare_parameter<int>("c32_fpga_type", 3);
        this->declare_parameter<bool>("add_multicast", false);
        this->declare_parameter<bool>("config_vert", true);
        this->declare_parameter<std::string>("group_ip", "234.2.3.2");
        this->declare_parameter<std::string>("device_ip", "192.168.1.200");
        this->declare_parameter<int>("msop_port",2368);
        this->declare_parameter<int>("difop_port",2369);
        this->declare_parameter<int>("point_num", 2000);
        this->declare_parameter<int>("scan_num", 8);
        this->declare_parameter<double>("min_range", 0.3);
        this->declare_parameter<double>("max_range", 150.0);
        this->declare_parameter<double>("distance_unit", 0.25);
        this->declare_parameter<int>("angle_disable_min", 0);
        this->declare_parameter<int>("angle_disable_max", 0);
        this->declare_parameter<int>("packet_size", 1212);
        this->declare_parameter<bool>("use_gps_ts", false);
        this->declare_parameter<bool>("pcl_type", false);
        this->declare_parameter<bool>("publish_scan", false);
        this->declare_parameter<bool>("coordinate_opt", false);
        this->declare_parameter<std::string>("topic_name", "lslidar_point_cloud");

        msop_udp_port = 0;
        difop_udp_port = 0;
        this->get_parameter("packet_size", packet_size);
        this->get_parameter("pcap", dump_file);
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("lidar_type", lidar_type);
        this->get_parameter("c16_type", c16_type);
        this->get_parameter("c32_type", c32_type);
        this->get_parameter("c32_fpga_type", c32_fpga_type);
        this->get_parameter("add_multicast", add_multicast);
        this->get_parameter("config_vert", config_vert);
        this->get_parameter("group_ip", group_ip_string);
        this->get_parameter("device_ip", lidar_ip_string);
        this->get_parameter("msop_port", msop_udp_port);
        this->get_parameter("difop_port", difop_udp_port);
        this->get_parameter("point_num", point_num);
        this->get_parameter("scan_num", scan_num);
        this->get_parameter("min_range", min_range);
        this->get_parameter("max_range", max_range);
        this->get_parameter("distance_unit", distance_unit);
        this->get_parameter("angle_disable_min", angle_disable_min);
        this->get_parameter("angle_disable_max", angle_disable_max);
        this->get_parameter("use_gps_ts", use_gps_ts);
        this->get_parameter("pcl_type", pcl_type);
        this->get_parameter("publish_scan", publish_scan);
        this->get_parameter("coordinate_opt", coordinate_opt);
        this->get_parameter("topic_name", pointcloud_topic);

        RCLCPP_INFO(this->get_logger(), "dump_file: %s", dump_file.c_str());
        RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "lidar_type: %s", lidar_type.c_str());
        RCLCPP_INFO(this->get_logger(), "c16_type: %s", c16_type.c_str());
        RCLCPP_INFO(this->get_logger(), "c32_type: %s", c32_type.c_str());
        RCLCPP_INFO(this->get_logger(), "device_ip: %s", lidar_ip_string.c_str());
        RCLCPP_INFO(this->get_logger(), "c32_fpag_type: %d", c32_fpga_type);
        RCLCPP_INFO(this->get_logger(), "config_vert: %d", config_vert);
        RCLCPP_INFO(this->get_logger(), "msop_port: %d", msop_udp_port);
        RCLCPP_INFO(this->get_logger(), "use_gps_ts: %d", use_gps_ts);
        RCLCPP_INFO(this->get_logger(), "pcl_type: %d", pcl_type);
        RCLCPP_INFO(this->get_logger(), "distance_unit: %f", distance_unit);
        RCLCPP_INFO(this->get_logger(), "difop_udp_port: %d", difop_udp_port);
        RCLCPP_INFO(this->get_logger(), "coordinate_opt: %d", coordinate_opt);
        RCLCPP_INFO(this->get_logger(), "topic_name: %s", pointcloud_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "packet_size: %d", packet_size);

        if(lidar_type=="c16") {
            if(scan_num>15) { scan_num=15;}
            else if(scan_num<0) { scan_num=0;}
        }else {
            if(scan_num>31) { scan_num=31;}
            else if(scan_num<0) { scan_num=0;}
        }

        if(packet_size!=1206 && packet_size!=1212) {
            packet_size=1212;
            RCLCPP_WARN(this->get_logger(), "packet_size input error, reassign to: %d", packet_size);
        }


        inet_aton(lidar_ip_string.c_str(), &lidar_ip);
        if (add_multicast)
            RCLCPP_INFO(this->get_logger(), "opening UDP socket: group_address %s", group_ip_string.c_str());
        return true;
    }

    void lslidarDriver::initTimeStamp() {
        for (int i = 0; i < 10; i++) {
            this->packetTimeStamp[i] = 0;
        }
        this->packet_time_s = 0;
        this->packet_time_ns = 0;
        this->timeStamp = rclcpp::Time(0.0);
		this->timeStamp_bak = rclcpp::Time(0.0);
    }

    bool lslidarDriver::createRosIO() {
        pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10);
        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
//        lslidar_control =
//                this->create_service<lslidar_msgs::srv::Lslidarcontrol>(
//                        "lslidarcontrol",
//                        std::bind(&lslidarC16Driver::lslidarControl, this));

        double packet_rate = 840;
        if (lidar_type == "c32") {
            packet_rate = packet_rate * 2;
        }
        if (dump_file != "") {
            msop_input_.reset(new lslidar_driver::InputPCAP(this, msop_udp_port, packet_rate, dump_file));
            difop_input_.reset(new lslidar_driver::InputPCAP(this, difop_udp_port, 1, dump_file));
        } else {
            msop_input_.reset(new lslidar_driver::InputSocket(this, msop_udp_port));
            difop_input_.reset(new lslidar_driver::InputSocket(this, difop_udp_port));
        }
        difop_thread_ = std::shared_ptr<std::thread>(
                new std::thread(std::bind(&lslidarDriver::difopPoll, this)));
        return true;
    }

    void lslidarDriver::publishPointcloud() {
        if (sweep_data->points.size() < 32 || !is_update_gps_time) {
            return;
        }
/*        if (pointcloud_pub.getNumSubscribers() == 0) {
            return;
        }*/
        if (pcl_type) {

            pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud <pcl::PointXYZI>);
            // pcl_conversions::toPCL(sweep_data->header).stamp;
            point_cloud->header.frame_id = frame_id;
            point_cloud->height = 1;
            point_cloud->header.stamp = static_cast<uint64_t>(sweep_end_time * 1e6);
            size_t j;
            pcl::PointXYZI point;
            if (sweep_data->points.size() > 0) {
                for (j = 0; j < sweep_data->points.size(); ++j) {
                    if ((sweep_data->points[j].azimuth > angle_disable_min) &&
                        (sweep_data->points[j].azimuth < angle_disable_max)) {
                        continue;
                    }
                    point.x = sweep_data->points[j].x;
                    point.y = sweep_data->points[j].y;
                    point.z = sweep_data->points[j].z;
                    point.intensity = sweep_data->points[j].intensity;
                    point_cloud->points.push_back(point);
                    ++point_cloud->width;
                }
            }
            sensor_msgs::msg::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud, pc_msg);
            pointcloud_pub->publish(pc_msg);
        } else {
            VPointcloud::Ptr point_cloud(new VPointcloud());
            //pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            // pcl_conversions::toPCL(sweep_data->header).stamp;
            point_cloud->header.frame_id = frame_id;
            point_cloud->height = 1;
            point_cloud->header.stamp = static_cast<uint64_t>(sweep_end_time * 1e6);
            size_t j;
            VPoint point;
            if (sweep_data->points.size() > 0) {
                for (j = 0; j < sweep_data->points.size(); ++j) {
                    if ((sweep_data->points[j].azimuth > angle_disable_min) &&
                        (sweep_data->points[j].azimuth < angle_disable_max)) {
                        continue;
                    }
                    point.x = sweep_data->points[j].x;
                    point.y = sweep_data->points[j].y;
                    point.z = sweep_data->points[j].z;
                    point.intensity = sweep_data->points[j].intensity;
                    point.ring = sweep_data->points[j].ring;
                    point.time = sweep_data->points[j].time;

                    point_cloud->points.push_back(point);
                    ++point_cloud->width;
                    current_point_time = point.time;
                    //if(current_point_time - last_point_time<0.0)
                    //    RCLCPP_WARN(this->get_logger(),"timestamp is rolled back! current point time: %.12f  last point time: %.12f",current_point_time,last_point_time);
                    //last_point_time = current_point_time;
                }
            }
            sensor_msgs::msg::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud, pc_msg);
            pointcloud_pub->publish(pc_msg);
        }
        return;
    }

    void lslidarDriver::publishScan() {
        if (sweep_data->points.size() < 32) {
            return;
        }
        sensor_msgs::msg::LaserScan::UniquePtr scan_msg(new sensor_msgs::msg::LaserScan());
        scan_msg->header.frame_id = frame_id;
        int layer_num_local = scan_num;
        RCLCPP_INFO_ONCE(this->get_logger(), "default channel is %d", layer_num_local);

        scan_msg->header.stamp = rclcpp::Time(sweep_end_time);
        scan_msg->angle_min = 0.0;
        scan_msg->angle_max = 2.0 * M_PI;
        scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / point_num;
        scan_msg->range_min = min_range;
        scan_msg->range_max = max_range;

        uint point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
        scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        if (coordinate_opt) {
            for (size_t j = 0; j < sweep_data->points.size(); ++j) {
                if (layer_num_local == sweep_data->points[j].ring) {
                    float horizontal_angle = sweep_data->points[j].azimuth * 0.01 * DEG_TO_RAD;
                    uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                    point_index = (point_index <= point_size) ? point_index : (point_index % point_size);
                    scan_msg->ranges[point_size - point_index - 1] = sweep_data->points[j].distance;
                    scan_msg->intensities[point_size - point_index - 1] = sweep_data->points[j].intensity;
                }
            }
        } else {
            for (size_t j = 0; j < sweep_data->points.size(); ++j) {
                if (layer_num_local == sweep_data->points[j].ring) {
/*                    float h_angle =
                            (sweep_data->points[j].azimuth + 9000.0) < 36000.0 ? sweep_data->points[j].azimuth + 9000.0
                                                                               : sweep_data->points[j].azimuth - 27000.0;
                    if(h_angle < 18000.0){
                        h_angle = 18000.0 - h_angle;
                    }else{
                        h_angle = 36000.0 - (h_angle - 18000.0);
                    }*/
                    float h_angle = (45000.0 - sweep_data->points[j].azimuth) < 36000.0 ?  45000.0 - sweep_data->points[j].azimuth:
                                    9000.0 - sweep_data->points[j].azimuth;
                    // ROS_INFO("a=%f,b=%f",sweep_data->points[j].azimuth,h_angle);
                    float horizontal_angle = h_angle * 0.01 * DEG_TO_RAD;
                    uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                    point_index = (point_index <= point_size) ? point_index : (point_index % point_size);
                    scan_msg->ranges[point_index] = sweep_data->points[j].distance;
                    scan_msg->intensities[point_index] = sweep_data->points[j].intensity;
                }
            }
        }
        scan_pub->publish(std::move(scan_msg));
    }

    bool lslidarDriver::lslidarControl(lslidar_msgs::srv::Lslidarcontrol::Request &req,
                                       lslidar_msgs::srv::Lslidarcontrol::Response &res) {
        RCLCPP_INFO(this->get_logger(), "--------------------------");
        // sleep(1);
        lslidar_msgs::msg::LslidarPacket::UniquePtr packet0(new lslidar_msgs::msg::LslidarPacket());
        packet0->data[0] = 0x00;
        packet0->data[1] = 0x00;
        int rc_msop = -1;

        if (req.lasercontrol == 1) {

            if ((rc_msop = msop_input_->getPacket(packet0)) == 0) {
                res.status = 1;
                RCLCPP_WARN(this->get_logger(), "receive cmd: %d,already power on status", req.lasercontrol);
                return true;
            }
            RCLCPP_WARN(this->get_logger(), "receive cmd: %d,power on", req.lasercontrol);
            SendPacketTolidar(true);
            double time1 = get_clock()->now().seconds();

            do {
                rc_msop = msop_input_->getPacket(packet0);
                double time2 = get_clock()->now().seconds();
                if ((time2 - time1) > 20) {
                    res.status = 0;
                    RCLCPP_WARN(this->get_logger(), "lidar connect error");
                    return true;
                }
            } while ((rc_msop != 0) && (packet0->data[0] != 0xff) && (packet0->data[1] != 0xee));
            sleep(5);
            res.status = 1;
        } else if (req.lasercontrol == 0) {
            RCLCPP_WARN(this->get_logger(), "receive cmd: %d,power off", req.lasercontrol);
            SendPacketTolidar(false);
            res.status = 1;
        } else {
            RCLCPP_WARN(this->get_logger(), "cmd error");
            res.status = 0;
        }

        return true;
    }

    bool lslidarDriver::SendPacketTolidar(bool power_switch) {
        int socketid;
        unsigned char config_data[1206];
        //int data_port = difop_data[24] * 256 + difop_data[25];
        mempcpy(config_data, difop_data, 1206);
        config_data[0] = 0xAA;
        config_data[1] = 0x00;
        config_data[2] = 0xFF;
        config_data[3] = 0x11;
        config_data[4] = 0x22;
        config_data[5] = 0x22;
        config_data[6] = 0xAA;
        config_data[7] = 0xAA;
        config_data[8] = 0x02;
        config_data[9] = 0x58;
        if (power_switch) {
            config_data[45] = 0x00;
        } else {
            config_data[45] = 0x01;
        }

        sockaddr_in addrSrv;
        socketid = socket(2, 2, 0);
        addrSrv.sin_addr.s_addr = inet_addr(lidar_ip_string.c_str());
        addrSrv.sin_family = AF_INET;
        addrSrv.sin_port = htons(2368);
        sendto(socketid, (const char *) config_data, 1206, 0, (struct sockaddr *) &addrSrv, sizeof(addrSrv));
        return 0;

    }


    lslidarC16Driver::lslidarC16Driver() : lslidarDriver() {
//        this->initialize();
        return;
    }

    lslidarC16Driver::~lslidarC16Driver() {
        if (difop_thread_ != nullptr) {
//            difop_thread_->interrupt();
            difop_thread_->join();
        }
    }

    bool lslidarC16Driver::initialize() {
        RCLCPP_INFO(this->get_logger(), "lslidarC16Driver::initialize()");
        this->initTimeStamp();
        if (!loadParameters()) {
            RCLCPP_ERROR(this->get_logger(), "cannot load all required ROS parameters.");
            return false;
        }
        if (c16_type == "c16_2") {
            for (int j = 0; j < 16; ++j) {
                c16_vertical_angle[j] = c16_2_vertical_angle[j];
                c16_config_vertical_angle[j] = c16_2_vertical_angle[j];
                c16_sin_scan_altitude[j] = sin(c16_vertical_angle[j] * DEG_TO_RAD);
                c16_cos_scan_altitude[j] = cos(c16_vertical_angle[j] * DEG_TO_RAD);
            }
        } else {
            for (int j = 0; j < 16; ++j) {
                c16_vertical_angle[j] = c16_1_vertical_angle[j];
                c16_config_vertical_angle[j] = c16_1_vertical_angle[j];
                c16_sin_scan_altitude[j] = sin(c16_vertical_angle[j] * DEG_TO_RAD);
                c16_cos_scan_altitude[j] = cos(c16_vertical_angle[j] * DEG_TO_RAD);
            }
        }


        if (!createRosIO()) {
            RCLCPP_ERROR(this->get_logger(), "cannot create all ROS IO.");
            return false;
        }

        // create the sin and cos table for different azimuth values
        for (int j = 0; j < 36000; ++j) {
            double angle = static_cast<double>(j) / 100.0 * DEG_TO_RAD;
            sin_azimuth_table[j] = sin(angle);
            cos_azimuth_table[j] = cos(angle);
        }
        angle_base = M_PI * 2 / point_num;
        return true;
    }

/*    bool lslidarC16Driver::checkPacketValidity(const RawPacket *packet) {
        for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
            if (packet->blocks[blk_idx].header != UPPER_BANK) {
                return false;
            }
        }
        return true;
    }*/


    void lslidarC16Driver::difopPoll() {
        // reading and publishing scans as fast as possible.
        lslidar_msgs::msg::LslidarPacket::UniquePtr difop_packet_ptr(new lslidar_msgs::msg::LslidarPacket);
        while (rclcpp::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet_ptr);
            if (rc == 0) {
                if (difop_packet_ptr->data[0] != 0xa5 || difop_packet_ptr->data[1] != 0xff ||
                    difop_packet_ptr->data[2] != 0x00 || difop_packet_ptr->data[3] != 0x5a) {
                    return;
                }
                count++;
                if (count == 2) { is_update_gps_time = true; }
                for (int i = 0; i < packet_size; i++) {
                    difop_data[i] = difop_packet_ptr->data[i];
                }

                int version_data = difop_packet_ptr->data[1202];
                if (config_vert) {
                    if (2 == version_data) {
                        for (int j = 0; j < 16; ++j) {
                            uint8_t data1 = difop_packet_ptr->data[234 + 2 * j];
                            uint8_t data2 = difop_packet_ptr->data[234 + 2 * j + 1];
                            int vert_angle = data1 * 256 + data2;
                            vert_angle = vert_angle > 32767 ? (vert_angle - 65536) : vert_angle;
                            c16_config_vertical_angle[j] = (double) vert_angle * 0.01f;
                            if (fabs(c16_vertical_angle[j] - c16_config_vertical_angle[j]) > 1.5) {
                                //c16_config_vertical_angle[j] = c16_vertical_angle[j];
                                config_vert_num++;
                            }

                        }
                        if (config_vert_num == 0) {
                            for (int k = 0; k < 16; ++k) {
                                c16_sin_scan_altitude[k] = sin(c16_config_vertical_angle[k] * DEG_TO_RAD);
                                c16_cos_scan_altitude[k] = cos(c16_config_vertical_angle[k] * DEG_TO_RAD);
                            }
                        }

                    } else {
                        for (int j = 0; j < 16; ++j) {
                            uint8_t data1 = difop_packet_ptr->data[245 + 2 * j];
                            uint8_t data2 = difop_packet_ptr->data[245 + 2 * j + 1];
                            int vert_angle = data1 * 256 + data2;
                            vert_angle = vert_angle > 32767 ? (vert_angle - 65536) : vert_angle;
                            c16_config_vertical_angle[j] = (double) vert_angle * 0.01f;
                            if (fabs(c16_vertical_angle[j] - c16_config_vertical_angle[j]) > 1.5) {
                                config_vert_num++;
                            }
                        }
                        if (config_vert_num == 0) {
                            for (int k = 0; k < 16; ++k) {
                                c16_sin_scan_altitude[k] = sin(c16_config_vertical_angle[k] * DEG_TO_RAD);
                                c16_cos_scan_altitude[k] = cos(c16_config_vertical_angle[k] * DEG_TO_RAD);
                            }
                        }
                    }
                    config_vert = false;
                }
                if (packet_size==1206) {
                    if (2 == version_data) {
                    this->packetTimeStamp[4] = difop_packet_ptr->data[41];
                    this->packetTimeStamp[5] = difop_packet_ptr->data[40];
                    this->packetTimeStamp[6] = difop_packet_ptr->data[39];
                    this->packetTimeStamp[7] = difop_packet_ptr->data[38];
                    this->packetTimeStamp[8] = difop_packet_ptr->data[37];
                    this->packetTimeStamp[9] = difop_packet_ptr->data[36];
                } else {
                    this->packetTimeStamp[4] = difop_packet_ptr->data[57];
                    this->packetTimeStamp[5] = difop_packet_ptr->data[56];
                    this->packetTimeStamp[6] = difop_packet_ptr->data[55];
                    this->packetTimeStamp[7] = difop_packet_ptr->data[54];
                    this->packetTimeStamp[8] = difop_packet_ptr->data[53];
                    this->packetTimeStamp[9] = difop_packet_ptr->data[52];
                }
                }

            } else if (rc < 0) {
                return;
            }
//            rclcpp::spinOnce();

        }
    }


    void lslidarC16Driver::decodePacket(const RawPacket *packet) {
        //couputer azimuth angle for each firing
        //even numbers
        for (size_t b_idx = 0; b_idx < BLOCKS_PER_PACKET; ++b_idx) {
            firings.firing_azimuth[b_idx] = packet->blocks[b_idx].rotation % 36000; //* 0.01 * DEG_TO_RAD;
        }

        // computer distance ,intensity
        //12 blocks
        for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
            const RawBlock &raw_block = packet->blocks[blk_idx];

            int32_t azimuth_diff = 0;
            if (1 == return_mode) {
                if (blk_idx < BLOCKS_PER_PACKET - 1) {
                    azimuth_diff = firings.firing_azimuth[blk_idx + 1] - firings.firing_azimuth[blk_idx];
                    azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 36000 : azimuth_diff;
                } else {
                    azimuth_diff = firings.firing_azimuth[blk_idx] - firings.firing_azimuth[blk_idx - 1];
                    azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 36000 : azimuth_diff;
                }
            } else {
                if (blk_idx < BLOCKS_PER_PACKET - 2) {
                    azimuth_diff = firings.firing_azimuth[blk_idx + 2] - firings.firing_azimuth[blk_idx];
                    azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 36000 : azimuth_diff;
                } else {
                    azimuth_diff = firings.firing_azimuth[blk_idx] - firings.firing_azimuth[blk_idx - 2];
                    azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 36000 : azimuth_diff;
                }

            }
            for (size_t scan_idx = 0; scan_idx < SCANS_PER_BLOCK; ++scan_idx) {
                size_t byte_idx = RAW_SCAN_SIZE * scan_idx;
                //azimuth
                firings.azimuth[blk_idx * 32 + scan_idx] =
                        firings.firing_azimuth[blk_idx] + scan_idx * azimuth_diff / FIRING_TOFFSET;
                firings.azimuth[blk_idx * 32 + scan_idx] = firings.azimuth[blk_idx * 32 + scan_idx] % 36000;
                // distance
                firings.distance[blk_idx * 32 + scan_idx] =
                        static_cast<double>(raw_block.data[byte_idx] + raw_block.data[byte_idx + 1] * 256) *
                        DISTANCE_RESOLUTION * distance_unit;

                //intensity
                firings.intensity[blk_idx * 32 + scan_idx] = static_cast<double>(
                        raw_block.data[byte_idx + 2]);
            }

        }
        return;
    }

/** poll the device
 *  @returns true unless end of file reached
 */
    bool lslidarC16Driver::poll(void) {  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        lslidar_msgs::msg::LslidarPacket::UniquePtr packet(new lslidar_msgs::msg::LslidarPacket());

        // Since the rslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        while (true) {
            int rc = msop_input_->getPacket(packet);
            if (rc == 0) break;
            if (rc < 0) {
                return false;
            }
        }
        const RawPacket *raw_packet = (const RawPacket *) (&(packet->data[0]));

        //check if the packet is valid
        if (!checkPacketValidity(raw_packet)) {
            return false;
        }

        // packet timestamp
        if (use_gps_ts) {
            if(packet_size==1212) {
                lslidar_msgs::msg::LslidarPacket pkt = *packet;
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_year = pkt.data[1200] + 2000 - 1900;
                cur_time.tm_mon = pkt.data[1201] - 1;
                cur_time.tm_mday = pkt.data[1202];
                cur_time.tm_hour = pkt.data[1203];
                cur_time.tm_min = pkt.data[1204];
                cur_time.tm_sec = pkt.data[1205];
                packet_time_s = static_cast<uint64_t>(timegm(&cur_time)); //s
                packet_time_ns = (pkt.data[1206] +
                                  pkt.data[1207] * pow(2, 8) +
                                  pkt.data[1208] * pow(2, 16) +
                                  pkt.data[1209] * pow(2, 24)) * 1e3; //ns
                timeStamp = rclcpp::Time(packet_time_s, packet_time_ns);
                packet->stamp = timeStamp;
                current_packet_time = timeStamp.seconds();

                if (packet->data[1210] == 0x39) {
                    return_mode = 2;
                }
            }else {
                lslidar_msgs::msg::LslidarPacket pkt = *packet;
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
                cur_time.tm_mon = this->packetTimeStamp[8] - 1;
                cur_time.tm_mday = this->packetTimeStamp[7];
                cur_time.tm_hour = this->packetTimeStamp[6];
                cur_time.tm_min = this->packetTimeStamp[5];
                cur_time.tm_sec = this->packetTimeStamp[4];
                packet_time_s = static_cast<uint64_t>(timegm(&cur_time)); //s
                packet_time_ns = (pkt.data[1200] +
                                  pkt.data[1201] * pow(2, 8) +
                                  pkt.data[1202] * pow(2, 16) +
                                  pkt.data[1203] * pow(2, 24)) * 1e3; //ns
                timeStamp = rclcpp::Time(packet_time_s, packet_time_ns);
                //使用gps授时
                if ((timeStamp.seconds() - timeStamp_bak.seconds()) < 1e-9  && (timeStamp.seconds() - timeStamp_bak.seconds()) > -1.0
                && is_update_gps_time  && packet_time_ns < 100000000) {
                    timeStamp = rclcpp::Time(packet_time_s + 1, packet_time_ns);
                } else if ((timeStamp - timeStamp_bak).seconds() > 1.0  && (timeStamp - timeStamp_bak).seconds() < 1.2  && is_update_gps_time
                && packet_time_ns > 900000000) {
                    timeStamp = rclcpp::Time(packet_time_s - 1, packet_time_ns);
                }
                timeStamp_bak = timeStamp;
                packet->stamp = timeStamp;
                current_packet_time = timeStamp.seconds();

                if (packet->data[1204] == 0x39) {
                    return_mode = 2;
                }
            }

        } else {
            packet->stamp = get_clock()->now();
            current_packet_time = rclcpp::Time(packet->stamp).seconds();
        }

        RCLCPP_INFO_ONCE(this->get_logger(), "return mode: %d", return_mode);


        //decode the packet
        decodePacket(raw_packet);
        // find the start of a new revolution
        // if there is one, new_sweep_start will be the index of the start firing,
        // otherwise, new_sweep_start will be FIRINGS_PER_PACKET.
        size_t new_sweep_start = 0;
        do {
            if (last_azimuth - firings.azimuth[new_sweep_start] > 35000) {
                break;
            } else {
                last_azimuth = firings.azimuth[new_sweep_start];
                ++new_sweep_start;
            }
        } while (new_sweep_start < SCANS_PER_PACKET);

        // The first sweep may not be complete. So, the firings with
        // the first sweep will be discarded. We will wait for the
        // second sweep in order to find the 0 azimuth angle.
        size_t start_fir_idx = 0;
        size_t end_fir_idx = new_sweep_start;
        if (is_first_sweep && new_sweep_start == SCANS_PER_PACKET) {
            return true;
        } else {
            if (is_first_sweep) {
                is_first_sweep = false;
                start_fir_idx = new_sweep_start;
                end_fir_idx = SCANS_PER_PACKET;
                //sweep_start_time = packet->stamp.toSec() - (end_fir_idx - start_fir_idx) * 3.125 * 1e-6;
            }
        }
        for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
            //check if the point is valid
            if (!isPointInRange(firings.distance[fir_idx]))continue;
            //if (firings.azimuth[fir_idx]>=36000) { firings.azimuth[fir_idx]-=36000;}  //todo
            //convert the point to xyz coordinate
            size_t table_idx = firings.azimuth[fir_idx];
            double cos_azimuth = cos_azimuth_table[table_idx];
            double sin_azimuth = sin_azimuth_table[table_idx];
            double x_coord, y_coord, z_coord;
            if (coordinate_opt) {
                x_coord = firings.distance[fir_idx] * c16_cos_scan_altitude[fir_idx % 16] * cos_azimuth +
                          R1_ * cos((14.68 - firings.azimuth[fir_idx] * 0.01) * DEG_TO_RAD);
                y_coord = -firings.distance[fir_idx] * c16_cos_scan_altitude[fir_idx % 16] * sin_azimuth +
                          R1_ * sin((14.68 - firings.azimuth[fir_idx] * 0.01) * DEG_TO_RAD);
                z_coord = firings.distance[fir_idx] * c16_sin_scan_altitude[fir_idx % 16];

            } else {
                //Y-axis correspondence 0 degree
                x_coord = firings.distance[fir_idx] * c16_cos_scan_altitude[fir_idx % 16] * sin_azimuth +
                          R1_ * sin((firings.azimuth[fir_idx] * 0.01 - 14.68) * DEG_TO_RAD);
                y_coord = firings.distance[fir_idx] * c16_cos_scan_altitude[fir_idx % 16] * cos_azimuth +
                          R1_ * cos((firings.azimuth[fir_idx] * 0.01 - 14.68) * DEG_TO_RAD);
                z_coord = firings.distance[fir_idx] * c16_sin_scan_altitude[fir_idx % 16];
            }
            // computer the time of the point
            double time;
//            if (1 == return_mode) {
//
//                time = rclcpp::Time(packet->stamp).seconds() - (SCANS_PER_PACKET - fir_idx - 1) * 3.125 * 1e-6;
//            } else {
//                time = rclcpp::Time(packet->stamp).seconds() -
//                       (SCANS_PER_PACKET * 0.5 - (fir_idx - round((fir_idx / 32) * 0.5) * 32) - 1) * 3.125 * 1e-6;
//            }

            if (1 == return_mode) {
                // modify
                if(last_packet_time >1e-6) {
                    time = current_packet_time - (SCANS_PER_PACKET - fir_idx - 1) * (current_packet_time - last_packet_time)/(SCANS_PER_PACKET);
                } else{              //fist packet
                    time = current_packet_time;
                }
            } else {
                if(last_packet_time >1e-6) {
                    time = current_packet_time - (SCANS_PER_PACKET - fir_idx - 1) * (current_packet_time - last_packet_time)/(SCANS_PER_PACKET);
//                    time = current_packet_time -
//                           (SCANS_PER_PACKET * 0.5 - (fir_idx - round((fir_idx / 32) * 0.5) * 32) - 1)
//                           * (current_packet_time - last_packet_time) / (SCANS_PER_PACKET);
                }else{              //fist packet
                    time = current_packet_time;
                }
            }

            int remapped_scan_idx = (fir_idx % 16) % 2 == 0 ? (fir_idx % 16) / 2 : (fir_idx % 16) / 2 + 8;

            sweep_data->points.push_back(lslidar_msgs::msg::LslidarPoint());
            lslidar_msgs::msg::LslidarPoint &new_point = sweep_data->points[
                    sweep_data->points.size() - 1];
            //pack the data into point msg
            new_point.time = time;
            new_point.x = x_coord;
            new_point.y = y_coord;
            new_point.z = z_coord;
            new_point.intensity = firings.intensity[fir_idx];
            new_point.ring = remapped_scan_idx;
            new_point.azimuth = firings.azimuth[fir_idx];
            new_point.distance = firings.distance[fir_idx];
        }
        // a new sweep begins ----------------------------------------------------

        if (end_fir_idx != SCANS_PER_PACKET) {
            //publish Last frame scan
//            if (1 == return_mode) {
//                sweep_end_time =
//                        rclcpp::Time(packet->stamp).seconds() -
//                        static_cast<double>(SCANS_PER_PACKET - end_fir_idx) * 3.125 * 1e-6;
//            } else {
//                sweep_end_time =
//                        rclcpp::Time(packet->stamp).seconds() -
//                        static_cast<double>( SCANS_PER_PACKET * 0.5 -
//                                             (end_fir_idx - round((end_fir_idx / 32) * 0.5) * 32)) * 3.125 * 1e-6;
//            }

            if (1 == return_mode) {
                // modify
                if(last_packet_time >1e-6) {
                    sweep_end_time = current_packet_time - static_cast<double>(SCANS_PER_PACKET - end_fir_idx) *
                                                           (current_packet_time - last_packet_time)/(SCANS_PER_PACKET);
                } else{              //fist packet
                    sweep_end_time = current_packet_time;
                }
            } else {
                if(last_packet_time >1e-6) {
                    sweep_end_time = current_packet_time - static_cast<double>(SCANS_PER_PACKET - end_fir_idx) *
                                                           (current_packet_time - last_packet_time)/(SCANS_PER_PACKET);
                }else{              //fist packet
                    sweep_end_time = current_packet_time;
                }
            }


            sweep_end_time = sweep_end_time > 0 ? sweep_end_time : 0;
            publishPointcloud();
            if (publish_scan) publishScan();

            sweep_data = lslidar_msgs::msg::LslidarScan::UniquePtr(new lslidar_msgs::msg::LslidarScan());
            //prepare the next frame scan
            //sweep_start_time = packet->stamp.toSec() - (end_fir_idx - start_fir_idx) * 3.125 * 1e-6;
            last_azimuth = firings.azimuth[SCANS_PER_PACKET - 1];
            start_fir_idx = end_fir_idx;
            end_fir_idx = SCANS_PER_PACKET;
            for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {

                //check if the point is valid
                if (!isPointInRange(firings.distance[fir_idx]))continue;
                //if (firings.azimuth[fir_idx]>=36000) { firings.azimuth[fir_idx]-=36000;} todo
                //convert the point to xyz coordinate
                size_t table_idx = firings.azimuth[fir_idx];
                double cos_azimuth = cos_azimuth_table[table_idx];
                double sin_azimuth = sin_azimuth_table[table_idx];
                double x_coord, y_coord, z_coord;
                if (coordinate_opt) {
                    x_coord = firings.distance[fir_idx] * c16_cos_scan_altitude[fir_idx % 16] * cos_azimuth +
                              R1_ * cos((14.68 - firings.azimuth[fir_idx] * 0.01) * DEG_TO_RAD);
                    y_coord = -firings.distance[fir_idx] * c16_cos_scan_altitude[fir_idx % 16] * sin_azimuth +
                              R1_ * sin((14.68 - firings.azimuth[fir_idx] * 0.01) * DEG_TO_RAD);
                    z_coord = firings.distance[fir_idx] * c16_sin_scan_altitude[fir_idx % 16];

                } else {
                    //Y-axis correspondence 0 degree
                    x_coord = firings.distance[fir_idx] * c16_cos_scan_altitude[fir_idx % 16] * sin_azimuth +
                              R1_ * sin((firings.azimuth[fir_idx] * 0.01 - 14.68) * DEG_TO_RAD);
                    y_coord = firings.distance[fir_idx] * c16_cos_scan_altitude[fir_idx % 16] * cos_azimuth +
                              R1_ * cos((firings.azimuth[fir_idx] * 0.01 - 14.68) * DEG_TO_RAD);
                    z_coord = firings.distance[fir_idx] * c16_sin_scan_altitude[fir_idx % 16];

                }
                // computer the time of the point
                double time;
//                if (1 == return_mode) {
//                    time = rclcpp::Time(packet->stamp).seconds() - (SCANS_PER_PACKET - fir_idx - 1) * 3.125 * 1e-6;
//                } else {
//                    time = rclcpp::Time(packet->stamp).seconds() -
//                           (SCANS_PER_PACKET * 0.5 - (fir_idx - round((fir_idx / 32) * 0.5) * 32) - 1) * 3.125 * 1e-6;
//                }

                if (1 == return_mode) {
                    // modify
                    if(last_packet_time >1e-6) {
                        time = current_packet_time - (SCANS_PER_PACKET - fir_idx - 1) * (current_packet_time - last_packet_time)/(SCANS_PER_PACKET);
                    } else{              //fist packet
                        time = current_packet_time;
                    }
                } else {
                    if(last_packet_time >1e-6) {
                        time = current_packet_time - (SCANS_PER_PACKET - fir_idx - 1) * (current_packet_time - last_packet_time)/(SCANS_PER_PACKET);
                    }else{              //fist packet
                        time = current_packet_time;
                    }
                }

                int remapped_scan_idx = (fir_idx % 16) % 2 == 0 ? (fir_idx % 16) / 2 : (fir_idx % 16) / 2 + 8;
                sweep_data->points.push_back(lslidar_msgs::msg::LslidarPoint());
                lslidar_msgs::msg::LslidarPoint &new_point = sweep_data->points[
                        sweep_data->points.size() - 1];
                //pack the data into point msg
                new_point.time = time;
                new_point.x = x_coord;
                new_point.y = y_coord;
                new_point.z = z_coord;
                new_point.intensity = firings.intensity[fir_idx];
                new_point.ring = remapped_scan_idx;
                new_point.azimuth = firings.azimuth[fir_idx];
                new_point.distance = firings.distance[fir_idx];
            }

        }
        last_packet_time = current_packet_time;
        //packet_pub.publish(*packet);
        return true;
    }


    lslidarC32Driver::lslidarC32Driver() : lslidarDriver() {
//        this->initialize();
        get_difop = false;
        return;
    }

    lslidarC32Driver::~lslidarC32Driver() {
        if (difop_thread_ != nullptr) {
//            difop_thread_->interrupt();
            difop_thread_->join();
        }
    }

    bool lslidarC32Driver::initialize() {
        RCLCPP_INFO(this->get_logger(), "lslidarC32Driver::initialize()");
        this->initTimeStamp();
        if (!loadParameters()) {
            RCLCPP_ERROR(this->get_logger(), "cannot load all required ROS parameters.");
            return false;
        }
        if (c32_type == "c32_2") {
            if (c32_fpga_type == 2) {
                for (int j = 0; j < 32; ++j) {
                    c32_vertical_angle[j] = c32_2_vertical_angle_26[j];
                    c32_config_vertical_angle[j] = c32_2_vertical_angle_26[j];
                    c32_sin_scan_altitude[j] = sin(c32_vertical_angle[j] * DEG_TO_RAD);
                    c32_cos_scan_altitude[j] = cos(c32_vertical_angle[j] * DEG_TO_RAD);
                }
            } else {
                for (int j = 0; j < 32; ++j) {
                    c32_vertical_angle[j] = c32_2_vertical_angle[j];
                    c32_config_vertical_angle[j] = c32_2_vertical_angle[j];
                    c32_sin_scan_altitude[j] = sin(c32_vertical_angle[j] * DEG_TO_RAD);
                    c32_cos_scan_altitude[j] = cos(c32_vertical_angle[j] * DEG_TO_RAD);
                }
            }
        } else {
            if (c32_fpga_type == 2) {
                for (int j = 0; j < 32; ++j) {
                    c32_vertical_angle[j] = c32_1_vertical_angle_26[j];
                    c32_config_vertical_angle[j] = c32_1_vertical_angle_26[j];
                    c32_sin_scan_altitude[j] = sin(c32_vertical_angle[j] * DEG_TO_RAD);
                    c32_cos_scan_altitude[j] = cos(c32_vertical_angle[j] * DEG_TO_RAD);
                }
            } else {
                for (int j = 0; j < 32; ++j) {
                    c32_vertical_angle[j] = c32_1_vertical_angle[j];
                    c32_config_vertical_angle[j] = c32_1_vertical_angle[j];
                    c32_sin_scan_altitude[j] = sin(c32_vertical_angle[j] * DEG_TO_RAD);
                    c32_cos_scan_altitude[j] = cos(c32_vertical_angle[j] * DEG_TO_RAD);
                }
            }
        }

        if (!createRosIO()) {
            RCLCPP_ERROR(this->get_logger(), "cannot create all ROS IO.");
            return false;
        }

        for (int i = 0; i < 4; ++i) {
            adjust_angle[i] = 0.0;
        }

        for (int j = 0; j < 36000; ++j) {
            double angle = static_cast<double>(j) / 100.0 * DEG_TO_RAD;
            sin_azimuth_table[j] = sin(angle);
            cos_azimuth_table[j] = cos(angle);
        }
        angle_base = M_PI * 2 / point_num;
        return true;
    }

    void lslidarC32Driver::difopPoll() {
        // reading and publishing scans as fast as possible.
        lslidar_msgs::msg::LslidarPacket::UniquePtr difop_packet_ptr(new lslidar_msgs::msg::LslidarPacket());
        while (rclcpp::ok()) {
        //RCLCPP_INFO(this->get_logger(), "423542354354356563");
            // keep reading
            int rc = difop_input_->getPacket(difop_packet_ptr);
            if (rc == 0) {
                if (difop_packet_ptr->data[0] != 0xa5 || difop_packet_ptr->data[1] != 0xff ||
                    difop_packet_ptr->data[2] != 0x00 || difop_packet_ptr->data[3] != 0x5a) {
                    return;
                }
                count++;
                if (count == 2) { is_update_gps_time = true; }
                for (int i = 0; i < packet_size; i++) {
                    difop_data[i] = difop_packet_ptr->data[i];
                }
                RCLCPP_INFO_ONCE(this->get_logger(), "c32 vertical angle resolution type: %s; c32 fpga type: %0.1f",
                                 c32_type.c_str(),
                                 difop_data[1202] + int(difop_data[1203] / 16) * 0.1);
                //int version_data = difop_packet_ptr->data[1202];
                if (config_vert) {
                    if (3 == c32_fpga_type) {
                        for (int i = 0; i < 32; ++i) {
                            uint8_t data1 = difop_packet_ptr->data[245 + 2 * i];
                            uint8_t data2 = difop_packet_ptr->data[245 + 2 * i + 1];
                            int vert_angle = data1 * 256 + data2;
                            vert_angle = vert_angle > 32767 ? (vert_angle - 65536) : vert_angle;
                            c32_config_tmp_angle[i] = (double) vert_angle * 0.01f;

                        }
                        for (int j = 0; j < 32; ++j) {
                            c32_config_vertical_angle[j] = c32_config_tmp_angle[adjust_angle_index[j]];

                            if (fabs(c32_vertical_angle[j] - c32_config_vertical_angle[j]) > 3) {
                                config_vert_num++;
                            }
                        }
                        if (config_vert_num == 0) {
                            for (int k = 0; k < 32; ++k) {
                                c32_sin_scan_altitude[k] = sin(c32_config_vertical_angle[k] * DEG_TO_RAD);
                                c32_cos_scan_altitude[k] = cos(c32_config_vertical_angle[k] * DEG_TO_RAD);
                            }
                        }
                        // horizontal correction angle
                        int angle_a0 = difop_packet_ptr->data[186] * 256 + difop_packet_ptr->data[187];
                        adjust_angle[0] = angle_a0 > 32767 ? angle_a0 - 65536 : angle_a0;

                        int angle_a1 = difop_packet_ptr->data[190] * 256 + difop_packet_ptr->data[191];
                        adjust_angle[1] = angle_a1 > 32767 ? angle_a1 - 65536 : angle_a1;

                        int angle_a2 = difop_packet_ptr->data[188] * 256 + difop_packet_ptr->data[189];
                        adjust_angle[2] = angle_a2 > 32767 ? angle_a2 - 65536 : angle_a2;

                        int angle_a3 = difop_packet_ptr->data[192] * 256 + difop_packet_ptr->data[193];
                        adjust_angle[3] = angle_a3 > 32767 ? angle_a3 - 65536 : angle_a3;
                    } else {
                        for (int j = 0; j < 32; ++j) {
                            uint8_t data1 = difop_packet_ptr->data[882 + 2 * j];
                            uint8_t data2 = difop_packet_ptr->data[882 + 2 * j + 1];
                            int vert_angle = data1 * 256 + data2;
                            vert_angle = vert_angle > 32767 ? (vert_angle - 65536) : vert_angle;
                            c32_config_vertical_angle[j] = (double) vert_angle * 0.01f;

                            if (fabs(c32_vertical_angle[j] - c32_config_vertical_angle[j]) > 3.0) {
                                config_vert_num++;
                            }
                        }
                        if (config_vert_num == 0) {
                            for (int k = 0; k < 32; ++k) {
                                c32_sin_scan_altitude[k] = sin(c32_config_vertical_angle[k] * DEG_TO_RAD);
                                c32_cos_scan_altitude[k] = cos(c32_config_vertical_angle[k] * DEG_TO_RAD);
                            }
                        }
                        // horizontal correction angle
                        int angle_a0 = difop_packet_ptr->data[34] * 256 + difop_packet_ptr->data[35];
                        adjust_angle[0] = angle_a0 > 32767 ? angle_a0 - 65536 : angle_a0;

                        int angle_a1 = difop_packet_ptr->data[42] * 256 + difop_packet_ptr->data[43];
                        adjust_angle[1] = angle_a1 > 32767 ? angle_a1 - 65536 : angle_a1;

                        int angle_a2 = difop_packet_ptr->data[66] * 256 + difop_packet_ptr->data[67];
                        adjust_angle[2] = angle_a2 > 32767 ? angle_a2 - 65536 : angle_a2;

                        int angle_a3 = difop_packet_ptr->data[68] * 256 + difop_packet_ptr->data[69];
                        adjust_angle[3] = angle_a3 > 32767 ? angle_a3 - 65536 : angle_a3;
                    }
                    get_difop = true;
                    config_vert = false;
                }
				if(packet_size==1206) {
				if (2 == c32_fpga_type) {
                    this->packetTimeStamp[4] = difop_packet_ptr->data[41];
                    this->packetTimeStamp[5] = difop_packet_ptr->data[40];
                    this->packetTimeStamp[6] = difop_packet_ptr->data[39];
                    this->packetTimeStamp[7] = difop_packet_ptr->data[38];
                    this->packetTimeStamp[8] = difop_packet_ptr->data[37];
                    this->packetTimeStamp[9] = difop_packet_ptr->data[36];
                } else {
                    this->packetTimeStamp[4] = difop_packet_ptr->data[57];
                    this->packetTimeStamp[5] = difop_packet_ptr->data[56];
                    this->packetTimeStamp[6] = difop_packet_ptr->data[55];
                    this->packetTimeStamp[7] = difop_packet_ptr->data[54];
                    this->packetTimeStamp[8] = difop_packet_ptr->data[53];
                    this->packetTimeStamp[9] = difop_packet_ptr->data[52];
                }
				}

            } else if (rc < 0) {
                return;
            }
//            rclcpp::spinOnce();
        }
    }


    void lslidarC32Driver::decodePacket(const RawPacket *packet) {
        //couputer azimuth angle for each firing, 12

        for (size_t fir_idx = 0; fir_idx < BLOCKS_PER_PACKET; ++fir_idx) {
            firings.firing_azimuth[fir_idx] = packet->blocks[fir_idx].rotation % 36000; //* 0.01 * DEG_TO_RAD;
            // ROS_INFO("azi==%d",packet->blocks[fir_idx].rotation);
        }
        // ROS_WARN("-----------------");
        // computer distance ,intensity
        //12 blocks
        for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
            const RawBlock &raw_block = packet->blocks[blk_idx];

            int32_t azimuth_diff = 0;
            if (1 == return_mode) {
                if (blk_idx < BLOCKS_PER_PACKET - 1) {
                    azimuth_diff = firings.firing_azimuth[blk_idx + 1] - firings.firing_azimuth[blk_idx];
                } else {
                    azimuth_diff = firings.firing_azimuth[blk_idx] - firings.firing_azimuth[blk_idx - 1];
                }
                azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 36000 : azimuth_diff;
            } else {
                if (blk_idx < BLOCKS_PER_PACKET - 2) {
                    azimuth_diff = firings.firing_azimuth[blk_idx + 2] - firings.firing_azimuth[blk_idx];

                } else {
                    azimuth_diff = firings.firing_azimuth[blk_idx] - firings.firing_azimuth[blk_idx - 2];
                }
                azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 36000 : azimuth_diff;
            }
            // 32 scan
            for (size_t scan_idx = 0; scan_idx < SCANS_PER_BLOCK; ++scan_idx) {
                size_t byte_idx = RAW_SCAN_SIZE * scan_idx;
                //azimuth
                firings.azimuth[blk_idx * 32 + scan_idx] = firings.firing_azimuth[blk_idx] +
                                                           scan_idx * azimuth_diff / FIRING_TOFFSET;

                firings.azimuth[blk_idx * 32 + scan_idx] = firings.azimuth[blk_idx * 32 + scan_idx] % 36000;

                /*
                // calibration azimuth ，1°
              if ("c32_2" == c32_type) {
                    // -----结果是否是正数 ？
                    int adjust_diff = adjust_angle[1] - adjust_angle[0];
                    if (adjust_diff > 300 && adjust_diff < 460) {
                        // fpga :v 2.7
                        if (3 == c32_fpga_type) {
                            if ( 1 >= scan_fir_idx % 4 ) {
                                firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[1];
                            } else {
                                firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                            }
                           // ROS_INFO("id: %d--azi: %d",blk_idx * 32 + scan_fir_idx,firings.azimuth[blk_idx * 32 + scan_fir_idx]);
                        } else {
                            if (0 == scan_fir_idx % 2) {
                                firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[1];
                            } else {
                                firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                            }
                        }
                    } else {
                        // fpga: v2.6
                        if (0 == scan_fir_idx % 2) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                        } else {
                            int tmp_azimuth = firings.azimuth[blk_idx * 32 + scan_fir_idx] - adjust_angle[0];
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] =
                                    tmp_azimuth < 0 ? tmp_azimuth + 36000 : tmp_azimuth;
                        }
                    }
                }
                //  calibration azimuth ,0.33°
                if ("c32_1" == c32_type) {
                    int adjust_diff_one = adjust_angle[1] - adjust_angle[0];
                    int adjust_diff_two = adjust_angle[3] - adjust_angle[2];
                    if (3 == c32_fpga_type) {
                        // fpga v3.0
                        if (0 == scan_fir_idx || 1 == scan_fir_idx || 4 == scan_fir_idx || 8 == scan_fir_idx ||
                            9 == scan_fir_idx || 12 == scan_fir_idx
                            || 16 == scan_fir_idx || 17 == scan_fir_idx || 21 == scan_fir_idx || 24 == scan_fir_idx ||
                            25 == scan_fir_idx || 29 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[3];
                        }
                        if (2 == scan_fir_idx || 3 == scan_fir_idx || 6 == scan_fir_idx || 10 == scan_fir_idx ||
                            11 == scan_fir_idx || 14 == scan_fir_idx
                            || 18 == scan_fir_idx || 19 == scan_fir_idx || 23 == scan_fir_idx || 26 == scan_fir_idx ||
                            27 == scan_fir_idx || 31 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[2];
                        }
                        if (5 == scan_fir_idx || 13 == scan_fir_idx || 20 == scan_fir_idx || 28 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[1];
                        }
                        if (7 == scan_fir_idx || 15 == scan_fir_idx || 22 == scan_fir_idx || 30 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                        }
                    } else if (adjust_diff_one > 500 && adjust_diff_one < 660 && adjust_diff_two > 150 &&
                               adjust_diff_two < 350) {
                        //fpga v2.7
                        if (10 == scan_fir_idx || 14 == scan_fir_idx || 18 == scan_fir_idx || 22 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[3];
                        }
                        if (11 == scan_fir_idx || 15 == scan_fir_idx || 19 == scan_fir_idx || 23 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[2];
                        }
                        if (0 == scan_fir_idx || 2 == scan_fir_idx || 4 == scan_fir_idx || 6 == scan_fir_idx ||
                            8 == scan_fir_idx || 12 == scan_fir_idx
                            || 16 == scan_fir_idx || 20 == scan_fir_idx || 24 == scan_fir_idx || 26 == scan_fir_idx ||
                            28 == scan_fir_idx || 30 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[1];
                        }
                        if (1 == scan_fir_idx || 3 == scan_fir_idx || 5 == scan_fir_idx || 7 == scan_fir_idx ||
                            9 == scan_fir_idx || 13 == scan_fir_idx
                            || 17 == scan_fir_idx || 21 == scan_fir_idx || 25 == scan_fir_idx || 27 == scan_fir_idx ||
                            29 == scan_fir_idx || 31 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                        }
                    } else {
                        // fpga v2.6
                        if (10 == scan_fir_idx || 14 == scan_fir_idx || 18 == scan_fir_idx || 22 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                        }
                        if (11 == scan_fir_idx || 15 == scan_fir_idx || 19 == scan_fir_idx || 23 == scan_fir_idx) {
                            int tmp_azimuth = firings.azimuth[blk_idx * 32 + scan_fir_idx] - adjust_angle[0];
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] =
                                    tmp_azimuth < 0 ? tmp_azimuth + 36000 : tmp_azimuth;
                        }
                        if (0 == scan_fir_idx || 2 == scan_fir_idx || 4 == scan_fir_idx || 6 == scan_fir_idx ||
                            8 == scan_fir_idx || 12 == scan_fir_idx
                            || 16 == scan_fir_idx || 20 == scan_fir_idx || 24 == scan_fir_idx || 26 == scan_fir_idx ||
                            28 == scan_fir_idx || 30 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[1];
                        }
                        if (1 == scan_fir_idx || 3 == scan_fir_idx || 5 == scan_fir_idx || 7 == scan_fir_idx ||
                            9 == scan_fir_idx || 13 == scan_fir_idx
                            || 17 == scan_fir_idx || 21 == scan_fir_idx || 25 == scan_fir_idx || 27 == scan_fir_idx ||
                            29 == scan_fir_idx || 31 == scan_fir_idx) {
                            int tmp_azimuth = firings.azimuth[blk_idx * 32 + scan_fir_idx] - adjust_angle[1];
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] =
                                    tmp_azimuth < 0 ? tmp_azimuth + 36000 : tmp_azimuth;
                        }

                    }

                }


                firings.azimuth[blk_idx * 32 + scan_fir_idx] = firings.azimuth[blk_idx * 32 + scan_fir_idx] % 36000;
                 */

                // distance

                firings.distance[blk_idx * 32 + scan_idx] =
                        static_cast<double>(raw_block.data[byte_idx] + raw_block.data[byte_idx + 1] * 256) *
                        DISTANCE_RESOLUTION * distance_unit;

                //intensity
                firings.intensity[blk_idx * 32 + scan_idx] = static_cast<double>(
                        raw_block.data[byte_idx + 2]);
            }

        }
        return;
    }

/** poll the device
 *  @returns true unless end of file reached
 */
    bool lslidarC32Driver::poll(void) {
        lslidar_msgs::msg::LslidarPacket::UniquePtr packet(new lslidar_msgs::msg::LslidarPacket());

        // Since the rslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        while (true) {
            int rc = msop_input_->getPacket(packet);
            if (rc == 0) break;
            if (rc < 0) return false;
        }
        const RawPacket *raw_packet = (const RawPacket *) (&(packet->data[0]));
        //check if the packet is valid
        if (!checkPacketValidity(raw_packet)) return false;
        // packet timestamp
        if (use_gps_ts) {
            if(packet_size==1212) {
                lslidar_msgs::msg::LslidarPacket pkt = *packet;
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_year = pkt.data[1200] + 2000 - 1900;
                cur_time.tm_mon = pkt.data[1201] - 1;
                cur_time.tm_mday = pkt.data[1202];
                cur_time.tm_hour = pkt.data[1203];
                cur_time.tm_min = pkt.data[1204];
                cur_time.tm_sec = pkt.data[1205];
                packet_time_s = static_cast<uint64_t>(timegm(&cur_time)); //s
                packet_time_ns = (pkt.data[1206] +
                                  pkt.data[1207] * pow(2, 8) +
                                  pkt.data[1208] * pow(2, 16) +
                                  pkt.data[1209] * pow(2, 24)) * 1e3; //ns
                timeStamp = rclcpp::Time(packet_time_s, packet_time_ns);
                packet->stamp = timeStamp;
                current_packet_time = timeStamp.seconds();

                if (packet->data[1210] == 0x39) {
                    return_mode = 2;
                }
            }else{
                lslidar_msgs::msg::LslidarPacket pkt = *packet;
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
                cur_time.tm_mon = this->packetTimeStamp[8] - 1;
                cur_time.tm_mday = this->packetTimeStamp[7];
                cur_time.tm_hour = this->packetTimeStamp[6];
                cur_time.tm_min = this->packetTimeStamp[5];
                cur_time.tm_sec = this->packetTimeStamp[4];
                packet_time_s = static_cast<uint64_t>(timegm(&cur_time)); //s
                packet_time_ns = (pkt.data[1200] +
                                  pkt.data[1201] * pow(2, 8) +
                                  pkt.data[1202] * pow(2, 16) +
                                  pkt.data[1203] * pow(2, 24)) * 1e3; //ns
                timeStamp = rclcpp::Time(packet_time_s, packet_time_ns);
                RCLCPP_INFO_ONCE(this->get_logger(), "timeStamp: %f", timeStamp.seconds());
                RCLCPP_INFO_ONCE(this->get_logger(), "timeStamp_bak: %f", timeStamp_bak.seconds());
                //使用gps授时
                if ((timeStamp.seconds() - timeStamp_bak.seconds()) < 1e-9  && (timeStamp.seconds() - timeStamp_bak.seconds()) > -1.0
                    && is_update_gps_time  && packet_time_ns < 100000000) {
                    timeStamp = rclcpp::Time(packet_time_s + 1, packet_time_ns);
                } else if ((timeStamp - timeStamp_bak).seconds() > 1.0  && (timeStamp - timeStamp_bak).seconds() < 1.2  && is_update_gps_time
                           && packet_time_ns > 900000000) {
                    timeStamp = rclcpp::Time(packet_time_s - 1, packet_time_ns);
                }
                timeStamp_bak = timeStamp;
                packet->stamp = timeStamp;
                current_packet_time = timeStamp.seconds();
                if (packet->data[1204] == 0x39) {
                    return_mode = 2;
                }
            }
        } else {
            packet->stamp = get_clock()->now();;
            current_packet_time = rclcpp::Time(packet->stamp).seconds();
        }

        RCLCPP_INFO_ONCE(this->get_logger(), "return mode: %d", return_mode);

        //decode the packet
        decodePacket(raw_packet);
        // find the start of a new revolution
        // if there is one, new_sweep_start will be the index of the start firing,
        // otherwise, new_sweep_start will be FIRINGS_PER_PACKET.
        size_t new_sweep_start = 0;
        do {
            if (last_azimuth - firings.azimuth[new_sweep_start] > 35000) {
                break;
            } else {
                last_azimuth = firings.azimuth[new_sweep_start];
                ++new_sweep_start;
            }
        } while (new_sweep_start < SCANS_PER_PACKET);

        // The first sweep may not be complete. So, the firings with
        // the first sweep will be discarded. We will wait for the
        // second sweep in order to find the 0 azimuth angle.
        size_t start_fir_idx = 0;
        size_t end_fir_idx = new_sweep_start;
        if (is_first_sweep && new_sweep_start == SCANS_PER_PACKET) {
            return true;
        } else {
            if (is_first_sweep) {
                is_first_sweep = false;
                start_fir_idx = new_sweep_start;
                end_fir_idx = SCANS_PER_PACKET;
                //sweep_start_time = packet->stamp.toSec() - (end_fir_idx - start_fir_idx) * 3.125 * 1e-6;
            }
        }
        last_azimuth_tmp = firings.azimuth[SCANS_PER_PACKET - 1];

        for (int blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
            for (int scan_fir_idx = 0; scan_fir_idx < SCANS_PER_BLOCK; ++scan_fir_idx) {

                // calibration azimuth ，1°
                if ("c32_2" == c32_type) {

                    int adjust_diff = adjust_angle[1] - adjust_angle[0];

                    if (adjust_diff > 300 && adjust_diff < 460) {
                        // fpga :v 2.8 3.0
                        if (3 == c32_fpga_type) {
                            if (1 >= scan_fir_idx % 4) {
                                firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[1];
                            } else {
                                firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                            }
                            // ROS_INFO("id: %d--azi: %d",blk_idx * 32 + scan_fir_idx,firings.azimuth[blk_idx * 32 + scan_fir_idx]);
                        }
                    } else {
                        // fpga: v2.6
                        if (0 == scan_fir_idx % 2) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                        } else {
                            int tmp_azimuth = firings.azimuth[blk_idx * 32 + scan_fir_idx] - adjust_angle[0];
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] =
                                    tmp_azimuth < 0 ? tmp_azimuth + 36000 : tmp_azimuth;
                        }
                    }
                }
                //  calibration azimuth ,0.33°
                if ("c32_1" == c32_type) {
                    int adjust_diff_one = adjust_angle[1] - adjust_angle[0];
                    int adjust_diff_two = adjust_angle[3] - adjust_angle[2];
                    if (3 == c32_fpga_type) {
                        // fpga v 2.8 3.0
                        if (0 == scan_fir_idx || 1 == scan_fir_idx || 4 == scan_fir_idx || 8 == scan_fir_idx ||
                            9 == scan_fir_idx || 12 == scan_fir_idx
                            || 16 == scan_fir_idx || 17 == scan_fir_idx || 21 == scan_fir_idx || 24 == scan_fir_idx ||
                            25 == scan_fir_idx || 29 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[3];
                        }
                        if (2 == scan_fir_idx || 3 == scan_fir_idx || 6 == scan_fir_idx || 10 == scan_fir_idx ||
                            11 == scan_fir_idx || 14 == scan_fir_idx
                            || 18 == scan_fir_idx || 19 == scan_fir_idx || 23 == scan_fir_idx || 26 == scan_fir_idx ||
                            27 == scan_fir_idx || 31 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[2];
                        }
                        if (5 == scan_fir_idx || 13 == scan_fir_idx || 20 == scan_fir_idx || 28 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[1];
                        }
                        if (7 == scan_fir_idx || 15 == scan_fir_idx || 22 == scan_fir_idx || 30 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                        }
                    } else {
                        // fpga v2.6
                        if (10 == scan_fir_idx || 14 == scan_fir_idx || 18 == scan_fir_idx || 22 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                        }
                        if (11 == scan_fir_idx || 15 == scan_fir_idx || 19 == scan_fir_idx || 23 == scan_fir_idx) {
                            int tmp_azimuth = firings.azimuth[blk_idx * 32 + scan_fir_idx] - adjust_angle[0];
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] =
                                    tmp_azimuth < 0 ? tmp_azimuth + 36000 : tmp_azimuth;
                        }
                        if (0 == scan_fir_idx || 2 == scan_fir_idx || 4 == scan_fir_idx || 6 == scan_fir_idx ||
                            8 == scan_fir_idx || 12 == scan_fir_idx
                            || 16 == scan_fir_idx || 20 == scan_fir_idx || 24 == scan_fir_idx || 26 == scan_fir_idx ||
                            28 == scan_fir_idx || 30 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[1];
                        }
                        if (1 == scan_fir_idx || 3 == scan_fir_idx || 5 == scan_fir_idx || 7 == scan_fir_idx ||
                            9 == scan_fir_idx || 13 == scan_fir_idx
                            || 17 == scan_fir_idx || 21 == scan_fir_idx || 25 == scan_fir_idx || 27 == scan_fir_idx ||
                            29 == scan_fir_idx || 31 == scan_fir_idx) {
                            int tmp_azimuth = firings.azimuth[blk_idx * 32 + scan_fir_idx] - adjust_angle[1];
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] =
                                    tmp_azimuth < 0 ? tmp_azimuth + 36000 : tmp_azimuth;
                        }

                    }

                }
            }

        }


        for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
            //check if the point is valid
            if (!isPointInRange(firings.distance[fir_idx]))continue;
            //if (firings.azimuth[fir_idx]>=36000) { firings.azimuth[fir_idx]-=36000;}  // todo
            //convert the point to xyz coordinate
            firings.azimuth[fir_idx] =
                    firings.azimuth[fir_idx] >= 36000 ? firings.azimuth[fir_idx] - 36000 : firings.azimuth[fir_idx];
            size_t table_idx = firings.azimuth[fir_idx];
            double cos_azimuth = cos_azimuth_table[table_idx];
            double sin_azimuth = sin_azimuth_table[table_idx];
            double x_coord, y_coord, z_coord;
            if (coordinate_opt) {
                //X-axis correspondence 0 degree
                x_coord = firings.distance[fir_idx] * c32_cos_scan_altitude[fir_idx % 32] * cos_azimuth +
                          R2_ * cos((12.98 - firings.azimuth[fir_idx] * 0.01) * DEG_TO_RAD);
                y_coord = -firings.distance[fir_idx] * c32_cos_scan_altitude[fir_idx % 32] * sin_azimuth +
                          R2_ * sin((12.98 - firings.azimuth[fir_idx] * 0.01) * DEG_TO_RAD);
                z_coord = firings.distance[fir_idx] * c32_sin_scan_altitude[fir_idx % 32];

            } else {
                //Y-axis correspondence 0 degree
                x_coord = firings.distance[fir_idx] * c32_cos_scan_altitude[fir_idx % 32] * sin_azimuth +
                          R2_ * sin((firings.azimuth[fir_idx] * 0.01 - 12.98) * DEG_TO_RAD);
                y_coord = firings.distance[fir_idx] * c32_cos_scan_altitude[fir_idx % 32] * cos_azimuth +
                          R2_ * cos((firings.azimuth[fir_idx] * 0.01 - 12.98) * DEG_TO_RAD);
                z_coord = firings.distance[fir_idx] * c32_sin_scan_altitude[fir_idx % 32];
            }
            // computer the time of the point
            double time;
//            if (1 == return_mode) {
//                time = rclcpp::Time(packet->stamp).seconds() - (SCANS_PER_PACKET - fir_idx - 1) * 3.125 * 1e-6;
//            } else {
//                time = rclcpp::Time(packet->stamp).seconds() -
//                       (SCANS_PER_PACKET * 0.5 - (fir_idx - round((fir_idx / 32) * 0.5) * 32) - 1) * 3.125 * 1e-6;
//            }

            if (1 == return_mode) {
                // modify
                if(last_packet_time >1e-6) {
                    time = current_packet_time - (SCANS_PER_PACKET - fir_idx - 1) * (current_packet_time - last_packet_time)/(SCANS_PER_PACKET);
                } else{              //fist packet
                    time = current_packet_time;
                }
            } else {
                if(last_packet_time >1e-6) {
                    time = current_packet_time - (SCANS_PER_PACKET - fir_idx - 1) * (current_packet_time - last_packet_time)/(SCANS_PER_PACKET);
                }else{              //fist packet
                    time = current_packet_time;
                }
            }

            int remapped_scan_idx;
            if (c32_fpga_type == 2) {
                remapped_scan_idx = fir_idx % 32;
            } else {
                remapped_scan_idx = (fir_idx % 32) % 2 == 0 ? (fir_idx % 32) / 2 : (fir_idx % 32) / 2 + 16;
            }
            sweep_data->points.push_back(lslidar_msgs::msg::LslidarPoint());
            lslidar_msgs::msg::LslidarPoint &new_point = sweep_data->points[
                    sweep_data->points.size() - 1];
            //pack the data into point msg
            new_point.time = time;
            new_point.x = x_coord;
            new_point.y = y_coord;
            new_point.z = z_coord;
            new_point.intensity = firings.intensity[fir_idx];
            new_point.ring = remapped_scan_idx;
            new_point.azimuth = firings.azimuth[fir_idx];
            new_point.distance = firings.distance[fir_idx];
        }
        // a new sweep begins ----------------------------------------------------

        if (end_fir_idx != SCANS_PER_PACKET) {
            //publish Last frame scan

//            sweep_end_time = rclcpp::Time(packet->stamp).seconds() -
//                             static_cast<double>(SCANS_PER_PACKET - end_fir_idx) * 3.125 * 1e-6;
            sweep_end_time = current_packet_time - static_cast<double>(SCANS_PER_PACKET - end_fir_idx) * (current_packet_time - last_packet_time)/(SCANS_PER_PACKET);
            sweep_end_time = sweep_end_time > 0 ? sweep_end_time : 0;
            if (get_difop) publishPointcloud();
            if (publish_scan) publishScan();

            sweep_data = lslidar_msgs::msg::LslidarScan::UniquePtr(new lslidar_msgs::msg::LslidarScan());
            //prepare the next frame scan
            //sweep_start_time = packet->stamp.toSec() - (end_fir_idx - start_fir_idx) * 3.125 * 1e-6;
            last_azimuth = last_azimuth_tmp;
            start_fir_idx = end_fir_idx;
            end_fir_idx = SCANS_PER_PACKET;
            for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
                //check if the point is valid
                if (!isPointInRange(firings.distance[fir_idx]))continue;
                //if (firings.azimuth[fir_idx]>=36000) { firings.azimuth[fir_idx]-=36000;}  // todo
                firings.azimuth[fir_idx] =
                        firings.azimuth[fir_idx] >= 36000 ? firings.azimuth[fir_idx] - 36000 : firings.azimuth[fir_idx];
                //convert the point to xyz coordinate
                size_t table_idx = firings.azimuth[fir_idx];
                double cos_azimuth = cos_azimuth_table[table_idx];
                double sin_azimuth = sin_azimuth_table[table_idx];
                double x_coord, y_coord, z_coord;
                if (coordinate_opt) {
                    //X-axis correspondence 0 degree
                    x_coord = firings.distance[fir_idx] * c32_cos_scan_altitude[fir_idx % 32] * cos_azimuth +
                              R2_ * cos((12.98 - firings.azimuth[fir_idx] * 0.01) * DEG_TO_RAD);
                    y_coord = -firings.distance[fir_idx] * c32_cos_scan_altitude[fir_idx % 32] * sin_azimuth +
                              R2_ * sin((12.98 - firings.azimuth[fir_idx] * 0.01) * DEG_TO_RAD);
                    z_coord = firings.distance[fir_idx] * c32_sin_scan_altitude[fir_idx % 32];

                } else {
                    //Y-axis correspondence 0 degree
                    x_coord = firings.distance[fir_idx] * c32_cos_scan_altitude[fir_idx % 32] * sin_azimuth +
                              R2_ * sin((firings.azimuth[fir_idx] * 0.01 - 12.98) * DEG_TO_RAD);
                    y_coord = firings.distance[fir_idx] * c32_cos_scan_altitude[fir_idx % 32] * cos_azimuth +
                              R2_ * cos((firings.azimuth[fir_idx] * 0.01 - 12.98) * DEG_TO_RAD);
                    z_coord = firings.distance[fir_idx] * c32_sin_scan_altitude[fir_idx % 32];

                }
                // computer the time of the point
//                double time = rclcpp::Time(packet->stamp).seconds() - (SCANS_PER_PACKET - fir_idx) * 3.125 * 1e-6;
                double time = current_packet_time - static_cast<double>(SCANS_PER_PACKET - fir_idx) * (current_packet_time - last_packet_time)/(SCANS_PER_PACKET);
                int remapped_scan_idx;
                if (c32_fpga_type == 2) {
                    remapped_scan_idx = fir_idx % 32;
                } else {
                    remapped_scan_idx = (fir_idx % 32) % 2 == 0 ? (fir_idx % 32) / 2 : (fir_idx % 32) / 2 + 16;
                }
                sweep_data->points.push_back(lslidar_msgs::msg::LslidarPoint());
                lslidar_msgs::msg::LslidarPoint &new_point = sweep_data->points[
                        sweep_data->points.size() - 1];
                //pack the data into point msg
                new_point.time = time;
                new_point.x = x_coord;
                new_point.y = y_coord;
                new_point.z = z_coord;
                new_point.intensity = firings.intensity[fir_idx];
                new_point.ring = remapped_scan_idx;
                new_point.azimuth = firings.azimuth[fir_idx];
                new_point.distance = firings.distance[fir_idx];
            }

        }
        last_packet_time = current_packet_time;
        //packet_pub.publish(*packet);
        return true;
    }


}  // namespace lslidar_driver
