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

#include "driver.h"
#include <algorithm>
#include <memory>

using namespace std::chrono;

namespace apollo {
namespace drivers {
namespace ls180s2_gazel {
    float g_fDistanceAcc = 0.1 * 0.01f;
    float m_offset = 6.37f;
    double sin30 = sin(DEG2RAD(30));
    double sin60 = sin(DEG2RAD(60));
    double cos30 = cos(DEG2RAD(30));

    LslidarChDriver::LslidarChDriver(const std::shared_ptr<::apollo::cyber::Node>& node,
                    const ::apollo::drivers::ls180s2_gazel::Config& conf) :
        node_(node), 
        conf_(conf),
        socket_id(-1),
        min_range(0.15),
        max_range(200),
        packet_rate(11111.0),
        packet_end_time(0.0),
        current_packet_time(0.0),
        last_packet_time(0.0),
        use_time_service(false),
        return_mode(1),
        g_fAngleAcc_V(0.01),
        packet_loss(false),
        is_add_frame_(false),
        get_ms06_param(true),
        is_get_difop_(false), 
        last_packet_number_(-1), 
        current_packet_number_(0), 
        total_packet_loss_(0),
        frame_count(0),
        threadPool_(std::make_unique<ThreadPool>(2)),
        point_cloud_xyzirt_(new pcl::PointCloud<VPoint>),
        point_cloud_xyzirt_bak_(new pcl::PointCloud<VPoint>),
        point_cloud_xyzirt_pub_(new pcl::PointCloud<VPoint>){
        AERROR << "101 LslidarChDriver::LslidarChDriver ls180s2_gazel::Config start line" << std::endl;
        // node_ = apollo::cyber::CreateNode("lslidar_node", "");

        // create the sin and cos table for different azimuth and vertical values
        for (int j = 0; j < 36000; ++j) {
            double angle = static_cast<double>(j) * 0.01 * 0.017453293;
            sin_table[j] = sin(angle);
            cos_table[j] = cos(angle);
        }

        //double mirror_angle[4] = {0, -2, -1, -3};   //摆镜角度   //根据通道不同偏移角度不同
        double mirror_angle[4] = {1.5, -0.5, 0.5, -1.5};   //摆镜角度   //根据通道不同偏移角度不同
        for (int i = 0; i < 4; ++i) {
            cos_mirror_angle[i] = cos(DEG2RAD(mirror_angle[i]));
            sin_mirror_angle[i] = sin(DEG2RAD(mirror_angle[i]));
        }
        AERROR << "117 LslidarChDriver::LslidarChDriver ls180s2_gazel::Config end line" << std::endl;
    }

        LslidarChDriver::LslidarChDriver(const std::shared_ptr<::apollo::cyber::Node>& node,
                    const ::apollo::drivers::lidar::config& conf) :
        node_(node), 
        conf_(conf.lslidar()),
        socket_id(-1),
        min_range(0.15),
        max_range(200),
        packet_rate(11111.0),
        packet_end_time(0.0),
        current_packet_time(0.0),
        last_packet_time(0.0),
        use_time_service(false),
        return_mode(1),
        g_fAngleAcc_V(0.01),
        packet_loss(false),
        is_add_frame_(false),
        get_ms06_param(true),
        is_get_difop_(false), 
        last_packet_number_(-1), 
        current_packet_number_(0), 
        total_packet_loss_(0),
        frame_count(0),
        threadPool_(std::make_unique<ThreadPool>(2)),
        point_cloud_xyzirt_(new pcl::PointCloud<VPoint>),
        point_cloud_xyzirt_bak_(new pcl::PointCloud<VPoint>),
        point_cloud_xyzirt_pub_(new pcl::PointCloud<VPoint>){
        AERROR << "146 LslidarChDriver::LslidarChDriver start line" << std::endl;

        // node_ = apollo::cyber::CreateNode("lslidar_node", "");

        // create the sin and cos table for different azimuth and vertical values
        for (int j = 0; j < 36000; ++j) {
            double angle = static_cast<double>(j) * 0.01 * 0.017453293;
            sin_table[j] = sin(angle);
            cos_table[j] = cos(angle);
        }

        //double mirror_angle[4] = {0, -2, -1, -3};   //摆镜角度   //根据通道不同偏移角度不同
        double mirror_angle[4] = {1.5, -0.5, 0.5, -1.5};   //摆镜角度   //根据通道不同偏移角度不同
        for (int i = 0; i < 4; ++i) {
            cos_mirror_angle[i] = cos(DEG2RAD(mirror_angle[i]));
            sin_mirror_angle[i] = sin(DEG2RAD(mirror_angle[i]));
        }
        AERROR << "163 LslidarChDriver::LslidarChDriver end line" << std::endl;
    }

    LslidarChDriver::~LslidarChDriver() {
        if (nullptr == difop_thread_) {
            difop_thread_->join();
        }
        (void) close(socket_id);
    }

    bool LslidarChDriver::loadParameters() {
        AERROR << "174 LslidarChDriver::loadParameters start line" << std::endl;
        dump_file = ""; // comented in original version of launch file and "" in code
        packet_rate = conf_.packet_rate();
        lidar_ip_string = conf_.devip_str();
        msop_udp_port = conf_.msop_port();
        difop_udp_port = conf_.difop_port();
        add_multicast = conf_.add_multicast();
        group_ip_string = conf_.group_ip();
        min_range = conf_.min_range();
        max_range = conf_.max_range();
        scan_start_angle = conf_.scan_start_angle();
        scan_end_angle = conf_.scan_end_angle();
        frame_id = conf_.frame_id();
        pointcloud_topic = conf_.pointcloud_topic();
        use_time_service = conf_.use_time_service();
        packet_loss = conf_.packet_loss();
        
        AINFO << "Using time service or not: " << use_time_service;
        AINFO << "Is packet loss detection enabled: " << packet_loss;
        //inet_aton(lidar_ip_string.c_str(), &lidar_ip);
        AINFO << "Only accepting packets from IP address: " << lidar_ip_string.c_str();
        if (add_multicast) 
            AINFO << "Opening UDP socket: group_address " << group_ip_string;

        AERROR << "214 LslidarChDriver::loadParameters end line" << std::endl;
        return true;
    }

    bool LslidarChDriver::createCyberIO() {
        AERROR << "219 LslidarChDriver::createCyberIO start line" << std::endl;
        point_cloud_pub = node_->CreateWriter<PointCloud2>(pointcloud_topic);
        point_cloud_final = node_->CreateWriter<PointCloud>("/apollo/sensor/lslidar_point_cloud_parsed");
        if (packet_loss){
            packet_loss_pub = node_->CreateWriter<INT64>("packet_loss");
        }
        frame_rate_service_ = node_->CreateService<LslidarSrvFrameRate, LslidarSrvResult>(
            "set_frame_rate", [this](const std::shared_ptr<LslidarSrvFrameRate>& req, std::shared_ptr<LslidarSrvResult>& res){
                if (!is_get_difop_) {
                    res->set_result(false);
                    std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
                    return true;
                }

                unsigned char config_data[1206];
                mempcpy(config_data, difop_data, 1206);
                if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
                    res->set_result(false);
                    std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
                    return true;
                }
                setPacketHeader(config_data);
                std::string frame_rate_ = "";
                if (req->frame_rate() == 0) {
                    config_data[100] = 0x00;
                    frame_rate_ = "Standard frame rate";
                } else if (req->frame_rate() == 1) {
                    config_data[100] = 0x01;
                    frame_rate_ = "50 percent frame rate";
                } else if (req->frame_rate() == 2) {
                    config_data[100] = 0x02;
                    frame_rate_ = "25 percent frame rate";
                } else {
                    std::cout << "\033[1m\033[31m" << "Parameter error, please check the input parameters" << "\033[0m" << std::endl;
                    res->set_result(false);
                    return true;
                }
                res->set_result(true);
                sendPacketTolidar(config_data);

                std::cout << "\033[1m\033[32m" << "------------------------------------------------------------" << "\033[0m" << std::endl;
                std::cout << "\033[1m\033[32m" <<"Set successfully! Current lidar " << frame_rate_ << "\033[0m" << std::endl;
                
                return true;
        });
        data_ip_service_ = node_->CreateService<LslidarSrvDataIp, LslidarSrvResult>(
            "set_data_ip", [this](const std::shared_ptr<LslidarSrvDataIp>& req, std::shared_ptr<LslidarSrvResult>& res){
                std::regex ipv4(
                        "\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
                if (!regex_match(req->data_ip(), ipv4)) {
                    std::cout << "\033[1m\033[31m" << "Parameter error, please check the input parameters" << "\033[0m" << std::endl;
                    res->set_result(false);
                    return true;
                }

                if (!is_get_difop_) {
                    res->set_result(false);
                    std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
                    return true;
                }

                unsigned char config_data[1206];
                mempcpy(config_data, difop_data, 1206);
                if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
                    res->set_result(false);
                    std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
                    return true;
                }
                setPacketHeader(config_data);
                is_get_difop_ = false;

                unsigned short first_value, second_value, third_value, end_value;
                sscanf(req->data_ip().c_str(), "%hu.%hu.%hu.%hu", &first_value, &second_value, &third_value, &end_value);

                std::string destination_ip = std::to_string(config_data[14]) + "." + std::to_string(config_data[15]) + "." +
                                            std::to_string(config_data[16]) + "." + std::to_string(config_data[17]);
                if (first_value == 0 || first_value == 127 ||
                    (first_value >= 240 && first_value <= 255) || destination_ip == req->data_ip()) {
                    std::cout << "\033[1m\033[31m" << "Parameter error, please check the input parameters" << "\033[0m" << std::endl;
                    res->set_result(false);
                    return true;
                } else {
                    config_data[10] = first_value;
                    config_data[11] = second_value;
                    config_data[12] = third_value;
                    config_data[13] = end_value;
                }
                res->set_result(true);
                sendPacketTolidar(config_data);

                std::cout << "\033[1m\033[32m" << "------------------------------------------------------------" << "\033[0m" << std::endl;
                std::cout << "\033[1m\033[32m" <<"Set successfully! Current lidar ip:" << req->data_ip().c_str() << "\033[0m" << std::endl;
                std::cout << "\033[1m\033[32m" <<"Please modify the corresponding parameters in the launch file" << "\033[0m" << std::endl;

                return true;
            });

        destination_ip_service_ = node_->CreateService<LslidarSrvDestinationIp, LslidarSrvResult>("set_destination_ip", [this](
            const std::shared_ptr<LslidarSrvDestinationIp>& req, std::shared_ptr<LslidarSrvResult>& res){
                std::regex ipv4(
                        "\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
                if (!regex_match(req->data_ip(), ipv4)) {
                    std::cout << "\033[1m\033[31m" << "Parameter error, please check the input parameters" << "\033[0m" << std::endl;
                    res->set_result(false);
                    return true;
                }

                if (!is_get_difop_) {
                    res->set_result(false);
                    std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
                    return true;
                }

                unsigned char config_data[1206];
                mempcpy(config_data, difop_data, 1206);
                if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
                    res->set_result(false);
                    std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
                    return true;
                }
                setPacketHeader(config_data);
                is_get_difop_ = false;
                unsigned short first_value, second_value, third_value, end_value;
                sscanf(req->data_ip().c_str(), "%hu.%hu.%hu.%hu", &first_value, &second_value, &third_value, &end_value);

                std::string data_ip = std::to_string(config_data[10]) + "." + std::to_string(config_data[11]) + "." +
                                    std::to_string(config_data[12]) + "." + std::to_string(config_data[13]);
                if (first_value == 0 || first_value == 127 ||
                    (first_value >= 240 && first_value <= 255) || data_ip == req->data_ip()) {
                    std::cout << "\033[1m\033[31m" << "Parameter error, please check the input parameters" << "\033[0m" << std::endl;
                    res->set_result(false);
                    return true;
                } else {
                    config_data[14] = first_value;
                    config_data[15] = second_value;
                    config_data[16] = third_value;
                    config_data[17] = end_value;
                }
                res->set_result(true);
                sendPacketTolidar(config_data);
                std::cout << "\033[1m\033[32m" << "------------------------------------------------------------" << "\033[0m" << std::endl;
                std::cout << "\033[1m\033[32m" <<"Set successfully! Current lidar destination ip:" << req->data_ip().c_str() << "\033[0m" << std::endl;
                std::cout << "\033[1m\033[32m" <<"Please modify the local IP address" << "\033[0m" << std::endl;

                return true;
            });

        data_port_service_ = node_->CreateService<LslidarSrvDataPort, LslidarSrvResult>("set_data_port", [this](
            const std::shared_ptr<LslidarSrvDataPort>& req, std::shared_ptr<LslidarSrvResult>& res){
                if (!is_get_difop_) {
                    res->set_result(false);
                    std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
                    return true;
                }

                unsigned char config_data[1206];
                mempcpy(config_data, difop_data, 1206);
                if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
                    res->set_result(false);
                    std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
                    return true;
                }
                setPacketHeader(config_data);
                is_get_difop_ = false;
                int dev_port = config_data[26] * 256 + config_data[27];
                if (req->data_port() < 1025 || req->data_port() > 65535 || req->data_port() == dev_port) {
                    std::cout << "\033[1m\033[31m" << "Parameter error, please check the input parameters" << "\033[0m" << std::endl;
                    res->set_result(false);
                    return true;
                } else {
                    config_data[24] = req->data_port() / 256;
                    config_data[25] = req->data_port() % 256;
                }
                res->set_result(true);
                sendPacketTolidar(config_data);

                std::cout << "\033[1m\033[32m" << "------------------------------------------------------------" << "\033[0m" << std::endl;
                std::cout << "\033[1m\033[32m" <<"Set successfully! Current lidar MSOP port:" << req->data_port() << "\033[0m" << std::endl;
                std::cout << "\033[1m\033[32m" <<"Please modify the corresponding parameters in the launch file" << "\033[0m" << std::endl;

                return true;
            });
        dev_port_service_ = node_->CreateService<LslidarSrvDevPort, LslidarSrvResult>("set_dev_port", [this](
            const std::shared_ptr<LslidarSrvDevPort>& req, std::shared_ptr<LslidarSrvResult>& res){
                if (!is_get_difop_) {
                    res->set_result(false);
                    std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
                    return true;
                }

                unsigned char config_data[1206];
                mempcpy(config_data, difop_data, 1206);
                if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
                    res->set_result(false);
                    std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
                    return true;
                }
                setPacketHeader(config_data);
                is_get_difop_ = false;

                int data_port = config_data[24] * 256 + config_data[25];
                if (req->data_port() < 1025 || req->data_port() > 65535 || req->data_port() == data_port) {
                    std::cout << "\033[1m\033[31m" << "Parameter error, please check the input parameters" << "\033[0m" << std::endl;
                    res->set_result(false);
                    return true;
                } else {
                    config_data[26] = req->data_port() / 256;
                    config_data[27] = req->data_port() % 256;
                }
                res->set_result(true);
                sendPacketTolidar(config_data);

                std::cout << "\033[1m\033[32m" << "------------------------------------------------------------" << "\033[0m" << std::endl;
                std::cout << "\033[1m\033[32m" <<"Set successfully! Current lidar DIFOP port:" << req->data_port() << "\033[0m" << std::endl;
                std::cout << "\033[1m\033[32m" <<"Please modify the corresponding parameters in the launch file" << "\033[0m" << std::endl;

                return true;
                });
            
            if (!dump_file.empty()) {
                msop_input_.reset(new apollo::drivers::ls180s2_gazel::InputPCAP(node_, msop_udp_port, packet_rate, dump_file));
                difop_input_.reset(new apollo::drivers::ls180s2_gazel::InputPCAP(node_, difop_udp_port, 1, dump_file));
            } else {
                msop_input_.reset(new apollo::drivers::ls180s2_gazel::InputSocket(node_, msop_udp_port));
                difop_input_.reset(new apollo::drivers::ls180s2_gazel::InputSocket(node_, difop_udp_port));
            }

            difop_thread_ = std::make_shared<std::thread>([this]() { difopPoll(); });

            AERROR << "448 LslidarChDriver::createCyberIO end line" << std::endl;
            return true;
    }

    bool LslidarChDriver::Init() {
        AERROR << "453 LslidarChDriver::Init start line" << std::endl;
        this->initTimeStamp();
        if (!loadParameters()) {
            AERROR << "Cannot load all required ROS parameters...";
            return false;
        }

        if (!createCyberIO()) {
            AERROR << "Cannot create all Cyber IO...";
            return false;
        }
        AERROR << "464 LslidarChDriver::Init end line" << std::endl;
        return true;
    }

    bool LslidarChDriver::polling() {
        AERROR << "469 LslidarChDriver::polling start line " << std::endl;
        // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        // LslidarMsgRawPacket packet(new LslidarMsgRawPacket());
        std::shared_ptr<LslidarMsgRawPacket> packet = std::make_shared<LslidarMsgRawPacket>();
        std::shared_ptr<LslidarRecvData> packet_struct = std::make_shared<LslidarRecvData>();
        LslidarHeader* headerLslidarRawPacket = packet->mutable_header();

        // packet->mutable_prism_angle()->Reserve(4);
        // packet->mutable_data()->Reserve(1206);

        packet->mutable_prism_angle()->Resize(4, 0.0);
        packet->mutable_data()->Resize(1206, 0);

        // Since the lslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        
        while (true) {
            // keep reading until full packet received
            int rc = msop_input_->getPacket(*packet_struct);
            if (rc == 0) {
                break;       // got a full packet?
            }
            if (rc < 0) return false; // end of file reached?
        }

        headerLslidarRawPacket->set_seq(packet_struct->seq);
        headerLslidarRawPacket->set_stamp_mil(packet_struct->header_stamp_mil);
        headerLslidarRawPacket->set_stamp_sec(packet_struct->header_stamp_sec);
        headerLslidarRawPacket->set_frame_id(packet_struct->frame_id);
        packet->set_stamp_sec(packet_struct->stamp_sec);
        packet->set_stamp_mil(packet_struct->stamp_mil);
        for (unsigned int i = 0; i < sizeof(packet_struct->prism_angle) / 8; i++){
            packet->set_prism_angle(i, packet_struct->prism_angle[i]);
        }
        for (unsigned int i = 0; i < sizeof(packet_struct->data); i++){
            packet->set_data(i, packet_struct->data[i]);
        }

        // publish message using time of last packet read

        if (use_time_service) {
            AERROR << std::endl << "515 inside use_time_service " << std::endl;
            // it is already the msop msg
            // use the first packets

            std::shared_ptr<LslidarMsgRawPacket> pkt = packet;
            LslidarHeader* headerLslidarRawPkt = pkt->mutable_header();
            if (0xff == pkt->data(1194)) {    //ptp授时 // No changes here, see if it works for code itself, but uint64_t -> uint32_t     
                uint32_t timestamp_s = (pkt->data(1195) * 0 + (pkt->data(1196) << 24) + 
                                        (pkt->data(1197) << 16) + (pkt->data(1198) << 8) + pkt->data(1199) * pow(2, 0));
                uint32_t timestamp_nsce = (pkt->data(1200) << 24) + (pkt->data(1201) << 16) +
                                          (pkt->data(1202) << 8) + (pkt->data(1203));
                timeStamp = apollo::cyber::Time(timestamp_s, timestamp_nsce);// s,ns
                headerLslidarRawPkt->set_stamp_mil(timeStamp.ToNanosecond());
                headerLslidarRawPkt->set_stamp_sec(timeStamp.ToSecond());
                //packet->header.stamp = timeStamp;             // packet->header.stamp = timeStamp;
                current_packet_time = timeStamp.ToSecond();     // packet->header.stamp.toSec();
            } else {          //gps授时
                this->packetTimeStamp[4] = pkt->data(1199);
                this->packetTimeStamp[5] = pkt->data(1198);
                this->packetTimeStamp[6] = pkt->data(1197);
                this->packetTimeStamp[7] = pkt->data(1196);
                this->packetTimeStamp[8] = pkt->data(1195);
                this->packetTimeStamp[9] = pkt->data(1194);
                struct tm cur_time{};
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_sec = this->packetTimeStamp[4];
                cur_time.tm_min = this->packetTimeStamp[5];
                cur_time.tm_hour = this->packetTimeStamp[6];
                cur_time.tm_mday = this->packetTimeStamp[7];
                cur_time.tm_mon = this->packetTimeStamp[8] - 1;
                cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
                this->pointcloudTimeStamp = static_cast<uint64_t>(timegm(&cur_time)); //s
                uint64_t packet_timestamp;
                packet_timestamp = pkt->data(1203) +
                                   (pkt->data(1202) << 8) +
                                   (pkt->data(1201) << 16) +
                                   (pkt->data(1200) << 24); //ns                           TIME IS DIFFERENT HERE
                timeStamp = apollo::cyber::Time(this->pointcloudTimeStamp, packet_timestamp);
                headerLslidarRawPacket->set_stamp_mil(timeStamp.ToNanosecond());
                headerLslidarRawPacket->set_stamp_sec(timeStamp.ToSecond());
                current_packet_time = timeStamp.ToSecond();       // packet->header.stamp.toSec();
            }
        } else {
            AERROR << std::endl << "515 inside else use_time_service " << std::endl;
            // packet->header.stamp = apollo::cyber::Time::now();
            // current_packet_time = packet->header.stamp.toSec();
            headerLslidarRawPacket->set_stamp_sec(apollo::cyber::Time::Now().ToSecond());
            headerLslidarRawPacket->set_stamp_mil(apollo::cyber::Time::Now().ToNanosecond());
            current_packet_time = apollo::cyber::Time::Now().ToSecond();
        }

        // AERROR << "BEFORE FILE" << std::endl;
        // std::time_t now = std::time(nullptr);
        // std::stringstream filename;
        // filename << "/apollo/modules/drivers/lidar/ls180s2_gazel/driver/data_" << now << ".txt";
        // std::ofstream file(filename.str());
        // if (file.is_open()){
        //     for (int i = 0; i < 1206; i++){
        //         if (i < 1205){
        //             file << "index: " << i << "value: " << packet->data(i) << std::endl;
        //         }else{
        //             file << "index: " << i << "value: " << packet->data(i) << "LAST ONE" << std::endl << std::endl;
        //         }
        //     }
        // }
        // file.close();
        // AERROR << "AFTER FILE" << std::endl;

        lslidarChPacketProcess(packet);
        AERROR << "554 LslidarChDriver::polling end line " << std::endl;
        return true;
    }

    void LslidarChDriver::initTimeStamp() {
        AERROR << "559 LslidarChDriver::initTimeStamp start line " << std::endl;
        for (unsigned char &i : this->packetTimeStamp) {
            i = 0;
        }
        this->pointcloudTimeStamp = 0;
        this->timeStamp = apollo::cyber::Time(0.0);
        AERROR << "565 LslidarChDriver::initTimeStamp end line " << std::endl;
    }

    void LslidarChDriver::difopPoll() {
        AERROR << "569 LslidarChDriver::difopPoll start line " << std::endl;
        //LslidarMsgRawPacket difop_packet(new LslidarMsgRawPacket());
        std::shared_ptr<LslidarMsgRawPacket> difop_packet = std::make_shared<LslidarMsgRawPacket>();
        std::shared_ptr<LslidarRecvData> difop_packet_recv = std::make_shared<LslidarRecvData>();
        LslidarHeader* headerLslidarRawPacketDifop = difop_packet->mutable_header();

        difop_packet->mutable_prism_angle()->Resize(4, 0.0);
        difop_packet->mutable_data()->Resize(1206, 0);


        // reading and publishing scans as fast as possible.
        while (apollo::cyber::OK()) { // ros::ok()
            // keep reading
            int rc = difop_input_->getPacket(*difop_packet_recv);
            
            headerLslidarRawPacketDifop->set_seq(difop_packet_recv->seq);
            headerLslidarRawPacketDifop->set_stamp_mil(difop_packet_recv->header_stamp_mil);
            headerLslidarRawPacketDifop->set_stamp_sec(difop_packet_recv->header_stamp_sec);
            headerLslidarRawPacketDifop->set_frame_id(difop_packet_recv->frame_id);
            difop_packet->set_stamp_mil(difop_packet_recv->stamp_mil);
            difop_packet->set_stamp_sec(difop_packet_recv->stamp_sec);
            for (unsigned int i = 0; i < sizeof(difop_packet_recv->prism_angle) / 8; i++){
                difop_packet->set_prism_angle(i, difop_packet_recv->prism_angle[i]);
            }
            for (unsigned int i = 0; i < sizeof(difop_packet_recv->data); i++){
                difop_packet->set_data(i, difop_packet_recv->data[i]);
            }

            if (rc == 0) {

                AERROR << std::endl << std::endl << "difop_packet->data(0) " << difop_packet->data(0) << " difop_packet->data(1) " << difop_packet->data(1) << " difop_packet->data(2) " << difop_packet->data(2) << " difop_packet->data(3) " << difop_packet->data(3) << std::endl << std::endl;
                if (difop_packet->data(0) == 0x00 || difop_packet->data(0) == 0xa5) {
                    if (difop_packet->data(1) == 0xff && difop_packet->data(2) == 0x00 &&
                        difop_packet->data(3) == 0x5a) {

                        AERROR << std::endl << std::endl << "difop_packet->data(231) " << difop_packet->data(231) << std::endl << std::endl;

                        if (difop_packet->data(231) == 64 || difop_packet->data(231) == 65) {
                            is_add_frame_ = true;
                        }

                        for (int i = 0; i < 1206; i++) {
                            difop_data[i] = difop_packet->data(i);
                        }

                        m_horizontal_point = difop_packet->data(184) * 256 + difop_packet->data(185);

                        int majorVersion = difop_packet->data(1202);
                        int minorVersion1 = difop_packet->data(1203) / 16;
                        //int minorVersion2 = difop_packet->data(1203) % 16;

                        //v1.1 :0.01   //v1.2以后  ： 0.0025
                        if (1 > majorVersion || (1 == majorVersion && minorVersion1 > 1)) {
                            g_fAngleAcc_V = 0.0025;
                        } else {
                            g_fAngleAcc_V = 0.01;
                        }

                        float fInitAngle_V = difop_packet->data(188) * 256 + difop_packet->data(189);
                        if (fInitAngle_V > 32767) {
                            fInitAngle_V = fInitAngle_V - 65536;
                        }
                        this->prism_angle[0] = fInitAngle_V * g_fAngleAcc_V;

                        fInitAngle_V = difop_packet->data(190) * 256 + difop_packet->data(191);
                        if (fInitAngle_V > 32767) {
                            fInitAngle_V = fInitAngle_V - 65536;
                        }
                        this->prism_angle[1] = fInitAngle_V * g_fAngleAcc_V;

                        fInitAngle_V = difop_packet->data(192) * 256 + difop_packet->data(193);
                        if (fInitAngle_V > 32767) {
                            fInitAngle_V = fInitAngle_V - 65536;
                        }
                        this->prism_angle[2] = fInitAngle_V * g_fAngleAcc_V;

                        fInitAngle_V = difop_packet->data(194) * 256 + difop_packet->data(195);
                        if (fInitAngle_V > 32767) {
                            fInitAngle_V = fInitAngle_V - 65536;
                        }
                        this->prism_angle[3] = fInitAngle_V * g_fAngleAcc_V;
                        is_get_difop_ = true;
                    }
                }
            } else if (rc < 0) {
                return;
            }
            AERROR << "631 LslidarChDriver::difopPoll end line " << std::endl;
            // ros::spinOnce(); nothing to add here, still should work
        }
    }

    void LslidarChDriver::publishPointCloudNew() {
        AERROR << "637 LslidarChDriver::publishPointCloudNew start line " << std::endl;
        if (!is_get_difop_) return;
        std::unique_lock<std::mutex> lock(pc_mutex_);
        point_cloud_xyzirt_pub_->header.frame_id = frame_id;
        point_cloud_xyzirt_pub_->height = 1;
        PointCloud2 pc_msg;
        std::shared_ptr<PointCloud> apollo_msg = std::make_shared<PointCloud>();
        LslidarHeader* headerPointCloud2 = pc_msg.mutable_header();
        // LslidarChDriver::convertPCLtoROSMsg(*point_cloud_xyzirt_pub_, pc_msg, *headerPointCloud2); // possible problem
        converter(*point_cloud_xyzirt_pub_, pc_msg);
        convert_PointCloud(apollo_msg, pc_msg);
        headerPointCloud2->set_stamp_mil(packet_timeStamp);  // pc_msg.header.stamp = packet_timeStamp; // quetinable but will try
        point_cloud_pub->Write(pc_msg);
        point_cloud_final->Write(apollo_msg);
        AERROR << "pointcloud size: " << pc_msg.width();
        AERROR << "648 LslidarChDriver::publishPointCloudNew end line " << std::endl;
    }

    int LslidarChDriver::convertCoordinate(const struct Firing &lidardata) {
        //AERROR << "652 LslidarChDriver::convertCoordinate start line " << std::endl;
        double fAngle_H = 0.0;         //水平角度
        double fAngle_V = 0.0;         // 垂直角度
        fAngle_H = lidardata.azimuth;
        fAngle_V = lidardata.vertical_angle;

        //加畸变
        double fSinV_angle = 0;
        double fCosV_angle = 0;

        //振镜偏移角度 = 实际垂直角度 / 2  - 偏移值
        double fGalvanometrtAngle = 0;
        //fGalvanometrtAngle = (((fAngle_V + 0.05) / 0.8) + 1) * 0.46 + 6.72;
        //fGalvanometrtAngle = fAngle_V + 7.26;
        //fGalvanometrtAngle = fAngle_V + 6.37;
        fGalvanometrtAngle = fAngle_V + m_offset;

        while (fGalvanometrtAngle < 0.0) {
            fGalvanometrtAngle += 360.0;
        }
        while (fAngle_H < 0.0) {
            fAngle_H += 360.0;
        }

        int table_index_V = int(fGalvanometrtAngle * 100) % 36000;
        int table_index_H = int(fAngle_H * 100) % 36000;

        double fAngle_R0 = apollo::drivers::ls180s2_gazel::cos30 * cos_mirror_angle[lidardata.channel_number % 4] * cos_table[table_index_V] -
                           sin_table[table_index_V] * sin_mirror_angle[lidardata.channel_number % 4];

        fSinV_angle = 2 * fAngle_R0 * sin_table[table_index_V] + sin_mirror_angle[lidardata.channel_number % 4];
        fCosV_angle = sqrt(1 - pow(fSinV_angle, 2));

        double fSinCite = (2 * fAngle_R0 * cos_table[table_index_V] * apollo::drivers::ls180s2_gazel::sin30 -
                           cos_mirror_angle[lidardata.channel_number % 4] * apollo::drivers::ls180s2_gazel::sin60) / fCosV_angle;
        double fCosCite = sqrt(1 - pow(fSinCite, 2));

        double fSinCite_H = sin_table[table_index_H] * fCosCite + cos_table[table_index_H] * fSinCite;
        double fCosCite_H = cos_table[table_index_H] * fCosCite - sin_table[table_index_H] * fSinCite;

        double x_coord = 0.0, y_coord = 0.0, z_coord = 0.0;
        x_coord = (lidardata.distance * fCosV_angle * fSinCite_H) * g_fDistanceAcc;
        y_coord = (lidardata.distance * fCosV_angle * fCosCite_H) * g_fDistanceAcc;
        z_coord = (lidardata.distance * fSinV_angle) * g_fDistanceAcc;

        //pcl::PointXYZI point;
        VPoint point;
        point.x = x_coord;
        point.y = y_coord;
        point.z = z_coord;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.channel_number;
        point.timestamp = lidardata.time;
        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        point_cloud_xyzirt_bak_->points.push_back(point);
        ++point_cloud_xyzirt_bak_->width;
        //AERROR << "710 LslidarChDriver::convertCoordinate end line " << std::endl;
        return 0;
    }

    bool LslidarChDriver::lslidarChPacketProcess(const std::shared_ptr<LslidarMsgRawPacket> &msg) { // Possible problems with msg value setting
        AERROR << "715 LslidarChDriver::lslidarChPacketProcess start line " << std::endl;
        struct Firing lidardata{};
        // Convert the msg to the raw packet type.
        //apollo::cyber::Time(timestamp).ToSecond();
        packet_timeStamp = msg->header().stamp_mil();
        packet_end_time = apollo::cyber::Time(static_cast<int>(msg->header().stamp_mil())).ToSecond();
        bool packetType = false;
        if (msg->data(1205) == 0x02) {
            return_mode = 2;
        }
        if(get_ms06_param && m_horizontal_point != 0 && msg->data(1204) == 192){
          //ms06  param
            double mirror_angle[4] = {1.5, 0.5, -0.5, -1.5};   //摆镜角度   //根据通道不同偏移角度不同
            for (int i = 0; i < 4; ++i) {
                cos_mirror_angle[i] = cos(DEG2RAD(mirror_angle[i]));
                sin_mirror_angle[i] = sin(DEG2RAD(mirror_angle[i]));
            }
            //m_offset = 10.82;
            g_fAngleAcc_V = 0.01;
            //g_fDistanceAcc = 0.1 * 0.04;
            get_ms06_param = false;
        }

        if (return_mode == 1) {
            if (packet_loss){
                current_packet_number_ = (msg->data(1192) << 8) + msg->data(1193);
                tmp_packet_number_ = current_packet_number_;

                if(current_packet_number_ - last_packet_number_ < 0){current_packet_number_ += 65536;}

                if (current_packet_number_ - last_packet_number_ > 1  && last_packet_number_ != -1) {
                    total_packet_loss_ += current_packet_number_ - last_packet_number_ - 1;
                    ADEBUG << "packet loss = " << total_packet_loss_;
                    INT64 loss_data;
                    loss_data.set_loss(total_packet_loss_);
                    packet_loss_pub->Write(loss_data);
                }
                last_packet_number_ = tmp_packet_number_;
            }
            double packet_interval_time =
                    (current_packet_time - last_packet_time) / (POINTS_PER_PACKET_SINGLE_ECHO / 8.0);
            for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET_SINGLE_ECHO; point_idx += 8) {
                
                // AERROR << std::endl << std::endl << " msg->data(point_idx) " << msg->data(point_idx) << 
                // " msg->data(point_idx + 1) " << msg->data(point_idx + 1) << " msg->data(point_idx + 2) " << 
                // msg->data(point_idx + 2)  << " msg->data(point_idx + 3) " << msg->data(point_idx + 3) << 
                // " msg->data(point_idx + 4) " << msg->data(point_idx + 4) << std::endl << std::endl;

                if ((msg->data(point_idx) == 0xff) && (msg->data(point_idx + 1) == 0xaa) &&
                    (msg->data(point_idx + 2) == 0xbb) && (msg->data(point_idx + 3) == 0xcc) &&
                    (msg->data(point_idx + 4) == 0xdd)) {
                    AERROR << std::endl << std::endl << std::endl << "packedType is true" << std::endl << std::endl << std::endl;
                    packetType = true;
                    frame_count++;
                } else {
                    // Compute the time of the point
                    double point_time;
                    if (last_packet_time > 1e-6) {
                        point_time = packet_end_time -
                                     packet_interval_time * ((POINTS_PER_PACKET_SINGLE_ECHO - point_idx) / 8 - 1);
                    } else {
                        point_time = current_packet_time;
                    }

                    memset(&lidardata, 0, sizeof(lidardata));
                    //水平角度
                    double fAngle_H = msg->data(point_idx + 1) + (msg->data(point_idx) << 8);
                    if (fAngle_H > 32767) {
                        fAngle_H = (fAngle_H - 65536);
                    }
                    lidardata.azimuth = fAngle_H * 0.01;
                    //垂直角度+通道号
                    int iTempAngle = msg->data(point_idx + 2);
                    int iChannelNumber = iTempAngle >> 6; //左移六位 通道号
                    int iSymmbol = (iTempAngle >> 5) & 0x01; //左移五位 符号位
                    double fAngle_V = 0.0;
                    if (1 == iSymmbol) // 符号位 0：正数 1：负数
                    {
                        int iAngle_V = msg->data(point_idx + 3) + (msg->data(point_idx + 2) << 8);

                        fAngle_V = iAngle_V | 0xc000;
                        if (fAngle_V > 32767) {
                            fAngle_V = (fAngle_V - 65536);
                        }
                    } else {
                        int iAngle_Hight = iTempAngle & 0x3f;
                        fAngle_V = msg->data(point_idx + 3) + (iAngle_Hight << 8);
                    }
                    lidardata.vertical_angle = fAngle_V * g_fAngleAcc_V;
                    if ((lidardata.azimuth < scan_start_angle) || (lidardata.azimuth > scan_end_angle)) continue;
                    lidardata.channel_number = iChannelNumber;
                    // AERROR << "804 before lidardata.distance" << std::endl;
                    lidardata.distance = ((msg->data(point_idx + 4) << 16) + (msg->data(point_idx + 5) << 8) +
                                        msg->data(point_idx + 6));
                    // AERROR << "807 !isPointInRange   " << lidardata.distance <<  std::endl;

                    // AERROR << "809 lidardata.vertical_angle   " << lidardata.vertical_angle <<  std::endl;
                    // AERROR << "810 lidardata.channel_number   " << lidardata.channel_number <<  std::endl;
                    // AERROR << "811 lidardata.distance   "       << lidardata.distance       <<  std::endl;
                    
                    if (!isPointInRange(lidardata.distance * g_fDistanceAcc)) continue;
                    // AERROR << "809 after isPointInRange" << std::endl;
                    lidardata.intensity = msg->data(point_idx + 7);
                    lidardata.time = point_time;
                    lidardata.azimuth = fAngle_H * 0.01;
                    convertCoordinate(lidardata);
                }
                //AERROR << "837 packetType before Write" << std::endl;
                if (packetType) {
                    if (is_add_frame_) {
                        AERROR << "840 is_add_frame_" << std::endl;
                        if (frame_count >= 2) {
                            {
                                std::unique_lock<std::mutex> lock(pc_mutex_);
                                point_cloud_xyzirt_pub_ = point_cloud_xyzirt_;
                            }
                            threadPool_->enqueue([&]() { publishPointCloudNew(); });
                        }
                        packetType = false;
                        point_cloud_xyzirt_ = point_cloud_xyzirt_bak_;
                        point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<VPoint>);
                    } else {
                        AERROR << "852 not is_add_frame_" << std::endl;
                        {
                            std::unique_lock<std::mutex> lock(pc_mutex_);
                            point_cloud_xyzirt_pub_ = point_cloud_xyzirt_;
                        }
                        threadPool_->enqueue([&]() { publishPointCloudNew(); });
                        packetType = false;
                        point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>);
                        point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<VPoint>);
                    }
                }
                //AERROR << "861 packetType after Write" << std::endl;
            }
        } else {
            if (packet_loss){
                current_packet_number_ = (msg->data(1188) * 1099511627776) + (msg->data(1189) * 4294967296) +
                                         (msg->data(1190) * 16777216) + (msg->data(1191) * 65536) +
                                         (msg->data(1192) * 256) + msg->data(1193);
                if (current_packet_number_ - last_packet_number_ > 1 && last_packet_number_ != -1) {
                    total_packet_loss_ += current_packet_number_ - last_packet_number_ - 1;
                    ADEBUG << "packet loss = " << total_packet_loss_;
                    INT64 loss_data;
                    loss_data.set_loss(total_packet_loss_);
                    packet_loss_pub->Write(loss_data);
                }
                last_packet_number_ = current_packet_number_;
            }
            double packet_interval_time =
                    (current_packet_time - last_packet_time) / (POINTS_PER_PACKET_DOUBLE_ECHO / 12.0);
            AERROR << "761 before for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET_DOUBLE_ECHO; point_idx += 12)" << std::endl;
            for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET_DOUBLE_ECHO; point_idx += 12) {
                if ((msg->data(point_idx) == 0xff) && (msg->data(point_idx + 1) == 0xaa) &&
                    (msg->data(point_idx + 2) == 0xbb) && (msg->data(point_idx + 3) == 0xcc) &&
                    (msg->data(point_idx + 4) == 0xdd)) {
                    packetType = true;
                    frame_count++;
                } else {
                    // Compute the time of the point
                    double point_time;
                    if (last_packet_time > 1e-6) {
                        point_time = packet_end_time -
                                     packet_interval_time * ((POINTS_PER_PACKET_DOUBLE_ECHO - point_idx) / 12 - 1);
                    } else {
                        point_time = current_packet_time;
                    }
                    memset(&lidardata, 0, sizeof(lidardata));
                    //水平角度
                    double fAngle_H = msg->data(point_idx + 1) + (msg->data(point_idx) << 8);
                    if (fAngle_H > 32767) {
                        fAngle_H = (fAngle_H - 65536);
                    }
                    lidardata.azimuth = fAngle_H * 0.01;

                    //垂直角度+通道号
                    int iTempAngle = msg->data(point_idx + 2);
                    int iChannelNumber = iTempAngle >> 6; //左移六位 通道号
                    int iSymmbol = (iTempAngle >> 5) & 0x01; //左移五位 符号位
                    double fAngle_V = 0.0;
                    if (1 == iSymmbol) // 符号位 0：正数 1：负数
                    {
                        int iAngle_V = msg->data(point_idx + 3) + (msg->data(point_idx + 2) << 8);

                        fAngle_V = iAngle_V | 0xc000;
                        if (fAngle_V > 32767) {
                            fAngle_V = (fAngle_V - 65536);
                        }
                    } else {
                        int iAngle_Hight = iTempAngle & 0x3f;
                        fAngle_V = msg->data(point_idx + 3) + (iAngle_Hight << 8);
                    }

                    lidardata.vertical_angle = fAngle_V * g_fAngleAcc_V;
                    if ((lidardata.azimuth < scan_start_angle) || (lidardata.azimuth > scan_end_angle)) continue;
                    lidardata.channel_number = iChannelNumber;
                    lidardata.distance = ((msg->data(point_idx + 4) << 16) + (msg->data(point_idx + 5) << 8) +
                                        msg->data(point_idx + 6));
                    if (!isPointInRange(lidardata.distance * g_fDistanceAcc)) continue;
                    lidardata.intensity = msg->data(point_idx + 7);
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);  // 第一个点

                    lidardata.distance = ((msg->data(point_idx + 8) << 16) + (msg->data(point_idx + 9) << 8) +
                                           msg->data(point_idx + 10));
                    if (!isPointInRange(lidardata.distance * g_fDistanceAcc)) continue;
                    lidardata.intensity = msg->data(point_idx + 11);
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);  // 第二个点
                }
                AERROR << "914 packetType before Write" << std::endl;
                if (packetType) {
                    if (is_add_frame_) {
                        if (frame_count >= 2) {
                            {
                                std::unique_lock<std::mutex> lock(pc_mutex_);
                                point_cloud_xyzirt_pub_ = point_cloud_xyzirt_;
                            }
                            threadPool_->enqueue([&]() { publishPointCloudNew(); });
                        }
                        packetType = false;
                        point_cloud_xyzirt_ = point_cloud_xyzirt_bak_;
                        point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<VPoint>);
                    } else {
                        {
                            std::unique_lock<std::mutex> lock(pc_mutex_);
                            point_cloud_xyzirt_pub_ = point_cloud_xyzirt_;
                        }
                        threadPool_->enqueue([&]() { publishPointCloudNew(); });
                        packetType = false;
                        point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>);
                        point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<VPoint>);
                    }
                }
            }
        }
        last_packet_time = current_packet_time;
        AERROR << "941 LslidarChDriver::lslidarChPacketProcess end line " << std::endl;
        return true;
    }

    void LslidarChDriver::setPacketHeader(unsigned char *config_data) {
        AERROR << "946 LslidarChDriver::setPacketHeader start line " << std::endl;
        config_data[0] = 0xAA;
        config_data[1] = 0x00;
        config_data[2] = 0xFF;
        config_data[3] = 0x11;
        config_data[4] = 0x22;
        config_data[5] = 0x22;
        config_data[6] = 0xAA;
        config_data[7] = 0xAA;
        AERROR << "955 LslidarChDriver::setPacketHeader end line " << std::endl;
    }

    // this one  will probably not require any proto setting changes. sockaddr_in is struct
    bool LslidarChDriver::sendPacketTolidar(unsigned char *config_data) const { 
        AERROR << "960 LslidarChDriver::sendPacketTolidar start line " << std::endl;
        int socketid;
        sockaddr_in addrSrv{};
        socketid = socket(2, 2, 0);
        addrSrv.sin_addr.s_addr = inet_addr(lidar_ip_string.c_str());
        addrSrv.sin_family = AF_INET;
        addrSrv.sin_port = htons(2368);
        sendto(socketid, (const char *) config_data, 1206, 0, (struct sockaddr *) &addrSrv, sizeof(addrSrv));
        AERROR << "968 LslidarChDriver::sendPacketTolidar end line " << std::endl;
        return true;
    }   
} // namespace lslidar_driver
}
}