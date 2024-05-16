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

#ifndef LSLIDAR_Ch_DRIVER_H
#define LSLIDAR_Ch_DRIVER_H

#include <unistd.h>
#include <cstdio>
#include <netinet/in.h>
#include <string>
#include <memory>
#include <thread>
#include <regex>
#include <pcl/conversions.h>
#include <pcl/register_point_struct.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <cmath>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/file.h>
#include <chrono>
#include "modules/drivers/lidar/proto/lslidar.pb.h"
#include "modules/drivers/lidar/proto/lslidar_config.pb.h"
#include "modules/drivers/lidar/common/driver_factory/driver_base.h"
#include "modules/drivers/lidar/proto/config.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include <deque>
#include <mutex>
#include <iostream>
#include <fstream>
#include "ThreadPool.h"

#include "modules/drivers/lidar/ls180s2_gazel/driver/input.h"
// #include "modules/drivers/lidar/ls180s2_gazel/driver/pcl_conversions.h"

namespace apollo {
namespace drivers {
namespace ls180s2_gazel {
    using apollo::cyber::Component;
    
    /** Special Defines for LSCh support **/
    const int POINTS_PER_PACKET_SINGLE_ECHO = 1192;       // modify
    const int POINTS_PER_PACKET_DOUBLE_ECHO = 1188;       // modify
    const int packet_size_proto_check = 1206;             // modify
    extern float g_fDistanceAcc;
    extern float m_offset;
    extern double sin30;
    extern double sin60;
    extern double cos30;
    // static double cos60 = cos(DEG2RAD(60));
    

    struct PointXYZIRT {
        PCL_ADD_POINT4D;
        PCL_ADD_INTENSITY;
        uint32_t ring;
        double timestamp;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
    } EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment

    struct Firing {
        double vertical_angle;
//        int vertical_line;
        double azimuth;
        double distance;
        float intensity;
        double time;
        int channel_number;
    };

    class LslidarChDriver : public apollo::drivers::lidar::LidarDriver {
    public:
        //LslidarChDriver();

        LslidarChDriver(const std::shared_ptr<::apollo::cyber::Node>& node,
                    const ::apollo::drivers::lidar::config& conf);
            // : node_(node), conf_(conf) {}
        LslidarChDriver(const std::shared_ptr<::apollo::cyber::Node>& node,
                        const ::apollo::drivers::ls180s2_gazel::Config& conf);
        //     : node_(node), conf_(conf) {}

        ~LslidarChDriver();

        bool Init() override;

        bool lslidarChPacketProcess(const std::shared_ptr<LslidarMsgRawPacket> &msg);
                    //const LslidarMsgRawPacket &msg

        bool polling();

        void difopPoll();

        void initTimeStamp();

        bool isPointInRange(const double &distance) const {
            return (distance >= min_range && distance <= max_range);
        }

        int convertCoordinate(const struct Firing &lidardata);

        // Publish data
        void publishPointCloudNew();

        static void setPacketHeader(unsigned char *config_data);

        bool sendPacketTolidar(unsigned char *config_data) const;


        typedef boost::shared_ptr<LslidarChDriver> LslidarChDriverPtr;
        typedef boost::shared_ptr<const LslidarChDriver> LslidarChDriverConstPtr;

        bool loadParameters();

        bool createCyberIO();

        bool frameRate(LslidarSrvFrameRate &req,
                        LslidarSrvResult &res);

        bool setDataIp(LslidarSrvDataIp &req,
                         LslidarSrvResult &res);

        bool setDestinationIp(LslidarSrvDestinationIp &req,
                        LslidarSrvResult &res);

        bool setDataPort(LslidarSrvDataPort &req,
                          LslidarSrvResult &res);

        bool setDevPort(LslidarSrvDevPort &req,
                         LslidarSrvResult &res);

        // template<typename T>
        // void convertPCLtoROSMsg(const pcl::PointCloud<T> &pcl_cloud, PointCloud2 &cloud, LslidarHeader &headerPointCloud2)
        // {
        //     pcl::PCLPointCloud2 pcl_pc2;
        //     pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);

        //     headerPointCloud2.set_stamp_sec(static_cast<uint32_t>(pcl_cloud.header.stamp));
        //     headerPointCloud2.set_stamp_mil(static_cast<uint32_t>(pcl_cloud.header.stamp));
        //     headerPointCloud2.set_frame_id(pcl_cloud.header.frame_id);
        //     cloud.set_height(pcl_cloud.height);
        //     cloud.set_width(pcl_cloud.width);
        //     cloud.set_is_bigendian(false);
        //     cloud.set_is_dense(pcl_cloud.is_dense);
        //     cloud.set_point_step(sizeof(T));
        //     cloud.set_row_step(cloud.point_step() * cloud.width());
        //     for(size_t i = 0; i < pcl_pc2.data.size(); ++i){
        //         cloud.set_data(i, static_cast<uint32_t>(pcl_pc2.data[i]));
        //     }
        //     //memcpy(&cloud.data[0], &pcl_pc2.data[0], pcl_pc2.data.size());
        // }
        
      template<typename T>
      void converter(const pcl::PointCloud<T> &pcl_cloud, PointCloud2 &cloud_proto)
      {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);

        cloud_proto.mutable_header()->set_stamp_mil(pcl_pc2.header.stamp);
        cloud_proto.mutable_header()->set_stamp_sec(pcl_pc2.header.stamp / 1000);
        cloud_proto.mutable_header()->set_frame_id(pcl_pc2.header.frame_id);
        cloud_proto.set_height(pcl_pc2.height);
        cloud_proto.set_width(pcl_pc2.width);

        for (const auto &field : pcl_pc2.fields)
        {
            auto* field_proto = cloud_proto.add_fields();
            field_proto->set_name(field.name);
            field_proto->set_offset(field.offset);
            field_proto->set_datatype(field.datatype);
            field_proto->set_count(field.count);
        }

        cloud_proto.set_is_bigendian(pcl_pc2.is_bigendian);
        cloud_proto.set_point_step(pcl_pc2.point_step);
        cloud_proto.set_row_step(pcl_pc2.row_step);
        cloud_proto.set_is_dense(pcl_pc2.is_dense);


        for (size_t i = 0; i < pcl_pc2.data.size(); ++i) {
            //AERROR << std::endl << std::endl << "data: " << static_cast<uint32_t>pcl_pc2.data[i] << std::endl << std::endl;
            cloud_proto.add_data(pcl_pc2.data[i]);
        }
      }

        bool convert_PointCloud(std::shared_ptr<apollo::drivers::PointCloud> proto,
                        PointCloud2 &rawdata) {
            auto header = proto->mutable_header();
            header->set_timestamp_sec(rawdata.header().stamp_sec());
            header->set_frame_id(rawdata.header().frame_id());
            header->set_sequence_num(rawdata.header().seq());
            proto->set_frame_id(rawdata.header().frame_id());
            proto->set_measurement_time(rawdata.header().stamp_sec());
            proto->set_width(rawdata.width());
            proto->set_height(rawdata.height());

            int x_offset = -1;
            int y_offset = -1;
            int z_offset = -1;
            int stamp_offset = -1;
            int intensity_offset = -1;
            for (const auto &field : rawdata.fields()) {
                if (field.name() == "x") {
                x_offset = field.offset();
                } else if (field.name() == "y") {
                y_offset = field.offset();
                } else if (field.name() == "z") {
                z_offset = field.offset();
                } else if (field.name() == "timestamp") {
                stamp_offset = field.offset();
                } else if (field.name() == "intensity") {
                intensity_offset = field.offset();
                }
            }

            if (x_offset == -1 || y_offset == -1 || z_offset == -1 ||
                stamp_offset == -1 || intensity_offset == -1) {
                std::cerr << "Field not contains x, y, z, timestamp, instensity"
                        << std::endl;
                return false;
            }

            int total = rawdata.width() * rawdata.height();
            auto data = rawdata.data();
            for (int i = 0; i < total; ++i) {
                auto cyber_point = proto->add_point();
                int offset = i * rawdata.point_step();
                cyber_point->set_x(*reinterpret_cast<float *>(&data[offset + x_offset]));
                cyber_point->set_y(*reinterpret_cast<float *>(&data[offset + y_offset]));
                cyber_point->set_z(*reinterpret_cast<float *>(&data[offset + z_offset]));
                cyber_point->set_intensity(
                    *reinterpret_cast<uint8_t *>(&data[offset + intensity_offset]));
                cyber_point->set_timestamp(static_cast<std::uint64_t>(
                    *reinterpret_cast<double *>(&data[offset + stamp_offset]) * 1e9));
            }

            return true;
        }

        // bool convert_PointCloud(std::shared_ptr<apollo::drivers::PointCloud> proto,
        //                 std::shared_ptr<PointCloud2> &rawdata) {
        //     auto header = proto->mutable_header();
        //     header->set_timestamp_sec(rawdata->header().stamp_sec());
        //     header->set_frame_id(rawdata->header().frame_id());
        //     header->set_sequence_num(rawdata->header().seq());
        //     proto->set_frame_id(rawdata->header().frame_id());
        //     proto->set_measurement_time(rawdata->header().stamp_sec());
        //     proto->set_width(rawdata->width());
        //     proto->set_height(rawdata->height());

        //     int x_offset = -1;
        //     int y_offset = -1;
        //     int z_offset = -1;
        //     int stamp_offset = -1;
        //     int intensity_offset = -1;
        //     for (const auto &field : rawdata->fields()) {
        //         if (field.name() == "x") {
        //         x_offset = field.offset();
        //         } else if (field.name() == "y") {
        //         y_offset = field.offset();
        //         } else if (field.name() == "z") {
        //         z_offset = field.offset();
        //         } else if (field.name() == "timestamp") {
        //         stamp_offset = field.offset();
        //         } else if (field.name() == "intensity") {
        //         intensity_offset = field.offset();
        //         }
        //     }

        //     if (x_offset == -1 || y_offset == -1 || z_offset == -1 ||
        //         stamp_offset == -1 || intensity_offset == -1) {
        //         std::cerr << "Field not contains x, y, z, timestamp, instensity"
        //                 << std::endl;
        //         return false;
        //     }

        //     int total = rawdata->width() * rawdata->height();
        //     auto data = rawdata->data();
        //     for (int i = 0; i < total; ++i) {
        //         auto cyber_point = proto->add_point();
        //         int offset = i * rawdata->point_step();
        //         cyber_point->set_x(*reinterpret_cast<float *>(&data[offset + x_offset]));
        //         cyber_point->set_y(*reinterpret_cast<float *>(&data[offset + y_offset]));
        //         cyber_point->set_z(*reinterpret_cast<float *>(&data[offset + z_offset]));
        //         cyber_point->set_intensity(
        //             *reinterpret_cast<uint8_t *>(&data[offset + intensity_offset]));
        //         cyber_point->set_timestamp(static_cast<std::uint64_t>(
        //             *reinterpret_cast<double *>(&data[offset + stamp_offset]) * 1e9));
        //     }

        //     return true;
        // }

        // ROS related variables
        std::shared_ptr<::apollo::cyber::Node> node_ = nullptr;
        Config conf_;
        // apollo::cyber::Node pnh;

        in_addr lidar_ip{};
        int socket_id;
        bool add_multicast{};
        double prism_angle[4]{};

        // Configuration parameters
        double min_range;
        double max_range;
        double packet_rate;

        double/*apollo::cyber::Time*/ packet_timeStamp;  // ros::Time for header 
        double packet_end_time;
        double current_packet_time;
        double last_packet_time;

        bool use_time_service;
        int return_mode;
        double g_fAngleAcc_V;
        bool packet_loss;
        bool is_add_frame_;
        bool get_ms06_param;
        bool is_get_difop_;
        int64_t last_packet_number_;
        int64_t current_packet_number_;
        int64_t total_packet_loss_;
        int frame_count;
        std::unique_ptr <ThreadPool> threadPool_;

        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_bak_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_pub_;

        //socket Parameters
        int msop_udp_port{};
        int difop_udp_port{};

        std::shared_ptr<Input> msop_input_;
        std::shared_ptr<Input> difop_input_;

        // Converter convtor_
        std::shared_ptr<std::thread> difop_thread_;

        // Ethernet relate variables
        std::string lidar_ip_string;
        std::string group_ip_string;
        std::string frame_id;
        std::string dump_file;
       
        uint64_t pointcloudTimeStamp{};
        unsigned char packetTimeStamp[10]{};
        unsigned char difop_data[1206]{};
        apollo::cyber::Time timeStamp;
        
        int scan_start_angle{};
        int scan_end_angle{};
        std::mutex pc_mutex_;
        
        std::string pointcloud_topic;
        std::shared_ptr<apollo::cyber::Writer<PointCloud2>> point_cloud_pub;      
        std::shared_ptr<apollo::cyber::Writer<PointCloud>> point_cloud_final;                   
        std::shared_ptr<apollo::cyber::Writer<INT64>> packet_loss_pub;                      
        std::shared_ptr<apollo::cyber::Service<LslidarSrvFrameRate, LslidarSrvResult>> frame_rate_service_;              
        std::shared_ptr<apollo::cyber::Service<LslidarSrvDataIp, LslidarSrvResult>> data_ip_service_;                   
        std::shared_ptr<apollo::cyber::Service<LslidarSrvDestinationIp, LslidarSrvResult>> destination_ip_service_;     
        std::shared_ptr<apollo::cyber::Service<LslidarSrvDataPort, LslidarSrvResult>> data_port_service_;               
        std::shared_ptr<apollo::cyber::Service<LslidarSrvDevPort, LslidarSrvResult>> dev_port_service_;                 
        int64_t tmp_packet_number_;
        int m_horizontal_point = -1;
        //double m_offset;

        double cos_table[36000]{};
        double sin_table[36000]{};
        double cos_mirror_angle[4]{};
        double sin_mirror_angle[4]{};
        // ---
        //apollo::drivers::ls180s2_gazel::Config config;
        // ---
    };
    typedef PointXYZIRT VPoint;
    typedef LslidarChDriver::LslidarChDriverPtr LslidarChDriverPtr;
    typedef LslidarChDriver::LslidarChDriverConstPtr LslidarChDriverConstPtr;

    //CYBER_REGISTER_COMPONENT(LslidarChDriver)

} // namespace lslidar_driver
}
}

POINT_CLOUD_REGISTER_POINT_STRUCT(apollo::drivers::ls180s2_gazel::PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)
                                          (float, intensity, intensity)
                                          (std::uint32_t, ring, ring)
                                          (double, timestamp, timestamp)
)

#endif // _LSLIDAR_Ch_DRIVER_H_