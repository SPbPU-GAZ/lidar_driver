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
 ****************************************************************************
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#ifndef __LSLIDAR_INPUT_H_
#define __LSLIDAR_INPUT_H_

#include <unistd.h>
#include <cstdio>
#include <netinet/in.h>
//#include <ros/ros.h>
// #include "modules/drivers/lidar/ls180s2_gazel/proto/lslidar.pb.h"
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/file.h>
#include <csignal>
#include <pcap.h>
#include <iostream>
// #include <sensor_msgs/TimeReference.h>

// ----------------
#include "cyber/cyber.h"
#include "cyber/parameter/parameter_client.h"
#include "cyber/parameter/parameter_server.h"
#include "modules/drivers/lidar/ls180s2_gazel/proto/lslidar.pb.h"
#include "modules/drivers/lidar/ls180s2_gazel/proto/lslidar_config.pb.h"
// ----------------

namespace apollo {
namespace drivers {
namespace ls180s2_gazel {
    extern uint32_t PACKET_SIZE;
    static uint32_t MSOP_DATA_PORT_NUMBER = 2368;   // lslidar default data port on PC
    //static uint32_t DIFOP_DATA_PORT_NUMBER = 2369;  // lslidar default difop data port on PC
/**
 *  从在线的网络数据或离线的网络抓包数据（pcap文件）中提取出lidar的原始数据，即packet数据包 ->
 *  Extract lidar's original data
 * @brief The Input class,
     *
     * @param private_nh  一个NodeHandled,用于通过节点传递参数 -> A NodeHandled for passing parameters through nodes
     * @param port
     * @returns 0 if successful,
     *          -1 if end of file
     *          >0 if incomplete packet (is this possible?)
 */
    class Input {
    public:
        Input(const std::shared_ptr<::apollo::cyber::Node>& private_nh, uint32_t port);

        virtual ~Input() {
        }

        virtual int getPacket(LslidarMsgRawPacket &packet) = 0;

        int getRpm(void);

        int getReturnMode(void);

        bool getUpdateFlag(void);

        void clearUpdateFlag(void);

    protected:
        // apollo::cyber::ParameterServer parameter_server_;
        // apollo::cyber::ParameterClient parameter_client_;
        const std::shared_ptr<::apollo::cyber::Node>& private_nh_;
        uint32_t port_;
        std::string devip_str_; /// apollo::cyber::Parameter // device ip as string
        int cur_rpm_;
        int return_mode_;
        bool npkt_update_flag_;
        bool add_multicast; /// apollo::cyber::Parameter
        std::string group_ip; /// apollo::cyber::Parameter
        // -------
        //apollo::drivers::ls180s2_gazel::Config config;
        // -------
    };

/** @brief Live lslidar input from socket. */
    class InputSocket : public Input {
    public:
        InputSocket(const std::shared_ptr<::apollo::cyber::Node>& private_nh, uint32_t port = MSOP_DATA_PORT_NUMBER);

        virtual ~InputSocket();

        virtual int getPacket(LslidarMsgRawPacket &packet);

    private:
        int sockfd_; // socket field? || file descriptor or -1
        in_addr devip_; // IPv4 structure
        //struct ip_mreq group;
    };

    class InputPCAP : public Input {
    public:
        InputPCAP(const std::shared_ptr<::apollo::cyber::Node>& private_nh, uint32_t port = MSOP_DATA_PORT_NUMBER, double packet_rate = 0.0, \
    std::string filename = "", bool read_once = false, bool read_fast = false, double repeat_delay = 0.0);

        virtual ~InputPCAP();

        virtual int getPacket(LslidarMsgRawPacket &packet);

    private:
        apollo::cyber::Rate packet_rate_; // ros::Rate
        std::string filename_;
        pcap_t *pcap_; // type to capture network packets
        bpf_program pcap_packet_filter_;
        char errbuf_[PCAP_ERRBUF_SIZE];
        bool empty_;
        bool read_once_;
        bool read_fast_;
        double repeat_delay_;
    };

} //lslidar_ch_driver 
} //drivers
} //apollo
#endif  // __LSLIDAR_INPUT_H
