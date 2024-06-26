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

#include "input.h"

namespace apollo {
namespace drivers {
namespace ls180s2_gazel {
    volatile sig_atomic_t flag = 1;
    uint32_t PACKET_SIZE = 1206; //sizeof(LslidarMsgRawPacket->get_data());
////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */

    static void my_handler(int sig) {
        flag = 0;
    }
    
    Input::Input(const std::shared_ptr<::apollo::cyber::Node>& private_nh, uint32_t port) : private_nh_(private_nh), port_(port) {
        AERROR << "41 Input::Input start line" << std::endl; 

        npkt_update_flag_ = false;
        cur_rpm_ = 0;
        return_mode_ = 1;

        signal(SIGINT, my_handler);

        devip_str_ = conf_.devip_str();
        add_multicast = conf_.add_multicast();
        group_ip = conf_.group_ip();
        AERROR << "52 Input::Input end line" << std::endl; 
    }

    

    int Input::getRpm(void) {
        return cur_rpm_;
    }

    int Input::getReturnMode(void) {
        return return_mode_;
    }

    bool Input::getUpdateFlag(void) {
        return npkt_update_flag_;
    }

    void Input::clearUpdateFlag(void) {
        npkt_update_flag_ = false;
    }
////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
*/
    InputSocket::InputSocket(const std::shared_ptr<::apollo::cyber::Node>& private_nh, uint32_t port) : Input(private_nh, port) {
        AERROR << "82 InputSocket::InputSocket start line" << std::endl; 

        sockfd_ = -1;

        if (!devip_str_.empty()) { // if string IP not empty convert it and write in it devip_
            inet_aton(devip_str_.c_str(), &devip_);
        }

        AERROR << "Opening UDP socket port: " << port;
        sockfd_ = socket(PF_INET, SOCK_DGRAM, 0); // Returns a file descriptor for the new socket, or -1 for errors
        if (sockfd_ == -1) {
            AERROR << "socket" << std::endl;;  // TODO: ROS_ERROR errno
            return;
        }
        int opt = 1;
        if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0){ // socket settings
            AERROR << "setsockopt error!" << std::endl;
            return;
        }
        sockaddr_in my_addr{};                   // my address information (sockaddr_in -> internet addres socket, sockaddr -> generic adress socket)
        memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros (addres, value, len of memory to fill with value)
        my_addr.sin_family = AF_INET;          // host byte order (family addres, here AF_INET for IPv4)
        my_addr.sin_port = htons(port);        // port in network byte order
        my_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // automatically fill in my IP (check or bind this socket for all net cards in system)

        if (bind(sockfd_, (sockaddr *) &my_addr, sizeof(sockaddr)) == -1) {
            AERROR << "bind" << std::endl;  // TODO: ROS_ERROR errno
            return;
        }
        if (add_multicast) {
            struct ip_mreq group{};
            //char *group_ip_ = (char *) group_ip.c_str();
            group.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
            //group.imr_interface.s_addr =  INADDR_ANY;
            group.imr_interface.s_addr = htonl(INADDR_ANY);
            //group.imr_interface.s_addr = inet_addr("192.168.1.102");

            if (setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &group, sizeof(group)) < 0) {
                AERROR << "Adding multicast group error " << std::endl;
                close(sockfd_);
                exit(1);
            } else
                printf("Adding multicast group...OK.\n");
        }

        // set the file status flags of a socket file descriptor (add non blocking read and write and async notifications then data available or events occur)
        if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) { 
            AERROR << "non-block" << std::endl;
            return;
        }
        AERROR << "132 InputSocket::InputSocket end line" << std::endl; 
    }

/** @brief destructor */
    InputSocket::~InputSocket(void) {
        (void) close(sockfd_);
    }

    int InputSocket::getPacket(LslidarRecvData &packet) {
        AERROR << "141 InputSocket::getPacket start line" << std::endl; 
        struct pollfd fds[1];
        fds[0].fd = sockfd_;
        fds[0].events = POLLIN;
        static const int POLL_TIMEOUT = 3000; // one second (in msec)
        // AERROR << "143 inside InputSocket::getPacket line" << std::endl;  

        sockaddr_in sender_address{};
        socklen_t sender_address_len = sizeof(sender_address);
        //while (true) 
        while (flag == 1) {
            // poll() until input available
            do {
                int retval = poll(fds, 1, POLL_TIMEOUT); 
                if (retval == 0)            // poll() timeout?
                {
                    AERROR << "lslidar poll() timeout, port:" << port_;
                    return 1;
                }  
            } while ((fds[0].revents & POLLIN) == 0);

            // Receive packets that should now be available from the
            // socket using a blocking read.
        AERROR << "169 InputSocket::getPacket after poll line" << std::endl;  
            ssize_t nbytes = recvfrom(sockfd_, &packet.data[0], apollo::drivers::ls180s2_gazel::PACKET_SIZE, 0,
                                      (sockaddr *) &sender_address, &sender_address_len);

            if ((size_t) nbytes == apollo::drivers::ls180s2_gazel::PACKET_SIZE) {
                // read successful,
                // if packet is not from the lidar scanner we selected by IP,
                // continue otherwise we are done
                if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr) {
                    AERROR << "lidar ip parameter error, please reset lidar ip in launch file";
                    continue;
                } else {
                    break; //done
                }                  
            }
        }
        if (flag == 0) {
            abort();
        }
        AERROR << "181 InputSocket::getPacket end line" << std::endl;  
        return 0;
    }


    InputPCAP::InputPCAP(const std::shared_ptr<::apollo::cyber::Node>& private_nh, uint32_t port, 
                        double packet_rate, std::string filename,
                        bool read_once, bool read_fast, double repeat_delay) : Input(private_nh, port),
                                                                                packet_rate_(packet_rate),
                                                                                filename_(filename) {
        AERROR << "203 InputPCAP::InputPCAP start line" << std::endl; 
        pcap_ = nullptr;
        empty_ = true;
        // private_nh.param("read_once", read_once_, false);
        // private_nh.param("read_fast", read_fast_, false);
        // private_nh.param("repeat_delay", repeat_delay_, 0.0);
        // ----------------------------------------------
        // parameter_server.SetParameter("read_once", read_once_);
        // apollo::cyber::Parameter("read_fast", read_fast_, false);
        // apollo::cyber::Parameter("repeat_delay", repeat_delay_, 0.0);

        read_once_ = read_once;
        read_fast_ = read_fast;
        repeat_delay_ = repeat_delay;
        // ----------------------------------------------
        if (read_once_)
            AINFO << "Read input file only once.";
        if (read_fast_)
            AINFO << "Read input file as  quickly as possible.";
        if (repeat_delay_ > 0.0)
            AINFO << "Delay %.3f seconds before repeating input file.", repeat_delay_;
        AINFO << "Opening PCAP file: " << filename_;
        if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == nullptr) {
            AFATAL << "Error opening lslidar socket dump file.";
            return;
        }
        std::stringstream filter;
        if (devip_str_ != "") {
            filter << "src host " << devip_str_ << "&&";
        }
        filter << "udp dst port " << port;
        pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
        AERROR << "235 InputPCAP::InputPCAP end line" << std::endl; 
    }

    InputPCAP::~InputPCAP() {
        pcap_close(pcap_);
    }

    int InputPCAP::getPacket(LslidarRecvData &pkt) {
        AERROR << "243 InputPCAP::getPacket start line" << std::endl; 
        struct pcap_pkthdr *header;
        const u_char *pkt_data;
        static int count_frame= 0;
        // ------------------
        // auto y = pkt.data(0);
        // ------------------
        while (flag == 1) {
            int res;
            if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0) {
//                ROS_INFO("read pcap file count = %d",count_frame);
                count_frame++;
                if (!devip_str_.empty() && (0 == pcap_offline_filter(&pcap_packet_filter_, header, pkt_data))) {
                    continue;
                }

                if (read_fast_ == false) {
                    packet_rate_.Sleep();
                }

                mempcpy(&pkt.data[0], pkt_data + 42, PACKET_SIZE);
                if (pkt.data[0] == 0x00 && pkt.data[1] == 0xFF && pkt.data[2] == 0x00 &&
                    pkt.data[3] == 0x5A) {
                    int rpm = (pkt.data[8] << 8) | pkt_data[9];
                    int mode = 1;
                    if ((pkt.data[45] == 0x08 && pkt.data[46] == 0x02 && pkt.data[47] >= 0x09) ||
                        (pkt.data[45] > 0x08) ||
                        (pkt_data[45] == 0x08 && pkt.data[46] > 0x02)) {
                        if (pkt.data[300] != 0x01 && pkt.data[300] != 0x02) {
                            mode = 0;
                        }

                    }
                    if (cur_rpm_ != rpm || return_mode_ != mode) {
                        cur_rpm_ = rpm;
                        return_mode_ = mode;
                        npkt_update_flag_ = true;
                    }
                }
                pkt.stamp_sec = apollo::cyber::Time().Now().ToNanosecond();
                pkt.stamp_mil = apollo::cyber::Time().Now().ToSecond();
                //pkt->set_stamp(apollo::cyber::Time().Now().ToNanosecond());
                empty_ = false;
                return 0;
            }
            if (empty_) {
                AWARN << "Error " << res <<" reading lslidar packet: " << pcap_geterr(pcap_);
            }
            if (read_once_) {
                AINFO <<"end of file reached-- done reading.";
            }
            if (repeat_delay_ > 0.0) {
                AINFO << "end of file reached -- delaying" << repeat_delay_ << "seconds." ;
                usleep(rint(repeat_delay_ * 1000000.0));
            }
            ADEBUG << "replaying lslidar dump file";
            pcap_close(pcap_);
            pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
            empty_ = true;
//            ROS_INFO("replaying lslidar dump file");
            count_frame = 0;
        }
        AERROR << "304 inside InputPCAP::getPacket almost end line" << std::endl; 
        if (flag == 0) {
            abort();
        }
        return -1;
    }


} //lslidar_ch_driver 
} //drivers
} //apollo

// done