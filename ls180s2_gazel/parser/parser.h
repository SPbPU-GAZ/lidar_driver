/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#ifndef LIDAR_HESAI_SRC_PARSE_H
#define LIDAR_HESAI_SRC_PARSE_H

#include <deque>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "modules/drivers/lidar/proto/lslidar_config.pb.h"
#include "modules/drivers/lidar/proto/lslidar.pb.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/drivers/lidar/ls180s2_gazel/driver/driver.h"

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace ls180s2_gazel {

const double PI = 3.14159265358979323846;

inline double degreeToRadian(double degree) { return degree * PI / 180; }

class Parser {
 public:
  Parser(/*const std::shared_ptr<::apollo::cyber::Node>& node,
         const Config& conf*/);
  virtual ~Parser();
  bool Parse(std::shared_ptr<PointCloud2>& scan_old_poincloud, std::shared_ptr<PointCloud>& raw_pointcloud_out_, std::shared_ptr<pcl::PointCloud<PointXYZIRT>>& scan);
  bool Init();

 private:
  std::thread online_calibration_thread_;
//   bool CheckIsEnd(bool is_end);
  void PublishRawPointCloud(int seq, std::shared_ptr<PointCloud2>& scan);

  bool inited_ = false;
  void Stop();
  std::atomic<bool> running_ = {true};

 protected:
  // void ParseRawPacket(const uint8_t* buf, const int len, bool* is_end) = 0;
  void CheckPktTime(double time_sec);
  void ResetRawPointCloud();
  //void CalcPointXYZIT(std::shared_ptr<PointCloud2>& scan, std::shared_ptr<PointCloud>& raw_pointcloud_out_)
  void CalcPointXYZIT(std::shared_ptr<pcl::PointCloud<PointXYZIRT>>& scan, std::shared_ptr<PointCloud>& raw_pointcloud_out_);

  bool is_calibration_ = false;
  std::shared_ptr<::apollo::cyber::Node> node_;
  Config conf_;
  std::shared_ptr<::apollo::cyber::Writer<PointCloud>> raw_pointcloud_writer_;
  int pool_size_ = 8;
  int pool_index_ = 0;
  uint64_t raw_last_time_ = 0;
  int seq_index_ = 0;
  std::deque<std::shared_ptr<PointCloud>> raw_pointcloud_pool_;
  std::shared_ptr<PointCloud> raw_pointcloud_out_ = nullptr;

  int last_azimuth_ = 0;
  int tz_second_ = 0;
  int start_angle_ = 0;
};

}  // namespace ls180s2_gazel
}  // namespace drivers
}  // namespace apollo

#endif
