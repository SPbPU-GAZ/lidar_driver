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

#include <cmath>
#include <memory>
#include <string>

#include "modules/drivers/lidar/ls180s2_gazel/parser/parser.h"

namespace apollo {
namespace drivers {
namespace ls180s2_gazel {

using apollo::drivers::PointCloud;

Parser::Parser(const std::shared_ptr<::apollo::cyber::Node>& node,
               const Config& conf)
    : node_(node), conf_(conf) {
  tz_second_ = conf_.time_zone() * 3600;
  start_angle_ = static_cast<int>(conf_.start_angle() * 100);
}

Parser::~Parser() { Stop(); }

bool Parser::Init() {
  if (inited_) {
    return true;
  }

  // init calibration
  if (!conf_.is_online_calibration() && conf_.calibration_file() != "") {
    if (!LoadCalibration(conf_.calibration_file().c_str())) {
      AERROR << "load local calibration file[" << conf_.calibration_file()
             << "] error";
      return false;
    }
    is_calibration_ = true;
  } else {
    online_calibration_thread_ =
        std::thread(&Parser::LoadCalibrationThread, this);
  }

  // init writer
  raw_pointcloud_writer_ =
      node_->CreateWriter<PointCloud>(conf_.pointcloud_channel());

  if (raw_pointcloud_writer_ == nullptr) {
    AERROR << "create writer:" << conf_.pointcloud_channel()
           << " error, check cyber is init?";
    return false;
  }

  raw_pointcloud_pool_.resize(pool_size_);
  for (int i = 0; i < pool_size_; i++) {
    raw_pointcloud_pool_[i] = std::make_shared<PointCloud>();
    if (raw_pointcloud_pool_[i] == nullptr) {
      AERROR << "make shared PointCloud error,oom";
      return false;
    }
    raw_pointcloud_pool_[i]->mutable_point()->Reserve(70000);
  }

  ResetRawPointCloud();
  inited_ = true;
  return true;
}

void Parser::ResetRawPointCloud() {
  raw_pointcloud_out_ = raw_pointcloud_pool_.at(pool_index_);
  AINFO << "pool index:" << pool_index_;
  raw_pointcloud_out_->Clear();
  raw_pointcloud_out_->mutable_point()->Reserve(70000);
  pool_index_ = (pool_index_ + 1) % pool_size_;
}

bool Parser::Parse(std::shared_ptr<PointCloud2>& scan, std::shared_ptr<PointXYZIRT>& point_cloud_xyzirt_pub_) {
  //ResetRawPointCloud();
  ParseRawPacket(*scan, *point_cloud_xyzirt_pub_);
  PublishRawPointCloud(scan->header().sequence_num());
  return true;
}

bool Parser::CheckIsEnd(bool is_end) {
  if (packet_nums_ >= max_packets_) {
    AWARN << "over max packets, packets:" << packet_nums_
          << ", max packets:" << max_packets_;
    return true;
  }
  if (is_end && packet_nums_ < min_packets_) {
    AWARN << "receive too less packets:" << packet_nums_ << ", not end"
          << ", min packets:" << min_packets_;
    return false;
  }
  return is_end;
}

void Parser::PublishRawPointCloud(int seq, std::shared_ptr<PointCloud2>& scan) {
  int size = raw_pointcloud_out_->point_size();
  if (size == 0) {
    AWARN << "All points size is NAN! Please check lslidar:" << conf_.model();
    return;
  }

  raw_pointcloud_out_->mutable_header()->set_sequence_num(seq_index_++);
  if (seq > 0) {
    raw_pointcloud_out_->mutable_header()->set_sequence_num(seq);
  }
  raw_pointcloud_out_->mutable_header()->set_frame_id("lslidar");
  raw_pointcloud_out_->mutable_header()->set_timestamp_sec(
      cyber::Time().Now().ToSecond());
  raw_pointcloud_out_->set_height(scan.height());
  raw_pointcloud_out_->set_width(scan.width());
  const auto timestamp =
      raw_pointcloud_out_->point(static_cast<int>(size) - 1).timestamp();
  raw_pointcloud_out_->set_measurement_time(static_cast<double>(timestamp) /
                                            1e9);
  raw_pointcloud_out_->mutable_header()->set_lidar_timestamp(timestamp);
  raw_pointcloud_writer_->Write(raw_pointcloud_out_);
}

void Parser::CheckPktTime(double time_sec) {
  double now = apollo::cyber::Time().Now().ToSecond();
  double diff = std::abs(now - time_sec);
  if (diff > 0.1) {
    // AWARN << conf_.frame_id() << " time too big, diff:" << diff
    //       << "host time:" << now << ";lidar time:" << time_sec;
  }
}

void Parser::CalcPointXYZIT(std::shared_ptr<PointCloud2>& scan, std::shared_ptr<PointCloud>& raw_pointcloud_out_) {
  // Hesai40PBlock *block = &pkt->blocks[blockid];
  // double unix_second = static_cast<double>(mktime(&pkt->t) + tz_second_);
  // double timestamp = unix_second + (static_cast<double>(pkt->usec)) / 1000000.0;
  // CheckPktTime(timestamp);

    //  double xyDistance =
    //     unit.distance * cosf(degreeToRadian(elev_angle_map_[i]));
    // float x = static_cast<float>(
    //     xyDistance *
    //     sinf(degreeToRadian(horizatal_azimuth_offset_map_[i] +
    //                         (static_cast<double>(block->azimuth)) / 100.0)));
    // float y = static_cast<float>(
    //     xyDistance *
    //     cosf(degreeToRadian(horizatal_azimuth_offset_map_[i] +
    //                         (static_cast<double>(block->azimuth)) / 100.0)));
    // float z = static_cast<float>(unit.distance *
    //                              sinf(degreeToRadian(elev_angle_map_[i])));

    PointXYZIT *new_point = raw_pointcloud_out_->add_point();
    new_point->set_x(scan.x);
    new_point->set_y(scan.y);
    new_point->set_z(scan.z);
    new_point->set_intensity(scan.intensity);
    // ?
    new_point->set_timestamp(scan.timestap);
  }
}

}  // namespace ls180s2_gazel
}  // namespace drivers
}  // namespace apollo
