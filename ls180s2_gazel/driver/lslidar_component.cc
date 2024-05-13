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
#include "modules/drivers/lidar/ls180s2_gazel/driver/lslidar_component.h"

namespace apollo {
namespace drivers {
namespace ls180s2_gazel {


bool LslidarComponent::Init() {
  AERROR << "24 Init start component" << std::endl;

  if (!GetProtoConfig(&lslidar_conf_)) {

    AERROR << " Init component load config error, file:" << config_file_path_;
    return false;
  }

  // signal(SIGINT, my_handler);

  AINFO << "conf:" << lslidar_conf_.DebugString();

  driver_.reset(new LslidarChDriver(node_, lslidar_conf_));

  if (!driver_->Init()) {

    AERROR << "Init component driver init error" << std::endl;
    return false;
  }

  while (apollo::cyber::OK())
  {
    driver_->polling();
    AERROR << "55 Init component polling already in process" << std::endl; 
  }
  
  return true;
}

}  // namespace ls180s2_gazel
}  // namespace drivers
}  // namespace apollo