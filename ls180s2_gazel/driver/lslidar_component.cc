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
#include "lslidar_component.h"

namespace apollo {
namespace drivers {
namespace ls180s2_gazel {

bool LslidarComponent::Init() {
  if (!GetProtoConfig(&lslidar_conf_)) {
    AERROR << "load config error, file:" << config_file_path_;
    return false;
  }

  AINFO << "conf:" << lslidar_conf_.DebugString();

  driver_.reset(new LslidarChDriver(node_, lslidar_conf_));

  if (!driver_->Init()) {
    AERROR << "driver init error";
    return false;
  }
  while (apollo::cyber::OK())
  {
    driver_->polling();
  }
  
  return true;
}

}  // namespace ls180s2_gazel
}  // namespace drivers
}  // namespace apollo