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

#include <memory>
#include <string>
#include "cyber/cyber.h"
#include "cyber/component/component.h"
#include "modules/drivers/lidar/ls180s2_gazel/driver/driver.h"

#include "modules/drivers/lidar/proto/lslidar_config.pb.h"
#include "modules/drivers/lidar/proto/config.pb.h"



namespace apollo {
namespace drivers {
namespace ls180s2_gazel {
using apollo::cyber::Component;

class LslidarComponent : public Component<> {
 public:
  ~LslidarComponent() {}
  bool Init() override;

 private:
  std::shared_ptr<LslidarChDriver> driver_;
  Config lslidar_conf_;
};

CYBER_REGISTER_COMPONENT(LslidarComponent)

}  // namespace ls180s2_gazel
}  // namespace drivers
}  // namespace apollo
//CYBER_REGISTER_COMPONENT(apollo::drivers::ls180s2_gazel::LslidarComponent)