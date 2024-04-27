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

#include "modules/drivers/lidar/ls180s2_gazel/parser/parser_factory.h"


namespace apollo {
namespace drivers {
namespace ls180s2_gazel {

using ::apollo::cyber::Node;

Parser* ParserFactory::CreateParser(const std::shared_ptr<Node>& node,
                                    const Config& conf) {

  return new Parser(node, conf);

  AERROR << "Couldn't create parser for lslidar";
  return nullptr;
}

}  // namespace ls180s2_gazel
}  // namespace drivers
}  // namespace apollo
