// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#undef WITH_CAM_MODELS

#include <rclcpp/rclcpp.hpp>
#include "mynteye/logger.h"
#include "wrapper_nodelet.h"

int main(int argc, char *argv[]) {
  glog_init _(argc, argv);

  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("mynteye_wrapper_node");

  // mynteye::ROSWrapperNodelet nodelet(ros2_node);

  // ros::init(argc, argv, "mynteye_wrapper_node");
  // ros::console::set_logger_level(
  //     ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  // nodelet::Loader nodelet;
  // nodelet::M_string remap(ros::names::getRemappings());
  // nodelet::V_string nargv;
  // nodelet.load(
  //     ros::this_node::getName(), "mynteye/ROSWrapperNodelet", remap, nargv);

  // ros::spin();
  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(ros2_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
