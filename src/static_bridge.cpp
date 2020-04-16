// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <string>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"

#include "ros1_bridge/bridge.hpp"


int main(int argc, char * argv[])
{
  // ROS 1 node
  ros::init(argc, argv, "ros_bridge");
  ros::NodeHandle ros1_node;

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("ros_bridge");

  // bridge one example topic
  std::string topic_name = "iiwa/joint_states";
  std::string ros1_type_name = "sensor_msgs/JointState";
  std::string ros2_type_name = "sensor_msgs/msg/JointState";
  size_t queue_size = 10;

  auto handles = ros1_bridge::create_bidirectional_bridge(
    ros1_node, ros2_node, ros1_type_name, ros2_type_name, topic_name, queue_size);

//  auto handles_srv = ros1_bridge::create_bidirectional_bridge(
//    ros1_node, ros2_node, "iiwa_msgs/SetEndpointFrame", "iiwa_msgs/srv/SetEndpointFrame"," /iiwa/configuration/setEndpointFrame", 10);

  // create bridges for ros1 services
  auto request_data_relay_2to1_factory = ros1_bridge::get_service_factory(
    "ros1", "iiwa_msgs", "SetEndpointFrame");
  if (request_data_relay_2to1_factory) {
    try {
      ros1_bridge::ServiceBridge2to1 request_data_relay_2to1 = request_data_relay_2to1_factory->service_bridge_2_to_1(ros1_node, ros2_node, "/iiwa/configuration/setEndpointFrame");
      printf("Created 2 to 1 bridge for service %s\n", "/iiwa/configuration/setEndpointFrame\n");
    } catch (std::runtime_error & e) {
      fprintf(stderr, "Failed to created a bridge: %s\n", "/iiwa/configuration/setEndpointFrame\n");
    }
  }
  else
  {
      printf("NOPE\n");
  }

  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // ROS 2 spinning loop
  rclcpp::executors::SingleThreadedExecutor executor;
  while (ros1_node.ok() && rclcpp::ok()) {
    executor.spin_node_once(ros2_node, std::chrono::milliseconds(1000));
  }

  return 0;
}
