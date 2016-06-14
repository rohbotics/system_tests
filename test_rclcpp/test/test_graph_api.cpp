// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <iostream>
#include <string>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "test_rclcpp/msg/u_int32.hpp"
#include "test_rclcpp/utils.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)

#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif


// Basic count_publishers and count_subscribers test.
TEST(CLASSNAME(test_graph_api, RMW_IMPLEMENTATION), test_count_subscribers) {
  auto node = rclcpp::Node::make_shared("test_count_subscribers");
  std::string topic_name = "test_count_subscribers";
  EXPECT_EQ(node->count_subscribers(topic_name), 0);

  auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
      topic_name, 1, [](const test_rclcpp::msg::UInt32::SharedPtr){});

  auto publisher = node->create_publisher<test_rclcpp::msg::UInt32>(topic_name, 1);
  test_rclcpp::busy_wait_for_subscriber(node, topic_name);
  EXPECT_EQ(node->count_subscribers(topic_name), 1);
  EXPECT_EQ(node->count_publishers(topic_name), 1);
}

TEST(CLASSNAME(test_graph_api, RMW_IMPLEMENTATION), test_get_topic_names_and_types) {
  auto node = rclcpp::Node::make_shared("test_get_topic_names_and_types");
  std::string topic_name = "test_count_subscribers";
  auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
      topic_name, 1, [](const test_rclcpp::msg::UInt32::SharedPtr){});

  auto publisher = node->create_publisher<test_rclcpp::msg::UInt32>(topic_name, 1);
  test_rclcpp::busy_wait_for_subscriber(node, topic_name);

  auto map = node->get_topic_names_and_types();
  EXPECT_EQ(map.at(topic_name), "test_rclcpp/UInt32");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
