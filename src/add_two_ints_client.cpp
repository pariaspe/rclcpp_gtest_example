// Copyright 2024 Pedro Arias-Perez
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

#include "rclcpp_gtest_example/add_two_ints_client.hpp"

AddTwoIntsClient::AddTwoIntsClient()
: Node("add_two_ints_client")
{
  client_ =
    this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
}

AddTwoIntsClient::~AddTwoIntsClient() {}

rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture
AddTwoIntsClient::call_client(int a, int b)
{
  auto request =
    std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = a;
  request->b = b;

  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for the service. Exiting.");
      return rclcpp::Client<
        example_interfaces::srv::AddTwoInts>::SharedFuture();
    }
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp"),
      "service not available, waiting again...");
  }

  auto result_future = client_->async_send_request(request);
  return result_future;
}
